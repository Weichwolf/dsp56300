// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"
#include "instruction_boundary.hpp"
#include "jit.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace dsp56300 {
namespace {
std::uint64_t elapsed(const std::uint64_t before, const std::uint64_t after) {
  if (after >= before) { return after - before; }
  return (std::numeric_limits<std::uint64_t>::max() - before) + after + 1;
}
}

std::optional<Word> Core::Impl::budget_word(const Word address) const {
  if (cacheable(address) && icache) {
    if (const auto word = icache->peek(address)) { return word; }
  }
  if (external) {
    if (io == nullptr) { return std::nullopt; }
    const auto value = io->inspect_program(address);
    return value && *value <= word_mask ? value : std::nullopt;
  }
  if (address >= buffers.program.size()) { return std::nullopt; }
  return buffers.program[address] & word_mask;
}

std::optional<detail::Instruction> Core::Impl::budget_instruction() const {
  if (cpu.repeat_remaining != 0 && cpu.repeat_loaded) { return repeated; }
  const auto opcode = budget_word(cpu.reg.pc);
  if (!opcode) { return std::nullopt; }
  auto instruction = detail::decode(*opcode);
  if (instruction.words == 2) {
    const auto extension_word = budget_word((cpu.reg.pc + 1) & word_mask);
    if (!extension_word) { return std::nullopt; }
    instruction.extension = *extension_word;
  }
  return instruction;
}

std::optional<RunStop> Core::Impl::budget_stop(const RunBudget budget,
                                               const BoundedRunResult &result) const {
  if (fault.kind != FaultKind::none) { return RunStop::fault; }
  if (cpu.processing != ProcessingState::running) { return RunStop::state; }
  if (yield_requested) { return RunStop::yielded; }
  if (result.execution.dispatches == budget.instructions) { return RunStop::instruction_budget; }
  if (result.cycles == budget.cycles) { return RunStop::cycle_budget; }
  if (debug_ready() || exception_ready()) { return RunStop::control_event; }
  return std::nullopt;
}

BoundedRunResult Core::Impl::run_bounded(const RunBudget budget) {
  BoundedRunResult result;
  const auto before = cpu.cycles;
  while (true) {
    result.cycles = elapsed(before, cpu.cycles);
    if (const auto stop = budget_stop(budget, result)) {
      result.stop = *stop;
      break;
    }
    const auto instructions = budget.instructions - result.execution.dispatches;
    const auto cycles = budget.cycles - result.cycles;
    if (jit && cpu.repeat_remaining == 0 && (cpu.reg.sr & detail::cache_enable) == 0 &&
        (cpu.debug_flags & detail::debug_interrupt_pending) == 0) {
      const auto retired = jit->run_bounded(instructions, cycles);
      result.execution.dispatches += retired;
      if (retired != 0 || fault.kind != FaultKind::none || yield_requested) { continue; }
    }
    const auto instruction = budget_instruction();
    if (!instruction) {
      result.stop = RunStop::unknown_code;
      break;
    }
    result.next_cycles = instruction->cycles;
    result.next = InstructionWords{.address = cpu.reg.pc, .opcode = instruction->opcode};
    if (instruction->words == 2) { result.next->extension = instruction->extension; }
    if (instruction->cycles > cycles) {
      result.stop = RunStop::cycle_budget;
      break;
    }
    if ((cpu.reg.sr & detail::cache_enable) != 0) {
      result.stop = RunStop::control_event;
      break;
    }
    if (detail::requires_owner(*instruction)) {
      result.stop = RunStop::external_effect;
      break;
    }
    step_decoded(*instruction);
    synchronize_program_view();
    if (fault.kind == FaultKind::none) {
      ++result.execution.dispatches;
      if (jit) { jit->helper_retired(); }
    }
    result.next_cycles.reset();
    result.next.reset();
  }
  result.pc = cpu.reg.pc;
  result.execution.progress = {.instructions = cpu.instructions, .estimated_cycles = cpu.cycles};
  result.execution.fault = fault;
  result.execution.state = cpu.processing;
  result.execution.yielded = yield_requested;
  return result;
}

BoundedRunResult Core::run(const RunBudget budget) {
  impl_->validate_staged_mutation();
  BoundedRunResult result;
  {
    const detail::ExecutionGuard guard(impl_->executing);
    if (budget.instructions != 0 && budget.cycles != 0) { impl_->last_outcome.reset(); }
    impl_->yield_requested = false;
    impl_->synchronize_program_view();
    impl_->yield_requested = false;
    result = impl_->run_bounded(budget);
  }
  if (result.stop == RunStop::fault || result.stop == RunStop::state) { return result; }
  const auto fetch = instruction_fetch();
  result.pc = fetch.address;
  if (impl_->debug_ready()) {
    result.next.reset();
    result.next_cycles.reset();
    return result;
  }
  if (result.next && result.next->address == fetch.address && !fetch.exception_vector) {
    return result;
  }
  const auto word = [&](const Word address) -> std::optional<Word> {
    if (fetch.exception_vector && impl_->vector.words && address >= impl_->vector.address &&
        address - impl_->vector.address < 2) {
      return impl_->vector.words->at(address - impl_->vector.address);
    }
    return impl_->budget_word(address);
  };
  auto opcode = fetch.latched_opcode;
  if (!opcode) { opcode = word(fetch.address); }
  if (!opcode) { return result; }
  const auto decoded = detail::decode(*opcode);
  InstructionWords next{.address = fetch.address, .opcode = *opcode};
  if (decoded.words == 2) {
    next.extension = word((fetch.address + 1) & word_mask);
    if (!next.extension) { return result; }
  }
  result.next = next;
  result.next_cycles = decoded.cycles;
  return result;
}
}
