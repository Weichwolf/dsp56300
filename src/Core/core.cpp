// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"
#include "jit.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>

namespace dsp56300 {
namespace {

Buffers validate(const Buffers buffers, const Options options) {
  if (buffers.program.empty() || buffers.x.empty() || buffers.y.empty() ||
      buffers.program.size() > address_words || buffers.x.size() > address_words ||
      buffers.y.size() > address_words) {
    throw std::invalid_argument("DSP requires nonempty memory spans of at most 2^24 words");
  }
  constexpr Word maximum_block = 256;
  if (options.instructions_per_block == 0 || options.instructions_per_block > maximum_block) {
    throw std::invalid_argument("Translation block length must be in [1,256]");
  }
  if (options.backend == Backend::jit && !detail::Jit::available()) {
    throw std::invalid_argument("Native translation requires Linux x86-64");
  }
  return buffers;
}
}

Core::Impl::Impl(const Buffers supplied, Io *const host_io, const Options settings)
    : io(host_io), buffers(validate(supplied, settings)), options(settings) {
  aliases_program[1] = overlaps_program(buffers.x);
  aliases_program[2] = overlaps_program(buffers.y);
  if (options.backend == Backend::jit) {
    jit = std::make_unique<detail::Jit>(
        cpu,
        buffers.program,
        options,
        detail::JitRuntime{.owner = this,
                           .latched = &latched,
                           .execute = jit_execute,
                           .moves = jit_moves,
                           .memory = {{{.data = buffers.x.data(),
                                        .words = buffers.x.size(),
                                        .aliases_program = aliases_program[1]},
                                       {.data = buffers.y.data(),
                                        .words = buffers.y.size(),
                                        .aliases_program = aliases_program[2]}}}});
  }
}

bool Core::Impl::overlaps_program(const std::span<const Word> memory) const {
  const auto program = reinterpret_cast<std::uintptr_t>(buffers.program.data());
  const auto other = reinterpret_cast<std::uintptr_t>(memory.data());
  return other < program ? program - other < memory.size_bytes()
                         : other - program < buffers.program.size_bytes();
}

void Core::Impl::invalidate_program_word(const Word *const location) {
  const auto program = reinterpret_cast<std::uintptr_t>(buffers.program.data());
  const auto address = reinterpret_cast<std::uintptr_t>(location);
  if (address < program || address - program >= buffers.program.size_bytes()) { return; }
  const auto index = (address - program) / sizeof(Word);
  if (jit) { jit->invalidate(); }
  pages[index / page_words].reset();
  if (index != 0) { pages[(index - 1) / page_words].reset(); }
}

std::span<Word> Core::Impl::area(const Space space) const {
  switch (space) {
    case Space::program: return buffers.program;
    case Space::x: return buffers.x;
    case Space::y: return buffers.y;
    default: throw std::invalid_argument("Invalid DSP memory space");
  }
}

void Core::Impl::fail(const FaultKind kind,
                      const Space space,
                      const Word address,
                      const Word value) {
  if (fault.kind == FaultKind::none) {
    fault = {.kind = kind, .space = space, .address = address, .value = value, .pc = current_pc};
  }
}

Word Core::Impl::read_memory(const Space space, const Word address) {
  if (space != Space::program && address >= detail::io_begin) {
    Word value = 0;
    if (io == nullptr || !io->read(space, address, current_pc, value)) {
      fail(FaultKind::io_read, space, address);
    }
    return value & word_mask;
  }
  const auto memory = area(space);
  if (address >= memory.size()) {
    fail(FaultKind::memory_read, space, address);
    return 0;
  }
  return memory[address] & word_mask;
}

void Core::Impl::write_memory(const Space space, const Word address, const Word value) {
  if (space != Space::program && address >= detail::io_begin) {
    if (io == nullptr || !io->write(space, address, current_pc, value & word_mask)) {
      fail(FaultKind::io_write, space, address, value);
    }
    return;
  }
  const auto memory = area(space);
  if (address >= memory.size()) {
    fail(FaultKind::memory_write, space, address, value);
    return;
  }
  memory[address] = value & word_mask;
  if (aliases_program[static_cast<std::size_t>(space)]) {
    invalidate_program_word(&memory[address]);
  }
}

detail::Instruction Core::Impl::instruction(const Word pc) {
  const auto opcode = read_memory(Space::program, pc);
  if (options.backend != Backend::reference && pages[pc / page_words]) {
    const auto &cached = (*pages[pc / page_words])[pc % page_words];
    if (cached.operation != detail::Operation::unsupported && cached.opcode == opcode &&
        (cached.words == 1 ||
         cached.extension == read_memory(Space::program, (pc + 1) & word_mask))) {
      return cached;
    }
  }
  auto decoded = detail::decode(opcode);
  if (decoded.words == 2) { decoded.extension = read_memory(Space::program, (pc + 1) & word_mask); }
  if (options.backend == Backend::reference) { return decoded; }
  auto &page = pages[pc / page_words];
  if (!page) { page = std::make_unique<Page>(); }
  auto &cached = (*page)[pc % page_words];
  if (cached != decoded) { cached = decoded; }
  return cached;
}

void Core::Impl::step() {
  current_pc = cpu.reg.pc;
  const auto decoded =
      cpu.repeat_remaining != 0 && cpu.repeat_loaded ? repeated : instruction(current_pc);
  if (fault.kind != FaultKind::none) { return; }
  step_decoded(decoded);
}

void Core::Impl::step_decoded(const detail::Instruction &decoded) {
  current_pc = cpu.reg.pc;
  const bool repeating = cpu.repeat_remaining != 0;
  if (repeating && (decoded.words != 1 || !detail::repeatable(decoded.operation))) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
    return;
  }
  current_words = decoded.words;
  cpu.interrupt_inhibited = false;
  cpu.reg.pc = (current_pc + decoded.words) & word_mask;
  execute(decoded);
  if (fault.kind != FaultKind::none) { return; }
  if (repeating) {
    cpu.repeat_opcode = decoded.opcode;
    repeated = decoded;
    cpu.repeat_loaded = true;
    --cpu.repeat_remaining;
    if (cpu.repeat_remaining != 0) {
      cpu.reg.lc = cpu.repeat_remaining;
      cpu.reg.pc = current_pc;
      ++cpu.instructions;
      cpu.cycles += decoded.cycles;
      return;
    }
    cpu.reg.lc = cpu.repeat_lc;
    cpu.repeat_completed = true;
    cpu.repeat_loaded = false;
  } else if (!detail::is_repeat(decoded.operation)) {
    cpu.repeat_completed = false;
  }
  finish_loop();
  if (fault.kind == FaultKind::none) {
    ++cpu.instructions;
    cpu.cycles += decoded.cycles;
  }
}

Core::Core(const Buffers buffers, Io *const io, const Options options)
    : impl_(std::make_unique<Impl>(buffers, io, options)) {
  reset();
}

Core::~Core() = default;

void Core::reset() {
  if (impl_->executing) { throw std::logic_error("Reset requires stopped DSP execution"); }
  impl_->cpu = {};
  impl_->cpu.reg.m.fill(word_mask);
  impl_->fault = {};
  clear_code_cache();
  if (impl_->jit) { impl_->jit->reset(); }
}

Progress Core::progress() const noexcept {
  return {.instructions = impl_->cpu.instructions, .estimated_cycles = impl_->cpu.cycles};
}

Fault Core::fault() const noexcept {
  return impl_->fault;
}

JitStatistics Core::jit_statistics() const noexcept {
  return impl_->jit ? impl_->jit->statistics() : JitStatistics{};
}

Word Core::read(const Space space, const Word address) const {
  const auto memory = impl_->area(space);
  if (address >= memory.size()) { throw std::out_of_range("DSP host read outside memory span"); }
  return memory[address] & word_mask;
}

void Core::write(const Space space, const Word address, const Word value) {
  load(space, address, std::span(&value, 1));
}

void Core::load(const Space space, const Word address, const std::span<const Word> words) {
  const auto memory = impl_->area(space);
  if (address > memory.size() || words.size() > memory.size() - address) {
    throw std::out_of_range("DSP host load outside memory span");
  }
  if (std::ranges::any_of(words, [](const Word word) { return word > word_mask; })) {
    throw std::invalid_argument("DSP memory values must fit in 24 bits");
  }
  for (std::size_t index = 0; index < words.size(); ++index) {
    memory[address + index] = words[index];
  }
  if (space == Space::program) {
    clear_code_cache();
  } else if (impl_->aliases_program[static_cast<std::size_t>(space)]) {
    for (std::size_t index = 0; index < words.size(); ++index) {
      impl_->invalidate_program_word(&memory[address + index]);
    }
  }
}

std::uint32_t Core::Impl::run_general(const std::uint32_t budget) {
  std::uint32_t retired = 0;
  while (retired < budget && fault.kind == FaultKind::none) {
    step();
    if (fault.kind == FaultKind::none) { ++retired; }
  }
  return retired;
}

RunResult Core::run(const std::uint32_t instruction_budget) {
  const detail::ExecutionGuard guard(impl_->executing);
  RunResult result;
  if (!impl_->jit) {
    result.dispatches = impl_->run_general(instruction_budget);
    result.progress = progress();
    result.fault = fault();
    return result;
  }
  while (result.dispatches < instruction_budget && fault().kind == FaultKind::none) {
    if (impl_->cpu.repeat_remaining == 0) {
      const auto retired = impl_->jit->run(instruction_budget - result.dispatches);
      result.dispatches += retired;
      if (retired != 0 || fault().kind != FaultKind::none) { continue; }
    }
    impl_->step();
    if (fault().kind == FaultKind::none) {
      ++result.dispatches;
      impl_->jit->helper_retired();
    }
  }
  result.progress = progress();
  result.fault = fault();
  return result;
}

bool Core::interruptible() const {
  if (fault().kind != FaultKind::none || impl_->cpu.repeat_remaining != 0 ||
      impl_->cpu.interrupt_inhibited) {
    return false;
  }
  if (!impl_->cpu.repeat_completed) { return true; }
  const auto pc = impl_->cpu.reg.pc;
  if (pc >= impl_->buffers.program.size()) { return false; }
  return !detail::is_repeat(detail::decode(impl_->buffers.program[pc] & word_mask).operation);
}

void Core::clear_code_cache() {
  if (impl_->jit) { impl_->jit->invalidate(); }
  for (auto &page : impl_->pages) { page.reset(); }
}
} // namespace dsp56300
