// SPDX-License-Identifier: GPL-3.0-only
#include "jit/preparation.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fetch.hpp"
#include "instruction.hpp"
#include "jit.hpp"

#include <queue>
#include <set>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr Word effective_mask = 0x3f, absolute_effective = 0x30, short_address_mask = 0xfff;
constexpr Word relative_group_mask = 0xff0000, short_relative_group = 0x050000;
constexpr Word upper_displacement = 0x1e0, lower_displacement = 0x1f;
constexpr Word displacement_sign = 0x100, displacement_mask = 0x1ff;

Successors jump_successors(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const auto kind = instruction.operation;
  const bool conditional = kind == Operation::conditional_jump;
  const bool short_form = conditional
                              ? (opcode & (1U << 18)) != 0
                              : kind == Operation::jump_short || kind == Operation::call_short;
  const bool call = conditional
                        ? (opcode & (1U << 16)) != 0
                        : kind == Operation::call_short || kind == Operation::call_effective;
  Successors result;
  if (conditional || call) {
    result.direct[result.count++] = (operation.pc + instruction.words) & word_mask;
  }
  if (short_form || ((opcode >> 8) & effective_mask) == absolute_effective) {
    result.direct[result.count++] =
        short_form ? opcode & short_address_mask : instruction.extension;
  } else {
    result.indirect = true;
  }
  return result;
}

Successors relative_successors(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto opcode = instruction.opcode;
  const bool short_form = (opcode & relative_group_mask) == short_relative_group;
  const bool unconditional = (opcode & (1U << (short_form ? 11U : 7U))) != 0;
  const bool call = (opcode & (1U << (short_form ? 10U : 6U))) == 0;
  Successors result;
  if (!unconditional || call) {
    result.direct[result.count++] = (operation.pc + instruction.words) & word_mask;
  }
  if (!short_form && (opcode & (1U << 11)) != 0) {
    result.indirect = true;
    return result;
  }
  auto offset = instruction.extension;
  if (short_form) {
    offset = ((opcode >> 1) & upper_displacement) | (opcode & lower_displacement);
    if ((offset & displacement_sign) != 0) { offset |= word_mask ^ displacement_mask; }
  }
  result.direct[result.count++] = (operation.pc + offset) & word_mask;
  return result;
}
}

Successors successors(const BlockInstruction &operation) {
  const auto &instruction = operation.instruction;
  const auto next = (operation.pc + instruction.words) & word_mask;
  switch (instruction.operation) {
    case Operation::unsupported: throw std::logic_error("Unsupported DSP preparation instruction");
    case Operation::jump_short:
    case Operation::jump_effective:
    case Operation::call_short:
    case Operation::call_effective:
    case Operation::conditional_jump: return jump_successors(operation);
    case Operation::relative_branch: return relative_successors(operation);
    case Operation::bit_jump: return {.direct = {next, instruction.extension}, .count = 2};
    case Operation::bit_branch:
      return {.direct = {next, (operation.pc + instruction.extension) & word_mask}, .count = 2};
    case Operation::return_subroutine:
    case Operation::return_interrupt: return {.indirect = true};
    case Operation::loop_immediate:
    case Operation::loop_register:
    case Operation::loop_short:
    case Operation::loop_effective:
    case Operation::loop_forever: {
      const auto opcode = instruction.opcode;
      const bool relative = instruction.operation == Operation::loop_forever ? (opcode & 1U) == 0
                                                                             : (opcode & 16U) != 0;
      const auto last =
          relative ? (operation.pc + instruction.extension) & word_mask : instruction.extension;
      return {.direct = {next, (last + 1) & word_mask}, .count = 2, .indirect = true};
    }
    case Operation::illegal:
      return {.direct = {(operation.pc + 2) & word_mask}, .count = 1, .indirect = true};
    case Operation::repeat_immediate:
    case Operation::repeat_register:
    case Operation::repeat_short:
    case Operation::repeat_effective:
    case Operation::break_loop:
    case Operation::end_loop:
    case Operation::trap:
    case Operation::conditional_trap:
    case Operation::wait:
    case Operation::stop:
    case Operation::debug:
    case Operation::conditional_debug: return {.direct = {next}, .count = 1, .indirect = true};
    case Operation::parallel:
    case Operation::nop:
    case Operation::and_control:
    case Operation::or_control:
    case Operation::arithmetic_shift_immediate:
    case Operation::arithmetic_shift_register:
    case Operation::logical_shift_immediate:
    case Operation::logical_shift_register:
    case Operation::immediate_alu:
    case Operation::load_relative_address:
    case Operation::load_updated_address:
    case Operation::divide_iteration:
    case Operation::normalize_iteration:
    case Operation::normalize_fast:
    case Operation::viterbi_shift:
    case Operation::product_immediate:
    case Operation::product_power:
    case Operation::product_mixed:
    case Operation::product_double:
    case Operation::increment:
    case Operation::decrement:
    case Operation::compare_unsigned:
    case Operation::bit_field_extract:
    case Operation::bit_field_insert:
    case Operation::bit_field_merge:
    case Operation::count_leading_bits:
    case Operation::conditional_transfer:
    case Operation::move_control_effective:
    case Operation::move_displacement:
    case Operation::move_control_short:
    case Operation::move_control_register:
    case Operation::move_control_immediate:
    case Operation::move_program_effective:
    case Operation::move_program_short:
    case Operation::move_peripheral_register:
    case Operation::move_peripheral_memory:
    case Operation::move_peripheral_program:
    case Operation::bit_register:
    case Operation::bit_memory:
    case Operation::cache_flush:
    case Operation::cache_flush_unlocked:
    case Operation::cache_free:
    case Operation::cache_lock:
    case Operation::cache_unlock:
    case Operation::cache_lock_relative:
    case Operation::cache_unlock_relative:
    case Operation::reset_peripherals: return {.direct = {next}, .count = 1};
    default: throw std::logic_error("Invalid DSP preparation instruction kind");
  }
}

PreparationGraph preparation_graph(const std::span<const Word> program,
                                   const JitRuntime &runtime,
                                   const std::span<const Word> established_entries) {
  PreparationGraph result;
  if (established_entries.empty()) { return result; }
  std::queue<Word> pending;
  std::set<Word> headers(established_entries.begin(), established_entries.end());
  std::vector<bool> visited(address_words);
  const auto enqueue = [&](const Word address) {
    if (address > word_mask) { throw std::invalid_argument("DSP code entry exceeds 24 bits"); }
    if (visited[address]) { return; }
    visited[address] = true;
    pending.push(address);
  };
  for (const auto entry : established_entries) { enqueue(entry); }
  while (!pending.empty()) {
    const auto address = pending.front();
    pending.pop();
    const auto capture = capture_instruction(program, address, runtime);
    if (capture.failure.kind != FaultKind::none) {
      result.unresolved.push_back(address);
      continue;
    }
    const auto &instruction = capture.instruction;
    if (instruction.operation == Operation::unsupported) {
      if (result.failure.kind == FaultKind::none) {
        result.failure = {.kind = FaultKind::unsupported_instruction,
                          .space = Space::program,
                          .address = address,
                          .value = instruction.opcode,
                          .pc = address};
      }
      result.unresolved.push_back(address);
      continue;
    }
    const BlockInstruction operation{.instruction = instruction, .pc = address};
    result.instructions.push_back(operation);
    const auto flow = successors(operation);
    if (flow.indirect) { result.unresolved.push_back(address); }
    const bool boundary = !repeatable(instruction.operation) ||
                          instruction.operation == Operation::conditional_transfer;
    for (const auto target : std::span(flow.direct).first(flow.count)) {
      enqueue(target);
      if (boundary) { headers.insert(target); }
    }
  }
  result.entries.assign(headers.begin(), headers.end());
  return result;
}
}
