// SPDX-License-Identifier: GPL-3.0-only
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "encoding.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace dsp56300::detail {
namespace {
constexpr Word short_jump_bit = Word{1} << 18;

constexpr std::array parallel_groups{
    EncodingPattern{.mask = 0x800000, .value = 0x800000},
    EncodingPattern{.mask = memory_group_mask, .value = memory_group},
    EncodingPattern{.mask = parallel_group_mask, .value = immediate_group},
    EncodingPattern{.mask = pair_group_mask, .value = pair_group},
    EncodingPattern{.mask = class_two_mask, .value = class_two}};

consteval bool parallel_rules_disjoint() {
  for (const auto &rule : encoding_rules) {
    for (const auto group : parallel_groups) {
      if (((rule.bits.value ^ group.value) & rule.bits.mask & group.mask) == 0) { return false; }
    }
  }
  return true;
}

static_assert(parallel_rules_disjoint(), "Prioritize overlapping specific instruction encodings");

constexpr bool parallel_encoding(const Word opcode) {
  return std::ranges::any_of(
      parallel_groups, [opcode](const EncodingPattern group) { return group.matches(opcode); });
}

bool has_extension(const Word opcode) noexcept {
  const unsigned mode = (opcode >> 11) & 7;
  if ((opcode & memory_group_mask) == 0x400000U) { return (opcode & 0x4000U) != 0 && mode == 6; }
  if ((opcode & pair_group_mask) == 0x100000U) { return mode == 6; }
  return false;
}

bool address_penalty(const Word opcode) noexcept {
  const unsigned mode = (opcode >> 11) & 7;
  return mode == 5 || mode == 6 || mode == 7;
}

bool parallel_address(const Word opcode) noexcept {
  const bool memory = (opcode & memory_group_mask) == memory_group && (opcode & (1U << 14)) != 0;
  return memory || (opcode & pair_group_mask) == pair_group ||
         (opcode & class_two_mask) == class_two;
}

std::uint8_t parallel_cycles(const Word opcode) noexcept {
  constexpr Word effective_address = 0x4000;
  constexpr Word load = 0x8000;
  // FM table A-1, A-6: absolute-short MOVE reads cost one clock, stores two.
  const bool short_store = (opcode & memory_group_mask) == memory_group &&
                           (opcode & long_selector_mask) != 0 &&
                           (opcode & (effective_address | load)) == 0;
  if (short_store) { return 2; }
  return static_cast<std::uint8_t>(1 +
                                   (parallel_address(opcode) && address_penalty(opcode) ? 1 : 0));
}

bool transfer_address(const Operation operation) noexcept {
  return operation == Operation::viterbi_shift || operation == Operation::move_control_effective ||
         operation == Operation::move_program_effective ||
         operation == Operation::move_peripheral_memory ||
         operation == Operation::move_peripheral_program;
}

bool rule_penalty(const EncodingRule &rule, const Word opcode) noexcept {
  if (rule.operation == Operation::bit_memory || rule.operation == Operation::bit_jump ||
      rule.operation == Operation::bit_branch) {
    const auto operand = bit_operand({.opcode = opcode, .operation = rule.operation});
    return operand.location == BitLocation::effective && address_penalty(opcode);
  }
  if (rule.operation == Operation::loop_effective ||
      rule.operation == Operation::repeat_effective) {
    return address_penalty(opcode);
  }
  if (transfer_address(rule.operation)) {
    const auto mode = (opcode >> 11) & 7;
    const bool immediate_peripheral = rule.operation == Operation::move_peripheral_memory &&
                                      mode == 6 && ((opcode >> 8) & 7) == 4;
    return (mode == 5 || mode == 6 || mode == 7) && !immediate_peripheral;
  }
  return (rule.operation == Operation::jump_effective ||
          rule.operation == Operation::call_effective || rule.operation == Operation::cache_lock ||
          rule.operation == Operation::cache_unlock) &&
         address_penalty(opcode);
}
}

Instruction decode(const Word opcode) noexcept {
  if (parallel_encoding(opcode)) {
    return {.opcode = opcode,
            .operation = Operation::parallel,
            .words = static_cast<std::uint8_t>(has_extension(opcode) ? 2 : 1),
            .cycles = parallel_cycles(opcode)};
  }
  for (const auto &rule : encoding_rules) {
    if (!rule.bits.matches(opcode)) { continue; }
    auto words = rule.words;
    if ((rule.operation == Operation::jump_effective ||
         rule.operation == Operation::call_effective || rule.operation == Operation::cache_lock ||
         rule.operation == Operation::cache_unlock || transfer_address(rule.operation) ||
         (rule.operation == Operation::bit_memory &&
          bit_operand({.opcode = opcode, .operation = rule.operation}).location ==
              BitLocation::effective) ||
         (rule.operation == Operation::conditional_jump && (opcode & short_jump_bit) == 0)) &&
        ((opcode >> 11) & 7) == 6) {
      words = 2;
    }
    return {.opcode = opcode,
            .operation = rule.operation,
            .words = words,
            .cycles =
                static_cast<std::uint8_t>(rule.cycles + (rule_penalty(rule, opcode) ? 1 : 0))};
  }

  return {.opcode = opcode};
}
} // namespace dsp56300::detail
