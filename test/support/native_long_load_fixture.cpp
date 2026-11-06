// SPDX-License-Identifier: GPL-3.0-only
#include "native_long_load_fixture.hpp"

#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::long_load {
namespace {
template <unsigned Bits> std::uint64_t sign_extend(const std::uint64_t value) {
  const auto sign = std::uint64_t{1} << (Bits - 1);
  return (value & sign) == 0 ? value : value | (mask56 & ~((sign << 1) - 1));
}
}

Word opcode(const Input input) {
  constexpr Word memory_load = 0x408000;
  constexpr unsigned selector_high_shift = 17;
  const auto form =
      input.short_address ? address << 8 : 0x4000U | (input.mode << 11) | (input.index << 8);
  return memory_load | ((input.selector & 3U) << 16) |
         ((input.selector & 4U) << selector_high_shift) | form;
}

Registers seed(const Input input) {
  constexpr std::uint64_t initial_a = 0x5a123456abcdef;
  constexpr std::uint64_t initial_b = 0xa5fedcba654321;
  constexpr std::uint64_t initial_x = 0x2468ac13579b;
  constexpr std::uint64_t initial_y = 0x765432fedcba;
  constexpr Word normal_mode = 0x305;
  Registers state;
  state.a = initial_a;
  state.b = initial_b;
  state.x = initial_x;
  state.y = initial_y;
  state.omr = normal_mode;
  state.sr = input.status;
  state.r.at(input.index) = address;
  state.n.at(input.index) = offset;
  state.m.fill(word_mask);
  return state;
}

Word effective_address(const Input input) {
  if (input.short_address) { return address; }
  if (input.mode == 5) { return address + offset; }
  if (input.mode == 7) { return address - 1; }
  return address;
}

void expected_load(Registers &state, const Input input) {
  const auto high = (input.value >> word_bits) & word_mask;
  const auto low = input.value & word_mask;
  constexpr auto extension = std::uint64_t{0xff} << double_word_bits;
  switch (input.selector) {
    case 0: state.a = (state.a & extension) | input.value; break;
    case 1: state.b = (state.b & extension) | input.value; break;
    case 2: state.x = input.value; break;
    case 3: state.y = input.value; break;
    case 4: state.a = sign_extend<double_word_bits>(input.value); break;
    case 5: state.b = sign_extend<double_word_bits>(input.value); break;
    case 6:
      state.a = (sign_extend<word_bits>(high) << word_bits) & mask56;
      state.b = (sign_extend<word_bits>(low) << word_bits) & mask56;
      break;
    case 7:
      state.b = (sign_extend<word_bits>(high) << word_bits) & mask56;
      state.a = (sign_extend<word_bits>(low) << word_bits) & mask56;
      break;
    default: break;
  }
  state.pc += !input.short_address && input.mode == 6 ? 2U : 1U;
  if (!input.short_address) {
    constexpr std::array deltas{-3, 3, -1, 1, 0, 0, 0, -1};
    const auto next = static_cast<int>(state.r.at(input.index)) + deltas.at(input.mode);
    state.r.at(input.index) = static_cast<Word>(next);
  }
}

void expected_alu(Registers &state, const Registers &old, const Word instruction) {
  constexpr Word alu_mask = 0xf7;
  constexpr Word xor_x0 = 0x43;
  if ((instruction & alu_mask) != xor_x0) {
    sequence_model::alu(state, old, instruction);
    return;
  }
  const bool destination_b = (instruction & 8U) != 0;
  auto logical = old;
  bfu_model::apply(
      logical, old, {.function = bfu_model::Function::bit_xor, .destination_b = destination_b});
  constexpr auto middle = std::uint64_t{word_mask} << word_bits;
  auto &destination = destination_b ? state.b : state.a;
  const auto result = destination_b ? logical.b : logical.a;
  // Logical ALU replaces the middle word only; the other destination bits
  // are the post-MOVE value, while the truth-table inputs are pre-MOVE values.
  destination = (destination & ~middle) | (result & middle);
  state.sr = logical.sr;
}
} // namespace dsp56300::test::long_load
