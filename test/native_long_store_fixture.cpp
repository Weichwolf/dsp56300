// SPDX-License-Identifier: GPL-3.0-only
#include "native_long_store_fixture.hpp"

#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>

namespace dsp56300::test::native_long {
namespace {
constexpr std::uint64_t input_x = 0x2468ac13579b, input_y = 0x765432abcdef;
constexpr Word normal_mode = 0x305;

template <unsigned Bits> std::uint64_t limited(Registers &state, const std::uint64_t raw) {
  auto number = signed56(raw);
  if ((state.sr & mode_up) != 0) { number *= 2; }
  if ((state.sr & mode_down) != 0) { number = number >= 0 ? number / 2 : -((-number + 1) / 2); }
  constexpr unsigned growth_bit = 45;
  const auto shifted = static_cast<std::uint64_t>(number);
  if (((shifted >> growth_bit) & 1U) != ((shifted >> (growth_bit + 1)) & 1U)) {
    state.sr |= flag_s;
  }
  const auto boundary = std::int64_t{1} << (Bits - 1);
  if constexpr (Bits == word_bits) {
    constexpr auto unit = std::int64_t{1} << word_bits;
    number = number >= 0 ? number / unit : -((-number + unit - 1) / unit);
  }
  if (number < -boundary || number >= boundary) { state.sr |= flag_l; }
  return static_cast<std::uint64_t>(std::clamp(number, -boundary, boundary - 1));
}
}

Word opcode(const Input input) {
  constexpr Word memory = 0x400000;
  constexpr unsigned low_shift = 16;
  constexpr unsigned high_shift = 17;
  constexpr unsigned mode_shift = 11;
  constexpr unsigned index_shift = 8;
  const auto form = input.short_address
                        ? address << index_shift
                        : 0x4000U | (input.mode << mode_shift) | (input.index << index_shift);
  return memory | ((input.selector & 3U) << low_shift) | ((input.selector & 4U) << high_shift) |
         form;
}

Registers seed(const Input input) {
  Registers state;
  state.a = input.value;
  state.b = input.value ^ mask56;
  state.x = input_x;
  state.y = input_y;
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

Result expected(const Input input) {
  Result result{.state = seed(input), .data = {}};
  auto &state = result.state;
  const auto accumulator = (input.selector & 1U) != 0 ? state.b : state.a;
  auto value = accumulator;
  if (input.selector == 2) { value = state.x; }
  if (input.selector == 3) { value = state.y; }
  if (input.selector == 4 || input.selector == 5) {
    constexpr unsigned long_bits = 48;
    value = limited<long_bits>(state, accumulator);
  }
  if (input.selector >= 6) {
    const auto high = limited<word_bits>(state, input.selector == 6 ? state.a : state.b);
    const auto low = limited<word_bits>(state, input.selector == 6 ? state.b : state.a);
    result.data = {static_cast<Word>(high) & word_mask, static_cast<Word>(low) & word_mask};
  } else {
    result.data = {static_cast<Word>(value >> word_bits) & word_mask,
                   static_cast<Word>(value) & word_mask};
  }
  state.pc = !input.short_address && input.mode == 6 ? 2U : 1U;
  if (!input.short_address) {
    constexpr std::array deltas{-3, 3, -1, 1, 0, 0, 0, -1};
    const auto next = static_cast<int>(address) + deltas.at(input.mode);
    state.r.at(input.index) = static_cast<Word>(next);
  }
  return result;
}

Timing workload(const std::string_view domain, const bool long_store, const unsigned repetitions) {
  constexpr std::size_t pairs = 1024;
  constexpr unsigned intervals = 8;
  constexpr Word scalar_store = 0x446000;
  constexpr unsigned maximum_repetitions = 256;
  if (repetitions == 0 || repetitions > maximum_repetitions) {
    throw std::invalid_argument("Expected long-store repetitions in [1,256]");
  }
  constexpr std::array<Word, 3> operations{0x2000d2, 0x200040, 0x200043};
  const auto *const selected = std::ranges::find(domains, domain);
  if (selected == domains.end()) { throw std::invalid_argument("Unknown long-store workload"); }
  const auto alu = operations.at(static_cast<std::size_t>(selected - domains.begin()));
  const Input input;
  std::array<Word, pairs * 2> program{};
  for (std::size_t pair = 0; pair < pairs; ++pair) {
    program.at(pair * 2) = alu;
    program.at((pair * 2) + 1) = long_store ? opcode(input) : scalar_store;
  }
  std::array<Word, words> x{};
  std::array<Word, words> y{};
  Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
  Timing result;
  for (unsigned sample = 0; sample <= repetitions * intervals; ++sample) {
    auto state = seed(input);
    cpu.set_registers(state);
    x.fill(sentinel);
    y.fill(sentinel);
    std::array<Word, 2> data{};
    for (std::size_t pair = 0; pair < pairs; ++pair) {
      const auto old = state;
      if (domain == domains[2]) {
        bfu_model::apply(state, old, {.function = bfu_model::Function::bit_xor});
      } else {
        sequence_model::alu(state, old, alu);
      }
      if (long_store) {
        constexpr unsigned long_bits = 48;
        const auto value = limited<long_bits>(state, state.a);
        data = {static_cast<Word>(value >> word_bits) & word_mask,
                static_cast<Word>(value) & word_mask};
      } else {
        data = {static_cast<Word>(state.x) & word_mask, sentinel};
      }
    }
    state.pc = program.size();
    auto expected_x = x;
    auto expected_y = y;
    expected_x.at(address) = data[0];
    expected_y.at(address) = data[1];
    const auto before = cpu.jit_statistics();
    const auto start = std::chrono::steady_clock::now();
    const auto run = cpu.run(program.size());
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
    require(run.dispatches == program.size() && run.fault.kind == FaultKind::none &&
                cpu.registers() == state && x == expected_x && y == expected_y,
            "Shared long-store domain workload checks independently modeled full state and RAM");
    const auto after = cpu.jit_statistics();
    if (sample == 0) { continue; }
    result.seconds += elapsed.count();
    result.instructions += run.dispatches;
    result.native += after.native_alu_instructions - before.native_alu_instructions;
    result.moves += after.native_move_instructions - before.native_move_instructions;
    result.helpers += after.helper_instructions - before.helper_instructions;
  }
  return result;
}
} // namespace dsp56300::test::native_long
