// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/runtime.hpp"
#include "loop_frames.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word entry = 8, count = 5;
constexpr Word first = 0x123456, second = 0x654321;
unsigned fallback_calls = 0;

std::uint32_t fallback(detail::Cpu * /*cpu*/, detail::NativeContext * /*context*/) noexcept {
  ++fallback_calls;
  return 0;
}

void admitted_native() {
  constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
  std::array<Word, 32> program{};
  std::array<Word, 32> x{};
  std::array<Word, 32> y{};
  constexpr Word load = pair | (2U << 18) | (3U << 16) | (1U << 15) | (1U << 22);
  constexpr Word store = pair | (3U << 18) | (2U << 16) | (3U << 11) | (3U << 20);
  program[entry] = load;
  program[entry + 1] = store;
  Registers initial;
  initial.pc = entry;
  initial.la = entry + 1;
  initial.lc = count;
  initial.sr = 1U << 15;
  initial.sp = initial.sc = 2;
  initial.stack[2] = std::uint64_t{entry} << test::word_bits;
  initial.r[4] = 1;
  initial.m.fill(word_mask);
  const detail::JitRuntime runtime{
      .memory = {detail::JitMemory{.data = x.data(), .words = x.size()},
                 detail::JitMemory{.data = y.data(), .words = y.size()}}};
  const auto operations = detail::parse_block(program, initial, runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  const auto image =
      detail::compile_region(compiler, operations, runtime, std::bit_cast<const void *>(&fallback));
  if (!image) { throw std::runtime_error("A real RAM loop must be accepted"); }
  require(image->native_bytes > 0, "A real RAM loop must emit LLVM native text");
  using Function = std::uint32_t (*)(detail::Cpu *, detail::NativeContext *) noexcept;
  const auto run = std::bit_cast<Function>(image->entry);
  for (Word budget = 0; budget <= count * 2; ++budget) {
    for (std::uint64_t deadline = 0; deadline <= std::uint64_t{count} * 2; ++deadline) {
      x.fill(first);
      y.fill(second);
      detail::Cpu cpu{.reg = initial};
      detail::NativeContext context{.runtime = {}, .budget = budget, .cycle_deadline = deadline};
      const auto before = fallback_calls;
      const auto turns = std::min(std::uint64_t{budget / 2}, deadline / 2);
      require(
          run(&cpu, &context) == turns * 2 && fallback_calls == before + (turns == 0 ? 1U : 0U) &&
              cpu.instructions == turns * 2 && cpu.cycles == turns * 2 && cpu.reg.r[0] == turns &&
              cpu.reg.r[4] == turns + 1 && (turns == 0 || (x[0] == second && y[1] == first)),
          "Positive whole-loop grants actually execute LLVM, without the ordinary fallback");
    }
  }
  require(compiler->native_bytes() == image->native_bytes,
          "Warm execution never emits code or changes cold text accounting");
}

constexpr Word ring_turns = 9, ring_mask = 3, ring_modifier = 0x8003;
constexpr Word raw_ring_modifier = 0x128003, reserved_modifier = 0xc003, invalid_modifier = 0x8002;
constexpr Word modifier_bits = 0xffff, prior_la = 300, prior_lc = 7;
constexpr Word x_load_bit = 15, y_load_bit = 22, y_mode_shift = 20;
constexpr Word x_pattern = 0xf12345, y_pattern = 0x876543, x_stride = 7937, y_stride = 10177;
constexpr std::size_t ring_extent = 32;
using RingData = std::array<Word, ring_extent>;
using RingKnown = std::array<std::uint8_t, ring_extent>;
using RingFunction = std::uint32_t (*)(detail::Cpu *, detail::NativeContext *) noexcept;

Word ring_advance(const Word value, const Word modifier, const bool subtract) {
  const auto next = subtract ? value - 1 : value + 1;
  return (modifier & modifier_bits) == modifier_bits ? next & word_mask
                                                     : (value & ~ring_mask) | (next & ring_mask);
}

struct RingCase {
  Word x_modifier, y_modifier, offset, grant, deadline;
};

class RingFixture {
public:
  RingFixture(const bool subtract, const bool tracked, const bool alias, const bool stationary)
      : subtract_(subtract), tracked_(tracked), alias_(alias), stationary_(stationary) {
    x_known_.fill(1);
    y_known_.fill(1);
    constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
    program_[entry] = pair | (1U << x_load_bit) | (1U << y_load_bit);
    const Word mode = subtract ? 2U : 3U;
    program_[entry + 1] = pair | ((stationary ? 0U : mode) << 11) | (mode << y_mode_shift);
    initial_ = test::initial_state();
    initial_.pc = entry;
    initial_.la = entry + 1;
    initial_.lc = ring_turns;
    initial_.sp = initial_.sc = 2;
    initial_.stack[1] = (std::uint64_t{prior_la} << test::word_bits) | prior_lc;
    initial_.stack[2] = (std::uint64_t{entry} << test::word_bits) | test::status_byte;
  }

  void compile(const Word numeric) {
    initial_.sr = (1U << 15) | test::status_byte | numeric;
    const auto bitmap = [&](RingKnown &known) {
      return tracked_ ? std::span(known) : std::span<std::uint8_t>{};
    };
    const std::array x_windows{
        MemoryWindow{.words = x_, .known = bitmap(x_known_), .writable = true}};
    const std::array y_windows{MemoryWindow{.words = alias_ ? x_ : y_,
                                            .known = bitmap(alias_ ? x_known_ : y_known_),
                                            .writable = true}};
    const detail::JitRuntime runtime{.memory = {}, .windows = {x_windows, y_windows}};
    const auto operations = detail::parse_block(program_, initial_, runtime);
    require(operations.size() == 2, "Authored body contains exactly two instructions");
    compiler_.reset();
    const auto image = detail::compile_region(
        compiler_, operations, runtime, std::bit_cast<const void *>(&fallback));
    if (!image) { throw std::runtime_error("An authored ring-capable loop must be prepared"); }
    run_ = std::bit_cast<RingFunction>(image->entry);
  }

  void modifiers() {
    const auto values = stationary_ ? std::array{0U, reserved_modifier, invalid_modifier}
                                    : std::array{ring_modifier, raw_ring_modifier, word_mask};
    for (const auto x_modifier : values) {
      for (const auto y_modifier : {ring_modifier, word_mask}) {
        offsets({.x_modifier = x_modifier,
                 .y_modifier = y_modifier,
                 .offset = 0,
                 .grant = 0,
                 .deadline = 0});
      }
    }
  }

private:
  void offsets(RingCase input) {
    for (input.offset = 0; input.offset <= ring_mask; ++input.offset) {
      for (input.grant = 0; input.grant <= ring_turns * 2; ++input.grant) {
        for (input.deadline = 0; input.deadline <= ring_turns * 2; ++input.deadline) {
          check(input);
        }
      }
    }
  }

  void check(const RingCase input) {
    auto seed = initial_;
    seed.r[0] = 12 + input.offset;
    seed.r[4] = 12 + ((input.offset + 1) & ring_mask);
    seed.m[0] = input.x_modifier;
    seed.m[4] = input.y_modifier;
    for (Word i = 0; i < x_.size(); ++i) {
      x_[i] = (x_pattern ^ (i * x_stride)) & word_mask;
      y_[i] = (y_pattern ^ (i * y_stride)) & word_mask;
    }
    const auto wanted_x = x_;
    const auto wanted_y = y_;
    auto &other = alias_ ? x_ : y_;
    const auto turns = std::min({input.grant / 2, input.deadline / 2, ring_turns});
    auto expected = seed;
    for (Word turn = 0; turn < turns; ++turn) {
      expected.x = (expected.x & ~std::uint64_t{word_mask}) | x_[expected.r[0]];
      expected.y = (expected.y & ~std::uint64_t{word_mask}) | other[expected.r[4]];
      if (!stationary_) {
        expected.r[0] = ring_advance(expected.r[0], input.x_modifier, subtract_);
      }
      expected.r[4] = ring_advance(expected.r[4], input.y_modifier, subtract_);
    }
    if (turns != 0) {
      expected.lc -= turns;
      if (turns == ring_turns) {
        test::loop_frames::complete(expected);
        expected.pc = entry + 2;
      }
    }
    detail::Cpu cpu{.reg = seed};
    detail::NativeContext context{
        .runtime = {}, .budget = input.grant, .cycle_deadline = input.deadline};
    const auto before = fallback_calls;
    const auto retired = run_(&cpu, &context);
    const auto expected_retired = std::uint64_t{turns} * 2;
    require(retired == expected_retired && fallback_calls == before + (turns == 0 ? 1U : 0U) &&
                cpu.reg == expected && cpu.instructions == expected_retired &&
                cpu.cycles == expected_retired && x_ == wanted_x && y_ == wanted_y &&
                std::ranges::all_of(x_known_, [](auto value) { return value == 1; }) &&
                std::ranges::all_of(y_known_, [](auto value) { return value == 1; }),
            "Actual fast batches preserve independent wrapped full state, grants and RAM");
  }

  bool subtract_, tracked_, alias_, stationary_;
  RingData program_{}, x_{}, y_{};
  RingKnown x_known_{}, y_known_{};
  Registers initial_;
  std::unique_ptr<detail::LlvmCompiler> compiler_;
  RingFunction run_ = nullptr;
};

void wrapped_native(const bool subtract,
                    const bool tracked,
                    const bool alias,
                    const bool stationary) {
  RingFixture fixture(subtract, tracked, alias, stationary);
  constexpr std::array modes{0U,
                             test::mode_down,
                             test::mode_up,
                             test::mode_rm,
                             test::mode_rm | test::mode_down,
                             test::mode_rm | test::mode_up};
  for (const auto numeric : modes) {
    fixture.compile(numeric);
    fixture.modifiers();
  }
}

}

int main() try {
  admitted_native();
  for (const bool subtract : {false, true}) {
    for (const bool tracked : {false, true}) {
      for (const bool alias : {false, true}) {
        for (const bool stationary : {false, true}) {
          wrapped_native(subtract, tracked, alias, stationary);
        }
      }
    }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
