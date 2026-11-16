// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word opcode = static_cast<Word>(operation);
constexpr Word entry = 16, body = entry + 2, iterations = 16, body_size = 3;
constexpr Word total = iterations * body_size, loop_flag = Word{1} << 15, saved_lc = 37;
constexpr std::uint64_t seed_x = 0x234567000003, seed_y = 0x876543fffffd;
constexpr Word mac = opcode<isa::Opcode::mac_register_a> | (5U << 4);
// X:(R0)+ -> X0 and Y:(R4)+ -> Y0; MAC uses their previous values.
constexpr Word parallel_mac =
    opcode<isa::Opcode::move_xy> | (7U << 20) | (1U << 15) | (3U << 11) | (mac & 0xffU);
constexpr std::array source{opcode<isa::Opcode::do_immediate> | (iterations << 8),
                            body + body_size - 1,
                            parallel_mac,
                            opcode<isa::Opcode::add_x1_a>,
                            opcode<isa::Opcode::asr_a>,
                            opcode<isa::Opcode::stop>};
constexpr std::array<Word, iterations + 1> xdata{0xab000001,
                                                 test::sign24,
                                                 test::sign24 - 1,
                                                 0,
                                                 3,
                                                 word_mask,
                                                 7,
                                                 9,
                                                 11,
                                                 13,
                                                 17,
                                                 19,
                                                 23,
                                                 29,
                                                 31,
                                                 37,
                                                 41};
constexpr std::array<Word, iterations + 1> ydata{
    word_mask, 1, test::sign24, 9, test::sign24 - 1, 0, 11, 5, 7, 13, 17, 19, 23, 29, 31, 37, 41};
constexpr std::array modes{0U,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm,
                           test::mode_rm | test::mode_down,
                           test::mode_rm | test::mode_up};

constexpr std::array single_source{opcode<isa::Opcode::do_immediate> | (iterations << 8),
                                   body,
                                   parallel_mac,
                                   opcode<isa::Opcode::stop>};

constexpr std::array repeat_source{opcode<isa::Opcode::nop>,
                                   opcode<isa::Opcode::rep_immediate> | (iterations << 8),
                                   parallel_mac,
                                   opcode<isa::Opcode::stop>};

// Change both post-increment fields to post-decrement; ALU encoding is unchanged.
constexpr std::array descending_source{
    single_source[0], single_source[1], parallel_mac ^ (1U << 11) ^ (1U << 20), single_source[3]};

std::span<const Word> program(const bool single, const bool repeat, const bool descending) {
  if (repeat) { return repeat_source; }
  if (descending) { return descending_source; }
  return single ? std::span<const Word>(single_source) : std::span<const Word>(source);
}

struct Storage {
  explicit Storage(const std::span<const Word> code = source) {
    std::ranges::copy(code, p.begin() + entry);
  }

  [[nodiscard]] Buffers buffers() { return {.program = p, .x = x, .y = y}; }

  std::array<Word, 64> p{};
  std::array<Word, iterations + 1> x = xdata, y = ydata;
};

Registers initial(const Word mode, const bool single, const bool below) {
  Registers state;
  state.pc = body;
  state.sr = mode | (below ? 0U : test::flag_c | test::flag_s) | loop_flag;
  state.a = below ? test::sign56 + 2 : test::sign56 - 2;
  state.x = seed_x;
  state.y = seed_y;
  if (single) { state.y = (seed_y & ~std::uint64_t{word_mask}) | (below ? word_mask - 6U : 7U); }
  state.m.fill(word_mask);
  state.la = single ? body : body + body_size - 1;
  state.lc = iterations;
  state.sp = state.sc = 2;
  state.stack[2] = (std::uint64_t{body} << test::word_bits) | (state.sr & ~loop_flag);
  return state;
}

void advance(Registers &state,
             const Word index,
             const bool single,
             const bool repeat,
             const bool descending) {
  const auto old = state;
  const auto operation = single ? parallel_mac : source[2 + (index % body_size)];
  if (single || index % body_size == 0) {
    test::register_model::write(state, 4, xdata.at(old.r[0]) & word_mask);
    test::register_model::write(state, 6, ydata.at(old.r[4]) & word_mask);
    const Word step = descending ? word_mask : 1U;
    state.r[0] = (state.r[0] + step) & word_mask;
    state.r[4] = (state.r[4] + step) & word_mask;
  }
  test::sequence_model::alu(state, old, operation);
  ++state.pc;
  if (single || index % body_size == body_size - 1) {
    if (state.lc > 1) {
      --state.lc;
      state.pc = body;
    } else if (repeat) {
      state.lc = saved_lc;
    } else {
      test::loop_frames::complete(state);
    }
  }
}

struct Input {
  Word mode, prefix;
  bool single = false, below = false, repeat = false, descending = false;
};

void execute(const NativeImage &image, const Input input) {
  const auto [mode, prefix, single, below, repeat, descending] = input;
  const auto code = program(single, repeat, descending);
  const auto count = single ? iterations : total;
  Storage storage(code);
  Core core(Memory::separate(storage.buffers()));
  auto expected = initial(mode, single, below);
  if (descending) { expected.r[0] = expected.r[4] = iterations - 1; }
  if (repeat) {
    expected.pc = entry + 1;
    expected.sr &= ~loop_flag;
    expected.lc = saved_lc;
    expected.sp = expected.sc = 0;
    expected.stack.fill(0);
  }
  core.set_registers(expected);
  core.install(image, 0);
  if (repeat) {
    expected.pc = body;
    expected.lc = iterations;
    require(core.run(1).retired == 1 && core.registers() == expected,
            "Real REP setup preserves the initialized state and saves LC");
  }
  const Word setup_instructions = repeat ? 1 : 0;
  const Word setup_cycles = repeat ? 5 : 0;
  for (Word index = 0; index < prefix; ++index) {
    advance(expected, index, single, repeat, descending);
  }
  if (single && prefix == 1) {
    require((expected.sr & (test::flag_v | test::flag_l)) == (test::flag_v | test::flag_l),
            "The first MAC overflows in both signs");
  }
  if (single && !descending && prefix == 2) {
    require((expected.sr & (test::flag_v | test::flag_l)) == test::flag_l,
            "The next MAC clears V while retaining the earlier sticky limit");
  }
  const auto first = core.run(prefix);
  require(first.retired == prefix && first.fault == Fault{} && core.registers() == expected &&
              core.progress() == Progress{.instructions = setup_instructions + prefix,
                                          .estimated_cycles = setup_cycles + prefix},
          "A fixed-layout loop prefix preserves old parallel operands and complete state");
  for (Word index = prefix; index < count; ++index) {
    advance(expected, index, single, repeat, descending);
  }
  const auto resumed = core.run_until(setup_cycles + count);
  require(resumed.retired == count - prefix && resumed.fault == Fault{} &&
              core.registers() == expected &&
              core.progress() == Progress{.instructions = setup_instructions + count,
                                          .estimated_cycles = setup_cycles + count},
          "Resuming at every instruction boundary preserves the final loop frame and flags");
  const Storage original(code);
  require(storage.p == original.p && storage.x == xdata && storage.y == ydata,
          "Declared code and both borrowed data banks remain unchanged");
}
}

int main() try {
  constexpr std::array selected{Word{0}};
  using Hint = std::optional<std::span<const Word>>;
  for (const unsigned shape : {0U, 1U, 2U, 3U}) {
    const bool single = shape != 0;
    const bool repeat = shape == 2;
    const bool descending = shape == 3;
    const auto code = program(single, repeat, descending);
    Storage storage(code);
    const auto count = single ? iterations : total;
    for (const auto hint : {Hint{}, Hint{selected}}) {
      const auto image = test::compile_program(code, entry, storage.buffers(), hint);
      for (const auto mode : modes) {
        for (const bool below : {false, true}) {
          for (Word prefix = 0; prefix <= count; ++prefix) {
            execute(image,
                    {.mode = mode,
                     .prefix = prefix,
                     .single = single,
                     .below = below,
                     .repeat = repeat,
                     .descending = descending});
          }
        }
      }
    }
  }
  std::cout << test::checks << " fixed-layout loop boundary checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
