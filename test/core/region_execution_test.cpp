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
constexpr Word total = iterations * body_size, loop_flag = Word{1} << 15;
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
constexpr std::array<Word, iterations + 1> xdata{
    1, test::sign24, test::sign24 - 1, 0, 3, word_mask, 7, 9, 11, 13, 17, 19, 23, 29, 31, 37, 41};
constexpr std::array<Word, iterations + 1> ydata{
    word_mask, 1, test::sign24, 9, test::sign24 - 1, 0, 11, 5, 7, 13, 17, 19, 23, 29, 31, 37, 41};
constexpr std::array modes{0U,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm,
                           test::mode_rm | test::mode_down,
                           test::mode_rm | test::mode_up};

struct Storage {
  Storage() { std::ranges::copy(source, p.begin() + entry); }

  [[nodiscard]] Buffers buffers() { return {.program = p, .x = x, .y = y}; }

  std::array<Word, 64> p{};
  std::array<Word, iterations + 1> x = xdata, y = ydata;
};

Registers initial(const Word mode) {
  Registers state;
  state.pc = body;
  state.sr = mode | test::flag_c | test::flag_s | loop_flag;
  state.a = test::sign56 - 2;
  state.x = seed_x;
  state.y = seed_y;
  state.m.fill(word_mask);
  state.la = body + body_size - 1;
  state.lc = iterations;
  state.sp = state.sc = 2;
  state.stack[2] = (std::uint64_t{body} << test::word_bits) | (state.sr & ~loop_flag);
  return state;
}

void advance(Registers &state, const Word index) {
  const auto old = state;
  const auto operation = source[2 + (index % body_size)];
  if (index % body_size == 0) {
    test::register_model::write(state, 4, xdata.at(old.r[0]));
    test::register_model::write(state, 6, ydata.at(old.r[4]));
    ++state.r[0];
    ++state.r[4];
  }
  test::sequence_model::alu(state, old, operation);
  ++state.pc;
  if (index % body_size == body_size - 1) {
    if (state.lc > 1) {
      --state.lc;
      state.pc = body;
    } else {
      test::loop_frames::complete(state);
    }
  }
}

struct Input {
  Word mode, prefix;
};

void execute(const NativeImage &image, const Input input) {
  const auto [mode, prefix] = input;
  Storage storage;
  Core core(Memory::separate(storage.buffers()));
  auto expected = initial(mode);
  core.set_registers(expected);
  core.install(image, 0);
  for (Word index = 0; index < prefix; ++index) { advance(expected, index); }
  const auto first = core.run(prefix);
  require(first.retired == prefix && first.fault == Fault{} && core.registers() == expected &&
              core.progress() == Progress{.instructions = prefix, .estimated_cycles = prefix},
          "A fixed-layout DO prefix preserves old parallel operands and complete state");
  for (Word index = prefix; index < total; ++index) { advance(expected, index); }
  const auto resumed = core.run_until(total);
  require(resumed.retired == total - prefix && resumed.fault == Fault{} &&
              core.registers() == expected &&
              core.progress() == Progress{.instructions = total, .estimated_cycles = total},
          "Resuming at every instruction boundary preserves the final loop frame and flags");
  const Storage original;
  require(storage.p == original.p && storage.x == xdata && storage.y == ydata,
          "Declared code and both borrowed data banks remain unchanged");
}
}

int main() try {
  Storage storage;
  constexpr std::array selected{Word{0}};
  using Hint = std::optional<std::span<const Word>>;
  for (const auto hint : {Hint{}, Hint{selected}}) {
    const auto image = test::compile_program(source, entry, storage.buffers(), hint);
    for (const auto mode : modes) {
      for (Word prefix = 0; prefix <= total; ++prefix) {
        execute(image, {.mode = mode, .prefix = prefix});
      }
    }
  }
  std::cout << test::checks << " fixed-layout loop boundary checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
