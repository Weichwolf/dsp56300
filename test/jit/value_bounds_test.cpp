// SPDX-License-Identifier: GPL-3.0-only
// Public execution versus raw 56-bit arithmetic and quotient/remainder rounding.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word input = 32, output = 80, previous_end = 0x350, previous_count = 19;
constexpr Word loop_flag = Word{1} << 15, loop_bits = Word{3} << 15, pointer_mask = 63;
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word post = (3U << 11) | (3U << 20);
constexpr Word loads = (Word{1} << 15) | (Word{1} << 22);
constexpr Word stores = (2U << 18) | (3U << 16) | (1U << 8) | (1U << 13);
constexpr std::array<Word, 9> samples{
    sign24, sign24 + 1, word_mask, 0, 1, 2, sign24 - 2, sign24 - 1, 0x400001};
constexpr std::size_t data_words = 128;
constexpr Word body_words = 7;
using Data = std::array<Word, data_words>;
using Program = std::array<Word, body_words>;

enum class Shape : std::uint8_t { sum, difference, product, negative_product, mac, negative_mac };

Program program(const Shape shape, const unsigned shifts, const bool rounded, const bool mirrored) {
  constexpr Word nop = static_cast<Word>(isa::Opcode::nop);
  Program code{pair | post | loads,
               static_cast<Word>(isa::Opcode::tfr_x0_a),
               nop,
               nop,
               nop,
               rounded ? static_cast<Word>(isa::Opcode::rnd_a) : nop,
               pair | post | stores};
  if (shape == Shape::sum || shape == Shape::difference) {
    code[2] =
        static_cast<Word>(shape == Shape::sum ? isa::Opcode::add_y0_a : isa::Opcode::sub_y0_a);
  } else if (shape == Shape::mac || shape == Shape::negative_mac) {
    code[2] = static_cast<Word>(isa::Opcode::mac_register_a) | (5U << 4) |
              (shape == Shape::negative_mac ? 4U : 0U);
  } else {
    code[1] = static_cast<Word>(isa::Opcode::mpy_register_a) | (5U << 4) |
              (shape == Shape::negative_product ? 4U : 0U);
  }
  for (unsigned index = 0; index < shifts; ++index) {
    code[3 + index] = static_cast<Word>(isa::Opcode::asr_a);
  }
  if (mirrored) { code.back() &= ~(Word{1} << 16); }
  return code;
}

struct Point {
  std::size_t left = 0, right = 0;
  Word count = 1, mode = 0, sticky = 0;
};

struct Model {
  Registers registers;
  Data x, y;
  bool mirrored = false;
};

void step(Model &model, const Program &code) {
  auto &state = model.registers;
  const auto index = state.pc - start;
  if (index == 0) {
    state.x = (state.x & ~std::uint64_t{word_mask}) | (model.x.at(state.r[0]++) & word_mask);
    state.y = (state.y & ~std::uint64_t{word_mask}) | (model.y.at(state.r[4]++) & word_mask);
  } else if (index + 1 == body_words) {
    model.x.at(state.r[1]++) = register_model::bus_word(state, state.a);
    model.y.at(state.r[5]++) = register_model::bus_word(state, model.mirrored ? state.a : state.b);
  } else {
    const auto before = state;
    sequence_model::alu(state, before, code.at(index));
  }
  ++state.pc;
  if (index + 1 != body_words) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
    return;
  }
  const auto saved = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  const auto outer = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  state.sc -= 2;
  state.sr = (state.sr & ~loop_bits) | (static_cast<Word>(saved) & loop_bits);
  state.la = static_cast<Word>(outer >> word_bits);
  state.lc = static_cast<Word>(outer) & word_mask;
}

class Machine {
public:
  Machine(const Shape shape, const unsigned shifts, const bool rounded, const bool mirrored)
      : code_(program(shape, shifts, rounded, mirrored)),
        mirrored_(mirrored),
        core_(Memory::separate({.program = p_, .x = x_, .y = y_})) {
    std::ranges::copy(code_, p_.begin() + start);
  }

  void check(const Point point, const bool split) {
    constexpr Word poison = 0xab000000;
    x_.fill(poison);
    y_.fill(~poison);
    for (std::size_t index = 0; index < point.count; ++index) {
      x_.at(input + index) = poison | samples[(point.left + index) % samples.size()];
      y_.at(input + index) = poison | samples[(point.right + (3 * index)) % samples.size()];
    }
    auto initial = register_model::seed(point.mode | point.sticky);
    initial.sr |= loop_flag;
    initial.b = 0;
    initial.r[0] = initial.r[4] = input;
    initial.r[1] = initial.r[5] = output;
    initial.sp = pointer_mask;
    initial.la = start + body_words - 1;
    initial.lc = point.count;
    initial.stack[pointer_mask % stack_entries] =
        (std::uint64_t{start} << word_bits) | (initial.sr & ~loop_bits);
    initial.stack[(pointer_mask - 1) % stack_entries] =
        (std::uint64_t{previous_end} << word_bits) | previous_count;
    Model model{.registers = initial, .x = x_, .y = y_, .mirrored = mirrored_};
    const auto original_program = p_;
    core_.set_registers(initial);
    const Word total = point.count * body_words;
    const Word prefix = split ? total / 2 : total;
    for (const auto grant : std::array{prefix, total - prefix}) {
      for (Word index = 0; index < grant; ++index) { step(model, code_); }
      const auto before = core_.state().progress;
      const auto result = core_.run(grant);
      const auto progress = Progress{.instructions = before.instructions + grant,
                                     .estimated_cycles = before.estimated_cycles + grant};
      const bool matched = result.retired == grant && result.fault.kind == FaultKind::none &&
                           result.state == ExecutionState::running && !result.yielded &&
                           core_.registers() == model.registers && result.progress == progress &&
                           x_ == model.x && y_ == model.y && p_ == original_program;
      if (!matched) {
        std::cerr << "bounds pair=" << point.left << '/' << point.right << " count=" << point.count
                  << " RM=" << point.mode << " sticky=" << point.sticky << " grant=" << grant
                  << " registers=" << (core_.registers() == model.registers)
                  << " memory=" << (x_ == model.x && y_ == model.y) << '\n';
      }
      require(matched,
              "Signed value bounds preserve independent rounding, saturation, sticky flags and "
              "full state");
    }
  }

private:
  const Program code_;
  bool mirrored_;
  std::array<Word, memory_words> p_{};
  Data x_{}, y_{};
  Core core_;
};

void cases(Machine &machine) {
  constexpr std::array<Word, 4> sticky{0, flag_s, flag_l, flag_s | flag_l};
  for (const auto rm : {Word{0}, mode_rm}) {
    for (std::size_t left = 0; left < samples.size(); ++left) {
      for (std::size_t right = 0; right < samples.size(); ++right) {
        for (const Word count : {1U, 3U, 4U, 5U, 8U}) {
          const Point point{.left = left,
                            .right = right,
                            .count = count,
                            .mode = rm,
                            .sticky = sticky[(left + right) % sticky.size()]};
          machine.check(point, false);
          machine.check(point, true);
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    for (const Shape shape : {Shape::sum,
                              Shape::difference,
                              Shape::product,
                              Shape::negative_product,
                              Shape::mac,
                              Shape::negative_mac}) {
      for (unsigned shifts = 0; shifts <= 2; ++shifts) {
        for (const bool rounded : {false, true}) {
          for (const bool mirrored : {false, true}) {
            Machine machine(shape, shifts, rounded, mirrored);
            cases(machine);
          }
        }
      }
    }
    std::cout << checks << " independent signed value-bound checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
