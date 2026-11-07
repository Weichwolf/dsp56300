// SPDX-License-Identifier: GPL-3.0-only
// Serial MSB-first carry expectations through the public Core API.
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace dsp56300::test {
namespace {
constexpr std::size_t data_words = 8192;
constexpr Word input_base = 1024, output_x = 4096, output_y = 6144;
constexpr Word pointer_mask = 63, loop_flag = Word{1} << 15, loop_bits = Word{3} << 15;
constexpr Word previous_end = 0x350, previous_count = 19;
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word loads = (Word{1} << 15) | (Word{1} << 22);
constexpr Word post_n = (1U << 11) | (1U << 20);
constexpr Word stores = pair | (3U << 11) | (3U << 20) | (1U << 8) | (1U << 13);
using Data = std::array<Word, data_words>;

struct Point {
  Word count = 1, amount = 1, phase = 0, pointer = 2, mode = 0, modifier = 0;
  bool overlap = false, reverse_store = false;
};

struct Model {
  Registers registers;
  Data x, y;
};

void advance(Registers &state, const unsigned index, const unsigned mode) {
  state.r[index] = address_model::updated({.modifier = state.m[index],
                                           .cursor = state.r[index],
                                           .offset = state.n[index],
                                           .mode = mode});
}

void load(Model &model, const bool y, const unsigned index, const bool shared) {
  auto &state = model.registers;
  auto &bank = y ? state.y : state.x;
  const auto &memory = y && !shared ? model.y : model.x;
  bank = (bank & ~std::uint64_t{word_mask}) | (memory.at(state.r[index]) & word_mask);
  advance(state, index, 1);
}

void step(Model &model, const bool split, const bool shared) {
  auto &state = model.registers;
  const auto index = state.pc - start;
  const Word last = split ? 2U : 1U;
  if (index < last) {
    if (!split || index == 0) { load(model, false, 0, shared); }
    if (!split || index == 1) { load(model, true, split ? 0U : 4U, shared); }
  } else {
    model.x.at(state.r[1]) = static_cast<Word>(state.x) & word_mask;
    auto &memory = shared ? model.x : model.y;
    memory.at(state.r[5]) = static_cast<Word>(state.y) & word_mask;
    advance(state, 1, 3);
    advance(state, 5, 3);
  }
  ++state.pc;
  if (index != last) { return; }
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
  Machine(const bool split, const bool shared, const std::size_t words = data_words)
      : split_(split),
        shared_(shared),
        core_(Memory::separate({.program = program_,
                                .x = std::span(x_).first(words),
                                .y = (shared ? std::span(x_) : std::span(y_)).first(words)})) {
    if (split) {
      program_[start] =
          static_cast<Word>(isa::Opcode::move_x_effective) | (Word{1} << 15) | (1U << 11);
      program_[start + 1] = static_cast<Word>(isa::Opcode::move_y_effective) | (2U << 16) |
                            (Word{1} << 15) | (1U << 11);
      program_[start + 2] = stores;
    } else {
      program_[start] = pair | loads | post_n;
      program_[start + 1] = stores;
    }
    for (std::size_t index = 0; index < x_.size(); ++index) {
      constexpr Word poison = 0xab000000;
      constexpr Word mix = 0x9e3779;
      seed_x_[index] =
          poison | static_cast<Word>((static_cast<std::uint64_t>(index) * mix) & word_mask);
      seed_y_[index] = ~seed_x_[index];
    }
  }

  void check(Point point, const std::span<const Word> grants) {
    x_ = seed_x_;
    y_ = seed_y_;
    auto state = register_model::seed(point.mode);
    state.sr |= loop_flag;
    state.la = start + (split_ ? 2U : 1U);
    state.lc = point.count;
    state.sp = point.pointer;
    state.r[0] = input_base + point.phase;
    state.r[4] = input_base + ((point.phase + 1) % input_base);
    state.r[1] = point.overlap ? input_base + 1 : output_x;
    state.r[5] = output_y;
    state.n[0] = state.n[4] = point.amount;
    state.m[0] = state.m[4] = point.modifier;
    if (point.reverse_store) { state.m[1] = state.m[5] = 0; }
    state.stack[state.sp % stack_entries] =
        (std::uint64_t{start} << word_bits) | (state.sr & ~loop_bits);
    state.stack[(state.sp + pointer_mask) % stack_entries] =
        (std::uint64_t{previous_end} << word_bits) | previous_count;
    Model model{.registers = state, .x = x_, .y = y_};
    const auto program = program_;
    core_.set_registers(state);
    for (const auto grant : grants) {
      for (Word index = 0; index < grant; ++index) { step(model, split_, shared_); }
      const auto before = core_.state().progress;
      const auto result = core_.run(grant);
      const auto expected = Progress{.instructions = before.instructions + grant,
                                     .estimated_cycles = before.estimated_cycles + grant};
      const bool matched = result.retired == grant && result.fault.kind == FaultKind::none &&
                           result.state == ExecutionState::running && !result.yielded &&
                           core_.registers() == model.registers && result.progress == expected &&
                           x_ == model.x && y_ == model.y && program_ == program;
      if (!matched) {
        std::cerr << "reverse count=" << point.count << " N=" << point.amount
                  << " phase=" << point.phase << " grant=" << grant << " split=" << split_
                  << " shared=" << shared_ << " overlap=" << point.overlap
                  << " registers=" << (core_.registers() == model.registers)
                  << " memory=" << (x_ == model.x && y_ == model.y) << '\n';
      }
      require(matched,
              "Reverse streams retain serial carry, all registers/RAM, loop frames and progress");
    }
  }

  [[nodiscard]] Word length() const { return split_ ? 3U : 2U; }

private:
  bool split_, shared_;
  std::array<Word, memory_words> program_{};
  Data x_{}, y_{}, seed_x_{}, seed_y_{};
  Core core_;
};

void cases(Machine &machine) {
  constexpr std::array<Word, 15> counts{1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17, 31, 32, 33, 129};
  for (const Word n : {1U, 2U, 4U, 16U, 128U, 512U, 0U, 3U, 5U, 7U}) {
    for (const Word phase : {0U, 1U, 17U, input_base - 1}) {
      for (const auto count : counts) {
        Point point{.count = count, .amount = n, .phase = phase};
        const std::array whole{count * machine.length()};
        machine.check(point, whole);
        point.pointer = pointer_mask;
        constexpr Word high_reverse = 0xab0000;
        point.modifier = high_reverse;
        const auto partial = std::min(whole[0], Word{7});
        const std::array split{partial, whole[0] - partial};
        machine.check(point, split);
        point.overlap = true;
        machine.check(point, whole);
      }
    }
  }
  // A retained region must recheck a runtime change in M and raw N.
  for (const Word modifier : {0U, word_mask, 0U}) {
    const std::array whole{machine.length() * 8};
    machine.check({.count = 8, .amount = 1, .modifier = modifier}, whole);
  }
  for (const Word count : {1U, 2U, 5U, 8U}) {
    const std::array whole{machine.length() * count};
    machine.check({.count = count, .amount = 16, .reverse_store = true}, whole);
  }
  const std::array long_run{machine.length() * input_base};
  machine.check({.count = input_base, .amount = 512}, long_run);
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const bool split : {false, true}) {
      for (const bool shared : {false, true}) {
        dsp56300::test::Machine machine(split, shared);
        dsp56300::test::cases(machine);
        constexpr std::size_t truncated_words = 8000;
        dsp56300::test::Machine truncated(split, shared, truncated_words);
        const std::array grants{truncated.length() * 4};
        truncated.check({.count = 4, .amount = 4096}, grants);
      }
    }
    std::cout << dsp56300::test::checks << " independent reverse stream checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
