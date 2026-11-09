// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#if defined(__aarch64__)
#include "architecture.hpp"
#include "jit.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "jit/resident_region.hpp"

#include <memory>
#endif

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word input = 32, output = 512, body_words = 7;
constexpr Word loop_flag = Word{1} << 15, pointer_mask = 63;
constexpr Word previous_end = 0x350, previous_count = 19;
constexpr std::array<Word, 9> samples{
    sign24, sign24 + 1, word_mask, 0, 1, 2, sign24 - 2, sign24 - 1, 0x400001};
using Data = std::array<Word, memory_words>;
using Program = std::array<Word, body_words>;

Program program(const bool target_b) {
  constexpr Word post_loads = (3U << 11) | (3U << 20) | (1U << 15) | (1U << 22);
  constexpr Word source_a = 14;
  constexpr Word source_b = 15;
  constexpr Word target_x0 = 4;
  constexpr Word register_move = static_cast<Word>(isa::Opcode::move_none);
  constexpr Word store = static_cast<Word>(isa::Opcode::move_x_effective) | (3U << 11) | (1U << 8);
  const auto arithmetic_destination = target_b ? 0U : 8U;
  return {static_cast<Word>(isa::Opcode::move_xy) | post_loads,
          static_cast<Word>(isa::Opcode::tfr_x0_a) | arithmetic_destination,
          static_cast<Word>(isa::Opcode::add_y0_a) | arithmetic_destination,
          static_cast<Word>(isa::Opcode::rnd_a) | arithmetic_destination,
          register_move | ((target_b ? source_a : source_b) << 13) | (target_x0 << 8),
          store,
          static_cast<Word>(isa::Opcode::add_x0_a) | (target_b ? 8U : 0U)};
}

struct Model {
  Registers registers;
  Data x, y;
  bool target_b;
};

void step(Model &model, const Program &code) {
  auto &state = model.registers;
  const auto index = state.pc - start;
  if (index == 0) {
    state.x = (state.x & ~std::uint64_t{word_mask}) | (model.x.at(state.r[0]++) & word_mask);
    state.y = (state.y & ~std::uint64_t{word_mask}) | (model.y.at(state.r[4]++) & word_mask);
  } else if (index == 4) {
    const auto value = register_model::bus_word(state, model.target_b ? state.a : state.b);
    state.x = (state.x & ~std::uint64_t{word_mask}) | value;
  } else if (index == 5) {
    model.x.at(state.r[1]++) = static_cast<Word>(state.x) & word_mask;
  } else {
    const auto before = state;
    sequence_model::alu(state, before, code.at(index));
  }
  ++state.pc;
  if (index + 1 != body_words) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
  } else {
    loop_frames::complete(state);
  }
}

struct Point {
  Word count, mode;
  std::uint64_t accumulator;
  std::size_t phase;
  bool constant = false;
};

class Machine {
public:
  explicit Machine(const bool target_b)
      : target_b_(target_b),
        code_(program(target_b)),
        core_(Memory::separate({.program = p_, .x = x_, .y = y_})) {
    std::ranges::copy(code_, p_.begin() + start);
  }

  void check(const Point point, const bool split) {
    constexpr Word tag = 0xab000000;
    x_.fill(tag);
    y_.fill(~tag);
    for (std::size_t index = 0; index < point.count; ++index) {
      x_.at(input + index) =
          tag | samples[(point.phase + (point.constant ? 0 : index)) % samples.size()];
      y_.at(input + index) =
          tag | samples[(point.phase + (point.constant ? 0 : 3 * index)) % samples.size()];
    }
    constexpr std::array<Word, 4> sticky{0, flag_s, flag_l, flag_s | flag_l};
    auto initial = register_model::seed(point.mode | sticky[point.phase % sticky.size()]);
    initial.sr |= loop_flag;
    (target_b_ ? initial.b : initial.a) = point.accumulator;
    initial.r[0] = initial.r[4] = input;
    initial.r[1] = output;
    initial.la = start + body_words - 1;
    initial.lc = point.count;
    initial.sp = pointer_mask;
    initial.stack[pointer_mask % stack_entries] =
        (std::uint64_t{start} << word_bits) | (initial.sr & ~loop_frames::loop_bits);
    initial.stack[(pointer_mask - 1) % stack_entries] =
        (std::uint64_t{previous_end} << word_bits) | previous_count;
    Model model{.registers = initial, .x = x_, .y = y_, .target_b = target_b_};
#if defined(__aarch64__)
    if (!split) { direct(model); }
#endif
    core_.set_registers(initial);
    const auto total = point.count * body_words;
    const auto prefix = split ? total / 2 : total;
    for (const auto grant : std::array{prefix, total - prefix}) {
      for (Word index = 0; index < grant; ++index) { step(model, code_); }
      const auto before = core_.state().progress;
      const auto run = core_.run(grant);
      const bool matched =
          run.retired == grant && run.fault.kind == FaultKind::none &&
          run.state == ExecutionState::running && !run.yielded &&
          core_.registers() == model.registers && x_ == model.x && y_ == model.y &&
          run.progress == Progress{.instructions = before.instructions + grant,
                                   .estimated_cycles = before.estimated_cycles + grant};
      if (!matched) {
        std::cerr << "reduction B=" << target_b_ << " count=" << point.count
                  << " initial=" << point.accumulator << " mode=" << point.mode
                  << " grant=" << grant << " registers=" << (core_.registers() == model.registers)
                  << " RAM=" << (x_ == model.x && y_ == model.y) << '\n';
      }
      require(matched,
              "Ordered reduction preserves every register, flag, RAM word and prefix cost");
    }
  }

private:
#if defined(__aarch64__)
  void direct(const Model &seed) {
    const detail::JitRuntime runtime{.memory = {{{.data = x_.data(), .words = x_.size()},
                                                 {.data = y_.data(), .words = y_.size()}}}};
    auto &region = regions_[(seed.registers.sr & mode_rm) != 0 ? 1U : 0U];
    if (!region) {
      const auto operations = detail::parse_block(p_, seed.registers, runtime);
      region = detail::resident_region_plan(p_, seed.registers, runtime, operations);
      require(region && region->graph.reduction.valid(),
              "The independent trace admits a reduction");
      for (const unsigned lanes : {1U, 2U, 4U}) {
        region->native[lanes / 2] = arena_.append(detail::emit_resident_region(*region, lanes));
      }
    }
    detail::Cpu cpu;
    cpu.reg = seed.registers;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    auto iterations = std::min(seed.registers.lc, Word{256});
    auto lanes = iterations >= 4 ? 4U : 2U;
    if (iterations < 2) { lanes = 1; }
    iterations -= iterations % lanes;
    const auto value = signed56(target_b_ ? seed.registers.b : seed.registers.a);
    constexpr auto unit = std::int64_t{1} << word_bits;
    const auto lower = -static_cast<std::int64_t>(sign56) +
                       (std::int64_t{iterations} * static_cast<std::int64_t>(sign24) * unit);
    const auto upper = static_cast<std::int64_t>(sign56) - 1 -
                       (std::int64_t{iterations} * static_cast<std::int64_t>(sign24 - 1) * unit);
    const auto expected_retirement = value >= lower && value <= upper ? iterations * body_words : 0;
    detail::NativeContext context{.runtime = runtime, .budget = seed.registers.lc * body_words};
    const auto retired = detail::run_resident_region(&cpu, &context, region.get());
    auto expected = seed;
    for (Word index = 0; index < expected_retirement; ++index) { step(expected, code_); }
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(
        retired == expected_retirement && cpu.reg == expected.registers &&
            cpu.instructions == retired && cpu.cycles == retired && x_ == expected.x &&
            y_ == expected.y,
        "Actual native reductions and headroom rejections match independent sequential execution");
    x_ = seed.x;
    y_ = seed.y;
  }

  detail::ExecutableArena arena_;
  std::array<std::unique_ptr<detail::ResidentRegion>, 2> regions_;
#endif
  bool target_b_;
  Program code_;
  Data p_{}, x_{}, y_{};
  Core core_;
};

void cases(Machine &machine) {
  constexpr std::array<std::uint64_t, 10> accumulators{0,
                                                       1,
                                                       accumulator_mask,
                                                       sign56,
                                                       sign56 - 1,
                                                       sign56 + 1,
                                                       sign56 / 2,
                                                       sign56 + (sign56 / 2),
                                                       (std::uint64_t{1} << 32) - 1,
                                                       std::uint64_t{1} << 32};
  for (const auto mode : {Word{0}, mode_rm}) {
    for (const Word count : {1U, 2U, 3U, 4U, 5U, 8U, 19U, 20U, 21U, 92U, 255U, 256U, 257U, 511U}) {
      for (const auto accumulator : accumulators) {
        for (std::size_t phase = 0; phase < samples.size(); ++phase) {
          for (const bool constant : {false, true}) {
            const Point point{.count = count,
                              .mode = mode,
                              .accumulator = accumulator,
                              .phase = phase,
                              .constant = constant};
            machine.check(point, false);
            machine.check(point, true);
          }
        }
      }
    }
  }
}
}
}

int main() {
  try {
    for (const bool target_b : {false, true}) {
      dsp56300::test::Machine machine(target_b);
      dsp56300::test::cases(machine);
    }
    std::cout << dsp56300::test::checks << " independent reduction checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
