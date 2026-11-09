// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
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
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word input = 32, output = 512, loop_flag = Word{1} << 15, pointer_mask = 63;
constexpr Word previous_end = 0x350, previous_count = 19;
constexpr std::array<Word, 9> samples{
    sign24, sign24 + 1, word_mask, 0, 1, 2, sign24 - 2, sign24 - 1, 0x400001};
using Data = std::array<Word, memory_words>;

std::vector<Word> program(const Word period) {
  constexpr Word load = static_cast<Word>(isa::Opcode::move_y_effective) | (2U << 16) | (3U << 11) |
                        (4U << 8) | 0x8000U;
  constexpr Word store = static_cast<Word>(isa::Opcode::move_long) | (6U << 16) | (8U << 17) |
                         (3U << 11) | (1U << 8) | 0x4000U;
  std::vector<Word> result;
  for (Word index = 0; index < period; ++index) {
    result.push_back(load);
    result.push_back(static_cast<Word>(index == 0 ? isa::Opcode::tfr_y0_a : isa::Opcode::add_y0_a));
  }
  result.push_back(static_cast<Word>(isa::Opcode::rnd_a));
  result.push_back(store);
  return result;
}

struct Model {
  Registers registers;
  Data x, y;
};

void step(Model &model, const Word period, const std::vector<Word> &code) {
  auto &state = model.registers;
  const auto index = state.pc - start;
  if (index < 2 * period && index % 2 == 0) {
    const auto address = state.r[4];
    state.r[4] = address_model::updated(
        {.modifier = state.m[4], .cursor = address, .offset = state.n[4], .mode = 3});
    state.y = (state.y & ~std::uint64_t{word_mask}) | (model.y.at(address) & word_mask);
  } else if (index + 1 == code.size()) {
    model.x.at(state.r[1]++) = register_model::bus_word(state, state.a);
  } else {
    const auto before = state;
    if ((state.sr & saturation_model::mode) != 0) {
      saturation_model::alu(state, before, code.at(index));
    } else {
      sequence_model::alu(state, before, code.at(index));
    }
  }
  ++state.pc;
  if (index + 1 != code.size()) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
  } else {
    loop_frames::complete(state);
  }
}

struct Point {
  Word count, mode;
  unsigned variant;
  std::size_t phase;
};

class Machine {
public:
  explicit Machine(const Word period)
      : period_(period),
        code_(program(period)),
        core_(Memory::separate({.program = p_, .x = x_, .y = y_})) {
    std::ranges::copy(code_, p_.begin() + start);
  }

  void check(const Point point, const bool split) {
    constexpr Word tag = 0xab000000;
    constexpr std::array<Word, 4> sticky{0, flag_s, flag_l, flag_s | flag_l};
    x_.fill(tag);
    y_.fill(~tag);
    for (Word index = 0; index <= period_; ++index) {
      y_.at(input + index) = tag | samples[(point.phase + index) % samples.size()];
    }
    auto initial = register_model::seed(point.mode | sticky[point.phase % sticky.size()]);
    initial.sr |= loop_flag;
    initial.r[4] = input + (point.variant == 1 ? 1U : 0U);
    initial.m[4] = period_ - 1 + (point.variant == 2 ? 1U : 0U);
    initial.r[1] = output;
    initial.la = start + static_cast<Word>(code_.size()) - 1;
    initial.lc = point.count;
    initial.sp = pointer_mask;
    initial.stack[pointer_mask % stack_entries] =
        (std::uint64_t{start} << word_bits) | (initial.sr & ~loop_frames::loop_bits);
    initial.stack[(pointer_mask - 1) % stack_entries] =
        (std::uint64_t{previous_end} << word_bits) | previous_count;
    Model model{.registers = initial, .x = x_, .y = y_};
#if defined(__aarch64__)
    if (!split) { direct(model, point.variant); }
#endif
    core_.set_registers(initial);
    const auto total = point.count * static_cast<Word>(code_.size());
    const auto prefix = split ? total / 2 : total;
    for (const auto grant : std::array{prefix, total - prefix}) {
      for (Word index = 0; index < grant; ++index) { step(model, period_, code_); }
      const auto before = core_.state().progress;
      const auto run = core_.run(grant);
      const bool matched =
          run.retired == grant && run.fault.kind == FaultKind::none &&
          run.state == ExecutionState::running && !run.yielded &&
          core_.registers() == model.registers && x_ == model.x && y_ == model.y &&
          run.progress == Progress{.instructions = before.instructions + grant,
                                   .estimated_cycles = before.estimated_cycles + grant};
      if (!matched) {
        std::cerr << "cycle period=" << period_ << " count=" << point.count
                  << " mode=" << point.mode << " variant=" << point.variant << " grant=" << grant
                  << " registers=" << (core_.registers() == model.registers)
                  << " RAM=" << (x_ == model.x && y_ == model.y) << '\n';
      }
      require(matched,
              "Read cycles preserve independently addressed RAM, flags and every API prefix");
    }
  }

private:
#if defined(__aarch64__)
  void direct(const Model &seed, const unsigned variant) {
    const detail::JitRuntime runtime{.memory = {{{.data = x_.data(), .words = x_.size()},
                                                 {.data = y_.data(), .words = y_.size()}}}};
    const auto mode = seed.registers.sr;
    auto &region = regions_[((mode & mode_rm) != 0 ? 1U : 0U) +
                            ((mode & saturation_model::mode) != 0 ? 2U : 0U)];
    if (!region) {
      const auto operations = detail::parse_block(p_, seed.registers, runtime);
      region = detail::resident_region_plan(p_, seed.registers, runtime, operations);
      require(region && region->periods[4] == period_,
              "The native plan proves a complete read cycle");
      for (const unsigned lanes : {1U, 2U, 4U}) {
        region->native[lanes / 2] = arena_.append(detail::emit_resident_region(*region, lanes));
      }
    }
    detail::Cpu cpu;
    cpu.reg = seed.registers;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    const auto count = seed.registers.lc;
    auto lanes = count >= 4 ? 4U : 2U;
    if (count < 2) { lanes = 1; }
    const auto expected_retirement = variant == 0 ? (count - count % lanes) * code_.size() : 0;
    detail::NativeContext context{.runtime = runtime,
                                  .budget = count * static_cast<Word>(code_.size())};
    const auto retired = detail::run_resident_region(&cpu, &context, region.get());
    auto expected = seed;
    for (Word index = 0; index < expected_retirement; ++index) { step(expected, period_, code_); }
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(
        retired == expected_retirement && cpu.reg == expected.registers &&
            cpu.instructions == retired && cpu.cycles == retired && x_ == expected.x &&
            y_ == expected.y,
        "Actual native read cycles and rejected M/R mutations retain complete independent state");
    x_ = seed.x;
    y_ = seed.y;
  }

  detail::ExecutableArena arena_;
  std::array<std::unique_ptr<detail::ResidentRegion>, 4> regions_;
#endif
  Word period_;
  std::vector<Word> code_;
  Data p_{}, x_{}, y_{};
  Core core_;
};

void cases(Machine &machine) {
  for (const auto mode :
       {Word{0}, mode_rm, saturation_model::mode, saturation_model::mode | mode_rm}) {
    for (const Word count : {1U, 2U, 3U, 4U, 5U, 8U, 19U, 20U, 21U, 92U, 257U}) {
      for (std::size_t phase = 0; phase < samples.size(); ++phase) {
        for (const unsigned variant : {0U, 1U, 2U}) {
          const Point point{.count = count, .mode = mode, .variant = variant, .phase = phase};
          machine.check(point, false);
          machine.check(point, true);
        }
      }
    }
  }
}
}
}

int main() {
  try {
    constexpr std::array<dsp56300::Word, 4> periods{2, 3, 19, 20};
    for (const auto period : periods) {
      dsp56300::test::Machine machine(period);
      dsp56300::test::cases(machine);
    }
    std::cout << dsp56300::test::checks << " independent read-cycle checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
