// SPDX-License-Identifier: GPL-3.0-only
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word input = 32, state_output = 4096, output = 8192, ram_words = 16384;
constexpr Word loop_flag = Word{1} << 15, pointer_mask = 63, previous_end = 0x350,
               previous_count = 19;
constexpr std::array<Word, 9> samples{
    sign24, sign24 + 1, word_mask, 0, 1, 2, sign24 - 2, sign24 - 1, 0x400001};
using Data = std::array<Word, ram_words>;

enum class Action : std::uint8_t { load, alu, store };
enum class Alias : std::uint8_t { separate, state, stores };

struct Shape {
  Word fields;
  bool store_first, rounded;
};

struct Step {
  Word word;
  Action action;
  unsigned cursor = 0;
};

struct Port {
  unsigned reg, cursor;
  bool load;
};

Word memory_word(const Port port) {
  constexpr Word effective = 0x4000;
  constexpr Word load = 0x8000;
  constexpr unsigned high_mask = 24;
  constexpr unsigned high_shift = 17;
  return static_cast<Word>(isa::Opcode::move_long) | ((port.reg & 7U) << 16) |
         ((port.reg & high_mask) << high_shift) | effective | (3U << 11) | (port.cursor << 8) |
         (port.load ? load : 0U);
}

std::vector<Step> program(const Shape shape) {
  std::vector<Step> result;
  for (Word column = 0; column < shape.fields; ++column) {
    if (shape.store_first) {
      result.push_back({.word = static_cast<Word>(isa::Opcode::tfr_y0_a), .action = Action::alu});
      result.push_back({.word = memory_word({.reg = 14, .cursor = 3, .load = false}),
                        .action = Action::store,
                        .cursor = 3});
    }
    result.push_back({.word = memory_word({.reg = 4, .cursor = 2, .load = true}),
                      .action = Action::load,
                      .cursor = 2});
    if (!shape.store_first) {
      result.push_back({.word = static_cast<Word>(isa::Opcode::tfr_x0_a), .action = Action::alu});
    }
    result.push_back({.word = static_cast<Word>(shape.store_first ? isa::Opcode::add_x0_a
                                                                  : isa::Opcode::add_y0_a),
                      .action = Action::alu});
    if (shape.rounded) {
      result.push_back({.word = static_cast<Word>(isa::Opcode::rnd_a), .action = Action::alu});
    }
    const auto cursor = shape.store_first ? 1U : 3U;
    result.push_back({.word = memory_word({.reg = 14, .cursor = cursor, .load = false}),
                      .action = Action::store,
                      .cursor = cursor});
  }
  return result;
}

struct Model {
  Registers registers;
  Data x;
};

void step(Model &model, const std::vector<Step> &code) {
  auto &state = model.registers;
  const auto index = state.pc - start;
  const auto instruction = code.at(index);
  if (instruction.action == Action::load) {
    state.x = (state.x & ~std::uint64_t{word_mask}) |
              (model.x.at(state.r[instruction.cursor]++) & word_mask);
  } else if (instruction.action == Action::store) {
    model.x.at(state.r[instruction.cursor]++) = register_model::bus_word(state, state.a);
  } else {
    const auto before = state;
    if ((state.sr & saturation_model::mode) != 0) {
      saturation_model::alu(state, before, instruction.word);
    } else {
      sequence_model::alu(state, before, instruction.word);
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
  Alias alias;
  std::size_t phase;
};

class Machine {
public:
  explicit Machine(const Shape shape)
      : shape_(shape),
        code_(program(shape)),
        core_(Memory::separate({.program = p_, .x = x_, .y = y_})) {
    for (std::size_t index = 0; index < code_.size(); ++index) {
      p_.at(start + index) = code_[index].word;
    }
  }

  void check(const Point point, const bool split) {
    constexpr Word tag = 0xab000000;
    constexpr std::array<Word, 4> sticky{0, flag_s, flag_l, flag_s | flag_l};
    const auto read_base = input + static_cast<Word>(point.phase % 4);
    x_.fill(tag);
    y_.fill(~tag);
    for (std::size_t index = 0; index < std::size_t{point.count} * shape_.fields; ++index) {
      x_.at(read_base + index) = tag | samples[(point.phase + (3 * index)) % samples.size()];
    }
    auto initial = register_model::seed(point.mode | sticky[point.phase % sticky.size()]);
    initial.sr |= loop_flag;
    initial.y =
        (initial.y & ~std::uint64_t{word_mask}) | samples[(point.phase + 2) % samples.size()];
    initial.r[2] = read_base;
    initial.r[3] = point.alias == Alias::state
                       ? read_base
                       : state_output + static_cast<Word>((point.phase + 1) % 4);
    initial.r[1] = point.alias == Alias::stores ? initial.r[3]
                                                : output + static_cast<Word>((point.phase + 2) % 4);
    initial.la = start + static_cast<Word>(code_.size()) - 1;
    initial.lc = point.count;
    initial.sp = pointer_mask;
    initial.stack[pointer_mask % stack_entries] =
        (std::uint64_t{start} << word_bits) | (initial.sr & ~loop_frames::loop_bits);
    initial.stack[(pointer_mask - 1) % stack_entries] =
        (std::uint64_t{previous_end} << word_bits) | previous_count;
    Model model{.registers = initial, .x = x_};
    const auto original_program = p_;
    const auto original_y = y_;
#if defined(__aarch64__)
    if (!split) { direct(model, point.alias); }
#endif
    core_.set_registers(initial);
    const auto total = point.count * static_cast<Word>(code_.size());
    const auto prefix = split ? total / 2 : total;
    for (const auto grant : std::array{prefix, total - prefix}) {
      for (Word index = 0; index < grant; ++index) { step(model, code_); }
      const auto before = core_.state().progress;
      const auto run = core_.run(grant);
      const bool matched =
          run.retired == grant && run.fault.kind == FaultKind::none &&
          run.state == ExecutionState::running && !run.yielded &&
          core_.registers() == model.registers && x_ == model.x && y_ == original_y &&
          p_ == original_program &&
          run.progress == Progress{.instructions = before.instructions + grant,
                                   .estimated_cycles = before.estimated_cycles + grant};
      if (!matched) {
        std::cerr << "interleaved fields=" << shape_.fields << " early=" << shape_.store_first
                  << " round=" << shape_.rounded << " count=" << point.count
                  << " mode=" << point.mode << " alias=" << static_cast<unsigned>(point.alias)
                  << " grant=" << grant << " registers=" << (core_.registers() == model.registers)
                  << " RAM=" << (x_ == model.x) << '\n';
      }
      require(
          matched,
          "Interleaved execution preserves every independently ordered RAM access and register");
    }
  }

private:
#if defined(__aarch64__)
  void direct(const Model &seed, const Alias alias) {
    const detail::JitRuntime runtime{.memory = {{{.data = x_.data(), .words = x_.size()},
                                                 {.data = y_.data(), .words = y_.size()}}}};
    const auto mode = seed.registers.sr;
    auto &region = regions_[((mode & mode_rm) != 0 ? 1U : 0U) +
                            ((mode & saturation_model::mode) != 0 ? 2U : 0U)];
    if (!region) {
      const auto operations = detail::parse_block(p_, seed.registers, runtime);
      region = detail::resident_region_plan(p_, seed.registers, runtime, operations);
      require(region != nullptr, "The interleaved trace admits native value regions");
      for (const unsigned lanes : {1U, 2U, 4U}) {
        region->native[lanes / 2] = arena_.append(detail::emit_resident_region(*region, lanes));
      }
      require(!region->interleaved.empty(),
              "The tested native bytes include structured RAM accesses");
    }
    detail::Cpu cpu;
    cpu.reg = seed.registers;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    const auto count = seed.registers.lc;
    auto lanes = count >= 4 ? 4U : 2U;
    if (count < 2) { lanes = 1; }
    const bool reject = lanes == 4 && shape_.store_first && alias != Alias::separate;
    const auto expected_retirement = reject ? 0 : (count - count % lanes) * code_.size();
    detail::NativeContext context{.runtime = runtime,
                                  .budget = count * static_cast<Word>(code_.size())};
    const auto retired = detail::run_resident_region(&cpu, &context, region.get());
    auto expected = seed;
    for (Word index = 0; index < expected_retirement; ++index) { step(expected, code_); }
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    const bool matched = retired == expected_retirement && cpu.reg == expected.registers &&
                         cpu.instructions == retired && cpu.cycles == retired && x_ == expected.x;
    if (!matched) {
      std::cerr << "direct interleaved fields=" << shape_.fields << " early=" << shape_.store_first
                << " round=" << shape_.rounded << " count=" << count << " mode=" << mode
                << " alias=" << static_cast<unsigned>(alias) << " retired=" << retired
                << " expected=" << expected_retirement
                << " registers=" << (cpu.reg == expected.registers) << " RAM=" << (x_ == expected.x)
                << '\n';
    }
    require(matched,
            "Structured native bytes and read-only alias rejections match full independent state");
    x_ = seed.x;
  }

  detail::ExecutableArena arena_;
  std::array<std::unique_ptr<detail::ResidentRegion>, 4> regions_;
#endif
  Shape shape_;
  std::vector<Step> code_;
  std::array<Word, memory_words> p_{};
  Data x_{};
  std::array<Word, 64> y_{};
  Core core_;
};

void cases(Machine &machine) {
  for (const auto mode :
       {Word{0}, mode_rm, saturation_model::mode, saturation_model::mode | mode_rm}) {
    for (const Word count : {1U, 2U, 3U, 4U, 5U, 8U, 19U, 20U, 21U, 257U}) {
      for (std::size_t phase = 0; phase < samples.size(); ++phase) {
        for (const auto alias : {Alias::separate, Alias::state, Alias::stores}) {
          const Point point{.count = count, .mode = mode, .alias = alias, .phase = phase};
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
    for (const auto fields : {4U, 8U}) {
      for (const bool early : {false, true}) {
        for (const bool rounded : {false, true}) {
          dsp56300::test::Machine machine(
              {.fields = fields, .store_first = early, .rounded = rounded});
          dsp56300::test::cases(machine);
        }
      }
    }
    std::cout << dsp56300::test::checks << " independent structured-memory checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
