// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/emitter.hpp"
#include "native/direct.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array values{std::uint64_t{0},
                            std::uint64_t{1},
                            sign56 - 1,
                            sign56,
                            accumulator_mask,
                            saturation_model::positive,
                            saturation_model::positive + 1,
                            saturation_model::negative,
                            saturation_model::negative - 1};

struct Step {
  bool target_b = false;
  unsigned source = 0, destination = 0;
};

Word encoding(const Step step) {
  return static_cast<Word>(step.target_b ? isa::Opcode::asl_b : isa::Opcode::asl_a) |
         (step.source << 13) | (step.destination << 8);
}

void apply(Registers &state, const Step step) {
  const auto old = state;
  if (step.source != 0) {
    const auto word = register_model::read(state, step.source);
    register_model::write(state, step.destination, word);
  }
  shift_model::apply(
      state, old, {.left = true, .source_b = step.target_b, .destination_b = step.target_b});
  ++state.pc;
}

std::uint32_t forbidden(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

class Machine {
public:
  explicit Machine(const std::span<const Step> steps)
      : steps_(steps),
        program_(program(steps)),
        core_(Memory::separate({.program = program_, .x = x_, .y = y_})) {}

  void check(const Registers &initial, const Word count, const bool deadline) {
    auto expected = initial;
    for (const auto step : steps_.first(count)) { apply(expected, step); }
    core_.set_registers(initial);
    const auto before = core_.progress();
    const auto result =
        deadline ? core_.run_until(before.estimated_cycles + count) : core_.run(count);
    require(result.retired == count && result.fault.kind == FaultKind::none && !result.yielded &&
                core_.registers() == expected &&
                result.progress == Progress{.instructions = before.instructions + count,
                                            .estimated_cycles = before.estimated_cycles + count},
            "ASL prefixes/deadlines retain independent bitwise results, all registers and cycles");
    require(x_ == std::array<Word, memory_words>{} && y_ == x_ && program_ == program(steps_),
            "ASL and parallel register transfers never modify supplied memory");
  }

private:
  static std::vector<Word> program(const std::span<const Step> steps) {
    std::vector<Word> words;
    for (const auto step : steps) { words.push_back(encoding(step)); }
    return words;
  }

  std::span<const Step> steps_;
  std::vector<Word> program_;
  std::array<Word, memory_words> x_{}, y_{};
  Core core_;
};

void native_case(const native::Direct &code,
                 const std::span<const Step> steps,
                 const Registers &initial) {
  auto expected = initial;
  for (const auto step : steps) { apply(expected, step); }
  detail::Cpu cpu;
  cpu.reg = initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  unsigned calls = 0;
  bool interrupted = false;
  std::array<std::uint64_t, 4> latched{};
  detail::NativeContext context{.runtime = {.owner = &calls,
                                            .latched = &latched,
                                            .execute = forbidden,
                                            .moves = forbidden,
                                            .memory = {}},
                                .exit_requested = &interrupted,
                                .budget = static_cast<Word>(steps.size())};
  const auto count = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  const bool correct = count == steps.size() && calls == 0 && cpu.reg == expected &&
                       cpu.instructions == steps.size() && cpu.cycles == steps.size();
  if (!correct) {
    std::cerr << "native count=" << count << " size=" << steps.size() << " helpers=" << calls
              << std::hex << " input: A=" << initial.a << " B=" << initial.b << " SR=" << initial.sr
              << " actual: A=" << cpu.reg.a << " B=" << cpu.reg.b << " SR=" << cpu.reg.sr
              << " PC=" << cpu.reg.pc << " expected: A=" << expected.a << " B=" << expected.b
              << " SR=" << expected.sr << " PC=" << expected.pc << std::dec << '\n';
  }
  require(correct,
          "Native ASL emits exact full-state results without instruction or transfer helpers");
}

void shape(const std::span<const Step> steps) {
  Machine machine(steps);
  for (const Word sm : {0U, saturation_model::mode}) {
    for (const auto mode : modes) {
      std::vector<detail::BlockInstruction> operations;
      Word pc = 0;
      for (const auto step : steps) {
        operations.push_back(
            {.instruction = detail::decode(encoding(step)), .pc = pc++, .mode = mode | sm});
      }
      detail::BlockEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}});
      const native::Direct native(emitter.generate(operations));
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const auto raw : values) {
          auto initial = register_model::seed(mode | sm);
          initial.pc = 0;
          initial.sr = mode | sm | flags;
          initial.a = raw;
          initial.b = accumulator_mask ^ raw;
          native_case(native, steps, initial);
          for (Word count = 1; count <= steps.size(); ++count) {
            machine.check(initial, count, false);
            machine.check(initial, count, true);
          }
        }
      }
    }
  }
}

Registers loop_prefix(Registers state, const Word count) {
  for (Word index = 0; index < count; ++index) {
    if (state.pc == 2) {
      state.pc = 0;
    } else {
      shift_model::apply(state, state, {.left = state.pc == 0});
      ++state.pc;
    }
  }
  return state;
}

void closed_loops() {
  std::array program{static_cast<Word>(isa::Opcode::asl_a),
                     static_cast<Word>(isa::Opcode::asr_a),
                     static_cast<Word>(isa::Opcode::jmp_absolute_short)};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  constexpr Word prefixes = 17;
  for (const Word sm : {0U, saturation_model::mode}) {
    for (const auto mode : modes) {
      for (const auto raw : values) {
        auto initial = register_model::seed(mode | sm);
        initial.pc = 0;
        initial.a = raw;
        for (Word count = 0; count <= prefixes; ++count) {
          const auto expected = loop_prefix(initial, count);
          const auto cycles = count + (2 * (count / 3));
          for (const bool deadline : {false, true}) {
            core.set_registers(initial);
            const auto before = core.progress();
            const auto run =
                deadline ? core.run_until(before.estimated_cycles + cycles) : core.run(count);
            require(run.retired == count && run.fault.kind == FaultKind::none &&
                        core.registers() == expected &&
                        run.progress ==
                            Progress{.instructions = before.instructions + count,
                                     .estimated_cycles = before.estimated_cycles + cycles},
                    "Retained ASL/ASR backedges preserve all natural-loop prefixes and deadlines");
          }
        }
      }
    }
  }
}
}
} // namespace dsp56300::test

int main() try {
  using dsp56300::test::Step;
  dsp56300::test::shape(std::array{Step{}});
  dsp56300::test::shape(std::array{Step{.target_b = true}});
  dsp56300::test::shape(std::array{Step{.source = 14, .destination = 4},
                                   Step{.target_b = true, .source = 4, .destination = 14},
                                   Step{},
                                   Step{.target_b = true}});
  dsp56300::test::closed_loops();
  std::cout << dsp56300::test::checks << " independent native ASL checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
