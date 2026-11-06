// SPDX-License-Identifier: GPL-3.0-only
// Public-API witnesses for pre-transfer widths, parallel writes and scaling.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
struct Step {
  isa::Opcode alu = isa::Opcode::move_none;
  unsigned source = 0, destination = 0;
};

Word encoding(const Step step) {
  return static_cast<Word>(step.alu) | (step.source << 13) | (step.destination << 8);
}

void model(Registers &state, const Step step) {
  const auto old = state;
  if (step.source != 0) {
    const auto value = register_model::read(state, step.source);
    register_model::write(state, step.destination, value);
  }
  const auto alu = static_cast<Word>(step.alu) & status_byte;
  if (alu != 0) {
    if (step.alu == isa::Opcode::asr_a) {
      shift_model::apply(state, old, {});
    } else if ((state.sr & saturation_model::mode) != 0) {
      saturation_model::alu(state, old, alu);
    } else {
      sequence_model::alu(state, old, alu);
    }
  }
  ++state.pc;
}

class Machine {
public:
  explicit Machine(const std::span<const Step> steps)
      : program_(program(steps)),
        core_(Memory::separate({.program = program_, .x = x_, .y = y_})) {}

  void check(const std::span<const Step> steps, const Registers &initial, const bool partition) {
    core_.set_registers(initial);
    const auto before = core_.state().progress;
    auto expected = initial;
    const auto count = static_cast<Word>(steps.size());
    Word retired = 0;
    do {
      const auto budget = partition ? 1U : count;
      for (const auto step : steps.subspan(retired, budget)) { model(expected, step); }
      retired += budget;
      const auto result = core_.run(budget);
      require(result.retired == budget && result.fault.kind == FaultKind::none && !result.yielded &&
                  result.state == ExecutionState::running && core_.registers() == expected &&
                  result.progress ==
                      Progress{.instructions = before.instructions + retired,
                               .estimated_cycles = before.estimated_cycles + retired},
              "Whole and partitioned public runs retain old transfer values, scaling, flags and "
              "all registers");
    } while (retired < count);
    for (std::size_t index = 0; index < steps.size(); ++index) {
      require(program_[index] == encoding(steps[index]), "Transfers never modify their program");
    }
    require(x_ == std::array<Word, memory_words>{} && y_ == x_,
            "Register transfers leave all X/Y memory intact");
  }

private:
  static std::vector<Word> program(const std::span<const Step> steps) {
    std::vector<Word> result;
    for (const auto step : steps) { result.push_back(encoding(step)); }
    return result;
  }

  std::vector<Word> program_;
  std::array<Word, memory_words> x_{}, y_{};
  Core core_;
};

void shape(const std::span<const Step> steps) {
  Machine machine(steps);
  constexpr std::array<std::uint64_t, 8> raw{0,
                                             1,
                                             sign56 - 1,
                                             sign56,
                                             accumulator_mask,
                                             saturation_model::positive,
                                             saturation_model::negative,
                                             0x123456789abc};
  constexpr std::array<Word, 5> words{0, 1, sign24 - 1, sign24, word_mask};
  for (const Word mode :
       {0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up}) {
    for (const Word sm : {0U, saturation_model::mode}) {
      for (const Word flags : {0U, flag_l, status_byte}) {
        for (const auto value : raw) {
          for (const auto word : words) {
            auto initial = register_model::seed(mode | sm | flags);
            initial.pc = 0;
            initial.a = value;
            initial.b = value ^ accumulator_mask;
            initial.x = (std::uint64_t{word ^ word_mask} << word_bits) | word;
            initial.y = (std::uint64_t{word} << word_bits) | (word ^ word_mask);
            machine.check(steps, initial, false);
            machine.check(steps, initial, true);
          }
        }
      }
    }
  }
}
}
} // namespace dsp56300::test

int main() {
  try {
    using dsp56300::isa::Opcode;
    using dsp56300::test::Step;
    dsp56300::test::shape(std::array{Step{.alu = Opcode::clr_a, .source = 14, .destination = 4}});
    dsp56300::test::shape(std::array{Step{.alu = Opcode::clr_b, .source = 15, .destination = 6}});
    dsp56300::test::shape(
        std::array{Step{.alu = Opcode::clr_a}, Step{.source = 14, .destination = 4}});
    dsp56300::test::shape(
        std::array{Step{.alu = Opcode::tfr_x0_a}, Step{.source = 14, .destination = 6}});
    dsp56300::test::shape(
        std::array{Step{.alu = Opcode::mpyr_register_a}, Step{.source = 14, .destination = 4}});
    dsp56300::test::shape(std::array{Step{.alu = Opcode::tfr_x0_a},
                                     Step{.alu = Opcode::asr_a},
                                     Step{.alu = Opcode::asr_a},
                                     Step{.source = 14, .destination = 6}});
    std::cout << dsp56300::test::checks << " independent public transfer-range checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
