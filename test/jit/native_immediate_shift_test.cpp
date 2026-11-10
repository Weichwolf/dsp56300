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

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <vector>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using Control = shift_model::Control;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 8> values{0,
                                              1,
                                              sign56 - 1,
                                              sign56,
                                              accumulator_mask,
                                              saturation_model::positive + 1,
                                              saturation_model::negative - 1,
                                              0x123456abcdef12};
constexpr std::array selected_flags{0U, flag_c | flag_v | flag_l, status_byte};
constexpr std::array exhaustive_counts{0U, 1U, word_bits, accumulator_bits - 1};
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();

Word encoding(const Control control) {
  const auto base = control.left ? isa::Opcode::asl_immediate : isa::Opcode::asr_immediate;
  constexpr Word source_b = 1U << 7;
  return static_cast<Word>(base) | (control.source_b ? source_b : 0U) | (control.count << 1) |
         (control.destination_b ? 1U : 0U);
}

struct Seed {
  Word status;
  std::uint64_t raw;
};

Registers seed(const Seed input) {
  auto result = register_model::seed(input.status);
  result.pc = 0;
  result.sr = input.status;
  result.a = input.raw;
  result.b = accumulator_mask ^ input.raw;
  return result;
}

Registers model(Registers state, const std::span<const Control> steps) {
  for (auto step : steps) {
    step.saturate = false;
    shift_model::apply(state, state, step);
    ++state.pc;
  }
  return state;
}

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

void native_case(const native::Direct &code,
                 const std::span<const Control> steps,
                 const Registers &initial) {
  const auto expected = model(initial, steps);
  detail::Cpu cpu;
  cpu.reg = initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = cpu.cycles = maximum;
  unsigned helpers = 0;
  bool stopped = false;
  detail::NativeContext context{
      .runtime = {.owner = &helpers, .execute = forbidden, .moves = forbidden, .memory = {}},
      .exit_requested = &stopped,
      .budget = static_cast<Word>(steps.size())};
  const auto retired = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(retired == steps.size() && helpers == 0 && cpu.reg == expected &&
              cpu.instructions == steps.size() - 1 && cpu.cycles == cpu.instructions &&
              context.failure == Fault{},
          "Immediate shifts execute natively with independent raw56 values and complete status");
}

std::vector<Word> program(const std::span<const Control> steps) {
  std::vector<Word> result;
  result.reserve(steps.size() + 1);
  for (const auto step : steps) { result.push_back(encoding(step)); }
  result.push_back(static_cast<Word>(isa::Opcode::stop));
  return result;
}

class Machine {
public:
  explicit Machine(const std::span<const Control> steps, const Word mode)
      : steps_(steps),
        program_(program(steps)),
        core_(Memory::separate({.program = program_, .x = x_, .y = y_})) {
    std::vector<detail::BlockInstruction> operations;
    operations.reserve(steps.size());
    for (Word index = 0; index < steps.size(); ++index) {
      operations.push_back(
          {.instruction = detail::decode(program_[index]), .pc = index, .mode = mode});
    }
    detail::BlockEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}});
    code_ = emitter.generate(operations);
  }

  void check(const Registers &initial, const Word count, const bool deadline) {
    const auto expected = model(initial, steps_.first(count));
    core_.set_registers(initial);
    const auto before = core_.progress();
    const auto result =
        deadline ? core_.run_until(before.estimated_cycles + count) : core_.run(count);
    require(result.retired == count && result.fault.kind == FaultKind::none && !result.yielded &&
                core_.registers() == expected &&
                result.progress == Progress{.instructions = before.instructions + count,
                                            .estimated_cycles = before.estimated_cycles + count},
            "Public immediate shifts preserve complete state and every observable prefix");
    require(program_ == program(steps_) && x_ == std::array<Word, 1>{} && y_ == x_,
            "Accumulator shifts leave every supplied memory word unchanged");
  }

  [[nodiscard]] std::span<const std::uint8_t> code() const { return code_; }

private:
  std::span<const Control> steps_;
  std::vector<Word> program_;
  std::array<Word, 1> x_{}, y_{};
  Core core_;
  std::vector<std::uint8_t> code_;
};

void flags(const Machine &machine,
           const Control step,
           const Word mode,
           const std::span<const Word> selections) {
  const native::Direct native(machine.code());
  for (const auto status : selections) {
    for (const auto raw : values) {
      native_case(native, std::span(&step, 1), seed({.status = mode | status, .raw = raw}));
    }
  }
}

void single(const Control step, const Word mode) {
  Machine machine(std::span(&step, 1), mode);
  flags(machine, step, mode, selected_flags);
  if (std::ranges::find(exhaustive_counts, step.count) != exhaustive_counts.end()) {
    std::array<Word, status_byte + 1> all{};
    for (Word value = 0; value <= status_byte; ++value) { all[value] = value; }
    flags(machine, step, mode, all);
  }
  const auto initial =
      seed({.status = mode | status_byte, .raw = values[step.count % values.size()]});
  machine.check(initial, 1, false);
  machine.check(initial, 1, true);
}

void single_shifts() {
  for (const auto mode : modes) {
    for (const Word sm : {0U, saturation_model::mode}) {
      for (const bool source_b : {false, true}) {
        for (const bool destination_b : {false, true}) {
          for (const bool left : {false, true}) {
            for (unsigned count = 0; count < accumulator_bits; ++count) {
              single({.count = count,
                      .left = left,
                      .source_b = source_b,
                      .destination_b = destination_b,
                      .saturate = false},
                     mode | sm);
            }
          }
        }
      }
    }
  }
}

void chains() {
  constexpr std::array<Control, 6> steps{{{.count = 1, .left = true, .destination_b = true},
                                          {.count = 7, .source_b = true},
                                          {.count = 0, .source_b = true, .destination_b = true},
                                          {.count = word_bits, .left = true},
                                          {.count = accumulator_bits - 1, .destination_b = true},
                                          {.count = 2, .left = true, .source_b = true}}};
  for (const auto mode : modes) {
    for (const Word sm : {0U, saturation_model::mode}) {
      Machine machine(steps, mode | sm);
      const native::Direct native(machine.code());
      for (Word status = 0; status <= status_byte; ++status) {
        for (const auto raw : values) {
          const auto initial = seed({.status = mode | sm | status, .raw = raw});
          native_case(native, steps, initial);
          for (Word count = 0; count <= steps.size(); ++count) {
            machine.check(initial, count, false);
            machine.check(initial, count, true);
          }
        }
      }
    }
  }
}

void invalid_counts() {
  constexpr unsigned count_limit = 64;
  for (unsigned count = accumulator_bits; count < count_limit; ++count) {
    for (const bool left : {false, true}) {
      const Control invalid{.count = count, .left = left, .source_b = true};
      const Control valid{.count = 3, .left = true};
      const std::array steps{valid, invalid};
      auto code = program(steps);
      Core core(Memory::unified(code));
      const auto initial = seed({.status = flag_c | flag_v | flag_l, .raw = values.back()});
      core.set_registers(initial);
      auto expected = model(initial, std::span(&valid, 1));
      // Fetch advances PC for the invalid word, without retiring its effects.
      expected.pc = static_cast<Word>(steps.size());
      const auto result = core.run(2);
      const Fault fault{.kind = FaultKind::invalid_instruction,
                        .space = Space::program,
                        .address = 1,
                        .value = count,
                        .pc = 1};
      require(result.retired == 1 && result.fault == fault && core.registers() == expected &&
                  result.progress == Progress{.instructions = 1, .estimated_cycles = 1},
              "Invalid immediate counts fault after the exact committed native prefix");
    }
  }
}
} // namespace

int main() {
  try {
    single_shifts();
    chains();
    invalid_counts();
    std::cout << checks << " immediate arithmetic shift checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
