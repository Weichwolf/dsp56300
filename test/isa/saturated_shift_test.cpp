// SPDX-License-Identifier: GPL-3.0-only
// Mode reuse, partial budgets and observable parallel-shift transfer boundaries.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word port = 0xffff80, datum = 0xfedcba, io_asr_b = 0x57d82a;
constexpr Word bit_sm = 20, register_bit = 0x0ac040;
constexpr Word clear_sm = register_bit | (register_model::reg_sr << 8) | bit_sm;
constexpr Word set_sm = clear_sm | 0x20U;
constexpr Word asl_a = 0x200032, asr_a = 0x200022;
constexpr Word multi_right = 0x0c1ca2, multi_left = 0x0c1da2;
constexpr unsigned multi_count = 17;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Machine {
  explicit Machine(const Options options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

Registers seed(const Word mode) {
  auto state = register_model::seed(mode);
  state.sr = mode | flag_s | flag_c | flag_v;
  state.a = saturation_model::positive + 1;
  state.b = saturation_model::negative - 1;
  state.r[0] = port;
  return state;
}

void step(Registers &state, const Word opcode) {
  if (opcode == set_sm) {
    state.sr |= saturation_model::mode;
  } else if (opcode == clear_sm) {
    state.sr &= ~saturation_model::mode;
  } else {
    const bool multi = opcode == multi_right || opcode == multi_left;
    shift_model::apply(state,
                       state,
                       {.count = multi ? multi_count : 1U,
                        .left = opcode == asl_a || opcode == multi_left,
                        .source_b = multi,
                        .saturate = !multi});
  }
  ++state.pc;
}

void prefixes(const Options options, const Word mode) {
  constexpr std::array program{asr_a, set_sm, asl_a, multi_right, multi_left, clear_sm, asr_a};
  for (Word budget = 1; budget <= program.size(); ++budget) {
    Machine machine(options);
    machine.core.load(Space::program, start, program);
    auto expected = seed(mode);
    machine.core.set_registers(expected);
    for (Word i = 0; i < budget; ++i) { step(expected, program[i]); }
    const auto run = machine.core.run(budget);
    const auto costs = budget + (budget >= 2 ? 1U : 0U) + (budget >= 6 ? 1U : 0U);
    require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = budget, .estimated_cycles = costs} &&
                machine.core.registers() == expected,
            "Every ordinary/SM shift prefix publishes exact state and base costs");
    if (budget == program.size() && options.backend == Backend::jit) {
      const auto stats = machine.core.jit_statistics();
      const auto native = options.optimize ? 2U : 0U;
      require(stats.native_alu_instructions == native &&
                  stats.helper_instructions == budget - native,
              "Ordinary ASR stays native; qualified SM shifts use the exact general fallback");
    }
  }
}

void mode_reuse(const Options options, const Word mode) {
  Machine machine(options);
  machine.core.write(Space::program, start, asr_a);
  for (const bool sm : {false, true, false, true}) {
    auto expected = seed(mode | (sm ? saturation_model::mode : 0));
    machine.core.set_registers(expected);
    shift_model::apply(expected, expected, {});
    ++expected.pc;
    const auto before = machine.core.jit_statistics();
    const auto progress = machine.core.progress();
    const auto run = machine.core.run(1);
    require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = progress.instructions + 1,
                                         .estimated_cycles = progress.estimated_cycles + 1} &&
                machine.core.registers() == expected,
            "Same-PC reuse cannot carry ordinary assumptions into SM or vice versa");
    if (options.backend == Backend::jit) {
      const auto after = machine.core.jit_statistics();
      const auto native = options.optimize && !sm ? 1U : 0U;
      require(after.native_alu_instructions == before.native_alu_instructions + native &&
                  after.helper_instructions == before.helper_instructions + 1 - native,
              "Same-PC mode reuse exercises the actual native/general split");
    }
  }
}

void undefined_counts(const Options options) {
  constexpr unsigned maximum_count = 63;
  constexpr Word left_x0 = 0x0c1e48;
  constexpr Word right_x0 = 0x0c1e68;
  for (unsigned count = accumulator_bits; count <= maximum_count; ++count) {
    for (const bool left : {false, true}) {
      const shift_model::Control input{.count = count, .left = left};
      for (const auto opcode : {shift_model::immediate(input), left ? left_x0 : right_x0}) {
        Machine machine(options);
        machine.core.write(Space::program, start, opcode);
        auto expected = seed(saturation_model::mode);
        register_model::write(expected, 4, count);
        machine.core.set_registers(expected);
        ++expected.pc;
        const auto run = machine.core.run(1);
        require(run.dispatches == 0 && run.progress == Progress{} &&
                    run.fault == Fault{.kind = FaultKind::invalid_instruction,
                                       .space = Space::program,
                                       .address = start,
                                       .value = count,
                                       .pc = start} &&
                    machine.core.registers() == expected,
                "Undefined SM shift counts retain the explicit diagnostic before ALU effects");
      }
    }
  }
}

class Observer final : public Io {
public:
  Core *core = nullptr;
  Registers expected;
  bool accept = true, patch = false, observed = false;
  unsigned calls = 0;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++calls;
    observed = core != nullptr && space == Space::x && address == port && pc == start + 1 &&
               core->registers() == expected &&
               core->progress() == Progress{.instructions = 1, .estimated_cycles = 1};
    if (observed && accept && patch) { core->write(Space::program, start + 2, multi_left); }
    value = datum;
    return accept && observed;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

struct Request {
  Word mode = 0;
  bool accept = false, patch = false;
};

void callback(const Options options, const Request input) {
  Observer observer;
  observer.accept = input.accept;
  observer.patch = input.patch;
  Machine machine(options, &observer);
  observer.core = &machine.core;
  constexpr std::array program{asl_a, io_asr_b, multi_right};
  machine.core.load(Space::program, start, program);
  auto expected = seed(input.mode | saturation_model::mode);
  machine.core.set_registers(expected);
  step(expected, asl_a);
  ++expected.r[0];
  ++expected.pc;
  observer.expected = expected;
  const auto old = expected;
  register_model::write(expected, 15, datum);
  const auto run = machine.core.run(3);
  require(observer.observed && observer.calls == 1,
          "Synchronous I/O sees the exact earlier saturated result/flags and only one request");
  if (!input.accept) {
    require(run.dispatches == 1 &&
                run.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = port,
                                   .pc = start + 1} &&
                machine.core.registers() == expected,
            "Rejected parallel load retains its destination but performs no shift or retirement");
    require(machine.core.run(3).dispatches == 0 && machine.core.registers() == expected &&
                observer.calls == 1,
            "Failure remains fail-stop without replaying a saturated prefix or I/O");
    return;
  }
  shift_model::apply(expected, old, {.source_b = true, .destination_b = true});
  step(expected, input.patch ? multi_left : multi_right);
  require(run.dispatches == 3 && run.fault.kind == FaultKind::none &&
              run.progress == Progress{.instructions = 3, .estimated_cycles = 3} &&
              machine.core.registers() == expected,
          "Parallel shifts use pre-load operands and P edits replace the following shift exactly");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      for (const bool optimize : {false, true}) {
        for (const auto limit : {1U, 64U}) {
          const dsp56300::Options options{
              .backend = backend, .instructions_per_block = limit, .optimize = optimize};
          dsp56300::test::undefined_counts(options);
          for (const auto mode : dsp56300::test::modes) {
            dsp56300::test::prefixes(options, mode);
            dsp56300::test::mode_reuse(options, mode);
            for (const bool accept : {false, true}) {
              for (const bool patch : {false, true}) {
                dsp56300::test::callback(options, {.mode = mode, .accept = accept, .patch = patch});
              }
            }
          }
        }
      }
    }
    std::cout << dsp56300::test::checks << " independent saturated-shift boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
