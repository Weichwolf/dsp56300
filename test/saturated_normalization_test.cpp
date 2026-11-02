// SPDX-License-Identifier: GPL-3.0-only
// Public-Core normalization boundaries with independent raw-bit expectations.
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "saturated_normalization_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using normalization_model::Input;
using normalization_model::Kind;
constexpr Word port = 0xffff80, datum = 0xfedcba, io_load_b = 0x57d800;
constexpr Word add_a = 0x200040, add_b = 0x200048, round_a = 0x200011;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr Input first{.selector = 3}, replacement{.selector = 5};

struct Machine {
  explicit Machine(const Options options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

Registers seed(const Word mode) {
  auto state = register_model::seed(mode);
  state.sr = mode | flag_s | flag_c | flag_l;
  state.a = saturation_model::positive + 1;
  state.b = saturation_model::negative - 1;
  register_model::write(state, 4, 1);
  register_model::write(state, 5, 1);
  register_model::write(state, 7, word_mask);
  state.r[0] = port;
  return state;
}

void arithmetic(Registers &state, const Word opcode) {
  saturation_model::alu(state, state, opcode);
  ++state.pc;
}

void normalize(Registers &state, const Input input) {
  normalization_model::apply(state, input);
  ++state.pc;
}

void prefixes(const Options options, const Word mode) {
  constexpr Input iterative{.kind = Kind::iteration, .selector = 7, .destination_b = true};
  const std::array program{add_a,
                           normalization_model::opcode(first),
                           round_a,
                           normalization_model::opcode(iterative),
                           add_b};
  for (Word budget = 1; budget <= program.size(); ++budget) {
    Machine machine(options);
    machine.core.load(Space::program, start, program);
    auto expected = seed(mode | saturation_model::mode);
    machine.core.set_registers(expected);
    for (Word i = 0; i < budget; ++i) {
      if (i == 1 || i == 3) {
        normalize(expected, i == 1 ? first : iterative);
      } else {
        arithmetic(expected, program[i]);
      }
    }
    const auto run = machine.core.run(budget);
    require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = budget,
                                         .estimated_cycles = budget + (budget >= 4 ? 4U : 0U)} &&
                machine.core.registers() == expected,
            "Every native/SM normalization prefix publishes exact state, flags and costs");
    if (budget == program.size() && options.backend == Backend::jit) {
      const auto stats = machine.core.jit_statistics();
      const auto native = options.optimize ? 3U : 0U;
      require(stats.native_alu_instructions == native &&
                  stats.helper_instructions == budget - native,
              "Qualified SM normalization remains general between actual native arithmetic");
    }
  }
}

void mode_reuse(const Options options, const Word mode) {
  for (const auto kind : {Kind::iteration, Kind::fast}) {
    Machine machine(options);
    const Input input{.kind = kind, .selector = 2};
    machine.core.write(Space::program, start, normalization_model::opcode(input));
    for (const bool sm : {false, true, false, true}) {
      auto expected = seed(mode | (sm ? saturation_model::mode : 0));
      expected.sr |= flag_e | flag_v;
      register_model::write(expected, 4, 0);
      machine.core.set_registers(expected);
      normalize(expected, input);
      const auto previous = machine.core.progress();
      const auto run = machine.core.run(1);
      require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                  run.progress ==
                      Progress{.instructions = previous.instructions + 1,
                               .estimated_cycles = previous.estimated_cycles +
                                                   normalization_model::cycles(input)} &&
                  machine.core.registers() == expected,
              "Same-PC ordinary/SM normalization cannot reuse stale numerical assumptions");
    }
  }
}

void compatibility_step(const Options options, const Word mode, const Input input) {
  constexpr Word compatibility = Word{1} << 13;

  struct Sample {
    std::uint64_t value;
    Word flags, exponent;
  };

  constexpr std::array samples{
      Sample{.value = saturation_model::positive, .flags = flag_u, .exponent = 0},
      Sample{.value = sign56, .flags = flag_e, .exponent = 0xffff},
      Sample{.value = saturation_model::negative, .flags = flag_u, .exponent = 0xffffff},
      Sample{.value = accumulator_mask, .flags = flag_e, .exponent = 0xabcdef},
      Sample{.value = saturation_model::positive + 1, .flags = 0, .exponent = 0xabcdef},
      Sample{.value = 0, .flags = flag_z | flag_v, .exponent = 0xffffff}};
  Machine machine(options);
  machine.core.write(Space::program, start, normalization_model::opcode(input));
  for (const bool sm : {false, true}) {
    for (const auto sample : samples) {
      auto expected = seed(mode | compatibility | (sm ? saturation_model::mode : 0));
      expected.sr |= sample.flags;
      (input.destination_b ? expected.b : expected.a) = sample.value;
      expected.r[input.selector] = sample.exponent;
      machine.core.set_registers(expected);
      normalize(expected, input);
      const auto before = machine.core.progress();
      const auto run = machine.core.run(1);
      require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                  run.progress == Progress{.instructions = before.instructions + 1,
                                           .estimated_cycles = before.estimated_cycles + 5} &&
                  machine.core.registers() == expected,
              "Established SC NORM retains exact SM/bypass flags and 16-bit shifted exponent");
    }
  }
}

void compatibility_steps(const Options options, const Word mode) {
  for (unsigned selector = 0; selector < agu_registers; ++selector) {
    for (const bool destination_b : {false, true}) {
      compatibility_step(
          options,
          mode,
          {.kind = Kind::iteration, .selector = selector, .destination_b = destination_b});
    }
  }
}

void bfu_forms(const Options options, const Word mode) {
  struct Form {
    Word opcode;
    bool fast = false, saturate = false;
  };

  constexpr std::array forms{Form{.opcode = 0x200032, .saturate = true},
                             Form{.opcode = 0x0c1d02},
                             Form{.opcode = 0x0c1e48},
                             Form{.opcode = 0x0c1e2c, .fast = true}};
  constexpr std::array<std::uint64_t, 2> values{0x00400000000000, saturation_model::negative - 1};
  for (const auto form : forms) {
    Machine machine(options);
    machine.core.write(Space::program, start, form.opcode);
    for (const auto value : values) {
      for (const Word flags : {0U, status_byte}) {
        auto expected = seed(mode | saturation_model::mode);
        expected.sr = mode | saturation_model::mode | flags;
        expected.a = value;
        register_model::write(expected, 5, word_mask);
        machine.core.set_registers(expected);
        if (form.fast) {
          normalize(expected, first);
        } else {
          shift_model::apply(expected, expected, {.left = true, .saturate = form.saturate});
          ++expected.pc;
        }
        const auto before = machine.core.progress();
        const auto run = machine.core.run(1);
        require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                    run.progress == Progress{.instructions = before.instructions + 1,
                                             .estimated_cycles = before.estimated_cycles + 1} &&
                    machine.core.registers() == expected,
                "Equal one-bit shifts distinguish saturating ALU from bypassing BFU forms");
      }
    }
  }
}

void invalid_inputs(const Options options) {
  constexpr std::array counts{-8388608, -56, 57, 8388607};
  for (unsigned selector = 0; selector < normalization_model::sources.size(); ++selector) {
    for (const bool destination_b : {false, true}) {
      for (const int count : counts) {
        const Input input{.selector = selector, .destination_b = destination_b};
        Machine machine(options);
        const auto opcode = normalization_model::opcode(input);
        machine.core.write(Space::program, start, opcode);
        auto expected = seed(saturation_model::mode);
        register_model::write(
            expected, normalization_model::sources[selector], static_cast<Word>(count) & word_mask);
        machine.core.set_registers(expected);
        ++expected.pc;
        const auto run = machine.core.run(1);
        require(run.dispatches == 0 && run.progress == Progress{} &&
                    run.fault == Fault{.kind = FaultKind::unsupported_mode,
                                       .space = Space::program,
                                       .address = start,
                                       .value = opcode,
                                       .pc = start} &&
                    machine.core.registers() == expected,
                "Out-of-domain SM counts diagnose before shifting or retirement");
      }
    }
  }
}

void unqualified_modes(const Options options) {
  constexpr Word divide = 0x018040;
  constexpr std::array unsupported{0x2000U, 0x4000U, 0x20000U, mode_up | mode_down};
  for (const auto kind : {Kind::iteration, Kind::fast}) {
    for (const auto mode : unsupported) {
      Machine machine(options);
      const Input input{.kind = kind, .selector = 2};
      machine.core.write(Space::program, start, normalization_model::opcode(input));
      auto expected = seed(mode | saturation_model::mode);
      machine.core.set_registers(expected);
      if (kind == Kind::iteration && mode == 0x2000U) {
        normalization_model::apply(expected, input);
        ++expected.pc;
        const auto run = machine.core.run(1);
        require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                    run.progress == Progress{.instructions = 1, .estimated_cycles = 5} &&
                    machine.core.registers() == expected,
                "Non-shift NORM qualifies SM/SC without changing data or the exponent");
        continue;
      }
      ++expected.pc;
      const auto run = machine.core.run(1);
      require(run.dispatches == 0 && run.progress == Progress{} &&
                  run.fault == Fault{.kind = FaultKind::unsupported_mode,
                                     .space = Space::program,
                                     .address = start,
                                     .value = expected.sr,
                                     .pc = start} &&
                  machine.core.registers() == expected,
              "Other numerical domains remain explicit diagnostics before normalization");
    }
  }
  Machine machine(options);
  machine.core.write(Space::program, start, divide);
  auto expected = seed(saturation_model::mode);
  machine.core.set_registers(expected);
  ++expected.pc;
  const auto run = machine.core.run(1);
  require(run.dispatches == 0 && run.progress == Progress{} &&
              run.fault == Fault{.kind = FaultKind::unsupported_mode,
                                 .space = Space::program,
                                 .address = start,
                                 .value = expected.sr,
                                 .pc = start} &&
              machine.core.registers() == expected,
          "Normalization admission does not accidentally admit unqualified DIV with SM");
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
    if (observed && accept && patch) {
      core->write(Space::program, start + 2, normalization_model::opcode(replacement));
    }
    value = datum;
    return observed && accept;
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
  const std::array program{add_a, io_load_b, normalization_model::opcode(first)};
  machine.core.load(Space::program, start, program);
  auto expected = seed(input.mode | saturation_model::mode);
  machine.core.set_registers(expected);
  arithmetic(expected, add_a);
  ++expected.r[0];
  ++expected.pc;
  observer.expected = expected;
  const auto run = machine.core.run(3);
  require(observer.observed && observer.calls == 1,
          "I/O observes the canonical native prefix exactly once before SM normalization");
  if (!input.accept) {
    const auto actual = machine.core.registers();
    if (actual != expected || run.fault.kind != FaultKind::io_read || run.fault.pc != start + 1 ||
        run.fault.address != port || run.fault.value != 0 || run.dispatches != 1) {
      std::cerr << "CALLBACK backend=" << static_cast<unsigned>(options.backend)
                << " optimize=" << options.optimize << " mode=" << input.mode << std::hex
                << " actual-pc=" << actual.pc << " expected-pc=" << expected.pc
                << " actual-sr=" << actual.sr << " expected-sr=" << expected.sr
                << " actual-a=" << actual.a << " expected-a=" << expected.a
                << " actual-b=" << actual.b << " expected-b=" << expected.b
                << " actual-r0=" << actual.r[0] << " expected-r0=" << expected.r[0]
                << " fault-kind=" << static_cast<unsigned>(run.fault.kind)
                << " fault-address=" << run.fault.address << " fault-pc=" << run.fault.pc
                << " fault-value=" << run.fault.value << std::dec << '\n';
    }
    require(run.dispatches == 1 &&
                run.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = port,
                                   .pc = start + 1} &&
                machine.core.registers() == expected,
            "Rejected load preserves partial transfer effects without executing normalization");
    require(machine.core.run(3).dispatches == 0 && machine.core.registers() == expected &&
                observer.calls == 1,
            "Fault retry never replays the earlier saturated prefix or I/O");
    return;
  }
  // A declined read publishes no destination data; only an accepted load owns datum.
  register_model::write(expected, 15, datum);
  normalize(expected, input.patch ? replacement : first);
  require(run.dispatches == 3 && run.fault.kind == FaultKind::none &&
              run.progress == Progress{.instructions = 3, .estimated_cycles = 3} &&
              machine.core.registers() == expected,
          "Accepted I/O and P edits select the new normalization with exact earlier state");
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
          dsp56300::test::invalid_inputs(options);
          dsp56300::test::unqualified_modes(options);
          for (const auto mode : dsp56300::test::modes) {
            dsp56300::test::prefixes(options, mode);
            dsp56300::test::mode_reuse(options, mode);
            dsp56300::test::compatibility_steps(options, mode);
            dsp56300::test::bfu_forms(options, mode);
            for (const bool accept : {false, true}) {
              for (const bool patch : {false, true}) {
                dsp56300::test::callback(options, {.mode = mode, .accept = accept, .patch = patch});
              }
            }
          }
        }
      }
    }
    std::cout << dsp56300::test::checks << " independent SM normalization boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
