// SPDX-License-Identifier: GPL-3.0-only
// Prepared normalization boundaries with independent raw-bit expectations.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "saturated_normalization_model.hpp"
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
using normalization_model::Input;
using normalization_model::Kind;
constexpr Word port = 0xffff80, datum = 0xfedcba;
constexpr unsigned b_register = 15;
constexpr Word io_load_b = static_cast<Word>(isa::Opcode::move_long) | ((b_register & 7U) << 16) |
                           ((b_register & 0x18U) << 17) | (1U << 14) | (1U << 15) | (3U << 11);
constexpr Word add_a = static_cast<Word>(isa::Opcode::add_x0_a);
constexpr Word add_b = static_cast<Word>(isa::Opcode::add_x0_b);
constexpr Word round_a = static_cast<Word>(isa::Opcode::rnd_a);
constexpr Word divide = static_cast<Word>(isa::Opcode::div_register);
constexpr std::size_t compatibility_first = 3, bfu_first = 19, fast_first = 23;
constexpr std::size_t divide_variant = 35, callback_first = 36;
std::size_t cases = 0;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr Input first{.selector = 3}, replacement{.selector = 5};

struct Machine {
  explicit Machine(Io *io = nullptr) : core(Memory::separate({.program = p, .x = x, .y = y}), io) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

struct Bfu {
  Word opcode;
  bool fast = false, saturate = false;
};

constexpr std::array bfu_forms{
    Bfu{.opcode = static_cast<Word>(isa::Opcode::asl_a), .saturate = true},
    Bfu{.opcode = static_cast<Word>(isa::Opcode::asl_immediate) | 2},
    Bfu{.opcode = static_cast<Word>(isa::Opcode::asl_register) | 8},
    Bfu{.opcode = static_cast<Word>(isa::Opcode::normf_register) | 12, .fast = true}};

std::array<Word, 5> prefix_program() {
  constexpr Input iterative{.kind = Kind::iteration, .selector = 7, .destination_b = true};
  return {add_a,
          normalization_model::opcode(first),
          round_a,
          normalization_model::opcode(iterative),
          add_b};
}

NativeImage prepare() {
  const auto prefix = prefix_program();
  std::vector<std::vector<Word>> programs{{prefix.begin(), prefix.end()}};
  programs.reserve(callback_first + 3);
  for (const auto kind : {Kind::iteration, Kind::fast}) {
    programs.push_back({normalization_model::opcode({.kind = kind, .selector = 2})});
  }
  for (unsigned selector = 0; selector < agu_registers; ++selector) {
    for (const bool destination_b : {false, true}) {
      programs.push_back({normalization_model::opcode(
          {.kind = Kind::iteration, .selector = selector, .destination_b = destination_b})});
    }
  }
  for (const auto form : bfu_forms) { programs.push_back({form.opcode}); }
  for (unsigned selector = 0; selector < normalization_model::sources.size(); ++selector) {
    for (const bool destination_b : {false, true}) {
      programs.push_back(
          {normalization_model::opcode({.selector = selector, .destination_b = destination_b})});
    }
  }
  programs.push_back({divide});
  programs.push_back({add_a, io_load_b, normalization_model::opcode(first)});
  programs.push_back({add_a, io_load_b});
  programs.push_back({add_a, io_load_b, normalization_model::opcode(replacement)});
  std::vector<CodeSpan> spans;
  spans.reserve(programs.size());
  for (const auto &program : programs) { spans.push_back({.address = start, .words = program}); }
  std::vector<Word> entries(programs.size(), start);
  entries.back() = start + 2;
  constexpr std::array transitions{
      CodeTransition{.from = callback_first + 1, .to = callback_first + 2, .entry = start + 2}};
  return compile_programs(spans, entries, transitions);
}

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

void prefixes(const NativeImage &image, const Word mode) {
  const auto program = prefix_program();
  constexpr Input iterative{.kind = Kind::iteration, .selector = 7, .destination_b = true};
  for (Word budget = 1; budget <= program.size(); ++budget) {
    Machine machine;
    machine.core.load(Space::program, start, program);
    machine.core.install(image, 0);
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
    ++cases;
    require(run.retired == budget && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = budget,
                                         .estimated_cycles = budget + (budget >= 4 ? 4U : 0U)} &&
                machine.core.registers() == expected,
            "Every native/SM normalization prefix publishes exact state, flags and costs");
  }
}

void mode_reuse(const NativeImage &image, const Word mode) {
  for (const auto kind : {Kind::iteration, Kind::fast}) {
    Machine machine;
    const Input input{.kind = kind, .selector = 2};
    machine.core.load(Space::program, start, std::array{normalization_model::opcode(input)});
    machine.core.install(image, kind == Kind::iteration ? 1 : 2);
    for (const bool sm : {false, true, false, true}) {
      auto expected = seed(mode | (sm ? saturation_model::mode : 0));
      expected.sr |= flag_e | flag_v;
      register_model::write(expected, 4, 0);
      machine.core.set_registers(expected);
      normalize(expected, input);
      const auto previous = machine.core.progress();
      const auto run = machine.core.run(1);
      ++cases;
      require(run.retired == 1 && run.fault.kind == FaultKind::none &&
                  run.progress ==
                      Progress{.instructions = previous.instructions + 1,
                               .estimated_cycles = previous.estimated_cycles +
                                                   normalization_model::cycles(input)} &&
                  machine.core.registers() == expected,
              "Same-PC ordinary/SM normalization cannot reuse stale numerical assumptions");
    }
  }
}

void compatibility_step(const NativeImage &image, const Word mode, const Input input) {
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
  Machine machine;
  machine.core.load(Space::program, start, std::array{normalization_model::opcode(input)});
  machine.core.install(image,
                       compatibility_first + (static_cast<std::size_t>(input.selector) * 2) +
                           (input.destination_b ? 1 : 0));
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
      ++cases;
      require(run.retired == 1 && run.fault.kind == FaultKind::none &&
                  run.progress == Progress{.instructions = before.instructions + 1,
                                           .estimated_cycles = before.estimated_cycles + 5} &&
                  machine.core.registers() == expected,
              "Established SC NORM retains exact SM/bypass flags and 16-bit shifted exponent");
    }
  }
}

void compatibility_steps(const NativeImage &image, const Word mode) {
  for (unsigned selector = 0; selector < agu_registers; ++selector) {
    for (const bool destination_b : {false, true}) {
      compatibility_step(
          image,
          mode,
          {.kind = Kind::iteration, .selector = selector, .destination_b = destination_b});
    }
  }
}

void bfu_cases(const NativeImage &image, const Word mode) {
  constexpr std::array<std::uint64_t, 2> values{0x00400000000000, saturation_model::negative - 1};
  for (std::size_t index = 0; index < bfu_forms.size(); ++index) {
    const auto form = bfu_forms[index];
    Machine machine;
    machine.core.load(Space::program, start, std::array{form.opcode});
    machine.core.install(image, bfu_first + index);
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
        ++cases;
        require(run.retired == 1 && run.fault.kind == FaultKind::none &&
                    run.progress == Progress{.instructions = before.instructions + 1,
                                             .estimated_cycles = before.estimated_cycles + 1} &&
                    machine.core.registers() == expected,
                "Equal one-bit shifts distinguish saturating ALU from bypassing BFU forms");
      }
    }
  }
}

void invalid_inputs(const NativeImage &image) {
  constexpr std::array counts{-8388608, -56, 57, 8388607};
  for (unsigned selector = 0; selector < normalization_model::sources.size(); ++selector) {
    for (const bool destination_b : {false, true}) {
      for (const int count : counts) {
        const Input input{.selector = selector, .destination_b = destination_b};
        Machine machine;
        const auto opcode = normalization_model::opcode(input);
        machine.core.load(Space::program, start, std::array{opcode});
        machine.core.install(
            image, fast_first + (static_cast<std::size_t>(selector) * 2) + (destination_b ? 1 : 0));
        auto expected = seed(saturation_model::mode);
        register_model::write(
            expected, normalization_model::sources[selector], static_cast<Word>(count) & word_mask);
        machine.core.set_registers(expected);
        ++expected.pc;
        const auto run = machine.core.run(1);
        ++cases;
        require(run.retired == 0 && run.progress == Progress{} &&
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

void unqualified_modes(const NativeImage &image) {
  constexpr std::array unsupported{0x2000U, 0x4000U, 0x20000U, mode_up | mode_down};
  for (const auto kind : {Kind::iteration, Kind::fast}) {
    for (const auto mode : unsupported) {
      Machine machine;
      const Input input{.kind = kind, .selector = 2};
      machine.core.load(Space::program, start, std::array{normalization_model::opcode(input)});
      machine.core.install(image, kind == Kind::iteration ? 1 : 2);
      auto expected = seed(mode | saturation_model::mode);
      machine.core.set_registers(expected);
      if (kind == Kind::iteration && mode == 0x2000U) {
        normalization_model::apply(expected, input);
        ++expected.pc;
        const auto run = machine.core.run(1);
        ++cases;
        require(run.retired == 1 && run.fault.kind == FaultKind::none &&
                    run.progress == Progress{.instructions = 1, .estimated_cycles = 5} &&
                    machine.core.registers() == expected,
                "Non-shift NORM qualifies SM/SC with limiting and an unchanged exponent");
        continue;
      }
      ++expected.pc;
      const auto run = machine.core.run(1);
      ++cases;
      require(run.retired == 0 && run.progress == Progress{} &&
                  run.fault == Fault{.kind = FaultKind::unsupported_mode,
                                     .space = Space::program,
                                     .address = start,
                                     .value = expected.sr,
                                     .pc = start} &&
                  machine.core.registers() == expected,
              "Other numerical domains remain explicit diagnostics before normalization");
    }
  }
  Machine machine;
  machine.core.load(Space::program, start, std::array{divide});
  machine.core.install(image, divide_variant);
  auto expected = seed(saturation_model::mode);
  machine.core.set_registers(expected);
  ++expected.pc;
  const auto run = machine.core.run(1);
  ++cases;
  // Raw DIV: 2*(+2^47)+1-(1*2^24) exceeds the positive SM rail.
  expected.a = saturation_model::positive;
  expected.sr |= flag_v;
  require(run.retired == 1 && run.fault.kind == FaultKind::none &&
              machine.core.registers() == expected,
          "Original rejected DIV now checks SM data limiting, raw quotient Carry and V/L");
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
      try {
        core->load(Space::program, start + 2, std::array{normalization_model::opcode(replacement)});
      } catch (...) { observed = false; }
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

void callback(const NativeImage &image, const Request input) {
  Observer observer;
  observer.accept = input.accept;
  observer.patch = input.patch;
  Machine machine(&observer);
  observer.core = &machine.core;
  const std::array program{add_a, io_load_b, normalization_model::opcode(first)};
  machine.core.load(Space::program, start, program);
  machine.core.install(image, callback_first + (input.patch ? 1 : 0));
  auto expected = seed(input.mode | saturation_model::mode);
  machine.core.set_registers(expected);
  arithmetic(expected, add_a);
  ++expected.r[0];
  ++expected.pc;
  observer.expected = expected;
  const auto run = machine.core.run(3);
  ++cases;
  require(observer.observed && observer.calls == 1,
          "I/O observes the canonical native prefix exactly once before SM normalization");
  if (!input.accept) {
    require(run.retired == 1 &&
                run.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = port,
                                   .pc = start + 1} &&
                machine.core.registers() == expected,
            "Rejected load preserves partial transfer effects without executing normalization");
    require(machine.core.run(3).retired == 0 && machine.core.registers() == expected &&
                observer.calls == 1,
            "Fault retry never replays the earlier saturated prefix or I/O");
    return;
  }
  std::uint32_t suffix_retired = 0;
  if (input.patch) {
    require(run.retired == 2 && run.yielded && run.fault == Fault{} &&
                run.progress == Progress{.instructions = 2, .estimated_cycles = 2},
            "The real P write hands off after the original load at its declared transition");
    machine.core.install(image, callback_first + 2);
    const auto suffix = machine.core.run(1);
    suffix_retired = suffix.retired;
    require(suffix.fault == Fault{}, "The new prepared normalization executes without a fault");
  }
  // A declined read publishes no destination data; only an accepted load owns datum.
  register_model::write(expected, 15, datum);
  normalize(expected, input.patch ? replacement : first);
  require(run.retired + suffix_retired == 3 && run.fault.kind == FaultKind::none &&
              machine.core.progress() == Progress{.instructions = 3, .estimated_cycles = 3} &&
              machine.core.registers() == expected,
          "Accepted I/O and P edits select the new normalization with exact earlier state");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    const auto image = dsp56300::test::prepare();
    dsp56300::test::invalid_inputs(image);
    dsp56300::test::unqualified_modes(image);
    for (const auto mode : dsp56300::test::modes) {
      dsp56300::test::prefixes(image, mode);
      dsp56300::test::mode_reuse(image, mode);
      dsp56300::test::compatibility_steps(image, mode);
      dsp56300::test::bfu_cases(image, mode);
      for (const bool accept : {false, true}) {
        for (const bool patch : {false, true}) {
          dsp56300::test::callback(image, {.mode = mode, .accept = accept, .patch = patch});
        }
      }
    }
    std::cout << dsp56300::test::cases << " normalization cases and " << dsp56300::test::checks
              << " independent SM boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
