// SPDX-License-Identifier: GPL-3.0-only
// Mode reuse, partial budgets and observable parallel-shift transfer boundaries.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
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
constexpr Word port = 0xffff80, datum = 0xfedcba;
constexpr unsigned b_register = 15;
constexpr Word io_asr_b = static_cast<Word>(isa::Opcode::move_long) | ((b_register & 7U) << 16) |
                          ((b_register & 0x18U) << 17) | (1U << 14) | (1U << 15) | (3U << 11) |
                          (static_cast<Word>(isa::Opcode::asr_b) & status_byte);
constexpr Word bit_sm = 20, register_bit = static_cast<Word>(isa::Opcode::bclr_register);
constexpr Word clear_sm = register_bit | (register_model::reg_sr << 8) | bit_sm;
constexpr Word set_sm = clear_sm | 0x20U;
constexpr Word asl_a = static_cast<Word>(isa::Opcode::asl_a);
constexpr Word asr_a = static_cast<Word>(isa::Opcode::asr_a);
constexpr Word multi_right = static_cast<Word>(isa::Opcode::asr_immediate) | 0xa2;
constexpr Word multi_left = static_cast<Word>(isa::Opcode::asl_immediate) | 0xa2;
constexpr Word left_x0 = static_cast<Word>(isa::Opcode::asl_register) | 8;
constexpr Word right_x0 = static_cast<Word>(isa::Opcode::asr_register) | 8;
constexpr unsigned multi_count = 17, maximum_count = 63;
constexpr std::size_t immediate_first = 4, callback_first = 20;
constexpr std::array prefix{asr_a, set_sm, asl_a, multi_right, multi_left, clear_sm, asr_a};
std::size_t cases = 0;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Machine {
  explicit Machine(Io *io = nullptr) : core(Memory::separate({.program = p, .x = x, .y = y}), io) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

NativeImage prepare() {
  std::vector<std::vector<Word>> programs{
      {prefix.begin(), prefix.end()}, {asr_a}, {right_x0}, {left_x0}};
  for (unsigned count = accumulator_bits; count <= maximum_count; ++count) {
    for (const bool left : {false, true}) {
      programs.push_back({shift_model::immediate({.count = count, .left = left})});
    }
  }
  programs.push_back({asl_a, io_asr_b, multi_right});
  programs.push_back({asl_a, io_asr_b});
  programs.push_back({asl_a, io_asr_b, multi_left});
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

void prefixes(const NativeImage &image, const Word mode) {
  constexpr auto program = prefix;
  for (Word budget = 1; budget <= program.size(); ++budget) {
    Machine machine;
    machine.core.load(Space::program, start, program);
    machine.core.install(image, 0);
    auto expected = seed(mode);
    machine.core.set_registers(expected);
    for (Word i = 0; i < budget; ++i) { step(expected, program[i]); }
    const auto run = machine.core.run(budget);
    ++cases;
    const auto costs = budget + (budget >= 2 ? 1U : 0U) + (budget >= 6 ? 1U : 0U);
    require(run.retired == budget && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = budget, .estimated_cycles = costs} &&
                machine.core.registers() == expected,
            "Every ordinary/SM shift prefix publishes exact state and base costs");
  }
}

void mode_reuse(const NativeImage &image, const Word mode) {
  Machine machine;
  machine.core.load(Space::program, start, std::array{asr_a});
  machine.core.install(image, 1);
  for (const bool sm : {false, true, false, true}) {
    auto expected = seed(mode | (sm ? saturation_model::mode : 0));
    machine.core.set_registers(expected);
    shift_model::apply(expected, expected, {});
    ++expected.pc;

    const auto progress = machine.core.progress();
    const auto run = machine.core.run(1);
    ++cases;
    require(run.retired == 1 && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = progress.instructions + 1,
                                         .estimated_cycles = progress.estimated_cycles + 1} &&
                machine.core.registers() == expected,
            "Same-PC reuse cannot carry ordinary assumptions into SM or vice versa");
  }
}

struct Undefined {
  Word opcode;
  unsigned count;
  std::size_t variant;
};

void undefined_count(const NativeImage &image, const Undefined input) {
  Machine machine;
  machine.core.load(Space::program, start, std::array{input.opcode});
  machine.core.install(image, input.variant);
  auto expected = seed(saturation_model::mode);
  register_model::write(expected, 4, input.count);
  machine.core.set_registers(expected);
  ++expected.pc;
  const auto run = machine.core.run(1);
  ++cases;
  require(run.retired == 0 && run.progress == Progress{} &&
              run.fault == Fault{.kind = FaultKind::invalid_instruction,
                                 .space = Space::program,
                                 .address = start,
                                 .value = input.count,
                                 .pc = start} &&
              machine.core.registers() == expected,
          "Undefined SM shift counts retain the explicit diagnostic before ALU effects");
}

void undefined_counts(const NativeImage &image) {
  for (unsigned count = accumulator_bits; count <= maximum_count; ++count) {
    for (const bool left : {false, true}) {
      const std::size_t direction = left ? 1 : 0;
      const auto immediate = shift_model::immediate({.count = count, .left = left});
      undefined_count(image,
                      {.opcode = immediate,
                       .count = count,
                       .variant = immediate_first +
                                  (static_cast<std::size_t>(count - accumulator_bits) * 2) +
                                  direction});
      undefined_count(
          image, {.opcode = left ? left_x0 : right_x0, .count = count, .variant = 2 + direction});
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
    if (observed && accept && patch) {
      try {
        core->load(Space::program, start + 2, std::array{multi_left});
      } catch (...) { observed = false; }
    }
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

void callback(const NativeImage &image, const Request input) {
  Observer observer;
  observer.accept = input.accept;
  observer.patch = input.patch;
  Machine machine(&observer);
  observer.core = &machine.core;
  constexpr std::array program{asl_a, io_asr_b, multi_right};
  machine.core.load(Space::program, start, program);
  machine.core.install(image, callback_first + (input.patch ? 1 : 0));
  auto expected = seed(input.mode | saturation_model::mode);
  machine.core.set_registers(expected);
  step(expected, asl_a);
  ++expected.r[0];
  ++expected.pc;
  observer.expected = expected;
  const auto old = expected;
  const auto run = machine.core.run(3);
  ++cases;
  require(observer.observed && observer.calls == 1,
          "Synchronous I/O sees the exact earlier saturated result/flags and only one request");
  if (!input.accept) {
    require(run.retired == 1 &&
                run.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = port,
                                   .pc = start + 1} &&
                machine.core.registers() == expected,
            "Rejected parallel load retains its destination but performs no shift or retirement");
    require(machine.core.run(3).retired == 0 && machine.core.registers() == expected &&
                observer.calls == 1,
            "Failure remains fail-stop without replaying a saturated prefix or I/O");
    return;
  }
  std::uint32_t suffix_retired = 0;
  if (input.patch) {
    require(run.retired == 2 && run.yielded && run.fault == Fault{} &&
                run.progress == Progress{.instructions = 2, .estimated_cycles = 2},
            "A real P edit reaches the declared changed-code boundary after the original I/O");
    machine.core.install(image, callback_first + 2);
    const auto suffix = machine.core.run(1);
    suffix_retired = suffix.retired;
    require(suffix.fault == Fault{}, "The declared new shift executes without a fault");
  }
  register_model::write(expected, 15, datum);
  shift_model::apply(expected, old, {.source_b = true, .destination_b = true});
  step(expected, input.patch ? multi_left : multi_right);
  require(run.retired + suffix_retired == 3 && run.fault.kind == FaultKind::none &&
              machine.core.progress() == Progress{.instructions = 3, .estimated_cycles = 3} &&
              machine.core.registers() == expected,
          "Parallel shifts use pre-load operands and P edits replace the following shift exactly");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    const auto image = dsp56300::test::prepare();
    dsp56300::test::undefined_counts(image);
    for (const auto mode : dsp56300::test::modes) {
      dsp56300::test::prefixes(image, mode);
      dsp56300::test::mode_reuse(image, mode);
      for (const bool accept : {false, true}) {
        for (const bool patch : {false, true}) {
          dsp56300::test::callback(image, {.mode = mode, .accept = accept, .patch = patch});
        }
      }
    }
    std::cout << dsp56300::test::cases << " saturated-shift cases and " << dsp56300::test::checks
              << " independent boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
