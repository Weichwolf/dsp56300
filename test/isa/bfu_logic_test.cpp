// SPDX-License-Identifier: GPL-3.0-only
// Prepared BFU old-input, observer and mode boundaries with independent bit models.
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "register_model.hpp"
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
using bfu_model::Form;
using bfu_model::Function;
using bfu_model::Input;
constexpr Word port = 0xffff80, datum = 0xfedcba;
constexpr Word add_a = static_cast<Word>(isa::Opcode::add_x0_a);
constexpr Word add_b = static_cast<Word>(isa::Opcode::add_x0_b);
constexpr Word load_x0 = static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) | (3U << 11);
constexpr Word compare_a = static_cast<Word>(isa::Opcode::cmp_x0_a);
constexpr Word compare_long = static_cast<Word>(isa::Opcode::cmp_immediate_long);
constexpr std::size_t invalid_first = 4, unqualified_first = 32, compare_first = 35,
                      callback_first = 37;
constexpr std::array reused_functions{
    Function::complement, Function::rotate_right, Function::bit_xor};
constexpr std::array shifted_functions{Function::left, Function::right};
constexpr std::array shifted_forms{Form::shift_immediate, Form::shift_register};
std::size_t cases = 0;
constexpr Input bit_or{.function = Function::bit_or};
constexpr Input bit_and{.function = Function::bit_and};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Machine {
  explicit Machine(Io *io = nullptr) : core(Memory::separate({.program = p, .x = x, .y = y}), io) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

std::array<Word, 6> prefix_program() {
  constexpr Input long_and{.form = Form::long_immediate, .value = 0xaaaaaa};
  constexpr Input rotate{.function = Function::rotate_left, .destination_b = true};
  return {add_a,
          bfu_model::opcode(bit_or),
          bfu_model::opcode(long_and),
          long_and.value,
          bfu_model::opcode(rotate),
          add_b};
}

const std::array unqualified_inputs{
    bit_or,
    Input{.form = Form::long_immediate, .value = 1},
    Input{.function = Function::left, .form = Form::shift_immediate}};

NativeImage prepare() {
  const auto prefix = prefix_program();
  std::vector<std::vector<Word>> programs{{prefix.begin(), prefix.end()}};
  programs.reserve(callback_first + 3);
  for (const auto function : reused_functions) {
    programs.push_back({bfu_model::opcode({.function = function})});
  }
  for (const auto function : shifted_functions) {
    for (unsigned count = word_bits + 1; count < 32; ++count) {
      for (const auto form : shifted_forms) {
        programs.push_back(
            {bfu_model::opcode({.function = function, .form = form, .source = 2, .count = count})});
      }
    }
  }
  for (const auto input : unqualified_inputs) {
    programs.push_back({bfu_model::opcode(input)});
    if (bfu_model::words(input) == 2) { programs.back().push_back(input.value); }
  }
  programs.push_back({compare_a});
  programs.push_back({compare_long, datum});
  const auto parallel = load_x0 | (bfu_model::opcode(bit_or) & status_byte);
  programs.push_back({add_a, parallel, bfu_model::opcode(bit_or)});
  programs.push_back({add_a, parallel});
  programs.push_back({add_a, parallel, bfu_model::opcode(bit_and)});
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
  state.sr = mode | status_byte;
  state.a = saturation_model::positive + 1;
  state.r[0] = port;
  return state;
}

void arithmetic(Registers &state, const Word opcode) {
  saturation_model::alu(state, state, opcode);
  ++state.pc;
}

void logic(Registers &state, const Input input) {
  bfu_model::apply(state, state, input);
  state.pc += bfu_model::words(input);
}

void prefixes(const NativeImage &image, const Word mode) {
  constexpr Input long_and{.form = Form::long_immediate, .value = 0xaaaaaa};
  constexpr Input rotate{.function = Function::rotate_left, .destination_b = true};
  const auto program = prefix_program();
  for (Word budget = 1; budget <= 5; ++budget) {
    Machine machine;
    machine.core.load(Space::program, start, program);
    machine.core.install(image, 0);
    auto expected = seed(mode | bfu_model::sm);
    machine.core.set_registers(expected);
    arithmetic(expected, add_a);
    if (budget >= 2) { logic(expected, bit_or); }
    if (budget >= 3) { logic(expected, long_and); }
    if (budget >= 4) { logic(expected, rotate); }
    if (budget >= 5) { arithmetic(expected, add_b); }
    const auto run = machine.core.run(budget);
    ++cases;
    require(run.retired == budget && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = budget,
                                         .estimated_cycles = budget + (budget >= 3 ? 1U : 0U)} &&
                machine.core.registers() == expected,
            "Every native/BFU prefix preserves exact flags, old operands, word lengths and costs");
  }
}

void mode_reuse(const NativeImage &image, const Word mode) {
  for (std::size_t index = 0; index < reused_functions.size(); ++index) {
    const auto function = reused_functions[index];
    Machine machine;
    const Input input{.function = function};
    machine.core.load(Space::program, start, std::array{bfu_model::opcode(input)});
    machine.core.install(image, 1 + index);
    for (const bool sm : {false, true, false, true}) {
      auto expected = seed(mode | (sm ? bfu_model::sm : 0));
      machine.core.set_registers(expected);
      logic(expected, input);
      const auto previous = machine.core.progress();
      const auto run = machine.core.run(1);
      ++cases;
      require(run.retired == 1 && run.fault.kind == FaultKind::none &&
                  run.progress == Progress{.instructions = previous.instructions + 1,
                                           .estimated_cycles = previous.estimated_cycles + 1} &&
                  machine.core.registers() == expected,
              "Same-PC ordinary/SM forms keep the raw BFU result and current preserved flags");
    }
  }
}

void invalid_counts(const NativeImage &image) {
  std::size_t variant = invalid_first;
  for (const auto function : shifted_functions) {
    for (unsigned count = word_bits + 1; count < 32; ++count) {
      for (const auto form : shifted_forms) {
        Machine machine;
        const Input input{.function = function, .form = form, .source = 2, .count = count};
        machine.core.load(Space::program, start, std::array{bfu_model::opcode(input)});
        machine.core.install(image, variant++);
        auto expected = seed(bfu_model::sm);
        register_model::write(expected, 4, count);
        machine.core.set_registers(expected);
        ++expected.pc;
        const auto run = machine.core.run(1);
        ++cases;
        require(run.retired == 0 && run.progress == Progress{} &&
                    run.fault == Fault{.kind = FaultKind::invalid_instruction,
                                       .space = Space::program,
                                       .address = start,
                                       .value = count,
                                       .pc = start} &&
                    machine.core.registers() == expected,
                "Undefined BFU SM counts retain fail-before-result diagnostics");
      }
    }
  }
}

void unqualified(const NativeImage &image) {
  constexpr std::array unsupported{0x2000U, 0x4000U, 0x20000U, mode_up | mode_down};
  std::size_t variant = unqualified_first;
  for (const auto input : unqualified_inputs) {
    for (const auto mode : unsupported) {
      Machine machine;
      machine.core.load(Space::program, start, std::array{bfu_model::opcode(input), input.value});
      machine.core.install(image, variant);
      auto expected = seed(mode | bfu_model::sm);
      machine.core.set_registers(expected);
      expected.pc += bfu_model::words(input);
      const auto run = machine.core.run(1);
      ++cases;
      require(run.retired == 0 && run.progress == Progress{} &&
                  run.fault == Fault{.kind = FaultKind::unsupported_mode,
                                     .space = Space::program,
                                     .address = start,
                                     .value = expected.sr,
                                     .pc = start} &&
                  machine.core.registers() == expected,
              "BFU SM admission does not admit unqualified SC/SA/DM or reserved scaling");
    }
    ++variant;
  }
  variant = compare_first;
  for (const Word opcode : {compare_a, compare_long}) {
    Machine machine;
    machine.core.load(Space::program, start, std::array{opcode, datum});
    machine.core.install(image, variant++);
    auto expected = seed(bfu_model::sm);
    machine.core.set_registers(expected);
    // SM CMP was independently qualified after this BFU test was introduced.
    // The long immediate is the same signed word as X0; compare preserves A.
    saturation_model::alu(expected, expected, compare_a);
    const auto words = opcode == compare_a ? 1U : 2U;
    expected.pc += words;
    const auto run = machine.core.run(1);
    ++cases;
    require(run.retired == 1 &&
                run.progress == Progress{.instructions = 1, .estimated_cycles = words} &&
                run.fault.kind == FaultKind::none && machine.core.registers() == expected,
            "Qualified SM parallel/immediate CMP preserve A and independent saturation flags");
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
        core->load(Space::program, start + 2, std::array{bfu_model::opcode(bit_and)});
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
  const auto opcode = load_x0 | (bfu_model::opcode(bit_or) & status_byte);
  const std::array program{add_a, opcode, bfu_model::opcode(bit_or)};
  machine.core.load(Space::program, start, program);
  machine.core.install(image, callback_first + (input.patch ? 1 : 0));
  auto expected = seed(input.mode | bfu_model::sm);
  machine.core.set_registers(expected);
  arithmetic(expected, add_a);
  const auto old = expected;
  ++expected.r[0];
  ++expected.pc;
  observer.expected = expected;
  register_model::write(expected, 4, datum);
  const auto run = machine.core.run(3);
  ++cases;
  require(observer.observed && observer.calls == 1,
          "I/O sees exact canonical native state before a BFU old-source parallel load");
  if (!input.accept) {
    require(run.retired == 1 &&
                run.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = port,
                                   .pc = start + 1} &&
                machine.core.registers() == expected,
            "Rejected load writes its destination but performs no BFU operation or retirement");
    require(machine.core.run(3).retired == 0 && machine.core.registers() == expected &&
                observer.calls == 1,
            "Fail-stop retry never replays native prefixes or I/O");
    return;
  }
  std::uint32_t suffix_retired = 0;
  if (input.patch) {
    require(run.retired == 2 && run.yielded && run.fault == Fault{} &&
                run.progress == Progress{.instructions = 2, .estimated_cycles = 2},
            "A real P edit reaches its prepared handoff after the accepted old-input BFU");
    machine.core.install(image, callback_first + 2);
    const auto suffix = machine.core.run(1);
    suffix_retired = suffix.retired;
    require(suffix.fault == Fault{}, "The prepared changed BFU suffix executes without a fault");
  }
  bfu_model::apply(expected, old, bit_or);
  logic(expected, input.patch ? bit_and : bit_or);
  require(run.retired + suffix_retired == 3 && run.fault.kind == FaultKind::none &&
              machine.core.progress() == Progress{.instructions = 3, .estimated_cycles = 3} &&
              machine.core.registers() == expected,
          "Accepted BFU uses old X0; a patched suffix uses current X0 with exact progress");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    const auto image = dsp56300::test::prepare();
    dsp56300::test::invalid_counts(image);
    dsp56300::test::unqualified(image);
    for (const auto mode : dsp56300::test::modes) {
      dsp56300::test::prefixes(image, mode);
      dsp56300::test::mode_reuse(image, mode);
      for (const bool accept : {false, true}) {
        for (const bool patch : {false, true}) {
          dsp56300::test::callback(image, {.mode = mode, .accept = accept, .patch = patch});
        }
      }
    }
    std::cout << dsp56300::test::cases << " BFU cases and " << dsp56300::test::checks
              << " independent boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
