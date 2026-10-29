// SPDX-License-Identifier: GPL-3.0-only
// Exact public-Core old-input, observer and mode boundaries for BFU SM forms.
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using bfu_model::Form;
using bfu_model::Function;
using bfu_model::Input;
constexpr Word port = 0xffff80, datum = 0xfedcba, add_a = 0x200040, add_b = 0x200048;
constexpr Word load_x0 = 0x44d800;
constexpr Input bit_or{.function = Function::bit_or};
constexpr Input bit_and{.function = Function::bit_and};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Machine {
  explicit Machine(const Options options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

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

void prefixes(const Options options, const Word mode) {
  constexpr Input long_and{.form = Form::long_immediate, .value = 0xaaaaaa};
  constexpr Input rotate{.function = Function::rotate_left, .destination_b = true};
  const std::array program{add_a,
                           bfu_model::opcode(bit_or),
                           bfu_model::opcode(long_and),
                           long_and.value,
                           bfu_model::opcode(rotate),
                           add_b};
  for (Word budget = 1; budget <= 5; ++budget) {
    Machine machine(options);
    machine.core.load(Space::program, start, program);
    auto expected = seed(mode | bfu_model::sm);
    machine.core.set_registers(expected);
    arithmetic(expected, add_a);
    if (budget >= 2) { logic(expected, bit_or); }
    if (budget >= 3) { logic(expected, long_and); }
    if (budget >= 4) { logic(expected, rotate); }
    if (budget >= 5) { arithmetic(expected, add_b); }
    const auto run = machine.core.run(budget);
    require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
                run.progress == Progress{.instructions = budget,
                                         .estimated_cycles = budget + (budget >= 3 ? 1U : 0U)} &&
                machine.core.registers() == expected,
            "Every native/BFU prefix preserves exact flags, old operands, word lengths and costs");
    if (budget == 5 && options.backend == Backend::jit) {
      const auto stats = machine.core.jit_statistics();
      const auto native = options.optimize ? 2U : 0U;
      require(stats.native_alu_instructions == native &&
                  stats.helper_instructions == budget - native,
              "New SM BFU domains execute general helpers between real native arithmetic");
    }
  }
}

void mode_reuse(const Options options, const Word mode) {
  for (const auto function : {Function::complement, Function::rotate_right, Function::bit_xor}) {
    Machine machine(options);
    const Input input{.function = function};
    machine.core.write(Space::program, start, bfu_model::opcode(input));
    for (const bool sm : {false, true, false, true}) {
      auto expected = seed(mode | (sm ? bfu_model::sm : 0));
      machine.core.set_registers(expected);
      logic(expected, input);
      const auto previous = machine.core.progress();
      const auto run = machine.core.run(1);
      require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
                  run.progress == Progress{.instructions = previous.instructions + 1,
                                           .estimated_cycles = previous.estimated_cycles + 1} &&
                  machine.core.registers() == expected,
              "Same-PC ordinary/SM forms keep the raw BFU result and current preserved flags");
    }
  }
}

void invalid_counts(const Options options) {
  for (const auto function : {Function::left, Function::right}) {
    for (unsigned count = word_bits + 1; count < 32; ++count) {
      for (const auto form : {Form::shift_immediate, Form::shift_register}) {
        Machine machine(options);
        const Input input{.function = function, .form = form, .source = 2, .count = count};
        machine.core.write(Space::program, start, bfu_model::opcode(input));
        auto expected = seed(bfu_model::sm);
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
                "Undefined BFU SM counts retain fail-before-result diagnostics");
      }
    }
  }
}

void unqualified(const Options options) {
  constexpr Word compare_a = 0x200045;
  constexpr Word compare_long = 0x0140c5;
  constexpr std::array unsupported{0x2000U, 0x4000U, 0x20000U, mode_up | mode_down};
  const std::array inputs{bit_or,
                          Input{.form = Form::long_immediate, .value = 1},
                          Input{.function = Function::left, .form = Form::shift_immediate}};
  for (const auto input : inputs) {
    for (const auto mode : unsupported) {
      Machine machine(options);
      machine.core.write(Space::program, start, bfu_model::opcode(input));
      machine.core.write(Space::program, start + 1, input.value);
      auto expected = seed(mode | bfu_model::sm);
      machine.core.set_registers(expected);
      expected.pc += bfu_model::words(input);
      const auto run = machine.core.run(1);
      require(run.dispatches == 0 && run.progress == Progress{} &&
                  run.fault == Fault{.kind = FaultKind::unsupported_mode,
                                     .space = Space::program,
                                     .address = start,
                                     .value = expected.sr,
                                     .pc = start} &&
                  machine.core.registers() == expected,
              "BFU SM admission does not admit unqualified SC/SA/DM or reserved scaling");
    }
  }
  for (const Word opcode : {compare_a, compare_long}) {
    Machine machine(options);
    machine.core.write(Space::program, start, opcode);
    auto expected = seed(bfu_model::sm);
    machine.core.set_registers(expected);
    expected.pc += opcode == compare_a ? 1U : 2U;
    const auto run = machine.core.run(1);
    require(run.dispatches == 0 && run.progress == Progress{} &&
                run.fault.kind == FaultKind::unsupported_mode &&
                machine.core.registers() == expected,
            "Unqualified parallel and immediate CMP remain outside the new whitelist");
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
      core->write(Space::program, start + 2, bfu_model::opcode(bit_and));
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
  const auto opcode = load_x0 | (bfu_model::opcode(bit_or) & status_byte);
  const std::array program{add_a, opcode, bfu_model::opcode(bit_or)};
  machine.core.load(Space::program, start, program);
  auto expected = seed(input.mode | bfu_model::sm);
  machine.core.set_registers(expected);
  arithmetic(expected, add_a);
  const auto old = expected;
  ++expected.r[0];
  ++expected.pc;
  observer.expected = expected;
  register_model::write(expected, 4, datum);
  const auto run = machine.core.run(3);
  require(observer.observed && observer.calls == 1,
          "I/O sees exact canonical native state before a BFU old-source parallel load");
  if (!input.accept) {
    require(run.dispatches == 1 &&
                run.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = port,
                                   .pc = start + 1} &&
                machine.core.registers() == expected,
            "Rejected load writes its destination but performs no BFU operation or retirement");
    require(machine.core.run(3).dispatches == 0 && machine.core.registers() == expected &&
                observer.calls == 1,
            "Fail-stop retry never replays native prefixes or I/O");
    return;
  }
  bfu_model::apply(expected, old, bit_or);
  logic(expected, input.patch ? bit_and : bit_or);
  require(run.dispatches == 3 && run.fault.kind == FaultKind::none &&
              run.progress == Progress{.instructions = 3, .estimated_cycles = 3} &&
              machine.core.registers() == expected,
          "Accepted BFU uses old X0; a patched suffix uses current X0 with exact progress");
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
          dsp56300::test::invalid_counts(options);
          dsp56300::test::unqualified(options);
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
    std::cout << dsp56300::test::checks << " independent BFU logic boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
