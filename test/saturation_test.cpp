// SPDX-License-Identifier: GPL-3.0-only
// Independent SM execution, native-mode transitions and exact callback boundaries.
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word port = 0xffff80, datum = 0xfedcba;
constexpr Word bit_sm = 20, register_bit = 0x0ac040, set_bit = 0x20;
constexpr Word set_sm = register_bit | (register_model::reg_sr << 8) | set_bit | bit_sm;
constexpr Word clear_sm = register_bit | (register_model::reg_sr << 8) | bit_sm;
constexpr Word mac = 0x2000d3, io_mac = 0x44e0d3, clear_a = 0x200013;
constexpr std::array<Word, 13> program{0x2000d2,
                                       0x200040,
                                       0x200019,
                                       0x20006c,
                                       set_sm,
                                       io_mac,
                                       0x200019,
                                       0x200010,
                                       clear_sm,
                                       0x2000d2,
                                       0x200040,
                                       0x200019,
                                       0x20006c};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

void arithmetic(Registers &state, const Word opcode) {
  const auto old = state;
  if ((state.sr & saturation_model::mode) != 0) {
    saturation_model::alu(state, old, opcode);
  } else {
    sequence_model::alu(state, old, opcode);
  }
  ++state.pc;
}

struct Machine {
  explicit Machine(const Backend backend, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, {.backend = backend}) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

class Observer final : public Io {
public:
  Core *core = nullptr;
  Registers expected;
  Progress progress;
  bool accept = true, patch = false, observed = false;
  unsigned calls = 0;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++calls;
    observed = core != nullptr && space == Space::x && address == port && pc == start + 5 &&
               core->registers() == expected && core->progress() == progress;
    if (observed && accept && patch) { core->write(Space::program, start + 6, clear_a); }
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

Registers seed(const Word mode) {
  auto state = register_model::seed(mode);
  state.sr = mode | status_byte;
  state.a = saturation_model::positive;
  state.b = saturation_model::negative - 1;
  state.r[0] = port;
  return state;
}

void callback(const Backend backend, const Word mode, const bool accept, const bool patch) {
  Observer observer;
  observer.accept = accept;
  observer.patch = patch;
  Machine machine(backend, &observer);
  observer.core = &machine.core;
  machine.core.load(Space::program, start, program);
  auto expected = seed(mode);
  machine.core.set_registers(expected);
  for (const auto word : std::span(program).first(4)) { arithmetic(expected, word); }
  expected.sr |= saturation_model::mode;
  ++expected.pc;
  observer.expected = expected;
  ++observer.expected.pc; // The synchronous request observes the fetched instruction's PC.
  observer.progress = {.instructions = 5, .estimated_cycles = 6};
  const auto run = machine.core.run(static_cast<Word>(program.size()));
  require(observer.observed && observer.calls == 1,
          "SM callback observes full native prefix state, numeric mode and exact progress");
  const auto old = expected;
  register_model::write(expected, 4, datum);
  if (!accept) {
    ++expected.pc;
    require(run.dispatches == 5 && run.progress == observer.progress &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = port,
                                   .pc = start + 5} &&
                machine.core.registers() == expected,
            "Rejected SM transfer preserves partial destination with no ALU or retirement");
    require(machine.core.run(1).dispatches == 0 && machine.core.registers() == expected,
            "Rejected SM requests remain fail-stop without replay");
    return;
  }
  saturation_model::alu(expected, old, io_mac);
  ++expected.pc;
  arithmetic(expected, patch ? clear_a : program[6]);
  arithmetic(expected, program[7]);
  expected.sr &= ~saturation_model::mode;
  ++expected.pc;
  for (const auto word : std::span(program).last(4)) { arithmetic(expected, word); }
  require(run.dispatches == program.size() && run.fault.kind == FaultKind::none &&
              run.progress == Progress{.instructions = program.size(),
                                       .estimated_cycles = program.size() + 2} &&
              machine.core.registers() == expected && machine.core.interruptible(),
          "Native/SM/native execution keeps old parallel operands, sticky flags and mode changes");
  if (backend == Backend::jit) {
    const auto stats = machine.core.jit_statistics();
    require(stats.native_alu_instructions == 11 && stats.helper_instructions == 2,
            "Ordinary and SM arithmetic execute natively; mode-control instructions use helpers");
  }
  auto expected_p = std::array<Word, memory_words>{};
  for (std::size_t index = 0; index < program.size(); ++index) {
    expected_p.at(start + index) = patch && index == 6 ? clear_a : program[index];
  }
  require(machine.p == expected_p && machine.x == std::array<Word, memory_words>{} &&
              machine.y == std::array<Word, memory_words>{},
          "Only the admitted callback edit changes P; X/Y and every guard remain intact");
}

void same_pc(const Backend backend, const Word mode) {
  Machine machine(backend);
  constexpr std::array words{mac, mac, mac, mac};
  machine.core.load(Space::program, start, words);
  for (const bool sm : {false, true, false, true}) {
    auto expected = seed(mode);
    if (sm) { expected.sr |= saturation_model::mode; }
    machine.core.set_registers(expected);
    const auto before = machine.core.progress();
    const auto stats = machine.core.jit_statistics();
    for (const auto word : words) { arithmetic(expected, word); }
    const auto run = machine.core.run(static_cast<Word>(words.size()));
    require(run.dispatches == words.size() && run.fault.kind == FaultKind::none &&
                machine.core.registers() == expected &&
                run.progress ==
                    Progress{.instructions = before.instructions + words.size(),
                             .estimated_cycles = before.estimated_cycles + words.size()},
            "Same-PC SM changes preserve independent full state without reusing ordinary code");
    if (backend == Backend::jit) {
      const auto after = machine.core.jit_statistics();
      require(after.native_alu_instructions == stats.native_alu_instructions + 4 &&
                  after.helper_instructions == stats.helper_instructions,
              "Both exact numeric profiles execute independent native arithmetic");
    }
  }
}

void prefixes(const Backend backend, const Word mode) {
  Machine machine(backend);
  constexpr std::array<Word, 8> words{
      0x2000d2, 0x200040, 0x200019, 0x20006c, set_sm, 0x2000d3, clear_sm, 0x200040};
  machine.core.load(Space::program, start, words);
  for (std::size_t budget = 0; budget <= words.size(); ++budget) {
    auto expected = seed(mode);
    machine.core.set_registers(expected);
    const auto before = machine.core.progress();
    std::uint64_t cycles = 0;
    for (const auto word : std::span(words).first(budget)) {
      const bool control = word == set_sm || word == clear_sm;
      if (control) {
        expected.sr = word == set_sm ? expected.sr | saturation_model::mode
                                     : expected.sr & ~saturation_model::mode;
        ++expected.pc;
      } else {
        arithmetic(expected, word);
      }
      cycles += control ? 2U : 1U;
    }
    const auto run = machine.core.run(static_cast<Word>(budget));
    require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
                machine.core.registers() == expected &&
                run.progress == Progress{.instructions = before.instructions + budget,
                                         .estimated_cycles = before.estimated_cycles + cycles},
            "Every native/SM/control prefix is canonical with exact partial budgets and flags");
  }
}

void transfer(const Backend backend, const Word mode, const Word flags, const bool paired) {
  constexpr Word address = 8;
  constexpr Word paired_store = 0xbb18d3;
  constexpr Word self_move = 0x21ce00;
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> shared{};
  shared.fill(datum);
  Core core({.program = p, .x = shared, .y = shared}, nullptr, {.backend = backend});
  core.write(Space::program, start, paired ? paired_store : self_move);
  auto initial = seed(mode);
  initial.sr = mode | flags | saturation_model::mode;
  initial.r[0] = initial.r[4] = address;
  initial.a = saturation_model::positive + 1;
  initial.b = saturation_model::negative - 1;
  auto expected = initial;
  auto expected_memory = shared;
  if (paired) {
    static_cast<void>(register_model::bus_word(expected, initial.a));
    expected_memory[address] = register_model::bus_word(expected, initial.b);
    ++expected.r[0];
    ++expected.r[4];
    saturation_model::alu(expected, initial, paired_store);
  } else {
    const auto value = register_model::bus_word(expected, initial.a);
    register_model::write(expected, 14, value);
  }
  ++expected.pc;
  core.set_registers(initial);
  const auto run = core.run(1);
  require(run.dispatches == 1 && run.fault.kind == FaultKind::none &&
              run.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
              core.registers() == expected && shared == expected_memory,
          "SM preserves paired old operands/X-before-Y aliases and MOVE's distinct V/L contract");
}

struct Unsupported {
  Word instruction, status;
};

void reject(const Backend backend, const Unsupported input) {
  Machine machine(backend);
  machine.core.write(Space::program, start, input.instruction);
  auto expected = seed(0);
  expected.sr = input.status;
  machine.core.set_registers(expected);
  ++expected.pc;
  const auto run = machine.core.run(1);
  require(run.dispatches == 0 && run.progress == Progress{} &&
              run.fault == Fault{.kind = FaultKind::unsupported_mode,
                                 .space = Space::program,
                                 .address = start,
                                 .value = input.status,
                                 .pc = start} &&
              machine.core.registers() == expected,
          "Unqualified SM arithmetic/combinations reject before register, RAM or ALU effects");
  require(machine.core.run(1).dispatches == 0 && machine.core.registers() == expected,
          "An unqualified mode remains fail-stop without invented execution");
}

void rejected_modes(const Backend backend) {
  constexpr std::array<Word, 8> instructions{
      0x200022, 0x200032, 0x200017, 0x200003, 0x200045, 0x014085, 0x200012, 0x200002};
  for (const auto instruction : instructions) {
    reject(backend, {.instruction = instruction, .status = saturation_model::mode});
  }
  for (const Word other : {0x2000U, 0x4000U, 0x20000U, mode_up | mode_down}) {
    reject(backend, {.instruction = mac, .status = saturation_model::mode | other});
  }
}

} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::rejected_modes(backend);
      for (const auto mode : dsp56300::test::modes) {
        dsp56300::test::same_pc(backend, mode);
        dsp56300::test::prefixes(backend, mode);
        for (dsp56300::Word flags = 0; flags <= dsp56300::test::status_byte; ++flags) {
          for (const bool paired : {false, true}) {
            dsp56300::test::transfer(backend, mode, flags, paired);
          }
        }
        for (const bool accept : {false, true}) {
          for (const bool patch : {false, true}) {
            dsp56300::test::callback(backend, mode, accept, patch);
          }
        }
      }
    }
    std::cout << dsp56300::test::checks << " independent SM boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
