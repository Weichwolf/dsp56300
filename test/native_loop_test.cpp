// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <set>
#include <string>
#include <tuple>

namespace dsp56300::test {
namespace {
constexpr Word loop_flag = Word{1} << 15, mac_a = 0x2000d3, mac_b = 0x2000fb;
constexpr Word old_la = 0x70, old_lc = 7;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array ends{0U, start, start + 1, start + 2, word_mask};

struct Machine {
  explicit Machine(const Options options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

Registers seeded(const Word mode) {
  auto state = register_model::seed(mode);
  state.sp = 2;
  state.sc = 2;
  state.stack[1] = (std::uint64_t{old_la} << word_bits) | old_lc;
  state.stack[2] = std::uint64_t{start + 2} << word_bits;
  return state;
}

void model(Registers &state, const Word opcode) {
  const auto old = state;
  sequence_model::alu(state, old, opcode);
  ++state.pc;
}

using Keys = std::set<std::tuple<Word, bool, Word>>;

struct LoopInput {
  Word mode = 0, end = 0, count = 0;
  bool active = false;
};

void warm_case(Machine &machine, const Options options, Keys &keys, const LoopInput input) {
  auto initial = seeded(input.mode | (input.active ? loop_flag : 0));
  initial.la = input.end;
  initial.lc = input.count;
  auto expected = initial;
  model(expected, mac_a);
  const bool ending = input.active && input.end == start;
  if (ending && input.count > 1) {
    expected.lc = input.count - 1;
    expected.pc = start + 2;
  } else if (ending) {
    expected.sr &= ~loop_flag;
    expected.la = old_la;
    expected.lc = old_lc;
    expected.sp = 0;
    expected.sc = 0;
  }
  machine.core.set_registers(initial);
  const auto before = machine.core.jit_statistics();
  require(machine.core.run(1).dispatches == 1 && machine.core.registers() == expected,
          "Warm translations distinguish numeric modes, active LF and every LA context");
  if (options.backend == Backend::jit && options.optimize) {
    const bool native = !ending || input.count > 1;
    const auto after = machine.core.jit_statistics();
    require(after.native_alu_instructions == before.native_alu_instructions + (native ? 1U : 0U),
            "Continuing loop endings use native code; final cleanup remains general");
    keys.emplace(input.mode, input.active, input.active ? input.end : 0);
    require(after.compiled_blocks == keys.size(), "LC changes reuse the same guarded translation");
  }
}

void warm_contexts(const Options options) {
  Machine machine(options);
  machine.core.write(Space::program, start, mac_a);
  Keys keys;
  for (const auto mode : modes) {
    for (const bool active : {false, true}) {
      for (const auto end : ends) {
        for (const Word count : {0U, 1U, 3U}) {
          warm_case(
              machine, options, keys, {.mode = mode, .end = end, .count = count, .active = active});
        }
      }
    }
  }
}

void stack_slots(const Options options) {
  Machine machine(options);
  machine.core.write(Space::program, start, mac_a);
  for (Word pointer = 0; pointer < 64; ++pointer) {
    auto initial = seeded(loop_flag);
    initial.sp = pointer;
    initial.sc = 0; // Continuing a loop reads its frame without popping it.
    initial.la = start;
    initial.lc = 2;
    const auto destination = start + 3 + pointer;
    initial.stack[pointer % stack_entries] = std::uint64_t{destination} << word_bits;
    auto expected = initial;
    model(expected, mac_a);
    expected.pc = destination;
    expected.lc = 1;
    machine.core.set_registers(initial);
    require(machine.core.run(1).dispatches == 1 && machine.core.registers() == expected,
            "Continuing loops read the current eight-byte frame for all SP wrap positions");
  }
}

void guest_changes(const Options options, const unsigned kind) {
  constexpr Word write_la = 0x04c4be;
  constexpr Word write_sr = 0x04c4b9;
  constexpr Word write_lc = 0x04c4bf;
  constexpr Word write_sp = 0x04c4bb;
  const std::array controls{write_la, write_sr, write_sr, write_lc, write_sp};
  Machine machine(options);
  const std::array program{controls.at(kind), mac_a, mac_b};
  machine.core.load(Space::program, start, program);
  auto initial = seeded(kind == 1 ? 0 : loop_flag);
  initial.la = kind == 0 ? start + 10 : start + 1;
  initial.lc = kind == 3 ? 1 : 3;
  initial.stack[5] = std::uint64_t{start + 2} << word_bits;
  const std::array values{start + 1, loop_flag, Word{0}, Word{3}, Word{5}};
  initial.x = (initial.x & ~std::uint64_t{word_mask}) | values.at(kind);
  auto expected = initial;
  if (kind == 0) { expected.la = start + 1; }
  if (kind == 1 || kind == 2) { expected.sr = values.at(kind); }
  if (kind == 3) { expected.lc = 3; }
  if (kind == 4) { expected.sp = 5; }
  ++expected.pc;
  model(expected, mac_a);
  if (kind != 2) { --expected.lc; }
  model(expected, mac_b);
  machine.core.set_registers(initial);
  require(machine.core.run(3).dispatches == 3 && machine.core.registers() == expected,
          "Guest changes to LA/LF exit stale plans; LC/SP remain dynamically observed");
}

class Writer final : public Io {
public:
  Core *core = nullptr;
  bool observed = false;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    observed = space == Space::x && address == port && pc == start && core != nullptr &&
               core->registers().lc == 2 && core->progress().instructions == 0;
    if (!observed) { return false; }
    core->write(Space::program, start, clear_a);
    value = 1;
    return true;
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

  static constexpr Word port = 0xffff80, clear_a = 0x200013;
};

void callback_change(const Options options) {
  Writer writer;
  Machine machine(options, &writer);
  writer.core = &machine.core;
  constexpr Word read_x_negate_a = 0x44e036;
  machine.core.write(Space::program, start, read_x_negate_a);
  auto initial = seeded(loop_flag);
  initial.sr = loop_flag;
  initial.a = 3;
  initial.x = 0;
  initial.r[0] = Writer::port;
  initial.lc = 2;
  initial.la = start;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  auto expected = initial;
  expected.pc = start + 1;
  expected.a = 0;
  expected.x = 1;
  expected.sr = flag_z | flag_u;
  expected.la = old_la;
  expected.lc = old_lc;
  expected.sp = 0;
  expected.sc = 0;
  machine.core.set_registers(initial);
  require(machine.core.run(2).dispatches == 2 && writer.observed &&
              machine.core.registers() == expected,
          "A loop-ending callback completes its fetched instruction before modified code executes");
}

void invalid_modes(const Options options) {
  constexpr std::array unsupported{0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_up | mode_down};
  for (const auto mode : unsupported) {
    Machine machine(options);
    machine.core.write(Space::program, start, mac_a);
    auto initial = seeded(loop_flag);
    initial.la = start + 1;
    initial.lc = 3;
    machine.core.set_registers(initial);
    require(machine.core.run(1).dispatches == 1, "Prime a valid numeric/loop translation");
    initial.sr |= mode;
    machine.core.set_registers(initial);
    const auto result = machine.core.run(1);
    require(result.dispatches == 0 && result.progress.instructions == 1 &&
                result.fault.kind == FaultKind::unsupported_mode,
            "No compressed-key collision can reuse valid native code for an unsupported mode");
  }
}

void high_addresses(const Options options) {
  const MemoryArena arena;
  Core core(arena.buffers(), nullptr, options);
  constexpr std::array addresses{0U, start, 0x7fffffU, 0xfffffeU, word_mask};
  for (const auto address : addresses) {
    core.write(Space::program, address, mac_a);
    auto initial = seeded(loop_flag);
    initial.pc = address;
    initial.la = address;
    initial.lc = 2;
    initial.stack[2] = std::uint64_t{word_mask} << word_bits;
    auto expected = initial;
    model(expected, mac_a);
    expected.pc = word_mask;
    expected.lc = 1;
    core.set_registers(initial);
    require(core.run(1).dispatches == 1 && core.registers() == expected,
            "Loop keys, ending words and saved PCs preserve address " + std::to_string(address) +
                " on " + std::string(backend_name(options.backend)));
  }
  constexpr Word immediate_add = 0x0140c0;
  core.write(Space::program, word_mask, immediate_add);
  core.write(Space::program, 0, 1);
  auto initial = seeded(loop_flag);
  initial.pc = word_mask;
  initial.la = 0; // The extension word is the loop's final word.
  initial.lc = 2;
  initial.stack[2] = std::uint64_t{word_mask} << word_bits;
  auto expected = initial;
  auto operand = initial;
  operand.x = 1;
  sequence_model::alu(expected, operand, 0x40);
  expected.pc = word_mask;
  expected.lc = 1;
  core.set_registers(initial);
  const auto before = core.progress();
  require(core.run(1).dispatches == 1 && core.registers() == expected &&
              core.progress().estimated_cycles == before.estimated_cycles + 2,
          "A wrapped two-word instruction can end a loop at its extension word");
}

void ending_fault(const Options options) {
  Machine machine(options);
  constexpr Word load_x_negate_a = 0x44e036;
  machine.core.write(Space::program, start, load_x_negate_a);
  auto initial = seeded(loop_flag);
  initial.la = start;
  initial.lc = 2;
  initial.r[0] = memory_words;
  machine.core.set_registers(initial);
  const auto result = machine.core.run(1);
  require(result.dispatches == 0 && result.progress == Progress{} &&
              result.fault.kind == FaultKind::memory_read && result.fault.pc == start,
          "A failing transfer at a continuing loop ending does not retire its ALU");
  const auto actual = machine.core.registers();
  require(actual.lc == initial.lc && actual.sp == initial.sp && actual.sc == initial.sc &&
              actual.a == initial.a && actual.stack == initial.stack,
          "A loop-ending transfer fault cannot decrement LC or read/pop a continuation frame");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      for (const bool optimize : {false, true}) {
        const dsp56300::Options options{.backend = backend, .optimize = optimize};
        dsp56300::test::warm_contexts(options);
        dsp56300::test::stack_slots(options);
        for (unsigned kind = 0; kind < 5; ++kind) { dsp56300::test::guest_changes(options, kind); }
        dsp56300::test::callback_change(options);
        dsp56300::test::invalid_modes(options);
        dsp56300::test::high_addresses(options);
        dsp56300::test::ending_fault(options);
      }
    }
    std::cout << dsp56300::test::checks << " native loop boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
