// SPDX-License-Identifier: GPL-3.0-only
#include "external_run_fixture.hpp"

#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <chrono>
#include <initializer_list>
#include <stdexcept>

namespace dsp56300::test::external_run {
bool Bus::read(const Space space, const Word index, const Word pc, Word &value) noexcept {
  try {
    ++reads;
    if (record) {
      events.push_back({.operation = Operation::read, .space = space, .address = index, .pc = pc});
    }
    const auto key = Key{space, index};
    const auto found = memory.find(key);
    if (refuse_read == key || found == memory.end()) { return false; }
    value = found->second;
    return true;
  } catch (...) { return false; }
}

bool Bus::write(const Space space, const Word index, const Word pc, const Word value) noexcept {
  try {
    ++writes;
    if (record) {
      events.push_back({.operation = Operation::write,
                        .space = space,
                        .address = index,
                        .pc = pc,
                        .value = value});
    }
    const auto key = Key{space, index};
    if (refuse_write == key) { return false; }
    memory[key] = value;
    return true;
  } catch (...) { return false; }
}

bool Bus::bit(const Space space,
              const Word index,
              const Word pc,
              const Word bit,
              const BitAction action,
              bool &previous) noexcept {
  try {
    ++bits;
    if (record) {
      events.push_back({.operation = Operation::bit,
                        .space = space,
                        .address = index,
                        .pc = pc,
                        .value = bit,
                        .action = action});
    }
    const auto found = memory.find({space, index});
    if (refuse_bit || found == memory.end() || bit >= word_bits) { return false; }
    const auto mask = Word{1} << bit;
    previous = (found->second & mask) != 0;
    switch (action) {
      case BitAction::clear: found->second &= ~mask; break;
      case BitAction::set: found->second |= mask; break;
      case BitAction::change: found->second ^= mask; break;
      case BitAction::test: break;
      default: return false;
    }
    return true;
  } catch (...) { return false; }
}

bool Bus::reset_peripherals(const Word pc) noexcept {
  try {
    ++resets;
    if (record) { events.push_back({.operation = Operation::reset, .pc = pc}); }
    return !refuse_reset;
  } catch (...) { return false; }
}

void Bus::program(const Word first, const std::initializer_list<Word> words) {
  auto cursor = first;
  for (const auto word : words) { memory[{Space::program, cursor++}] = word; }
}

namespace {
void complete_owned(Core &core, InstructionProgress progress) {
  for (unsigned effects = 0; !progress.retired && effects < 32; ++effects) {
    if (!progress.pending || progress.fault.kind != FaultKind::none) {
      throw std::runtime_error("Authored external-Io staged workload fault");
    }
    progress = core.advance_instruction(progress.pending->ticket);
  }
  require(progress.retired, "The authored staged workload retires its actual instruction");
}

void execute_staged(Core &core, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) { complete_owned(core, core.begin_fetch()); }
}

void execute_synchronous(Core &core, const unsigned count, const bool single_step) {
  const auto calls = single_step ? count : 1U;
  for (unsigned index = 0; index < calls; ++index) {
    const auto budget = single_step ? 1U : count;
    const auto result = core.run(budget);
    require(result.fault.kind == FaultKind::none && result.dispatches == budget,
            "The authored synchronous workload retires its complete real instruction budget");
  }
}

Registers expected_workload(Registers state, const Workload input, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) {
    const auto before = state;
    if (input.name == "communications-xor") {
      bfu_model::apply(state, before, {.function = bfu_model::Function::bit_xor});
    } else {
      sequence_model::alu(state, before, input.opcode);
    }
    ++state.pc;
  }
  return state;
}
}

Timing
workload(const Workload input, const Backend backend, const bool staged, const bool single_step) {
  constexpr unsigned count = 4096;
  Bus bus;
  bus.record = false;
  for (unsigned index = 0; index < count; ++index) {
    bus.memory[{Space::program, origin + index}] = input.opcode;
  }
  Core core(bus, {.backend = backend});
  auto initial = initial_state();
  initial.x = input.x;
  initial.y = input.y;
  core.set_registers(initial);
  const auto expected = expected_workload(initial, input, count);
  const auto begin = std::chrono::steady_clock::now();
  if (staged) {
    execute_staged(core, count);
  } else {
    execute_synchronous(core, count, single_step);
  }
  const auto seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  require(
      core.registers() == expected &&
          core.progress() == Progress{.instructions = count, .estimated_cycles = count},
      "Independent raw56 arithmetic/bit expectations include every register, flag and retirement");
  require(bus.reads == count && bus.writes == 0 && bus.bits == 0 && bus.resets == 0,
          "Each uncached opcode has one real P read and no fabricated data/peripheral operations");
  require(core.jit_statistics().compiled_blocks == 0,
          "External memory execution makes no native-JIT claim");
  return {.seconds = seconds, .instructions = count, .reads = bus.reads};
}

Timing captured_workload(const Workload input, const Backend backend) {
  constexpr unsigned count = 4096;
  Bus bus;
  bus.record = false;
  Core core(bus, {.backend = backend});
  auto initial = initial_state();
  initial.x = input.x;
  initial.y = input.y;
  core.set_registers(initial);
  const auto expected = expected_workload(initial, input, count);
  const auto begin = std::chrono::steady_clock::now();
  for (unsigned index = 0; index < count; ++index) {
    complete_owned(core,
                   core.begin_instruction({.address = origin + index, .opcode = input.opcode},
                                          InstructionPublication::pcu_updates));
  }
  const auto seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  require(
      core.registers() == expected &&
          core.progress() == Progress{.instructions = count, .estimated_cycles = count},
      "Captured PCU workloads retain every independently calculated register, flag and base cost");
  require(bus.reads == 0 && bus.writes == 0 && bus.bits == 0 && bus.resets == 0,
          "Supplied captured words cause no implicit program read or fabricated peripheral input");
  return {.seconds = seconds, .instructions = count, .reads = bus.reads};
}

}
