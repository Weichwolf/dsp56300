// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "hidden_retirement.hpp"
#include "instruction.hpp"
#include "support.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace dsp56300::test {
namespace {
using detail::HiddenRetirement;
using detail::retirement_bit;
using detail::RetirementField;
using detail::VectorPhase;

constexpr Word active_pc = 0x123456, active_r = 0x654321, repeated_pc = 0x234567;
constexpr Word saved_lc = 0x345678, add_x0 = 0x200040;
constexpr Word resume_pc = 0x456789, vector_address = 0x56789a, debug_address = 0x6789ab;
constexpr std::uint64_t live_data = 0x123456fedcba, live_frame = 0x987654321abc;
constexpr std::uint64_t instructions_before = 41, cycles_before = 97;
constexpr std::uint64_t instructions_after = 42, two_cycles_after = 99, three_cycles_after = 100;
constexpr Word repeat_before = 7;
constexpr std::uint8_t all_debug_bits = 0xff;

struct Live {
  detail::Cpu cpu;
  detail::VectorState vector;
  Word debug_vector = 0;
  detail::Instruction repeated;
};

Live seed() {
  Live result;
  result.cpu.reg.pc = active_pc;
  result.cpu.reg.x = live_data;
  result.cpu.reg.r[0] = active_r;
  result.cpu.reg.stack[2] = live_frame;
  result.cpu.instructions = instructions_before;
  result.cpu.cycles = cycles_before;
  result.cpu.repeat_remaining = repeat_before;
  result.cpu.repeat_pc = repeated_pc;
  result.cpu.repeat_lc = saved_lc;
  result.cpu.repeat_opcode = add_x0;
  result.cpu.repeat_loaded = true;
  result.cpu.repeat_completed = true;
  result.cpu.interrupt_inhibited = true;
  result.cpu.processing = ProcessingState::waiting;
  result.cpu.debug_flags = detail::debug_interrupt_mode | detail::debug_software_occurrence |
                           detail::debug_interrupt_pending;
  result.vector = {
      .resume = resume_pc, .address = vector_address, .level = 2, .phase = VectorPhase::first};
  result.debug_vector = debug_address;
  result.repeated = {.opcode = add_x0, .operation = detail::Operation::parallel};
  return result;
}

void expect(const Live &actual, const Live &expected) {
  require(actual.cpu.reg == expected.cpu.reg,
          "Hidden retirement cannot restore an old architectural register file");
  require(actual.cpu.instructions == expected.cpu.instructions &&
              actual.cpu.cycles == expected.cpu.cycles,
          "Only native deltas advance the already current host accounting");
  require(actual.cpu.repeat_remaining == expected.cpu.repeat_remaining &&
              actual.cpu.repeat_pc == expected.cpu.repeat_pc &&
              actual.cpu.repeat_lc == expected.cpu.repeat_lc &&
              actual.cpu.repeat_opcode == expected.cpu.repeat_opcode &&
              actual.cpu.repeat_loaded == expected.cpu.repeat_loaded &&
              actual.cpu.repeat_completed == expected.cpu.repeat_completed,
          "An old payload cannot overwrite unwritten current REP fields");
  require(actual.cpu.interrupt_inhibited == expected.cpu.interrupt_inhibited &&
              actual.cpu.processing == expected.cpu.processing &&
              actual.cpu.debug_flags == expected.cpu.debug_flags,
          "Control flags change only under their recorded native ownership masks");
  require(actual.vector.resume == expected.vector.resume &&
              actual.vector.address == expected.vector.address &&
              actual.vector.level == expected.vector.level &&
              actual.vector.phase == expected.vector.phase &&
              actual.debug_vector == expected.debug_vector && actual.repeated == expected.repeated,
          "A phase-only update preserves later addresses, vector level and repeated instruction");
}

void merge(Live &live, const HiddenRetirement &result) {
  detail::merge_hidden_retirement(live.cpu, live.vector, live.debug_vector, live.repeated, result);
}

void unrelated_fields() {
  auto live = seed();
  auto expected = live;
  const HiddenRetirement result{
      .fields = retirement_bit(RetirementField::repeat_remaining) |
                retirement_bit(RetirementField::debug_flags) |
                retirement_bit(RetirementField::vector_phase),
      .debug_mask = detail::debug_interrupt_pending,
      .value = {.repeat_remaining = 3, .vector = {.phase = VectorPhase::second}, .repeated = {}},
      .delta = {.instructions = 1, .estimated_cycles = 2}};
  expected.cpu.repeat_remaining = 3;
  expected.cpu.debug_flags = detail::debug_interrupt_mode | detail::debug_software_occurrence;
  expected.vector.phase = VectorPhase::second;
  expected.cpu.instructions = instructions_after;
  expected.cpu.cycles = two_cycles_after;
  merge(live, result);
  expect(live, expected);
}

void empty_mask() {
  auto live = seed();
  auto expected = live;
  const HiddenRetirement result{
      .debug_mask = all_debug_bits,
      .value = {.processing = ProcessingState::debugging, .vector = {}, .repeated = {}},
      .delta = {.instructions = 1, .estimated_cycles = 3}};
  expected.cpu.instructions = instructions_after;
  expected.cpu.cycles = three_cycles_after;
  merge(live, result);
  expect(live, expected);
}

void native_counter_wrap() {
  auto live = seed();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  live.cpu.instructions = maximum;
  live.cpu.cycles = maximum - 1;
  auto expected = live;
  expected.cpu.instructions = 0;
  expected.cpu.cycles = 1;
  merge(live, {.value = {}, .delta = {.instructions = 1, .estimated_cycles = 3}});
  expect(live, expected);
}

void unknown_writer() {
  auto live = seed();
  const auto expected = live;
  bool refused = false;
  try {
    merge(live,
          {.fields = retirement_bit(RetirementField::count),
           .value = {},
           .delta = {.instructions = 1, .estimated_cycles = 3}});
  } catch (const std::logic_error &) { refused = true; }
  require(refused, "Unknown internal writer masks fail before modifying any live field");
  expect(live, expected);
}
}
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    unrelated_fields();
    empty_mask();
    native_counter_wrap();
    unknown_writer();
    std::cout << checks << " supporting sparse hidden merge assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
