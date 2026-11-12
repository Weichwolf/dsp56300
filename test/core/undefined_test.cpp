// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string_view>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word start = 0x200, handler = 0x400, relocated = 0x800;
constexpr Word repeat_start = start + 32, instruction_stride = 4, priority_mask = 0x300;
// Independently selected members of documented unallocated encoding domains.
// These are not manufacturer program words or substitutes for every unsupported opcode.
constexpr std::array reserved{
    Word{0x000007}, Word{0x00325a}, Word{0x000100}, Word{0x0001ff}, Word{0x0d2000}, Word{0x0d2fff}};
constexpr std::array fast_vector{op<isa::Opcode::nop>, op<isa::Opcode::nop>};
constexpr std::array long_vector{op<isa::Opcode::jsr_effective> | (6U << 11), handler};
constexpr std::array service{op<isa::Opcode::rti>};
constexpr std::array repeat{op<isa::Opcode::rep_immediate> | (3U << 8),
                            reserved.front(),
                            op<isa::Opcode::nop>,
                            op<isa::Opcode::stop>};
constexpr auto bodies = [] {
  std::array<std::array<Word, 3>, reserved.size()> result{};
  for (std::size_t index = 0; index < reserved.size(); ++index) {
    result[index] = {reserved[index], op<isa::Opcode::nop>, op<isa::Opcode::stop>};
  }
  return result;
}();

std::array<CodeSpan, reserved.size() + 4> streams(const bool long_service) {
  const auto vector = long_service ? std::span(long_vector) : std::span(fast_vector);
  std::array<CodeSpan, reserved.size() + 4> result{};
  for (std::size_t index = 0; index < bodies.size(); ++index) {
    result[index] = {.address = start + (static_cast<Word>(index) * instruction_stride),
                     .words = bodies[index]};
  }
  result[reserved.size()] = {.address = repeat_start, .words = repeat};
  result[reserved.size() + 1] = {.address = handler, .words = service};
  result[reserved.size() + 2] = {.address = 4, .words = vector};
  result[reserved.size() + 3] = {.address = relocated + 4, .words = vector};
  return result;
}

const NativeImage &image() {
  static const NativeImage result = [] {
    const auto fast = streams(false);
    const auto slow = streams(true);
    std::array<Word, reserved.size() + 2> entries{};
    for (std::size_t index = 0; index < reserved.size(); ++index) {
      entries[index] = start + (static_cast<Word>(index) * instruction_stride);
    }
    entries[reserved.size()] = handler;
    entries[reserved.size() + 1] = repeat_start;
    const std::array variants{
        CodeVariant{.id = 0, .code = fast, .entries = entries, .failed_fetches = {}},
        CodeVariant{.id = 1, .code = slow, .entries = entries, .failed_fetches = {}}};
    constexpr std::array interrupts{InterruptRequest{.vector = 4, .level = 3},
                                    InterruptRequest{.vector = relocated + 4, .level = 3}};
    return Core::compile({.variants = variants},
                         {.memory = {},
                          .interrupts = interrupts,
                          .transitions = {},
                          .callbacks = {},
                          .bindings = {}});
  }();
  return result;
}

struct Machine {
  Machine(const bool prepared, const bool long_service)
      : core(Memory::separate({.program = program, .x = x, .y = y})) {
    for (const auto span : streams(long_service)) {
      std::ranges::copy(span.words, program.begin() + span.address);
    }
    if (prepared) { core.install(image(), long_service ? 1 : 0); }
  }

  void expect(const Registers &expected, const Word count) {
    const auto result = core.run(count);
    require(result.retired == count && result.fault == Fault{} && core.registers() == expected,
            "Undefined encoding preserves every exposed register, frame and exact continuation");
  }

  std::array<Word, 4096> program{};
  std::array<Word, 8> x{}, y{};
  Core core;
};

struct Initial {
  Word pc, vba, status;
};

Registers seed(const Initial input) {
  auto result = test::register_model::seed(0);
  result.pc = input.pc;
  result.vba = input.vba;
  result.sr = input.status;
  return result;
}

void vector_return(Machine &machine, Registers expected, const bool long_service) {
  machine.expect(expected, 1);
  if (long_service) {
    const auto status = expected.sr;
    const auto resume = expected.pc;
    ++expected.sp;
    ++expected.sc;
    constexpr unsigned bits = 24;
    expected.stack[expected.sp % stack_entries] = (std::uint64_t{resume} << bits) | status;
    constexpr Word cleared = (1U << 10) | (1U << 11) | (1U << 17) | (1U << 15) | (1U << 16) | 0x300;
    expected.sr = (status & ~cleared) | priority_mask;
    expected.pc = handler;
    machine.expect(expected, 1);
    --expected.sp;
    --expected.sc;
    expected.sr = status;
    expected.pc = resume;
    machine.expect(expected, 1);
  } else {
    machine.expect(expected, 2);
  }
  ++expected.pc;
  machine.expect(expected, 1);
  ++expected.pc;
  machine.expect(expected, 1);
  require(machine.core.state().execution == ExecutionState::stopped,
          "Execution resumes at the actual following word and reaches STOP");
}

void returns() {
  for (const bool prepared : {true, false}) {
    for (const bool long_service : {false, true}) {
      Machine machine(prepared, long_service);
      for (const Word vba : {0U, relocated}) {
        for (const Word status : {0xd5U, 0x3d5U, 0x7d5U, 0xbd5U}) {
          for (const Word depth : {0U, 2U}) {
            for (std::size_t index = 0; index < reserved.size(); ++index) {
              machine.core.reset();
              auto initial = seed({.pc = start + (static_cast<Word>(index) * instruction_stride),
                                   .vba = vba,
                                   .status = status});
              initial.sp = depth;
              initial.sc = depth;
              machine.core.set_registers(initial);
              ++initial.pc;
              vector_return(machine, initial, long_service);
            }
          }
        }
      }
    }
  }
}

void unsupported() {
  constexpr Word unaudited = 0x0d3000;
  Machine machine(false, false);
  machine.program[start] = unaudited;
  machine.core.set_registers(seed({.pc = start, .vba = 0, .status = priority_mask}));
  const auto result = machine.core.run(1);
  require(result.retired == 0 && result.fault == Fault{.kind = FaultKind::unsupported_instruction,
                                                       .address = start,
                                                       .value = unaudited,
                                                       .pc = start},
          "An unaudited encoding still refuses execution with its actual PC and readable word");
  constexpr std::array words{unaudited, op<isa::Opcode::nop>};
  const std::array spans{CodeSpan{.address = start, .words = words}};
  constexpr std::array entries{start};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  bool refused = false;
  try {
    static_cast<void>(Core::compile({.variants = variants}, {}));
  } catch (const std::invalid_argument &error) {
    refused = std::string_view(error.what()).contains("unsupported instruction encoding");
  }
  require(refused, "An unsupported word cannot become a prepared III or silent late translation");
}

void repeated() {
  for (const bool prepared : {true, false}) {
    Machine machine(prepared, false);
    auto initial = seed({.pc = repeat_start, .vba = relocated, .status = priority_mask});
    machine.core.set_registers(initial);
    const auto count = machine.core.run(4);
    initial.pc = repeat_start + 2;
    require(count.retired == 4 && count.fault == Fault{} && machine.core.registers() == initial &&
                machine.core.progress().instructions == 4,
            "REP retires all three undefined attempts before its single pending III");
    machine.expect(initial, 2);
    ++initial.pc;
    machine.expect(initial, 1);
    ++initial.pc;
    machine.expect(initial, 1);
    require(machine.core.state().execution == ExecutionState::stopped,
            "The pending undefined exception returns once after the completed REP");
  }
}
}

int main() try {
  returns();
  repeated();
  unsupported();
  std::cout << test::checks << " undefined-instruction checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
