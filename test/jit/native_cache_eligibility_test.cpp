// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "external_run_fixture.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
constexpr Word start = 0x100, memory_words = 1024, data_address = 0x80;
constexpr Word ce = 0x80000, datum = 0x9654a1, changed = 0x2579b3;
constexpr Word jump = 0x0c0100, immediate_x = 0x44f400, load_x = 0x44f000, store_x = 0x447000;
constexpr std::array empty_bounds{Word{0}, start, static_cast<Word>(address_words)};

class Bus final : public Io {
public:
  std::array<Word, memory_words> program{};
  std::array<Word, memory_words> data{};
  std::array<std::uint8_t, memory_words> known{};
  std::array<MemoryWindow, 1> windows{{{.words = data, .known = known, .writable = true}}};
  std::uint64_t revision = 0;
  unsigned program_reads = 0, data_reads = 0, writes = 0;
  bool yield = false, grant = true;
  TestCore *owner = nullptr;

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program.size() ? std::optional(program[address]) : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return space == Space::x && grant ? std::span<const MemoryWindow>(windows)
                                      : std::span<const MemoryWindow>{};
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (space == Space::program && address < program.size()) {
      ++program_reads;
      value = program[address];
      return true;
    }
    ++data_reads;
    if (space != Space::x || address >= data.size() || known[address] == 0) { return false; }
    value = data[address];
    if (yield && owner != nullptr) { owner->request_yield(); }
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    ++writes;
    if (space != Space::x || address >= data.size()) { return false; }
    data[address] = value;
    known[address] = 1;
    return true;
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

Registers initial() {
  auto result = dsp56300::test::initial_state();
  result.sr = ce;
  return result;
}

void arithmetic(Registers &state, const Word opcode) {
  const auto old = state;
  if (opcode == dsp56300::test::external_run::workloads.back().opcode) {
    dsp56300::test::bfu_model::apply(
        state, old, {.function = dsp56300::test::bfu_model::Function::bit_xor});
  } else {
    dsp56300::test::sequence_model::alu(state, old, opcode);
  }
}

struct DomainInput {
  Word bound = 0;
  unsigned instructions = 0;
};

void domain(const dsp56300::test::external_run::Workload workload, const DomainInput input) {
  const auto bound = input.bound;
  const auto instruction_budget = input.instructions;
  Bus bus;
  for (unsigned index = 0; index < 3; ++index) { bus.program[start + index] = workload.opcode; }
  bus.program[start + 3] = jump;
  TestCore cpu(bus, {.cacheable_begin = bound, .cacheable_end = bound});
  auto state = initial();
  state.x = workload.x;
  state.y = workload.y;
  cpu.set_registers(state);
  auto expected = state;
  unsigned retired = 0;
  unsigned cycles = 0;
  while (retired < instruction_budget) {
    const bool branch = expected.pc == start + 3;
    const auto cost = branch ? 3U : 1U;
    if (branch) {
      expected.pc = start;
    } else {
      arithmetic(expected, workload.opcode);
      ++expected.pc;
    }
    ++retired;
    cycles += cost;
  }
  const auto result = cpu.run(instruction_budget);
  require(result.retired == retired && cpu.registers() == expected &&
              cpu.progress() == Progress{.instructions = retired, .estimated_cycles = cycles} &&
              result.fault.kind == FaultKind::none,
          "CE retains independent full arithmetic state, flags, PC and exact budgets");
  require(bus.program_reads == 0,
          "An empty cache interval cannot disable native execution solely because CE is set");
}

void domains() {
  for (const auto workload : dsp56300::test::external_run::workloads) {
    for (const auto bound : empty_bounds) {
      for (unsigned instructions = 0; instructions <= 4; ++instructions) {
        domain(workload, {.bound = bound, .instructions = instructions});
      }
    }
  }
}

void native_memory(const Word bound) {
  Bus bus;
  bus.program[start] = load_x;
  bus.program[start + 1] = data_address;
  bus.program[start + 2] = store_x;
  bus.program[start + 3] = data_address + 1;
  bus.data[data_address] = datum;
  bus.known[data_address] = 1;
  TestCore cpu(bus, {.cacheable_begin = bound, .cacheable_end = bound});
  cpu.set_registers(initial());
  const auto result = cpu.run(2);
  auto expected = initial();
  expected.x = datum;
  expected.pc += 4;
  require(result.retired == 2 && cpu.registers() == expected &&
              cpu.progress() == Progress{.instructions = 2, .estimated_cycles = 4} &&
              bus.data[data_address + 1] == datum && bus.known[data_address + 1] == 1 &&
              bus.program_reads == 0 && bus.data_reads == 0 && bus.writes == 0,
          "CE with an empty interval retains genuine native RAM and matching knowledge bytes");
}

void empty_code_mutation(const Word bound) {
  Bus bus;
  bus.program[start] = immediate_x;
  bus.program[start + 1] = datum;
  TestCore cpu(bus, {.cacheable_begin = bound, .cacheable_end = bound});
  cpu.set_registers(initial());
  require(cpu.run(1).retired == 1 && cpu.registers().x == datum,
          "The first CE instruction reads its actual uncached immediate");
  bus.program[start + 1] = changed;
  ++bus.revision;
  cpu.set_registers(initial());
  require(cpu.run(1).retired == 1 && cpu.registers().x == changed && cpu.registers().sr == ce &&
              bus.program_reads == 0,
          "The empty cache interval cannot retain a stale immediate after real code mutation");
}

void active_cache() {
  Bus bus;
  bus.program[start] = immediate_x;
  bus.program[start + 1] = datum;
  TestCore cpu(bus, {.cacheable_begin = start, .cacheable_end = start + 16});
  cpu.set_registers(initial());
  require(cpu.run(1).retired == 1 && cpu.registers().x == datum && bus.program_reads > 0,
          "Real cache population still performs the actual owned program reads");
  const auto reads = bus.program_reads;
  bus.program[start + 1] = changed;
  ++bus.revision;
  cpu.set_registers(initial());
  require(
      cpu.run(1).retired == 1 && cpu.registers().x == datum && cpu.registers().sr == ce &&
          bus.program_reads == reads,
      "The actual nonempty cache preserves old instruction data and genuine cache-hit behavior");
}

void fallback_refusal() {
  Bus bus;
  bus.program[start] = load_x;
  bus.program[start + 1] = data_address;
  bus.data[data_address] = datum;
  bus.known[data_address] = 1;
  // Withhold X grants by using a different ungranted data bank's encoding.
  constexpr Word read_y = 0x4cf000;
  bus.program[start] = read_y;
  TestCore cpu(bus);
  cpu.set_registers(initial());
  const auto refused = cpu.run(1);
  require(refused.retired == 0 && refused.fault.kind == FaultKind::memory_read &&
              refused.fault.space == Space::y && refused.fault.address == data_address &&
              refused.fault.pc == start && bus.data_reads == 1,
          "Ungrantable native operands still reach the genuine precise read refusal");
}

void fallback_yield() {
  Bus bus;
  bus.program[start] = load_x;
  bus.program[start + 1] = data_address;
  bus.data[data_address] = datum;
  bus.known[data_address] = 1;
  bus.grant = false;
  bus.yield = true;
  TestCore cpu(bus);
  bus.owner = &cpu;
  cpu.set_registers(initial());
  const auto result = cpu.run(2);
  auto expected = initial();
  expected.x = datum;
  expected.pc += 2;
  require(result.retired == 1 && result.yielded && result.fault.kind == FaultKind::none &&
              cpu.registers() == expected && bus.data_reads == 1 && bus.program_reads == 0 &&
              cpu.progress() == Progress{.instructions = 1, .estimated_cycles = 2},
          "A genuine fallback callback can yield after its CE instruction without overrunning");
  ++expected.pc;
  const auto resumed = cpu.run(1);
  require(resumed.retired == 1 && !resumed.yielded && cpu.registers() == expected &&
              bus.data_reads == 1 &&
              cpu.progress() == Progress{.instructions = 2, .estimated_cycles = 3},
          "Resume executes the actual following NOP without replaying the completed read");
}
} // namespace

int main() {
  try {
    domains();
    for (const auto bound : empty_bounds) {
      native_memory(bound);
      empty_code_mutation(bound);
    }
    active_cache();
    fallback_refusal();
    fallback_yield();
    std::cout << dsp56300::test::checks << " cache eligibility checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "cache eligibility: " << error.what() << '\n';
    return 1;
  }
}
