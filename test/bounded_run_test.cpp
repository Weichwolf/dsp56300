// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "dsp56300/backend.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word add = 0x200040, bit_xor = 0x200043, complement = 0x200017;
constexpr Word immediate_x = 0x44f400, load_post_xor = 0x44d843, store_x = 0x447000;
constexpr Word cache_enable = 0x80000;
constexpr Word jump_zero = 0x0c0000, loop_trials = 24, loop_count = 1000;
constexpr Word jump = 0x0c0300, target = 0x300, datum_address = 0x80, datum = 0x123456;
constexpr std::array<Word, 5> code{add, immediate_x, datum, bit_xor, jump};
constexpr std::array<Word, 4> operations{add, immediate_x, bit_xor, jump};
constexpr std::array<Word, 4> costs{1, 2, 1, 3};
static_assert(CoreImplementation<BackendCore<Backend::automatic>>);

class Memory final : public Io {
public:
  std::array<Word, memory_words> program{};
  std::array<bool, memory_words> known{};
  Word data = datum;
  unsigned reads = 0, writes = 0;
  unsigned program_reads = 0, data_reads = 0;
  std::uint64_t revision = 0;
  bool accept = true;

  void load(const std::span<const Word> words) {
    for (std::size_t index = 0; index < words.size(); ++index) {
      program[start + index] = words[index];
      known[start + index] = true;
    }
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (address >= known.size() || !known[address]) { return std::nullopt; }
    return program[address];
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    ++reads;
    if (space == Space::program) {
      ++program_reads;
    } else {
      ++data_reads;
    }
    if (space == Space::x && address == datum_address && accept) {
      value = data;
      return true;
    }
    const auto word = inspect_program(address);
    if (space != Space::program || !word) { return false; }
    value = *word;
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    ++writes;
    if (space != Space::x || address != datum_address) { return false; }
    data = value;
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

void alu(Registers &state, const Word opcode) {
  const auto old = state;
  if (opcode == bit_xor || opcode == complement) {
    bfu_model::apply(state,
                     old,
                     {.function = opcode == bit_xor ? bfu_model::Function::bit_xor
                                                    : bfu_model::Function::complement});
    ++state.pc;
  } else if (opcode == immediate_x) {
    register_model::write(state, 4, datum);
    state.pc += 2;
  } else if (opcode == jump) {
    state.pc = target;
  } else {
    sequence_model::alu(state, old, opcode);
    ++state.pc;
  }
}

struct Expected {
  Registers state;
  Word count = 0, cycles = 0;
};

Expected prefix(const Registers &initial, const RunBudget budget) {
  Expected expected{.state = initial};
  for (std::size_t index = 0; index < operations.size(); ++index) {
    if (expected.count == budget.instructions || expected.cycles + costs[index] > budget.cycles) {
      break;
    }
    alu(expected.state, operations[index]);
    ++expected.count;
    expected.cycles += costs[index];
  }
  return expected;
}

void budget_case(Core &core, Memory &memory, const RunBudget budget) {
  auto initial = register_model::seed(0);
  initial.sr = flag_c | flag_l | flag_s;
  core.set_registers(initial);
  const auto before = core.progress();
  const auto expected = prefix(initial, budget);
  const auto result = core.run(budget);
  const auto remaining = budget.cycles - expected.cycles;
  auto stop = RunStop::cycle_budget;
  if (expected.count == budget.instructions) {
    stop = RunStop::instruction_budget;
  } else if (expected.count == operations.size() && remaining != 0) {
    stop = RunStop::unknown_code;
  }
  require(result.execution.dispatches == expected.count && result.cycles == expected.cycles &&
              result.stop == stop && result.pc == expected.state.pc &&
              core.registers() == expected.state && memory.reads == 0 && memory.writes == 0 &&
              result.execution.fault.kind == FaultKind::none && !result.execution.yielded &&
              result.execution.progress ==
                  Progress{.instructions = before.instructions + expected.count,
                           .estimated_cycles = before.estimated_cycles + expected.cycles},
          "Finite instruction/cycle horizons preserve literal full-state prefixes without I/O");
  const std::optional<Word> next =
      expected.count < operations.size() ? std::optional(costs[expected.count]) : std::nullopt;
  require(result.next_cycles == next,
          "A known over-budget instruction reports its real base cost without starting it");
}

void budgets(const Options options) {
  Memory memory;
  memory.load(code);
  Core core(memory, options);
  for (Word instructions = 0; instructions <= 7; ++instructions) {
    for (Word cycles = 0; cycles <= 10; ++cycles) {
      budget_case(core, memory, {.instructions = instructions, .cycles = cycles});
    }
  }
}

void effects(const Backend backend, const bool accept) {
  Memory memory;
  memory.accept = accept;
  const std::array program{add, load_post_xor, store_x, datum_address};
  memory.load(program);
  Core core(memory, {.backend = backend});
  auto expected = register_model::seed(0);
  expected.r[0] = datum_address;
  core.set_registers(expected);
  alu(expected, add);
  const auto first = core.run(RunBudget{.instructions = 10, .cycles = 10});
  require(first.execution.dispatches == 1 && first.cycles == 1 &&
              first.stop == RunStop::external_effect && first.next_cycles == 1 &&
              core.registers() == expected && memory.reads == 0,
          "Stop before the entire external load, including its postincrement, PC and parallel ALU");
  const auto repeated = core.run(RunBudget{.instructions = 10, .cycles = 10});
  require(repeated.execution.dispatches == 0 && repeated.cycles == 0 &&
              repeated.stop == RunStop::external_effect && core.registers() == expected &&
              memory.reads == 0 && memory.writes == 0,
          "Repeated boundary queries never consume or replay external effects");
  const auto old = expected;
  ++expected.r[0];
  ++expected.pc;
  if (accept) {
    register_model::write(expected, 4, datum);
    bfu_model::apply(expected, old, {.function = bfu_model::Function::bit_xor});
  }
  const auto owned = core.run(1);
  if (owned.dispatches != (accept ? 1U : 0U) || core.registers() != expected ||
      memory.data_reads != 1) {
    const auto actual = core.registers();
    std::cerr << "effect backend=" << backend_name(backend) << " accept=" << accept
              << " count=" << owned.dispatches << " reads=" << memory.reads
              << " fault=" << static_cast<unsigned>(owned.fault.kind) << std::hex
              << " pc=" << actual.pc << '/' << expected.pc << " r0=" << actual.r[0] << '/'
              << expected.r[0] << " a=" << actual.a << '/' << expected.a << " x=" << actual.x << '/'
              << expected.x << " sr=" << actual.sr << '/' << expected.sr << std::dec << '\n';
  }
  require(owned.dispatches == (accept ? 1U : 0U) && core.registers() == expected &&
              memory.data_reads == 1 &&
              memory.program_reads == (core.backend() == Backend::jit ? 0U : 1U),
          "The owner executes the actual load exactly once with old parallel inputs and refusal");
  if (!accept) {
    const auto failed = core.run(RunBudget{.instructions = 10, .cycles = 10});
    require(failed.stop == RunStop::fault && failed.execution.dispatches == 0 &&
                memory.data_reads == 1 &&
                memory.program_reads == (core.backend() == Backend::jit ? 0U : 1U),
            "A refused owned access remains faulted and cannot be fabricated or retried");
    return;
  }
  const auto short_budget = core.run(RunBudget{.instructions = 10, .cycles = 1});
  const auto store = core.run(RunBudget{.instructions = 10, .cycles = 2});
  require(short_budget.stop == RunStop::cycle_budget && short_budget.next_cycles == 2 &&
              store.stop == RunStop::external_effect && store.next_cycles == 2 &&
              store.execution.dispatches == 0 && core.registers() == expected && memory.writes == 0,
          "The two-cycle actual store cannot cross a one-cycle deadline or commit at inspection");
  if (!store.next) {
    throw std::logic_error("A known store boundary lost its instruction snapshot");
  }
  require(store.next->address == start + 2 && store.next->opcode == store_x &&
              store.next->extension == datum_address,
          "The boundary returns genuine opcode/extension words for a stable in-flight reservation");
  auto captured = core.begin_instruction(*store.next);
  require(captured.pending && !captured.retired && memory.writes == 0,
          "Latching the pending instruction performs no future store");
  memory.program[start + 2] = jump;
  ++memory.revision;
  constexpr unsigned maximum_effects = 16;
  for (unsigned index = 0; captured.pending && index < maximum_effects; ++index) {
    captured = core.advance_instruction(captured.pending->ticket);
  }
  expected.pc += 2;
  require(captured.retired && captured.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.writes == 1 && memory.data == datum,
          "The real captured store retires once with original code despite a later P change");
}

void loops(const Options options) {
  Memory memory;
  constexpr std::array program{immediate_x, datum, add, complement};
  memory.load(program);
  Core core(memory, options);
  for (Word instructions = 1; instructions <= loop_trials; ++instructions) {
    for (Word cycles = 1; cycles <= 32; ++cycles) {
      auto initial = register_model::seed(0);
      initial.sr = 0x8000 | flag_c;
      initial.la = start + 3;
      initial.lc = loop_count;
      initial.sp = initial.sc = 2;
      initial.stack[2] = std::uint64_t{start} << word_bits;
      core.set_registers(initial);
      auto expected = initial;
      Word retired = 0;
      Word consumed = 0;
      constexpr std::array body{immediate_x, add, complement};
      constexpr std::array<Word, 3> base_costs{2, 1, 1};
      while (retired < instructions && consumed + base_costs[retired % 3] <= cycles) {
        alu(expected, body[retired % 3]);
        consumed += base_costs[retired % 3];
        ++retired;
        if (retired % 3 == 0) {
          --expected.lc;
          expected.pc = start;
        }
      }
      const auto result = core.run(RunBudget{.instructions = instructions, .cycles = cycles});
      require(result.execution.dispatches == retired && result.cycles == consumed &&
                  result.execution.fault.kind == FaultKind::none && core.registers() == expected &&
                  memory.reads == 0 && memory.writes == 0,
              "Native DO backedges and shorter cached prefixes honor both mixed-cost bounds");
    }
  }
}

void unknown_and_control() {
  Memory missing;
  Core unknown(missing);
  auto state = register_model::seed(0);
  unknown.set_registers(state);
  const auto stopped = unknown.run(RunBudget{.instructions = 1, .cycles = 10});
  require(stopped.stop == RunStop::unknown_code && stopped.execution.dispatches == 0 &&
              stopped.pc == start && !stopped.next && !stopped.next_cycles && missing.reads == 0 &&
              unknown.registers() == state,
          "Unknown code returns an explicit pure boundary with no invented zero instruction");
  const auto actual = unknown.run(1);
  require(actual.dispatches == 0 && missing.reads == 1 &&
              actual.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::program,
                                    .address = start,
                                    .pc = start},
          "The genuine owner fetch still fails with actual operation, address and PC");
  Memory memory;
  constexpr std::array<Word, 1> nop{0};
  memory.load(nop);
  for (const bool cache : {false, true}) {
    Core core(memory);
    state.sr = cache ? cache_enable : 0;
    core.set_registers(state);
    if (!cache) {
      core.set_debug_interrupt_mode(true);
      core.request_debug();
    }
    const auto result = core.run(RunBudget{.instructions = 10, .cycles = 10});
    if (cache) {
      auto expected = state;
      ++expected.pc;
      require(result.stop == RunStop::unknown_code && result.execution.dispatches == 1 &&
                  result.cycles == 1 && core.registers() == expected && memory.reads == 0 &&
                  core.jit_statistics().native_alu_instructions == 1 &&
                  core.jit_statistics().helper_instructions == 0,
              "Raw CE with the default empty cache interval retains actual native execution");
    } else {
      require(result.stop == RunStop::control_event && result.execution.dispatches == 0 &&
                  result.cycles == 0 && core.registers() == state && memory.reads == 0,
              "Debug admission stays before actual control or bus effects");
    }
  }
}

void previews() {
  Memory memory;
  constexpr Word repeat_two = 0x0602a0;
  constexpr std::array program{repeat_two, Word{0}, Word{0}};
  memory.load(program);
  Core core(memory);
  const auto initial = register_model::seed(0);
  core.set_registers(initial);
  const auto zero = core.run(RunBudget{});
  require(zero.execution.dispatches == 0 && zero.cycles == 0 && zero.pc == start && zero.next &&
              zero.next->opcode == repeat_two && zero.next_cycles && core.registers() == initial &&
              memory.reads == 0,
          "A zero-budget preview exposes known cost and words without execution or bus traffic");
  require(core.run(2).dispatches == 2 && core.registers().pc == start + 1,
          "A genuine REP setup and first body iteration latch the original opcode");
  const auto state = core.registers();
  const auto before = core.progress();
  const auto reads = memory.reads;
  memory.program[start + 1] = store_x;
  ++memory.revision;
  const auto latched = core.run(RunBudget{});
  require(latched.pc == start + 1 && latched.next && latched.next->opcode == 0 &&
              !latched.next->extension && latched.next_cycles == 1 && core.registers() == state &&
              core.progress() == before && memory.reads == reads,
          "REP preview uses the actual retained opcode rather than newer backing program bytes");
}

void vector_previews() {
  constexpr Word vector = 0x54;
  Memory memory;
  memory.program[vector] = 0;
  memory.program[vector + 1] = 0;
  memory.known[vector] = memory.known[vector + 1] = true;
  Core core(memory);
  const auto state = register_model::seed(0);
  core.set_registers(state);
  core.request_interrupt(vector, 0, {.address = start, .opcode = 0});
  const auto before = core.run(RunBudget{});
  require(before.pc == vector && before.next && before.next->opcode == 0 &&
              before.next_cycles == 1 && core.registers() == state && memory.reads == 0 &&
              core.progress() == Progress{},
          "Pending-vector metadata preview neither performs vector reads nor acknowledges an IRQ");
  auto actual = core.begin_fetch();
  constexpr unsigned maximum_effects = 16;
  for (unsigned index = 0; actual.pending && index < maximum_effects; ++index) {
    actual = core.advance_instruction(actual.pending->ticket);
  }
  require(actual.retired && memory.reads == 2 && core.progress().instructions == 1,
          "Both genuine vector reads precede actual first-vector retirement");
  memory.program[vector + 1] = store_x;
  ++memory.revision;
  const auto progress = core.progress();
  const auto second = core.run(RunBudget{});
  require(second.pc == vector + 1 && second.next && second.next->opcode == 0 &&
              !second.next->extension && second.next_cycles == 1 && memory.reads == 2 &&
              core.progress() == progress,
          "Second-vector preview uses its genuinely accepted pair latch after backing P changes");
}

void short_jumps() {
  Memory memory;
  constexpr Word source = start;
  for (const Word address : {0U, 0xfffU}) {
    memory.program[source] = jump_zero | address;
    memory.known[source] = true;
    Core core(memory);
    for (Word flags = 0; flags <= status_byte; ++flags) {
      auto expected = register_model::seed(0x2000 | 0x4000 | 0x20000 | 0x100000);
      expected.sr = 0x2000 | 0x4000 | 0x20000 | 0x100000 | flags;
      core.set_registers(expected);
      const auto before = core.jit_statistics();
      const auto result = core.run(1);
      expected.pc = address;
      const auto after = core.jit_statistics();
      require(
          result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.reads == 0 &&
              after.native_alu_instructions == before.native_alu_instructions + 1 &&
              after.helper_instructions == before.helper_instructions,
          "Short JMP executes native PC/cycle retirement and preserves all numerical modes/CCR");
    }
  }
}
}
}

int main() {
  try {
    for (const auto backend : {dsp56300::Backend::automatic,
                               dsp56300::Backend::jit,
                               dsp56300::Backend::cached,
                               dsp56300::Backend::reference}) {
      for (const bool optimize : {false, true}) {
        for (const dsp56300::Word limit : {1U, 64U}) {
          const dsp56300::Options options{
              .backend = backend, .instructions_per_block = limit, .optimize = optimize};
          dsp56300::test::budgets(options);
          dsp56300::test::loops(options);
        }
      }
      dsp56300::test::effects(backend, true);
      dsp56300::test::effects(backend, false);
    }
    dsp56300::test::short_jumps();
    dsp56300::test::unknown_and_control();
    dsp56300::test::previews();
    dsp56300::test::vector_previews();
    std::cout << dsp56300::test::checks << " bounded native prefix/effect checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
