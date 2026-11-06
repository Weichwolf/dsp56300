// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word origin = 0xff0000;
constexpr unsigned count = 32;
constexpr Word datum_address = 0x80;
constexpr Word datum = 0x123456;
constexpr Word jump = 0x0c0300;
constexpr Word target = 0x300;
constexpr Word load_x = 0x44f000;
constexpr Word store_x = 0x447000;
constexpr Word vector = 0x54;
constexpr Word call = 0x0d0300;
constexpr Word rti = 4;
constexpr Word move_omr = 0x05f43a;

struct Access {
  Space space;
  Word address, pc, value = 0;
};

class Memory final : public Io {
public:
  std::array<Word, count> program{};
  std::array<Word, 2> interrupt_words{};
  unsigned reads = 0;
  unsigned writes = 0;
  Word data = datum;
  Word read_pc = 0;
  Word write_pc = 0;
  std::uint64_t revision = 0;
  bool inspect = true;
  bool wrapping = false;
  std::optional<Word> refused_program;
  bool known = true;
  bool writable = true;
  bool mutate_code_on_read = false;
  bool yield_on_read = false;
  TestCore *cpu = nullptr;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    return load({.space = space, .address = address, .pc = pc}, value);
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    return store({.space = space, .address = address, .pc = pc, .value = value});
  }

  bool load(const Access access, Word &value) noexcept {
    ++reads;
    if (access.space == Space::program && refused_program == access.address) { return false; }
    if (wrapping && access.space == Space::program) {
      const auto inspected = inspect_program(access.address);
      if (!inspected) { return false; }
      value = *inspected;
      return true;
    }
    if (access.space == Space::program && access.address == target) {
      value = rti;
      return true;
    }
    if (access.space == Space::program && access.address >= vector && access.address < vector + 2) {
      value = interrupt_words[access.address - vector];
      return true;
    }
    if (access.space == Space::x && access.address == datum_address) {
      read_pc = access.pc;
      if (!known) { return false; }
      value = data;
      if (yield_on_read && cpu != nullptr) { cpu->request_yield(); }
      if (mutate_code_on_read) {
        program[2] = jump;
        ++revision;
      }
      return true;
    }
    if (access.space != Space::program || access.address < origin ||
        access.address - origin >= program.size()) {
      return false;
    }
    value = program[access.address - origin];
    return true;
  }

  bool store(const Access access) noexcept {
    if (access.space != Space::x || access.address != datum_address) { return false; }
    ++writes;
    write_pc = access.pc;
    if (!writable) { return false; }
    data = access.value;
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

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (wrapping && address == word_mask) { return load_x; }
    if (wrapping && address == 0) { return datum_address; }
    if (inspect && address == target) { return rti; }
    if (!inspect || address < origin || address - origin >= program.size()) { return std::nullopt; }
    return program[address - origin];
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }
};

Registers seed() {
  auto state = initial_state();
  state.pc = origin;
  return state;
}

void external_default(const bool inspect) {
  Memory memory;
  memory.inspect = inspect;
  TestCore core(memory);
  const auto before = seed();
  core.set_registers(before);
  auto expected = before;
  expected.pc += count;
  const auto published = codegen::publications();
  const auto result = core.run(count);
  require(result.retired == count && result.fault.kind == FaultKind::none &&
              core.registers() == expected &&
              result.progress == Progress{.instructions = count, .estimated_cycles = count},
          "Default sparse external memory retains literal complete NOP state and costs");
  require(
      (inspect ? codegen::publications().count > published.count
               : codegen::publications() == published) &&
          memory.reads == (inspect ? 0U : count),
      "The default actually enters native code with a stable view, otherwise preserves fetches");
}

void host_code_mutation() {
  Memory memory;
  TestCore core(memory);
  const auto before = seed();
  core.set_registers(before);
  require(core.run(count).retired == count, "Initial stable external code must run natively");
  const auto blocks = codegen::publications().count;
  memory.program[0] = jump; // Literal short JMP $300, three base cycles.
  ++memory.revision;
  core.set_registers(before);
  const auto result = core.run(1);
  auto expected = before;
  expected.pc = target;
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              core.registers() == expected && codegen::publications().count > blocks &&
              result.progress == Progress{.instructions = count + 1, .estimated_cycles = count + 3},
          "A genuine owner revision invalidates code; original stored bytes determine the jump");
}

void unknown_program() {
  Memory memory;
  TestCore core(memory);
  auto before = seed();
  before.pc = origin + count;
  core.set_registers(before);
  const auto published = codegen::publications();
  const auto result = core.run(1);
  require(result.retired == 0 && core.registers() == before &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::program,
                                    .address = before.pc,
                                    .pc = before.pc} &&
              memory.reads == 1 && codegen::publications() == published,
          "An unknown code view performs the actual refused fetch with precise context");
}

void missing_extension() {
  Memory memory;
  memory.program.back() = load_x;
  TestCore core(memory);
  auto expected = seed();
  expected.pc = origin + count - 1;
  core.set_registers(expected);
  const auto result = core.run(1);
  require(result.retired == 0 && result.progress == Progress{} && core.registers() == expected &&
              memory.reads == 2 &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::program,
                                    .address = origin + count,
                                    .pc = expected.pc},
          "Unknown extension inspection falls back to the two actual fetches with precise refusal");
  require(core.run(1).retired == 0 && memory.reads == 2,
          "A failed extension is fail-stop and never replays accepted fetches");
}

void wrapping_extension() {
  Memory memory;
  memory.wrapping = true;
  TestCore core(memory);
  auto expected = seed();
  expected.pc = word_mask;
  core.set_registers(expected);
  const auto result = core.run(1);
  expected.pc = 1;
  expected.x = datum;
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = 2} &&
              core.registers() == expected && memory.reads == 1 && memory.read_pc == word_mask,
          "A two-word native instruction fetches its extension across the 24-bit P wrap");
}

void refused_vectors() {
  for (const Word index : {0U, 1U}) {
    Memory memory;
    memory.refused_program = vector + index;
    TestCore core(memory);
    const auto expected = seed();
    core.set_registers(expected);
    core.interrupt(vector, 1);
    require(core.registers() == expected && core.progress() == Progress{} &&
                memory.reads == index + 1 &&
                core.fault() == Fault{.kind = FaultKind::memory_read,
                                      .space = Space::program,
                                      .address = vector + index,
                                      .pc = vector},
            "Both actual vector reads must succeed before any first-vector instruction retires");
    require(core.run(1).retired == 0 && memory.reads == index + 1,
            "Refused vector reads retain fail-stop state without replay or invented IRQ success");
  }
}

void repetition_boundary() {
  constexpr Word repeat_two = 0x0602a0;
  for (const bool next_repeat : {false, true}) {
    Memory memory;
    memory.program[0] = repeat_two;
    memory.program[2] = next_repeat ? repeat_two : 0;
    TestCore core(memory);
    auto expected = seed();
    core.set_registers(expected);
    const auto result = core.run(3);
    expected.pc += 2;
    require(result.retired == 3 && result.fault.kind == FaultKind::none &&
                core.registers() == expected && core.interrupt_ready() != next_repeat,
            "Known external post-REP metadata admits IRQs except before another real REP");
    const auto reads = memory.reads;
    memory.inspect = false;
    bool rejected = false;
    try {
      static_cast<void>(core.interrupt_ready());
    } catch (const std::logic_error &) { rejected = true; }
    require(rejected && memory.reads == reads,
            "Unknown post-REP metadata rejects eligibility without a side-effecting P fetch");
  }
}

void ordinary_buffers() {
  constexpr unsigned words = 512;
  std::array<Word, words> program{};
  std::array<Word, words> x{};
  std::array<Word, words> y{};
  TestCore core({.program = program, .x = x, .y = y});
  const auto before = initial_state();
  core.set_registers(before);
  auto expected = before;
  expected.pc += count;
  const auto result = core.run(count);
  require(result.retired == count && result.fault.kind == FaultKind::none &&
              core.registers() == expected,
          "Ordinary memory construction selects actual native execution without backend options");
}

void data_callbacks(const bool known, const bool writable) {
  Memory memory;
  memory.known = known;
  memory.writable = writable;
  // Literal absolute-long X load and store, two words and two cycles each.
  memory.program[0] = load_x;
  memory.program[1] = datum_address;
  memory.program[2] = store_x;
  memory.program[3] = datum_address;
  TestCore core(memory);
  auto expected = seed();
  constexpr std::uint64_t previous_x = 0x456789000007;
  expected.x = previous_x;
  core.set_registers(expected);
  const auto result = core.run(2);
  expected.pc += known ? 4U : 2U;
  if (known) { expected.x = (previous_x & (long_mask ^ word_mask)) | datum; }
  std::uint64_t retired = known ? 1U : 0U;
  if (known && writable) { retired = 2; }
  Fault failure;
  if (!known) {
    failure = {
        .kind = FaultKind::memory_read, .space = Space::x, .address = datum_address, .pc = origin};
  } else if (!writable) {
    failure = {.kind = FaultKind::memory_write,
               .space = Space::x,
               .address = datum_address,
               .value = datum,
               .pc = origin + 2};
  }
  require(
      result.retired == retired && result.fault == failure &&
          result.progress == Progress{.instructions = retired, .estimated_cycles = retired * 2} &&
          core.registers() == expected && memory.reads == 1 && memory.read_pc == origin &&
          memory.writes == (known ? 1U : 0U) && (!known || memory.write_pc == origin + 2) &&
          memory.data == datum,
      "Native external memory performs real callbacks once and retains literal refusal prefixes");
}

void callback_code_mutation() {
  Memory memory;
  memory.program[0] = load_x;
  memory.program[1] = datum_address;
  memory.mutate_code_on_read = true;
  TestCore core(memory);
  auto expected = seed();
  core.set_registers(expected);
  const auto published = codegen::publications();
  const auto result = core.run(2);
  expected.pc = target;
  expected.x = datum;
  require(result.retired == 2 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 2, .estimated_cycles = 5} &&
              core.registers() == expected && memory.reads == 1 &&
              codegen::publications().count > published.count + 1,
          "Synchronous Io code mutation exits the old native block before its stale successor");
}

void cooperative_yield(const Backend backend) {
  Memory memory;
  memory.program[0] = load_x;
  memory.program[1] = datum_address;
  memory.program[2] = store_x;
  memory.program[3] = datum_address;
  memory.yield_on_read = true;
  TestCore core(memory, {.backend = backend});
  memory.cpu = &core;
  const auto before = seed();
  core.set_registers(before);
  auto expected = before;
  expected.pc += 2;
  expected.x = datum;
  const auto first = core.run(2);
  require(first.retired == 1 && first.yielded && first.fault.kind == FaultKind::none &&
              first.state == ExecutionState::running && core.registers() == expected &&
              first.progress == Progress{.instructions = 1, .estimated_cycles = 2} &&
              memory.writes == 0,
          "A real read callback yields only after retiring its load, before the future store");
  const auto second = core.run(1);
  expected.pc += 2;
  require(second.retired == 1 && !second.yielded && second.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.writes == 1 &&
              second.progress == Progress{.instructions = 2, .estimated_cycles = 4},
          "The ordinary next run resumes at the actual successor without replaying the read");
  const auto compiled = codegen::publications().count;
  core.set_registers(before);
  const auto repeated = core.run(2);
  require(repeated.retired == 1 && repeated.yielded && codegen::publications().count == compiled &&
              memory.writes == 1 &&
              repeated.progress == Progress{.instructions = 3, .estimated_cycles = 6},
          "Ordinary event yields preserve compiled code and independently counted retirement");
}

void external_interrupt(const bool long_entry) {
  Memory memory;
  if (long_entry) { memory.interrupt_words[0] = call; }
  TestCore core(memory);
  auto expected = seed();
  core.set_registers(expected);
  require(core.run(2).retired == 2, "Default native foreground runs before a genuine IRQ");
  expected.pc += 2;
  require(core.interrupt_ready(), "The normal external-memory boundary admits an actual IRQ");
  core.interrupt(vector, 1);
  if (long_entry) {
    expected.sp = expected.sc = 1;
    expected.stack[1] = std::uint64_t{expected.pc} << word_bits;
    expected.pc = target;
    expected.sr = 0x200;
  }
  const auto instructions = long_entry ? 3U : 4U;
  const auto cycles = long_entry ? 5U : 4U;
  require(core.fault().kind == FaultKind::none && core.registers() == expected &&
              core.progress() ==
                  Progress{.instructions = instructions, .estimated_cycles = cycles} &&
              memory.reads == 2,
          "External IRQ entry reads both real vector words and retains literal fast/long state");
  if (!long_entry) { return; }
  const auto returned = core.run(1);
  expected.pc = origin + 2;
  expected.sr = expected.sp = expected.sc = 0;
  require(returned.retired == 1 && returned.fault.kind == FaultKind::none &&
              core.registers() == expected && !core.interrupt_ready() &&
              core.progress() == Progress{.instructions = 4, .estimated_cycles = 8} &&
              memory.reads == 2,
          "A genuine RTI restores the original frame and one-instruction interrupt inhibition");
}

void memory_context_boundary() {
  Memory memory;
  memory.program[0] = move_omr;
  memory.program[1] = 1;
  TestCore core(memory);
  auto expected = seed();
  core.set_registers(expected);
  const auto changed = core.run(2);
  expected.pc += 2;
  expected.omr = 1;
  require(changed.retired == 1 && changed.yielded && changed.fault.kind == FaultKind::none &&
              core.registers() == expected &&
              changed.progress == Progress{.instructions = 1, .estimated_cycles = 2},
          "Actual OMR programming returns a native boundary for the owner's memory-map update");
  const auto continued = core.run(1);
  ++expected.pc;
  require(continued.retired == 1 && !continued.yielded && continued.fault.kind == FaultKind::none &&
              core.registers() == expected &&
              continued.progress == Progress{.instructions = 2, .estimated_cycles = 3},
          "The next ordinary native run continues under the real new mapping context");
}
}
}

int main() {
  try {
    dsp56300::test::external_default(true);
    dsp56300::test::external_default(false);
    dsp56300::test::host_code_mutation();
    dsp56300::test::unknown_program();
    dsp56300::test::ordinary_buffers();
    dsp56300::test::missing_extension();
    dsp56300::test::wrapping_extension();
    dsp56300::test::refused_vectors();
    dsp56300::test::repetition_boundary();
    dsp56300::test::data_callbacks(true, true);
    dsp56300::test::data_callbacks(false, true);
    dsp56300::test::data_callbacks(true, false);
    dsp56300::test::callback_code_mutation();
    dsp56300::test::cooperative_yield(dsp56300::Backend::jit);
    dsp56300::test::cooperative_yield(dsp56300::Backend::jit);
    dsp56300::test::external_interrupt(false);
    dsp56300::test::external_interrupt(true);
    dsp56300::test::memory_context_boundary();
    std::cout << dsp56300::test::checks << " automatic native external-memory checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
