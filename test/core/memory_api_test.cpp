// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "memory_arena.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {
using namespace dsp56300;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

void default_run() {
  constexpr std::array<Word, 1> expected_nop{0};
  constexpr std::array<Word, 1> expected_stop{0x87};
  static_assert(isa::nop == expected_nop && isa::stop == expected_stop);
  constexpr auto program = isa::program(isa::nop, isa::stop);
  constexpr std::size_t memory_words = 64;
  std::array<Word, memory_words> backing{};
  std::ranges::copy(program, backing.begin());
  const auto expected_memory = backing;
  Core core(Memory::unified(backing));
  auto expected = core.registers();
  expected.pc = program.size();
  const auto result = core.run();
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
              result.state == ProcessingState::stopped && core.registers() == expected &&
              backing == expected_memory && core.backend() == Backend::jit,
          "Default native run stops at guest STOP with exact PC, registers and RAM");
}

void natural_translation() {
  constexpr std::size_t nop_count = 513;
  std::array<Word, nop_count + 1> backing{};
  backing.back() = isa::stop[0];
  const auto expected_memory = backing;
  Core core(Memory::unified(backing));
  auto expected = core.registers();
  expected.pc = backing.size();
  const auto result = core.run();
  const auto statistics = core.jit_statistics();
  require(result.dispatches == backing.size() && result.fault.kind == FaultKind::none &&
              result.state == ProcessingState::stopped && core.registers() == expected &&
              backing == expected_memory && statistics.compiled_blocks == 1 &&
              statistics.native_alu_instructions >= nop_count,
          "Straight-line code beyond 256 instructions reaches STOP in one native translation");
}

void partition() {
  constexpr std::size_t region_words = 64;
  std::array<Word, region_words * 3> backing{};
  const auto memory = Memory::partition(
      backing, {.program_words = region_words, .x_words = region_words, .y_words = region_words});
  const auto views = memory.storage();
  require(!memory.externally_owned() && views.program.data() == backing.data() &&
              views.x.data() == backing.data() + region_words &&
              views.y.data() == backing.data() + region_words * 2,
          "A single host allocation has disjoint explicit Harvard views");
  Core core(memory, nullptr, {});
  require(core.backend() == Backend::jit,
          "The ordinary API selects the native backend on the supported host");
  const auto result = core.run(1);
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              core.registers().pc == 1,
          "The new Core/Memory entry runs the actual native NOP");
  constexpr Word data = 0x123456;
  core.write(Space::x, 0, data);
  require(views.x[0] == data && views.program[0] == 0 && views.y[0] == 0,
          "Disjoint physical regions retain independent contents");
}

void unified() {
  constexpr Word entry = 16;
  constexpr Word clear_a = 0x200013;
  constexpr Word negate_a = 0x200036;
  std::array<Word, 64> backing{};
  const auto memory = Memory::unified(backing);
  const auto views = memory.storage();
  require(views.program.data() == views.x.data() && views.x.data() == views.y.data(),
          "Unified P/X/Y are explicitly aliased, without data copies");
  Core core(memory, nullptr, {});
  core.write(Space::program, entry, clear_a);
  auto state = core.registers();
  state.pc = entry;
  state.a = 3;
  core.set_registers(state);
  auto result = core.run(1);
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none && core.registers().a == 0,
          "Aliased backing executes a real program instruction");
  const auto invalidations = core.jit_statistics().invalidations;
  core.write(Space::x, entry, negate_a);
  require(core.read(Space::program, entry) == negate_a && core.read(Space::y, entry) == negate_a &&
              core.jit_statistics().invalidations == invalidations + 1,
          "An executable data alias invalidates native code through the existing architectural "
          "write API");
  core.set_registers(state);
  result = core.run(1);
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              core.registers().a == accumulator_mask - 2,
          "Changed aliased code executes its independent numeric expectation");
}

class Device final : public Io {
public:
  Word value = 0;
  unsigned reads = 0, writes = 0, fetches = 0;
  static constexpr Word port = 0xffff80;

  bool read(const Space space, const Word address, Word /*pc*/, Word &data) noexcept override {
    if (space == Space::program && address == 0) {
      ++fetches;
      data = 0;
      return true;
    }
    ++reads;
    if (space != Space::x || address != port) { return false; }
    data = value;
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word data) noexcept override {
    ++writes;
    if (space != Space::x || address != port) { return false; }
    value = data;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

void external() {
  Device peripherals;
  Core core(Memory::external(), &peripherals, {});
  constexpr Word data = 0x654321;
  const auto nop = core.run(1);
  require(nop.dispatches == 1 && nop.fault.kind == FaultKind::none && peripherals.fetches == 1,
          "An actual external program fetch uses its supplied owner");
  auto state = core.registers();
  state.x = data;
  core.set_registers(state);
  constexpr Word move_port_x0 = 0x044480;
  constexpr Word write_direction = 0x8000;
  const auto written =
      core.run_instruction({.address = 1, .opcode = move_port_x0 | write_direction});
  state = core.registers();
  state.x = 0;
  core.set_registers(state);
  const auto read = core.run_instruction({.address = 2, .opcode = move_port_x0});
  require(written.dispatches == 1 && read.dispatches == 1 &&
              written.fault.kind == FaultKind::none && read.fault.kind == FaultKind::none &&
              peripherals.value == data && core.registers().x == data && peripherals.writes == 1 &&
              peripherals.reads == 1,
          "External memory executes actual addressed I/O exactly once");
  bool rejected = false;
  try {
    const Core invalid(Memory::external(), nullptr, {});
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "External DSP memory requires an explicit bus owner");
}

void validation() {
  bool empty = false;
  try {
    static_cast<void>(Memory::unified({}));
  } catch (const std::invalid_argument &) { empty = true; }
  require(empty, "An empty unified declaration cannot silently become an external bus");
  std::array<Word, 1> backing{};
  bool short_storage = false;
  try {
    static_cast<void>(Memory::partition(backing, {.program_words = 1, .x_words = 1, .y_words = 1}));
  } catch (const std::invalid_argument &) { short_storage = true; }
  require(short_storage, "Partition extents are checked before slicing host memory");
  test::MemoryArena arena;
  const auto view = arena.memory();
  require(view.storage().program.size() == address_words,
          "Owned lazy arena exposes the same typed memory interface");
  const auto moved = std::move(arena);
  require(moved.memory().storage().program.data() == view.storage().program.data(),
          "Moving the owner retains physical backing for existing borrowed views");
}
}

int main() {
  try {
    default_run();
    natural_translation();
    partition();
    unified();
    external();
    validation();
    std::cout << "Memory/Core/Io/Options API, Harvard partition, executable aliases, external "
                 "access and ownership passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
