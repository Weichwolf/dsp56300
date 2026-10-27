// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <sys/mman.h>
#include <unistd.h>

namespace dsp56300::test {
namespace {
constexpr Word load_x0 = 0x44c000, store_x0 = 0x444000;
constexpr Word datum = 0xfedcba, other_datum = 0x123456, io_begin = 0xffff80;
constexpr unsigned xreg_shift = 18;
constexpr std::array xregs{4U, 5U, 14U, 15U}, yregs{6U, 7U, 14U, 15U};

struct Machine {
  explicit Machine(Io *io = nullptr, const bool optimize = true)
      : core({.program = p, .x = x, .y = y}, io, {.backend = Backend::jit, .optimize = optimize}) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

void changing_modifier() {
  Machine machine;
  for (const unsigned mode : {1U, 4U}) {
    machine.core.write(Space::program, start, load_x0 | (mode << 11));
    for (const Word modifier : {word_mask, 0xabffffU, 15U, 0U, 0x800fU}) {
      auto state = initial_state();
      state.r[0] = 7;
      state.n[0] = 1;
      state.m[0] = modifier;
      machine.core.write(Space::x, 7, datum);
      machine.core.set_registers(state);
      const auto before = machine.core.jit_statistics().native_move_instructions;
      require(machine.core.run(1).dispatches == 1, "Dynamic address guard preserves execution");
      const bool native = mode == 4 || (modifier & 0xffffU) == 0xffffU;
      require(machine.core.jit_statistics().native_move_instructions == before + (native ? 1U : 0U),
              "Changing M at a warm PC selects native linear or general addressing");
      auto expected = state;
      expected.pc = start + 1;
      expected.x = datum;
      if (mode == 1) { expected.r[0] = modifier == 0 ? 6U : 8U; }
      require(machine.core.registers() == expected, "Failed guards perform each AGU update once");
    }
  }
}

void rejected_address(const unsigned mode, const bool store) {
  Machine machine;
  machine.core.write(Space::program, start, (store ? store_x0 : load_x0) | (mode << 11));
  auto state = initial_state();
  state.r[0] = mode == 7 ? 0 : memory_words;
  state.n[0] = 1;
  state.x = datum;
  machine.core.set_registers(state);
  const auto result = machine.core.run(1);
  auto address = memory_words;
  if (mode == 5) { ++address; }
  if (mode == 7) { address = word_mask; }
  auto fault = store ? FaultKind::memory_write : FaultKind::memory_read;
  if (mode == 7) { fault = store ? FaultKind::io_write : FaultKind::io_read; }
  require(result.dispatches == 0 && result.progress == Progress{} && result.fault.kind == fault &&
              result.fault.address == address && result.fault.pc == start,
          "Out-of-span and predecrement-wrap addresses retain the general fault contract");
  require(machine.core.jit_statistics().native_move_instructions == 0,
          "Rejected native address never records a committed native transfer");
  require(machine.x == std::array<Word, memory_words>{} &&
              machine.y == std::array<Word, memory_words>{},
          "Rejected native address never writes beyond or inside supplied RAM");
}

class Port final : public Io {
public:
  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++reads;
    last = {.space = space, .address = address, .pc = pc};
    value = datum;
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

  struct Access {
    Space space = Space::program;
    Word address = 0, pc = 0;
  };

  Word reads = 0;
  Access last;
};

void io_boundary() {
  const MemoryArena arena;
  Port port;
  Core core(arena.buffers(), &port, {.backend = Backend::jit});
  constexpr Word paired_load_increment = 0xf09800; // X:(R0)+,X0 Y:(R4)+,Y0.
  core.write(Space::program, start, paired_load_increment);
  for (const bool both_io : {false, true}) {
    auto state = initial_state();
    state.r[0] = both_io ? io_begin : 7;
    state.r[4] = io_begin;
    core.write(Space::x, 7, other_datum);
    core.set_registers(state);
    const auto before = port.reads;
    require(core.run(1).dispatches == 1, "Paired I/O fallback retires");
    require(port.reads == before + 1 && port.last.pc == start && port.last.address == io_begin &&
                port.last.space == (both_io ? Space::x : Space::y),
            "Paired I/O fallback calls each admitted port once and preserves X-only dual-I/O");
    auto expected = state;
    expected.pc = start + 1;
    ++expected.r[0];
    ++expected.r[4];
    expected.x = both_io ? datum : other_datum;
    expected.y = both_io ? 0 : datum;
    require(core.registers() == expected, "I/O guards run before either address update");
    require(core.jit_statistics().native_move_instructions == 0,
            "Even full address-space backing cannot bypass the I/O callback");
  }
}

struct AliasInput {
  unsigned xreg = 0, yreg = 0, directions = 0;
  bool optimize = true;
};

void aliased_ports(const AliasInput input) {
  constexpr Word displacement = 16;
  constexpr Word address = 7;
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words + displacement> shared{};
  shared[address + displacement] = datum;
  auto expected_memory = shared;
  const std::span x{shared.data(), memory_words};
  const std::span y{shared.data() + displacement, memory_words};
  Core core({.program = p, .x = x, .y = y},
            nullptr,
            {.backend = Backend::jit, .optimize = input.optimize});
  const bool xload = (input.directions & 1U) != 0;
  const bool yload = (input.directions & 2U) != 0;
  const Word opcode = 0x800000 | (input.xreg << xreg_shift) | (input.yreg << 16) |
                      (xload ? 0x8000U : 0U) | (yload ? 0x400000U : 0U);
  core.write(Space::program, start, opcode);
  auto state = register_model::seed(mode_up);
  state.r[0] = address + displacement;
  state.r[4] = address;
  auto expected = state;
  const auto xv = xload ? datum : register_model::read(expected, xregs.at(input.xreg));
  const auto yv = yload ? datum : register_model::read(expected, yregs.at(input.yreg));
  if (xload) {
    register_model::write(expected, xregs.at(input.xreg), xv);
  } else {
    expected_memory[address + displacement] = xv;
  }
  if (yload) {
    register_model::write(expected, yregs.at(input.yreg), yv);
  } else {
    expected_memory[address + displacement] = yv;
  }
  expected.pc = start + 1;
  core.set_registers(state);
  require(core.run(1).dispatches == 1 && core.registers() == expected && shared == expected_memory,
          "Physically aliased X/Y ports capture both old values and commit in X/Y order");
  require(core.jit_statistics().native_move_instructions == (input.optimize ? 1U : 0U),
          "Native paired transfer accounts once per instruction and honors optimize=false");
}

// An inaccessible following page detects any generated load/store wider than Word.
class GuardedWord {
public:
  GuardedWord() {
    const auto supplied = sysconf(_SC_PAGESIZE);
    if (supplied <= 0) { throw std::runtime_error("Cannot obtain page size"); }
    page_ = static_cast<std::size_t>(supplied);
    mapping_ = mmap(nullptr, page_ * 2, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (mapping_ == MAP_FAILED) { throw std::runtime_error("Cannot allocate guard pages"); }
    if (mprotect(mapping_, page_, PROT_READ | PROT_WRITE) != 0) {
      munmap(mapping_, page_ * 2);
      throw std::runtime_error("Cannot enable test page");
    }
    word_ = reinterpret_cast<Word *>(static_cast<std::byte *>(mapping_) + page_ - sizeof(Word));
    *word_ = datum;
  }

  ~GuardedWord() { munmap(mapping_, page_ * 2); }

  GuardedWord(const GuardedWord &) = delete;
  GuardedWord &operator=(const GuardedWord &) = delete;

  [[nodiscard]] std::span<Word> words() const noexcept { return {word_, 1}; }

private:
  std::size_t page_ = 0;
  void *mapping_ = nullptr;
  Word *word_ = nullptr;
};

void exact_word_access() {
  const GuardedWord x;
  const GuardedWord y;
  std::array<Word, memory_words> p{};
  Core core({.program = p, .x = x.words(), .y = y.words()}, nullptr, {.backend = Backend::jit});
  // Read/write both address registers, exercising REX.X on the Y-port index R8.
  const std::array program{Word{0xc08000}, Word{0x800000}};
  core.load(Space::program, start, program);
  auto state = initial_state();
  core.set_registers(state);
  require(core.run(2).dispatches == 2 && core.registers().x == datum &&
              core.registers().y == datum && x.words()[0] == datum && y.words()[0] == datum,
          "Native guest word accesses stop exactly at an inaccessible page boundary");
  require(core.jit_statistics().native_move_instructions == 2,
          "Guard-page qualification executes emitted RAM accesses");
  state.r[4] = 1;
  core.set_registers(state);
  const auto before = core.jit_statistics().native_move_instructions;
  require(core.run(1).fault.kind == FaultKind::memory_read &&
              core.jit_statistics().native_move_instructions == before,
          "A failing Y guard falls back before a native X access or commit");
}

void raw_word_mask() {
  Machine machine;
  constexpr Word raw = 0xff123456;
  machine.x[0] = raw;
  machine.core.write(Space::program, start, load_x0 | (4U << 11));
  machine.core.set_registers(initial_state());
  require(machine.core.run(1).dispatches == 1 && machine.core.registers().x == (raw & word_mask) &&
              machine.x[0] == raw && machine.core.jit_statistics().native_move_instructions == 1,
          "Native reads mask caller-owned raw words without modifying host storage");
}

void program_alias(const bool y_store) {
  std::array<Word, memory_words> shared{};
  std::array<Word, memory_words> other{};
  Core core({.program = shared, .x = y_store ? other : shared, .y = y_store ? shared : other},
            nullptr,
            {.backend = Backend::jit});
  const Word opcode = y_store ? 0x808000 : 0xc00000;
  constexpr Word clear_a = 0x200013;
  core.load(Space::program, start, std::array{opcode, clear_a});
  auto state = initial_state();
  state.a = 3;
  state.r[y_store ? 4 : 0] = start + 1;
  core.set_registers(state);
  require(core.run(2).dispatches == 2 && core.registers().a == 3 && shared[start + 1] == 0,
          "A paired store through a P alias invalidates its already compiled successor");
  require(core.jit_statistics().native_move_instructions == 0 &&
              core.jit_statistics().invalidations == 1,
          "Either program-alias store port excludes the complete native move pair");
}

void budgets_and_ram_end() {
  Machine machine;
  std::array<Word, 8> program{};
  program.fill(store_x0 | (3U << 11));
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  constexpr Word begin = memory_words - program.size();
  state.r[0] = begin;
  state.x = datum;
  machine.core.set_registers(state);
  auto expected_memory = machine.x;
  for (std::size_t index = begin; index < expected_memory.size(); ++index) {
    expected_memory[index] = datum;
  }
  auto expected = state;
  expected.r[0] = memory_words;
  expected.pc = start + static_cast<Word>(program.size());
  require(machine.core.run(8).dispatches == 8 && machine.core.registers() == expected &&
              machine.x == expected_memory &&
              machine.core.jit_statistics().native_move_instructions == 8,
          "Native store blocks include the final valid word with exact retirement");
  machine.x.fill(0);
  machine.core.set_registers(state);
  expected = state;
  expected.r[0] += 3;
  expected.pc += 3;
  expected_memory.fill(0);
  expected_memory[begin] = datum;
  expected_memory[begin + 1] = datum;
  expected_memory[begin + 2] = datum;
  require(machine.core.run(0).dispatches == 0 && machine.core.run(3).dispatches == 3 &&
              machine.core.registers() == expected && machine.x == expected_memory &&
              machine.core.progress() == Progress{.instructions = 11, .estimated_cycles = 11},
          "A shorter warm-block budget commits only its admitted memory transfers");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::changing_modifier();
    for (const unsigned mode : {0U, 1U, 2U, 3U, 4U, 5U, 7U}) {
      dsp56300::test::rejected_address(mode, false);
      dsp56300::test::rejected_address(mode, true);
    }
    dsp56300::test::io_boundary();
    dsp56300::test::exact_word_access();
    dsp56300::test::raw_word_mask();
    dsp56300::test::program_alias(false);
    dsp56300::test::program_alias(true);
    dsp56300::test::budgets_and_ram_end();
    for (const bool optimize : {false, true}) {
      for (unsigned xreg = 0; xreg < 4; ++xreg) {
        for (unsigned yreg = 0; yreg < 4; ++yreg) {
          for (unsigned direction = 0; direction < 4; ++direction) {
            dsp56300::test::aliased_ports(
                {.xreg = xreg, .yreg = yreg, .directions = direction, .optimize = optimize});
          }
        }
      }
    }
    std::cout << dsp56300::test::checks << " native move boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
