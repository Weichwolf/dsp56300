// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
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
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word load = pair | (2U << 18) | (3U << 16) | (1U << 15) | (1U << 22);
constexpr Word store = pair | (3U << 18) | (2U << 16) | (3U << 11) | (3U << 20);
constexpr std::size_t extent = 64, split = extent / 2;
constexpr Word x_host_bits = 0xab000000, y_host_bits = 0xcd000000;
constexpr Word io_address = 0xffff80;
constexpr Word callback_value = 0x13579b, ram_value = 0x2468ac;
using Data = std::array<Word, extent>;

struct MemoryIo final : Io {
  MemoryIo()
      : windows_{MemoryWindow{.words = std::span(x).first(split), .known = {}, .writable = true},
                 MemoryWindow{.address = split,
                              .words = std::span(x).subspan(split),
                              .known = known,
                              .writable = true},
                 MemoryWindow{.words = std::span(y).first(split), .known = {}, .writable = true},
                 MemoryWindow{.address = split,
                              .words = std::span(y).subspan(split),
                              .known = known,
                              .writable = true}} {
    program[start] = load;
    program[start + 1] = store;
    known.fill(1);
  }

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    if (space == Space::program) { return {}; }
    return std::span(windows_).subspan(space == Space::x ? 0U : 2U, 2);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program_words ? std::optional{program[address]} : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (callback_core != nullptr && space == Space::x && address == io_address) {
      ++callbacks;
      callback_pc = callback_core->registers().pc;
      callback_progress = callback_core->progress();
      callback_core->request_yield();
      value = callback_value;
      return true;
    }
    const auto word = inspect_program(address);
    if (space != Space::program || !word) { return false; }
    value = *word;
    return true;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  Data x{}, y{};
  std::array<std::uint8_t, split> known{};
  std::array<Word, memory_words> program{};
  std::size_t program_words = memory_words;
  Core *callback_core = nullptr;
  unsigned callbacks = 0;
  Word callback_pc = 0;
  Progress callback_progress{};
  std::uint64_t revision = 0;

  std::array<MemoryWindow, 4> windows_;
};

struct ExpectedMemory {
  Data x, y;
};

void owned_loop_access() {
  MemoryIo memory;
  Core core(Memory::external(), &memory);
  memory.callback_core = &core;
  auto initial = register_model::seed(0);
  initial.sr |= Word{1} << 15;
  initial.la = start + 1;
  initial.lc = 5;
  initial.r[0] = io_address;
  initial.r[4] = 4;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  memory.y[4] = ram_value;
  core.set_registers(initial);
  auto expected = initial;
  register_model::write(expected, 14, callback_value);
  register_model::write(expected, 15, memory.y[4]);
  ++expected.pc;
  const auto x = memory.x;
  const auto y = memory.y;
  const auto result = core.run(10);
  require(result.fault.kind == FaultKind::none && result.retired == 1 && result.yielded &&
              memory.callbacks == 1 && memory.callback_pc == start + 1 &&
              memory.callback_progress == Progress{} && core.registers() == expected &&
              memory.x == x && memory.y == y,
          "A real loop I/O callback observes exact state and yields before the next instruction");
}

void model_step(Registers &state, ExpectedMemory &memory) {
  auto &x = memory.x;
  auto &y = memory.y;
  if ((state.pc - start) % 2 == 0) {
    register_model::write(state, 14, x[state.r[0]] & word_mask);
    register_model::write(state, 15, y[state.r[4]] & word_mask);
    ++state.pc;
    return;
  }
  const auto a = register_model::bus_word(state, state.a);
  const auto b = register_model::bus_word(state, state.b);
  x[state.r[0]++] = b;
  y[state.r[4]++] = a;
  ++state.pc;
  if (state.pc - 1 != state.la) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
  } else {
    loop_frames::complete(state);
  }
}

void prefixes(const Word first, const Word count) {
  MemoryIo memory;
  Core core(Memory::external(), &memory);
  auto initial = register_model::seed(0);
  initial.sr |= Word{1} << 15;
  initial.la = start + 1;
  initial.lc = count;
  initial.r[0] = initial.r[4] = first;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  constexpr std::array<Word, 7> values{0, 1, sign24, word_mask, sign24 - 1, 0x123456, 0xabcdef};
  for (unsigned warmth = 0; warmth < 2; ++warmth) {
    for (Word prefix = 0; prefix <= count * 2; ++prefix) {
      for (std::size_t index = 0; index < extent; ++index) {
        memory.x[index] = values[index % values.size()] | x_host_bits;
        memory.y[index] = values[(index + 3) % values.size()] | y_host_bits;
      }
      ExpectedMemory contents{.x = memory.x, .y = memory.y};
      auto expected = initial;
      for (Word step = 0; step < prefix; ++step) { model_step(expected, contents); }
      core.set_registers(initial);
      const auto before = core.progress();
      const auto result =
          warmth == 0 ? core.run(prefix) : core.run_until(before.estimated_cycles + prefix);
      require(result.fault.kind == FaultKind::none && result.retired == prefix &&
                  result.progress ==
                      Progress{.instructions = before.instructions + prefix,
                               .estimated_cycles = before.estimated_cycles + prefix} &&
                  core.registers() == expected && memory.x == contents.x && memory.y == contents.y,
              "Cold and warm short loops preserve every prefix, full state and sparse RAM");
    }
  }
}

Registers loop_state() {
  auto state = register_model::seed(0);
  state.sr |= Word{1} << 15;
  state.la = start + 1;
  state.lc = 5;
  state.r[0] = state.r[4] = 4;
  state.stack[2] = std::uint64_t{start} << word_bits;
  return state;
}

Registers window_state(MemoryIo &memory) {
  constexpr Word instructions = 4;
  memory.program[start + 2] = load;
  memory.program[start + 3] = store;
  memory.program_words = start + instructions;
  auto state = loop_state();
  state.la = start + instructions + 1;
  return state;
}

void rebound_windows() {
  MemoryIo memory;
  MemoryIo other;
  Core core(Memory::external(), &memory);
  const auto initial = window_state(memory);
  for (const bool alternate : {false, true, false, true}) {
    auto &selected = alternate ? other : memory;
    auto &inactive = alternate ? memory : other;
    memory.windows_[0].words = std::span(selected.x).first(split);
    memory.windows_[2].words = std::span(selected.y).first(split);
    ++memory.revision;
    for (Word prefix = 0; prefix <= memory.program_words - start; ++prefix) {
      memory.x.fill(callback_value | x_host_bits);
      memory.y.fill(ram_value | y_host_bits);
      other.x.fill(sign24 | x_host_bits);
      other.y.fill(word_mask | y_host_bits);
      ExpectedMemory contents{.x = selected.x, .y = selected.y};
      const ExpectedMemory untouched{.x = inactive.x, .y = inactive.y};
      auto expected = initial;
      for (Word step = 0; step < prefix; ++step) { model_step(expected, contents); }
      core.set_registers(initial);
      const auto before = core.progress();
      const auto result = core.run(prefix);
      require(
          result.fault.kind == FaultKind::none && result.retired == prefix &&
              result.progress == Progress{.instructions = before.instructions + prefix,
                                          .estimated_cycles = before.estimated_cycles + prefix} &&
              core.registers() == expected && selected.x == contents.x &&
              selected.y == contents.y && inactive.x == untouched.x && inactive.y == untouched.y,
          "Returning immutable entries preserve every prefix after RAM window rebinding");
    }
  }
}

void revoked_windows() {
  enum class Revocation : std::uint8_t { unknown, read_only, shortened, program_alias };
  for (const auto revocation : {Revocation::unknown,
                                Revocation::read_only,
                                Revocation::shortened,
                                Revocation::program_alias}) {
    MemoryIo memory;
    Core core(Memory::external(), &memory);
    const auto initial = window_state(memory);
    core.set_registers(initial);
    require(core.run(4).retired == 4, "Prime an initialized writable RAM entry");
    switch (revocation) {
      case Revocation::unknown:
        memory.windows_[0].known = memory.known;
        memory.known[4] = 0;
        break;
      case Revocation::read_only: memory.windows_[0].writable = false; break;
      case Revocation::shortened: memory.windows_[0].words = std::span(memory.x).first(4); break;
      case Revocation::program_alias: memory.windows_[0].aliases_program = true; break;
      default: throw std::runtime_error("Invalid authored revocation");
    }
    ++memory.revision;
    core.set_registers(initial);
    const auto x = memory.x;
    const auto y = memory.y;
    const auto result = core.run(4);
    const bool load_fault =
        revocation == Revocation::unknown || revocation == Revocation::shortened;
    require(result.retired == (load_fault ? 0U : 1U) &&
                result.fault.kind ==
                    (load_fault ? FaultKind::memory_read : FaultKind::memory_write) &&
                result.fault.space == Space::x && result.fault.address == initial.r[0] &&
                result.fault.pc == start + (load_fault ? 0U : 1U) && memory.x == x && memory.y == y,
            "A retained initialized entry cannot survive knowledge, bound or write revocation");
  }
}

void rejected_windows() {
  constexpr Word first = 4;
  for (const bool executable : {false, true}) {
    MemoryIo memory;
    memory.windows_[0].writable = executable;
    memory.windows_[0].aliases_program = executable;
    Core core(Memory::external(), &memory);
    auto initial = initial_state();
    initial.sr = Word{1} << 15;
    initial.la = start + 1;
    initial.lc = 5;
    initial.r[0] = initial.r[4] = first;
    memory.x[first] = 1;
    memory.y[first] = sign24;
    core.set_registers(initial);
    const auto result = core.run(10);
    require(result.retired == 1 && result.fault.kind == FaultKind::memory_write &&
                result.fault.space == Space::x && result.fault.pc == start + 1 &&
                result.fault.address == first && memory.x[first] == 1 && memory.y[first] == sign24,
            "Read-only and executable-alias windows cannot acquire an unchecked writable view");
  }
  MemoryIo memory;
  memory.windows_[0].known = memory.known;
  memory.known[first] = 0;
  Core core(Memory::external(), &memory);
  auto initial = initial_state();
  initial.sr = Word{1} << 15;
  initial.la = start + 1;
  initial.lc = 5;
  initial.r[0] = initial.r[4] = first;
  core.set_registers(initial);
  const auto result = core.run(10);
  auto expected = initial;
  ++expected.pc; // The failed instruction was fetched; its fault keeps the original PC.
  require(result.retired == 0 && result.fault.kind == FaultKind::memory_read &&
              result.fault.space == Space::x && result.fault.pc == start &&
              result.fault.address == first && core.registers() == expected,
          "A tracked unknown input is a precise fault, never an initialized RAM prefix");
}

}
}

int main() try {
  dsp56300::test::rebound_windows();
  dsp56300::test::revoked_windows();
  dsp56300::test::owned_loop_access();
  dsp56300::test::rejected_windows();
  for (const dsp56300::Word first : {4U, 28U, 36U}) {
    for (const dsp56300::Word count : {1U, 2U, 3U, 5U, 8U}) {
      dsp56300::test::prefixes(first, count);
    }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
