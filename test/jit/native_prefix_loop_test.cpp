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

namespace dsp56300::test {
namespace {
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word load = pair | (2U << 18) | (3U << 16) | (1U << 15) | (1U << 22);
constexpr Word store = pair | (3U << 18) | (2U << 16) | (3U << 11) | (3U << 20);
constexpr std::size_t extent = 64, split = extent / 2;
constexpr Word x_host_bits = 0xab000000, y_host_bits = 0xcd000000;
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
    return address < program.size() ? std::optional{program[address]} : std::nullopt;
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
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

  std::array<MemoryWindow, 4> windows_;
};

struct ExpectedMemory {
  Data x, y;
};

void model_step(Registers &state, ExpectedMemory &memory) {
  auto &x = memory.x;
  auto &y = memory.y;
  if (state.pc == start) {
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
