// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_long_load_fixture.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::test::long_load {
namespace {
class Memory final : public Io {
public:
  [[nodiscard]] std::optional<Word> inspect_program(const Word target) const noexcept override {
    return target == 0 ? std::optional<Word>(instruction) : std::nullopt;
  }

  bool read(const Space space, const Word target, const Word pc, Word &value) noexcept override {
    if (space == Space::program) {
      if (const auto word = inspect_program(target)) {
        value = *word;
        return true;
      }
      return false;
    }
    const auto index = reads++;
    if (index >= data.size()) { return false; }
    valid = valid && target == port && pc == 0 && space == (index == 0 ? Space::x : Space::y);
    if (!valid || reads == fail_at || known.at(index) == 0) { return false; }
    value = data.at(index);
    return true;
  }

  bool write(Space /*space*/, Word /*target*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*target*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    if (!mapped || space == Space::program) { return {}; }
    return std::span<const MemoryWindow>(windows).subspan(space == Space::x ? 0U : 1U, 1);
  }

  std::array<Word, 2> data{0x923456, 0xabcdef};
  std::array<std::uint8_t, 2> known{1, 1};
  Word instruction = opcode({}), port = address;
  unsigned reads = 0, fail_at = 0;
  bool valid = true, mapped = false;
  std::array<MemoryWindow, 2> windows{MemoryWindow{.address = address,
                                                   .words = std::span(data).first(1),
                                                   .known = std::span(known).first(1)},
                                      MemoryWindow{.address = address,
                                                   .words = std::span(data).last(1),
                                                   .known = std::span(known).last(1)}};
};

void sparse_windows(const unsigned selector) {
  const Input input{.selector = selector, .mode = 3};
  Memory memory;
  memory.instruction = opcode(input);
  memory.mapped = true;
  TestCore cpu(memory, {.backend = Backend::jit});
  cpu.set_registers(seed(input));
  auto expected = seed(input);
  expected_load(expected, input);
  const auto result = cpu.run(1);
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              cpu.registers() == expected && memory.reads == 0 &&
              memory.known == std::array<std::uint8_t, 2>{1, 1},
          "Long loads read both sparse known windows natively, through the public execution API");
}

void unknown_y_window() {
  const Input input{.mode = 3};
  Memory memory;
  memory.instruction = opcode(input);
  memory.mapped = true;
  memory.known[1] = 0;
  TestCore cpu(memory, {.backend = Backend::jit});
  const auto before = seed(input);
  cpu.set_registers(before);
  const auto actual = cpu.run(1);
  auto expected = before;
  ++expected.r[0];
  expected.pc = 1;

  require(actual.retired == 0 && actual.fault.kind == FaultKind::memory_read &&
              actual.fault.space == Space::y && actual.fault.address == address &&
              actual.fault.pc == 0 && cpu.registers() == expected && memory.reads == 2 &&
              memory.valid,
          "Unbounded unknown Y retains real ordered reads, first fault and sole AGU update");
}

void peripheral(const unsigned failure) {
  constexpr Word peripheral_port = 0xffff90;
  const Input input{.selector = 6, .mode = 3};
  Memory memory;
  memory.instruction = opcode(input);
  memory.port = peripheral_port;
  memory.fail_at = failure;
  auto expected = seed(input);
  expected.r[0] = memory.port;
  TestCore cpu(memory, {.backend = Backend::jit});
  cpu.set_registers(expected);
  if (failure == 0) {
    expected_load(expected, input);
  } else {
    ++expected.r[0];
    expected.pc = 1;
  }
  const auto result = cpu.run(1);
  require(result.retired == (failure == 0 ? 1U : 0U) && cpu.registers() == expected &&
              memory.valid && memory.reads == (failure == 1 ? 1U : 2U) &&
              result.fault.kind == (failure == 0 ? FaultKind::none : FaultKind::io_read) &&
              (failure == 0 || (result.fault.address == memory.port && result.fault.pc == 0 &&
                                result.fault.space == (failure == 1 ? Space::x : Space::y))),
          "Peripheral loads retain real ordered callbacks, partial faults and general ownership");
}

void host_aliases() {
  constexpr std::uint64_t shared_value = 0x923456923456;
  constexpr Word shared_word = 0x923456;
  for (unsigned selector = 0; selector < 8; ++selector) {
    const Input input{.selector = selector, .value = shared_value};
    std::array program{opcode(input)};
    std::array<Word, words> data{};
    data.at(address) = shared_word;
    const auto before_data = data;
    auto expected = seed(input);
    expected_load(expected, input);
    TestCore cpu({.program = program, .x = data, .y = data}, nullptr, {.backend = Backend::jit});
    cpu.set_registers(seed(input));
    const auto result = cpu.run(1);
    require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                cpu.registers() == expected && data == before_data,
            "Both long-load bus reads accept shared host storage without modifying RAM");
  }
}

void y_bounds() {
  constexpr Word high_word = 0x923456;
  const Input input{.mode = 3};
  std::array program{opcode(input)};
  std::array<Word, words> x{};
  std::array<Word, address> y{};
  x.at(address) = high_word;
  auto expected = seed(input);
  ++expected.r[0];
  expected.pc = 1;
  TestCore cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
  cpu.set_registers(seed(input));
  const auto result = cpu.run(1);
  require(result.retired == 0 && result.fault.kind == FaultKind::memory_read &&
              result.fault.space == Space::y && result.fault.address == address &&
              result.fault.pc == 0 && cpu.registers() == expected,
          "The second RAM bound guard preserves genuine partial-fault and AGU semantics");
}
}

void boundary_cases() {
  for (unsigned selector = 0; selector < 8; ++selector) { sparse_windows(selector); }
  unknown_y_window();
  for (unsigned failure = 0; failure < 3; ++failure) { peripheral(failure); }
  host_aliases();
  y_bounds();
}

void repeated_cases() {
  constexpr Word repeat_four = 0x0604a0;
  constexpr Word saved_lc = 0x357;
  for (unsigned selector = 0; selector < 8; ++selector) {
    const Input input{.selector = selector, .mode = 3};
    std::array program{repeat_four, opcode(input)};
    std::array<Word, words> x{};
    std::array<Word, words> y{};
    auto before = seed(input);
    before.lc = saved_lc;
    auto expected = before;
    for (unsigned index = 0; index < 4; ++index) {
      const auto value = input.value + index;
      x.at(address + index) = static_cast<Word>(value >> word_bits);
      y.at(address + index) = static_cast<Word>(value & word_mask);
      expected_load(expected, {.selector = selector, .mode = 3, .value = value});
    }
    expected.pc = 2;
    TestCore cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
    cpu.set_registers(before);
    require(cpu.run(1).retired == 1, "REP setup retires genuinely before long-load bodies");

    const auto result = cpu.run(4);
    require(result.retired == 4 && result.fault.kind == FaultKind::none &&
                cpu.registers() == expected &&
                cpu.progress() == Progress{.instructions = 5, .estimated_cycles = 9},
            "REP long loads use fresh X/Y data and one shared AGU update per native retirement");
  }
}
} // namespace dsp56300::test::long_load
