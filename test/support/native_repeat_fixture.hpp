// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "external_run_fixture.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::test::native_repeat {
inline constexpr Word body = start + 1, saved_lc = 0x357, ce = 0x80000;
inline constexpr Word zero_count = 65536, data_address = 0x80;
inline constexpr Word repeat_base = 0x0600a0, byte_mask = 0xff;
inline constexpr std::array modes{
    Word{0}, mode_down, mode_up, mode_rm, Word{0x100000}, Word{0x100000} | mode_down};

constexpr Word repeat(const Word count) {
  return repeat_base | ((count & byte_mask) << 8) | ((count >> 8) & 15U);
}

class Bus final : public Io {
public:
  std::array<Word, memory_words> program{}, data{};
  std::array<std::uint8_t, memory_words> known{};
  std::array<MemoryWindow, 1> windows{{{.words = data, .known = known, .writable = true}}};
  std::uint64_t revision = 0;
  unsigned program_reads = 0, reads = 0, writes = 0;
  bool grant = true, visible = true, yield = false;
  Core *owner = nullptr;

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return visible && address < program.size() ? std::optional(program[address]) : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return grant && space == Space::x ? std::span<const MemoryWindow>(windows)
                                      : std::span<const MemoryWindow>{};
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (space == Space::program) {
      ++program_reads;
      if (address >= program.size()) { return false; }
      value = program[address];
      return true;
    }
    ++reads;
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

inline Registers seed(const external_run::Workload input, const Word mode = 0) {
  auto state = initial_state();
  state.sr = mode | ce;
  state.lc = saved_lc;
  state.x = input.x;
  state.y = input.y;
  return state;
}

inline void arithmetic(Registers &state, const Word opcode) {
  const auto old = state;
  if (opcode == external_run::workloads.back().opcode) {
    bfu_model::apply(state, old, {.function = bfu_model::Function::bit_xor});
  } else {
    sequence_model::alu(state, old, opcode);
  }
}

struct Repetition {
  Word mode = 0, count = 0, retired = 0;
};

inline Registers expected(const external_run::Workload input, const Repetition run) {
  auto state = seed(input, run.mode);
  for (Word index = 0; index < run.retired; ++index) { arithmetic(state, input.opcode); }
  const auto total = run.count == 0 ? zero_count : run.count;
  state.pc = run.retired == total ? body + 1 : body;
  state.lc = total - run.retired;
  if (run.retired == total) { state.lc = saved_lc; }
  if (run.count == 0 && run.retired == 0) { state.lc = 0; }
  return state;
}
} // namespace dsp56300::test::native_repeat
