// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::test::transfer_run {
inline constexpr Word length = 2, ram_words = 128, loop_flag = Word{1} << 15;
using Data = std::array<Word, ram_words>;
using Knowledge = std::array<std::uint8_t, ram_words>;

struct Seed {
  Word iterations, mode, modifier;
};

struct Banks {
  Data &x, &y;
};

struct Access {
  Word address, pc, value = 0;
};

struct Kernel {
  isa::Opcode load_alu = isa::Opcode::move_none, store_alu = isa::Opcode::move_none;
};

inline std::array<Word, length> program(const Kernel kernel) {
  constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
  constexpr Word loads = (Word{1} << 15) | (Word{1} << 22);
  constexpr unsigned x_register_shift = 18;
  constexpr Word increment = (3U << 11) | (3U << 20);
  return {pair | loads | (2U << x_register_shift) | (3U << 16) |
              (static_cast<Word>(kernel.load_alu) & status_byte),
          pair | increment | (3U << x_register_shift) | (2U << 16) |
              (static_cast<Word>(kernel.store_alu) & status_byte)};
}

inline Registers initial(const Seed seed) {
  const auto [iterations, mode, modifier] = seed;
  constexpr Word previous_end = 0x350;
  constexpr Word previous_count = 19;
  auto state = register_model::seed(mode | loop_flag);
  state.r[0] = state.r[4] = 0;
  state.m[0] = state.m[4] = modifier;
  state.la = start + length - 1;
  state.lc = iterations;
  state.stack[1] = (std::uint64_t{previous_end} << word_bits) | previous_count;
  state.stack[2] = (std::uint64_t{start} << word_bits) | mode;
  return state;
}

inline void step(Registers &state, const Banks banks, const Kernel kernel) {
  const auto &[x, y] = banks;
  const auto old = state;
  const bool loading = state.pc == start;
  if (loading) {
    register_model::write(state, 14, x[state.r[0]] & word_mask);
    register_model::write(state, 15, y[state.r[4]] & word_mask);
  } else {
    const auto left = register_model::bus_word(state, state.b);
    const auto right = register_model::bus_word(state, state.a);
    x[state.r[0]] = left;
    y[state.r[4]] = right;
    for (const unsigned index : {0U, 4U}) {
      state.r[index] = address_model::updated({.modifier = state.m[index],
                                               .cursor = state.r[index],
                                               .offset = state.n[index],
                                               .mode = 3,
                                               .index = index});
    }
  }
  const auto alu = static_cast<Word>(loading ? kernel.load_alu : kernel.store_alu) & status_byte;
  if (alu != 0) { sequence_model::alu(state, old, alu); }
  ++state.pc;
  if (loading) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
  } else {
    loop_frames::complete(state);
  }
}

struct Machine final : public Io {
  explicit Machine(const Kernel input = {}, const bool shared = false)
      : kernel(input),
        shared_(shared),
        code_(program(input)),
        windows_{
            {{.words = x, .known = x_known, .writable = true},
             {.words = shared ? std::span(x) : std::span(y), .known = y_known, .writable = true}}},
        core(Memory::external(), this) {
    for (Word index = 0; index < ram_words; ++index) {
      constexpr std::array<Word, 8> values{
          0, 1, sign24 - 1, sign24, word_mask, 0x400000, 0xc00000, 0xab123456};
      x[index] = values[index % values.size()];
      y[index] = values[(3 * index + 5) % values.size()];
    }
    x_known.fill(1);
    y_known.fill(1);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address >= start && address - start < length ? std::optional{code_[address - start]}
                                                        : std::nullopt;
  }

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space,
                                                             Word /*mode*/) noexcept override {
    return space == Space::program ? std::span<const MemoryWindow>{}
                                   : std::span(&windows_[space == Space::x ? 0 : 1], 1);
  }

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (space == Space::program) {
      const auto word = inspect_program(address);
      if (!word) { return false; }
      value = *word;
      return true;
    }
    if (!observe({.address = address, .pc = pc})) { return false; }
    value = (space == Space::x || shared_ ? x : y)[address];
    return true;
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    if (space == Space::program || !observe({.address = address, .pc = pc, .value = value})) {
      return false;
    }
    (space == Space::x || shared_ ? x : y)[address] = value;
    (space == Space::x ? x_known : y_known)[address] = 1;
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

  bool observe(const Access access) noexcept {
    ++accesses;
    const auto progress = core.state().progress;
    observed &= access.pc == start + ((progress.instructions - base) % length) &&
                progress.instructions == progress.estimated_cycles;
    if (yield_on_access) { core.request_yield(); }
    return access.address < ram_words && access.value <= word_mask;
  }

  const Kernel kernel;
  Data x{}, y{};
  Knowledge x_known{}, y_known{};
  Word accesses = 0;
  std::uint64_t base = 0;
  bool observed = true, yield_on_access = false;

  bool shared_;
  std::array<Word, length> code_;
  std::array<MemoryWindow, 2> windows_;

  Core core;
};
} // namespace dsp56300::test::transfer_run
