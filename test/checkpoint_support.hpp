// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::checkpoint_test {
inline constexpr std::array backends{Backend::reference, Backend::cached, Backend::jit};
inline constexpr Word words = 4096, add_x0 = 0x200040, repeat_four = 0x0604a0;
inline constexpr Word trap = 6, service = 768, short_jsr = 0x0d0000, rti = 4;

struct Memory {
  std::array<Word, words> p{}, x{}, y{};
};

struct Machine {
  Memory memory;
  Core core;

  explicit Machine(const Backend backend, Io *io = nullptr, const unsigned cache_width = 128)
      : core({.program = memory.p, .x = memory.x, .y = memory.y},
             io,
             {.backend = backend,
              .cacheable_begin = 1024,
              .cacheable_end = words,
              .cache_sector_words = cache_width}) {}
};

inline Registers seed() {
  auto state = initial_state();
  state.x = 1;
  state.lc = 7;
  return state;
}

inline void step(Core &core) {
  const auto result = core.run(1);
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none,
          "One independently authored instruction retires without a fault");
}

inline void add(Registers &state) {
  // Positive tiny fractional sums: C/V/N/Z/E/L/S clear, U set; X0 stays one.
  state.a += std::uint64_t{1} << word_bits;
  state.sr = (state.sr & ~status_byte) | flag_u;
}

template <typename Action> void invalid(Action action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "An incompatible checkpoint is explicitly rejected");
}

void repetition();
void vectors();
void processing();
void cache();
void geometry();
void callbacks();
void shared_checkpoint();
void alias_offsets();
} // namespace dsp56300::test::checkpoint_test
