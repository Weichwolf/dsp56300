// SPDX-License-Identifier: GPL-3.0-only
#include "checkpoint_support.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>

namespace dsp56300::test::checkpoint_test {
namespace {
enum class Pair : std::uint8_t { program_x, program_y, x_y };

struct Aliased {
  std::array<Word, words + 2> shared{};
  std::array<Word, words> separate{};

  Buffers buffers(const Pair pair, const bool reverse, const std::size_t offset) {
    auto low = std::span(shared).first(words);
    auto high = std::span(shared).subspan(offset, words);
    if (reverse) { std::swap(low, high); }
    switch (pair) {
      case Pair::program_x: return {.program = low, .x = high, .y = separate};
      case Pair::program_y: return {.program = low, .x = separate, .y = high};
      case Pair::x_y: return {.program = separate, .x = low, .y = high};
      default: throw std::invalid_argument("Unknown independent alias pair");
    }
  }
};

void alias_case(const Backend backend, const Pair pair, const bool reverse) {
  Aliased original;
  Core source(original.buffers(pair, reverse, 1), nullptr, {.backend = backend});
  source.set_registers(seed());
  source.write(Space::program, start, add_x0);
  const auto saved = source.checkpoint();
  Aliased incorrect;
  Core wrong(incorrect.buffers(pair, reverse, 2), nullptr, {.backend = backend});
  const auto before = wrong.registers();
  invalid([&] { wrong.restore(saved); });
  require(wrong.registers() == before && wrong.progress() == Progress{},
          "Same sizes and overlapping pairs cannot hide a changed positive/negative offset");
  Aliased identical = original;
  Core fresh(identical.buffers(pair, reverse, 1), nullptr, {.backend = backend});
  fresh.restore(saved);
  step(fresh);
  auto expected = seed();
  add(expected);
  ++expected.pc;
  require(fresh.registers() == expected &&
              fresh.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Matching alias offsets restore into different buffers and execute literal arithmetic");
}
}

void alias_offsets() {
  for (const auto backend : backends) {
    for (const auto pair : {Pair::program_x, Pair::program_y, Pair::x_y}) {
      for (const bool reverse : {false, true}) { alias_case(backend, pair, reverse); }
    }
  }
}
} // namespace dsp56300::test::checkpoint_test
