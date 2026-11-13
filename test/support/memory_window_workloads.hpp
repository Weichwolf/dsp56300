// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "external_run_fixture.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::test::window_workload {
inline constexpr Word body = 32, address = 0x80, loop_flag = 0x8000;
inline constexpr Word load_x0 = 0x44e000, alu_mask = 0xff;

// Authored RAM-read kernels share the independently modeled arithmetic domains.
// They diagnose memory/helper cost; they are not complete application workloads.
class Memory final : public Io {
public:
  Memory(const external_run::Workload input, const bool grant)
      : datum(static_cast<Word>(input.x & word_mask)),
        mapped(grant),
        windows{{{.address = address, .words = std::span(&datum, 1), .known = {}}}} {
    program.fill(load_x0 | (input.opcode & alu_mask));
  }

  Word datum;
  bool mapped;
  std::array<Word, body> program{};
  std::array<MemoryWindow, 1> windows;
  std::uint64_t reads = 0;

  [[nodiscard]] std::optional<Word> inspect_program(const Word pc) const noexcept override {
    if (pc < start || pc - start >= body) { return std::nullopt; }
    return program[pc - start];
  }

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space,
                                                             Word /*mode*/) noexcept override {
    return mapped && space == Space::x ? std::span<const MemoryWindow>(windows)
                                       : std::span<const MemoryWindow>{};
  }

  bool read(const Space space, const Word index, Word /*pc*/, Word &value) noexcept override {
    ++reads;
    if (space == Space::x && index == address) {
      value = datum;
      return true;
    }
    const auto word = inspect_program(index);
    if (space != Space::program || !word) { return false; }
    value = *word;
    return true;
  }

  bool write(Space /*space*/, Word /*index*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*index*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

inline Registers initial(const external_run::Workload input, const Word instructions) {
  auto state = initial_state();
  state.x = input.x;
  state.y = input.y;
  state.r[0] = address;
  state.sr = loop_flag;
  state.la = start + body - 1;
  state.lc = instructions / body + 2;
  state.sp = state.sc = 2;
  state.stack[2] = std::uint64_t{start} << word_bits;
  return state;
}

inline Registers expected(const external_run::Workload input, const Word instructions) {
  auto state = initial(input, instructions);
  for (Word index = 0; index < instructions; ++index) {
    const auto old = state;
    if (input.name == "communications-xor") {
      bfu_model::apply(state, old, {.function = bfu_model::Function::bit_xor});
    } else {
      sequence_model::alu(state, old, input.opcode);
    }
  }
  state.pc = start + instructions % body;
  state.lc -= instructions / body;
  return state;
}
}
