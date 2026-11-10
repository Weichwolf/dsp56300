// SPDX-License-Identifier: GPL-3.0-only
#include "fetch.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"

#include <optional>
#include <span>

namespace dsp56300::detail {
InstructionCapture capture_instruction(const std::span<const Word> program,
                                       const Word address,
                                       const JitRuntime &runtime) {
  const auto word = [&](const Word pc) -> std::optional<Word> {
    if (pc > word_mask) { return std::nullopt; }
    if (runtime.program != nullptr) { return runtime.program(runtime.owner, pc); }
    return pc < program.size() ? std::optional(program[pc] & word_mask) : std::nullopt;
  };
  const auto unavailable = [&](const Word pc) {
    return InstructionCapture{
        .instruction = {},
        .failure = {
            .kind = FaultKind::memory_read, .space = Space::program, .address = pc, .pc = address}};
  };
  const auto opcode = word(address);
  if (!opcode) { return unavailable(address); }
  auto decoded = decode(*opcode);
  if (decoded.words == 2) {
    const auto next = (address + 1) & word_mask;
    const auto following = word(next);
    if (!following) { return unavailable(next); }
    decoded.extension = *following;
  }
  return {.instruction = decoded, .failure = {}};
}
}
