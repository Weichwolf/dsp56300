// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "translation.hpp"

#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
namespace {
bool terminates(const Operation operation) {
  return !repeatable(operation) || operation == Operation::conditional_transfer ||
         operation == Operation::illegal || operation == Operation::debug ||
         operation == Operation::conditional_debug;
}
}

std::vector<BlockInstruction> parse_block(const std::span<const Word> program,
                                          const Registers &state,
                                          const JitRuntime &runtime) {
  const auto word = [&](const Word address) -> std::optional<Word> {
    if (runtime.program != nullptr) { return runtime.program(runtime.owner, address); }
    return address < program.size() ? std::optional(program[address] & word_mask) : std::nullopt;
  };
  std::vector<BlockInstruction> operations;
  auto pc = state.pc;
  const auto mode = state.sr & numeric_mode_mask;
  const auto loop_address = active_loop_address(state);
  while (true) {
    const auto opcode = word(pc);
    if (!opcode) { break; }
    auto decoded = decode(*opcode);
    if (decoded.words == 2) {
      const auto following = word((pc + 1) & word_mask);
      if (!following) { break; }
      decoded.extension = *following;
    }
    operations.push_back(
        {.instruction = decoded, .pc = pc, .mode = mode, .loop_address = loop_address});
    if (terminates(decoded.operation) || at_loop_end(operations.back())) { break; }
    if (runtime.program == nullptr && decoded.words >= program.size() - pc) { break; }
    const auto next = (pc + decoded.words) & word_mask;
    if (next <= pc) { break; }
    pc = next;
  }
  return operations;
}
}
