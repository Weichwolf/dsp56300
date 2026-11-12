// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fetch.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#if defined(__aarch64__)
#include "jit/a64/block_boundary.hpp"
#endif
#include "transfers.hpp"
#include "translation.hpp"

#include <span>
#include <vector>

namespace dsp56300::detail {
namespace {
bool terminates(const Instruction &instruction) {
#if defined(__aarch64__)
  if (a64::block_boundary(instruction)) { return true; }
#endif
  const auto operation = instruction.operation;
  return !repeatable(operation) || operation == Operation::conditional_transfer ||
         operation == Operation::illegal || operation == Operation::undefined ||
         operation == Operation::debug || operation == Operation::conditional_debug;
}

bool callback_boundary(const Instruction &instruction) {
  if (instruction.operation == Operation::bit_memory) { return true; }
  const auto transfers = inspect_transfers(instruction.opcode);
  if (!transfers) { return false; }
  for (unsigned index = 0; index < transfers->count; ++index) {
    const auto &move = transfers->moves[index];
    if (move.source.kind == TransferOperandKind::memory ||
        move.destination.kind == TransferOperandKind::memory) {
      return true;
    }
  }
  return false;
}
}

std::vector<BlockInstruction> parse_block(const std::span<const Word> program,
                                          const Registers &state,
                                          const JitRuntime &runtime,
                                          const bool callback_boundaries) {
  std::vector<BlockInstruction> operations;
  auto pc = state.pc;
  const auto mode = state.sr & numeric_mode_mask;
  const auto loop_address = active_loop_address(state);
  while (true) {
    const auto capture = capture_instruction(program, pc, runtime);
    if (capture.failure.kind != FaultKind::none) { break; }
    const auto &decoded = capture.instruction;
    operations.push_back(
        {.instruction = decoded, .pc = pc, .mode = mode, .loop_address = loop_address});
    if (terminates(decoded) || at_loop_end(operations.back()) ||
        (callback_boundaries && callback_boundary(decoded))) {
      break;
    }
    if (runtime.program == nullptr && decoded.words >= program.size() - pc) { break; }
    const auto next = (pc + decoded.words) & word_mask;
    if (next <= pc) { break; }
    pc = next;
  }
  return operations;
}
}
