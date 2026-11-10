// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"

#include <span>

namespace dsp56300::detail {
struct InstructionCapture {
  Instruction instruction;
  Fault failure;
};

// Inspect an immutable code view, never architectural fetch/I/O/cache state.
// A supplied program reader must obey the same side-effect-free view contract.
[[nodiscard]] InstructionCapture
capture_instruction(std::span<const Word> program, Word address, const JitRuntime &runtime);
}
