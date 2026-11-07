// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "jit/resident.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300::detail {
std::vector<std::uint8_t> emit_block(const std::span<const BlockInstruction> operations,
                                     const JitRuntime &runtime,
                                     const std::array<Word, agu_registers> &modifiers) {
  if (operations.empty()) { throw std::invalid_argument("Cannot emit an empty DSP block"); }
  if (operations.size() == 1) {
    BlockEmitter single(runtime, modifiers);
    return single.generate(operations);
  }
  BlockEmitter precise(runtime, modifiers);
  const auto checked = precise.generate(operations, ExecutionBounds::precise);
  BlockEmitter fast(runtime, modifiers);
  if (auto region = fast.generate_guarded(operations, checked)) { return std::move(*region); }
  BlockEmitter whole(runtime, modifiers, RegionMode::ordinary);
  auto full = whole.generate(operations);
  x64::Code code;
  x64::Label interior;
  constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
  constexpr unsigned compare = 7;
  code.raw(landing);
  code.load(x64::Reg::ax, x64::Reg::si, offsetof(NativeContext, budget), x64::Width::dword);
  code.immediate(
      compare, x64::Reg::ax, static_cast<Word>(operations.size() - 1), x64::Width::dword);
  code.jump_if(x64::Branch::below_equal, interior);
  code.align_entry();
  code.raw(full);
  code.align_entry();
  code.bind(interior);
  code.raw(checked);
  return code.bytes();
}
}
