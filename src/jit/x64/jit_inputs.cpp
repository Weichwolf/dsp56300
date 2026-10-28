// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dataflow.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "x64.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Xmm;
constexpr std::array snapshots{Xmm::x3, Xmm::x4, Xmm::x5, Xmm::x6};
constexpr std::array offsets{
    offsetof(Registers, a), offsetof(Registers, b), offsetof(Registers, x), offsetof(Registers, y)};
constexpr auto latched_offset = offsetof(NativeContext, runtime) + offsetof(JitRuntime, latched);
}

bool JitEmitter::admit_cached_inputs(const std::span<const NativeOp> operations) const {
  return optimize_ && std::ranges::any_of(operations, [this](const auto &operation) {
           return lowerable(operation) && analyze_dataflow(operation.instruction).old_inputs != 0;
         });
}

void JitEmitter::value(const Reg destination, const unsigned index, const bool latched) {
  if (!latched || (dataflow_.old_inputs & (Word{1} << index)) == 0) {
    current_value(destination, index);
    return;
  }
  if (cached_inputs_) {
    code_.packed_to(destination, snapshots.at(index));
    return;
  }
  if (cached_status_) { code_.load(destination, Reg::r13, latched_offset); }
  code_.load(destination, cached_status_ ? destination : Reg::r14, index * sizeof(std::uint64_t));
}

void JitEmitter::capture_inputs() {
  if (dataflow_.old_inputs == 0) { return; }
  if (!cached_inputs_ && cached_status_) { code_.load(Reg::r11, Reg::r13, latched_offset); }
  for (unsigned index = 0; index < snapshots.size(); ++index) {
    if ((dataflow_.old_inputs & (Word{1} << index)) == 0) { continue; }
    if (!cached_inputs_) {
      current_value(Reg::ax, index);
      code_.store(cached_status_ ? Reg::r11 : Reg::r14, index * sizeof(std::uint64_t), Reg::ax);
    } else if (index < accumulator_dirty_.size() && accumulator_dirty_.at(index)) {
      code_.packed_from<64>(snapshots.at(index), index == 0 ? Reg::bp : Reg::bx);
    } else {
      code_.packed_load_lane<64>(
          snapshots.at(index), Reg::r12, offsetof(Cpu, reg) + offsets.at(index));
    }
  }
}

void JitEmitter::restore_inputs() {
  if (!cached_inputs_ || dataflow_.old_inputs == 0) { return; }
  code_.load(Reg::r11, Reg::r13, latched_offset);
  for (unsigned index = 0; index < snapshots.size(); ++index) {
    if ((dataflow_.old_inputs & (Word{1} << index)) != 0) {
      code_.packed_load_lane<64>(snapshots.at(index), Reg::r11, index * sizeof(std::uint64_t));
    }
  }
}
} // namespace dsp56300::detail
