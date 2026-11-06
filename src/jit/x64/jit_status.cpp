// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit_emitter.hpp"
#include "x64.hpp"

#include <cstddef>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr auto status_offset = offsetof(Cpu, reg) + offsetof(Registers, sr);
constexpr unsigned op_or = 0x09, group_and = 4;
constexpr unsigned shift_left = 4, shift_right = 5;
}

void JitEmitter::initialize_status() {
  if (cached_status_) { code_.load(Reg::r14, Reg::r12, status_offset, Width::dword); }
  if (deferred_growth_shift_ != 0) {
    code_.packed(x64::Packed::bit_xor, x64::Xmm::x12, x64::Xmm::x12);
  }
}

void JitEmitter::materialize_status() {
  if (deferred_growth_shift_ != 0) {
    code_.packed_to(Reg::r9, x64::Xmm::x12);
    code_.shift(shift_right, Reg::r9, deferred_growth_shift_);
    code_.immediate(group_and, Reg::r9, 1, Width::dword);
    code_.shift(shift_left, Reg::r9, 7, Width::dword);
    merge_status(Reg::r9);
  }
  if (cached_status_) { code_.store(Reg::r12, status_offset, Reg::r14, Width::dword); }
}

void JitEmitter::read_status(const Reg destination) {
  if (cached_status_) {
    code_.mov({.destination = destination, .source = Reg::r14}, Width::dword);
  } else {
    code_.load(destination, Reg::r12, status_offset, Width::dword);
  }
}

void JitEmitter::merge_status(const Reg source, const Word changed, const Reg scratch) {
  if (cached_status_) {
    if (changed != 0) { code_.immediate(group_and, Reg::r14, ~changed, Width::dword); }
    code_.binary(op_or, {.destination = Reg::r14, .source = source}, Width::dword);
    return;
  }
  code_.load(scratch, Reg::r12, status_offset, Width::dword);
  if (changed != 0) { code_.immediate(group_and, scratch, ~changed, Width::dword); }
  code_.binary(op_or, {.destination = scratch, .source = source}, Width::dword);
  code_.store(Reg::r12, status_offset, scratch, Width::dword);
}

// Initially use the already qualified native-only admission. Keep the decision
// separate so local measurements can disable status caching alone.
bool JitEmitter::admit_cached_status() const {
  return packed_retirement_;
}
} // namespace dsp56300::detail
