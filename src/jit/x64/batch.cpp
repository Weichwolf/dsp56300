// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "mac_batch.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::detail {
namespace {
constexpr std::array<std::uint8_t, 2> call_ax{0xff, 0xd0};
}

void BlockEmitter::batch_entry(const std::span<const BlockInstruction> operations) {
  if (!packed_retirement_ || !retirement_helpers_ || operations.size() != 1) { return; }
  const auto call = compile_mac_batch(operations.front());
  if (!call) { return; }
  mac_batches_ = true;
  using x64::Branch;
  using x64::Reg;
  using x64::Width;
  constexpr unsigned group_cmp = 7;
  constexpr unsigned group_and = 4;
  constexpr unsigned shift_left = 4;
  constexpr unsigned shift_right = 5;
  x64::Label skipped;
  code_.load(Reg::ax, Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, lc), Width::dword);
  code_.immediate(group_cmp, Reg::ax, mac_batch_width, Width::dword);
  code_.jump_if(Branch::below_equal, skipped);
  code_.load(Reg::ax, Reg::r13, offsetof(NativeContext, budget), Width::dword);
  code_.immediate(group_cmp, Reg::ax, mac_batch_width - 1, Width::dword);
  code_.jump_if(Branch::below_equal, skipped);
  code_.load(Reg::ax, Reg::r12, offsetof(Cpu, reg) + offsetof(Registers, sp), Width::dword);
  code_.immediate(group_and, Reg::ax, stack_entries - 1, Width::dword);
  code_.shift(shift_left, Reg::ax, 1, Width::dword);
  code_.load_indexed(Reg::dx,
                     {.base = Reg::r12,
                      .index = Reg::ax,
                      .offset = offsetof(Cpu, reg) + offsetof(Registers, stack)},
                     Width::qword);
  code_.shift(shift_right, Reg::dx, word_bits);
  code_.immediate(group_cmp, Reg::dx, operations.front().pc);
  code_.jump_if(Branch::not_equal, skipped);
  code_.mov({.destination = Reg::di, .source = Reg::r12});
  code_.mov({.destination = Reg::si, .source = Reg::r13});
  code_.constant(Reg::dx, reinterpret_cast<std::uintptr_t>(&operations.front()));
  code_.constant(Reg::cx, call->operands);
  code_.constant(Reg::ax, std::bit_cast<std::uintptr_t>(call->entry));
  code_.raw(call_ax);
  code_.mov({.destination = Reg::r15, .source = Reg::ax}, Width::dword);
  code_.bind(skipped);
}
} // namespace dsp56300::detail
