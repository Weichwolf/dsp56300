// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <algorithm>
#include <cstddef>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned op_add = 0x01, op_sub = 0x29;
constexpr unsigned group_and = 4, group_sub = 5, group_cmp = 7;
constexpr auto r_offset = offsetof(Cpu, reg) + offsetof(Registers, r);
constexpr auto n_offset = offsetof(Cpu, reg) + offsetof(Registers, n);
constexpr auto m_offset = offsetof(Cpu, reg) + offsetof(Registers, m);
}

void JitEmitter::move_guard(const NativeMove &move, x64::Label &slow) {
  if (move.immediate()) { return; }
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  const auto &form = move.address;
  const auto offset = form.index * sizeof(Word);
  if (form.mode != 4 && form.mode != 6) {
    code_.load(Reg::ax, Reg::r12, m_offset + offset, Width::dword);
    code_.immediate(group_and, Reg::ax, modifier_mask, Width::dword);
    code_.immediate(group_cmp, Reg::ax, modifier_mask, Width::dword);
    code_.jump_if(Branch::not_equal, slow);
  }
  if (form.mode == 6) {
    code_.constant(address, form.extra);
  } else {
    code_.load(address, Reg::r12, r_offset + offset, Width::dword);
    if (form.mode == 5) {
      code_.load(Reg::ax, Reg::r12, n_offset + offset, Width::dword);
      code_.binary(op_add, {.destination = address, .source = Reg::ax}, Width::dword);
    } else if (form.mode == 7) {
      code_.immediate(group_sub, address, 1, Width::dword);
    }
    code_.immediate(group_and, address, word_mask, Width::dword);
  }
  const auto &memory = runtime_.memory.at(move.space == Space::x ? 0 : 1);
  const auto bound = static_cast<Word>(std::min(memory.words, std::size_t{io_begin}));
  code_.immediate(group_cmp, address, bound, Width::dword);
  code_.jump_if(Branch::above_equal, slow);
}

void JitEmitter::move_update(const NativeMove &move) {
  const auto &form = move.address;
  if (form.mode == 4 || form.mode == 5 || form.mode == 6) { return; }
  const auto offset = form.index * sizeof(Word);
  if (form.mode == 7) {
    code_.store(
        Reg::r12, r_offset + offset, move.space == Space::x ? Reg::cx : Reg::r8, Width::dword);
    return;
  }
  code_.load(Reg::ax, Reg::r12, r_offset + offset, Width::dword);
  if (form.mode < 2) {
    code_.load(Reg::r9, Reg::r12, n_offset + offset, Width::dword);
  } else {
    code_.constant(Reg::r9, 1);
  }
  code_.binary((form.mode & 1U) != 0 ? op_add : op_sub,
               {.destination = Reg::ax, .source = Reg::r9},
               Width::dword);
  code_.immediate(group_and, Reg::ax, word_mask, Width::dword);
  code_.store(Reg::r12, r_offset + offset, Reg::ax, Width::dword);
}
} // namespace dsp56300::detail
