// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "native_region.hpp"
#include "x64.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned group_add = 0, group_and = 4, group_cmp = 7, op_cmp = 0x39, op_add = 0x01;
constexpr auto r_offset = offsetof(Cpu, reg) + offsetof(Registers, r);
constexpr auto n_offset = offsetof(Cpu, reg) + offsetof(Registers, n);
constexpr auto m_offset = offsetof(Cpu, reg) + offsetof(Registers, m);
constexpr auto lc_offset = offsetof(Cpu, reg) + offsetof(Registers, lc);
constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
}

std::vector<std::uint8_t> JitEmitter::generate(const std::span<const NativeOp> operations) {
  if (region_mode_ == RegionMode::automatic) { return generate_selected(operations); }
  if (region_mode_ == RegionMode::guarded) {
    region_ = plan_native_region(operations, runtime_.memory, modifiers_);
    if (!region_.valid ||
        !std::ranges::all_of(operations, [this](const auto &op) { return lowerable(op); })) {
      throw std::logic_error("Unproved native region");
    }
  }
  auto result = generate_body(operations);
  if (region_mode_ == RegionMode::guarded && !packed_retirement_) {
    throw std::logic_error("Native region requires complete retirement admission");
  }
  return result;
}

std::vector<std::uint8_t>
JitEmitter::generate_selected(const std::span<const NativeOp> operations) {
  region_mode_ = RegionMode::ordinary;
  auto ordinary = generate_body(operations);
  if (!packed_retirement_) { return ordinary; }
  region_ = plan_native_region(operations, runtime_.memory, modifiers_);
  if (!region_.valid) { return ordinary; }
  JitEmitter fast(runtime_, optimize_, batch_backend_, modifiers_, RegionMode::guarded);
  const auto guarded = fast.generate(operations);
  x64::Code selector;
  x64::Label fallback;
  selector.raw(landing);
  region_guard(selector, Reg::di, fallback);
  selector.raw(guarded);
  selector.bind(fallback);
  selector.raw(ordinary);
  return selector.bytes();
}

// Both the initial selector and every guarded loop header are read-only. The
// latter can return already-retired iterations without losing canonical state.
void JitEmitter::region_guard(x64::Code &code, const Reg cpu, x64::Label &failed) const {
  if (region_.loop) {
    code.load(Reg::ax, cpu, lc_offset, Width::dword);
    code.immediate(group_cmp, Reg::ax, 1, Width::dword);
    code.jump_if(Branch::below_equal, failed);
  }
  for (std::size_t index = 0; index < region_.addresses.size(); ++index) {
    const auto &address = region_.addresses[index];
    if (!address.accessed) { continue; }
    const auto offset = index * sizeof(Word);
    if (address.updated) {
      code.load(Reg::ax, cpu, m_offset + offset, Width::dword);
      code.immediate(group_and, Reg::ax, modifier_mask, Width::dword);
      code.immediate(group_cmp, Reg::ax, address.modifier, Width::dword);
      code.jump_if(Branch::not_equal, failed);
    }
    code.load(Reg::cx, cpu, r_offset + offset, Width::dword);
    if (!address.updated || address.modifier == modifier_mask) {
      if (address.stride_accesses != 0) {
        // Positive strides are monotone; use 64 bits so an overflowing 24/32-bit
        // product cannot turn an out-of-bounds trace into an admitted one.
        code.load(Reg::ax, cpu, n_offset + offset, Width::dword);
        code.immediate(group_cmp, Reg::ax, sign24_bit, Width::dword);
        code.jump_if(Branch::above_equal, failed);
        code.constant(Reg::r9, address.stride_accesses);
        code.multiply({.destination = Reg::ax, .source = Reg::r9});
        code.binary(op_add, {.destination = Reg::cx, .source = Reg::ax});
      }
      code.immediate(group_cmp,
                     Reg::cx,
                     address.linear_limit,
                     address.stride_accesses != 0 ? Width::qword : Width::dword);
      code.jump_if(Branch::above_equal, failed);
      continue;
    }
    const auto length = address.modifier + 1;
    const auto span = std::bit_ceil(length) - 1;
    code.mov({.destination = Reg::ax, .source = Reg::cx}, Width::dword);
    code.immediate(group_and, Reg::ax, word_mask & ~span, Width::dword);
    code.immediate(group_add, Reg::ax, length, Width::dword);
    code.immediate(group_cmp, Reg::ax, address.bound + 1, Width::dword);
    code.jump_if(Branch::above_equal, failed);
    code.binary(op_cmp, {.destination = Reg::cx, .source = Reg::ax}, Width::dword);
    code.jump_if(Branch::above_equal, failed);
  }
}

void JitEmitter::region_address(const NativeMove &move) {
  if (move.immediate()) { return; }
  const auto &form = move.address;
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  if (form.mode == 6) {
    code_.constant(address, form.extra);
  } else if (form.mode == 4) {
    code_.load(address, Reg::r12, r_offset + (form.index * sizeof(Word)), Width::dword);
  } else {
    const auto modifier = modifiers_.at(form.index) & modifier_mask;
    if (modifier == modifier_mask) {
      move_delta(move);
      if (form.mode == 1) {
        // The last post-N result need not stay in RAM; it is canonical at exit
        // and rechecked before a following loop body observes it.
        code_.immediate(
            group_and, move.space == Space::x ? Reg::si : Reg::di, word_mask, Width::dword);
      }
    } else {
      move_profiled_next(move, modifier);
    }
  }
}
} // namespace dsp56300::detail
