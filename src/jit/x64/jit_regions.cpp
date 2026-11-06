// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "native_region.hpp"
#include "region_guards.hpp"
#include "value_ranges.hpp"
#include "word_reuse.hpp"
#include "x64.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr unsigned group_and = 4;
constexpr auto r_offset = offsetof(Cpu, reg) + offsetof(Registers, r);
constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
}

std::vector<std::uint8_t> JitEmitter::generate(const std::span<const NativeOp> operations,
                                               const ExecutionBounds bounds) {
  bounds_ = bounds;
  if (bounds == ExecutionBounds::precise) {
    // Each interior exit needs real flags and ordinary guarded memory effects.
    region_mode_ = RegionMode::ordinary;
    residency_mode_ = LoopResidency::disabled;
    range_mode_ = RangeMode::disabled;
    word_mode_ = WordReuseMode::disabled;
    window_mode_ = LoopWindowMode::disabled;
  }
  if (region_mode_ == RegionMode::automatic) { return generate_selected(operations); }
  if (region_mode_ == RegionMode::guarded) {
    region_ = plan_native_region(operations, runtime_.memory, modifiers_);
    if (!region_.valid ||
        !std::ranges::all_of(operations, [](const auto &op) { return lowerable(op); })) {
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
  JitEmitter fast(runtime_,
                  modifiers_,
                  RegionMode::guarded,
                  residency_mode_,
                  range_mode_,
                  word_mode_,
                  window_mode_);
  const auto guarded = fast.generate(operations);
  x64::Code selector;
  x64::Label fallback;
  selector.raw(landing);
  region_guard(selector, Reg::di, fallback);
  selector.align_entry();
  selector.raw(guarded);
  selector.align_entry();
  selector.bind(fallback);
  selector.raw(ordinary);
  return selector.bytes();
}

// Initial and fallback guards remain read-only for every architectural field.
void JitEmitter::region_guard(x64::Code &code, const Reg cpu, x64::Label &failed) const {
  emit_region_loop_guard(code, cpu, region_, failed);
  emit_region_address_guards(code, cpu, region_, failed);
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
