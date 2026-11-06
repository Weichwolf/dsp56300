// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
namespace {
using x64::Reg;
using x64::Width;
constexpr unsigned op_or = 0x09, op_xor = 0x31, op_cmp = 0x39;
constexpr unsigned group_and = 4, shift_left = 4, shift_right = 5, shift_arithmetic = 7;
constexpr unsigned condition_greater = 0x9f, condition_less = 0x9c;
constexpr unsigned move_greater = 0x4f, move_less = 0x4c;
constexpr unsigned aligned_growth_shift = 53;
}

void JitEmitter::long_guards(const NativeMoves &moves, x64::Label &slow) {
  move_effective(moves.transfers[0]);
  // Both buses use one guest EA. Preserve it before a window maps X to a host pointer.
  code_.mov({.destination = Reg::r8, .source = Reg::cx}, Width::dword);
  move_memory_guard(moves.transfers[0], slow);
  move_memory_guard(moves.transfers[1], slow);
}

void JitEmitter::long_capture(const NativeMoves &moves, const Word mode) {
  const auto source = moves.transfers[0].reg;
  if (moves.transfers[0].load || source != moves.transfers[1].reg) {
    move_capture(moves.transfers[0], mode);
    move_capture(moves.transfers[1], mode);
    return;
  }
  // Full A/B uses one scaled 48-bit limiter. A10/B10 bypass both scaling and limiting.
  current_value(Reg::si, source == a_register ? 0U : 1U);
  const unsigned down = (mode & scale_down) != 0 ? 1U : 0U;
  const unsigned up = (mode & scale_up) != 0 ? 1U : 0U;
  code_.mov({.destination = Reg::r9, .source = Reg::si});
  code_.shift(shift_right, Reg::r9, aligned_growth_shift + down - up);
  code_.mov({.destination = Reg::r10, .source = Reg::r9});
  code_.shift(shift_right, Reg::r10, 1);
  code_.binary(op_xor, {.destination = Reg::r9, .source = Reg::r10});
  code_.immediate(group_and, Reg::r9, 1, Width::dword);
  code_.shift(shift_left, Reg::r9, 7, Width::dword);
  merge_status(Reg::r9);
  code_.shift(shift_arithmetic, Reg::si, padding_bits + down - up);
  constexpr auto minimum = -(std::int64_t{1} << (double_word_bits - 1));
  constexpr auto maximum = (std::int64_t{1} << (double_word_bits - 1)) - 1;
  code_.constant(Reg::r9, static_cast<std::uint64_t>(maximum));
  code_.binary(op_cmp, {.destination = Reg::si, .source = Reg::r9});
  code_.set(condition_greater, Reg::r10);
  code_.move_if(move_greater, {.destination = Reg::si, .source = Reg::r9});
  code_.constant(Reg::r9, static_cast<std::uint64_t>(minimum));
  code_.binary(op_cmp, {.destination = Reg::si, .source = Reg::r9});
  code_.set(condition_less, Reg::r11);
  code_.move_if(move_less, {.destination = Reg::si, .source = Reg::r9});
  code_.binary(op_or, {.destination = Reg::r10, .source = Reg::r11}, Width::dword);
  code_.shift(shift_left, Reg::r10, 6, Width::dword);
  merge_status(Reg::r10);
  code_.mov({.destination = Reg::di, .source = Reg::si});
  code_.shift(shift_right, Reg::si, word_bits);
  code_.immediate(group_and, Reg::si, word_mask, Width::dword);
  code_.immediate(group_and, Reg::di, word_mask, Width::dword);
}

void JitEmitter::long_commit(const NativeMoves &moves) {
  const auto high = moves.transfers[0].reg;
  const auto low = moves.transfers[1].reg;
  if (high >= a_register && high != low) {
    move_commit(moves.transfers[0]);
    move_commit(moves.transfers[1]);
    return;
  }
  // Both reads have completed before changing any destination. A10/B10 keeps
  // the old extension byte; full A/B sign-extends the actual 48-bit memory value.
  code_.mov({.destination = Reg::ax, .source = Reg::si});
  code_.shift(shift_left, Reg::ax, word_bits);
  code_.binary(op_or, {.destination = Reg::ax, .source = Reg::di});
  if (high == x1_register || high == y1_register) {
    const bool x = high == x1_register;
    if (loop_bank_cache_) {
      code_.packed_from<64>(x ? x64::Xmm::x5 : x64::Xmm::x6, Reg::ax);
    } else {
      code_.store(Reg::r12,
                  offsetof(Cpu, reg) + (x ? offsetof(Registers, x) : offsetof(Registers, y)),
                  Reg::ax);
    }
    return;
  }
  const bool partial = high == a1_register || high == b1_register;
  const bool b = high == b1_register || high == b_register;
  if (partial) {
    constexpr unsigned extension_shift = 56;
    current_value(Reg::r9, b ? 1U : 0U);
    code_.shift(shift_right, Reg::r9, extension_shift);
    code_.shift(shift_left, Reg::r9, extension_shift);
    code_.shift(shift_left, Reg::ax, padding_bits);
    code_.binary(op_or, {.destination = Reg::ax, .source = Reg::r9});
  } else {
    code_.shift(shift_left, Reg::ax, 2 * padding_bits);
    code_.shift(shift_arithmetic, Reg::ax, padding_bits);
  }
  if (loop_residency_) {
    store_accumulator(b, Reg::ax);
  } else {
    code_.store(Reg::r12,
                offsetof(Cpu, reg) + (b ? offsetof(Registers, b) : offsetof(Registers, a)),
                Reg::ax);
  }
}
} // namespace dsp56300::detail
