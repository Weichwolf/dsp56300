// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "move.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned op_add = 0x01, op_test = 0x85;
constexpr unsigned group_sub = 5, group_cmp = 7, shift_left = 4;
constexpr unsigned word_scale = 2;

std::size_t bank(const NativeMove &move) {
  return move.space == Space::x ? 0 : 1;
}

std::size_t known_offset(const NativeMove &move) {
  return offsetof(NativeContext, write_known) + (bank(move) * sizeof(std::uint8_t *));
}

bool tracks_writes(const std::span<const MemoryWindow> windows) {
  return std::ranges::any_of(windows, [](const auto &window) {
    return window.writable && !window.aliases_program && !window.known.empty();
  });
}
}

bool BlockEmitter::direct_memory(const BlockInstruction &operation) const {
  if (!lowerable(operation)) { return false; }
  const auto plan = plan_moves(operation.instruction);
  if (plan.count == 0) { return false; }
  return std::ranges::all_of(std::span(plan.transfers).first(plan.count), [this](const auto &move) {
    if (move.immediate()) { return true; }
    const auto &memory = runtime_.memory.at(bank(move));
    return !runtime_.windows.at(bank(move)).empty() ||
           (memory.data != nullptr && (move.load || !memory.aliases_program));
  });
}

void BlockEmitter::window_known(const NativeMove &move,
                                const MemoryWindow &window,
                                x64::Label &slow) {
  const bool save = !move.load || (window_capture_ & (Word{1} << bank(move))) != 0;
  if (!window.known.empty()) {
    code_.constant(Reg::r9, reinterpret_cast<std::uintptr_t>(window.known.data()));
    code_.binary(op_add, {.destination = Reg::r9, .source = Reg::ax});
    if (move.load) {
      code_.load(Reg::r11, Reg::r9, 0, Width::byte);
      code_.immediate(group_cmp, Reg::r11, 0, Width::dword);
      code_.jump_if(Branch::equal, slow);
    }
    if (save) { code_.store(Reg::r13, known_offset(move), Reg::r9); }
  } else if (save && tracks_writes(runtime_.windows.at(bank(move)))) {
    code_.store_constant(Reg::r13, known_offset(move), Width::qword, 0);
  }
}

void BlockEmitter::window_candidate(const NativeMove &move,
                                    const MemoryWindow &window,
                                    x64::Label &next) {
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  code_.mov({.destination = Reg::ax, .source = address}, Width::dword);
  if (window.address != 0) { code_.immediate(group_sub, Reg::ax, window.address, Width::dword); }
  code_.immediate(group_cmp, Reg::ax, static_cast<Word>(window.words.size()), Width::dword);
  code_.jump_if(Branch::above_equal, next);
}

void BlockEmitter::window_guard(const NativeMove &move, x64::Label &slow) {
  const auto address = move.space == Space::x ? Reg::cx : Reg::r8;
  x64::Label done;
  for (const auto &window : runtime_.windows.at(bank(move))) {
    if (!move.load && (!window.writable || window.aliases_program)) { continue; }
    x64::Label next;
    window_candidate(move, window, next);
    window_known(move, window, slow);
    code_.shift(shift_left, Reg::ax, word_scale);
    code_.constant(Reg::r11, reinterpret_cast<std::uintptr_t>(window.words.data()));
    code_.binary(op_add, {.destination = Reg::ax, .source = Reg::r11});
    code_.mov({.destination = address, .source = Reg::ax});
    if ((window_capture_ & (Word{1} << bank(move))) != 0) {
      const auto slot = move.space == Space::x ? x64::Xmm::x14 : x64::Xmm::x15;
      if (window.writable && !window.aliases_program) {
        code_.packed_from<64>(slot, address);
      } else {
        code_.constant(Reg::r11, 0);
        code_.packed_from<64>(slot, Reg::r11);
      }
    }
    code_.jump(done);
    code_.bind(next);
  }
  code_.jump(slow);
  code_.bind(done);
}

void BlockEmitter::mark_window_written(const NativeMove &move) {
  if (!tracks_writes(runtime_.windows.at(bank(move)))) { return; }
  x64::Label done;
  code_.load(Reg::r9, Reg::r13, known_offset(move));
  code_.binary(op_test, {.destination = Reg::r9, .source = Reg::r9});
  code_.jump_if(Branch::equal, done);
  code_.store_constant(Reg::r9, 0, Width::byte, 1);
  code_.bind(done);
}
}
