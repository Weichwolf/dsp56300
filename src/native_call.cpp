// SPDX-License-Identifier: GPL-3.0-only
#include "native_call.hpp"

#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_entry.hpp"
#include "jit/llvm/image.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace dsp56300::detail {
static_assert(std::atomic<NativeCall *>::is_always_lock_free &&
              std::atomic<std::uintptr_t>::is_always_lock_free &&
              std::atomic<bool>::is_always_lock_free);
thread_local std::atomic<NativeCall *> NativeCall::active_{nullptr};

std::uint64_t NativeCall::invoke(const ImageEntry entry,
                                 void *const owner,
                                 const Word pc,
                                 const std::uint32_t instructions,
                                 const std::uint64_t cycles) noexcept {
  auto *const previous = active_.load(std::memory_order_relaxed);
  active_.store(this, std::memory_order_relaxed);
  if (sigsetjmp(saved_, 0) == 0) {
    const auto result = dsp56300_image_enter(entry, owner, pc, instructions, cycles);
    active_.store(previous, std::memory_order_relaxed);
    return result;
  }
  active_.store(previous, std::memory_order_relaxed);
  constexpr unsigned reason_shift = 32;
  return static_cast<std::uint64_t>(ImageExit::fault) << reason_shift;
}

bool NativeCall::failed() const noexcept {
  return failed_.load(std::memory_order_relaxed);
}

bool NativeCall::capture(const std::uintptr_t address,
                         const bool writing,
                         const std::uintptr_t native_pc) noexcept {
  auto *const call = active_.load(std::memory_order_relaxed);
  if (call == nullptr) { return false; }
  bool owned = false;
  for (const auto &range : call->text_) {
    if (native_pc >= range.address && native_pc - range.address < range.size) {
      owned = true;
      break;
    }
  }
  if (!owned) { return false; }
  bool lent = false;
  for (const auto &space : call->views_) {
    const auto &view = writing ? space.write : space.read;
    const auto first = reinterpret_cast<std::uintptr_t>(view.data.data());
    if (address >= first && address - first < view.data.size_bytes()) {
      lent = true;
      break;
    }
  }
  if (!lent) { return false; }
  call->address_.store(address, std::memory_order_relaxed);
  call->pc_.store(native_pc, std::memory_order_relaxed);
  call->writing_.store(writing, std::memory_order_relaxed);
  call->failed_.store(true, std::memory_order_relaxed);
  siglongjmp(call->saved_, 1);
}

Fault NativeCall::fault(const std::span<const NativeSymbols::Site> sites) const {
  const auto pc = pc_.load(std::memory_order_relaxed);
  const NativeSymbols::Site *source = nullptr;
  for (const auto &site : sites) {
    if (pc >= site.address && pc - site.address < site.size) {
      source = &site;
      break;
    }
  }
  if (source == nullptr) {
    throw std::runtime_error("Terminal DSP host fault has no native source interval");
  }
  const auto writing = writing_.load(std::memory_order_relaxed);
  const auto address = address_.load(std::memory_order_relaxed);
  for (std::size_t bank = 0; bank < views_.size(); ++bank) {
    const auto &view = writing ? views_[bank].write : views_[bank].read;
    const auto first = reinterpret_cast<std::uintptr_t>(view.data.data());
    if (address >= first && address - first < view.data.size_bytes()) {
      return {.kind = writing ? FaultKind::memory_write : FaultKind::memory_read,
              .space = static_cast<Space>(bank),
              .address = static_cast<Word>((address - first) / sizeof(Word)),
              .pc = source->guest_pc,
              .variant = source->variant};
    }
  }
  throw std::logic_error("Terminal DSP host fault lost its lent view");
}
}

namespace dsp56300 {
std::uint64_t Core::Impl::call_image(const detail::ImageEntry entry,
                                     const Word pc,
                                     const std::uint32_t instructions,
                                     const std::uint64_t cycles) {
  if (image->memory_mode == CompileOptions::MemoryMode::exact) {
    return detail::dsp56300_image_enter(entry, this, pc, instructions, cycles);
  }
  const auto before = Progress{.instructions = cpu.instructions, .estimated_cycles = cpu.cycles};
  const auto result = native_call->invoke(entry, this, pc, instructions, cycles);
  if (native_call->failed()) {
    terminal_fault = true;
    image_ready = false;
    cpu.instructions = before.instructions;
    cpu.cycles = before.estimated_cycles;
    fault.kind = FaultKind::host_event;
    fault = native_call->fault(image->sites);
  }
  return result;
}

bool Core::host_fault(const std::uintptr_t address,
                      const bool writing,
                      const std::uintptr_t native_pc) noexcept {
  return detail::NativeCall::capture(address, writing, native_pc);
}
}
