// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "support.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace dsp56300::test::a64 {
struct BusEvent {
  Registers state{};
  Fault failure{};
  Fault prior_failure{};
  Space space = Space::x;
  Word address = 0, pc = start, value = 0;
  std::uint64_t instructions = 0, cycles = 0;
  bool write = false;
  const Word *observed = nullptr;
  Word observed_value = 0;
};

class Bus {
public:
  Bus(const Invocation &invocation, const std::span<const BusEvent> events)
      : invocation_(invocation), events_(events) {}

  [[nodiscard]] detail::NativeBus binding(const bool external = false) noexcept {
    return {.owner = this, .access = access, .external = external};
  }

  void remap(detail::NativeWindow &window, Word *const storage, const bool writable) noexcept {
    remapped_ = &window;
    storage_ = storage;
    writable_ = writable;
  }

  void expect() const {
    require(valid_ && next_ == events_.size(), "Exact bus events and published state");
  }

private:
  static std::uint64_t wrapped(const std::uint64_t count) noexcept {
    return count == 0 ? std::numeric_limits<std::uint64_t>::max() : count - 1;
  }

  static void access(void *const owner, detail::NativeAccess &request) noexcept {
    static_cast<Bus *>(owner)->perform(request);
  }

  void perform(detail::NativeAccess &request) noexcept {
    if (next_ == events_.size()) {
      valid_ = false;
      request.failure.kind = FaultKind::host_event;
      return;
    }
    const auto &expected = events_[next_++];
    const auto &cpu = invocation_.cpu();
    auto physical = expected.state;
    constexpr unsigned padding = 8;
    physical.a <<= padding;
    physical.b <<= padding;
    valid_ =
        valid_ && request.space == expected.space && request.address == expected.address &&
        request.pc == expected.pc &&
        request.action == (expected.write ? detail::BusAction::write : detail::BusAction::read) &&
        request.value == (expected.write ? expected.value : 0U) &&
        request.failure == expected.prior_failure && cpu.reg == physical &&
        cpu.instructions == wrapped(expected.instructions) &&
        cpu.cycles == wrapped(expected.cycles) && !cpu.interrupt_inhibited &&
        cpu.repeat_completed == (expected.instructions == 0) &&
        cpu.processing == ExecutionState::running &&
        (expected.observed == nullptr || *expected.observed == expected.observed_value);
    if (!expected.write) { request.value = expected.value; }
    request.failure = expected.failure;
    if (next_ == 1 && remapped_ != nullptr) {
      remapped_->data = storage_;
      remapped_->writable = writable_;
    }
  }

  const Invocation &invocation_;
  std::span<const BusEvent> events_;
  std::size_t next_ = 0;
  bool valid_ = true;
  detail::NativeWindow *remapped_ = nullptr;
  Word *storage_ = nullptr;
  bool writable_ = false;
};
}
