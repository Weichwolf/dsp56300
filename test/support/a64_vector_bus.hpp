// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "a64_loop_state.hpp"
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "support.hpp"
#include "vector_state.hpp"

namespace dsp56300::test::a64 {
inline bool visible_vector_state(const loop_model::Runner &runner,
                                 const loop_model::State &visible) {
  const auto &cpu = runner.cpu();
  auto physical = visible.reg;
  constexpr unsigned padding = 8;
  physical.a <<= padding;
  physical.b <<= padding;
  const auto &vector = runner.vector();
  return cpu.reg == physical && cpu.instructions == visible.instructions &&
         cpu.cycles == visible.cycles && cpu.repeat_remaining == visible.repeat.remaining &&
         cpu.repeat_pc == visible.repeat.pc && cpu.repeat_lc == visible.repeat.lc &&
         cpu.repeat_opcode == visible.repeat.opcode && cpu.repeat_loaded == visible.repeat.loaded &&
         cpu.repeat_completed == visible.completed &&
         cpu.interrupt_inhibited == visible.inhibited && cpu.processing == visible.processing &&
         cpu.debug_flags == visible.debug_flags && vector.resume == visible.vector.resume &&
         vector.address == visible.vector.address && vector.level == visible.vector.level &&
         vector.phase == visible.vector.phase && vector.words == visible.vector.words &&
         vector.loaded == visible.vector.loaded;
}

struct VectorAccess {
  loop_model::State visible;
  Word address = 0, pc = 0, value = 0;
  bool write = false, accept = true;
};

class VectorBus {
public:
  VectorBus(const loop_model::Runner &runner, const VectorAccess &access)
      : runner_(runner), access_(access) {}

  [[nodiscard]] detail::NativeBus binding() noexcept { return {.owner = this, .access = perform}; }

  void expect(const unsigned calls) const {
    require(
        valid_ && calls_ == calls,
        "Vector I/O observes the exact foreground PC, latched slot and pre-retirement CPU state");
  }

private:
  static void perform(void *const owner, detail::NativeAccess &request) noexcept {
    auto &self = *static_cast<VectorBus *>(owner);
    ++self.calls_;
    self.valid_ = self.valid_ && visible_vector_state(self.runner_, self.access_.visible) &&
                  request.space == Space::x && request.address == self.access_.address &&
                  request.pc == self.access_.pc && request.failure == Fault{} &&
                  request.action ==
                      (self.access_.write ? detail::BusAction::write : detail::BusAction::read) &&
                  request.value == (self.access_.write ? self.access_.value : 0U);
    if (!self.access_.write) { request.value = self.access_.value; }
    if (!self.access_.accept) {
      request.failure = {.kind = self.access_.write ? FaultKind::io_write : FaultKind::io_read,
                         .space = Space::x,
                         .address = request.address,
                         .value = self.access_.write ? self.access_.value : 0U,
                         .pc = request.pc};
    }
  }

  const loop_model::Runner &runner_;
  VectorAccess access_;
  unsigned calls_ = 0;
  bool valid_ = true;
};

class VectorReset final : public Io {
public:
  VectorReset(const loop_model::Runner &runner, const VectorAccess &access)
      : runner_(runner), access_(access) {}

  bool reset_peripherals(const Word pc) noexcept override {
    ++calls_;
    valid_ = valid_ && pc == access_.pc && visible_vector_state(runner_, access_.visible);
    return access_.accept;
  }

  void expect(const unsigned calls) const {
    require(valid_ && calls_ == calls,
            "Vector RESET publishes the exact pre-retirement foreground state to real I/O");
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    valid_ = false;
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    valid_ = false;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    valid_ = false;
    return false;
  }

private:
  const loop_model::Runner &runner_;
  VectorAccess access_;
  unsigned calls_ = 0;
  bool valid_ = true;
};

}
