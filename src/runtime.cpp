// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "program_cache.hpp"
#include "translation.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::detail {
bool reset_io(Io *const io, const Word pc) noexcept {
  return io != nullptr && io->reset_peripherals(pc);
}

void access_bit_io(Io *const io, NativeAccess &access) noexcept {
  constexpr std::array actions{
      BitAction::test, BitAction::clear, BitAction::set, BitAction::change};
  const auto bit = access.value;
  const auto action =
      actions[static_cast<unsigned>(access.action) - static_cast<unsigned>(BusAction::bit_test)];
  bool previous = false;
  if ((io == nullptr || !io->bit(access.space, access.address, access.pc, bit, action, previous)) &&
      access.failure.kind == FaultKind::none) {
    access.failure = {.kind = FaultKind::io_bit,
                      .space = access.space,
                      .address = access.address,
                      .value = bit,
                      .pc = access.pc};
  }
  access.value = previous ? 1U : 0U;
}
}

namespace dsp56300 {
detail::CacheState *Core::Impl::jit_cache(void *const owner) noexcept {
  return &static_cast<Impl *>(owner)->program_cache().state();
}

void Core::Impl::jit_access(void *const owner, detail::NativeAccess &access) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  self.current_pc = access.pc;
  if (self.fault.kind == FaultKind::none) { self.fault = access.failure; }
  if (access.action == detail::BusAction::read) {
    access.value = self.read_memory(access.space, access.address);
  } else if (access.action == detail::BusAction::write) {
    self.write_memory(access.space, access.address, access.value);
  } else {
    detail::access_bit_io(self.io, access);
    if (self.fault.kind == FaultKind::none) { self.fault = access.failure; }
  }
  self.synchronize_program_view();
  access.failure = self.fault;
}

std::uint32_t Core::Impl::jit_execute(void *const owner,
                                      const detail::BlockInstruction *const operation) noexcept {
  auto &self = *static_cast<Impl *>(owner);

  self.step_decoded(operation->instruction);
  self.synchronize_program_view();
  if (self.fault.kind != FaultKind::none) { return 0; }

  const auto next = (operation->pc + operation->instruction.words) & word_mask;
  return self.cpu.processing == ExecutionState::running &&
                 (self.cpu.debug_flags & detail::debug_interrupt_pending) == 0 &&
                 self.vector.phase == VectorPhase::none && !self.instruction_cache_active() &&
                 self.cpu.reg.pc == next && self.cpu.repeat_remaining == 0 &&
                 !self.jit->exit_requested() &&
                 (self.cpu.reg.sr & detail::numeric_mode_mask) == operation->mode &&
                 detail::active_loop_address(self.cpu.reg) == operation->loop_address
             ? 2U
             : 1U;
}

std::uint32_t Core::Impl::jit_moves(void *const owner,
                                    const detail::BlockInstruction *const operation) noexcept {
  auto &self = *static_cast<Impl *>(owner);

  self.current_pc = operation->pc;
  self.current_words = operation->instruction.words;
  self.cpu.interrupt_inhibited = false;
  self.cpu.reg.pc = (operation->pc + operation->instruction.words) & word_mask;
  self.latched = {self.cpu.reg.a, self.cpu.reg.b, self.cpu.reg.x, self.cpu.reg.y};
  self.moves(operation->instruction);
  self.synchronize_program_view();
  return self.fault.kind == FaultKind::none ? 1U : 0U;
}
} // namespace dsp56300
