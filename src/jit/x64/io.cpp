// SPDX-License-Identifier: GPL-3.0-only
#include "jit/x64/io.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dsp56300::detail {
void native_io_access(x64::Code &code,
                      const BlockInstruction &operation,
                      const NativeIoOperand operand,
                      const NativeIoExits exits) {
  using x64::Branch;
  using x64::Reg;
  using x64::Width;
  constexpr unsigned group_add = 0;
  constexpr unsigned group_sub = 5;
  constexpr unsigned group_cmp = 7;
  constexpr unsigned op_test = 0x85;
  constexpr auto bus = offsetof(NativeContext, bus);
  constexpr auto pc_offset = offsetof(Cpu, reg) + offsetof(Registers, pc);
  constexpr std::size_t stack_alignment = 16;
  constexpr auto allocation =
      ((sizeof(NativeAccess) + stack_alignment - 1) / stack_alignment) * stack_alignment;
  constexpr std::array<std::uint8_t, 2> call_ax{0xff, 0xd0};
  static_assert(sizeof(Fault) % sizeof(std::uint64_t) == 0);
  code.load(Reg::ax, Reg::r13, bus + offsetof(NativeBus, access));
  code.binary(op_test, {.destination = Reg::ax, .source = Reg::ax});
  code.jump_if(Branch::equal, exits.fallback);
  code.immediate(group_sub, Reg::sp, allocation);
  for (std::size_t offset = 0; offset < sizeof(Fault); offset += sizeof(std::uint64_t)) {
    code.store_constant(Reg::sp, offsetof(NativeAccess, failure) + offset, Width::qword, 0);
  }
  code.store(Reg::sp, offsetof(NativeAccess, address), operand.address, Width::dword);
  code.store(Reg::sp, offsetof(NativeAccess, value), Reg::dx, Width::dword);
  code.store_constant(Reg::sp, offsetof(NativeAccess, pc), Width::dword, operation.pc);
  code.store_constant(
      Reg::sp, offsetof(NativeAccess, space), Width::byte, static_cast<Word>(operand.space));
  code.store_constant(
      Reg::sp, offsetof(NativeAccess, action), Width::byte, static_cast<Word>(operand.action));
  code.store_constant(
      Reg::r12, pc_offset, Width::dword, (operation.pc + operation.instruction.words) & word_mask);
  code.store_constant(Reg::r12, offsetof(Cpu, interrupt_inhibited), Width::byte, 0);
  code.store_constant(Reg::r13,
                      offsetof(NativeContext, boundary),
                      Width::byte,
                      static_cast<Word>(NativeBoundary::owner));
  code.load(Reg::di, Reg::r13, bus + offsetof(NativeBus, owner));
  code.mov({.destination = Reg::si, .source = Reg::sp});
  code.raw(call_ax);
  x64::Label accepted;
  x64::Label ready;
  code.memory_immediate(
      group_cmp, Reg::sp, offsetof(NativeAccess, failure) + offsetof(Fault, kind), Width::byte, 0);
  code.jump_if(Branch::equal, accepted);
  for (std::size_t offset = 0; offset < sizeof(Fault); offset += sizeof(std::uint64_t)) {
    code.load(Reg::ax, Reg::sp, offsetof(NativeAccess, failure) + offset);
    code.store(Reg::r13, offsetof(NativeContext, failure) + offset, Reg::ax);
  }
  code.immediate(group_add, Reg::sp, allocation);
  code.jump(exits.failure);
  code.bind(accepted);
  code.load(Reg::dx, Reg::sp, offsetof(NativeAccess, value), Width::dword);
  code.immediate(group_add, Reg::sp, allocation);
  code.load(Reg::ax, Reg::r13, offsetof(NativeContext, exit_requested));
  code.binary(op_test, {.destination = Reg::ax, .source = Reg::ax});
  code.jump_if(Branch::equal, ready);
  code.memory_immediate(group_cmp, Reg::ax, 0, Width::byte, 0);
  code.jump_if(Branch::not_equal, ready);
  code.store_constant(Reg::r13,
                      offsetof(NativeContext, boundary),
                      Width::byte,
                      static_cast<Word>(NativeBoundary::none));
  code.bind(ready);
}

void BlockEmitter::instruction_fallback(const BlockInstruction &operation) {
  using x64::Reg;
  using x64::Width;
  constexpr unsigned op_test = 0x85;
  if (repeat_mode_) {
    code_.jump(exits_[0]);
    return;
  }
  helper(runtime_.execute, operation);
  code_.binary(op_test, {.destination = Reg::ax, .source = Reg::ax}, Width::dword);
  code_.jump_if(x64::Branch::equal, exits_[0]);
  constexpr std::array<std::uint8_t, 3> increment_retired{0x41, 0xff, 0xc7};
  code_.raw(increment_retired);
  code_.jump(exits_[0]);
}
}
