// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr Word short_address_mask = 0xfff;
constexpr unsigned control_byte_shift = 8;
constexpr unsigned interrupt_level_shift = 8;
constexpr Word interrupt_level_mask = 3;
constexpr Word stack_pointer_mask = 0x3f;
constexpr unsigned maximum_level = 2;
}

void Core::Impl::push(const Word high, const Word low) {
  if (cpu.reg.sc >= stack_entries - 1) {
    fail(FaultKind::stack, Space::program, current_pc, cpu.reg.sp);
    return;
  }
  cpu.reg.sp = (cpu.reg.sp + 1) & stack_pointer_mask;
  ++cpu.reg.sc;
  cpu.reg.stack[cpu.reg.sp & (stack_entries - 1)] = (std::uint64_t{high} << word_bits) | low;
}

std::uint64_t Core::Impl::pop() {
  if (cpu.reg.sc == 0) {
    fail(FaultKind::stack, Space::program, current_pc, cpu.reg.sp);
    return 0;
  }
  const auto value = cpu.reg.stack[cpu.reg.sp & (stack_entries - 1)];
  cpu.reg.sp = address_add(cpu.reg.sp, -1) & stack_pointer_mask;
  --cpu.reg.sc;
  return value;
}

void Core::Impl::execute(const Instruction &decoded) {
  auto &reg = cpu.reg;
  const auto opcode = decoded.opcode;
  switch (decoded.operation) {
    case Operation::nop: return;
    case Operation::parallel: parallel(decoded); return;
    case Operation::bit_register:
    case Operation::bit_memory:
    case Operation::bit_jump:
    case Operation::bit_branch: execute_bit(decoded); return;
    case Operation::move_control_effective:
    case Operation::move_control_short:
    case Operation::move_control_register:
    case Operation::move_control_immediate: control_move(decoded); return;
    case Operation::move_program_effective:
    case Operation::move_program_short: program_move(decoded); return;
    case Operation::move_peripheral_register:
    case Operation::move_peripheral_memory:
    case Operation::move_peripheral_program: peripheral_move(decoded); return;
    case Operation::arithmetic_shift_immediate:
    case Operation::arithmetic_shift_register:
    case Operation::logical_shift_immediate:
    case Operation::logical_shift_register: execute_shift(decoded); return;
    case Operation::immediate_alu: execute_immediate(decoded); return;
    case Operation::increment:
    case Operation::decrement: execute_unit(decoded); return;
    case Operation::compare_unsigned: compare_unsigned(opcode); return;
    case Operation::bit_field_extract:
    case Operation::bit_field_insert:
    case Operation::bit_field_merge:
    case Operation::count_leading_bits: execute_bit_field(decoded); return;
    case Operation::conditional_transfer: conditional_transfer(opcode); return;
    case Operation::relative_branch: execute_branch(decoded); return;
    case Operation::conditional_jump:
    case Operation::jump_short:
    case Operation::jump_effective:
    case Operation::call_short:
    case Operation::call_effective: execute_jump(decoded); return;
    case Operation::return_subroutine: reg.pc = static_cast<Word>(pop() >> word_bits); return;
    case Operation::return_interrupt: {
      const auto frame = pop();
      reg.pc = static_cast<Word>(frame >> word_bits);
      reg.sr = static_cast<Word>(frame) & word_mask;
      cpu.interrupt_inhibited = true;
      return;
    }
    case Operation::and_control:
    case Operation::or_control: {
      const unsigned field = opcode & 3;
      if (field > 2) {
        fail(FaultKind::unsupported_mode, Space::program, current_pc, opcode);
        return;
      }
      auto &target = field == 2 ? reg.omr : reg.sr;
      const unsigned shift = field == 0 ? control_byte_shift : 0;
      const auto value = ((opcode >> control_byte_shift) & 0xff) << shift;
      const Word byte_mask = Word{0xff} << shift;
      target = decoded.operation == Operation::and_control ? target & (value | ~byte_mask)
                                                           : target | value;
      return;
    }
    case Operation::repeat_immediate:
    case Operation::repeat_register:
    case Operation::repeat_short:
    case Operation::repeat_effective:
    case Operation::loop_immediate:
    case Operation::loop_register:
    case Operation::loop_short:
    case Operation::loop_effective:
    case Operation::loop_forever:
    case Operation::break_loop:
    case Operation::end_loop: execute_loop(decoded); return;
    case Operation::unsupported:
      fail(FaultKind::unsupported_instruction, Space::program, current_pc, opcode);
      return;
    default: fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode); return;
  }
}

void Core::interrupt(const Word vector, const unsigned level) {
  const ExecutionGuard guard(impl_->executing);
  auto &state = impl_->cpu.reg;
  const auto mask = (state.sr >> interrupt_level_shift) & interrupt_level_mask;
  if (vector > word_mask || (vector & 1U) != 0 || level > maximum_level || level < mask ||
      !interruptible()) {
    throw std::invalid_argument("DSP interrupt is not admissible at this boundary");
  }
  const auto foreground = state.pc;
  for (unsigned offset = 0; offset < 2; ++offset) {
    impl_->current_pc = vector + offset;
    const auto decoded = impl_->instruction(vector + offset);
    if (decoded.operation == Operation::call_short ||
        decoded.operation == Operation::call_effective) {
      const auto target = decoded.operation == Operation::call_short
                              ? decoded.opcode & short_address_mask
                              : impl_->effective({.mode = (decoded.opcode >> 11) & 7,
                                                  .index = (decoded.opcode >> 8) & 7,
                                                  .extra = decoded.extension});
      impl_->push(foreground, state.sr);
      constexpr Word cleared = scale_up | scale_down | arithmetic_16 | loop_flag | forever_flag |
                               (interrupt_level_mask << interrupt_level_shift);
      state.sr = (state.sr & ~cleared) | ((level + 1) << interrupt_level_shift);
      state.pc = target;
      ++impl_->cpu.instructions;
      return;
    }
    if ((decoded.operation != Operation::parallel && decoded.operation != Operation::nop) ||
        decoded.words != 1) {
      impl_->fail(
          FaultKind::unsupported_instruction, Space::program, vector + offset, decoded.opcode);
      return;
    }
    impl_->execute(decoded);
    if (impl_->fault.kind != FaultKind::none) { return; }
    ++impl_->cpu.instructions;
  }
  state.pc = foreground;
  impl_->cpu.interrupt_inhibited = true;
}
} // namespace dsp56300
