// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr unsigned control_byte_shift = 8;
constexpr unsigned interrupt_level_shift = 8;
constexpr Word interrupt_level_mask = 3;
constexpr Word stack_pointer_mask = 0x3f;
constexpr unsigned maximum_level = 3;
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

void Core::Impl::execute_return(const Instruction &decoded) {
  const auto frame = pop();
  if (fault.kind != FaultKind::none) { return; }

  cpu.reg.pc = static_cast<Word>(frame >> word_bits);
  if (decoded.operation == Operation::return_interrupt) {

    cpu.reg.sr = static_cast<Word>(frame) & word_mask;

    cpu.interrupt_inhibited = true;
  }
}

void Core::Impl::execute_operation(const Instruction &decoded) {
  auto &reg = cpu.reg;
  const auto opcode = decoded.opcode;
  switch (decoded.operation) {
    case Operation::nop: return;
    case Operation::debug:
    case Operation::conditional_debug: execute_debug(decoded); return;
    case Operation::reset_peripherals:
    case Operation::wait:
    case Operation::stop: execute_processing(decoded); return;
    case Operation::cache_flush:
    case Operation::cache_flush_unlocked:
    case Operation::cache_free:
    case Operation::cache_lock:
    case Operation::cache_unlock:
    case Operation::cache_lock_relative:
    case Operation::cache_unlock_relative: execute_cache(decoded); return;
    case Operation::illegal:
    case Operation::trap:
    case Operation::conditional_trap: execute_exception(decoded); return;
    case Operation::parallel: parallel(decoded); return;
    case Operation::move_displacement: displacement_move(decoded); return;
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
    case Operation::load_relative_address:
    case Operation::load_updated_address: execute_address_calculation(decoded); return;
    case Operation::divide_iteration:
    case Operation::normalize_iteration: execute_iteration(decoded); return;
    case Operation::normalize_fast: normalize_fast(opcode); return;
    case Operation::viterbi_shift: viterbi_shift(decoded); return;
    case Operation::product_immediate:
    case Operation::product_power:
    case Operation::product_mixed:
    case Operation::product_double: execute_product(decoded); return;
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
    case Operation::return_subroutine:
    case Operation::return_interrupt: execute_return(decoded); return;
    case Operation::and_control:
    case Operation::or_control: {
      const unsigned field = opcode & 3;
      auto &target = field >= 2 ? reg.omr : reg.sr;
      const unsigned shift = field == 0 || field == 3 ? control_byte_shift : 0;
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
      !interrupt_ready()) {
    throw std::invalid_argument("DSP interrupt is not admissible at this boundary");
  }
  impl_->cpu.processing = ExecutionState::running;
  impl_->vector = {
      .resume = state.pc, .address = vector, .level = level, .phase = Impl::VectorPhase::first};
  impl_->cpu.repeat_completed = false;
  while (impl_->vector.phase != Impl::VectorPhase::none && impl_->fault.kind == FaultKind::none &&
         impl_->cpu.processing == ExecutionState::running) {
    impl_->step_vector();
  }
}
} // namespace dsp56300
