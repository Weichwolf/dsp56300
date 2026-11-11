// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "io.hpp"

#include <cstdint>

namespace dsp56300 {
using namespace detail;

namespace {
Word modified_bit(const Word original, const Word mask, const BitAction action) noexcept {
  switch (action) {
    case BitAction::clear: return original & ~mask;
    case BitAction::set: return original | mask;
    case BitAction::change: return original ^ mask;
    case BitAction::test:
    default: return original;
  }
}
}

Word Core::Impl::bit_address(const BitOperand &operand, const Instruction &decoded) {
  switch (operand.location) {
    case BitLocation::short_address: return operand.selector;
    case BitLocation::port_high: {
      constexpr Word high_begin = 0xffffc0;
      return high_begin + operand.selector;
    }
    case BitLocation::port_low: return io_begin + operand.selector;
    case BitLocation::effective: {
      if (!numeric_mode_supported(arithmetic_saturation)) { return 0; }
      const AddressForm form{
          .mode = operand.selector >> 3, .index = operand.selector & 7, .extra = decoded.extension};
      if (form.mode == 6 && (form.index != 0 || decoded.operation != Operation::bit_memory)) {
        fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
        return 0;
      }
      return effective(form);
    }
    case BitLocation::reg:
    default: return 0;
  }
}

bool Core::Impl::register_bit(const BitAccess &access) {
  const auto number = access.operand.selector;
  const bool modify = access.action != BitAction::test;
  const bool stack_high = number == ssh_register && modify;
  if ((number == a_register || number == b_register) &&
      !numeric_mode_supported(arithmetic_saturation)) {
    return false;
  }
  const auto original =
      stack_high ? static_cast<Word>(cpu.reg.stack[cpu.reg.sp & (stack_entries - 1)] >> word_bits)
                 : read_register(number);
  if (fault.kind != FaultKind::none) { return false; }
  const Word mask = Word{1} << access.bit;
  if (modify) {
    const auto value = modified_bit(original, mask, access.action);
    if (stack_high) {

      auto &frame = cpu.reg.stack[cpu.reg.sp & (stack_entries - 1)];
      frame = (std::uint64_t{value} << word_bits) | (frame & word_mask);
    } else {
      write_register(number, value);
    }
  }
  return (original & mask) != 0;
}

bool Core::Impl::access_bit(const Instruction &decoded, const BitAction action) {
  const auto bit = decoded.opcode & five_bit_mask;
  if (bit >= word_bits) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
    return false;
  }
  const auto operand = bit_operand(decoded);
  if (operand.location == BitLocation::reg) {
    return register_bit({.operand = operand, .action = action, .bit = bit});
  }
  const auto address = bit_address(operand, decoded);
  if (fault.kind != FaultKind::none) { return false; }
  if (address >= io_begin) {

    bool previous = false;
    if (io == nullptr ||
        !dsp56300_io_bit(*io, operand.space, address, current_pc, bit, action, previous)) {
      fail(FaultKind::io_bit, operand.space, address, bit);
    }
    image_io_completed();
    return previous;
  }
  const auto original = read_memory(operand.space, address);
  if (fault.kind != FaultKind::none) { return false; }
  const Word mask = Word{1} << bit;
  if (action != BitAction::test) {
    write_memory(operand.space, address, modified_bit(original, mask, action));
  }
  return (original & mask) != 0;
}

void Core::Impl::execute_bit(const Instruction &decoded) {

  const bool branching =
      decoded.operation == Operation::bit_jump || decoded.operation == Operation::bit_branch;
  const auto action = branching ? BitAction::test : bit_action(decoded.opcode);
  const bool previous = access_bit(decoded, action);
  if (fault.kind != FaultKind::none) { return; }
  if (branching) {
    // Native source pop/limiting precedes the PCU call/branch, including its faults.

    if (previous != ((decoded.opcode & (1U << 5)) != 0)) { return; }
    if (bit_call(decoded)) {
      push(cpu.reg.pc, cpu.reg.sr);
      if (fault.kind != FaultKind::none) { return; }
    }

    cpu.reg.pc = decoded.operation == Operation::bit_jump
                     ? decoded.extension
                     : address_add(current_pc, signed_word(decoded.extension));
    return;
  }
  const auto operand = bit_operand(decoded);
  if (operand.location != BitLocation::reg || operand.selector != sr_register ||
      action == BitAction::test) {

    cpu.reg.sr = (cpu.reg.sr & ~carry) | (previous ? carry : 0);
  }
}
} // namespace dsp56300
