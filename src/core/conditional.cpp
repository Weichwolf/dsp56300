// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "conditions.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

namespace dsp56300 {
using namespace detail;

namespace {
constexpr Word short_address_mask = 0xfff;
constexpr unsigned condition_shift = 12;
constexpr Word call_bit = Word{1} << 16;
constexpr Word short_jump_bit = Word{1} << 18;
constexpr Word condition_update_bit = Word{1} << 12;

struct JumpForm {
  bool short_address = false, call = false, conditional = false;
};

JumpForm jump_form(const Instruction &decoded) {
  const auto operation = decoded.operation;
  if (operation == Operation::conditional_jump) {
    return {.short_address = (decoded.opcode & short_jump_bit) != 0,
            .call = (decoded.opcode & call_bit) != 0,
            .conditional = true};
  }
  return {.short_address = operation == Operation::jump_short || operation == Operation::call_short,
          .call = operation == Operation::call_short || operation == Operation::call_effective};
}
}

Word Core::Impl::control_address(const Instruction &decoded) {
  const AddressForm form{.mode = (decoded.opcode >> 11) & 7,
                         .index = (decoded.opcode >> 8) & 7,
                         .extra = decoded.extension};
  // The immediate-data EA is not a memory-alterable control-flow address.
  if (form.mode == 6 && form.index != 0) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
    return 0;
  }
  return effective(form);
}

void Core::Impl::execute_jump(const Instruction &decoded) {
  const auto form = jump_form(decoded);
  const auto address =
      form.short_address ? decoded.opcode & short_address_mask : control_address(decoded);
  if (fault.kind != FaultKind::none) { return; }
  const unsigned selector =
      form.short_address ? (decoded.opcode >> condition_shift) & 15 : decoded.opcode & 15;
  // Address calculation/update occurs even when the conditional jump is not taken.
  if (form.conditional && !condition({.status = cpu.reg.sr, .selector = selector})) { return; }
  if (form.call) { push(cpu.reg.pc, cpu.reg.sr); }
  cpu.reg.pc = address;
}

void Core::Impl::execute_branch(const Instruction &decoded) {
  constexpr Word short_form_mask = 0xff0000;
  constexpr Word short_form = 0x050000;
  constexpr Word short_unconditional = Word{1} << 11;
  constexpr Word long_unconditional = Word{1} << 7;
  constexpr Word short_no_call = Word{1} << 10;
  constexpr Word long_no_call = Word{1} << 6;
  constexpr Word register_form_bit = Word{1} << 11;
  const auto opcode = decoded.opcode;
  const bool short_offset = (opcode & short_form_mask) == short_form;
  const bool unconditional =
      (opcode & (short_offset ? short_unconditional : long_unconditional)) != 0;
  const unsigned selector = short_offset ? (opcode >> condition_shift) & 15 : opcode & 15;
  if (!unconditional && !condition({.status = cpu.reg.sr, .selector = selector})) { return; }
  Word offset = decoded.extension;
  if (short_offset) {
    constexpr Word upper_offset_mask = 0x1e0;
    constexpr Word lower_offset_mask = 0x1f;
    offset = ((opcode >> 1) & upper_offset_mask) | (opcode & lower_offset_mask);
    constexpr Word offset_sign = Word{1} << 8;
    constexpr Word offset_mask = (offset_sign * 2) - 1;
    if ((offset & offset_sign) != 0) { offset |= word_mask & ~offset_mask; }
  } else if ((opcode & register_form_bit) != 0) {
    offset = cpu.reg.r[(opcode >> 8) & 7];
  }
  if ((opcode & (short_offset ? short_no_call : long_no_call)) == 0) {
    push(cpu.reg.pc, cpu.reg.sr);
  }
  cpu.reg.pc = address_add(current_pc, signed_word(offset));
}

void Core::Impl::conditional_transfer(const Word opcode) {
  if (!condition({.status = cpu.reg.sr, .selector = (opcode >> condition_shift) & 15})) { return; }
  constexpr Word address_only_bit = Word{1} << 11;
  constexpr Word dual_bit = Word{1} << 16;
  if ((opcode & address_only_bit) == 0) {
    if (!numeric_mode_supported(arithmetic_saturation)) { return; }
    const unsigned selector = (opcode >> 4) & 7;
    if (selector != 0 && selector < 4) {
      fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
      return;
    }
    const bool destination_b = (opcode & 8U) != 0;
    latched = {cpu.reg.a, cpu.reg.b, cpu.reg.x, cpu.reg.y};
    mark_alu_accumulator(destination_b);
    (destination_b ? cpu.reg.b : cpu.reg.a) = source(selector, destination_b);
  }
  if ((opcode & (address_only_bit | dual_bit)) != 0) {
    mark_alu_address(opcode & 7);
    cpu.reg.r[opcode & 7] = cpu.reg.r[(opcode >> 8) & 7];
  }
}

void Core::Impl::conditional_alu(const Word opcode) {
  const bool supported = (cpu.reg.sr & arithmetic_saturation) != 0
                             ? numeric_alu_supported(opcode & byte_mask)
                             : numeric_mode_supported();
  if (!supported) { return; }
  const auto status = cpu.reg.sr;
  if (!condition({.status = status, .selector = (opcode >> 8) & 15})) { return; }
  latched = {cpu.reg.a, cpu.reg.b, cpu.reg.x, cpu.reg.y};
  alu(opcode & byte_mask);
  if ((opcode & condition_update_bit) == 0) {
    cpu.reg.sr = status;
    omit_alu_status(word_mask);
  }
}
} // namespace dsp56300
