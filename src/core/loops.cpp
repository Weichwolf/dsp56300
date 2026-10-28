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
constexpr Word loop_status = loop_flag | forever_flag;
constexpr Word repeat_zero_count = 65536;
}

void Core::Impl::restore_loop() {
  const auto saved = pop();
  const auto previous = pop();
  cpu.reg.sr = (cpu.reg.sr & ~loop_status) | (static_cast<Word>(saved) & loop_status);
  cpu.reg.la = static_cast<Word>(previous >> word_bits);
  cpu.reg.lc = static_cast<Word>(previous) & word_mask;
}

void Core::Impl::finish_loop() {
  auto &reg = cpu.reg;
  const auto last = (current_pc + current_words - 1) & word_mask;
  const auto next = (reg.la + 1) & word_mask;
  if ((reg.sr & loop_flag) == 0 || last != reg.la || reg.pc != next || cpu.repeat_remaining != 0) {
    return;
  }
  if ((reg.sr & compatibility_mode) != 0) {
    fail(FaultKind::unsupported_mode, Space::program, current_pc, reg.sr);
    return;
  }
  if (reg.lc != 1 || (reg.sr & forever_flag) != 0) {
    reg.lc = address_add(reg.lc, -1);
    reg.pc = static_cast<Word>(reg.stack[reg.sp & (stack_entries - 1)] >> word_bits);
    return;
  }
  restore_loop();
}

Word Core::Impl::loop_count(const Instruction &decoded) {
  const auto operation = decoded.operation;
  const auto opcode = decoded.opcode;
  if (operation == Operation::loop_immediate || operation == Operation::repeat_immediate) {
    return ((opcode >> 8) & byte_mask) | ((opcode & 15U) << 8);
  }
  if (!numeric_mode_supported()) { return 0; }
  if (operation == Operation::loop_register || operation == Operation::repeat_register) {
    const auto selector = (opcode >> 8) & six_bit_mask;
    // Unlike REP SSH, DO/DOR cannot pop their own just-created loop frame.
    if (operation == Operation::loop_register && selector == ssh_register) {
      fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
      return 0;
    }
    return read_register(selector);
  }
  Word address = (opcode >> 8) & six_bit_mask;
  if (operation == Operation::loop_effective || operation == Operation::repeat_effective) {
    const auto mode = (opcode >> 11) & 7U;
    // These count operands have no address-extension word; mode 110 is reserved.
    if (mode == 6) {
      fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
      return 0;
    }
    address = effective({.mode = mode, .index = (opcode >> 8) & 7U});
    if (fault.kind != FaultKind::none) { return 0; }
  }
  return read_memory((opcode & (1U << 6)) != 0 ? Space::y : Space::x, address);
}

void Core::Impl::execute_loop(const Instruction &decoded) {
  auto &reg = cpu.reg;
  const auto operation = decoded.operation;
  if (operation == Operation::end_loop || operation == Operation::break_loop) {
    if (operation == Operation::break_loop) {
      if (!condition({.status = reg.sr, .selector = decoded.opcode & 15U})) { return; }
      reg.pc = (reg.la + 1) & word_mask;
    }
    restore_loop();
    return;
  }
  if (is_repeat(operation)) {
    const auto saved_count = reg.lc;
    const auto count = loop_count(decoded);
    if (fault.kind != FaultKind::none) { return; }
    cpu.repeat_remaining = count == 0 ? repeat_zero_count : count;
    cpu.repeat_pc = reg.pc;
    cpu.repeat_lc = saved_count;
    cpu.repeat_loaded = false;
    reg.lc = count;
    return;
  }
  if ((reg.sr & compatibility_mode) != 0) {
    fail(FaultKind::unsupported_mode, Space::program, current_pc, reg.sr);
    return;
  }
  const bool forever = operation == Operation::loop_forever;
  const bool relative = forever ? (decoded.opcode & 1U) == 0 : (decoded.opcode & 16U) != 0;
  const auto last =
      relative ? address_add(current_pc, signed_word(decoded.extension)) : decoded.extension;
  push(reg.la, reg.lc);
  if (fault.kind != FaultKind::none) { return; }
  // The count is sampled after the first push: DO SP/SC see its increment and
  // DO SSL reads the saved outer LC, as specified by the instruction sequence.
  const auto count = forever ? reg.lc : loop_count(decoded);
  if (fault.kind != FaultKind::none) { return; }
  if (!forever && count == 0) {
    static_cast<void>(pop());
    reg.pc = (last + 1) & word_mask;
    return;
  }
  reg.lc = count;
  push(reg.pc, reg.sr);
  if (fault.kind != FaultKind::none) { return; }
  reg.la = last;
  reg.sr = (reg.sr & ~forever_flag) | loop_flag | (forever ? forever_flag : 0U);
}
} // namespace dsp56300
