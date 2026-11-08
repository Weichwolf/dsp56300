// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/emitter.hpp"
#include "jit/x64/io.hpp"

#include <cstddef>

namespace dsp56300::detail {
bool BlockEmitter::is_peripheral_register(const BlockInstruction &operation) {
  if (operation.instruction.operation != Operation::move_peripheral_register ||
      at_loop_end(operation)) {
    return false;
  }
  const auto reg = (operation.instruction.opcode >> 8U) & six_bit_mask;
  return (reg >= x0_register && reg <= y1_register) || (reg >= a_register && reg < m_register_base);
}

void BlockEmitter::peripheral_register(const BlockInstruction &operation) {
  using x64::Reg;
  using x64::Width;
  const auto opcode = operation.instruction.opcode;
  const auto reg = (opcode >> 8U) & six_bit_mask;
  const bool high = (opcode & (1U << 19U)) != 0;
  const auto index =
      high ? opcode & six_bit_mask : (opcode & five_bit_mask) | ((opcode >> 1U) & 32U);
  constexpr Word high_begin = 0xffffc0;
  const auto address = (high ? high_begin : io_begin) + index;
  const bool y = (opcode & (high ? 1U << 16U : 1U << 5U)) != 0;
  const auto space = y ? Space::y : Space::x;
  const bool write = (opcode & (1U << 15U)) != 0;
  // The callback observes the authoritative pre-effect prefix and sequential PC.
  flush_accumulators(dirty_accumulators());
  accumulator_dirty_.fill(false);
  if (write) {
    move_read_register(Reg::dx, {.address = {}, .reg = reg}, operation.mode);
  } else {
    code_.constant(Reg::dx, 0);
  }
  code_.constant(Reg::cx, address);
  x64::Label fallback;
  x64::Label done;
  native_io_access(
      code_,
      operation,
      {.space = space, .action = write ? BusAction::write : BusAction::read, .address = Reg::cx},
      {.fallback = fallback, .failure = exits_[0]});
  if (!write) {
    constexpr unsigned group_and = 4;
    code_.immediate(group_and, Reg::dx, word_mask, Width::dword);
    move_write_register(Reg::dx, reg);
  }
  retire(operation, true);
  constexpr unsigned group_cmp = 7;
  code_.memory_immediate(group_cmp, Reg::r13, offsetof(NativeContext, boundary), Width::byte, 0);
  code_.jump_if(x64::Branch::not_equal, exits_[0]);
  code_.jump(done);
  code_.bind(fallback);
  instruction_fallback(operation);
  code_.bind(done);
}
}
