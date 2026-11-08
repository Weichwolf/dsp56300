// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>

namespace dsp56300::detail::a64 {
void Emitter::memory_flat(const BusOperand operand, const MemoryLabels labels) {
  const auto offset = operand.space == Space::program
                          ? offsetof(NativeContext, bus) + offsetof(NativeBus, program)
                          : offsetof(NativeContext, runtime) + offsetof(JitRuntime, memory) +
                                ((operand.space == Space::x ? 0U : 1U) * sizeof(JitMemory));
  code_.load(Reg::x12, context, offset + offsetof(JitMemory, data), Access::doubleword);
  code_.branch_zero_far(Reg::x12, labels.unmapped, Width::x64);
  code_.load(Reg::x9, context, offset + offsetof(JitMemory, words), Access::doubleword);
  code_.compare(Reg::x0, Reg::x9, Width::x64);
  code_.branch_far(Condition::carry, labels.unmapped);
  code_.shift(Shift::left, Reg::x9, Reg::x0, 2, Width::x64);
  code_.alu(Alu::add, Reg::x12, Reg::x12, Reg::x9, Width::x64);
  code_.constant(Reg::x13, 0);
  if (operand.space == Space::program) {
    code_.constant(Reg::x14, 1);
  } else {
    code_.load(Reg::x14, context, offset + offsetof(JitMemory, aliases_program), Access::byte);
  }
  code_.branch(labels.mapped);
}

void Emitter::mark_program_written() {
  code_.constant(Reg::x17, 1);
  code_.store(Reg::x17, context, offsetof(NativeContext, program_written), Access::byte);
  code_.store(Reg::x17, context, offsetof(NativeContext, boundary), Access::byte);
}

void Emitter::memory(const BlockInstruction &operation, const BusOperand operand) {
  if (guarded()) {
    guarded_memory(operand);
    return;
  }
  Label mapped;
  Label owned;
  Label invalid;
  Label complete;
  Label flat;
  constexpr auto bus = offsetof(NativeContext, bus);
  if (failure_ != nullptr) {
    Label permitted;
    code_.load(Reg::x9, context, bus + offsetof(NativeBus, external), Access::byte);
    code_.branch_zero_far(Reg::x9, permitted);
    code_.load(
        Reg::x9, context, offsetof(NativeContext, failure) + offsetof(Fault, kind), Access::byte);
    code_.branch_zero_far(Reg::x9, permitted);
    if (!operand.write) { code_.constant(Reg::x0, 0); }
    code_.branch(*failure_);
    code_.bind(permitted);
  }
  if (operand.space != Space::program) {
    code_.constant(Reg::x17, io_begin);
    code_.compare(Reg::x0, Reg::x17);
    code_.branch_far(Condition::carry, owned);
  } else {
    Label uncached;
    mask(Reg::x9, status, cache_enable, Width::w32);
    code_.branch_zero_far(Reg::x9, uncached);
    code_.load(Reg::x9, context, bus + offsetof(NativeBus, cache) + offsetof(CacheRegion, begin));
    code_.compare(Reg::x0, Reg::x9);
    code_.branch_far(Condition::no_carry, uncached);
    code_.load(Reg::x9, context, bus + offsetof(NativeBus, cache) + offsetof(CacheRegion, end));
    code_.compare(Reg::x0, Reg::x9);
    code_.branch_far(Condition::no_carry, owned);
    code_.bind(uncached);
  }
  code_.load(Reg::x9, context, bus + offsetof(NativeBus, external), Access::byte);
  code_.branch_zero_far(Reg::x9, flat);
  if (operand.space == Space::program) {
    code_.branch(owned);
  } else {
    memory_windows(operand, {.mapped = mapped, .unmapped = owned});
  }
  code_.bind(flat);
  memory_flat(operand, {.mapped = mapped, .unmapped = invalid});
  code_.bind(mapped);
  if (operand.write) {
    code_.store(Reg::x1, Reg::x12, 0);
    Label initialized;
    code_.branch_zero_far(Reg::x13, initialized, Width::x64);
    code_.constant(Reg::x17, 1);
    code_.store(Reg::x17, Reg::x13, 0, Access::byte);
    code_.bind(initialized);
    code_.branch_zero_far(Reg::x14, complete);
    if (operand.space != Space::program) {
      constexpr auto program = bus + offsetof(NativeBus, program);
      code_.load(Reg::x9, context, program + offsetof(JitMemory, data), Access::doubleword);
      code_.alu(Alu::subtract, Reg::x9, Reg::x12, Reg::x9, Width::x64);
      code_.load(Reg::x10, context, program + offsetof(JitMemory, words), Access::doubleword);
      code_.shift(Shift::left, Reg::x10, Reg::x10, 2, Width::x64);
      code_.compare(Reg::x9, Reg::x10, Width::x64);
      code_.branch_far(Condition::carry, complete);
    }
    mark_program_written();
  } else {
    code_.load(Reg::x0, Reg::x12, 0);
    mask(Reg::x0, Reg::x0, word_mask, Width::w32);
  }
  code_.branch(complete);
  code_.bind(owned);
  memory_owned(operation, operand);
  code_.branch(complete);
  code_.bind(invalid);
  memory_fault(operation, operand);
  code_.bind(complete);
}
}
