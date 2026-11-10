// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/resident.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace dsp56300::detail::a64 {
void Emitter::guard_storage() {
  code_.load(Reg::x9, cpu, offsetof(Cpu, repeat_remaining));
  code_.branch_zero_far(Reg::x9, done_, Width::w32, true);
  for (const auto &memory : runtime_.memory) {
    if (memory.data == nullptr || memory.words == 0) { continue; }
    const auto begin = reinterpret_cast<std::uintptr_t>(memory.data);
    const auto length = std::min(memory.words, std::size_t{io_begin}) * sizeof(Word);
    for (const auto &[object, size] :
         {std::pair{cpu, sizeof(Cpu)}, std::pair{context, sizeof(NativeContext)}}) {
      Label disjoint;
      code_.constant(Reg::x9, begin);
      code_.constant(Reg::x10, size);
      code_.alu(Alu::add, Reg::x10, object, Reg::x10, Width::x64);
      code_.compare(Reg::x9, Reg::x10, Width::x64);
      code_.branch_far(Condition::carry, disjoint);
      code_.constant(Reg::x9, begin + length);
      code_.compare(object, Reg::x9, Width::x64);
      code_.branch_far(Condition::no_carry, done_);
      code_.bind(disjoint);
    }
  }
}

void Emitter::guard_loop(Label &failed) {
  if (!region_.loop) { return; }
  code_.load(Reg::x9, cpu, offsetof(Registers, lc));
  code_.constant(Reg::x17, 1);
  code_.compare(Reg::x9, Reg::x17);
  code_.branch_far(Condition::no_carry, failed);
  Label continuing;
  code_.branch_far(Condition::above, continuing);
  code_.load(Reg::x9, cpu, offsetof(Registers, sc));
  code_.constant(Reg::x17, 2);
  code_.compare(Reg::x9, Reg::x17);
  code_.branch_far(Condition::no_carry, failed);
  mask(Reg::x9, status, forever_flag, Width::w32);
  code_.branch_zero_far(Reg::x9, failed, Width::w32, true);
  code_.bind(continuing);
}

void Emitter::guard_signed_address(const RegionAddress &address,
                                   const bool repeated,
                                   Label &failed) {
  const bool varying = repeated && !address.constant && address.displacement != 0;
  if (varying) {
    code_.load(Reg::x10, Reg::sp, Frame::iterations);
    code_.subtract(Reg::x10, Reg::x10, 1, Width::x64);
    code_.constant(Reg::x11, static_cast<std::uint64_t>(address.displacement));
    code_.multiply(Reg::x10, Reg::x10, Reg::x11, Width::x64);
  }
  code_.constant(Reg::x17, static_cast<std::uint64_t>(address.minimum));
  code_.alu(Alu::add, Reg::x11, Reg::x9, Reg::x17, Width::x64);
  if (varying && address.displacement < 0) {
    code_.alu(Alu::add, Reg::x11, Reg::x11, Reg::x10, Width::x64);
  }
  code_.compare(Reg::x11, Reg::zr, Width::x64);
  code_.branch_far(Condition::less, failed);
  code_.constant(Reg::x17, static_cast<std::uint64_t>(address.maximum));
  code_.alu(Alu::add, Reg::x11, Reg::x9, Reg::x17, Width::x64);
  if (varying && address.displacement > 0) {
    code_.alu(Alu::add, Reg::x11, Reg::x11, Reg::x10, Width::x64);
  }
  code_.constant(Reg::x17, address.bound);
  code_.compare(Reg::x11, Reg::x17, Width::x64);
  code_.branch_far(Condition::carry, failed);
}

void Emitter::guard_address(const std::size_t index, const bool repeated, Label &failed) {
  const auto &address = region_.addresses.at(index);
  const auto offset = index * sizeof(Word);
  if (address.updated) {
    code_.load(Reg::x9, cpu, offsetof(Registers, m) + offset);
    mask(Reg::x9, Reg::x9, modifier_mask);
    code_.constant(Reg::x17, address.modifier);
    code_.compare(Reg::x9, Reg::x17);
    code_.branch_far(Condition::not_equal, failed);
  }
  if (address.constant) {
    code_.constant(Reg::x9, *address.constant);
  } else {
    code_.load(Reg::x9, cpu, offsetof(Registers, r) + offset);
  }
  if (address.retreats) {
    guard_signed_address(address, repeated, failed);
    return;
  }
  if (address.updated && address.modifier != modifier_mask) {
    const auto length = address.modifier + 1;
    const auto span = std::bit_ceil(length) - 1;
    mask(Reg::x10, Reg::x9, word_mask & ~span);
    code_.constant(Reg::x17, length);
    code_.alu(Alu::add, Reg::x10, Reg::x10, Reg::x17);
    code_.constant(Reg::x17, address.bound);
    code_.compare(Reg::x10, Reg::x17);
    code_.branch_far(Condition::above, failed);
    code_.compare(Reg::x9, Reg::x10);
    code_.branch_far(Condition::carry, failed);
    return;
  }
  const bool window = repeated && address.updated && !address.constant;
  if (window) {
    code_.load(Reg::x10, Reg::sp, Frame::iterations);
    code_.constant(Reg::x17, 1);
    code_.alu(Alu::subtract, Reg::x10, Reg::x10, Reg::x17);
    code_.constant(Reg::x17, address.advances);
    code_.multiply(Reg::x11, Reg::x10, Reg::x17, Width::x64);
    code_.alu(Alu::add, Reg::x9, Reg::x9, Reg::x11, Width::x64);
  }
  if (address.stride_accesses != 0 || (window && address.stride_steps != 0)) {
    code_.load(Reg::x11, cpu, offsetof(Registers, n) + offset);
    code_.constant(Reg::x17, sign24_bit);
    code_.compare(Reg::x11, Reg::x17);
    code_.branch_far(Condition::carry, failed);
    if (window) {
      code_.constant(Reg::x17, address.stride_steps);
      code_.multiply(Reg::x10, Reg::x10, Reg::x17, Width::x64);
      code_.constant(Reg::x17, address.stride_accesses);
      code_.alu(Alu::add, Reg::x10, Reg::x10, Reg::x17, Width::x64);
    } else {
      code_.constant(Reg::x10, address.stride_accesses);
    }
    code_.multiply(Reg::x11, Reg::x11, Reg::x10, Width::x64);
    code_.alu(Alu::add, Reg::x9, Reg::x9, Reg::x11, Width::x64);
  }
  code_.constant(Reg::x17, address.linear_limit);
  code_.compare(Reg::x9, Reg::x17, Width::x64);
  code_.branch_far(Condition::carry, failed);
}

void Emitter::guard_addresses(const bool repeated, Label &failed) {
  for (std::size_t index = 0; index < region_.addresses.size(); ++index) {
    if (region_.addresses[index].accessed) { guard_address(index, repeated, failed); }
  }
}

void Emitter::guard_iterations(const std::span<const BlockInstruction> operations) {
  Label single;
  Label admitted;
  if (region_.loop) {
    mask(Reg::x9, status, forever_flag, Width::w32);
    code_.branch_zero_far(Reg::x9, single, Width::w32, true);
    code_.load(Reg::x9, cpu, offsetof(Registers, lc));
    code_.constant(Reg::x17, word_mask);
    code_.compare(Reg::x9, Reg::x17);
    code_.branch_far(Condition::above, single);
    code_.constant(Reg::x17, 2);
    code_.compare(Reg::x9, Reg::x17);
    code_.branch_far(Condition::below_equal, single);
    code_.constant(Reg::x17, 1);
    code_.alu(Alu::subtract, Reg::x12, Reg::x9, Reg::x17);
    code_.constant(Reg::x17, operations.size());
    code_.divide(Reg::x13, remaining, Reg::x17, false);
    code_.compare(Reg::x13, Reg::x12);
    code_.select(Reg::x12, Reg::x13, Reg::x12, Condition::no_carry);
    code_.constant(Reg::x17, 2);
    code_.compare(Reg::x12, Reg::x17);
    code_.branch_far(Condition::no_carry, single);
    code_.store(Reg::x12, Reg::sp, Frame::iterations);
    stack_slot(false);
    code_.load(Reg::x9, Reg::x11, offsetof(Registers, stack), Access::doubleword);
    code_.shift(Shift::right, Reg::x9, Reg::x9, word_bits, Width::x64);
    code_.constant(Reg::x17, operations.front().pc);
    code_.compare(Reg::x9, Reg::x17, Width::x64);
    code_.branch_far(Condition::not_equal, single);
    guard_addresses(true, single);
    code_.branch(admitted);
  }
  code_.bind(single);
  code_.constant(Reg::x9, 1);
  code_.store(Reg::x9, Reg::sp, Frame::iterations);
  guard_addresses(false, done_);
  code_.bind(admitted);
}
}
