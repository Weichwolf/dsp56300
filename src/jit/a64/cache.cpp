// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "cache_state.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"

#include <cstddef>
#include <cstdint>

namespace dsp56300::detail::a64 {
namespace {
constexpr auto sectors = offsetof(CacheState, sectors), order = offsetof(CacheState, order);
constexpr auto validity = offsetof(CacheSector, valid) + offsetof(CacheValidity, words);
}

void Emitter::cache_flush(const bool unlocked_only) {
  code_.load(Reg::x1, Reg::x15, offsetof(CacheState, sector_words));
  code_.constant(Reg::x2, word_mask);
  code_.divide(Reg::x2, Reg::x2, Reg::x1, false);
  for (std::size_t slot = 0; slot < cache_sectors; ++slot) {
    const auto sector = sectors + (slot * sizeof(CacheSector));
    Label next;
    if (unlocked_only) {
      code_.load(Reg::x9, Reg::x15, sector + offsetof(CacheSector, locked), Access::byte);
      code_.branch_zero_far(Reg::x9, next, Width::w32, true);
    }
    for (std::size_t offset = 0; offset < sizeof(CacheValidity); offset += sizeof(std::uint64_t)) {
      code_.store(Reg::zr, Reg::x15, sector + validity + offset, Access::doubleword);
    }
    code_.store(Reg::x2, Reg::x15, sector + offsetof(CacheSector, tag));
    code_.store(Reg::zr, Reg::x15, sector + offsetof(CacheSector, locked), Access::byte);
    code_.bind(next);
  }
  for (std::size_t slot = 0; slot < cache_sectors; ++slot) {
    code_.constant(Reg::x9, slot);
    code_.store(Reg::x9, Reg::x15, order + (slot * sizeof(std::size_t)), Access::doubleword);
  }
}

void Emitter::cache_touch() {
  code_.constant(Reg::x4, 0);
  code_.add(Reg::x12, Reg::x15, order, Width::x64);
  Label search;
  Label found;
  Label complete;
  code_.bind(search);
  code_.load(Reg::x9, Reg::x12, 0, Access::doubleword);
  code_.compare(Reg::x9, Reg::x3, Width::x64);
  code_.branch_far(Condition::equal, found);
  code_.add(Reg::x12, Reg::x12, sizeof(std::size_t), Width::x64);
  code_.add(Reg::x4, Reg::x4, 1);
  code_.constant(Reg::x17, cache_sectors);
  code_.compare(Reg::x4, Reg::x17);
  code_.branch_far(Condition::no_carry, search);
  code_.branch(complete);
  code_.bind(found);
  Label shift;
  Label front;
  code_.bind(shift);
  code_.branch_zero_far(Reg::x4, front);
  code_.constant(Reg::x17, sizeof(std::size_t));
  code_.alu(Alu::subtract, Reg::x13, Reg::x12, Reg::x17, Width::x64);
  code_.load(Reg::x9, Reg::x13, 0, Access::doubleword);
  code_.store(Reg::x9, Reg::x12, 0, Access::doubleword);
  code_.move(Reg::x12, Reg::x13);
  code_.constant(Reg::x17, 1);
  code_.alu(Alu::subtract, Reg::x4, Reg::x4, Reg::x17);
  code_.branch(shift);
  code_.bind(front);
  code_.store(Reg::x3, Reg::x15, order, Access::doubleword);
  code_.bind(complete);
}

void Emitter::cache_lock(const bool locked) {
  code_.load(Reg::x1, Reg::x15, offsetof(CacheState, sector_words));
  code_.divide(Reg::x2, Reg::x0, Reg::x1, false);
  code_.constant(Reg::x3, 0);
  code_.add(Reg::x12, Reg::x15, sectors, Width::x64);
  Label search;
  Label found;
  Label replace;
  Label complete;
  code_.bind(search);
  code_.load(Reg::x9, Reg::x12, offsetof(CacheSector, tag));
  code_.compare(Reg::x9, Reg::x2);
  code_.branch_far(Condition::equal, found);
  code_.add(Reg::x12, Reg::x12, sizeof(CacheSector), Width::x64);
  code_.add(Reg::x3, Reg::x3, 1);
  code_.constant(Reg::x17, cache_sectors);
  code_.compare(Reg::x3, Reg::x17);
  code_.branch_far(Condition::no_carry, search);
  for (std::size_t position = cache_sectors; position > 0; --position) {
    code_.load(
        Reg::x3, Reg::x15, order + ((position - 1) * sizeof(std::size_t)), Access::doubleword);
    code_.constant(Reg::x17, sizeof(CacheSector));
    code_.multiply(Reg::x12, Reg::x3, Reg::x17, Width::x64);
    code_.alu(Alu::add, Reg::x12, Reg::x15, Reg::x12, Width::x64);
    code_.add(Reg::x12, Reg::x12, sectors, Width::x64);
    code_.load(Reg::x9, Reg::x12, offsetof(CacheSector, locked), Access::byte);
    code_.branch_zero_far(Reg::x9, replace);
  }
  code_.branch(complete);
  code_.bind(replace);
  code_.store(Reg::x2, Reg::x12, offsetof(CacheSector, tag));
  for (std::size_t offset = 0; offset < sizeof(CacheValidity); offset += sizeof(std::uint64_t)) {
    code_.store(Reg::zr, Reg::x12, validity + offset, Access::doubleword);
  }
  code_.bind(found);
  code_.constant(Reg::x9, locked ? 1U : 0U);
  code_.store(Reg::x9, Reg::x12, offsetof(CacheSector, locked), Access::byte);
  cache_touch();
  code_.bind(complete);
}

bool Emitter::cache_control(const BlockInstruction &operation) {
  const auto kind = operation.instruction.operation;
  const bool relative =
      kind == Operation::cache_lock_relative || kind == Operation::cache_unlock_relative;
  const bool addressed = kind == Operation::cache_lock || kind == Operation::cache_unlock;
  if (!relative && !addressed && kind != Operation::cache_flush &&
      kind != Operation::cache_flush_unlocked && kind != Operation::cache_free) {
    return false;
  }
  Label enabled;
  Label complete;
  mask(Reg::x0, status, cache_enable, Width::w32);
  code_.branch_zero_far(Reg::x0, enabled, Width::w32, true);
  auto illegal = operation;
  illegal.instruction.operation = Operation::illegal;
  exception_request(illegal);
  code_.branch(complete);
  code_.bind(enabled);
  if (relative) {
    code_.constant(Reg::x0, (operation.pc + operation.instruction.extension) & word_mask);
  } else if (addressed) {
    const AddressForm form{.mode = (operation.instruction.opcode >> 11) & 7U,
                           .index = (operation.instruction.opcode >> 8) & 7U,
                           .extra = operation.instruction.extension};
    if (form.mode == 6 && form.index != 0) {
      code_.constant(Reg::x0, operation.instruction.opcode);
      fault(operation, FaultKind::invalid_instruction, Reg::x0);
    } else {
      effective(operation, form);
    }
  }
  code_.store(Reg::x0, Reg::sp, Frame::address);
  publish();
  constexpr auto events = offsetof(NativeContext, events);
  code_.load(Reg::x0, context, events + offsetof(NativeEvents, cache_owner), Access::doubleword);
  code_.load(Reg::x16, context, events + offsetof(NativeEvents, cache), Access::doubleword);
  code_.call(Reg::x16);
  code_.move(Reg::x15, Reg::x0);
  code_.load(Reg::x0, Reg::sp, Frame::address);
  if (kind == Operation::cache_flush || kind == Operation::cache_flush_unlocked) {
    cache_flush(kind == Operation::cache_flush_unlocked);
  } else if (kind == Operation::cache_free) {
    for (std::size_t slot = 0; slot < cache_sectors; ++slot) {
      code_.store(Reg::zr,
                  Reg::x15,
                  sectors + (slot * sizeof(CacheSector)) + offsetof(CacheSector, locked),
                  Access::byte);
    }
  } else {
    cache_lock(kind == Operation::cache_lock || kind == Operation::cache_lock_relative);
  }
  code_.bind(complete);
  boundary(NativeBoundary::owner);
  return true;
}
}
