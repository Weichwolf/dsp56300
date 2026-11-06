// SPDX-License-Identifier: GPL-3.0-only
#include "region_guards.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit.hpp"
#include "jit/code.hpp"
#include "jit/resident.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace dsp56300::detail {
namespace {
using x64::Branch;
using x64::Reg;
using x64::Width;
constexpr unsigned group_add = 0, group_and = 4, group_sub = 5, group_cmp = 7;
constexpr unsigned op_add = 0x01, op_cmp = 0x39, op_xor = 0x31, move_above = 0x47;
constexpr auto r_offset = offsetof(Cpu, reg) + offsetof(Registers, r);
constexpr auto n_offset = offsetof(Cpu, reg) + offsetof(Registers, n);
constexpr auto m_offset = offsetof(Cpu, reg) + offsetof(Registers, m);
constexpr auto lc_offset = offsetof(Cpu, reg) + offsetof(Registers, lc);
constexpr auto sc_offset = offsetof(Cpu, reg) + offsetof(Registers, sc);
constexpr auto sr_offset = offsetof(Cpu, reg) + offsetof(Registers, sr);
// The emitted admission compares twice the body length with a signed immediate.
constexpr Word maximum_operations =
    (static_cast<Word>(std::numeric_limits<std::int32_t>::max()) + 1) / 2;

void modifier_guard(x64::Code &code,
                    const Reg cpu,
                    const RegionAddress &address,
                    const std::size_t offset,
                    x64::Label &failed) {
  if (!address.updated) { return; }
  code.load(Reg::ax, cpu, m_offset + offset, Width::dword);
  code.immediate(group_and, Reg::ax, modifier_mask, Width::dword);
  code.immediate(group_cmp, Reg::ax, address.modifier, Width::dword);
  code.jump_if(Branch::not_equal, failed);
}

void single_guard(x64::Code &code,
                  const Reg cpu,
                  const RegionAddress &address,
                  const std::size_t offset,
                  x64::Label &failed) {
  modifier_guard(code, cpu, address, offset, failed);
  if (address.constant) {
    code.constant(Reg::cx, *address.constant);
  } else {
    code.load(Reg::cx, cpu, r_offset + offset, Width::dword);
  }
  if (!address.updated || address.modifier == modifier_mask) {
    if (address.stride_accesses != 0) {
      code.load(Reg::ax, cpu, n_offset + offset, Width::dword);
      code.immediate(group_cmp, Reg::ax, sign24_bit, Width::dword);
      code.jump_if(Branch::above_equal, failed);
      code.constant(Reg::r9, address.stride_accesses);
      code.multiply({.destination = Reg::ax, .source = Reg::r9});
      code.binary(op_add, {.destination = Reg::cx, .source = Reg::ax});
    }
    code.immediate(group_cmp,
                   Reg::cx,
                   address.linear_limit,
                   address.stride_accesses != 0 ? Width::qword : Width::dword);
    code.jump_if(Branch::above_equal, failed);
    return;
  }
  const auto length = address.modifier + 1;
  const auto span = std::bit_ceil(length) - 1;
  code.mov({.destination = Reg::ax, .source = Reg::cx}, Width::dword);
  code.immediate(group_and, Reg::ax, word_mask & ~span, Width::dword);
  code.immediate(group_add, Reg::ax, length, Width::dword);
  code.immediate(group_cmp, Reg::ax, address.bound + 1, Width::dword);
  code.jump_if(Branch::above_equal, failed);
  code.binary(op_cmp, {.destination = Reg::cx, .source = Reg::ax}, Width::dword);
  code.jump_if(Branch::above_equal, failed);
}

void repeated_guard(x64::Code &code,
                    const RegionAddress &address,
                    const std::size_t offset,
                    x64::Label &failed) {
  modifier_guard(code, Reg::r12, address, offset, failed);
  code.load(Reg::cx, Reg::r12, r_offset + offset, Width::dword);
  code.load(Reg::r8, Reg::sp, window_count_offset, Width::dword);
  code.immediate(group_sub, Reg::r8, 1, Width::dword);
  if (address.advances != 0) {
    code.mov({.destination = Reg::r9, .source = Reg::r8});
    code.constant(Reg::r10, address.advances);
    code.multiply({.destination = Reg::r9, .source = Reg::r10});
    code.binary(op_add, {.destination = Reg::cx, .source = Reg::r9});
  }
  if (address.stride_steps != 0 || address.stride_accesses != 0) {
    code.constant(Reg::r9, address.stride_steps);
    code.multiply({.destination = Reg::r8, .source = Reg::r9});
    code.immediate(group_add, Reg::r8, address.stride_accesses);
    code.load(Reg::ax, Reg::r12, n_offset + offset, Width::dword);
    code.immediate(group_cmp, Reg::ax, sign24_bit, Width::dword);
    code.jump_if(Branch::above_equal, failed);
    code.multiply({.destination = Reg::ax, .source = Reg::r8});
    code.binary(op_add, {.destination = Reg::cx, .source = Reg::ax});
  }
  code.immediate(group_cmp, Reg::cx, address.linear_limit);
  code.jump_if(Branch::above_equal, failed);
}
}

void emit_region_loop_guard(x64::Code &code,
                            const Reg cpu,
                            const ResidentPlan &region,
                            x64::Label &failed) {
  if (!region.loop) { return; }
  code.load(Reg::ax, cpu, lc_offset, Width::dword);
  code.immediate(group_cmp, Reg::ax, 1, Width::dword);
  code.jump_if(Branch::below, failed);
  x64::Label continuing;
  code.jump_if(Branch::above, continuing);
  code.memory_immediate(group_cmp, cpu, sc_offset, Width::dword, 2);
  code.jump_if(Branch::below, failed);
  code.load(Reg::ax, cpu, sr_offset, Width::dword);
  code.immediate(group_and, Reg::ax, forever_flag, Width::dword);
  code.jump_if(Branch::not_equal, failed);
  code.bind(continuing);
}

void emit_region_address_guards(x64::Code &code,
                                const Reg cpu,
                                const ResidentPlan &region,
                                x64::Label &failed) {
  for (std::size_t index = 0; index < region.addresses.size(); ++index) {
    const auto &address = region.addresses[index];
    if (address.accessed) { single_guard(code, cpu, address, index * sizeof(Word), failed); }
  }
}

void emit_region_window(x64::Code &code,
                        const ResidentPlan &region,
                        const Word operations,
                        x64::Label &failed) {
  if (!region.valid || !region.loop || operations == 0 || operations > maximum_operations) {
    throw std::logic_error("Unproved loop window");
  }
  code.load(Reg::cx, Reg::r12, lc_offset, Width::dword);
  code.immediate(group_cmp, Reg::cx, 2, Width::dword);
  code.jump_if(Branch::below_equal, failed);
  code.immediate(group_sub, Reg::cx, 1, Width::dword);
  code.load(Reg::ax, Reg::r13, offsetof(NativeContext, budget), Width::dword);
  code.immediate(group_cmp, Reg::ax, (2 * operations) - 1, Width::dword);
  code.jump_if(Branch::below_equal, failed);
  code.binary(op_xor, {.destination = Reg::dx, .source = Reg::dx}, Width::dword);
  code.constant(Reg::r11, operations);
  code.divide(Reg::r11);
  code.binary(op_cmp, {.destination = Reg::ax, .source = Reg::cx}, Width::dword);
  code.move_if(move_above, {.destination = Reg::ax, .source = Reg::cx});
  code.store(Reg::sp, window_count_offset, Reg::ax, Width::dword);
  for (std::size_t index = 0; index < region.addresses.size(); ++index) {
    const auto &address = region.addresses[index];
    if (!address.accessed) { continue; }
    if (address.updated && address.modifier == modifier_mask && !address.constant) {
      repeated_guard(code, address, index * sizeof(Word), failed);
    } else {
      single_guard(code, Reg::r12, address, index * sizeof(Word), failed);
    }
  }
}
} // namespace dsp56300::detail
