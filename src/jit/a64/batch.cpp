// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/mac_batch.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail::a64 {
std::vector<std::uint8_t> select_mac_batch(const BlockInstruction &operation,
                                           const std::span<const std::uint8_t> fallback) {
  const auto call = compile_mac_batch(operation);
  if (!call) { return {fallback.begin(), fallback.end()}; }
  Code code;
  Label fallback_entry;
  Label ordinary;
  code.load(Reg::x9, Reg::x0, offsetof(Registers, lc));
  code.constant(Reg::x10, mac_batch_width);
  code.compare(Reg::x9, Reg::x10);
  code.branch_far(Condition::below_equal, fallback_entry);
  code.load(Reg::x9, Reg::x1, offsetof(NativeContext, budget));
  code.compare(Reg::x9, Reg::x10);
  code.branch_far(Condition::no_carry, fallback_entry);
  code.push(Reg::fp, Reg::lr);
  code.add(Reg::fp, Reg::sp, 0, Width::x64);
  code.push(Reg::x19, Reg::x20);
  code.move(Reg::x19, Reg::x0);
  code.move(Reg::x20, Reg::x1);
  code.constant(Reg::x2, reinterpret_cast<std::uintptr_t>(&operation));
  code.constant(Reg::x3, call->operands);
  code.constant(Reg::x16, std::bit_cast<std::uintptr_t>(call->entry));
  code.call(Reg::x16);
  code.branch_zero(Reg::x0, ordinary);
  code.pop(Reg::x19, Reg::x20);
  code.pop(Reg::fp, Reg::lr);
  code.return_();
  code.bind(ordinary);
  code.move(Reg::x0, Reg::x19);
  code.move(Reg::x1, Reg::x20);
  code.pop(Reg::x19, Reg::x20);
  code.pop(Reg::fp, Reg::lr);
  code.bind(fallback_entry);
  code.append(fallback);
  return code.bytes();
}
}
