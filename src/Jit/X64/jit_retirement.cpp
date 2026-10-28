// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_moves.hpp"
#include "x64.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace dsp56300::detail {
namespace {
using x64::Packed;
using x64::Reg;
using x64::Width;
using x64::Xmm;
constexpr unsigned group_add = 0, lane_sign = 63;
constexpr std::size_t minimum_packed_operations = 4;
static_assert(offsetof(Cpu, cycles) == offsetof(Cpu, instructions) + sizeof(std::uint64_t));
}

void JitEmitter::initialize_retirement() {
  if (!packed_retirement_ || !retirement_helpers_) { return; }
  code_.packed(Packed::bit_xor, Xmm::x0, Xmm::x0);
  code_.packed(Packed::equal_dword, Xmm::x1, Xmm::x1);
  code_.packed_shift_right(Xmm::x1, lane_sign);
}

void JitEmitter::materialize_retirement() {
  if (!packed_retirement_) { return; }
  if (!retirement_helpers_) {
    // Helper-free regions have no committed prefix inside the call. R15D is
    // their exact count; MOVD zero-extends it into a qword before duplication.
    code_.packed_from(Xmm::x0, Reg::r15);
    code_.packed(Packed::unpack_low_qword, Xmm::x0, Xmm::x0);
  }
  code_.packed_load(Xmm::x2, Reg::r12, offsetof(Cpu, instructions));
  code_.packed(Packed::add_qword, Xmm::x2, Xmm::x0);
  code_.packed_store(Reg::r12, offsetof(Cpu, instructions), Xmm::x2);
}

void JitEmitter::retire_progress(const NativeOp &operation) {
  if (packed_retirement_) {
    if (retirement_helpers_) { code_.packed(Packed::add_qword, Xmm::x0, Xmm::x1); }
    if (operation.instruction.cycles > 1) {
      code_.memory_immediate(group_add,
                             Reg::r12,
                             offsetof(Cpu, cycles),
                             Width::qword,
                             operation.instruction.cycles - 1);
    }
    return;
  }
  code_.memory_immediate(group_add, Reg::r12, offsetof(Cpu, instructions), Width::qword, 1);
  code_.memory_immediate(
      group_add, Reg::r12, offsetof(Cpu, cycles), Width::qword, operation.instruction.cycles);
}

bool JitEmitter::publish_pc(const std::span<const NativeOp> operations,
                            const std::size_t index) const {
  if (!packed_retirement_ || index + 1 == operations.size() || has_moves(operations[index])) {
    return true;
  }
  const auto &next = operations[index + 1];
  return has_moves(next) || at_loop_end(next);
}

// Keep admission separate for independent local performance controls; this
// changes no CPU/runtime layout or consumer option.
bool JitEmitter::admit_packed_retirement(const std::span<const NativeOp> operations) const {
  if (!optimize_ || operations.empty() ||
      (operations.size() < minimum_packed_operations && loop_size_ == 0)) {
    return false;
  }
  for (const auto &operation : operations) {
    if (!lowerable(operation) || operation.instruction.cycles == 0) { return false; }
    if (!has_moves(operation)) { continue; }
    const auto plan = plan_moves(operation.instruction);
    if (plan.count == 0) { return false; }
    for (const auto &move : std::span(plan.transfers).first(plan.count)) {
      if (!move.load && runtime_.memory.at(move.space == Space::x ? 0 : 1).aliases_program) {
        return false;
      }
    }
  }
  return true;
}
} // namespace dsp56300::detail
