// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_guard.hpp"
#include "move_storage.hpp"

#include <algorithm>
#include <span>
#include <stdexcept>
#include <tuple>

namespace dsp56300 {
using namespace detail;

bool Core::Impl::pcu_publication() const noexcept {
  return staged_computing() && staged->publication == InstructionPublication::pcu_updates &&
         fault.kind == FaultKind::none;
}

void Core::Impl::mark_pcu_field(const RegisterUpdate update) const {
  if (!pcu_publication() || update.mask == 0) { return; }
  auto &result = staged->pcu_result;
  RegisterUpdate *selected = nullptr;
  for (unsigned index = 0; index < result.count; ++index) {
    auto &field = result.updates.at(index);
    if (field.storage == update.storage && field.index == update.index) {
      selected = &field;
      break;
    }
  }
  if (selected == nullptr) {
    if (result.count == result.updates.size()) {
      throw std::overflow_error("Native PCU result capacity exhausted");
    }
    selected = &result.updates.at(result.count++);
    *selected = {.storage = update.storage, .index = update.index};
  }
  selected->previous |= move_storage(cpu.reg, *selected) & update.mask & ~selected->mask;
  selected->mask |= update.mask;
}

void Core::Impl::mark_control_transfer() const {
  if (staged_computing()) { staged->control_transfer = true; }
  mark_pcu_field({.storage = RegisterStorage::pc, .mask = word_mask});
}

void Core::Impl::publish_pcu() const {
  if (!pcu_publication() || staged->pcu_result.count == 0) { return; }
  auto result = staged->pcu_result;
  result.opcode = staged->input.opcode;
  auto fields = std::span(result.updates).first(result.count);
  std::ranges::sort(fields, [](const RegisterUpdate &left, const RegisterUpdate &right) {
    return std::tie(left.storage, left.index) < std::tie(right.storage, right.index);
  });
  for (auto &field : fields) { field.value = move_storage(cpu.reg, field) & field.mask; }
  static_cast<void>(sample_instruction({.kind = EffectKind::pcu_update, .pcu_update = result}));
  staged->pcu_result = {};
}

void Core::Impl::apply_pcu(const PcuWriteback &update) {
  for (unsigned index = 0; index < update.count; ++index) {
    merge_move_storage(cpu.reg, update.updates.at(index));
  }
}

Word Core::Impl::read_pcu_register(const unsigned number) {
  if (!pcu_publication()) { return read_register(number); }
  Word value = 0;
  {
    const ExecutionGuard guard(staged->pcu_source);
    value = read_register(number, StackOwner::pcu);
  }
  publish_pcu();
  return value;
}
} // namespace dsp56300
