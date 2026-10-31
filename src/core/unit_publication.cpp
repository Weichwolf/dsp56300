// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "move_storage.hpp"

#include <algorithm>
#include <span>
#include <stdexcept>
#include <tuple>

namespace dsp56300 {
using namespace detail;

bool Core::Impl::unit_publication() const noexcept {
  return staged_computing() && staged->publication == InstructionPublication::unit_updates &&
         fault.kind == FaultKind::none;
}

void Core::Impl::begin_unit(const RegisterUnit unit) const {
  if (!unit_publication()) { return; }
  staged->unit_active = true;
  staged->unit_result.unit = unit;
}

void Core::Impl::mark_unit_field(const RegisterUpdate update) const {
  if (!unit_publication() || !staged->unit_active || update.mask == 0) { return; }
  auto &result = staged->unit_result;
  RegisterUpdate *selected = nullptr;
  for (unsigned position = 0; position < result.count; ++position) {
    auto &field = result.updates.at(position);
    if (field.storage == update.storage && field.index == update.index) {
      selected = &field;
      break;
    }
  }
  if (selected == nullptr) {
    if (result.count == result.updates.size()) {
      throw std::overflow_error("Native scalar-unit result capacity exhausted");
    }
    selected = &result.updates.at(result.count++);
    *selected = {.storage = update.storage, .index = update.index};
  }
  selected->previous |= move_storage(cpu.reg, *selected) & update.mask & ~selected->mask;
  selected->mask |= update.mask;
}

Word Core::Impl::read_unit_register(const unsigned number) {
  if (number == ssh_register) {
    mark_unit_field({.storage = RegisterStorage::sp, .mask = (stack_entries * 4) - 1});
    mark_unit_field({.storage = RegisterStorage::sc, .mask = (stack_entries * 2) - 1});
  }
  return read_register(number);
}

void Core::Impl::write_unit_register(const unsigned number, const Word value) {
  if (unit_publication() && staged->unit_active) {
    const auto destination = move_destination(number);
    for (unsigned index = 0; index < destination.count; ++index) {
      mark_unit_field(destination.updates.at(index));
    }
  }
  write_register(number, value);
}

void Core::Impl::publish_unit() const {
  if (!unit_publication() || !staged->unit_active) { return; }
  auto result = staged->unit_result;
  if (result.count != 0) {
    result.opcode = staged->input.opcode;
    auto fields = std::span(result.updates).first(result.count);
    std::ranges::sort(fields, [](const RegisterUpdate &left, const RegisterUpdate &right) {
      return std::tie(left.storage, left.index) < std::tie(right.storage, right.index);
    });
    for (auto &field : fields) { field.value = move_storage(cpu.reg, field) & field.mask; }
    static_cast<void>(sample_instruction({.kind = EffectKind::unit_update, .unit_update = result}));
  }
  staged->unit_result = {};
  staged->unit_active = false;
}

void Core::Impl::apply_unit(const UnitWriteback &update) {
  for (unsigned index = 0; index < update.count; ++index) {
    merge_move_storage(cpu.reg, update.updates.at(index));
  }
}
} // namespace dsp56300
