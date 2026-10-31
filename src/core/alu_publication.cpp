// SPDX-License-Identifier: GPL-3.0-only
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "move_storage.hpp"

#include <algorithm>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <tuple>

namespace dsp56300 {
bool Core::Impl::alu_publication() const noexcept {
  return staged_computing() &&
         (staged->publication == InstructionPublication::alu_updates ||
          staged->publication == InstructionPublication::unit_updates) &&
         fault.kind == FaultKind::none;
}

void Core::Impl::mark_alu(const RegisterStorage storage, const std::uint64_t mask) const {
  mark_alu_field({.storage = storage, .mask = mask});
}

void Core::Impl::mark_alu_address(const unsigned index) const {
  mark_alu_field({.storage = RegisterStorage::r,
                  .index = static_cast<std::uint8_t>(index),
                  .mask = word_mask});
}

void Core::Impl::mark_alu_field(const RegisterUpdate update) const {
  const auto storage = update.storage;
  const auto mask = update.mask;
  const auto index = update.index;
  if (!alu_publication() || mask == 0) { return; }
  auto &result = staged->alu_result;
  RegisterUpdate *selected = nullptr;
  for (unsigned position = 0; position < result.count; ++position) {
    auto &field = result.updates.at(position);
    if (field.storage == storage && field.index == index) {
      selected = &field;
      break;
    }
  }
  if (selected == nullptr) {
    if (result.count == result.updates.size()) {
      throw std::overflow_error("Native ALU result capacity exhausted");
    }
    selected = &result.updates.at(result.count++);
    *selected = {.storage = storage, .index = static_cast<std::uint8_t>(index)};
  }
  // Preserve the actual pre-write bits on their first native write, not a diff.
  selected->previous |= detail::move_storage(cpu.reg, *selected) & mask & ~selected->mask;
  selected->mask |= mask;
}

void Core::Impl::mark_alu_accumulator(const bool destination_b, const std::uint64_t mask) const {
  mark_alu(destination_b ? RegisterStorage::b : RegisterStorage::a, mask);
}

void Core::Impl::omit_alu_status(const Word mask) const {
  if (!alu_publication()) { return; }
  auto &result = staged->alu_result;
  for (unsigned index = 0; index < result.count; ++index) {
    auto &field = result.updates.at(index);
    if (field.storage != RegisterStorage::sr) { continue; }
    field.mask &= ~std::uint64_t{mask};
    field.previous &= field.mask;
    if (field.mask == 0) {
      for (unsigned next = index + 1; next < result.count; ++next) {
        result.updates.at(next - 1) = result.updates.at(next);
      }
      result.updates.at(--result.count) = {};
    }
    return;
  }
}

void Core::Impl::publish_alu() const {
  if (!alu_publication() || staged->alu_result.count == 0) { return; }
  auto result = staged->alu_result;
  result.opcode = staged->input.opcode;
  auto fields = std::span(result.updates).first(result.count);
  std::ranges::sort(fields, [](const RegisterUpdate &left, const RegisterUpdate &right) {
    return std::tie(left.storage, left.index) < std::tie(right.storage, right.index);
  });
  for (auto &field : fields) { field.value = detail::move_storage(cpu.reg, field) & field.mask; }
  static_cast<void>(sample_instruction({.kind = EffectKind::alu_update, .alu_update = result}));
}

void Core::Impl::apply_alu(const AluWriteback &update) {
  for (unsigned index = 0; index < update.count; ++index) {
    detail::merge_move_storage(cpu.reg, update.updates.at(index));
  }
}

void Core::Impl::execute(const detail::Instruction &decoded) {
  execute_operation(decoded);
  publish_alu();
}
} // namespace dsp56300
