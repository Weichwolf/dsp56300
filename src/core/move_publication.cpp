// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "move_storage.hpp"

#include <cstdint>
#include <optional>

namespace dsp56300 {
using namespace detail;

bool Core::Impl::move_publication() const noexcept {
  return staged_computing() && staged->publication == InstructionPublication::move_updates &&
         fault.kind == FaultKind::none;
}

void Core::Impl::capture_move(MoveWriteback &update) const {
  for (unsigned index = 0; index < update.count; ++index) {
    auto &field = update.updates.at(index);
    field.previous = move_storage(cpu.reg, field) & field.mask;
  }
}

void Core::Impl::publish_move(MoveWriteback update) const {
  if (!move_publication() || update.count == 0) { return; }
  for (unsigned index = 0; index < update.count; ++index) {
    auto &field = update.updates.at(index);
    field.value = move_storage(cpu.reg, field) & field.mask;
  }
  static_cast<void>(sample_instruction({.kind = EffectKind::move_update, .move_update = update}));
}

void Core::Impl::apply_move(const MoveWriteback &update) {
  for (unsigned index = 0; index < update.count; ++index) {
    merge_move_storage(cpu.reg, update.updates.at(index));
  }
}

void Core::Impl::write_move_register(const unsigned number, const Word value) {
  if (!move_publication()) {
    write_register(number, value);
    return;
  }
  auto update = move_destination(number);
  capture_move(update);
  write_register(number, value);
  publish_move(update);
}

Word Core::Impl::read_move_register(const unsigned number) {
  if (!move_publication()) { return read_register(number); }
  if (number == a_register || number == b_register) {
    return transfer_word(number == a_register ? cpu.reg.a : cpu.reg.b, number);
  }
  if (number != ssh_register) { return read_register(number); }
  MoveWriteback update{
      .producer = MoveProducer::stack_read,
      .selector = ssh_register,
      .count = 2,
      .updates = {{{.storage = RegisterStorage::sp, .mask = (stack_entries * 4) - 1},
                   {.storage = RegisterStorage::sc, .mask = (stack_entries * 2) - 1}}}};
  capture_move(update);
  const auto value = read_register(number);
  publish_move(update);
  return value;
}

void Core::Impl::transfer_status(const Word flags, const std::optional<unsigned> source_register) {
  MoveWriteback update;
  if (source_register && move_publication() && flags != 0) {
    update = {.producer = MoveProducer::accumulator_read,
              .selector = static_cast<std::uint8_t>(*source_register),
              .count = 1,
              .updates = {{{.storage = RegisterStorage::sr, .mask = flags}}}};
    capture_move(update);
  }
  cpu.reg.sr |= flags;
  publish_move(update);
}
} // namespace dsp56300
