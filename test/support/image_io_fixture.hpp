// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::test {
inline constexpr Word io_entry = 16, io_port = 0xffffc0, io_bit = 2;
inline constexpr Word io_old_word = 0x123456, io_new_word = 0x765432;
inline constexpr unsigned io_roles = 7;
// One remap adds X/Y grants during native refresh and stopped installation.
inline constexpr unsigned io_observations = 32 + (2 * 2);
enum class IoEvent : std::uint8_t { read, write, bit, reset, pending, revision, windows };

struct IoObservation {
  IoEvent kind;
  Space space;
  Word address, pc, value;
  Registers registers;
  Progress progress;
  bool operator==(const IoObservation &) const = default;
};

struct ImageIo final : Io {
  void observe(const IoEvent kind,
               const Space space,
               const Word address,
               const Word pc,
               const Word value) const noexcept {
    if (core == nullptr || count == observed.size()) { return; }
    observed[count++] = {.kind = kind,
                         .space = space,
                         .address = address,
                         .pc = pc,
                         .value = value,
                         .registers = core->registers(),
                         .progress = core->progress()};
  }

  bool read_body(const Space space, const Word address, const Word pc, Word &value) const noexcept {
    observe(IoEvent::read, space, address, pc, input);
    value = input;
    if (yield == IoEvent::read) { core->request_yield(); }
    return failure != IoEvent::read;
  }

  bool write_body(const Space space, const Word address, const Word pc, const Word value) noexcept {
    observe(IoEvent::write, space, address, pc, value);
    output = value;
    if (remap) { ++revision; }
    if (yield == IoEvent::write) { core->request_yield(); }
    return failure != IoEvent::write;
  }

  bool bit_body(const Space space,
                const Word address,
                const Word pc,
                const Word bit,
                const BitAction action,
                bool &previous) noexcept {
    observe(IoEvent::bit, space, address, pc, bit);
    previous = (output & (Word{1} << bit)) != 0;
    if (action == BitAction::set) { output |= Word{1} << bit; }
    if (yield == IoEvent::bit) { core->request_yield(); }
    return failure != IoEvent::bit;
  }

  bool reset_body(const Word pc) noexcept {
    observe(IoEvent::reset, Space::program, pc, pc, 0);
    output = 0;
    if (yield == IoEvent::reset) { core->request_yield(); }
    return failure != IoEvent::reset;
  }

  bool pending_body(const Word status) const noexcept {
    observe(IoEvent::pending, Space::program, 0, core->registers().pc, status);
    return pending;
  }

  std::uint64_t revision_body() const noexcept {
    observe(
        IoEvent::revision, Space::program, 0, core->registers().pc, static_cast<Word>(revision));
    return revision;
  }

  std::span<const MemoryWindow> windows_body(const Space space, const Word mode) noexcept {
    observe(IoEvent::windows, space, 0, core != nullptr ? core->registers().pc : 0, mode);
    if (!mapped || space != Space::x) { return {}; }
    window = {
        .words = std::span(ram).subspan(revision == 0 ? 0U : 1U, 1), .known = {}, .writable = true};
    return {&window, 1};
  }

  bool read(Space space, Word address, Word pc, Word &value) noexcept override {
    ++dynamic_calls[static_cast<unsigned>(IoEvent::read)];
    return read_body(space, address, pc, value);
  }

  bool write(Space space, Word address, Word pc, Word value) noexcept override {
    ++dynamic_calls[static_cast<unsigned>(IoEvent::write)];
    return write_body(space, address, pc, value);
  }

  bool bit(Space space, Word address, Word pc, Word bit, BitAction action, bool &previous) noexcept
      override {
    ++dynamic_calls[static_cast<unsigned>(IoEvent::bit)];
    return bit_body(space, address, pc, bit, action, previous);
  }

  bool reset_peripherals(Word pc) noexcept override {
    ++dynamic_calls[static_cast<unsigned>(IoEvent::reset)];
    return reset_body(pc);
  }

  bool interrupt_pending(Word status) const noexcept override {
    ++dynamic_calls[static_cast<unsigned>(IoEvent::pending)];
    return pending_body(status);
  }

  std::uint64_t program_revision() const noexcept override {
    ++dynamic_calls[static_cast<unsigned>(IoEvent::revision)];
    return core == nullptr ? 0 : revision_body();
  }

  std::span<const MemoryWindow> memory_windows(const Space space,
                                               const Word mode) noexcept override {
    ++dynamic_calls[static_cast<unsigned>(IoEvent::windows)];
    return windows_body(space, mode);
  }

  std::optional<Word> inspect_program(Word address) const noexcept override {
    return address < program.size() ? std::optional(program[address]) : std::nullopt;
  }

  Core *core = nullptr;
  std::array<Word, 64> program{};
  Word input = 0, output = 0;
  // pending/revision identify no read/write/reset fault or synchronous yield.
  IoEvent failure = IoEvent::revision, yield = IoEvent::revision;
  bool pending = false, remap = false, mapped = false;
  std::array<Word, 2> ram{io_old_word, io_new_word};
  MemoryWindow window{};
  std::uint64_t revision = 0;
  mutable std::array<unsigned, io_roles> dynamic_calls{}, bound_calls{};
  mutable std::array<IoObservation, io_observations> observed{};
  mutable std::size_t count = 0;
};
}
