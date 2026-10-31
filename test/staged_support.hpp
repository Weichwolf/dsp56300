// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace dsp56300::test::staged_test {
inline constexpr Word boot = 0xff0000, data_address = 0x100, datum = 0x800001;
inline constexpr Word add_x0 = 0x200040, read_x0 = 0x44d800, write_x0 = 0x445800;
inline constexpr Word vector = 0x54, handler = 0x300, short_call = 0x0d0000, rti = 4;
inline constexpr Word repeat_four = 0x0604a0, movem_x0 = 0x07f084, raw_datum = 0xab800001;
inline constexpr std::array backends{Backend::reference, Backend::cached, Backend::jit};

struct Event {
  EffectKind kind = EffectKind::read;
  Space space = Space::x;
  Word address = data_address, value = raw_datum;
  BitAction action = BitAction::test;
  bool accept = true;
};

class Bus final : public Io {
public:
  std::vector<Event> events;
  std::size_t calls = 0, accepted = 0;
  Word pc = boot;
  Word x_written = 0, y_written = 0;
  bool exact = true;

  bool read(const Space space,
            const Word address,
            const Word instruction,
            Word &value) noexcept override {
    Event event;
    if (!next({.kind = EffectKind::read, .space = space, .address = address, .pc = instruction},
              event)) {
      return false;
    }
    value = event.value;
    return event.accept;
  }

  bool write(const Space space,
             const Word address,
             const Word instruction,
             const Word value) noexcept override {
    Event event;
    if (!next({.kind = EffectKind::write,
               .space = space,
               .address = address,
               .value = value,
               .pc = instruction},
              event)) {
      return false;
    }
    const bool committed = event.accept && exact;
    if (committed && space == Space::x) { x_written = value; }
    if (committed && space == Space::y) { y_written = value; }
    return committed;
  }

  bool bit(const Space space,
           const Word address,
           const Word instruction,
           const Word bit,
           const BitAction action,
           bool &previous) noexcept override {
    Event event;
    if (!next({.kind = EffectKind::bit,
               .space = space,
               .address = address,
               .pc = instruction,
               .bit = bit,
               .action = action},
              event)) {
      return false;
    }
    previous = event.value != 0;
    return event.accept && exact;
  }

  bool reset_peripherals(const Word instruction) noexcept override {
    Event event;
    return next({.kind = EffectKind::reset,
                 .space = Space::program,
                 .address = instruction,
                 .pc = instruction},
                event) &&
           event.accept;
  }

private:
  bool next(const InstructionEffect request, Event &event) noexcept {
    if (calls >= events.size()) {
      exact = false;
      ++calls;
      return false;
    }
    event = events[calls++];
    exact = exact && event.kind == request.kind && event.space == request.space &&
            event.address == request.address && request.pc == pc;
    if (request.kind == EffectKind::write) { exact = exact && event.value == request.value; }
    if (request.kind == EffectKind::bit) {
      exact = exact && request.bit == 0 && event.action == request.action;
    }
    if (exact && event.accept) { ++accepted; }
    return exact;
  }
};

inline Registers seed() {
  auto state = initial_state();
  state.pc = boot;
  state.x = 1;
  state.r[0] = data_address;
  state.lc = 7;
  return state;
}

inline PendingInstruction pending(Core &core) {
  const auto value = core.pending_instruction();
  if (!value) { throw std::runtime_error("Expected genuine pending instruction effect"); }
  return *value;
}

inline void retire(Core &core) {
  const auto value = pending(core);
  require(value.effect.kind == EffectKind::retire,
          "Only actual computed instruction results admit retirement");
  require(core.advance_instruction(value.ticket).retired,
          "Native result is published exactly once");
}

template <typename Action> void rejected(Action action) {
  bool result = false;
  try {
    action();
  } catch (const std::logic_error &) { result = true; }
  require(result, "Stale/foreign/mismatched instruction ownership is rejected");
}

void arithmetic();
void memory();
void multiple_effects();
void restore();
void interrupt();
void vector_provenance();
void repetition();
void faults();
void events();
} // namespace dsp56300::test::staged_test
