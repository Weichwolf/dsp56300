// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace dsp56300::test::scripted_io {
inline constexpr Word boot = 0xff0000, data_address = 0x100, raw_datum = 0xab800001;
inline constexpr std::array backends{Backend::reference, Backend::cached, Backend::jit};
enum class Action : unsigned char { read, write, bit, reset };

struct Event {
  Action kind = Action::read;
  Space space = Space::x;
  Word address = data_address, value = raw_datum;
  BitAction action = BitAction::test;
  bool accept = true;
};

struct Request {
  Action kind;
  Space space;
  Word address, value = 0, pc = 0, bit = 0;
  BitAction action = BitAction::test;
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
    if (!next({.kind = Action::read, .space = space, .address = address, .pc = instruction},
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
    if (!next({.kind = Action::write,
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
    if (!next({.kind = Action::bit,
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
    return next({.kind = Action::reset,
                 .space = Space::program,
                 .address = instruction,
                 .pc = instruction},
                event) &&
           event.accept;
  }

private:
  bool next(const Request request, Event &event) noexcept {
    if (calls >= events.size()) {
      exact = false;
      ++calls;
      return false;
    }
    event = events[calls++];
    exact = exact && event.kind == request.kind && event.space == request.space &&
            event.address == request.address && request.pc == pc;
    if (request.kind == Action::write) { exact = exact && event.value == request.value; }
    if (request.kind == Action::bit) {
      exact = exact && request.bit == 0 && event.action == request.action;
    }
    if (exact && event.accept) { ++accepted; }
    return exact;
  }
};

}
