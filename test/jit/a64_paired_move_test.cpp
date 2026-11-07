// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_bus.hpp"
#include "a64_native.hpp"
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "move_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
using Ram = std::array<Word, memory_words>;
constexpr Word location = 384, peripheral = 0xffffc0, io_begin = 0xffff80;
constexpr Word incoming_x = 0xab923456, incoming_y = 0xcd765432;
constexpr Word mac = 0xd2, alu_transfer = 0x41, compare_magnitude = 0x47, maximum_magnitude = 0x15;

struct Pair {
  unsigned x = 0, y = 0, xi = 0, yi = 4, xm = 0, ym = 0, fail_at = 0;
  Word xa = location, ya = location, xmod = word_mask, ymod = word_mask, alu = 0, status = 0;
  bool xload = true, yload = true, external = false, aliases = false, executable = false;
};

Word opcode(const Pair point) {
  constexpr Word base = 0x800000;
  constexpr unsigned xreg = 18;
  constexpr unsigned ymode = 20;
  return base | (point.x << xreg) | (point.y << 16) | (point.xi << 8) | ((point.yi % 4) << 13) |
         (point.xm << 11) | (point.ym << ymode) | (point.xload ? 32768U : 0U) |
         (point.yload ? 4194304U : 0U) | point.alu;
}

Registers seed(const Pair point) {
  auto state = register_model::seed(0);
  state.sr = point.status;
  state.r.at(point.xi) = point.xa;
  state.r.at(point.yi) = point.ya;
  state.m.at(point.xi) = point.xmod;
  state.m.at(point.yi) = point.ymod;
  state.n.at(point.xi) = state.n.at(point.yi) = 1;
  return state;
}

struct Address {
  unsigned index, mode;
};

struct Access {
  Space space;
  Word address, value = 0;
  bool write = false;
};

struct Memory {
  const Ram &x;
  const Ram &y;
};

class Model {
public:
  Model(const Pair point, const Memory memory)
      : point_(point),
        initial_(seed(point)),
        expected_{.reg = initial_},
        x_(memory.x),
        y_(memory.y),
        observed_(memory.x.data() + location) {
    ++expected_.reg.pc;
    build();
    if (expected_.failure.kind != FaultKind::none) {
      expected_.retired = 0;
      expected_.cycles = 0;
      expected_.repeat_completed = true;
    }
  }

  [[nodiscard]] const a64::Expected &state() const noexcept { return expected_; }

  [[nodiscard]] const Ram &x() const noexcept { return x_; }

  [[nodiscard]] const Ram &y() const noexcept { return y_; }

  [[nodiscard]] std::span<const a64::BusEvent> events() const noexcept { return events_; }

private:
  void fail(const Fault failure) {
    if (expected_.failure.kind == FaultKind::none) { expected_.failure = failure; }
    expected_.boundary = detail::NativeBoundary::owner;
  }

  Word effective(const Address input) {
    const auto cursor = expected_.reg.r.at(input.index);
    const auto modifier = expected_.reg.m.at(input.index) % 65536;
    if (input.mode == 4) { return cursor; }
    if (modifier != address_model::linear && modifier >= 32768) {
      const auto length = (modifier % 16384) + 1;
      const auto span = address_model::span(length);
      constexpr Word reserved = 49152;
      if (modifier >= reserved || length == 1 || span != length) {
        fail({.kind = FaultKind::unsupported_mode,
              .space = Space::program,
              .address = start,
              .value = modifier,
              .pc = start});
        return cursor;
      }
    }
    const auto next = address_model::updated({.modifier = modifier,
                                              .cursor = cursor,
                                              .offset = expected_.reg.n.at(input.index),
                                              .mode = input.mode});
    expected_.reg.r.at(input.index) = next;
    return cursor;
  }

  Word owned(const Access request) {
    const auto previous = expected_.failure;
    ++accesses_;
    const bool peripheral_access = request.address >= io_begin;
    const bool accepted = peripheral_access && accesses_ != point_.fail_at;
    Word value = 0;
    if (peripheral_access) { value = request.space == Space::x ? incoming_x : incoming_y; }
    if (!accepted) {
      auto kind = request.write ? FaultKind::memory_write : FaultKind::memory_read;
      if (peripheral_access) { kind = request.write ? FaultKind::io_write : FaultKind::io_read; }
      fail({.kind = kind,
            .space = request.space,
            .address = request.address,
            .value = request.write ? request.value : 0U,
            .pc = start});
    }
    events_.push_back({.state = expected_.reg,
                       .failure = expected_.failure,
                       .prior_failure = previous,
                       .space = request.space,
                       .address = request.address,
                       .value = request.write ? request.value : value,
                       .write = request.write,
                       .observed = observed_,
                       .observed_value = x_.at(location)});
    expected_.boundary = detail::NativeBoundary::owner;
    return value & word_mask;
  }

  Word access(const Access request) {
    if (point_.external && expected_.failure.kind != FaultKind::none) { return 0; }
    if (request.address >= io_begin || (point_.external && request.address >= memory_words)) {
      return owned(request);
    }
    if (request.address >= memory_words) {
      fail({.kind = request.write ? FaultKind::memory_write : FaultKind::memory_read,
            .space = request.space,
            .address = request.address,
            .value = request.write ? request.value : 0U,
            .pc = start});
      return 0;
    }
    auto &memory = request.space == Space::x || point_.aliases ? x_ : y_;
    if (!request.write) { return memory.at(request.address) & word_mask; }
    memory.at(request.address) = request.value;
    if (point_.executable && (request.space == Space::x || point_.aliases)) {
      expected_.program_written = true;
      expected_.boundary = detail::NativeBoundary::owner;
    }
    return 0;
  }

  void commit(const Access transfer, const unsigned target, const bool load) {
    if (load) {
      register_model::write(expected_.reg, target, transfer.value);
    } else {
      static_cast<void>(access(transfer));
    }
  }

  void build() {
    constexpr Word cm = Word{1} << 13;
    const bool invalid_alu = point_.alu == 4 || point_.alu == 8 || point_.alu == 12;
    if ((point_.status & cm) != 0 ||
        (invalid_alu && (point_.status & saturation_model::mode) != 0)) {
      fail({.kind = FaultKind::unsupported_mode,
            .space = Space::program,
            .address = start,
            .value = point_.status,
            .pc = start});
      return;
    }
    const auto xa = effective({.index = point_.xi, .mode = point_.xm == 0 ? 4U : point_.xm});
    const auto ya = effective({.index = point_.yi, .mode = point_.ym == 0 ? 4U : point_.ym});
    const auto xr = move_model::xregs.at(point_.x);
    const auto yr = move_model::yregs.at(point_.y);
    const auto xv = point_.xload ? access({.space = Space::x, .address = xa})
                                 : register_model::read(expected_.reg, xr);
    const Access x{.space = Space::x, .address = xa, .value = xv, .write = !point_.xload};
    if (xa >= io_begin && ya >= io_begin) {
      commit(x, xr, point_.xload);
    } else {
      const auto yv = point_.yload ? access({.space = Space::y, .address = ya})
                                   : register_model::read(expected_.reg, yr);
      const Access y{.space = Space::y, .address = ya, .value = yv, .write = !point_.yload};
      commit(x, xr, point_.xload);
      commit(y, yr, point_.yload);
    }
    if (expected_.failure.kind != FaultKind::none) { return; }
    if (invalid_alu) {
      fail({.kind = point_.alu == 8 ? FaultKind::invalid_instruction
                                    : FaultKind::unsupported_instruction,
            .space = Space::program,
            .address = start,
            .value = point_.alu,
            .pc = start});
      return;
    }
    a64_alu_model::apply(expected_.reg, initial_, point_.alu);
  }

  Pair point_;
  Registers initial_;
  a64::Expected expected_;
  Ram x_, y_;
  const Word *observed_;
  unsigned accesses_ = 0;
  std::vector<a64::BusEvent> events_;
};

void probe(const Pair point) {
  const a64::Instruction input{.opcode = opcode(point), .mode = point.status};
  const a64::Program program(input);
  Ram x{};
  Ram y{};
  x.fill(incoming_x);
  y.fill(incoming_y);
  const Model expected(point, {.x = x, .y = y});
  a64::Invocation invocation(seed(point));
  a64::Bus bus(invocation, expected.events());
  auto binding = bus.binding(point.external);
  const std::array windows{
      detail::NativeWindow{.words = x.size(), .data = x.data(), .writable = true},
      detail::NativeWindow{
          .words = y.size(), .data = point.aliases ? x.data() : y.data(), .writable = true}};
  binding.windows[0] = {.data = windows.data(), .count = 1};
  binding.windows[1] = {.data = windows.data() + 1, .count = 1};
  if (point.executable) { binding.program = {.data = x.data(), .words = x.size()}; }
  invocation.bind({{{.data = x.data(), .words = x.size(), .aliases_program = point.executable},
                    {.data = point.aliases ? x.data() : y.data(),
                     .words = y.size(),
                     .aliases_program = point.executable && point.aliases}}},
                  binding);
  invocation.run(program);
  invocation.expect(expected.state(), input.opcode);
  bus.expect();
  require(x == expected.x() && y == expected.y(),
          "Paired captures precede commits; faults preserve exact physical writes");
}

void domains() {
  constexpr std::array<Word, 3> addresses{location, memory_words, peripheral};
  for (const Word xa : addresses) {
    for (const Word ya : addresses) {
      for (unsigned directions = 0; directions < 4; ++directions) {
        for (const bool external : {false, true}) {
          for (unsigned fail_at = 0; fail_at < 3; ++fail_at) {
            probe({.x = directions,
                   .y = (directions + 1) % 4,
                   .xm = 3,
                   .ym = 3,
                   .fail_at = fail_at,
                   .xa = xa,
                   .ya = ya,
                   .alu = mac,
                   .xload = (directions & 1U) != 0,
                   .yload = (directions & 2U) != 0,
                   .external = external});
          }
        }
      }
    }
  }
}

void modifiers() {
  constexpr std::array<Word, 5> mods{word_mask, 0, 3, 0x8000, 0xc001};
  for (const auto xmod : mods) {
    for (const auto ymod : mods) {
      for (unsigned addressing = 0; addressing < 16; ++addressing) {
        for (const bool external : {false, true}) {
          probe({.x = 2,
                 .y = 3,
                 .xm = addressing % 4,
                 .ym = addressing / 4,
                 .xmod = xmod,
                 .ymod = ymod,
                 .alu = alu_transfer,
                 .xload = false,
                 .yload = false,
                 .external = external});
        }
      }
    }
  }
}

void aliases() {
  for (unsigned xreg = 0; xreg < 4; ++xreg) {
    for (unsigned yreg = 0; yreg < 4; ++yreg) {
      for (unsigned directions = 0; directions < 4; ++directions) {
        for (const bool executable : {false, true}) {
          probe({.x = xreg,
                 .y = yreg,
                 .alu = compare_magnitude,
                 .xload = (directions & 1U) != 0,
                 .yload = (directions & 2U) != 0,
                 .aliases = true,
                 .executable = executable});
        }
      }
    }
  }
}

void conditions() {
  constexpr Word cm = Word{1} << 13;
  for (const Word status : {0U, status_byte, saturation_model::mode | mode_up, cm}) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (unsigned direction = 0; direction < 4; ++direction) {
        probe({.x = 2,
               .y = 2,
               .xi = index,
               .yi = (index < 4 ? 4U : 0U) + ((index + 1) % 4),
               .xm = 3,
               .ym = 2,
               .xa = peripheral,
               .alu = maximum_magnitude,
               .status = status,
               .xload = (direction & 1U) != 0,
               .yload = (direction & 2U) != 0});
      }
    }
  }
}

void inherited_failures() {
  constexpr Word invalid_modifier = 0xc001;
  for (const bool external : {false, true}) {
    probe({.x = 2,
           .y = 3,
           .xm = 3,
           .ym = 3,
           .fail_at = 1,
           .ya = peripheral,
           .xmod = invalid_modifier,
           .xload = false,
           .yload = false,
           .external = external,
           .executable = !external});
    probe({.x = 2,
           .y = 3,
           .xm = 3,
           .ym = 3,
           .fail_at = 1,
           .xa = peripheral,
           .ymod = invalid_modifier,
           .xload = true,
           .yload = false,
           .external = external});
  }
}

void invalid_alu() {
  for (const Word alu : {4U, 8U, 12U}) {
    for (const Word status : {0U, saturation_model::mode}) {
      for (const bool external : {false, true}) {
        probe({.x = 2,
               .y = 3,
               .ym = 3,
               .ya = peripheral,
               .alu = alu,
               .status = status,
               .xload = false,
               .yload = false,
               .external = external,
               .executable = !external});
      }
    }
  }
}

void remapping(const bool writable) {
  const Pair point{.x = 2,
                   .y = 0,
                   .xm = 3,
                   .ym = 3,
                   .ya = peripheral,
                   .alu = mac,
                   .xload = false,
                   .yload = true,
                   .external = true};
  const a64::Instruction input{.opcode = opcode(point)};
  const a64::Program program(input);
  constexpr std::array<Word, 4> original{3, 5, 7, 9};
  auto old = original;
  auto replacement = original;
  std::array<std::uint8_t, 4> known{};
  detail::NativeWindow window{.address = location,
                              .words = old.size(),
                              .data = old.data(),
                              .known = known.data(),
                              .writable = true};
  const auto initial = seed(point);
  a64::Expected expected{.reg = initial, .boundary = detail::NativeBoundary::owner};
  ++expected.reg.pc;
  ++expected.reg.r[0];
  ++expected.reg.r[4];
  const auto value = register_model::read(expected.reg, 14);
  const std::array events{a64::BusEvent{.state = expected.reg,
                                        .space = Space::y,
                                        .address = peripheral,
                                        .value = incoming_y,
                                        .observed = old.data(),
                                        .observed_value = original[0]},
                          a64::BusEvent{.state = expected.reg,
                                        .address = location,
                                        .value = value,
                                        .write = true,
                                        .observed = replacement.data(),
                                        .observed_value = original[0]}};
  register_model::write(expected.reg, 6, incoming_y & word_mask);
  a64_alu_model::apply(expected.reg, initial, mac);
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, std::span<const a64::BusEvent>(events).first(writable ? 1U : 2U));
  bus.remap(window, replacement.data(), writable);
  auto binding = bus.binding(true);
  binding.windows[0] = {.data = &window, .count = 1};
  invocation.bind({}, binding);
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
  require(old == original && replacement[0] == (writable ? value : original[0]) &&
              known[0] == (writable ? 1 : 0),
          "Y capture callbacks precede X stores and can replace their host mapping or permission");
}
}
}

int main() {
  try {
    dsp56300::test::domains();
    dsp56300::test::modifiers();
    dsp56300::test::aliases();
    dsp56300::test::conditions();
    dsp56300::test::inherited_failures();
    dsp56300::test::invalid_alu();
    dsp56300::test::remapping(false);
    dsp56300::test::remapping(true);
    std::cout << dsp56300::test::checks
              << " independent A64 paired bus/fault/alias checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
