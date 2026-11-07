// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_bus.hpp"
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "move_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
using move_model::Form;
using move_model::Kind;
constexpr Word peripheral = 0xffffc0, incoming_x = 0xab923456, incoming_y = 0xcd765432;
constexpr move_model::Data data{.high = 0x923456, .low = 0x765432};
constexpr Word mac = 0xd2;

struct Point {
  Form form{};
  Word status = 0;
  unsigned fail_at = 0;
};

a64::Instruction instruction(const Point point) {
  return {.opcode = move_model::opcode(point.form, mac),
          .extension = point.form.extension,
          .mode = point.status,
          .words = move_model::words(point.form),
          .cycles = move_model::cycles(point.form)};
}

Registers seed(const Point point) {
  auto initial = register_model::seed(0);
  initial.sr = point.status;
  initial.r.at(point.form.index) = peripheral;
  initial.n.at(point.form.index) = 1;
  return initial;
}

void fail(a64::Expected &expected, const Fault failure) {
  expected.failure = failure;
  expected.retired = 0;
  expected.cycles = 0;
  expected.repeat_completed = true;
}

struct Event {
  Space space;
  Word address, value;
  bool write;
};

a64::BusEvent event(const Registers &state, const Event input, const bool rejected) {
  Fault failure;
  if (rejected) {
    failure = {.kind = input.write ? FaultKind::io_write : FaultKind::io_read,
               .space = input.space,
               .address = input.address,
               .value = input.write ? input.value : 0U,
               .pc = start};
  }
  return {.state = state,
          .failure = failure,
          .space = input.space,
          .address = input.address,
          .value = input.value,
          .write = input.write};
}

void long_case(const Point point) {
  const auto input = instruction(point);
  const a64::Program program(input);
  const auto initial = seed(point);
  a64::Expected expected{
      .reg = initial, .cycles = input.cycles, .boundary = detail::NativeBoundary::owner};
  expected.reg.pc += input.words;
  const auto address = move_model::address(expected.reg, point.form);
  const auto values =
      point.form.load ? data : move_model::store_long(expected.reg, point.form.selector);
  std::vector<a64::BusEvent> events;
  events.push_back(event(expected.reg,
                         {.space = Space::x,
                          .address = address,
                          .value = point.form.load ? incoming_x : values.high,
                          .write = !point.form.load},
                         point.fail_at == 1));
  if (point.fail_at != 1) {
    events.push_back(event(expected.reg,
                           {.space = Space::y,
                            .address = address,
                            .value = point.form.load ? incoming_y : values.low,
                            .write = !point.form.load},
                           point.fail_at == 2));
  }
  if (point.fail_at != 0) {
    fail(expected, events.back().failure);
  } else {
    if (point.form.load) { move_model::load_long(expected.reg, point.form.selector, data); }
    a64_alu_model::apply(expected.reg, initial, mac);
  }
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, events);
  invocation.bind({}, bus.binding());
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
}

void long_io() {
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (const bool load : {false, true}) {
      for (unsigned fail_at = 0; fail_at < 3; ++fail_at) {
        for (const Word status : {0U, mode_down, saturation_model::mode | mode_up}) {
          long_case({.form = {.selector = selector, .mode = 3, .index = selector, .load = load},
                     .status = status,
                     .fail_at = fail_at});
        }
      }
    }
  }
}

void mixed_case(const Point point) {
  const auto input = instruction(point);
  const a64::Program program(input);
  const auto initial = seed(point);
  a64::Expected expected{
      .reg = initial, .cycles = input.cycles, .boundary = detail::NativeBoundary::owner};
  expected.reg.pc += input.words;
  const auto address = move_model::address(expected.reg, point.form);
  const auto secondary = register_model::read(expected.reg, point.form.source_b ? 15U : 14U);
  const auto primary = point.form.y ? move_model::yregs.at(point.form.selector)
                                    : move_model::xregs.at(point.form.selector);
  const auto value = point.form.load ? data.high : register_model::read(expected.reg, primary);
  const auto space = point.form.y ? Space::y : Space::x;
  const auto read_value = point.form.y ? incoming_y : incoming_x;
  const std::array events{event(expected.reg,
                                {.space = space,
                                 .address = address,
                                 .value = point.form.load ? read_value : value,
                                 .write = !point.form.load},
                                point.fail_at != 0)};
  if (point.fail_at != 0) {
    fail(expected, events.front().failure);
  } else {
    if (point.form.load) {
      register_model::write(expected.reg, primary, point.form.y ? data.low : data.high);
    }
    register_model::write(expected.reg,
                          (point.form.y ? 4U : 6U) + static_cast<unsigned>(point.form.second_high),
                          secondary);
    a64_alu_model::apply(expected.reg, initial, mac);
  }
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, events);
  invocation.bind({}, bus.binding());
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
}

void mixed_variants(const Form form) {
  for (const bool load : {false, true}) {
    auto variant = form;
    variant.load = load;
    for (const unsigned fail_at : {0U, 1U}) {
      for (const Word status : {0U, mode_down, saturation_model::mode | mode_up}) {
        mixed_case({.form = variant, .status = status, .fail_at = fail_at});
      }
    }
  }
}

void mixed_io() {
  for (unsigned selector = 0; selector < 4; ++selector) {
    for (const bool y : {false, true}) {
      for (const bool source_b : {false, true}) {
        for (const bool second_high : {false, true}) {
          mixed_variants({.kind = Kind::mixed_one,
                          .selector = selector,
                          .mode = 3,
                          .index = selector,
                          .y = y,
                          .source_b = source_b,
                          .second_high = second_high});
        }
      }
    }
  }
}

void remapping(const bool load, const bool writable) {
  const Form form{.selector = 6, .load = load};
  const auto input = instruction({.form = form});
  const a64::Program program(input);
  constexpr std::array<Word, 4> original_x{3, 5, 7, 9};
  constexpr std::array<Word, 4> original_y{11, 13, 15, 17};
  auto x = original_x;
  auto y = original_y;
  auto replacement = original_y;
  replacement[0] = incoming_y;
  std::array<std::uint8_t, 4> known{};
  const std::array windows{detail::NativeWindow{.address = move_model::default_address,
                                                .words = x.size(),
                                                .data = x.data(),
                                                .known = known.data()},
                           detail::NativeWindow{.address = move_model::default_address,
                                                .words = y.size(),
                                                .data = y.data(),
                                                .writable = true}};
  auto native_windows = windows;
  auto initial = register_model::seed(0);
  initial.sr = 0;
  initial.r[0] = move_model::default_address;
  a64::Expected expected{.reg = initial, .boundary = detail::NativeBoundary::owner};
  ++expected.reg.pc;
  const auto values = load ? data : move_model::store_long(expected.reg, form.selector);
  std::vector<a64::BusEvent> events{a64::BusEvent{.state = expected.reg,
                                                  .address = move_model::default_address,
                                                  .value = load ? incoming_x : values.high,
                                                  .write = !load}};
  if (!load && !writable) {
    events.push_back({.state = expected.reg,
                      .space = Space::y,
                      .address = move_model::default_address,
                      .value = values.low,
                      .write = true});
  }
  if (load) { move_model::load_long(expected.reg, form.selector, data); }
  a64_alu_model::apply(expected.reg, initial, mac);
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, events);
  bus.remap(native_windows[1], replacement.data(), writable);
  auto binding = bus.binding(true);
  binding.windows[0] = {.data = native_windows.data(), .count = 1};
  binding.windows[1] = {.data = native_windows.data() + 1, .count = 1};
  invocation.bind({}, binding);
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
  require(x == original_x && y == original_y &&
              replacement[0] == (!load && writable ? values.low : incoming_y),
          "The second long half uses the mapping returned by the first owned access");
}
}
}

int main() {
  try {
    dsp56300::test::long_io();
    dsp56300::test::mixed_io();
    dsp56300::test::remapping(true, false);
    dsp56300::test::remapping(false, false);
    dsp56300::test::remapping(false, true);
    std::cout << dsp56300::test::checks
              << " independent A64 long/mixed I/O/fault/remapping checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
