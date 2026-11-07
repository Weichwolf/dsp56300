// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_bus.hpp"
#include "a64_native.hpp"
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word location = 384, payload = 0x923456, incoming = 0xab923456;

Registers seed(const Word mode = 0) {
  auto state = register_model::seed(0);
  state.sr = mode;
  state.r[0] = location;
  state.n[0] = 1;
  return state;
}

struct Move {
  unsigned reg = 4, mode = 4, index = 0;
  Word address = location, alu = 0;
  bool load = true, y = false, short_form = false;
};

a64::Instruction instruction(const Move point, const Word mode = 0) {
  constexpr Word base = 0x400000;
  const Word target = ((point.reg % 8) << 16) | ((point.reg / 8) << 20);
  Word opcode = base | target | (point.load ? 32768U : 0U) | (point.y ? 524288U : 0U) | point.alu;
  unsigned words = 1;
  unsigned cycles = point.load ? 1U : 2U;
  if (point.short_form) {
    opcode |= point.address << 8;
  } else {
    opcode |= 16384U | (point.mode << 11) | (point.index << 8);
    words = point.mode == 6 ? 2U : 1U;
    cycles = point.mode >= 5 ? 2U : 1U;
  }
  return {
      .opcode = opcode, .extension = point.address, .mode = mode, .words = words, .cycles = cycles};
}

Word address(Registers &state, const Move point) {
  if (point.short_form || point.mode == 6) { return point.address; }
  const auto original = state.r.at(point.index);
  const auto next = address_model::updated({.modifier = state.m.at(point.index),
                                            .cursor = original,
                                            .offset = state.n.at(point.index),
                                            .mode = point.mode});
  if (point.mode != 4 && point.mode != 5) { state.r.at(point.index) = next; }
  return point.mode == 5 || point.mode == 7 ? next : original;
}

void fail(a64::Expected &expected, const Fault failure) {
  expected.failure = failure;
  expected.retired = 0;
  expected.cycles = 0;
  expected.repeat_completed = true;
  expected.boundary = detail::NativeBoundary::owner;
}

void flat_case(const Move point, const Word mode) {
  const auto input = instruction(point, mode);
  const a64::Program program(input);
  std::array<Word, memory_words> data{};
  data.fill(incoming);
  auto expected_data = data;
  const auto initial = seed(mode);
  a64::Expected expected{.reg = initial, .cycles = input.cycles};
  expected.reg.pc += input.words;
  const auto effective = address(expected.reg, point);
  Word value = payload;
  if (!point.load) { value = register_model::read(expected.reg, point.reg); }
  const bool immediate = point.load && !point.short_form && point.mode == 6 && point.index == 4;
  if (!immediate && effective >= memory_words) {
    fail(expected,
         {.kind = point.load ? FaultKind::memory_read : FaultKind::memory_write,
          .space = point.y ? Space::y : Space::x,
          .address = effective,
          .value = point.load ? 0U : value,
          .pc = start});
  } else {
    if (point.load) {
      register_model::write(expected.reg, point.reg, immediate ? point.address : payload);
    } else {
      expected_data.at(effective) = value;
    }
    a64_alu_model::apply(expected.reg, initial, point.alu);
  }
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, {});
  invocation.bind(
      {{{.data = data.data(), .words = data.size()}, {.data = data.data(), .words = data.size()}}},
      bus.binding());
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
  require(data == expected_data,
          "Flat RAM changes only the expected word and stores normalized bus data");
}

void flat_memory() {
  constexpr Word short_address = 31;
  for (const Word mode : {0U, mode_down, mode_up, saturation_model::mode}) {
    for (unsigned reg = 4; reg < 32; ++reg) {
      for (const bool load : {false, true}) {
        for (const bool y : {false, true}) {
          for (unsigned addressing = 0; addressing < 8; ++addressing) {
            flat_case({.reg = reg, .mode = addressing, .load = load, .y = y}, mode);
          }
          flat_case({.reg = reg, .mode = 6, .index = 4, .load = load, .y = y}, mode);
          flat_case(
              {.reg = reg, .address = short_address, .load = load, .y = y, .short_form = true},
              mode);
        }
      }
    }
  }
  for (const Word alu : {0U, 0x13U, 0x41U, 0x87U, 0xd2U, 0x17U, 0x32U, 0x47U}) {
    for (const unsigned reg : {4U, 8U, 10U, 12U, 14U, 16U}) {
      for (const bool load : {false, true}) {
        flat_case({.reg = reg, .mode = 3, .alu = alu, .load = load}, 0);
        flat_case({.reg = reg, .mode = 6, .address = memory_words, .alu = alu, .load = load}, 0);
      }
    }
  }
}

struct Window {
  bool load = true, writable = true, alias = false, known = true, accept = true;
};

void window_case(const Window point) {
  const Move move{.reg = 14, .mode = 3, .alu = 0xd2, .load = point.load};
  const auto input = instruction(move);
  const a64::Program program(input);
  std::array<Word, 4> data{incoming, 3, 5, 7};
  auto expected_data = data;
  std::array<std::uint8_t, 4> known{static_cast<std::uint8_t>(point.known ? 7 : 0), 1, 1, 1};
  auto expected_known = known;
  const detail::NativeWindow grant{.address = location,
                                   .words = data.size(),
                                   .data = data.data(),
                                   .known = known.data(),
                                   .writable = point.writable,
                                   .aliases_program = point.alias};
  const auto initial = seed();
  a64::Expected expected{.reg = initial};
  ++expected.reg.pc;
  ++expected.reg.r[0];
  const auto value = point.load ? payload : register_model::read(expected.reg, 14);
  const bool owned = point.load ? !point.known : !point.writable || point.alias;
  const Fault failure =
      point.accept ? Fault{}
                   : Fault{.kind = point.load ? FaultKind::memory_read : FaultKind::memory_write,
                           .space = Space::x,
                           .address = location,
                           .value = point.load ? 0U : value,
                           .pc = start};
  const std::array events{a64::BusEvent{.state = expected.reg,
                                        .failure = failure,
                                        .address = location,
                                        .value = point.load ? incoming : value,
                                        .write = !point.load}};
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation,
               owned ? std::span<const a64::BusEvent>(events) : std::span<const a64::BusEvent>{});
  auto binding = bus.binding(true);
  binding.windows[0] = {.data = &grant, .count = 1};
  invocation.bind({}, binding);
  if (owned && !point.accept) {
    fail(expected, failure);
  } else {
    if (point.load) { register_model::write(expected.reg, 14, payload); }
    if (!point.load && !owned) {
      expected_data[0] = value;
      expected_known[0] = 1;
    }
    a64_alu_model::apply(expected.reg, initial, move.alu);
    if (owned) { expected.boundary = detail::NativeBoundary::owner; }
  }
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
  require(data == expected_data && known == expected_known,
          "Native grants preserve ownership and known-byte semantics");
}

void windows() {
  for (const bool load : {false, true}) {
    for (const bool writable : {false, true}) {
      for (const bool alias : {false, true}) {
        for (const bool known : {false, true}) {
          for (const bool accept : {false, true}) {
            window_case({.load = load,
                         .writable = writable,
                         .alias = alias,
                         .known = known,
                         .accept = accept});
          }
        }
      }
    }
  }
}

void window_search() {
  const a64::Instruction input{.opcode = 0x44e000};
  const a64::Program program(input);
  std::array<Word, 4> first{};
  first.fill(incoming);
  std::array<Word, 4> middle{};
  middle.fill(incoming);
  std::array<Word, 4> last{};
  last.fill(incoming);
  const std::array grants{
      detail::NativeWindow{.address = 16, .words = first.size(), .data = first.data()},
      detail::NativeWindow{.address = location, .words = middle.size(), .data = middle.data()},
      detail::NativeWindow{.address = 512, .words = last.size(), .data = last.data()}};
  constexpr std::array<Word, 12> addresses{
      0, 15, 16, 19, 20, location - 1, location, location + 3, location + 4, 512, 516, word_mask};
  for (const unsigned count : {0U, 3U}) {
    for (const auto cursor : addresses) {
      auto initial = seed();
      initial.r[0] = cursor;
      a64::Expected expected{.reg = initial};
      ++expected.reg.pc;
      bool owned = true;
      for (unsigned index = 0; index < count; ++index) {
        const auto &grant = grants.at(index);
        if (cursor >= grant.address && cursor - grant.address < grant.words) { owned = false; }
      }
      const std::array events{
          a64::BusEvent{.state = expected.reg, .address = cursor, .value = incoming}};
      a64::Invocation invocation(initial);
      a64::Bus bus(invocation,
                   owned ? std::span<const a64::BusEvent>(events)
                         : std::span<const a64::BusEvent>{});
      auto binding = bus.binding(true);
      binding.windows[0] = {.data = grants.data(), .count = count};
      invocation.bind({}, binding);
      register_model::write(expected.reg, 4, payload);
      if (owned) { expected.boundary = detail::NativeBoundary::owner; }
      invocation.run(program);
      invocation.expect(expected, input.opcode);
      bus.expect();
    }
  }
}

void aliases() {
  for (const Word cursor : {0U, 64U, 191U, 192U}) {
    const Move move{.reg = 14, .load = false};
    const auto first = instruction(move);
    const std::array steps{first, a64::Instruction{.opcode = 0x200013, .pc = start + 1}};
    const a64::Program program(steps);
    std::array<Word, 512> data{};
    auto expected_data = data;
    auto initial = seed();
    initial.r[0] = cursor;
    a64::Expected expected{.reg = initial};
    ++expected.reg.pc;
    expected_data.at(cursor) = register_model::read(expected.reg, 14);
    const bool executable = cursor >= 64 && cursor < 192;
    if (executable) {
      expected.boundary = detail::NativeBoundary::owner;
      expected.program_written = true;
    } else {
      constexpr Word clear = 0x13;
      a64_alu_model::apply(expected.reg, clear);
      ++expected.reg.pc;
      expected.retired = 2;
      expected.cycles = 2;
    }
    a64::Invocation invocation(initial);
    auto bus = detail::NativeBus{.program = {.data = data.data() + 64, .words = 128}};
    invocation.bind({{{.data = data.data(), .words = data.size(), .aliases_program = true}, {}}},
                    bus);
    invocation.run(program, 2);
    invocation.expect(expected, first.opcode);
    require(data == expected_data,
            "Only real executable aliases invalidate and stop a retained successor");
  }
}
}
}

int main() {
  try {
    dsp56300::test::flat_memory();
    dsp56300::test::windows();
    dsp56300::test::window_search();
    dsp56300::test::aliases();
    std::cout << dsp56300::test::checks << " native A64 RAM/window/bus checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
