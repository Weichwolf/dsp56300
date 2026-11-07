// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_bus.hpp"
#include "a64_native.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
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
constexpr Word location = 384, payload = 0x923456, incoming = 0xab923456;
constexpr Word low_port = 0xffff80, high_port = 0xffffc0;
constexpr Word sa = Word{1} << 17, cm = Word{1} << 13;

void fail(a64::Expected &expected, const Fault failure) {
  expected.failure = failure;
  expected.retired = 0;
  expected.cycles = 0;
  expected.repeat_completed = true;
  expected.boundary = detail::NativeBoundary::owner;
}

struct Port {
  unsigned reg = 4;
  Word index = 0;
  bool high = true, y = false, write = false, accept = true;
};

Word opcode(const Port point) {
  constexpr Word high = 0x084400;
  constexpr Word low_x = 0x044480;
  constexpr Word low_y = 0x044420;
  Word result = point.y ? low_y : low_x;
  if (point.high) { result = high | (point.y ? 65536U : 0U); }
  constexpr Word selector = 0x3f00;
  result = (result & ~selector) | (point.reg << 8);
  const auto packed = (point.index % 32) | ((point.index / 32) << 6);
  return result | (point.write ? 32768U : 0U) | (point.high ? point.index : packed);
}

bool rejected(const Port point, const Word mode) {
  const bool checked = point.reg == 14 || point.reg == 15 || (point.reg <= 15 && (mode & sa) != 0);
  return checked &&
         ((mode & (sa | cm)) != 0 || (mode & (mode_up | mode_down)) == (mode_up | mode_down));
}

void port_case(const Port point, const Word mode) {
  const a64::Instruction input{.opcode = opcode(point), .mode = mode};
  const a64::Program program(input);
  auto initial = register_model::seed(0);
  initial.sr = mode;
  a64::Expected expected{.reg = initial, .boundary = detail::NativeBoundary::owner};
  ++expected.reg.pc;
  const auto address = (point.high ? high_port : low_port) + point.index;
  const auto space = point.y ? Space::y : Space::x;
  const auto value = point.write ? register_model::read(expected.reg, point.reg) : payload;
  const Fault failure = point.accept
                            ? Fault{}
                            : Fault{.kind = point.write ? FaultKind::io_write : FaultKind::io_read,
                                    .space = space,
                                    .address = address,
                                    .value = point.write ? value : 0U,
                                    .pc = start};
  auto event_state = expected.reg;
  if (rejected(point, mode)) {
    expected.reg = initial;
    ++expected.reg.pc;
    fail(expected,
         {.kind = FaultKind::unsupported_mode,
          .space = Space::program,
          .address = start,
          .value = mode,
          .pc = start});
  } else if (!point.accept) {
    fail(expected, failure);
  } else if (!point.write) {
    register_model::write(expected.reg, point.reg, value);
  }
  const std::array events{a64::BusEvent{.state = event_state,
                                        .failure = failure,
                                        .space = space,
                                        .address = address,
                                        .value = point.write ? value : incoming,
                                        .write = point.write}};
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation,
               rejected(point, mode) ? std::span<const a64::BusEvent>{}
                                     : std::span<const a64::BusEvent>(events));
  invocation.bind({}, bus.binding());
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
}

void ports() {
  for (const bool high : {false, true}) {
    for (const bool y : {false, true}) {
      for (const bool write : {false, true}) {
        for (Word index = 0; index < 64; ++index) {
          port_case({.index = index, .high = high, .y = y, .write = write}, 0);
          port_case({.reg = 14, .index = index, .high = high, .y = y, .write = write},
                    saturation_model::mode | mode_up);
          port_case(
              {.reg = 14, .index = index, .high = high, .y = y, .write = write, .accept = false},
              0);
        }
      }
    }
  }
  for (unsigned reg = 4; reg < 64; ++reg) {
    if (!register_model::valid(reg)) { continue; }
    for (const bool write : {false, true}) {
      for (const Word mode : {0U, sa, cm, mode_up | mode_down}) {
        port_case({.reg = reg, .write = write}, mode);
      }
    }
  }
}

struct MemoryPort {
  Word opcode;
  Space memory = Space::x, peripheral = Space::x;
  unsigned cycles = 2;
  bool write = false, immediate = false;
};

void memory_port(const MemoryPort point, const Word mode, const bool accept) {
  const a64::Instruction input{.opcode = point.opcode,
                               .extension = payload,
                               .mode = mode,
                               .words = point.immediate ? 2U : 1U,
                               .cycles = point.cycles};
  const a64::Program program(input);
  std::array<Word, memory_words> data{};
  data.fill(incoming);
  auto expected_data = data;
  auto initial = register_model::seed(0);
  initial.sr = mode;
  initial.r[0] = location;
  a64::Expected expected{
      .reg = initial, .cycles = point.cycles, .boundary = detail::NativeBoundary::owner};
  expected.reg.pc += input.words;
  if (!point.immediate) { ++expected.reg.r[0]; }
  const Fault failure = accept
                            ? Fault{}
                            : Fault{.kind = point.write ? FaultKind::io_write : FaultKind::io_read,
                                    .space = point.peripheral,
                                    .address = low_port + 5,
                                    .value = point.write ? payload : 0U,
                                    .pc = start};
  const std::array events{a64::BusEvent{.state = expected.reg,
                                        .failure = failure,
                                        .space = point.peripheral,
                                        .address = low_port + 5,
                                        .value = point.write ? payload : incoming,
                                        .write = point.write}};
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, events);
  auto binding = bus.binding();
  binding.program = {.data = data.data(), .words = data.size()};
  invocation.bind(
      {{{.data = data.data(), .words = data.size()}, {.data = data.data(), .words = data.size()}}},
      binding);
  if (!accept) {
    fail(expected, failure);
  } else if (!point.write) {
    expected_data[location] = payload;
    if (point.memory == Space::program) { expected.program_written = true; }
  }
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
  require(data == expected_data,
          "Peripheral transfers preserve all actual RAM words on success and failure");
}

void memory_ports() {
  constexpr std::array points{
      MemoryPort{.opcode = 0x075805},
      MemoryPort{.opcode = 0x07d805, .write = true},
      MemoryPort{.opcode = 0x075845, .memory = Space::y},
      MemoryPort{.opcode = 0x07d845, .memory = Space::y, .write = true},
      MemoryPort{.opcode = 0x071885, .peripheral = Space::y},
      MemoryPort{.opcode = 0x079885, .peripheral = Space::y, .write = true},
      MemoryPort{.opcode = 0x009805, .memory = Space::program, .cycles = 6},
      MemoryPort{.opcode = 0x00d805, .memory = Space::program, .cycles = 6, .write = true},
      MemoryPort{.opcode = 0x07f405, .write = true, .immediate = true},
      MemoryPort{.opcode = 0x07b485, .peripheral = Space::y, .write = true, .immediate = true}};
  for (const auto point : points) {
    for (const Word mode : {0U, sa, sa | mode_up, saturation_model::mode | sa | mode_down}) {
      for (const bool accept : {false, true}) { memory_port(point, mode, accept); }
    }
  }
}

void remapping() {
  constexpr Word operation = 0x075805;
  const a64::Instruction input{.opcode = operation, .cycles = 2};
  const a64::Program program(input);
  for (const bool writable : {false, true}) {
    std::array<Word, 4> old_data{3, 5, 7, 9};
    constexpr std::array<Word, 4> replacement{11, 13, 15, 17};
    auto new_data = replacement;
    std::array<std::uint8_t, 4> known{};
    detail::NativeWindow window{.address = location,
                                .words = old_data.size(),
                                .data = old_data.data(),
                                .known = known.data(),
                                .writable = true};
    auto initial = register_model::seed(0);
    initial.sr = 0;
    initial.r[0] = location;
    a64::Expected expected{.reg = initial, .cycles = 2, .boundary = detail::NativeBoundary::owner};
    ++expected.reg.pc;
    ++expected.reg.r[0];
    const std::array events{
        a64::BusEvent{.state = expected.reg, .address = low_port + 5, .value = incoming},
        a64::BusEvent{.state = expected.reg, .address = location, .value = payload, .write = true}};
    a64::Invocation invocation(initial);
    a64::Bus bus(invocation, std::span<const a64::BusEvent>(events).first(writable ? 1U : 2U));
    bus.remap(window, new_data.data(), writable);
    auto binding = bus.binding(true);
    binding.windows[0] = {.data = &window, .count = 1};
    invocation.bind({}, binding);
    invocation.run(program);
    invocation.expect(expected, operation);
    bus.expect();
    require(old_data[0] == 3 && new_data[0] == (writable ? payload : 11U) &&
                known[0] == (writable ? 1 : 0),
            "A callback's new data pointer and write permission own the same instruction's later "
            "transfer");
  }
}

void live_prefix() {
  const std::array steps{
      a64::Instruction{.opcode = 0x2000d2},
      a64::Instruction{.opcode = opcode({.reg = 14, .write = true}), .pc = start + 1},
      a64::Instruction{.opcode = 0x200013, .pc = start + 2}};
  const a64::Program program(steps);
  auto initial = register_model::seed(0);
  initial.sr = status_byte;
  a64::Expected expected{
      .reg = initial, .retired = 2, .cycles = 2, .boundary = detail::NativeBoundary::owner};
  constexpr Word mac = 0xd2;
  a64_alu_model::apply(expected.reg, mac);
  expected.reg.pc += 2;
  const auto value = register_model::read(expected.reg, 14);
  const std::array events{a64::BusEvent{.state = expected.reg,
                                        .address = high_port,
                                        .pc = start + 1,
                                        .value = value,
                                        .instructions = 1,
                                        .cycles = 1,
                                        .write = true}};
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, events);
  invocation.bind({}, bus.binding());
  invocation.run(program, 3);
  invocation.expect(expected, steps.front().opcode);
  bus.expect();
}

void full_memory_io_priority() {
  const a64::Instruction input{.opcode = 0x44e000};
  const a64::Program program(input);
  std::vector<Word> memory(address_words);
  auto initial = register_model::seed(0);
  initial.sr = 0;
  initial.r[0] = low_port;
  a64::Expected expected{.reg = initial, .boundary = detail::NativeBoundary::owner};
  ++expected.reg.pc;
  const std::array events{
      a64::BusEvent{.state = expected.reg, .address = low_port, .value = incoming}};
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, events);
  invocation.bind({{{.data = memory.data(), .words = memory.size()}, {}}}, bus.binding());
  register_model::write(expected.reg, 4, payload);
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
  require(memory[low_port] == 0,
          "Peripheral addresses retain I/O ownership even in a full 24-bit RAM span");
}

void custom_failure() {
  const a64::Instruction input{.opcode = opcode({})};
  const a64::Program program(input);
  auto initial = register_model::seed(0);
  initial.sr = 0;
  a64::Expected expected{.reg = initial};
  ++expected.reg.pc;
  constexpr Fault failure{.kind = FaultKind::host_event,
                          .space = Space::program,
                          .address = 0x123,
                          .value = 0xabcdef,
                          .pc = 0x456};
  const std::array events{a64::BusEvent{
      .state = expected.reg, .failure = failure, .address = high_port, .value = incoming}};
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, events);
  invocation.bind({}, bus.binding());
  fail(expected, failure);
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
}

void empty_grant() {
  const a64::Instruction input{.opcode = 0x07d805, .cycles = 2};
  const a64::Program program(input);
  auto initial = register_model::seed(0);
  initial.sr = 0;
  initial.r[0] = location;
  std::array<Word, memory_words> data{};
  a64::Invocation invocation(initial);
  a64::Bus bus(invocation, {});
  invocation.bind({{{.data = data.data(), .words = data.size()}, {}}}, bus.binding());
  invocation.run(program, 0);
  invocation.expect(
      {.reg = initial, .retired = 0, .cycles = 0, .inhibited = true, .repeat_completed = true},
      input.opcode);
  bus.expect();
}

void cached_program_bus() {
  constexpr Word cache_enabled = Word{1} << 19;
  constexpr CacheRegion cached{.begin = 16, .end = 20};
  for (const Word status : {0U, cache_enabled}) {
    for (const Word cursor : {15U, 16U, 19U, 20U}) {
      constexpr Word movem_short_load = 0x078004;
      const a64::Instruction input{.opcode = movem_short_load | (cursor << 8), .cycles = 6};
      const a64::Program program(input);
      std::array<Word, 64> memory{};
      auto initial = register_model::seed(0);
      initial.sr = status;
      a64::Expected expected{.reg = initial, .cycles = 6};
      ++expected.reg.pc;
      const bool owned = status != 0 && cursor >= cached.begin && cursor < cached.end;
      const std::array events{a64::BusEvent{
          .state = expected.reg, .space = Space::program, .address = cursor, .value = incoming}};
      a64::Invocation invocation(initial);
      a64::Bus bus(invocation,
                   owned ? std::span<const a64::BusEvent>(events)
                         : std::span<const a64::BusEvent>{});
      auto binding = bus.binding();
      binding.program = {.data = memory.data(), .words = memory.size()};
      binding.cache = cached;
      invocation.bind({}, binding);
      register_model::write(expected.reg, 4, owned ? payload : 0);
      if (owned) { expected.boundary = detail::NativeBoundary::owner; }
      invocation.run(program);
      invocation.expect(expected, input.opcode);
      bus.expect();
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::ports();
    dsp56300::test::memory_ports();
    dsp56300::test::remapping();
    dsp56300::test::live_prefix();
    dsp56300::test::full_memory_io_priority();
    dsp56300::test::custom_failure();
    dsp56300::test::empty_grant();
    dsp56300::test::cached_program_bus();
    std::cout << dsp56300::test::checks << " native A64 I/O order/state/remapping checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
