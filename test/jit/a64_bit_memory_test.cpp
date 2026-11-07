// SPDX-License-Identifier: GPL-3.0-only
#include "a64_bus.hpp"
#include "a64_native.hpp"
#include "address_model.hpp"
#include "bit_access_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>

namespace dsp56300::test {
namespace {
using Input = bit_model::Input;
using Flow = bit_model::Flow;
using Location = bit_model::Location;
using Ram = std::array<Word, memory_words>;
constexpr Word location = 384, io_begin = 0xffff80, high_begin = 0xffffc0, tag = 0xab000000;
constexpr std::array actions{BitAction::test, BitAction::clear, BitAction::set, BitAction::change};

struct Point {
  Input input{.location = Location::short_address, .selector = 12};
  address_model::Input agu{
      .modifier = address_model::linear, .cursor = location, .offset = 1, .mode = 4};
  Word mode = 0;
  Word stack_count = 2, stack_pointer = 2;
  bool previous = false, external = false, executable = false;
};

Registers seed(const Point point) {
  auto state = register_model::seed(point.mode);
  state.sr = point.mode | (status_byte & ~flag_c) | (point.previous ? 0U : flag_c);
  state.r.at(point.agu.index) = point.agu.cursor;
  state.n.at(point.agu.index) = point.agu.offset;
  state.m.at(point.agu.index) = point.agu.modifier;
  state.sc = point.stack_count;
  state.sp = point.stack_pointer;
  return state;
}

a64::Instruction instruction(const Point point) {
  return {.opcode = bit_model::opcode(point.input),
          .extension = point.input.extension,
          .mode = point.mode,
          .words = bit_model::words(point.input),
          .cycles = bit_model::cycles(point.input)};
}

struct Address {
  a64::Expected state;
  Word address = 0;
};

Address prepare(const Point point) {
  Address result{.state = {.reg = seed(point), .cycles = bit_model::cycles(point.input)},
                 .address = point.input.selector};
  auto &state = result.state;
  state.reg.pc += bit_model::words(point.input);
  if (point.input.bit >= word_bits) {
    state.failure =
        bit_model::failure(FaultKind::invalid_instruction, bit_model::opcode(point.input));
    return result;
  }
  if (point.input.location != Location::effective) { return result; }
  constexpr Word unsupported = 0x026000;
  if ((point.mode & unsupported) != 0 ||
      (point.mode & (mode_up | mode_down)) == (mode_up | mode_down)) {
    state.failure = bit_model::failure(FaultKind::unsupported_mode, state.reg.sr);
    return result;
  }
  const auto mode = point.agu.mode;
  if (mode == 6) {
    result.address = point.input.extension;
    if (point.agu.index != 0 || point.input.flow != Flow::ordinary) {
      state.failure =
          bit_model::failure(FaultKind::invalid_instruction, bit_model::opcode(point.input));
    }
    return result;
  }
  result.address = point.agu.cursor;
  if (mode == 4) { return result; }
  const auto modifier = point.agu.modifier % 65536;
  if (modifier >= 32768 && modifier != address_model::linear) {
    const auto length = (modifier % 16384) + 1;
    constexpr Word reserved = 49152;
    if (modifier >= reserved || length == 1 || !std::has_single_bit(length)) {
      state.failure = bit_model::failure(FaultKind::unsupported_mode, modifier);
      return result;
    }
  }
  const auto next = address_model::updated(point.agu);
  if (mode != 5) { state.reg.r.at(point.agu.index) = next; }
  if (mode == 5 || mode == 7) { result.address = next; }
  return result;
}

void failure(a64::Expected &state) {
  if (state.failure.kind == FaultKind::none) { return; }
  state.retired = 0;
  state.cycles = 0;
  state.repeat_completed = true;
  state.boundary = detail::NativeBoundary::owner;
}

struct Memory {
  Ram &x;
  Ram &y;
};

void ram_effects(Address &expected, const Point point, const Memory memory) {
  if (expected.state.failure.kind != FaultKind::none) { return; }
  if (expected.address >= memory_words) {
    expected.state.failure = {.kind = FaultKind::memory_read,
                              .space = point.input.y ? Space::y : Space::x,
                              .address = expected.address,
                              .pc = start};
    return;
  }
  const auto access = point.input.flow == Flow::ordinary ? point.input.action : BitAction::test;
  if (access != BitAction::test) {
    auto &target = point.input.y ? memory.y : memory.x;
    target.at(expected.address) = bit_model::changed(point.previous ? word_mask : 0U,
                                                     {.action = access, .bit = point.input.bit});
    if (point.executable) {
      expected.state.program_written = true;
      expected.state.boundary = detail::NativeBoundary::owner;
    }
  }
  expected.state.failure = bit_model::finish(expected.state.reg, point.input, point.previous);
}

void ram(const Point point) {
  const auto input = instruction(point);
  const a64::Program program(input);
  auto expected = prepare(point);
  Ram x{};
  Ram y{};
  const auto before = tag | (point.previous ? word_mask : 0U);
  x.fill(before);
  y.fill(before);
  auto expected_x = x;
  auto expected_y = y;
  ram_effects(expected, point, {.x = expected_x, .y = expected_y});
  failure(expected.state);
  a64::Invocation invocation(seed(point));
  std::array<detail::NativeWindow, 2> windows{
      {{.words = x.size(), .data = x.data(), .writable = true},
       {.words = y.size(), .data = y.data(), .writable = true}}};
  detail::NativeBus binding{.external = point.external};
  binding.windows[0] = {.data = windows.data(), .count = 1};
  binding.windows[1] = {.data = windows.data() + 1, .count = 1};
  if (point.executable) {
    binding.program = {.data = point.input.y ? y.data() : x.data(), .words = x.size()};
  }
  invocation.bind({{{.data = x.data(),
                     .words = x.size(),
                     .aliases_program = point.executable && !point.input.y},
                    {.data = y.data(),
                     .words = y.size(),
                     .aliases_program = point.executable && point.input.y}}},
                  binding);
  invocation.run(program);
  invocation.expect(expected.state, input.opcode);
  require(x == expected_x && y == expected_y,
          "Native bit RAM effects preserve every other word and raw BTST storage");
}

void ram_variants(Point point) {
  for (const bool previous : {false, true}) {
    for (const bool y : {false, true}) {
      point.previous = previous;
      point.input.y = y;
      ram(point);
    }
  }
}

void matrix() {
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (const auto action : actions) {
        for (const bool external : {false, true}) {
          ram_variants({.input = {.location = Location::effective,
                                  .action = action,
                                  .selector = (mode * 8) + index,
                                  .bit = index % word_bits,
                                  .extension = location},
                        .agu = {.modifier = address_model::linear,
                                .cursor = location,
                                .offset = 1,
                                .mode = mode,
                                .index = index},
                        .external = external});
        }
      }
    }
  }
  for (const auto flow : {Flow::ordinary, Flow::jump, Flow::relative}) {
    for (const auto action : actions) {
      for (const bool call : {false, true}) {
        for (const bool wanted : {false, true}) {
          ram_variants({.input = {.location = Location::short_address,
                                  .flow = flow,
                                  .action = action,
                                  .selector = 12,
                                  .bit = word_bits - 1,
                                  .call = call,
                                  .wanted = wanted}});
          ram_variants(
              {.input = {.location = Location::effective,
                         .flow = flow,
                         .action = action,
                         .selector = 3 * 8,
                         .bit = 7,
                         .call = call,
                         .wanted = wanted},
               .agu = {
                   .modifier = address_model::linear, .cursor = location, .offset = 1, .mode = 3}});
        }
      }
    }
  }
  for (const auto action : actions) {
    ram_variants(
        {.input = {.location = Location::short_address, .action = action, .selector = 12, .bit = 0},
         .executable = true});
    ram_variants(
        {.input = {.location = Location::effective, .action = action, .selector = 3 * 8, .bit = 0},
         .agu = {
             .modifier = address_model::linear, .cursor = memory_words, .offset = 1, .mode = 3}});
  }
}

struct AtomicPoint {
  Point point;
  bool accept = true, callback = true, io = true;
};

class AtomicIo final : public Io {
public:
  AtomicIo(const a64::Invocation &invocation, const AtomicPoint point, const Registers &visible)
      : invocation_(invocation), point_(point), visible_(visible) {}

  [[nodiscard]] detail::NativeBus binding() noexcept {
    return {.owner = this,
            .access = point_.callback ? access : nullptr,
            .external = point_.point.external};
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    valid_ = false;
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    valid_ = false;
    return false;
  }

  bool bit(const Space space,
           const Word address,
           const Word pc,
           const Word bit,
           const BitAction action,
           bool &previous) noexcept override {
    ++calls_;
    auto physical = visible_;
    constexpr unsigned padding = 8;
    physical.a <<= padding;
    physical.b <<= padding;
    const auto &cpu = invocation_.cpu();
    const auto &input = point_.point.input;
    const auto expected_action = input.flow == Flow::ordinary ? input.action : BitAction::test;
    valid_ = valid_ && calls_ == 1 && space == (input.y ? Space::y : Space::x) &&
             address == address_ && pc == start && bit == input.bit && action == expected_action &&
             cpu.reg == physical && cpu.instructions == std::numeric_limits<std::uint64_t>::max() &&
             cpu.cycles == cpu.instructions && !cpu.interrupt_inhibited && cpu.repeat_completed &&
             cpu.processing == ExecutionState::running;
    previous = point_.point.previous;
    return point_.accept;
  }

  void bind(const Word address) { address_ = address; }

  void expect() const {
    require(valid_ && calls_ == (point_.callback && point_.io ? 1U : 0U),
            "One exact atomic Io::bit callback with canonical pre-commit state");
  }

private:
  static void access(void *const owner, detail::NativeAccess &request) noexcept {
    auto &self = *static_cast<AtomicIo *>(owner);
    const auto &input = self.point_.point.input;
    constexpr std::array bus_actions{detail::BusAction::bit_test,
                                     detail::BusAction::bit_clear,
                                     detail::BusAction::bit_set,
                                     detail::BusAction::bit_change};
    const auto action = input.flow == Flow::ordinary ? input.action : BitAction::test;
    self.valid_ = self.valid_ && request.action == bus_actions[static_cast<std::size_t>(action)] &&
                  request.value == input.bit && request.address == self.address_ &&
                  request.failure == Fault{};
    detail::access_bit_io(self.point_.io ? &self : nullptr, request);
  }

  const a64::Invocation &invocation_;
  AtomicPoint point_;
  Registers visible_;
  Word address_ = 0;
  unsigned calls_ = 0;
  bool valid_ = true;
};

void atomic(const AtomicPoint sample) {
  const auto &point = sample.point;
  const auto input = instruction(point);
  const a64::Program program(input);
  auto expected = prepare(point);
  if (point.input.location == Location::low) { expected.address = io_begin + point.input.selector; }
  if (point.input.location == Location::high) {
    expected.address = high_begin + point.input.selector;
  }
  const auto visible = expected.state.reg;
  expected.state.boundary = detail::NativeBoundary::owner;
  if (!sample.accept || !sample.callback || !sample.io) {
    expected.state.failure = {.kind = FaultKind::io_bit,
                              .space = point.input.y ? Space::y : Space::x,
                              .address = expected.address,
                              .value = point.input.bit,
                              .pc = start};
  } else {
    expected.state.failure = bit_model::finish(expected.state.reg, point.input, point.previous);
  }
  failure(expected.state);
  a64::Invocation invocation(seed(point));
  AtomicIo io(invocation, sample, visible);
  io.bind(expected.address);
  invocation.bind({}, io.binding());
  invocation.run(program);
  invocation.expect(expected.state, input.opcode);
  io.expect();
}

void atomic_variants(AtomicPoint sample) {
  for (const bool previous : {false, true}) {
    for (const bool accept : {false, true}) {
      sample.point.previous = previous;
      sample.accept = accept;
      atomic(sample);
    }
  }
}

void ports() {
  for (unsigned port = 0; port < 128; ++port) {
    for (const auto flow : {Flow::ordinary, Flow::jump, Flow::relative}) {
      for (const auto action : actions) {
        atomic_variants({.point = {.input = {.location = port < 64 ? Location::low : Location::high,
                                             .flow = flow,
                                             .action = action,
                                             .selector = port % 64,
                                             .bit = port % word_bits,
                                             .call = (port & 1U) != 0,
                                             .wanted = (port & 2U) != 0,
                                             .y = (port & 4U) != 0},
                                   .external = (port & 8U) != 0}});
      }
    }
  }
  for (const auto action : actions) {
    const Point point{
        .input = {.location = Location::effective, .action = action, .selector = 3 * 8, .bit = 23},
        .agu = {.modifier = address_model::linear, .cursor = io_begin, .offset = 1, .mode = 3}};
    atomic_variants({.point = point});
    atomic({.point = point, .callback = false});
    atomic({.point = point, .io = false});
  }
}

void ram_faults() {
  for (const Word mode : {0U,
                          saturation_model::mode,
                          mode_up,
                          mode_down,
                          Word{1} << 13,
                          Word{1} << 14,
                          Word{1} << 17,
                          mode_up | mode_down}) {
    for (unsigned bit = 0; bit < 32; ++bit) {
      for (const auto action : actions) {
        ram_variants({.input = {.location = Location::short_address,
                                .action = action,
                                .selector = 12,
                                .bit = bit},
                      .mode = mode});
        ram_variants(
            {.input =
                 {.location = Location::effective, .action = action, .selector = 3 * 8, .bit = bit},
             .agu = {.modifier = address_model::linear, .cursor = location, .offset = 1, .mode = 3},
             .mode = mode});
      }
    }
  }
  for (const auto flow : {Flow::ordinary, Flow::relative}) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      ram_variants(
          {.input = {.location = Location::effective,
                     .flow = flow,
                     .selector = (6 * 8) + index,
                     .call = true},
           .agu = {
               .modifier = address_model::linear, .cursor = location, .mode = 6, .index = index}});
    }
  }
  for (const Word modifier : {0U, 3U, 0x8003U, 0x8000U, 0xc001U}) {
    ram_variants({.input = {.location = Location::effective, .selector = 3 * 8},
                  .agu = {.modifier = modifier, .cursor = location, .offset = 1, .mode = 3}});
  }
  for (const Word count : {0U, static_cast<Word>(stack_entries - 1)}) {
    atomic_variants({.point = {.input = {.location = Location::high,
                                         .flow = Flow::relative,
                                         .selector = 3,
                                         .bit = 0,
                                         .call = true,
                                         .wanted = true},
                               .stack_count = count}});
  }
}

void displaced_aliases() {
  constexpr Word base = 0x0a7080;
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (const bool y : {false, true}) {
      for (const bool load : {false, true}) {
        const auto opcode = base | (index << 8) | (y ? 0x10000U : 0U) | (load ? 0x40U : 0U);
        const a64::Program program(
            {.opcode = opcode, .extension = location, .words = 2, .cycles = 3});
        const auto initial = seed({});
        a64::Expected expected{.reg = initial,
                               .retired = 0,
                               .cycles = 0,
                               .failure =
                                   bit_model::failure(FaultKind::invalid_instruction, opcode),
                               .boundary = detail::NativeBoundary::owner,
                               .repeat_completed = true};
        expected.reg.pc += 2;
        a64::Invocation invocation(initial);
        invocation.run(program);
        invocation.expect(expected, opcode);
      }
    }
  }
}

void owned_rmw(const BitAction action, const bool writable) {
  const Point point{
      .input = {.location = Location::effective, .action = action, .selector = 3 * 8},
      .agu = {.modifier = address_model::linear, .cursor = location, .offset = 1, .mode = 3},
      .previous = true,
      .external = true};
  const a64::Program program(instruction(point));
  constexpr std::array<Word, 4> original{3, 5, 7, 9};
  auto old = original;
  auto replacement = original;
  std::array<std::uint8_t, 4> known{};
  detail::NativeWindow window{.address = location,
                              .words = old.size(),
                              .data = old.data(),
                              .known = known.data(),
                              .writable = true};
  auto expected = prepare(point).state;
  const auto visible = expected.reg;
  const bool modify = action != BitAction::test;
  const auto value = bit_model::changed(word_mask, {.action = action, .bit = 0});
  const Fault rejected{.kind = FaultKind::memory_write,
                       .space = Space::x,
                       .address = location,
                       .value = value,
                       .pc = start};
  expected.boundary = detail::NativeBoundary::owner;
  if (modify && !writable) {
    expected.failure = rejected;
  } else {
    expected.failure = bit_model::finish(expected.reg, point.input, true);
  }
  failure(expected);
  const std::array events{a64::BusEvent{.state = visible,
                                        .address = location,
                                        .value = tag | word_mask,
                                        .observed = old.data(),
                                        .observed_value = original[0]},
                          a64::BusEvent{.state = visible,
                                        .failure = rejected,
                                        .address = location,
                                        .value = value,
                                        .write = true,
                                        .observed = replacement.data(),
                                        .observed_value = original[0]}};
  a64::Invocation invocation(seed(point));
  a64::Bus bus(invocation,
               std::span<const a64::BusEvent>(events).first(modify && !writable ? 2U : 1U));
  bus.remap(window, replacement.data(), writable);
  auto binding = bus.binding(true);
  binding.windows[0] = {.data = &window, .count = 1};
  invocation.bind({}, binding);
  invocation.run(program);
  invocation.expect(expected, bit_model::opcode(point.input));
  bus.expect();
  require(old == original && replacement[0] == (modify && writable ? value : original[0]) &&
              known[0] == (modify && writable ? 1 : 0),
          "Bit RMW rechecks callback mappings and permissions before writes and commits C only "
          "after success");
}
}
}

int main() {
  try {
    dsp56300::test::matrix();
    dsp56300::test::ports();
    dsp56300::test::ram_faults();
    dsp56300::test::displaced_aliases();
    for (const auto action : dsp56300::test::actions) {
      dsp56300::test::owned_rmw(action, false);
      dsp56300::test::owned_rmw(action, true);
    }
    std::cout << dsp56300::test::checks
              << " independent native A64 memory/atomic-bit checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
