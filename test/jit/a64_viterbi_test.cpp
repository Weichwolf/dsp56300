// SPDX-License-Identifier: GPL-3.0-only
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
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
using Ram = std::array<Word, memory_words>;
constexpr Word base = 0x0ac0c0, location = 384, io_begin = 0xffff80;
constexpr std::uint64_t pattern = 0xa5123456654321;
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};

struct Point {
  address_model::Input agu{
      .modifier = address_model::linear, .cursor = location, .offset = 3, .mode = 4};
  Word status = status_byte;
  std::uint64_t value = pattern;
  unsigned reject = 0;
  bool source_b = false, insert_one = false, external = false, aliases = false, executable = false,
       missing_y = false;
};

a64::Instruction instruction(const Point point) {
  return {.opcode = base | (point.source_b ? 0x010000U : 0U) | (point.insert_one ? 16U : 0U) |
                    (point.agu.mode << 11) | (point.agu.index << 8),
          .extension = point.agu.mode == 6 ? point.agu.cursor : 0U,
          .mode = point.status,
          .words = point.agu.mode == 6 ? 2U : 1U,
          .cycles = point.agu.mode >= 5 ? 2U : 1U};
}

Registers seed(const Point point) {
  auto initial = register_model::seed(point.status);
  initial.sr = point.status;
  (point.source_b ? initial.b : initial.a) = point.value;
  initial.r.at(point.agu.index) = point.agu.cursor;
  initial.n.at(point.agu.index) = point.agu.offset;
  initial.m.at(point.agu.index) = point.agu.modifier;
  return initial;
}

Fault fault(const FaultKind kind, const Word value) {
  return {.kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
}

Fault address(Registers &state, const Point point) {
  constexpr Word unsupported = 0x026000;
  if ((point.status & unsupported) != 0 ||
      (point.status & (mode_up | mode_down)) == (mode_up | mode_down)) {
    return fault(FaultKind::unsupported_mode, point.status);
  }
  const auto mode = point.agu.mode;
  if (mode == 6) {
    return point.agu.index == 0 ? Fault{}
                                : fault(FaultKind::invalid_instruction, instruction(point).opcode);
  }
  if (mode == 4) { return {}; }
  const auto modifier = point.agu.modifier % 65536;
  if (modifier != address_model::linear && modifier >= 32768) {
    const auto length = (modifier % 16384) + 1;
    constexpr Word reserved = 49152;
    if (modifier >= reserved || length == 1 || !std::has_single_bit(length)) {
      return fault(FaultKind::unsupported_mode, modifier);
    }
  }
  const auto next = address_model::updated(point.agu);
  if (mode != 5) { state.r.at(point.agu.index) = next; }
  return {};
}

Word effective(const Point point) {
  return point.agu.mode == 5 || point.agu.mode == 7 ? address_model::updated(point.agu)
                                                    : point.agu.cursor;
}

struct Words {
  Word high = 0, low = 0;
};

Words output(const Point point) {
  Words result{.low = point.insert_one ? 1U : 0U};
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    if (((point.value / (std::uint64_t{1} << (bit + word_bits))) % 2) != 0) {
      result.high += Word{1} << bit;
    }
    if (bit + 1 < word_bits && ((point.value / (std::uint64_t{1} << bit)) % 2) != 0) {
      result.low += Word{1} << (bit + 1);
    }
  }
  return result;
}

struct Store {
  Space space;
  Word address, value;
  unsigned half;
};

class Model {
public:
  explicit Model(const Point point)
      : point_(point), expected_{.reg = seed(point), .cycles = instruction(point).cycles} {
    x_.fill(word_mask);
    y_.fill(word_mask);
    expected_.failure = address(expected_.reg, point);
    expected_.reg.pc += instruction(point).words;
    if (expected_.failure.kind == FaultKind::none) {
      const auto value = output(point);
      store({.space = Space::x, .address = effective(point), .value = value.high, .half = 1});
      if (expected_.failure.kind == FaultKind::none) {
        store({.space = Space::y, .address = effective(point), .value = value.low, .half = 2});
      }
    }
    if (expected_.failure.kind != FaultKind::none) {
      expected_.retired = 0;
      expected_.cycles = 0;
      expected_.repeat_completed = true;
      expected_.boundary = detail::NativeBoundary::owner;
    }
  }

  [[nodiscard]] const a64::Expected &state() const { return expected_; }

  [[nodiscard]] const Ram &x() const { return x_; }

  [[nodiscard]] const Ram &y() const { return y_; }

  [[nodiscard]] const std::vector<a64::BusEvent> &events() const { return events_; }

private:
  void store(const Store input) {
    const bool peripheral = input.address >= io_begin;
    const bool missing =
        input.address >= memory_words || (input.space == Space::y && point_.missing_y);
    if (peripheral || missing) {
      const bool reject = !peripheral || input.half == point_.reject;
      const Fault failure{.kind = peripheral ? FaultKind::io_write : FaultKind::memory_write,
                          .space = input.space,
                          .address = input.address,
                          .value = input.value,
                          .pc = start};
      if (reject) { expected_.failure = failure; }
      if (peripheral || point_.external) {
        events_.push_back({.state = expected_.reg,
                           .failure = expected_.failure,
                           .space = input.space,
                           .address = input.address,
                           .value = input.value,
                           .write = true});
        expected_.boundary = detail::NativeBoundary::owner;
      }
      return;
    }
    auto &memory = input.space == Space::x || point_.aliases ? x_ : y_;
    memory.at(input.address) = input.value;
    if (point_.executable && (input.space == Space::x || point_.aliases)) {
      expected_.program_written = true;
      expected_.boundary = detail::NativeBoundary::owner;
    }
  }

  Point point_;
  a64::Expected expected_;
  Ram x_{}, y_{};
  std::vector<a64::BusEvent> events_;
};

void probe(const Point point) {
  const auto input = instruction(point);
  const a64::Program program(input);
  const Model expected(point);
  Ram x{};
  Ram y{};
  x.fill(word_mask);
  y.fill(word_mask);
  a64::Invocation invocation(seed(point));
  a64::Bus bus(invocation, expected.events());
  auto binding = bus.binding(point.external);
  const auto ywords = point.missing_y ? 0U : y.size();
  const std::array windows{
      detail::NativeWindow{.words = x.size(), .data = x.data(), .writable = true},
      detail::NativeWindow{
          .words = ywords, .data = point.aliases ? x.data() : y.data(), .writable = true}};
  binding.windows[0] = {.data = windows.data(), .count = 1};
  binding.windows[1] = {.data = windows.data() + 1, .count = 1};
  if (point.executable) { binding.program = {.data = x.data(), .words = x.size()}; }
  invocation.bind({{{.data = x.data(), .words = x.size(), .aliases_program = point.executable},
                    {.data = point.aliases ? x.data() : y.data(),
                     .words = ywords,
                     .aliases_program = point.executable && point.aliases}}},
                  binding);
  invocation.run(program);
  invocation.expect(expected.state(), input.opcode);
  bus.expect();
  require(x == expected.x() && y == expected.y(),
          "VSL preserves all RAM and exact first/second write ordering");
}

void variants(Point point) {
  for (const bool source_b : {false, true}) {
    for (const bool insert_one : {false, true}) {
      point.source_b = source_b;
      point.insert_one = insert_one;
      probe(point);
    }
  }
}

void matrix() {
  constexpr std::array modifiers{0U, 3U, 0x8003U, address_model::linear};
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (const Word modifier : modifiers) {
        for (const bool external : {false, true}) {
          variants({.agu = {.modifier = modifier,
                            .cursor = location,
                            .offset = 1,
                            .mode = mode,
                            .index = index},
                    .external = external});
        }
      }
    }
  }
  for (const auto scale : scales) {
    for (const auto saturation : {0U, saturation_model::mode}) {
      for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
        const auto value = std::uint64_t{1} << bit;
        variants({.status = scale | saturation | status_byte, .value = value});
        variants({.status = scale | saturation, .value = accumulator_mask ^ value});
      }
    }
  }
}

void boundaries() {
  for (const bool external : {false, true}) {
    variants(
        {.agu = {.modifier = address_model::linear, .cursor = memory_words, .offset = 1, .mode = 3},
         .external = external});
    variants(
        {.agu = {.modifier = address_model::linear, .cursor = location, .offset = 1, .mode = 3},
         .external = external,
         .missing_y = true});
    for (Word port = io_begin; port <= word_mask; ++port) {
      for (unsigned reject = 0; reject < 3; ++reject) {
        variants(
            {.agu = {.modifier = address_model::linear, .cursor = port, .offset = 1, .mode = 3},
             .reject = reject,
             .external = external});
      }
    }
    for (const Word modifier : {0x8000U, 0xc001U}) {
      variants(
          {.agu = {.modifier = modifier, .cursor = io_begin, .mode = 3}, .external = external});
    }
  }
  for (const bool aliases : {false, true}) {
    for (const bool executable : {false, true}) {
      variants({.aliases = aliases, .executable = executable});
      variants({.aliases = aliases, .executable = executable, .missing_y = true});
    }
  }
  for (const Word mode : {Word{1} << 13, Word{1} << 14, Word{1} << 17, mode_up | mode_down}) {
    variants({.status = mode | status_byte});
  }
}

void remapping(const bool writable) {
  const Point point{
      .agu = {.modifier = address_model::linear, .cursor = location, .offset = 1, .mode = 3},
      .insert_one = true,
      .external = true};
  const auto input = instruction(point);
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
  a64::Expected expected{.reg = seed(point), .boundary = detail::NativeBoundary::owner};
  ++expected.reg.pc;
  ++expected.reg.r[0];
  const auto values = output(point);
  const std::array events{a64::BusEvent{.state = expected.reg,
                                        .address = location,
                                        .value = values.high,
                                        .write = true,
                                        .observed = old.data(),
                                        .observed_value = original[0]},
                          a64::BusEvent{.state = expected.reg,
                                        .space = Space::y,
                                        .address = location,
                                        .value = values.low,
                                        .write = true,
                                        .observed = replacement.data(),
                                        .observed_value = original[0]}};
  a64::Invocation invocation(seed(point));
  a64::Bus bus(invocation, std::span<const a64::BusEvent>(events).first(writable ? 1U : 2U));
  bus.remap(window, replacement.data(), writable);
  auto binding = bus.binding(true);
  binding.windows[1] = {.data = &window, .count = 1};
  invocation.bind({}, binding);
  invocation.run(program);
  invocation.expect(expected, input.opcode);
  bus.expect();
  require(old == original && replacement[0] == (writable ? values.low : original[0]) &&
              known[0] == (writable ? 1 : 0),
          "VSL Y store sees mapping and permission changes performed by the X callback");
}
}
}

int main() {
  try {
    dsp56300::test::matrix();
    dsp56300::test::boundaries();
    dsp56300::test::remapping(false);
    dsp56300::test::remapping(true);
    std::cout << dsp56300::test::checks << " independent native A64 VSL checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
