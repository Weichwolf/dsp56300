// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word start = 16, destination = 32, value = 0x123456;
constexpr Word load = static_cast<Word>(isa::Opcode::move_x_absolute_short) | (1U << 15);
constexpr std::array prefix{load, static_cast<Word>(isa::Opcode::jmp_absolute_short) | destination};
constexpr std::array body{load, static_cast<Word>(isa::Opcode::stop)};
constexpr CodeTransition transfer{.from = 0, .to = 1, .entry = destination};
constexpr std::array memory{RegionLayout{.space = Space::x, .first = 0, .size = 1}};

NativeImage compile(const std::span<const RegionLayout> layout = memory) {
  const std::array first{CodeSpan{.address = start, .words = prefix}};
  const std::array second{CodeSpan{.address = destination, .words = body}};
  constexpr std::array before{start};
  constexpr std::array after{destination};
  const std::array variants{
      CodeVariant{.id = 0, .code = first, .entries = before, .failed_fetches = {}},
      CodeVariant{.id = 1, .code = second, .entries = after, .failed_fetches = {}}};
  const std::array layouts{VariantLayout{.id = 1, .memory = layout}};
  return Core::compile({.variants = variants},
                       {.memory = layouts,
                        .interrupts = {},
                        .transitions = std::span(&transfer, 1),
                        .callbacks = {},
                        .bindings = {}});
}

struct Bus final : Io {
  Bus() {
    std::ranges::copy(prefix, program.begin() + start);
    std::ranges::copy(body, program.begin() + destination);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program.size() ? std::optional<Word>{program[address]} : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    window = {.words = data,
              .known = tracked ? std::span(known) : std::span<std::uint8_t>{},
              .writable = true};
    return space == Space::x && granted ? std::span(&window, 1) : std::span<const MemoryWindow>{};
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    ++reads;
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  std::array<Word, 64> program{};
  std::array<Word, 1> data{value};
  std::array<std::uint8_t, 1> known{1};
  MemoryWindow window{};
  bool tracked = true, granted = true;
  unsigned reads = 0;
};

bool same(const State &left, const State &right) {
  return left.registers == right.registers && left.failure == right.failure &&
         left.execution == right.execution && left.progress == right.progress;
}

void mixed_layouts(const NativeImage &image) {
  Bus bus;
  Core core(Memory::external(), &bus);
  auto initial = core.registers();
  initial.pc = start;
  core.set_registers(initial);
  core.install(image, 0);
  const auto prefix_result = core.run();
  auto expected = initial;
  expected.pc = destination;
  expected.x = value;
  require(prefix_result.retired == 2 && prefix_result.yielded && prefix_result.fault == Fault{} &&
              core.registers() == expected && bus.reads == 0,
          "An omitted layout keeps actual tracked grants and completes the native prefix once");
  const auto boundary = core.state();
  bool rejected = false;
  try {
    core.install(image, 1);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && same(boundary, core.state()) && core.run().retired == 0,
          "A wrong destination layout preserves state and its pending handoff");
  bus.tracked = false;
  core.install(image, 1);
  require(same(boundary, core.state()), "A valid fixed layout changes no architectural state");
  const auto result = core.run();
  expected.pc = destination + body.size();
  require(result.retired == 2 && result.fault == Fault{} &&
              result.state == ExecutionState::stopped && core.registers() == expected &&
              core.progress().instructions == 4 && bus.reads == 0,
          "The fixed destination executes original data and STOP without replay or callbacks");
}

void unknown_read(const NativeImage &image) {
  Bus bus;
  bus.known[0] = 0;
  Core core(Memory::external(), &bus);
  auto initial = core.registers();
  initial.pc = start;
  core.set_registers(initial);
  core.install(image, 0);
  const auto result = core.run();
  ++initial.pc; // The failed read retains the already completed instruction fetch.
  require(
      result.retired == 0 && core.registers() == initial &&
          result.fault ==
              Fault{.kind = FaultKind::memory_read, .space = Space::x, .address = 0, .pc = start},
      "Omitting a layout does not fabricate initialized data in tracked RAM");
}

void empty_layout() {
  const auto image = compile({});
  Bus bus;
  Core core(Memory::external(), &bus);
  auto initial = core.registers();
  initial.pc = start;
  core.set_registers(initial);
  core.install(image, 0);
  require(core.run().yielded, "The generic prefix still accepts its actual grants");
  const auto boundary = core.state();
  bool rejected = false;
  try {
    core.install(image, 1);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && same(boundary, core.state()),
          "An explicitly empty layout requires zero grants, unlike an omitted layout");
  bus.granted = false;
  core.install(image, 1);
  require(same(boundary, core.state()) && core.run().fault.kind == FaultKind::memory_read,
          "A zero-grant destination binds but cannot invent a readable X word");
}
}

int main() {
  try {
    const auto image = compile();
    mixed_layouts(image);
    unknown_read(image);
    empty_layout();
    std::cout << test::checks << " per-variant memory layout checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
