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

namespace {
using namespace dsp56300;
using test::require;
constexpr Word start = 16;
constexpr Word vector = 32;
constexpr std::array inserted{static_cast<Word>(isa::Opcode::nop),
                              static_cast<Word>(isa::Opcode::nop)};
constexpr Word omr_selector = 26;
constexpr Word first_value = 0x123456, second_value = 0x654321;
constexpr std::array code{static_cast<Word>(isa::Opcode::movec_immediate) | (1U << 8) |
                              omr_selector,
                          static_cast<Word>(isa::Opcode::move_x_absolute_short) | (1U << 15),
                          static_cast<Word>(isa::Opcode::stop)};

struct Bus final : Io {
  Bus() {
    std::ranges::copy(code, program.begin() + start);
    std::ranges::copy(inserted, program.begin() + vector);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program.size() ? std::optional<Word>{program[address]} : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    ++grants;
    if (space != Space::x) { return {}; }
    const bool selected = (owner->registers().omr & 1U) != 0;
    window[0] = {.words = selected ? std::span(second) : std::span(first),
                 .known = tracked ? std::span(known) : std::span<std::uint8_t>{},
                 .writable = true};
    return window;
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
  std::array<Word, 1> first{first_value}, second{second_value};
  std::array<MemoryWindow, 1> window{};
  std::array<std::uint8_t, 1> known{1};
  bool tracked = false;
  Core *owner = nullptr;
  unsigned grants = 0, reads = 0;
};

bool same(const State &left, const State &right) {
  return left.registers == right.registers && left.progress == right.progress &&
         left.failure == right.failure && left.execution == right.execution;
}

void guest_mapping(const NativeImage &image, const Word initial_mode, const bool tracked) {
  Bus bus;
  bus.tracked = tracked;
  Core core(Memory::external(), &bus);
  bus.owner = &core;
  auto initial = core.registers();
  initial.pc = start;
  initial.omr = initial_mode;
  core.set_registers(initial);
  core.install(image, 0);
  const auto grants = bus.grants;
  auto result = core.run();
  if (initial_mode == 0) {
    auto boundary = initial;
    boundary.pc = start + 1;
    boundary.omr = 1;
    require(
        result.retired == 1 && result.yielded && result.fault == Fault{} &&
            core.registers() == boundary &&
            result.progress == Progress{.instructions = 1, .estimated_cycles = 1},
        "A real OMR mapping change yields at its completed instruction before stale RAM access");
    const auto before = core.state();
    core.install(image, 0);
    require(same(before, core.state()),
            "Rebinding new grants preserves the exact real continuation");
    result = core.run();
  } else {
    require(bus.grants == grants, "An unchanged OMR does not refresh direct-memory grants");
  }
  auto expected = initial;
  expected.pc = start + code.size();
  expected.omr = 1;
  expected.x = bus.second[0];
  require(result.fault == Fault{} && !result.yielded && result.state == ExecutionState::stopped &&
              result.retired == (initial_mode == 0 ? 2U : 3U) && core.registers() == expected &&
              result.progress == Progress{.instructions = 3, .estimated_cycles = 12} &&
              bus.reads == 0,
          "The resumed original load uses the new direct grant without an I/O fallback or replay");
}

void host_mapping(const NativeImage &image, const bool interrupt, const bool tracked) {
  Bus bus;
  bus.tracked = tracked;
  Core core(Memory::external(), &bus);
  bus.owner = &core;
  auto initial = core.registers();
  initial.pc = start;
  core.set_registers(initial);
  core.install(image, 0);
  initial.omr = 1;
  core.set_registers(initial);
  if (interrupt) {
    core.request_interrupt(vector, 0);
    require(core.state().failure.kind == FaultKind::unprepared_execution &&
                core.registers() == initial && core.progress() == Progress{} && bus.reads == 0,
            "Host mode changes cannot acknowledge an IRQ against stale installed grants");
  } else {
    const auto result = core.run();
    require(
        result.retired == 0 && result.fault.kind == FaultKind::unprepared_execution &&
            core.registers() == initial && result.progress == Progress{} && bus.reads == 0,
        "Host register initialization cannot silently retain an incompatible installed mapping");
  }
}

void unknown_word(const NativeImage &image) {
  Bus bus;
  bus.tracked = true;
  bus.known[0] = 0;
  Core core(Memory::external(), &bus);
  bus.owner = &core;
  auto initial = core.registers();
  initial.pc = start + 1;
  initial.omr = 1;
  initial.x = first_value;
  core.set_registers(initial);
  core.install(image, 0);
  const auto result = core.run();
  const Fault failure{
      .kind = FaultKind::memory_read, .space = Space::x, .address = 0, .pc = start + 1};
  initial.pc = start + 2;
  require(result.fault == failure && result.retired == 0 && bus.reads == 1 &&
              core.registers() == initial && core.progress() == Progress{},
          "Fixed bounds never turn an unknown RAM word into a known read or retire its fault");
}

struct ControlCase {
  Word operation, initial_mode, target_mode;
};

void control_mapping(const ControlCase item) {
  constexpr std::array entries{start};
  const std::array words{item.operation, code[1], code[2]};
  const std::array spans{CodeSpan{.address = start, .words = words}};
  const std::array variants{CodeVariant{.id = 0, .code = spans, .entries = entries}};
  constexpr std::array memory{RegionLayout{.space = Space::x, .first = 0, .size = 1}};
  const std::array layouts{VariantLayout{.id = 0, .memory = memory}};
  const auto image = Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
  Bus bus;
  bus.program[start] = item.operation;
  Core core(Memory::external(), &bus);
  bus.owner = &core;
  auto expected = core.registers();
  expected.pc = start;
  expected.omr = item.initial_mode;
  core.set_registers(expected);
  core.install(image, 0);
  const auto grants = bus.grants;
  auto result = core.run();
  expected.omr = item.target_mode;
  expected.pc = start + 1;
  const bool changed = item.initial_mode != item.target_mode;
  if (changed) {
    require(result.retired == 1 && result.yielded && result.fault == Fault{} &&
                core.registers() == expected &&
                core.progress() == Progress{.instructions = 1, .estimated_cycles = 3},
            "ANDI/ORI OMR changes yield once before any read through stale installed RAM grants");
    core.install(image, 0);
    result = core.run();
  } else {
    require(!result.yielded && bus.grants == grants,
            "An unchanged OMR byte neither refreshes grants nor adds a native return");
  }
  expected.pc = start + code.size();
  expected.x = (item.target_mode & 1U) == 0 ? first_value : second_value;
  require(result.retired == (changed ? 2U : 3U) && result.state == ExecutionState::stopped &&
              result.fault == Fault{} && !result.yielded && bus.reads == 0 &&
              core.registers() == expected &&
              core.progress() == Progress{.instructions = 3, .estimated_cycles = 14},
          "The original next load observes the control instruction's newly selected RAM");
}
}

int main() {
  try {
    const std::array spans{CodeSpan{.address = start, .words = code},
                           CodeSpan{.address = vector, .words = inserted}};
    constexpr std::array entries{start};
    const CodeVariant version{.id = 0, .code = spans, .entries = entries};
    constexpr std::array interrupts{InterruptRequest{.vector = vector, .level = 0}};
    for (const bool tracked : {false, true}) {
      const std::array memory{
          RegionLayout{.space = Space::x, .first = 0, .size = 1, .tracked = tracked}};
      const std::array layouts{VariantLayout{.id = 0, .memory = memory}};
      for (const bool fixed : {false, true}) {
        const auto image = Core::compile({.variants = {&version, 1}},
                                         {.memory = fixed ? std::span<const VariantLayout>(layouts)
                                                          : std::span<const VariantLayout>{},
                                          .interrupts = interrupts,
                                          .transitions = {},
                                          .callbacks = {},
                                          .bindings = {}});
        guest_mapping(image, 0, tracked);
        guest_mapping(image, 1, tracked);
        host_mapping(image, false, tracked);
        host_mapping(image, true, tracked);
        if (tracked) { unknown_word(image); }
      }
    }
    constexpr unsigned byte_shift = 8;
    constexpr Word low_control = 2;
    constexpr Word high_control = 3;
    constexpr Word clear_low_bit = 0xfe;
    constexpr Word high_mode = 1U << byte_shift;
    constexpr Word set_mode = static_cast<Word>(isa::Opcode::ori_control) | (1U << byte_shift);
    constexpr Word clear_mode =
        static_cast<Word>(isa::Opcode::andi_control) | (clear_low_bit << byte_shift);
    constexpr std::array cases{
        ControlCase{.operation = set_mode | low_control, .initial_mode = 0, .target_mode = 1},
        ControlCase{.operation = set_mode | low_control, .initial_mode = 1, .target_mode = 1},
        ControlCase{.operation = clear_mode | low_control, .initial_mode = 1, .target_mode = 0},
        ControlCase{.operation = clear_mode | low_control, .initial_mode = 0, .target_mode = 0},
        ControlCase{
            .operation = set_mode | high_control, .initial_mode = 0, .target_mode = high_mode},
        ControlCase{
            .operation = clear_mode | high_control, .initial_mode = high_mode, .target_mode = 0}};
    for (const auto item : cases) { control_mapping(item); }
    std::cout << test::checks << " public image-mapping checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
