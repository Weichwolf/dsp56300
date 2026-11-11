// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
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
    window[0] = {
        .words = selected ? std::span(second) : std::span(first), .known = {}, .writable = true};
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
  Core *owner = nullptr;
  unsigned grants = 0, reads = 0;
};

bool same(const State &left, const State &right) {
  return left.registers == right.registers && left.progress == right.progress &&
         left.failure == right.failure && left.execution == right.execution;
}

void guest_mapping(const NativeImage &image, const Word initial_mode) {
  Bus bus;
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

void host_mapping(const NativeImage &image, const bool interrupt) {
  Bus bus;
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
}

int main() {
  try {
    const std::array spans{CodeSpan{.address = start, .words = code},
                           CodeSpan{.address = vector, .words = inserted}};
    constexpr std::array entries{start};
    const CodeVariant version{.id = 0, .code = spans, .entries = entries};
    constexpr std::array interrupts{InterruptRequest{.vector = vector, .level = 0}};
    const auto image = Core::compile({.variants = {&version, 1}},
                                     {.memory = {}, .interrupts = interrupts, .transitions = {}});
    guest_mapping(image, 0);
    guest_mapping(image, 1);
    host_mapping(image, false);
    host_mapping(image, true);
    std::cout << test::checks << " public image-mapping checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
