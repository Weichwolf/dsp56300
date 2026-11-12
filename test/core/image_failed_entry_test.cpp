// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word start = 16, absent = 0x180;
constexpr std::array jump{static_cast<Word>(isa::Opcode::jmp_absolute_short) | absent};
constexpr std::array before{start}, after{absent};
constexpr CodeTransition transfer{.from = 0, .to = 1, .entry = absent};

NativeImage compile(const bool known_failure, const bool fixed_storage = false) {
  const std::array code{CodeSpan{.address = start, .words = jump}};
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = before, .failed_fetches = {}},
      CodeVariant{.id = 1,
                  .code = code,
                  .entries = after,
                  .failed_fetches =
                      known_failure ? std::span<const Word>(after) : std::span<const Word>{}}};
  const std::array layouts{VariantLayout{.id = 1, .memory = {}}};
  return Core::compile({.variants = variants},
                       {.memory = fixed_storage ? std::span<const VariantLayout>(layouts)
                                                : std::span<const VariantLayout>{},
                        .interrupts = {},
                        .transitions = std::span(&transfer, 1),
                        .callbacks = {},
                        .bindings = {}});
}

struct Bus final : Io {
  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (address == start) { return jump[0]; }
    if (address == absent && readable) { return static_cast<Word>(isa::Opcode::nop); }
    return std::nullopt;
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (space != Space::program) { return false; }
    if (address == absent) { ++reads; }
    const auto word = inspect_program(address);
    if (!word) { return false; }
    value = *word;
    return true;
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

  bool readable = false;
  unsigned reads = 0;
};

bool same(const State &left, const State &right) {
  return left.registers == right.registers && left.failure == right.failure &&
         left.progress == right.progress && left.execution == right.execution;
}

void failed_destination(const NativeImage &image) {
  Bus bus;
  Core core(Memory::external(), &bus);
  auto initial = test::register_model::seed(0);
  initial.pc = start;
  initial.sr = 0;
  core.set_registers(initial);
  core.install(image, 0);
  const auto jumped = core.run();
  auto expected = initial;
  expected.pc = absent;
  constexpr Progress progress{.instructions = 1, .estimated_cycles = 3};
  require(jumped.retired == 1 && jumped.yielded && jumped.fault == Fault{} &&
              core.registers() == expected && core.progress() == progress && bus.reads == 0,
          "The actual jump finishes before the explicitly prepared failed-fetch destination");
  const auto boundary = core.state();
  bus.readable = true;
  bool rejected = false;
  try {
    core.install(image, 1);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && same(boundary, core.state()) && core.run().retired == 0 && bus.reads == 0,
          "Even a readable NOP refuses the failed-fetch binding without changing the handoff");
  bus.readable = false;
  core.install(image, 1);
  require(same(boundary, core.state()) && bus.reads == 0,
          "Binding a genuine unknown source executes no guest instruction or program read");
  const auto prepared = core.prepare(after);
  require(prepared.failure == Fault{} && prepared.unresolved_entries == 0 &&
              prepared.prepared_entries == 1 && same(boundary, core.state()) && bus.reads == 0,
          "Known failure endpoints are prepared without becoming translated instructions");
  const auto empty = core.run(0);
  require(empty.retired == 0 && empty.fault == Fault{} && same(boundary, core.state()) &&
              bus.reads == 0,
          "A zero grant cannot attempt the prepared program fetch");
  const auto fault = core.run();
  require(fault.retired == 0 && !fault.yielded && core.registers() == expected &&
              core.progress() == progress && fault.state == ExecutionState::running &&
              fault.fault == Fault{.kind = FaultKind::memory_read,
                                   .space = Space::program,
                                   .address = absent,
                                   .pc = absent} &&
              bus.reads == 1,
          "Execution performs the real refused fetch, preserving the complete native prefix");
}

void initial_failure(const NativeImage &image) {
  Bus bus;
  Core core(Memory::external(), &bus);
  auto initial = test::register_model::seed(0);
  initial.pc = absent;
  initial.sr = 0;
  core.set_registers(initial);
  core.install(image, 1);
  const auto result = core.run();
  require(result.retired == 0 && result.fault.kind == FaultKind::memory_read &&
              result.fault.address == absent && result.fault.pc == absent &&
              core.registers() == initial && core.progress() == Progress{} && bus.reads == 1,
          "A declared initial fault entry performs its actual read without emitting an opcode");
}
}

int main() {
  try {
    bool rejected = false;
    try {
      static_cast<void>(compile(false));
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "An absent entry with no failed-fetch contract still rejects preparation");
    for (const auto fixed_storage : {false, true}) {
      const auto image = compile(true, fixed_storage);
      require(image.report().failure == Fault{} && image.report().prepared_entries == 2,
              "Only the two real jump streams count as translated instructions");
      failed_destination(image);
      initial_failure(image);
    }
    std::cout << test::checks << " declared native failed-entry checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
