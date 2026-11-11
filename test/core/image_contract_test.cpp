// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word opcode = static_cast<Word>(operation);
constexpr Word start = 16, writer = 32, program_words = 64, data_words = 8;
constexpr std::uint64_t accumulator = 0x2000000;
constexpr std::array code0{opcode<isa::Opcode::nop>, opcode<isa::Opcode::stop>};
constexpr std::array code1{opcode<isa::Opcode::asr_a>, opcode<isa::Opcode::stop>};
constexpr Word absolute = 0x30, x0 = 4;
constexpr std::array writing{opcode<isa::Opcode::movem_effective> | (absolute << 8) | x0,
                             start,
                             opcode<isa::Opcode::jmp_absolute_short> | start};
constexpr std::array<RegionLayout, 3> memory{
    RegionLayout{.space = Space::program, .first = 0, .size = program_words},
    RegionLayout{.space = Space::x, .first = 0, .size = data_words},
    RegionLayout{.space = Space::y, .first = 0, .size = data_words}};

NativeImage compile(const bool fixed_memory = true) {
  const std::array first{CodeSpan{.address = start, .words = code0},
                         CodeSpan{.address = writer, .words = writing}};
  const std::array second{CodeSpan{.address = start, .words = code1},
                          CodeSpan{.address = writer, .words = writing}};
  constexpr std::array entries{start};
  const std::array variants{
      CodeVariant{.id = 0, .code = first, .entries = entries, .failed_fetches = {}},
      CodeVariant{.id = 1, .code = second, .entries = entries, .failed_fetches = {}}};
  const std::array layouts{VariantLayout{.id = 0, .memory = memory},
                           VariantLayout{.id = 1, .memory = memory}};
  return Core::compile({.variants = variants},
                       {.memory = fixed_memory ? std::span<const VariantLayout>(layouts)
                                               : std::span<const VariantLayout>{},
                        .interrupts = {},
                        .transitions = {},
                        .callbacks = {},
                        .bindings = {}});
}

const NativeImage &image() {
  static const NativeImage value = compile();
  return value;
}

struct Storage {
  Storage() {
    program[start] = code0[0];
    program[start + 1] = code0[1];
    for (std::size_t i = 0; i < writing.size(); ++i) { program[writer + i] = writing[i]; }
  }

  [[nodiscard]] Memory view() { return Memory::separate({.program = program, .x = x, .y = y}); }

  std::array<Word, program_words> program{};
  std::array<Word, data_words> x{}, y{};
};

Registers registers() {
  Registers result;
  result.pc = start;
  result.a = accumulator;
  result.m.fill(word_mask);
  return result;
}

bool same(const State &left, const State &right) {
  return left.registers == right.registers && left.failure == right.failure &&
         left.execution == right.execution && left.progress == right.progress;
}

void transactional_install() {
  Storage storage;
  Core core(storage.view());
  core.set_registers(registers());
  const auto before = core.state();
  core.install(image(), 0);
  require(same(core.state(), before),
          "Installing prepared code preserves all observable CPU state");
  for (const bool empty : {false, true}) {
    bool rejected = false;
    try {
      core.install(empty ? NativeImage{} : image(), empty ? 0 : 1);
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected && same(core.state(), before),
            "Empty images and source-byte mismatches cannot replace the live binding");
  }
  const auto first = core.step();
  require(first.retired == 1 && first.fault == Fault{} && core.registers().pc == start + 1,
          "The previous live image still executes after failed installations");
  const auto established = core.prepare(std::array<Word, 1>{start});
  require(established.failure == Fault{} && established.unresolved_entries == 0,
          "Already installed entries need no ordinary decoder or emitter");
}

void code_transition() {
  Storage storage;
  Core core(storage.view());
  core.set_registers(registers());
  core.install(image(), 0);
  const std::array changed{code1[0]};
  core.load(Space::program, start, changed);
  const auto before = core.state();
  core.install(image(), 1);
  require(same(core.state(), before), "A code-version change preserves registers and progress");
  const auto shifted = core.step();
  require(shifted.retired == 1 && shifted.fault == Fault{} &&
              core.registers().a == accumulator / 2 && core.registers().pc == start + 1,
          "Selection executes the prepared new version without changing its original RAM words");
  require(storage.program[start] == code1[0], "Code words remain observable in consumer RAM");
}

void strict_binding() {
  Storage storage;
  Core core(storage.view());
  core.set_registers(registers());
  core.install(image(), 0);
  core.load(Space::program, start, std::array{code1[0]});
  const auto before = core.registers();
  require(core.run().fault.kind == FaultKind::unprepared_execution && core.registers() == before &&
              core.progress() == Progress{},
          "Changed executable words cannot silently continue through stale code or a fallback");
  core.reset();
  core.set_registers(registers());
  core.install(image(), 1);
  const auto mismatch = core.run_instruction({.address = start, .opcode = code0[0]});
  require(mismatch.fault.kind == FaultKind::unprepared_execution && core.registers() == before &&
              core.progress() == Progress{},
          "Explicit instruction execution also refuses unprepared captured words");
}

void owner_retention() {
  Storage first;
  Storage second;
  first.program[start] = code1[0];
  second.program[start] = code1[0];
  Core a(first.view());
  Core b(second.view());
  auto left = registers();
  auto right = left;
  right.a *= 2;
  a.set_registers(left);
  b.set_registers(right);
  {
    const auto local = compile();
    a.install(local, 1);
    b.install(local, 1);
  }
  require(a.step().fault == Fault{} && b.step().fault == Fault{} && a.registers().a == left.a / 2 &&
              b.registers().a == right.a / 2,
          "Independent owners retain shared code after the public image handle is released");
}

void guest_code_write() {
  Storage storage;
  Core core(storage.view());
  auto before = registers();
  before.pc = writer;
  before.x = code1[0];
  core.set_registers(before);
  core.install(image(), 0);
  const auto changed = core.run();
  require(changed.retired == 1 && changed.yielded && changed.fault == Fault{} &&
              storage.program[start] == code1[0] && core.registers().pc == writer + 2,
          "A real MOVEM write retires once and yields before stale executable continuation");
  const auto stopped = core.state();
  core.install(image(), 1);
  require(same(stopped, core.state()),
          "A guest code-write transition keeps the real CPU continuation");
  const auto resumed = core.run(2);
  require(resumed.retired == 2 && resumed.fault == Fault{} && core.registers().pc == start + 1 &&
              core.registers().a == accumulator / 2,
          "The original writer's jump continues through the explicitly selected new code version");
}

struct External final : Io {
  std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (probe) {
      probe = false;
      try {
        owner->install(*native, 0);
      } catch (const std::logic_error &) { refused = true; }
    }
    if (address < storage.program.size()) { return storage.program[address]; }
    return std::nullopt;
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    if (space != Space::program || address >= storage.program.size()) { return false; }
    storage.program[address] = value;
    ++writes;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  Storage storage;
  Core *owner = nullptr;
  const NativeImage *native = nullptr;
  mutable bool probe = true, refused = false;
  unsigned writes = 0;
};

void external_install() {
  const auto native = compile(false);
  External io;
  Core core(Memory::external(), &io);
  io.owner = &core;
  io.native = &native;
  auto initial = registers();
  initial.pc = writer;
  initial.x = code1[0];
  core.set_registers(initial);
  core.install(native, 0);
  require(io.refused, "Side-effect-free source inspection cannot reenter image installation");
  const auto changed = core.run();
  require(changed.retired == 1 && changed.yielded && changed.fault == Fault{} && io.writes == 1 &&
              io.storage.program[start] == code1[0] && io.program_revision() == 0,
          "An accepted external P write yields before stale code even without a revision bump");
  core.install(native, 1);
  const auto resumed = core.run(2);
  require(resumed.retired == 2 && resumed.fault == Fault{} && core.registers().a == accumulator / 2,
          "External code selection preserves the real native continuation and program write");
}

void unchanged_external_program_write() {
  const auto native = compile(false);
  External io;
  Core core(Memory::external(), &io);
  io.owner = &core;
  io.native = &native;
  auto initial = registers();
  initial.pc = writer;
  initial.x = code0[0];
  core.set_registers(initial);
  core.install(native, 0);
  auto expected = initial;
  expected.pc += 2;
  const auto stored = core.run(1);
  require(stored.retired == 1 && !stored.yielded && stored.fault == Fault{} &&
              core.registers() == expected && core.progress().instructions == 1 && io.writes == 1 &&
              io.storage.program[start] == code0[0],
          "An unchanged external code store performs its callback once without revoking the image");
  const auto resumed = core.run(2);
  expected.pc = start + 1;
  require(resumed.retired == 2 && !resumed.yielded && resumed.fault == Fault{} &&
              core.registers() == expected && core.progress().instructions == 3 && io.writes == 1,
          "The still-valid native image continues through the original jump and resident NOP");
}

void undeclared_interrupt() {
  Storage storage;
  Core core(storage.view());
  core.set_registers(registers());
  core.install(image(), 0);
  const auto before = core.state();
  constexpr Word vector = 0x20;
  core.interrupt(vector, 0);
  const auto after = core.state();
  require(after.failure.kind == FaultKind::unprepared_execution &&
              after.failure.address == vector && after.registers == before.registers &&
              after.progress == before.progress && after.execution == before.execution,
          "An undeclared vector cannot invoke first-use native emission or architectural entry");
}

void bad_layout() {
  Storage storage;
  std::array<Word, data_words + 1> other{};
  Core core(Memory::separate({.program = storage.program, .x = other, .y = storage.y}));
  const auto before = core.state();
  bool rejected = false;
  try {
    core.install(image(), 0);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && same(core.state(), before),
          "Pointer-free layout contracts are checked against actual owner storage");
}
}

int main() {
  try {
    transactional_install();
    code_transition();
    strict_binding();
    owner_retention();
    bad_layout();
    guest_code_write();
    external_install();
    unchanged_external_program_write();
    undeclared_interrupt();
    std::cout << test::checks << " public image lifecycle checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
