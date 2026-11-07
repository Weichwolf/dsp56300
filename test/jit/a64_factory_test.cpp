// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_jit_state.hpp"
#include "a64_loop_state.hpp"
#include "architecture.hpp"
#include "codegen_observer.hpp"
#include "control_byte_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "loop_control_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
using State = loop_model::State;
constexpr Word jump = 0x0c0000, add = 0x200040, clear = 0x200013, load_add = 0x44d840;
constexpr Word long_body = 513, saved_count = 0x345, location = 384;

State seed(const Word mode = 0) {
  State state;
  state.reg = register_model::seed(mode);
  state.reg.sr = mode;
  return state;
}

detail::BlockInstruction operation(const State &state, const Word opcode = 0) {
  const auto decoded = detail::decode(opcode);
  require(decoded.words == 1 && decoded.cycles == 1,
          "Authored captured/REP arithmetic has one word and one independent base cycle");
  return {.instruction = decoded,
          .pc = state.reg.pc,
          .mode = state.reg.sr & detail::numeric_mode_mask,
          .loop_address = (state.reg.sr & loop_model::lf) != 0 ? state.reg.la : word_mask + 1};
}

void ordinary(State &state, const Word opcode = 0) {
  if (opcode != 0) { a64_alu_model::apply(state.reg, opcode & status_byte); }
  ++state.reg.pc;
  state.inhibited = false;
  state.completed = false;
  loop_model::finish(state, {.pc = state.reg.pc - 1});
  if (state.fault.kind == FaultKind::none) { loop_model::retire(state, 1); }
}

void repeated(State &state, const Word opcode = 0) {
  if (opcode != 0) { a64_alu_model::apply(state.reg, opcode & status_byte); }
  state.inhibited = false;
  state.reg.pc = state.repeat.pc + 1;
  state.repeat.opcode = opcode;
  state.repeat.loaded = true;
  --state.repeat.remaining;
  if (state.repeat.remaining == 0) {
    state.reg.lc = state.repeat.lc;
    state.repeat.loaded = false;
    state.completed = true;
    loop_model::finish(state, {.pc = state.repeat.pc});
  } else {
    state.reg.lc = state.repeat.remaining;
    state.reg.pc = state.repeat.pc;
  }
  if (state.fault.kind == FaultKind::none) { loop_model::retire(state, 1); }
}

struct Image {
  const std::uint8_t *entry;
  std::vector<std::uint8_t> bytes;
};

Image latest() {
  const auto publications = codegen::published_images();
  require(!publications.empty(), "A factory translation publishes an actual native image");
  const auto last = publications.back();
  return {.entry = last.entry, .bytes = std::vector(last.entry, last.entry + last.bytes)};
}

void preserved(const Image &image) {
  require(std::ranges::equal(image.bytes, std::span(image.entry, image.bytes.size())),
          "Every original factory image remains mapped, readable and byte-for-byte immutable");
}

void nop_ring(State &state, const Word budget) {
  for (Word retired = 0; retired < budget; ++retired) {
    if (state.reg.pc == start + long_body) {
      state.reg.pc = start;
      state.inhibited = state.completed = false;
      loop_model::retire(state, 3);
    } else {
      ordinary(state);
    }
  }
}

void complete_blocks() {
  const auto initial = seed(status_byte);
  a64::Machine machine(initial);
  auto &program = machine.program();
  std::ranges::fill(std::span(program).subspan(start, long_body), 0);
  program[start + long_body] = jump | start;
  const auto before = codegen::publications();
  require(machine.jit().run(0) == 0 && codegen::publications() == before,
          "A zero factory grant has no CPU effects or code publication");
  machine.expect(initial);
  require(machine.jit().run(1) == 1, "The cold factory enters ARM64 code");
  auto expected = initial;
  ordinary(expected);
  machine.expect(expected);
  const auto image = latest();
  const auto warm = codegen::publications();
  require(warm.count == before.count + 1, "One complete logical block has one publication");
  for (const Word budget : {0U, 2U, long_body + 1, (2 * long_body) + 3}) {
    expected = initial;
    nop_ring(expected, budget);
    machine.initialize(initial);
    require(machine.jit().run(budget) == budget,
            "A complete retained native block respects grants and chains its backedge");
    machine.expect(expected);
    require(codegen::publications() == warm, "Warm changing grants never translate fragments");
    preserved(image);
  }
  a64::Machine whole(initial);
  whole.program() = program;
  expected = initial;
  nop_ring(expected, long_body + 1);
  require(whole.jit().run(long_body + 1) == long_body + 1,
          "A cold large grant executes its full block");
  whole.expect(expected);
  require(latest().bytes == image.bytes,
          "Cold grants of one and a complete block publish identical native bytes");
}

void internal_backedges() {
  constexpr Word iterations = 9;
  constexpr Word outer_address = 99;
  auto initial = seed(loop_model::lf);
  initial.reg.la = start;
  initial.reg.lc = iterations;
  initial.reg.sp = 0;
  initial.reg.sc = 2;
  initial.reg.stack[0] = std::uint64_t{start} << word_bits;
  initial.reg.stack[stack_entries - 1] = (std::uint64_t{outer_address} << word_bits) | saved_count;
  a64::Machine machine(initial);
  machine.program()[start] = add;
  require(machine.jit().run(1) == 1, "Compile one complete native self-loop body");
  const auto image = latest();
  const auto warm = codegen::publications();
  for (const Word depth : {0U, 2U}) {
    initial.reg.sc = depth;
    for (const Word budget : {0U, 1U, 3U, iterations, iterations + 2}) {
      detail::Cpu cpu{};
      cpu.reg = initial.reg;
      cpu.reg.a <<= detail::padding_bits;
      cpu.reg.b <<= detail::padding_bits;
      cpu.instructions = initial.instructions;
      cpu.cycles = initial.cycles;
      cpu.repeat_completed = initial.completed;
      cpu.interrupt_inhibited = initial.inhibited;
      detail::NativeContext context{};
      context.budget = budget;
      auto expected = initial;
      Word retired = 0;
      while (retired < budget && expected.boundary == detail::NativeBoundary::none) {
        ordinary(expected, add);
        if (expected.fault.kind != FaultKind::none) { break; }
        ++retired;
      }
      require(detail::enter_block(image.entry, cpu, context) == retired,
              "One direct ABI entry follows its own native backedges without a dispatcher");
      auto physical = expected.reg;
      physical.a <<= detail::padding_bits;
      physical.b <<= detail::padding_bits;
      require(cpu.reg == physical && cpu.instructions == expected.instructions &&
                  cpu.cycles == expected.cycles && cpu.repeat_completed == expected.completed &&
                  cpu.interrupt_inhibited == expected.inhibited &&
                  context.failure == expected.fault && context.boundary == expected.boundary,
              "Internal loops retain exact grants, wrapping progress, final cleanup and faults");
      require(codegen::publications() == warm,
              "Direct native backedges never allocate or translate another entry");
      preserved(image);
    }
  }
}

void semantic_boundaries() {
  using byte_model::Field;
  const std::array opcodes{
      Word{1},
      Word{2},
      Word{3},
      Word{0x0be081},
      Word{0x0ae081},
      Word{0x84},
      Word{0x077084},
      byte_model::opcode({.field = Field::mr, .immediate = 0, .inclusive_or = true}),
      byte_model::opcode({.field = Field::com, .immediate = 1, .inclusive_or = true}),
      byte_model::opcode({.field = Field::eom, .immediate = 1, .inclusive_or = true})};
  const auto initial = seed(status_byte);
  a64::Machine machine(initial);
  machine.program().fill(0);
  for (const auto opcode : opcodes) {
    machine.program()[start] = opcode;
    const auto block = detail::parse_block(machine.program(), initial.reg, {.memory = {}});
    require(block.size() == 1 && block.front().instruction.opcode == opcode,
            "Actual cache, program-write and control owner boundaries terminate complete blocks");
  }
  for (const Word opcode : {0x44e000U, 0xf09800U, 0x044480U}) {
    machine.program()[start] = opcode;
    const auto block = detail::parse_block(machine.program(), initial.reg, {.memory = {}}, true);
    require(block.size() == 1 && block.front().instruction.opcode == opcode,
            "A possible memory callback terminates its complete owner block");
    require(detail::parse_block(machine.program(), initial.reg, {.memory = {}}).size() > 1,
            "Direct memory without callbacks remains in a complete multi-instruction block");
  }
  machine.program()[start] = 0;
  require(detail::parse_block(machine.program(), initial.reg, {.memory = {}}, true).size() ==
              machine.program().size() - start,
          "Callback ownership never imposes an instruction-count limit on a NOP block");
}

void versions() {
  const auto initial = seed(status_byte);
  a64::Machine machine(initial);
  machine.program()[start + 1] = jump | start;
  const auto before = codegen::publications();
  std::vector<Image> images;
  for (const Word opcode : {add, clear, add, clear}) {
    machine.initialize(initial);
    machine.program()[start] = opcode;
    machine.jit().invalidate();
    auto expected = initial;
    ordinary(expected, opcode);
    require(machine.jit().run(1) == 1, "Each actual program version executes its ARM64 image");
    machine.expect(expected);
    images.push_back(latest());
    for (const auto &image : images) { preserved(image); }
  }
  require(codegen::publications().count == before.count + 2,
          "Returning program versions reuse retained complete translations after invalidation");
  for (const Word opcode : {clear, add}) {
    machine.initialize(initial);
    machine.program()[start] = opcode;
    machine.jit().reset();
    auto expected = initial;
    ordinary(expected, opcode);
    require(machine.jit().run(1) == 1, "Jit reset preserves executable versions for its lifetime");
    machine.expect(expected);
    require(codegen::publications().count == before.count + 2,
            "Resetting lookup ownership does not recompile retained matching source");
  }
  for (const auto &image : images) { preserved(image); }
}

void captured_versions() {
  const auto initial = seed(status_byte);
  a64::Machine machine(initial);
  const auto before = codegen::publications();
  for (const Word opcode : {add, clear, add, clear}) {
    machine.initialize(initial);
    auto expected = initial;
    ordinary(expected, opcode);
    require(machine.jit().run_captured(operation(initial, opcode)) == 1,
            "Captured words use the native ARM64 factory independently of backing P");
    machine.expect(expected);
  }
  require(codegen::publications().count == before.count + 2,
          "Captured opcode changes retain and reuse every version");
}

void repeat_versions() {
  auto initial = seed(status_byte);
  initial.reg.lc = 3;
  initial.repeat = {.remaining = 3, .pc = start, .lc = saved_count};
  a64::Machine machine(initial);
  const auto input = operation(initial, add);
  const auto before = codegen::publications();
  require(machine.jit().run_repeat(input, 0) == 0 && codegen::publications() == before,
          "Zero REP grants do not translate or touch state");
  auto expected = initial;
  repeated(expected, add);
  require(machine.jit().run_repeat(input, 1) == 1, "A genuine ARM64 repeat entry executes");
  machine.expect(expected);
  const auto image = latest();
  machine.program()[start] = clear;
  machine.jit().invalidate();
  repeated(expected, add);
  repeated(expected, add);
  require(machine.jit().run_repeat(input, 8) == 2,
          "A loaded REP keeps its real opcode and exact remaining count after backing-P mutation");
  machine.expect(expected);
  require(codegen::publications().count == before.count + 1,
          "Invalidation reuses the still-held REP body without recompilation");
  for (const Word opcode : {clear, add, clear}) {
    machine.initialize(initial);
    expected = initial;
    for (unsigned index = 0; index < 3; ++index) { repeated(expected, opcode); }
    require(machine.jit().run_repeat(operation(initial, opcode), 8) == 3,
            "A new REP at the same PC selects its own genuine opcode");
    machine.expect(expected);
    preserved(image);
  }
  require(codegen::publications().count == before.count + 2,
          "Each repeated opcode has exactly one retained translation");
}

void entry_kinds() {
  const auto initial = seed(status_byte);
  auto repeat = initial;
  repeat.reg.lc = 2;
  repeat.repeat = {.remaining = 2, .pc = start, .lc = saved_count};
  a64::Machine machine(initial);
  const auto before = codegen::publications();
  for (unsigned pass = 0; pass < 2; ++pass) {
    auto expected = initial;
    machine.initialize(initial);
    ordinary(expected, add);
    require(machine.jit().run_captured(operation(initial, add)) == 1,
            "An ordinary captured entry has no REP side effects");
    machine.expect(expected);
    machine.initialize(repeat);
    expected = repeat;
    repeated(expected, add);
    repeated(expected, add);
    require(machine.jit().run_repeat(operation(repeat, add), 8) == 2,
            "Ordinary and repeated translations of identical instruction metadata stay distinct");
    machine.expect(expected);
    require(codegen::publications().count == before.count + 2,
            "Both entry kinds retain their own immutable translation across reuse");
  }
}

void captured_repeat_endpoints() {
  for (const Word mode : {0U, loop_model::cm}) {
    for (const Word stack : {0U, 1U, 2U}) {
      auto initial = seed(mode | loop_model::lf);
      initial.reg.la = start;
      initial.reg.lc = 1;
      initial.reg.sc = stack;
      initial.repeat = {.remaining = 1, .pc = start, .lc = 1, .loaded = true};
      a64::Machine machine(initial);
      auto expected = initial;
      repeated(expected);
      const auto retired = expected.fault.kind == FaultKind::none ? 1U : 0U;
      require(machine.jit().run_captured(operation(initial)) == retired,
              "Captured final REP executes actual native loop cleanup and precise pop/mode faults");
      machine.expect(expected);
    }
  }
}

void loop_contexts() {
  const auto initial = seed(status_byte);
  a64::Machine machine(initial);
  const loop_model::Input loop{.source = 2, .extension = start + 3};
  machine.program()[start] = loop_model::opcode(loop);
  machine.program()[start + 1] = loop.extension;
  machine.program()[start + 2] = machine.program()[start + 3] = 0;
  auto expected = initial;
  loop_model::setup(expected, loop);
  require(machine.jit().run(5) == 1, "A cold DO leaves for its genuinely missing loop context");
  machine.expect(expected);
  for (unsigned index = 0; index < 4; ++index) { ordinary(expected); }
  require(machine.jit().run(4) == 4,
          "The complete native body chains both hardware-loop iterations");
  machine.expect(expected);
  const auto warm = codegen::publications();
  machine.initialize(initial);
  require(machine.jit().run(5) == 5,
          "Production factory tables chain native DO entry, iterations and restored context");
  machine.expect(expected);
  require(codegen::publications() == warm, "Warm native context transitions publish no code");
}

void memory_ring() {
  auto initial = seed(status_byte);
  initial.reg.r[0] = location;
  a64::Machine machine(initial);
  machine.program()[start] = load_add;
  machine.program()[start + 1] = jump | start;
  for (Word index = 0; index < 8; ++index) { machine.x()[location + index] = index + 1; }
  const auto x = machine.x();
  const auto y = machine.y();
  auto expected = initial;
  const auto step = [&] {
    const auto old = expected.reg;
    const auto address = expected.reg.r[0]++;
    register_model::write(expected.reg, 4, machine.x()[address]);
    a64_alu_model::apply(expected.reg, old, 0x40);
    ++expected.reg.pc;
    expected.inhibited = expected.completed = false;
    loop_model::retire(expected, 1);
  };
  step();
  require(machine.jit().run(1) == 1,
          "Factory code directly loads RAM and preserves parallel inputs");
  machine.expect(expected);
  expected.reg.pc = start;
  loop_model::retire(expected, 3);
  require(machine.jit().run(1) == 1, "The branch suffix gets its complete native entry");
  const auto warm = codegen::publications();
  for (unsigned index = 0; index < 4; ++index) {
    step();
    expected.reg.pc = start;
    loop_model::retire(expected, 3);
  }
  require(machine.jit().run(8) == 8,
          "Retained production entries chain direct RAM and control flow");
  machine.expect(expected);
  require(codegen::publications() == warm && machine.x() == x && machine.y() == y,
          "Warm direct-RAM execution has no helper, recompilation or collateral memory writes");
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    codegen::track_images();
    complete_blocks();
    internal_backedges();
    semantic_boundaries();
    versions();
    captured_versions();
    repeat_versions();
    entry_kinds();
    captured_repeat_endpoints();
    loop_contexts();
    memory_ring();
    std::cout << checks << " independent native A64 Jit factory checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
