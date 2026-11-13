// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/region_memory.hpp"
#include "jit/llvm/region_plan.hpp"
#include "jit/llvm/runtime.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <llvm-c/Core.h>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace copy_cases {
using namespace dsp56300;
using namespace dsp56300::detail;
using Data = std::array<Word, 8>;
constexpr Word entry = 2, loop_flag_bit = 1U << 15;
using Admission = std::uint64_t (*)(const Cpu *, const NativeContext *);

void require(const bool ok, const char *message) {
  if (!ok) { throw std::runtime_error(message); }
}

template <typename T> auto representation(const T &value) {
  std::array<std::byte, sizeof(T)> result{};
  std::ranges::copy(std::as_bytes(std::span(&value, 1)), result.begin());
  return result;
}

struct Input {
  Word source, destination, count;
  std::uint64_t credit;
  Word modifier;
};

struct Frame {
  Admission admission;
  const void *execute;
  const Registers &seed;
  const JitRuntime &runtime;
  Data &data;
};

bool exercise(const Frame &frame, const Input input) {
  const auto [source, destination, count, credit, modifier] = input;
  auto &data = frame.data;
  const auto &seed = frame.seed;
  const auto &runtime = frame.runtime;
  const auto admission = frame.admission;
  const auto *execute = frame.execute;
  data = {1, 2, 3, 4, 5, 6, 7, 8};
  Cpu cpu{.reg = seed};
  cpu.reg.r[0] = source;
  cpu.reg.r[1] = destination;
  cpu.reg.lc = count;
  cpu.reg.m[1] = modifier;
  NativeContext owner{.runtime = runtime, .budget = 16, .cycle_deadline = credit};
  const auto before_cpu = representation(cpu);
  const auto before_owner = representation(owner);
  const auto before_data = data;
  const auto bound = admission(&cpu, &owner);
  require(representation(cpu) == before_cpu && representation(owner) == before_owner &&
              data == before_data,
          "Admission changes no guest, owner or RAM byte");
  const std::array<std::uint64_t, 3> limits{count, credit / 2, 8};
  const auto turns = std::ranges::min(limits);
  const bool fits = turns != 0 && modifier == word_mask && source < 8 && destination < 8 &&
                    turns <= 8 - source && turns <= 8 - destination;
  const auto expected = fits ? turns * 2 : 0;
  if (bound != expected) {
    std::cerr << "source=" << source << " destination=" << destination << " count=" << count
              << " credit=" << credit << " modifier=" << modifier << " bound=" << bound
              << " expected=" << expected << '\n';
  }
  require(bound == expected,
          "Native guards independently match range, count, modifier and credit bounds");
  if (bound != 0) {
    auto expected_data = before_data;
    auto expected_registers = cpu.reg;
    for (std::uint64_t turn = 0; turn < turns; ++turn) {
      expected_registers.x = expected_data[static_cast<std::size_t>(source + turn)];
      expected_data[static_cast<std::size_t>(destination + turn)] =
          static_cast<Word>(expected_registers.x);
    }
    expected_registers.r[0] += static_cast<Word>(turns);
    expected_registers.r[1] += static_cast<Word>(turns);
    if (turns == count) {
      expected_registers.pc = entry + 2;
      expected_registers.sr = expected_registers.sp = expected_registers.sc =
          expected_registers.la = expected_registers.lc = 0;
    } else {
      expected_registers.lc -= static_cast<Word>(turns);
    }
    require(enter_block(execute, cpu, owner) == bound && cpu.cycles == bound &&
                cpu.instructions == bound,
            "The certified prefix executes exactly its admitted complete loop turns");
    require(cpu.reg == expected_registers && data == expected_data,
            "Independent sequential C++ copies match complete registers and RAM, "
            "including overlapping copies and partial loops");
  }
  return bound != 0;
}

struct Counts {
  std::uint64_t cases = 0;
  std::uint64_t accepted = 0;
};

Counts matrix(const Frame &frame) {
  Counts result;
  for (const Word source : {0U, 1U, 3U, 6U, 7U, 8U, word_mask}) {
    for (const Word destination : {0U, 1U, 3U, 4U, 6U, 7U, 8U, word_mask}) {
      for (const Word count : {1U, 2U, 3U, 4U, 8U}) {
        for (const std::uint64_t credit : {0U, 1U, 2U, 3U, 4U, 6U, 16U}) {
          for (const Word modifier : {word_mask, 0x8003U, 0U}) {
            result.accepted += static_cast<unsigned>(exercise(frame,
                                                              {.source = source,
                                                               .destination = destination,
                                                               .count = count,
                                                               .credit = credit,
                                                               .modifier = modifier}));
            ++result.cases;
          }
        }
      }
    }
  }
  return result;
}

}

namespace copy_cases {
Counts run() {
  std::array<Word, 16> program{};
  program[entry] = static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) | (3U << 11);
  program[entry + 1] = static_cast<Word>(isa::Opcode::move_x_effective) | (3U << 11) | (1U << 8);
  Data data{1, 2, 3, 4, 5, 6, 7, 8};
  Registers seed;
  seed.pc = entry;
  seed.la = entry + 1;
  seed.lc = 3;
  seed.sr = loop_flag_bit;
  seed.sp = seed.sc = 2;
  seed.stack[2] = std::uint64_t{entry} << word_bits;
  seed.m.fill(word_mask);
  seed.r[1] = 4;
  const MemoryWindow window{.words = data, .known = {}, .writable = true};
  const JitRuntime runtime{.memory = {}, .windows = {std::span(&window, 1), {}}};
  const auto operations = parse_block(program, seed, runtime);
  const std::array memory{RegionLayout{.space = Space::x, .first = 0, .size = data.size()}};
  LlvmModule admission_module("native_admission_probe");
  require(lift_region_admission(admission_module, operations, memory),
          "The complete linear loop has an admission query");
  LlvmCompiler admission_compiler;
  admission_compiler.define("dsp56300_region_memory",
                            std::bit_cast<const void *>(&dsp56300_region_memory));
  const auto admission = std::bit_cast<Admission>(admission_compiler.install(admission_module));
  LlvmModule execution_module("native_execution_probe");
  require(lift_region(execution_module, operations, memory),
          "The same semantic plan has native execution");
  LlvmCompiler execution_compiler;
  execution_compiler.define("dsp56300_region_memory",
                            std::bit_cast<const void *>(&dsp56300_region_memory));
  const auto *execute = execution_compiler.install(execution_module);
  const Frame frame{
      .admission = admission, .execute = execute, .seed = seed, .runtime = runtime, .data = data};
  const auto [cases, accepted] = matrix(frame);
  {
    Cpu cpu{.reg = seed};
    const auto before = cpu.reg;
    const MemoryWindow protected_window{.words = cpu.reg.r, .known = {}, .writable = true};
    const NativeContext owner{
        .runtime = {.memory = {}, .windows = {std::span(&protected_window, 1), {}}},
        .budget = 6,
        .cycle_deadline = 6};
    require(admission(&cpu, &owner) == 0 && cpu.reg == before,
            "Admission rejects a RAM binding overlapping core state");
  }
  return {.cases = cases, .accepted = accepted};
}
}

namespace pointer_cases {
using copy_cases::representation;
using copy_cases::require;
using namespace dsp56300;
using namespace dsp56300::detail;
using Data = std::array<Word, 8>;
using Admission = std::uint64_t (*)(const Cpu *, const NativeContext *);
constexpr Word entry = 2, loop_bit = 1U << 15;
constexpr unsigned high_mask = 0x18, high_shift = 17, first_r = 17;
constexpr std::array<Word, 7> values{0, 1, 3, 6, 7, 8, word_mask};

Word load(const unsigned target, const unsigned cursor) {
  constexpr unsigned register_bits = (7U << 16) | (high_mask << high_shift);
  return (static_cast<Word>(isa::Opcode::move_x_effective) & ~register_bits) |
         ((target & 7U) << 16) | ((target & high_mask) << high_shift) | (1U << 15) | (4U << 11) |
         (cursor << 8);
}

struct Fixture {
  unsigned depth;
  std::array<Word, 16> program{};
  Data data{};
  MemoryWindow window{.words = data, .known = {}, .writable = true};
  JitRuntime runtime{.memory = {}, .windows = {std::span(&window, 1), {}}};
  Registers seed;
  std::array<RegionLayout, 1> memory{{{.space = Space::x, .first = 0, .size = data.size()}}};
  LlvmCompiler admission_compiler;
  LlvmCompiler execution_compiler;
  Admission admission = nullptr;
  const void *execution = nullptr;

  explicit Fixture(const unsigned count) : depth(count) {
    for (unsigned index = 0; index < depth; ++index) {
      program[entry + index] = load(first_r + index, index);
    }
    program[entry + depth] =
        static_cast<Word>(isa::Opcode::move_x_effective) | (4U << 11) | (depth << 8);
    seed.pc = entry;
    seed.la = entry + depth;
    seed.lc = 3;
    seed.sr = loop_bit;
    seed.sp = seed.sc = 2;
    seed.stack[2] = std::uint64_t{entry} << word_bits;
    seed.m.fill(word_mask);
    constexpr Word sample = 0x123456;
    seed.x = sample;
    const auto operations = parse_block(program, seed, runtime);
    require(operations.size() == depth + 1, "The pointer chain retains all original operations");
    const auto plan = plan_region(operations, memory);
    require(plan && std::ranges::count_if(
                        plan->steps, [](const auto &step) { return step.guards != 0; }) == depth,
            "Every loaded pointer retains its real later address guard");
    LlvmModule query("computed_admission_" + std::to_string(depth));
    LlvmModule run("computed_execution_" + std::to_string(depth));
    require(lift_region_admission(query, operations, memory) &&
                lift_region(run, operations, memory),
            "The same native semantic plan supplies read-only admission and execution");
    const auto *binding = std::bit_cast<const void *>(&dsp56300_region_memory);
    admission_compiler.define("dsp56300_region_memory", binding);
    execution_compiler.define("dsp56300_region_memory", binding);
    admission = std::bit_cast<Admission>(admission_compiler.install(query));
    execution = execution_compiler.install(run);
  }
};

struct Input {
  Word source, rotation, count;
  std::uint64_t credit;
};

bool exercise(Fixture &fixture, const Input input) {
  for (std::size_t index = 0; index < fixture.data.size(); ++index) {
    fixture.data[index] = values[(index + input.rotation) % values.size()];
  }
  auto wanted_data = fixture.data;
  Cpu cpu{.reg = fixture.seed};
  cpu.reg.r[0] = input.source;
  cpu.reg.lc = input.count;
  auto wanted = cpu.reg;
  Word pointer = input.source;
  bool fits = true;
  for (unsigned index = 0; index < fixture.depth; ++index) {
    if (pointer >= fixture.data.size()) {
      fits = false;
      break;
    }
    pointer = fixture.data[pointer];
    wanted.r[index + 1] = pointer;
  }
  fits = fits && pointer < fixture.data.size() && input.credit >= fixture.depth + 1;
  NativeContext owner{.runtime = fixture.runtime, .budget = 16, .cycle_deadline = input.credit};
  const auto before_cpu = representation(cpu);
  const auto before_owner = representation(owner);
  const auto bound = fixture.admission(&cpu, &owner);
  require(representation(cpu) == before_cpu && representation(owner) == before_owner &&
              fixture.data == wanted_data,
          "Computed admission is byte-identical and performs no store or architectural effect");
  require(bound == (fits ? fixture.depth + 1 : 0),
          "Independent pointer chasing gives the exact complete admitted prefix");
  if (bound == 0) { return false; }
  wanted_data[pointer] = static_cast<Word>(wanted.x & word_mask);
  if (input.count == 1) {
    wanted.pc = entry + fixture.depth + 1;
    wanted.sr = wanted.sp = wanted.sc = wanted.la = wanted.lc = 0;
  } else {
    --wanted.lc;
  }
  require(enter_block(fixture.execution, cpu, owner) == bound && cpu.cycles == bound &&
              cpu.instructions == bound && cpu.reg == wanted && fixture.data == wanted_data,
          "Complete native state and aliased RAM match independent sequential pointer chasing");
  return true;
}

copy_cases::Counts matrix(Fixture &fixture) {
  copy_cases::Counts result;
  for (const auto source : values) {
    for (Word rotation = 0; rotation < values.size(); ++rotation) {
      for (const Word count : {1U, 2U, 3U}) {
        for (const std::uint64_t credit : {0U, 1U, 2U, 3U, 4U, 6U, 16U}) {
          result.accepted += static_cast<unsigned>(exercise(
              fixture, {.source = source, .rotation = rotation, .count = count, .credit = credit}));
          ++result.cases;
        }
      }
    }
  }
  return result;
}
}

namespace pointer_cases {
copy_cases::Counts run() {
  copy_cases::Counts result;
  for (const unsigned depth : {1U, 2U, 3U}) {
    Fixture fixture(depth);
    const auto counts = matrix(fixture);
    result.cases += counts.cases;
    result.accepted += counts.accepted;
  }
  return result;
}

}

namespace contracts {
using namespace dsp56300;
using namespace dsp56300::detail;
using copy_cases::representation;
using copy_cases::require;
using Admission = copy_cases::Admission;

struct Fixture {
  std::array<Word, 16> program{};
  copy_cases::Data data{1, 2, 3, 4, 5, 6, 7, 8};
  std::array<std::uint8_t, 8> known{};
  MemoryWindow view{.words = data, .known = known, .writable = true};
  JitRuntime runtime{.memory = {}, .windows = {std::span(&view, 1), {}}};
  Registers seed;
  std::vector<BlockInstruction> operations;
  std::array<RegionLayout, 1> memory{
      {{.space = Space::x, .first = 0, .size = data.size(), .tracked = true}}};

  Fixture() {
    constexpr auto entry = copy_cases::entry;
    known.fill(1);
    program[entry] = static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) | (3U << 11);
    program[entry + 1] = static_cast<Word>(isa::Opcode::move_x_effective) | (3U << 11) | (1U << 8);
    seed.pc = entry;
    seed.la = entry + 1;
    seed.lc = 3;
    seed.sr = loop_flag;
    seed.sp = seed.sc = 2;
    seed.stack[2] = std::uint64_t{entry} << word_bits;
    seed.m.fill(word_mask);
    seed.r[1] = 4;
    operations = parse_block(program, seed, runtime);
  }
};

void modules() {
  Fixture fixture;
  LlvmModule module("admission_and_execution");
  require(lift_region(module, fixture.operations, fixture.memory) &&
              lift_region_admission(module, fixture.operations, fixture.memory, "admission"),
          "Execution and read-only admission share a native module with distinct exports");
  auto *const query = LLVMGetNamedFunction(module.code(), "admission");
  auto *const execute = LLVMGetNamedFunction(module.code(), module.name().c_str());
  require(LLVMCountParams(query) == 2 && LLVMCountParams(execute) == 2 &&
              LLVMGetIntTypeWidth(LLVMGetReturnType(LLVMGlobalGetValueType(query))) == 64 &&
              LLVMGetIntTypeWidth(LLVMGetReturnType(LLVMGlobalGetValueType(execute))) == 32,
          "Query and execution exports retain their exact native ABI widths");
  const auto before = module.bitcode();
  bool duplicate = false;
  try {
    static_cast<void>(
        lift_region_admission(module, fixture.operations, fixture.memory, "admission"));
  } catch (const std::invalid_argument &) { duplicate = true; }
  require(duplicate && module.bitcode() == before &&
              !lift_region_admission(module, {}, fixture.memory, "unsupported") &&
              module.bitcode() == before,
          "Duplicate exports and unsupported plans preserve the whole existing module");
  LlvmCompiler compiler;
  compiler.define("dsp56300_region_memory", std::bit_cast<const void *>(&dsp56300_region_memory));
  const auto *native_execute = compiler.install(module);
  const auto native_query = std::bit_cast<Admission>(compiler.resolve("admission"));
  Cpu cpu{.reg = fixture.seed};
  NativeContext owner{.runtime = fixture.runtime, .budget = 6, .cycle_deadline = 6};
  require(native_query(&cpu, &owner) == 6 && enter_block(native_execute, cpu, owner) == 6,
          "The shared native object retains each entry's exact 64/32-bit ABI");
  bool transferred = false;
  try {
    static_cast<void>(lift_region_admission(module, fixture.operations, fixture.memory));
  } catch (const std::logic_error &) { transferred = true; }
  require(transferred, "Admission cannot mutate a transferred module");
}

void revisions() {
  Fixture fixture;
  LlvmModule module("admission_knowledge");
  require(lift_region_admission(module, fixture.operations, fixture.memory),
          "Tracked memory supports a read-only admission query");
  LlvmCompiler compiler;
  compiler.define("dsp56300_region_memory", std::bit_cast<const void *>(&dsp56300_region_memory));
  const auto query = std::bit_cast<Admission>(compiler.install(module));
  Cpu cpu{.reg = fixture.seed};
  const NativeContext owner{.runtime = fixture.runtime, .budget = 6, .cycle_deadline = 6};
  const auto check = [&](const std::uint64_t expected) {
    const auto initial_cpu = representation(cpu);
    const auto initial_owner = representation(owner);
    const auto initial_data = fixture.data;
    const auto initial_known = fixture.known;
    const auto initial_view = representation(fixture.view);
    require(query(&cpu, &owner) == expected && representation(cpu) == initial_cpu &&
                representation(owner) == initial_owner && fixture.data == initial_data &&
                fixture.known == initial_known && representation(fixture.view) == initial_view,
            "Admission rechecks live rights/knowledge without changing state or metadata");
  };
  check(6);
  fixture.known[0] = 0;
  check(0);
  fixture.known[0] = 2;
  check(6);
  fixture.known[4] = 0;
  check(0);
  fixture.known[4] = 2;
  check(0);
  fixture.known[4] = 1;
  fixture.view.writable = false;
  check(0);
  fixture.view.writable = true;
  fixture.view.words = std::span(fixture.data).first(7);
  fixture.view.known = std::span(fixture.known).first(7);
  check(0);
}
}

}

int main() try {
  const auto copies = copy_cases::run();
  const auto pointers = pointer_cases::run();
  constexpr unsigned copy_count = 7 * 8 * 5 * 7 * 3;
  constexpr unsigned pointer_count = 3 * 7 * 7 * 3 * 7;
  constexpr unsigned copy_admitted = 571;
  constexpr unsigned pointer_admitted = 735;
  copy_cases::require(copies.cases == copy_count && copies.accepted == copy_admitted &&
                          pointers.cases == pointer_count && pointers.accepted == pointer_admitted,
                      "Every independent admission matrix case was executed");
  contracts::modules();
  contracts::revisions();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
