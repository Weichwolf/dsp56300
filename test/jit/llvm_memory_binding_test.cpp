// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/region_memory.hpp"
#include "jit/llvm/runtime.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace {
using namespace dsp56300;
using namespace dsp56300::detail;
using test::require;
constexpr std::size_t words = 8;
using Data = std::array<Word, words>;
using Known = std::array<std::uint8_t, words>;

void flat_storage() {
  const Cpu cpu;
  NativeContext owner;
  Data first{};
  Data second{};
  Known sentinel{};
  owner.runtime.memory[0] = {.data = first.data(), .words = words};
  RegionMemory memory{.bank = 0,
                      .index = 0,
                      .regions = 1,
                      .first = 0,
                      .words = words,
                      .flags = region_writable | region_writing};
  auto *data = second.data();
  auto *known = sentinel.data();
  require(dsp56300_region_memory(&cpu, &owner, &memory, &data, &known) == 1 &&
              data == first.data() && known == nullptr,
          "Flat bindings return current owner RAM without an invented knowledge mask");
  owner.runtime.memory[0].data = second.data();
  require(dsp56300_region_memory(&cpu, &owner, &memory, &data, &known) == 1 &&
              data == second.data(),
          "The same compiled layout resolves another actual storage owner");
  memory.flags |= region_program_alias;
  owner.runtime.memory[0].aliases_program = true;
  require(dsp56300_region_memory(&cpu, &owner, &memory, &data, &known) == 0 &&
              data == second.data() && known == nullptr,
          "Ordinary region stores cannot overwrite executable aliases");
  memory.flags &= ~region_writing;
  require(dsp56300_region_memory(&cpu, &owner, &memory, &data, &known) == 1,
          "A declared read may still observe an actual program alias");
}

void protected_storage() {
  Cpu cpu;
  NativeContext owner;
  Data ordinary{};
  Known sentinel{};
  MemoryWindow view{.words = cpu.reg.r, .known = {}, .writable = true};
  owner.runtime.windows[0] = {&view, 1};
  RegionMemory memory{.bank = 0,
                      .index = 0,
                      .regions = 1,
                      .first = 0,
                      .words = words,
                      .flags = region_windowed | region_writable | region_writing};
  auto *data = ordinary.data();
  auto *known = sentinel.data();
  const auto reject = [&] {
    require(dsp56300_region_memory(&cpu, &owner, &memory, &data, &known) == 0 &&
                data == ordinary.data() && known == sentinel.data(),
            "Rejected storage cannot overwrite output bindings or bypass precise execution");
  };
  reject();
  view.words = ordinary;
  view.known = {reinterpret_cast<std::uint8_t *>(&cpu), words};
  memory.flags |= region_tracked;
  reject();
  view.known = {reinterpret_cast<std::uint8_t *>(&owner), words};
  reject();
  view.known = {reinterpret_cast<std::uint8_t *>(ordinary.data()), words};
  reject();
  view.known = sentinel;
  view.words = std::span(ordinary).first(words - 1);
  reject();
  view.words = ordinary;
  memory.index = 1;
  reject();
  memory.index = 0;
  memory.regions = 2;
  reject();
  require(cpu.instructions == 0 && cpu.cycles == 0 && cpu.reg == Registers{},
          "Binding admission neither executes nor materializes another CPU state");
}

void linked_flat_storage() {
  constexpr Word entry = 2;
  constexpr Word loop = 1U << 15;
  constexpr unsigned high_word = 24;
  std::array<Word, 16> program{};
  program[entry] = static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) | (3U << 11);
  program[entry + 1] = static_cast<Word>(isa::Opcode::move_x_effective) | (3U << 11) | (1U << 8);
  Data first{1, 2, 3, 4, 5, 6, 7, 8};
  Data second{8, 7, 6, 5, 4, 3, 2, 1};
  Registers seed;
  seed.pc = entry;
  seed.la = entry + 1;
  seed.lc = 3;
  seed.sr = loop;
  seed.sp = seed.sc = 2;
  seed.stack[2] = std::uint64_t{entry} << high_word;
  seed.m.fill(word_mask);
  seed.r[1] = 4;
  const JitRuntime source{.memory = {JitMemory{.data = first.data(), .words = words}, {}}};
  const auto operations = parse_block(program, seed, source);
  LlvmModule module("flat_region_binding");
  require(lift_region(module, operations, source), "A flat storage loop has linkable SSA code");
  LlvmCompiler compiler;
  compiler.define("dsp56300_region_memory", std::bit_cast<const void *>(&dsp56300_region_memory));
  const auto *const entry_point = compiler.install(module);
  for (auto *buffer : {&first, &second}) {
    auto wanted = *buffer;
    auto expected = seed;
    for (Word index = 0; index < 3; ++index) { wanted[4 + index] = (*buffer)[index]; }
    expected.x = (*buffer)[2];
    expected.r[0] = 3;
    expected.r[1] = 7;
    expected.sr = expected.sp = expected.sc = expected.la = expected.lc = 0;
    expected.pc = entry + 2;
    Cpu cpu{.reg = seed};
    NativeContext owner{
        .runtime = {.memory = {JitMemory{.data = buffer->data(), .words = words}, {}}},
        .budget = 6,
        .cycle_deadline = 6};
    require(enter_block(entry_point, cpu, owner) == 6 && cpu.reg == expected &&
                cpu.instructions == 6 && cpu.cycles == 6 && *buffer == wanted &&
                owner.boundary == NativeBoundary::context,
            "A single lowered flat loop resolves each owner's actual words and exact exit state");
  }
}
}

int main() try {
  flat_storage();
  protected_storage();
  linked_flat_storage();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
