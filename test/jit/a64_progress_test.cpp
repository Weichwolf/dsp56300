// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "native/direct.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word port = 0xffff80, datum = 0x123456;
constexpr Word load_x0 = 0x44f000, saved_lc = 17;
constexpr std::array costs{1U, 2U, 255U, 2U, 1U, 3U};
constexpr std::array seeds{std::uint64_t{0},
                           (std::uint64_t{1} << 32) - 2,
                           (std::uint64_t{1} << 63) - 2,
                           std::numeric_limits<std::uint64_t>::max() - 2};

std::uint64_t sum(const std::uint64_t value, const std::uint64_t increment) {
  const auto room = std::numeric_limits<std::uint64_t>::max() - value;
  return increment <= room ? value + increment : increment - room - 1;
}

struct Observer {
  detail::Cpu &cpu;
  std::uint64_t instructions, cycles;
  bool accept, observed = false;

  static void access(void *const owner, detail::NativeAccess &request) noexcept {
    auto &self = *static_cast<Observer *>(owner);
    self.observed = self.cpu.instructions == self.instructions && self.cpu.cycles == self.cycles &&
                    request.address == port && request.pc == start + 3 &&
                    request.space == Space::x && request.action == detail::BusAction::read;
    if (self.accept) {
      request.value = datum;
    } else {
      request.failure = {
          .kind = FaultKind::io_read, .space = Space::x, .address = port, .pc = start + 3};
    }
  }
};

auto operations(const bool io) {
  std::array<detail::BlockInstruction, costs.size()> result{};
  Word pc = start;
  for (std::size_t index = 0; index < result.size(); ++index) {
    const bool read = io && index == 3;
    auto instruction = detail::decode(read ? load_x0 : 0U);
    instruction.extension = read ? port : 0;
    instruction.cycles = static_cast<std::uint8_t>(costs[index]);
    require(instruction.words == (read ? 2U : 1U), "Authored NOP and absolute read lengths");
    result[index] = {.instruction = instruction, .pc = pc};
    pc += instruction.words;
  }
  return result;
}

void compact_control_flow() {
  const auto ops = operations(false);
  detail::a64::Emitter emitter;
  const auto bytes = emitter.generate_chained(ops);
  constexpr unsigned byte_bits = 8;
  constexpr std::uint32_t register_and_kind = 0xff00001f;
  constexpr std::uint32_t nonzero_w17 = 0x35000011;
  bool compact = false;
  for (std::size_t offset = 0; offset < bytes.size(); offset += sizeof(std::uint32_t)) {
    std::uint32_t word = 0;
    for (unsigned byte = 0; byte < sizeof(word); ++byte) {
      word |= std::uint32_t{bytes[offset + byte]} << (byte * byte_bits);
    }
    compact |= (word & register_and_kind) == nonzero_w17;
  }
  require(compact, "A bounded native body uses one direct CBNZ for its owner boundary");
}

void large_control_flow() {
  constexpr Word count = 32768;
  std::vector<detail::BlockInstruction> ops;
  ops.reserve(count);
  for (Word index = 0; index < count; ++index) {
    ops.push_back({.instruction = detail::decode(0), .pc = start + index});
  }
  detail::a64::Emitter emitter;
  const auto bytes = emitter.generate_chained(ops);
  require(bytes.size() > detail::a64::Code::near_branch_bytes,
          "The independently authored large body exceeds the conditional immediate span");
  detail::ExecutableArena arena;
  const auto *entry = arena.append(bytes);
  for (const Word budget : {0U, 1U, count - 1, count}) {
    detail::Cpu cpu;
    cpu.reg.pc = start;
    detail::NativeContext context{.runtime = {}, .budget = budget};
    auto expected = cpu.reg;
    expected.pc += budget;
    require(detail::enter_block(entry, cpu, context) == budget && cpu.reg == expected &&
                cpu.instructions == budget && cpu.cycles == budget,
            "Large native bodies retain full-range jumps and exact entry/exit prefixes");
  }
}

void ordinary(const bool io, const bool accept, const bool chained, const Word uniform = 0) {
  auto ops = operations(io);
  if (uniform != 0) {
    for (auto &op : ops) { op.instruction.cycles = static_cast<std::uint8_t>(uniform); }
  }
  const auto prefix_cycles = uniform == 0 ? costs[0] + costs[1] + costs[2] : 3 * uniform;
  detail::a64::Emitter emitter;
  const native::Direct code(chained ? emitter.generate_chained(ops) : emitter.generate(ops));
  for (const auto seed : seeds) {
    for (const Word budget : {0U, 1U, 2U, 3U, 4U, 5U, 6U, 100U}) {
      detail::Cpu cpu;
      cpu.reg.pc = start;
      cpu.instructions = seed;
      cpu.cycles = seed;
      Observer observer{.cpu = cpu,
                        .instructions = sum(seed, 3),
                        .cycles = sum(seed, prefix_cycles),
                        .accept = accept};
      detail::NativeContext context{.runtime = {}, .budget = budget};
      context.bus.owner = &observer;
      context.bus.access = Observer::access;
      const auto retired = code.enter(cpu, context);
      const Word wanted = std::min(budget, io ? 3U + static_cast<Word>(accept) : 6U);
      std::uint64_t cycles = seed;
      for (Word index = 0; index < wanted; ++index) {
        cycles = sum(cycles, uniform == 0 ? costs[index] : uniform);
      }
      require(retired == wanted && cpu.instructions == sum(seed, wanted) && cpu.cycles == cycles,
              "Every budget and fault prefix publishes exact modulo-64 counts and mixed costs");
      require(observer.observed == (io && budget >= 4),
              "I/O observes the completed prefix before either accepted or rejected retirement");
      require((context.failure.kind == FaultKind::io_read) == (io && !accept && budget >= 4),
              "A rejected transaction retains its actual failure");
    }
  }
}

void repeated(const Word cost = 1) {
  auto op = operations(false).front();
  op.instruction.cycles = static_cast<std::uint8_t>(cost);
  detail::a64::Emitter emitter;
  const native::Direct code(emitter.generate_repeat(op));
  for (const auto seed : seeds) {
    for (const Word budget : {0U, 1U, 4U, 5U, 6U, 100U}) {
      detail::Cpu cpu;
      cpu.reg.pc = start;
      cpu.reg.lc = cpu.repeat_remaining = 5;
      cpu.repeat_lc = saved_lc;
      cpu.instructions = cpu.cycles = seed;
      detail::NativeContext context{.runtime = {}, .budget = budget};
      const auto retired = code.enter(cpu, context);
      const auto wanted = std::min(budget, 5U);
      require(retired == wanted && cpu.instructions == sum(seed, wanted) &&
                  cpu.cycles == sum(seed, std::uint64_t{wanted} * cost) &&
                  cpu.repeat_remaining == 5 - wanted &&
                  cpu.reg.pc == start + (wanted == 5 ? 1U : 0U) &&
                  cpu.reg.lc == (wanted == 5 ? saved_lc : 5 - wanted),
              "REP budget exits and completion publish each iteration once, including wrap");
    }
  }
}

void finite_admission() {
  constexpr Word assign_r0 = 0x60f400;
  constexpr Word ccr_mask = 255;
  constexpr std::array<Word, 3> words{0, assign_r0, 0};
  constexpr std::array<Word, 4> lengths{0, 1, 3, 4};
  constexpr std::array<Word, 4> cycles{0, 1, 3, 4};
  std::array<detail::BlockInstruction, words.size()> ops{};
  for (std::size_t index = 0; index < ops.size(); ++index) {
    auto instruction = detail::decode(words[index]);
    instruction.extension = datum;
    require(instruction.words == lengths[index + 1] - lengths[index] &&
                instruction.cycles == cycles[index + 1] - cycles[index],
            "Authored NOP/MOVE/NOP lengths and costs are independently fixed");
    ops[index] = {.instruction = instruction, .pc = start + lengths[index]};
  }
  detail::a64::Emitter emitter;
  const native::Direct code(detail::admit_cycles(emitter.generate_chained(ops), ops));
  constexpr std::array<std::uint64_t, 3> beginnings{
      0, std::uint64_t{1} << 32, std::numeric_limits<std::uint64_t>::max() - 8};
  for (const auto beginning : beginnings) {
    for (const auto credit : {4U, 5U, 6U}) {
      for (Word budget = 0; budget <= 6; ++budget) {
        for (Word ccr = 0; ccr <= ccr_mask; ++ccr) {
          detail::Cpu cpu;
          cpu.reg.pc = start;
          cpu.reg.sr = ccr;
          cpu.instructions = cpu.cycles = beginning;
          auto expected = cpu.reg;
          const auto count = std::min(budget, Word{3});
          expected.pc += lengths[count];
          if (count >= 2) { expected.r[0] = datum; }
          detail::NativeContext context{
              .runtime = {}, .budget = budget, .cycle_deadline = beginning + credit};
          require(code.enter(cpu, context) == count && cpu.reg == expected &&
                      cpu.instructions == beginning + count &&
                      cpu.cycles == beginning + cycles[count] &&
                      context.failure.kind == FaultKind::none,
                  "A complete finite block fits its exact mixed cost in one native entry");
        }
      }
    }
  }
}

void uniform_admission() {
  for (const Word cost : {1U, 2U, 4U, 128U}) {
    auto ops = operations(false);
    for (auto &op : ops) { op.instruction.cycles = static_cast<std::uint8_t>(cost); }
    for (const std::size_t extent : {2U, 6U}) {
      const auto body = std::span(ops).first(extent);
      detail::a64::Emitter emitter;
      const native::Direct code(detail::admit_cycles(emitter.generate_chained(body), body));
      for (const auto beginning : seeds) {
        for (Word available = 0; available <= body.size(); ++available) {
          const auto credit = std::uint64_t{available} * cost;
          if (credit >= std::numeric_limits<std::uint64_t>::max() - beginning) { continue; }
          for (Word budget = 0; budget <= body.size() + 1; ++budget) {
            detail::Cpu cpu;
            cpu.reg.pc = start;
            cpu.instructions = cpu.cycles = beginning;
            auto expected = cpu.reg;
            const auto count = std::min(budget, available);
            expected.pc += count;
            detail::NativeContext context{
                .runtime = {}, .budget = budget, .cycle_deadline = beginning + credit};
            require(code.enter(cpu, context) == count && cpu.reg == expected &&
                        cpu.instructions == sum(beginning, count) &&
                        cpu.cycles == sum(beginning, std::uint64_t{count} * cost) &&
                        context.failure.kind == FaultKind::none,
                    "Uniform bodies publish every exact finite deadline and budget prefix");
          }
        }
      }
    }
  }
}

void uniform_publication() {
  auto ops = operations(false);
  for (auto &op : ops) { op.instruction.cycles = 1; }
  for (const std::size_t count : {2U, 6U}) {
    detail::a64::Emitter emitter;
    const auto bytes = emitter.generate_chained(std::span(ops).first(count));
    constexpr std::uint32_t store_base_mask = 0xffc003e0;
    constexpr std::uint32_t store_to_cpu = 0xf9000260;
    constexpr std::uint32_t immediate_mask = 0xfff;
    constexpr unsigned immediate_shift = 10;
    constexpr unsigned byte_bits = 8;
    unsigned writes = 0;
    for (std::size_t offset = 0; offset < bytes.size(); offset += sizeof(std::uint32_t)) {
      std::uint32_t word = 0;
      for (unsigned byte = 0; byte < sizeof(word); ++byte) {
        word |= std::uint32_t{bytes[offset + byte]} << (byte * byte_bits);
      }
      if ((word & store_base_mask) == store_to_cpu &&
          ((word >> immediate_shift) & immediate_mask) * sizeof(std::uint64_t) ==
              offsetof(detail::Cpu, cycles)) {
        ++writes;
      }
    }
    require(writes == 1, "An unobserved uniform body publishes cycles once at its precise exit");
  }
}

void unused_banks() {
  constexpr std::array<std::uint32_t, 10> spills{0xf9400a75,
                                                 0xf9400e76,
                                                 0xf9400277,
                                                 0xf9400678,
                                                 0xf9000a75,
                                                 0xf9000e76,
                                                 0xf9000277,
                                                 0xf9000678,
                                                 0xa9bf5bf5,
                                                 0xa9bf63f7};
  const std::array ops{operations(false).front()};
  detail::a64::Emitter emitter;
  const auto bytes = emitter.generate_chained(ops);
  for (std::size_t offset = 0; offset < bytes.size(); offset += sizeof(std::uint32_t)) {
    std::uint32_t word = 0;
    for (unsigned byte = 0; byte < sizeof(word); ++byte) {
      word |= std::uint32_t{bytes[offset + byte]} << (byte * 8U);
    }
    require(std::ranges::find(spills, word) == spills.end(),
            "A NOP entry must not load, store or save unreferenced A/B/X/Y banks");
  }
  const native::Direct code(bytes);
  constexpr Word ccr_mask = 255;
  constexpr std::uint64_t accumulator = 0x123456789abc00;
  constexpr std::uint64_t bank = 0xabcdef123456;
  for (Word ccr = 0; ccr <= ccr_mask; ++ccr) {
    detail::Cpu cpu;
    cpu.reg.pc = start;
    cpu.reg.sr = ccr;
    cpu.reg.a = accumulator;
    cpu.reg.b = accumulator ^ (accumulator >> 8U);
    cpu.reg.x = bank;
    cpu.reg.y = bank ^ (bank >> 8U);
    auto expected = cpu.reg;
    ++expected.pc;
    detail::NativeContext context{.runtime = {}, .budget = 1};
    require(code.enter(cpu, context) == 1 && cpu.reg == expected && cpu.instructions == 1 &&
                cpu.cycles == 1 && context.failure.kind == FaultKind::none,
            "Removing unused host bank spills preserves the complete guest state");
  }
}
}
}

int main() try {
  for (const bool chained : {false, true}) {
    dsp56300::test::ordinary(false, true, chained);
    dsp56300::test::ordinary(true, false, chained);
    dsp56300::test::ordinary(true, true, chained);
    for (const dsp56300::Word cost : {1U, 2U, 3U, 255U}) {
      dsp56300::test::ordinary(false, true, chained, cost);
      dsp56300::test::ordinary(true, true, chained, cost);
      dsp56300::test::ordinary(true, false, chained, cost);
    }
  }
  dsp56300::test::compact_control_flow();
  dsp56300::test::large_control_flow();
  dsp56300::test::uniform_publication();
  dsp56300::test::uniform_admission();
  for (const dsp56300::Word cost : {1U, 2U, 3U, 255U}) { dsp56300::test::repeated(cost); }
  dsp56300::test::finite_admission();
  dsp56300::test::unused_banks();
  std::cout << dsp56300::test::checks << " ARM64 progress publication checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
