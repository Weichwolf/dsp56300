// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/emitter.hpp"
#include "native/direct.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace dsp56300::test {
namespace {
constexpr Word port = 0xffff80, datum = 0x123456;
constexpr Word load_x0 = 0x44f000, saved_lc = 17;
constexpr std::uint64_t prefix_cycles = 258;
constexpr std::array costs{1U, 2U, 255U, 2U, 1U, 3U};
constexpr std::array seeds{
    std::uint64_t{0}, (std::uint64_t{1} << 32) - 2, std::numeric_limits<std::uint64_t>::max() - 2};

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

void ordinary(const bool io, const bool accept, const bool chained) {
  const auto ops = operations(io);
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
      for (Word index = 0; index < wanted; ++index) { cycles = sum(cycles, costs[index]); }
      require(retired == wanted && cpu.instructions == sum(seed, wanted) && cpu.cycles == cycles,
              "Every budget and fault prefix publishes exact modulo-64 counts and mixed costs");
      require(observer.observed == (io && budget >= 4),
              "I/O observes the completed prefix before either accepted or rejected retirement");
      require((context.failure.kind == FaultKind::io_read) == (io && !accept && budget >= 4),
              "A rejected transaction retains its actual failure");
    }
  }
}

void repeated() {
  const auto op = operations(false).front();
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
                  cpu.cycles == sum(seed, wanted) && cpu.repeat_remaining == 5 - wanted &&
                  cpu.reg.pc == start + (wanted == 5 ? 1U : 0U) &&
                  cpu.reg.lc == (wanted == 5 ? saved_lc : 5 - wanted),
              "REP budget exits and completion publish each iteration once, including wrap");
    }
  }
}
}
}

int main() try {
  for (const bool chained : {false, true}) {
    dsp56300::test::ordinary(false, true, chained);
    dsp56300::test::ordinary(true, false, chained);
    dsp56300::test::ordinary(true, true, chained);
  }
  dsp56300::test::repeated();
  std::cout << dsp56300::test::checks << " ARM64 progress publication checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
