// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word jump = isa::opcode<isa::Opcode::jmp_absolute_short>().front();
constexpr Word nop = isa::nop.front(), reset = isa::reset.front();
constexpr unsigned loop_cycles = 4;
constexpr std::uint64_t last_sweep_deadline = 512, warm_deadline = 400000;
constexpr std::uint64_t warm_instructions = 200000, device_deadline = 100000;
constexpr std::uint64_t later_deadline = 200000;

void loop_deadlines() {
  std::array<Word, 16> ram{nop, jump};
  Core core(Memory::unified(ram));
  for (std::uint64_t deadline = 0; deadline <= last_sweep_deadline; ++deadline) {
    const auto previous = core.state();
    const auto result = core.run_until(deadline);
    const auto remainder = deadline % loop_cycles;
    const auto expected_cycles = deadline + (remainder > 1 ? loop_cycles - remainder : 0);
    const auto instructions =
        ((expected_cycles / loop_cycles) * 2) + (expected_cycles % loop_cycles);
    require(result.fault.kind == FaultKind::none && result.state == ExecutionState::running &&
                result.progress ==
                    Progress{.instructions = instructions, .estimated_cycles = expected_cycles} &&
                result.retired == instructions - previous.progress.instructions &&
                core.registers().pc == expected_cycles % loop_cycles,
            "NOP/three-cycle jump stop at the first whole instruction at each absolute deadline");
  }
  const auto images = codegen::publications();
  const auto warm = core.run_until(warm_deadline);
  require(warm.progress ==
                  Progress{.instructions = warm_instructions, .estimated_cycles = warm_deadline} &&
              core.registers().pc == 0 && warm.fault.kind == FaultKind::none,
          "A long warm native run retains exact mixed-cost progress across linked blocks");
  require(codegen::publications() == images,
          "Deadline changes never produce budget-specific translations or replace retained code");
  const auto previous = core.state();
  require(core.run_until(0).retired == 0 && core.run_until(warm_deadline).retired == 0 &&
              core.state().registers == previous.registers &&
              core.state().progress == previous.progress,
          "Reached and past deadlines perform no guest instruction or state replacement");
}

void jump_and_repeat_deadlines() {
  constexpr unsigned jump_cost = 3;
  constexpr std::array<std::uint64_t, 4> deadlines{37, 511, 8193, 100003};
  for (const auto deadline : deadlines) {
    std::array<Word, 16> ram{jump};
    Core core(Memory::unified(ram));
    const auto result = core.run_until(deadline);
    const auto instructions = (deadline + jump_cost - 1) / jump_cost;
    require(result.retired == instructions && result.progress.instructions == instructions &&
                result.progress.estimated_cycles == instructions * jump_cost &&
                result.fault.kind == FaultKind::none && core.registers().pc == 0,
            "A linked stream of three-cycle jumps cannot overrun more than its final instruction");
  }
  constexpr Word repetitions = 17;
  constexpr Word count_shift = 8;
  constexpr Word setup_cycles = 5;
  constexpr Word final_cycles = setup_cycles + repetitions + 10;
  std::array<Word, 16> ram{isa::opcode<isa::Opcode::rep_immediate>().front() |
                               (repetitions << count_shift),
                           nop,
                           isa::stop.front()};
  Core core(Memory::unified(ram));
  for (std::uint64_t deadline = 1; deadline <= final_cycles; ++deadline) {
    const auto result = core.run_until(deadline);
    auto cycles = std::max<std::uint64_t>(deadline, setup_cycles);
    if (deadline > setup_cycles + repetitions) { cycles = final_cycles; }
    const auto body = std::min<std::uint64_t>(cycles - setup_cycles, repetitions);
    const auto instructions = 1 + body + static_cast<unsigned>(cycles == final_cycles);
    Word pc = 1;
    if (cycles == setup_cycles + repetitions) { pc = 2; }
    if (cycles == final_cycles) { pc = 3; }
    require(
        result.fault.kind == FaultKind::none &&
            result.progress == Progress{.instructions = instructions, .estimated_cycles = cycles} &&
            core.registers().pc == pc,
        "Successive absolute deadlines preserve the actual REP body latch and count through STOP");
  }
}

class Device final : public Io {
public:
  Core *core = nullptr;
  unsigned calls = 0, rejections = 0;
  Progress before_effect;
  bool refuse = false;

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
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

  bool reset_peripherals(Word /*pc*/) noexcept override {
    ++calls;
    before_effect = core->progress();
    try {
      static_cast<void>(core->run_until(0));
    } catch (const std::logic_error &) { ++rejections; }
    core->request_yield();
    return !refuse;
  }
};

void effect_exit() {
  std::array<Word, 16> ram{nop, reset, nop, jump};
  Device device;
  Core core(Memory::unified(ram), &device);
  device.core = &core;
  const auto first = core.run_until(device_deadline);
  require(
      first.retired == 2 && first.yielded && first.fault.kind == FaultKind::none &&
          first.progress == Progress{.instructions = 2, .estimated_cycles = 8} &&
          core.registers().pc == 2 && device.calls == 1 && device.rejections == 1 &&
          device.before_effect == Progress{.instructions = 1, .estimated_cycles = 1},
      "A real seven-cycle RESET yields once, preserving its prefix and rejecting callback reentry");
  const auto resumed = core.run_until(9);
  require(resumed.retired == 1 && !resumed.yielded &&
              resumed.progress == Progress{.instructions = 3, .estimated_cycles = 9} &&
              core.registers().pc == 3 && device.calls == 1,
          "The next absolute deadline resumes after the accepted effect without replay");
  const auto jump_result = core.run_until(10);
  require(jump_result.retired == 1 && jump_result.progress.estimated_cycles == 12 &&
              core.registers().pc == 0 && !jump_result.yielded,
          "A deadline inside a three-cycle jump completes that one atomic instruction");
  device.refuse = true;
  const auto failed = core.run_until(device_deadline);
  require(failed.fault.kind == FaultKind::host_event && failed.retired == 1 &&
              failed.progress == Progress{.instructions = 5, .estimated_cycles = 13} &&
              device.calls == 2 && device.rejections == 2,
          "A refused real effect stops with only its actually retired prefix");
  require(core.run_until(later_deadline).retired == 0 && device.calls == 2,
          "A latched failure cannot become a successful later deadline");
}

void sleep_and_overflow() {
  std::array<Word, 16> ram{nop, isa::stop.front()};
  Core core(Memory::unified(ram));
  const auto stopped = core.run_until(device_deadline);
  require(stopped.retired == 2 && stopped.state == ExecutionState::stopped &&
              stopped.progress == Progress{.instructions = 2, .estimated_cycles = 11} &&
              core.run_until(later_deadline).retired == 0,
          "Native STOP ends the deadline run without fabricating idle CPU instructions");
  core.reset();
  const auto before = core.state();
  bool overflow = false;
  try {
    static_cast<void>(core.run_until(std::numeric_limits<std::uint64_t>::max()));
  } catch (const std::overflow_error &) { overflow = true; }
  require(overflow && core.state().registers == before.registers &&
              core.state().progress == before.progress,
          "An unrepresentable final atomic boundary rejects before guest execution");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::loop_deadlines();
    dsp56300::test::jump_and_repeat_deadlines();
    dsp56300::test::effect_exit();
    dsp56300::test::sleep_and_overflow();
    std::cout << dsp56300::test::checks << " public deadline checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
