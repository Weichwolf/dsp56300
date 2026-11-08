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
#include <vector>

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

void long_block_deadlines() {
  constexpr Word body = 257;
  constexpr Word period = body + 3;
  std::vector<Word> ram(body + 1, nop);
  ram.back() = jump;
  Core core(Memory::unified(ram));
  for (std::uint64_t deadline = 1; deadline < std::uint64_t{period} * 4; ++deadline) {
    const auto result = core.run_until(deadline);
    const auto remainder = deadline % period;
    const auto cycles = deadline + (remainder > body ? period - remainder : 0);
    const auto pc = cycles % period;
    require(result.fault.kind == FaultKind::none && core.registers().pc == pc &&
                result.progress == Progress{.instructions = ((cycles / period) * (body + 1)) + pc,
                                            .estimated_cycles = cycles},
            "Long retained logical blocks preserve exact deadlines at every interior offset");
  }
  const auto before = core.progress();
  require(core.run(body + 1).retired == body + 1 &&
              core.progress().estimated_cycles == before.estimated_cycles + period,
          "Untimed execution after a deadline has no stale admission limit");
  const auto target = core.progress().estimated_cycles + (std::uint64_t{period} * 100);
  require(core.run_until(target).progress.estimated_cycles == target && core.registers().pc == 0,
          "Warm deadline execution resumes across long native blocks after an untimed run");
}

void chunked_deadlines() {
  constexpr std::array<Word, 3> lengths{1, 31, 257};
  constexpr std::array<std::uint64_t, 9> grants{1, 2, 3, 7, 16, 31, 64, 127, 4096};
  for (const auto body : lengths) {
    const auto period = body + 3;
    std::vector<Word> ram(body + 1, nop);
    ram.back() = jump;
    Core core(Memory::unified(ram));
    std::uint64_t deadline = 0;
    for (unsigned pass = 0; pass < 4; ++pass) {
      for (const auto grant : grants) {
        const auto before = core.progress();
        deadline += grant;
        const auto result = core.run_until(deadline);
        const auto remainder = deadline % period;
        const auto cycles = deadline + (remainder > body ? period - remainder : 0);
        const auto pc = cycles % period;
        const auto instructions = ((cycles / period) * (body + 1)) + pc;
        require(result.fault.kind == FaultKind::none && result.state == ExecutionState::running &&
                    result.progress ==
                        Progress{.instructions = instructions, .estimated_cycles = cycles} &&
                    result.retired == instructions - before.instructions &&
                    core.registers().pc == pc,
                "Changing event spans retain exact mixed-cost progress through complete and "
                "partial native blocks");
      }
    }
  }
}

void hardware_loop_deadlines() {
  constexpr Word count = 67;
  constexpr Word body = 31;
  constexpr Word setup_cycles = 5;
  constexpr Word body_cycles = count * body;
  constexpr Word end_cycles = setup_cycles + body_cycles;
  std::vector<Word> ram(body + 3, nop);
  ram[0] = isa::opcode<isa::Opcode::do_immediate>().front() | (count << 8);
  ram[1] = body + 1;
  ram.back() = isa::stop.front();
  Core core(Memory::unified(ram));
  constexpr std::array<Word, 9> deadlines{1, 6, 35, 36, 37, 1000, 2000, end_cycles, end_cycles + 1};
  for (const auto deadline : deadlines) {
    const auto result = core.run_until(deadline);
    const auto stopped = deadline > end_cycles;
    const auto cycles = stopped ? end_cycles + 10 : std::max(deadline, setup_cycles);
    const auto retired_body = std::min(cycles - setup_cycles, body_cycles);
    const auto finished = retired_body == body_cycles;
    auto pc = 2 + (retired_body % body);
    if (finished) { pc = body + 2; }
    if (stopped) { pc = body + 3; }
    require(result.fault.kind == FaultKind::none && core.registers().pc == pc &&
                core.registers().lc == (finished ? 0 : count - (retired_body / body)) &&
                core.registers().sc == (finished ? 0U : 2U) &&
                result.progress ==
                    Progress{.instructions = 1 + retired_body + static_cast<unsigned>(stopped),
                             .estimated_cycles = cycles},
            "Deadlines retain DO counters and stack frames across batched iterations and STOP");
  }
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
    dsp56300::test::long_block_deadlines();
    dsp56300::test::chunked_deadlines();
    dsp56300::test::hardware_loop_deadlines();
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
