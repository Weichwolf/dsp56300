// SPDX-License-Identifier: GPL-3.0-only
#include "debug_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "return_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace dsp56300::test {
namespace {
using debug_model::Input;
constexpr Word words = 2048, external = 1024, host_vector = 64, outer_handler = 640;
constexpr Word service = 768, jsr = 0x0d0000, repeat_four = 0x0604a0, trap = 6;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_a = 0x200021, round_a = 0x200011;
constexpr Word wait = 0x86, stop = 0x87, reset = 0x84;
constexpr Word ce = Word{1} << 19, be = Word{1} << 10;
constexpr Word excess_budget = 100, invalid = 7;
constexpr std::array backends{Backend::reference, Backend::cached, Backend::jit};

struct Machine {
  std::array<Word, words> p{};
  std::array<Word, 1> x{}, y{};
  Core core;

  explicit Machine(const Backend backend)
      : core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend}) {}
};

template <typename Action> bool rejected(Action action) {
  try {
    action();
  } catch (const std::exception &) { return true; }
  return false;
}

void alu(Registers &state, const Word operation) {
  const auto old = state;
  sequence_model::alu(state, old, operation);
  ++state.pc;
}

void quiescent(Core &core,
               const Registers &state,
               const Progress progress,
               const DebugStatus status) {
  for (const auto budget : {0U, std::numeric_limits<std::uint32_t>::max()}) {
    const auto result = core.run(budget);
    require(result.dispatches == 0 && result.state == ProcessingState::debugging &&
                result.fault.kind == FaultKind::none && result.progress == progress &&
                core.registers() == state && core.debug_status() == status && !core.interruptible(),
            "Debug halt cannot execute, fetch a successor or consume an arbitrary budget");
  }
  core.clear_code_cache();
  core.set_registers(state);
  require(core.processing_state() == ProcessingState::debugging && core.debug_status() == status &&
              core.run(1).dispatches == 0 && core.registers() == state,
          "Cache clearing and visible-register edits preserve debug mode and cause");
  require(rejected([&core] { core.interrupt(host_vector, 3); }) &&
              rejected([&core] { core.resume_from_stop(); }) &&
              rejected([&core] { core.request_debug(); }) && core.debug_status() == status,
          "Ordinary IRQ, STOP completion and duplicate debug requests cannot replace a halt");
}

void native_debug(const Backend backend, const Input input) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array code{mac_a, mac_b, debug_model::opcode(input), adc_a, round_a, invalid};
  core.load(Space::program, start, code);
  core.load(Space::program, debug_model::vector(input), std::array{jsr | service, Word{0}});
  core.load(Space::program, service, std::array{adc_a, round_a, return_model::rti});
  auto initial = debug_model::seed(input);
  auto warm = initial;
  warm.pc = start + 3;
  warm.sr = 0;
  core.set_registers(warm);
  require(core.run(2).dispatches == 2, "Warm the debug continuation under an old mode");
  core.set_registers(initial);
  core.set_debug_interrupt_mode(input.interrupt);
  auto expected = initial;
  alu(expected, mac_a);
  alu(expected, mac_b);
  const bool requested = debug_model::taken(input, expected.sr);
  ++expected.pc;
  const Word prefix_cost = 4 + debug_model::cycles(input);
  const auto result = core.run(3);
  const auto status = debug_model::status(input, requested);
  const auto state =
      requested && !input.interrupt ? ProcessingState::debugging : ProcessingState::running;
  require(result.dispatches == 3 && result.state == state && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 5, .estimated_cycles = prefix_cost} &&
              core.registers() == expected && core.debug_status() == status &&
              core.interruptible() == !requested,
          "DEBUGcc consumes freshly materialized native flags and preserves the complete state");
  if (requested && !input.interrupt) {
    quiescent(core, expected, result.progress, status);
    core.set_debug_interrupt_mode(true);
    require(core.debug_status() == DebugStatus{.interrupt_mode = true, .software_occurrence = true},
            "Changing IME while halted cannot erase software-debug occurrence");
    core.set_debug_interrupt_mode(false);
    expected.sr = (expected.sr & status_byte) | mode_rm | mode_down;
    core.set_registers(expected);
    core.write(Space::program, start + 3, mac_b);
    core.resume_from_debug();
    require(core.debug_status() == DebugStatus{} &&
                core.processing_state() == ProcessingState::running,
            "Explicit resume clears SWO, retaining debugger register/program edits");
  }
  Word remaining = 2;
  Word extra_cost = 2;
  if (requested && input.interrupt) {
    require(core.run(0).dispatches == 0 && !core.interruptible(),
            "Zero budget retains a pending debug IRQ");
    debug_model::enter(expected);
    const auto entry = core.run(1);
    require(entry.dispatches == 1 && core.registers() == expected,
            "IME uses VBA:06, a complete per-bit frame and priority three");
    alu(expected, adc_a);
    alu(expected, round_a);
    debug_model::leave(expected);
    remaining = 5;
    extra_cost = 10;
  }
  const Word consumer = requested && !input.interrupt ? mac_b : adc_a;
  alu(expected, consumer);
  alu(expected, round_a);
  const auto continued = core.run(remaining);
  const Word retired = requested && input.interrupt ? 11U : 7U;
  require(continued.dispatches == remaining && continued.state == ProcessingState::running &&
              continued.fault.kind == FaultKind::none && core.registers() == expected &&
              continued.progress ==
                  Progress{.instructions = retired, .estimated_cycles = prefix_cost + extra_cost} &&
              core.debug_status() == DebugStatus{.interrupt_mode = input.interrupt},
          "Debug resume/RTI runs correct fresh Carry/mode consumers despite warm old native code");
  if (backend == Backend::jit) {
    const auto stats = core.jit_statistics();
    require(stats.native_alu_instructions == (requested && input.interrupt ? 8U : 6U) &&
                stats.helper_instructions == (requested && input.interrupt ? 3U : 1U),
            "Debug boundaries contain actual native warmup, prefix, handler and continuation work");
  }
}

void external_debug(const Backend backend, const ProcessingState source, const bool interrupt) {
  Machine machine(backend);
  auto &core = machine.core;
  const Word operation = source == ProcessingState::waiting ? wait : stop;
  core.write(Space::program, start, operation);
  const Input input{.status = mode_up | flag_c, .bank = 512, .pointer = 2, .interrupt = interrupt};
  auto expected = debug_model::seed(input);
  core.load(Space::program, debug_model::vector(input), std::array{jsr | service, Word{0}});
  core.set_registers(expected);
  core.set_debug_interrupt_mode(interrupt);
  Progress progress;
  if (source != ProcessingState::running) {
    ++expected.pc;
    progress = {.instructions = 1, .estimated_cycles = 10};
    require(core.run(1).state == source, "Enter WAIT/STOP before external DE/JTAG request");
  }
  core.request_debug();
  require(core.registers() == expected && core.progress() == progress &&
              core.debug_status() == DebugStatus{.interrupt_mode = interrupt},
          "Host debug request has no software occurrence, retirement or register effects");
  if (!interrupt) {
    quiescent(core, expected, progress, {});
    core.resume_from_debug();
    require(core.processing_state() == ProcessingState::running && core.registers() == expected &&
                core.progress() == progress,
            "DE/JTAG wake resumes the program rather than returning to a prior sleep");
  } else {
    require(core.processing_state() == ProcessingState::running && !core.interruptible() &&
                core.run(0).dispatches == 0,
            "IME wakes sleepers but leaves bounded pending work");
    debug_model::enter(expected);
    ++progress.instructions;
    progress.estimated_cycles += 3;
    require(core.run(1).dispatches == 1 && core.registers() == expected &&
                core.progress() == progress,
            "An external IME request enters its vector without a fabricated DEBUG instruction");
  }
}

void repeated_halts(const Backend backend, const bool conditional) {
  Machine machine(backend);
  auto &core = machine.core;
  const Word operation = conditional ? debug_model::conditional + 10 : debug_model::debug;
  core.load(Space::program, start, std::array{repeat_four, operation, Word{0}, invalid});
  const Input input{.status = flag_z | mode_rm, .pointer = 2};
  const auto initial = debug_model::seed(input);
  auto expected = initial;
  ++expected.pc;
  expected.lc = 4;
  core.set_registers(initial);
  require(core.run(1).dispatches == 1 && core.registers() == expected,
          "REP initializes its counter before a debug body");
  const Word cost = conditional ? 5U : 1U;
  for (Word iteration = 0; iteration < 4; ++iteration) {
    const Word left = 3 - iteration;
    expected.pc = start + (left == 0 ? 2U : 1U);
    expected.lc = left == 0 ? initial.lc : left;
    const auto result = core.run(excess_budget);
    require(result.dispatches == 1 && result.state == ProcessingState::debugging &&
                result.fault.kind == FaultKind::none && core.registers() == expected &&
                result.progress == Progress{.instructions = 2 + iteration,
                                            .estimated_cycles = 5 + ((iteration + 1) * cost)} &&
                core.debug_status().software_occurrence,
            "Each repeated debug halt preserves the exact remaining REP/LC continuation");
    core.write(Space::program, start + 1, 0);
    core.clear_code_cache();
    core.resume_from_debug();
  }
  ++expected.pc;
  require(core.run(1).dispatches == 1 && core.registers() == expected &&
              !core.debug_status().software_occurrence,
          "Debug resume preserves the repeated instruction latch across program/cache mutation");
}

void repeat_condition_change(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  constexpr Word debug_equal = debug_model::conditional + 10;
  core.load(Space::program, start, std::array{repeat_four, debug_equal, invalid});
  const Input input{.status = flag_z, .pointer = 2};
  auto expected = debug_model::seed(input);
  core.set_registers(expected);
  require(core.run(2).state == ProcessingState::debugging,
          "Conditional REP halts at the first true condition");
  expected.pc = start + 1;
  expected.lc = 3;
  expected.sr = flag_c;
  core.set_registers(expected);
  core.resume_from_debug();
  expected.pc = start + 2;
  expected.lc = 7;
  const auto result = core.run(3);
  constexpr Word cycles = 25;
  require(result.dispatches == 3 && result.state == ProcessingState::running &&
              result.fault.kind == FaultKind::none && core.registers() == expected &&
              result.progress == Progress{.instructions = 5, .estimated_cycles = cycles} &&
              core.debug_status() == DebugStatus{},
          "Latched DEBUGcc reevaluates debugger-edited flags on every remaining REP iteration");
}

void repeat_interrupt(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  const Input input{.status = mode_up | flag_c, .bank = 512, .pointer = 2, .interrupt = true};
  core.load(Space::program, start, std::array{repeat_four, debug_model::debug, invalid});
  core.load(Space::program, debug_model::vector(input), std::array{jsr | service, Word{0}});
  auto expected = debug_model::seed(input);
  core.set_registers(expected);
  core.set_debug_interrupt_mode(true);
  const auto partial = core.run(2);
  expected.pc = start + 1;
  expected.lc = 3;
  require(partial.dispatches == 2 && core.registers() == expected && !core.interruptible(),
          "REP IME debug requests defer without halting or entering a vector early");
  core.clear_code_cache();
  expected.vba = external;
  core.set_registers(expected);
  require(core.run(3).dispatches == 3, "Coalesced IME requests finish all repeated instructions");
  expected.pc = start + 2;
  expected.lc = 7;
  require(core.registers() == expected && !core.interruptible(),
          "Pending debug survives completed REP");
  core.set_debug_interrupt_mode(false);
  debug_model::enter(expected);
  const auto result = core.run(1);
  require(result.dispatches == 1 && core.registers() == expected &&
              result.progress == Progress{.instructions = 6, .estimated_cycles = 12} &&
              core.debug_status() == DebugStatus{},
          "Queued debug retains its original VBA/mode identity across control/cache changes");
}

void fast_halt(const Backend backend, const bool second_slot) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array code =
      second_slot ? std::array{mac_a, debug_model::debug} : std::array{debug_model::debug, mac_b};
  core.load(Space::program, host_vector, code);
  const Input input{.status = mode_rm | mode_down | flag_c, .pointer = 2};
  auto expected = debug_model::seed(input);
  core.set_registers(expected);
  if (second_slot) {
    const auto old = expected;
    sequence_model::alu(expected, old, mac_a);
  }
  core.interrupt(host_vector, 0);
  Progress progress{.instructions = second_slot ? 2U : 1U,
                    .estimated_cycles = second_slot ? 2U : 1U};
  require(core.processing_state() == ProcessingState::debugging && core.registers() == expected &&
              core.progress() == progress && core.debug_status().software_occurrence,
          "DEBUG in either fast slot pauses with the foreground PC and exact vector progress");
  quiescent(core, expected, progress, {.software_occurrence = true});
  core.resume_from_debug();
  if (!second_slot) {
    const auto old = expected;
    sequence_model::alu(expected, old, mac_b);
  } else {
    ++expected.pc;
  }
  ++progress.instructions;
  ++progress.estimated_cycles;
  require(core.run(1).dispatches == 1 && core.registers() == expected &&
              core.progress() == progress && core.processing_state() == ProcessingState::running,
          "Debug resume completes an unfinished vector before any foreground instruction");
}

void fast_interrupt(const Backend backend, const bool long_second) {
  Machine machine(backend);
  auto &core = machine.core;
  const Input input{
      .status = mode_up | flag_s | flag_c, .bank = 512, .pointer = 2, .interrupt = true};
  core.load(Space::program,
            host_vector,
            std::array{debug_model::debug, long_second ? jsr | outer_handler : mac_a});
  core.load(Space::program, debug_model::vector(input), std::array{jsr | service, Word{0}});
  auto expected = debug_model::seed(input);
  core.set_registers(expected);
  core.set_debug_interrupt_mode(true);
  if (long_second) {
    debug_model::enter(expected);
    expected.pc = outer_handler;
    // Outer host level zero entry masks IPL0, unlike the priority-three debug request.
    constexpr Word mask = 0x300;
    expected.sr = (expected.sr & ~mask) | 256;
  } else {
    const auto old = expected;
    sequence_model::alu(expected, old, mac_a);
  }
  core.interrupt(host_vector, 0);
  const Word outer_cost = long_second ? 4U : 2U;
  require(core.registers() == expected && core.processing_state() == ProcessingState::running &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = outer_cost} &&
              !core.interruptible() && core.debug_status() == DebugStatus{.interrupt_mode = true},
          "Fast DEBUG cannot overwrite its outer vector or lose a deferred nonmaskable request");
  core.clear_code_cache();
  core.set_debug_interrupt_mode(false);
  expected.vba = external;
  core.set_registers(expected);
  debug_model::enter(expected);
  const auto result = core.run(1);
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              core.registers() == expected &&
              result.progress == Progress{.instructions = 3, .estimated_cycles = outer_cost + 3} &&
              core.debug_status() == DebugStatus{},
          "Deferred debug frames use the actual post-vector PC/SR and the original captured bank");
}

void interrupted_vector(const Backend backend, const bool interrupt) {
  Machine machine(backend);
  auto &core = machine.core;
  const Input input{.status = mode_rm | flag_c, .pointer = 2, .interrupt = interrupt};
  const std::array trap_vector{mac_a, mac_b};
  core.load(Space::program, start, std::array{trap, invalid});
  core.load(Space::program, 8, trap_vector);
  core.load(Space::program, debug_model::vector(input), std::array{jsr | service, Word{0}});
  auto expected = debug_model::seed(input);
  core.set_registers(expected);
  core.set_debug_interrupt_mode(interrupt);
  ++expected.pc;
  const auto old = expected;
  sequence_model::alu(expected, old, mac_a);
  require(core.run(2).dispatches == 2 && core.registers() == expected,
          "Stop exactly between two internal fast-vector instructions");
  core.request_debug();
  if (!interrupt) {
    quiescent(core, expected, {.instructions = 2, .estimated_cycles = 10}, {});
    core.resume_from_debug();
  }
  const auto before_second = expected;
  sequence_model::alu(expected, before_second, mac_b);
  const auto second = core.run(1);
  require(second.dispatches == 1 && core.registers() == expected &&
              second.progress == Progress{.instructions = 3, .estimated_cycles = 11},
          "Host debug requests preserve partially executed internal vector continuation");
  if (interrupt) {
    debug_model::enter(expected);
    require(core.run(1).dispatches == 1 && core.registers() == expected &&
                core.progress() == Progress{.instructions = 4, .estimated_cycles = 14},
            "Queued external debug follows the completed internal vector without losing effects");
  }
}

void request_during_repeat(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  core.load(Space::program, start, std::array{repeat_four, mac_a, invalid});
  const Input input{.status = flag_c, .pointer = 2};
  auto expected = debug_model::seed(input);
  core.set_registers(expected);
  const auto old = expected;
  sequence_model::alu(expected, old, mac_a);
  expected.pc = start + 1;
  expected.lc = 3;
  require(core.run(2).dispatches == 2 && core.registers() == expected, "Run one MAC REP iteration");
  core.request_debug();
  quiescent(core, expected, {.instructions = 2, .estimated_cycles = 6}, {});
  core.resume_from_debug();
  for (unsigned index = 0; index < 3; ++index) {
    const auto previous = expected;
    sequence_model::alu(expected, previous, mac_a);
  }
  expected.pc = start + 2;
  expected.lc = 7;
  const auto result = core.run(3);
  require(result.dispatches == 3 &&
              result.progress == Progress{.instructions = 5, .estimated_cycles = 9} &&
              result.fault.kind == FaultKind::none && core.registers() == expected,
          "External debug halt/resume preserves a non-debug REP body's hidden remaining count");
}

void debug_faults(const Backend backend) {
  for (const bool full_stack : {false, true}) {
    Machine machine(backend);
    auto &core = machine.core;
    const Input input{.status = flag_c,
                      .bank = full_stack ? 0U : words,
                      .pointer = full_stack ? 15U : 2U,
                      .interrupt = true};
    auto expected = debug_model::seed(input);
    core.write(Space::program, start, debug_model::debug);
    if (full_stack) {
      core.load(Space::program, debug_model::vector(input), std::array{jsr | service, Word{0}});
    }
    core.set_registers(expected);
    core.set_debug_interrupt_mode(true);
    ++expected.pc;
    require(core.run(1).dispatches == 1 && !core.interruptible(),
            "Queue debug before a failing vector");
    const auto result = core.run(1);
    const auto kind = full_stack ? FaultKind::stack : FaultKind::memory_read;
    const Word vector = debug_model::vector(input);
    const Word value = full_stack ? 15U : 0U;
    require(result.dispatches == 0 && core.registers() == expected &&
                result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                result.fault ==
                    Fault{.kind = kind, .address = vector, .value = value, .pc = vector} &&
                core.processing_state() == ProcessingState::running &&
                !core.debug_status().software_occurrence,
            "Failed debug entry cannot fabricate stack/service state, retirement or SWO");
    require(core.run(1).dispatches == 0 && core.registers() == expected &&
                rejected([&core] { core.request_debug(); }) &&
                rejected([&core] { core.resume_from_debug(); }),
            "Faulted debug entry cannot replay or be resumed through debugger APIs");
    core.reset();
    require(core.debug_status() == DebugStatus{} &&
                core.processing_state() == ProcessingState::running,
            "Library reset clears IME, SWO and queued debug work");
  }
}

class ReentryIo final : public Io {
public:
  Core *core = nullptr;
  unsigned rejections = 0;

  bool reset_peripherals(Word /*pc*/) noexcept override {
    rejections += static_cast<unsigned>(rejected([this] { core->set_debug_interrupt_mode(true); }));
    rejections += static_cast<unsigned>(rejected([this] { core->request_debug(); }));
    rejections += static_cast<unsigned>(rejected([this] { core->resume_from_debug(); }));
    return true;
  }

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
};

void callback_boundaries(const Backend backend) {
  std::array<Word, memory_words> memory{};
  ReentryIo io;
  Core core({.program = memory, .x = memory, .y = memory}, &io, {.backend = backend});
  io.core = &core;
  core.write(Space::program, start, reset);
  core.set_registers(initial_state());
  core.set_debug_interrupt_mode(true);
  const auto result = core.run(1);
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none && io.rejections == 3 &&
              core.debug_status() == DebugStatus{.interrupt_mode = true} &&
              core.processing_state() == ProcessingState::running,
          "Guest RESET preserves IME and rejects debug API reentry from callbacks");
  require(rejected([&core] { core.resume_from_debug(); }), "Running CPU cannot leave debug twice");
}

void cached_halt(const Backend backend) {
  std::array<Word, words> memory{};
  Core core({.program = memory, .x = memory, .y = memory},
            nullptr,
            {.backend = backend, .cacheable_begin = external, .cacheable_end = words});
  core.load(Space::program, external, std::array{debug_model::debug, mac_a, round_a, Word{0}});
  auto expected = debug_model::seed({.status = ce | flag_c, .pointer = 2});
  expected.pc = external;
  expected.omr |= be;
  core.set_registers(expected);
  ++expected.pc;
  require(core.run(1).state == ProcessingState::debugging,
          "Cache-enabled program enters debug halt");
  core.write(Space::program, external + 1, adc_a);
  quiescent(
      core, expected, {.instructions = 1, .estimated_cycles = 1}, {.software_occurrence = true});
  core.resume_from_debug();
  alu(expected, mac_a);
  require(
      core.run(1).dispatches == 1 && core.registers() == expected &&
          core.debug_status() == DebugStatus{},
      "Debug entry/resume and backing-RAM edits preserve architectural cached instruction words");
}

void native_matrix(const Backend backend) {
  for (unsigned form = 0; form < debug_model::forms; ++form) {
    for (const bool interrupt : {false, true}) {
      for (const auto mode : debug_model::modes) {
        for (const Word flags : {0U, status_byte}) {
          auto input = debug_model::input({.form = form, .flags = flags, .interrupt = interrupt});
          input.status = mode | flags;
          native_debug(backend, input);
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend : dsp56300::test::backends) {
      dsp56300::test::native_matrix(backend);
      for (const auto state : {dsp56300::ProcessingState::running,
                               dsp56300::ProcessingState::waiting,
                               dsp56300::ProcessingState::stopped}) {
        for (const bool interrupt : {false, true}) {
          dsp56300::test::external_debug(backend, state, interrupt);
        }
      }
      for (const bool conditional : {false, true}) {
        dsp56300::test::repeated_halts(backend, conditional);
      }
      dsp56300::test::repeat_condition_change(backend);
      dsp56300::test::repeat_interrupt(backend);
      for (const bool second : {false, true}) {
        dsp56300::test::fast_halt(backend, second);
        dsp56300::test::fast_interrupt(backend, second);
        dsp56300::test::interrupted_vector(backend, second);
      }
      dsp56300::test::request_during_repeat(backend);
      dsp56300::test::debug_faults(backend);
      dsp56300::test::callback_boundaries(backend);
      dsp56300::test::cached_halt(backend);
    }
    std::cout << dsp56300::test::checks << " debug/native/continuation checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
