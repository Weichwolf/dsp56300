// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "processing_model.hpp"
#include "return_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace dsp56300::test {
namespace {
using processing_model::Kind;
constexpr Word program_words = 2048, vector_pc = 64, service_pc = 768;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_a = 0x200021, round_a = 0x200011;
constexpr Word jsr = 0x0d0000, repeat_four = 0x0604a0, do_three = 0x060380;
constexpr Word ce = Word{1} << 19, be = Word{1} << 10, lf = Word{1} << 15;
constexpr Word excess_budget = 100, native_cycles = 21, wake_cycles = 22;
constexpr Word loop_retired = 20, loop_cycles = 51;
constexpr Word initial_priority = 0x654321;
constexpr std::array backends{Backend::jit};

class NoResetIo : public Io {
public:
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

struct Observation {
  Registers state;
  Progress progress;
  Word pc = 0;
  ExecutionState processing = ExecutionState::running;
};

template <typename Action> bool rejected(Action action) {
  try {
    action();
  } catch (const std::exception &) { return true; }
  return false;
}

class ResetDevice final : public NoResetIo {
public:
  TestCore *core = nullptr;
  std::array<Observation, 8> observations{};
  std::size_t calls = 0, reject_at = 0, reentry_rejections = 0;
  Word priority = initial_priority;
  bool edit = false, reentry = false, valid = true;

  bool reset_peripherals(const Word pc) noexcept override {
    try {
      if (calls < observations.size()) {
        observations[calls] = {.state = core->registers(),
                               .progress = core->progress(),
                               .pc = pc,
                               .processing = core->processing_state()};
      }
      ++calls;
      if (reentry) {
        reentry_rejections +=
            static_cast<std::size_t>(rejected([this] { static_cast<void>(core->run(1)); }));
        reentry_rejections += static_cast<std::size_t>(rejected([this] { core->reset(); }));
        reentry_rejections +=
            static_cast<std::size_t>(rejected([this] { core->set_registers(core->registers()); }));
        reentry_rejections +=
            static_cast<std::size_t>(rejected([this] { core->interrupt(vector_pc, 3); }));
        reentry_rejections +=
            static_cast<std::size_t>(rejected([this] { core->resume_from_stop(); }));
      }
      if (calls == reject_at) { return false; }
      // An explicit synthetic derivative action, never a fabricated core result.
      priority = 0;
      if (edit) {
        core->write(Space::program, start + 3, adc_a);
        core->clear_code_cache();
      }
      return true;
    } catch (const std::exception &) {
      valid = false;
      return false;
    }
  }
};

struct Machine {
  std::array<Word, program_words> p{};
  std::array<Word, 1> x{}, y{};
  ResetDevice device;
  TestCore core;

  explicit Machine(const Backend backend, const bool connected = true)
      : core({.program = p, .x = x, .y = y}, connected ? &device : nullptr, {.backend = backend}) {
    device.core = &core;
  }
};

void quiescent(TestCore &core,
               const Registers &expected,
               const Progress progress,
               const ExecutionState state) {
  for (const auto budget : {0U, std::numeric_limits<std::uint32_t>::max()}) {
    const auto result = core.run(budget);
    require(result.retired == 0 && result.fault.kind == FaultKind::none &&
                result.progress == progress && result.state == state &&
                core.processing_state() == state && core.registers() == expected,
            "Sleeping neither retires nor fetches a successor, even for maximum budgets");
  }
  core.clear_code_cache();
  core.set_registers(expected);
  require(core.processing_state() == state && core.run(1).retired == 0 &&
              core.registers() == expected && core.progress() == progress,
          "Cache clearing and visible-register initialization do not resume sleeping execution");
}

struct NativeInput {
  Word mode = 0, flags = 0;
  bool edit = false;
};

void native_reset(const Backend backend, const NativeInput input) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array code{mac_a,
                        mac_b,
                        processing_model::reset,
                        mac_a,
                        round_a,
                        processing_model::wait,
                        processing_model::invalid};
  core.load(Space::program, start, code);
  const auto initial = processing_model::seed(input.mode | input.flags);
  core.set_registers(initial);
  machine.device.edit = input.edit;
  machine.device.reentry = true;
  auto expected = initial;
  for (const Word operation : {mac_a, mac_b}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  auto callback = expected;
  ++callback.pc;
  for (const Word operation : {input.edit ? adc_a : mac_a, round_a}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
  }
  expected.pc = start + 6;
  const auto result = core.run(excess_budget);
  require(result.retired == 6 && result.state == ExecutionState::waiting &&
              result.fault.kind == FaultKind::none && core.registers() == expected &&
              result.progress == Progress{.instructions = 6, .estimated_cycles = native_cycles},
          "Native/general RESET/native/WAIT preserves exact flags, precision and budget boundary");
  const auto &observed = machine.device.observations[0];
  require(machine.device.valid && machine.device.calls == 1 && machine.device.priority == 0 &&
              machine.device.reentry_rejections == 5 && observed.state == callback &&
              observed.pc == start + 2 && observed.processing == ExecutionState::running &&
              observed.progress == Progress{.instructions = 2, .estimated_cycles = 2},
          "RESET callback sees canonical pre-retirement state and rejects all execution reentry");

  quiescent(core, expected, result.progress, ExecutionState::waiting);
  require(rejected([&core] { core.resume_from_stop(); }) &&
              core.processing_state() == ExecutionState::waiting,
          "IRQA STOP completion cannot arbitrarily resume WAIT");
}

struct WakeInput {
  Kind kind = Kind::wait;
  Word status = 0;
  unsigned level = 0;
};

void wake_interrupt(const Backend backend, const WakeInput input) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array foreground{
      processing_model::opcode(input.kind), adc_a, round_a, processing_model::invalid};
  const std::array handler{mac_a, round_a, return_model::rti};
  core.load(Space::program, start, foreground);
  core.load(Space::program, vector_pc, std::array{jsr | service_pc, Word{0}});
  core.load(Space::program, service_pc, handler);
  auto initial = processing_model::seed(input.status);
  auto warm = initial;
  warm.pc = start + 1;
  warm.sr = 0;
  core.set_registers(warm);
  require(core.run(2).retired == 2, "Warm foreground under an old numeric mode");
  core.set_registers(initial);
  auto expected = initial;
  ++expected.pc;
  const auto sleeping = core.run(10);
  require(sleeping.retired == 1 && sleeping.state == processing_model::state(input.kind) &&
              sleeping.progress == Progress{.instructions = 3, .estimated_cycles = 12} &&
              core.registers() == expected,
          "WAIT/STOP retire once before returning control to the host");
  const auto mask = (input.status / 256) % 4;
  const bool admissible = input.level >= mask;
  if (input.kind == Kind::stop || !admissible) {
    require(rejected([&] { core.interrupt(vector_pc, input.level); }) &&
                core.registers() == expected && core.progress() == sleeping.progress &&
                core.processing_state() == sleeping.state,
            "Masked WAIT IRQs and every pre-IRQA STOP IRQ leave the sleeper unchanged");
  }
  if (!admissible) { return; }
  if (input.kind == Kind::stop) {
    core.resume_from_stop();
    require(core.processing_state() == ExecutionState::running && core.registers() == expected &&
                core.progress() == sleeping.progress,
            "Host-completed IRQA stabilization resumes STOP without fabricated work");
  }
  processing_model::interrupt_entry(expected, input.level);
  core.interrupt(vector_pc, input.level);
  require(core.processing_state() == ExecutionState::running && core.registers() == expected &&
              core.progress() == Progress{.instructions = 4, .estimated_cycles = 15},
          "Admitted IRQ wakes WAIT and saves the independently bit-built continuation frame");
  for (const Word operation : {mac_a, round_a}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  return_model::apply(expected);
  for (const Word operation : {adc_a, round_a}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  const auto result = core.run(5);
  require(result.retired == 5 && result.state == ExecutionState::running &&
              result.fault.kind == FaultKind::none && core.registers() == expected &&
              result.progress == Progress{.instructions = 9, .estimated_cycles = wake_cycles},
          "Handler, RTI and warm foreground use restored Carry/scaling/RM after wake");
}

void wake_failures(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  core.write(Space::program, start, processing_model::wait);
  auto expected = processing_model::seed(mode_up | flag_c);
  core.set_registers(expected);
  ++expected.pc;
  require(core.run(1).state == ExecutionState::waiting, "Enter WAIT before failed IRQ admission");
  for (const Word vector : {3U, word_mask + 1}) {
    require(rejected([&core, vector] { core.interrupt(vector, 3); }) &&
                core.processing_state() == ExecutionState::waiting &&
                core.registers() == expected &&
                core.progress() == Progress{.instructions = 1, .estimated_cycles = 10},
            "Invalid vector metadata cannot wake or mutate a waiting CPU");
  }
  require(rejected([&core] { core.interrupt(vector_pc, 4); }) &&
              core.processing_state() == ExecutionState::waiting,
          "Invalid priorities cannot wake WAIT");
  core.interrupt(program_words, 3);
  require(core.processing_state() == ExecutionState::running && core.registers() == expected &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 10} &&
              core.fault() == Fault{.kind = FaultKind::memory_read,
                                    .address = program_words,
                                    .pc = program_words} &&
              machine.device.calls == 0,
          "Admitted wake survives a failed vector fetch without fake frame or retirement");
  const auto result = core.run(1);
  require(result.retired == 0 && result.state == ExecutionState::running &&
              result.fault == core.fault() && core.registers() == expected,
          "A failed wake remains fail-stopped without reentering WAIT or replaying the vector");
}

void stop_foreground(const Backend backend, const Word mask) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array code{processing_model::stop, adc_a, round_a, processing_model::invalid};
  core.load(Space::program, start, code);
  auto expected = processing_model::seed(mode_rm | mode_up | flag_c | (mask * 256));
  core.set_registers(expected);
  ++expected.pc;
  const auto stopped = core.run(10);
  require(stopped.retired == 1 && stopped.state == ExecutionState::stopped &&
              core.registers() == expected && !core.interrupt_ready(),
          "STOP records its own processing state instead of a generic WAIT");
  quiescent(core, expected, stopped.progress, ExecutionState::stopped);
  core.resume_from_stop();
  for (const auto operation : {adc_a, round_a}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  const auto result = core.run(2);
  require(result.retired == 2 && result.fault.kind == FaultKind::none &&
              result.state == ExecutionState::running && core.registers() == expected &&
              result.progress == Progress{.instructions = 3, .estimated_cycles = 12},
          "IRQA wake without an admitted IRQ continues at the instruction following STOP");
  require(rejected([&core] { core.resume_from_stop(); }),
          "A running core cannot complete STOP twice");
}

void reset_sleepers(const Backend backend) {
  for (const auto kind : {Kind::wait, Kind::stop}) {
    Machine machine(backend);
    auto &core = machine.core;
    core.write(Space::program, start, processing_model::opcode(kind));
    core.set_registers(processing_model::seed(mode_down | flag_c));
    require(core.run(1).state == processing_model::state(kind), "Enter a sleeper before reset");
    core.reset();
    Registers initial;
    initial.m.fill(word_mask);
    const auto result = core.run(0);
    require(result.state == ExecutionState::running && result.retired == 0 &&
                result.progress == Progress{} && result.fault.kind == FaultKind::none &&
                core.registers() == initial && machine.device.calls == 0,
            "Library reset cancels sleep without issuing a guest peripheral-reset request");
  }
}

void reset_failures(const Backend backend) {
  for (const Word status : {0U, status_byte, mode_down | flag_c, word_mask}) {
    for (const bool connected : {false, true}) {
      Machine machine(backend, connected);
      auto &core = machine.core;
      machine.device.reject_at = 1;
      core.write(Space::program, start, processing_model::reset);
      const auto input = processing_model::seed(status);
      core.set_registers(input);
      auto expected = input;
      ++expected.pc;
      const auto result = core.run(4);
      require(result.retired == 0 && result.state == ExecutionState::running &&
                  result.progress == Progress{} && core.registers() == expected &&
                  result.fault == Fault{.kind = FaultKind::host_event,
                                        .address = start,
                                        .value = processing_model::reset,
                                        .pc = start} &&
                  machine.device.calls == static_cast<std::size_t>(connected) &&
                  machine.device.priority == initial_priority,
              "Absent/rejected peripheral reset never fabricates CPU reset or retirement");
      require(core.run(1).retired == 0 && core.registers() == expected &&
                  machine.device.calls == static_cast<std::size_t>(connected),
              "Failed RESET stays stopped without replaying an external action");
      require(rejected([&core] { core.resume_from_stop(); }),
              "Event faults cannot be resumed as STOP");
    }
  }
  std::array<Word, memory_words> memory{};
  NoResetIo device;
  TestCore core({.program = memory, .x = memory, .y = memory}, &device, {.backend = backend});
  core.write(Space::program, start, processing_model::reset);
  core.set_registers(initial_state());
  require(core.run(1).fault.kind == FaultKind::host_event,
          "Existing Io implementations explicitly decline RESET by default");
}

void repeated_reset(const Backend backend, const std::size_t reject_at) {
  Machine machine(backend);
  auto &core = machine.core;
  machine.device.reject_at = reject_at;
  core.load(Space::program,
            start,
            std::array{repeat_four, processing_model::reset, Word{0}, processing_model::invalid});
  const auto input = processing_model::seed(mode_rm | flag_s | flag_c);
  core.set_registers(input);
  auto expected = input;
  const Word accepted = reject_at == 0 ? 4U : static_cast<Word>(reject_at - 1);
  expected.pc = start + (reject_at == 0 ? 3U : 2U);
  expected.lc = reject_at == 0 ? input.lc : 4 - accepted;
  const auto result = core.run(6);
  const Word retired = reject_at == 0 ? 6U : 1 + accepted;
  const Word cost = 5 + (accepted * 7) + (reject_at == 0 ? 1U : 0U);
  require(result.retired == retired && result.state == ExecutionState::running &&
              result.progress == Progress{.instructions = retired, .estimated_cycles = cost} &&
              core.registers() == expected &&
              machine.device.calls == (reject_at == 0 ? 4 : reject_at),
          "REP RESET performs exactly its accepted actions and preserves LC on rejection");
  require(result.fault.kind == (reject_at == 0 ? FaultKind::none : FaultKind::host_event),
          "Repeated peripheral reset has an explicit acceptance/failure boundary");
  for (std::size_t index = 0; index < machine.device.calls; ++index) {
    auto observed = input;
    observed.pc = start + 2;
    observed.lc = 4 - static_cast<Word>(index);
    const auto &actual = machine.device.observations[index];
    require(actual.pc == start + 1 && actual.state == observed &&
                actual.progress ==
                    Progress{.instructions = 1 + index, .estimated_cycles = 5 + (index * 7)},
            "Each REP RESET callback has the exact old LC and pre-retirement progress");
  }
}

struct FrameWords {
  Word high = 0, low = 0;
};

std::uint64_t frame(const FrameWords words) {
  std::uint64_t result = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    if ((words.high >> bit) % 2 != 0) { result += std::uint64_t{1} << (word_bits + bit); }
    if ((words.low >> bit) % 2 != 0) { result += std::uint64_t{1} << bit; }
  }
  return result;
}

void loop_reset(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  const std::array code{do_three,
                        start + 7,
                        processing_model::reset,
                        Word{0},
                        Word{0},
                        Word{0},
                        Word{0},
                        Word{0},
                        processing_model::wait,
                        processing_model::invalid};
  core.load(Space::program, start, code);
  const auto input = processing_model::seed(flag_s | flag_c);
  core.set_registers(input);
  auto expected = input;
  expected.pc = start + 9;
  expected.stack[input.sp + 1] = frame({.high = input.la, .low = input.lc});
  expected.stack[input.sp + 2] = frame({.high = start + 2, .low = input.sr});
  const auto result = core.run(excess_budget);
  require(result.retired == loop_retired && result.state == ExecutionState::waiting &&
              result.fault.kind == FaultKind::none && core.registers() == expected &&
              result.progress ==
                  Progress{.instructions = loop_retired, .estimated_cycles = loop_cycles} &&
              machine.device.calls == 3,
          "RESET inside a legal finite DO preserves frames and restores the outer loop state");
  for (std::size_t index = 0; index < 3; ++index) {
    auto callback = input;
    callback.pc = start + 3;
    callback.la = start + 7;
    callback.lc = 3 - static_cast<Word>(index);
    callback.sp += 2;
    callback.sc += 2;
    callback.sr |= lf;
    callback.stack = expected.stack;
    const auto &observed = machine.device.observations[index];
    require(observed.pc == start + 2 && observed.state == callback &&
                observed.progress ==
                    Progress{.instructions = 1 + (index * 6), .estimated_cycles = 5 + (index * 12)},
            "Loop RESET callbacks observe current LC, untouched frames and nominal costs");
  }
}

struct FastInput {
  Word first = 0, second = 0;
  std::size_t reject_at = 0;
};

void fast_reset(const Backend backend, const FastInput input) {
  Machine machine(backend);
  auto &core = machine.core;
  machine.device.reject_at = input.reject_at;
  core.load(Space::program, vector_pc, std::array{input.first, input.second});
  const auto initial = processing_model::seed(mode_rm | mode_up | flag_s | flag_c);
  core.set_registers(initial);
  const std::array words{input.first, input.second};
  auto expected = initial;
  Progress progress;
  std::size_t calls = 0;
  for (const Word operation : words) {
    if (operation == processing_model::reset) {
      ++calls;
      if (calls == input.reject_at) { break; }
    }
    ++progress.instructions;
    progress.estimated_cycles += operation == processing_model::reset ? 7U : 1U;
  }
  core.interrupt(vector_pc, 0);
  const bool failed = input.reject_at != 0;
  require(core.processing_state() == ExecutionState::running && core.registers() == expected &&
              core.progress() == progress && machine.device.calls == calls,
          "Fast RESET preserves the complete foreground state and counts only accepted actions");
  require(core.fault().kind == (failed ? FaultKind::host_event : FaultKind::none),
          "A rejected fast RESET is an explicit host-event error");
  for (std::size_t index = 0; index < calls; ++index) {
    const auto &observed = machine.device.observations[index];
    const Word slot = input.first == processing_model::reset ? static_cast<Word>(index) : 1U;
    Word cost = 0;
    if (slot != 0) { cost = input.first == processing_model::reset ? 7U : 1U; }
    require(observed.pc == vector_pc + slot && observed.state == initial &&
                observed.progress == Progress{.instructions = slot, .estimated_cycles = cost},
            "Fast RESET callback PC identifies its vector while register PC stays foreground");
  }
  if (failed) {
    require(core.run(2).retired == 0 && core.progress() == progress &&
                machine.device.calls == calls,
            "A rejected vector RESET cannot replay an accepted or failed peripheral action");
  } else {
    require(!core.interrupt_ready() && core.run(0).retired == 0,
            "Fast RESET retains the ordinary post-vector exclusion over zero budgets");
    ++expected.pc;
    ++progress.instructions;
    ++progress.estimated_cycles;
    require(core.run(1).retired == 1 && core.registers() == expected &&
                core.progress() == progress && core.interrupt_ready(),
            "A completed fast RESET service resumes foreground execution exactly once");
  }
}

void reset_long_vector(const Backend backend, const WakeInput input) {
  Machine machine(backend);
  auto &core = machine.core;
  core.load(Space::program, vector_pc, std::array{processing_model::reset, jsr | service_pc});
  const auto initial = processing_model::seed(input.status);
  core.set_registers(initial);
  auto expected = initial;
  processing_model::interrupt_entry(expected, input.level);
  core.interrupt(vector_pc, input.level);
  require(core.fault().kind == FaultKind::none &&
              core.processing_state() == ExecutionState::running && core.registers() == expected &&
              machine.device.calls == 1 &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = 10},
          "Accepted vector RESET preserves old state for the subsequent long IRQ frame");
  const auto &observed = machine.device.observations[0];
  require(observed.state == initial && observed.pc == vector_pc && observed.progress == Progress{},
          "Vector reset callback precedes long service state and frame changes");
}

void fast_restrictions(const Backend backend) {
  for (const auto kind : {Kind::wait, Kind::stop}) {
    for (Word slot = 0; slot < 2; ++slot) {
      Machine machine(backend);
      auto &core = machine.core;
      std::array<Word, 2> code{};
      code[slot] = processing_model::opcode(kind);
      core.load(Space::program, vector_pc, code);
      const auto initial = processing_model::seed(flag_c);
      core.set_registers(initial);
      core.interrupt(vector_pc, 0);
      require(core.processing_state() == ExecutionState::running && core.registers() == initial &&
                  core.progress() == Progress{.instructions = slot, .estimated_cycles = slot} &&
                  core.fault() == Fault{.kind = FaultKind::unsupported_instruction,
                                        .address = vector_pc + slot,
                                        .value = processing_model::opcode(kind),
                                        .pc = vector_pc + slot},
              "Forbidden fast WAIT/STOP cannot sleep, retire or fabricate a continuation");
    }
  }
  Machine machine(backend);
  auto &core = machine.core;
  core.load(Space::program, vector_pc, std::array{processing_model::reset, jsr | service_pc});
  auto initial = processing_model::seed(flag_c);
  initial.sp = initial.sc = 15;
  core.set_registers(initial);
  core.interrupt(vector_pc, 0);
  require(
      core.fault().kind == FaultKind::stack && core.registers() == initial &&
          core.processing_state() == ExecutionState::running &&
          core.progress() == Progress{.instructions = 1, .estimated_cycles = 7} &&
          machine.device.calls == 1 && machine.device.priority == 0,
      "A failed later vector push retains an already accepted RESET without fake service state");
}

void cache_sleep(const Backend backend, const Kind kind) {
  std::array<Word, program_words> memory{};
  ResetDevice device;
  constexpr Word external = 1024;
  TestCore core({.program = memory, .x = memory, .y = memory},
                &device,
                {.backend = backend, .cacheable_begin = external, .cacheable_end = program_words});
  device.core = &core;
  const std::array code{processing_model::opcode(kind), mac_a, round_a, Word{0}};
  core.load(Space::program, external, code);
  auto expected = processing_model::seed(ce | flag_c);
  expected.pc = external;
  expected.omr |= be;
  expected.x = 3;
  expected.y = 5;
  core.set_registers(expected);
  ++expected.pc;
  const auto asleep = core.run(10);
  require(asleep.retired == 1 && asleep.state == processing_model::state(kind),
          "Burst-enabled external program enters its explicit sleeping state");
  core.write(Space::program, external + 1, adc_a);
  quiescent(core, expected, asleep.progress, asleep.state);
  if (kind == Kind::stop) {
    core.resume_from_stop();
  } else {
    core.interrupt(vector_pc, 0); // Two internal NOP vector words leave the saved CCR unchanged.
  }
  const auto old = expected;
  sequence_model::alu(expected, old, mac_a);
  ++expected.pc;
  require(core.run(1).fault.kind == FaultKind::none && core.registers() == expected &&
              core.processing_state() == ExecutionState::running && device.calls == 0,
          "Sleep/wake and host P edits preserve already cached instruction data");
}

void software_reset_preserves_cache(const Backend backend) {
  std::array<Word, program_words> memory{};
  ResetDevice device;
  constexpr Word external = 1024;
  TestCore core({.program = memory, .x = memory, .y = memory},
                &device,
                {.backend = backend, .cacheable_begin = external, .cacheable_end = program_words});
  device.core = &core;
  auto input = processing_model::seed(ce | flag_c);
  input.pc = external;
  input.x = 3;
  input.y = 5;
  core.set_registers(input);
  require(core.run(1).retired == 1, "Prime a valid cached NOP before guest RESET");
  core.write(Space::program, external, mac_a);
  core.write(Space::program, start, processing_model::reset);
  input.pc = start;
  core.set_registers(input);
  require(core.run(1).retired == 1 && device.calls == 1,
          "Guest RESET is explicitly accepted by the synthetic derivative");
  input.pc = external;
  core.set_registers(input);
  auto expected = input;
  ++expected.pc;
  require(core.run(1).retired == 1 && core.registers() == expected,
          "Guest peripheral RESET preserves CE, cached instructions and CPU numeric state");
}

void wake_matrix(const Backend backend) {
  for (const auto kind : {Kind::wait, Kind::stop}) {
    for (const auto mode : processing_model::modes) {
      for (const Word flags : {0U, status_byte}) {
        for (Word mask = 0; mask < 4; ++mask) {
          for (unsigned level = 0; level < 4; ++level) {
            wake_interrupt(backend,
                           {.kind = kind, .status = mode | flags | (mask * 256), .level = level});
          }
        }
      }
    }
  }
}

void native_matrix(const Backend backend) {
  for (const auto mode : processing_model::modes) {
    for (const auto flags : {0U, flag_c, status_byte}) {
      for (const bool edit : {false, true}) {
        native_reset(backend, {.mode = mode, .flags = flags, .edit = edit});
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
      dsp56300::test::wake_matrix(backend);
      dsp56300::test::wake_failures(backend);
      for (dsp56300::Word mask = 0; mask < 4; ++mask) {
        dsp56300::test::stop_foreground(backend, mask);
      }
      dsp56300::test::reset_sleepers(backend);
      dsp56300::test::reset_failures(backend);
      for (const std::size_t reject_at : {0U, 1U, 3U}) {
        dsp56300::test::repeated_reset(backend, reject_at);
      }
      dsp56300::test::loop_reset(backend);
      for (const auto input :
           {dsp56300::test::FastInput{.second = dsp56300::test::processing_model::reset},
            dsp56300::test::FastInput{.first = dsp56300::test::processing_model::reset},
            dsp56300::test::FastInput{.first = dsp56300::test::processing_model::reset,
                                      .second = dsp56300::test::processing_model::reset}}) {
        dsp56300::test::fast_reset(backend, input);
      }
      for (const std::size_t failure : {1U, 2U}) {
        dsp56300::test::fast_reset(backend,
                                   {.first = dsp56300::test::processing_model::reset,
                                    .second = dsp56300::test::processing_model::reset,
                                    .reject_at = failure});
      }
      for (const auto mode : dsp56300::test::processing_model::modes) {
        for (const unsigned level : {0U, 3U}) {
          dsp56300::test::reset_long_vector(
              backend, {.status = mode | dsp56300::test::flag_c, .level = level});
        }
      }
      dsp56300::test::fast_restrictions(backend);
      for (const auto kind : {dsp56300::test::Kind::wait, dsp56300::test::Kind::stop}) {
        dsp56300::test::cache_sleep(backend, kind);
      }
      dsp56300::test::software_reset_preserves_cache(backend);
    }
    std::cout << dsp56300::test::checks << " processing/native/host boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
