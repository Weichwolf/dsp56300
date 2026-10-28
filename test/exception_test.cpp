// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "exception_model.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
using exception_model::Input;
using exception_model::Kind;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_x_a = 0x200021, round_a = 0x200011;
constexpr Word rti = 4, load_x0_post = 0x44d800, io_begin = 0xffff80, incoming = 0xab800001;
constexpr Word payload = 0x800001, nmi_vector = 10;
constexpr unsigned form_count = 18;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Machine {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core;

  explicit Machine(const Backend backend)
      : core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend}) {}
};

void alu(Registers &state, const Word operation) {
  const auto old = state;
  sequence_model::alu(state, old, operation);
}

void native_call(const Backend backend, const Input input) {
  Machine machine(backend);
  auto &core = machine.core;
  const Word signal_pc = start + 2;
  const Word resume = exception_model::resume(input, signal_pc);
  const std::array continuation{adc_x_a, round_a};
  const std::array service{adc_x_a, round_a, rti};
  constexpr Word absolute_call = 0x0bf080;
  const bool long_vector = (input.status & flag_s) != 0;
  const auto vector_cost = long_vector ? 4U : 3U;
  core.write(Space::program,
             exception_model::vector(input),
             long_vector ? absolute_call : (exception_model::call | exception_model::service));
  if (long_vector) {
    core.write(Space::program, exception_model::vector(input) + 1, exception_model::service);
  }
  core.load(Space::program, exception_model::service, service);
  const std::array prefix{
      mac_a, mac_b, exception_model::opcode(input), exception_model::invalid_canary};
  core.load(Space::program, start, prefix);
  core.load(Space::program, resume, continuation);
  auto state = exception_model::seed(input);
  auto warm = state;
  warm.pc = resume;
  warm.sr = 0;
  core.set_registers(warm);
  require(core.run(2).dispatches == 2, "Warm ordinary continuation before a software exception");
  core.set_registers(state);
  const auto progress = core.progress();
  const auto native = core.jit_statistics().native_alu_instructions;
  auto expected = state;
  alu(expected, mac_a);
  alu(expected, mac_b);
  const bool requested = exception_model::requested(input, expected.sr);
  expected.pc = signal_pc + 1;
  const auto queued = core.run(3);
  const auto request_cost = exception_model::cycles(input);
  require(
      queued.dispatches == 3 && queued.fault.kind == FaultKind::none &&
          core.registers() == expected && core.interruptible() != requested &&
          queued.progress ==
              Progress{.instructions = progress.instructions + 3,
                       .estimated_cycles = progress.estimated_cycles + 2 + request_cost},
      "The signal uses fresh native flags and stops before any vector or successor instruction");
  if (requested) {
    require(core.run(0).dispatches == 0 && core.registers() == expected && !core.interruptible(),
            "Zero budgets preserve pending exception state");
    bool rejected = false;
    try {
      core.interrupt(nmi_vector, 3);
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected && core.registers() == expected && core.progress() == queued.progress,
            "Host admission cannot replace a pending internal exception");
    if ((input.status & flag_c) != 0) { core.clear_code_cache(); }
    exception_model::enter(expected, resume);
    require(core.run(1).dispatches == 1 && core.registers() == expected && core.interruptible(),
            "One vector JSR saves the exact resume/SR frame and saturates IPL at three");
    alu(expected, adc_x_a);
    alu(expected, round_a);
    expected.pc += 2;
    require(core.run(2).dispatches == 2 && core.registers() == expected,
            "The long service executes arithmetic under the admitted service mode");
    exception_model::leave(expected);
    require(core.run(1).dispatches == 1 && core.registers() == expected && !core.interruptible(),
            "RTI restores the software exception frame and the specified resume address");
  }
  for (const auto operation : continuation) {
    alu(expected, operation);
    ++expected.pc;
  }
  const auto result = core.run(2);
  const auto count = requested ? 9U : 5U;
  const auto cost = request_cost + (requested ? 9U + vector_cost : 4U);
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
              core.registers() == expected && core.interruptible() &&
              result.progress == Progress{.instructions = progress.instructions + count,
                                          .estimated_cycles = progress.estimated_cycles + cost},
          "Continuation preserves exact DSP precision, guest retirement and base costs");
  if (backend == Backend::jit) {
    const auto expected_native = requested ? 6U : 4U;
    require(core.jit_statistics().native_alu_instructions == native + expected_native,
            "Both sides and the long handler use actual native arithmetic");
  }
}

class Observer final : public Io {
public:
  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++calls;
    value = incoming;
    valid = valid && space == Space::x && address == io_begin && pc == vector_pc &&
            core->registers() == expected && core->progress() == progress;
    try {
      bool rejected = false;
      try {
        static_cast<void>(core->run(1));
      } catch (const std::logic_error &) { rejected = true; }
      valid = valid && rejected;
      core->write(Space::program, resume, adc_x_a);
      core->write(Space::program, vector_pc + 1, mac_b);
      core->clear_code_cache();
    } catch (...) { valid = false; }
    return accept;
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

  Core *core = nullptr;
  Registers expected;
  Progress progress;
  Word resume = 0, vector_pc = 0;
  unsigned calls = 0;
  bool accept = true, valid = true;
};

void vector_callback(const Backend backend, const Input input, const bool accept) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Observer io;
  Core core({.program = p, .x = x, .y = y}, &io, {.backend = backend});
  io.core = &core;
  io.accept = accept;
  io.resume = exception_model::resume(input, start + 1);
  io.vector_pc = exception_model::vector(input);
  const std::array program{mac_a, exception_model::opcode(input)};
  core.load(Space::program, start, program);
  core.write(Space::program, io.resume + 1, round_a);
  core.write(Space::program, io.vector_pc, load_x0_post);
  auto state = exception_model::seed(input);
  state.r[0] = io_begin;
  core.set_registers(state);
  auto expected = state;
  alu(expected, mac_a);
  expected.pc = start + 2;
  require(core.run(2).dispatches == 2 && core.registers() == expected && io.calls == 0,
          "A pending fast vector has not called I/O before its next instruction budget");
  expected.pc = io.resume;
  ++expected.r[0];
  io.expected = expected;
  io.progress = {.instructions = 2, .estimated_cycles = 1 + exception_model::cycles(input)};
  const auto result = core.run(1);
  require(io.valid && io.calls == 1,
          "The vector callback sees canonical native state and rejects reentry");
  if (!accept) {
    require(result.dispatches == 0 && core.progress() == io.progress &&
                core.fault() == Fault{.kind = FaultKind::io_read,
                                      .space = Space::x,
                                      .address = io_begin,
                                      .pc = io.vector_pc} &&
                core.registers().pc == io.resume && !core.interruptible(),
            "Rejected vector I/O does not retire or execute its second vector word");
    require(core.run(4).dispatches == 0 && io.calls == 1,
            "Rejected internal exception I/O remains stopped without replay");
    return;
  }
  register_model::write(expected, 4, payload);
  require(result.dispatches == 1 && core.registers() == expected && !core.interruptible(),
          "The first vector word commits once and keeps the second word pending");
  require(core.run(0).dispatches == 0 && core.registers() == expected,
          "A zero budget cannot consume the pending second vector word");
  alu(expected, mac_b);
  require(core.run(1).dispatches == 1 && core.registers() == expected && !core.interruptible(),
          "A callback's replacement second vector word executes from fresh program memory");
  for (const auto operation : {adc_x_a, round_a}) {
    alu(expected, operation);
    ++expected.pc;
  }
  require(core.run(2).dispatches == 2 && core.registers() == expected && core.interruptible() &&
              core.progress() == Progress{.instructions = 6,
                                          .estimated_cycles = exception_model::cycles(input) + 5},
          "Fast service resumes modified native continuation with exact new input and flags");
  if (backend == Backend::jit) {
    require(core.jit_statistics().native_alu_instructions == 3,
            "The vector callback boundary has actual native arithmetic before and after service");
  }
}

void level_three(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  core.write(Space::program, nmi_vector, exception_model::call | exception_model::service);
  core.write(Space::program, exception_model::service, rti);
  auto expected =
      exception_model::seed({.status = exception_model::priority_bits | mode_up | flag_c});
  core.set_registers(expected);
  exception_model::enter(expected, start);
  core.interrupt(nmi_vector, 3);
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 3},
          "Host level-three admission works at IPL3 and never encodes priority four as scaling");
  exception_model::leave(expected);
  require(core.run(1).dispatches == 1 && core.registers() == expected &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = 6},
          "Level-three service returns the saved foreground flags/mode unchanged");
}

enum class Failure : std::uint8_t {
  missing_word,
  missing_extension,
  full_stack,
  invalid_ea,
  invalid_m,
  forbidden
};

struct FailedVector {
  Input input;
  Failure failure = Failure::missing_word;
  unsigned slot = 0;
};

void failed_vector(const Backend backend, const FailedVector item) {
  constexpr Word call_long = 0x0bf080;
  constexpr Word call_post = 0x0bd880;
  constexpr Word invalid_call = 0x0bf480;
  constexpr Word invalid_modifier = 0x8000;
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  const Word pc = exception_model::vector(item.input) + item.slot;
  std::size_t size = memory_words;
  if (item.failure == Failure::missing_word) { size = pc; }
  if (item.failure == Failure::missing_extension) { size = pc + 1; }
  Core core({.program = std::span(p).first(size), .x = x, .y = y}, nullptr, {.backend = backend});
  auto expected = exception_model::seed(item.input);
  expected.r[0] = exception_model::service;
  if (item.failure == Failure::full_stack) {
    expected.sp = static_cast<Word>(stack_entries - 1);
    expected.sc = expected.sp;
  }
  if (item.failure == Failure::invalid_m) { expected.m[0] = invalid_modifier; }
  core.set_registers(expected);
  core.write(Space::program, start, exception_model::opcode(item.input));
  Word opcode = exception_model::call | exception_model::service;
  if (item.failure == Failure::full_stack || item.failure == Failure::invalid_m) {
    opcode = call_post;
  }
  if (item.failure == Failure::missing_extension) { opcode = call_long; }
  if (item.failure == Failure::invalid_ea) { opcode = invalid_call; }
  if (item.failure == Failure::forbidden) { opcode = rti; }
  if (item.failure != Failure::missing_word) { core.write(Space::program, pc, opcode); }
  if (item.failure == Failure::invalid_ea) {
    core.write(Space::program, pc + 1, exception_model::service);
  }
  Fault fault{.kind = FaultKind::stack, .address = pc, .value = expected.sp, .pc = pc};
  if (item.failure == Failure::missing_word || item.failure == Failure::missing_extension) {
    fault.kind = FaultKind::memory_read;
    fault.address = static_cast<Word>(size);
    fault.value = 0;
  }
  if (item.failure == Failure::invalid_ea) {
    fault.kind = FaultKind::invalid_instruction;
    fault.value = opcode;
  }
  if (item.failure == Failure::invalid_m) {
    fault.kind = FaultKind::unsupported_mode;
    fault.value = invalid_modifier;
  }
  if (item.failure == Failure::forbidden) {
    fault.kind = FaultKind::unsupported_instruction;
    fault.value = opcode;
  }
  expected.pc = exception_model::resume(item.input, start);
  if (item.failure == Failure::full_stack) { ++expected.r[0]; }
  const Progress progress{.instructions = 1 + item.slot,
                          .estimated_cycles = exception_model::cycles(item.input) + item.slot};
  const auto result = core.run(6);
  require(
      result.dispatches == 1 + item.slot && result.fault == fault && result.progress == progress &&
          core.registers() == expected && !core.interruptible(),
      "Failed internal vectors retain only accepted request/slot/AGU effects, never service state");
  require(
      core.run(3).dispatches == 0 && core.registers() == expected && core.progress() == progress,
      "Failed internal vector entry is fail-stop without replaying the request or accepted slot");
}

void failed_vectors(const Backend backend) {
  constexpr std::array failures{Failure::missing_word,
                                Failure::missing_extension,
                                Failure::full_stack,
                                Failure::invalid_ea,
                                Failure::invalid_m,
                                Failure::forbidden};
  for (const auto kind : {Kind::illegal, Kind::trap}) {
    for (const auto failure : failures) {
      for (unsigned slot = 0; slot < 2; ++slot) {
        failed_vector(
            backend,
            {.input = {.kind = kind,
                       .status = mode_rm | mode_up | flag_c | exception_model::priority_bits,
                       .vba = 512,
                       .pointer = 2},
             .failure = failure,
             .slot = slot});
      }
    }
  }
}

void reset_pending(const Backend backend) {
  Machine machine(backend);
  auto &core = machine.core;
  core.write(Space::program, start, exception_model::opcode({}));
  auto state = exception_model::seed({});
  core.set_registers(state);
  require(core.run(1).dispatches == 1 && !core.interruptible(),
          "Create a pending exception for reset");
  core.reset();
  state = initial_state();
  state.pc = start + 1;
  core.set_registers(state);
  auto expected = state;
  ++expected.pc;
  require(core.run(1).dispatches == 1 && core.registers() == expected && core.interruptible() &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Reset clears hidden exception continuation without executing a stale vector");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::level_three(backend);
      dsp56300::test::reset_pending(backend);
      dsp56300::test::failed_vectors(backend);
      for (const auto mode : dsp56300::test::modes) {
        for (const dsp56300::Word flags : {0U, 1U, 128U, 255U}) {
          for (unsigned form = 0; form < dsp56300::test::form_count; ++form) {
            dsp56300::test::Input input{
                .status = mode | flags,
                .pointer = static_cast<dsp56300::Word>(flags % (dsp56300::stack_entries - 1))};
            if (form == 0) {
              input.kind = dsp56300::test::Kind::illegal;
            } else if (form >= 2) {
              input.kind = dsp56300::test::Kind::conditional;
              input.condition = form - 2;
            }
            dsp56300::test::native_call(backend, input);
          }
        }
        for (const bool accept : {false, true}) {
          dsp56300::test::vector_callback(backend, {.status = mode}, accept);
          dsp56300::test::vector_callback(
              backend, {.kind = dsp56300::test::Kind::illegal, .status = mode}, accept);
        }
      }
    }
    std::cout << dsp56300::test::checks
              << " software exception native/vector boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << dsp56300::test::checks << " checks before failure: " << error.what() << '\n';
    return 1;
  }
}
