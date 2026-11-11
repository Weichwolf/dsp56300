// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word foreground = 0x100, service = 0x180, base = 0x20;
constexpr Word reset_vector = base + 8, self_vector = base + 12;
constexpr Word indirect_vector = base + 14;
constexpr unsigned word_bits = 24;
constexpr Word arithmetic_16 = Word{1} << 17;
constexpr Word pointer_mask = 63, return_status = (1U << 15) | (1U << 11) | 16U | 4U | 1U;
constexpr Word trap_pc = 0x110, illegal_pc = 0x120, repeat_pc = 0x130, wait_pc = 0x140;
constexpr Word move_seven = op<isa::Opcode::move_immediate_r0> | (2U << 16) | (7U << 8);
constexpr Word move_nine = op<isa::Opcode::move_immediate_r0> | (2U << 16) | (9U << 8);
constexpr Word move_service = op<isa::Opcode::move_immediate_r0> | (3U << 16) | (9U << 8);
constexpr Word short_call = op<isa::Opcode::jsr_absolute_short> | service;
constexpr Word long_call = op<isa::Opcode::jsr_effective> | (6U << 11);
constexpr Word self_write = op<isa::Opcode::movem_absolute_short> | ((self_vector + 1) << 8) | 4U;
constexpr std::array vector0{op<isa::Opcode::nop>,
                             move_seven,
                             short_call,
                             op<isa::Opcode::nop>,
                             op<isa::Opcode::nop>,
                             short_call,
                             op<isa::Opcode::nop>,
                             op<isa::Opcode::nop>,
                             op<isa::Opcode::reset>,
                             op<isa::Opcode::nop>,
                             long_call,
                             service,
                             self_write,
                             move_seven,
                             op<isa::Opcode::jsr_effective> | (3U << 11),
                             op<isa::Opcode::nop>};
constexpr auto vector1 = [] {
  auto words = vector0;
  words[self_vector - base + 1] = move_nine;
  return words;
}();
constexpr std::array continuation{
    op<isa::Opcode::nop>, op<isa::Opcode::nop>, op<isa::Opcode::nop>, op<isa::Opcode::stop>};
constexpr std::array handler{move_service, op<isa::Opcode::rti>};
constexpr std::array software_vector{short_call, op<isa::Opcode::nop>};
constexpr std::array trapping{op<isa::Opcode::trap>, op<isa::Opcode::nop>};
constexpr std::array illegal{op<isa::Opcode::illegal>, op<isa::Opcode::nop>, op<isa::Opcode::nop>};
constexpr std::array repeating{op<isa::Opcode::rep_immediate> | (2U << 8),
                               op<isa::Opcode::nop>,
                               op<isa::Opcode::nop>,
                               op<isa::Opcode::stop>};
constexpr std::array waiting{op<isa::Opcode::wait>, op<isa::Opcode::nop>, op<isa::Opcode::stop>};

auto streams(const std::span<const Word> vectors) {
  return std::array{CodeSpan{.address = 4, .words = software_vector},
                    CodeSpan{.address = 8, .words = software_vector},
                    CodeSpan{.address = base, .words = vectors},
                    CodeSpan{.address = foreground, .words = continuation},
                    CodeSpan{.address = service, .words = handler},
                    CodeSpan{.address = trap_pc, .words = trapping},
                    CodeSpan{.address = illegal_pc, .words = illegal},
                    CodeSpan{.address = repeat_pc, .words = repeating},
                    CodeSpan{.address = wait_pc, .words = waiting}};
}

const NativeImage &image() {
  static const NativeImage result = [] {
    const auto first = streams(vector0);
    const auto second = streams(vector1);
    constexpr std::array entries{foreground, service, trap_pc, illegal_pc, repeat_pc, wait_pc};
    const std::array versions{CodeVariant{.id = 0, .code = first, .entries = entries},
                              CodeVariant{.id = 1, .code = second, .entries = entries}};
    std::vector<InterruptRequest> interrupts;
    for (const Word vector : {4U,
                              8U,
                              base,
                              base + 2,
                              base + 4,
                              base + 6,
                              reset_vector,
                              base + 10,
                              self_vector,
                              indirect_vector}) {
      for (unsigned level = 0; level < 4; ++level) {
        interrupts.push_back({.vector = vector, .level = level});
      }
    }
    return Core::compile({.variants = versions},
                         {.memory = {}, .interrupts = interrupts, .transitions = {}});
  }();
  return result;
}

Registers seed(const Word status = 0) {
  auto result = test::register_model::seed(0);
  result.pc = foreground;
  result.sr = status;
  result.vba = 0;
  return result;
}

struct Machine {
  explicit Machine(Io *io = nullptr)
      : core(Memory::separate({.program = program, .x = x, .y = y}), io) {
    for (const auto &span : streams(vector0)) {
      std::ranges::copy(span.words, program.begin() + span.address);
    }
    core.install(image(), 0);
  }

  void reset(const Registers &state) {
    core.reset();
    core.set_registers(state);
  }

  std::array<Word, 512> program{};
  std::array<Word, 8> x{}, y{};
  Core core;
};

struct Input {
  Word vector = base;
  unsigned level = 0;
  Word depth = 0, status = 0;
};

struct Expected {
  Registers registers;
  Progress progress;
  Fault fault;
  bool vector = true;
  Word fetch = 0;
};

unsigned slots(const Word vector) {
  return vector == base + 2 || vector == base + 10 || vector == indirect_vector ? 1 : 2;
}

struct Entry {
  unsigned level;
  Word resume;
};

void push(Registers &state, const Entry entry) {
  std::uint64_t frame = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    frame |= std::uint64_t{(entry.resume >> bit) & 1U} << (bit + word_bits);
    frame |= std::uint64_t{(state.sr >> bit) & 1U} << bit;
  }
  state.sp = (state.sp + 1) % 64;
  ++state.sc;
  state.stack[state.sp % stack_entries] = frame;
  constexpr Word cleared = (1U << 10) | (1U << 11) | (1U << 15) | (1U << 16) | (1U << 17) | 0x300U;
  state.sr = (state.sr & ~cleared) | ((entry.level == 3 ? 3 : entry.level + 1) << 8);
  state.pc = service;
}

Expected expected(const Registers &initial, const Input input, const unsigned quota) {
  Expected result{
      .registers = initial, .progress = {}, .fault = {}, .vector = true, .fetch = input.vector};
  if (quota == 0) { return result; }
  const bool fast = input.vector == base;
  const unsigned before_call = input.vector == base + 4 ? 1 : 0;
  if (fast || (before_call != 0 && quota == 1)) {
    result.progress = {.instructions = quota, .estimated_cycles = quota};
    if (fast && quota == 2) {
      if ((initial.sr & arithmetic_16) != 0) {
        result.progress = {.instructions = 1, .estimated_cycles = 1};
        ++result.fetch;
        result.fault = {.kind = FaultKind::unsupported_mode,
                        .space = Space::program,
                        .address = result.fetch,
                        .value = initial.sr,
                        .pc = result.fetch};
        return result;
      }
      result.registers.r[2] = 7;
      result.vector = false;
      result.fetch = initial.pc;
    } else {
      ++result.fetch;
    }
    return result;
  }
  result.fetch += before_call;
  if (input.vector == indirect_vector) { ++result.registers.r[0]; }
  if (input.depth == 15) {
    result.progress = {.instructions = before_call, .estimated_cycles = before_call};
    result.fault = {.kind = FaultKind::stack,
                    .space = Space::program,
                    .address = result.fetch,
                    .value = initial.sp,
                    .pc = result.fetch};
    return result;
  }
  push(result.registers, {.level = input.level, .resume = initial.pc});
  result.progress = {.instructions = before_call + 1,
                     .estimated_cycles = before_call + (input.vector == base + 10 ? 4U : 3U)};
  result.vector = false;
  result.fetch = service;
  return result;
}

void compare(const Core &core, const Expected &value) {
  const auto actual = core.state();
  if (actual.registers != value.registers || actual.progress != value.progress ||
      actual.failure != value.fault) {
    std::cerr << "actual/expected pc=" << actual.registers.pc << '/' << value.registers.pc
              << " sr=" << actual.registers.sr << '/' << value.registers.sr
              << " r2=" << actual.registers.r[2] << '/' << value.registers.r[2]
              << " sp=" << actual.registers.sp << '/' << value.registers.sp
              << " sc=" << actual.registers.sc << '/' << value.registers.sc
              << " instructions=" << actual.progress.instructions << '/'
              << value.progress.instructions << " cycles=" << actual.progress.estimated_cycles
              << '/' << value.progress.estimated_cycles
              << " fault=" << static_cast<unsigned>(actual.failure.kind) << '/'
              << static_cast<unsigned>(value.fault.kind) << '\n';
  }
  require(
      actual.registers == value.registers && actual.progress == value.progress &&
          actual.failure == value.fault && actual.execution == ExecutionState::running,
      "Prepared vector prefixes match independent complete registers, frames, progress and faults");
  const auto fetch = core.instruction_fetch();
  require(fetch.exception_vector == value.vector && fetch.address == value.fetch,
          "Prepared entry selection follows vector fetch ownership instead of the foreground PC");
}

void prefix(Machine &machine, const Input input) {
  auto initial = seed(input.status);
  initial.sc = input.depth;
  initial.sp = pointer_mask;
  for (unsigned quota = 0; quota <= slots(input.vector); ++quota) {
    machine.reset(initial);
    machine.core.request_interrupt(input.vector, input.level);
    require(
        machine.core.registers() == initial && machine.core.progress() == Progress{} &&
            !machine.core.instruction_fetch().latched_opcode,
        "Admission changes no foreground state, retires nothing and does not fetch vector words");
    const auto result = machine.core.run(quota);
    const auto model = expected(initial, input, quota);
    compare(machine.core, model);
    require(result.retired == model.progress.instructions && result.fault == model.fault,
            "Native vector quotas account each actually retired instruction once");
  }
  machine.reset(initial);
  machine.core.interrupt(input.vector, input.level);
  compare(machine.core, expected(initial, input, slots(input.vector)));
}

void prefixes() {
  Machine machine;
  constexpr Word modes = (1U << 11) | (1U << 15) | (1U << 16);
  for (Word flags = 0; flags < 256; ++flags) {
    for (unsigned level = 0; level < 4; ++level) {
      const auto mask = flags % (level + 1);
      for (const Word depth : {0U, 14U, 15U}) {
        for (const Word vector : {base, base + 2, base + 4, base + 10, indirect_vector}) {
          prefix(
              machine,
              {.vector = vector,
               .level = level,
               .depth = depth,
               .status = flags | (mask << 8) | modes | ((flags & 1U) != 0 ? arithmetic_16 : 0U)});
        }
      }
    }
  }
}

void return_and_deadlines() {
  Machine machine;
  for (unsigned level = 0; level < 4; ++level) {
    auto initial = seed(return_status);
    machine.reset(initial);
    machine.core.request_interrupt(base + 2, level);
    const auto called = machine.core.run(1);
    auto model = expected(initial, {.vector = base + 2, .level = level}, 1);
    compare(machine.core, model);
    require(called.retired == 1 && machine.core.run(1).retired == 1,
            "Prepared ordinary service execution follows the real long-vector call");
    const auto returned = machine.core.run(1);
    initial.stack = model.registers.stack;
    initial.r[3] = 9;
    require(returned.retired == 1 && returned.fault == Fault{} &&
                machine.core.registers() == initial && !machine.core.interrupt_ready() &&
                machine.core.progress() == Progress{.instructions = 3, .estimated_cycles = 7},
            "Native RTI restores the original foreground PC/SR and preserves real stack writes");
    require(machine.core.run(1).retired == 1 && machine.core.interrupt_ready(),
            "Exactly one foreground instruction consumes the post-RTI IRQ exclusion");
  }
  for (std::uint64_t deadline = 0; deadline <= 3; ++deadline) {
    const auto initial = seed();
    machine.reset(initial);
    machine.core.request_interrupt(base, 0);
    const auto result = machine.core.run_until(deadline);
    auto wanted = initial;
    if (deadline >= 2) { wanted.r[2] = 7; }
    if (deadline == 3) { ++wanted.pc; }
    require(
        result.retired == deadline && result.fault == Fault{} &&
            machine.core.registers() == wanted &&
            machine.core.progress() ==
                Progress{.instructions = deadline, .estimated_cycles = deadline},
        "A deadline can stop before/between vector words or after the first resumed instruction");
  }
}

void latched_variant() {
  Machine machine;
  auto initial = seed();
  initial.x = move_nine;
  machine.reset(initial);
  machine.core.request_interrupt(self_vector, 0);
  const auto written = machine.core.run();
  const auto fetch = machine.core.instruction_fetch();
  require(written.retired == 1 && written.yielded && written.fault == Fault{} &&
              machine.program[self_vector + 1] == move_nine && fetch.exception_vector &&
              fetch.address == self_vector + 1 && fetch.latched_opcode == move_seven,
          "The original first vector operation changes RAM while the accepted second word stays "
          "latched");
  const auto before = machine.core.state();
  machine.core.install(image(), 1);
  require(machine.core.state().registers == before.registers &&
              machine.core.progress() == before.progress,
          "Selecting resident new code retains the actual older in-flight vector");
  const auto resumed = machine.core.run(1);
  auto wanted = initial;
  wanted.r[2] = 7;
  require(
      resumed.retired == 1 && resumed.fault == Fault{} && machine.core.registers() == wanted &&
          machine.program[self_vector + 1] == move_nine,
      "Continuation executes the prepared captured word without replacing the newly written RAM");
  static_cast<void>(machine.core.run(1));
  machine.core.request_interrupt(self_vector, 0);
  require(machine.core.run(2).retired == 2 && machine.core.registers().r[2] == 9,
          "The next admitted vector captures and executes the prepared newer resident word");
}

void internal_and_wait() {
  Machine machine;
  for (const bool invalid : {false, true}) {
    auto initial = seed();
    initial.pc = invalid ? illegal_pc : trap_pc;
    machine.reset(initial);
    const auto result = machine.core.run(2);
    auto wanted = initial;
    const auto resume = initial.pc + (invalid ? 2U : 1U);
    push(wanted, {.level = 3, .resume = resume});
    require(
        result.retired == 2 && result.fault == Fault{} && machine.core.registers() == wanted &&
            machine.core.progress() ==
                Progress{.instructions = 2, .estimated_cycles = (invalid ? 5U : 9U) + 3U},
        "Internal ILLEGAL/TRAP enter prepared vectors and keep their distinct original resume PCs");
  }
  auto initial = seed();
  initial.pc = wait_pc;
  machine.reset(initial);
  require(machine.core.step().state == ExecutionState::waiting,
          "The original WAIT enters sleep through ordinary prepared code");
  machine.core.request_interrupt(base, 0);
  const auto woke = machine.core.run(2);
  ++initial.pc;
  initial.r[2] = 7;
  require(woke.retired == 2 && woke.fault == Fault{} && machine.core.registers() == initial &&
              machine.core.progress() == Progress{.instructions = 3, .estimated_cycles = 12},
          "Admitted prepared IRQ wakes WAIT and resumes its exact following PC");
}

struct ResetBus final : Io {
  bool reset_peripherals(const Word pc) noexcept override {
    ++calls;
    observed_pc = pc;
    observed = owner->state();
    try {
      owner->request_interrupt(base, 0);
    } catch (const std::logic_error &) { reentry_refused = true; }
    if (yield) { owner->request_yield(); }
    return accept;
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

  Core *owner = nullptr;
  State observed;
  Word observed_pc = 0;
  unsigned calls = 0;
  bool yield = false, accept = true, reentry_refused = false;
};

void io_boundary(const bool synchronous, const bool accept, const bool yield) {
  ResetBus bus;
  bus.accept = accept;
  bus.yield = yield;
  Machine machine(&bus);
  bus.owner = &machine.core;
  const auto initial = seed();
  machine.reset(initial);
  require(bus.calls == 0, "Compilation and binding perform no peripheral reset or acknowledgement");
  RunResult result;
  if (synchronous) {
    machine.core.interrupt(reset_vector, 0);
  } else {
    machine.core.request_interrupt(reset_vector, 0);
    result = machine.core.run(2);
  }
  require(bus.calls == 1 && bus.observed_pc == reset_vector && bus.reentry_refused &&
              bus.observed.registers == initial && bus.observed.progress == Progress{},
          "Real vector I/O sees the vector address, foreground PC and pre-retirement progress");
  const auto actual = machine.core.state();
  if (!accept) {
    require(
        actual.registers == initial && actual.progress == Progress{} &&
            actual.failure == Fault{.kind = FaultKind::host_event,
                                    .space = Space::program,
                                    .address = reset_vector,
                                    .value = op<isa::Opcode::reset>,
                                    .pc = reset_vector},
        "Rejected peripheral reset preserves its real error and cannot retire a fictitious vector");
    static_cast<void>(machine.core.run(2));
    require(bus.calls == 1, "A faulted vector never replays a real peripheral transaction");
    return;
  }
  const bool partial = yield && !synchronous;
  require(actual.registers == initial && actual.failure == Fault{} &&
              actual.progress == Progress{.instructions = partial ? 1U : 2U,
                                          .estimated_cycles = partial ? 7U : 8U},
          "Accepted reset follows the actual synchronous or yielded vector prefix");
  if (partial) {
    require(result.yielded && machine.core.instruction_fetch().address == reset_vector + 1 &&
                machine.core.run(1).retired == 1 && bus.calls == 1,
            "A real I/O yield resumes the captured second slot without repeating the first action");
  }
  require(!machine.core.interrupt_ready(),
          "Completed fast service retains the one-instruction IRQ exclusion");
}

void admission_guards() {
  Machine machine;
  for (Word mask = 1; mask < 4; ++mask) {
    const auto initial = seed(mask << 8);
    machine.reset(initial);
    bool refused = false;
    try {
      machine.core.request_interrupt(base, mask - 1);
    } catch (const std::invalid_argument &) { refused = true; }
    require(refused && machine.core.registers() == initial && machine.core.progress() == Progress{},
            "Masked prepared vectors cannot mutate architectural state or fetch their code");
  }
  auto initial = seed();
  initial.pc = base;
  machine.reset(initial);
  machine.core.request_interrupt(base, 0);
  bool refused = false;
  try {
    static_cast<void>(
        machine.core.run_instruction({.address = base, .opcode = op<isa::Opcode::nop>}));
  } catch (const std::invalid_argument &) { refused = true; }
  require(refused && machine.core.progress() == Progress{} && machine.core.registers() == initial,
          "Coincident foreground/vector addresses cannot turn captured ordinary execution into "
          "vector execution");
}
}

int main() {
  try {
    prefixes();
    return_and_deadlines();
    latched_variant();
    internal_and_wait();
    admission_guards();
    for (const bool synchronous : {false, true}) {
      for (const bool accept : {false, true}) {
        for (const bool yield : {false, true}) { io_boundary(synchronous, accept, yield); }
      }
    }
    std::cout << test::checks << " prepared native vector checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
