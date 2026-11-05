// SPDX-License-Identifier: GPL-3.0-only
// Independent stack-coordinate and control models, including partial failures.
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native/direct.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "translation.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word do_prefix = 0x060080, relative_bit = 0x10;
constexpr Word mac_a = 0x2000d3, mac_b = 0x2000fb, io_mac = 0x44e0d3;
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16, compatibility = Word{1} << 13;
constexpr Word port = 0xffff80, datum = 0xfedcba, displacement = 0x120;
constexpr Word modulus = word_mask + 1, pointer_cycle = 64, native_capacity = 14;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

Word opcode(const Word count, const bool relative) {
  return do_prefix | ((count % 256) << 8) | (count / 256) | (relative ? relative_bit : 0U);
}

struct Shape {
  Word count = 2, mode = 0, first = start, extension = displacement;
  bool relative = false, prefix = false, outer_end = false;
};

Word setup_pc(const Shape shape) {
  return (shape.first + (shape.prefix ? 2U : 0U)) % modulus;
}

Word ending(const Shape shape) {
  return ((shape.relative ? setup_pc(shape) : 0U) + shape.extension) % modulus;
}

void arithmetic(Registers &state, const Word word) {
  sequence_model::alu(state, state, word);
  state.pc = (state.pc + 1) % modulus;
}

void prefix(Registers &state, const Shape shape) {
  if (shape.prefix) {
    arithmetic(state, mac_a);
    arithmetic(state, mac_b);
  }
}

bool push(Registers &state, const Word high, const Word low) {
  if (state.sc >= stack_entries - 1) { return false; }
  state.sp = (state.sp + 1) % 64;
  ++state.sc;
  state.stack[state.sp % stack_entries] = (std::uint64_t{high} << word_bits) | low;
  return true;
}

std::uint64_t pop(Registers &state) {
  require(state.sc != 0, "Independent model pops only a present frame");
  const auto value = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_cycle - 1) % pointer_cycle;
  --state.sc;
  return value;
}

void finish(Registers &state, const Word last) {
  if ((state.sr & lf) == 0 || last != state.la || state.pc != (state.la + 1) % modulus) { return; }
  if (state.lc != 1 || (state.sr & fv) != 0) {
    state.lc = (state.lc + word_mask) % modulus;
    state.pc = static_cast<Word>(state.stack[state.sp % stack_entries] >> word_bits);
    return;
  }
  const auto saved = pop(state);
  const auto previous = pop(state);
  state.sr = (state.sr & ~(lf | fv)) | (static_cast<Word>(saved) & (lf | fv));
  state.la = static_cast<Word>(previous >> word_bits);
  state.lc = static_cast<Word>(previous) & word_mask;
}

enum class Failure : std::uint8_t { none, first, second, mode };

Failure setup(Registers &state, const Shape shape) {
  state.pc = (setup_pc(shape) + 2) % modulus;
  if ((state.sr & compatibility) != 0) { return Failure::mode; }
  if (!push(state, state.la, state.lc)) { return Failure::first; }
  if (shape.count == 0) {
    static_cast<void>(pop(state));
    state.pc = (ending(shape) + 1) % modulus;
  } else {
    state.lc = shape.count;
    if (!push(state, state.pc, state.sr)) { return Failure::second; }
    state.la = ending(shape);
    state.sr = (state.sr & ~fv) | lf;
  }
  finish(state, (setup_pc(shape) + 1) % modulus);
  return Failure::none;
}

struct Input {
  Word sp = 2, sc = 2, flags = 0, loops = 0;
};

Registers seed(const Shape shape, const Input input) {
  auto state = register_model::seed(shape.mode);
  state.pc = shape.first;
  state.sr = shape.mode | input.flags | input.loops;
  state.sp = input.sp;
  state.sc = input.sc;
  if (shape.outer_end) { state.la = (setup_pc(shape) + 1) % modulus; }
  return state;
}

bool fast_shape(const Shape shape) {
  constexpr Word rejected = (Word{1} << 13) | (Word{1} << 14) | (Word{1} << 17) | (Word{1} << 20);
  return shape.count != 0 && !shape.outer_end && ending(shape) != (setup_pc(shape) + 1) % modulus &&
         (shape.mode & rejected) == 0 &&
         (shape.mode & (mode_down | mode_up)) != (mode_down | mode_up);
}

std::uint64_t sum(const std::uint64_t a, const std::uint64_t b) {
  const auto room = maximum - a;
  return b <= room ? a + b : b - room - 1;
}

struct Probe {
  detail::Cpu *cpu = nullptr;
  Registers expected;
  Progress progress;
  unsigned calls = 0;
  bool observed = false;
};

std::uint32_t forbidden(void *const owner, const detail::NativeOp * /*operation*/) noexcept {
  auto &probe = *static_cast<Probe *>(owner);
  ++probe.calls;
  probe.observed = probe.cpu->reg == probe.expected &&
                   probe.cpu->instructions == probe.progress.instructions &&
                   probe.cpu->cycles == probe.progress.estimated_cycles;
  return 0;
}

std::vector<detail::NativeOp> operations(const Shape shape) {
  std::vector<detail::NativeOp> result;
  if (shape.prefix) {
    result.push_back({.instruction = detail::decode(mac_a), .pc = shape.first, .mode = shape.mode});
    result.push_back({.instruction = detail::decode(mac_b),
                      .pc = (shape.first + 1) % modulus,
                      .mode = shape.mode});
  }
  auto instruction = detail::decode(opcode(shape.count, shape.relative));
  instruction.extension = shape.extension;
  require(instruction.operation == detail::Operation::loop_immediate && instruction.words == 2 &&
              instruction.cycles == 5,
          "The decoder matches the independent two-word/five-unit DO/DOR description");
  result.push_back(
      {.instruction = instruction,
       .pc = setup_pc(shape),
       .mode = shape.mode,
       .loop_address = shape.outer_end ? (setup_pc(shape) + 1) % modulus : detail::inactive_loop});
  return result;
}

void direct_case(const native::Direct &code, const Shape shape, const Input input) {
  auto expected = seed(shape, input);
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = maximum - 2;
  cpu.cycles = maximum - 3;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  prefix(expected, shape);
  const Word before = shape.prefix ? 2U : 0U;
  Probe probe{.cpu = &cpu,
              .expected = expected,
              .progress = {.instructions = sum(maximum - 2, before),
                           .estimated_cycles = sum(maximum - 3, before)}};
  probe.expected.a <<= detail::padding_bits;
  probe.expected.b <<= detail::padding_bits;
  const bool admitted = fast_shape(shape) && input.sc < native_capacity;
  if (admitted) {
    require(setup(expected, shape) == Failure::none, "Admitted model setup cannot fail");
  }
  detail::NativeContext context{.runtime = {.owner = &probe, .memory = {}}, .budget = before + 1};
  const auto retired = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(retired == before + (admitted ? 1U : 0U) && cpu.reg == expected &&
              probe.calls == (admitted ? 0U : 1U) && (admitted || probe.observed),
          "Native setup has exact frames/flags; declined state reaches the helper before effects");
  require(cpu.instructions == sum(maximum - 2, retired) &&
              cpu.cycles == sum(maximum - 3, before + (admitted ? 5U : 0U)) &&
              cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0),
          "Control retirement preserves wrapped counters and instruction-boundary exclusions");
}

void direct_shape(const Shape shape) {
  const auto ops = operations(shape);
  detail::JitEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}}, true);
  const native::Direct code(emitter.generate(ops));
  require(!emitter.packed_retirement() && !emitter.cached_status(),
          "DO setup retains conservative full-state accounting before its guarded helper exit");
  for (Word flags = 0; flags <= status_byte; ++flags) {
    direct_case(
        code, shape, {.sp = flags % 64, .flags = flags, .loops = (flags & 1U) != 0 ? lf | fv : 0U});
  }
  for (Word sp = 0; sp < 64; ++sp) {
    for (const Word sc : {0U, 1U, 12U, 13U, 14U, 15U, 16U, 31U}) {
      for (const Word loops : {0U, lf, fv, lf | fv}) {
        direct_case(code, shape, {.sp = sp, .sc = sc, .flags = status_byte, .loops = loops});
      }
    }
  }
}

void direct_matrix() {
  for (const auto mode : modes) {
    for (const Word count : {1U, 2U, 255U, 256U, 257U, 4095U}) {
      for (const bool relative : {false, true}) {
        for (const bool mixed : {false, true}) {
          for (const Word first : {start, word_mask - 3}) {
            direct_shape({.count = count,
                          .mode = mode,
                          .first = first,
                          .relative = relative,
                          .prefix = mixed});
          }
        }
      }
    }
  }
  for (const auto shape : {Shape{.count = 0},
                           Shape{.outer_end = true},
                           Shape{.extension = start + 1},
                           Shape{.extension = 1, .relative = true},
                           Shape{.extension = word_mask - 7, .relative = true}}) {
    direct_shape(shape);
  }
  for (const Word mode :
       {compatibility, Word{1} << 14, Word{1} << 17, Word{1} << 20, mode_down | mode_up}) {
    direct_shape({.mode = mode});
  }
}

struct Machine {
  explicit Machine(const Options options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

void core_case(Machine &machine, const Options options, const Shape shape, const Input input) {
  const auto initial = seed(shape, input);
  auto expected = initial;
  prefix(expected, shape);
  const auto failure = setup(expected, shape);
  const Word before = shape.prefix ? 2U : 0U;
  const Word retired = before + (failure == Failure::none ? 1U : 0U);
  machine.core.set_registers(initial);
  const auto base = machine.core.progress();
  const auto statistics = machine.core.jit_statistics();
  const auto run = machine.core.run(before + 1);
  require(run.dispatches == retired && machine.core.registers() == expected &&
              run.progress == Progress{.instructions = base.instructions + retired,
                                       .estimated_cycles = base.estimated_cycles + before +
                                                           (failure == Failure::none ? 5U : 0U)},
          "Every stack capacity preserves exact full state, partial pushes and base costs: " +
              std::string(backend_name(options.backend)) + " count=" + std::to_string(shape.count) +
              " mode=" + std::to_string(shape.mode) +
              " relative=" + std::to_string(static_cast<unsigned>(shape.relative)) +
              " sp=" + std::to_string(input.sp) + " sc=" + std::to_string(input.sc) +
              " expected_pc=" + std::to_string(expected.pc) +
              " actual_pc=" + std::to_string(machine.core.registers().pc));
  if (failure != Failure::none) {
    const auto kind = failure == Failure::mode ? FaultKind::unsupported_mode : FaultKind::stack;
    const auto value = failure == Failure::mode ? initial.sr : expected.sp;
    require(run.fault == Fault{.kind = kind,
                               .space = Space::program,
                               .address = setup_pc(shape),
                               .value = value,
                               .pc = setup_pc(shape)},
            "First/second push and mode failures retain the precise general fault");
  } else {
    require(run.fault.kind == FaultKind::none && machine.core.interruptible(),
            "A successful setup leaves an ordinary interrupt-admission boundary");
  }
  if (options.backend == Backend::jit && options.optimize) {
    const auto after = machine.core.jit_statistics();
    const auto native = before + (fast_shape(shape) && input.sc < native_capacity ? 1U : 0U);
    require(after.native_alu_instructions == statistics.native_alu_instructions + native &&
                after.native_move_instructions == statistics.native_move_instructions,
            "Actual native control execution and guarded fallback are explicitly witnessed");
  }
  if (failure != Failure::none) {
    require(machine.core.run(1).dispatches == 0 && machine.core.registers() == expected &&
                machine.core.progress() == run.progress,
            "Faults remain fail-stop until reset; no partial frame can execute again");
    machine.core.reset();
  }
}

void core_shape(const Options options, const Shape shape) {
  Machine machine(options);
  std::vector<Word> words;
  if (shape.prefix) { words = {mac_a, mac_b}; }
  words.insert(words.end(), {opcode(shape.count, shape.relative), shape.extension});
  machine.core.load(Space::program, shape.first, words);
  for (const Word sp : {0U, 15U, 30U, 31U, 62U, 63U}) {
    for (const Word sc : {0U, 12U, 13U, 14U, 15U, 16U, 31U}) {
      core_case(
          machine, options, shape, {.sp = sp, .sc = sc, .flags = status_byte, .loops = lf | fv});
    }
  }
}

void core_matrix(const Options options) {
  for (const bool relative : {false, true}) {
    for (const Word count : {0U, 1U, 2U, 256U, 4095U}) {
      for (const auto mode : modes) {
        core_shape(options, {.count = count, .mode = mode, .relative = relative, .prefix = true});
      }
    }
    core_shape(options, {.extension = start + 1, .relative = relative});
    core_shape(options, {.extension = 1, .relative = relative});
    core_shape(options, {.relative = relative, .outer_end = true});
    core_shape(options, {.count = 1, .extension = relative ? 1U : start + 1, .relative = relative});
  }
  for (const Word mode :
       {compatibility, Word{1} << 14, Word{1} << 17, Word{1} << 20, mode_down | mode_up}) {
    core_shape(options, {.mode = mode});
  }
}

void encoding_probe() {
  using detail::x64::Reg;
  using detail::x64::Width;
  detail::x64::Code code;
  constexpr std::size_t displacement_bytes = 152;
  code.store_indexed(
      {.base = Reg::r12, .index = Reg::cx, .offset = displacement_bytes}, Reg::ax, Width::qword);
  const std::vector<std::uint8_t> expected{0x49, 0x89, 0x84, 0x8c, 0x98, 0x00, 0x00, 0x00};
  require(code.bytes() == expected,
          "Indexed qword store matches independently checked LLVM MC bytes");
}

class Observer final : public Io {
public:
  Core *core = nullptr;
  Registers expected;
  Progress progress;
  Word pc = 0, patch_pc = 0, patch_value = 0;
  unsigned calls = 0;
  bool observed = false, accept = true, invalidate = false;

  bool
  read(const Space space, const Word address, const Word current, Word &value) noexcept override {
    ++calls;
    observed = core != nullptr && space == Space::x && address == port && current == pc &&
               core->registers() == expected && core->progress() == progress;
    if (observed && invalidate) { core->write(Space::program, patch_pc, patch_value); }
    value = datum;
    return observed && accept;
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

struct Callback {
  Options options;
  bool relative = false, accept = true, invalidate = false, ending = false;
};

void callback(const Callback input) {
  Observer observer;
  Machine machine(input.options, &observer);
  observer.core = &machine.core;
  observer.accept = input.accept;
  observer.invalidate = input.invalidate;
  constexpr Word io_pc = start + 4;
  const Word last = io_pc + (input.ending ? 0U : 16U);
  const Shape first{.count = 1,
                    .mode = mode_rm,
                    .extension = input.relative ? last - (start + 2) : last,
                    .relative = input.relative,
                    .prefix = true};
  Shape next{.mode = mode_rm,
             .first = io_pc + 1,
             .extension = input.relative ? 8U : io_pc + 9,
             .relative = input.relative};
  const std::array words{mac_a,
                         mac_b,
                         opcode(first.count, input.relative),
                         first.extension,
                         io_mac,
                         opcode(next.count, input.relative),
                         next.extension};
  machine.core.load(Space::program, start, words);
  auto initial = seed(first, {.flags = status_byte, .loops = lf | fv});
  initial.r[0] = 7;
  machine.x[7] = datum;
  machine.core.set_registers(initial);
  require(machine.core.run(5).dispatches == 5 && observer.calls == 0,
          "Warm both setup blocks and the RAM path before reusing callback/cleanup paths");
  initial.r[0] = port;
  machine.core.set_registers(initial);
  const auto base = machine.core.progress();
  auto expected = initial;
  prefix(expected, first);
  require(setup(expected, first) == Failure::none, "Independent first callback-loop frame fits");
  observer.expected = expected;
  ++observer.expected.pc;
  observer.progress = {.instructions = base.instructions + 3,
                       .estimated_cycles = base.estimated_cycles + 7};
  observer.pc = io_pc;
  observer.patch_pc = next.first + 1;
  observer.patch_value = input.relative ? 10U : next.first + 10;
  const auto result = machine.core.run(5);
  require(observer.observed && observer.calls == 1,
          "Callbacks see exact flags/frames/dirty A/B and both counters after native loop setup");
  if (!input.accept) {
    require(result.dispatches == 3 && result.progress == observer.progress &&
                result.fault.kind == FaultKind::io_read &&
                machine.core.registers() == observer.expected,
            "Rejected I/O preserves the complete post-setup checkpoint and cannot retire");
    return;
  }
  const auto old = expected;
  register_model::write(expected, 4, datum);
  sequence_model::alu(expected, old, io_mac);
  ++expected.pc;
  finish(expected, io_pc);
  if (input.invalidate) { next.extension = observer.patch_value; }
  require(setup(expected, next) == Failure::none, "Independent successor loop frame fits");
  require(result.dispatches == 5 && result.fault.kind == FaultKind::none &&
              machine.core.registers() == expected &&
              result.progress == Progress{.instructions = base.instructions + 5,
                                          .estimated_cycles = base.estimated_cycles + 13},
          "Nested/cleaned-up setup and successor extension invalidation preserve full state");
}

void partitions(const Options options) {
  Machine machine(options);
  constexpr Word last = start + 5;
  const Shape shape{.extension = last, .prefix = true};
  const std::array words{mac_a, mac_b, opcode(2, false), last, mac_a, mac_b};
  machine.core.load(Space::program, start, words);
  auto expected = seed(shape, {.flags = status_byte, .loops = lf | fv});
  machine.core.set_registers(expected);
  Word retired = 0;
  Word cycles = 0;
  for (const Word budget : {0U, 1U, 1U, 1U, 0U, 2U, 2U}) {
    for (Word index = 0; index < budget; ++index) {
      if (retired == 2) {
        require(setup(expected, shape) == Failure::none, "Partitioned setup pushes two frames");
        cycles += 5;
      } else {
        const auto instruction = retired < 2 ? retired : (retired - 3) % 2;
        arithmetic(expected, instruction == 0 ? mac_a : mac_b);
        finish(expected, (expected.pc + word_mask) % modulus);
        ++cycles;
      }
      ++retired;
    }
    const auto run = machine.core.run(budget);
    require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
                machine.core.registers() == expected &&
                run.progress == Progress{.instructions = retired, .estimated_cycles = cycles},
            "Zero/partial budgets continue through setup, both iterations and final restoration");
  }
}

void interrupt_boundary(const Options options) {
  Machine machine(options);
  const Shape shape{.extension = start + 3};
  const std::array words{opcode(2, false), shape.extension, mac_a, mac_b};
  machine.core.load(Space::program, start, words);
  constexpr Word vector = 32;
  machine.core.load(Space::program, vector, std::array{0U, 0U});
  auto expected = seed(shape, {.flags = status_byte, .loops = fv});
  machine.core.set_registers(expected);
  require(setup(expected, shape) == Failure::none, "Independent interrupted setup creates frames");
  require(machine.core.run(1).dispatches == 1 && machine.core.registers() == expected &&
              machine.core.interruptible(),
          "The host may admit an interrupt at the completed native setup checkpoint");
  machine.core.interrupt(vector, 1);
  const Progress interrupted{.instructions = 3, .estimated_cycles = 7};
  require(machine.core.fault().kind == FaultKind::none && machine.core.registers() == expected &&
              machine.core.progress() == interrupted && !machine.core.interruptible(),
          "A fast vector preserves loop frames and resumes with its required exclusion");
  require(machine.core.run(0).dispatches == 0 && machine.core.registers() == expected &&
              machine.core.progress() == interrupted && !machine.core.interruptible(),
          "A zero budget after eager host-admitted service cannot release its exclusion");
  arithmetic(expected, mac_a);
  require(machine.core.run(1).dispatches == 1 && machine.core.registers() == expected &&
              machine.core.interruptible(),
          "Native foreground arithmetic resumes from the loop's exact saved continuation");
}

void mutations(const Options options) {
  Machine machine(options);
  constexpr Word last = start + 10;
  const Shape first{.extension = last, .prefix = true};
  const std::array words{mac_a, mac_b, opcode(1, false), last};
  machine.core.load(Space::program, start, words);
  const auto initial = seed(first, {.flags = status_byte, .loops = lf | fv});
  for (const bool relative : {false, true}) {
    for (const Word count : {1U, 2U, 256U, 4095U, 0U, 1U}) {
      const Shape shape{
          .count = count, .extension = relative ? 8U : last, .relative = relative, .prefix = true};
      machine.core.write(Space::program, start + 2, opcode(count, relative));
      machine.core.write(Space::program, start + 3, shape.extension);
      machine.core.set_registers(initial);
      auto expected = initial;
      prefix(expected, shape);
      require(setup(expected, shape) == Failure::none, "Modified loop-count models fit the stack");
      const auto base = machine.core.progress();
      const auto run = machine.core.run(3);
      require(run.dispatches == 3 && run.fault.kind == FaultKind::none &&
                  machine.core.registers() == expected &&
                  run.progress == Progress{.instructions = base.instructions + 3,
                                           .estimated_cycles = base.estimated_cycles + 7},
              "Opcode/count and absolute/relative extension edits replace native setup constants");
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::encoding_probe();
    dsp56300::test::direct_matrix();
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::core_matrix({.backend = backend});
      for (const dsp56300::Word limit : {1U, 2U, 4U, 64U}) {
        dsp56300::test::partitions({.backend = backend, .instructions_per_block = limit});
      }
      dsp56300::test::interrupt_boundary({.backend = backend});
      dsp56300::test::mutations({.backend = backend});
      for (const bool relative : {false, true}) {
        for (const bool accept : {false, true}) {
          for (const bool invalidate : {false, true}) {
            for (const bool ending : {false, true}) {
              dsp56300::test::callback({.options = {.backend = backend},
                                        .relative = relative,
                                        .accept = accept,
                                        .invalidate = invalidate,
                                        .ending = ending});
            }
          }
        }
      }
    }
    dsp56300::test::core_matrix({.backend = dsp56300::Backend::jit, .optimize = false});
    std::cout << dsp56300::test::checks << " independent native loop-setup checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
