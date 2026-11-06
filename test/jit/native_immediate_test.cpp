// SPDX-License-Identifier: GPL-3.0-only
// Independently specified R/N assignments and complete native observation boundaries.
#include "architecture.hpp"
#include "core_access.hpp"
#include "dataflow.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/emitter.hpp"
#include "memory_arena.hpp"
#include "native/direct.hpp"
#include "native_moves.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "translation.hpp"

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
constexpr Word mac_a = 0x2000d3, mac_b = 0x2000fb, io_mac = 0x44e0d3;
constexpr Word loop_flag = Word{1} << 15, port_address = 0xffff80, datum = 0xfedcba;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr Word immediate_prefix = 0x40f400, raw_datum = 0xfffedcba, do_three = 0x060380;
constexpr unsigned high_selector = 24, high_selector_shift = 17, r7 = 23, n0 = 24, n7 = 31;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array values{0U, 1U, sign24 - 1, sign24, word_mask, 0xfffedcbaU};

Word encoding(const unsigned selector, const bool y) {
  return immediate_prefix | ((selector & 7U) << 16) |
         ((selector & high_selector) << high_selector_shift) | (y ? 0x080000U : 0U);
}

struct Step {
  Word word = 0, value = 0;
  unsigned selector = 0;
};

Step assignment(const unsigned selector, const Word value, const bool y = false) {
  return {.word = encoding(selector, y), .value = value, .selector = selector};
}

Word length(const Step step) {
  return step.selector != 0 ? 2U : 1U;
}

void model(Registers &state, const Step step) {
  if (step.selector != 0) {
    register_model::write(state, step.selector, step.value & word_mask);
  } else {
    sequence_model::alu(state, state, step.word);
  }
  state.pc = (state.pc + length(step)) & word_mask;
}

std::uint64_t sum(const std::uint64_t a, const std::uint64_t b) {
  const auto room = maximum - a;
  return b <= room ? a + b : b - room - 1;
}

std::vector<Word> program(const std::span<const Step> steps) {
  std::vector<Word> result;
  for (const auto step : steps) {
    result.push_back(step.word);
    if (step.selector != 0) { result.push_back(step.value); }
  }
  return result;
}

void admission() {
  for (unsigned selector = 0; selector < 32; ++selector) {
    for (const bool y : {false, true}) {
      auto decoded = detail::decode(encoding(selector, y));
      decoded.extension = raw_datum;
      const auto planned = detail::plan_address_immediate(decoded);
      const bool admitted = selector >= 16;
      require(planned.valid() == admitted, "Only R/N selectors enter the address constant path");
      if (admitted) {
        require(decoded.words == 2 && decoded.cycles == 2 && planned.destination == selector &&
                    planned.value == datum,
                "Two-word assignments retain explicit base cost, selector and low24 constant");
        require(detail::analyze_dataflow(decoded).move_writes == 0,
                "R/N constants cannot discard cached data banks");
      }
      for (const Word changed : {0x80U, 0x4000U, 0x8000U, 0x100U}) {
        auto invalid = decoded;
        invalid.opcode ^= changed;
        require(!detail::plan_address_immediate(invalid).valid(),
                "Changed ALU/load/effective/index fields retain their existing execution paths");
      }
      decoded.words = 1;
      require(!detail::plan_address_immediate(decoded).valid(), "Missing extensions are rejected");
    }
  }
  auto decoded = detail::decode(encoding(16, false));
  decoded.operation = detail::Operation::nop;
  require(!detail::plan_address_immediate(decoded).valid(),
          "Decoded operation remains authoritative");
}

struct Probe {
  unsigned calls = 0;
};

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++static_cast<Probe *>(owner)->calls;
  return 0;
}

struct Shape {
  unsigned selector = 16;
  Word value = 0, mode = 0, first = start;
  bool y = false, mixed = false, loop = false;
};

std::vector<Step> body(const Shape input) {
  const auto move = assignment(input.selector, input.value, input.y);
  if (!input.mixed) { return {move}; }
  return {{.word = mac_a}, {.word = mac_b}, {.word = mac_a}, move};
}

std::vector<detail::BlockInstruction> operations(const Shape input,
                                                 const std::span<const Step> steps) {
  auto ending = input.first;
  for (const auto step : steps) { ending += length(step); }
  ending = (ending - 1) & word_mask;
  std::vector<detail::BlockInstruction> result;
  Word cursor = input.first;
  for (const auto step : steps) {
    auto instruction = detail::decode(step.word);
    instruction.extension = step.value;
    result.push_back({.instruction = instruction,
                      .pc = cursor,
                      .mode = input.mode,
                      .loop_address = input.loop ? ending : detail::inactive_loop});
    cursor = (cursor + length(step)) & word_mask;
  }
  return result;
}

struct Invocation {
  Word flags = 0, lc = 3, budget = 4;
  bool different_target = false;
};

void finish_loop(Registers &state) {
  constexpr Word loop_bits = loop_flag | (1U << 16);
  constexpr Word pointer_mask = 63;
  const auto saved = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  const auto previous = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  state.sc -= 2;
  state.sr = (state.sr & ~loop_bits) | (static_cast<Word>(saved) & loop_bits);
  state.la = static_cast<Word>(previous >> word_bits);
  state.lc = static_cast<Word>(previous) & word_mask;
}

void direct_case(const native::Direct &code, const Shape shape, const Invocation input) {
  const auto steps = body(shape);
  auto expected = register_model::seed(shape.mode);
  expected.sr = shape.mode | input.flags | (shape.loop ? loop_flag : 0);
  expected.pc = shape.first;
  expected.lc = input.lc;
  expected.la = (shape.first + 4) & word_mask;
  const auto target = (shape.first + (input.different_target ? 7U : 0U)) & word_mask;
  expected.stack[expected.sp % stack_entries] = (std::uint64_t{target} << word_bits) | 7;
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = maximum - 2;
  cpu.cycles = maximum - 3;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  Word retired = 0;
  Word cycles = 0;
  bool repeat = true;
  while (repeat && input.budget - retired >= steps.size()) {
    repeat = false;
    for (std::size_t index = 0; index < steps.size(); ++index) {
      const bool ending = shape.loop && index + 1 == steps.size();
      if (ending && (expected.lc == 0 || (expected.lc == 1 && expected.sc < 2))) { break; }
      model(expected, steps[index]);
      ++retired;
      cycles += length(steps[index]);
      if (ending) {
        if (expected.lc == 1) {
          finish_loop(expected);
        } else {
          --expected.lc;
          expected.pc = target;
          repeat = target == shape.first;
        }
      }
    }
  }
  Probe probe;
  detail::NativeContext context{.runtime = {.owner = &probe, .memory = {}}, .budget = input.budget};
  const auto actual_retired = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(
      actual_retired == retired && cpu.reg == expected && probe.calls == 0 &&
          cpu.instructions == sum(maximum - 2, retired) && cpu.cycles == sum(maximum - 3, cycles),
      "Helper-free constants preserve every register, dirty accumulators, PC and wrapped costs");
  require(!cpu.repeat_completed && !cpu.interrupt_inhibited,
          "Completed native prefixes retain instruction-boundary exclusion semantics");
}

void direct_shape(const Shape shape) {
  const auto steps = body(shape);
  const auto ops = operations(shape, steps);
  detail::BlockEmitter emitter(
      {.owner = nullptr, .execute = forbidden, .moves = forbidden, .memory = {}});
  const native::Direct code(emitter.generate(ops));
  require(emitter.packed_retirement() == shape.mixed && emitter.cached_status() == shape.mixed,
          "Long pure regions including R/N constants retain packed retirement and status holding");
  for (Word flags = 0; flags <= status_byte; ++flags) {
    direct_case(code, shape, {.flags = flags});
  }
  if (shape.loop) {
    for (const Word lc : {0U, 1U, 2U, 7U}) {
      for (const Word budget : {4U, 8U, 13U}) {
        for (const bool other : {false, true}) {
          direct_case(
              code,
              shape,
              {.flags = status_byte, .lc = lc, .budget = budget, .different_target = other});
        }
      }
    }
  }
}

void direct_matrix() {
  for (unsigned selector = 16; selector < 32; ++selector) {
    for (const bool y : {false, true}) {
      for (const auto mode : modes) {
        for (const auto value : values) {
          for (const Word first : {start, word_mask}) {
            direct_shape(
                {.selector = selector, .value = value, .mode = mode, .first = first, .y = y});
          }
        }
        for (const bool loop : {false, true}) {
          direct_shape({.selector = selector,
                        .value = datum,
                        .mode = mode,
                        .first = word_mask - 2,
                        .y = y,
                        .mixed = true,
                        .loop = loop});
        }
      }
    }
  }
}

struct Machine {
  explicit Machine(const TestOptions options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  TestCore core;
};

void prefixes(const TestOptions options) {
  for (const auto mode : modes) {
    Machine machine(options);
    const std::array steps{Step{.word = mac_a},
                           Step{.word = mac_b},
                           assignment(18, word_mask),
                           assignment(26, 0x800001, true),
                           Step{.word = mac_b},
                           assignment(23, datum),
                           Step{.word = mac_a},
                           assignment(31, 1, true)};
    const auto words = program(steps);
    machine.core.load(Space::program, start, words);
    const auto initial = register_model::seed(mode);
    machine.core.set_registers(initial);
    require(machine.core.run(static_cast<Word>(steps.size())).retired == steps.size(),
            "Warm mixed assignment/arithmetic code before partition checks");
    for (Word budget = 0; budget <= steps.size(); ++budget) {
      auto expected = initial;
      Word cycles = 0;
      for (const auto step : std::span(steps).first(budget)) {
        model(expected, step);
        cycles += length(step);
      }
      machine.core.set_registers(initial);
      const auto base = machine.core.progress();
      const auto run = machine.core.run(budget);
      require(run.retired == budget && run.fault.kind == FaultKind::none &&
                  machine.core.registers() == expected &&
                  run.progress == Progress{.instructions = base.instructions + budget,
                                           .estimated_cycles = base.estimated_cycles + cycles},
              "Every warm instruction prefix preserves full state and exact two-word costs");
    }
    machine.core.set_registers(initial);
    auto expected = initial;
    std::size_t offset = 0;
    for (const Word budget : {1U, 3U, 2U, 2U}) {
      for (const auto step : std::span(steps).subspan(offset, budget)) { model(expected, step); }
      require(machine.core.run(budget).retired == budget && machine.core.registers() == expected,
              "Partitioned assignments resume with canonical R/N and live arithmetic");
      offset += budget;
    }
  }
}

class Observer final : public Io {
public:
  TestCore *core = nullptr;
  Registers expected;
  Progress progress;
  Word pc = 0, extension = 0;
  unsigned calls = 0;
  bool observed = false, accept = true, invalidate = false;

  bool read(const Space space, const Word address, const Word at, Word &value) noexcept override {
    ++calls;
    observed = core != nullptr && space == Space::x && address == port_address && at == pc &&
               core->registers() == expected && core->progress() == progress;
    if (observed && invalidate) { core->write(Space::program, extension, datum); }
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

void callback(const TestOptions options, const bool accept, const bool invalidate) {
  Observer observer;
  Machine machine(options, &observer);
  observer.core = &machine.core;
  observer.accept = accept;
  observer.invalidate = invalidate;
  const std::array prefix{Step{.word = mac_a},
                          Step{.word = mac_b},
                          assignment(24, 1),
                          assignment(16, port_address, true)};
  const std::array suffix{assignment(25, 0x123456, true), Step{.word = mac_b}};
  auto words = program(prefix);
  words.push_back(io_mac);
  const auto following = program(suffix);
  words.insert(words.end(), following.begin(), following.end());
  machine.core.load(Space::program, start, words);
  const auto initial = register_model::seed(mode_rm | flag_l);
  auto expected = initial;
  for (const auto step : prefix) { model(expected, step); }
  observer.pc = expected.pc;
  observer.extension = expected.pc + 2;
  observer.expected = expected;
  ++observer.expected.pc;
  observer.progress = {.instructions = prefix.size(), .estimated_cycles = 6};
  machine.core.set_registers(initial);
  const auto run = machine.core.run(7);
  require(observer.observed && observer.calls == 1,
          "I/O sees canonical dirty A/B, R/N, flags, PC and both retirement lanes");
  if (!accept) {
    require(run.retired == prefix.size() && run.progress == observer.progress &&
                run.fault.kind == FaultKind::io_read &&
                machine.core.registers() == observer.expected,
            "Rejected I/O retains the exact pre-transfer checkpoint after packed constants");
    return;
  }
  const auto old = expected;
  register_model::write(expected, 4, datum);
  sequence_model::alu(expected, old, io_mac);
  ++expected.pc;
  for (auto step : suffix) {
    if (step.selector != 0 && invalidate) { step.value = datum; }
    model(expected, step);
  }
  require(run.retired == 7 && run.fault.kind == FaultKind::none &&
              machine.core.registers() == expected &&
              run.progress == Progress{.instructions = 7, .estimated_cycles = 10},
          "Successor extension edits invalidate embedded constants before their execution");
}

void extension_and_loops(const TestOptions options) {
  Machine machine(options);
  auto initial = register_model::seed(mode_rm);
  const std::array instruction{encoding(23, true), 1U};
  machine.core.load(Space::program, start, instruction);
  for (const auto value : values) {
    machine.core.write(Space::program, start + 1, value & word_mask);
    machine.core.set_registers(initial);
    auto expected = initial;
    model(expected, assignment(r7, value, true));
    require(machine.core.run(1).retired == 1 && machine.core.registers() == expected,
            "Host extension mutation replaces every embedded low24 constant");
  }
  const std::array body_steps{Step{.word = mac_a}, assignment(19, datum), assignment(27, 1, true)};
  const auto words = program(body_steps);
  const auto ending = start + 1 + static_cast<Word>(words.size());
  std::vector<Word> loop{do_three, ending};
  loop.insert(loop.end(), words.begin(), words.end());
  machine.core.load(Space::program, start, loop);
  for (Word budget = 0; budget <= 10; ++budget) {
    auto expected = initial;
    Word cycles = 0;
    if (budget != 0) {
      cycles = 5;
      expected.stack[3] = (std::uint64_t{initial.la} << word_bits) | initial.lc;
      expected.stack[4] = (std::uint64_t{start + 2} << word_bits) | initial.sr;
      expected.sp += 2;
      expected.sc += 2;
      expected.pc = start + 2;
      expected.la = ending;
      expected.lc = 3;
      expected.sr |= loop_flag;
      for (Word retired = 1; retired < budget; ++retired) {
        const auto position = (retired - 1) % 3;
        model(expected, body_steps[position]);
        cycles += length(body_steps[position]);
        if (position == 2) {
          if (expected.lc > 1) {
            --expected.lc;
            expected.pc = start + 2;
          } else {
            expected.la = initial.la;
            expected.lc = initial.lc;
            expected.sp = initial.sp;
            expected.sc = initial.sc;
            expected.sr &= ~loop_flag;
          }
        }
      }
    }
    machine.core.set_registers(initial);
    const auto base = machine.core.progress();
    const auto run = machine.core.run(budget);
    require(run.retired == budget && run.fault.kind == FaultKind::none &&
                machine.core.registers() == expected &&
                run.progress == Progress{.instructions = base.instructions + budget,
                                         .estimated_cycles = base.estimated_cycles + cycles},
            "Every loop prefix and final general cleanup recognizes an ending extension word");
  }
}

void aliased_extensions(const TestOptions options, const bool y) {
  std::array<Word, memory_words> shared{};
  std::array<Word, memory_words> other{};
  TestCore core(
      {.program = shared, .x = y ? other : shared, .y = y ? shared : other}, nullptr, options);
  constexpr Word boundary = 0x1ff;
  const std::array code{encoding(16, y), 1U};
  core.load(Space::program, boundary, code);
  auto initial = register_model::seed(mode_rm);
  initial.pc = boundary;
  core.set_registers(initial);
  require(core.run(1).retired == 1, "Warm an immediate extension across a decode-page boundary");
  core.write(y ? Space::y : Space::x, boundary + 1, datum);
  core.set_registers(initial);
  auto expected = initial;
  model(expected, assignment(16, datum, y));
  require(core.run(1).retired == 1 && core.registers() == expected,
          "Host writes through either data alias invalidate a preceding-page embedded constant");

  constexpr Word store = 0x446000;
  const std::array steps{Step{.word = mac_a}, Step{.word = mac_b}, assignment(24, 1)};
  auto words = program(steps);
  words.insert(words.begin() + 2, store | (y ? 0x080000U : 0U));
  core.load(Space::program, start, words);
  initial.pc = start;
  initial.r[0] = start + 4;
  initial.x = (initial.x & (std::uint64_t{word_mask} << word_bits)) | datum;
  expected = initial;
  model(expected, steps[0]);
  model(expected, steps[1]);
  ++expected.pc;
  model(expected, assignment(n0, datum));
  core.set_registers(initial);
  require(core.run(4).retired == 4 && core.registers() == expected && shared[start + 4] == datum,
          "A guest alias store replaces a planned successor extension before it executes");
}

void wrapped_and_unsupported(const TestOptions options) {
  const MemoryArena arena;
  arena.buffers().program[0] = raw_datum;
  TestCore core(arena.buffers(), nullptr, options);
  core.write(Space::program, word_mask, encoding(n7, true));
  auto initial = register_model::seed(mode_rm);
  initial.pc = word_mask;
  auto expected = initial;
  model(expected, assignment(n7, datum, true));
  core.set_registers(initial);
  const auto run = core.run(1);
  require(run.retired == 1 && run.fault.kind == FaultKind::none && core.registers() == expected &&
              run.progress == Progress{.instructions = 1, .estimated_cycles = 2},
          "A wrapped P extension supplies a native constant and advances PC/cost exactly");
  initial.sr = mode_down | mode_up;
  core.set_registers(initial);
  const auto failed = core.run(1);
  require(failed.retired == 0 && failed.fault.kind == FaultKind::unsupported_mode &&
              core.registers().n == initial.n && failed.progress == run.progress,
          "Unsupported numeric modes retain general rejection before native constant assignment");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::admission();
    dsp56300::test::direct_matrix();
    for (const auto backend : {dsp56300::Backend::jit}) {
      {
        const dsp56300::TestOptions options{.backend = backend};
        dsp56300::test::prefixes(options);
        dsp56300::test::extension_and_loops(options);
        for (const bool accept : {false, true}) {
          for (const bool invalidate : {false, true}) {
            dsp56300::test::callback(options, accept, invalidate);
          }
        }
      }
      const dsp56300::TestOptions options{.backend = backend};
      dsp56300::test::wrapped_and_unsupported(options);
      for (const bool y : {false, true}) { dsp56300::test::aliased_extensions(options, y); }
    }
    dsp56300::test::prefixes({.backend = dsp56300::Backend::jit});
    std::cout << dsp56300::test::checks << " independent native address-immediate checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
