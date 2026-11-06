// SPDX-License-Identifier: GPL-3.0-only
// Independent mixed-body accounting, modulo counters and guarded RX execution.
#include "address_model.hpp"
#include "architecture.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/emitter.hpp"
#include "jit/resident.hpp"
#include "loop_frames.hpp"
#include "native/direct.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"
#include "translation.hpp"
#include "value_ranges.hpp"
#include "word_reuse.hpp"

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
constexpr Word ram_words = 128, lf = Word{1} << 15, io_begin = 0xffff80;
constexpr Word r0_constant = 16, r4_constant = 32, single_prefix = 0x404000;
constexpr unsigned selector_shift = 17, high_selector = 24;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr auto boundary32 = std::uint64_t{1} << 32, boundary63 = std::uint64_t{1} << 63;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array progress_seeds{
    Progress{},
    Progress{.instructions = maximum - 2, .estimated_cycles = maximum - 4},
    Progress{.instructions = boundary32 - 1, .estimated_cycles = maximum},
    Progress{.instructions = maximum, .estimated_cycles = boundary63 - 1}};

enum class Action : std::uint8_t { constant, alu, nop, immediate, reg, ram };
constexpr Word default_budget = 36, datum = 0xfedcba;
constexpr unsigned r4_register = 20;

enum Opcode : Word {
  assign_r0 = 0x60f400,
  assign_r4 = 0x64f400,
  multiply_a = 0x2000d0,
  mac_round_a = 0xd3,
  mac_round_b = 0xfb,
  clear_a = 0x13,
  adc_x_a = 0x21,
  sub_x1_b = 0x6c,
  immediate_add = 0x0140c0,
  immediate_value = 0xfffe01,
  move_and_tfr = 0x21c479,
  adc_x_b = 0x29
};

struct Port {
  unsigned reg = 4, cursor = 0;
  bool y = false, load = true;
};

struct Step {
  Word word = 0, extension = 0;
  Action action = Action::alu;
  std::array<Port, 2> ports;
  unsigned count = 0, source = 0, destination = 0;
};

Step single(const Port port, const Word alu) {
  constexpr Word postincrement = 0x1800;
  return {.word = single_prefix | ((port.reg & 7U) << 16) |
                  ((port.reg & high_selector) << selector_shift) | (port.cursor << 8) |
                  postincrement | (port.y ? 0x080000U : 0U) | (port.load ? 0x8000U : 0U) | alu,
          .action = Action::ram,
          .ports = {port, {}},
          .count = 1};
}

Step pair(const bool load, const Word alu) {
  constexpr Word prefix = 0xb01800;
  constexpr Word load_bits = 0x408000;
  constexpr Word load_selectors = 0x050000;
  constexpr Word store_selectors = 0x0b0000;
  return {.word = prefix | (load ? load_bits | load_selectors : store_selectors) | alu,
          .action = Action::ram,
          .ports = {Port{.reg = load ? 5U : 14U, .cursor = 0, .load = load},
                    Port{.reg = load ? 7U : 15U, .cursor = 4, .y = true, .load = load}},
          .count = 2};
}

std::vector<Step> body() {
  return {
      {.word = assign_r0,
       .extension = r0_constant,
       .action = Action::constant,
       .ports = {},
       .destination = 16},
      {.word = assign_r4,
       .extension = r4_constant,
       .action = Action::constant,
       .ports = {},
       .destination = r4_register},
      {.word = multiply_a, .ports = {}},
      {.action = Action::nop, .ports = {}},
      single({}, mac_round_a),
      single({.reg = 6, .cursor = 1, .y = true}, mac_round_b),
      single({.reg = 14, .load = false}, clear_a),
      single({.reg = 15, .cursor = 4, .y = true, .load = false}, adc_x_a),
      pair(true, sub_x1_b),
      {.word = immediate_add,
       .extension = immediate_value,
       .action = Action::immediate,
       .ports = {}},
      {.word = move_and_tfr, .action = Action::reg, .ports = {}, .source = 14, .destination = 4},
      pair(false, adc_x_b)};
}

Word words(const Step step) {
  return step.action == Action::constant || step.action == Action::immediate ? 2U : 1U;
}

enum class Pattern : std::uint8_t { mixed, loads, crossed, only_a, only_b, reused };

struct Shape {
  Word mode = 0, first = start;
  bool loop = false, alias = false, maximum_cost = false, two_word_end = false;
  std::size_t padding = 0;
  Pattern pattern = Pattern::mixed;
  detail::LoopResidency residency = detail::LoopResidency::automatic;
  detail::RangeMode ranges = detail::RangeMode::automatic;
  detail::WordReuseMode word_reuse = detail::WordReuseMode::automatic;
  detail::LoopWindowMode window = detail::LoopWindowMode::automatic;
};

std::vector<Step> steps(const Shape shape) {
  auto result = body();
  if (shape.pattern == Pattern::reused) {
    constexpr std::array<Word, 8> products{
        0x200080, 0x200090, 0x2000d2, 0x2000f3, 0x2000b0, 0x2000e1, 0x2000a2, 0x2000c3};
    for (const auto product : products) {
      result.insert(result.begin() + 3, {.word = product, .ports = {}});
    }
  }
  if (shape.pattern == Pattern::loads) {
    result[4] = single({.reg = 14}, mac_round_b);
    result[5] = single({.reg = 15, .cursor = 1, .y = true}, adc_x_a);
    constexpr Word move_a_b = 0x21cf00;
    constexpr Word move_b_a_mac_b = 0x21eefb;
    constexpr Word move_a_a = 0x21ce00;
    result.insert(
        result.begin() + 6,
        {{.word = move_a_b, .action = Action::reg, .ports = {}, .source = 14, .destination = 15},
         {.word = move_b_a_mac_b,
          .action = Action::reg,
          .ports = {},
          .source = 15,
          .destination = 14},
         {.word = move_a_a, .action = Action::reg, .ports = {}, .source = 14, .destination = 14}});
  }
  if (shape.pattern == Pattern::crossed) {
    result[4] = single({.reg = 6}, mac_round_a);
    result[5] = single({.reg = 4, .cursor = 1, .y = true}, mac_round_b);
    constexpr Word x1_y1 = 0x20a700;
    constexpr Word y1_x0 = 0x20e400;
    result.insert(
        result.begin() + 6,
        {{.word = x1_y1, .action = Action::reg, .ports = {}, .source = 5, .destination = 7},
         {.word = y1_x0, .action = Action::reg, .ports = {}, .source = 7, .destination = 4}});
  }
  if (shape.pattern == Pattern::only_a || shape.pattern == Pattern::only_b) {
    for (auto &step : result) {
      if ((step.word & status_byte) == 0 || step.action == Action::constant) { continue; }
      step.word = (step.word & ~Word{8}) | (shape.pattern == Pattern::only_b ? 8U : 0U);
    }
  }
  const Step nop{.action = Action::nop, .ports = {}};
  result.insert(result.begin() + 4, shape.padding, nop);
  if (shape.two_word_end) {
    result.push_back(
        {.word = immediate_add, .extension = datum, .action = Action::immediate, .ports = {}});
  }
  return result;
}

std::vector<detail::BlockInstruction> operations(const Shape shape) {
  std::vector<detail::BlockInstruction> result;
  Word pc = shape.first;
  for (const auto step : steps(shape)) {
    auto decoded = detail::decode(step.word);
    decoded.extension = step.extension;
    require(decoded.words == words(step) && decoded.cycles == words(step),
            "Every mixed-body encoding has its independently declared length and base cost");
    if (shape.maximum_cost) { decoded.cycles = std::numeric_limits<std::uint8_t>::max(); }
    result.push_back({.instruction = decoded, .pc = pc, .mode = shape.mode});
    pc = (pc + decoded.words) & word_mask;
  }
  for (auto &op : result) {
    op.loop_address = shape.loop ? (pc - 1) & word_mask : detail::inactive_loop;
  }
  return result;
}

struct Memory {
  std::array<Word, std::size_t{ram_words} * 2> data{};
  bool alias = false;

  [[nodiscard]] std::span<Word> bank(const bool y) {
    return std::span(data).subspan(y && !alias ? ram_words : 0, ram_words);
  }

  void initialize() {
    constexpr Word tag = 0xab000000;
    constexpr Word multiplier = 7919;
    for (Word index = 0; index < data.size(); ++index) {
      data[index] = tag | ((index * multiplier) & word_mask);
    }
  }
};

std::array<detail::JitMemory, 2> banks(Memory &memory) {
  return {{{.data = memory.bank(false).data(), .words = ram_words},
           {.data = memory.bank(true).data(), .words = ram_words}}};
}

void arithmetic(Registers &state, const Registers &old, const Word word) {
  if ((state.sr & saturation_model::mode) != 0) {
    saturation_model::alu(state, old, word);
  } else {
    sequence_model::alu(state, old, word);
  }
}

void execute(Registers &state, Memory &memory, const Step step) {
  const auto old = state;
  std::array<Word, 2> addresses{};
  std::array<Word, 2> values{};
  for (unsigned index = 0; index < step.count; ++index) {
    const auto port = step.ports[index];
    const auto address = state.r[port.cursor];
    addresses[index] = address;
    state.r[port.cursor] = address_model::updated({.modifier = state.m[port.cursor],
                                                   .cursor = address,
                                                   .offset = state.n[port.cursor],
                                                   .mode = 3});
    values[index] = port.load ? memory.bank(port.y)[address] & word_mask
                              : register_model::read(state, port.reg);
  }
  for (unsigned index = 0; index < step.count; ++index) {
    const auto port = step.ports[index];
    if (port.load) {
      register_model::write(state, port.reg, values[index]);
    } else {
      memory.bank(port.y)[addresses[index]] = values[index];
    }
  }
  if (step.action == Action::constant) {
    register_model::write(state, step.destination, step.extension);
  } else if (step.action == Action::reg) {
    register_model::write(state, step.destination, register_model::read(state, step.source));
    arithmetic(state, old, step.word);
  } else if (step.action == Action::immediate) {
    auto immediate_old = old;
    immediate_old.x = step.extension;
    constexpr Word add_x0 = 0x40;
    arithmetic(state, immediate_old, add_x0 | (step.word & 8U));
  } else if (step.action != Action::nop) {
    arithmetic(state, old, step.word);
  }
  state.pc = (state.pc + words(step)) & word_mask;
}

struct Probe {
  unsigned calls = 0;
};

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++static_cast<Probe *>(owner)->calls;
  return 0;
}

std::uint64_t wrapping_sum(const std::uint64_t value, const std::uint64_t increment) {
  const auto room = maximum - value;
  return increment <= room ? value + increment : increment - room - 1;
}

struct Input {
  Progress progress;
  Word flags = 0, r1 = 0, lc = 4, sp = 2, target = start, budget = default_budget;
  Word sc = 2, saved = 0;
  bool mismatch = false, forever = false;
};

Registers initial(const Shape shape, const Input input) {
  auto state = register_model::seed(shape.mode);
  state.pc = shape.first;
  state.sr = shape.mode | input.flags | (shape.loop ? lf : 0);
  state.a = saturation_model::positive + 1;
  state.b = sign56;
  state.r[0] = state.r[4] = io_begin;
  state.r[1] = input.r1;
  if (input.mismatch) { state.m[1] = 15; }
  state.sp = input.sp;
  state.sc = input.sc;
  if (input.forever) { state.sr |= 1U << 16; }
  state.lc = input.lc;
  state.stack[state.sp % stack_entries] =
      (std::uint64_t{input.target} << word_bits) | 7 | input.saved;
  auto next = shape.first;
  for (const auto step : steps(shape)) { next = (next + words(step)) & word_mask; }
  state.la = (next + word_mask) & word_mask;
  return state;
}

void direct_case(const native::Direct &code, Memory &memory, const Shape shape, const Input input) {
  const auto trace = steps(shape);
  auto expected = initial(shape, input);
  memory.initialize();
  auto expected_memory = memory;
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = input.progress.instructions;
  cpu.cycles = input.progress.estimated_cycles;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  Word retired = 0;
  std::uint64_t cycles = 0;
  while (input.budget - retired >= trace.size() && expected.r[1] < ram_words &&
         expected.m[1] == word_mask && (!shape.loop || loop_frames::admitted(expected))) {
    for (const auto step : trace) {
      execute(expected, expected_memory, step);
      ++retired;
      cycles += shape.maximum_cost ? std::numeric_limits<std::uint8_t>::max() : words(step);
    }
    if (!shape.loop) { break; }
    if (expected.lc == 1) {
      loop_frames::complete(expected);
      break;
    }
    --expected.lc;
    expected.pc = input.target;
    if (input.target != shape.first) { break; }
  }
  Probe probe;

  detail::NativeContext context{.runtime = {.owner = &probe, .memory = banks(memory)},
                                .budget = input.budget};
  const auto count = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(probe.calls == 0 && count == retired && cpu.reg == expected &&
              memory.data == expected_memory.data,
          "Coalesced bytes preserve complete independent state, aliases and every guarded exit");
  require(cpu.instructions == wrapping_sum(input.progress.instructions, retired) &&
              cpu.cycles == wrapping_sum(input.progress.estimated_cycles, cycles),
          "Retired/mixed-cycle/move lanes independently wrap without charging NOP/ALU or paired "
          "ports twice");
  require(cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0),
          "Read-only rejection retains exclusion state and successful prefixes clear it");
}

void final_frames(const native::Direct &code, Memory &memory, const Shape shape) {
  const auto size = static_cast<Word>(steps(shape).size());
  for (const Word sp : {0U, 15U, 31U, 63U}) {
    for (const Word sc : {0U, 1U, 2U, 31U}) {
      for (const Word saved : {0U, lf, Word{1} << 16, loop_frames::loop_bits}) {
        for (const bool forever : {false, true}) {
          direct_case(code,
                      memory,
                      shape,
                      {.progress = progress_seeds[1],
                       .flags = status_byte,
                       .lc = 1,
                       .sp = sp,
                       .target = shape.first,
                       .budget = size,
                       .sc = sc,
                       .saved = saved,
                       .forever = forever});
        }
      }
    }
  }
  // A whole-call proof may cover every continuing iteration while the final
  // iteration would leave RAM. That last body must still reject before effects.
  direct_case(
      code,
      memory,
      shape,
      {.progress = {}, .r1 = ram_words - 2, .lc = 3, .target = shape.first, .budget = 3 * size});
}

void exit_cases(const native::Direct &code, Memory &memory, const Shape shape) {
  const auto size = static_cast<Word>(steps(shape).size());
  const std::array budgets{
      size, size + 1, (2 * size) - 1, 2 * size, (2 * size) + 1, (3 * size) - 1, 3 * size};
  for (const Word r1 : {0U, 126U, 127U, 128U, io_begin}) {
    for (const Word lc : {0U, 1U, 2U, 3U, 4U}) {
      for (const auto budget : budgets) {
        for (const Word target : {shape.first, (shape.first + 7) & word_mask, word_mask}) {
          direct_case(code,
                      memory,
                      shape,
                      {.progress = progress_seeds[1],

                       .flags = status_byte,
                       .r1 = r1,
                       .lc = lc,
                       .target = target,
                       .budget = budget});
        }
      }
    }
  }
  direct_case(code, memory, shape, {.progress = {}, .target = shape.first, .mismatch = true});
  direct_case(code,
              memory,
              shape,
              {.progress = progress_seeds[1],

               .r1 = ram_words,
               .target = shape.first});
}

void direct_shape(const Shape shape) {
  Memory memory{.alias = shape.alias};
  const auto ops = operations(shape);
  const detail::JitRuntime runtime{
      .execute = forbidden, .moves = forbidden, .memory = banks(memory)};
  detail::BlockEmitter emitter(runtime,
                               register_model::seed(0).m,
                               detail::RegionMode::guarded,
                               shape.residency,
                               shape.ranges,
                               shape.word_reuse,
                               shape.window);
  const native::Direct code(emitter.generate(ops));
  require(emitter.coalesced_accounting() && emitter.guarded_region() &&
              emitter.packed_retirement() && emitter.cached_status(),
          "Only bounded proved helper-free regions use the tested coalesced counter path");
  require(emitter.loop_residency() ==
              (shape.loop && shape.residency != detail::LoopResidency::disabled),
          "Only admitted closed loops seed authoritative accumulator slots across backedges");
  require(emitter.loop_bank_cache() ==
              (shape.loop && shape.residency == detail::LoopResidency::vectors),
          "Only helper-free resident loops hold X/Y separately from old parallel snapshots");
  require(emitter.loop_window() == (shape.loop && shape.window != detail::LoopWindowMode::disabled),
          "Only bounded closed RAM loops emit whole-call window preflights");
  if (shape.pattern == Pattern::reused) {
    require(emitter.word_reuse() == (shape.word_reuse != detail::WordReuseMode::disabled),
            "Only cost-admitted repeated words populate extra native slots");
  }
  for (Word flags = 0; flags <= status_byte; ++flags) {
    direct_case(code,
                memory,
                shape,
                {.progress = progress_seeds[flags % progress_seeds.size()],

                 .flags = flags,
                 .target = shape.first,
                 .budget = 3 * static_cast<Word>(ops.size())});
  }
  if (shape.padding == 0) {
    exit_cases(code, memory, shape);
    if (shape.loop) { final_frames(code, memory, shape); }
  }
}

void direct_matrix() {
  for (const auto mode : modes) {
    for (const bool saturated : {false, true}) {
      for (const Word first : {start, word_mask - 5}) {
        for (const bool loop : {false, true}) {
          for (const bool alias : {false, true}) {
            direct_shape({.mode = mode | (saturated ? saturation_model::mode : 0),
                          .first = first,
                          .loop = loop,
                          .alias = alias});
          }
        }
      }
    }
  }
}

void residency_patterns(const Word mode) {
  for (const auto pattern : {Pattern::mixed,
                             Pattern::loads,
                             Pattern::crossed,
                             Pattern::only_a,
                             Pattern::only_b,
                             Pattern::reused}) {
    for (const auto residency : {detail::LoopResidency::automatic,
                                 detail::LoopResidency::vectors,
                                 detail::LoopResidency::disabled}) {
      for (const bool alias : {false, true}) {
        direct_shape({.mode = mode,
                      .first = word_mask - 5,
                      .loop = true,
                      .alias = alias,
                      .pattern = pattern,
                      .residency = residency});
        direct_shape({.mode = mode,
                      .first = word_mask - 5,
                      .loop = true,
                      .alias = alias,
                      .pattern = pattern,
                      .residency = residency,
                      .ranges = detail::RangeMode::disabled});
        direct_shape({.mode = mode,
                      .first = word_mask - 5,
                      .loop = true,
                      .alias = alias,
                      .pattern = pattern,
                      .residency = residency,
                      .word_reuse = detail::WordReuseMode::disabled});
        direct_shape({.mode = mode,
                      .first = word_mask - 5,
                      .loop = true,
                      .alias = alias,
                      .pattern = pattern,
                      .residency = residency,
                      .word_reuse = detail::WordReuseMode::vectors});
        direct_shape({.mode = mode,
                      .first = word_mask - 5,
                      .loop = true,
                      .alias = alias,
                      .pattern = pattern,
                      .residency = residency,
                      .window = detail::LoopWindowMode::disabled});
      }
    }
  }
}

std::vector<Word> program(const std::span<const Step> trace) {
  std::vector<Word> result;
  for (const auto step : trace) {
    result.push_back(step.word);
    if (words(step) == 2) { result.push_back(step.extension); }
  }
  return result;
}

void api_case(TestCore &core, Memory &memory, const Shape shape, const Word budget) {
  memory.initialize();
  auto expected_memory = memory;
  auto expected = initial(shape, {.progress = {}, .flags = status_byte});
  core.set_registers(expected);
  const auto before = core.progress();

  const auto trace = steps(shape);
  Word cycles = 0;

  for (const auto step : std::span(trace).first(budget)) {
    execute(expected, expected_memory, step);
    cycles += words(step);
  }
  const auto run = core.run(budget);
  require(run.retired == budget && run.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.data == expected_memory.data &&
              run.progress == Progress{.instructions = before.instructions + budget,
                                       .estimated_cycles = before.estimated_cycles + cycles},
          "Every warm mixed API prefix exposes independent full state and exact base costs");
}

void api_prefixes(const TestOptions options, const bool alias) {
  Memory memory{.alias = alias};
  std::array<Word, memory_words> p{};
  TestCore core({.program = p, .x = memory.bank(false), .y = memory.bank(true)}, nullptr, options);
  for (const auto mode : modes) {
    for (const bool saturated : {false, true}) {
      const Shape shape{.mode = mode | (saturated ? saturation_model::mode : 0), .alias = alias};
      const auto trace = steps(shape);
      core.load(Space::program, start, program(trace));
      api_case(core, memory, shape, static_cast<Word>(trace.size()));
      for (Word budget = 0; budget <= trace.size(); ++budget) {
        api_case(core, memory, shape, budget);
      }
    }
  }
}

void memory_fault(const Word mode, const bool alias, const Word completed) {
  Memory memory{.alias = alias};
  memory.initialize();
  auto expected_memory = memory;
  std::array<Word, memory_words> program_memory{};
  TestCore core({.program = program_memory, .x = memory.bank(false), .y = memory.bank(true)});
  const Shape shape{.mode = mode, .loop = true, .alias = alias};
  const auto trace = steps(shape);
  core.load(Space::program, start, program(trace));
  auto expected = initial(shape, {.progress = {}, .r1 = ram_words - completed, .lc = 3});
  core.set_registers(expected);
  Word retired = 0;
  Word cycles = 0;
  for (Word iteration = 0; iteration < completed; ++iteration) {
    for (const auto step : trace) {
      execute(expected, expected_memory, step);
      ++retired;
      cycles += words(step);
    }
    --expected.lc;
    expected.pc = start;
  }
  constexpr std::size_t failure = 5;
  for (const auto step : std::span(trace).first(failure)) {
    execute(expected, expected_memory, step);
    ++retired;
    cycles += words(step);
  }
  const auto fault_pc = expected.pc;
  const auto port = trace[failure].ports[0];
  expected.r[port.cursor] = ram_words + 1;
  expected.pc += words(trace[failure]);
  const auto result = core.run((completed + 1) * static_cast<Word>(trace.size()));
  require(result.retired == retired &&
              result.progress == Progress{.instructions = retired, .estimated_cycles = cycles} &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::y,
                                    .address = ram_words,
                                    .pc = fault_pc} &&
              !result.yielded && result.state == ExecutionState::running &&
              core.registers() == expected && memory.data == expected_memory.data,
          "Entry and backedge admission preserve the completed prefix and fault before "
          "the rejected load/ALU/frame cleanup");
}

void memory_faults() {
  for (const auto mode : modes) {
    for (const bool alias : {false, true}) {
      for (const Word completed : {0U, 1U, 2U}) {
        memory_fault(mode, alias, completed);
        memory_fault(mode | saturation_model::mode, alias, completed);
      }
    }
  }
}

void api_loop_case(TestCore &core, Memory &memory, const Shape shape, const Word budget) {
  memory.initialize();
  auto expected_memory = memory;
  auto expected = initial(shape, {.progress = {}, .flags = status_byte, .target = start});
  core.set_registers(expected);
  const auto progress = core.progress();

  const auto trace = steps(shape);
  Word cycles = 0;

  for (Word index = 0; index < budget; ++index) {
    const auto position = index % trace.size();
    const auto step = trace[position];
    execute(expected, expected_memory, step);
    cycles += words(step);

    if (position + 1 == trace.size()) {
      --expected.lc;
      expected.pc = start;
    }
  }
  const auto run = core.run(budget);
  require(run.retired == budget && run.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.data == expected_memory.data &&
              run.progress == Progress{.instructions = progress.instructions + budget,
                                       .estimated_cycles = progress.estimated_cycles + cycles},
          "Every resident multi-iteration API prefix exposes exact registers, memory and costs");
  if (budget != 3 * trace.size()) { return; }

  require(core.interrupt_ready(), "A returned resident loop exposes an admitted IRQ boundary");
  constexpr Word vector = 32;
  core.interrupt(vector, 1);
  require(core.registers() == expected && memory.data == expected_memory.data &&
              !core.interrupt_ready() &&
              core.progress() == Progress{.instructions = run.progress.instructions + 2,
                                          .estimated_cycles = run.progress.estimated_cycles + 2},
          "Fast service preserves the complete resident-loop checkpoint and live loop frame");
  execute(expected, expected_memory, trace.front());
  const auto resumed = core.run(1);
  require(resumed.retired == 1 && resumed.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.data == expected_memory.data &&
              resumed.progress == Progress{.instructions = run.progress.instructions + 3,
                                           .estimated_cycles = run.progress.estimated_cycles + 4} &&
              core.interrupt_ready(),
          "Foreground reentry with final LC retains canonical accumulators and releases exclusion");
}

void api_loops(const TestOptions options, const Word mode) {
  for (const auto pattern :
       {Pattern::mixed, Pattern::loads, Pattern::crossed, Pattern::only_a, Pattern::only_b}) {
    Memory memory{.alias = true};
    std::array<Word, memory_words> p{};
    TestCore core(
        {.program = p, .x = memory.bank(false), .y = memory.bank(true)}, nullptr, options);
    const Shape shape{.mode = mode, .loop = true, .alias = true, .pattern = pattern};
    const auto trace = steps(shape);
    constexpr Word vector = 32;
    core.load(Space::program, vector, std::array{0U, 0U});
    core.load(Space::program, start, program(trace));
    const auto maximum_budget = 3 * static_cast<Word>(trace.size());

    api_loop_case(core, memory, shape, maximum_budget);
    for (Word budget = 0; budget <= maximum_budget; ++budget) {
      api_loop_case(core, memory, shape, budget);
    }
  }
}

class Observer final : public Io {
public:
  TestCore *core = nullptr;
  Registers expected;
  Progress progress;
  Word opcode_pc = 0, extension_address = 0;
  bool accept = true, patch = false, observed = false;
  unsigned calls = 0;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++calls;
    observed = core != nullptr && space == Space::y && address == io_begin && pc == opcode_pc &&
               core->registers() == expected && core->progress() == progress;
    if (observed && accept && patch) { core->write(Space::program, extension_address, datum); }
    value = datum;
    asm volatile("pcmpeqd %%xmm8, %%xmm8\n\tpxor %%xmm9, %%xmm9\n\t"
                 "pcmpeqd %%xmm10, %%xmm10\n\tpxor %%xmm11, %%xmm11"
                 :
                 :
                 : "xmm8", "xmm9", "xmm10", "xmm11");
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

struct CallbackInput {
  Backend backend = Backend::jit;
  Word mode = 0;
  bool accept = true, patch = false, loop = false;
};

void callback(const CallbackInput input) {
  constexpr std::size_t io_index = 5;
  constexpr std::size_t immediate_index = 9;
  Memory memory;
  std::array<Word, memory_words> p{};
  Observer observer;
  observer.accept = input.accept;
  observer.patch = input.patch;
  TestCore core({.program = p, .x = memory.bank(false), .y = memory.bank(true)},
                &observer,
                {.backend = input.backend});
  observer.core = &core;
  const Shape shape{.mode = input.mode, .loop = input.loop};
  auto trace = steps(shape);
  core.load(Space::program, start, program(trace));
  memory.initialize();
  core.set_registers(initial(shape, {.progress = {}, .flags = status_byte}));
  require(core.run(static_cast<Word>(trace.size())).retired == trace.size() && observer.calls == 0,
          "Warm an actual closed region before changing the same-PC guard to I/O");
  memory.initialize();
  auto expected_memory = memory;
  auto state = initial(shape, {.progress = {}, .flags = status_byte, .r1 = io_begin});
  core.set_registers(state);
  const auto before = core.progress();
  Word cycles = 0;
  for (const auto step : std::span(trace).first(io_index)) {
    execute(state, expected_memory, step);
    cycles += words(step);
  }
  observer.opcode_pc = state.pc;
  const auto ops = operations(shape);
  observer.extension_address = ops[immediate_index].pc + 1;
  observer.expected = state;
  ++observer.expected.pc;
  ++observer.expected.r[1];
  observer.progress = {.instructions = before.instructions + io_index,
                       .estimated_cycles = before.estimated_cycles + cycles};
  const auto run = core.run(static_cast<Word>(trace.size()));
  require(observer.observed && observer.calls == 1,
          "Guard failure publishes exact mixed prefix counters and canonical state before I/O");
  const auto old = state;
  state = observer.expected;
  if (!input.accept) {
    require(run.retired == io_index && run.progress == observer.progress &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::y,
                                   .address = io_begin,
                                   .pc = observer.opcode_pc} &&
                core.registers() == state && memory.data == expected_memory.data,
            "Rejected mixed fallback preserves the destination and has no ALU/retirement");
    require(core.run(1).retired == 0 && core.registers() == state &&
                core.progress() == run.progress,
            "A rejected fallback remains fail-stop without replaying a charged prefix");
    return;
  }
  register_model::write(state, 6, datum);
  arithmetic(state, old, trace[io_index].word);
  if (input.patch) { trace[immediate_index].extension = datum; }
  for (const auto step : std::span(trace).subspan(io_index + 1)) {
    execute(state, expected_memory, step);
  }
  if (input.loop) {
    --state.lc;
    state.pc = start;
  }
  require(run.retired == trace.size() && run.fault.kind == FaultKind::none &&
              run.progress == Progress{.instructions = before.instructions + trace.size(),
                                       .estimated_cycles = before.estimated_cycles + 15} &&
              core.registers() == state && memory.data == expected_memory.data,
          "Successful I/O/P edits discard stale closed code, retaining exact old operands and all "
          "counters");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::direct_matrix();
    dsp56300::test::memory_faults();
    for (const auto mode : dsp56300::test::modes) {
      dsp56300::test::residency_patterns(mode);
      dsp56300::test::residency_patterns(mode | dsp56300::test::saturation_model::mode);
    }
    for (const bool alias : {false, true}) {
      for (const bool loop : {false, true}) {
        dsp56300::test::direct_shape({.mode = dsp56300::test::saturation_model::mode |
                                              dsp56300::test::mode_rm | dsp56300::test::mode_up,
                                      .loop = loop,
                                      .alias = alias,
                                      .two_word_end = true});
      }
    }
    constexpr std::size_t nop_padding = 31;
    dsp56300::test::direct_shape({.loop = true, .padding = nop_padding});
    for (const std::size_t padding : {244U, 245U, 1000U}) {
      dsp56300::test::direct_shape({.mode = dsp56300::test::saturation_model::mode,
                                    .loop = true,
                                    .maximum_cost = true,
                                    .padding = padding});
    }
    for (const auto backend : {dsp56300::Backend::jit}) {
      for (const auto mode : dsp56300::test::modes) {
        dsp56300::test::api_loops({.backend = backend}, mode);
        dsp56300::test::api_loops({.backend = backend},
                                  mode | dsp56300::test::saturation_model::mode);
      }
      {
        for (const bool alias : {false, true}) {
          dsp56300::test::api_prefixes({.backend = backend}, alias);
        }
      }
      for (const auto mode : {dsp56300::Word{0}, dsp56300::test::saturation_model::mode}) {
        for (const bool accept : {false, true}) {
          for (const bool patch : {false, true}) {
            dsp56300::test::callback(
                {.backend = backend, .mode = mode, .accept = accept, .patch = patch});
            dsp56300::test::callback(
                {.backend = backend, .mode = mode, .accept = accept, .patch = patch, .loop = true});
          }
        }
      }
    }
    dsp56300::test::api_prefixes({.backend = dsp56300::Backend::jit}, true);
    std::cout << dsp56300::test::checks << " independent region-accounting checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
