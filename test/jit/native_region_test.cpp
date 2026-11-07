// SPDX-License-Identifier: GPL-3.0-only
// Original mixed RAM traces and independent ordered transfer/arithmetic expectations.
#include "address_model.hpp"
#include "architecture.hpp"
#include "codegen_observer.hpp"
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
constexpr Word io_begin = 0xffff80, lf = Word{1} << 15;
constexpr std::size_t ram_words = 128, short_x_words = 96, short_y_words = 120;
constexpr Word ring = 19, ring_high = 0xab0013, high_linear = 0xabffff;
constexpr unsigned selector_high_shift = 17, pair_x_shift = 18, pair_y_mode_shift = 20;
constexpr Word single_base = 0x404000, reserved_modifier = 0x8002, low_modifier = 0xffff;
constexpr Word output_begin = 48, coefficient_begin = 96, near_end = 115, agu_only = 0x204800;
constexpr Word last_replaced = 17;

enum Alu : std::uint8_t {
  mac_round_a = 0xd3,
  mac_round_b = 0xfb,
  clear_a = 0x13,
  add_x1_b = 0x68,
  mac_a = 0xd2,
  add_a_b = 0x18,
  mac_x1_a = 0xe2,
  add_y1_a = 0x70,
  asr_a = 0x22,
  adc_x_a = 0x21,
  round_b = 0x19,
  sub_x1_a = 0x64,
  mac_b = 0xfa,
  negate_b = 0x3e,
  adc_x_b = 0x29
};

constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

enum class Shape : std::uint8_t {
  unit,
  strided,
  mixed,
  y_strided,
  terminal_stride,
  terminal_y_stride
};
constexpr std::array shapes{Shape::unit,
                            Shape::strided,
                            Shape::mixed,
                            Shape::y_strided,
                            Shape::terminal_stride,
                            Shape::terminal_y_stride};

bool terminal(const Shape shape) {
  return shape == Shape::terminal_stride || shape == Shape::terminal_y_stride;
}

struct Transfer {
  unsigned reg = 4, index = 0, mode = 3;
  bool y = false, load = true;
};

struct Step {
  Word word = 0;
  std::array<Transfer, 2> ports;
  unsigned count = 1;
};

Step single(const Transfer transfer, const Word alu) {
  constexpr Word high_selector_mask = 24;
  return {.word = single_base | ((transfer.reg & 7U) << 16) |
                  ((transfer.reg & high_selector_mask) << selector_high_shift) |
                  (transfer.y ? 0x080000U : 0U) | (transfer.load ? 0x8000U : 0U) |
                  (transfer.mode << 11) | (transfer.index << 8) | alu,
          .ports = {transfer, {}}};
}

Step pair(const Transfer x, const Transfer y, const Word alu) {
  const auto xreg = x.reg < 8 ? x.reg - 4 : x.reg - 12;
  const auto yreg = y.reg < 8 ? y.reg - 6 : y.reg - 12;
  return {.word = 0x800000 | ((y.mode == 4 ? 0U : y.mode) << pair_y_mode_shift) |
                  ((x.mode == 4 ? 0U : x.mode) << 11) | (xreg << pair_x_shift) | (yreg << 16) |
                  (x.load ? 0x8000U : 0U) | (y.load ? 0x400000U : 0U) | alu,
          .ports = {x, y},
          .count = 2};
}

std::vector<Step> trace(const Shape shape = Shape::unit) {
  std::vector<Step> result{
      single({}, mac_round_a),
      single({.reg = 6, .index = 4, .y = true}, mac_round_b),
      single({.reg = 14, .index = 1, .load = false}, clear_a),
      single({.reg = 5, .index = 2, .mode = 4}, add_x1_b),
      pair({}, {.reg = 6, .index = 4, .y = true}, mac_a),
      single({.reg = 14}, add_a_b),
      single({.reg = 5}, mac_x1_a),
      single({.reg = 7, .index = 4, .y = true}, add_y1_a),
      pair({.reg = 14, .load = false}, {.reg = 15, .index = 4, .y = true, .load = false}, asr_a),
      single({.index = 2, .mode = 4}, adc_x_a),
      single({.reg = 6, .index = 4, .y = true}, round_b),
      single({.reg = 5}, sub_x1_a),
      pair({.load = false}, {.reg = 6, .index = 4, .y = true}, mac_b),
      single({.reg = 15, .index = 1, .load = false}, 0),
      single({}, negate_b),
      single({.reg = 6, .index = 4, .y = true}, mac_round_a),
      single({.reg = 14, .index = 1, .load = false}, add_a_b),
      single({.reg = 5, .index = 2, .mode = 4}, adc_x_b)};
  for (std::size_t index = 0; index < result.size(); ++index) {
    auto &step = result[index];
    for (auto &port : std::span(step.ports).first(step.count)) {
      if (((shape == Shape::strided || (shape == Shape::mixed && index % 2 == 0)) &&
           port.mode == 3 && !port.y) ||
          (shape == Shape::y_strided && port.y && port.mode == 3) ||
          (terminal(shape) && index + 1 == result.size())) {
        port.mode = 1;
        if (shape == Shape::terminal_y_stride) { port.y = true; }
      }
    }
    step = step.count == 2 ? pair(step.ports[0], step.ports[1], step.word & status_byte)
                           : single(step.ports[0], step.word & status_byte);
  }
  return result;
}

std::array<Word, agu_registers> modifiers(const Shape shape = Shape::unit) {
  std::array<Word, agu_registers> result{};
  result.fill(word_mask);
  result[2] =
      terminal(shape) ? word_mask : reserved_modifier; // No-update addresses must not depend on M.
  result[4] = shape == Shape::y_strided ? word_mask : ring;
  return result;
}

struct Memory {
  std::array<Word, ram_words * 2> words{};
  bool alias = false;
  std::array<std::size_t, 2> limits{ram_words, ram_words};

  [[nodiscard]] std::span<Word> bank(const bool y) {
    return std::span(words).subspan(y && !alias ? ram_words : 0, limits[y ? 1 : 0]);
  }

  void initialize() {
    constexpr Word high_bits = 0xab000000;
    constexpr Word multiplier = 7919;
    for (Word index = 0; index < words.size(); ++index) {
      words[index] = high_bits | ((index * multiplier) & word_mask);
    }
  }
};

void execute(Registers &state, Memory &memory, const Step step) {
  const auto old = state;
  std::array<Word, 2> addresses{};
  std::array<Word, 2> values{};
  for (unsigned index = 0; index < step.count; ++index) {
    const auto &port = step.ports[index];
    addresses[index] = state.r[port.index];
    if (port.mode != 4) {
      state.r[port.index] = address_model::updated({.modifier = state.m[port.index],
                                                    .cursor = addresses[index],
                                                    .offset = state.n[port.index],
                                                    .mode = port.mode});
    }
    values[index] = port.load ? memory.bank(port.y)[addresses[index]] & word_mask
                              : register_model::read(state, port.reg);
  }
  for (unsigned index = 0; index < step.count; ++index) {
    const auto &port = step.ports[index];
    if (port.load) {
      register_model::write(state, port.reg, values[index]);
    } else {
      memory.bank(port.y)[addresses[index]] = values[index];
    }
  }
  sequence_model::alu(state, old, step.word);
  state.pc = (state.pc + 1) & word_mask;
}

std::vector<detail::BlockInstruction>
operations(const Word mode, const bool loop, const Shape shape = Shape::unit) {
  const auto steps = trace(shape);
  const auto last = start + static_cast<Word>(steps.size()) - 1;
  std::vector<detail::BlockInstruction> result;
  for (std::size_t index = 0; index < steps.size(); ++index) {
    auto decoded = detail::decode(steps[index].word);
    require(decoded.operation == detail::Operation::parallel && decoded.words == 1 &&
                decoded.cycles == 1,
            "Every independently encoded region step has its declared operation and base cost");
    result.push_back({.instruction = decoded,
                      .pc = start + static_cast<Word>(index),
                      .mode = mode,
                      .loop_address = loop ? last : detail::inactive_loop});
  }
  return result;
}

struct Input {
  Word r0 = 0, r1 = output_begin, r2 = 64, r4 = coefficient_begin, m0 = word_mask, m4 = ring;
  Word flags = 0, lc = 4, bodies = 1;
  Word n0 = 1, n1 = 1, n2 = 0, n4 = 1;
};

bool admitted(const Registers &state, const bool loop, const Shape shape, const Memory &memory) {
  if ((loop && !loop_frames::admitted(state)) || (state.m[0] & low_modifier) != low_modifier ||
      (state.m[1] & low_modifier) != low_modifier ||
      (state.m[4] & low_modifier) != (shape == Shape::y_strided ? low_modifier : ring) ||
      (terminal(shape) && (state.m[2] & low_modifier) != low_modifier) ||
      ((shape == Shape::strided || shape == Shape::mixed) &&
       (state.n[0] >= sign24 || state.n[1] >= sign24)) ||
      (shape == Shape::y_strided && state.n[4] >= sign24)) {
    return false;
  }
  const auto base = (state.r[4] / 32) * 32;
  if (shape != Shape::y_strided &&
      (state.r[4] >= base + ring + 1 || base + ring + 1 > memory.limits[1])) {
    return false;
  }
  auto cursors = state.r;
  for (const auto step : trace(shape)) {
    for (unsigned index = 0; index < step.count; ++index) {
      const auto &port = step.ports[index];
      const auto cursor = cursors[port.index];
      if (cursor >= memory.limits[port.y ? 1 : 0]) { return false; }
      cursors[port.index] = address_model::updated({.modifier = state.m[port.index],
                                                    .cursor = cursor,
                                                    .offset = state.n[port.index],
                                                    .mode = port.mode});
    }
  }
  return true;
}

std::uint64_t sum(const std::uint64_t a, const std::uint64_t b) {
  const auto room = maximum - a;
  return b <= room ? a + b : b - room - 1;
}

void direct_case(const native::Direct &code,
                 Memory &actual_memory,
                 const Word mode,
                 const bool loop,
                 const Input input,
                 const Shape shape = Shape::unit) {
  actual_memory.initialize();
  auto expected_memory = actual_memory;
  auto expected = register_model::seed(mode);
  expected.sr = mode | input.flags | (loop ? lf : 0U);
  expected.n[0] = (input.flags & 1U) != 0 ? word_mask : sign24;
  expected.n[1] = input.n1;
  expected.n[2] = input.n2;
  if (shape == Shape::strided || shape == Shape::mixed) { expected.n[0] = input.n0; }
  expected.n[4] = (input.flags & 2U) != 0 ? 0U : word_mask;
  if (shape == Shape::y_strided) { expected.n[4] = input.n4; }
  expected.m = modifiers(shape);
  expected.m[0] = input.m0;
  expected.m[4] = shape == Shape::y_strided && input.m4 == ring ? word_mask : input.m4;
  expected.r[0] = input.r0;
  expected.r[1] = input.r1;
  expected.r[2] = input.r2;
  expected.r[4] = input.r4;
  expected.lc = input.lc;
  const auto steps = trace(shape);
  const auto size = static_cast<Word>(steps.size());
  expected.la = start + size - 1;
  expected.stack[expected.sp % stack_entries] = std::uint64_t{start} << word_bits;
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = maximum - 2;
  cpu.cycles = maximum - 3;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  Word retired = 0;
  while (input.bodies * size - retired >= size && admitted(expected, loop, shape, actual_memory)) {
    for (const auto step : steps) { execute(expected, expected_memory, step); }
    retired += size;
    if (!loop) { break; }
    if (expected.lc == 1) {
      loop_frames::complete(expected);
      break;
    }
    --expected.lc;
    expected.pc = start;
  }
  detail::NativeContext context{.runtime = {.memory = {}}, .budget = input.bodies * size};
  const auto result = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(result == retired && cpu.reg == expected && actual_memory.words == expected_memory.words,
          "Guarded bytes preserve complete independent state, old inputs and ordered RAM aliases");
  require(cpu.instructions == sum(maximum - 2, retired) &&
              cpu.cycles == sum(maximum - 3, retired) && cpu.repeat_completed == (retired == 0) &&
              cpu.interrupt_inhibited == (retired == 0),
          "Initial/backedge guard rejection preserves exact progress, moves and exclusion fields");
}

void stride_matrix(const native::Direct &code,
                   Memory &memory,
                   const Word mode,
                   const bool loop,
                   const Shape shape) {
  if (shape != Shape::unit) {
    for (const Word n0 :
         {0U, 1U, 2U, 8U, 15U, 16U, 127U, 128U, sign24 - 1, sign24, word_mask, 0xffffffffU}) {
      for (const Word n1 : {0U, 1U, 2U, 26U, 27U, sign24, word_mask}) {
        for (const Word n2 : {0U, 1U, 64U, 65U, sign24 - 1, sign24, word_mask, 0xffffffffU}) {
          direct_case(code,
                      memory,
                      mode,
                      loop,
                      {.flags = status_byte, .bodies = 3, .n0 = n0, .n1 = n1, .n2 = n2, .n4 = n0},
                      shape);
        }
      }
    }
  }
}

void direct_shape(
    const Word mode, const bool loop, const bool alias, const Shape shape, const bool shortened) {
  Memory memory{.alias = alias};
  if (shortened) { memory.limits = {short_x_words, short_y_words}; }
  const detail::JitRuntime runtime{
      .memory = {{{.data = memory.bank(false).data(), .words = memory.limits[0]},
                  {.data = memory.bank(true).data(), .words = memory.limits[1]}}}};
  const auto ops = operations(mode, loop, shape);
  detail::BlockEmitter emitter(runtime, modifiers(shape), detail::RegionMode::guarded);
  const native::Direct code(emitter.generate(ops));
  require(emitter.guarded_region() && emitter.cached_status() && emitter.packed_retirement(),
          "Actual guarded bytes qualify held status and helper-free packed retirement");
  for (Word flags = 0; flags <= status_byte; ++flags) {
    direct_case(code, memory, mode, loop, {.flags = flags, .bodies = 3}, shape);
  }
  for (const Word r0 : {0U, 64U, 115U, 116U, 127U, 128U, io_begin, word_mask}) {
    for (const Word m0 : {word_mask, high_linear, 15U, 0U, 0x800fU, 0x8002U}) {
      for (const Word r4 : {1U, 96U, 114U, 115U, 116U, 127U, 128U}) {
        for (const Word m4 : {ring, ring_high, 15U, word_mask}) {
          direct_case(code,
                      memory,
                      mode,
                      loop,
                      {.r0 = r0, .r4 = r4, .m0 = m0, .m4 = m4, .flags = status_byte, .bodies = 3},
                      shape);
        }
      }
    }
  }
  for (const Word lc : {0U, 1U, 2U, 3U, 7U}) {
    direct_case(code,
                memory,
                mode,
                loop,
                {.r0 = near_end, .flags = status_byte, .lc = lc, .bodies = 5},
                shape);
  }
  for (const Word r1 : {0U, 125U, 126U, 127U, 128U, io_begin}) {
    for (const Word r2 : {0U, 127U, 128U, io_begin}) {
      direct_case(
          code, memory, mode, loop, {.r1 = r1, .r2 = r2, .flags = status_byte, .bodies = 3}, shape);
    }
  }
  stride_matrix(code, memory, mode, loop, shape);
}

void direct_matrix() {
  for (const auto mode : modes) {
    for (const bool loop : {false, true}) {
      for (const bool alias : {false, true}) {
        for (const auto shape : shapes) {
          for (const bool shortened : {false, true}) {
            direct_shape(mode, loop, alias, shape, shortened);
          }
        }
      }
    }
  }
}

void admission() {
  Memory memory;
  const std::array<detail::JitMemory, 2> banks{
      {{.data = memory.bank(false).data(), .words = ram_words},
       {.data = memory.bank(true).data(), .words = ram_words}}};
  auto ops = operations(0, false);
  require(detail::resident_plan(ops, banks, modifiers()).valid,
          "The generic proof admits the original mixed trace");
  auto changed = ops;
  changed[0].instruction = detail::decode(agu_only);
  require(!detail::resident_plan(changed, banks, modifiers()).valid,
          "Unproved pure AGU changes cannot enter a closed region");
  changed = ops;
  ++changed[3].pc;
  require(!detail::resident_plan(changed, banks, modifiers()).valid,
          "Noncontiguous metadata cannot hide control changes");
  auto altered = banks;
  altered[0].aliases_program = true;
  require(!detail::resident_plan(ops, altered, modifiers()).valid,
          "Every possibly aliased P store retains ordinary invalidation boundaries");
  auto mods = modifiers();
  mods[0] = 0;
  require(!detail::resident_plan(ops, banks, mods).valid,
          "Reverse/multiple-wrap updates are outside this region proof");
  const auto stride_ops = operations(0, false, Shape::strided);
  require(detail::resident_plan(stride_ops, banks, modifiers()).valid,
          "A linear post-N trace admits a runtime range proof without sampling N");
  for (const Word modifier : {0U, 15U, ring, 0x800fU, reserved_modifier}) {
    mods = modifiers();
    mods[0] = modifier;
    require(!detail::resident_plan(stride_ops, banks, mods).valid,
            "Post-N modulo, reverse and reserved modes retain the general path");
  }
}

Registers initial(const Word mode, const Shape shape = Shape::unit) {
  auto state = register_model::seed(mode);
  state.m = modifiers(shape);
  state.n[0] = 2;
  state.n[1] = 3;
  state.n[2] = word_mask;
  state.r[0] = 0;
  state.r[1] = output_begin;
  state.r[2] = 64;
  state.r[4] = shape == Shape::y_strided ? coefficient_begin : near_end;
  state.n[4] = 1;
  return state;
}

std::vector<Word> program(const Shape shape = Shape::unit) {
  std::vector<Word> words;
  for (const auto step : trace(shape)) { words.push_back(step.word); }
  return words;
}

void prefixes(const Backend backend, const Word mode, const Shape shape) {
  Memory memory;
  std::array<Word, memory_words> p{};
  const auto words = program(shape);
  TestCore core({.program = std::span(p).first(start + words.size()),
                 .x = memory.bank(false),
                 .y = memory.bank(true)},
                nullptr,
                {.backend = backend});
  core.load(Space::program, start, words);
  const auto state = initial(mode, shape);
  const auto steps = trace(shape);
  memory.initialize();
  core.set_registers(state);
  require(core.run(static_cast<Word>(steps.size())).retired == steps.size(),
          "Warm the original trace before testing exact call prefixes");
  const auto publications = codegen::publications();
  for (Word budget = 0; budget <= steps.size(); ++budget) {
    memory.initialize();
    auto expected_memory = memory;
    auto expected = state;
    for (const auto step : std::span(steps).first(budget)) {
      execute(expected, expected_memory, step);
    }
    core.set_registers(state);
    const auto base = core.progress();
    const auto run = core.run(budget);
    require(run.retired == budget && run.fault.kind == FaultKind::none &&
                core.registers() == expected && memory.words == expected_memory.words &&
                run.progress == Progress{.instructions = base.instructions + budget,
                                         .estimated_cycles = base.estimated_cycles + budget},
            "Every warm API budget exposes exact full state, sticky flags and all RAM");
    require(codegen::publications() == publications,
            "Whole and partial memory-region grants retain the same published translations");
  }
}

void changed_offsets(const Backend backend, const Word mode, const Shape shape) {
  Memory memory;
  std::array<Word, memory_words> p{};
  const auto words = program(shape);
  TestCore core({.program = std::span(p).first(start + words.size()),
                 .x = memory.bank(false),
                 .y = memory.bank(true)},
                nullptr,
                {.backend = backend});
  core.load(Space::program, start, words);
  const auto steps = trace(shape);
  for (const Word offset : {1U, 0U, 2U, word_mask, 1U}) {
    auto state = initial(mode, shape);
    state.r[0] = 32;
    state.n[0] = state.n[1] = state.n[2] = offset;
    if (shape == Shape::y_strided) { state.n[4] = offset; }
    memory.initialize();
    auto expected_memory = memory;
    auto expected = state;
    for (const auto step : steps) { execute(expected, expected_memory, step); }
    core.set_registers(state);
    const auto base = core.progress();
    const auto run = core.run(static_cast<Word>(steps.size()));
    require(run.retired == steps.size() && run.fault.kind == FaultKind::none &&
                core.registers() == expected && memory.words == expected_memory.words &&
                core.progress() ==
                    Progress{.instructions = base.instructions + steps.size(),
                             .estimated_cycles = base.estimated_cycles + steps.size()},
            "Same-PC changed N selects proved or ordinary execution without stale addresses");
  }
}

class Observer final : public Io {
public:
  TestCore *core = nullptr;
  Registers expected;
  Progress progress;
  bool accept = true, replace_next = false, observed = false;
  unsigned calls = 0;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++calls;
    observed = core != nullptr && space == Space::x && address == io_begin && pc == start + 3 &&
               core->registers() == expected && core->progress() == progress;
    if (observed && accept) {
      core->write(Space::program, start + 9, 0);
      core->write(Space::program, start + last_replaced, 0);
      if (replace_next) { core->write(Space::program, start + 4, 0); }
    }
    value = word_mask;
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

void callback(const Backend backend, const bool accept, const bool replace_next) {
  Observer observer;
  observer.accept = accept;
  observer.replace_next = replace_next;
  Memory memory{.alias = true};
  std::array<Word, memory_words> p{};
  TestCore core({.program = p, .x = memory.bank(false), .y = memory.bank(true)},
                &observer,
                {.backend = backend});
  observer.core = &core;
  const auto words = program();
  core.load(Space::program, start, words);
  auto state = initial(mode_rm | mode_up);
  memory.initialize();
  core.set_registers(state);
  const auto steps = trace();
  require(core.run(static_cast<Word>(steps.size())).retired == steps.size() && observer.calls == 0,
          "Populate a proved RAM translation before changing the same-PC R to I/O");
  memory.initialize();
  auto expected_memory = memory;
  state.r[2] = io_begin;
  core.set_registers(state);
  const auto base = core.progress();
  auto expected = state;
  for (const auto step : std::span(steps).first(3)) { execute(expected, expected_memory, step); }
  observer.expected = expected;
  ++observer.expected.pc;
  observer.progress = {.instructions = base.instructions + 3,
                       .estimated_cycles = base.estimated_cycles + 3};
  const auto run = core.run(static_cast<Word>(steps.size()));
  require(observer.observed && observer.calls == 1,
          "A changed entry guard selects ordinary execution with complete callback state");
  if (!accept) {
    auto rejected = observer.expected;
    require(run.retired == 3 && run.progress == observer.progress &&
                run.fault == Fault{.kind = FaultKind::io_read,
                                   .space = Space::x,
                                   .address = io_begin,
                                   .pc = start + 3} &&
                core.registers() == rejected && memory.words == expected_memory.words,
            "Rejected ordinary loads preserve the destination and do not execute the ALU");
    require(core.run(1).retired == 0 && core.registers() == rejected &&
                core.progress() == run.progress,
            "Faulted region fallback remains stopped without replay or invented retirement");
    return;
  }
  const auto old = expected;
  register_model::write(expected, 5, word_mask);
  sequence_model::alu(expected, old, steps[3].word);
  ++expected.pc;
  for (std::size_t index = 4; index < steps.size(); ++index) {
    const bool replaced = index == 9 || index == 17 || (index == 4 && replace_next);
    execute(expected, expected_memory, replaced ? Step{.ports = {}, .count = 0} : steps[index]);
  }
  require(run.retired == steps.size() && run.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.words == expected_memory.words,
          "Callback P changes discard stale region code and preserve old parallel operands");
}

void interrupt_boundary(const Backend backend) {
  Memory memory;
  std::array<Word, memory_words> p{};
  TestCore core({.program = p, .x = memory.bank(false), .y = memory.bank(true)},
                nullptr,
                {.backend = backend});
  const auto words = program();
  core.load(Space::program, start, words);
  constexpr Word vector = 32;
  core.load(Space::program, vector, std::array{0U, 0U});
  auto expected = initial(mode_rm);
  memory.initialize();
  auto expected_memory = memory;
  core.set_registers(expected);
  const auto steps = trace();
  for (const auto step : steps) { execute(expected, expected_memory, step); }
  require(core.run(static_cast<Word>(steps.size())).retired == steps.size() &&
              core.registers() == expected && core.interrupt_ready(),
          "A completed region exposes full status/PC for host interrupt admission");
  core.interrupt(vector, 1);
  require(core.registers() == expected && memory.words == expected_memory.words &&
              !core.interrupt_ready() &&
              core.progress() ==
                  Progress{.instructions = steps.size() + 2, .estimated_cycles = steps.size() + 2},
          "Fast service preserves the canonical region checkpoint and its retirement");
  ++expected.pc;
  require(core.run(1).retired == 1 && core.registers() == expected && core.interrupt_ready(),
          "Foreground execution releases vector exclusion after a guarded region");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::admission();
    dsp56300::test::direct_matrix();
    for (const auto backend : {dsp56300::Backend::jit}) {
      for (const auto mode : dsp56300::test::modes) {
        for (const auto shape : dsp56300::test::shapes) {
          dsp56300::test::prefixes(backend, mode, shape);
          dsp56300::test::changed_offsets(backend, mode, shape);
        }
      }
      dsp56300::test::interrupt_boundary(backend);
      for (const bool accept : {false, true}) {
        for (const bool replace_next : {false, true}) {
          dsp56300::test::callback(backend, accept, replace_next);
        }
      }
    }
    std::cout << dsp56300::test::checks << " independent native-region checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
