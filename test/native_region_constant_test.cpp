// SPDX-License-Identifier: GPL-3.0-only
// Independent constant-address traces and helper-forbidden guarded execution.
#include "address_model.hpp"
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native/direct.hpp"
#include "native_region.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"
#include "translation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::size_t ram_words = 128, short_x_words = 96, short_y_words = 120;
constexpr Word io_begin = 0xffff80, lf = Word{1} << 15;
constexpr Word low_modifier = 0xffff, single_prefix = 0x404000, immediate_prefix = 0x40f400;
constexpr Word r_base = 16, n_base = 24, high_selector = 24;
constexpr unsigned high_shift = 17;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<Word, 8> alu{0xd3, 0x10, 0x19, 0x36, 0xf3, 0x48, 0xd2, 0x11};
constexpr std::array<unsigned, 8> registers{4, 14, 6, 15, 5, 4, 7, 14};

struct Shape {
  Word value = 32, modifier = word_mask, mode = 0;
  unsigned index = 0, address_mode = 3;
  bool loop = false;
};

struct Port {
  unsigned reg = 4, index = 0, mode = 3;
  bool y = false, load = true;
};

struct Step {
  Word word = 0, value = 0;
  unsigned destination = 0;
  std::optional<Port> port;
};

Word constant_word(const unsigned selector) {
  return immediate_prefix | ((selector & 7U) << 16) | ((selector & high_selector) << high_shift);
}

std::vector<Step> trace(const Shape shape) {
  const auto destination = r_base + shape.index;
  std::vector<Step> result{{.word = constant_word(destination),
                            .value = shape.value,
                            .destination = destination,
                            .port = {}}};
  for (unsigned index = 0; index < alu.size(); ++index) {
    const Port port{.reg = registers[index],
                    .index = shape.index,
                    .mode = shape.address_mode,
                    .y = index % 4 == 1 || index % 4 == 2,
                    .load = index % 2 == 0};
    result.push_back({.word = single_prefix | ((port.reg & 7U) << 16) |
                              ((port.reg & high_selector) << high_shift) | (port.mode << 11) |
                              (port.index << 8) | (port.y ? 0x080000U : 0U) |
                              (port.load ? 0x8000U : 0U) | alu[index],
                      .port = port});
  }
  return result;
}

Word words(const Step step) {
  return step.port ? 1U : 2U;
}

void number(std::vector<detail::NativeOp> &ops, const bool loop) {
  Word pc = start;
  for (auto &operation : ops) {
    operation.pc = pc;
    pc += operation.instruction.words;
  }
  for (auto &operation : ops) { operation.loop_address = loop ? pc - 1 : detail::inactive_loop; }
}

std::vector<detail::NativeOp> operations(const Shape shape) {
  std::vector<detail::NativeOp> result;
  for (const auto step : trace(shape)) {
    auto decoded = detail::decode(step.word);
    decoded.extension = step.value;
    require(decoded.words == words(step) && decoded.cycles == words(step),
            "Constant and RAM encodings retain their independently declared length and cost");
    result.push_back({.instruction = decoded, .mode = shape.mode});
  }
  number(result, shape.loop);
  return result;
}

struct Memory {
  std::array<Word, ram_words * 2> data{};
  std::array<std::size_t, 2> limits{ram_words, ram_words};
  bool alias = false;

  [[nodiscard]] std::span<Word> bank(const bool y) {
    return std::span(data).subspan(y && !alias ? ram_words : 0, limits[y ? 1 : 0]);
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
  return {{{.data = memory.bank(false).data(), .words = memory.limits[0]},
           {.data = memory.bank(true).data(), .words = memory.limits[1]}}};
}

std::array<Word, agu_registers> modifiers(const Shape shape) {
  std::array<Word, agu_registers> result{};
  result.fill(word_mask);
  result[shape.index] = shape.modifier;
  return result;
}

void execute(Registers &state, Memory &memory, const Step step) {
  if (!step.port) {
    register_model::write(state, step.destination, step.value & word_mask);
  } else {
    const auto old = state;
    const auto port = *step.port;
    const auto address = state.r[port.index];
    state.r[port.index] = address_model::updated({.modifier = state.m[port.index],
                                                  .cursor = address,
                                                  .offset = state.n[port.index],
                                                  .mode = port.mode});
    if (port.load) {
      register_model::write(state, port.reg, memory.bank(port.y)[address] & word_mask);
    } else {
      memory.bank(port.y)[address] = register_model::read(state, port.reg);
    }
    if ((state.sr & saturation_model::mode) != 0) {
      saturation_model::alu(state, old, step.word);
    } else {
      sequence_model::alu(state, old, step.word);
    }
  }
  state.pc += words(step);
}

bool admitted(const Registers &state, const Shape shape, const Memory &memory) {
  if (shape.loop && state.lc <= 1) { return false; }
  if (shape.address_mode != 4 &&
      (state.m[shape.index] & low_modifier) != (shape.modifier & low_modifier)) {
    return false;
  }
  if (shape.address_mode == 1 && state.n[shape.index] >= sign24) { return false; }
  Word cursor = shape.value & word_mask;
  if (shape.address_mode == 3 && shape.modifier != word_mask) {
    const Word span = shape.modifier == 7 ? 8U : 32U;
    const auto base = (cursor / span) * span;
    const auto end = base + shape.modifier + 1;
    if (cursor >= end || end > std::min(memory.limits[0], memory.limits[1])) { return false; }
  }
  for (const auto step : trace(shape)) {
    if (!step.port) { continue; }
    const auto port = *step.port;
    if (cursor >= memory.limits[port.y ? 1 : 0]) { return false; }
    cursor = address_model::updated({.modifier = state.m[shape.index],
                                     .cursor = cursor,
                                     .offset = state.n[shape.index],
                                     .mode = port.mode});
  }
  return true;
}

struct Probe {
  unsigned calls = 0;
};

std::uint32_t forbidden(void *const owner, const detail::NativeOp * /*operation*/) noexcept {
  ++static_cast<Probe *>(owner)->calls;
  return 0;
}

std::uint64_t wrapping_sum(const std::uint64_t value, const Word increment) {
  const auto room = maximum - value;
  return increment <= room ? value + increment : increment - room - 1;
}

struct Input {
  Word flags = 0, old_r = io_begin, offset = 1, lc = 4;
  bool mismatch = false;
};

Registers initial(const Shape shape, const Input input) {
  auto state = register_model::seed(shape.mode);
  state.sr = shape.mode | input.flags | (shape.loop ? lf : 0);
  state.r[shape.index] = input.old_r;
  state.n[shape.index] = input.offset;
  state.m = modifiers(shape);
  if (input.mismatch) { state.m[shape.index] ^= 1; }
  state.lc = input.lc;
  state.la = start + 9;
  state.stack[state.sp % stack_entries] = std::uint64_t{start} << word_bits;
  return state;
}

void direct_case(const native::Direct &code, Memory &memory, const Shape shape, const Input input) {
  const auto steps = trace(shape);
  constexpr Word body_size = 9;
  constexpr Word budget = 3 * body_size;
  memory.initialize();
  auto expected_memory = memory;
  auto expected = initial(shape, input);
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = maximum - 2;
  cpu.cycles = maximum - 3;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  Word retired = 0;
  Word bodies = 0;
  while (budget - retired >= body_size && admitted(expected, shape, memory)) {
    for (const auto step : steps) { execute(expected, expected_memory, step); }
    retired += body_size;
    ++bodies;
    if (!shape.loop) { break; }
    --expected.lc;
    expected.pc = start;
  }
  Probe probe;
  std::uint64_t transfers = 0;
  detail::NativeContext context{.runtime = {.owner = &probe, .memory = banks(memory)},
                                .native_moves = &transfers,
                                .budget = budget};
  const auto result = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(result == retired && probe.calls == 0 && cpu.reg == expected &&
              memory.data == expected_memory.data,
          "Constant-before-use guarded bytes preserve every register, old operand and RAM alias");
  require(
      cpu.instructions == wrapping_sum(maximum - 2, retired) &&
          cpu.cycles == wrapping_sum(maximum - 3, retired + bodies) && transfers == retired &&
          cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0),
      "Every initial/header rejection is read-only; constant retirement and costs wrap exactly");
}

void direct_shape(const Shape shape, const bool alias, const bool shortened) {
  Memory memory{.alias = alias};
  if (shortened) { memory.limits = {short_x_words, short_y_words}; }
  const auto ops = operations(shape);
  const detail::JitRuntime runtime{
      .execute = forbidden, .moves = forbidden, .memory = banks(memory)};
  const auto plan = detail::plan_native_region(ops, runtime.memory, modifiers(shape));
  require(plan.valid && plan.addresses[shape.index].constant == (shape.value & word_mask),
          "The proof records the actual 24-bit assignment instead of constraining old R");
  detail::JitEmitter emitter(
      runtime, true, BatchBackend::disabled, modifiers(shape), detail::RegionMode::guarded);
  const native::Direct code(emitter.generate(ops));
  require(emitter.guarded_region() && emitter.cached_status() && emitter.packed_retirement(),
          "The tested bytes are actual helper-free guarded regions with held status");
  for (Word flags = 0; flags <= status_byte; ++flags) {
    direct_case(code, memory, shape, {.flags = flags});
  }
  for (const Word old_r : {0U, 127U, io_begin, word_mask, 0xffffffffU}) {
    for (const Word offset : {0U, 1U, 2U, sign24 - 1, sign24, word_mask}) {
      for (const Word lc : {0U, 1U, 2U, 4U}) {
        direct_case(code,
                    memory,
                    shape,
                    {.flags = status_byte, .old_r = old_r, .offset = offset, .lc = lc});
      }
    }
  }
  direct_case(code, memory, shape, {.flags = status_byte, .mismatch = true});
}

void direct_mode(const Word mode) {
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (const bool loop : {false, true}) {
      for (const bool alias : {false, true}) {
        for (const bool shortened : {false, true}) {
          direct_shape({.mode = mode, .index = index, .loop = loop}, alias, shortened);
        }
      }
    }
  }
}

void direct_matrix() {
  for (const auto mode : modes) {
    direct_mode(mode);
    direct_mode(mode | saturation_model::mode);
  }
}

void boundary_shapes() {
  for (const Word value :
       {0U, 32U, 95U, 96U, 115U, 120U, 121U, 127U, 128U, io_begin, word_mask, 0xff000020U}) {
    for (const bool loop : {false, true}) {
      for (const bool alias : {false, true}) {
        for (const unsigned address_mode : {1U, 3U, 4U}) {
          direct_shape({.value = value,
                        .mode = saturation_model::mode | mode_up,
                        .index = 4,
                        .address_mode = address_mode,
                        .loop = loop},
                       alias,
                       true);
        }
        for (const Word modifier : {7U, 19U}) {
          direct_shape(
              {.value = value, .modifier = modifier, .mode = mode_rm, .index = 7, .loop = loop},
              alias,
              true);
        }
      }
    }
  }
}

void admission() {
  Memory memory;
  auto ops = operations({});
  const auto mods = modifiers({});
  auto changed = ops;
  changed[0].instruction = detail::decode(constant_word(n_base));
  require(!detail::plan_native_region(changed, banks(memory), mods).valid,
          "Rewriting N cannot silently reuse a stride proof based on its initial value");
  std::swap(changed[0], changed[1]);
  changed[1] = ops[0];
  number(changed, false);
  require(!detail::plan_native_region(changed, banks(memory), mods).valid,
          "A constant after the first use retains ordinary execution");
  changed = ops;
  changed.insert(changed.begin(), ops[0]);
  changed.front().instruction.extension = 64;
  number(changed, false);
  const auto result = detail::plan_native_region(changed, banks(memory), mods);
  require(result.valid && result.addresses[0].constant == 32,
          "Consecutive pre-use constants use the last actual assignment");
  auto alias = banks(memory);
  alias[0].aliases_program = true;
  require(!detail::plan_native_region(ops, alias, mods).valid,
          "Constants do not remove program-alias invalidation boundaries");
}

void api_case(Core &core, Memory &memory, const Shape shape, const Word budget) {
  memory.initialize();
  auto expected_memory = memory;
  auto expected = initial(shape, {.flags = status_byte});
  core.set_registers(expected);
  Word cycles = 0;
  const auto steps = trace(shape);
  for (const auto step : std::span(steps).first(budget)) {
    execute(expected, expected_memory, step);
    cycles += words(step);
  }
  const auto before = core.progress();
  const auto run = core.run(budget);
  require(run.dispatches == budget && run.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.data == expected_memory.data &&
              run.progress == Progress{.instructions = before.instructions + budget,
                                       .estimated_cycles = before.estimated_cycles + cycles},
          "Every warm API prefix and same-PC P-extension change preserves full independent state");
}

void api_prefixes(const Options options, const bool alias) {
  Memory memory{.alias = alias};
  std::array<Word, memory_words> program_memory{};
  Core core({.program = program_memory, .x = memory.bank(false), .y = memory.bank(true)},
            nullptr,
            options);
  constexpr std::array selected_modes{0U,
                                      mode_rm | mode_up,
                                      saturation_model::mode,
                                      saturation_model::mode | mode_down,
                                      saturation_model::mode | mode_up};
  for (const auto mode : selected_modes) {
    Shape shape{.mode = mode, .index = 4};
    std::vector<Word> program_words;
    for (const auto step : trace(shape)) {
      program_words.push_back(step.word);
      if (!step.port) { program_words.push_back(step.value); }
    }
    core.load(Space::program, start, program_words);
    api_case(core, memory, shape, 9);
    for (const Word value : {32U, 64U, 80U, 32U}) {
      shape.value = value;
      core.write(Space::program, start + 1, value);
      const auto expected_program = program_memory;
      for (Word budget = 0; budget <= 9; ++budget) { api_case(core, memory, shape, budget); }
      require(program_memory == expected_program,
              "Constant specialization never writes P or its neighboring canaries");
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::admission();
    dsp56300::test::direct_matrix();
    dsp56300::test::boundary_shapes();
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      for (const auto limit : {1U, 4U, 64U}) {
        for (const bool alias : {false, true}) {
          dsp56300::test::api_prefixes({.backend = backend, .instructions_per_block = limit},
                                       alias);
        }
      }
    }
    dsp56300::test::api_prefixes({.backend = dsp56300::Backend::jit, .optimize = false}, true);
    std::cout << dsp56300::test::checks << " independent constant-region checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
