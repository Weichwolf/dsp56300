// SPDX-License-Identifier: GPL-3.0-only
// Helper-forbidden RX execution against independent raw-width SM expectations.
#include "architecture.hpp"
#include "bfu_logic_model.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/emitter.hpp"
#include "native/direct.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

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
constexpr Word sm = saturation_model::mode, lf = Word{1} << 15, forever = Word{1} << 16;
constexpr Word short_mask = 63, alu_destination_mask = 0xf7;
constexpr unsigned extension_shift = 48;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array raw_values{std::uint64_t{0},
                                std::uint64_t{1},
                                saturation_model::positive,
                                saturation_model::positive + 1,
                                saturation_model::negative - 1,
                                saturation_model::negative,
                                sign56 - 1,
                                sign56,
                                accumulator_mask};
constexpr std::array operands{std::uint64_t{0},
                              std::uint64_t{0x800000800000},
                              std::uint64_t{0x7fffff7fffff},
                              std::uint64_t{0xffffff000001}};

struct Probe {
  unsigned calls = 0;
};

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++static_cast<Probe *>(owner)->calls;
  return 0;
}

struct Step {
  Word word = 0, extension = 0;
};

struct Shape {
  Word mode = 0;
  bool loop = false;
};

std::vector<detail::BlockInstruction> operations(const std::span<const Step> steps,
                                                 const Shape shape) {
  std::vector<detail::BlockInstruction> result;
  Word pc = start;
  for (const auto step : steps) {
    auto decoded = detail::decode(step.word);
    decoded.extension = step.extension;
    result.push_back({.instruction = decoded, .pc = pc, .mode = shape.mode | sm});
    pc += decoded.words;
  }
  if (shape.loop) {
    for (auto &operation : result) { operation.loop_address = pc - 1; }
  }
  return result;
}

std::optional<bfu_model::Input> logical_input(const Word code) {
  struct Form {
    Word code;
    bfu_model::Function function;
    unsigned source;
  };
  using enum bfu_model::Function;
  // Independent instruction coordinates: X0/X1/Y0/Y1, then unary NOT.
  constexpr std::array forms{Form{.code = 0x46, .function = bit_and, .source = 0},
                             Form{.code = 0x66, .function = bit_and, .source = 1},
                             Form{.code = 0x56, .function = bit_and, .source = 2},
                             Form{.code = 0x76, .function = bit_and, .source = 3},
                             Form{.code = 0x42, .function = bit_or, .source = 0},
                             Form{.code = 0x62, .function = bit_or, .source = 1},
                             Form{.code = 0x52, .function = bit_or, .source = 2},
                             Form{.code = 0x72, .function = bit_or, .source = 3},
                             Form{.code = 0x43, .function = bit_xor, .source = 0},
                             Form{.code = 0x63, .function = bit_xor, .source = 1},
                             Form{.code = 0x53, .function = bit_xor, .source = 2},
                             Form{.code = 0x73, .function = bit_xor, .source = 3},
                             Form{.code = 0x17, .function = complement, .source = 0}};
  for (const auto form : forms) {
    if ((code & alu_destination_mask) == form.code) {
      return bfu_model::Input{
          .function = form.function, .source = form.source, .destination_b = (code & 8U) != 0};
    }
  }
  return std::nullopt;
}

void model(Registers &state, const Step step) {
  if (step.word < parallel_none) {
    auto old = state;
    const bool extended = (step.word & 0x40U) != 0;
    old.x = extended ? step.extension & word_mask : (step.word >> 8) & short_mask;
    saturation_model::alu(state, old, 0x40U | (step.word & 15U));
    state.pc += extended ? 2U : 1U;
  } else {
    if (const auto logical = logical_input(step.word & status_byte)) {
      const auto old = state;
      bfu_model::apply(state, old, *logical);
    } else {
      saturation_model::alu(state, state, step.word);
    }
    ++state.pc;
  }
}

std::uint64_t wrapping_sum(const std::uint64_t value, const Word increment) {
  const auto room = maximum - value;
  return increment <= room ? value + increment : increment - room - 1;
}

struct Invocation {
  Registers state;
  Word budget = 1;
  bool loop = false;
};

bool admitted_ending(const Registers &state) {
  return state.lc > 1 || (state.lc == 1 && state.sc >= 2 && (state.sr & forever) == 0);
}

// DSP56300FM Rev.5, DO: the last pass executes its instruction, then restores
// the enclosing LA/LC and loop flags from two stack frames without a backedge.
void complete_loop(Registers &state) {
  constexpr Word loop_status = lf | forever;
  const auto saved_status = state.stack.at(state.sp % stack_entries);
  const auto previous = (state.sp + register_model::pointer_mask) & register_model::pointer_mask;
  const auto frame = state.stack.at(previous % stack_entries);
  state.sr = (state.sr & ~loop_status) | (static_cast<Word>(saved_status) & loop_status);
  state.la = static_cast<Word>(frame >> word_bits) & word_mask;
  state.lc = static_cast<Word>(frame) & word_mask;
  state.sp = (previous + register_model::pointer_mask) & register_model::pointer_mask;
  state.sc -= 2;
}

void check(const native::Direct &code, const std::span<const Step> steps, const Invocation input) {
  auto expected = input.state;
  Word retired = 0;
  Word cycles = 0;
  bool repeat = true;
  while (repeat && input.budget - retired >= steps.size()) {
    repeat = false;
    for (std::size_t index = 0; index < steps.size(); ++index) {
      const bool ending = input.loop && index + 1 == steps.size();
      if (ending && !admitted_ending(expected)) { break; }
      const auto before = expected.pc;
      model(expected, steps[index]);
      cycles += expected.pc - before;
      ++retired;
      if (ending) {
        if (expected.lc == 1) {
          complete_loop(expected);
        } else {
          --expected.lc;
          expected.pc = start;
          repeat = true;
        }
      }
    }
  }
  detail::Cpu cpu;
  cpu.reg = input.state;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = maximum - 2;
  cpu.cycles = maximum - 3;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  std::array<std::uint64_t, 4> latched{};
  Probe probe;
  detail::NativeContext context{.runtime = {.owner = &probe,
                                            .latched = &latched,
                                            .execute = forbidden,
                                            .moves = forbidden,
                                            .memory = {}},
                                .budget = input.budget};
  const auto count = code.enter(cpu, context);
  expected.a <<= detail::padding_bits;
  expected.b <<= detail::padding_bits;
  require(probe.calls == 0 && count == retired, "Qualified SM bytes execute without any helper");
  require(cpu.reg == expected, "Native SM preserves every independent architectural register");
  require(cpu.instructions == wrapping_sum(maximum - 2, retired) &&
              cpu.cycles == wrapping_sum(maximum - 3, cycles) &&
              cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0),
          "Native saturation preserves both wrapping progress lanes and return state");
}

Registers seed(const Word mode, const Word flags) {
  auto result = register_model::seed(mode | sm);
  result.sr = mode | sm | flags;
  return result;
}

bool admitted(const Word code) {
  constexpr std::array<Word, 27> unary_and_sums{
      0x01, 0x11, 0x13, 0x26, 0x36, 0x41, 0x51, 0x61, 0x71, 0x10, 0x14, 0x20, 0x24, 0x30,
      0x34, 0x40, 0x44, 0x50, 0x54, 0x60, 0x64, 0x70, 0x74, 0x21, 0x25, 0x31, 0x35};
  return code >= 0x80 || logical_input(code).has_value() ||
         std::ranges::find(unary_and_sums, code & alu_destination_mask) != unary_and_sums.end();
}

void rejected(const native::Direct &code, const Word mode) {
  detail::Cpu cpu;
  cpu.reg = seed(mode, status_byte);
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  const auto before = cpu.reg;
  Probe probe;
  std::array<std::uint64_t, 4> latched{};
  detail::NativeContext context{.runtime = {.owner = &probe,
                                            .latched = &latched,
                                            .execute = forbidden,
                                            .moves = forbidden,
                                            .memory = {}},
                                .budget = 1};
  require(code.enter(cpu, context) == 0 && probe.calls == 1 && cpu.reg == before &&
              cpu.instructions == 0 && cpu.cycles == 0,
          "Every other ALU byte retains full general execution before native effects");
}

void alu_matrix() {
  for (const auto mode : modes) {
    for (Word alu = 0; alu <= status_byte; ++alu) {
      const std::array steps{Step{.word = parallel_none | alu}};
      const auto ops = operations(steps, {.mode = mode});
      const detail::JitRuntime runtime{.execute = forbidden, .moves = forbidden, .memory = {}};
      detail::BlockEmitter emitter(runtime);
      const native::Direct code(emitter.generate(ops));
      if (!admitted(alu)) {
        rejected(code, mode);
        continue;
      }
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (std::size_t index = 0; index < operands.size(); ++index) {
          auto state = seed(mode, flags);
          state.a = raw_values.at((flags + index) % raw_values.size());
          state.b = raw_values.at((flags + index + 3) % raw_values.size());
          state.x = operands.at(index);
          state.y = operands.at((index + 1) % operands.size());
          check(code, steps, {.state = state});
        }
      }
    }
  }
}

void extension_matrix() {
  constexpr std::array<Word, 4> forms{0x11, 0x36, 0x40, 0xd3};
  for (const auto mode : modes) {
    for (const auto form : forms) {
      for (const Word destination : {0U, 8U}) {
        const std::array steps{Step{.word = parallel_none | form | destination}};
        const auto ops = operations(steps, {.mode = mode});
        detail::BlockEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}});
        const native::Direct code(emitter.generate(ops));
        for (Word extension = 0; extension <= status_byte; ++extension) {
          for (const Word high : {0U, sign24}) {
            auto state = seed(mode, extension);
            state.a = (std::uint64_t{extension} << extension_shift) |
                      (std::uint64_t{high} << word_bits) | word_mask;
            state.b = state.a;
            state.x = state.y = operands[1];
            check(code, steps, {.state = state});
          }
        }
      }
    }
  }
}

void sequences() {
  constexpr std::array<Step, 8> steps{
      {{.word = isa::opcode<isa::Opcode::neg_a>()[0]},
       {.word = isa::opcode<isa::Opcode::adc_x_a>()[0]},
       {.word = isa::opcode<isa::Opcode::macr_register_a>()[0] | (5U << 4)},
       {.word = isa::opcode<isa::Opcode::rnd_b>()[0]},
       {.word = isa::opcode<isa::Opcode::sub_y_a>()[0]},
       {.word = isa::opcode<isa::Opcode::macr_register_b>()[0] | (7U << 4)},
       {.word = isa::opcode<isa::Opcode::rnd_a>()[0]},
       {.word = isa::opcode<isa::Opcode::clr_b>()[0]}}};
  for (const auto mode : modes) {
    for (const bool loop : {false, true}) {
      const auto ops = operations(steps, {.mode = mode, .loop = loop});
      detail::BlockEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}});
      const native::Direct code(emitter.generate(ops));
      require(emitter.packed_retirement() && emitter.cached_status(),
              "SM mixed sequences use the packed counters and held status path");
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const Word budget : {8U, 16U, 24U}) {
          auto state = seed(mode, flags);
          state.a = saturation_model::positive + 1;
          state.b = sign56;
          state.x = state.y = operands[1];
          if (loop) {
            state.sr |= lf;
            state.la = start + static_cast<Word>(steps.size()) - 1;
            state.lc = 3;
            state.stack[state.sp % stack_entries] = std::uint64_t{start} << word_bits;
          }
          check(code, steps, {.state = state, .budget = budget, .loop = loop});
        }
      }
    }
  }
}

void immediate_case(const Step step, const Word mode) {
  const std::array steps{step};
  const auto ops = operations(steps, {.mode = mode});
  detail::BlockEmitter emitter({.execute = forbidden, .moves = forbidden, .memory = {}});
  const native::Direct code(emitter.generate(ops));
  for (Word flags = 0; flags <= status_byte; ++flags) {
    for (const auto raw : raw_values) {
      auto state = seed(mode, flags);
      state.a = raw;
      state.b = accumulator_mask - raw;
      check(code, steps, {.state = state});
    }
  }
}

Step immediate_step(const bool extended, const Word kind, const Word value) {
  constexpr Word extended_prefix = 0x0140c0;
  constexpr Word short_prefix = 0x014080;
  return {.word = (extended ? extended_prefix : short_prefix) | kind |
                  (extended ? 0 : ((value & short_mask) << 8)),
          .extension = value};
}

void immediates() {
  constexpr std::array values{0U, 1U, sign24 - 1, sign24, word_mask};
  for (const auto mode : modes) {
    for (const bool extended : {false, true}) {
      for (const Word kind : {0U, 4U, 8U, 12U}) {
        for (const auto value : values) {
          immediate_case(immediate_step(extended, kind, value), mode);
        }
      }
    }
  }
}

Registers
loop_expected(const Registers &initial, const std::span<const Step> steps, const Word budget) {
  auto result = initial;
  if (budget == 0) { return result; }
  const auto body = start + 2;
  result.stack[3] = (std::uint64_t{initial.la} << word_bits) | initial.lc;
  result.stack[4] = (std::uint64_t{body} << word_bits) | initial.sr;
  result.sp += 2;
  result.sc += 2;
  result.pc = body;
  result.sr |= lf;
  result.la = body + static_cast<Word>(steps.size()) - 1;
  result.lc = 3;
  for (Word retired = 1; retired < budget; ++retired) {
    const auto index = (retired - 1) % steps.size();
    model(result, steps[index]);
    if (index + 1 != steps.size()) { continue; }
    if (result.lc != 1) {
      --result.lc;
      result.pc = body;
    } else {
      result.la = initial.la;
      result.lc = initial.lc;
      result.sp = initial.sp;
      result.sc = initial.sc;
      result.sr &= ~lf;
    }
  }
  return result;
}

void loop_case(TestCore &core, const std::span<const Step> steps, const Invocation input) {
  const auto budget = input.budget;
  const auto expected = loop_expected(input.state, steps, budget);
  core.set_registers(input.state);
  const auto before = core.progress();

  const auto run = core.run(budget);
  require(run.retired == budget && run.fault.kind == FaultKind::none &&
              core.registers() == expected &&
              run.progress == Progress{.instructions = before.instructions + budget,
                                       .estimated_cycles = before.estimated_cycles + budget +
                                                           (budget != 0 ? 4U : 0U)},
          "Every complete SM DO prefix and final general stack cleanup preserve exact state");
}

void core_loops(const TestOptions options) {
  constexpr std::array<Step, 4> steps{
      {{.word = 0x2000d3}, {.word = 0x200029}, {.word = 0x200036}, {.word = 0x200011}}};
  constexpr Word do_three = 0x060380;
  constexpr Word budget_limit = 1 + (3 * static_cast<Word>(steps.size()));
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  TestCore core({.program = p, .x = x, .y = y}, nullptr, options);
  std::vector<Word> words{do_three, start + 1 + static_cast<Word>(steps.size())};
  for (const auto step : steps) { words.push_back(step.word); }
  core.load(Space::program, start, words);
  for (const auto mode : modes) {
    for (const Word flags : {0U, 0x55U, 0xaaU, status_byte}) {
      auto initial = seed(mode, flags);
      initial.a = saturation_model::positive + 1;
      initial.b = sign56;
      initial.x = initial.y = operands[1];
      for (Word budget = 0; budget <= budget_limit; ++budget) {
        loop_case(core, steps, {.state = initial, .budget = budget});
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::alu_matrix();
    dsp56300::test::extension_matrix();
    dsp56300::test::sequences();
    dsp56300::test::immediates();
    for (const auto backend : {dsp56300::Backend::jit}) {
      { dsp56300::test::core_loops({.backend = backend}); }
    }
    dsp56300::test::core_loops({.backend = dsp56300::Backend::jit});
    std::cout << dsp56300::test::checks << " independent native SM checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
