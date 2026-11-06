// SPDX-License-Identifier: GPL-3.0-only
// Original causal biquad cascade and independent architectural/algorithm oracles.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "iir_model.hpp"
#include "iir_program.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test::iir {
namespace {
constexpr Word input_base = 128, output_base = 512, sentinel = 0x5a36c9;
constexpr unsigned stack_pointer_mask = 63;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<Word, 3> flags{0, flag_c, flag_c | flag_s | flag_l};
constexpr std::array<Word, 3> pointers{2, 15, 63};
constexpr Word holdout_length = 129;
constexpr std::array<Word, 4> lengths{1, 7, 32, holdout_length};

struct Input {
  Word samples = 1, mode = 0;
  unsigned sections = 1, signal = 0, design = 0, context = 0;
  bool scheduled = false;
};

struct Data {
  std::vector<Word> input, output, state, coefficients;
  std::vector<Section> mathematical;
  std::vector<Coefficients> weights;
  Word input_address = 0, output_address = 0;
};

Data data(const Input input) {
  constexpr Word output_stride = 5;
  Data result;
  result.input_address = input_base + (input.context * 3);
  result.output_address =
      input.context == 1 ? result.input_address : output_base + (input.context * output_stride);
  result.input.resize(input.samples + 2, sentinel);
  for (Word index = 0; index < input.samples; ++index) {
    result.input[index + 1] = signal(static_cast<Signal>(input.signal), index);
  }
  result.output =
      input.context == 1 ? result.input : std::vector<Word>(input.samples + 2, sentinel);
  for (unsigned section = 0; section < input.sections; ++section) {
    result.weights.push_back(designs[(input.design + section) % designs.size()]);
    for (const auto weight : result.weights.back()) {
      result.coefficients.push_back(static_cast<Word>(weight) & word_mask);
    }
    const auto state = seed({.context = input.context, .section = section});
    result.mathematical.push_back(state);
    result.state.insert(result.state.end(), {state.first, state.second});
  }
  return result;
}

void setup(BinaryCase &item, const Input input, const Data &memory) {
  item.initial = register_model::seed(input.mode);
  item.initial.sr = input.mode | flags.at(input.context);
  item.initial.sp = pointers.at(input.context);
  item.initial.r[0] = memory.input_address;
  item.initial.r[1] = memory.output_address;
  item.initial.m[0] = item.initial.m[1] = word_mask;
  item.expected = item.initial;
  item.before = {{.words = memory.input, .address = memory.input_address - 1, .space = Space::x},
                 {.words = memory.state, .address = state_base, .space = Space::x},
                 {.words = memory.coefficients, .address = coefficient_base, .space = Space::y}};
  if (memory.input_address != memory.output_address) {
    item.before.push_back(
        {.words = memory.output, .address = memory.output_address - 1, .space = Space::x});
  }
  item.before.push_back({.words = {sentinel}, .address = state_base - 1, .space = Space::x});
  item.before.push_back({.words = {sentinel},
                         .address = state_base + static_cast<Word>(memory.state.size()),
                         .space = Space::x});
  item.before.push_back({.words = {sentinel}, .address = coefficient_base - 1, .space = Space::y});
  item.before.push_back(
      {.words = {sentinel},
       .address = coefficient_base + static_cast<Word>(memory.coefficients.size()),
       .space = Space::y});
}

void begin_loop(BinaryCase &item, const Input input, const Word body_size) {
  auto &state = item.expected;
  state.stack[(state.sp + 1) % stack_entries] = (std::uint64_t{state.la} << word_bits) | state.lc;
  state.stack[(state.sp + 2) % stack_entries] = (std::uint64_t{start + 2} << word_bits) | state.sr;
  state.sp = (state.sp + 2) & stack_pointer_mask;
  state.sc += 2;
  state.la = start + 1 + body_size;
  state.lc = input.samples;
  state.sr |= loop_flag;
  state.pc = start + 2;
}

void end_sample(BinaryCase &item) {
  auto &state = item.expected;
  if (state.lc > 1) {
    --state.lc;
    state.pc = start + 2;
    return;
  }
  state.sr &= ~loop_flag;
  state.la = item.initial.la;
  state.lc = item.initial.lc;
  state.sp = item.initial.sp;
  state.sc = item.initial.sc;
}

void step(Registers &state, Data &memory, const Step instruction, const Word sample) {
  const auto old = state;
  switch (instruction.action) {
    case Action::input:
      register_model::write(state, 4, memory.input.at(sample + 1));
      ++state.r[0];
      break;
    case Action::output:
      memory.output.at(sample + 1) = register_model::read(state, 4);
      ++state.r[1];
      break;
    case Action::state_load:
      register_model::write(state, instruction.reg, memory.state.at(instruction.slot));
      break;
    case Action::state_store:
      memory.state.at(instruction.slot) = register_model::read(state, 14);
      break;
    case Action::coefficient:
      register_model::write(state, 6, memory.coefficients.at(instruction.slot));
      break;
    case Action::move: {
      const auto value = register_model::read(state, instruction.slot);
      register_model::write(state, instruction.reg, value);
      break;
    }
    case Action::alu: break;
    default: throw std::logic_error("Unknown IIR trace action");
  }
  sequence_model::alu(state, old, instruction.word);
  ++state.pc;
}

std::vector<Word> mathematical_sample(Data &memory, const Word index, const Input input) {
  std::vector<Word> outputs;
  auto value = memory.input.at(index + 1);
  for (std::size_t section = 0; section < memory.mathematical.size(); ++section) {
    value = sample(memory.mathematical[section], value, memory.weights[section], input.mode);
    outputs.push_back(value);
  }
  return outputs;
}

void check_section(const Registers &state,
                   const Data &memory,
                   const std::vector<Word> &outputs,
                   const std::size_t section) {
  require((static_cast<Word>(state.x) & word_mask) == outputs.at(section) &&
              memory.state.at(2 * section) == memory.mathematical.at(section).first &&
              memory.state.at((2 * section) + 1) == memory.mathematical.at(section).second,
          "Every complete biquad stage agrees with an independent algebraic quantized recurrence");
}

void trace(BinaryCase &item, Data &memory, const Input input, const std::vector<Step> &steps) {
  if (item.budget == 0) { return; }
  begin_loop(item, input, static_cast<Word>(steps.size()));
  Word retired = 1;
  for (Word index = 0; index < input.samples && retired < item.budget; ++index) {
    const auto outputs = mathematical_sample(memory, index, input);
    unsigned section = 0;
    for (std::size_t pc = 0; pc < steps.size() && retired < item.budget; ++pc) {
      step(item.expected, memory, steps[pc], index);
      // Each executed short state store adds one clock beyond the one-clock body baseline.
      if (steps[pc].action == Action::state_store) { ++item.progress.estimated_cycles; }
      if (steps[pc].completes_section) {
        check_section(item.expected, memory, outputs, section);
        ++section;
      }
      ++retired;
      if (pc + 1 == steps.size()) { end_sample(item); }
    }
  }
}

BinaryCase fixture(const Input input, const std::optional<Word> requested = std::nullopt) {
  BinaryCase item;
  const auto steps = body({.sections = input.sections, .scheduled = input.scheduled});
  auto memory = data(input);
  setup(item, input, memory);
  const auto total = 1 + (input.samples * static_cast<Word>(steps.size()));
  item.budget = requested.value_or(total);
  if (item.budget > total) { throw std::logic_error("An IIR prefix cannot run beyond completion"); }
  item.retired = item.budget;
  item.progress = {.instructions = item.budget,
                   .estimated_cycles = item.budget == 0 ? 0U : item.budget + 4};
  item.name = "Original causal biquad samples=" + std::to_string(input.samples) +
              " sections=" + std::to_string(input.sections) +
              " signal=" + std::to_string(input.signal) +
              " design=" + std::to_string(input.design) +
              " context=" + std::to_string(input.context) + " mode=" + std::to_string(input.mode) +
              " scheduled=" + std::to_string(static_cast<unsigned>(input.scheduled)) +
              " budget=" + std::to_string(item.budget);
  item.before.push_back({.words = program(input.samples, steps), .address = start});
  trace(item, memory, input, steps);
  item.after = {
      {.words = std::move(memory.state), .address = state_base, .space = Space::x},
      {.words = std::move(memory.output), .address = memory.output_address - 1, .space = Space::x}};
  return item;
}

void modes_and_contexts(std::vector<BinaryCase> &cases, Input input) {
  for (const auto mode : modes) {
    input.mode = mode;
    for (unsigned context = 0; context < flags.size(); ++context) {
      input.context = context;
      cases.push_back(fixture(input));
    }
  }
}

void signals_and_designs(std::vector<BinaryCase> &cases, Input input) {
  for (unsigned signal = 0; signal < 6; ++signal) {
    input.signal = signal;
    for (unsigned design = 0; design < designs.size(); ++design) {
      input.design = design;
      modes_and_contexts(cases, input);
    }
  }
}

void prefixes(std::vector<BinaryCase> &cases, Input input) {
  const auto size =
      static_cast<Word>(body({.sections = input.sections, .scheduled = input.scheduled}).size());
  const std::array budgets{0U,
                           1U,
                           2U,
                           3U,
                           4U,
                           5U,
                           6U,
                           7U,
                           8U,
                           14U,
                           15U,
                           21U,
                           22U,
                           23U,
                           24U,
                           size,
                           size + 1,
                           size + 2,
                           2 * size,
                           (2 * size) + 1,
                           (2 * size) + 2,
                           3 * size,
                           (3 * size) + 1};
  for (const auto mode : modes) {
    input.mode = mode;
    for (unsigned context = 0; context < flags.size(); ++context) {
      input.context = context;
      for (const auto budget : budgets) { cases.push_back(fixture(input, budget)); }
    }
  }
}

void generate(const char *const output) {
  std::vector<BinaryCase> cases;
  for (const bool scheduled : {false, true}) {
    for (const auto samples : lengths) {
      for (const unsigned sections : {1U, 2U, 4U}) {
        signals_and_designs(cases,
                            {.samples = samples, .sections = sections, .scheduled = scheduled});
      }
    }
    for (const unsigned signal : {3U, 5U}) {
      for (unsigned design = 0; design < designs.size(); ++design) {
        prefixes(cases,
                 {.samples = 3,
                  .sections = 2,
                  .signal = signal,
                  .design = design,
                  .scheduled = scheduled});
      }
    }
  }
  const auto count = cases.size();
  write_fixture(output, std::move(cases));
  std::cout << count << " independently modeled original IIR cases written; " << checks
            << " algebraic stage checks\n";
}
} // namespace
} // namespace dsp56300::test::iir

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_iir OUTPUT.d56"); }
    dsp56300::test::iir::generate(argv[1]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
