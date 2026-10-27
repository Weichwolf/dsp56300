// SPDX-License-Identifier: GPL-3.0-only
// Independent PC coordinates, serial AGU arithmetic and raw register destinations.
#include "address_model.hpp"
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
enum class Form : std::uint8_t { relative_register, relative_long, updated, displacement };
constexpr Word lra_register = 0x04c000, lra_long = 0x044040;
constexpr Word lua_updated = 0x044000, lua_displacement = 0x040000;
constexpr Word ignored_modifier = 0xab8000, lf = Word{1} << 15, fv = Word{1} << 16;
constexpr Word multiple_modifier = 0x800f;
constexpr Word repeat_base = 0x0600a0, pointer_range = 64;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array addresses{
    0U, 1U, 63U, sign24 - 1, sign24, word_mask - 1, word_mask, 0xffff80U};

struct Input {
  address_model::Input update{};
  Form form = Form::relative_register;
  Word destination = 4, status = 0, pc = start;
};

Word opcode(const Input input) {
  const auto &update = input.update;
  const Word source = update.index << 8;
  switch (input.form) {
    case Form::relative_register: return lra_register | source | input.destination;
    case Form::relative_long: return lra_long | input.destination;
    case Form::updated: return lua_updated | (update.mode << 11) | source | input.destination;
    case Form::displacement: {
      const Word bits = update.offset % 128;
      constexpr Word high = 0x70;
      return lua_displacement | ((bits & high) << 7) | source | ((bits & 15U) << 4) |
             (input.destination - 16);
    }
    default: throw std::runtime_error("Unknown address-calculation form");
  }
}

void model(Registers &state, const Input input) {
  const auto index = input.update.index;
  Word result = 0;
  if (input.form == Form::relative_register || input.form == Form::relative_long) {
    const auto offset =
        input.form == Form::relative_register ? state.r.at(index) : input.update.offset;
    result = address_model::wrap24(input.pc + signed24(offset));
  } else {
    auto update = input.update;
    update.cursor = state.r.at(index);
    update.modifier = state.m.at(index);
    if (input.form == Form::updated) {
      update.offset = state.n.at(index);
    } else {
      update.mode = 1;
    }
    result = address_model::updated(update);
  }
  register_model::write(state, input.destination, result);
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  item.initial.pc = input.pc;
  const auto index = input.update.index;
  item.initial.r.at(index) = input.update.cursor;
  constexpr Word ignored_offset = 0xa54321;
  item.initial.n.at(index) =
      input.form == Form::displacement ? ignored_offset : input.update.offset;
  item.initial.m.at(index) = input.update.modifier;
  item.expected = item.initial;
  model(item.expected, input);
  const bool long_form = input.form == Form::relative_long;
  item.before = {{.words = {opcode(input)}, .address = input.pc}};
  if (long_form) {
    item.before.push_back({.words = {input.update.offset}, .address = (input.pc + 1) & word_mask});
  }
  if (input.pc == word_mask) { item.memory_words[0] = static_cast<Word>(address_words); }
  item.expected.pc = (input.pc + (long_form ? 2U : 1U)) & word_mask;
  item.progress.estimated_cycles = 3;
  item.name = "Address calculation form=" + std::to_string(static_cast<unsigned>(input.form)) +
              "; source=" + std::to_string(index) +
              "; destination=" + std::to_string(input.destination);
  return item;
}

void relative_statuses(Cases &cases, Input input) {
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      input.status = mode | flags;
      cases.push_back(single(input));
    }
  }
}

void relative_cases(Cases &cases) {
  for (Word destination = 4; destination < 32; ++destination) {
    for (const Word value : addresses) {
      relative_statuses(cases,
                        {.update = {.modifier = ignored_modifier, .offset = value},
                         .form = Form::relative_long,
                         .destination = destination});
      for (unsigned index = 0; index < agu_registers; ++index) {
        relative_statuses(cases,
                          {.update = {.modifier = ignored_modifier,
                                      .cursor = value,
                                      .offset = sign24,
                                      .index = index},
                           .destination = destination});
      }
    }
  }
}

constexpr std::array specs{
    address_model::Input{.cursor = word_mask, .offset = 1},
    address_model::Input{.cursor = 0, .offset = word_mask},
    address_model::Input{.cursor = sign24, .offset = sign24},
    address_model::Input{.modifier = 0, .cursor = 1, .offset = sign24},
    address_model::Input{.modifier = 0, .cursor = 0x400000, .offset = 2},
    address_model::Input{.modifier = 3, .cursor = 1023, .offset = 3},
    address_model::Input{.modifier = 6, .cursor = 1022, .offset = 3},
    address_model::Input{.modifier = 15, .cursor = 1023, .offset = 16},
    address_model::Input{.modifier = 15, .cursor = 1023, .offset = 32},
    address_model::Input{.modifier = 0xab000f, .cursor = 1008, .offset = word_mask},
    address_model::Input{.modifier = 0x800f, .cursor = 1023, .offset = sign24 - 1},
    address_model::Input{.modifier = 0x8003, .cursor = 1008, .offset = sign24},
    address_model::Input{.modifier = 0xbfff, .cursor = word_mask, .offset = 1},
    address_model::Input{.modifier = 0, .cursor = word_mask, .offset = word_mask}};

void updated_destinations(Cases &cases, address_model::Input update) {
  for (Word destination = 4; destination < 32; ++destination) {
    const auto variant = destination + update.index + update.mode + update.modifier;
    const auto status = modes.at(variant % modes.size()) | (variant % 2 == 0 ? status_byte : 0U);
    cases.push_back(single(
        {.update = update, .form = Form::updated, .destination = destination, .status = status}));
  }
}

void updated_cases(Cases &cases) {
  for (auto update : specs) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      update.index = index;
      for (unsigned mode = 0; mode < 4; ++mode) {
        update.mode = mode;
        updated_destinations(cases, update);
      }
    }
  }
}

void displacement_destinations(Cases &cases, const address_model::Input source) {
  constexpr std::array modifiers{0xffffU, 0U, 15U, 0x800fU};
  for (Word destination = 16; destination < 32; ++destination) {
    auto update = source;
    update.modifier = modifiers.at((destination + source.index) % modifiers.size());
    const auto signed_offset = signed24(update.offset);
    const auto magnitude = signed_offset < 0 ? -signed_offset : signed_offset;
    if (update.modifier == 15 && magnitude > 16 && magnitude % 16 != 0) {
      update.modifier = address_model::linear;
    }
    update.cursor = addresses.at((destination + source.index) % addresses.size());
    if (update.modifier == 15 || update.modifier == multiple_modifier) {
      constexpr Word base = 1008;
      update.cursor = base + (destination + source.index) % 16;
    }
    const auto selector = update.offset + destination + source.index;
    const auto status = modes.at(selector % modes.size()) | (selector % 2 == 0 ? status_byte : 0U);
    cases.push_back(single({.update = update,
                            .form = Form::displacement,
                            .destination = destination,
                            .status = status}));
  }
}

void displacement_cases(Cases &cases) {
  for (int offset = -64; offset < 64; ++offset) {
    const auto raw = address_model::wrap24(offset);
    for (unsigned index = 0; index < agu_registers; ++index) {
      displacement_destinations(cases, {.offset = raw, .mode = 1, .index = index});
    }
  }
}

BinaryCase repeated(Input input) {
  constexpr Word count = 7;
  auto item = single(input);
  item.expected = item.initial;
  input.pc = start + 1;
  for (Word index = 0; index < count; ++index) { model(item.expected, input); }
  item.expected.pc = start + 2;
  item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
  item.budget = item.dispatches = count + 1;
  item.progress = {.instructions = count + 1, .estimated_cycles = 5 + (3 * count)};
  item.name += "; REP recomputes current source and PC with explicit destination alias";
  return item;
}

struct Loop {
  Word pointer = 0, count = 1, outer = 0;
};

BinaryCase ending(const Input input, const Loop frame) {
  constexpr Word saved_la = 0x70;
  constexpr Word saved_lc = 7;
  auto item = single(input);
  auto &initial = item.initial;
  initial.sr |= lf;
  initial.la = item.expected.pc - 1;
  initial.lc = frame.count;
  initial.sp = frame.pointer;
  initial.sc = 2;
  initial.stack[frame.pointer % stack_entries] =
      (std::uint64_t{start} << word_bits) | frame.outer | flag_v;
  initial.stack[(frame.pointer + pointer_range - 1) % stack_entries] =
      (std::uint64_t{saved_la} << word_bits) | saved_lc;
  item.expected = initial;
  model(item.expected, input);
  item.expected.pc = frame.count > 1 ? start : initial.la + 1;
  if (frame.count > 1) {
    --item.expected.lc;
  } else {
    item.expected.la = saved_la;
    item.expected.lc = saved_lc;
    item.expected.sp = (frame.pointer + pointer_range - 2) % pointer_range;
    item.expected.sc = 0;
    item.expected.sr = (item.expected.sr & ~(lf | fv)) | frame.outer;
  }
  item.name += "; DO ending and saved loop flags";
  return item;
}

void interactions(Cases &cases) {
  const std::array inputs{
      Input{.update = {.cursor = 0x180, .offset = 1}, .destination = 16},
      Input{.update = {.offset = word_mask}, .form = Form::relative_long, .destination = 15},
      Input{.update = {.cursor = 0x180, .offset = 1, .mode = 1},
            .form = Form::updated,
            .destination = 16},
      Input{
          .update = {.cursor = 0x180, .offset = 7}, .form = Form::displacement, .destination = 24}};
  for (const auto input : inputs) {
    if (input.form != Form::relative_long) { cases.push_back(repeated(input)); }
    for (const Word pointer : {0U, 15U, 31U, 63U}) {
      for (const Word count : {1U, 2U}) {
        for (const Word outer : {0U, lf, fv, lf | fv}) {
          cases.push_back(ending(input, {.pointer = pointer, .count = count, .outer = outer}));
        }
      }
    }
  }
  cases.push_back(single({.update = {.offset = 1}, .form = Form::relative_long, .pc = word_mask}));
}

enum class Stage : std::uint8_t { alu, calculation, memory };

struct Step {
  Input calculation{};
  Word instruction = 0;
  Stage stage = Stage::alu;
};

void sequence_step(BinaryCase &item, const Step step) {
  if (step.stage == Stage::calculation) {
    auto input = step.calculation;
    input.pc = item.expected.pc;
    model(item.expected, input);
    item.expected.pc += input.form == Form::relative_long ? 2U : 1U;
    item.progress.estimated_cycles += 3;
  } else {
    if (step.stage == Stage::memory) {
      constexpr Word datum = 0x456789;
      item.expected.x = (item.expected.x & ~std::uint64_t{word_mask}) | datum;
    } else {
      const auto previous = item.expected;
      sequence_model::alu(item.expected, previous, step.instruction);
    }
    ++item.expected.pc;
    ++item.progress.estimated_cycles;
  }
  ++item.budget;
  item.dispatches = item.budget;
  item.progress.instructions = item.budget;
}

void sequence_prefixes(Cases &cases) {
  constexpr Word mac_a = 0x2000d2;
  constexpr Word add_b_a = 0x200010;
  constexpr Word adc_x_a = 0x200021;
  constexpr Word load_x0 = 0x44e000;
  constexpr Word datum = 0x456789;
  constexpr Word data_location = 0x181;
  const std::array steps{
      Step{.instruction = mac_a},
      Step{.calculation = {.destination = 15}, .stage = Stage::calculation},
      Step{.instruction = add_b_a},
      Step{.calculation = {.update = {.mode = 3}, .form = Form::updated, .destination = 16},
           .stage = Stage::calculation},
      Step{.instruction = load_x0, .stage = Stage::memory},
      Step{.instruction = mac_a},
      Step{.calculation = {.update = {.mode = 1, .index = 1},
                           .form = Form::updated,
                           .destination = 4},
           .stage = Stage::calculation},
      Step{.instruction = mac_a},
      Step{.calculation = {.update = {.offset = sign24 + 1},
                           .form = Form::relative_long,
                           .destination = 8},
           .stage = Stage::calculation},
      Step{.instruction = adc_x_a},
      Step{.calculation = {.update = {.offset = 7, .index = 1},
                           .form = Form::displacement,
                           .destination = 24},
           .stage = Stage::calculation},
      Step{.instruction = add_b_a}};
  std::vector<Word> program;
  program.reserve(steps.size() * 2);
  for (const auto step : steps) {
    program.push_back(step.stage == Stage::calculation ? opcode(step.calculation)
                                                       : step.instruction);
    if (step.stage == Stage::calculation && step.calculation.form == Form::relative_long) {
      program.push_back(step.calculation.update.offset);
    }
  }
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      BinaryCase item;
      item.initial = register_model::seed(mode | flags);
      item.initial.sr = mode | flags;
      item.expected = item.initial;
      item.budget = 0;
      item.progress = {};
      item.before = {{.words = program, .address = start},
                     {.words = {datum}, .address = data_location, .space = Space::x}};
      item.name = "Native ALU/address helper/native transfer prefixes; fresh R/X/Y and "
                  "partial/full A/B writes";
      for (const auto step : steps) {
        sequence_step(item, step);
        cases.push_back(item);
      }
    }
  }
}

BinaryCase rejected(const Input input, const FaultKind kind, const Word value) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  item.initial.m.at(input.update.index) = input.update.modifier;
  item.expected = item.initial;
  item.before = {{.words = {opcode(input)}, .address = start}};
  if (input.form == Form::relative_long) {
    item.before.push_back({.words = {input.update.offset}, .address = start + 1});
  }
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
  item.name = "Address calculation invalid selector/unqualified mode/boundary";
  return item;
}

void faults(Cases &cases) {
  for (const auto form :
       {Form::relative_register, Form::relative_long, Form::updated, Form::displacement}) {
    for (const Word mode : {0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_down | mode_up}) {
      cases.push_back(rejected({.update = {}, .form = form, .destination = 16, .status = mode},
                               FaultKind::unsupported_mode,
                               mode));
    }
    if (form == Form::displacement) { continue; }
    for (Word destination = 0; destination < 4; ++destination) {
      const Input input{.update = {}, .form = form, .destination = destination};
      cases.push_back(rejected(input, FaultKind::invalid_instruction, opcode(input)));
    }
  }
  for (const auto form : {Form::updated, Form::displacement}) {
    for (const Word modifier : {0x8000U, 0xc000U, 0x8002U}) {
      const Input input{.update = {.modifier = modifier}, .form = form, .destination = 16};
      cases.push_back(rejected(input, FaultKind::unsupported_mode, modifier));
    }
  }
  auto missing = rejected({.update = {}, .form = Form::relative_long}, FaultKind::memory_read, 0);
  missing.before.resize(1);
  missing.memory_words[0] = start + 1;
  missing.fault.address = start + 1;
  cases.push_back(std::move(missing));
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_address_calc OUTPUT.d56"); }
    dsp56300::test::Cases cases;
    dsp56300::test::relative_cases(cases);
    dsp56300::test::updated_cases(cases);
    dsp56300::test::displacement_cases(cases);
    dsp56300::test::interactions(cases);
    dsp56300::test::sequence_prefixes(cases);
    dsp56300::test::faults(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent address-calculation cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
