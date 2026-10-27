// SPDX-License-Identifier: GPL-3.0-only
// Independent per-bit BFU expectations. Does not link or execute the DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
enum class Kind : std::uint8_t { extract, extract_unsigned, insert, merge, leading };
enum class Part : std::uint8_t { main, low };
using Bits = std::array<bool, accumulator_bits>;
constexpr Word saturation = Word{1} << 20, loop_flag = Word{1} << 15;
constexpr Word ignored_control_bits = 0xfc0fc0, repeat_base = 0x0600a0;
constexpr Word extract_register = 0x0c1a00, extract_immediate = 0x0c1800;
constexpr Word insert_register = 0x0c1b00, insert_immediate = 0x0c1900;
constexpr Word merge_base = 0x0c1b80, leading_base = 0x0c1e00;
constexpr unsigned field_shift = 12, fractional_bit = 47;
constexpr std::uint64_t pattern = 0xa572e19c46b38d;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array windows{
    std::pair{1U, 55U}, std::pair{24U, 24U}, std::pair{56U, 0U}, std::pair{12U, 12U}};
constexpr std::array insert_windows{
    std::pair{0U, 56U}, std::pair{1U, 55U}, std::pair{24U, 16U}, std::pair{24U, 32U}};

struct Input {
  Kind kind = Kind::extract;
  Word width = 1, offset = 0, status = 0, data = 0;
  unsigned controller = 4, source = 4;
  std::uint64_t value = pattern;
  bool immediate = false, source_b = false, destination_b = false;
};

Bits bits(std::uint64_t value) {
  Bits result{};
  for (auto &bit : result) {
    bit = value % 2 != 0;
    value /= 2;
  }
  return result;
}

std::uint64_t number(const Bits &value) {
  std::uint64_t result = 0;
  for (std::size_t index = value.size(); index != 0; --index) {
    result = result * 2 + (value[index - 1] ? 1U : 0U);
  }
  return result;
}

Word control(const Input input) {
  return (input.width << field_shift) | input.offset | ignored_control_bits;
}

template <class State> auto &storage(State &state, const unsigned selector) {
  if (selector < 4) { return selector == 2 ? state.a : state.b; }
  return (selector % 2) == 0 ? state.x : state.y;
}

unsigned word_offset(const unsigned selector, const Part part) {
  if (selector < 4) { return part == Part::main ? word_bits : 0U; }
  return selector < 6 ? 0U : word_bits;
}

Word partial(const Registers &state, const Input input, const Part part) {
  const auto selector = part == Part::main ? input.controller : input.source;
  return static_cast<Word>(storage(state, selector) >> word_offset(selector, part)) & word_mask;
}

void set_partial(Registers &state, const Input input, const Part part, const Word value) {
  const auto selector = part == Part::main ? input.controller : input.source;
  auto &target = storage(state, selector);
  const auto shift = word_offset(selector, part);
  target = (target & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
}

Word opcode(const Input input) {
  const Word destination = input.destination_b ? 1U : 0U;
  switch (input.kind) {
    case Kind::extract:
    case Kind::extract_unsigned:
      return (input.immediate ? extract_immediate : extract_register) |
             (input.kind == Kind::extract_unsigned ? 0x80U : 0U) | (input.source_b ? 16U : 0U) |
             destination | (input.immediate ? 0U : input.controller * 2);
    case Kind::insert:
      return (input.immediate ? insert_immediate : insert_register) | (input.source * 16) |
             destination | (input.immediate ? 0U : input.controller * 2);
    case Kind::merge: return merge_base | (input.controller * 2) | destination;
    case Kind::leading: return leading_base | (input.source_b ? 2U : 0U) | destination;
    default: throw std::runtime_error("Unknown BFU fixture kind");
  }
}

void result_flags(Registers &state, const Input input) {
  const auto raw = input.destination_b ? state.b : state.a;
  if (input.kind == Kind::merge || input.kind == Kind::leading) {
    const auto main = (raw / (std::uint64_t{1} << word_bits)) & word_mask;
    state.sr &= ~(flag_n | flag_z | flag_v);
    if (main >= sign24) { state.sr |= flag_n; }
    if (main == 0) { state.sr |= flag_z; }
    return;
  }
  const auto signed_result = signed56(raw);
  const unsigned position = fractional_bit + ((state.sr & mode_down) != 0 ? 1U : 0U) -
                            ((state.sr & mode_up) != 0 ? 1U : 0U);
  const auto boundary = std::int64_t{1} << position;
  Word flags = raw == 0 ? flag_z : 0;
  if (signed_result < 0) { flags |= flag_n; }
  if (signed_result < -boundary || signed_result >= boundary) { flags |= flag_e; }
  const auto divisor = static_cast<std::uint64_t>(boundary);
  if ((raw / divisor) % 2 == (raw / (divisor / 2)) % 2) { flags |= flag_u; }
  state.sr = (state.sr & ~(flag_n | flag_z | flag_v | flag_c | flag_e | flag_u)) | flags;
}

std::uint64_t model_leading(const Registers &state, const Input input) {
  const auto source = input.source_b ? state.b : state.a;
  const auto source_bits = bits(source);
  int count = 9;
  for (std::size_t index = accumulator_bits; index != 0; --index) {
    if (source_bits[index - 1] != source_bits.back()) { break; }
    --count;
  }
  if (source == 0) { count = 0; }
  return static_cast<std::uint64_t>(static_cast<std::int64_t>(count) *
                                    (std::int64_t{1} << word_bits)) &
         accumulator_mask;
}

std::uint64_t model_field(const Registers &state, const Input input) {
  auto result = bits(input.destination_b ? state.b : state.a);
  const auto source = bits(input.kind == Kind::merge ? partial(state, input, Part::main)
                                                     : partial(state, input, Part::low));
  const Word descriptor = input.immediate ? control(input) : partial(state, input, Part::main);
  const unsigned width = (descriptor / (Word{1} << field_shift)) % 64;
  const unsigned offset = descriptor % 64;
  if (input.kind == Kind::merge) {
    for (unsigned index = 0; index < field_shift; ++index) {
      result[word_bits + field_shift + index] = source[index];
    }
  } else if (input.kind == Kind::insert) {
    for (unsigned index = 0; index < width; ++index) { result[offset + index] = source[index]; }
  } else {
    const auto original = bits(input.source_b ? state.b : state.a);
    for (unsigned index = 0; index < accumulator_bits; ++index) {
      result[index] = index < width ? original[offset + index]
                                    : (input.kind == Kind::extract && original[offset + width - 1]);
    }
  }
  return number(result);
}

void model(Registers &state, const Input input) {
  const auto result =
      input.kind == Kind::leading ? model_leading(state, input) : model_field(state, input);
  (input.destination_b ? state.b : state.a) = result;
  result_flags(state, input);
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  (input.destination_b ? item.initial.b : item.initial.a) = input.value;
  if (input.kind == Kind::extract || input.kind == Kind::extract_unsigned ||
      input.kind == Kind::leading) {
    (input.source_b ? item.initial.b : item.initial.a) = input.value;
  }
  if (input.kind == Kind::insert) { set_partial(item.initial, input, Part::low, input.data); }
  if (!input.immediate && input.kind != Kind::leading) {
    set_partial(
        item.initial, input, Part::main, input.kind == Kind::merge ? input.data : control(input));
  }
  item.expected = item.initial;
  model(item.expected, input);
  std::vector program{opcode(input)};
  if (input.immediate) { program.push_back(control(input)); }
  item.expected.pc += static_cast<Word>(program.size());
  item.progress.estimated_cycles = program.size();
  item.before = {{.words = std::move(program), .address = start}};
  item.name = "BFU kind=" + std::to_string(static_cast<unsigned>(input.kind)) +
              "; width=" + std::to_string(input.width) + "; offset=" + std::to_string(input.offset);
  return item;
}

void status_cases(std::vector<BinaryCase> &cases, Input input) {
  for (const auto mode : modes) {
    for (const Word sm : {0U, saturation}) {
      for (const Word flags : {0U, status_byte}) {
        input.status = mode | sm | flags;
        cases.push_back(single(input));
      }
    }
  }
}

void extract_aliases(std::vector<BinaryCase> &cases, Input input) {
  for (const bool source_b : {false, true}) {
    input.source_b = source_b;
    for (const bool destination_b : {false, true}) {
      input.destination_b = destination_b;
      for (const auto &[width, offset] : windows) {
        input.width = width;
        input.offset = offset;
        status_cases(cases, input);
      }
    }
  }
}

void extract_cases(std::vector<BinaryCase> &cases) {
  for (const auto kind : {Kind::extract, Kind::extract_unsigned}) {
    for (unsigned selector = 2; selector < 8; ++selector) {
      for (const auto value : {pattern, accumulator_mask ^ pattern}) {
        extract_aliases(cases, {.kind = kind, .controller = selector, .value = value});
      }
    }
  }
}

Input varying_status(Input input) {
  const auto index = input.width + input.offset;
  input.status = modes.at(index % modes.size()) | (index % 2 == 0 ? status_byte : 0U) |
                 (input.width % 2 == 0 ? saturation : 0U);
  input.source_b = input.width % 2 == 0;
  input.destination_b = input.offset % 2 == 0;
  input.source = 2 + index % 6;
  return input;
}

void immediate_cases(std::vector<BinaryCase> &cases, const Kind kind) {
  const unsigned maximum = kind == Kind::insert ? word_bits : accumulator_bits;
  const unsigned minimum = kind == Kind::extract ? 1U : 0U;
  for (unsigned width = minimum; width <= maximum; ++width) {
    for (unsigned offset = 0; width + offset <= accumulator_bits; ++offset) {
      for (const auto value : {pattern, accumulator_mask ^ pattern}) {
        auto input = varying_status({.kind = kind,
                                     .width = width,
                                     .offset = offset,
                                     .data = static_cast<Word>(value) & word_mask,
                                     .value = value,
                                     .immediate = true});
        cases.push_back(single(input));
      }
    }
  }
}

void insert_aliases(std::vector<BinaryCase> &cases, Input input) {
  for (const bool destination_b : {false, true}) {
    input.destination_b = destination_b;
    for (const auto &[width, offset] : insert_windows) {
      input.width = width;
      input.offset = offset;
      for (const auto value : {pattern, accumulator_mask ^ pattern}) {
        input.data = static_cast<Word>(value) & word_mask;
        input.value = value;
        status_cases(cases, input);
      }
    }
  }
}

void insert_cases(std::vector<BinaryCase> &cases) {
  for (unsigned controller = 2; controller < 8; ++controller) {
    for (unsigned source = 2; source < 8; ++source) {
      insert_aliases(cases, {.kind = Kind::insert, .controller = controller, .source = source});
    }
  }
}

void insert_boundaries(std::vector<BinaryCase> &cases, Input input) {
  for (const auto value : {std::uint64_t{0}, accumulator_mask, sign56, sign56 - 1}) {
    input.value = value;
    for (const auto &[width, offset] : insert_windows) {
      input.width = width;
      input.offset = offset;
      for (const Word data : {0U, word_mask, sign24}) {
        input.data = data;
        status_cases(cases, input);
      }
    }
  }
}

void merge_cases(std::vector<BinaryCase> &cases, Input input) {
  constexpr std::array values{0U, 1U, 0x80U, 0x100U, 0x800U, 0xfffU, 0x123456U, 0xfedcbaU};
  for (const auto value :
       {std::uint64_t{0}, pattern, accumulator_mask ^ pattern, accumulator_mask}) {
    input.value = value;
    for (const auto data : values) {
      input.data = data;
      status_cases(cases, input);
    }
  }
}

void merge_leading_cases(std::vector<BinaryCase> &cases) {
  for (const bool destination_b : {false, true}) {
    for (unsigned controller = 2; controller < 8; ++controller) {
      merge_cases(cases,
                  {.kind = Kind::merge, .controller = controller, .destination_b = destination_b});
    }
    for (const bool source_b : {false, true}) {
      for (unsigned position = 0; position < accumulator_bits; ++position) {
        for (const bool inverse : {false, true}) {
          const auto single_bit = std::uint64_t{1} << position;
          status_cases(cases,
                       {.kind = Kind::leading,
                        .value = inverse ? accumulator_mask ^ single_bit : single_bit,
                        .source_b = source_b,
                        .destination_b = destination_b});
        }
      }
      for (const auto value : {std::uint64_t{0}, accumulator_mask}) {
        status_cases(cases,
                     {.kind = Kind::leading,
                      .value = value,
                      .source_b = source_b,
                      .destination_b = destination_b});
      }
    }
  }
}

BinaryCase rejected(const Input input, const FaultKind kind, const Word value) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  if (input.controller >= 2 && !input.immediate && input.kind != Kind::leading) {
    set_partial(item.initial, input, Part::main, control(input));
  }
  item.expected = item.initial;
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = {
      .kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
  std::vector program{opcode(input)};
  if (input.immediate) { program.push_back(control(input)); }
  item.before = {{.words = std::move(program), .address = start}};
  item.name = "BFU explicit undefined-input/unqualified-mode boundary";
  return item;
}

void rejection_cases(std::vector<BinaryCase> &cases) {
  for (const auto kind :
       {Kind::extract, Kind::extract_unsigned, Kind::insert, Kind::merge, Kind::leading}) {
    for (const Word mode : {0x2000U, 0x4000U, 0x20000U, mode_up | mode_down}) {
      cases.push_back(rejected({.kind = kind, .status = mode}, FaultKind::unsupported_mode, mode));
    }
    if (kind == Kind::leading) { continue; }
    for (unsigned selector = 0; selector < 2; ++selector) {
      const Input input{.kind = kind, .controller = selector};
      cases.push_back(rejected(input, FaultKind::invalid_instruction, opcode(input)));
    }
  }
  for (const auto kind : {Kind::extract, Kind::extract_unsigned, Kind::insert}) {
    for (const bool immediate : {false, true}) {
      const Input invalid{.kind = kind, .width = 63, .offset = 63, .immediate = immediate};
      cases.push_back(rejected(invalid,
                               FaultKind::invalid_instruction,
                               kind == Kind::insert ? opcode(invalid) : control(invalid)));
      const Input empty{.kind = Kind::extract, .width = 0, .immediate = immediate};
      cases.push_back(rejected(empty, FaultKind::unsupported_mode, control(empty)));
    }
  }
  for (unsigned source = 0; source < 2; ++source) {
    const Input input{.kind = Kind::insert, .source = source, .immediate = true};
    cases.push_back(rejected(input, FaultKind::invalid_instruction, opcode(input)));
  }
  const Input too_wide{.kind = Kind::insert, .width = 25, .immediate = true};
  cases.push_back(rejected(too_wide, FaultKind::invalid_instruction, opcode(too_wide)));
}

void extension_boundaries(std::vector<BinaryCase> &cases) {
  for (const auto kind : {Kind::extract, Kind::extract_unsigned, Kind::insert}) {
    const Input input{.kind = kind, .immediate = true};
    auto truncated = single(input);
    truncated.memory_words[0] = start + 1;
    truncated.before.front().words.resize(1);
    truncated.dispatches = 0;
    truncated.progress = {};
    truncated.interruptible = false;
    truncated.fault = {
        .kind = FaultKind::memory_read, .space = Space::program, .address = start + 1, .pc = start};
    truncated.name = "BFU missing control extension must fault";
    cases.push_back(std::move(truncated));
    auto repeat = single(input);
    repeat.before.front().words.insert(repeat.before.front().words.begin(),
                                       repeat_base | (2U << 8));
    repeat.budget = 2;
    repeat.dispatches = 1;
    repeat.progress = {.instructions = 1, .estimated_cycles = 5};
    repeat.interruptible = false;
    repeat.fault = {.kind = FaultKind::invalid_instruction,
                    .space = Space::program,
                    .address = start + 1,
                    .value = opcode(input),
                    .pc = start + 1};
    repeat.name = "REP cannot execute two-word BFU control form";
    cases.push_back(std::move(repeat));
  }
}

BinaryCase repeated(const Input input, const Word count) {
  auto item = single(input);
  item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
  for (Word iteration = 1; iteration < count; ++iteration) { model(item.expected, input); }
  item.expected.pc = start + 2;
  item.budget = item.dispatches = count + 1;
  item.progress = {.instructions = count + 1, .estimated_cycles = count + 5};
  item.name += "; REP reevaluates current operands";
  return item;
}

void repeated_cases(std::vector<BinaryCase> &cases) {
  const std::array inputs{
      Input{.kind = Kind::extract, .width = 24, .offset = 12},
      Input{.kind = Kind::extract_unsigned, .width = 24, .offset = 24, .controller = 2},
      Input{.kind = Kind::insert, .width = 24, .offset = 12, .data = 0x123456, .source = 2},
      Input{.kind = Kind::merge, .data = 0x123456, .controller = 2},
      Input{.kind = Kind::leading}};
  for (const auto input : inputs) {
    for (const Word count : {1U, 2U, 7U}) { cases.push_back(repeated(input, count)); }
  }
}

struct LoopState {
  Word pointer = 0, count = 1;
};

BinaryCase loop(const Input input, const LoopState frame) {
  const auto pointer = frame.pointer;
  const auto count = frame.count;
  auto item = single(input);
  auto &initial = item.initial;
  constexpr Word pointer_range = 64;
  constexpr Word saved_la = 0x70;
  constexpr Word saved_lc = 7;
  initial.sr |= loop_flag;
  initial.la = item.expected.pc - 1;
  initial.lc = count;
  initial.sp = pointer;
  initial.sc = 2;
  const Word saved_flag = pointer % 2 == 0 ? loop_flag : 0U;
  initial.stack[pointer % stack_entries] =
      (std::uint64_t{start} << word_bits) | saved_flag | flag_v;
  initial.stack[(pointer + pointer_range - 1) % stack_entries] =
      (std::uint64_t{saved_la} << word_bits) | saved_lc;
  item.expected = initial;
  model(item.expected, input);
  item.expected.pc = count > 1 ? start : initial.la + 1;
  if (count > 1) {
    --item.expected.lc;
  } else {
    item.expected.lc = saved_lc;
    item.expected.la = saved_la;
    item.expected.sp = (pointer + pointer_range - 2) % pointer_range;
    item.expected.sc = 0;
    item.expected.sr = (item.expected.sr & ~loop_flag) | saved_flag;
  }
  item.name += "; DO ending at instruction or extension; wrapped frames";
  return item;
}

void loop_cases(std::vector<BinaryCase> &cases) {
  const std::array inputs{Input{.kind = Kind::extract},
                          Input{.kind = Kind::extract_unsigned},
                          Input{.kind = Kind::insert},
                          Input{.kind = Kind::merge},
                          Input{.kind = Kind::leading},
                          Input{.kind = Kind::extract, .immediate = true},
                          Input{.kind = Kind::extract_unsigned, .immediate = true},
                          Input{.kind = Kind::insert, .immediate = true}};
  for (const auto input : inputs) {
    for (const Word pointer : {0U, 15U, 31U, 63U}) {
      for (const Word count : {1U, 2U}) {
        cases.push_back(loop(input, {.pointer = pointer, .count = count}));
      }
    }
  }
}

void mixed_sequences(std::vector<BinaryCase> &cases) {
  constexpr Word add_b_a = 0x200010;
  constexpr Word mac_a = 0x2000d2;
  const std::array inputs{
      Input{.kind = Kind::merge, .controller = 4},
      Input{.kind = Kind::extract, .width = 24, .offset = 24, .immediate = true},
      Input{.kind = Kind::insert,
            .width = 24,
            .offset = 32,
            .source = 2,
            .immediate = true,
            .destination_b = true},
      Input{.kind = Kind::leading, .source_b = true}};
  std::vector<Word> program{mac_a};
  for (const auto input : inputs) {
    program.push_back(opcode(input));
    if (input.immediate) { program.push_back(control(input)); }
    program.push_back(add_b_a);
  }
  for (const auto mode : modes) {
    for (const Word flags : {0U, status_byte}) {
      BinaryCase item;
      item.initial = register_model::seed(mode | flags);
      item.expected = item.initial;
      item.before = {{.words = program, .address = start}};
      item.name = "Native ALU/BFU helpers: all shorter prefixes and dirty A/B/status";
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, mac_a);
      ++item.expected.pc;
      cases.push_back(item);
      for (const auto input : inputs) {
        model(item.expected, input);
        item.expected.pc += input.immediate ? 2U : 1U;
        ++item.budget;
        item.dispatches = item.budget;
        item.progress = {.instructions = item.budget, .estimated_cycles = item.expected.pc - start};
        cases.push_back(item);
        const auto previous = item.expected;
        sequence_model::alu(item.expected, previous, add_b_a);
        ++item.expected.pc;
        ++item.budget;
        item.dispatches = item.budget;
        item.progress = {.instructions = item.budget, .estimated_cycles = item.expected.pc - start};
        cases.push_back(item);
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_bit_fields OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    dsp56300::test::extract_cases(cases);
    dsp56300::test::insert_cases(cases);
    for (const bool destination_b : {false, true}) {
      dsp56300::test::insert_boundaries(
          cases,
          {.kind = dsp56300::test::Kind::insert, .source = 5, .destination_b = destination_b});
    }
    for (const auto kind : {dsp56300::test::Kind::extract,
                            dsp56300::test::Kind::extract_unsigned,
                            dsp56300::test::Kind::insert}) {
      dsp56300::test::immediate_cases(cases, kind);
    }
    dsp56300::test::merge_leading_cases(cases);
    dsp56300::test::rejection_cases(cases);
    dsp56300::test::extension_boundaries(cases);
    dsp56300::test::repeated_cases(cases);
    dsp56300::test::loop_cases(cases);
    dsp56300::test::mixed_sequences(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent per-bit BFU cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
