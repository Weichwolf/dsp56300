// SPDX-License-Identifier: GPL-3.0-only
// Independent block/ring coordinates; no DSP execution or implementation helpers.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
enum class Kind : std::uint8_t { ordinary, linear, multi };
enum class Form : std::uint8_t {
  update,
  lua,
  short_lua,
  load_x,
  store_x,
  load_y,
  store_y,
  short_load_x,
  short_store_x,
  short_load_y,
  short_store_y,
  long_load_x,
  long_store_x,
  long_load_y,
  long_store_y
};
constexpr Word sm = Word{1} << 20, base = 0x40000, address_range = 0x1000000;
constexpr Word old_status = 0x81, old_loop = 0x70, old_size = 0xabc, old_vba = 0x123400;
constexpr Word low_frame = 0x567800, operand_base = 0x180;
constexpr Word incoming = 0x765432, sentinel = 0x2468ac;
constexpr Word update_code = 0x204000, lua_code = 0x044000, short_code = 0x040000;
constexpr Word load_x_code = 0x44c000, store_x_code = 0x444000;
constexpr Word load_y_code = 0x4ec000, store_y_code = 0x4e4000;
constexpr Word last_offset_register = 31, short_mask = 127, short_upper = 0x70;
constexpr Word multiple_bit = 0x8000;
constexpr Word short_move_code = 0x020080, long_move_code = 0x0a7080, ignored_n = 0xa54321;
constexpr std::int32_t maximum_short_offset = 63;
constexpr std::uint64_t seed_x = 0x345678fedcba, seed_y = 0x876543123456;
constexpr std::uint64_t seed_a = 0x7e345678abcdef, seed_b = 0x813456abcdef12;

struct Input {
  Word length = 2, base = 0, coordinate = 0;
  std::int32_t offset = 0;
  unsigned mode = 0, index = 0;
  Form form = Form::update;
  Kind kind = Kind::ordinary;
  bool saturated = false;
};

Word alignment(const Word length) {
  Word result = 1;
  while (result < length) { result *= 2; }
  return result;
}

Word wrap(const std::int64_t value) {
  return static_cast<Word>((value % address_range + address_range) % address_range);
}

Word address(const Input input) {
  const auto cursor = input.base + input.coordinate;
  const auto span = alignment(input.length);
  const auto amount = input.form == Form::short_lua || input.mode != 0
                          ? input.offset
                          : -static_cast<std::int64_t>(input.offset);
  const Word positive = input.offset > 0 ? static_cast<Word>(input.offset) : 0;
  const bool stride =
      input.kind == Kind::ordinary && positive != 0 && (positive / span) * span == positive;
  if (input.kind == Kind::linear || stride) { return wrap(cursor + amount); }
  const auto coordinate = static_cast<std::int64_t>(input.coordinate) + amount;
  return input.base + static_cast<Word>((coordinate % input.length + input.length) % input.length);
}

Registers seed(const Input input) {
  auto result = initial_state();
  result.x = seed_x;
  result.y = seed_y;
  result.a = seed_a;
  result.b = seed_b;
  result.sr = old_status | (input.saturated ? sm : 0U);
  result.omr = 5;
  result.la = old_loop;
  result.lc = 7;
  result.sp = result.sc = 2;
  result.sz = old_size;
  result.ep = operand_base;
  result.vba = old_vba;
  for (unsigned index = 0; index < agu_registers; ++index) {
    result.r[index] = operand_base + index;
    result.n[index] = 3;
  }
  for (unsigned index = 0; index < stack_entries; ++index) {
    result.stack[index] = (std::uint64_t{old_vba + index} << word_bits) | (low_frame + index);
  }
  result.r[input.index] = input.base + input.coordinate;
  result.n[input.index] = static_cast<Word>(input.offset) & word_mask;
  result.m[input.index] = input.kind == Kind::linear ? word_mask : input.length - 1;
  if (input.kind == Kind::multi) { result.m[input.index] |= multiple_bit; }
  return result;
}

bool displaced(const Form form) {
  return form >= Form::short_load_x;
}

struct Displacement {
  bool long_form;
  bool y;
  bool load;
};

Displacement displacement(const Form form) {
  const auto fields = static_cast<unsigned>(form) - static_cast<unsigned>(Form::short_load_x);
  return {.long_form = fields >= 4, .y = (fields & 2U) != 0, .load = (fields & 1U) == 0};
}

Word displacement_opcode(const Input input) {
  const auto fields = displacement(input.form);
  Word result = (fields.long_form ? long_move_code : short_move_code) | (input.index << 8) |
                (fields.y ? 6U : 4U);
  if (fields.y) { result |= Word{1} << (fields.long_form ? 16U : 5U); }
  if (fields.load) { result |= Word{1} << (fields.long_form ? 6U : 4U); }
  if (!fields.long_form) {
    const auto raw = static_cast<Word>(input.offset) & short_mask;
    for (unsigned bit = 0; bit < 7; ++bit) {
      if (((raw >> bit) & 1U) != 0) { result |= Word{1} << (bit == 0 ? 6U : bit + 10); }
    }
  }
  return result;
}

Word opcode(const Input input) {
  if (displaced(input.form)) { return displacement_opcode(input); }
  const Word index = input.index << 8;
  const Word mode = input.mode << 11;
  switch (input.form) {
    case Form::update: return update_code | mode | index;
    case Form::lua: return lua_code | mode | index | last_offset_register;
    case Form::short_lua: {
      const auto bits = static_cast<Word>(input.offset) & short_mask;
      return short_code | ((bits & short_upper) << 7) | index | ((bits & 15U) << 4) | 15U;
    }
    case Form::load_x: return load_x_code | mode | index;
    case Form::store_x: return store_x_code | mode | index;
    case Form::load_y: return load_y_code | mode | index;
    case Form::store_y: return store_y_code | mode | index;
    case Form::short_load_x:
    case Form::short_store_x:
    case Form::short_load_y:
    case Form::short_store_y:
    case Form::long_load_x:
    case Form::long_store_x:
    case Form::long_load_y:
    case Form::long_store_y:
    default: throw std::logic_error("Unknown authored stride form");
  }
}

std::string name(const Input input) {
  constexpr std::array kinds{"ordinary", "linear", "multi"};
  const auto *kind = kinds.at(static_cast<unsigned>(input.kind));
  constexpr std::array forms{"update",
                             "lua",
                             "short",
                             "load-x",
                             "store-x",
                             "load-y",
                             "store-y",
                             "short-load-x",
                             "short-store-x",
                             "short-load-y",
                             "short-store-y",
                             "long-load-x",
                             "long-store-x",
                             "long-load-y",
                             "long-store-y"};
  return std::string("Authored ") + kind + " modulo " +
         forms.at(static_cast<unsigned>(input.form)) + ": length=" + std::to_string(input.length) +
         " R=" + std::to_string(input.base + input.coordinate) +
         (displaced(input.form) ? " D=" : " N=") + std::to_string(input.offset) +
         " mode=" + std::to_string(input.mode) + " index=" + std::to_string(input.index) +
         " SM=" + std::to_string(static_cast<unsigned>(input.saturated));
}

void memory_effect(BinaryCase &item, const Input input, const Word effective) {
  const bool y = displaced(input.form) ? displacement(input.form).y
                                       : input.form == Form::load_y || input.form == Form::store_y;
  const bool load = displaced(input.form)
                        ? displacement(input.form).load
                        : input.form == Form::load_x || input.form == Form::load_y;
  const auto space = y ? Space::y : Space::x;
  const auto previous = input.base + input.coordinate;
  require(effective < memory_words && previous < memory_words,
          "Authored memory strides stay in RAM");
  if (previous != effective) {
    item.before.push_back({.words = {sentinel}, .address = previous, .space = space});
  }
  item.before.push_back({.words = {incoming}, .address = effective, .space = space});
  auto &bank = y ? item.expected.y : item.expected.x;
  if (load) {
    bank = (bank & ~std::uint64_t{word_mask}) | incoming;
  } else {
    item.after.push_back(
        {.words = {static_cast<Word>(bank) & word_mask}, .address = effective, .space = space});
  }
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.name = name(input);
  item.initial = seed(input);
  if (displaced(input.form)) { item.initial.n[input.index] = ignored_n; }
  item.expected = item.initial;
  const auto effective = address(input);
  std::vector program{opcode(input)};
  const bool long_form = displaced(input.form) && displacement(input.form).long_form;
  if (long_form) { program.push_back(static_cast<Word>(input.offset) & word_mask); }
  item.before = {{.words = std::move(program), .address = start}};
  if (input.form == Form::update) {
    item.expected.r[input.index] = effective;
  } else if (input.form == Form::lua || input.form == Form::short_lua) {
    item.expected.n.back() = effective;
    item.progress.estimated_cycles = 3;
  } else {
    memory_effect(item, input, effective);
    item.progress.estimated_cycles = long_form ? 3U : 2U;
  }
  item.expected.pc += long_form ? 2U : 1U;
  return item;
}

void saturated(Cases &cases, Input input) {
  for (const bool enabled : {false, true}) {
    input.saturated = enabled;
    cases.push_back(single(input));
  }
}

void forms(Cases &cases, Input input) {
  for (unsigned index = 0; index < agu_registers; ++index) {
    input.index = index;
    for (const auto form : {Form::update, Form::lua}) {
      input.form = form;
      saturated(cases, input);
    }
  }
}

void offsets(Cases &cases, Input input) {
  const auto length = static_cast<std::int32_t>(input.length);
  for (const auto offset : {length, -length, length - 1}) {
    input.offset = offset;
    for (unsigned mode = 0; mode < 2; ++mode) {
      input.mode = mode;
      forms(cases, input);
    }
  }
}

void short_forms(Cases &cases, Input input) {
  if (input.length > 64) { return; }
  const auto length = static_cast<std::int32_t>(input.length);
  for (const auto offset : {length, -length}) {
    if (offset > maximum_short_offset) { continue; }
    input.offset = offset;
    input.mode = 1;
    input.form = Form::short_lua;
    for (unsigned index = 0; index < agu_registers; ++index) {
      input.index = index;
      saturated(cases, input);
    }
  }
}

void ordinary(Cases &cases) {
  constexpr std::array lengths{2U, 3U, 4U, 5U, 8U, 16U, 32U, 64U, 32768U};
  for (const auto length : lengths) {
    const std::array positions{std::array{0U, 0U},
                               std::array{0U, length - 1},
                               std::array{base, length - 1},
                               std::array{address_range - alignment(length), length - 1}};
    for (const auto position : positions) {
      const Input input{.length = length, .base = position[0], .coordinate = position[1]};
      offsets(cases, input);
      short_forms(cases, input);
    }
  }
}

void controls(Cases &cases) {
  for (const auto kind : {Kind::linear, Kind::multi}) {
    for (const Word length : {2U, 16U}) {
      for (const auto position : {std::array{0U, 0U}, std::array{base, length - 1}}) {
        const auto modulus = static_cast<std::int32_t>(length);
        for (const auto offset : {modulus, -modulus}) {
          for (unsigned mode = 0; mode < 2; ++mode) {
            forms(cases,
                  {.length = length,
                   .base = position[0],
                   .coordinate = position[1],
                   .offset = offset,
                   .mode = mode,
                   .kind = kind});
          }
        }
      }
    }
  }
}

void memory_forms(Cases &cases, Input input) {
  input.mode = 5;
  for (unsigned index = 0; index < agu_registers; ++index) {
    input.index = index;
    for (const auto form : {Form::load_x, Form::store_x, Form::load_y, Form::store_y}) {
      input.form = form;
      saturated(cases, input);
    }
  }
}

void displacement_forms(Cases &cases, Input input) {
  input.mode = 5;
  constexpr std::array forms{Form::short_load_x,
                             Form::long_load_x,
                             Form::short_store_x,
                             Form::long_store_x,
                             Form::short_load_y,
                             Form::long_load_y,
                             Form::short_store_y,
                             Form::long_store_y};
  for (unsigned index = 0; index < agu_registers; ++index) {
    input.index = index;
    for (unsigned direction = 0; direction < 4; ++direction) {
      for (const bool enabled : {false, true}) {
        input.saturated = enabled;
        for (unsigned width = 0; width < 2; ++width) {
          input.form = forms[(direction * 2) + width];
          cases.push_back(single(input));
        }
      }
    }
  }
}

void memory(Cases &cases, const bool displacement_cases) {
  for (const Word length : {2U, 16U, 32U}) {
    for (const Word position : {0U, length - 1}) {
      const auto modulus = static_cast<std::int32_t>(length);
      for (const auto offset : {modulus, -modulus}) {
        const Input input{.length = length, .base = 128, .coordinate = position, .offset = offset};
        if (displacement_cases) {
          displacement_forms(cases, input);
        } else {
          memory_forms(cases, input);
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error(
          "Usage: generate_modulo_strides pure|memory|displacement OUTPUT.d56");
    }
    const std::string_view kind = argv[1];
    dsp56300::test::Cases cases;
    if (kind == "pure") {
      dsp56300::test::ordinary(cases);
      dsp56300::test::controls(cases);
    } else if (kind == "memory") {
      dsp56300::test::memory(cases, false);
    } else if (kind == "displacement") {
      dsp56300::test::memory(cases, true);
    } else {
      throw std::runtime_error("Unknown modulo stride matrix");
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent " << kind << " modulo-stride cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
