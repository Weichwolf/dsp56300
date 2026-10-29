// SPDX-License-Identifier: GPL-3.0-only
// Independent raw-width selection/status and register/bus sequence expectations.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
using selection_model::Kind;
using Alu = selection_model::Input;
constexpr std::array selections{Alu{.kind = Kind::maximum}, Alu{.kind = Kind::magnitude}};
constexpr std::array unaries{Alu{.kind = Kind::complement},
                             Alu{.kind = Kind::complement, .accumulator_b = true},
                             Alu{.kind = Kind::test},
                             Alu{.kind = Kind::test, .accumulator_b = true}};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 18> values{0,
                                               1,
                                               2,
                                               word_mask,
                                               std::uint64_t{1} << word_bits,
                                               (std::uint64_t{1} << 47) - 1,
                                               std::uint64_t{1} << 47,
                                               (std::uint64_t{1} << 47) + 1,
                                               sign56 - 2,
                                               sign56 - 1,
                                               sign56,
                                               sign56 + 1,
                                               accumulator_mask - 1,
                                               accumulator_mask,
                                               0x400000000000,
                                               0xffc00000000000,
                                               0x0055aa00ff1234,
                                               0xffabcdfedcba98};
constexpr Word xbase = 384, ybase = 400, ring_length = 16, xdata = 0x654321, ydata = 0x923456;
constexpr Word ring_modifier = 0xab800f, ignored_n = 0xa54321, full_immediate = 0x800001;
constexpr Word memory_base = 0x400000, pair_base = 0x800000, effective = 0x4000;
constexpr Word load_bit = 0x8000, y_bit = 0x080000;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_x_a = 0x200021, add_a_b = 0x200018;
constexpr Word repeat_base = 0x0600a0, lf = Word{1} << 15, fv = Word{1} << 16, pointer_range = 64;
constexpr Word saved_la = 0x70;
constexpr unsigned xreg_shift = 18, ymode_shift = 20;
constexpr Word y_load_bit = 0x400000, pointer_address = ybase + ring_length,
               pointer_value = xbase + 3;
constexpr Word load_x0 = 0x44e000;

enum class Bus : std::uint8_t {
  none,
  register_load,
  register_store,
  immediate_short,
  single_load,
  single_store,
  pair_mix,
  pair_stores,
  pair_inputs,
  absolute_load,
  immediate_long,
  pointer_load
};
constexpr std::array buses{Bus::none,
                           Bus::register_load,
                           Bus::register_store,
                           Bus::immediate_short,
                           Bus::single_load,
                           Bus::single_store,
                           Bus::pair_mix,
                           Bus::pair_stores,
                           Bus::pair_inputs,
                           Bus::absolute_load,
                           Bus::immediate_long,
                           Bus::pointer_load};

struct Input {
  Alu alu;
  std::uint64_t a = 0, b = 0;
  Word status = 0;
  Bus bus = Bus::none;
};

bool selecting(const Alu alu) {
  return alu.kind == Kind::maximum || alu.kind == Kind::magnitude;
}

unsigned load_register(const Alu alu) {
  if (selecting(alu)) { return 14; }
  const bool target_b = alu.kind == Kind::test ? alu.accumulator_b : !alu.accumulator_b;
  return target_b ? 15U : 14U;
}

unsigned store_register(const Alu alu) {
  return selecting(alu) || alu.accumulator_b ? 15U : 14U;
}

Word memory_opcode(
    const unsigned reg, const unsigned index, const unsigned mode, const bool load, const bool y) {
  constexpr Word high_reg_mask = 0x18;
  constexpr unsigned high_reg_shift = 17;
  return memory_base | ((reg & 7U) << 16) | ((reg & high_reg_mask) << high_reg_shift) | effective |
         (mode << 11) | (index << 8) | (load ? load_bit : 0U) | (y ? y_bit : 0U);
}

std::vector<Word> program(const Input input) {
  const auto load = load_register(input.alu);
  const auto store = store_register(input.alu);
  Word word = parallel_none;
  switch (input.bus) {
    case Bus::none: break;
    case Bus::register_load: word |= (4U << 13) | (load << 8); break;
    case Bus::register_store: word |= (store << 13) | (4U << 8); break;
    case Bus::immediate_short: word |= (load << 16) | (128U << 8); break;
    case Bus::single_load: word = memory_opcode(load, 0, 3, true, false); break;
    case Bus::single_store: word = memory_opcode(store, 4, 3, false, true); break;
    case Bus::pair_mix: {
      const bool xload = load == 14;
      word = pair_base | (2U << xreg_shift) | (3U << 16) | (3U << 11) | (3U << ymode_shift) |
             (xload ? load_bit : y_load_bit);
      break;
    }
    case Bus::pair_stores:
      word = pair_base | (2U << xreg_shift) | (3U << 16) | (3U << 11) | (3U << ymode_shift);
      break;
    case Bus::pair_inputs:
      word = pair_base | (3U << 11) | (3U << ymode_shift) | load_bit | y_load_bit;
      break;
    case Bus::absolute_load: word = memory_opcode(load, 0, 6, true, false); break;
    case Bus::immediate_long: word = memory_opcode(load, 4, 6, true, false); break;
    case Bus::pointer_load: word = memory_opcode(16, 2, 4, true, true); break;
    default: throw std::logic_error("Unknown selection transfer");
  }
  std::vector<Word> result{word | selection_model::code(input.alu)};
  if (input.bus == Bus::absolute_load) { result.push_back(xbase + ring_length - 1); }
  if (input.bus == Bus::immediate_long) { result.push_back(full_immediate); }
  return result;
}

Word sample(const Space space, const Word address) {
  return space == Space::x ? xdata + address - xbase : ydata - (address - ybase);
}

Word advance(Registers &state, const unsigned index) {
  auto &cursor = state.r.at(index);
  const auto original = cursor;
  const auto base = (cursor / ring_length) * ring_length;
  cursor = base + ((cursor + 1) % ring_length);
  return original;
}

void store(BinaryCase &item, const Space space, const Word address, const Word value) {
  item.after.push_back({.words = {value}, .address = address, .space = space});
}

void transfers(BinaryCase &item, const Input input) {
  auto &state = item.expected;
  const auto load = load_register(input.alu);
  const auto source = store_register(input.alu);
  switch (input.bus) {
    case Bus::none: return;
    case Bus::register_load:
      register_model::write(state, load, register_model::read(state, 4));
      return;
    case Bus::register_store:
      register_model::write(state, 4, register_model::read(state, source));
      return;
    case Bus::immediate_short: register_model::write(state, load, sign24); return;
    case Bus::single_load: {
      const auto address = advance(state, 0);
      register_model::write(state, load, sample(Space::x, address));
      return;
    }
    case Bus::single_store: {
      const auto address = advance(state, 4);
      store(item, Space::y, address, register_model::read(state, source));
      return;
    }
    case Bus::absolute_load:
      register_model::write(state, load, sample(Space::x, xbase + ring_length - 1));
      return;
    case Bus::immediate_long: register_model::write(state, load, full_immediate); return;
    case Bus::pointer_load: register_model::write(state, 16, pointer_value); return;
    case Bus::pair_mix:
    case Bus::pair_stores:
    case Bus::pair_inputs: break;
    default: throw std::logic_error("Unknown selection transfer");
  }
  const auto xa = advance(state, 0);
  const auto ya = advance(state, 4);
  const bool xload = input.bus == Bus::pair_inputs || (input.bus == Bus::pair_mix && load == 14);
  const bool yload = input.bus == Bus::pair_inputs || (input.bus == Bus::pair_mix && load == 15);
  const auto xv = xload ? sample(Space::x, xa) : register_model::read(state, 14);
  const auto yv = yload ? sample(Space::y, ya) : register_model::read(state, 15);
  const unsigned xr = input.bus == Bus::pair_inputs ? 4U : 14U;
  const unsigned yr = input.bus == Bus::pair_inputs ? 6U : 15U;
  if (xload) {
    register_model::write(state, xr, xv);
  } else {
    store(item, Space::x, xa, xv);
  }
  if (yload) {
    register_model::write(state, yr, yv);
  } else {
    store(item, Space::y, ya, yv);
  }
}

void execute(BinaryCase &item, const Input input) {
  const auto old = item.expected;
  transfers(item, input);
  selection_model::apply(item.expected, old, input.alu);
}

BinaryCase single(const Input input) {
  BinaryCase item;
  item.initial = register_model::seed(input.status);
  item.initial.sr = input.status;
  item.initial.a = input.a;
  item.initial.b = input.b;
  item.initial.r[0] = xbase + ring_length - 1;
  item.initial.r[4] = ybase + ring_length - 1;
  item.initial.r[2] = pointer_address;
  item.initial.m[0] = item.initial.m[4] = ring_modifier;
  item.initial.n[0] = item.initial.n[4] = ignored_n;
  std::vector<Word> xwords;
  std::vector<Word> ywords;
  for (Word position = 0; position < ring_length; ++position) {
    xwords.push_back(sample(Space::x, xbase + position));
    ywords.push_back(sample(Space::y, ybase + position));
  }
  const auto words = program(input);
  item.before = {{.words = words, .address = start},
                 {.words = std::move(xwords), .address = xbase, .space = Space::x},
                 {.words = std::move(ywords), .address = ybase, .space = Space::y},
                 {.words = {pointer_value}, .address = pointer_address, .space = Space::y}};
  item.expected = item.initial;
  execute(item, input);
  item.expected.pc += static_cast<Word>(words.size());
  item.progress.estimated_cycles = words.size();
  item.name = "Selection/status raw-56; ALU=" + std::to_string(selection_model::code(input.alu)) +
              "; bus=" + std::to_string(static_cast<unsigned>(input.bus));
  return item;
}

void bit_basis(Cases &cases, const Alu alu) {
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const auto power = std::uint64_t{1} << bit;
    for (const auto value : {power, accumulator_mask - power}) {
      const auto opposite = static_cast<std::uint64_t>(-signed56(value)) & accumulator_mask;
      cases.push_back(single({.alu = alu,
                              .a = value,
                              .b = selecting(alu) ? opposite : value,
                              .status = modes.at(bit % modes.size()) | status_byte}));
    }
  }
}

void ordinary(Cases &cases, const Alu alu) {
  for (const auto a : values) {
    for (const auto b : values) {
      if (!selecting(alu) && b != values.front()) { continue; }
      for (const auto mode : modes) {
        for (const Word flags : {0U, status_byte}) {
          cases.push_back(
              single({.alu = alu, .a = a, .b = selecting(alu) ? b : a, .status = mode | flags}));
        }
      }
    }
  }
  for (Word flags = 0; flags <= status_byte; ++flags) {
    cases.push_back(single({.alu = alu,
                            .a = values.at(flags % values.size()),
                            .b = values.at((flags + 5) % values.size()),
                            .status = modes.at(flags % modes.size()) | flags}));
  }
  bit_basis(cases, alu);
}

void bus_matrix(Cases &cases, const std::span<const Alu> inputs) {
  for (const auto alu : inputs) {
    for (const auto bus : buses) {
      for (std::size_t index = 0; index < values.size(); ++index) {
        for (const auto mode : modes) {
          for (const Word flags : {0U, status_byte}) {
            cases.push_back(single({.alu = alu,
                                    .a = values[index],
                                    .b = values[(index + 7) % values.size()],
                                    .status = mode | flags,
                                    .bus = bus}));
          }
        }
      }
    }
  }
}

void conditional(Cases &cases, const std::span<const Alu> inputs) {
  constexpr Word if_base = 0x202000;
  constexpr Word update_bit = 0x1000;
  for (const auto alu : inputs) {
    for (Word condition = 0; condition < 16; ++condition) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const bool update : {false, true}) {
          const Input input{.alu = alu,
                            .a = values.at(flags % values.size()),
                            .b = values.at((flags + condition) % values.size()),
                            .status = modes.at(flags % modes.size()) | flags};
          auto item = single(input);
          const auto word =
              if_base | (condition << 8) | (update ? update_bit : 0U) | selection_model::code(alu);
          item.before.front().words = {word};
          item.expected = item.initial;
          if (selection_model::test_condition({.status = item.initial.sr, .selector = condition})) {
            selection_model::apply(item.expected, item.initial, alu);
          }
          if (!update) { item.expected.sr = item.initial.sr; }
          ++item.expected.pc;
          item.name = "IFcc and IFcc.U selection/status: all conditions and old CCR bytes";
          cases.push_back(std::move(item));
        }
      }
    }
  }
}

void repeated(Cases &cases, const Input input) {
  constexpr Word count = 4;
  if (program(input).size() != 1) { return; }
  for (const Word budget : {1U, 2U, 3U, count + 1}) {
    auto item = single(input);
    item.expected = item.initial;
    item.after.clear();
    item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
    for (Word iteration = 1; iteration < budget; ++iteration) { execute(item, input); }
    const bool completed = budget == count + 1;
    item.expected.pc = start + (completed ? 2U : 1U);
    item.expected.lc = completed ? item.initial.lc : count + 1 - budget;
    item.budget = item.dispatches = budget;
    item.progress = {.instructions = budget, .estimated_cycles = 5 + budget - 1};
    item.interruptible = completed;
    item.name = "REP selection/status uses current accumulators and current parallel addresses";
    cases.push_back(std::move(item));
  }
}

void ending(Cases &cases, const Input input) {
  for (const Word pointer : {0U, 15U, 31U, 63U}) {
    for (const Word count : {1U, 2U}) {
      for (const Word saved : {0U, lf, fv, lf | fv}) {
        auto item = single(input);
        auto &state = item.initial;
        const Word words = static_cast<Word>(program(input).size());
        state.sr |= lf;
        state.la = start + words - 1;
        state.lc = count;
        state.sp = pointer;
        state.sc = 2;
        state.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
        state.stack[(pointer + pointer_range - 1) % stack_entries] =
            (std::uint64_t{saved_la} << word_bits) | 7U;
        item.expected = state;
        item.after.clear();
        execute(item, input);
        item.expected.pc = count == 1 ? start + words : start;
        if (count == 1) {
          item.expected.la = saved_la;
          item.expected.lc = 7;
          item.expected.sp = (pointer + pointer_range - 2) % pointer_range;
          item.expected.sc = 0;
          item.expected.sr = (item.expected.sr & ~(lf | fv)) | saved;
        } else {
          --item.expected.lc;
        }
        item.name = "Selection/status at DO opcode/extension endings retains Carry and saved LF/FV";
        cases.push_back(std::move(item));
      }
    }
  }
}

void sequence(Cases &cases, const Input input) {
  auto words = program(input);
  std::vector<Word> ops{mac_a, mac_b};
  ops.insert(ops.end(), words.begin(), words.end());
  if (input.bus == Bus::pointer_load) { ops.push_back(load_x0); }
  ops.insert(ops.end(), {adc_x_a, add_a_b});
  auto item = single(input);
  item.before.front().words = ops;
  item.expected = item.initial;
  item.after.clear();
  item.budget = 0;
  item.progress = {};
  for (std::size_t index = 0; index < ops.size(); ++index) {
    const bool selected = index == 2;
    if (selected) {
      execute(item, input);
    } else if (input.bus == Bus::pointer_load && index == 3) {
      register_model::write(item.expected, 4, sample(Space::x, item.expected.r[0]));
    } else {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, ops[index]);
    }
    const auto length = selected ? static_cast<Word>(words.size()) : 1U;
    item.expected.pc += length;
    item.progress.estimated_cycles += length;
    ++item.budget;
    item.dispatches = item.budget;
    item.progress.instructions = item.budget;
    item.name =
        "Native/selection-status helper/native Carry and accumulator consumers: every prefix";
    cases.push_back(item);
    if (length == 2) { ++index; }
  }
}

void interactions(Cases &cases, const std::span<const Alu> inputs) {
  for (const auto alu : inputs) {
    for (const auto bus : buses) {
      const Input input{
          .alu = alu, .a = sign56, .b = sign56 - 1, .status = status_byte, .bus = bus};
      repeated(cases, input);
      if (bus == Bus::none || bus == Bus::single_store || bus == Bus::absolute_load) {
        ending(cases, input);
      }
      for (const auto mode : modes) {
        auto variant = input;
        variant.status |= mode;
        sequence(cases, variant);
      }
    }
    for (const Word mode : {0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_down | mode_up}) {
      auto item = single({.alu = alu});
      item.initial.sr = mode;
      item.dispatches = 0;
      item.progress = {};
      item.interruptible = false;
      item.fault = {
          .kind = FaultKind::unsupported_mode, .address = start, .value = mode, .pc = start};
      item.name = "Selection/status remaining numerical modes are explicit unsupported boundaries";
      if (mode == 0x100000U && alu.kind == Kind::complement) {
        item.expected = item.initial;
        selection_model::apply(item.expected, item.initial, alu);
        ++item.expected.pc;
        item.dispatches = 1;
        item.progress = {.instructions = 1, .estimated_cycles = 1};
        item.interruptible = true;
        item.fault = {};
        item.name = "NOT uses the qualified BFU arithmetic-saturation bypass";
      }
      cases.push_back(std::move(item));
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3 ||
        (std::string_view(argv[1]) != "select" && std::string_view(argv[1]) != "unary")) {
      throw std::runtime_error("Usage: dsp56300_generate_selection select|unary OUTPUT.d56");
    }
    const std::span<const dsp56300::test::Alu> inputs =
        std::string_view(argv[1]) == "select"
            ? std::span<const dsp56300::test::Alu>{dsp56300::test::selections}
            : std::span<const dsp56300::test::Alu>{dsp56300::test::unaries};
    dsp56300::test::Cases cases;
    for (const auto alu : inputs) { dsp56300::test::ordinary(cases, alu); }
    dsp56300::test::bus_matrix(cases, inputs);
    dsp56300::test::conditional(cases, inputs);
    dsp56300::test::interactions(cases, inputs);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent selection/complement/test cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
