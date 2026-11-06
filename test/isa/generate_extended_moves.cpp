// SPDX-License-Identifier: GPL-3.0-only
// Independent MOVE-form input/expectation authoring; no emulator execution/link.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "move_model.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
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
using move_model::Form;
using move_model::Kind;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr Word immediate_data = 0x800001, pointer_range = 64;
constexpr Word location = 384, saved_la = 0x70, lf = Word{1} << 15, fv = Word{1} << 16;
constexpr Word repeat_base = 0x0600a0, mac_a = 0x2000d2, mac_b = 0x2000da;
constexpr Word adc_x_a = 0x200021, round_a = 0x200011;
constexpr std::array<std::uint64_t, 16> values{0,
                                               1,
                                               word_mask,
                                               std::uint64_t{1} << word_bits,
                                               (std::uint64_t{1} << 47) - 1,
                                               std::uint64_t{1} << 47,
                                               (std::uint64_t{1} << 47) + 1,
                                               0xff800000000000,
                                               0xff7fffffffffff,
                                               sign56 - 1,
                                               sign56,
                                               sign56 + 1,
                                               accumulator_mask,
                                               0x0123456789abcd,
                                               0xff55aa00ff1234,
                                               0x00400000000001};

struct Seed {
  Word status = 0, modifier = 0xabffff, cursor = location, offset = 3;
  std::uint64_t a = register_model::seed_accumulator_a, b = register_model::seed_accumulator_b;
  move_model::Data data{};
};

std::vector<Word> program(const Form form, const Word alu = 0) {
  std::vector<Word> result{move_model::opcode(form, alu)};
  if (move_model::words(form) == 2) { result.push_back(form.extension); }
  return result;
}

void alu_result(Registers &state, const Registers &old, const Word alu) {
  if (alu == 0) { return; }
  if (alu == 3 || alu == 11) {
    selection_model::apply(
        state, old, {.kind = selection_model::Kind::test, .accumulator_b = alu == 11});
  } else {
    sequence_model::alu(state, old, alu);
  }
}

void execute(BinaryCase &item, const Form form, const move_model::Data data, const Word alu) {
  const auto old = item.expected;
  const auto result = move_model::apply(item.expected, form, data);
  if (result.stores != 0) {
    item.after.push_back(
        {.words = {result.first}, .address = result.address, .space = result.space});
    if (result.stores == 2) {
      item.after.push_back(
          {.words = {result.second}, .address = result.address, .space = Space::y});
    }
  }
  alu_result(item.expected, old, alu);
}

BinaryCase single(const Form form, const Seed seed = {}, const Word alu = 0) {
  BinaryCase item;
  item.initial = register_model::seed(seed.status);
  item.initial.sr = seed.status;
  item.initial.a = seed.a;
  item.initial.b = seed.b;
  item.initial.r.at(form.index) = seed.cursor;
  item.initial.n.at(form.index) = seed.offset;
  item.initial.m.at(form.index) = seed.modifier;
  item.before.push_back({.words = program(form, alu), .address = start});
  if (form.kind != Kind::immediate) {
    auto address_state = item.initial;
    const auto address = move_model::address(address_state, form);
    const bool immediate = form.kind == Kind::mixed_one && form.mode == 6 && form.index == 4;
    if (!immediate) {
      item.before.push_back(
          {.words = {seed.data.high}, .address = address, .space = form.y ? Space::y : Space::x});
      if (form.kind == Kind::long_word) {
        item.before.push_back({.words = {seed.data.low}, .address = address, .space = Space::y});
      }
    }
  }
  item.expected = item.initial;
  execute(item, form, seed.data, alu);
  item.expected.pc += move_model::words(form);
  item.progress.estimated_cycles = move_model::cycles(form);
  item.name =
      "Independent extended MOVE; kind=" + std::to_string(static_cast<unsigned>(form.kind)) +
      "; opcode=" + std::to_string(move_model::opcode(form, alu)) +
      "; status=" + std::to_string(seed.status);
  return item;
}

void immediate_matrix(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned reg = 4; reg < 32; ++reg) {
      for (Word value = 0; value <= status_byte; ++value) {
        cases.push_back(single({.kind = Kind::immediate, .reg = reg, .extension = value},
                               {.status = mode | flag_c | flag_v | flag_s}));
      }
    }
  }
  for (Word flags = 0; flags <= status_byte; ++flags) {
    for (unsigned reg = 4; reg < 32; ++reg) {
      cases.push_back(
          single({.kind = Kind::immediate, .reg = reg, .extension = (flags + reg) & status_byte},
                 {.status = flags}));
    }
  }
}

void address_variants(Cases &cases, Form form, const Seed seed) {
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (mode == 6 &&
          (form.kind == Kind::mixed_two ||
           (index != 0 && (form.kind != Kind::mixed_one || !form.load || index != 4)))) {
        continue;
      }
      form.mode = mode;
      form.index = index;
      form.extension = mode == 6 && index == 4 ? immediate_data : location;
      cases.push_back(single(form, seed));
    }
  }
}

void long_matrix(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned selector = 0; selector < 8; ++selector) {
      for (const bool load : {false, true}) {
        const Form form{.selector = selector, .load = load};
        address_variants(cases, form, {.status = mode | flag_c | flag_v});
        for (Word address = 0; address < 64; ++address) {
          auto short_form = form;
          short_form.short_form = true;
          short_form.extension = address;
          cases.push_back(single(short_form, {.status = mode | flag_c | flag_v}));
        }
      }
    }
  }
}

void mixed_matrix(Cases &cases) {
  for (const auto mode : modes) {
    for (const bool y : {false, true}) {
      for (unsigned selector = 0; selector < 4; ++selector) {
        for (const bool source_b : {false, true}) {
          for (const bool second_high : {false, true}) {
            for (const bool load : {false, true}) {
              address_variants(cases,
                               {.kind = Kind::mixed_one,
                                .selector = selector,
                                .load = load,
                                .y = y,
                                .source_b = source_b,
                                .second_high = second_high},
                               {.status = mode});
            }
          }
        }
      }
      for (unsigned selector = 0; selector < 2; ++selector) {
        address_variants(cases,
                         {.kind = Kind::mixed_two, .selector = selector, .load = false, .y = y},
                         {.status = mode});
      }
    }
  }
}

std::vector<Form> representative(const Kind kind) {
  std::vector<Form> result;
  if (kind == Kind::immediate) {
    for (unsigned reg = 4; reg < 32; ++reg) {
      result.push_back({.kind = kind, .reg = reg, .extension = 128});
    }
  } else if (kind == Kind::long_word) {
    for (unsigned selector = 0; selector < 8; ++selector) {
      for (const bool load : {false, true}) {
        result.push_back({.selector = selector, .mode = 3, .load = load});
      }
    }
  } else {
    for (const bool y : {false, true}) {
      for (unsigned selector = 0; selector < 4; ++selector) {
        for (const bool load : {false, true}) {
          result.push_back({.kind = Kind::mixed_one,
                            .selector = selector,
                            .mode = 3,
                            .load = load,
                            .y = y,
                            .source_b = selector % 2 != 0,
                            .second_high = selector % 2 == 0});
        }
        if (selector < 2) {
          result.push_back(
              {.kind = Kind::mixed_two, .selector = selector, .mode = 3, .load = false, .y = y});
        }
      }
    }
  }
  return result;
}

void precision(Cases &cases, const std::span<const Form> forms) {
  for (const auto form : forms) {
    if (form.kind == Kind::immediate) { continue; }
    for (const auto mode : modes) {
      for (std::size_t index = 0; index < values.size(); ++index) {
        cases.push_back(single(
            form,
            {.status = mode | (index % 2 == 0 ? 0U : flag_s | flag_l),
             .a = values[index],
             .b = values[(index + 5) % values.size()],
             .data = {.high = static_cast<Word>(values[index] & word_mask),
                      .low = static_cast<Word>((values[(index + 7) % values.size()] >> word_bits) &
                                               word_mask)}}));
      }
    }
    for (Word flags = 0; flags <= status_byte; ++flags) {
      cases.push_back(single(form, {.status = flags}));
    }
  }
}

void basis(Cases &cases) {
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (const bool load : {false, true}) {
      for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
        for (const bool inverted : {false, true}) {
          const auto raw = (std::uint64_t{1} << bit) ^ (inverted ? accumulator_mask : 0);
          const auto data = raw & move_model::long_mask;
          cases.push_back(single({.selector = selector, .load = load},
                                 {.a = raw,
                                  .b = raw ^ accumulator_mask,
                                  .data = {.high = static_cast<Word>(data >> word_bits),
                                           .low = static_cast<Word>(data & word_mask)}}));
        }
      }
    }
  }
}

void agu(Cases &cases, const std::span<const Form> forms) {
  for (auto form : forms) {
    if (form.kind == Kind::immediate) { continue; }
    for (const Word modifier : {0xab0000U, 0xab000eU, 0xab800fU}) {
      for (unsigned mode = 0; mode < 8; ++mode) {
        if (mode == 6) { continue; }
        form.mode = mode;
        for (unsigned index = 0; index < agu_registers; ++index) {
          form.index = index;
          const Word cursor = modifier == 0xab000e ? location + 14 : location + 15;
          cases.push_back(single(form, {.modifier = modifier, .cursor = cursor}));
        }
      }
    }
  }
}

void parallel_alu(Cases &cases, const std::span<const Form> forms) {
  for (const auto form : forms) {
    for (const auto mode : modes) {
      for (const Word alu : {3U, 11U, 0x45U}) {
        cases.push_back(
            single(form, {.status = mode | status_byte, .a = sign56, .b = sign56 - 1}, alu));
      }
    }
  }
}

void repeated(Cases &cases, const Form form) {
  constexpr Word count = 4;
  if (move_model::words(form) != 1) { return; }
  for (const Word budget : {1U, 2U, 3U, count + 1}) {
    auto item = single(form);
    item.before.front().words.insert(item.before.front().words.begin(), repeat_base | (count << 8));
    item.expected = item.initial;
    item.after.clear();
    for (Word iteration = 1; iteration < budget; ++iteration) {
      auto address_state = item.expected;
      const auto address =
          form.kind == Kind::immediate ? 0 : move_model::address(address_state, form);
      if (form.kind != Kind::immediate) {
        item.before.push_back({.words = {move_model::Data{}.high},
                               .address = address,
                               .space = form.y ? Space::y : Space::x});
        if (form.kind == Kind::long_word) {
          item.before.push_back(
              {.words = {move_model::Data{}.low}, .address = address, .space = Space::y});
        }
      }
      execute(item, form, {}, 0);
    }
    const bool completed = budget == count + 1;
    item.expected.pc = start + (completed ? 2U : 1U);
    item.expected.lc = completed ? item.initial.lc : count + 1 - budget;
    item.budget = item.retired = budget;
    item.progress = {.instructions = budget,
                     .estimated_cycles = 5 + ((budget - 1) * move_model::cycles(form))};
    item.interrupt_ready = completed;
    item.name = "Extended MOVE REP current operands/AGU and exact partial budget";
    cases.push_back(std::move(item));
  }
}

void ending(Cases &cases, const Form form) {
  for (const Word pointer : {0U, 15U, 31U, 63U}) {
    for (const Word count : {1U, 2U}) {
      for (const Word saved : {0U, lf, fv, lf | fv}) {
        auto item = single(form);
        auto &state = item.initial;
        state.sr = (state.sr & ~fv) | lf;
        state.la = start + move_model::words(form) - 1;
        state.lc = count;
        state.sp = pointer;
        state.sc = 2;
        state.stack[pointer % stack_entries] = (std::uint64_t{start} << word_bits) | saved;
        state.stack[(pointer + pointer_range - 1) % stack_entries] =
            (std::uint64_t{saved_la} << word_bits) | 7U;
        item.expected = state;
        item.after.clear();
        execute(item, form, {}, 0);
        item.expected.pc = count == 1 ? start + move_model::words(form) : start;
        if (count == 1) {
          item.expected.la = saved_la;
          item.expected.lc = 7;
          item.expected.sp = (pointer + pointer_range - 2) % pointer_range;
          item.expected.sc = 0;
          item.expected.sr = (item.expected.sr & ~(lf | fv)) | saved;
        } else {
          --item.expected.lc;
        }
        item.name = "Extended MOVE at DO opcode/extension ending restores exact frames and LF/FV";
        cases.push_back(std::move(item));
      }
    }
  }
}

void sequence(Cases &cases, const Form form) {
  auto item = single(form);
  const auto words = program(form);
  std::vector<Word> ops{mac_a, mac_b};
  ops.insert(ops.end(), words.begin(), words.end());
  ops.insert(ops.end(), {adc_x_a, round_a});
  item.before.front().words = ops;
  item.expected = item.initial;
  item.after.clear();
  item.budget = 0;
  item.progress = {};
  for (std::size_t index = 0; index < ops.size(); ++index) {
    const bool transfer = index == 2;
    if (transfer) {
      execute(item, form, {}, 0);
    } else {
      const auto old = item.expected;
      sequence_model::alu(item.expected, old, ops[index]);
    }
    item.expected.pc += transfer ? move_model::words(form) : 1U;
    item.progress.estimated_cycles += transfer ? move_model::cycles(form) : 1U;
    item.retired = ++item.budget;
    item.progress.instructions = item.budget;
    item.name = "Native/extended-MOVE/native Carry and rounding: every prefix";
    cases.push_back(item);
    if (transfer && words.size() == 2) { ++index; }
  }
}

void interactions(Cases &cases, const std::span<const Form> forms) {
  parallel_alu(cases, forms);
  for (const auto form : forms) {
    repeated(cases, form);
    ending(cases, form);
    sequence(cases, form);
  }
}

void unsupported_modes(Cases &cases, const std::span<const Form> forms) {
  for (const auto form : forms) {
    for (const Word mode : {0x2000U, 0x4000U, 0x20000U, 0x100000U, mode_up | mode_down}) {
      if (mode == saturation_model::mode) {
        cases.push_back(single(form, {.status = mode}));
        continue;
      }
      auto item = single(form);
      item.initial.sr = mode;
      item.expected = item.initial;
      item.expected.pc += move_model::words(form);
      item.after.clear();
      item.progress = {};
      item.retired = 0;
      item.interrupt_ready = false;
      item.fault = {
          .kind = FaultKind::unsupported_mode, .address = start, .value = mode, .pc = start};
      item.name = "Extended MOVE unsupported numeric modes are explicit library diagnostics";
      cases.push_back(std::move(item));
    }
  }
}

void missing_extension(Cases &cases, const std::span<const Form> forms) {
  for (auto form : forms) {
    if (form.kind == Kind::immediate || form.kind == Kind::mixed_two) { continue; }
    form.mode = 6;
    form.index = 0;
    auto item = single(form);
    item.memory_words[0] = start + 1;
    item.before.front().words.resize(1);
    item.expected = item.initial;
    item.after.clear();
    item.progress = {};
    item.retired = 0;
    item.interrupt_ready = false;
    item.fault = {.kind = FaultKind::memory_read, .address = start + 1, .pc = start};
    item.name = "Extended MOVE missing P extension performs no data/AGU/flag effects";
    cases.push_back(std::move(item));
  }
}

void invalid_mixed_variants(Cases &cases, Form form) {
  for (const bool source_b : {false, true}) {
    for (const bool second_high : {false, true}) {
      form.source_b = source_b;
      form.second_high = second_high;
      auto ordinary = form;
      ordinary.mode = 4;
      auto item = single(ordinary);
      item.before.front().words = program(form);
      item.expected = item.initial;
      item.expected.pc += 2;
      item.after.assign(item.before.begin() + 1, item.before.end());
      item.after.push_back({.words = {0}, .space = Space::x});
      item.after.push_back({.words = {0}, .space = Space::y});
      item.retired = 0;
      item.progress = {};
      item.interrupt_ready = false;
      item.fault = {.kind = FaultKind::invalid_instruction,
                    .address = start,
                    .value = form.index,
                    .pc = start};
      item.name = "Class I mixed MOVE rejects reserved/immediate-store EAs before bus effects";
      cases.push_back(std::move(item));
    }
  }
}

void invalid_mixed(Cases &cases) {
  for (const bool y : {false, true}) {
    for (unsigned selector = 0; selector < 4; ++selector) {
      for (const bool load : {false, true}) {
        for (unsigned index = 0; index < agu_registers; ++index) {
          if (index == 0 || (index == 4 && load)) { continue; }
          invalid_mixed_variants(cases,
                                 {.kind = Kind::mixed_one,
                                  .selector = selector,
                                  .mode = 6,
                                  .index = index,
                                  .load = load,
                                  .y = y});
        }
      }
    }
  }
}

void invalid_class_two(Cases &cases) {
  for (const bool y : {false, true}) {
    for (unsigned selector = 0; selector < 2; ++selector) {
      for (unsigned index = 0; index < agu_registers; ++index) {
        const Form form{.kind = Kind::mixed_two,
                        .selector = selector,
                        .mode = 6,
                        .index = index,
                        .load = false,
                        .y = y};
        auto ordinary = form;
        ordinary.mode = 4;
        auto item = single(ordinary);
        item.before.front().words = {move_model::opcode(form)};
        item.expected = item.initial;
        ++item.expected.pc;
        item.after.assign(item.before.begin() + 1, item.before.end());
        item.retired = 0;
        item.progress = {};
        item.interrupt_ready = false;
        item.fault = {.kind = FaultKind::invalid_instruction,
                      .address = start,
                      .value = move_model::opcode(form),
                      .pc = start};
        item.name = "Class II mixed MOVE prohibits every EA-extension selector";
        cases.push_back(std::move(item));
      }
    }
  }
}

void invalid_long(Cases &cases) {
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (const bool load : {false, true}) {
      for (unsigned index = 1; index < agu_registers; ++index) {
        const Form form{.selector = selector, .mode = 6, .index = index, .load = load};
        auto item = single({.selector = selector, .load = load});
        item.before.front().words = program(form);
        item.expected = item.initial;
        item.expected.pc += 2;
        item.progress = {};
        item.retired = 0;
        item.interrupt_ready = false;
        item.fault = {
            .kind = FaultKind::invalid_instruction, .address = start, .value = index, .pc = start};
        item.after.push_back({.words = {0}, .space = Space::x});
        item.after.push_back({.words = {0}, .space = Space::y});
        item.name =
            "L memory allows long absolute, never immediate or reserved EA-extension selectors";
        cases.push_back(std::move(item));
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
          "Usage: dsp56300_generate_extended_moves immediate|long|mixed OUTPUT.d56");
    }
    const std::string_view mode(argv[1]);
    using dsp56300::test::Kind;
    auto kind = Kind::mixed_one;
    if (mode == "immediate") { kind = Kind::immediate; }
    if (mode == "long") { kind = Kind::long_word; }
    dsp56300::test::Cases cases;
    if (mode == "immediate") {
      dsp56300::test::immediate_matrix(cases);
    } else if (mode == "long") {
      dsp56300::test::long_matrix(cases);
      dsp56300::test::basis(cases);
    } else if (mode == "mixed") {
      dsp56300::test::mixed_matrix(cases);
    } else {
      throw std::runtime_error("Unknown extended MOVE group");
    }
    const auto forms = dsp56300::test::representative(kind);
    dsp56300::test::precision(cases, forms);
    dsp56300::test::agu(cases, forms);
    dsp56300::test::interactions(cases, forms);
    dsp56300::test::unsupported_modes(cases, forms);
    dsp56300::test::missing_extension(cases, forms);
    if (mode == "long") { dsp56300::test::invalid_long(cases); }
    if (mode == "mixed") {
      dsp56300::test::invalid_mixed(cases);
      dsp56300::test::invalid_class_two(cases);
    }
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[2], std::move(cases));
    std::cout << count << " independent " << mode << " MOVE cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
