// SPDX-License-Identifier: GPL-3.0-only
// Original SM arithmetic/bypass programs with independent complete-state expectations.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "saturation_products.hpp"
#include "selection_model.hpp"
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
using Cases = std::vector<BinaryCase>;
constexpr Word no_move = 0x200000, inc = 8, dec = 10;
constexpr unsigned extension_position = 48;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 16> boundaries{0,
                                                   1,
                                                   (std::uint64_t{1} << 23) - 1,
                                                   std::uint64_t{1} << 23,
                                                   (std::uint64_t{1} << 23) + 1,
                                                   saturation_model::positive - 1,
                                                   saturation_model::positive,
                                                   saturation_model::positive + 1,
                                                   saturation_model::negative - 1,
                                                   saturation_model::negative,
                                                   saturation_model::negative + 1,
                                                   sign56 - 1,
                                                   sign56,
                                                   sign56 + 1,
                                                   accumulator_mask - 1,
                                                   accumulator_mask};

Registers seed(const Word status) {
  auto state = register_model::seed(status);
  state.sr = status | saturation_model::mode;
  return state;
}

void append(Cases &cases,
            const Registers &initial,
            Registers expected,
            std::vector<Word> program,
            const std::string &name) {
  BinaryCase item;
  item.initial = initial;
  item.expected = expected;
  item.expected.pc += static_cast<Word>(program.size());
  item.progress.estimated_cycles = program.size();
  item.before = {{.words = std::move(program), .address = start}};
  item.name = name;
  cases.push_back(std::move(item));
}

void single(Cases &cases, const Registers &initial, const Word alu) {
  auto expected = initial;
  saturation_model::alu(expected, initial, alu);
  append(cases, initial, expected, {no_move | alu}, "SM independent raw arithmetic and flags");
}

void table(Cases &cases) {
  constexpr std::uint64_t low_part = 0x123456;
  constexpr Word add_x0 = 0x40;
  for (const auto mode : modes) {
    for (Word extension = 0; extension <= status_byte; ++extension) {
      for (const bool high : {false, true}) {
        for (const bool destination_b : {false, true}) {
          auto state = seed(mode | status_byte);
          state.x = 0;
          const auto raw = (std::uint64_t{extension} << extension_position) |
                           (high ? std::uint64_t{1} << 47 : 0) | low_part;
          (destination_b ? state.b : state.a) = raw;
          single(cases, state, add_x0 | (destination_b ? 8U : 0U));
        }
      }
    }
  }
}

struct Sum {
  Word alu, mode;
};

void sum_shape(Cases &cases, const Sum input) {
  for (const auto raw : boundaries) {
    for (const auto incoming : {0U, status_byte}) {
      auto state = seed(input.mode | incoming);
      state.a = raw;
      state.b = accumulator_mask ^ raw;
      single(cases, state, input.alu);
    }
  }
}

void sums(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned selector = 1; selector < 8; ++selector) {
      for (const bool destination_b : {false, true}) {
        for (const Word kind : {0U, 4U}) {
          sum_shape(cases,
                    {.alu = (selector << 4) | (destination_b ? 8U : 0U) | kind, .mode = mode});
        }
      }
    }
    for (const Word alu : {0x21U, 0x29U, 0x31U, 0x39U, 0x25U, 0x2dU, 0x35U, 0x3dU}) {
      sum_shape(cases, {.alu = alu, .mode = mode});
    }
  }
}

void products(Cases &cases) {
  constexpr std::array<Word, 4> words{0, 1, sign24, word_mask};
  for (const auto mode : modes) {
    for (Word alu = 0x80; alu <= status_byte; ++alu) {
      for (std::size_t index = 0; index < words.size(); ++index) {
        auto state = seed(mode | status_byte);
        state.x = (std::uint64_t{words[index]} << word_bits) | words[(index + 1) % words.size()];
        state.y = (std::uint64_t{words[(index + 2) % words.size()]} << word_bits) |
                  words[(index + 3) % words.size()];
        state.a = boundaries[index + 5];
        state.b = boundaries[index + 8];
        single(cases, state, alu);
      }
    }
  }
}

void unaries(Cases &cases) {
  for (const auto mode : modes) {
    for (const Word alu : {0x11U, 0x13U, 0x26U, 0x36U, 0x01U, 0x41U, 0x51U, 0x61U, 0x71U}) {
      for (const bool destination_b : {false, true}) {
        sum_shape(cases, {.alu = alu | (destination_b ? 8U : 0U), .mode = mode});
      }
    }
  }
}

void units(Cases &cases) {
  for (const auto mode : modes) {
    for (const auto raw : boundaries) {
      for (const bool subtract : {false, true}) {
        for (const bool destination_b : {false, true}) {
          auto state = seed(mode | status_byte);
          (destination_b ? state.b : state.a) = raw;
          auto expected = state;
          saturation_model::unit(expected, subtract, destination_b);
          append(cases,
                 state,
                 expected,
                 {(subtract ? dec : inc) | (destination_b ? 1U : 0U)},
                 "SM raw-LSB increment/decrement independent result and carry");
        }
      }
    }
  }
}

void all_flags(Cases &cases) {
  constexpr std::array<Word, 4> operations{0x40, 0x21, 0xd3, 0x11};
  for (const auto mode : modes) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      auto state = seed(mode | flags);
      state.a = (flags & 1U) == 0 ? saturation_model::positive : saturation_model::negative - 1;
      state.b = saturation_model::positive + 1;
      for (const auto alu : operations) {
        for (const bool destination_b : {false, true}) {
          single(cases, state, alu | (destination_b ? 8U : 0U));
        }
      }
    }
  }
}

void immediates(Cases &cases) {
  constexpr Word immediate_prefix = 0x0140c0;
  for (const auto mode : modes) {
    for (const auto raw : boundaries) {
      for (const Word operand : {0U, 1U, sign24 - 1, sign24, word_mask}) {
        for (const bool subtract : {false, true}) {
          auto state = seed(mode | status_byte);
          state.a = raw;
          auto expected = state;
          auto old = state;
          old.x = operand;
          const Word alu = subtract ? 0x44U : 0x40U;
          saturation_model::alu(expected, old, alu);
          append(cases,
                 state,
                 expected,
                 {immediate_prefix | (subtract ? 4U : 0U), operand},
                 "SM two-word immediate sum preserves source registers and exact base costs");
        }
      }
    }
  }
}

void extended_ordinary_shape(Cases &cases,
                             const Word mode,
                             saturation_products::Operation operation) {
  for (const bool destination_b : {false, true}) {
    operation.destination_b = destination_b;
    for (Word variant = 0; variant < 8; ++variant) {
      operation.variant = variant;
      auto state = seed(mode | status_byte);
      state.a = saturation_model::positive;
      state.b = saturation_model::negative - 1;
      saturation_products::put(
          state, {.selector = operation.left, .value = variant % 2 == 0 ? sign24 : word_mask});
      saturation_products::ordinary(cases, state, operation);
    }
  }
}

void extended_ordinary(Cases &cases) {
  constexpr std::array<Word, 7> coefficients{
      0, 1, sign24 - 1, sign24, sign24 + 1, word_mask - 1, word_mask};
  for (const auto mode : modes) {
    for (unsigned source = 0; source < 4; ++source) {
      for (const auto coefficient : coefficients) {
        extended_ordinary_shape(cases, mode, {.left = source, .coefficient = coefficient});
      }
      for (Word power = 1; power < word_bits; ++power) {
        extended_ordinary_shape(cases, mode, {.left = source, .power = power});
      }
    }
  }
}

void mixed_shape(Cases &cases, const Word mode, saturation_products::Operation operation) {
  for (const bool destination_b : {false, true}) {
    operation.destination_b = destination_b;
    for (const Word variant : {0U, 4U}) {
      operation.variant = variant;
      for (const bool extended : {false, true}) {
        auto state = seed(mode | status_byte);
        state.a = extended ? sign56 - 1 : saturation_model::positive;
        state.b = extended ? sign56 : saturation_model::negative - 1;
        saturation_products::put(state, {.selector = operation.left, .value = sign24});
        saturation_products::put(state, {.selector = operation.right, .value = word_mask});
        saturation_products::mixed(cases, state, operation);
      }
    }
  }
}

void mixed_variants(Cases &cases, const Word mode, saturation_products::Operation operation) {
  for (unsigned signs = 1; signs <= 2; ++signs) {
    operation.signs = signs;
    for (const bool accumulate : {false, true}) {
      operation.accumulate = accumulate;
      mixed_shape(cases, mode, operation);
    }
  }
  operation.double_product = operation.accumulate = true;
  for (unsigned signs = 0; signs <= 2; ++signs) {
    operation.signs = signs;
    mixed_shape(cases, mode, operation);
  }
}

void extended_mixed(Cases &cases) {
  for (const auto mode : modes) {
    for (unsigned left = 0; left < 4; ++left) {
      for (unsigned right = 0; right < 4; ++right) {
        mixed_variants(cases, mode, {.left = left, .right = right});
      }
    }
  }
}

struct ConditionalTransfer {
  Word condition, source;
  bool destination_b, taken, dual;
};

void conditional_transfer(Cases &cases, const Registers &initial, const ConditionalTransfer input) {
  constexpr Word data_prefix = 0x020000;
  constexpr Word dual_bit = 0x010000;
  constexpr Word address_fields = 0x0105;
  auto expected = initial;
  const Word alu = (input.source << 4) | (input.destination_b ? 8U : 0U);
  if (input.taken) {
    (input.destination_b ? expected.b : expected.a) =
        static_cast<std::uint64_t>(sequence_model::source(initial, alu)) & accumulator_mask;
    if (input.dual) { expected.r[5] = initial.r[1]; }
  }
  const Word opcode =
      data_prefix | (input.condition << 12) | alu | (input.dual ? dual_bit | address_fields : 0U);
  append(cases,
         initial,
         expected,
         {opcode},
         "SM Tcc preserves full unscaled operands/CCR and conditional address transfers");
}

void conditional_transfers(Cases &cases) {
  std::array<std::array<Word, 2>, 16> witnessed{};
  for (const auto mode : modes) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      const Word condition = (flags ^ (flags >> 4)) & 15;
      const bool taken = selection_model::test_condition({.status = flags, .selector = condition});
      ++witnessed[condition][taken ? 1 : 0];
      auto initial = seed(mode | flags);
      initial.a = saturation_model::positive + 1;
      initial.b = saturation_model::negative - 1;
      for (const bool destination_b : {false, true}) {
        for (const Word source : {0U, 4U, 5U, 6U, 7U}) {
          conditional_transfer(cases,
                               initial,
                               {.condition = condition,
                                .source = source,
                                .destination_b = destination_b,
                                .taken = taken,
                                .dual = (flags & flag_s) != 0});
        }
      }
    }
  }
  for (const auto &counts : witnessed) {
    if (counts[0] == 0 || counts[1] == 0) {
      throw std::runtime_error("SM Tcc matrix must take and reject all sixteen conditions");
    }
  }
}

void conditional_arithmetic(Cases &cases) {
  constexpr Word prefix = 0x202000;
  constexpr Word update_bit = 0x1000;
  constexpr Word alu = 0xd3;
  for (Word condition = 0; condition < 16; ++condition) {
    for (const bool taken : {false, true}) {
      Word flags = 0;
      while (selection_model::test_condition({.status = flags, .selector = condition}) != taken) {
        ++flags;
        if (flags > status_byte) { throw std::runtime_error("Missing condition polarity"); }
      }
      for (const bool update : {false, true}) {
        auto initial = seed(modes[condition % modes.size()] | flags);
        initial.a = saturation_model::positive + 1;
        initial.x = 0;
        auto expected = initial;
        if (taken) { saturation_model::alu(expected, initial, alu); }
        if (!update) { expected.sr = initial.sr; }
        append(cases,
               initial,
               expected,
               {prefix | (condition << 8) | (update ? update_bit : 0U) | alu},
               "SM IFcc/IFcc.U keeps conditional saturation separate from CCR publication");
      }
    }
  }
}

} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_saturation OUTPUT.d56"); }
    dsp56300::test::Cases cases;
    dsp56300::test::table(cases);
    dsp56300::test::sums(cases);
    dsp56300::test::products(cases);
    dsp56300::test::unaries(cases);
    dsp56300::test::units(cases);
    dsp56300::test::all_flags(cases);
    dsp56300::test::immediates(cases);
    dsp56300::test::extended_ordinary(cases);
    dsp56300::test::extended_mixed(cases);
    dsp56300::test::conditional_transfers(cases);
    dsp56300::test::conditional_arithmetic(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent SM arithmetic/bypass cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
