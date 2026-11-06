// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace dsp56300::test {
namespace {
enum class LogicKind : std::uint8_t { bit_or, bit_xor, bit_and, bit_not, lsr, lsl, ror, rol };

struct Operation {
  LogicKind kind;
  Word byte;
  bool binary = false;
};

constexpr std::array operations{Operation{.kind = LogicKind::bit_or, .byte = 0x42, .binary = true},
                                Operation{.kind = LogicKind::bit_xor, .byte = 0x43, .binary = true},
                                Operation{.kind = LogicKind::bit_and, .byte = 0x46, .binary = true},
                                Operation{.kind = LogicKind::bit_not, .byte = 0x17},
                                Operation{.kind = LogicKind::lsr, .byte = 0x23},
                                Operation{.kind = LogicKind::lsl, .byte = 0x33},
                                Operation{.kind = LogicKind::ror, .byte = 0x27},
                                Operation{.kind = LogicKind::rol, .byte = 0x37}};

struct Part {
  Word selector;
  bool b = false, extension = false;
};

// Independent FM selector table and physical view identities.
constexpr std::array parts{Part{.selector = 8},
                           Part{.selector = 9, .b = true},
                           Part{.selector = 10, .extension = true},
                           Part{.selector = 11, .b = true, .extension = true}};
constexpr Word extension_byte = 0xff;
constexpr std::uint64_t main_mask = std::uint64_t{word_mask} << word_bits;
constexpr std::uint64_t extension_mask = std::uint64_t{extension_byte} << (word_bits * 2);
constexpr std::uint64_t initial_a = 0x1200abc1a5b6c7;
constexpr std::uint64_t initial_b = 0x650055a3328175;
constexpr std::uint64_t x_pair = 0x579bdf13579b;
constexpr std::uint64_t y_high = 0x642086;
constexpr Word move_y0 = 0x20c000;
constexpr Word destination_b = 8;
std::size_t cases = 0;

struct Inputs {
  Word main, source;
  bool carry;
};

struct WordResult {
  Word main;
  bool carry;
};

WordResult expected_word(const LogicKind kind, const Inputs input) {
  Word value = 0;
  bool carry = input.carry;
  switch (kind) {
    case LogicKind::bit_or: value = input.main | input.source; break;
    case LogicKind::bit_xor: value = input.main ^ input.source; break;
    case LogicKind::bit_and: value = input.main & input.source; break;
    case LogicKind::bit_not: value = input.main ^ word_mask; break;
    case LogicKind::lsr:
    case LogicKind::ror:
      value = input.main / 2;
      if (kind == LogicKind::ror && input.carry) { value |= sign24; }
      carry = (input.main & 1U) != 0;
      break;
    case LogicKind::lsl:
    case LogicKind::rol:
      value = (input.main * 2) & word_mask;
      if (kind == LogicKind::rol && input.carry) { ++value; }
      carry = (input.main & sign24) != 0;
      break;
    default: throw std::runtime_error("Unknown independently specified word operation");
  }
  return {.main = value, .carry = carry};
}

struct Encoding {
  Operation operation;
  unsigned source;
  bool b;
  Part part;
};

Word encode(const Encoding input) {
  const auto alu = input.operation.byte | (input.operation.binary ? input.source << 4 : 0U) |
                   (input.b ? destination_b : 0U);
  return move_y0 | (input.part.selector << 8) | alu;
}

Registers expected(const Registers &before, const Encoding input) {
  const std::array sources{static_cast<Word>(before.x) & word_mask,
                           static_cast<Word>(before.y) & word_mask,
                           static_cast<Word>(before.x >> word_bits),
                           static_cast<Word>(before.y >> word_bits)};
  const auto old = input.b ? before.b : before.a;
  const auto word = expected_word(input.operation.kind,
                                  {.main = static_cast<Word>(old >> word_bits) & word_mask,
                                   .source = sources.at(input.source),
                                   .carry = (before.sr & flag_c) != 0});
  auto result = before;
  // The disjoint native MOVE field and ALU field have separate write masks.
  auto &moved = input.part.b ? result.b : result.a;
  const auto payload = before.y & word_mask;
  moved = input.part.extension
              ? (moved & ~extension_mask) | ((payload & extension_byte) << (word_bits * 2))
              : (moved & ~std::uint64_t{word_mask}) | payload;
  auto &alu = input.b ? result.b : result.a;
  alu = (alu & ~main_mask) | (std::uint64_t{word.main} << word_bits);
  constexpr Word replaced = flag_c | flag_v | flag_z | flag_n;
  result.sr &= ~replaced;
  if (word.carry) { result.sr |= flag_c; }
  if (word.main == 0) { result.sr |= flag_z; }
  if ((word.main & sign24) != 0) { result.sr |= flag_n; }
  ++result.pc;
  return result;
}

void check(TestCore &core, const Encoding input, const Registers &state) {
  ++cases;
  core.set_registers(state);
  const auto before = core.progress();
  const auto run = core.run(1);
  require(run.retired == 1 && run.fault.kind == FaultKind::none,
          "A legal parallel partial-word operation retires without a fault");
  const auto wanted = expected(state, input);
  if (core.registers() != wanted) {
    throw std::runtime_error(
        "Partial accumulator mismatch, opcode=" + std::to_string(encode(input)) +
        " source=" + std::to_string(input.source));
  }
  require(core.registers() == wanted,
          "Original main/source operands, actual partial MOVE and independent CCR agree");
  require(core.progress().instructions == before.instructions + 1,
          "A partial-word instruction changes retirement exactly once");
}

void values(TestCore &core, const Encoding input) {
  constexpr std::array mains{Word{0}, Word{1}, Word{0x7fffff}, sign24, word_mask, Word{0x13579b}};
  constexpr std::array payloads{Word{0}, Word{0x2468ac}, word_mask};
  for (const auto main : mains) {
    for (const auto payload : payloads) {
      auto state = initial_state();
      state.pc = 1;
      state.a = initial_a;
      state.b = initial_b;
      state.x = x_pair;
      state.y = (y_high << word_bits) | payload;
      auto &accumulator = input.b ? state.b : state.a;
      accumulator = (accumulator & ~main_mask) | (std::uint64_t{main} << word_bits);
      for (const bool carry : {false, true}) {
        constexpr Word flags_except_carry = 0xfe;
        state.sr = flags_except_carry | (carry ? flag_c : 0U);
        check(core, input, state);
      }
    }
  }
}

struct Configuration {
  Backend backend;
};

void backend(const Configuration configuration) {
  std::array<Word, 16> p{};
  std::array<Word, 16> x{};
  std::array<Word, 16> y{};
  TestCore core({.program = p, .x = x, .y = y}, nullptr, {.backend = configuration.backend});
  for (const auto operation : operations) {
    for (const bool b : {false, true}) {
      for (const auto part : parts) {
        for (unsigned source = 0; source < (operation.binary ? 4U : 1U); ++source) {
          const Encoding input{.operation = operation, .source = source, .b = b, .part = part};
          core.write(Space::program, 1, encode(input));
          values(core, input);
        }
      }
    }
  }
  require(x == std::array<Word, 16>{} && y == std::array<Word, 16>{},
          "Register-only parallel instructions cannot alter X/Y RAM");
}

void literal() {
  constexpr Word opcode = 0x20c94b;
  constexpr std::uint64_t old_b = 0x12000010abcdef;
  constexpr std::uint64_t new_b = 0x1213578b2468ac;
  std::array<Word, 16> p{};
  std::array<Word, 16> x{};
  std::array<Word, 16> y{};
  p[1] = opcode;
  TestCore core({.program = p, .x = x, .y = y});
  auto state = initial_state();
  state.pc = 1;
  state.b = old_b;
  constexpr Word x0 = 0x13579b;
  constexpr Word y0 = 0x2468ac;
  state.x = x0;
  state.y = y0;
  state.sr = flag_c;
  core.set_registers(state);
  const auto run = core.run(1);
  state.pc = 2;
  state.b = new_b;
  require(run.retired == 1 && run.fault.kind == FaultKind::none && core.registers() == state,
          "Literal EOR X0,B / Y0->B0 red input retains its independently expected full state");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300;
    constexpr std::array configurations{test::Configuration{.backend = Backend::jit}};
    test::literal();
    for (const auto configuration : configurations) { test::backend(configuration); }
    std::cout << test::checks << " partial accumulator assertions passed across " << test::cases
              << " independently described cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
