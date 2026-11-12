// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "instruction_decoder.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test {
namespace {
struct Expected {
  Word opcode;
  InstructionKind kind;
  unsigned words;
  bool repeatable;
};

// Independently encoded ISA forms; no assembler, device program or TestCore instance.
constexpr std::array cases{
    Expected{.opcode = 0x000000, .kind = InstructionKind::nop, .words = 1, .repeatable = true},
    Expected{.opcode = 0x200040, .kind = InstructionKind::parallel, .words = 1, .repeatable = true},
    Expected{.opcode = 0x44d800, .kind = InstructionKind::parallel, .words = 1, .repeatable = true},
    Expected{
        .opcode = 0x44f400, .kind = InstructionKind::parallel, .words = 2, .repeatable = false},
    Expected{.opcode = 0x07f084,
             .kind = InstructionKind::move_program_effective,
             .words = 2,
             .repeatable = false},
    Expected{.opcode = 0x0604a0,
             .kind = InstructionKind::repeat_immediate,
             .words = 1,
             .repeatable = false},
    Expected{.opcode = 0x06d300,
             .kind = InstructionKind::loop_register,
             .words = 2,
             .repeatable = false},
    Expected{
        .opcode = 0x0c0300, .kind = InstructionKind::jump_short, .words = 1, .repeatable = false},
    Expected{
        .opcode = 0x0d0300, .kind = InstructionKind::call_short, .words = 1, .repeatable = false},
    Expected{.opcode = 0x0af080,
             .kind = InstructionKind::jump_effective,
             .words = 2,
             .repeatable = false},
    Expected{.opcode = 0x0bf080,
             .kind = InstructionKind::call_effective,
             .words = 2,
             .repeatable = false},
    Expected{.opcode = 0x000004,
             .kind = InstructionKind::return_interrupt,
             .words = 1,
             .repeatable = false},
    Expected{.opcode = 0x00000c,
             .kind = InstructionKind::return_subroutine,
             .words = 1,
             .repeatable = false},
    Expected{.opcode = 0x000084,
             .kind = InstructionKind::reset_peripherals,
             .words = 1,
             .repeatable = true},
    Expected{.opcode = 0x000086, .kind = InstructionKind::wait, .words = 1, .repeatable = false},
    Expected{.opcode = 0x000087, .kind = InstructionKind::stop, .words = 1, .repeatable = false},
    Expected{.opcode = 0x000005, .kind = InstructionKind::illegal, .words = 1, .repeatable = true},
    Expected{.opcode = 0x000006, .kind = InstructionKind::trap, .words = 1, .repeatable = false},
    Expected{.opcode = 0x00000f,
             .kind = InstructionKind::cache_lock_relative,
             .words = 2,
             .repeatable = false}};

void reserved_domains() {
  constexpr std::array legal{Word{0x0001b8},
                             Word{0x0001b9},
                             Word{0x0001ba},
                             Word{0x0001bb},
                             Word{0x0001f8},
                             Word{0x0001f9},
                             Word{0x0001fa},
                             Word{0x0001fb}};
  constexpr std::array domains{std::array{Word{0x000100}, Word{0x0001ff}},
                               std::array{Word{0x0d2000}, Word{0x0d2fff}}};
  for (const auto domain : domains) {
    for (Word word = domain[0]; word <= domain[1]; ++word) {
      const auto shape = inspect_instruction(word);
      if (!shape) { throw std::runtime_error("Missing audited instruction shape"); }
      require(shape->words == 1 && shape->repeatable,
              "Every word in the audited bounded domains has its exact one-word shape");
      const bool ordinary = std::ranges::find(legal, word) != legal.end();
      const auto control =
          word < legal[4] ? InstructionKind::and_control : InstructionKind::or_control;
      const auto expected = ordinary ? control : InstructionKind::undefined;
      require(shape->kind == expected,
              "All eight genuine control operations remain distinct from undefined encodings");
    }
  }
  for (const Word word : {Word{0x000094}, Word{0x0d3000}}) {
    const auto shape = inspect_instruction(word);
    require(!shape, "Unaudited neighboring encodings retain visible unsupported classification");
  }
}

auto low_control_forms() {
  std::array<bool, 65536> legal{};
  using isa::Opcode;
  for (const auto operation : {Opcode::nop,
                               Opcode::pflushun,
                               Opcode::pfree,
                               Opcode::pflush,
                               Opcode::rti,
                               Opcode::illegal,
                               Opcode::trap,
                               Opcode::rts,
                               Opcode::plockr,
                               Opcode::punlockr,
                               Opcode::reset,
                               Opcode::wait,
                               Opcode::stop,
                               Opcode::enddo,
                               Opcode::debug,
                               Opcode::do_forever,
                               Opcode::dor_forever}) {
    legal[static_cast<Word>(operation)] = true;
  }
  for (Word reg = 0; reg < 2; ++reg) {
    legal[static_cast<Word>(Opcode::inc_a) | reg] = true;
    legal[static_cast<Word>(Opcode::dec_a) | reg] = true;
  }
  for (Word condition = 0; condition < 16; ++condition) {
    for (const auto operation : {Opcode::brkcc, Opcode::debugcc, Opcode::trapcc}) {
      legal[static_cast<Word>(operation) | condition] = true;
    }
  }
  for (Word immediate = 0; immediate < 256; ++immediate) {
    for (Word control = 0; control < 4; ++control) {
      legal[static_cast<Word>(Opcode::andi_control) | (immediate << 8) | control] = true;
      legal[static_cast<Word>(Opcode::ori_control) | (immediate << 8) | control] = true;
    }
  }
  // FM MOVEP qq,P:ea: bit15=1, bit7=0; all fourteen operand bits remain unconstrained.
  constexpr Word fields_end = Word{1} << 14;
  constexpr Word low_fields = 0x7f;
  constexpr Word high_fields = 0x3f80;
  for (Word fields = 0; fields < fields_end; ++fields) {
    const auto word = static_cast<Word>(Opcode::movep_program_low) | (fields & low_fields) |
                      ((fields & high_fields) << 1);
    legal[word] = true;
  }
  return legal;
}

void low_control_domain() {
  const auto legal = low_control_forms();
  constexpr std::array uncertain{Word{0x94},
                                 Word{0x96},
                                 Word{0x97},
                                 Word{0x9c},
                                 Word{0xc4},
                                 Word{0xc6},
                                 Word{0xc7},
                                 Word{0xcc},
                                 Word{0xd4},
                                 Word{0xd6},
                                 Word{0xd7},
                                 Word{0xdc}};
  constexpr int legal_count = 18501;
  require(std::ranges::count(legal, true) == legal_count,
          "Named ISA controls and operand fields cover all published low-control format members");
  for (Word word = 0; word < legal.size(); ++word) {
    const auto shape = inspect_instruction(word);
    if (legal[word]) {
      require(shape && shape->kind != InstructionKind::undefined,
              "Every documented low-control and MOVEP form retains its ordinary encoding kind");
    } else if (std::ranges::find(uncertain, word) != uncertain.end()) {
      require(!shape, "Unqualified permissive control aliases remain visibly unsupported");
    } else {
      require(shape && shape->kind == InstructionKind::undefined && shape->words == 1 &&
                  shape->repeatable,
              "Unallocated low-control words retain exact one-word III and REP classification");
    }
  }
}

void shapes() {
  for (const auto expected : cases) {
    const auto value = inspect_instruction(expected.opcode);
    require(value.has_value(), "Documented independently encoded form has a known shape");
    if (!value) { throw std::runtime_error("Missing instruction shape"); }
    require(value->kind == expected.kind && value->words == expected.words &&
                value->repeatable == expected.repeatable,
            "Literal family, word count and structural REP eligibility match the ISA");
  }
  for (const Word opcode : {Word{0x0d3000}, Word{0x1000000}, Word{0xffffffff}}) {
    require(!inspect_instruction(opcode),
            "Unknown and over-width inputs never become masked opcodes");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::shapes();
    dsp56300::test::reserved_domains();
    dsp56300::test::low_control_domain();
    std::cout << dsp56300::test::checks << " instruction shape assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
