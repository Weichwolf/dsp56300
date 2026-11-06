// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "instruction_decoder.hpp"
#include "support.hpp"

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

void shapes() {
  for (const auto expected : cases) {
    const auto value = inspect_instruction(expected.opcode);
    require(value.has_value(), "Documented independently encoded form has a known shape");
    if (!value) { throw std::runtime_error("Missing instruction shape"); }
    require(value->kind == expected.kind && value->words == expected.words &&
                value->repeatable == expected.repeatable,
            "Literal family, word count and structural REP eligibility match the ISA");
  }
  for (const Word opcode : {Word{7}, Word{0x1000000}, Word{0xffffffff}}) {
    require(!inspect_instruction(opcode),
            "Unknown and over-width inputs never become masked opcodes");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::shapes();
    std::cout << dsp56300::test::checks << " instruction shape assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
