// SPDX-License-Identifier: GPL-3.0-only
#include "control_byte_model.hpp"
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "unit_publication_support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test::unit_publication_test {
namespace {
void control(const Backend backend, const byte_model::Input input, const Word old) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = byte_model::seed(input.field, old);
  state.pc = boot;
  core.set_registers(state);
  const auto before = state;
  const auto opcode = byte_model::opcode(input);
  byte_model::apply(state, input);
  const auto shift = byte_model::first_bit(input.field);
  const auto storage = input.field == byte_model::Field::mr || input.field == byte_model::Field::ccr
                           ? RegisterStorage::sr
                           : RegisterStorage::omr;
  const UnitWriteback result{
      .unit = RegisterUnit::control_byte,
      .opcode = opcode,
      .count = 1,
      .updates = {{field(storage,
                         status_byte << shift,
                         storage == RegisterStorage::sr ? before.sr : before.omr,
                         storage == RegisterStorage::sr ? state.sr : state.omr)}}};
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, policy));
  held(core, before);
  require(pending(core).effect.omr == before.omr,
          "Native unit effect retains the original instruction context during an OMR change");
  publish(core, result, state);
  retire(core);
  ++state.pc;
  require(core.registers() == state && bus.calls == 0 &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 3},
          "Independent per-bit control-byte truth table preserves every other CPU bit");
}

struct Calculation {
  Word pc = boot, opcode = 0x04c000, extra = 0;
  Word r = 0, n = 1, m = 0xffff, result = 0;
  bool extension = false, displacement = false;
};

void calculation(const Backend backend, const Calculation input, const unsigned number) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.pc = input.pc;
  state.r[0] = input.r;
  state.n[0] = input.n;
  state.m[0] = input.m;
  const auto before = state;
  core.set_registers(state);
  register_model::write(state, number, input.result);
  const auto opcode = input.opcode | (input.displacement ? number - 16 : number);
  const UnitWriteback result{.unit = RegisterUnit::address_calculation,
                             .opcode = opcode,
                             .count = 1,
                             .updates = {{destination(before, state, number)}}};
  InstructionWords words{.address = input.pc, .opcode = opcode};
  if (input.extension) { words.extension = input.extra; }
  static_cast<void>(core.begin_instruction(words, policy));
  held(core, before);
  publish(core, result, state);
  retire(core);
  state.pc = (state.pc + (input.extension ? 2U : 1U)) & word_mask;
  require(
      core.registers() == state && core.progress().instructions == 1 && bus.calls == 0,
      "Literal LRA/LUA wrap/linear/modulo/reverse results reach each independent register view");
}
}

void controls() {
  constexpr std::array fields{byte_model::Field::mr,
                              byte_model::Field::ccr,
                              byte_model::Field::com,
                              byte_model::Field::eom};
  constexpr std::array<Word, 8> immediates{0, 1, 0x55, 0xaa, 0x7f, 0x80, 0xfe, status_byte};
  for (const auto backend : backends) {
    for (const auto selected : fields) {
      for (const bool inclusive_or : {false, true}) {
        for (const auto immediate : immediates) {
          for (const Word old : {0U, 0x5aU, 0xa5U, status_byte}) {
            control(backend,
                    {.field = selected, .immediate = immediate, .inclusive_or = inclusive_or},
                    old);
          }
        }
      }
    }
  }
}

void calculations() {
  constexpr std::array cases{
      Calculation{.r = 16, .result = boot + 16},
      Calculation{.pc = word_mask, .r = 2, .result = 1},
      Calculation{.opcode = 0x044040, .extra = word_mask, .result = boot - 1, .extension = true},
      Calculation{.opcode = 0x044800, .r = word_mask, .result = 0},
      Calculation{.opcode = 0x044800, .r = 0x3ff, .m = 15, .result = 0x3f0},
      Calculation{.opcode = 0x044800, .r = 0x800000, .n = 0x800000, .m = 0, .result = 0x400000},
      Calculation{.opcode = 0x044800, .r = 0x3ff, .n = 0x123456, .m = 0x800f, .result = 0x3f5},
      Calculation{.opcode = 0x0400f0, .r = 0xfffff8, .result = 7, .displacement = true}};
  for (const auto backend : backends) {
    for (const auto input : cases) {
      for (unsigned number = input.displacement ? 16U : 4U; number < 32; ++number) {
        calculation(backend, input, number);
      }
    }
  }
}
} // namespace dsp56300::test::unit_publication_test

int main() {
  try {
    using namespace dsp56300::test::unit_publication_test;
    bits();
    controls();
    calculations();
    boundaries();
    ram_boundaries();
    std::cout << dsp56300::test::checks << " native scalar-unit publication assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
