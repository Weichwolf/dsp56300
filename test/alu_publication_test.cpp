// SPDX-License-Identifier: GPL-3.0-only
#include "alu_publication_support.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test::alu_publication_test {
namespace {
constexpr unsigned seed_count = 6;
constexpr Word full_instruction = 0x200000;
constexpr Word maxm = 0x15, maximum = 0x1d;
constexpr std::array modes{
    Word{0}, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array ordinary{
    Word{0x01}, Word{0x11}, Word{0x13}, Word{0x26}, Word{0x36}, Word{0x22}, Word{0x32}, Word{0x17},
    Word{0x23}, Word{0x33}, Word{0x27}, Word{0x37}, Word{0x03}, Word{0x05}, Word{0x15}, Word{0x1d},
    Word{0x20}, Word{0x30}, Word{0x40}, Word{0x50}, Word{0x60}, Word{0x70}, Word{0x24}, Word{0x34},
    Word{0x44}, Word{0x54}, Word{0x64}, Word{0x74}, Word{0x21}, Word{0x31}, Word{0x25}, Word{0x35},
    Word{0x45}, Word{0x55}, Word{0x65}, Word{0x75}, Word{0x41}, Word{0x51}, Word{0x61}, Word{0x71},
    Word{0x42}, Word{0x52}, Word{0x62}, Word{0x72}, Word{0x43}, Word{0x53}, Word{0x63}, Word{0x73},
    Word{0x46}, Word{0x56}, Word{0x66}, Word{0x76}};

void unit(const Backend backend, const Registers &before, const Word code) {
  Bus bus;
  Core core(bus, {.backend = backend});
  core.set_registers(before);
  const auto result = expected(before, code);
  const Word opcode = full_instruction | code;
  const auto initial = core.begin_instruction({.address = boot, .opcode = opcode}, phased);
  held(core, before);
  if (!result.supported) {
    require(initial.fault == Fault{.kind = FaultKind::unsupported_mode,
                                   .space = Space::program,
                                   .address = boot,
                                   .value = before.sr,
                                   .pc = boot} &&
                !initial.pending,
            "Unqualified numerical modes fail before any computed unit publication");
    return;
  }
  AluWriteback update{.opcode = opcode};
  if (result.write_data) {
    update.updates.at(update.count++) =
        field(result.destination_b ? RegisterStorage::b : RegisterStorage::a,
              result.data_mask,
              result.destination_b ? before.b : before.a,
              result.destination_b ? result.state.b : result.state.a);
  }
  if (result.status_mask != 0) {
    update.updates.at(update.count++) =
        field(RegisterStorage::sr, result.status_mask, before.sr, result.state.sr);
  }
  publish(core, update, result.state);
  require(bus.calls == 0, "Native ALU result publication never invents a bus transaction");
  retire(core);
  auto after = result.state;
  ++after.pc;
  require(core.registers() == after &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Complete independent 56-bit arithmetic/logic/CCR state survives native retirement");
}

void codes(const Backend backend, const Registers &before) {
  for (const auto code : ordinary) {
    unit(backend, before, code);
    if (code != maxm && code != maximum) { unit(backend, before, code | 8U); }
  }
  constexpr Word product_begin = 0x80;
  constexpr Word product_end = 0x100;
  for (Word code = product_begin; code < product_end; ++code) { unit(backend, before, code); }
}
}

void matrix() {
  for (const auto backend : backends) {
    for (unsigned index = 0; index < seed_count; ++index) {
      for (const auto mode : modes) {
        for (const Word saturation : {Word{0}, sm}) {
          auto before = seed(index);
          before.sr |= mode | saturation;
          codes(backend, before);
        }
      }
    }
  }
}
} // namespace dsp56300::test::alu_publication_test

int main() {
  try {
    using namespace dsp56300::test::alu_publication_test;
    matrix();
    controls();
    restores();
    std::cout << dsp56300::test::checks << " native ALU publication assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
