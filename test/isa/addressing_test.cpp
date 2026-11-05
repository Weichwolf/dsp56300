// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
struct AddressCase {
  Word modifier = 0, cursor = 0, offset = 1, result = 0;
};

constexpr std::array replacements{
    AddressCase{.modifier = 15, .cursor = 0x4000f, .result = 0x40000},
    AddressCase{.modifier = 16, .cursor = 0x4000f, .result = 0x40010},
    AddressCase{.modifier = 16, .cursor = 0x40010, .result = 0x40000},
    AddressCase{.modifier = 0xab000f, .cursor = 0x4000f, .result = 0x40000},
    AddressCase{.modifier = 0xffff, .cursor = word_mask, .result = 0},
    AddressCase{.modifier = 0, .cursor = 1, .offset = sign24, .result = 0x800001},
    AddressCase{.modifier = 0x800f, .cursor = 0x4000f, .offset = 0x1234, .result = 0x40003},
    AddressCase{.modifier = 24575, .cursor = 0x45fff, .result = 0x40000},
    AddressCase{.modifier = 2, .cursor = 0x40002, .offset = 8, .result = 0x4000a},
    AddressCase{.modifier = 15, .cursor = 0x4000f, .result = 0x40000}};

void reinitialize(Core &core, const unsigned index) {
  for (const auto &item : replacements) {
    auto state = initial_state();
    state.sr = status_byte;
    state.n.fill(word_mask);
    state.r[index] = item.cursor;
    state.n[index] = item.offset;
    state.m[index] = item.modifier;
    auto expected = state;
    expected.r[index] = item.result;
    expected.pc = start + 1;
    const auto before = core.progress();
    core.set_registers(state);
    const auto result = core.run(1);
    require(result.dispatches == 1 && result.fault.kind == FaultKind::none,
            "Reinitialized AGU instruction retires");
    require(core.registers() == expected, "Address arithmetic follows host register replacement");
    require(core.progress().instructions == before.instructions + 1 &&
                core.progress().estimated_cycles == before.estimated_cycles + 1,
            "Reinitialized AGU retains exact accounting");
    require(core.interruptible(), "AGU instruction retains interrupt admission");
  }
}

void check(const Backend backend) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Core core(
      {.program = p, .x = x, .y = y}, nullptr, {.backend = backend, .instructions_per_block = 1});
  constexpr Word update = 0x204800;
  for (unsigned index = 0; index < agu_registers; ++index) {
    core.write(Space::program, start, update | (index << 8));
    const auto program = p;
    reinitialize(core, index);
    core.clear_code_cache();
    reinitialize(core, index);
    core.reset();
    reinitialize(core, index);
    require(p == program && x == y && x == std::array<Word, memory_words>{},
            "AGU register replacement never writes guest memory");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::check(backend);
    }
    std::cout << dsp56300::test::checks << " AGU host-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
