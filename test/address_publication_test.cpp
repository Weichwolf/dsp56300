// SPDX-License-Identifier: GPL-3.0-only
#include "address_publication_support.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::address_publication_test {
namespace {
struct Case {
  unsigned mode;
  Word initial, offset, modifier, result, address;
};

// Literal arithmetic expectations: linear wrap, four-word modulo boundaries,
// and reverse carry/borrow starting at bit 2. No production AGU helper is used.
constexpr std::array cases{
    Case{.mode = 0,
         .initial = 0x100,
         .offset = 3,
         .modifier = word_mask,
         .result = 0xfd,
         .address = 0x100},
    Case{.mode = 1,
         .initial = 0x100,
         .offset = 3,
         .modifier = word_mask,
         .result = 0x103,
         .address = 0x100},
    Case{.mode = 2,
         .initial = 0x100,
         .offset = 3,
         .modifier = word_mask,
         .result = 0xff,
         .address = 0x100},
    Case{.mode = 3,
         .initial = 0x100,
         .offset = 3,
         .modifier = word_mask,
         .result = 0x101,
         .address = 0x100},
    Case{.mode = 7,
         .initial = 0x100,
         .offset = 3,
         .modifier = word_mask,
         .result = 0xff,
         .address = 0xff},
    Case{.mode = 0, .initial = 0, .offset = 0, .modifier = word_mask, .result = 0, .address = 0},
    Case{.mode = 1,
         .initial = word_mask,
         .offset = 2,
         .modifier = word_mask,
         .result = 1,
         .address = word_mask},
    Case{.mode = 2,
         .initial = 0,
         .offset = 3,
         .modifier = word_mask,
         .result = word_mask,
         .address = 0},
    Case{.mode = 3,
         .initial = word_mask,
         .offset = 3,
         .modifier = word_mask,
         .result = 0,
         .address = word_mask},
    Case{.mode = 7,
         .initial = 0,
         .offset = 3,
         .modifier = word_mask,
         .result = word_mask,
         .address = word_mask},
    Case{
        .mode = 3, .initial = 0x103, .offset = 1, .modifier = 3, .result = 0x100, .address = 0x103},
    Case{
        .mode = 7, .initial = 0x100, .offset = 1, .modifier = 3, .result = 0x103, .address = 0x103},
    Case{
        .mode = 1, .initial = 0x100, .offset = 3, .modifier = 3, .result = 0x103, .address = 0x100},
    Case{
        .mode = 0, .initial = 0x100, .offset = 3, .modifier = 3, .result = 0x101, .address = 0x100},
    Case{.mode = 1, .initial = 6, .offset = 4, .modifier = 0, .result = 1, .address = 6},
    Case{.mode = 0, .initial = 1, .offset = 4, .modifier = 0, .result = 6, .address = 1}};

struct Encoding {
  unsigned index;
  bool load, y;
};

void scalar(const Backend backend, const Case input, const Encoding encoding) {
  Bus bus;
  bus.events = {{.kind = encoding.load ? EffectKind::read : EffectKind::write,
                 .space = encoding.y ? Space::y : Space::x,
                 .address = input.address,
                 .value = encoding.load ? staged_test::datum : 1U}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.r.at(encoding.index) = input.initial;
  state.n.at(encoding.index) = input.offset;
  state.m.at(encoding.index) = input.modifier;
  core.set_registers(state);
  const Word opcode = 0x444000U | (encoding.load ? 0x8000U : 0U) | (encoding.y ? 0x80000U : 0U) |
                      (input.mode << 11) | (encoding.index << 8);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, phased));
  unchanged(core, state);
  writeback(core,
            {.index = static_cast<std::uint8_t>(encoding.index),
             .previous = input.initial,
             .value = input.result},
            input.address);
  state.r.at(encoding.index) = input.result;
  unchanged(core, state);
  require(bus.calls == 0, "Native publication never calls the external bus");
  const auto access = pending(core);
  require(access.effect.kind == bus.events.front().kind && access.effect.address == input.address &&
              access.effect.space == bus.events.front().space,
          "Native EA still addresses the original data operation");
  static_cast<void>(core.advance_instruction(access.ticket));
  unchanged(core, state);
  require(bus.calls == 1 && bus.accepted == 1 && bus.exact,
          "The actual data access occurs exactly once after the admitted writeback");
  retire(core);
  state.pc = boot + 1;
  if (encoding.load) { state.x = staged_test::datum; }
  require(core.registers() == state &&
              core.progress() ==
                  Progress{.instructions = 1, .estimated_cycles = input.mode == 7 ? 2U : 1U},
          "Native retirement publishes the data result and progress without losing early R");
}

void matrix() {
  for (const auto backend : backends) {
    for (const auto input : cases) {
      for (unsigned index = 0; index < agu_registers; ++index) {
        for (const bool load : {false, true}) {
          for (const bool y : {false, true}) {
            scalar(backend, input, {.index = index, .load = load, .y = y});
          }
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test::address_publication_test

int main() {
  try {
    using namespace dsp56300::test::address_publication_test;
    matrix();
    paired_restore();
    controls();
    std::cout << dsp56300::test::checks << " native address publication assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
