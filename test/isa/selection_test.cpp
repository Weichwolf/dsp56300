// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test {
namespace {
using selection_model::Kind;
using Alu = selection_model::Input;
constexpr std::array operations{Alu{.kind = Kind::maximum},
                                Alu{.kind = Kind::magnitude},
                                Alu{.kind = Kind::complement},
                                Alu{.kind = Kind::complement, .accumulator_b = true},
                                Alu{.kind = Kind::test},
                                Alu{.kind = Kind::test, .accumulator_b = true}};
constexpr Word io_begin = 0xffff80, incoming = 0xab800001, payload = 0x800001;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, adc_x_a = 0x200021, load_x0_increment = 0x44d800;

class Observer final : public Io {
public:
  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++calls;
    value = incoming;
    valid = valid && space == Space::x && address == io_begin && pc == start + 2 &&
            core->registers() == expected &&
            core->progress() == Progress{.instructions = 2, .estimated_cycles = 2};
    try {
      bool rejected = false;
      try {
        static_cast<void>(core->run(1));
      } catch (const std::logic_error &) { rejected = true; }
      valid = valid && rejected;
      core->write(Space::program, start + 3, adc_x_a);
    } catch (...) { valid = false; }
    return accept;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  Core *core = nullptr;
  Registers expected;
  bool accept = true, valid = true;
  unsigned calls = 0;
};

void boundary(const Backend backend, const Alu alu, const bool accept) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Observer io;
  Core core({.program = p, .x = x, .y = y}, &io, {.backend = backend});
  io.core = &core;
  io.accept = accept;
  const std::array program{mac_a, mac_b, load_x0_increment | selection_model::code(alu), Word{0}};
  core.load(Space::program, start, program);
  auto initial = register_model::seed(0);
  initial.r[0] = io_begin;
  core.set_registers(initial);
  auto expected = initial;
  for (const auto operation : {mac_a, mac_b}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
  }
  const auto old_alu = expected;
  ++expected.r[0];
  expected.pc = start + 3;
  io.expected = expected;
  const auto result = core.run(4);
  require(io.valid && io.calls == 1,
          "Selection/status helper exposes canonical native state and rejects callback reentry");
  if (accept) {
    register_model::write(expected, 4, payload);
    selection_model::apply(expected, old_alu, alu);
    const auto old = expected;
    sequence_model::alu(expected, old, adc_x_a);
    ++expected.pc;
    require(result.dispatches == 4 && result.fault.kind == FaultKind::none &&
                result.progress == Progress{.instructions = 4, .estimated_cycles = 4} &&
                core.registers() == expected,
            "An accepted move executes the ALU once, then consumes new Carry/input after P "
            "invalidation");
  } else {
    const auto actual = core.registers();
    // A rejected parallel move can expose its own partial data-transfer state;
    // it must not execute the following ALU or count that guest instruction.
    require(result.dispatches == 2 &&
                result.progress == Progress{.instructions = 2, .estimated_cycles = 2} &&
                result.fault == Fault{.kind = FaultKind::io_read,
                                      .space = Space::x,
                                      .address = io_begin,
                                      .pc = start + 2} &&
                actual.a == expected.a && actual.b == expected.b && actual.sr == expected.sr &&
                actual.r == expected.r && actual.n == expected.n && actual.m == expected.m &&
                actual.pc == expected.pc && actual.stack == expected.stack &&
                actual.sp == expected.sp && actual.sc == expected.sc,
            "A rejected parallel transfer performs no MAX/MAXM/NOT/TST effect or guest retirement");
    require(core.run(4).dispatches == 0 && io.calls == 1 && core.registers() == actual,
            "Failed selection/status instructions remain stopped without replaying the transfer or "
            "ALU");
  }
  if (backend == Backend::jit) {
    require(core.jit_statistics().native_alu_instructions >= (accept ? 3U : 2U),
            "The observer test executes real native arithmetic around the selection/status helper");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      for (const auto operation : dsp56300::test::operations) {
        dsp56300::test::boundary(backend, operation, false);
        dsp56300::test::boundary(backend, operation, true);
      }
    }
    std::cout << dsp56300::test::checks << " selection/status callback boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
