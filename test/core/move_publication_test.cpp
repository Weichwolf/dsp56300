// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "move_publication_support.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test::move_publication_test {
namespace {
constexpr Word load_and_add = 0x44e040, eor_and_b0 = 0x20c94b;
constexpr Word logic_source = 0x13579b, partial_data = 0x2468ac;
constexpr std::uint64_t logic_initial = 0x12000010abcdef;
constexpr std::uint64_t logic_moved = 0x120000102468ac, logic_final = 0x1213578b2468ac;

std::uint64_t accumulator_write_mask(const unsigned number) {
  if (number < 10) { return word_mask; }
  if (number < 12) { return high_mask; }
  if (number < 14) { return main_mask; }
  return accumulator_mask;
}

RegisterUpdate
data_destination(const Registers &before, const Registers &after, const unsigned number) {
  if (number <= 7) {
    const auto mask = number % 2 == 0 ? std::uint64_t{word_mask} : main_mask;
    return field(number < 6 ? RegisterStorage::x : RegisterStorage::y,
                 mask,
                 number < 6 ? before.x : before.y,
                 number < 6 ? after.x : after.y);
  }
  return field(number % 2 == 0 ? RegisterStorage::a : RegisterStorage::b,
               accumulator_write_mask(number),
               number % 2 == 0 ? before.a : before.b,
               number % 2 == 0 ? after.a : after.b);
}

RegisterUpdate destination(const Registers &before, const Registers &after, const unsigned number) {
  if (number < 16) { return data_destination(before, after, number); }
  if (number < control_base) {
    const bool pointer = number < register_model::reg_n_base;
    const auto index = number % 8;
    return field(pointer ? RegisterStorage::r : RegisterStorage::n,
                 word_mask,
                 pointer ? before.r[index] : before.n[index],
                 pointer ? after.r[index] : after.n[index],
                 index);
  }
  if (number < register_model::end_modifiers) {
    const auto index = number - control_base;
    return field(RegisterStorage::m, word_mask, before.m[index], after.m[index], index);
  }
  switch (number) {
    case register_model::reg_ep: return field(RegisterStorage::ep, word_mask, before.ep, after.ep);
    case register_model::reg_vba:
      return field(RegisterStorage::vba, word_mask, before.vba, after.vba);
    case register_model::reg_sc:
      return field(RegisterStorage::sc, stack_counter_mask, before.sc, after.sc);
    case register_model::reg_sz: return field(RegisterStorage::sz, word_mask, before.sz, after.sz);
    case register_model::reg_sr: return field(RegisterStorage::sr, word_mask, before.sr, after.sr);
    case register_model::reg_omr:
      return field(RegisterStorage::omr, word_mask, before.omr, after.omr);
    case register_model::reg_sp:
      return field(RegisterStorage::sp, stack_pointer_mask, before.sp, after.sp);
    case ssl: return field(RegisterStorage::stack, word_mask, before.stack[2], after.stack[2], 2);
    case register_model::reg_la: return field(RegisterStorage::la, word_mask, before.la, after.la);
    case register_model::reg_lc: return field(RegisterStorage::lc, word_mask, before.lc, after.lc);
    default: throw std::logic_error("No independent destination expectation");
  }
}

void scalar(const Backend backend, const unsigned number, const Word value) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.y = (state.y & main_mask) | value;
  core.set_registers(state);
  auto after = state;
  register_model::write(after, number, value);
  const Word opcode =
      number < control_base ? 0x20c000U | (number << 8) : 0x04c6a0U | (number - control_base);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, phased));
  held(core, state);
  MoveWriteback expected;
  if (number == ssh) {
    expected = {
        .selector = ssh,
        .count = 3,
        .updates = {{field(RegisterStorage::sp, stack_pointer_mask, 2, 3),
                     field(RegisterStorage::sc, stack_counter_mask, 2, 3),
                     field(RegisterStorage::stack, long_mask, state.stack[3], after.stack[3], 3)}}};
  } else {
    expected = single(number, destination(state, after, number));
  }
  publish(core, expected, after);
  require(bus.calls == 0, "Register MOVE tickets invoke no external memory or peripheral access");
  retire(core);
  ++after.pc;
  require(core.registers() == after && core.progress().instructions == 1,
          "Native scalar retirement retains already published MOVE state");
}
}

void scalars() {
  constexpr std::array values{Word{0}, Word{0x123456}, word_mask};
  for (const auto backend : backends) {
    for (unsigned number = 4; number < register_count; ++number) {
      if (!register_model::valid(number)) { continue; }
      for (const auto value : values) { scalar(backend, number, value); }
      auto unchanged = seed();
      scalar(backend, number, register_model::read(unchanged, number));
    }
  }
}

void parallel() {
  for (const auto backend : backends) {
    Bus bus;
    bus.events = {{.address = data_address, .value = payload}};
    Core core(bus, {.backend = backend});
    auto state = seed();
    state.x = 1;
    state.a = std::uint64_t{2} << word_bits;
    core.set_registers(state);
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = load_and_add}, phased));
    held(core, state);
    const auto access = pending(core);
    require(access.effect.kind == EffectKind::read, "Parallel X load retains its genuine bus read");
    static_cast<void>(core.advance_instruction(access.ticket));
    held(core, state);
    const auto before = state;
    state.x = payload;
    publish(core, single(4, field(RegisterStorage::x, word_mask, before.x, state.x)), state);
    require(pending(core).computed.a == (std::uint64_t{3} << word_bits),
            "Native ALU computation uses the original X0 despite its published load");
    retire(core);
    state.a = std::uint64_t{3} << word_bits;
    state.sr = flag_u;
    ++state.pc;
    require(core.registers() == state && bus.calls == 1 && bus.exact,
            "Only retirement publishes ALU state and flags; accepted data is not read twice");

    core.reset();
    bus.events.clear();
    bus.calls = 0;
    state = seed();
    state.b = logic_initial;
    state.x = logic_source;
    state.y = partial_data;
    state.sr = flag_c;
    core.set_registers(state);
    static_cast<void>(core.begin_instruction({.address = boot, .opcode = eor_and_b0}, phased));
    held(core, state);
    const auto old = state.b;
    state.b = logic_moved;
    publish(core, single(9, field(RegisterStorage::b, word_mask, old, state.b)), state);
    retire(core);
    state.b = logic_final;
    ++state.pc;
    require(core.registers() == state && bus.calls == 0,
            "Later 24-bit logic retains the independently expected early B0 write");
  }
}
} // namespace dsp56300::test::move_publication_test

int main() {
  try {
    using namespace dsp56300::test::move_publication_test;
    scalars();
    parallel();
    composite();
    side_effects();
    restores();
    std::cout << dsp56300::test::checks << " native MOVE publication assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
