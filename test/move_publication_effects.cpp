// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "move_publication_support.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>

namespace dsp56300::test::move_publication_test {
namespace {
constexpr Word long_address = 0x12, high_word = 0x8642ac, low_word = 0x123456;
constexpr Word saturated = 0x7fffff, program_address = 0x180;
constexpr std::uint64_t large_accumulator = 0x01400000000000;
constexpr Word limited_store = 0x481200, raw_store = 0x401200;
constexpr Word program_ssh_store = 0x0770bc, program_ssh_load = 0x07f0bc, ssh_to_x0 = 0x0444bc;

void access(Core &core, const EffectKind kind) {
  const auto action = pending(core);
  require(action.effect.kind == kind, "External operation retains its actual native position");
  static_cast<void>(core.advance_instruction(action.ticket));
}

void publish_accumulator(Core &core, Registers &state, const bool b, const std::uint64_t result) {
  auto &value = b ? state.b : state.a;
  const auto before = value;
  value = result;
  publish(
      core,
      single(b ? 15U : 14U,
             field(b ? RegisterStorage::b : RegisterStorage::a, accumulator_mask, before, result)),
      state);
}

void composite_load(const Backend backend, const unsigned selector) {
  Bus bus;
  bus.events = {{.space = Space::x, .address = long_address, .value = high_word},
                {.space = Space::y, .address = long_address, .value = low_word}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  const auto before = state;
  core.set_registers(state);
  const Word opcode = 0x409200U | ((selector % 4) << 16) | ((selector / 4) << 19);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, phased));
  for (unsigned index = 0; index < 2; ++index) {
    held(core, state);
    access(core, EffectKind::read);
  }
  held(core, state);
  if (selector < 6) {
    constexpr std::array results{std::uint64_t{0x7e8642ac123456},
                                 std::uint64_t{0x818642ac123456},
                                 std::uint64_t{0x8642ac123456},
                                 std::uint64_t{0x8642ac123456},
                                 std::uint64_t{0xff8642ac123456},
                                 std::uint64_t{0xff8642ac123456}};
    const auto result = results.at(selector);
    RegisterStorage storage = RegisterStorage::a;
    std::uint64_t old = before.a;
    if (selector == 1 || selector == 5) {
      state.b = result;
      storage = RegisterStorage::b;
      old = before.b;
    } else if (selector == 2) {
      state.x = result;
      storage = RegisterStorage::x;
      old = before.x;
    } else if (selector == 3) {
      state.y = result;
      storage = RegisterStorage::y;
      old = before.y;
    } else {
      state.a = result;
    }
    publish(core,
            single(selector,
                   field(storage, selector < 4 ? long_mask : accumulator_mask, old, result),
                   MoveProducer::long_load),
            state);
  } else {
    const bool reverse = selector == 7;
    constexpr std::uint64_t signed_high = 0xff8642ac000000;
    constexpr std::uint64_t signed_low = 0x00123456000000;
    publish_accumulator(core, state, reverse, signed_high);
    publish_accumulator(core, state, !reverse, signed_low);
  }
  require(bus.calls == 2 && bus.accepted == 2 && bus.exact,
          "Composite native loads consume exactly the actual X and Y response");
  retire(core);
  ++state.pc;
  require(core.registers() == state,
          "Raw A10/B10, signed A/B, X/Y and AB/BA retain distinct packing");
}

struct AccumulatorInput {
  std::uint64_t value;
  Word mode, initial_flags;
  bool b;
};

void accumulator_read(const Backend backend, const AccumulatorInput input) {
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = seed();
  (input.b ? state.b : state.a) = input.value;
  state.sr = input.mode | input.initial_flags;
  core.set_registers(state);
  auto math = state;
  math.sr = input.mode;
  const auto result = register_model::bus_word(math, input.value);
  const auto written = math.sr & (flag_s | flag_l);
  const unsigned source = input.b ? 15U : 14U;
  const Word opcode = 0x200400U | (source << 13);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, phased));
  held(core, state);
  if (written != 0) {
    const auto before = state.sr;
    state.sr |= written;
    publish(core,
            single(source,
                   field(RegisterStorage::sr, written, before, state.sr),
                   MoveProducer::accumulator_read),
            state);
  }
  const auto before = state.x;
  state.x = (before & main_mask) | result;
  publish(core, single(4, field(RegisterStorage::x, word_mask, before, state.x)), state);
  retire(core);
  ++state.pc;
  require(core.registers() == state && bus.calls == 0,
          "Independent quotient/remainder scaling and S/L expectations match native A/B reads");
}

void accumulator_matrix(const Backend backend) {
  constexpr std::array values{std::uint64_t{0},
                              std::uint64_t{0x00000112345678},
                              std::uint64_t{0x00400000000000},
                              std::uint64_t{0x00800000000000},
                              std::uint64_t{0x01400000000000},
                              std::uint64_t{0xff800000000000},
                              std::uint64_t{0xfec00000000000}};
  for (const auto value : values) {
    for (const Word scale : {Word{0}, mode_down, mode_up}) {
      for (const Word rounding : {Word{0}, mode_rm}) {
        for (const Word sticky : {Word{0}, flag_s | flag_l}) {
          for (const bool b : {false, true}) {
            accumulator_read(
                backend,
                {.value = value, .mode = scale | rounding, .initial_flags = sticky, .b = b});
          }
        }
      }
    }
  }
}

void long_store_fault(const Backend backend, const bool limited) {
  Bus bus;
  bus.events = {{.kind = EffectKind::write,
                 .space = Space::x,
                 .address = long_address,
                 .value = limited ? saturated : Word{0x400000}},
                {.kind = EffectKind::write,
                 .space = Space::y,
                 .address = long_address,
                 .value = limited ? word_mask : Word{0},
                 .accept = false}};
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.a = large_accumulator;
  core.set_registers(state);
  static_cast<void>(core.begin_instruction(
      {.address = boot, .opcode = limited ? limited_store : raw_store}, phased));
  held(core, state);
  if (limited) {
    state.sr = flag_s | flag_l;
    publish(core,
            single(14,
                   field(RegisterStorage::sr, flag_s | flag_l, 0, state.sr),
                   MoveProducer::accumulator_read),
            state);
  }
  access(core, EffectKind::write);
  held(core, state);
  const auto refused = pending(core);
  const auto result = core.advance_instruction(refused.ticket);
  require(result.fault == Fault{.kind = FaultKind::memory_write,
                                .space = Space::y,
                                .address = long_address,
                                .value = limited ? word_mask : Word{0},
                                .pc = boot} &&
              !result.retired && !result.pending,
          "A rejected second L: store preserves exact address, value and original PC");
  held(core, state);
  require(
      bus.calls == 2 && bus.accepted == 1 && bus.exact &&
          bus.x_written == (limited ? saturated : Word{0x400000}),
      "An accepted native prefix survives later refusal without a repeated store or fake success");
}

MoveWriteback stack_pop() {
  return {.producer = MoveProducer::stack_read,
          .selector = ssh,
          .count = 2,
          .updates = {{field(RegisterStorage::sp, stack_pointer_mask, 2, 1),
                       field(RegisterStorage::sc, stack_counter_mask, 2, 1)}}};
}

void stack_read(const Backend backend, const bool store_fault) {
  Bus bus;
  if (store_fault) {
    bus.events = {{.kind = EffectKind::write,
                   .space = Space::program,
                   .address = program_address,
                   .value = register_model::frame_high + 2,
                   .accept = false}};
  }
  Core core(bus, {.backend = backend});
  auto state = seed();
  core.set_registers(state);
  InstructionWords words{.address = boot, .opcode = store_fault ? program_ssh_store : ssh_to_x0};
  if (store_fault) { words.extension = program_address; }
  static_cast<void>(core.begin_instruction(words, phased));
  held(core, state);
  state.sp = 1;
  state.sc = 1;
  publish(core, stack_pop(), state);
  if (store_fault) {
    access(core, EffectKind::write);
    require(core.fault() == Fault{.kind = FaultKind::memory_write,
                                  .space = Space::program,
                                  .address = program_address,
                                  .value = register_model::frame_high + 2,
                                  .pc = boot} &&
                !core.pending_instruction(),
            "Rejected SSH store preserves its genuine earlier stack pop and exact failure");
    held(core, state);
    require(bus.calls == 1 && bus.accepted == 0 && bus.exact,
            "SSH replay never pops or writes twice");
  } else {
    const auto before = state.x;
    state.x = (before & main_mask) | (register_model::frame_high + 2);
    publish(core, single(4, field(RegisterStorage::x, word_mask, before, state.x)), state);
    retire(core);
    ++state.pc;
    require(core.registers() == state,
            "SSH pop and explicit destination publish as separate native producers");
  }
}

void stack_fault(const Backend backend, const bool overflow) {
  Bus bus;
  if (overflow) { bus.events = {{.space = Space::program, .address = program_address}}; }
  Core core(bus, {.backend = backend});
  auto state = seed();
  state.sc = overflow ? Word{15} : Word{0};
  core.set_registers(state);
  static_cast<void>(
      core.begin_instruction({.address = boot,
                              .opcode = overflow ? program_ssh_load : program_ssh_store,
                              .extension = program_address},
                             phased));
  if (overflow) { access(core, EffectKind::read); }
  require(core.fault() == Fault{.kind = FaultKind::stack,
                                .space = Space::program,
                                .address = boot,
                                .value = state.sp,
                                .pc = boot} &&
              !core.pending_instruction(),
          "Stack overflow/underflow cannot fabricate a MOVE publication");
  held(core, state);
  require(bus.calls == (overflow ? 1U : 0U) && bus.exact,
          "Stack fault preserves accepted reads and suppresses unperformed stores");
}
}

void composite() {
  for (const auto backend : backends) {
    for (unsigned selector = 0; selector < 8; ++selector) { composite_load(backend, selector); }
    for (const bool limited : {false, true}) { long_store_fault(backend, limited); }
  }
}

void side_effects() {
  for (const auto backend : backends) {
    accumulator_matrix(backend);
    stack_read(backend, false);
    stack_read(backend, true);
    stack_fault(backend, false);
    stack_fault(backend, true);
  }
}
} // namespace dsp56300::test::move_publication_test
