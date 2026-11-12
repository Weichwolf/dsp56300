// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/runtime.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using isa::Opcode;
constexpr Word entry = 16, base = 0x4000, size = 64, length = 9;
constexpr Word lf = 1U << 15;
constexpr Word linear = 0xffff, initial_target = 0x1234;
constexpr Word target_slot = 48, clobber_slot = 52, store_slot = 24, outer_tail = 40;
constexpr Word narrow_ring = 0x8003, wide_ring = 0x800f;
constexpr unsigned high_register_mask = 0x18, high_register_shift = 17, y_space_bit = 19;
constexpr unsigned r1 = 17, r2 = 18, n1 = 25, long_sign_bit = 47;
constexpr unsigned x_selector_shift = 18;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
using Data = std::array<std::array<Word, size>, 2>;
using Knowledge = std::array<std::array<std::uint8_t, size>, 2>;

Word named(const Opcode opcode) {
  return static_cast<Word>(opcode);
}

enum class Tail : std::uint8_t {
  conditional_register,
  jump_register,
  conditional_short,
  jump_short,
  conditional_absolute,
  jump_absolute
};

struct Input {
  Word modifier = linear, offset = 1, target = initial_target, flags = 0;
  unsigned selector = 2, condition = 5, shift = 7;
  bool left = false, alias = false;
  bool source_b = false, destination_b = true, outer_loop = true, test_load = false;
  Tail tail = Tail::conditional_register;
  bool at_tail = false;
  Word loop_count = 7;
  bool test_flags = false;
};

unsigned transfer_destination(const Input input) {
  return input.test_load ? 14U + static_cast<unsigned>(input.source_b) : 5U;
}

bool absolute(const Input input) {
  return input.tail == Tail::conditional_absolute || input.tail == Tail::jump_absolute;
}

bool unconditional(const Input input) {
  return input.tail == Tail::jump_register || input.tail == Tail::jump_short ||
         input.tail == Tail::jump_absolute;
}

Word tail_cost(const Input input) {
  return input.tail == Tail::jump_register || input.tail == Tail::jump_short ? 3U : 4U;
}

Word branch(const Input input) {
  constexpr Word target_mask = 0xfff;
  switch (input.tail) {
    case Tail::conditional_register:
      return named(Opcode::jcc_effective) | (4U << 11) | (1U << 8) | input.condition;
    case Tail::jump_register: return named(Opcode::jmp_effective) | (4U << 11) | (1U << 8);
    case Tail::conditional_short:
      return named(Opcode::jcc_absolute_short) | (input.condition << 12) |
             (input.target & target_mask);
    case Tail::jump_short: return named(Opcode::jmp_absolute_short) | (input.target & target_mask);
    case Tail::conditional_absolute:
      return named(Opcode::jcc_effective) | (6U << 11) | input.condition;
    case Tail::jump_absolute: return named(Opcode::jmp_effective) | (6U << 11);
    default: throw std::logic_error("Unknown independently authored branch form");
  }
}

Word scalar_load(const unsigned reg, const unsigned cursor, const bool y = false) {
  constexpr Word register_bits = (7U << 16) | (high_register_mask << high_register_shift);
  return (named(Opcode::move_x_effective) & ~register_bits) | ((reg & 7U) << 16) |
         ((reg & high_register_mask) << high_register_shift) | (1U << 15) | (4U << 11) |
         (cursor << 8) | (y ? 1U << y_space_bit : 0U);
}

Word long_move(const Input input, const bool load) {
  const unsigned cursor = load ? 3U : 4U;
  return named(Opcode::move_long) | ((input.selector & 3U) << 16) |
         ((input.selector & 4U) << high_register_shift) | (1U << 14) | (3U << 11) | (cursor << 8) |
         (load ? 1U << 15 : 0U);
}

std::array<Word, length + 1> program(const Input input) {
  const auto pair = named(Opcode::move_xy) | (1U << 15) | (1U << 22) | (2U << 8) | (2U << 13) |
                    (1U << 11) | (1U << 20);
  const auto product = input.test_flags ? named(Opcode::tst_a) & status_byte
                                        : (named(Opcode::mpy_register_a) & status_byte) | (5U << 4);
  const auto mac = input.test_flags ? named(Opcode::tst_b) & status_byte
                                    : (named(Opcode::mac_register_a) & status_byte) | (6U << 4);
  const auto alu =
      input.test_load ? named(input.source_b ? Opcode::tst_b : Opcode::tst_a) & status_byte : mac;
  const auto moved = (named(Opcode::move_register_x0_x1) & ~((31U << 13) | (31U << 8))) |
                     (r1 << 13) | (transfer_destination(input) << 8) | alu;
  return {pair | product | (input.test_flags ? 2U << x_selector_shift : 0U),
          named(input.left ? Opcode::asl_immediate : Opcode::asr_immediate) | (input.shift << 1) |
              (input.source_b ? 1U << 7 : 0U) | (input.destination_b ? 1U : 0U),
          long_move(input, true),
          scalar_load(r1, 5),
          long_move(input, false),
          scalar_load(r2, 7),
          scalar_load(n1, 7, true),
          input.test_load ? scalar_load(transfer_destination(input), 7) | alu : moved,
          branch(input),
          absolute(input) ? input.target : named(Opcode::rts)};
}

struct Fixture final : Io {
  explicit Fixture(const Input input) : input_(input) {
    const auto body = program(input);
    std::ranges::copy(body, p.begin() + entry);
    reset();
    for (unsigned bank = 0; bank < 2; ++bank) {
      windows[bank] = {.address = base,
                       .words = input.alias ? data[0] : data[bank],
                       .known = input.alias ? known[0] : known[bank],
                       .writable = true};
    }
  }

  void reset() {
    constexpr std::array values{0U, 1U, sign24, sign24 - 1, word_mask, 0x345678U, 0xabcdefU};
    for (unsigned bank = 0; bank < 2; ++bank) {
      known[bank].fill(1);
      for (unsigned i = 0; i < size; ++i) { data[bank][i] = values[(i + bank) % values.size()]; }
    }
    data[0][target_slot] = input_.target;
    data[0][clobber_slot] = base + 2;
    data[1][clobber_slot] = word_mask;
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word at) const noexcept override {
    return at < p.size() ? std::optional{p[at]} : std::nullopt;
  }

  bool read(const Space space, const Word at, Word /*pc*/, Word &value) noexcept override {
    if (space != Space::program || at >= p.size()) { return false; }
    value = p[at];
    return true;
  }

  bool write(Space /*space*/, Word /*at*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*at*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    if (space == Space::program) { return {}; }
    return std::span(&windows[space == Space::x ? 0U : 1U], 1);
  }

  [[nodiscard]] detail::JitRuntime runtime() {
    return {.memory = {}, .windows = {std::span(windows.data(), 1), std::span(&windows[1], 1)}};
  }

  std::array<Word, 64> p{};
  Data data{};
  Knowledge known{};
  std::array<MemoryWindow, 2> windows{};
  Input input_;
};

Registers initial(const Input input, const Word mode) {
  auto state = register_model::seed(mode);
  state.pc = entry;
  state.sr = mode | (input.outer_loop ? lf : 0U) | input.flags;
  state.la = input.at_tail ? entry + length - 1 + (absolute(input) ? 1U : 0U) : entry + outer_tail;
  state.lc = input.loop_count;
  state.stack[state.sp % stack_entries] = std::uint64_t{entry} << word_bits;
  state.m.fill(input.modifier);
  // Plain indirect accesses do not consult M; these deliberately forbid an AGU proof.
  constexpr Word reserved_modifier = 0xc003;
  state.m[5] = 0;
  state.m[7] = reserved_modifier;
  state.n.fill(input.offset);
  state.r.fill(base + 16);
  state.r[3] = base + 8;
  state.r[4] = base + store_slot;
  state.r[5] = base + target_slot;
  state.r[6] = base + 32;
  state.r[7] = base + clobber_slot;
  return state;
}

Word effective(Registers &state, const unsigned index, const unsigned mode) {
  const auto prior = state.r[index];
  const auto next = address_model::updated(
      {.modifier = state.m[index], .cursor = prior, .offset = state.n[index], .mode = mode});
  if (mode != 4 && mode != 5) { state.r[index] = next; }
  return mode == 5 || mode == 7 ? next : prior;
}

void load_long(Registers &state, const Input input, const std::uint64_t value) {
  if (input.selector < 2) {
    auto &out = input.selector == 0 ? state.a : state.b;
    out = (out & ~long_mask) | value;
  } else if (input.selector < 4) {
    (input.selector == 2 ? state.x : state.y) = value;
  } else if (input.selector < 6) {
    constexpr auto sign = std::uint64_t{1} << long_sign_bit;
    (input.selector == 4 ? state.a : state.b) =
        value | ((value & sign) != 0 ? accumulator_mask & ~long_mask : 0);
  } else {
    register_model::write(
        state, input.selector == 6 ? 14 : 15, static_cast<Word>(value >> word_bits));
    register_model::write(
        state, input.selector == 6 ? 15 : 14, static_cast<Word>(value) & word_mask);
  }
}

std::uint64_t store_long(Registers &state, const Input input) {
  if (input.selector >= 6) {
    const auto high = register_model::read(state, input.selector == 6 ? 14 : 15);
    return (std::uint64_t{high} << word_bits) |
           register_model::read(state, input.selector == 6 ? 15 : 14);
  }
  if (input.selector >= 2 && input.selector < 4) { return input.selector == 2 ? state.x : state.y; }
  const auto value = (input.selector & 1U) == 0 ? state.a : state.b;
  if (input.selector < 4) { return value & long_mask; }
  static_cast<void>(register_model::bus_word(state, value));
  const auto divisor = (state.sr & mode_down) != 0 ? 2 : 1;
  auto scaled = signed56(value) / divisor;
  if (signed56(value) < 0 && signed56(value) % divisor != 0) { --scaled; }
  if ((state.sr & mode_up) != 0) { scaled *= 2; }
  constexpr std::int64_t minimum = -(std::int64_t{1} << long_sign_bit);
  constexpr std::int64_t maximum = (std::int64_t{1} << long_sign_bit) - 1;
  return static_cast<std::uint64_t>(std::clamp(scaled, minimum, maximum)) & long_mask;
}

bool selected(const Registers &state, const unsigned condition) {
  const auto flags = state.sr;
  const bool c = (flags & flag_c) != 0;
  const bool z = (flags & flag_z) != 0;
  const bool n = (flags & flag_n) != 0;
  const bool v = (flags & flag_v) != 0;
  const bool u = (flags & flag_u) != 0;
  const bool e = (flags & flag_e) != 0;
  const bool l = (flags & flag_l) != 0;
  const std::array truth{!c,
                         n == v,
                         !z,
                         !n,
                         !z && (u || e),
                         !e,
                         !l,
                         !z && n == v,
                         c,
                         n != v,
                         z,
                         n,
                         z || (!u && !e),
                         e,
                         l,
                         z || n != v};
  return truth.at(condition);
}

void finish_loop(Registers &state, const Input input) {
  if (!input.at_tail || !input.outer_loop || state.pc != state.la + 1) { return; }
  constexpr Word forever = 1U << 16;
  if (state.lc == 1 && (state.sr & forever) == 0) {
    loop_frames::complete(state);
  } else {
    state.lc = (state.lc - 1) & word_mask;
    state.pc = static_cast<Word>(state.stack[state.sp % stack_entries] >> word_bits);
  }
}

void model_step(Registers &state, Data &data, const Input input) {
  const auto old = state;
  const auto step = state.pc - entry;
  auto &x = data[0];
  auto &y = data[input.alias ? 0U : 1U];
  ++state.pc;
  switch (step) {
    case 0: {
      const auto xa = effective(state, 2, 1) - base;
      const auto ya = effective(state, 6, 1) - base;
      register_model::write(state, input.test_flags ? 14U : 4U, x.at(xa));
      register_model::write(state, 6, y.at(ya));
      break;
    }
    case 1:
      shift_model::apply(state,
                         old,
                         {.count = input.shift,
                          .left = input.left,
                          .source_b = input.source_b,
                          .destination_b = input.destination_b,
                          .saturate = false});
      return;
    case 2: {
      const auto at = effective(state, 3, 3) - base;
      load_long(state, input, (std::uint64_t{x.at(at)} << word_bits) | y.at(at));
      break;
    }
    case 3: register_model::write(state, r1, x.at(state.r[5] - base)); break;
    case 4: {
      const auto at = effective(state, 4, 3) - base;
      const auto value = store_long(state, input);
      x.at(at) = static_cast<Word>(value >> word_bits) & word_mask;
      y.at(at) = static_cast<Word>(value) & word_mask;
      break;
    }
    case 5: register_model::write(state, r2, x.at(state.r[7] - base)); break;
    case 6: register_model::write(state, n1, y.at(state.r[7] - base)); break;
    case 7:
      register_model::write(
          state, transfer_destination(input), input.test_load ? x.at(state.r[7] - base) : old.r[1]);
      break;
    case 8: {
      if (absolute(input)) { ++state.pc; }
      if (unconditional(input) || selected(state, input.condition)) {
        constexpr Word target_mask = 0xfff;
        state.pc = input.target;
        if (input.tail == Tail::conditional_short || input.tail == Tail::jump_short) {
          state.pc &= target_mask;
        } else if (input.tail == Tail::conditional_register || input.tail == Tail::jump_register) {
          state.pc = state.r[1];
        }
      }
      finish_loop(state, input);
      return;
    }
    default: throw std::logic_error("Model left its independently authored region");
  }
  sequence_model::alu(state, old, program(input)[step]);
}

unsigned fallback_calls = 0;

std::uint32_t forbidden(detail::Cpu * /*cpu*/, detail::NativeContext * /*owner*/) noexcept {
  ++fallback_calls;
  return 0;
}

void direct(const Input input, const Word mode) {
  Fixture fixture(input);
  const auto seed = initial(input, mode);
  const auto runtime = fixture.runtime();
  const auto operations = detail::parse_block(fixture.p, seed, runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  const auto image = detail::compile_region(
      compiler, operations, runtime, std::bit_cast<const void *>(&forbidden));
  if (!image) { throw std::runtime_error("A complete compound RAM/control region must optimize"); }
  for (Word flags = 0; flags <= status_byte; ++flags) {
    fixture.reset();
    auto wanted = seed;
    wanted.sr |= flags;
    auto expected = fixture.data;
    auto shifted = wanted;
    for (unsigned i = 0; i < length; ++i) { model_step(wanted, expected, input); }
    shifted.a <<= detail::padding_bits;
    shifted.b <<= detail::padding_bits;
    detail::Cpu cpu{.reg = shifted};
    const Word cost = length - 1 + tail_cost(input);
    detail::NativeContext owner{.runtime = {}, .budget = length, .cycle_deadline = cost};
    const auto calls = fallback_calls;
    const auto retired = detail::enter_block(image->entry, cpu, owner);
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(retired == length && fallback_calls == calls && cpu.reg == wanted &&
                fixture.data == expected && cpu.instructions == length && cpu.cycles == cost,
            "Compound LLVM executes exact memory/old operands/long updates/flags/target without "
            "helpers");
  }
}

struct Expected {
  Registers reg;
  Data data;
  Word retired = 0, cycles = 0;
};

struct Grant {
  Word budget, deadline;
};

Expected
expected(const Fixture &fixture, const Registers &seed, const Input input, const Grant grant) {
  Expected result{.reg = seed, .data = fixture.data};
  while (result.retired < grant.budget && result.cycles < grant.deadline) {
    model_step(result.reg, result.data, input);
    result.cycles += result.retired == length - 1 ? tail_cost(input) : 1U;
    ++result.retired;
  }
  return result;
}

void prefixes(const Input input, const Word mode) {
  Fixture fixture(input);
  Core core(Memory::external(), &fixture);
  const auto seed = initial(input, mode);
  core.set_registers(seed);
  const std::array entries{entry};
  const auto report = core.prepare(entries);
  const auto again = core.prepare(entries);
  require(report.failure == Fault{} && report.prepared_entries == 1 && report.native_bytes > 0 &&
              again.prepared_entries == 1 && again.native_bytes == 0 && core.registers() == seed &&
              core.progress() == Progress{},
          "Preparation compiles the full region once and has no guest effects");
  const Word cost = length - 1 + tail_cost(input);
  for (const bool timed : {false, true}) {
    for (Word boundary = 0; boundary <= (timed ? cost : length); ++boundary) {
      const Word budget = timed ? length : boundary;
      const Word deadline = timed ? boundary : cost;
      fixture.reset();
      const auto wanted = expected(fixture, seed, input, {.budget = budget, .deadline = deadline});
      core.set_registers(seed);
      const auto before = core.progress();
      const auto result =
          timed ? core.run_until(before.estimated_cycles + deadline) : core.run(budget);
      require(result.fault == Fault{} && result.retired == wanted.retired &&
                  core.registers() == wanted.reg && fixture.data == wanted.data &&
                  result.progress.instructions - before.instructions == wanted.retired &&
                  result.progress.estimated_cycles - before.estimated_cycles == wanted.cycles,
              "Public prepared execution preserves every instruction and cycle prefix");
    }
  }
  core.set_registers(seed);
  const auto final = core.prepare(entries);
  require(final.prepared_entries == 1 && final.native_bytes == 0,
          "Warm execution retains the original prepared code");
}

void resumed_do_body() {
  const Input input{.at_tail = true};
  Fixture fixture(input);
  fixture.p[entry + length - 1] = named(Opcode::nop);
  Core core(Memory::external(), &fixture);
  const auto seed = initial(input, 0);
  core.set_registers(seed);
  const std::array entries{entry};
  require(core.prepare(entries).prepared_entries == 1, "Prepare a complete one-pass DO body");
  require(core.run(1).retired == 1, "Pause inside the complete prepared body");
  const auto paused = core.state();
  const std::array suffix{entry + 1};
  const auto report = core.prepare(suffix);
  require(report.failure == Fault{} && report.prepared_entries == 0 && report.native_bytes == 0 &&
              core.registers() == paused.registers && core.progress() == paused.progress,
          "A resume suffix does not create another LLVM DO body or execute guest work");
  auto wanted = seed;
  auto data = fixture.data;
  for (unsigned i = 0; i < length - 1; ++i) { model_step(wanted, data, input); }
  ++wanted.pc;
  finish_loop(wanted, input);
  const auto result = core.run(length - 1);
  require(result.fault == Fault{} && result.retired == length - 1 && core.registers() == wanted &&
              fixture.data == data && result.progress.instructions == length &&
              result.progress.estimated_cycles == length,
          "A resumed ordinary suffix finishes with the exact registers, RAM and DO effects");
}

void ordinary_copy_region() {
  Fixture fixture({});
  fixture.p[entry] = scalar_load(4, 5);
  fixture.p[entry + 1] = named(Opcode::jmp_effective) | (4U << 11) | (1U << 8);
  Core core(Memory::external(), &fixture);
  const auto seed = initial({}, 0);
  core.set_registers(seed);
  const std::array entries{entry};
  const auto report = core.prepare(entries);
  require(report.failure == Fault{} && report.prepared_entries == 0 && report.native_bytes == 0,
          "A scalar copy/control region retains its existing native lowering");
  auto wanted = seed;
  register_model::write(wanted, 4, fixture.data[0][target_slot]);
  wanted.pc = seed.r[1];
  const auto result = core.run(2);
  require(result.fault == Fault{} && result.retired == 2 && core.registers() == wanted,
          "Declining LLVM for a scalar copy preserves its native instrument work");
}

void admission_failures() {
  Fixture fixture({});
  const auto seed = initial({}, 0);
  const auto runtime = fixture.runtime();
  const auto operations = detail::parse_block(fixture.p, seed, runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  const auto image = detail::compile_region(
      compiler, operations, runtime, std::bit_cast<const void *>(&forbidden));
  if (!image) { throw std::runtime_error("Compile region before mutating its guarded inputs"); }
  constexpr Word reserved = 0xc003;
  constexpr Word invalid_ring = 0x8002;
  const std::array modifiers{0U, 3U, reserved, invalid_ring};
  for (unsigned scenario = 0; scenario < modifiers.size() + 4; ++scenario) {
    fixture.reset();
    auto state = seed;
    if (scenario < modifiers.size()) {
      state.m[2] = modifiers[scenario];
    } else if (scenario == modifiers.size()) {
      fixture.known[0][target_slot] = 0;
    } else if (scenario == modifiers.size() + 1) {
      fixture.known[1][store_slot] = 2;
    } else if (scenario == modifiers.size() + 2) {
      state.r[2] = base + size;
    } else {
      state.r[4] = base + size;
    }
    const auto prior = state;
    const auto data = fixture.data;
    state.a <<= detail::padding_bits;
    state.b <<= detail::padding_bits;
    detail::Cpu cpu{.reg = state};
    detail::NativeContext owner{.runtime = {}, .budget = length, .cycle_deadline = length + 3};
    const auto calls = fallback_calls;
    require(detail::enter_block(image->entry, cpu, owner) == 0 && fallback_calls == calls + 1,
            "An unproved complete region uses the exact ordinary entry");
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(cpu.reg == prior && fixture.data == data && cpu.instructions == 0 && cpu.cycles == 0,
            "Admission rejection has no register, memory or progress effects");
  }
  // A computed guard after committed stores cannot restart the complete region.
  fixture.p[entry] = long_move({}, false);
  fixture.p[entry + 1] = scalar_load(r2, 7);
  fixture.p[entry + 2] = program({})[0];
  const auto changed = detail::parse_block(fixture.p, seed, runtime);
  require(
      !detail::compile_region(compiler, changed, runtime, std::bit_cast<const void *>(&forbidden)),
      "A cursor clobber after a store cannot enter a replaying whole-region path");
}

void unknown_target_prefix() {
  Fixture fixture({});
  Core core(Memory::external(), &fixture);
  const auto seed = initial({}, 0);
  core.set_registers(seed);
  const std::array entries{entry};
  require(core.prepare(entries).prepared_entries == 1, "Prepare before a dynamic knowledge change");
  auto wanted = seed;
  auto data = fixture.data;
  constexpr Word completed = 3;
  for (unsigned i = 0; i < completed; ++i) { model_step(wanted, data, {}); }
  ++wanted.pc;
  fixture.known[0][target_slot] = 0;
  const auto result = core.run(length);
  const Fault failure{.kind = FaultKind::memory_read,
                      .space = Space::x,
                      .address = base + target_slot,
                      .pc = entry + completed};
  require(result.fault == failure && result.retired == completed && core.registers() == wanted &&
              fixture.data == data &&
              result.progress == Progress{.instructions = completed, .estimated_cycles = completed},
          "An unknown loaded target preserves all successful effects and the exact fault prefix");
}
}

int main() try {
  for (const auto mode : modes) {
    direct({}, mode);
    direct({.test_flags = true}, mode);
    prefixes({.test_flags = true}, mode);
    direct({.test_load = true}, mode);
    direct({.source_b = true, .test_load = true}, mode);
    prefixes({.test_load = true}, mode);
    prefixes({.source_b = true, .test_load = true}, mode);
    direct({.modifier = narrow_ring, .offset = word_mask}, mode);
    direct({.modifier = wide_ring, .offset = 9, .alias = true}, mode);
    prefixes({}, mode);
    prefixes({.modifier = narrow_ring, .offset = word_mask}, mode);
    for (unsigned selector = 0; selector < 8; ++selector) { direct({.selector = selector}, mode); }
    constexpr std::array counts{0U, 1U, 23U, 24U, 47U, 55U};
    for (const auto count : counts) {
      direct({.shift = count, .left = true}, mode);
      direct({.shift = count, .source_b = true, .destination_b = false}, mode);
    }
  }
  for (unsigned condition = 0; condition < 16; ++condition) {
    direct({.condition = condition, .test_load = true}, 0);
    direct({.condition = condition, .outer_loop = false}, 0);
    direct({.condition = condition, .tail = Tail::conditional_short}, 0);
    direct({.condition = condition, .tail = Tail::conditional_absolute}, 0);
  }
  for (const auto tail : {Tail::jump_register, Tail::jump_short, Tail::jump_absolute}) {
    direct({.tail = tail}, 0);
    prefixes({.tail = tail}, 0);
  }
  constexpr Word forever = 1U << 16;
  for (const auto count : {0U, 1U, 2U}) {
    direct({.at_tail = true, .loop_count = count}, 0);
    direct({.target = entry + length, .at_tail = true, .loop_count = count}, 0);
    direct({.flags = forever, .at_tail = true, .loop_count = count}, 0);
    prefixes({.at_tail = true, .loop_count = count}, 0);
  }
  resumed_do_body();
  ordinary_copy_region();
  admission_failures();
  unknown_target_prefix();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
