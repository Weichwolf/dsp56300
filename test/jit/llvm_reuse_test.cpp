// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using isa::Opcode;
constexpr Word entry = 16, root = 8, base = 0x4000, words = 64, length = 5;
constexpr Word lf = 1U << 15, forever = 1U << 16, linear = 0xffff;
constexpr Word target_slot = 16, store_slot = 24, target = 96, previous_la = 200;
constexpr unsigned x0 = 4, a = 14, r1 = 17, high_selector = 17;
constexpr Word high_register_mask = 0x18;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
using Data = std::array<Word, words>;

Word named(const Opcode opcode) {
  return static_cast<Word>(opcode);
}

struct Transfer {
  unsigned reg, cursor;
  bool load, increment;
};

Word memory(const Transfer transfer) {
  constexpr Word fields = (7U << 16) | (high_register_mask << high_selector);
  return (named(Opcode::move_x_effective) & ~fields) | ((transfer.reg & 7U) << 16) |
         ((transfer.reg & high_register_mask) << high_selector) | (1U << 14) |
         (transfer.load ? 1U << 15 : 0U) | ((transfer.increment ? 3U : 4U) << 11) |
         (transfer.cursor << 8);
}

struct Fixture final : Io {
  Fixture() {
    code[0] = named(Opcode::stop);
    code[1] = named(Opcode::rts);
    code[root] = named(Opcode::jmp_absolute_short) | entry;
    code[entry] = memory({.reg = x0, .cursor = 0, .load = true, .increment = true}) |
                  ((named(Opcode::mpy_register_a) | (5U << 4)) & status_byte);
    shift(false);
    code[entry + 2] = memory({.reg = a, .cursor = 3, .load = false, .increment = true});
    code[entry + 3] = memory({.reg = r1, .cursor = 2, .load = true, .increment = false});
    code[entry + 4] = named(Opcode::jcc_effective) | (4U << 11) | (1U << 8) | 2U;
    code[entry + length] = named(Opcode::rts);
    window = {.address = base, .words = data, .known = known, .writable = true};
    reset();
  }

  void reset() {
    known.fill(1);
    for (Word i = 0; i < words; ++i) { data[i] = (i * sign24) ^ i; }
    data[target_slot] = target;
  }

  void shift(const bool left) {
    left_shift = left;
    code[entry + 1] = named(left ? Opcode::asl_immediate : Opcode::asr_immediate) | (3U << 1) | 1U;
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word pc) const noexcept override {
    return pc < code.size() ? code[pc] : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return space == Space::x ? std::span(&window, 1) : std::span<const MemoryWindow>{};
  }

  bool read(const Space space, const Word at, Word /*pc*/, Word &value) noexcept override {
    ++accesses;
    if (space != Space::program || at >= code.size()) { return false; }
    const auto word = code[at];
    if (!word) { return false; }
    value = *word;
    return true;
  }

  bool write(Space /*space*/, Word /*at*/, Word /*pc*/, Word /*value*/) noexcept override {
    ++accesses;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*at*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    ++accesses;
    return false;
  }

  std::array<std::optional<Word>, 128> code{};
  Data data{}, replacement{};
  std::array<std::uint8_t, words> known{};
  MemoryWindow window;
  std::uint64_t revision = 0;
  unsigned accesses = 0;
  bool left_shift = false;
};

struct Loop {
  bool active = false;
  Word address = 0, count = 2;
};

Registers initial(const Word mode, const Loop loop = {}, const Word flags = 0) {
  auto state = register_model::seed(mode);
  state.pc = entry;
  state.sr = mode | flags | (loop.active ? lf : 0U);
  if ((flags & 1U) != 0) { state.x &= ~std::uint64_t{word_mask}; }
  state.la = loop.address;
  state.lc = loop.count;
  state.m.fill(linear);
  state.r.fill(base);
  state.r[0] = base + 4;
  state.r[2] = base + target_slot;
  state.r[3] = base + store_slot;
  state.sp = 2;
  state.sc = 2;
  state.stack[2] = std::uint64_t{entry} << word_bits;
  state.stack[1] = (std::uint64_t{previous_la} << word_bits) | 7U;
  return state;
}

struct Model {
  Registers reg;
  Data data;
  Word retired = 0, cycles = 0;
};

struct Grant {
  Word instructions = length, deadline = length + 3;
};

void step(Model &model, const Fixture &fixture) {
  const auto old = model.reg;
  auto &state = model.reg;
  const auto pc = state.pc;
  ++state.pc;
  switch (pc - entry) {
    case 0: {
      const auto value = model.data.at(old.r[0] - base) & word_mask;
      state.r[0] = address_model::updated(
          {.modifier = old.m[0], .cursor = old.r[0], .offset = old.n[0], .mode = 3});
      register_model::write(state, x0, value);
      sequence_model::alu(state, old, (named(Opcode::mpy_register_a) | (5U << 4)) & status_byte);
      break;
    }
    case 1:
      shift_model::apply(
          state,
          old,
          {.count = 3, .left = fixture.left_shift, .destination_b = true, .saturate = false});
      break;
    case 2:
      model.data.at(old.r[3] - base) = register_model::bus_word(state, old.a);
      state.r[3] = address_model::updated(
          {.modifier = old.m[3], .cursor = old.r[3], .offset = old.n[3], .mode = 3});
      break;
    case 3: register_model::write(state, r1, model.data.at(old.r[2] - base) & word_mask); break;
    case 4:
      if ((state.sr & flag_z) == 0) { state.pc = state.r[1]; }
      break;
    default: throw std::logic_error("The authored model left its natural region");
  }
  if ((state.sr & lf) != 0 && pc == state.la && state.pc == ((state.la + 1) & word_mask)) {
    if (state.lc == 1 && (state.sr & forever) == 0) {
      loop_frames::complete(state);
    } else {
      state.lc = (state.lc - 1) & word_mask;
      state.pc = static_cast<Word>(state.stack[state.sp % stack_entries] >> word_bits);
    }
  }
  ++model.retired;
  model.cycles += pc == entry + length - 1 ? 4U : 1U;
}

Model expected(const Fixture &fixture, const Registers &state, const Grant grant) {
  Model model{.reg = state, .data = {}};
  std::ranges::copy(fixture.window.words, model.data.begin());
  while (model.retired < grant.instructions && model.cycles < grant.deadline) {
    step(model, fixture);
  }
  return model;
}

void compare(Core &core,
             Fixture &fixture,
             const Registers &state,
             const Grant grant,
             const bool timed = false) {
  core.set_registers(state);
  const auto wanted = expected(fixture, state, grant);
  const auto before = core.progress();
  const auto result = timed ? core.run_until(before.estimated_cycles + grant.deadline)
                            : core.run(grant.instructions);
  if (core.registers() != wanted.reg || result.fault != Fault{}) {
    const auto actual = core.registers();
    std::cerr << "mode/LA/flags/grant/timed=" << state.sr << '/' << state.la << '/'
              << (state.sr & status_byte) << '/' << grant.instructions << '/' << grant.deadline
              << '/' << timed << " pc/A/B/X/SR wanted=" << wanted.reg.pc << '/' << wanted.reg.a
              << '/' << wanted.reg.b << '/' << wanted.reg.x << '/' << wanted.reg.sr
              << " actual=" << actual.pc << '/' << actual.a << '/' << actual.b << '/' << actual.x
              << '/' << actual.sr << '\n';
  }
  require(result.fault == Fault{} && core.registers() == wanted.reg &&
              result.retired == wanted.retired &&
              std::ranges::equal(fixture.window.words, wanted.data) &&
              result.progress.instructions - before.instructions == wanted.retired &&
              result.progress.estimated_cycles - before.estimated_cycles == wanted.cycles,
          "Shared regions preserve full registers, ordered stores and every execution prefix");
}

void prepare(Core &core, Fixture &fixture, const Word mode) {
  auto state = initial(mode);
  state.pc = 0;
  core.set_registers(state);
  const auto accesses = fixture.accesses;
  const auto before = core.progress();
  const std::array entries{root};
  const auto report = core.prepare(entries);
  require(report.failure == Fault{} && report.prepared_entries == 1 && report.native_bytes > 0 &&
              core.registers() == state && core.progress() == before &&
              fixture.accesses == accesses,
          "Prepare an encoded CFG successor before entering it, without guest execution or I/O");
  const auto repeated = core.prepare(entries);
  require(repeated.prepared_entries == 1 && repeated.native_bytes == 0,
          "Preparing the established CFG again retains its one native region");
}

void outside_contexts() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  constexpr std::array loops{Loop{},
                             Loop{.active = true, .address = entry - 1},
                             Loop{.active = true, .address = entry + length},
                             Loop{.active = true, .address = word_mask}};
  for (const auto loop : loops) {
    fixture.reset();
    core.set_registers(initial(0, loop));
    require(core.run(1).retired == 1, "Establish ordinary bindings before explicit preparation");
  }
  for (const auto mode : modes) {
    prepare(core, fixture, mode);
    for (const auto loop : loops) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        fixture.reset();
        const auto state = initial(mode, loop, flags | ((flags & 1U) != 0 ? forever : 0U));
        compare(core, fixture, state, {});
      }
      for (Word prefix = 0; prefix <= length; ++prefix) {
        fixture.reset();
        compare(core, fixture, initial(mode, loop), {.instructions = prefix});
      }
      for (Word deadline = 0; deadline <= length + 3; ++deadline) {
        fixture.reset();
        compare(core, fixture, initial(mode, loop), {.deadline = deadline}, true);
      }
      core.set_registers(initial(mode, loop));
      const std::array entries{entry};
      const auto report = core.prepare(entries);
      require(report.prepared_entries == 1 && report.native_bytes == 0,
              "An external enclosing loop reuses the prepared body without materialization");
    }
  }
}

void explicit_optimization() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  const std::array dormant{Word{0}};
  const auto initial_report = core.prepare(dormant);
  require(initial_report.failure == Fault{} && initial_report.prepared_entries == 0 &&
              initial_report.native_bytes == 0,
          "A dormant CFG does not establish an unrelated arithmetic entry");
  compare(core, fixture, initial(0, {.active = true, .address = previous_la}), {});
  // If run() had materialized O3, this explicit preparation would emit no new text.
  prepare(core, fixture, 0);
}

void active_origin() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  core.set_registers(initial(0, {.active = true, .address = previous_la}));
  const std::array entries{entry};
  const auto prepared = core.prepare(entries);
  require(prepared.failure == Fault{} && prepared.prepared_entries == 1 &&
              prepared.native_bytes > 0,
          "An active enclosing loop and inactive CFG binding share the initial image");
  compare(core, fixture, initial(0), {});
  fixture.reset();
  auto state = initial(0);
  state.m[0] = 3;
  state.m[3] = 0;
  compare(core, fixture, state, {});
  fixture.reset();
  compare(core, fixture, initial(0, {}, 1), {});
  core.set_registers(initial(0));
  require(core.prepare(entries).native_bytes == 0,
          "An inactive caller retains code first prepared under an external active loop");
}

void excluded_ends() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  prepare(core, fixture, 0);
  for (Word end = entry; end < entry + length; ++end) {
    for (const Word count : {0U, 1U, 2U}) {
      fixture.reset();
      fixture.data[target_slot] = entry + length;
      compare(core, fixture, initial(0, {.active = true, .address = end, .count = count}), {});
    }
  }
}

void invalidation() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  prepare(core, fixture, 0);
  fixture.shift(true);
  ++fixture.revision;
  prepare(core, fixture, 0);
  compare(core, fixture, initial(0, {.active = true, .address = previous_la}), {});
  fixture.reset();
  fixture.replacement = fixture.data;
  fixture.replacement[4] = sign24;
  fixture.window.words = fixture.replacement;
  ++fixture.revision;
  prepare(core, fixture, 0);
  compare(core, fixture, initial(0, {.active = true, .address = previous_la}), {});
  core.reset();
  prepare(core, fixture, 0);
  fixture.reset();
  auto state = initial(0, {.active = true, .address = previous_la});
  state.m[0] = 3;
  state.m[3] = 0;
  compare(core, fixture, state, {});
}

void denied_store() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  prepare(core, fixture, 0);
  fixture.window.writable = false;
  ++fixture.revision;
  const auto state = initial(0, {.active = true, .address = previous_la});
  core.set_registers(state);
  const std::array entries{root};
  const auto report = core.prepare(entries);
  require(report.failure == Fault{} && report.prepared_entries == 0 && report.native_bytes == 0,
          "A newly read-only grant cannot reuse a retained writable region");
  auto wanted = expected(fixture, state, {.instructions = 2});
  const auto address = wanted.reg.r[3];
  wanted.reg.r[3] = address_model::updated(
      {.modifier = wanted.reg.m[3], .cursor = address, .offset = wanted.reg.n[3], .mode = 3});
  ++wanted.reg.pc;
  const auto value = register_model::bus_word(wanted.reg, wanted.reg.a);
  const auto result = core.run(length);
  const Fault fault{.kind = FaultKind::memory_write,
                    .space = Space::x,
                    .address = address,
                    .value = value,
                    .pc = entry + 2};
  require(result.fault == fault && result.retired == 2 && core.registers() == wanted.reg &&
              fixture.data == wanted.data && result.progress.instructions == 2 &&
              result.progress.estimated_cycles == 2,
          "A denied store preserves its exact address update, bus flags and prior effects");
}

void unknown_prefix() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  prepare(core, fixture, 0);
  const auto state = initial(0, {.active = true, .address = previous_la});
  auto wanted = expected(fixture, state, {.instructions = 3});
  ++wanted.reg.pc;
  fixture.known[target_slot] = 0;
  core.set_registers(state);
  const auto result = core.run(length);
  const Fault fault{.kind = FaultKind::memory_read,
                    .space = Space::x,
                    .address = base + target_slot,
                    .pc = entry + 3};
  require(result.fault == fault && result.retired == 3 && core.registers() == wanted.reg &&
              fixture.data == wanted.data && result.progress.instructions == 3 &&
              result.progress.estimated_cycles == 3,
          "A shared body's rejected memory proof retains the exact ordinary fault prefix");
}
}

int main() try {
  outside_contexts();
  explicit_optimization();
  active_origin();
  excluded_ends();
  invalidation();
  unknown_prefix();
  denied_store();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
