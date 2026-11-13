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
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using isa::Opcode;
constexpr Word entry = 16, root = 8, base = 0x4000, words = 64, length = 5;
constexpr Word lf = 1U << 15, forever = 1U << 16, linear = 0xffff;
constexpr Word target_slot = 16, store_slot = 24, target = 96, previous_la = 200;
constexpr Word repeat_entry = 40, repeat_count = 3, saved_count = 7;
constexpr Word irq_entry = 30, irq_mask = 0x300, clear_irq_mask = 0xfc;
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
    code[irq_entry] = named(Opcode::andi_control) | (clear_irq_mask << 8);
    code[irq_entry + 1] = named(Opcode::nop);
    code[repeat_entry] = named(Opcode::rep_immediate) | (repeat_count << 8);
    code[repeat_entry + 1] = named(Opcode::asr_a);
    code[repeat_entry + 2] = named(Opcode::stop);
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

  [[nodiscard]] bool interrupt_pending(const Word status) const noexcept override {
    return pending && (status & irq_mask) == 0;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    ++window_queries;
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
  Data data{};
  std::array<std::uint8_t, words> known{};
  MemoryWindow window;
  std::uint64_t revision = 0;
  unsigned accesses = 0, window_queries = 0;
  bool left_shift = false;
  bool pending = false;
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

NativeImage prepare_image() {
  const Fixture fixture;
  std::array<Word, 128> program{};
  for (std::size_t pc = 0; pc < program.size(); ++pc) {
    program[pc] = fixture.code[pc].value_or(0);
  }
  constexpr std::array<std::array<Word, 2>, 6> units{std::array<Word, 2>{0, 2},
                                                     {root, 1},
                                                     {entry, 2},
                                                     {entry + 2, 4},
                                                     {irq_entry, 2},
                                                     {repeat_entry, 3}};
  std::array<CodeSpan, units.size()> code{};
  std::vector<Word> entries;
  for (std::size_t index = 0; index < units.size(); ++index) {
    code[index] = {.address = units[index][0],
                   .words = std::span(program).subspan(units[index][0], units[index][1])};
    for (Word offset = 0; offset < units[index][1]; ++offset) {
      entries.push_back(units[index][0] + offset);
    }
  }
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
  return Core::compile({.variants = variants}, {});
}

const NativeImage &compiled_image() {
  static const NativeImage image = prepare_image();
  return image;
}

void bind_image(Core &core) {
  // Establish the mapping registers before capturing the consumer's RAM grants.
  core.set_registers(initial(0));
  core.install(compiled_image(), 0);
}

void matrix() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  bind_image(core);
  constexpr std::array loops{Loop{},
                             Loop{.active = true, .address = entry - 1},
                             Loop{.active = true, .address = entry + length},
                             Loop{.active = true, .address = word_mask},
                             Loop{.active = true, .address = entry + 1},
                             Loop{.active = true, .address = entry + length - 1}};
  for (const auto mode : modes) {
    for (const auto loop : loops) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        fixture.reset();
        const auto state = initial(mode, loop, flags | ((flags & 1U) != 0 ? forever : 0U));
        compare(core, fixture, state, {});
      }
      for (Word count = 0; count <= length; ++count) {
        fixture.reset();
        compare(core, fixture, initial(mode, loop), {.instructions = count});
      }
      for (Word deadline = 0; deadline <= length + 3; ++deadline) {
        fixture.reset();
        compare(core,
                fixture,
                initial(mode, loop),
                {.instructions = std::numeric_limits<Word>::max(), .deadline = deadline},
                true);
      }
    }
  }
}

void interior_entries() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  bind_image(core);
  for (const auto mode : modes) {
    for (Word offset = 0; offset < length; ++offset) {
      for (Word quota = 0; quota <= length - offset; ++quota) {
        fixture.reset();
        auto state = initial(mode);
        state.pc += offset;
        compare(core, fixture, state, {.instructions = quota});
      }
    }
  }
}

void unknown_data() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  bind_image(core);
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
              fixture.data == wanted.data &&
              result.progress == Progress{.instructions = 3, .estimated_cycles = 3},
          "Compiled semantic code preserves the exact unknown-word fault prefix");
}

void denied_store() {
  Fixture fixture;
  fixture.window.writable = false;
  ++fixture.revision;
  Core core(Memory::external(), &fixture);
  bind_image(core);
  const auto state = initial(0, {.active = true, .address = previous_la});
  auto wanted = expected(fixture, state, {.instructions = 2});
  const auto address = wanted.reg.r[3];
  wanted.reg.r[3] = address_model::updated(
      {.modifier = wanted.reg.m[3], .cursor = address, .offset = wanted.reg.n[3], .mode = 3});
  ++wanted.reg.pc;
  const auto value = register_model::bus_word(wanted.reg, wanted.reg.a);
  core.set_registers(state);
  const auto result = core.run(length);
  const Fault fault{.kind = FaultKind::memory_write,
                    .space = Space::x,
                    .address = address,
                    .value = value,
                    .pc = entry + 2};
  require(result.fault == fault && result.retired == 2 && core.registers() == wanted.reg &&
              fixture.data == wanted.data &&
              result.progress == Progress{.instructions = 2, .estimated_cycles = 2},
          "Native AOT preserves the denied store address update, bus flags and fault prefix");
}

void unprepared_target() {
  Fixture fixture;
  fixture.code[target] = named(Opcode::nop);
  Core core(Memory::external(), &fixture);
  bind_image(core);
  auto state = initial(0);
  state.pc = target;
  core.set_registers(state);
  const auto accesses = fixture.accesses;
  require(core.run(0).fault == Fault{} && core.registers() == state &&
              core.progress() == Progress{} && fixture.accesses == accesses,
          "A zero grant does not inspect or execute an unprepared target");
  require(core.run_until(0).fault == Fault{} && core.registers() == state &&
              core.progress() == Progress{} && fixture.accesses == accesses,
          "An expired deadline does not inspect or execute an unprepared target");
  const auto result = core.run();
  const Fault fault{.kind = FaultKind::unprepared_execution,
                    .space = Space::program,
                    .address = target,
                    .pc = target};
  require(result.fault == fault && result.retired == 0 && core.registers() == state &&
              result.progress == Progress{} && fixture.accesses == accesses,
          "Valid but unprepared code fails without fetching or invoking lazy translation");
}

struct RepeatGrant {
  Word instructions = 0;
  Word deadline = 0;
  bool timed = false;
};

void repeat_case(const Word mode, const Word flags, const RepeatGrant grant) {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  bind_image(core);
  auto state = initial(mode, {}, flags);
  state.pc = repeat_entry;
  state.lc = saved_count;
  auto wanted = state;
  Word cycles = 0;
  Word retired = 0;
  constexpr Word setup_cycles = 5;
  constexpr Word stop_cycles = 10;
  const auto allowed = [&] {
    return grant.timed ? cycles < grant.deadline : retired < grant.instructions;
  };
  if (allowed()) {
    ++wanted.pc;
    wanted.lc = repeat_count;
    cycles += setup_cycles;
    ++retired;
    for (Word shifted = 0; shifted < repeat_count && allowed(); ++shifted) {
      const auto old = wanted;
      shift_model::apply(wanted, old, {.count = 1, .saturate = false});
      wanted.lc = repeat_count - shifted - 1;
      ++cycles;
      ++retired;
      if (wanted.lc == 0) {
        wanted.lc = saved_count;
        ++wanted.pc;
      }
    }
    if (wanted.pc == repeat_entry + 2 && allowed()) {
      ++wanted.pc;
      cycles += stop_cycles;
      ++retired;
    }
  }
  core.set_registers(state);
  const auto result = grant.timed ? core.run_until(grant.deadline) : core.run(grant.instructions);
  const bool stopped = wanted.pc == repeat_entry + 3;
  require(result.fault == Fault{} && result.retired == retired && core.registers() == wanted &&
              result.state == (stopped ? ExecutionState::stopped : ExecutionState::running) &&
              result.progress == Progress{.instructions = retired, .estimated_cycles = cycles},
          "Native AOT REP preserves each prefix, CCR, saved LC and the genuine STOP boundary");
  auto final = state;
  for (Word shifted = 0; shifted < repeat_count; ++shifted) {
    const auto old = final;
    shift_model::apply(final, old, {.count = 1, .saturate = false});
  }
  final.pc = repeat_entry + 3;
  final.lc = saved_count;
  const auto data_before = fixture.data;
  const auto accesses = fixture.accesses;
  const auto completed = core.run();
  require(completed.fault == Fault{} && completed.retired == repeat_count + 2 - retired &&
              completed.state == ExecutionState::stopped && core.registers() == final &&
              completed.progress ==
                  Progress{.instructions = repeat_count + 2,
                           .estimated_cycles = setup_cycles + repeat_count + stop_cycles} &&
              fixture.data == data_before && fixture.accesses == accesses,
          "Native REP resumes every hidden continuation across owner calls without a code fetch");
}

void repeat_boundaries() {
  for (const auto mode : modes) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      for (Word count = 0; count <= repeat_count + 2; ++count) {
        repeat_case(mode, flags, {.instructions = count});
      }
    }
    for (Word deadline = 0; deadline <= repeat_count + 6; ++deadline) {
      repeat_case(mode, 0, {.deadline = deadline, .timed = true});
    }
  }
}

void direct_grants() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  bind_image(core);
  const auto queries = fixture.window_queries;
  compare(core, fixture, initial(0), {});
  require(fixture.window_queries == queries && fixture.accesses == 0,
          "Ordinary native RAM accesses reuse owner-bound grants without virtual queries");
}

void owner_boundaries() {
  Fixture fixture;
  Core core(Memory::external(), &fixture);
  bind_image(core);
  auto state = initial(0);
  state.pc = 0;
  core.set_registers(state);
  const auto stopped = core.run();
  require(stopped.state == ExecutionState::stopped && stopped.retired == 1 &&
              stopped.fault == Fault{} && core.registers().pc == 1,
          "Original STOP semantics retire once and return to the native owner");
  Core interrupt(Memory::external(), &fixture);
  bind_image(interrupt);
  state = initial(0);
  state.pc = irq_entry;
  state.sr = irq_mask;
  auto wanted = state;
  wanted.pc = irq_entry + 1;
  wanted.sr = 0;
  fixture.pending = true;
  interrupt.set_registers(state);
  const auto yielded = interrupt.run(2);
  require(yielded.retired == 1 && yielded.yielded && yielded.fault == Fault{} &&
              interrupt.registers() == wanted,
          "A genuine mask change yields before the next compiled instruction on an eligible IRQ");
}
}

int main() {
  try {
    const auto report = compiled_image().report();
    constexpr std::uint64_t sites = 14;
    require(report.prepared_entries == sites && report.unresolved_entries == 0 &&
                report.native_bytes != 0 && report.failure == Fault{},
            "The public factory prepares the complete authored image before execution");
    matrix();
    interior_entries();
    unknown_data();
    owner_boundaries();
    direct_grants();
    denied_store();
    unprepared_target();
    repeat_boundaries();
    std::cout << checks << " public native-image full-state/prefix checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
