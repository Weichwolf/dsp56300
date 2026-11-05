// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word io_begin = 0xffff80, incoming = 0xab654321, payload = 0x654321;
constexpr Word long_base = 0x0a7080, short_base = 0x020080, negate = 0x200036;
constexpr Word alias_offset = 16, extension_pc = 1023;
constexpr Word short_minus_one = 0x01f840, ignored_n = 0xa54321, linear_high = 0xabffff;
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000da, data_address = 384;
constexpr Word round_a = 0x200011, load_x0_increment = 0x44d800;
constexpr unsigned stack_high = 60;
constexpr std::uint64_t rounding_seed = 0x1234567ffffe;

struct Form {
  unsigned reg = 4;
  bool long_form = false, y = false, load = true;
};

Word instruction(const Form form, const bool minus_one = false) {
  Word result = (form.long_form ? long_base : short_base) | form.reg;
  if (form.y) { result |= form.long_form ? 0x010000U : 0x20U; }
  if (form.load) { result |= form.long_form ? 0x40U : 0x10U; }
  if (!form.long_form && minus_one) { result |= short_minus_one; }
  return result;
}

Word word_count(const Form form) {
  return form.long_form ? 2U : 1U;
}

class Observer final : public Io {
public:
  bool read(const Space actual_space,
            const Word actual_address,
            const Word actual_pc,
            Word &incoming_value) noexcept override {
    incoming_value = incoming;
    return access(actual_space, actual_address, actual_pc, false, 0);
  }

  bool write(const Space actual_space,
             const Word actual_address,
             const Word actual_pc,
             const Word actual_value) noexcept override {
    return access(actual_space, actual_address, actual_pc, true, actual_value);
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
  Progress progress;
  Space space = Space::x;
  Word address = 0, pc = start, value = payload, replacement = 0;
  bool accept = true, expected_write = false, valid = true;
  unsigned calls = 0;

private:
  bool access(const Space actual_space,
              const Word actual_address,
              const Word actual_pc,
              const bool actual_write,
              const Word actual_value) noexcept {
    ++calls;
    valid = valid && actual_space == space && actual_address == address && actual_pc == pc &&
            actual_write == expected_write && (!expected_write || value == actual_value) &&
            core->registers() == expected && core->progress() == progress;
    try {
      bool reentry_rejected = false;
      try {
        static_cast<void>(core->run(1));
      } catch (const std::logic_error &) { reentry_rejected = true; }
      valid = valid && reentry_rejected;
      if (replacement != 0) { core->write(Space::program, replacement, negate); }
    } catch (...) { valid = false; }
    return accept;
  }
};

void port_case(Core &core, Observer &io, const Form form, const Word port, const bool accept) {
  core.reset();
  auto state = register_model::seed(0);
  state.x = payload;
  state.r[0] = form.long_form ? (port + sign24) & word_mask : (port + 1) & word_mask;
  state.n[0] = ignored_n;
  state.m[0] = linear_high;
  core.write(Space::program, start, instruction(form, true));
  if (form.long_form) { core.write(Space::program, start + 1, sign24); }
  core.set_registers(state);
  io.expected = state;
  io.expected.pc += word_count(form);
  io.progress = {};
  io.space = form.y ? Space::y : Space::x;
  io.address = port;
  io.pc = start;
  io.expected_write = !form.load;
  io.accept = accept;
  io.valid = true;
  io.calls = 0;
  auto expected = io.expected;
  if (!form.load && form.reg == stack_high) {
    io.value = register_model::read(expected, form.reg);
    io.expected = expected;
  } else {
    io.value = payload;
  }
  const auto result = core.run(1);
  if (form.load && accept) { register_model::write(expected, form.reg, payload); }
  require(io.valid && io.calls == 1 && core.registers() == expected,
          "Displaced I/O observes exact pre-transfer state, current stack effects and no R update");
  const auto cost = form.long_form ? 3U : 2U;
  const Fault fault = accept ? Fault{}
                             : Fault{.kind = form.load ? FaultKind::io_read : FaultKind::io_write,
                                     .space = io.space,
                                     .address = port,
                                     .value = form.load ? 0U : io.value,
                                     .pc = start};
  require(result.dispatches == (accept ? 1U : 0U) && result.fault == fault &&
              result.progress ==
                  (accept ? Progress{.instructions = 1, .estimated_cycles = cost} : Progress{}),
          "Accepted/rejected displaced I/O has exact fault and retirement accounting");
  require(core.read(io.space, port) == 0, "Full-span backing cannot bypass I/O routing");
  if (!accept) {
    require(core.run(2).dispatches == 0 && io.calls == 1 && core.registers() == expected,
            "Failed I/O remains stopped without replaying reads, writes or source-stack pops");
  }
}

void ports(const Backend backend) {
  const MemoryArena arena;
  Observer io;
  Core core(arena.buffers(), &io, {.backend = backend});
  io.core = &core;
  for (const bool long_form : {false, true}) {
    for (const bool y : {false, true}) {
      for (const bool load : {false, true}) {
        for (const bool accept : {false, true}) {
          for (Word port = io_begin; port <= word_mask; ++port) {
            port_case(core, io, {.long_form = long_form, .y = y, .load = load}, port, accept);
          }
        }
      }
    }
  }
  for (const bool accept : {false, true}) {
    port_case(core, io, {.reg = stack_high, .long_form = true, .load = false}, io_begin, accept);
  }
}

void callback_invalidation(const Backend backend, const bool long_form) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Observer io;
  Core core({.program = p, .x = x, .y = y}, &io, {.backend = backend});
  io.core = &core;
  auto state = register_model::seed(0);
  state.r[0] = io_begin;
  const Form form{.long_form = long_form};
  const Word next = start + 2 + word_count(form);
  core.write(Space::program, start, mac_a);
  core.write(Space::program, start + 1, mac_b);
  core.write(Space::program, start + 2, instruction(form));
  if (long_form) { core.write(Space::program, start + 3, 0); }
  core.write(Space::program, next, 0);
  core.set_registers(state);
  for (const Word alu : {mac_a, mac_b}) {
    const auto old = state;
    sequence_model::alu(state, old, alu);
  }
  state.pc = next;
  io.expected = state;
  io.progress = {.instructions = 2, .estimated_cycles = 2};
  io.address = io_begin;
  io.pc = start + 2;
  io.replacement = next;
  auto expected = state;
  register_model::write(expected, 4, payload);
  const auto old = expected;
  sequence_model::alu(expected, old, negate);
  expected.pc = next + 1;
  const auto result = core.run(4);
  require(
      result.dispatches == 4 && result.fault.kind == FaultKind::none &&
          result.progress == Progress{.instructions = 4, .estimated_cycles = long_form ? 6U : 5U} &&
          core.registers() == expected && io.calls == 1 && io.valid,
      "Displaced I/O materializes dirty native A/B, rejects reentry and invalidates its successor");
  if (backend == Backend::jit) {
    require(core.jit_statistics().native_alu_instructions >= 3,
            "The observer sequence includes actual native arithmetic around the general MOVE");
  }
}

void extension_cache(const Backend backend, const bool y) {
  std::array<Word, memory_words * 2> p{};
  std::array<Word, memory_words * 2> x{};
  std::array<Word, memory_words * 2> other{};
  const std::span shared{p.data() + alias_offset, p.size() - alias_offset};
  Core core({.program = p,
             .x = y ? std::span<Word>{x} : shared,
             .y = y ? shared : std::span<Word>{other}},
            nullptr,
            {.backend = backend});
  const auto space = y ? Space::y : Space::x;
  core.write(
      Space::program, extension_pc, instruction({.reg = y ? 6U : 4U, .long_form = true, .y = y}));
  core.write(space, data_address, payload);
  core.write(space, data_address + 1, payload - 1);
  auto initial = initial_state();
  initial.pc = extension_pc;
  initial.r[0] = data_address;
  for (const Word offset : {0U, 1U, 0U, 1U}) {
    // The P extension is in the next decode page, written through an aliased data bank.
    core.write(space, extension_pc + 1 - alias_offset, offset);
    core.set_registers(initial);
    auto expected = initial;
    expected.pc += 2;
    (y ? expected.y : expected.x) = payload - offset;
    require(core.run(1).dispatches == 1 && core.registers() == expected,
            "Warm displaced-MOVE extension changes invalidate the preceding decode page through "
            "X/Y aliases");
  }
}

void program_store(const Backend backend, const Form form) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> other{};
  const std::span shared{p.data() + alias_offset, p.size() - alias_offset};
  Core core({.program = p,
             .x = form.y ? std::span<Word>{other} : shared,
             .y = form.y ? shared : std::span<Word>{other}},
            nullptr,
            {.backend = backend});
  auto initial = register_model::seed(0);
  const Word next = start + word_count(form);
  initial.x = negate;
  initial.r[0] = next - alias_offset;
  core.write(Space::program, start, instruction(form));
  if (form.long_form) { core.write(Space::program, start + 1, 0); }
  core.write(Space::program, next, 0);
  core.set_registers(initial);
  auto expected = initial;
  sequence_model::alu(expected, initial, negate);
  expected.pc = next + 1;
  require(core.run(2).dispatches == 2 && core.registers() == expected && p[next] == negate,
          "A displaced X/Y store that aliases P discards the planned successor before execution");
}

void span_and_stack(const Backend backend, const Form form) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Core core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  auto initial = register_model::seed(0);
  initial.r[0] = memory_words;
  initial.x = payload;
  core.write(Space::program, start, instruction(form));
  if (form.long_form) { core.write(Space::program, start + 1, 0); }
  core.set_registers(initial);
  auto expected = initial;
  expected.pc += word_count(form);
  const auto result = core.run(1);
  require(result.dispatches == 0 && result.progress == Progress{} &&
              result.fault.kind == (form.load ? FaultKind::memory_read : FaultKind::memory_write) &&
              result.fault.address == memory_words && core.registers() == expected,
          "Failed displaced memory transfer preserves all data/AGU fields and retires nothing");
  if (!form.long_form) { return; }
  core.reset();
  initial.r[0] = 0;
  initial.sc = form.load ? static_cast<Word>(stack_entries - 1) : 0;
  core.write(Space::program,
             start,
             instruction({.reg = stack_high, .long_form = true, .y = form.y, .load = form.load}));
  core.write(Space::program, start + 1, 0);
  core.set_registers(initial);
  expected = initial;
  expected.pc += 2;
  require(core.run(1).dispatches == 0 && core.fault().kind == FaultKind::stack &&
              core.registers() == expected && x[0] == 0 && y[0] == 0,
          "Long displacement SSH underflow/overflow preserves the stack and avoids a store after "
          "failed pop");
}

void numeric_modes(const Backend backend) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Core core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  const std::array program{mac_a,
                           mac_b,
                           instruction({.reg = register_model::reg_sr, .long_form = true}),
                           Word{0},
                           round_a};
  core.load(Space::program, start, program);
  auto initial = register_model::seed(0);
  initial.r[0] = 0;
  initial.x = initial.y = 1;
  initial.a = rounding_seed; // The first MAC produces an exact normal-mode half tie.
  for (const Word mode :
       {0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up}) {
    for (const Word flags : {0U, status_byte}) {
      core.set_registers(initial);
      core.write(Space::x, 0, mode | flags);
      auto expected = initial;
      for (const auto operation : {mac_a, mac_b}) {
        const auto old = expected;
        sequence_model::alu(expected, old, operation);
      }
      expected.sr = mode | flags;
      const auto old = expected;
      sequence_model::alu(expected, old, round_a);
      expected.pc = start + static_cast<Word>(program.size());
      const auto before = core.progress();
      const auto result = core.run(4);
      require(
          result.dispatches == 4 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = before.instructions + 4,
                                          .estimated_cycles = before.estimated_cycles + 6} &&
              core.registers() == expected,
          "Warm long-MOVE SR writes discard stale numeric specialization before native rounding");
    }
  }
}

void modifier_changes(const Backend backend) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Core core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  const std::array program{
      instruction({.reg = 32, .long_form = true}) | (1U << 8), Word{0}, load_x0_increment};
  core.load(Space::program, start, program);
  auto initial = register_model::seed(0);
  initial.r[0] = 15;
  initial.r[1] = data_address;
  core.write(Space::x, initial.r[0], payload);
  for (const Word modifier : {linear_high, 0U, 0xab800fU, 0U, linear_high}) {
    core.set_registers(initial);
    core.write(Space::x, data_address, modifier);
    auto expected = initial;
    expected.m[0] = modifier;
    expected.r[0] =
        address_model::updated({.modifier = modifier, .cursor = initial.r[0], .mode = 3});
    register_model::write(expected, 4, payload);
    expected.pc = start + static_cast<Word>(program.size());
    const auto before = core.jit_statistics().native_move_instructions;
    const auto result = core.run(2);
    require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
                core.registers() == expected,
            "Guest long-MOVE M writes are visible to the following warm native AGU guard");
    if (backend == Backend::jit) {
      require(
          core.jit_statistics().native_move_instructions == before + (modifier == 0 ? 0U : 1U),
          "Guest M replacement selects actual native linear/ring transfers or reverse fallback");
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::ports(backend);
      dsp56300::test::numeric_modes(backend);
      dsp56300::test::modifier_changes(backend);
      for (const bool long_form : {false, true}) {
        dsp56300::test::callback_invalidation(backend, long_form);
        for (const bool y : {false, true}) {
          dsp56300::test::program_store(backend, {.long_form = long_form, .y = y, .load = false});
          for (const bool load : {false, true}) {
            dsp56300::test::span_and_stack(backend, {.long_form = long_form, .y = y, .load = load});
          }
        }
      }
      dsp56300::test::extension_cache(backend, false);
      dsp56300::test::extension_cache(backend, true);
    }
    std::cout << dsp56300::test::checks << " displaced MOVE boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
