// SPDX-License-Identifier: GPL-3.0-only
#include "a64_cache_state.hpp"
#include "a64_loop_state.hpp"
#include "a64_native.hpp"
#include "address_model.hpp"
#include "cache_isa_model.hpp"
#include "cache_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "loop_control_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "vector_state.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using State = loop_model::State;
using Form = cache_isa_model::Form;
using Kind = cache_isa_model::Kind;
constexpr std::array widths{128U, 256U};
constexpr std::array basics{Kind::flush, Kind::flush_unlocked, Kind::free};
constexpr Word location = 384, invalid_modifier = 0xc001;
constexpr Word ce = Word{1} << 19, cm = Word{1} << 13, sm = Word{1} << 20;
constexpr Word unqualified = 0x226c00, bank = 0xabcdf7, old_opcode = 0x200013, saved_lc = 0x345;

State seed(const Word status) {
  State state;
  state.reg = register_model::seed(status);
  state.reg.sr = status;
  state.reg.vba = bank;
  state.reg.r.fill(location);
  state.reg.n.fill(3);
  state.reg.m.fill(address_model::linear);
  return state;
}

a64::Instruction instruction(const Form form, const State &initial) {
  return {.opcode = cache_isa_model::opcode(form),
          .extension = form.extra,
          .pc = initial.reg.pc,
          .mode = initial.reg.sr,
          .words = cache_isa_model::words(form),
          .cycles = cache_isa_model::cycles(form)};
}

Word address(State &state, const Form form, const a64::Instruction input) {
  if (cache_isa_model::relative(form)) { return (input.pc + form.extra) % address_words; }
  if (!cache_isa_model::effective(form)) { return 0; }
  if (form.mode == 6) {
    if (form.index != 0) {
      loop_model::fail(state, {.pc = input.pc}, FaultKind::invalid_instruction, input.opcode);
    }
    return form.extra;
  }
  return loop_model::effective(
      state, {.address = {.mode = form.mode, .index = form.index}}, {.pc = input.pc});
}

void apply(cache_model::Model &cache, const Form form, const Word target) {
  if (form.kind == Kind::flush || form.kind == Kind::flush_unlocked) {
    cache.flush(form.kind == Kind::flush_unlocked);
  } else if (form.kind == Kind::free) {
    cache.free();
  } else {
    cache.lock(target, form.kind == Kind::lock || form.kind == Kind::lock_relative);
  }
}

void disabled(State &state, const Word pc) {
  if (state.vector.phase == detail::VectorPhase::none) {
    state.vector = {.resume = static_cast<Word>((pc + 2) % address_words),
                    .address = ((state.reg.vba / 256) * 256) + 4,
                    .level = 3,
                    .phase = detail::VectorPhase::pending};
  }
}

bool prepare(State &state, const Form form, const a64::Instruction input) {
  state.reg.pc = (input.pc + input.words) % address_words;
  state.inhibited = false;
  if ((state.reg.sr & ce) == 0) {
    disabled(state, input.pc);
    state.boundary = detail::NativeBoundary::owner;
    return false;
  }
  static_cast<void>(address(state, form, input));
  return state.fault.kind == FaultKind::none;
}

void finish(State &state, const a64::Instruction input, const bool repeat) {
  if (repeat) {
    state.repeat.opcode = input.opcode;
    state.repeat.loaded = true;
    --state.repeat.remaining;
    if (state.repeat.remaining != 0) {
      state.reg.lc = state.repeat.remaining;
      state.reg.pc = input.pc;
      loop_model::retire(state, input.cycles);
      return;
    }
    state.reg.lc = state.repeat.lc;
    state.repeat.loaded = false;
    state.completed = true;
  } else {
    state.completed = false;
  }
  loop_model::finish(state, {.pc = input.pc, .words = input.words});
  if (state.fault.kind == FaultKind::none) { loop_model::retire(state, input.cycles); }
}

void probe(const Form form, const State &initial, const unsigned width, const unsigned locks) {
  const auto input = instruction(form, initial);
  const a64::Program program(input);
  for (const Word budget : {0U, 1U, 8U}) {
    auto expected = initial;
    auto visible = initial;
    const bool accessed = budget != 0 && prepare(visible, form, input);
    loop_model::Runner runner(initial);
    a64::CacheStorage storage(runner, {.width = width, .locks = locks});
    storage.visible(visible);
    if (budget != 0) {
      if (accessed) { apply(storage.model(), form, address(expected, form, input)); }
      expected = visible;
      if (expected.fault.kind == FaultKind::none) {
        expected.boundary = detail::NativeBoundary::owner;
        finish(expected, input, false);
      }
    }
    const auto retired = budget != 0 && expected.fault.kind == FaultKind::none ? 1U : 0U;
    require(runner.run(program, budget) == retired,
            "Native cache controls preserve exact retirement, partial AGU effects and faults");
    runner.expect(expected);
    storage.expect(accessed ? 1U : 0U);
  }
}

void lock_masks(const unsigned width) {
  for (unsigned mask = 0; mask < 256; ++mask) {
    for (const auto kind : basics) { probe({.kind = kind}, seed(ce | status_byte), width, mask); }
    probe({.kind = Kind::lock_relative, .extra = width * 9}, seed(ce), width, mask);
    probe({.kind = Kind::unlock_relative, .extra = width * 9}, seed(ce), width, mask);
  }
}

void hits(const unsigned width) {
  for (const auto kind : {Kind::lock, Kind::unlock}) {
    for (Word tag = 0; tag < cache_model::slots; ++tag) {
      for (const unsigned locks : {0U, 0xaaU, status_byte}) {
        probe(
            {.kind = kind, .mode = 6, .extra = (tag * width) + width - 1}, seed(ce), width, locks);
      }
    }
  }
}

void sequences(const unsigned width) {
  const std::array forms{Form{.kind = Kind::flush},
                         Form{.kind = Kind::lock, .mode = 6, .extra = word_mask},
                         Form{.kind = Kind::lock_relative, .extra = width * 9},
                         Form{.kind = Kind::lock, .mode = 6, .extra = word_mask},
                         Form{.kind = Kind::flush_unlocked},
                         Form{.kind = Kind::free},
                         Form{.kind = Kind::lock, .mode = 6},
                         Form{.kind = Kind::unlock, .mode = 6, .extra = word_mask},
                         Form{.kind = Kind::flush}};
  auto expected = seed(ce | status_byte);
  loop_model::Runner runner(expected);
  a64::CacheStorage storage(runner, {.width = width});
  unsigned calls = 0;
  for (const auto form : forms) {
    const auto input = instruction(form, expected);
    const a64::Program program(input);
    auto original = expected;
    require(prepare(expected, form, input), "Known cache sequence has a valid address");
    storage.visible(expected);
    apply(storage.model(), form, address(original, form, input));
    expected.boundary = detail::NativeBoundary::owner;
    finish(expected, input, false);
    require(runner.run(program, 8) == 1,
            "Mixed cache controls preserve one shared storage object across owner exits");
    runner.expect(expected);
    storage.expect(++calls);
  }
}

void addressing(const unsigned width) {
  for (const auto kind : {Kind::lock, Kind::unlock}) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      for (unsigned index = 0; index < agu_registers; ++index) {
        for (const Word modifier : {0U, 3U, 0x8003U, 0x8000U, 0xc001U, address_model::linear}) {
          auto initial = seed(ce | unqualified);
          initial.reg.m.at(index) = modifier;
          probe({.kind = kind, .mode = mode, .index = index, .extra = word_mask},
                initial,
                width,
                status_byte);
        }
      }
    }
  }
}

void relative(const unsigned width) {
  for (const auto kind : {Kind::lock_relative, Kind::unlock_relative}) {
    for (const Word pc : {0U, start, word_mask}) {
      for (const Word displacement : {0U, 1U, word_mask, Word{1} << 23}) {
        auto initial = seed(ce | status_byte);
        initial.reg.pc = pc;
        probe({.kind = kind, .extra = displacement}, initial, width, 0);
      }
    }
  }
}

void cache_disabled() {
  for (const auto phase : {detail::VectorPhase::none,
                           detail::VectorPhase::pending,
                           detail::VectorPhase::first,
                           detail::VectorPhase::second}) {
    for (const auto kind :
         {Kind::flush, Kind::flush_unlocked, Kind::free, Kind::lock, Kind::unlock}) {
      for (unsigned mode = 0; mode < 8; ++mode) {
        auto initial = seed(unqualified);
        initial.vector = {.resume = start,
                          .address = bank,
                          .level = 1,
                          .phase = phase,
                          .words = {old_opcode, old_opcode},
                          .loaded = true};
        initial.reg.m.fill(invalid_modifier);
        probe({.kind = kind, .mode = mode, .index = 4, .extra = word_mask}, initial, widths[0], 0);
      }
    }
  }
}

void endpoints() {
  for (const auto kind : basics) {
    for (const Word mode : {0U, cm, sm}) {
      for (const Word stack : {0U, 1U, 2U}) {
        auto initial = seed(ce | mode | loop_model::lf);
        initial.reg.la = start;
        initial.reg.lc = 1;
        initial.reg.sc = stack;
        probe({.kind = kind}, initial, widths[0], status_byte);
      }
    }
  }
}

void repeats() {
  for (const auto kind :
       {Kind::flush, Kind::flush_unlocked, Kind::free, Kind::lock, Kind::unlock}) {
    for (const bool enabled : {false, true}) {
      auto expected = seed(enabled ? ce : 0U);
      expected.repeat = {.remaining = 3, .pc = start, .lc = saved_lc, .opcode = old_opcode};
      const Form form{.kind = kind, .mode = 3};
      const auto input = instruction(form, expected);
      const a64::Program program(input, a64::Emission::repeat);
      loop_model::Runner runner(expected);
      a64::CacheStorage storage(runner, {.width = widths[1], .locks = status_byte});
      unsigned calls = 0;
      for (const Word budget : {0U, 1U, 8U, 1U}) {
        expected.boundary = detail::NativeBoundary::none;
        const bool active = budget != 0 && expected.repeat.remaining != 0;
        if (active) {
          auto original = expected;
          if (prepare(expected, form, input)) {
            storage.visible(expected);
            apply(storage.model(), form, address(original, form, input));
            ++calls;
          }
          expected.boundary = detail::NativeBoundary::owner;
          finish(expected, input, true);
        }
        require(runner.run(program, budget) == (active ? 1U : 0U),
                "Retained REP cache controls stop at each owner boundary and keep pending ILLEGAL");
        runner.expect(expected);
        storage.expect(calls);
      }
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (const auto width : widths) {
      lock_masks(width);
      hits(width);
      sequences(width);
      addressing(width);
      relative(width);
    }
    cache_disabled();
    endpoints();
    repeats();
    std::cout << checks << " independent native A64 cache-control checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
