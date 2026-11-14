// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using enum isa::Opcode;
constexpr Word entry = 8, setup = entry - 2;
constexpr Word lf = 1U << 15, forever = 1U << 16;
constexpr Word host_bits = 0xab000000, previous_la = 0x123, previous_lc = 0x456;
constexpr Word pair = static_cast<Word>(move_xy), byte = 0xff;
constexpr Word first_ring = 0x8003, second_ring = 0x8007, irq_vector = 2, ring_turns = 9;
constexpr Word load = pair | (1U << 15) | (1U << 22);
constexpr Word store = pair | (2U << 18) | (3U << 16) | (3U << 11) | (3U << 20);
constexpr std::array body{load | (static_cast<Word>(mpy_register_a) & byte) | (5U << 4),
                          static_cast<Word>(move_register_x0_b_add_b_a),
                          static_cast<Word>(asr_b),
                          store | (static_cast<Word>(macr_register_b) & byte) | (5U << 4),
                          static_cast<Word>(rnd_a)};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::size_t extent = 128;
using Data = std::array<Word, extent>;

struct Input {
  Word mode = 0, count = 3, flags = 0;
  bool ring = false;
};

struct Banks {
  Data &x, &y;
};

struct Fixture {
  Fixture() {
    p[irq_vector] = p[irq_vector + 1] = static_cast<Word>(nop);
    p[setup] = static_cast<Word>(do_immediate) | (3U << 8);
    p[setup + 1] = entry + body.size() - 1;
    for (std::size_t i = 0; i < body.size(); ++i) { p[entry + i] = body[i]; }
    p[entry + body.size()] = isa::rts.front();
  }

  void reset() {
    constexpr std::array values{0U, 1U, sign24, word_mask, sign24 - 1, 0x600000U, 0x200000U};
    for (std::size_t i = 0; i < extent; ++i) {
      x[i] = values[i % values.size()] | host_bits;
      y[i] = values[(i + 3) % values.size()] | host_bits;
    }
  }

  static Registers state(const Input input) {
    const auto [mode, count, flags, ring] = input;
    auto result = register_model::seed(mode);
    result.pc = entry;
    result.m.fill(word_mask);
    result.sr = mode | lf | flags;
    result.la = entry + body.size() - 1;
    result.lc = count;
    result.stack[2] = std::uint64_t{entry} << word_bits;
    result.stack[1] = (std::uint64_t{previous_la} << word_bits) | previous_lc;
    result.r[0] = 4;
    result.r[4] = 5;
    if (ring) {
      result.m[0] = first_ring;
      result.m[4] = second_ring;
    }
    return result;
  }

  std::array<Word, extent> p{};
  Data x{}, y{};
};

void advance(Word &cursor, const Word modifier) {
  if (modifier == word_mask) {
    ++cursor;
    return;
  }
  const auto length = (modifier & 0x3fff) + 1;
  const auto origin = (cursor / length) * length;
  cursor = origin + ((cursor + 1) % length);
}

void step(Registers &current, const Banks data) {
  const auto &[x, y] = data;
  const auto old = current;
  const auto offset = current.pc - entry;
  if (offset == 0) {
    register_model::write(current, 4, x[current.r[0]] & word_mask);
    register_model::write(current, 6, y[current.r[4]] & word_mask);
  } else if (offset == 1) {
    register_model::write(current, 15, static_cast<Word>(old.x) & word_mask);
  } else if (offset == 3) {
    const auto left = register_model::bus_word(current, old.a);
    const auto right = register_model::bus_word(current, old.b);
    x[current.r[0]] = left;
    y[current.r[4]] = right;
    advance(current.r[0], current.m[0]);
    advance(current.r[4], current.m[4]);
  }
  sequence_model::alu(current, old, body[offset]);
  ++current.pc;
  if (offset != body.size() - 1) { return; }
  if ((current.sr & forever) != 0 || current.lc != 1) {
    current.lc = (current.lc - 1) & word_mask;
    current.pc = entry;
  } else {
    loop_frames::complete(current);
  }
}

void check(Core &core, Fixture &fixture, const Input input, const bool alias, const Word prefix) {
  fixture.reset();
  auto expected_x = fixture.x;
  auto expected_y = fixture.y;
  auto expected = Fixture::state(input);
  core.set_registers(expected);
  for (Word i = 0; i < prefix; ++i) {
    step(expected, {.x = expected_x, .y = alias ? expected_x : expected_y});
  }
  const auto progress = core.progress();
  const auto result =
      prefix % 2 == 0 ? core.run(prefix) : core.run_until(progress.estimated_cycles + prefix);
  const auto actual = core.registers();
  if (actual != expected || fixture.x != expected_x || fixture.y != expected_y) {
    std::cerr << "mode/count/flags/prefix/alias=" << input.mode << '/' << input.count << '/'
              << input.flags << '/' << prefix << '/' << alias << " A/B/SR/PC expected=" << std::hex
              << expected.a << '/' << expected.b << '/' << expected.sr << '/' << expected.pc
              << " actual=" << actual.a << '/' << actual.b << '/' << actual.sr << '/' << actual.pc
              << '\n';
  }
  require(result.fault == Fault{} && result.retired == prefix && actual == expected &&
              fixture.x == expected_x && fixture.y == expected_y &&
              result.progress == Progress{.instructions = progress.instructions + prefix,
                                          .estimated_cycles = progress.estimated_cycles + prefix},
          "LLVM loops and exact prefixes preserve arithmetic, old operands, clipping and "
          "aliases");
}

void prefixes(const Word mode, const bool alias) {
  Fixture fixture;
  Core core(Memory::separate({.program = fixture.p,
                              .x = fixture.x,
                              .y = alias ? std::span(fixture.x) : std::span(fixture.y)}));
  auto suffix = Fixture::state({.mode = mode});
  ++suffix.pc;
  core.set_registers(suffix);
  const auto suffix_progress = core.progress();
  constexpr std::array suffix_entries{entry + 1};
  const auto refused = core.prepare(suffix_entries);
  require(refused.failure == Fault{} && refused.prepared_entries == 0 &&
              refused.native_bytes == 0 && core.registers() == suffix &&
              core.progress() == suffix_progress,
          "A known suffix is not a whole-loop optimization entry and emits no unusable LLVM body");
  core.set_registers(Fixture::state({.mode = mode}));
  const auto before = core.state();
  constexpr std::array entries{entry};
  const auto prepared = core.prepare(entries);
  const auto repeated = core.prepare(entries);
  require(prepared.failure == Fault{} && prepared.prepared_entries == 1 &&
              prepared.native_bytes > 0 && repeated.prepared_entries == 1 &&
              repeated.native_bytes == 0 && core.registers() == before.registers &&
              core.progress() == before.progress,
          "Prepare arithmetic and parallel RAM as one optimized loop without guest execution");
  for (const Word flags : {0U, byte, forever, forever | byte}) {
    for (const Word count : {0U, 1U, 3U}) {
      const auto total = (count == 0 || (flags & forever) != 0 ? 3U : count) * body.size();
      for (Word prefix = 0; prefix <= total; ++prefix) {
        check(core, fixture, {.mode = mode, .count = count, .flags = flags}, alias, prefix);
      }
    }
  }
}

void ring_interrupts(const NativeImage &image,
                     Fixture &fixture,
                     const Input input,
                     const bool alias) {
  constexpr Word slots = 2;
  constexpr Word limit = slots + (ring_turns * body.size());
  for (Word prefix = 0; prefix <= limit; ++prefix) {
    fixture.reset();
    auto expected_x = fixture.x;
    auto expected_y = fixture.y;
    auto expected = Fixture::state(input);
    Core core(Memory::separate({.program = fixture.p,
                                .x = fixture.x,
                                .y = alias ? std::span(fixture.x) : std::span(fixture.y)}));
    core.set_registers(expected);
    core.install(image, 0);
    require(core.interrupt_ready(), "The ring context admits its prepared fast interrupt");
    core.request_interrupt(irq_vector, 3);
    for (Word step_index = slots; step_index < prefix; ++step_index) {
      step(expected, {.x = expected_x, .y = alias ? expected_x : expected_y});
    }
    const auto result = prefix % 2 == 0 ? core.run(prefix) : core.run_until(prefix);
    require(result.retired == prefix && result.fault == Fault{} &&
                result.progress == Progress{.instructions = prefix, .estimated_cycles = prefix} &&
                core.registers() == expected && fixture.x == expected_x && fixture.y == expected_y,
            "A captured fast IRQ preserves every resumed arithmetic ring prefix and DO frame");
  }
}

void ring_prefixes(const Word mode, const bool alias) {
  Fixture fixture;
  const std::array spans{
      CodeSpan{.address = irq_vector, .words = std::span(fixture.p).subspan(irq_vector, 2)},
      CodeSpan{.address = setup, .words = std::span(fixture.p).subspan(setup, body.size() + 3)}};
  constexpr std::array entries{entry};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array memory{RegionLayout{.space = Space::program, .first = 0, .size = extent},
                          RegionLayout{.space = Space::x, .first = 0, .size = extent},
                          RegionLayout{.space = Space::y, .first = 0, .size = extent}};
  const std::array layouts{VariantLayout{.id = 0, .memory = memory}};
  constexpr std::array interrupts{InterruptRequest{.vector = irq_vector, .level = 3}};
  const auto image = Core::compile({.variants = variants},
                                   {.memory = layouts,
                                    .interrupts = interrupts,
                                    .transitions = {},
                                    .callbacks = {},
                                    .bindings = {}});
  Core core(Memory::separate({.program = fixture.p,
                              .x = fixture.x,
                              .y = alias ? std::span(fixture.x) : std::span(fixture.y)}));
  core.set_registers(Fixture::state({.mode = mode, .ring = true}));
  core.install(image, 0);
  for (const Word flags : {0U, byte, forever, forever | byte}) {
    for (const Word count : {0U, 1U, ring_turns}) {
      const auto total = (count == 0 || (flags & forever) != 0 ? ring_turns : count) * body.size();
      for (Word prefix = 0; prefix <= total; ++prefix) {
        check(core,
              fixture,
              {.mode = mode, .count = count, .flags = flags, .ring = true},
              alias,
              prefix);
      }
    }
  }
  for (const Word flags : {0U, byte}) {
    ring_interrupts(
        image, fixture, {.mode = mode, .count = ring_turns, .flags = flags, .ring = true}, alias);
  }
}
}

int main() try {
  for (const Word mode : modes) {
    prefixes(mode, false);
    prefixes(mode, true);
    ring_prefixes(mode, false);
    ring_prefixes(mode, true);
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
