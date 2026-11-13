// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/image_regions.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/runtime.hpp"
#include "loop_frames.hpp"
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
using test::require;
using test::word_bits;
constexpr Word first = 16, body = first + 2, length = 4, base = 0x4000;
constexpr Word loop = 1U << 15, target = 15, output = 24;
constexpr Word high_register_mask = 0x18, seed_word = 0xa5a500, old_la = 300;
constexpr Word linear = 0xffff, ring_modifier = 0x8003;
constexpr unsigned high_register_shift = 17, r1_register = 17;
constexpr std::uint64_t seed_x = 0x654321123456;
constexpr std::array modes{0U,
                           test::mode_rm,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm | test::mode_down,
                           test::mode_rm | test::mode_up};
using Data = std::array<Word, 32>;
using Knowledge = std::array<std::uint8_t, 32>;

Word op(const isa::Opcode opcode) {
  return static_cast<Word>(opcode);
}

Word move(const unsigned reg, const unsigned root, const unsigned mode, const bool load) {
  constexpr Word fields = (7U << 16) | (high_register_mask << high_register_shift);
  return (op(isa::Opcode::move_x_effective) & ~fields) | ((reg & 7U) << 16) |
         ((reg & high_register_mask) << high_register_shift) | (root << 8) | (mode << 11) |
         (load ? 1U << 15 : 0U);
}

struct Fixture final : Io {
  explicit Fixture(const bool late = false) {
    p.fill(op(isa::Opcode::nop));
    p[first] = op(isa::Opcode::do_immediate) | (1U << 8);
    p[first + 1] = body + length - 1;
    p[body] = move(r1_register, 0, 3, true);
    p[body + 1] = move(4, 2, 4, false);
    if (late) { std::swap(p[body], p[body + 1]); }
    p[body + 2] = move(5, 1, 3, true);
    p[body + 3] = move(5, 3, 3, false);
    p[body + length] = op(isa::Opcode::stop);
    reset();
    window = {.address = base, .words = data, .known = known, .writable = true};
  }

  void reset() {
    known.fill(1);
    for (Word index = 0; index < data.size(); ++index) { data[index] = index ^ seed_word; }
    data[0] = base + target;
    callbacks = 0;
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word at) const noexcept override {
    return at < p.size() ? std::optional{p[at]} : std::nullopt;
  }

  bool read(Space /*space*/, Word /*at*/, Word /*pc*/, Word & /*value*/) noexcept override {
    ++callbacks;
    return false;
  }

  bool write(Space /*space*/, Word /*at*/, Word /*pc*/, Word /*value*/) noexcept override {
    ++callbacks;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*at*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    ++callbacks;
    return false;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    return space == Space::x ? std::span(&window, 1) : std::span<const MemoryWindow>{};
  }

  [[nodiscard]] detail::JitRuntime runtime() {
    return {.memory = {}, .windows = {std::span(&window, 1), {}}};
  }

  std::array<Word, 32> p{};
  Data data{};
  Knowledge known{};
  MemoryWindow window;
  unsigned callbacks = 0;
};

Registers seed(const Word mode, const bool ring) {
  auto reg = test::initial_state();
  reg.pc = body;
  reg.sr = mode | loop | test::status_byte;
  reg.la = body + length - 1;
  reg.lc = 1;
  reg.sp = reg.sc = 2;
  reg.stack[1] = (std::uint64_t{old_la} << word_bits) | 7;
  reg.stack[2] = (std::uint64_t{body} << word_bits) | (reg.sr & ~loop);
  reg.m.fill(linear);
  reg.m[1] = ring ? ring_modifier : linear;
  reg.r[0] = base;
  reg.r[1] = base + 1;
  reg.r[2] = base + target;
  reg.r[3] = base + output;
  reg.x = seed_x;
  return reg;
}

NativeImage compile(Fixture &fixture) {
  const std::array spans{
      CodeSpan{.address = first, .words = std::span(fixture.p).subspan(first, length + 3)}};
  constexpr std::array entries{first, body};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array regions{
      RegionLayout{.space = Space::x, .first = base, .size = fixture.data.size(), .tracked = true}};
  const std::array layouts{VariantLayout{.id = 0, .memory = regions}};
  return Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
}

void model(Registers &reg,
           Data &data,
           Knowledge &known,
           const Word count,
           const bool ring,
           const bool late = false) {
  for (Word index = 0; index < count; ++index) {
    const auto step = late && index < 2 ? 1 - index : index;
    if (step == 0) { reg.r[1] = data[reg.r[0]++ - base]; }
    if (step == 1) {
      data[reg.r[2] - base] = static_cast<Word>(reg.x) & word_mask;
      known[reg.r[2] - base] = 1;
    }
    if (step == 2) {
      reg.x = (reg.x & word_mask) | (std::uint64_t{data[reg.r[1] - base]} << word_bits);
      reg.r[1] = ring ? (reg.r[1] & ~3U) | ((reg.r[1] + 1) & 3U) : reg.r[1] + 1;
    }
    if (step == 3) {
      data[reg.r[3] - base] = static_cast<Word>(reg.x >> word_bits);
      known[reg.r[3]++ - base] = 1;
      test::loop_frames::complete(reg);
    }
    ++reg.pc;
  }
}

unsigned fallback_calls = 0;

std::uint32_t fallback(detail::Cpu * /*cpu*/, detail::NativeContext * /*owner*/) noexcept {
  ++fallback_calls;
  return 0;
}

void direct(const Word mode, const bool ring) {
  Fixture fixture;
  const auto initial = seed(mode, ring);
  const auto runtime = fixture.runtime();
  const auto operations = detail::parse_block(fixture.p, initial, runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  const auto image =
      detail::compile_region(compiler, operations, runtime, std::bit_cast<const void *>(&fallback));
  if (!image) {
    throw std::runtime_error("Computed roots must stay inside one complete SSA region");
  }
  for (const unsigned scenario : {0U, 1U, 2U}) {
    fixture.reset();
    if (scenario == 1) { fixture.data[0] = base + fixture.data.size(); }
    if (scenario == 2) {
      fixture.data[0] = base + target - 1;
      fixture.known[target - 1] = 0;
    }
    auto wanted = initial;
    auto expected = fixture.data;
    auto known = fixture.known;
    if (scenario == 0) { model(wanted, expected, known, length, ring); }
    auto padded = initial;
    padded.a <<= detail::padding_bits;
    padded.b <<= detail::padding_bits;
    detail::Cpu cpu{.reg = padded};
    detail::NativeContext owner{.runtime = {}, .budget = length, .cycle_deadline = length};
    const auto calls = fallback_calls;
    const auto retired = detail::enter_block(image->entry, cpu, owner);
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(retired == (scenario == 0 ? length : 0) && cpu.reg == wanted &&
                fixture.data == expected && fixture.known == known &&
                fallback_calls == calls + (scenario == 0 ? 0U : 1U) &&
                cpu.instructions == retired && cpu.cycles == retired,
            "Actual computed bounds guard before stores; success retains alias ordering and ring "
            "origin");
  }
}

void prefixes(const NativeImage &image, const Word mode, const bool ring, const bool late) {
  for (const bool timed : {false, true}) {
    for (Word count = 0; count <= length; ++count) {
      Fixture fixture(late);
      const auto initial = seed(mode, ring);
      auto wanted = initial;
      auto expected = fixture.data;
      auto known = fixture.known;
      model(wanted, expected, known, count, ring, late);
      Core core(Memory::external(), &fixture);
      core.set_registers(initial);
      core.install(image, 0);
      const auto result = timed ? core.run_until(count) : core.run(count);
      require(result.fault == Fault{} && result.retired == count && core.registers() == wanted &&
                  fixture.data == expected && fixture.known == known && fixture.callbacks == 0 &&
                  result.progress == Progress{.instructions = count, .estimated_cycles = count},
              "Installed native image preserves all computed-root instruction/deadline prefixes");
    }
  }
}

void fault(const NativeImage &image, const bool unmapped, const bool after_install) {
  Fixture fixture;
  const Word at = base + (unmapped ? static_cast<Word>(fixture.data.size()) : target - 1);
  fixture.data[0] = at;
  Core core(Memory::external(), &fixture);
  const auto initial = seed(0, false);
  core.set_registers(initial);
  if (!after_install && !unmapped) { fixture.known[target - 1] = 0; }
  core.install(image, 0);
  if (after_install && !unmapped) { fixture.known[target - 1] = 0; }
  auto wanted = initial;
  auto expected = fixture.data;
  auto known = fixture.known;
  model(wanted, expected, known, 2, false);
  ++wanted.r[1];
  ++wanted.pc;
  const auto result = core.run(length);
  const Fault failure{
      .kind = FaultKind::memory_read, .space = Space::x, .address = at, .pc = body + 2};
  require(result.fault == failure && result.retired == 2 && core.registers() == wanted &&
              fixture.data == expected && fixture.known == known && fixture.callbacks == 1 &&
              result.progress == Progress{.instructions = 2, .estimated_cycles = 2},
          "Rejected computed lookup executes the preceding store once and preserves the exact "
          "fault prefix");
}

void late_definition() {
  Fixture fixture(true);
  const auto runtime = fixture.runtime();
  const auto operations = detail::parse_block(fixture.p, seed(0, false), runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  require(!detail::compile_region(
              compiler, operations, runtime, std::bit_cast<const void *>(&fallback)),
          "A computed guard after a visible store cannot select a replaying whole-region path");
}

void interior_entry() {
  Fixture fixture;
  fixture.p[body - 1] = move(4, 2, 4, false);
  const std::array spans{
      CodeSpan{.address = body - 1, .words = std::span(fixture.p).subspan(body - 1, length + 2)}};
  constexpr std::array entries{body - 1, body};
  const std::array memory{
      RegionLayout{.space = Space::x, .first = base, .size = fixture.data.size(), .tracked = true}};
  const auto regions = detail::image_regions(detail::plan_image(spans, entries), memory);
  require(std::ranges::any_of(regions,
                              [](const auto &region) {
                                return region.front().pc == body && region.size() == length;
                              }),
          "An established interior entry retains its complete SSA path after an earlier store");
}
}

int main() try {
  Fixture fixture;
  const auto image = compile(fixture);
  Fixture late(true);
  const auto precise = compile(late);
  for (const auto mode : modes) {
    for (const bool ring : {false, true}) {
      direct(mode, ring);
      prefixes(image, mode, ring, false);
      prefixes(precise, mode, ring, true);
    }
  }
  fault(image, false, false);
  fault(image, false, true);
  fault(image, true, false);
  late_definition();
  interior_entry();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
