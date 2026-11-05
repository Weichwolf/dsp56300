// SPDX-License-Identifier: GPL-3.0-only
// Read-only production RX proof against independently declared forward AGU traces.
#include "address_model.hpp"
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "native/direct.hpp"
#include "native_region.hpp"
#include "region_guards.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "x64.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word x_words = 128, y_words = 96, io_begin = 0xffff80;
constexpr Word constant_r0 = 16, assign_r0 = 0x60f400, paired_load = 0xf09800;
constexpr unsigned body_transfers = 8;
constexpr auto maximum = std::numeric_limits<Word>::max();
constexpr std::array<std::uint8_t, 12> entry{
    0xf3, 0x0f, 0x1e, 0xfa, 0x41, 0x54, 0x41, 0x55, 0x48, 0x83, 0xec, 0x08};
constexpr std::array<std::uint8_t, 9> leave{0x48, 0x83, 0xc4, 0x08, 0x41, 0x5d, 0x41, 0x5c, 0xc3};
constexpr std::array<Word, 17> cursors{
    0, 1, 15, 16, 31, 32, 63, 64, 95, 96, 120, 121, 127, 128, io_begin, word_mask, maximum};
constexpr std::array<Word, 11> strides{
    0, 1, 2, 3, 7, 15, sign24 - 1, sign24, word_mask, word_mask + 1, maximum};
constexpr std::array<Word, 7> counts{0, 1, 2, 3, 17, word_mask, maximum};

enum class Pattern : std::uint8_t {
  unit,
  stride,
  mixed,
  held,
  reset,
  reset_tail,
  tail,
  paired,
  shared
};

struct Profile {
  Pattern pattern = Pattern::unit;
  Word modifier = word_mask;
  unsigned cursor = 0;
};

struct Port {
  unsigned index = 0, mode = 3;
  bool y = false;
};

struct Step {
  Word code = 0, extension = 0;
  std::array<Port, 2> ports{};
  unsigned count = 0;
  bool reset = false;
};

unsigned access_mode(const Pattern pattern, const unsigned index) {
  if (pattern == Pattern::stride || pattern == Pattern::reset) { return 1; }
  if (pattern == Pattern::mixed) { return index % 2 == 0 ? 1U : 3U; }
  if (pattern == Pattern::held) { return 4; }
  if (pattern == Pattern::tail || pattern == Pattern::reset_tail) {
    return index + 1 == body_transfers ? 1U : 4U;
  }
  return 3;
}

std::vector<Step> body(const Profile profile) {
  const auto pattern = profile.pattern;
  std::vector<Step> result;
  if (pattern == Pattern::reset || pattern == Pattern::reset_tail) {
    result.push_back({.code = assign_r0 | (profile.cursor << 16),
                      .extension = constant_r0,
                      .ports = {Port{.index = profile.cursor}, {}},
                      .reset = true});
  }
  for (unsigned index = 0; index < body_transfers; ++index) {
    if (pattern == Pattern::paired) {
      if (index % 2 == 0) {
        result.push_back({.code = paired_load,
                          .ports = {Port{}, Port{.index = 4, .mode = 3, .y = true}},
                          .count = 2});
      }
      continue;
    }
    const auto mode = access_mode(pattern, index);
    constexpr Word load_x0 = 0x44c000;
    constexpr Word y_bus = 0x080000;
    const bool y = pattern == Pattern::shared && index % 2 != 0;
    result.push_back({.code = load_x0 | (profile.cursor << 8) | (mode << 11) | (y ? y_bus : 0),
                      .ports = {Port{.index = profile.cursor, .mode = mode, .y = y}, {}},
                      .count = 1});
  }
  return result;
}

std::vector<detail::NativeOp> operations(const std::span<const Step> steps) {
  std::vector<detail::NativeOp> result;
  Word pc = start;
  for (const auto &step : steps) {
    auto decoded = detail::decode(step.code);
    decoded.extension = step.extension;
    require(decoded.operation == detail::Operation::parallel &&
                decoded.words == (step.reset ? 2 : 1),
            "Window traces use independently declared instruction lengths and addresses");
    result.push_back({.instruction = decoded, .pc = pc});
    pc += decoded.words;
  }
  for (auto &op : result) { op.loop_address = pc - 1; }
  return result;
}

std::vector<std::uint8_t> proof(const detail::NativeRegion &region, const Word instructions) {
  using detail::x64::Reg;
  using detail::x64::Width;
  detail::x64::Code code;
  code.raw(entry);
  code.mov({.destination = Reg::r12, .source = Reg::di});
  code.mov({.destination = Reg::r13, .source = Reg::si});
  detail::x64::Label rejected;
  detail::x64::Label done;
  detail::emit_region_window(code, region, instructions, rejected);
  code.load(Reg::ax, Reg::sp, detail::window_count_offset, Width::dword);
  code.jump(done);
  code.bind(rejected);
  code.constant(Reg::ax, 0);
  code.bind(done);
  code.raw(leave);
  return code.bytes();
}

bool safe(const std::span<const Step> steps, const Registers &initial, const Word iterations) {
  auto state = initial;
  std::vector<std::array<Word, agu_registers>> seen;
  for (Word iteration = 0; iteration < iterations; ++iteration) {
    if (std::ranges::find(seen, state.r) != seen.end()) { return true; }
    seen.push_back(state.r);
    for (const auto &step : steps) {
      if (step.reset) { state.r[step.ports[0].index] = constant_r0; }
      const auto old = state.r;
      for (const auto &port : std::span(step.ports).first(step.count)) {
        if (old[port.index] >= (port.y ? y_words : x_words)) { return false; }
        state.r[port.index] = address_model::updated({.modifier = state.m[port.index],
                                                      .cursor = old[port.index],
                                                      .offset = state.n[port.index],
                                                      .mode = port.mode});
      }
    }
  }
  return true;
}

struct Input {
  Word r = 0, n = 0, lc = 3, budget = 0;
  bool mismatch = false;
};

bool invoke(const native::Direct &code,
            const std::span<const Step> steps,
            const Profile profile,
            const Input input) {
  detail::Cpu cpu;
  cpu.reg = register_model::seed(0);
  cpu.reg.r.fill(io_begin);
  cpu.reg.n.fill(sign24);
  cpu.reg.m.fill(0);
  cpu.reg.r[profile.cursor] = input.r;
  cpu.reg.n[profile.cursor] = input.n;
  constexpr Word upper_alias = 0xab000000;
  cpu.reg.m[profile.cursor] = (profile.modifier ^ (input.mismatch ? 1U : 0U)) | upper_alias;
  if (profile.pattern == Pattern::paired) {
    cpu.reg.r[4] = static_cast<Word>((std::uint64_t{input.r} + 3) & word_mask);
    cpu.reg.n[4] = input.n;
    cpu.reg.m[4] = profile.modifier | upper_alias;
  }
  cpu.reg.lc = input.lc;
  cpu.instructions = std::numeric_limits<std::uint64_t>::max();
  cpu.cycles = cpu.instructions - 1;
  const auto old = cpu.reg;
  const auto cpu_bytes = std::as_bytes(std::span(&cpu, 1));
  const std::vector before_cpu(cpu_bytes.begin(), cpu_bytes.end());
  detail::NativeContext context{.runtime = {}, .budget = input.budget};
  const auto context_bytes = std::as_bytes(std::span(&context, 1));
  const std::vector before_context(context_bytes.begin(), context_bytes.end());
  const auto count = code.enter(cpu, context);
  require(std::ranges::equal(before_cpu, cpu_bytes) &&
              std::ranges::equal(before_context, context_bytes),
          "Every admitted/rejected window proof leaves all guest/context bytes unchanged");
  if (count == 0) { return false; }
  const auto expected =
      input.lc < 2 ? 0U : std::min(input.lc - 1, input.budget / static_cast<Word>(steps.size()));
  require(count >= 2 && count == expected,
          "RX division and LC cap select the exact whole-body count");
  require(safe(steps, old, count),
          "Every admitted window contains all independent forward AGU accesses");
  return true;
}

void matrix(const Profile profile) {
  const auto steps = body(profile);
  const auto ops = operations(steps);
  std::array<Word, x_words> x{};
  std::array<Word, y_words> y{};
  auto modifiers = register_model::seed(0).m;
  modifiers[profile.cursor] = profile.modifier;
  if (profile.pattern == Pattern::paired) { modifiers[4] = profile.modifier; }
  const std::array<detail::JitMemory, 2> memory{
      {{.data = x.data(), .words = x.size()}, {.data = y.data(), .words = y.size()}}};
  const auto region = detail::plan_native_region(ops, memory, modifiers);
  if (!region.valid) {
    require(profile.pattern == Pattern::paired && profile.modifier != word_mask &&
                profile.modifier >= y_words,
            "A complete modulo ring larger than the smaller port retains the existing fallback");
    return;
  }
  require(region.loop, "Declared guarded traces meet the existing independent admission");
  const auto size = static_cast<Word>(ops.size());
  const native::Direct code(proof(region, size));
  const std::array budgets{0U, size, (2 * size) - 1, 2 * size, 7 * size, word_mask, maximum};
  unsigned accepted = 0;
  unsigned rejected = 0;
  for (const auto r : cursors) {
    for (const auto n : strides) {
      for (const auto lc : counts) {
        for (const auto budget : budgets) {
          for (const bool mismatch : {false, true}) {
            const bool admitted =
                invoke(code,
                       steps,
                       profile,
                       {.r = r, .n = n, .lc = lc, .budget = budget, .mismatch = mismatch});
            admitted ? ++accepted : ++rejected;
          }
        }
      }
    }
  }
  require(accepted != 0 && rejected != 0,
          "Each trace proves both concrete positive and negative windows");
}

void rejected_ir() {
  for (const Word size : {0U, 257U}) {
    detail::NativeRegion region;
    region.valid = region.loop = true;
    detail::x64::Code code;
    detail::x64::Label failed;
    bool rejected = false;
    try {
      detail::emit_region_window(code, region, size, failed);
    } catch (const std::logic_error &) { rejected = true; }
    require(rejected && code.bytes().empty(),
            "Empty/oversized IR cannot emit an unqualified division or window proof");
  }
}

void encoding() {
  detail::x64::Code code;
  code.divide(detail::x64::Reg::r11);
  constexpr std::array<std::uint8_t, 3> expected{0x41, 0xf7, 0xf3};
  require(std::ranges::equal(code.bytes(), expected),
          "Unsigned dword DIV uses the qualified R11 encoding");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using dsp56300::test::Pattern;
    dsp56300::test::encoding();
    dsp56300::test::rejected_ir();
    for (const auto pattern : {Pattern::unit,
                               Pattern::stride,
                               Pattern::mixed,
                               Pattern::held,
                               Pattern::reset,
                               Pattern::reset_tail,
                               Pattern::tail,
                               Pattern::paired}) {
      dsp56300::test::matrix({.pattern = pattern});
    }
    for (const auto modifier : {1U, 2U, 3U, 5U, 7U, 15U, 31U, 63U, 127U}) {
      dsp56300::test::matrix({.pattern = Pattern::unit, .modifier = modifier});
      dsp56300::test::matrix({.pattern = Pattern::paired, .modifier = modifier});
    }
    dsp56300::test::matrix({.pattern = Pattern::shared});
    for (unsigned cursor = 1; cursor < dsp56300::agu_registers; ++cursor) {
      dsp56300::test::matrix({.pattern = Pattern::unit, .cursor = cursor});
      dsp56300::test::matrix({.pattern = Pattern::stride, .cursor = cursor});
    }
    std::cout << dsp56300::test::checks << " independent loop-window RX checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
