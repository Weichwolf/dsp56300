// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "move_model.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <vector>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
constexpr Word first = 16, base = 0x4000, turns = 3, loop = 1U << 15;
constexpr std::size_t words = 64, form_count = 128;
constexpr Word linear = 0xffff, wrapped = 0x8007, previous_la = 300, linear_cursor = 24;
constexpr Word immediate_word = 0xfedcba;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
using Data = std::array<Word, words>;

struct Input {
  move_model::Form form;
  Word alu = 0;
};

struct Program {
  explicit Program(const Input input) : input(input) {
    code = {static_cast<Word>(isa::Opcode::do_immediate) | (turns << 8),
            first + move_model::words(input.form),
            move_model::opcode(input.form, input.alu)};
    if (move_model::words(input.form) == 2) { code.push_back(input.form.extension); }
    code.push_back(static_cast<Word>(isa::Opcode::nop));
    code.push_back(static_cast<Word>(isa::Opcode::nop));
    span = {.address = first - 2, .words = code};
  }

  Input input;
  std::vector<Word> code;
  CodeSpan span;
};

struct Port final : Io {
  Port() {
    windows = {MemoryWindow{.address = base, .words = x, .known = known, .writable = true},
               MemoryWindow{.address = base, .words = y, .known = known, .writable = true}};
    known.fill(1);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word pc) const noexcept override {
    return pc >= first - 2 && pc - (first - 2) < code.size() ? std::optional{code[pc - (first - 2)]}
                                                             : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return space == Space::program ? std::span<const MemoryWindow>{}
                                   : std::span(&windows[space == Space::x ? 0U : 1U], 1);
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    ++accesses;
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    ++accesses;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    ++accesses;
    return false;
  }

  Data x{}, y{};
  std::array<std::uint8_t, words> known{};
  std::array<MemoryWindow, 2> windows{};
  std::span<const Word> code;
  unsigned accesses = 0;
};

struct Point {
  Word mode, modifier;
  std::uint64_t accumulator;
  Word quota;
  bool deadline;
};

void execute(const NativeImage &image,
             const Program &program,
             const std::uint64_t id,
             const Point point) {
  Port port;
  port.code = program.code;
  Core core(Memory::external(), &port);
  const auto [mode, modifier, accumulator, quota, deadline] = point;
  for (Word i = 0; i < words; ++i) {
    port.x[i] = (i * (sign24 / 8 - 3)) ^ (sign24 + 1);
    port.y[i] = (i * (sign24 / 16 - 1)) ^ (sign24 - 1);
  }
  auto expected = register_model::seed(mode);
  expected.pc = first;
  expected.sr = mode | loop;
  expected.a = accumulator;
  expected.b = accumulator_mask ^ accumulator;
  expected.la = first + move_model::words(program.input.form);
  expected.lc = turns;
  expected.sp = expected.sc = 2;
  expected.stack[1] = (std::uint64_t{previous_la} << word_bits) | 7U;
  expected.stack[2] = std::uint64_t{first} << word_bits;
  expected.r[0] = base + (modifier == linear ? linear_cursor : 7U);
  expected.n[0] = 3;
  expected.m[0] = modifier;
  core.reset();
  core.set_registers(expected);
  core.install(image, id);
  auto x = port.x;
  auto y = port.y;
  std::uint64_t cycles = 0;
  for (Word step = 0; step < quota; ++step) {
    if (expected.pc == first) {
      const auto old = expected;
      const auto form = program.input.form;
      auto probe = expected;
      const auto address = move_model::address(probe, form);
      auto &data = form.y ? y : x;
      const bool immediate = form.mode == 6 && form.index == 4;
      const auto value = immediate ? form.extension : data.at(address - base) & word_mask;
      const auto output = move_model::apply(expected, form, {.high = value});
      if (output.stores != 0) { data.at(output.address - base) = output.first; }
      sequence_model::alu(expected, old, program.input.alu);
      expected.pc += move_model::words(form);
      cycles += move_model::cycles(form);
    } else {
      ++expected.pc;
      ++cycles;
      if (expected.lc > 1) {
        --expected.lc;
        expected.pc = first;
      } else {
        loop_frames::complete(expected);
      }
    }
  }
  const auto run = deadline ? core.run_until(cycles) : core.run(quota);
  if (run.retired != quota || run.fault != Fault{} || run.yielded ||
      run.progress != Progress{.instructions = quota, .estimated_cycles = cycles} ||
      core.registers() != expected || port.x != x || port.y != y || port.accesses != 0) {
    const auto actual = core.registers();
    std::cerr << "id=" << id << " op=" << move_model::opcode(program.input.form, program.input.alu)
              << " mode=" << mode << " modifier=" << modifier << " acc=" << accumulator
              << " quota=" << quota << " deadline=" << deadline << " retired=" << run.retired
              << " cycles=" << run.progress.estimated_cycles << "/" << cycles
              << " fault=" << static_cast<unsigned>(run.fault.kind)
              << " callbacks=" << port.accesses << " ram=" << (port.x == x && port.y == y)
              << " pc=" << actual.pc << "/" << expected.pc << " sr=" << actual.sr << "/"
              << expected.sr << " x=" << actual.x << "/" << expected.x << " y=" << actual.y << "/"
              << expected.y << " a=" << actual.a << "/" << expected.a << " b=" << actual.b << "/"
              << expected.b << " r0=" << actual.r[0] << "/" << expected.r[0] << " lc=" << actual.lc
              << "/" << expected.lc << '\n';
  }
  require(run.retired == quota && run.fault == Fault{} && !run.yielded &&
              run.progress == Progress{.instructions = quota, .estimated_cycles = cycles} &&
              core.registers() == expected && port.x == x && port.y == y && port.accesses == 0,
          "Mixed regions preserve parallel old inputs, flags, RAM, AGU and exact DO "
          "boundaries");
}

void qualify(const NativeImage &image, const Program &program, const std::uint64_t id) {
  constexpr std::array accumulators{
      std::uint64_t{0}, std::uint64_t{1} << 47, sign56 - 1, sign56, accumulator_mask};
  for (const auto mode : modes) {
    for (const auto modifier : {linear, wrapped}) {
      for (const auto accumulator : accumulators) {
        for (Word quota = 1; quota <= 2 * turns; ++quota) {
          for (const bool deadline : {false, true}) {
            execute(image,
                    program,
                    id,
                    {.mode = mode,
                     .modifier = modifier,
                     .accumulator = accumulator,
                     .quota = quota,
                     .deadline = deadline});
          }
        }
      }
    }
  }
}

Input input(const unsigned ordinal) {
  const bool y = (ordinal & 1U) != 0;
  const auto alu =
      static_cast<Word>(y ? isa::Opcode::add_x0_a : isa::Opcode::add_y0_a) & status_byte;
  return {.form = {.kind = move_model::Kind::mixed_one,
                   .selector = (ordinal >> 2) & 3U,
                   .mode = 3,
                   .load = (ordinal & 2U) != 0,
                   .y = y,
                   .source_b = (ordinal & 16U) != 0,
                   .second_high = (ordinal & 32U) != 0},
          .alu = (ordinal & 64U) != 0 ? alu : 0};
}
}

int main() {
  try {
    std::vector<Program> programs;
    programs.reserve(2 * form_count);
    for (unsigned ordinal = 0; ordinal < form_count; ++ordinal) {
      programs.emplace_back(input(ordinal));
    }
    for (const bool y : {false, true}) {
      for (const bool load : {false, true}) {
        for (const auto mode : {0U, 1U, 2U, 4U, 5U, 6U, 7U}) {
          programs.emplace_back(Input{.form = {.kind = move_model::Kind::mixed_one,
                                               .selector = 2,
                                               .mode = mode,
                                               .extension = base + 12,
                                               .load = load,
                                               .y = y,
                                               .source_b = true}});
        }
      }
      programs.emplace_back(Input{.form = {.kind = move_model::Kind::mixed_one,
                                           .selector = 2,
                                           .mode = 6,
                                           .index = 4,
                                           .extension = immediate_word,
                                           .y = y,
                                           .source_b = true}});
    }
    const std::array entries{first - 2, first};
    const std::array regions{
        RegionLayout{.space = Space::x, .first = base, .size = words, .tracked = true},
        RegionLayout{.space = Space::y, .first = base, .size = words, .tracked = true}};
    std::vector<CodeVariant> variants;
    std::vector<VariantLayout> layouts;
    for (std::size_t i = 0; i < programs.size(); ++i) {
      variants.push_back({.id = i,
                          .code = std::span(&programs[i].span, 1),
                          .entries = entries,
                          .failed_fetches = {}});
      layouts.push_back({.id = i, .memory = regions});
    }
    const auto image = Core::compile(
        {.variants = variants},
        {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
    for (std::size_t i = 0; i < programs.size(); ++i) { qualify(image, programs[i], i); }
    std::cout << programs.size() << " mixed images, " << checks << " checks\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
