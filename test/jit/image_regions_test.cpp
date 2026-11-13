// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/image_regions.hpp"
#include "jit/llvm/runtime.hpp"
#include "loop_frames.hpp"
#include "saturated_shift_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <llvm/IR/InstrTypes.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Casting.h>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word first = 16, body = first + 2, base = 0x4000, destination = 16, turns = 4;
constexpr Word loop = 1U << 15, old_la = 300, old_lc = 7;
constexpr std::array modes{0U,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm,
                           test::mode_rm | test::mode_down,
                           test::mode_rm | test::mode_up};
using Data = std::array<Word, 32>;

Word op(const isa::Opcode opcode) {
  return static_cast<Word>(opcode);
}

struct Fixture final : Io {
  explicit Fixture(const Word length) : code(length + 3, op(isa::Opcode::nop)), length(length) {
    code[0] = op(isa::Opcode::do_immediate) | (turns << 8);
    code[1] = body + length - 1;
    code[2] = op(isa::Opcode::move_x_effective) | (1U << 15) | (3U << 11);
    code[1 + length] = op(isa::Opcode::move_x_effective) | (3U << 11) | (1U << 8);
    code.back() = op(isa::Opcode::stop);
    known.fill(1);
    for (Word index = 0; index < data.size(); ++index) {
      data[index] = ((index * test::sign24) ^ index) & word_mask;
    }
    window = {.address = base, .words = data, .known = known, .writable = true};
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address >= first && address - first < code.size()
               ? std::optional<Word>{code[address - first]}
               : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    if (space != Space::x) { return {}; }
    return views.empty() ? std::span(&window, 1) : views;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

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

  std::vector<Word> code;
  Word length;
  Data data{};
  std::array<std::uint8_t, 32> known{};
  MemoryWindow window;
  std::span<const MemoryWindow> views;
  unsigned callbacks = 0;
  std::uint64_t revision = 0;
};

SemanticImage lower(Fixture &fixture, const bool ordinary = false) {
  const auto code = std::span(fixture.code);
  const std::array spans{
      CodeSpan{.address = ordinary ? body : first, .words = ordinary ? code.subspan(2) : code}};
  const std::array entries{ordinary ? body : first, body};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array regions{
      RegionLayout{.space = Space::x, .first = base, .size = fixture.data.size(), .tracked = true}};
  const std::array layouts{VariantLayout{.id = 0, .memory = regions}};
  return Core::lower(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
}

NativeImage compile(const SemanticImage &semantic, const bool expect_loops = true) {
  detail::LlvmModule module(std::string(semantic.exports().front().name));
  module.link_bitcode(semantic.bitcode());
  unsigned loops = 0;
  for (const auto &function : *llvm::unwrap(module.code())) {
    for (const auto &block : function) {
      for (const auto &instruction : block) {
        if (const auto *call = llvm::dyn_cast<llvm::CallBase>(&instruction)) {
          const auto *callee = call->getCalledFunction();
          require(callee == nullptr || (callee->getName() != "dsp56300_region_memory" &&
                                        callee->getName() != "dsp56300_semantic_numeric_mode"),
                  "Fixed-image IR inlines storage and numeric-mode bindings before native "
                  "optimization");
        }
        if (llvm::isa<llvm::PHINode>(instruction) &&
            instruction.getName().starts_with("remaining")) {
          ++loops;
        }
      }
    }
  }
  if (expect_loops) {
    require(loops >= modes.size(),
            "Every supported numeric mode has an SSA loop before target optimization");
  }
  auto compiler = std::make_shared<detail::LlvmCompiler>();
  const auto *entry = compiler->install(module);
  const std::array symbols{NativeSymbol{.name = semantic.exports().front().name, .address = entry}};
  return Core::bind(semantic,
                    {.owner = compiler,
                     .symbols = symbols,
                     .identity = semantic.identity(),
                     .native_bytes = compiler->native_bytes()});
}

Registers initial(const Fixture &fixture, const Word mode) {
  auto state = test::initial_state();
  state.pc = body;
  state.sr = mode | loop | test::status_byte;
  state.la = body + fixture.length - 1;
  state.lc = turns;
  state.sp = state.sc = 2;
  state.stack[1] = (std::uint64_t{old_la} << test::word_bits) | old_lc;
  state.stack[2] = (std::uint64_t{body} << test::word_bits) | (state.sr & ~loop);
  state.r[0] = base;
  state.r[1] = base + destination;
  return state;
}

struct Case {
  Word length, mode;
  Word quota = std::numeric_limits<Word>::max();
  std::uint64_t deadline = std::numeric_limits<std::uint64_t>::max();
  bool noncanonical = false, overlap = false;
};

void prefix(const NativeImage &image, const Case &scenario) {
  const auto &[length, mode, quota, deadline, noncanonical, overlap] = scenario;
  Fixture fixture(length);
  const auto output = overlap ? 1U : destination;
  if (noncanonical) { fixture.known[output] = 2; }
  auto seed = initial(fixture, mode);
  seed.r[1] = base + output;
  auto expected = seed;
  fixture.data[0] ^= (mode + quota) & word_mask;
  auto wanted = fixture.data;
  auto expected_known = fixture.known;
  const auto retired =
      static_cast<Word>(std::min({std::uint64_t{quota}, deadline, std::uint64_t{length} * turns}));
  for (Word step = 0; step < retired; ++step) {
    const auto index = step / length;
    const auto slot = step % length;
    if (slot == 0) {
      expected.x = wanted[index];
      ++expected.r[0];
    }
    if (slot == length - 1) {
      wanted[output + index] = static_cast<Word>(expected.x);
      expected_known[output + index] = 1;
      ++expected.r[1];
      if (index + 1 == turns) {
        test::loop_frames::complete(expected);
      } else {
        --expected.lc;
      }
    }
    expected.pc = slot == length - 1 && index + 1 < turns ? body : body + slot + 1;
  }
  Core core(Memory::external(), &fixture);
  core.set_registers(seed);
  core.install(image, 0);
  const auto result = deadline == std::numeric_limits<std::uint64_t>::max()
                          ? core.run(quota)
                          : core.run_until(deadline);
  require(result.retired == retired && result.fault == Fault{} &&
              result.progress == Progress{.instructions = retired, .estimated_cycles = retired} &&
              core.registers() == expected && fixture.data == wanted &&
              fixture.known == expected_known && fixture.callbacks == 0,
          "A shared image preserves every quota/deadline prefix, owner RAM, flags and DO frame");
}

void unknown(const NativeImage &image, const bool after_install = false) {
  constexpr Word length = 3;
  constexpr Word missing = 2;
  Fixture fixture(length);
  if (!after_install) { fixture.known[missing] = 0; }
  auto wanted = fixture.data;
  for (Word index = 0; index < missing; ++index) {
    wanted[destination + index] = fixture.data[index];
  }
  Core core(Memory::external(), &fixture);
  core.set_registers(initial(fixture, 0));
  core.install(image, 0);
  if (after_install) { fixture.known[missing] = 0; }
  const auto result = core.run(length * turns);
  require(result.retired == length * missing &&
              result.progress == Progress{.instructions = std::uint64_t{length} * missing,
                                          .estimated_cycles = std::uint64_t{length} * missing} &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::x,
                                    .address = base + missing,
                                    .pc = body} &&
              fixture.data == wanted && fixture.known[missing] == 0 && fixture.callbacks == 1,
          "An unknown word reaches the real failed read after the exact prepared prefix");
}

void metadata_alias(const NativeImage &image) {
  constexpr Word length = 3;
  constexpr Word known_pattern = 0x01010101;
  Fixture fixture(length);
  fixture.data[0] = 0;
  auto bytes = std::as_writable_bytes(std::span(fixture.data).subspan(destination, 8));
  std::ranges::fill(bytes, std::byte{1});
  auto wanted = fixture.data;
  wanted[destination] = 0;
  auto expected = initial(fixture, 0);
  const auto seed = expected;
  expected.x &= ~std::uint64_t{word_mask};
  expected.r[0] += 2;
  ++expected.r[1];
  --expected.lc;
  expected.pc = body + 1;
  Core core(Memory::external(), &fixture);
  core.set_registers(seed);
  core.install(image, 0);
  fixture.window.known = {reinterpret_cast<std::uint8_t *>(bytes.data()), fixture.data.size()};
  ++fixture.revision;
  core.install(image, 0);
  const auto result = core.run(length * turns);
  require(result.retired == length &&
              result.progress == Progress{.instructions = length, .estimated_cycles = length} &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::x,
                                    .address = base + 1,
                                    .pc = body} &&
              core.registers() == expected && fixture.data == wanted &&
              fixture.data[destination + 1] == known_pattern && fixture.callbacks == 1,
          "Cold alias rejection preserves the real prefix and fault after a store clears metadata");
}

void setup(const NativeImage &image, const Word mode) {
  constexpr Word length = 3;
  constexpr std::uint64_t setup_cycles = 5;
  Fixture fixture(length);
  auto expected = initial(fixture, mode);
  auto seed = expected;
  seed.pc = first;
  seed.sr &= ~loop;
  seed.la = old_la;
  seed.lc = old_lc;
  seed.sp = seed.sc = 0;
  seed.stack[1] = seed.stack[2] = 0;
  Core core(Memory::external(), &fixture);
  core.set_registers(seed);
  core.install(image, 0);
  const auto begin = core.run(1);
  require(begin.retired == 1 && begin.fault == Fault{} && core.registers() == expected &&
              begin.progress == Progress{.instructions = 1, .estimated_cycles = setup_cycles},
          "The original DO instruction builds the exact frame before entering its SSA body");
  auto wanted = fixture.data;
  for (Word part = 1; part <= 2; ++part) {
    const auto completed = part * (turns / 2);
    for (Word index = 0; index < completed; ++index) {
      wanted[destination + index] = fixture.data[index];
    }
    expected.x = fixture.data[completed - 1];
    expected.r[0] = base + completed;
    expected.r[1] = base + destination + completed;
    expected.lc = turns - completed;
    if (part == 2) {
      test::loop_frames::complete(expected);
      expected.pc = body + length;
    }
    const auto instructions = std::uint64_t{completed} * length;
    const auto cycles = setup_cycles + instructions;
    const auto result = core.run_until(cycles);
    require(result.retired == (turns / 2) * length && result.fault == Fault{} &&
                result.progress ==
                    Progress{.instructions = 1 + instructions, .estimated_cycles = cycles} &&
                core.registers() == expected && fixture.data == wanted && fixture.callbacks == 0,
            "A resumed image retains absolute time, fresh loop credit and exact DO restoration");
  }
}

void mode_routing() {
  constexpr Word length = 3;
  constexpr Word saturation = 1U << 20;
  constexpr Word narrow = 1U << 17;
  Fixture source(length);
  source.code[3] = op(isa::Opcode::asl_a);
  const auto image = compile(lower(source));
  for (const auto mode : modes) {
    for (const Word flags : {0U, test::status_byte}) {
      Fixture fixture(length);
      fixture.code = source.code;
      fixture.data[0] = test::sign24 - 1;
      auto seed = initial(fixture, mode | saturation);
      seed.sr = (seed.sr & ~test::status_byte) | flags;
      seed.a = std::uint64_t{test::sign24 - 1} << test::word_bits;
      auto expected = seed;
      auto wanted = fixture.data;
      expected.x = (expected.x & ~std::uint64_t{word_mask}) | wanted[0];
      ++expected.r[0];
      const auto old = expected;
      test::shift_model::apply(expected, old, {.left = true});
      wanted[destination] = static_cast<Word>(expected.x) & word_mask;
      ++expected.r[1];
      --expected.lc;
      expected.pc = body;
      Core core(Memory::external(), &fixture);
      core.set_registers(seed);
      core.install(image, 0);
      const auto result = core.run(length);
      require(result.retired == length && result.fault == Fault{} &&
                  result.progress == Progress{.instructions = length, .estimated_cycles = length} &&
                  core.registers() == expected && fixture.data == wanted && fixture.callbacks == 0,
              "A numeric mode outside SSA coverage retains exact saturated arithmetic and flags");
    }
  }
  Fixture fixture(length);
  fixture.code = source.code;
  const auto seed = initial(fixture, narrow);
  auto expected = seed;
  ++expected.pc;
  const auto data = fixture.data;
  Core core(Memory::external(), &fixture);
  core.set_registers(seed);
  core.install(image, 0);
  const auto result = core.run(length);
  require(result.retired == 0 && result.progress == Progress{} &&
              result.fault == Fault{.kind = FaultKind::unsupported_mode,
                                    .space = Space::program,
                                    .address = body,
                                    .value = seed.sr,
                                    .pc = body} &&
              core.registers() == expected && fixture.data == data && fixture.callbacks == 0,
          "An unsupported numeric mode reaches its original exact fault before any RAM effects");
}

struct WindowCase {
  Word from, to, mode, quota;
  std::optional<Word> missing = std::nullopt;
  bool noncanonical = false, rebind = false;
};

void window_prefix(const NativeImage &image, const WindowCase input) {
  constexpr Word length = 4;
  constexpr Word half = 16;
  Fixture fixture(length);
  fixture.code[3] = fixture.code[2] | (2U << 8);
  auto spare = fixture.data;
  for (auto &word : spare) { word ^= test::sign24 - 1; }
  auto spare_known = fixture.known;
  std::array views{MemoryWindow{.address = base,
                                .words = std::span(fixture.data).first(half),
                                .known = std::span(fixture.known).first(half),
                                .writable = true},
                   MemoryWindow{.address = base + half,
                                .words = std::span(fixture.data).subspan(half),
                                .known = std::span(fixture.known).subspan(half),
                                .writable = true}};
  fixture.views = views;
  auto seed = initial(fixture, input.mode);
  constexpr std::uint64_t seed_x = 0xabcdef123456;
  seed.x = seed_x;
  seed.r[0] = base + input.from;
  seed.r[1] = base + input.to;
  seed.r[2] = base + (input.from < half ? half : 0);
  Core core(Memory::external(), &fixture);
  core.set_registers(seed);
  core.install(image, 0);
  if (input.rebind) {
    views[0].words = std::span(spare).first(half);
    views[1].words = std::span(spare).subspan(half);
    views[0].known = std::span(spare_known).first(half);
    views[1].known = std::span(spare_known).subspan(half);
    ++fixture.revision;
    core.install(image, 0);
  }
  auto &data = input.rebind ? spare : fixture.data;
  auto &known = input.rebind ? spare_known : fixture.known;
  if (input.missing) { known[*input.missing] = 0; }
  if (input.noncanonical) { known[input.to + 1] = 0x80; }
  const auto original = fixture.data;
  auto wanted = data;
  auto wanted_known = known;
  auto expected = seed;
  Fault failure;
  Word retired = 0;
  for (; retired < input.quota; ++retired) {
    const auto iteration = retired / length;
    const auto slot = retired % length;
    expected.pc = body + slot + 1;
    if (slot <= 1) {
      const auto index = expected.r[slot == 0 ? 0U : 2U]++ - base;
      if (wanted_known[index] == 0) {
        failure = {.kind = FaultKind::memory_read,
                   .space = Space::x,
                   .address = base + index,
                   .pc = body + slot};
        break;
      }
      expected.x = (expected.x & ~std::uint64_t{word_mask}) | wanted[index];
    }
    if (slot != length - 1) { continue; }
    const auto index = expected.r[1]++ - base;
    wanted[index] = static_cast<Word>(expected.x) & word_mask;
    wanted_known[index] = 1;
    if (iteration + 1 == turns) {
      test::loop_frames::complete(expected);
    } else {
      --expected.lc;
      expected.pc = body;
    }
  }
  const auto result = core.run(input.quota);
  require(
      result.retired == retired && result.fault == failure &&
          result.progress == Progress{.instructions = retired, .estimated_cycles = retired} &&
          core.registers() == expected && data == wanted && known == wanted_known &&
          fixture.callbacks == (failure == Fault{} ? 0U : 1U) &&
          (!input.rebind || fixture.data == original),
      "Window choice preserves crossed ranges, ordered aliases, faults, knowledge and rebinding");
}

void window_selection() {
  constexpr Word length = 4;
  constexpr Word half = 16;
  constexpr Word tail = (half * 2) - turns;
  constexpr Word last = (half * 2) - 1;
  constexpr Word missing_second = half + 2;
  Fixture fixture(length);
  fixture.code[3] = fixture.code[2] | (2U << 8);
  const std::array code{CodeSpan{.address = first, .words = fixture.code}};
  constexpr std::array entries{first, body};
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
  const std::array regions{
      RegionLayout{.space = Space::x, .first = base, .size = half, .tracked = true},
      RegionLayout{.space = Space::x, .first = base + half, .size = half, .tracked = true}};
  const std::array layouts{VariantLayout{.id = 0, .memory = regions}};
  const auto image = compile(Core::lower(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}}));
  constexpr std::array positions{std::array{0U, 16U},
                                 std::array{16U, 0U},
                                 std::array{14U, 12U},
                                 std::array{12U, 14U},
                                 std::array{28U, 0U},
                                 std::array{0U, 28U},
                                 std::array{0U, 17U}};
  for (const auto mode : modes) {
    for (const auto pair : positions) {
      for (Word quota = 0; quota <= length * turns; ++quota) {
        window_prefix(image, {.from = pair[0], .to = pair[1], .mode = mode, .quota = quota});
      }
    }
    window_prefix(image,
                  {.from = 0, .to = 16, .mode = mode, .quota = length * turns, .missing = 2});
    window_prefix(
        image,
        {.from = 0, .to = tail, .mode = mode, .quota = length * turns, .missing = missing_second});
    window_prefix(image,
                  {.from = tail, .to = 0, .mode = mode, .quota = length * turns, .missing = last});
    window_prefix(
        image, {.from = 16, .to = 0, .mode = mode, .quota = length * turns, .noncanonical = true});
    window_prefix(image,
                  {.from = 0, .to = 16, .mode = mode, .quota = length * turns, .rebind = true});
  }
}

void ordinary_discovery() {
  constexpr Word length = 3;
  const Fixture fixture(length);
  const std::array spans{CodeSpan{.address = body, .words = std::span(fixture.code).subspan(2)}};
  constexpr std::array entries{body};
  const auto regions = detail::image_regions(detail::plan_image(spans, entries));
  require(std::ranges::any_of(regions,
                              [](const auto &region) {
                                return region.front().pc == body &&
                                       region.back().pc == body + length - 1;
                              }),
          "An ordinary RAM block is a guarded SSA candidate without a declared DO");
}

struct OrdinaryCase {
  Word mode, quota;
  std::optional<Word> loop_address;
  bool timed = false;
};

void ordinary_prefix(const NativeImage &image, const OrdinaryCase &scenario) {
  constexpr Word length = 3;
  Fixture fixture(length);
  auto seed = initial(fixture, scenario.mode);
  if (scenario.loop_address) {
    seed.la = *scenario.loop_address;
  } else {
    seed.sr &= ~loop;
  }
  auto expected = seed;
  auto wanted = fixture.data;
  for (Word retired = 0; retired < scenario.quota; ++retired) {
    const auto pc = expected.pc++;
    require(pc >= body && pc < body + length, "Ordinary expectation stays in its supplied body");
    if (pc == body) {
      expected.x = (expected.x & ~std::uint64_t{word_mask}) | wanted.at(expected.r[0] - base);
      ++expected.r[0];
    }
    if (pc == body + length - 1) {
      wanted.at(expected.r[1] - base) = static_cast<Word>(expected.x) & word_mask;
      ++expected.r[1];
    }
    if ((expected.sr & loop) != 0 && pc == expected.la) {
      if (expected.lc == 1) {
        test::loop_frames::complete(expected);
      } else {
        --expected.lc;
        expected.pc = body;
      }
    }
  }
  Core core(Memory::external(), &fixture);
  core.set_registers(seed);
  core.install(image, 0);
  const auto result = scenario.timed ? core.run_until(scenario.quota) : core.run(scenario.quota);
  require(result.fault == Fault{} && result.retired == scenario.quota &&
              result.progress ==
                  Progress{.instructions = scenario.quota, .estimated_cycles = scenario.quota} &&
              core.registers() == expected && fixture.data == wanted && fixture.callbacks == 0,
          "Ordinary SSA preserves every prefix and unrelated or interior inherited DO state");
}

void ordinary_execution() {
  constexpr Word length = 3;
  Fixture fixture(length);
  const auto image = compile(lower(fixture, true), false);
  constexpr std::array<std::optional<Word>, 4> loops{
      std::nullopt, old_la, body + 1, body + length - 1};
  for (const auto mode : modes) {
    for (const auto address : loops) {
      const auto count = address && *address < body + length
                             ? ((*address - body + 1) * turns) + body + length - *address - 1
                             : length;
      for (Word quota = 0; quota <= count; ++quota) {
        ordinary_prefix(image, {.mode = mode, .quota = quota, .loop_address = address});
        ordinary_prefix(image,
                        {.mode = mode, .quota = quota, .loop_address = address, .timed = true});
      }
    }
  }
  Fixture unknown_fixture(length);
  unknown_fixture.known[0] = 0;
  Core core(Memory::external(), &unknown_fixture);
  auto seed = initial(unknown_fixture, 0);
  seed.sr &= ~loop;
  core.set_registers(seed);
  core.install(image, 0);
  const auto result = core.run(length);
  require(result.retired == 0 && result.progress == Progress{} &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::x,
                                    .address = base,
                                    .pc = body} &&
              unknown_fixture.callbacks == 1,
          "An unknown ordinary source reaches its precise real failed read without retirement");
}

constexpr std::array<Word, 6> raw_registers{8, 9, 10, 11, 12, 13};
constexpr Word raw_length = 5 * raw_registers.size();
constexpr Word raw_extension_mask = word_mask ^ 0xff;
constexpr std::uint64_t raw_a_seed = 0x80123456abcdef, raw_b_seed = 0x7ffedcba543210;
constexpr std::uint64_t raw_accumulator_mask = 0xffffffffffffff;

void raw_code(Fixture &fixture) {
  Word index = 2;
  const auto move = [](const Word source, const Word target) {
    return Word{0x200000} | (source << 13) | (target << 8);
  };
  const auto store = [](const Word reg) {
    return (op(isa::Opcode::move_x_effective) & ~(7U << 16)) | (reg << 16) | (3U << 11) | (1U << 8);
  };
  for (const auto reg : raw_registers) {
    fixture.code[index++] = move(reg, 5);
    fixture.code[index++] = store(5);
    fixture.code[index++] = move(4, reg);
    fixture.code[index++] = move(reg, 6);
    fixture.code[index++] = store(6);
  }
}

Word raw_word(const Registers &state, const Word reg) {
  const auto value = (reg & 1U) == 0 ? state.a : state.b;
  constexpr std::array shifts{0U, 48U, 24U};
  const auto part = (reg - 8) / 2;
  const auto word = static_cast<Word>(value >> shifts[part]) & word_mask;
  return part == 1 && (word & 0x80) != 0 ? word | raw_extension_mask : word;
}

void raw_write(Registers &state, const Word reg) {
  auto &value = (reg & 1U) == 0 ? state.a : state.b;
  const auto word = state.x & word_mask;
  constexpr std::array shifts{0U, 48U, 24U};
  const auto shift = shifts[(reg - 8) / 2];
  const auto mask = std::uint64_t{word_mask} << shift;
  value = ((value & ~mask) | (word << shift)) & raw_accumulator_mask;
}

struct RawCase {
  Word mode, value, quota;
};

void raw_prefix(const NativeImage &image, const RawCase &scenario) {
  const auto [mode, value, quota] = scenario;
  Fixture fixture(raw_length);
  raw_code(fixture);
  auto seed = initial(fixture, mode);
  seed.sr &= ~loop;
  seed.a = raw_a_seed;
  seed.b = raw_b_seed;
  seed.x = (seed.x & ~std::uint64_t{word_mask}) | value;
  auto expected = seed;
  auto wanted = fixture.data;
  for (Word step = 0; step < quota; ++step) {
    const auto reg = raw_registers[step / 5];
    switch (step % 5) {
      case 0:
        expected.x =
            (expected.x & word_mask) | (std::uint64_t{raw_word(expected, reg)} << test::word_bits);
        break;
      case 1:
        wanted[expected.r[1]++ - base] = static_cast<Word>(expected.x >> test::word_bits);
        break;
      case 2: raw_write(expected, reg); break;
      case 3:
        expected.y = (expected.y & ~std::uint64_t{word_mask}) | raw_word(expected, reg);
        break;
      case 4: wanted[expected.r[1]++ - base] = static_cast<Word>(expected.y) & word_mask; break;
      default: break;
    }
    ++expected.pc;
  }
  Core core(Memory::external(), &fixture);
  core.set_registers(seed);
  core.install(image, 0);
  const auto result = core.run(quota);
  require(result.retired == quota && result.fault == Fault{} &&
              result.progress == Progress{.instructions = quota, .estimated_cycles = quota} &&
              core.registers() == expected && fixture.data == wanted && fixture.callbacks == 0,
          "Raw accumulator parts preserve untouched fields, signed extension, flags and prefixes");
}

void raw_execution() {
  Fixture fixture(raw_length);
  raw_code(fixture);
  const std::array spans{CodeSpan{.address = body, .words = std::span(fixture.code).subspan(2)}};
  constexpr std::array entries{body};
  const std::array memory{
      RegionLayout{.space = Space::x, .first = base, .size = fixture.data.size(), .tracked = true}};
  const auto regions = detail::image_regions(detail::plan_image(spans, entries), memory);
  require(std::ranges::any_of(regions,
                              [](const auto &region) {
                                return region.front().pc == body && region.size() == raw_length;
                              }),
          "All raw partial-register transfers remain in one admitted ordinary SSA region");
  const auto image = compile(lower(fixture, true), false);
  for (const auto mode : modes) {
    for (const auto value : {0U, 0x7fffffU, 0x800000U, 0xffffffU, 0x123456U, 0x80U}) {
      for (Word quota = 0; quota <= raw_length; ++quota) {
        raw_prefix(image, {.mode = mode, .value = value, .quota = quota});
      }
    }
  }
}
}

int main() try {
  window_selection();
  mode_routing();
  ordinary_discovery();
  ordinary_execution();
  raw_execution();
  constexpr Word short_length = 3;
  constexpr Word long_length = 70;
  Fixture fixture(short_length);
  const auto image = compile(lower(fixture));
  unknown(image);
  unknown(image, true);
  metadata_alias(image);
  for (const auto mode : modes) {
    setup(image, mode);
    for (Word limit = 0; limit <= short_length * turns; ++limit) {
      prefix(image, {.length = short_length, .mode = mode, .quota = limit});
      prefix(image, {.length = short_length, .mode = mode, .deadline = limit});
    }
    prefix(image,
           {.length = short_length,
            .mode = mode,
            .quota = short_length * turns,
            .deadline = std::numeric_limits<std::uint64_t>::max(),
            .noncanonical = true});
    prefix(image,
           {.length = short_length, .mode = mode, .quota = short_length * turns, .overlap = true});
  }
  Fixture long_fixture(long_length);
  const auto long_image = compile(lower(long_fixture));
  prefix(long_image,
         {.length = long_length,
          .mode = 0,
          .quota = long_length * turns,
          .deadline = std::numeric_limits<std::uint64_t>::max()});
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
