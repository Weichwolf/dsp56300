// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit/llvm/runtime.hpp"
#include "loop_frames.hpp"
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

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return space == Space::x ? std::span(&window, 1) : std::span<const MemoryWindow>{};
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
  unsigned callbacks = 0;
  std::uint64_t revision = 0;
};

SemanticImage lower(Fixture &fixture) {
  const std::array spans{CodeSpan{.address = first, .words = fixture.code}};
  constexpr std::array entries{first, body};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array regions{
      RegionLayout{.space = Space::x, .first = base, .size = fixture.data.size(), .tracked = true}};
  const std::array layouts{VariantLayout{.id = 0, .memory = regions}};
  return Core::lower(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
}

NativeImage compile(const SemanticImage &semantic) {
  detail::LlvmModule module(std::string(semantic.exports().front().name));
  module.link_bitcode(semantic.bitcode());
  unsigned loops = 0;
  for (const auto &function : *llvm::unwrap(module.code())) {
    for (const auto &block : function) {
      for (const auto &instruction : block) {
        if (const auto *call = llvm::dyn_cast<llvm::CallBase>(&instruction)) {
          const auto *callee = call->getCalledFunction();
          require(callee == nullptr || callee->getName() != "dsp56300_region_memory",
                  "Fixed-image IR inlines the storage resolver before native optimization");
        }
        if (llvm::isa<llvm::PHINode>(instruction) &&
            instruction.getName().starts_with("remaining")) {
          ++loops;
        }
      }
    }
  }
  require(loops >= modes.size(),
          "Every supported numeric mode has an SSA loop before target optimization");
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
}

int main() try {
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
