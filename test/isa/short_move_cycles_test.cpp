// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr unsigned short_words = 64, address_shift = 8, register_shift = 24;
constexpr Word source = 0x345678, high = 0x654321;
constexpr Word xword = 0x123456, yword = 0x56789a;
constexpr Word move_x0 = static_cast<Word>(isa::Opcode::move_x_absolute_short);
constexpr Word y_space = Word{1} << 19, load_bit = Word{1} << 15;
std::uint64_t cases = 0;

struct Image final : Io {
  std::array<Word, 1> program{};
  std::array<Word, short_words> x{}, y{};
  unsigned reads = 0, writes = 0;

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address == 0 ? std::optional(program[0]) : std::nullopt;
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (space == Space::program && address == 0) {
      value = program[0];
    } else if (address < short_words && (space == Space::x || space == Space::y)) {
      value = (space == Space::x ? x : y)[address];
    } else {
      return false;
    }
    ++reads;
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    if (address >= short_words || (space != Space::x && space != Space::y)) { return false; }
    (space == Space::x ? x : y)[address] = value;
    ++writes;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

Core make_core(Image &image, const bool external) {
  if (external) { return Core(Memory::external(), &image); }
  return Core(Memory::separate({.program = image.program, .x = image.x, .y = image.y}));
}

struct Case {
  Space space;
  Word address;
  bool load;
};

Registers setup(Image &image, const Case item) {
  // FM MOVE X/Y:aa format: W at15, six address bits at13..8, scalar X0 selector4.
  image.program[0] = move_x0 | (item.space == Space::y ? y_space : 0) | (item.load ? load_bit : 0) |
                     (item.address << address_shift);
  image.x.fill(xword);
  image.y.fill(yword);
  Registers initial;
  initial.m.fill(word_mask);
  initial.x = (std::uint64_t{high} << register_shift) | source;
  return initial;
}

void verify(
    const Core &cpu, const Image &image, Registers initial, const Case item, const bool external) {
  std::array<Word, short_words> expected_x{};
  std::array<Word, short_words> expected_y{};
  expected_x.fill(xword);
  expected_y.fill(yword);
  if (item.load) {
    const auto value = item.space == Space::x ? xword : yword;
    initial.x = (std::uint64_t{high} << register_shift) | value;
  } else {
    (item.space == Space::x ? expected_x : expected_y)[item.address] = source;
  }
  initial.pc = 1;
  // FM table A-1, A-6: ordinary absolute-short read1/store2, independent of aa.
  const Progress expected{.instructions = 1, .estimated_cycles = item.load ? 1U : 2U};
  require(
      cpu.state().failure.kind == FaultKind::none && cpu.progress() == expected,
      "Literal absolute-short MOVE has the independently specified direction-dependent base cost");
  require(cpu.registers() == initial && image.x == expected_x && image.y == expected_y,
          "Timing correction preserves full registers and every untouched memory cell");
  if (external) {
    require(
        image.reads == (item.load ? 1U : 0U) && image.writes == (item.load ? 0U : 1U),
        "Prepared source inspection adds no runtime fetch; exactly one real data transfer occurs");
  }
}

NativeImage prepare() {
  std::vector<Word> opcodes;
  constexpr unsigned variants = short_words * 4;
  opcodes.reserve(variants);
  for (const auto space : {Space::x, Space::y}) {
    for (Word address = 0; address < short_words; ++address) {
      for (const bool load : {false, true}) {
        Image image;
        static_cast<void>(setup(image, {.space = space, .address = address, .load = load}));
        opcodes.push_back(image.program[0]);
      }
    }
  }
  return compile_operations(opcodes, 0);
}

void ordinary(const Case item, const NativeImage &prepared, const bool external) {
  Image image;
  const auto initial = setup(image, item);
  auto cpu = make_core(image, external);
  const auto variant = ((((item.space == Space::y ? 1U : 0U) * short_words) + item.address) * 2) +
                       (item.load ? 1U : 0U);
  cpu.install(prepared, variant);
  cpu.set_registers(initial);
  const auto result = cpu.run(1);
  ++cases;
  require(result.retired == 1, "One actual short MOVE dispatches");
  verify(cpu, image, initial, item, external);
}

}
}

int main() {
  using namespace dsp56300;
  using namespace dsp56300::test;
  try {

    const auto image = prepare();
    for (const auto space : {Space::x, Space::y}) {
      for (Word address = 0; address < short_words; ++address) {
        for (const bool load : {false, true}) {
          const Case item{.space = space, .address = address, .load = load};
          ordinary(item, image, false);
          ordinary(item, image, true);
        }
      }
    }
    std::cout << "Short X/Y MOVE: " << cases << " cases, " << checks
              << " assertions; all 64 addresses, both directions, installed LLVM images and "
                 "buffered/external execution\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
