// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word sm = Word{1} << 20, interrupt_mask = 0x300;
constexpr Word short_address = 0x20, operand_address = 0x180;
constexpr std::uint64_t seed_a = 0x123456789abcde, seed_b = 0xfedcba98765432;
constexpr std::uint64_t seed_x = 0x1357902468ac, seed_y = 0xabcdef765432;
constexpr Word control_sr = 0x19, transfer_read = Word{1} << 15, y_flag = Word{1} << 6;
constexpr Word long_address = 0x30U << 8, immediate_address = 0x34U << 8;
constexpr Word effective_sr = static_cast<Word>(isa::Opcode::movec_effective) | control_sr;
constexpr Word short_sr =
    static_cast<Word>(isa::Opcode::movec_absolute_short) | control_sr | (short_address << 8);
constexpr std::array<Word, 3> replacements{0, sm | status_byte, mode_rm | flag_l};
std::size_t cases = 0;

struct Form {
  Word opcode;
  Word extension = 0;
  unsigned words = 1;
  Word cycles = 1;
  bool load = true;
  bool immediate = false;
  Space space = Space::x;
  Word address = operand_address;
};

// FM13-130/131: SR control code11001, read W=1, X/Y S=0/1.
// Absolute long effective addresses use MMMRRR=110000; immediates=110100.
constexpr std::array forms{
    Form{.opcode = effective_sr | transfer_read | immediate_address,
         .words = 2,
         .cycles = 2,
         .immediate = true},
    Form{.opcode = effective_sr | transfer_read | long_address,
         .extension = operand_address,
         .words = 2,
         .cycles = 2},
    Form{.opcode = effective_sr | long_address,
         .extension = operand_address,
         .words = 2,
         .cycles = 2,
         .load = false},
    Form{.opcode = effective_sr | transfer_read | long_address | y_flag,
         .extension = operand_address,
         .words = 2,
         .cycles = 2,
         .space = Space::y},
    Form{.opcode = effective_sr | long_address | y_flag,
         .extension = operand_address,
         .words = 2,
         .cycles = 2,
         .load = false,
         .space = Space::y},
    Form{.opcode = short_sr | transfer_read, .address = short_address},
    Form{.opcode = short_sr, .load = false, .address = short_address},
    Form{.opcode = short_sr | transfer_read | y_flag, .space = Space::y, .address = short_address},
    Form{.opcode = short_sr | y_flag, .load = false, .space = Space::y, .address = short_address}};

struct Input {
  Form form;
  bool external;
  Word previous;
  Word replacement;
};

struct MemoryBus final : Io {
  MemoryBus(const Buffers buffers, const Input input) : buffers_(buffers), input_(input) {}

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < buffers_.program.size() ? std::optional(buffers_.program[address])
                                             : std::nullopt;
  }

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (!observe({.space = space, .address = address, .pc = pc})) { return false; }
    const auto memory = space == Space::x ? buffers_.x : buffers_.y;
    exact = exact && address < memory.size();
    if (!exact) { return false; }
    value = memory[address];
    exact = value == input_.replacement;
    return exact;
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    return store({.space = space, .address = address, .pc = pc, .value = value, .write = true});
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    ++calls;
    exact = false;
    return false;
  }

  unsigned calls = 0;
  bool exact = true;

  struct Access {
    Space space;
    Word address, pc, value = 0;
    bool write = false;
  };

  bool observe(const Access access) noexcept {
    ++calls;
    exact = exact && calls == 1 && input_.external && !input_.form.immediate &&
            access.space == input_.form.space && access.address == input_.form.address &&
            access.pc == start && access.write != input_.form.load;
    return exact;
  }

  bool store(const Access access) noexcept {
    if (!observe(access)) { return false; }
    const auto memory = access.space == Space::x ? buffers_.x : buffers_.y;
    exact = exact && access.address < memory.size() && access.value == input_.previous;
    if (!exact) { return false; }
    memory[access.address] = access.value;
    return true;
  }

  Buffers buffers_;
  Input input_;
};

Registers seed(const Word status) {
  auto result = initial_state();
  result.sr = status;
  result.a = seed_a;
  result.b = seed_b;
  result.x = seed_x;
  result.y = seed_y;
  result.sp = result.sc = 3;
  for (std::size_t i = 0; i < result.stack.size(); ++i) {
    result.stack[i] = (std::uint64_t{operand_address} << word_bits) | i;
  }
  result.r.fill(operand_address);
  result.n.fill(3);
  return result;
}

NativeImage prepare() {
  std::array<std::array<Word, 2>, forms.size() + replacements.size() - 1> programs{};
  std::vector<CodeSpan> spans;
  spans.reserve(programs.size());
  for (const auto form : forms) {
    const auto append = [&](const Word extension) {
      auto &program = programs[spans.size()];
      program = {form.opcode, extension};
      spans.push_back({.address = start, .words = std::span(program).first(form.words)});
    };
    if (form.immediate) {
      for (const auto replacement : replacements) { append(replacement); }
    } else {
      append(form.extension);
    }
  }
  return compile_programs(spans);
}

void one(const NativeImage &image, const std::size_t variant, const Input input) {
  std::array<Word, memory_words> program{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  const auto form = input.form;
  const auto extension = form.immediate ? input.replacement : form.extension;
  program[start] = form.opcode;
  program[start + 1] = extension;
  auto &memory = form.space == Space::x ? x : y;
  memory[form.address] = input.replacement;
  MemoryBus bus({.program = program, .x = x, .y = y}, input);
  Core core(input.external ? Memory::external()
                           : Memory::separate({.program = program, .x = x, .y = y}),
            &bus);
  core.install(image, variant);
  auto expected = seed(input.previous);
  core.set_registers(expected);
  expected.pc += form.words;
  if (form.load) { expected.sr = input.replacement; }
  const auto result = core.run(1);
  ++cases;
  require(result.retired == 1 && result.fault.kind == FaultKind::none,
          "Prepared MOVEC retires the real instruction with ordinary RAM or actual external I/O");
  require(core.registers() == expected,
          "MOVEC preserves all data/AGU/stack state and replaces SR only on a load");
  require(memory[form.address] == (form.load ? input.replacement : input.previous),
          "MOVEC memory stores the original full SR word without arithmetic limiting");
  const auto operand_calls = input.external && !form.immediate ? 1U : 0U;
  require(core.progress() == Progress{.instructions = 1, .estimated_cycles = form.cycles} &&
              bus.exact && bus.calls == operand_calls,
          "Control transfers preserve base cost and exact real operand bus events without opcode "
          "fetches");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300;
    using namespace dsp56300::test;
    const auto image = prepare();
    for (std::size_t index = 0; index < forms.size(); ++index) {
      const auto form = forms[index];
      for (const bool external : {false, true}) {
        for (const auto scale : {Word{0}, mode_down, mode_up}) {
          for (const auto ccr : {Word{0}, flag_c | flag_v, status_byte}) {
            for (std::size_t replacement = 0; replacement < replacements.size(); ++replacement) {
              const auto variant = form.immediate ? replacement : index + replacements.size() - 1;
              one(image,
                  variant,
                  {.form = form,
                   .external = external,
                   .previous = sm | interrupt_mask | scale | ccr,
                   .replacement = replacements[replacement]});
            }
          }
        }
      }
    }
    std::cout << cases << " SM control-transfer cases and " << checks << " checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
