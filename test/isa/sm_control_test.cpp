// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "scripted_io.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word sm = Word{1} << 20, interrupt_mask = 0x300;
constexpr Word short_address = 0x20, operand_address = 0x180;
constexpr std::uint64_t seed_a = 0x123456789abcde, seed_b = 0xfedcba98765432;
constexpr std::uint64_t seed_x = 0x1357902468ac, seed_y = 0xabcdef765432;
enum class Path : std::uint8_t { dispatched, owned, captured };

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
    Form{.opcode = 0x05f439, .words = 2, .cycles = 2, .immediate = true},
    Form{.opcode = 0x05f039, .extension = operand_address, .words = 2, .cycles = 2},
    Form{.opcode = 0x057039, .extension = operand_address, .words = 2, .cycles = 2, .load = false},
    Form{.opcode = 0x05f079,
         .extension = operand_address,
         .words = 2,
         .cycles = 2,
         .space = Space::y},
    Form{.opcode = 0x057079,
         .extension = operand_address,
         .words = 2,
         .cycles = 2,
         .load = false,
         .space = Space::y},
    Form{.opcode = 0x05a039, .address = short_address},
    Form{.opcode = 0x052039, .load = false, .address = short_address},
    Form{.opcode = 0x05a079, .space = Space::y, .address = short_address},
    Form{.opcode = 0x052079, .load = false, .space = Space::y, .address = short_address}};

struct Input {
  Form form;
  Backend backend;
  Path path;
  Word previous;
  Word replacement;
};

class MemoryBus final : public Io {
public:
  MemoryBus(const Buffers buffers, scripted_io::Bus &script)
      : script_(&script), x_(buffers.x), y_(buffers.y) {}

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (!script_->read(space, address, pc, value)) { return false; }
    if (space == Space::program) { return true; }
    const auto memory = space == Space::x ? x_ : y_;
    return address < memory.size() && memory[address] == value;
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    if (!script_->write(space, address, pc, value) || space == Space::program) { return false; }
    const auto memory = space == Space::x ? x_ : y_;
    if (address >= memory.size()) { return false; }
    memory[address] = value;
    return true;
  }

  bool bit(const Space space,
           const Word address,
           const Word pc,
           const Word bit,
           const BitAction action,
           bool &previous) noexcept override {
    return script_->bit(space, address, pc, bit, action, previous);
  }

private:
  scripted_io::Bus *script_;
  std::span<Word> x_;
  std::span<Word> y_;
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

void execute(TestCore &core, const Input input, const Word extension) {
  const auto result =
      input.path == Path::captured
          ? core.run_instruction(
                {.address = start,
                 .opcode = input.form.opcode,
                 .extension = input.form.words == 2 ? std::optional(extension) : std::nullopt})
          : core.run(1);
  require(result.retired == 1 && result.fault.kind == FaultKind::none,
          "Dispatched, externally fetched and captured execution retire the real instruction");
}

void one(const Input input) {
  std::array<Word, memory_words> program{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  const auto form = input.form;
  const auto extension = form.immediate ? input.replacement : form.extension;
  program[start] = form.opcode;
  program[start + 1] = extension;
  auto &memory = form.space == Space::x ? x : y;
  memory[form.address] = input.replacement;
  scripted_io::Bus script;
  MemoryBus bus({.program = program, .x = x, .y = y}, script);
  script.pc = start;
  if (input.path == Path::owned) {
    script.events.push_back({.space = Space::program, .address = start, .value = form.opcode});
    if (form.words == 2) {
      script.events.push_back({.space = Space::program, .address = start + 1, .value = extension});
    }
  }
  if (input.path != Path::dispatched && !form.immediate) {
    script.events.push_back(
        {.kind = form.load ? scripted_io::Action::read : scripted_io::Action::write,
         .space = form.space,
         .address = form.address,
         .value = form.load ? input.replacement : input.previous});
  }
  TestCore core(input.path == Path::dispatched
                    ? Memory::separate({.program = program, .x = x, .y = y})
                    : Memory::external(),
                &bus,
                {.backend = input.backend});
  auto expected = seed(input.previous);
  core.set_registers(expected);
  expected.pc += form.words;
  if (form.load) { expected.sr = input.replacement; }
  execute(core, input, extension);
  require(core.registers() == expected,
          "MOVEC preserves all data/AGU/stack state and replaces SR only on a load");
  require(memory[form.address] == (form.load ? input.replacement : input.previous),
          "MOVEC memory stores the original full SR word without arithmetic limiting");
  const auto fetch_calls = input.path == Path::owned ? form.words : 0U;
  const auto operand_calls = input.path != Path::dispatched && !form.immediate ? 1U : 0U;
  require(core.progress() == Progress{.instructions = 1, .estimated_cycles = form.cycles} &&
              script.exact && script.calls == fetch_calls + operand_calls,
          "Control transfers preserve base cost and exact program/operand bus events");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300;
    using namespace dsp56300::test;
    for (const auto form : forms) {
      for (const auto backend : scripted_io::backends) {
        for (const auto path : {Path::dispatched, Path::owned, Path::captured}) {
          for (const auto scale : {Word{0}, mode_down, mode_up}) {
            for (const auto ccr : {Word{0}, flag_c | flag_v, status_byte}) {
              for (const auto replacement : {Word{0}, sm | status_byte, mode_rm | flag_l}) {
                one({.form = form,
                     .backend = backend,
                     .path = path,
                     .previous = sm | interrupt_mask | scale | ccr,
                     .replacement = replacement});
              }
            }
          }
        }
      }
    }
    std::cout << checks << " SM control-transfer checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
