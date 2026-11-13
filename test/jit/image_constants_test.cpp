// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/region_memory.hpp"
#include "jit/llvm/runtime.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word first = 0x100, mask = 0xffffff;

struct Transfer {
  unsigned reg;
  Word at;
  bool load = true, y = false, immediate = false, indirect = false, extended = false,
       short_form = false;
};

constexpr Word register_high_mask = 24, register_high_shift = 17;
constexpr Word immediate_ea = 0x7400, indirect_ea = 0x6000, absolute_ea = 0x7000, stop = 0x87;
constexpr Word host_bits = 0xab000000, multiplier = 0x123457, cursor_source = 0xcd000004;
constexpr std::array modes{0U,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm,
                           test::mode_rm | test::mode_down,
                           test::mode_rm | test::mode_up};
constexpr std::array transfers{
    Transfer{.reg = 18, .at = 0x22, .immediate = true, .short_form = true},
    Transfer{.reg = 16, .at = 2, .immediate = true},
    Transfer{.reg = 4, .at = 2},
    Transfer{.reg = 17, .at = 3},
    Transfer{.reg = 14, .at = 1, .indirect = true},
    Transfer{.reg = 7, .at = 0xfedcba, .immediate = true},
    Transfer{.reg = 15, .at = 7, .load = false, .y = true, .extended = true},
    Transfer{.reg = 14, .at = 5, .load = false, .extended = true},
    Transfer{.reg = 15, .at = 5},
    Transfer{.reg = 6, .at = 7, .y = true}};
using Data = std::array<Word, 16>;
using Known = std::array<std::uint8_t, 16>;

constexpr Word length(const Transfer op) {
  return (op.immediate && !op.short_form) || op.extended ? 2U : 1U;
}

constexpr Word instruction_count = static_cast<Word>(transfers.size());
constexpr Word cycle_count = [] {
  Word sum = 0;
  for (const auto op : transfers) { sum += length(op); }
  return sum;
}();

std::vector<Word> program(const std::span<const Transfer> moves = transfers) {
  std::vector<Word> words;
  for (const auto op : moves) {
    if (op.short_form) {
      words.push_back(0x200000U | (op.reg << 16) | (op.at << 8));
      continue;
    }
    Word word = 0x400000 | ((op.reg & 7U) << 16) |
                ((op.reg & register_high_mask) << register_high_shift) | (op.y ? 0x80000U : 0U) |
                (op.load ? 0x8000U : 0U);
    if (op.immediate) {
      word |= immediate_ea;
    } else if (op.indirect) {
      word |= indirect_ea | (op.at << 8);
    } else if (op.extended) {
      word |= absolute_ea;
    } else {
      word |= op.at << 8;
    }
    words.push_back(word);
    if (length(op) == 2) { words.push_back(op.at); }
  }
  words.push_back(stop);
  return words;
}

struct Fixture final : Io {
  Fixture(const bool alias, const bool writable, const unsigned scenario)
      : code(program()), alias(alias) {
    for (unsigned bank = 0; bank < 2; ++bank) {
      known[bank].fill(1);
      for (Word i = 0; i < 16; ++i) {
        data[bank][i] = host_bits | ((i * multiplier + bank) & mask);
      }
    }
    data[0][3] = cursor_source;
    if (scenario == 1) { known[0][2] = 0; }
    if (scenario == 2) { known[0][3] = 0; }
    if (scenario == 3) { known[0][4] = 0; }
    if (scenario == 4) { known[0][5] = 0; }
    if (scenario == 5) { known[alias ? 0U : 1U][7] = 0; }
    if (scenario == 6) { known[0][5] = 2; }
    for (unsigned bank = 0; bank < 2; ++bank) {
      windows[bank] = {.address = 0,
                       .words = data[alias ? 0U : bank],
                       .known = known[alias ? 0U : bank],
                       .writable = writable};
    }
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word at) const noexcept override {
    return at >= first && at - first < code.size() ? std::optional(code[at - first]) : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    if (space == Space::program) { return {}; }
    return std::span(&windows[space == Space::x ? 0U : 1U], 1);
  }

  bool read(const Space space, const Word at, const Word pc, Word & /*value*/) noexcept override {
    ++callbacks;
    observed = {.kind = FaultKind::memory_read, .space = space, .address = at, .pc = pc};
    return false;
  }

  bool write(const Space space, const Word at, const Word pc, const Word value) noexcept override {
    ++callbacks;
    observed = {
        .kind = FaultKind::memory_write, .space = space, .address = at, .value = value, .pc = pc};
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
  bool alias;
  std::array<Data, 2> data{};
  std::array<Known, 2> known{};
  std::array<MemoryWindow, 2> windows;
  unsigned callbacks = 0;
  Fault observed;
};

Registers seed(const Word mode, const Word flags) {
  auto result = test::register_model::seed(mode);
  result.pc = first;
  result.sr = mode | flags;
  return result;
}

struct Model {
  Registers reg;
  std::array<Data, 2> data;
  std::array<Known, 2> known;
  Fault fault{};
  Word retired = 0;
  std::uint64_t cycles = 0;
};

bool model_move(Model &model, const Fixture &fixture, const Transfer op, const Word pc) {
  if (op.immediate) {
    test::register_model::write(model.reg, op.reg, op.at);
    return true;
  }
  const auto at = op.indirect ? model.reg.r[op.at] : op.at;
  const auto bank = fixture.alias || !op.y ? 0U : 1U;
  const auto value = op.load ? 0U : test::register_model::read(model.reg, op.reg);
  if ((op.load && model.known[bank][at] == 0) ||
      (!op.load && !fixture.windows[op.y ? 1U : 0U].writable)) {
    model.fault = {.kind = op.load ? FaultKind::memory_read : FaultKind::memory_write,
                   .space = op.y ? Space::y : Space::x,
                   .address = at,
                   .value = value,
                   .pc = pc};
    return false;
  }
  if (op.load) {
    test::register_model::write(model.reg, op.reg, model.data[bank][at] & mask);
  } else {
    model.data[bank][at] = value;
    model.known[bank][at] = 1;
  }
  return true;
}

Model expected(const Fixture &fixture,
               const Registers &initial,
               const Word budget,
               const std::uint64_t deadline) {
  Model model{.reg = initial, .data = fixture.data, .known = fixture.known};
  for (const auto op : transfers) {
    if (model.retired == budget || model.cycles >= deadline) { break; }
    const auto pc = model.reg.pc;
    model.reg.pc += length(op);
    if (!model_move(model, fixture, op, pc)) { break; }
    ++model.retired;
    model.cycles += length(op);
  }
  return model;
}

NativeImage compile(const bool writable) {
  Fixture fixture(false, writable, 0);
  const std::array spans{CodeSpan{.address = first, .words = fixture.code}};
  const std::array entries{first};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array regions{
      RegionLayout{
          .space = Space::x, .first = 0, .size = 16, .writable = writable, .tracked = true},
      RegionLayout{
          .space = Space::y, .first = 0, .size = 16, .writable = writable, .tracked = true}};
  const std::array layouts{VariantLayout{.id = 0, .memory = regions}};
  return Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
}

struct Input {
  Word mode, flags;
  bool alias;
  unsigned scenario;
};

template <typename Function> void inputs(const Word mode, Function &&function) {
  for (const Word flags : {0U, test::status_byte}) {
    for (const bool alias : {false, true}) {
      for (unsigned scenario = 0; scenario < 7; ++scenario) {
        function(Input{.mode = mode, .flags = flags, .alias = alias, .scenario = scenario});
      }
    }
  }
}

void direct_case(const detail::PreparedRegion &native, const Input input, const unsigned grant) {
  Fixture fixture(input.alias, true, input.scenario);
  const auto initial = seed(input.mode, input.flags);
  const auto wanted = expected(fixture, initial, instruction_count, cycle_count);
  auto shifted = initial;
  shifted.a <<= detail::padding_bits;
  shifted.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = shifted};
  detail::NativeContext owner{.runtime = {.memory = {},
                                          .windows = {std::span(fixture.windows.data(), 1),
                                                      std::span(&fixture.windows[1], 1)}},
                              .budget = grant == 1 ? instruction_count - 1 : instruction_count,
                              .cycle_deadline = grant == 2 ? cycle_count - 1 : cycle_count};
  const auto data = fixture.data;
  const auto known = fixture.known;
  const auto retired = detail::enter_block(native.entry, cpu, owner);
  if (input.scenario == 0 && grant == 0) {
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(retired == instruction_count && cpu.reg == wanted.reg && cpu.cycles == cycle_count &&
                cpu.instructions == instruction_count && fixture.data == wanted.data &&
                fixture.known == wanted.known,
            "Direct constant-region execution agrees with independent register and memory effects");
  } else {
    require(retired == 0 && cpu.reg == shifted && cpu.cycles == 0 && cpu.instructions == 0 &&
                fixture.data == data && fixture.known == known,
            "Constant-region admission misses have no effects, including after a computed cursor");
  }
}

std::vector<detail::BlockInstruction>
sequence(const std::span<const Word> code, const std::span<const Transfer> moves, const Word mode) {
  std::vector<detail::BlockInstruction> result;
  Word pc = first;
  for (const auto op : moves) {
    auto decoded = detail::decode(code[pc - first]);
    require(decoded.words == length(op), "Authored constant transfer lengths match decoding");
    if (decoded.words == 2) { decoded.extension = code[pc - first + 1]; }
    result.push_back({.instruction = decoded, .pc = pc, .mode = mode});
    pc += length(op);
  }
  return result;
}

void store_before_cursor() {
  Fixture fixture(false, true, 3);
  constexpr std::array moves{Transfer{.reg = 14, .at = 5, .load = false, .extended = true},
                             Transfer{.reg = 17, .at = 3},
                             Transfer{.reg = 14, .at = 1, .indirect = true}};
  const auto code = program(moves);
  const auto operations = sequence(code, moves, 0);
  auto shifted = seed(0, 0);
  shifted.a <<= detail::padding_bits;
  shifted.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = shifted};
  detail::NativeContext owner{.runtime = {.memory = {},
                                          .windows = {std::span(fixture.windows.data(), 1),
                                                      std::span(&fixture.windows[1], 1)}},
                              .budget = 3,
                              .cycle_deadline = 4};
  const auto data = fixture.data;
  const auto known = fixture.known;
  auto compiler = std::make_unique<detail::LlvmCompiler>();
  compiler->define("dsp56300_region_memory",
                   std::bit_cast<const void *>(&detail::dsp56300_region_memory));
  const auto native = detail::compile_region(compiler, operations, owner.runtime, nullptr);
  const auto retired = native ? detail::enter_block(native->entry, cpu, owner) : 0U;
  require(retired == 0 && cpu.reg == shifted && cpu.instructions == 0 && cpu.cycles == 0 &&
              fixture.data == data && fixture.known == known,
          "A later computed-cursor miss cannot replay an earlier absolute store");
}

void direct() {
  for (const Word mode : modes) {
    Fixture original(false, true, 0);
    const auto operations = sequence(original.code, transfers, mode);
    const detail::JitRuntime layout{
        .memory = {},
        .windows = {std::span(original.windows.data(), 1), std::span(&original.windows[1], 1)}};
    auto compiler = std::make_unique<detail::LlvmCompiler>();
    compiler->define("dsp56300_region_memory",
                     std::bit_cast<const void *>(&detail::dsp56300_region_memory));
    const auto native = detail::compile_region(compiler, operations, layout, nullptr);
    require(native.has_value(),
            "Constant and immediate sequence has one native region without a fallback");
    inputs(mode, [&](const Input input) {
      for (unsigned grant = 0; grant < 3; ++grant) { direct_case(*native, input, grant); }
    });
  }
}

void prefix_case(const NativeImage &image,
                 const bool writable,
                 const Input input,
                 const bool deadline,
                 const Word prefix) {
  Fixture fixture(input.alias, writable, input.scenario);
  const auto initial = seed(input.mode, input.flags);
  const auto wanted = expected(fixture,
                               initial,
                               deadline ? instruction_count : prefix,
                               deadline ? prefix : std::numeric_limits<std::uint64_t>::max());
  Core core(Memory::external(), &fixture);
  core.set_registers(initial);
  core.install(image, 0);
  const auto actual = deadline ? core.run_until(prefix) : core.run(prefix);
  const bool matches =
      core.registers() == wanted.reg && actual.fault == wanted.fault &&
      actual.retired == wanted.retired && actual.progress.instructions == wanted.retired &&
      actual.progress.estimated_cycles == wanted.cycles && fixture.data == wanted.data &&
      fixture.known == wanted.known && fixture.callbacks == (wanted.fault == Fault{} ? 0U : 1U) &&
      fixture.observed == wanted.fault;
  if (!matches) {
    std::cerr << "constant region prefix " << writable << '/' << input.mode << '/' << input.flags
              << '/' << input.alias << '/' << input.scenario << '/' << deadline << '/' << prefix
              << '\n';
  }
  require(matches,
          "Constant/immediate regions preserve every independent instruction/cycle/fault prefix "
          "and actual X/Y aliases");
}

void prefixes(const NativeImage &image, const bool writable) {
  for (const Word mode : modes) {
    inputs(mode, [&](const Input input) {
      for (Word prefix = 0; prefix <= instruction_count; ++prefix) {
        prefix_case(image, writable, input, false, prefix);
      }
      for (Word prefix = 0; prefix <= cycle_count; ++prefix) {
        prefix_case(image, writable, input, true, prefix);
      }
    });
  }
}
}

int main() try {
  store_before_cursor();
  direct();
  for (const bool writable : {false, true}) { prefixes(compile(writable), writable); }
  std::cout << "constant-region-prefixes " << dsp56300::test::checks << '\n';
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
