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
#include <stdexcept>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word first = 0x100, mask = 0xffffff;
constexpr Word bit_prefix = 0x0a0000, absolute_ea = 0x7000, stop = 0x87, host_bits = 0xab000000;
constexpr std::array modes{0U,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm,
                           test::mode_rm | test::mode_down,
                           test::mode_rm | test::mode_up};
constexpr std::array actions{BitAction::test,
                             BitAction::clear,
                             BitAction::set,
                             BitAction::set,
                             BitAction::change,
                             BitAction::test,
                             BitAction::change,
                             BitAction::clear,
                             BitAction::clear};

struct Operation {
  BitAction action;
  Word at, bit;
  bool y, extended;
};

std::vector<Operation> operations() {
  std::vector<Operation> result;
  for (Word bit = 0; bit < test::word_bits; ++bit) {
    for (unsigned i = 0; i < actions.size(); ++i) {
      result.push_back({.action = actions[i],
                        .at = i % 3 == 0 ? 2U : 7U,
                        .bit = bit,
                        .y = (i & 1) != 0,
                        .extended = (i & 2) != 0});
    }
  }
  return result;
}

Word length(const Operation op) {
  return op.extended ? 2U : 1U;
}

Word cycles(const Operation op) {
  return op.extended ? 3U : 2U;
}

Word modified(const Word original, const Operation op) {
  const auto bit = Word{1} << op.bit;
  switch (op.action) {
    case BitAction::clear: return (original & ~bit) & mask;
    case BitAction::set: return (original | bit) & mask;
    case BitAction::change: return (original ^ bit) & mask;
    case BitAction::test: return original;
    default: throw std::logic_error("Invalid authored bit action");
  }
}

std::vector<Word> program(const std::span<const Operation> ops) {
  std::vector<Word> result;
  for (const auto op : ops) {
    const bool upper = op.action == BitAction::change || op.action == BitAction::test;
    const bool low = op.action == BitAction::set || op.action == BitAction::test;
    result.push_back(bit_prefix | (upper ? 1U << 16 : 0U) | (low ? 1U << 5 : 0U) |
                     (op.y ? 1U << 6 : 0U) | (op.extended ? absolute_ea : op.at << 8) | op.bit);
    if (op.extended) { result.push_back(op.at); }
  }
  result.push_back(stop);
  return result;
}

using Data = std::array<Word, 16>;
using Known = std::array<std::uint8_t, 16>;

struct Input {
  Word mode = 0, flags = 0, datum = 0;
  bool alias = false;
  unsigned scenario = 0;
};

struct Fixture final : Io {
  Fixture(const std::span<const Operation> ops, const Input input, const bool writable)
      : code(program(ops)), alias(input.alias) {
    for (unsigned bank = 0; bank < 2; ++bank) {
      data[bank].fill(host_bits | input.datum);
      known[bank].fill(1);
    }
    if (input.scenario == 1) { known[0][2] = 0; }
    if (input.scenario == 2) { known[0][7] = 0; }
    if (input.scenario == 3) { known[alias ? 0U : 1U][2] = 0; }
    if (input.scenario == 4) { known[alias ? 0U : 1U][7] = 0; }
    if (input.scenario == 5) {
      known[0][2] = 2;
      known[0][7] = 2;
    }
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
    return space == Space::program ? std::span<const MemoryWindow>{}
                                   : std::span(&windows[space == Space::x ? 0U : 1U], 1);
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
  Fault fault;
  Word retired = 0;
  std::uint64_t cycles = 0;
};

bool step(Model &model, const Fixture &fixture, const Operation op, const Word pc) {
  const auto bank = fixture.alias || !op.y ? 0U : 1U;
  const auto space = op.y ? Space::y : Space::x;
  if (model.known[bank][op.at] == 0) {
    model.fault = {.kind = FaultKind::memory_read, .space = space, .address = op.at, .pc = pc};
    return false;
  }
  const auto before = model.data[bank][op.at] & mask;
  if (op.action != BitAction::test) {
    const auto value = modified(before, op);
    if (!fixture.windows[op.y ? 1U : 0U].writable) {
      model.fault = {.kind = FaultKind::memory_write,
                     .space = space,
                     .address = op.at,
                     .value = value,
                     .pc = pc};
      return false;
    }
    model.data[bank][op.at] = value;
    model.known[bank][op.at] = 1;
  }
  model.reg.sr = (model.reg.sr & ~Word{1}) | ((before >> op.bit) & 1U);
  return true;
}

Model expected(const Fixture &fixture,
               const std::span<const Operation> ops,
               const Registers &initial,
               const Word budget,
               const std::uint64_t deadline) {
  Model result{.reg = initial, .data = fixture.data, .known = fixture.known, .fault = {}};
  for (const auto op : ops) {
    if (result.retired == budget || result.cycles >= deadline) { break; }
    const auto pc = result.reg.pc;
    result.reg.pc += length(op);
    if (!step(result, fixture, op, pc)) { break; }
    ++result.retired;
    result.cycles += cycles(op);
  }
  return result;
}

NativeImage compile(const std::span<const Operation> ops, const bool writable) {
  Fixture fixture(ops, {}, writable);
  const std::array code{CodeSpan{.address = first, .words = fixture.code}};
  const std::array entries{first};
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
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

template <typename Function> void inputs(const Word mode, Function &&function) {
  for (const Word flags : {0U, test::status_byte}) {
    for (const bool alias : {false, true}) {
      for (const Word datum : {0U, mask}) {
        for (unsigned scenario = 0; scenario < 6; ++scenario) {
          function(Input{
              .mode = mode, .flags = flags, .datum = datum, .alias = alias, .scenario = scenario});
        }
      }
    }
  }
}

Word cost(const std::span<const Operation> ops) {
  Word result = 0;
  for (const auto op : ops) { result += cycles(op); }
  return result;
}

void check(const NativeImage &image,
           const std::span<const Operation> ops,
           const bool writable,
           const Input input,
           const bool deadline,
           const Word prefix) {
  Fixture fixture(ops, input, writable);
  const auto initial = seed(input.mode, input.flags);
  const auto wanted = expected(fixture,
                               ops,
                               initial,
                               deadline ? static_cast<Word>(ops.size()) : prefix,
                               deadline ? prefix : std::numeric_limits<std::uint64_t>::max());
  Core core(Memory::external(), &fixture);
  core.set_registers(initial);
  core.install(image, 0);
  const auto actual = deadline ? core.run_until(prefix) : core.run(prefix);
  const bool match =
      core.registers() == wanted.reg && actual.fault == wanted.fault &&
      actual.retired == wanted.retired && actual.progress.instructions == wanted.retired &&
      actual.progress.estimated_cycles == wanted.cycles && fixture.data == wanted.data &&
      fixture.known == wanted.known && fixture.observed == wanted.fault &&
      fixture.callbacks == (wanted.fault == Fault{} ? 0U : 1U);
  if (!match) {
    std::cerr << "bit-prefix " << writable << '/' << input.mode << '/' << input.flags << '/'
              << input.alias << '/' << input.datum << '/' << input.scenario << '/' << deadline
              << '/' << prefix << " retired=" << actual.retired << '/' << wanted.retired
              << " cycles=" << actual.progress.estimated_cycles << '/' << wanted.cycles
              << " regs=" << (core.registers() == wanted.reg)
              << " fault=" << (actual.fault == wanted.fault) << '\n';
  }
  require(match, "RAM bit regions preserve independent values/carry and every observable prefix");
}

void direct_case(const detail::PreparedRegion &native,
                 const std::span<const Operation> ops,
                 const Input input) {
  Fixture fixture(ops, input, true);
  const auto initial = seed(input.mode, input.flags);
  const auto wanted = expected(fixture, ops, initial, static_cast<Word>(ops.size()), cost(ops));
  auto shifted = initial;
  shifted.a <<= detail::padding_bits;
  shifted.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = shifted};
  detail::NativeContext owner{.runtime = {.memory = {},
                                          .windows = {std::span(fixture.windows.data(), 1),
                                                      std::span(&fixture.windows[1], 1)}},
                              .budget = static_cast<Word>(ops.size()),
                              .cycle_deadline = cost(ops)};
  const auto data = fixture.data;
  const auto known = fixture.known;
  const auto retired = detail::enter_block(native.entry, cpu, owner);
  if (input.scenario == 0) {
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(retired == ops.size() && cpu.reg == wanted.reg && cpu.instructions == ops.size() &&
                cpu.cycles == wanted.cycles && fixture.data == wanted.data &&
                fixture.known == wanted.known,
            "All RAM bit actions execute inside the direct native region without ISA calls");
  } else {
    require(retired == 0 && cpu.reg == shifted && cpu.instructions == 0 && cpu.cycles == 0 &&
                fixture.data == data && fixture.known == known,
            "RAM bit region guard misses have no register, carry or memory effects");
  }
}

void direct(const std::span<const Operation> ops) {
  for (const Word mode : modes) {
    Fixture fixture(ops, {}, true);
    std::vector<detail::BlockInstruction> sequence;
    Word pc = first;
    for (const auto op : ops) {
      auto instruction = detail::decode(fixture.code[pc - first]);
      require(instruction.words == length(op) && instruction.cycles == cycles(op),
              "Authored RAM bit lengths and base periods agree with the independent timing model");
      if (instruction.words == 2) { instruction.extension = op.at; }
      sequence.push_back({.instruction = instruction, .pc = pc, .mode = mode});
      pc += length(op);
    }
    const detail::JitRuntime runtime{
        .memory = {},
        .windows = {std::span(fixture.windows.data(), 1), std::span(&fixture.windows[1], 1)}};
    auto compiler = std::make_unique<detail::LlvmCompiler>();
    compiler->define("dsp56300_region_memory",
                     std::bit_cast<const void *>(&detail::dsp56300_region_memory));
    const auto native = detail::compile_region(compiler, sequence, runtime, nullptr);
    require(native.has_value(), "Every supported numeric mode has a direct RAM bit region");
    inputs(mode, [&](const Input input) { direct_case(*native, ops, input); });
  }
}

void run(const std::span<const Operation> ops) {
  direct(ops);
  const auto total = cost(ops);
  for (const bool writable : {false, true}) {
    const auto image = compile(ops, writable);
    for (const Word mode : modes) {
      inputs(mode, [&](const Input input) {
        for (Word prefix = 0; prefix <= ops.size(); ++prefix) {
          check(image, ops, writable, input, false, prefix);
        }
        for (Word prefix = 0; prefix <= total; ++prefix) {
          check(image, ops, writable, input, true, prefix);
        }
      });
    }
  }
}
}

int main() try {
  const auto ops = operations();
  run(ops);
  std::cout << "bit-region-checks " << test::checks << '\n';
} catch (const std::exception &e) {

  std::cerr << e.what() << '\n';
  return 1;
}
