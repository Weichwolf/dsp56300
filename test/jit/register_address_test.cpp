// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "native/direct.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using enum isa::Opcode;
constexpr unsigned first_address = 16;
constexpr Word io_begin = 0xffff80;
constexpr Word register_fields = (31U << 13) | (31U << 8) | status_byte;
constexpr Word register_move = static_cast<Word>(move_register_x0_x1) & ~register_fields;
constexpr std::array registers{4U,  5U,  6U,  7U,  14U, 15U, 16U, 17U, 18U, 19U, 20U,
                               21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U};
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 6> accumulators{
    0, 1, sign56, sign56 - 1, accumulator_mask, 0x007fffff800000};

struct Input {
  unsigned source = 4, destination = first_address;
  Word alu = 0, mode = 0;
};

Word encoding(const Input input) {
  return register_move | (input.source << 13) | (input.destination << 8) | input.alu;
}

Registers initial(const std::size_t seed, const Word mode) {
  auto result = register_model::seed(mode);
  result.pc = 0;
  result.a = accumulators[seed];
  result.b = accumulators[(seed + 1) % accumulators.size()];
  result.x = (accumulators[(seed + 2) % accumulators.size()] ^ register_model::seed_x) & long_mask;
  result.y = (accumulators[(seed + 4) % accumulators.size()] ^ register_model::seed_y) & long_mask;
  result.sr = mode | (seed % 2 == 0 ? 0U : status_byte);
  result.m.fill(word_mask);
  for (unsigned i = 0; i < agu_registers; ++i) {
    result.r[i] = static_cast<Word>((result.x >> (i % 2 * word_bits)) + i) & word_mask;
    result.n[i] = static_cast<Word>((result.y >> (i % 2 * word_bits)) - i) & word_mask;
  }
  return result;
}

Registers expected(const Registers &old, const Input input) {
  auto result = old;
  const auto value = register_model::read(result, input.source);
  register_model::write(result, input.destination, value);
  sequence_model::alu(result, old, input.alu);
  ++result.pc;
  return result;
}

struct Probe {
  unsigned calls = 0;
};

std::uint32_t forbidden(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
  ++static_cast<Probe *>(owner)->calls;
  return 0;
}

void check(Core &core,
           const native::Direct &code,
           const Input input,
           const std::size_t seed,
           Probe &probe) {
  const auto old = initial(seed, input.mode);
  const auto wanted = expected(old, input);
  core.set_registers(old);
  const auto before = core.progress();
  const auto result = core.run(1);
  require(result.retired == 1 && result.fault == Fault{} && core.registers() == wanted &&
              result.progress == Progress{.instructions = before.instructions + 1,
                                          .estimated_cycles = before.estimated_cycles + 1},
          "Cold/warm public register copies preserve full state, flags and old parallel operands");
  auto shifted = old;
  shifted.a <<= detail::padding_bits;
  shifted.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = shifted};
  std::array<std::uint64_t, 4> latched{};
  bool exit = false;
  detail::NativeContext context{.runtime = {.owner = &probe,
                                            .latched = &latched,
                                            .execute = forbidden,
                                            .moves = forbidden,
                                            .memory = {}},
                                .exit_requested = &exit,
                                .budget = 1};
  const auto retired = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  if (probe.calls != 0 || cpu.reg != wanted) {
    std::cerr << "source/destination/alu/mode=" << input.source << '/' << input.destination << '/'
              << input.alu << '/' << input.mode << " calls=" << probe.calls << '\n';
  }
  require(retired == 1 && probe.calls == 0 && cpu.reg == wanted && cpu.instructions == 1 &&
              cpu.cycles == 1,
          "R/N register copies execute native ISA without transfer or executor helpers");
}

void transfer_case(const Input input) {
  std::array<Word, 3> program{
      encoding(input), static_cast<Word>(jmp_absolute_short) | 2, static_cast<Word>(nop)};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  Probe probe;
  const detail::JitRuntime runtime{
      .owner = &probe, .execute = forbidden, .moves = forbidden, .memory = {}};
  const std::array operations{
      detail::BlockInstruction{.instruction = detail::decode(encoding(input)), .mode = input.mode}};
  const native::Direct code(detail::emit_block(operations, runtime, initial(0, input.mode).m));
  for (std::size_t seed = 0; seed < accumulators.size(); ++seed) {
    check(core, code, input, seed, probe);
  }
  require(x == std::array<Word, 1>{} && y == x, "Register transfers leave borrowed RAM untouched");
}

void matrix() {
  for (const auto source : registers) {
    for (const auto destination : registers) {
      if (source < first_address && destination < first_address) { continue; }
      const auto add = destination == 14 ? add_a_b : add_b_a;
      const auto compare = destination == 14 ? cmp_x0_b : cmp_x0_a;
      const std::array<Word, 3> operations{
          0, static_cast<Word>(add) & status_byte, static_cast<Word>(compare) & status_byte};
      for (const auto mode : modes) {
        for (const auto alu : operations) {
          transfer_case({.source = source, .destination = destination, .alu = alu, .mode = mode});
        }
      }
    }
  }
}

void changed_cursor(const Word address) {
  constexpr Word entry = 8;
  constexpr Word loads = 8;
  constexpr Word lf = 1U << 15;
  constexpr Word load = static_cast<Word>(move_x_effective) | (1U << 15) | (3U << 11);
  std::array<Word, 64> p{};
  std::array<Word, 64> x{};
  std::array<Word, 64> y{};
  for (Word i = 0; i < loads; ++i) { p[entry + i] = load; }
  p[entry + loads] = encoding({.source = 5, .destination = first_address});
  p[entry + loads + 1] = static_cast<Word>(move_x_effective) | (1U << 15) | (4U << 11);
  for (std::size_t i = 0; i < x.size(); ++i) { x[i] = static_cast<Word>(i + 1); }
  Core core(Memory::separate({.program = p, .x = x, .y = y}));
  auto old = initial(0, 0);
  old.pc = entry;
  old.sr = lf;
  old.la = entry + loads + 1;
  old.lc = 2;
  old.r[0] = 0;
  old.x = std::uint64_t{address} << word_bits;
  old.stack[2] = std::uint64_t{entry} << word_bits;
  core.set_registers(old);
  constexpr std::array entries{entry};
  require(core.prepare(entries).prepared_entries == 0,
          "Cursor replacement cannot enter a loop optimizer with stale hoisted RAM bounds");
  auto wanted = old;
  wanted.x |= loads;
  wanted.r[0] = address;
  wanted.pc = old.la;
  const auto result = core.run(loads + 2);
  if (address < x.size()) {
    wanted.x = (wanted.x & ~std::uint64_t{word_mask}) | x[address];
    wanted.pc = entry;
    --wanted.lc;
    require(result.fault == Fault{} && result.retired == loads + 2,
            "Following native RAM access uses the replaced address");
  } else {
    const Fault fault{
        .kind = FaultKind::io_read, .space = Space::x, .address = io_begin, .pc = wanted.pc};
    require(result.fault == fault && result.retired == loads + 1,
            "A cursor changed to I/O retains the exact real fault after its retired prefix");
    ++wanted.pc;
  }
  const auto actual = core.registers();
  if (actual != wanted) {
    std::cerr << "cursor=" << address << " wanted PC/LC/R0/X/SR=" << wanted.pc << '/' << wanted.lc
              << '/' << wanted.r[0] << '/' << wanted.x << '/' << wanted.sr
              << " actual=" << actual.pc << '/' << actual.lc << '/' << actual.r[0] << '/'
              << actual.x << '/' << actual.sr << '\n';
  }
  require(actual == wanted, "Cursor replacement retains all other architectural state");
}
}

int main() try {
  matrix();
  changed_cursor(16);
  changed_cursor(io_begin);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
