// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "architecture.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/resident.hpp"
#include "loop_frames.hpp"
#include "native/direct.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "translation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::size_t words = 128;
constexpr Word lf = 1U << 15, wrap = 0x8000;
constexpr Word single_base = 0x404000, high_bits = 0xab000000, multiplier = 7919;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

struct Move {
  Word index = 0, mode = 3;
  bool y = false, load = true;
};

constexpr std::array moves{Move{.mode = 1},
                           Move{.index = 4, .y = true},
                           Move{.index = 1, .mode = 2, .load = false},
                           Move{.mode = 7},
                           Move{.index = 4, .mode = 1, .y = true, .load = false},
                           Move{.mode = 4},
                           Move{.index = 4, .mode = 2, .y = true},
                           Move{.index = 1}};

Word opcode(const Move move) {
  const Word reg = move.y ? 6 : 4;
  return single_base | (reg << 16) | (move.y ? 0x080000U : 0U) | (move.load ? 0x8000U : 0U) |
         (move.mode << 11) | (move.index << 8);
}

struct Memory {
  std::array<Word, words * 2> data{};
  std::size_t limit = words;
  bool alias = false;

  std::span<Word> bank(const bool y) {
    return std::span(data).subspan(y && !alias ? words : 0, limit);
  }

  void initialize() {
    for (Word index = 0; index < data.size(); ++index) {
      data[index] = high_bits | (multiplier * index);
    }
  }

  detail::JitRuntime runtime() {
    return {.memory = {{{.data = bank(false).data(), .words = limit},
                        {.data = bank(true).data(), .words = limit}}}};
  }
};

std::vector<detail::BlockInstruction> operations(const Word mode, const bool loop) {
  std::vector<detail::BlockInstruction> result;
  for (const auto move : moves) {
    const auto instruction = detail::decode(opcode(move));
    require(instruction.operation == detail::Operation::parallel && instruction.words == 1 &&
                instruction.cycles == (move.mode == 7 ? 2U : 1U),
            "Independent ring move encodings have declared widths and costs");
    result.push_back({.instruction = instruction,
                      .pc = start + static_cast<Word>(result.size()),
                      .mode = mode,
                      .loop_address = loop ? start + static_cast<Word>(moves.size()) - 1
                                           : detail::inactive_loop});
  }
  return result;
}

Registers initial(const Word mode, const bool loop, const Word mask) {
  auto state = register_model::seed(mode);
  state.sr = mode | (loop ? lf : 0U);
  state.m.fill(wrap | mask);
  state.r[0] = 0;
  state.r[1] = 32;
  state.r[4] = 64;
  state.la = start + moves.size() - 1;
  state.lc = 5;
  state.stack[state.sp % stack_entries] = std::uint64_t{start} << word_bits;
  return state;
}

void execute(Registers &state, Memory &memory, const Move move) {
  const auto before = state.r[move.index];
  const auto after = address_model::updated({.modifier = state.m[move.index],
                                             .cursor = before,
                                             .offset = state.n[move.index],
                                             .mode = move.mode});
  const auto address = move.mode == 7 ? after : before;
  const Word reg = move.y ? 6 : 4;
  if (move.load) {
    register_model::write(state, reg, memory.bank(move.y)[address] & word_mask);
  } else {
    memory.bank(move.y)[address] = register_model::read(state, reg);
  }
  if (move.mode != 4) { state.r[move.index] = after; }
  ++state.pc;
}

bool admitted(const Registers &state, const Word mask, const bool loop, const Memory &memory) {
  if (loop && !loop_frames::admitted(state)) { return false; }
  const auto length = mask + 1;
  return std::ranges::all_of(std::array{0U, 1U, 4U}, [&](const auto index) {
    if (state.m[index] % 0x10000 != (wrap | mask)) { return false; }
    const auto end = (std::uint64_t{state.r[index]} / length + 1) * length;
    return end <= memory.limit;
  });
}

void direct_case(const native::Direct &code,
                 Memory &actual,
                 Registers expected,
                 const Word mask,
                 const bool loop,
                 const Word budget) {
  actual.initialize();
  auto memory = actual;
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = cpu.cycles = maximum - 2;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  Word retired = 0;
  Word cycles = 0;
  while (budget - retired >= moves.size() && admitted(expected, mask, loop, memory)) {
    for (const auto move : moves) {
      execute(expected, memory, move);
      cycles += move.mode == 7 ? 2U : 1U;
    }
    retired += moves.size();
    if (!loop) { break; }
    if (expected.lc == 1) {
      loop_frames::complete(expected);
      break;
    }
    --expected.lc;
    expected.pc = start;
  }
  detail::NativeContext context{.runtime = {}, .budget = budget};
  const auto result = code.enter(cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(result == retired && cpu.reg == expected && actual.data == memory.data &&
              cpu.instructions == maximum - 2 + retired && cpu.cycles == maximum - 2 + cycles &&
              cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0),
          "Guarded rings preserve full state, memory, DO frames and overflowing 64-bit progress");
}

void direct_matrix(const Word mode, const bool loop, const Word mask, const bool alias) {
  constexpr Word grant = 3 * moves.size();
  for (const std::size_t limit : {words, words - 1}) {
    Memory memory{.limit = limit, .alias = alias};
    const auto state = initial(mode, loop, mask);
    const auto ops = operations(mode, loop);
    require(!detail::resident_plan(ops, memory.runtime().memory, state.m).valid &&
                !detail::resident_plan(
                     ops, memory.runtime().memory, state.m, detail::RegionAddressing::bidirectional)
                     .valid,
            "Existing backend capabilities keep their previous admission");
    detail::a64::Emitter emitter;
    const auto bytes = emitter.generate_guarded(ops, memory.runtime(), state.m);
    if (mask + 1 > limit) {
      require(!bytes, "A whole ring larger than the grant cannot enter guarded execution");
      continue;
    }
    if (!bytes) {
      throw std::runtime_error("Multiple-wrap admission: mode=" + std::to_string(mode) +
                               " mask=" + std::to_string(mask) + " limit=" + std::to_string(limit));
    }
    const native::Direct code(bytes.value());
    for (Word flags = 0; flags <= status_byte; ++flags) {
      auto changed = state;
      changed.sr |= flags;
      changed.n[0] = flags;
      changed.n[4] = word_mask - flags;
      direct_case(code, memory, changed, mask, loop, grant);
    }
    for (const Word cursor :
         {0U, 1U, 15U, 16U, 63U, 64U, 126U, 127U, 128U, word_mask, 0xffffffffU}) {
      for (const Word stride : {0U, 1U, 16U, 127U, sign24 - 1, sign24, word_mask, 0xffffffffU}) {
        auto changed = state;
        changed.r[0] = changed.r[4] = cursor;
        changed.n[0] = changed.n[4] = stride;
        direct_case(code, memory, changed, mask, loop, grant);
      }
    }
    for (const Word lc : {0U, 1U, 2U, 3U, 5U, word_mask}) {
      for (Word budget = 0; budget <= 3 * moves.size(); ++budget) {
        auto changed = state;
        changed.lc = lc;
        direct_case(code, memory, changed, mask, loop, budget);
      }
    }
    for (const Word modifier :
         {0U, mask, wrap, wrap | mask, 0xc003U, 0xab8000U | mask, word_mask}) {
      auto changed = state;
      changed.m[0] = modifier;
      direct_case(code, memory, changed, mask, loop, grant);
    }
  }
}

void prefixes(const Word mode, const Word mask, const bool alias) {
  Memory memory{.alias = alias};
  std::array<Word, memory_words> p{};
  TestCore core({.program = p, .x = memory.bank(false), .y = memory.bank(true)},
                nullptr,
                {.backend = Backend::jit});
  std::vector<Word> program;
  program.reserve(moves.size());
  for (const auto move : moves) { program.push_back(opcode(move)); }
  core.load(Space::program, start, program);
  const auto state = initial(mode, false, mask);
  for (Word budget = 0; budget <= moves.size(); ++budget) {
    memory.initialize();
    auto expected_memory = memory;
    auto expected = state;
    std::uint64_t cycles = 0;
    for (const auto move : std::span(moves).first(budget)) {
      execute(expected, expected_memory, move);
      cycles += move.mode == 7 ? 2U : 1U;
    }
    core.set_registers(state);
    const auto progress = core.progress();
    const auto run = core.run(budget);
    require(run.retired == budget && run.fault.kind == FaultKind::none &&
                core.registers() == expected && memory.data == expected_memory.data &&
                run.progress == Progress{.instructions = progress.instructions + budget,
                                         .estimated_cycles = progress.estimated_cycles + cycles},
            "Ordinary fallback preserves every partial public grant through wrapped transfers");
  }
}

void rejected() {
  Memory memory;
  const auto ops = operations(0, false);
  for (const Word modifier : {wrap, 0x8002U, 0x8005U, 0xc003U, 0xbfffU}) {
    auto state = initial(0, false, 15);
    state.m[0] = modifier;
    require(!detail::resident_plan(
                 ops, memory.runtime().memory, state.m, detail::RegionAddressing::masked_rings)
                 .valid,
            "Invalid encodings and rings larger than the RAM grant retain ordinary execution");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::rejected();
    for (const auto mode : dsp56300::test::modes) {
      for (const auto mask : {1U, 3U, 7U, 15U, 127U}) {
        for (const bool alias : {false, true}) {
          for (const bool loop : {false, true}) {
            dsp56300::test::direct_matrix(mode, loop, mask, alias);
          }
          dsp56300::test::prefixes(mode, mask, alias);
        }
      }
    }
    std::cout << dsp56300::test::checks << " independent native ring checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
