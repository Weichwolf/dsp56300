// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/backend.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "mac_batch.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <tuple>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word paired_mac = 0xf098d2, loop = Word{1} << 15;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array variants{
    BatchBackend::disabled, BatchBackend::scalar, BatchBackend::avx2, BatchBackend::automatic};
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr Word padding = 0xa5000000;
constexpr Word initial_count = 33, unsupported_modifier = 0x80ff;
constexpr Word ordinary_flags = flag_v | flag_z | flag_n | flag_e | flag_u;
static_assert(detail::select_mac_batch(BatchBackend::automatic, true) == BatchBackend::avx2);
static_assert(detail::select_mac_batch(BatchBackend::automatic, false) == BatchBackend::scalar);
static_assert(detail::select_mac_batch(BatchBackend::scalar, true) == BatchBackend::scalar);
static_assert(detail::select_mac_batch(BatchBackend::avx2, false) == BatchBackend::scalar);
static_assert(detail::select_mac_batch(BatchBackend::disabled, true) == BatchBackend::disabled);

std::uint64_t modulo_sum(const std::uint64_t value, const std::uint64_t amount) {
  const auto room = maximum - value;
  return amount > room ? amount - room - 1 : value + amount;
}

auto snapshot(const detail::Cpu &cpu) {
  return std::tuple{cpu.reg,
                    cpu.instructions,
                    cpu.cycles,
                    cpu.repeat_remaining,
                    cpu.repeat_pc,
                    cpu.repeat_lc,
                    cpu.repeat_opcode,
                    cpu.repeat_loaded,
                    cpu.repeat_completed,
                    cpu.interrupt_inhibited,
                    cpu.processing,
                    cpu.debug_flags};
}

struct Fixture {
  std::array<Word, 128> x{}, y{};
  detail::Cpu cpu;
  detail::NativeOp operation{
      .instruction = detail::decode(paired_mac), .pc = start, .loop_address = start};
  std::uint64_t moves = maximum - 3, batches = maximum - 2, vector = maximum - 1, calls = maximum;
  bool dirty = false;
  std::array<unsigned, 2> indices{0, 4};
  detail::NativeContext context{.runtime = {.memory = {{{.data = x.data(), .words = x.size()},
                                                        {.data = y.data(), .words = y.size()}}}},
                                .dirty = &dirty,
                                .native_moves = &moves,
                                .batch_instructions = &batches,
                                .avx2_instructions = &vector,
                                .batch_calls = &calls,
                                .budget = 8};

  Fixture() {
    cpu.reg = register_model::seed(0);
    cpu.reg.sr = loop;
    cpu.reg.la = start;
    cpu.reg.lc = initial_count;
    cpu.reg.r[0] = 0;
    cpu.reg.r[4] = 0;
    cpu.reg.m[0] = cpu.reg.m[4] = word_mask;
    cpu.reg.stack[2] = std::uint64_t{start} << word_bits;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    cpu.instructions = maximum - 3;
    cpu.cycles = maximum - 2;
    cpu.repeat_completed = true;
    cpu.interrupt_inhibited = true;
    for (std::size_t index = 0; index < x.size(); ++index) {
      x[index] = padding | (index % 2 == 0 ? sign24 - 1 : sign24);
      y[index] = padding | (sign24 - 1);
    }
  }
};

struct Step {
  Word opcode = paired_mac, next_x = 0, next_y = 0;
  std::array<unsigned, 2> indices{0, 4};
};

void scalar_step(Registers &state, const Step step) {
  const auto old = state;
  register_model::write(state, 4, step.next_x & word_mask);
  register_model::write(state, 6, step.next_y & word_mask);
  ++state.r[step.indices[0]];
  ++state.r[step.indices[1]];
  sequence_model::alu(state, old, step.opcode);
  --state.lc;
}

void direct_check(Fixture &fixture, const BatchBackend backend, const Word expected_count) {
  auto expected = fixture.cpu.reg;
  expected.a >>= detail::padding_bits;
  expected.b >>= detail::padding_bits;
  const auto before = snapshot(fixture.cpu);
  const auto before_moves = fixture.moves;
  const auto before_batches = fixture.batches;
  const auto before_vector = fixture.vector;
  const auto before_calls = fixture.calls;
  const auto before_instructions = fixture.cpu.instructions;
  const auto before_cycles = fixture.cpu.cycles;
  const auto x = fixture.x;
  const auto y = fixture.y;
  for (Word index = 0; index < expected_count; ++index) {
    const auto &ports = fixture.context.runtime.memory;
    scalar_step(expected,
                {.opcode = fixture.operation.instruction.opcode,
                 .next_x = ports[0].data[expected.r[fixture.indices[0]]],
                 .next_y = ports[1].data[expected.r[fixture.indices[1]]],
                 .indices = fixture.indices});
  }
  const auto count =
      detail::execute_mac_batch(fixture.cpu, fixture.context, fixture.operation, backend);
  require(count == expected_count, "A guarded batch retires the exact expected multiple of four");
  if (count == 0) {
    require(snapshot(fixture.cpu) == before && fixture.moves == before_moves &&
                fixture.batches == before_batches && fixture.vector == before_vector &&
                fixture.calls == before_calls,
            "Rejected/disabled batches leave every CPU field and statistic untouched");
    return;
  }
  expected.a <<= detail::padding_bits;
  expected.b <<= detail::padding_bits;
  require(fixture.cpu.reg == expected &&
              fixture.cpu.instructions == modulo_sum(before_instructions, count) &&
              fixture.cpu.cycles == modulo_sum(before_cycles, count) &&
              !fixture.cpu.repeat_completed && !fixture.cpu.interrupt_inhibited && fixture.x == x &&
              fixture.y == y,
          "Scalar/AVX2 chunks match ordered independent raw56 arithmetic and complete state");
  require(fixture.moves == modulo_sum(before_moves, count) &&
              fixture.batches == modulo_sum(before_batches, count) &&
              fixture.calls == modulo_sum(before_calls, 1),
          "Compiled bulk diagnostics count actual guest instructions with defined qword wrap");
  if (backend == BatchBackend::scalar || backend == BatchBackend::avx2) {
    const auto vector_count =
        backend == BatchBackend::avx2 && detail::avx2_mac_available() ? count : 0U;
    require(fixture.vector == modulo_sum(before_vector, vector_count),
            "Forced AVX2 reports actual vector work; unavailable features retain scalar execution");
  }
}

struct ArithmeticInput {
  BatchBackend backend;
  Word mode = 0;
  unsigned flags = 0;
  bool b = false, subtract = false;
};

void arithmetic_case(const ArithmeticInput input, const std::span<const std::uint64_t> values) {
  Fixture fixture;
  const Word opcode = paired_mac | (input.b ? 8U : 0U) | (input.subtract ? 4U : 0U);
  fixture.operation.instruction = detail::decode(opcode);
  fixture.operation.mode = input.mode;
  for (const auto value : values) {
    fixture.cpu.reg.sr = loop | input.mode | ((value & 1U) != 0 ? ordinary_flags : 0) |
                         ((input.flags & 1U) != 0 ? flag_c : 0) |
                         ((input.flags & 2U) != 0 ? flag_s : 0) |
                         ((input.flags & 4U) != 0 ? flag_l : 0);
    (input.b ? fixture.cpu.reg.b : fixture.cpu.reg.a) = value << detail::padding_bits;
    fixture.cpu.reg.x = register_model::seed_x;
    fixture.cpu.reg.y = register_model::seed_y;
    fixture.cpu.reg.r[0] = fixture.cpu.reg.r[4] = 0;
    fixture.cpu.reg.lc = initial_count;
    fixture.cpu.repeat_completed = true;
    fixture.cpu.interrupt_inhibited = true;
    direct_check(fixture, input.backend, input.backend == BatchBackend::disabled ? 0U : 8U);
  }
}

void arithmetic_mode(const ArithmeticInput base, const std::span<const std::uint64_t> values) {
  for (unsigned flags = 0; flags < 8; ++flags) {
    for (const bool b : {false, true}) {
      for (const bool subtract : {false, true}) {
        arithmetic_case({.backend = base.backend,
                         .mode = base.mode,
                         .flags = flags,
                         .b = b,
                         .subtract = subtract},
                        values);
      }
    }
  }
}

void arithmetic() {
  std::vector<std::uint64_t> values{0, 1, sign56 - 1, sign56, accumulator_mask};
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    values.push_back(std::uint64_t{1} << bit);
    values.push_back(accumulator_mask ^ (std::uint64_t{1} << bit));
  }
  for (const auto backend : variants) {
    for (const auto mode : modes) { arithmetic_mode({.backend = backend, .mode = mode}, values); }
  }
}

void boundaries() {
  for (const auto backend : variants) {
    for (Word lc = 0; lc <= 13; ++lc) {
      for (Word budget = 0; budget <= 15; ++budget) {
        for (const Word words : {0U, 3U, 4U, 7U, 8U, 11U, 12U}) {
          Fixture fixture;
          fixture.cpu.reg.lc = lc;
          fixture.context.budget = budget;
          fixture.context.runtime.memory[0].words = words;
          Word expected = 0;
          while (backend != BatchBackend::disabled && expected + 4 <= budget && expected + 4 < lc &&
                 expected + 4 <= words) {
            expected += 4;
          }
          direct_check(fixture, backend, expected);
        }
      }
    }
  }
}

void rejections() {
  const std::array mutations{
      +[](Fixture &f) { f.cpu.reg.pc = start + 1; },
      +[](Fixture &f) { f.cpu.reg.la = start + 1; },
      +[](Fixture &f) { f.cpu.reg.sr &= ~loop; },
      +[](Fixture &f) { f.cpu.reg.sr |= detail::forever_flag; },
      +[](Fixture &f) { f.cpu.reg.sr |= detail::cache_enable; },
      +[](Fixture &f) { f.cpu.reg.sr |= mode_up; },
      +[](Fixture &f) { f.cpu.repeat_remaining = 1; },
      +[](Fixture &f) { f.cpu.processing = ProcessingState::waiting; },
      +[](Fixture &f) { f.cpu.debug_flags |= detail::debug_interrupt_pending; },
      +[](Fixture &f) { f.dirty = true; },
      +[](Fixture &f) { f.cpu.reg.stack[2] = std::uint64_t{start + 1} << word_bits; },
      +[](Fixture &f) { f.cpu.reg.m[0] = 0; },
      +[](Fixture &f) { f.cpu.reg.m[4] = unsupported_modifier; },
      +[](Fixture &f) { f.cpu.reg.r[0] = detail::io_begin; },
      +[](Fixture &f) { f.context.runtime.memory[1].data = nullptr; },
      +[](Fixture &f) { f.context.native_moves = nullptr; },
      +[](Fixture &f) { f.context.batch_instructions = nullptr; },
      +[](Fixture &f) { f.context.avx2_instructions = nullptr; },
      +[](Fixture &f) { f.context.batch_calls = nullptr; },
      +[](Fixture &f) { f.cpu.reg.a |= 1; },
      +[](Fixture &f) { f.operation.instruction = detail::decode(paired_mac | 1U); },
      +[](Fixture &f) { f.operation.instruction.words = 2; },
      +[](Fixture &f) { f.operation.loop_address = start + 1; },
      +[](Fixture &f) {
        f.context.runtime.memory[0] = {.data = f.cpu.reg.r.data(), .words = agu_registers};
      }};
  for (const auto backend : variants) {
    for (const auto mutation : mutations) {
      Fixture fixture;
      mutation(fixture);
      direct_check(fixture, backend, 0);
    }
  }
}

struct AddressInput {
  BatchBackend backend;
  unsigned x = 0, y = 0, sp = 0;
  bool alias = false;
};

void address_case(const AddressInput input) {
  Fixture fixture;
  constexpr Word fields = (7U << 8) | (3U << 13);
  fixture.operation.instruction =
      detail::decode((paired_mac & ~fields) | (input.x << 8) | (input.y << 13));
  fixture.indices = {input.x, input.y + (input.x < 4 ? 4U : 0U)};
  for (const auto index : fixture.indices) {
    fixture.cpu.reg.r[index] = 0;
    fixture.cpu.reg.m[index] = word_mask;
  }
  fixture.cpu.reg.sp = input.sp;
  fixture.cpu.reg.stack[input.sp % stack_entries] = std::uint64_t{start} << word_bits;
  fixture.context.runtime.memory[0] = {.data = fixture.x.data() + 1, .words = fixture.x.size() - 1};
  fixture.context.runtime.memory[1] = {.data = (input.alias ? fixture.x : fixture.y).data() + 3,
                                       .words = fixture.y.size() - 3};
  direct_check(fixture, input.backend, input.backend == BatchBackend::disabled ? 0U : 8U);
}

void addresses(const BatchBackend backend) {
  for (unsigned x = 0; x < agu_registers; ++x) {
    for (unsigned y = 0; y < 4; ++y) {
      for (const unsigned sp : {0U, 15U, 31U, 63U}) {
        for (const bool alias : {false, true}) {
          address_case({.backend = backend, .x = x, .y = y, .sp = sp, .alias = alias});
        }
      }
    }
  }
}

void overflow_prefixes(const BatchBackend backend) {
  for (const bool terminal : {false, true}) {
    Fixture fixture;
    fixture.cpu.reg.a = (sign56 - 1) << detail::padding_bits;
    fixture.cpu.reg.x = terminal ? 0U : sign24 - 1;
    fixture.cpu.reg.y = sign24 - 1;
    fixture.x.fill(0);
    fixture.y.fill(sign24 - 1);
    if (terminal) {
      fixture.x[6] = sign24 - 1;
    } else {
      fixture.x[0] = sign24;
    }
    direct_check(fixture, backend, backend == BatchBackend::disabled ? 0U : 8U);
    if (backend != BatchBackend::disabled) {
      require((fixture.cpu.reg.sr & flag_l) != 0 &&
                  ((fixture.cpu.reg.sr & flag_v) != 0) == terminal,
              "A canceled intermediate overflow remains sticky while terminal V follows only the "
              "last MAC");
    }
  }
}

void proven_range(const BatchBackend backend) {
  constexpr auto maximum_product = 2 * std::uint64_t{sign24} * sign24;
  constexpr auto window = 4 * maximum_product;
  constexpr std::array raw{
      sign56 - window - 1, sign56 - window, sign56 + window, sign56 + window - 1};
  for (const bool subtract : {false, true}) {
    for (const auto value : raw) {
      Fixture fixture;
      fixture.context.budget = 4;
      fixture.operation.instruction = detail::decode(paired_mac | (subtract ? 4U : 0U));
      fixture.cpu.reg.a = value << detail::padding_bits;
      fixture.cpu.reg.x = sign24;
      fixture.cpu.reg.y = sign24;
      fixture.x.fill(sign24);
      fixture.y.fill(sign24);
      direct_check(fixture, backend, backend == BatchBackend::disabled ? 0U : 4U);
    }
  }
}

void reduction_boundary_case(const Word count,
                             const ArithmeticInput input,
                             const std::uint64_t value) {
  for (const Word length : {count, count + 4, count * 2}) {
    Fixture fixture;
    fixture.cpu.reg.lc = static_cast<Word>(fixture.x.size()) + 1;
    fixture.context.budget = length;
    fixture.operation.instruction =
        detail::decode(paired_mac | (input.b ? 8U : 0U) | (input.subtract ? 4U : 0U));
    (input.b ? fixture.cpu.reg.b : fixture.cpu.reg.a) = value << detail::padding_bits;
    fixture.cpu.reg.x = sign24;
    fixture.cpu.reg.y = sign24;
    fixture.x.fill(sign24);
    fixture.y.fill(sign24);
    direct_check(fixture, input.backend, input.backend == BatchBackend::disabled ? 0U : length);
  }
}

void reduction_boundaries(const BatchBackend backend) {
  constexpr auto maximum_product = 2 * std::uint64_t{sign24} * sign24;
  for (const Word count : {16U, 64U}) {
    const auto margin = count * maximum_product;
    const std::array raw{sign56 - margin - 1,
                         sign56 - margin,
                         sign56 - margin + 1,
                         sign56 + margin - 1,
                         sign56 + margin,
                         sign56 + margin + 1};
    for (const bool b : {false, true}) {
      for (const bool subtract : {false, true}) {
        for (const auto value : raw) {
          reduction_boundary_case(count, {.backend = backend, .b = b, .subtract = subtract}, value);
        }
      }
    }
  }
}

void reduction_sequences(const ArithmeticInput input) {
  constexpr Word flags = flag_c | flag_s | flag_l | ordinary_flags;
  constexpr Word almost_wide = 60;
  constexpr Word beyond_wide = 68;
  constexpr Word mixed = 124;
  constexpr std::array lengths{12U, 16U, 20U, 32U, almost_wide, 64U, beyond_wide, mixed, 128U};
  for (const auto count : lengths) {
    Fixture fixture;
    fixture.operation.instruction =
        detail::decode(paired_mac | (input.b ? 8U : 0U) | (input.subtract ? 4U : 0U));
    fixture.operation.mode = input.mode;
    fixture.cpu.reg.sr = loop | input.mode | flags;
    fixture.cpu.reg.a = fixture.cpu.reg.b = 0;
    fixture.cpu.reg.lc = static_cast<Word>(fixture.x.size()) + 1;
    fixture.context.budget = count;
    direct_check(fixture, input.backend, input.backend == BatchBackend::disabled ? 0U : count);
  }
}

void reductions(const BatchBackend backend) {
  reduction_boundaries(backend);
  for (const auto mode : modes) {
    for (const bool b : {false, true}) {
      for (const bool subtract : {false, true}) {
        reduction_sequences({.backend = backend, .mode = mode, .b = b, .subtract = subtract});
      }
    }
  }
}

void reduction_selection() {
  constexpr auto product = std::int64_t{sign24} * sign24 * 2;
  constexpr auto bound = std::int64_t{1} << (accumulator_bits - 1);
  for (const unsigned count : {16U, 64U}) {
    const auto low = -bound + (count * product);
    const auto high = bound - (count * product) - 1;
    require(detail::mac_batch_math::reduction_width(low, count) == count &&
                detail::mac_batch_math::reduction_width(high, count) == count &&
                detail::mac_batch_math::reduction_width(low - 1, count) < count &&
                detail::mac_batch_math::reduction_width(high + 1, count) < count &&
                detail::mac_batch_math::reduction_width(0, count - 1) < count,
            "Larger reduction admission includes exact safe endpoints and excludes their neighbors "
            "or an insufficient remaining span");
  }
}

template <BatchBackend Variant> void core_prefixes(const Word mode) {
  std::array<Word, memory_words> p{};
  Fixture fixture;
  BackendCore<Backend::jit, Variant> core({.program = p, .x = fixture.x, .y = fixture.y});
  core.write(Space::program, start, paired_mac);
  auto initial = register_model::seed(mode | loop);
  initial.la = start;
  initial.lc = static_cast<Word>(fixture.x.size()) + 1;
  initial.r[0] = initial.r[4] = 0;
  initial.m[0] = initial.m[4] = word_mask;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  constexpr Word last = 127;
  constexpr Word before_wide = 63;
  constexpr Word after_wide = 65;
  constexpr Word mixed = 124;
  constexpr std::array budgets{last,
                               0U,
                               1U,
                               2U,
                               3U,
                               4U,
                               5U,
                               7U,
                               8U,
                               9U,
                               15U,
                               16U,
                               17U,
                               32U,
                               before_wide,
                               64U,
                               after_wide,
                               mixed,
                               last};
  for (const Word budget : budgets) {
    auto expected = initial;
    for (Word step = 0; step < budget; ++step) {
      scalar_step(expected, {.next_x = fixture.x[step], .next_y = fixture.y[step]});
    }
    core.set_registers(initial);
    const auto before = core.progress();
    const auto batches = core.jit_statistics().batch_instructions;
    const auto result = core.run(budget);
    require(result.dispatches == budget && result.fault.kind == FaultKind::none &&
                core.registers() == expected &&
                result.progress.instructions == before.instructions + budget &&
                result.progress.estimated_cycles == before.estimated_cycles + budget,
            "Warm compiled entries retain every short budget, canonical state and scalar tail");
    require((core.jit_statistics().batch_instructions != batches) ==
                (Variant != BatchBackend::disabled && budget >= 4),
            "Integration witnesses actually take or disable the compiled batch path");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::arithmetic();
    dsp56300::test::boundaries();
    dsp56300::test::rejections();
    dsp56300::test::reduction_selection();
    for (const auto backend : dsp56300::test::variants) {
      dsp56300::test::addresses(backend);
      dsp56300::test::overflow_prefixes(backend);
      dsp56300::test::proven_range(backend);
      dsp56300::test::reductions(backend);
    }
    for (const auto mode : dsp56300::test::modes) {
      dsp56300::test::core_prefixes<dsp56300::BatchBackend::disabled>(mode);
      dsp56300::test::core_prefixes<dsp56300::BatchBackend::scalar>(mode);
      dsp56300::test::core_prefixes<dsp56300::BatchBackend::avx2>(mode);
      dsp56300::test::core_prefixes<dsp56300::BatchBackend::automatic>(mode);
    }
    std::cout << dsp56300::test::checks << " guarded MAC batch checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
