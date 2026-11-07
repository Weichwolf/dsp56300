// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "register_model.hpp"
#include "saturated_shift_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::array scales{
    0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<std::uint64_t, 10> raw{0,
                                            1,
                                            sign56 - 1,
                                            sign56,
                                            accumulator_mask,
                                            saturation_model::positive,
                                            saturation_model::positive + 1,
                                            saturation_model::negative - 1,
                                            saturation_model::negative,
                                            0x123456abcdef12};
constexpr std::array selected_flags{0U, flag_c | flag_v | flag_s, status_byte};
constexpr std::array<unsigned, 18> counts{
    0, 1, 2, 7, 23, 24, 25, 31, 32, 47, 48, 54, 55, 56, 63, 64, 127, word_mask};
constexpr Word location = word_mask;
constexpr unsigned arithmetic_count_mask = 63, logical_count_mask = 31;

struct Shape {
  Word opcode = 0, mode = 0;
  bool arithmetic = false;
  shift_model::Control shift;
  bfu_model::Input logical;
  std::optional<unsigned> control = std::nullopt;
};

std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

struct ControlWord {
  unsigned selector;
  Word value;
};

void control_word(Registers &state, const ControlWord input) {
  const auto bank = input.selector < 2 ? input.selector : (input.selector & 1U) + 2;
  const auto position = input.selector < 2 || input.selector >= 4 ? word_bits : 0U;
  const auto mask = std::uint64_t{word_mask} << position;
  const std::array targets{&state.a, &state.b, &state.x, &state.y};
  auto &target = *targets.at(bank);
  target = (target & ~mask) | (std::uint64_t{input.value} << position);
}

void check(const void *const entry, const Shape shape, Registers expected, const unsigned count) {
  if (shape.control) { control_word(expected, {.selector = *shape.control, .value = count}); }
  detail::Cpu cpu;
  cpu.reg = expected;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = cpu.cycles = std::numeric_limits<std::uint64_t>::max();
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  unsigned helpers = 0;
  detail::NativeContext context{
      .runtime = {.owner = &helpers, .execute = forbidden, .moves = forbidden, .memory = {}},
      .budget = 1};
  const auto effective = count & (shape.arithmetic ? arithmetic_count_mask : logical_count_mask);
  const bool valid = shape.arithmetic ? effective < accumulator_bits : effective <= word_bits;
  if (valid) {
    const auto old = expected;
    if (shape.arithmetic) {
      auto control = shape.shift;
      control.count = effective;
      shift_model::apply(expected, old, control);
    } else {
      auto input = shape.logical;
      input.count = effective;
      bfu_model::apply(expected, old, input);
    }
  }
  expected.pc = 0;
  expected.a <<= detail::padding_bits;
  expected.b <<= detail::padding_bits;
  const auto retired = detail::enter_block(entry, cpu, context);
  if (cpu.reg != expected) {
    std::cerr << std::hex << "opcode=" << shape.opcode << " mode=" << shape.mode
              << " count=" << count << " actual_a=" << cpu.reg.a << " expected_a=" << expected.a
              << " actual_b=" << cpu.reg.b << " expected_b=" << expected.b
              << " actual_sr=" << cpu.reg.sr << " expected_sr=" << expected.sr << std::dec << '\n';
  }
  const Fault failure{.kind = FaultKind::invalid_instruction,
                      .space = Space::program,
                      .address = location,
                      .value = effective,
                      .pc = location};
  require(
      cpu.reg == expected && retired == static_cast<unsigned>(valid) && helpers == 0 &&
          cpu.instructions == (valid ? 0 : std::numeric_limits<std::uint64_t>::max()) &&
          cpu.cycles == cpu.instructions && cpu.repeat_completed != valid &&
          !cpu.interrupt_inhibited && context.failure == (valid ? Fault{} : failure) &&
          context.boundary ==
              (valid ? detail::NativeBoundary::none : detail::NativeBoundary::owner),
      "Native shifts preserve raw-width values, all flags, counts, progress and precise faults");
}

struct Probe {
  explicit Probe(const Shape shape) {
    const std::array operations{detail::BlockInstruction{
        .instruction = detail::decode(shape.opcode), .pc = location, .mode = shape.mode}};
    detail::a64::Emitter emitter;
    entry = arena.append(emitter.generate(operations));
  }

  detail::ExecutableArena arena;
  const void *entry;
};

struct Seed {
  Word flags;
  std::uint64_t value;
};

Registers seed(const Word mode, const Seed input) {
  auto state = register_model::seed(mode);
  state.pc = location;
  state.sr = mode | input.flags;
  state.a = input.value;
  state.b = accumulator_mask ^ input.value;
  constexpr std::uint64_t x_seed = 0x123456abcdef;
  state.x = x_seed;
  constexpr std::uint64_t y_seed = 0xfedcba654321;
  state.y = y_seed;
  return state;
}

void parallel(const Word mode) {
  std::vector<std::uint64_t> values(raw.begin(), raw.end());
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const auto position = std::uint64_t{1} << bit;
    values.push_back(position);
    values.push_back(accumulator_mask ^ position);
  }
  for (const bool destination_b : {false, true}) {
    for (const bool left : {false, true}) {
      const shift_model::Control control{
          .left = left, .source_b = destination_b, .destination_b = destination_b};
      const Shape shape{.opcode = shift_model::parallel(control),
                        .mode = mode,
                        .arithmetic = true,
                        .shift = control,
                        .logical = {}};
      const Probe probe(shape);
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (const auto value : values) {
          check(probe.entry, shape, seed(mode, {.flags = flags, .value = value}), 1);
        }
      }
    }
  }
}

void immediate(const Word mode) {
  for (const bool source_b : {false, true}) {
    for (const bool destination_b : {false, true}) {
      for (const bool left : {false, true}) {
        for (unsigned count = 0; count <= arithmetic_count_mask; ++count) {
          const shift_model::Control control{.count = count,
                                             .left = left,
                                             .source_b = source_b,
                                             .destination_b = destination_b,
                                             .saturate = false};
          const Shape shape{.opcode = shift_model::immediate(control),
                            .mode = mode,
                            .arithmetic = true,
                            .shift = control,
                            .logical = {}};
          const Probe probe(shape);
          for (const auto flags : selected_flags) {
            for (const auto value : raw) {
              check(probe.entry, shape, seed(mode, {.flags = flags, .value = value}), count);
            }
          }
        }
      }
    }
  }
}

void register_shift(const Word mode, const bool source_b, const bool destination_b) {
  for (unsigned source = 0; source < bfu_model::controls.size(); ++source) {
    for (const bool left : {false, true}) {
      constexpr Word right = 0x0c1e60;
      constexpr Word left_opcode = 0x0c1e40;
      const auto opcode = (left ? left_opcode : right) | (source_b ? 16U : 0U) |
                          ((source + 2U) * 2) | (destination_b ? 1U : 0U);
      const Shape shape{.opcode = opcode,
                        .mode = mode,
                        .arithmetic = true,
                        .shift = {.left = left,
                                  .source_b = source_b,
                                  .destination_b = destination_b,
                                  .saturate = false},
                        .logical = {},
                        .control = source};
      const Probe probe(shape);
      for (const auto count : counts) {
        for (const auto flags : selected_flags) {
          for (const auto value : raw) {
            check(probe.entry, shape, seed(mode, {.flags = flags, .value = value}), count);
          }
        }
      }
    }
  }
}

void logical_case(const Word mode, const bfu_model::Input input) {
  const bool register_count = input.form == bfu_model::Form::shift_register;
  const Shape shape{.opcode = bfu_model::opcode(input),
                    .mode = mode,
                    .shift = {},
                    .logical = input,
                    .control = register_count
                                   ? std::optional{bfu_model::control_codes.at(input.source) - 2U}
                                   : std::nullopt};
  const Probe probe(shape);
  const auto exercise = [&](const unsigned count) {
    for (const auto flags : selected_flags) {
      for (const auto value : raw) {
        check(probe.entry, shape, seed(mode, {.flags = flags, .value = value}), count);
      }
    }
  };
  if (register_count) {
    for (const auto count : counts) { exercise(count); }
  } else {
    exercise(input.count);
  }
}

struct FailureCase {
  Word opcode = 0, mode = 0, value = 0;
  FaultKind kind = FaultKind::invalid_instruction;
};

void failure_prefix(const FailureCase point) {
  const std::array operations{
      detail::BlockInstruction{.instruction = detail::decode(0), .pc = start, .mode = point.mode},
      detail::BlockInstruction{
          .instruction = detail::decode(point.opcode), .pc = start + 1, .mode = point.mode},
      detail::BlockInstruction{
          .instruction = detail::decode(0), .pc = start + 2, .mode = point.mode}};
  detail::a64::Emitter emitter;
  detail::ExecutableArena arena;
  const auto *entry = arena.append(emitter.generate(operations));
  for (const Word budget : {0U, 1U, 2U, 3U, 32U}) {
    auto expected = seed(point.mode, {.flags = status_byte, .value = sign56});
    expected.pc = start;
    detail::Cpu cpu;
    cpu.reg = expected;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    cpu.instructions = cpu.cycles = std::numeric_limits<std::uint64_t>::max();
    cpu.repeat_completed = cpu.interrupt_inhibited = true;
    unsigned helpers = 0;
    detail::NativeContext context{
        .runtime = {.owner = &helpers, .execute = forbidden, .moves = forbidden, .memory = {}},
        .budget = budget};
    const bool failed = budget >= 2;
    const auto count = budget != 0 ? 1U : 0U;
    expected.pc += failed ? 2 : count;
    expected.a <<= detail::padding_bits;
    expected.b <<= detail::padding_bits;
    const Fault failure{.kind = point.kind,
                        .space = Space::program,
                        .address = start + 1,
                        .value = point.value,
                        .pc = start + 1};
    require(
        detail::enter_block(entry, cpu, context) == count && cpu.reg == expected && helpers == 0 &&
            cpu.instructions == (count != 0 ? 0 : std::numeric_limits<std::uint64_t>::max()) &&
            cpu.cycles == cpu.instructions && cpu.repeat_completed == (count == 0) &&
            cpu.interrupt_inhibited == (count == 0) &&
            context.failure == (failed ? failure : Fault{}) &&
            context.boundary ==
                (failed ? detail::NativeBoundary::owner : detail::NativeBoundary::none),
        "Native numeric/count faults preserve their committed prefix and stop before later work");
  }
}

void faults() {
  constexpr unsigned invalid_arithmetic_count = 56;
  constexpr unsigned invalid_logical_count = 25;
  failure_prefix({.opcode = shift_model::immediate({.count = invalid_arithmetic_count}),
                  .value = invalid_arithmetic_count});
  failure_prefix({.opcode = bfu_model::opcode({.function = bfu_model::Function::right,
                                               .form = bfu_model::Form::shift_immediate,
                                               .count = invalid_logical_count}),
                  .value = invalid_logical_count});
  constexpr std::array<Word, 4> register_forms{0x0c1e40, 0x0c1e60, 0x0c1e10, 0x0c1e30};
  for (const auto form : register_forms) {
    for (const Word selector : {0U, 1U}) {
      const auto opcode = form | (selector * 2);
      failure_prefix({.opcode = opcode, .value = opcode});
    }
  }
  constexpr std::array modes{detail::compatibility_mode,
                             detail::multiply_mode,
                             detail::arithmetic_16,
                             detail::scale_up | detail::scale_down};
  for (const auto mode : modes) {
    failure_prefix({.opcode = shift_model::parallel({}),
                    .mode = mode,
                    .value = mode | status_byte,
                    .kind = FaultKind::unsupported_mode});
  }
}

void logical(const Word mode) {
  for (const bool destination_b : {false, true}) {
    for (const auto function : {bfu_model::Function::left,
                                bfu_model::Function::right,
                                bfu_model::Function::rotate_left,
                                bfu_model::Function::rotate_right}) {
      logical_case(mode, {.function = function, .destination_b = destination_b});
      if (function >= bfu_model::Function::rotate_left) { continue; }
      for (unsigned count = 0; count <= logical_count_mask; ++count) {
        logical_case(mode,
                     {.function = function,
                      .form = bfu_model::Form::shift_immediate,
                      .count = count,
                      .destination_b = destination_b});
      }
      for (unsigned source = 0; source < bfu_model::controls.size(); ++source) {
        logical_case(mode,
                     {.function = function,
                      .form = bfu_model::Form::shift_register,
                      .source = source,
                      .destination_b = destination_b});
      }
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (const auto scale : scales) {
      for (const auto saturation : {0U, saturation_model::mode}) {
        const auto mode = scale | saturation;
        parallel(mode);
        immediate(mode);
        logical(mode);
        for (const bool source_b : {false, true}) {
          for (const bool destination_b : {false, true}) {
            register_shift(mode, source_b, destination_b);
          }
        }
      }
    }
    faults();
    std::cout << "Native A64 shifts, rotates and precise count faults passed: " << checks
              << " cases\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
