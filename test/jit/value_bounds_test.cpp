// SPDX-License-Identifier: GPL-3.0-only
// Public execution versus raw 56-bit arithmetic and quotient/remainder rounding.
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"
#if defined(__aarch64__)
#include "architecture.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "jit/resident_region.hpp"

#include <memory>
#include <stdexcept>
#endif

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word input = 32, output = 80, previous_end = 0x350, previous_count = 19;
constexpr Word loop_flag = Word{1} << 15, loop_bits = Word{3} << 15, pointer_mask = 63;
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word post = (3U << 11) | (3U << 20);
constexpr Word loads = (Word{1} << 15) | (Word{1} << 22);
constexpr Word stores = (2U << 18) | (3U << 16) | (1U << 8) | (1U << 13);
constexpr std::array<Word, 9> samples{
    sign24, sign24 + 1, word_mask, 0, 1, 2, sign24 - 2, sign24 - 1, 0x400001};
constexpr std::size_t data_words = 256;
constexpr Word output_ring = 128, ring_words = 32, short_ring_words = 19, ring_tail = 8;
constexpr Word body_words = 7;
using Data = std::array<Word, data_words>;
using Program = std::vector<Word>;

enum class Shape : std::uint8_t {
  sum,
  difference,
  product,
  negative_product,
  mac,
  negative_mac,
  rounded_mac,
  independent_mac,
  independent_negative_mac,
  independent_rounded_mac,
  rounded_product,
  dual_product,
  dual_negative_product
};

bool independent_shape(const Shape shape) {
  return shape == Shape::independent_mac || shape == Shape::independent_negative_mac ||
         shape == Shape::independent_rounded_mac;
}

Word accumulate_opcode(const Shape shape) {
  const bool negative = shape == Shape::negative_mac || shape == Shape::independent_negative_mac;
  const bool rounded = shape == Shape::rounded_mac || shape == Shape::independent_rounded_mac;
  return static_cast<Word>(isa::Opcode::mac_register_a) | (5U << 4) | (negative ? 4U : 0U) |
         (rounded ? 1U : 0U);
}

Program program(const Shape shape,
                const unsigned shifts,
                const bool rounded,
                const bool mirrored,
                const bool reset) {
  constexpr Word nop = static_cast<Word>(isa::Opcode::nop);
  Program code{pair | post | loads,
               static_cast<Word>(isa::Opcode::tfr_x0_a),
               nop,
               nop,
               nop,
               rounded ? static_cast<Word>(isa::Opcode::rnd_a) : nop,
               pair | post | stores};
  if (shape == Shape::dual_product || shape == Shape::dual_negative_product) {
    code[1] = static_cast<Word>(isa::Opcode::mpy_register_a) | (5U << 4);
    code[2] = static_cast<Word>(isa::Opcode::mac_register_a) | (5U << 4) |
              (shape == Shape::dual_negative_product ? 4U : 0U);
  } else if (shape == Shape::sum || shape == Shape::difference) {
    code[2] =
        static_cast<Word>(shape == Shape::sum ? isa::Opcode::add_y0_a : isa::Opcode::sub_y0_a);
  } else if (independent_shape(shape)) {
    code[1] = static_cast<Word>(isa::Opcode::tfr_x1_a);
    code[2] = accumulate_opcode(shape);
  } else if (shape == Shape::mac || shape == Shape::negative_mac || shape == Shape::rounded_mac) {
    code[2] = accumulate_opcode(shape);
  } else {
    code[1] = static_cast<Word>(isa::Opcode::mpy_register_a) | (5U << 4) |
              (shape == Shape::negative_product ? 4U : 0U) |
              (shape == Shape::rounded_product ? 1U : 0U);
  }
  for (unsigned index = 0; index < shifts; ++index) {
    code[3 + index] = static_cast<Word>(isa::Opcode::asr_a);
  }
  if (mirrored) { code.back() &= ~(Word{1} << 16); }
  if (reset) {
    const auto assignment =
        static_cast<Word>(isa::Opcode::move_long) | (4U << 16) | (16U << 17) | 0xf400U;
    code.insert(code.begin(), {assignment, input + 1, assignment, ~word_mask | input});
  }
  return code;
}

struct Point {
  std::size_t left = 0, right = 0;
  Word count = 1, mode = 0, sticky = 0;
};

struct Model {
  Registers registers;
  Data x, y;
  bool mirrored = false, reset = false;
};

Word post_address(Registers &state, const unsigned index) {
  const auto address = state.r[index];
  state.r[index] = address_model::updated(
      {.modifier = state.m[index], .cursor = address, .offset = state.n[index], .mode = 3});
  return address;
}

Word step(Model &model, const Program &code) {
  auto &state = model.registers;
  auto index = state.pc - start;
  if (model.reset) {
    if (index < 4) {
      state.r[4] = code.at(index + 1) & word_mask;
      state.pc += 2;
      return 2;
    }
    index -= 4;
  }
  if (index == 0) {
    state.x =
        (state.x & ~std::uint64_t{word_mask}) | (model.x.at(post_address(state, 0)) & word_mask);
    state.y =
        (state.y & ~std::uint64_t{word_mask}) | (model.y.at(post_address(state, 4)) & word_mask);
  } else if (index + 1 == body_words) {
    model.x.at(post_address(state, 1)) = register_model::bus_word(state, state.a);
    model.y.at(post_address(state, 5)) =
        register_model::bus_word(state, model.mirrored ? state.a : state.b);
  } else {
    const auto before = state;
    if ((state.sr & saturation_model::mode) != 0) {
      saturation_model::alu(state, before, code.at(index + (model.reset ? 4U : 0U)));
    } else {
      sequence_model::alu(state, before, code.at(index + (model.reset ? 4U : 0U)));
    }
  }
  ++state.pc;
  if (index + 1 != body_words) { return 1; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
    return 1;
  }
  const auto saved = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  const auto outer = state.stack[state.sp % stack_entries];
  state.sp = (state.sp + pointer_mask) & pointer_mask;
  state.sc -= 2;
  state.sr = (state.sr & ~loop_bits) | (static_cast<Word>(saved) & loop_bits);
  state.la = static_cast<Word>(outer >> word_bits);
  state.lc = static_cast<Word>(outer) & word_mask;
  return 1;
}

class Machine {
public:
  Machine(const Shape shape,
          const unsigned shifts,
          const bool rounded,
          const bool mirrored,
          const bool reset = false,
          const Word ring = 0)
      : code_(program(shape, shifts, rounded, mirrored, reset)),
        mirrored_(mirrored),
        reset_(reset),
        independent_(independent_shape(shape)),
        ring_(ring),
        core_(Memory::separate({.program = p_, .x = x_, .y = y_})) {
    std::ranges::copy(code_, p_.begin() + start);
  }

  void check(const Point point, const bool split) {
    constexpr Word poison = 0xab000000;
    x_.fill(poison);
    y_.fill(~poison);
    for (std::size_t index = 0; index < point.count; ++index) {
      x_.at(input + index) = poison | samples[(point.left + index) % samples.size()];
      y_.at(input + index) = poison | samples[(point.right + (3 * index)) % samples.size()];
    }
    auto initial = register_model::seed(point.mode | point.sticky);
    initial.sr |= loop_flag;
    initial.b = 0;
    if (independent_) {
      initial.x =
          (initial.x & word_mask) |
          (std::uint64_t{samples[(point.left + point.right) % samples.size()]} << word_bits);
    }
    initial.r[0] = input;
    initial.r[4] = reset_ ? word_mask : input;
    initial.r[1] = initial.r[5] = output;
    if (ring_ != 0) {
      initial.r[1] = initial.r[5] = output_ring + ring_ - ring_tail;
      initial.m[1] = initial.m[5] = ring_ - 1;
    }
    initial.sp = pointer_mask;
    initial.la = start + static_cast<Word>(code_.size()) - 1;
    initial.lc = point.count;
    initial.stack[pointer_mask % stack_entries] =
        (std::uint64_t{start} << word_bits) | (initial.sr & ~loop_bits);
    initial.stack[(pointer_mask - 1) % stack_entries] =
        (std::uint64_t{previous_end} << word_bits) | previous_count;
    Model model{.registers = initial, .x = x_, .y = y_, .mirrored = mirrored_, .reset = reset_};
    const auto original_program = p_;
#if defined(__aarch64__)
    if (!split) { direct(model); }
#endif
    core_.set_registers(initial);
    const Word total = point.count * instructions();
    const Word prefix = split ? total / 2 : total;
    for (const auto grant : std::array{prefix, total - prefix}) {
      Word cycles = 0;
      for (Word index = 0; index < grant; ++index) { cycles += step(model, code_); }
      const auto before = core_.state().progress;
      const auto result = core_.run(grant);
      const auto progress = Progress{.instructions = before.instructions + grant,
                                     .estimated_cycles = before.estimated_cycles + cycles};
      const bool matched = result.retired == grant && result.fault.kind == FaultKind::none &&
                           result.state == ExecutionState::running && !result.yielded &&
                           core_.registers() == model.registers && result.progress == progress &&
                           x_ == model.x && y_ == model.y && p_ == original_program;
      if (!matched) {
        std::cerr << "bounds pair=" << point.left << '/' << point.right << " count=" << point.count
                  << " RM=" << point.mode << " sticky=" << point.sticky << " grant=" << grant
                  << " registers=" << (core_.registers() == model.registers)
                  << " memory=" << (x_ == model.x && y_ == model.y) << '\n';
      }
      require(matched,
              "Signed value bounds preserve independent rounding, saturation, sticky flags and "
              "full state");
    }
  }

private:
  [[nodiscard]] Word instructions() const { return body_words + (reset_ ? 2U : 0U); }
#if defined(__aarch64__)
  void constant_admission(const detail::JitRuntime &runtime,
                          const std::vector<detail::BlockInstruction> &operations,
                          const Registers &state) {
    if (!reset_) { return; }
    auto changed = operations;
    std::swap(changed[1], changed[2]);
    require(!detail::resident_region_plan(p_, state, runtime, changed),
            "A reset after the first RAM access cannot reuse a constant root");
    changed = operations;
    const auto value = changed[0].instruction.extension;
    constexpr Word assignment =
        static_cast<Word>(isa::Opcode::move_long) | (4U << 16) | (24U << 17) | 0xf400U;
    changed[0].instruction = detail::decode(assignment);
    changed[0].instruction.extension = value;
    require(!detail::resident_region_plan(p_, state, runtime, changed),
            "Rewriting N retains ordinary execution instead of reusing an initial stride");
  }

  void constant_alias(const detail::JitRuntime &runtime, const Model &seed) {
    if (!reset_) { return; }
    const auto mode = seed.registers.sr;
    auto &region = regions_[((mode & mode_rm) != 0 ? 1U : 0U) +
                            ((mode & saturation_model::mode) != 0 ? 2U : 0U)];
    detail::Cpu cpu;
    cpu.reg = seed.registers;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    cpu.reg.r[5] = input;
    const auto before = cpu.reg;
    detail::NativeContext context{.runtime = runtime, .budget = seed.registers.lc * instructions()};
    require(detail::run_resident_region(&cpu, &context, region.get()) == 0 && cpu.reg == before &&
                cpu.instructions == 0 && cpu.cycles == 0 && x_ == seed.x && y_ == seed.y,
            "A store aliasing a broadcast load rejects without publishing CPU or RAM changes");
  }

  void direct(const Model &seed) {
    const detail::JitRuntime runtime{.memory = {{{.data = x_.data(), .words = x_.size()},
                                                 {.data = y_.data(), .words = y_.size()}}}};
    const auto mode = seed.registers.sr;
    auto &region = regions_[((mode & mode_rm) != 0 ? 1U : 0U) +
                            ((mode & saturation_model::mode) != 0 ? 2U : 0U)];
    if (!region) {
      const auto operations = detail::parse_block(p_, seed.registers, runtime);
      region = detail::resident_region_plan(p_, seed.registers, runtime, operations);
      if (!region) {
        throw std::runtime_error("Independent bounds trace must admit native regions");
      }
      constant_admission(runtime, operations, seed.registers);
      for (const unsigned lanes : {1U, 2U, 4U}) {
        region->native[lanes / 2] = arena_.append(detail::emit_resident_region(*region, lanes));
      }
    }
    constant_alias(runtime, seed);
    detail::Cpu cpu;
    cpu.reg = seed.registers;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    const auto budget = seed.registers.lc * instructions();
    detail::NativeContext context{.runtime = runtime, .budget = budget};
    const auto retired = detail::run_resident_region(&cpu, &context, region.get());
    auto expected = seed;
    Word cycles = 0;
    for (Word index = 0; index < retired; ++index) { cycles += step(expected, code_); }
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    const bool matched = retired != 0 && retired <= budget && cpu.reg == expected.registers &&
                         cpu.instructions == retired && cpu.cycles == cycles && x_ == expected.x &&
                         y_ == expected.y;
    if (!matched) {
      std::cerr << "direct bounds mode=" << mode << " count=" << seed.registers.lc
                << " reset=" << reset_ << " retired=" << retired << " budget=" << budget
                << " registers=" << (cpu.reg == expected.registers)
                << " memory=" << (x_ == expected.x && y_ == expected.y) << '\n';
    }
    require(matched,
            "Actual retained native regions preserve independent bounds, flags and ordered memory");
    x_ = seed.x;
    y_ = seed.y;
  }

  detail::ExecutableArena arena_;
  std::array<std::unique_ptr<detail::ResidentRegion>, 4> regions_;
#endif
  const Program code_;
  bool mirrored_, reset_, independent_;
  Word ring_;
  std::array<Word, memory_words> p_{};
  Data x_{}, y_{};
  Core core_;
};

void cases(Machine &machine) {
  constexpr std::array<Word, 4> sticky{0, flag_s, flag_l, flag_s | flag_l};
  for (const auto rm :
       {Word{0}, mode_rm, saturation_model::mode, saturation_model::mode | mode_rm}) {
    for (std::size_t left = 0; left < samples.size(); ++left) {
      for (std::size_t right = 0; right < samples.size(); ++right) {
        for (const Word count : {1U, 2U, 3U, 4U, 5U, 8U, 19U, 20U, 21U, 32U, 33U, 40U, 41U}) {
          const Point point{.left = left,
                            .right = right,
                            .count = count,
                            .mode = rm,
                            .sticky = sticky[(left + right) % sticky.size()]};
          machine.check(point, false);
          machine.check(point, true);
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    for (const Shape shape : {Shape::sum,
                              Shape::difference,
                              Shape::product,
                              Shape::negative_product,
                              Shape::mac,
                              Shape::negative_mac,
                              Shape::rounded_mac,
                              Shape::independent_mac,
                              Shape::independent_negative_mac,
                              Shape::independent_rounded_mac,
                              Shape::rounded_product,
                              Shape::dual_product,
                              Shape::dual_negative_product}) {
      for (unsigned shifts = 0; shifts <= 2; ++shifts) {
        for (const bool rounded : {false, true}) {
          for (const bool mirrored : {false, true}) {
            Machine machine(shape, shifts, rounded, mirrored);
            cases(machine);
          }
        }
      }
    }
    for (const auto shape : {Shape::dual_product, Shape::rounded_mac}) {
      Machine machine(shape, 0, false, true, true);
      cases(machine);
      for (const auto length : {ring_words, short_ring_words}) {
        Machine circular(shape, 0, false, true, true, length);
        cases(circular);
      }
    }
    std::cout << checks << " independent signed value-bound checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
