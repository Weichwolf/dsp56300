// SPDX-License-Identifier: GPL-3.0-only
// Own helper-free DSP loops; expectations use the independent raw-56 model.
#pragma once

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::test::kvm::region {
inline constexpr std::array chain{Word{0x2000d0},
                                  Word{0x2000d2},
                                  Word{0x2000fa},
                                  Word{0x200010},
                                  Word{0x200018},
                                  Word{0x200036},
                                  Word{0x20002e},
                                  Word{0x200011},
                                  Word{0x200005},
                                  Word{0x200069}};
inline constexpr std::array modes{
    0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
inline constexpr Word loop_flag = Word{1} << 15;
inline constexpr Word maximum_iterations = 65536;

struct Shape {
  Word body = 1, iterations = 1, mode = 0;
};

inline void validate(const Shape shape) {
  if ((shape.body != 1 && shape.body != chain.size()) || shape.iterations == 0 ||
      shape.iterations > maximum_iterations) {
    throw std::invalid_argument("Unsupported qualified DSP region shape");
  }
  bool supported = false;
  for (const auto mode : modes) { supported = supported || shape.mode == mode; }
  if (!supported) { throw std::invalid_argument("Unsupported qualified DSP region mode"); }
}

inline Word instructions(const Shape shape) {
  return shape.body * shape.iterations;
}

inline Registers initial(const Shape shape) {
  validate(shape);
  auto result = register_model::seed(shape.mode | loop_flag | flag_l | flag_s | flag_c);
  result.pc = start;
  result.la = start + shape.body - 1;
  result.lc = shape.iterations + 1; // Leave final-loop cleanup to general execution.
  result.sp = result.sc = 2;
  result.stack[1] = 0;
  result.stack[2] = (std::uint64_t{start} << word_bits) | (result.sr & ~loop_flag);
  return result;
}

inline detail::Cpu cpu(const Shape shape) {
  detail::Cpu result;
  result.reg = initial(shape);
  result.reg.a <<= detail::padding_bits;
  result.reg.b <<= detail::padding_bits;
  result.instructions = 8;
  result.cycles = 16;
  result.repeat_completed = true;
  result.interrupt_inhibited = true;
  return result;
}

inline Registers expected(const Shape shape) {
  auto result = initial(shape);
  for (Word iteration = 0; iteration < shape.iterations; ++iteration) {
    for (const auto opcode : std::span(chain).first(shape.body)) {
      const auto old = result;
      sequence_model::alu(result, old, opcode);
      ++result.pc;
    }
    --result.lc;
    result.pc = start;
  }
  return result;
}

inline std::vector<std::uint8_t> compile(const Shape shape) {
  validate(shape);
  std::vector<detail::NativeOp> operations;
  for (const auto opcode : std::span(chain).first(shape.body)) {
    operations.push_back({.instruction = detail::decode(opcode),
                          .pc = start + static_cast<Word>(operations.size()),
                          .mode = shape.mode,
                          .loop_address = start + shape.body - 1});
  }
  detail::JitEmitter emitter({}, true);
  return emitter.generate(operations);
}
} // namespace dsp56300::test::kvm::region
