// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/runtime.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word entry = 8, count = 5;
constexpr Word first = 0x123456, second = 0x654321;
unsigned fallback_calls = 0;

std::uint32_t fallback(detail::Cpu * /*cpu*/, detail::NativeContext * /*context*/) noexcept {
  ++fallback_calls;
  return 0;
}

void admitted_native() {
  constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
  std::array<Word, 32> program{};
  std::array<Word, 32> x{};
  std::array<Word, 32> y{};
  constexpr Word load = pair | (2U << 18) | (3U << 16) | (1U << 15) | (1U << 22);
  constexpr Word store = pair | (3U << 18) | (2U << 16) | (3U << 11) | (3U << 20);
  program[entry] = load;
  program[entry + 1] = store;
  Registers initial;
  initial.pc = entry;
  initial.la = entry + 1;
  initial.lc = count;
  initial.sr = 1U << 15;
  initial.sp = initial.sc = 2;
  initial.stack[2] = std::uint64_t{entry} << test::word_bits;
  initial.r[4] = 1;
  initial.m.fill(word_mask);
  const detail::JitRuntime runtime{
      .memory = {detail::JitMemory{.data = x.data(), .words = x.size()},
                 detail::JitMemory{.data = y.data(), .words = y.size()}}};
  const auto operations = detail::parse_block(program, initial, runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  const auto image =
      detail::compile_region(compiler, operations, runtime, std::bit_cast<const void *>(&fallback));
  if (!image) { throw std::runtime_error("A real RAM loop must be accepted"); }
  require(image->native_bytes > 0, "A real RAM loop must emit LLVM native text");
  using Function = std::uint32_t (*)(detail::Cpu *, detail::NativeContext *) noexcept;
  const auto run = std::bit_cast<Function>(image->entry);
  for (Word budget = 0; budget <= count * 2; ++budget) {
    for (std::uint64_t deadline = 0; deadline <= std::uint64_t{count} * 2; ++deadline) {
      x.fill(first);
      y.fill(second);
      detail::Cpu cpu{.reg = initial};
      detail::NativeContext context{.runtime = {}, .budget = budget, .cycle_deadline = deadline};
      const auto before = fallback_calls;
      const auto turns = std::min(std::uint64_t{budget / 2}, deadline / 2);
      require(
          run(&cpu, &context) == turns * 2 && fallback_calls == before + (turns == 0 ? 1U : 0U) &&
              cpu.instructions == turns * 2 && cpu.cycles == turns * 2 && cpu.reg.r[0] == turns &&
              cpu.reg.r[4] == turns + 1 && (turns == 0 || (x[0] == second && y[1] == first)),
          "Positive whole-loop grants actually execute LLVM, without the ordinary fallback");
    }
  }
  require(compiler->native_bytes() == image->native_bytes,
          "Warm execution never emits code or changes cold text accounting");
}
}

int main() try { admitted_native(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
