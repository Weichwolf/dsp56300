// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>

namespace dsp56300 {
std::optional<Word> Core::Impl::known_program_word(const Word address) const {
  if (cacheable(address) && icache) {
    if (const auto word = icache->peek(address)) { return word; }
  }
  if (external) {
    if (io == nullptr) { return std::nullopt; }
    const auto value = io->inspect_program(address);
    return value && *value <= word_mask ? value : std::nullopt;
  }
  if (address >= buffers.program.size()) { return std::nullopt; }
  return buffers.program[address] & word_mask;
}

std::optional<detail::Instruction> Core::Impl::repeat_instruction() const {
  if (cpu.repeat_remaining != 0 && cpu.repeat_loaded) { return repeated; }
  const auto opcode = known_program_word(cpu.reg.pc);
  if (!opcode) { return std::nullopt; }
  auto instruction = detail::decode(*opcode);
  if (instruction.words == 2) {
    const auto extension_word = known_program_word((cpu.reg.pc + 1) & word_mask);
    if (!extension_word) { return std::nullopt; }
    instruction.extension = *extension_word;
  }
  return instruction;
}

std::uint32_t Core::Impl::run_repeat_native(const std::uint32_t instructions) {
  const auto decoded = repeat_instruction();
  if (!decoded || decoded->words != 1 || !detail::repeatable(decoded->operation) ||
      decoded->cycles == 0) {
    return 0;
  }
  auto count = std::min(instructions, cpu.repeat_remaining);
  // REP completion at a DO ending must perform genuine frame cleanup. Keep
  // that last instruction in the owner until native cleanup is qualified.
  if ((cpu.reg.sr & detail::loop_flag) != 0 && cpu.reg.pc == cpu.reg.la) {
    count = std::min(count, cpu.repeat_remaining - 1);
  }
  const auto retired = jit->run_repeat(
      {.instruction = *decoded, .pc = cpu.reg.pc, .mode = cpu.reg.sr & detail::numeric_mode_mask},
      count);
  // The native guard may reject before its first effect. Only a retired body
  // latches the decoded opcode; subsequent owner fallback must see that latch.
  if (retired != 0) { repeated = *decoded; }
  return retired;
}
}
