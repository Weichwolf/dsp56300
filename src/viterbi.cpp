// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

namespace dsp56300 {
using namespace detail;

void Core::Impl::viterbi_shift(const Instruction &decoded) {
  if (!numeric_mode_supported(arithmetic_saturation)) { return; }
  const auto opcode = decoded.opcode;
  const AddressForm form{
      .mode = (opcode >> 11) & 7U, .index = (opcode >> 8) & 7U, .extra = decoded.extension};
  if (form.mode == 6 && form.index != 0) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const auto source = ((opcode & (1U << 16)) != 0 ? cpu.reg.b : cpu.reg.a) >> padding_bits;
  const auto high = static_cast<Word>(source >> word_bits) & word_mask;
  const auto low = static_cast<Word>(source) & word_mask;
  const auto shifted = ((low << 1) | ((opcode >> 4) & 1U)) & word_mask;
  const auto address = effective(form);
  if (fault.kind != FaultKind::none) { return; }
  write_memory(Space::x, address, high);
  if (fault.kind == FaultKind::none) { write_memory(Space::y, address, shifted); }
}
} // namespace dsp56300
