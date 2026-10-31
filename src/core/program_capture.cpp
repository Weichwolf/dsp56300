// SPDX-License-Identifier: GPL-3.0-only
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"

namespace dsp56300 {
void Core::Impl::capture_program_words() {
  auto &work = *staged;
  current_pc = work.input.address;
  // The zero-initialized work storage is never an admitted instruction word.
  // Every non-latched word comes from the actual owned bus read continuation.
  work.input.opcode = work.latched_opcode ? *work.latched_opcode
                                          : sample_instruction({.kind = EffectKind::fetch_opcode,
                                                                .space = Space::program,
                                                                .address = work.input.address});
  work.input.extension.reset();
  work.decoded = detail::decode(work.input.opcode);
  if (work.decoded.words == 2) {
    const auto address = (work.input.address + 1) & word_mask;
    const auto value = sample_instruction(
        {.kind = EffectKind::fetch_extension, .space = Space::program, .address = address});
    work.input.extension = value;
    work.decoded.extension = value;
  }
}
} // namespace dsp56300
