// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"

#include <optional>

namespace dsp56300 {
void Core::Impl::capture_program_words() {
  auto &work = *staged;
  current_pc = work.input.address;
  const auto fetch = [this](const Word address, const EffectKind kind) -> Word {
    if (!cacheable(address)) {
      return sample_instruction({.kind = kind, .space = Space::program, .address = address});
    }
    struct Source {
      Impl *core;
      EffectKind kind;
    } source{.core = this, .kind = kind};
    const auto read = [](void *context, const Word index) -> std::optional<Word> {
      const auto &input = *static_cast<Source *>(context);
      return input.core->sample_instruction(
          {.kind = input.kind, .space = Space::program, .address = index});
    };
    const auto size = external ? address_words : buffers.program.size();
    const auto value = program_cache().fetch(address,
                                             size,
                                             (cpu.reg.omr & detail::burst_enable) != 0,
                                             options.cacheable_end,
                                             read,
                                             &source);
    if (!value) {
      fail(FaultKind::memory_read, Space::program, address);
      return 0;
    }
    return *value;
  };
  // The zero-initialized work storage is never an admitted instruction word.
  // Every non-latched word comes from the actual owned bus read continuation.
  work.input.opcode = work.latched_opcode ? *work.latched_opcode
                                          : fetch(work.input.address, EffectKind::fetch_opcode);
  if (fault.kind != FaultKind::none) { return; }
  work.input.extension.reset();
  work.decoded = detail::decode(work.input.opcode);
  // FM13-160/A.3.8: reject an invalid REP body before offering an extension
  // transaction. A missing extension must not hide the actual invalid opcode.
  if (cpu.repeat_remaining != 0 &&
      (work.decoded.words != 1 || !detail::repeatable(work.decoded.operation))) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, work.input.opcode);
    return;
  }
  if (work.decoded.words == 2) {
    const auto address = (work.input.address + 1) & word_mask;
    const auto value = fetch(address, EffectKind::fetch_extension);
    if (fault.kind != FaultKind::none) { return; }
    work.input.extension = value;
    work.decoded.extension = value;
  }
}
} // namespace dsp56300
