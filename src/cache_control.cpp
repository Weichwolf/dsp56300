// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "io.hpp"
#include "program_cache.hpp"

#include <memory>
#include <optional>

namespace dsp56300 {
using namespace detail;

ProgramCache &Core::Impl::program_cache() {
  if (!icache) { icache = std::make_unique<ProgramCache>(buffers.cache.sector_words); }
  return *icache;
}

Word Core::Impl::fetch_program(const Word address) {
  if (external) { observe_program_word(address); }
  if (!cacheable(address)) { return read_memory(Space::program, address); }
  const auto read = [](void *context, const Word index) -> std::optional<Word> {
    auto &core = *static_cast<Impl *>(context);
    Word value = 0;
    if (core.fault.kind != FaultKind::none || core.io == nullptr ||
        !dsp56300_io_read(core.io, Space::program, index, core.current_pc, value)) {
      core.fail(FaultKind::memory_read, Space::program, index);
      return std::nullopt;
    }
    return value & word_mask;
  };
  const auto value =
      external
          ? program_cache().fetch(address,
                                  address_words,
                                  (cpu.reg.omr & burst_enable) != 0,
                                  buffers.cache.end,
                                  read,
                                  this)
          : program_cache().fetch(
                address, buffers.program, (cpu.reg.omr & burst_enable) != 0, buffers.cache.end);
  if (!value) {
    fail(FaultKind::memory_read, Space::program, address);
    return 0;
  }
  return *value;
}

void Core::Impl::execute_cache(const Instruction &decoded) {
  if ((cpu.reg.sr & cache_enable) == 0) {
    // Cache controls are illegal with CE clear; the ordinary pending vector owns entry.
    execute_exception({.operation = Operation::illegal});
    return;
  }
  if (decoded.operation == Operation::cache_flush) {
    program_cache().flush(false);
    return;
  }
  if (decoded.operation == Operation::cache_flush_unlocked) {
    program_cache().flush(true);
    return;
  }
  if (decoded.operation == Operation::cache_free) {
    program_cache().free();
    return;
  }
  const bool relative = decoded.operation == Operation::cache_lock_relative ||
                        decoded.operation == Operation::cache_unlock_relative;
  const auto address =
      relative ? address_add(current_pc, signed_word(decoded.extension)) : control_address(decoded);
  if (fault.kind != FaultKind::none) { return; }
  const bool locked = decoded.operation == Operation::cache_lock ||
                      decoded.operation == Operation::cache_lock_relative;
  program_cache().lock(address, locked);
}
} // namespace dsp56300
