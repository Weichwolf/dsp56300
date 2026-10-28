// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "program_cache.hpp"

#include <memory>

namespace dsp56300 {
using namespace detail;

ProgramCache &Core::Impl::program_cache() {
  if (!icache) { icache = std::make_unique<ProgramCache>(options.cache_sector_words); }
  return *icache;
}

Word Core::Impl::fetch_program(const Word address) {
  if (!cacheable(address)) { return read_memory(Space::program, address); }
  const auto value = program_cache().fetch(
      address, buffers.program, (cpu.reg.omr & burst_enable) != 0, options.cacheable_end);
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
