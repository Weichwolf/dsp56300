// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "semantic.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace dsp56300 {
std::uint64_t Core::Impl::semantic_region(void *const owner,
                                          const std::uint32_t budget,
                                          const detail::SemanticRegion &region,
                                          const std::uint64_t window) noexcept {
  auto &self = *static_cast<Impl *>(owner);
  self.assume_image_admission();
  const bool ordinary = !self.exception_ready();
  __builtin_assume(ordinary);
  detail::NativeContext context{
      .runtime = {.memory = {detail::JitMemory{.data = self.buffers.x.data(),
                                               .words = self.buffers.x.size(),
                                               .aliases_program = self.aliases_program[1]},
                             detail::JitMemory{.data = self.buffers.y.data(),
                                               .words = self.buffers.y.size(),
                                               .aliases_program = self.aliases_program[2]}},
                  .windows = self.image_windows},
      .budget = budget,
      .cycle_deadline =
          self.cpu.cycles +
          std::min(window, std::numeric_limits<std::uint64_t>::max() - self.cpu.cycles)};
  const auto retired = region.entry(&self.cpu, &context);
  if (retired != 0) {
    self.current_pc = region.pc;
    self.current_words = region.words;
  }
  constexpr unsigned pc_shift = 32;
  return retired | (static_cast<std::uint64_t>(self.cpu.reg.pc) << pc_shift);
}
}
