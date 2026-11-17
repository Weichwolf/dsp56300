// SPDX-License-Identifier: GPL-3.0-only
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <optional>

namespace dsp56300 {
std::optional<Word> Core::Impl::known_program_word(const Word address) const {
  if (cacheable(address) && icache) {
    if (const auto word = icache->peek(address)) { return word; }
  }
  if (external) {
    if (const auto *region = memory_window(Space::program, address)) {
      const auto at = address - region->address;
      if (region->known.empty() || region->known[at] != 0) {
        const auto value = static_cast<const volatile Word *>(region->words.data())[at];
        return value <= word_mask ? std::optional(value) : std::nullopt;
      }
      return std::nullopt;
    }
    if (io == nullptr) { return std::nullopt; }
    const auto value = io->inspect_program(address);
    return value && *value <= word_mask ? value : std::nullopt;
  }
  if (address >= buffers.program.size()) { return std::nullopt; }
  return buffers.program[address] & word_mask;
}

}
