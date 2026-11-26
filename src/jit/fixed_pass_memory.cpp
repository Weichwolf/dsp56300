// SPDX-License-Identifier: GPL-3.0-only
#include "jit/fixed_pass_memory.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit/fixed_pass.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::detail {
namespace {
std::optional<PassWord>
ordinary_cell(const PassWindows &windows, const PassCell cell, const bool write) noexcept {
  if (cell.space == Space::program || cell.address >= io_begin) { return std::nullopt; }
  const auto candidates = windows[cell.space == Space::x ? 0 : 1];
  const auto window = std::ranges::find_if(candidates, [&](const MemoryWindow &view) {
    return cell.address >= view.address && cell.address - view.address < view.words.size();
  });
  if (window == candidates.end() || window->aliases_program || (write && !window->writable)) {
    return std::nullopt;
  }
  const auto offset = cell.address - window->address;
  if (window->known.empty()) {
    return PassWord{.word = &window->words[offset], .before = window->words[offset]};
  }
  if (offset >= window->known.size() || window->known[offset] == 0) { return std::nullopt; }
  return PassWord{.word = &window->words[offset],
                  .known = &window->known[offset],
                  .before = window->words[offset],
                  .known_before = window->known[offset]};
}

bool contains_byte(const Word *const word, const std::uint8_t *const address) noexcept {
  const auto cell = std::bit_cast<std::uintptr_t>(word);
  const auto byte = std::bit_cast<std::uintptr_t>(address);
  return byte >= cell && byte - cell < sizeof(Word);
}

bool touches_source(const PassWord &write, const PassWord &source) noexcept {
  return write.word == source.word ||
         (source.known != nullptr &&
          (write.known == source.known || contains_byte(write.word, source.known))) ||
         (write.known != nullptr && contains_byte(source.word, write.known));
}

std::optional<PassWord> safe_read(const PassWindows &windows,
                                  const PassRead &read,
                                  const std::span<const PassWord> writes,
                                  const bool canonical_data,
                                  PassFootprint *const footprint) noexcept {
  auto at = read.address.offset;
  if (read.address.base) {
    if (!canonical_data && !writes.empty()) { return std::nullopt; }
    const auto origin = ordinary_cell(windows, *read.address.base, false);
    if (!origin || std::ranges::any_of(writes, [&](const PassWord &write) {
          return touches_source(write, *origin);
        })) {
      return std::nullopt;
    }
    if (footprint != nullptr) { footprint->retain(*origin); }
    at = (at + (origin->before & word_mask)) & word_mask;
  }
  return ordinary_cell(windows, {.space = read.space, .address = at}, false);
}
}

bool capture_pass_memory(const PassEffects &effects,
                         const Registers &registers,
                         const PassWindows &windows,
                         const std::span<PassWord> writes,
                         const bool canonical_data,
                         PassFootprint *const footprint) noexcept {
  if (writes.size() != effects.writes.size()) { return false; }
  for (std::size_t index = 0; index < registers.m.size(); ++index) {
    if ((effects.linear & (1U << index)) != 0 &&
        (registers.m[index] & modifier_mask) != modifier_mask) {
      return false;
    }
  }
  for (std::size_t index = 0; index < writes.size(); ++index) {
    const auto cell = ordinary_cell(windows, effects.writes[index], true);
    if (!cell) { return false; }
    writes[index] = *cell;
    if (footprint != nullptr) { footprint->retain(*cell); }
  }
  for (const auto &read : effects.reads) {
    const auto cell = safe_read(windows, read, writes, canonical_data, footprint);
    if (!cell) { return false; }
    if (footprint != nullptr) { footprint->retain(*cell); }
  }
  return true;
}

bool same_pass_memory(const std::span<const PassWord> writes) noexcept {
  return std::ranges::all_of(writes, [](const PassWord &cell) {
    return cell.word != nullptr && *cell.word == cell.before &&
           (cell.known == nullptr || *cell.known == cell.known_before);
  });
}
}
