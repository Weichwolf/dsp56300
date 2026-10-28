// SPDX-License-Identifier: GPL-3.0-only
#include "program_cache.hpp"

#include "dsp56300/core.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>

namespace dsp56300::detail {
ProgramCache::ProgramCache(const unsigned sector_words) : sector_words_(sector_words) {
  if (sector_words != 128 && sector_words != maximum_sector_words) {
    throw std::invalid_argument("DSP instruction cache sectors require 128 or 256 words");
  }
  words_.resize(sector_count * sector_words_);
  flush(false);
}

std::size_t ProgramCache::find(const Word address) const {
  const Word tag = address / sector_words_;
  for (std::size_t slot = 0; slot < sector_count; ++slot) {
    if (sectors_[slot].tag == tag) { return slot; }
  }
  return sector_count;
}

std::size_t ProgramCache::acquire(const Word address) {
  const auto found = find(address);
  if (found != sector_count) { return found; }
  for (std::size_t position = sector_count; position > 0; --position) {
    const auto slot = order_[position - 1];
    auto &sector = sectors_[slot];
    if (sector.locked) { continue; }
    sector.tag = address / sector_words_;
    sector.valid.reset();
    return slot;
  }
  return sector_count;
}

void ProgramCache::touch(const std::size_t slot) {
  for (std::size_t position = 0; position < sector_count; ++position) {
    if (order_[position] != slot) { continue; }
    for (; position > 0; --position) { order_[position] = order_[position - 1]; }
    order_[0] = slot;
    return;
  }
}

std::optional<Word> ProgramCache::peek(const Word address) const {
  const auto slot = find(address);
  const auto offset = address % sector_words_;
  if (slot == sector_count || !sectors_[slot].valid.test(offset)) { return std::nullopt; }
  return words_[(slot * sector_words_) + offset];
}

std::optional<Word> ProgramCache::fetch(const Word address,
                                        const std::span<const Word> memory,
                                        const bool burst,
                                        const Word cacheable_end) {
  if (address >= memory.size()) { return std::nullopt; }
  const auto slot = acquire(address);
  if (slot == sector_count) { return memory[address] & word_mask; }
  touch(slot);
  auto &sector = sectors_[slot];
  const auto offset = address % sector_words_;
  if (sector.valid.test(offset)) { return words_[(slot * sector_words_) + offset]; }
  const auto count = burst ? 4U - (address % 4) : 1U;
  const auto end =
      std::ranges::min(std::array<std::size_t, 3>{address + count, memory.size(), cacheable_end});
  for (std::size_t next = address; next < end; ++next) {
    const auto word = next % sector_words_;
    words_[(slot * sector_words_) + word] = memory[next] & word_mask;
    sector.valid.set(word);
  }
  return words_[(slot * sector_words_) + offset];
}

void ProgramCache::write(const Write transfer) {
  const auto address = transfer.address;
  const auto slot = find(address);
  if (slot == sector_count) { return; }
  const auto offset = address % sector_words_;
  words_[(slot * sector_words_) + offset] = transfer.value & word_mask;
  sectors_[slot].valid.set(offset);
}

void ProgramCache::lock(const Word address, const bool locked) {
  const auto slot = acquire(address);
  if (slot == sector_count) { return; }
  sectors_[slot].locked = locked;
  touch(slot);
}

void ProgramCache::free() {
  for (auto &sector : sectors_) { sector.locked = false; }
}

void ProgramCache::flush(const bool unlocked_only) {
  for (auto &sector : sectors_) {
    if (unlocked_only && sector.locked) { continue; }
    sector.valid.reset();
    sector.tag = word_mask / sector_words_;
    sector.locked = false;
  }
  std::ranges::iota(order_, std::size_t{0});
}

const std::array<ProgramCache::Sector, ProgramCache::sector_count> &
ProgramCache::sectors() const noexcept {
  return sectors_;
}

std::span<const Word> ProgramCache::contents(const std::size_t slot) const noexcept {
  return std::span(words_).subspan(slot * sector_words_, sector_words_);
}

const std::array<std::size_t, ProgramCache::sector_count> &ProgramCache::order() const noexcept {
  return order_;
}
} // namespace dsp56300::detail
