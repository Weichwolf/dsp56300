// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "memory_views.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <unistd.h>

namespace dsp56300 {
bool detail::dsp56300_memory_direct() noexcept {
  return false;
}

namespace {
std::uintptr_t address(const std::span<Word> words) {
  return reinterpret_cast<std::uintptr_t>(words.data());
}

void view(const Memory::View &loan, const std::uintptr_t page) {
  const auto first = address(loan.data);
  if (first == 0 || first % page != 0 || loan.data.size() != address_words ||
      loan.data.size_bytes() > std::numeric_limits<std::uintptr_t>::max() - first ||
      !loan.mirrored_tail.empty()) {
    throw std::invalid_argument(
        "DSP complete memory requires aligned full word views without tails");
  }
}

void windows(const std::span<const MemoryWindow> regions,
             const Memory::View &loan,
             const Space space) {
  const auto limit = space == Space::program ? address_words : detail::io_begin;
  std::size_t end = 0;
  for (const auto &region : regions) {
    if (region.words.empty() || region.address < end || region.address >= limit ||
        region.words.size() > limit - region.address ||
        region.words.data() != loan.data.data() + region.address ||
        (!region.known.empty() && region.known.size() != region.words.size())) {
      throw std::invalid_argument("DSP complete region differs from its read view");
    }
    end = region.address + region.words.size();
  }
}

bool overlaps(const std::span<Word> first, const std::span<Word> second) {
  const auto a = address(first);
  const auto b = address(second);
  return a <= b ? b - a < first.size_bytes() : a - b < second.size_bytes();
}
}

void detail::validate_memory_views(const std::array<std::span<const MemoryWindow>, 3> &regions,
                                   const std::array<Memory::Views, 3> &views) {
  const auto granted = sysconf(_SC_PAGESIZE);
  if (granted <= 0 || static_cast<std::uint64_t>(granted) % sizeof(Word) != 0) {
    throw std::runtime_error("Cannot determine DSP host page geometry");
  }
  const auto page = static_cast<std::uintptr_t>(granted);
  if (regions[0].empty()) {
    throw std::invalid_argument("DSP complete memory needs program regions");
  }
  for (std::size_t bank = 0; bank < views.size(); ++bank) {
    view(views[bank].read, page);
    view(views[bank].write, page);
    windows(regions[bank], views[bank].read, static_cast<Space>(bank));
    for (std::size_t prior = 0; prior < bank; ++prior) {
      if (overlaps(views[bank].read.data, views[prior].read.data) ||
          overlaps(views[bank].write.data, views[prior].write.data)) {
        throw std::invalid_argument("DSP complete spaces require distinct virtual views");
      }
    }
  }
}

Memory Memory::complete(const std::array<std::span<const MemoryWindow>, 3> regions,
                        const std::array<Views, 3> views) {
  detail::validate_memory_views(regions, views);
  auto result = external();
  result.regions_ = regions;
  result.views_ = views;
  return result;
}

void Core::Impl::load_views(const Space space,
                            const Word address,
                            const std::span<const Word> words) {
  if (address > address_words || words.size() > address_words - address) {
    throw std::out_of_range("DSP host load exceeds the complete address space");
  }
  if (std::ranges::any_of(words, [](const Word word) { return word > word_mask; })) {
    throw std::invalid_argument("DSP memory values must fit in 24 bits");
  }
  for (std::size_t index = 0; index < words.size(); ++index) {
    const auto *region = memory_window(space, address + static_cast<Word>(index));
    if (region == nullptr || !region->writable) {
      throw std::out_of_range("DSP host load requires writable ordinary storage");
    }
  }
  for (std::size_t index = 0; index < words.size(); ++index) {
    const auto at = address + static_cast<Word>(index);
    const auto *region = memory_window(space, at);
    const auto value = static_cast<const volatile Word *>(words.data())[index];
    static_cast<volatile Word *>(views[static_cast<std::size_t>(space)].write.data.data())[at] =
        value;
    if (!region->known.empty()) { region->known[at - region->address] = 1; }
    if (space == Space::program) {
      image_word_written({.address = at, .value = value});
    } else if (region->aliases_program) {
      image_ready = false;
    }
  }
}
}
