// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <span>
#include <stdexcept>

namespace dsp56300 {
namespace {
void invalid() {
  throw std::invalid_argument("DSP native binding has an invalid fault map");
}

template <class Range> void intervals(const std::span<const Range> ranges) {
  std::uintptr_t end = 0;
  for (const auto &range : ranges) {
    if (range.address == 0 || range.size == 0 || range.address < end ||
        range.size > std::numeric_limits<std::uintptr_t>::max() - range.address) {
      invalid();
    }
    end = range.address + range.size;
  }
}

bool contains(const std::span<const NativeSymbols::TextRange> ranges,
              const std::uintptr_t address,
              const std::uint64_t size) {
  auto next = std::ranges::partition_point(
      ranges, [address](const auto &range) { return range.address <= address; });
  if (next == ranges.begin()) { return false; }
  auto prior = std::prev(next);
  auto end = prior->address + prior->size;
  const auto needed = address + size;
  while (end < needed && next != ranges.end() && next->address == end) {
    end = next->address + next->size;
    ++next;
  }
  return address < prior->address + prior->size && end >= needed;
}

bool source(const detail::ImageContracts &image, const NativeSymbols::Site &site) {
  const auto variant =
      std::ranges::find(image.variants, site.variant, &detail::ImageContracts::Variant::id);
  if (variant == image.variants.end()) { return false; }
  if (variant->instructions.contains(site.guest_pc)) { return true; }
  return std::ranges::any_of(image.vector_plans, [&](const auto &vector) {
    const auto &[key, plan] = vector;
    if (!plan.instructions.contains(site.guest_pc)) { return false; }
    const auto first = variant->words.find(std::get<0>(key));
    const auto second = variant->words.find((std::get<0>(key) + 1) & word_mask);
    return first != variant->words.end() && second != variant->words.end() &&
           first->second == std::get<1>(key) && second->second == std::get<2>(key);
  });
}
}

void NativeImage::Impl::bind_sites(const NativeSymbols::FaultMap map) {
  if (memory_mode == CompileOptions::MemoryMode::direct &&
      (map.text_ranges.empty() || map.sites.empty())) {
    throw std::invalid_argument(
        "Direct DSP binding requires linked executable ranges and source intervals");
  }
  if (map.text_ranges.empty() && map.sites.empty()) { return; }
  if (map.text_ranges.empty() || map.sites.empty()) { invalid(); }
  intervals(map.text_ranges);
  intervals(map.sites);
  for (const auto &site : map.sites) {
    if (!contains(map.text_ranges, site.address, site.size) || !source(*this, site)) { invalid(); }
  }
  for (const auto &variant : variants) {
    if (!contains(map.text_ranges, reinterpret_cast<std::uintptr_t>(variant.entry), 1)) {
      invalid();
    }
  }
  for (const auto &[key, entry] : vectors) {
    if (!contains(map.text_ranges, reinterpret_cast<std::uintptr_t>(entry), 1)) { invalid(); }
  }
  text_ranges.assign(map.text_ranges.begin(), map.text_ranges.end());
  sites.assign(map.sites.begin(), map.sites.end());
}
}
