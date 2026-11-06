// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "support.hpp"

#include <charconv>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace dsp56300::test {
struct ExecutableRange {
  std::uintptr_t first = 0;
  std::uintptr_t last = 0;
  bool operator==(const ExecutableRange &) const = default;
};

inline std::vector<ExecutableRange> anonymous_executable_mappings() {
  std::ifstream maps("/proc/self/maps");
  require(maps.good(), "The native retention test reads actual Linux executable mappings");
  std::vector<ExecutableRange> mappings;
  for (std::string line; std::getline(maps, line);) {
    std::istringstream fields(line);
    std::string range;
    std::string permissions;
    std::string offset;
    std::string device;
    std::string label;
    std::uint64_t inode = 0;
    require(static_cast<bool>(fields >> range >> permissions >> offset >> device >> inode),
            "A kernel mapping has its required fields");
    std::getline(fields, label);
    if (permissions != "r-xp" || inode != 0 ||
        label.find_first_not_of(" \t") != std::string::npos) {
      continue;
    }
    const auto dash = range.find('-');
    require(dash != std::string::npos, "An executable mapping contains both address bounds");
    std::uintptr_t first = 0;
    std::uintptr_t last = 0;
    const auto *const begin = range.data();
    const auto *const end = begin + range.size();
    const auto low = std::from_chars(begin, begin + dash, first, 16);
    const auto high = std::from_chars(begin + dash + 1, end, last, 16);
    require(low.ec == std::errc{} && low.ptr == begin + dash && high.ec == std::errc{} &&
                high.ptr == end && last > first,
            "Both actual executable address bounds parse exactly");
    mappings.push_back({.first = first, .last = last});
  }
  return mappings;
}

inline std::uint64_t anonymous_executable_bytes() {
  std::uint64_t bytes = 0;
  for (const auto range : anonymous_executable_mappings()) { bytes += range.last - range.first; }
  return bytes;
}
} // namespace dsp56300::test
