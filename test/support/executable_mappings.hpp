// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "support.hpp"

#include <cstdint>
#include <vector>
#if defined(__APPLE__)
#include <mach/arm/vm_types.h>
#include <mach/kern_return.h>
#include <mach/mach_init.h>
#include <mach/mach_vm.h>
#include <mach/message.h>
#include <mach/vm_prot.h>
#include <mach/vm_region.h>
#else
#include <charconv>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#endif

namespace dsp56300::test {
struct ExecutableRange {
  std::uintptr_t first = 0;
  std::uintptr_t last = 0;
  bool operator==(const ExecutableRange &) const = default;
};

inline std::vector<ExecutableRange> jit_executable_mappings() {
#if defined(__APPLE__)
  std::vector<ExecutableRange> mappings;
  mach_vm_address_t address = 0;
  mach_vm_size_t size = 0;
  natural_t depth = 0;
  while (true) {
    vm_region_submap_info_data_64_t region{};
    mach_msg_type_number_t count = VM_REGION_SUBMAP_INFO_COUNT_64;
    const auto result = mach_vm_region_recurse(mach_task_self(),
                                               &address,
                                               &size,
                                               &depth,
                                               reinterpret_cast<vm_region_recurse_info_t>(&region),
                                               &count);
    if (result == KERN_INVALID_ADDRESS) { break; }
    require(result == KERN_SUCCESS && size != 0,
            "The Mach kernel provides actual native mappings and protection bits");
    if (region.is_submap != 0) {
      ++depth;
      continue;
    }
    if (region.external_pager == 0 && (region.protection & VM_PROT_EXECUTE) != 0) {
      require((region.protection & VM_PROT_READ) != 0, "Native images remain readable");
      mappings.push_back({.first = address, .last = address + size});
    }
    address += size;
  }
  return mappings;
#else
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
    if (permissions != "r-xs" || inode == 0 ||
        label.find("/memfd:dsp56300-jit (deleted)") == std::string::npos) {
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
#endif
}

inline std::uint64_t jit_executable_bytes() {
  std::uint64_t bytes = 0;
  for (const auto range : jit_executable_mappings()) { bytes += range.last - range.first; }
  return bytes;
}
} // namespace dsp56300::test
