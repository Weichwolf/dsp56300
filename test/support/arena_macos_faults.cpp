// SPDX-License-Identifier: GPL-3.0-only
#include "arena_macos_faults.hpp"

#include <atomic>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <mach/arm/vm_types.h>
#include <mach/kern_return.h>
#include <mach/mach_init.h>
#include <mach/mach_vm.h>
#include <mach/message.h>
#include <mach/vm_region.h>
#include <sys/_types/_off_t.h>
#include <sys/mman.h>

namespace dsp56300::test {
namespace {
constinit std::atomic<ArenaFailure> failure{ArenaFailure::none};
constinit std::atomic<void *> allocation{nullptr};
static_assert(decltype(failure)::is_always_lock_free && decltype(allocation)::is_always_lock_free);
constexpr std::size_t arena_bytes = std::size_t{64} << 10;

void *map(void *const address,
          const std::size_t size,
          const int protection,
          const int flags,
          const int descriptor,
          const off_t offset) noexcept {
  const bool arena = descriptor == -1 && size == arena_bytes &&
                     (flags & (MAP_ANON | MAP_PRIVATE)) == (MAP_ANON | MAP_PRIVATE);
  if (arena && failure.load(std::memory_order_relaxed) == ArenaFailure::allocation) {
    errno = ENOMEM;
    return MAP_FAILED;
  }
  auto *const result = mmap(address, size, protection, flags, descriptor, offset);
  if (arena && result != MAP_FAILED) { allocation.store(result, std::memory_order_relaxed); }
  return result;
}

int protect(void *const address, const std::size_t size, const int protection) noexcept {
  const auto requested = failure.load(std::memory_order_relaxed);
  if (address == allocation.load(std::memory_order_relaxed) &&
      ((requested == ArenaFailure::writable && (protection & PROT_WRITE) != 0) ||
       (requested == ArenaFailure::executable && (protection & PROT_EXEC) != 0))) {
    errno = ENOMEM;
    return -1;
  }
  return mprotect(address, size, protection);
}

// A separate dylib lets dyld replace the caller's imports while preserving these real calls.
[[gnu::used, gnu::section("__DATA,__interpose")]] const struct {
  decltype(&mmap) replacement;
  decltype(&mmap) original;
} map_interpose{.replacement = map, .original = mmap};

[[gnu::used, gnu::section("__DATA,__interpose")]] const struct {
  decltype(&mprotect) replacement;
  decltype(&mprotect) original;
} protect_interpose{.replacement = protect, .original = mprotect};
}

void set_arena_failure(const ArenaFailure requested) noexcept {
  failure.store(requested, std::memory_order_relaxed);
  if (requested != ArenaFailure::none) { allocation.store(nullptr, std::memory_order_relaxed); }
}

bool arena_failure_released() noexcept {
  const auto *const mapped = allocation.load(std::memory_order_relaxed);
  if (mapped == nullptr) { return true; }
  const auto original = std::bit_cast<mach_vm_address_t>(mapped);
  auto address = original;
  mach_vm_size_t size = 0;
  natural_t depth = 0;
  vm_region_submap_info_data_64_t region{};
  mach_msg_type_number_t count = VM_REGION_SUBMAP_INFO_COUNT_64;
  const auto result = mach_vm_region_recurse(mach_task_self(),
                                             &address,
                                             &size,
                                             &depth,
                                             reinterpret_cast<vm_region_recurse_info_t>(&region),
                                             &count);
  return result == KERN_INVALID_ADDRESS || (result == KERN_SUCCESS && address > original);
}
}
