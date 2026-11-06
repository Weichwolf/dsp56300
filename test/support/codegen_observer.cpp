// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"

#include <cstddef>
#include <sys/mman.h>

namespace dsp56300::test::codegen {
namespace {
thread_local Publications observed;
}

Publications publications() noexcept {
  return observed;
}

extern "C" int system_protect(void *address, std::size_t length, int permissions) noexcept
    asm("__real_mprotect");
extern "C" int observe_protect(void *address, std::size_t length, int permissions) noexcept
    asm("__wrap_mprotect");

int observe_protect(void *const address, const std::size_t length, const int permissions) noexcept {
  const auto result = system_protect(address, length, permissions);
  if (result == 0 && (permissions & PROT_EXEC) != 0) {
    ++observed.count;
    observed.bytes += length;
  }
  return result;
}
}
