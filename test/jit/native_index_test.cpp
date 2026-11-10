// SPDX-License-Identifier: GPL-3.0-only
#include "jit/index.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test {
namespace {
void table(const unsigned levels) {
  constexpr std::array<std::uint32_t, 9> pcs{
      0, 1, 255, 256, 65535, 65536, 0x123456, 0xfffffe, 0xffffff};
  const unsigned first_token = 17;
  const unsigned second_token = 29;
  detail::NativeTable first(levels);
  detail::NativeTable second(levels);
  const auto *const view = first.index().data();
  for (const auto pc : pcs) {
    require(first.find(pc) == nullptr && second.find(pc) == nullptr,
            "Sparse target holes start empty across every address boundary");
    first.insert(pc, &first_token);
    second.insert(pc, &second_token);
  }
  for (const auto pc : pcs) {
    require(first.find(pc) == &first_token && second.find(pc) == &second_token,
            "Independent contexts retain independent native targets");
    first.insert(pc, nullptr);
    require(first.find(pc) == nullptr && second.find(pc) == &second_token,
            "Removing a target preserves the other context");
    first.insert(pc, &second_token);
  }
  first.clear();
  for (const auto pc : pcs) {
    require(first.find(pc) == nullptr && second.find(pc) == &second_token,
            "Clearing occupied targets restores holes without changing other tables");
    first.insert(pc, &first_token);
  }
  require(first.index().data() == view, "Clearing and reinsertion preserve the native table view");
#if defined(__x86_64__) && defined(__linux__)
  for (const auto pc : pcs) {
    require(view[pc] == first.find(pc), "Native indexed loads see the authoritative target");
  }
#endif
  if (levels == 4) {
    for (const auto key : {0x1000000U, 0x80000000U, 0xffffffffU}) {
      first.insert(key, &first_token);
      require(first.find(key) == &first_token && second.find(key) == nullptr,
              "Context catalog retains its full 32-bit key range");
#if defined(__x86_64__) && defined(__linux__)
      require(view[key] == &first_token, "Native indexed catalog loads preserve high key bits");
#endif
    }
    first.clear();
    for (const auto key : {0x1000000U, 0x80000000U, 0xffffffffU}) {
      require(first.find(key) == nullptr, "Clearing the catalog removes every high-key target");
    }
  }
  if (levels == 3) {
    require(first.find(0x1000000) == nullptr, "A wide PC never aliases its low 24 bits");
    bool rejected = false;
    try {
      first.insert(0x1000000, &first_token);
    } catch (const std::out_of_range &) { rejected = true; }
    require(rejected, "The PC table rejects a target beyond its address space");
  }
}
}
}

int main() {
  try {
    dsp56300::test::table(3);
    dsp56300::test::table(4);
    std::cout << "Native target tables preserve holes, contexts, bounds and stable views\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
