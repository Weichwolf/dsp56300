// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "executable_mappings.hpp"
#include "jit/arena.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <span>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/types.h>
#include <system_error>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::uint8_t pattern_one = 0x5a, pattern_two = 0xa5, pattern_three = 0x3c,
                       pattern_four = 0xc3;
constexpr std::size_t segment_bytes = std::size_t{64} << 10, alignment = 16;
enum class Failure : std::uint8_t { none, descriptor, truncate, writable, executable };
thread_local Failure failure = Failure::none;

std::size_t descriptors() {
  return static_cast<std::size_t>(std::distance(
      std::filesystem::directory_iterator("/proc/self/fd"), std::filesystem::directory_iterator()));
}

std::size_t aliases() {
  std::ifstream maps("/proc/self/maps");
  require(maps.good(), "Read actual kernel arena aliases");
  std::size_t count = 0;
  for (std::string line; std::getline(maps, line);) {
    if (line.find("/memfd:dsp56300-jit (deleted)") != std::string::npos) { ++count; }
  }
  return count;
}

void permissions() {
  std::ifstream maps("/proc/self/maps");
  require(maps.good(), "Read actual kernel permissions of both aliases");
  std::size_t writable = 0;
  std::size_t executable = 0;
  for (std::string line; std::getline(maps, line);) {
    if (line.find("/memfd:dsp56300-jit (deleted)") == std::string::npos) { continue; }
    const auto first = line.find(' ');
    require(first != std::string::npos && line.size() >= first + 5, "Actual arena mapping fields");
    const auto access = line.substr(first + 1, 4);
    require(access == "rw-s" || access == "r-xs", "No alias is simultaneously writable/executable");
    if (access == "rw-s") { ++writable; }
    if (access == "r-xs") { ++executable; }
  }
  require(executable > 0 && writable == executable, "Each arena has one RW and one RX mapping");
}

struct Image {
  const std::uint8_t *entry = nullptr;
  std::vector<std::uint8_t> expected;
};

void preserved(const std::span<const Image> images) {
  const auto mappings = jit_executable_mappings();
  for (const auto &image : images) {
    const auto first = std::bit_cast<std::uintptr_t>(image.entry);
    require(first % alignment == 0, "Every native image starts at its required alignment");
    require(std::ranges::any_of(mappings,
                                [&](const auto range) {
                                  return range.first <= first &&
                                         range.last >= first + image.expected.size();
                                }),
            "Every earlier complete image remains mapped RX");
    require(std::ranges::equal(std::span(image.entry, image.expected.size()), image.expected),
            "Every published byte remains equal to independently supplied input");
  }
}

void append(detail::ExecutableArena &arena,
            std::vector<Image> &images,
            const std::size_t bytes,
            const std::uint8_t pattern) {
  const std::vector<std::uint8_t> input(bytes, pattern);
  const auto *const entry = static_cast<const std::uint8_t *>(arena.append(input));
  images.push_back({.entry = entry, .expected = input});
  preserved(images);
  permissions();
}

void packing() {
  const auto original = jit_executable_mappings();
  const auto initial_descriptors = descriptors();
  {
    detail::ExecutableArena arena;
    std::vector<Image> images;
    for (const std::size_t size : {1U, 15U, 16U, 17U}) { append(arena, images, size, pattern_one); }
    const auto first = std::bit_cast<std::uintptr_t>(images.front().entry);
    constexpr std::array<std::size_t, 4> offsets{0, 16, 32, 48};
    for (std::size_t index = 0; index < offsets.size(); ++index) {
      require(std::bit_cast<std::uintptr_t>(images[index].entry) == first + offsets[index],
              "Small images pack contiguously with only necessary alignment padding");
    }
    require(jit_executable_bytes() == segment_bytes, "Small images share one 64-KiB RX segment");
    constexpr auto prefix_bytes = offsets.back() + 17;
    constexpr auto tail_offset = (prefix_bytes + alignment - 1) / alignment * alignment;
    append(arena, images, segment_bytes - tail_offset, pattern_two);
    require(jit_executable_bytes() == segment_bytes, "An exact-fit tail uses existing storage");
    append(arena, images, 1, pattern_three);
    require(jit_executable_bytes() == 2 * segment_bytes, "Full storage grows without rewriting");
    append(arena, images, segment_bytes * 2, pattern_four);
    require(jit_executable_bytes() == 4 * segment_bytes,
            "An oversized complete image gets sufficient contiguous storage");
    require(descriptors() == initial_descriptors, "Mapped arenas retain no open descriptors");
  }
  require(jit_executable_mappings() == original && descriptors() == initial_descriptors,
          "Destruction releases every alias and descriptor");
}

void empty_input() {
  detail::ExecutableArena arena;
  const auto mappings = jit_executable_mappings();
  const auto before = codegen::publications();
  bool rejected = false;
  try {
    static_cast<void>(arena.append({}));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && jit_executable_mappings() == mappings && codegen::publications() == before,
          "Empty images fail without allocation or false publication");
}

void failures() {
  for (const auto step :
       {Failure::descriptor, Failure::truncate, Failure::writable, Failure::executable}) {
    detail::ExecutableArena arena;
    const auto mappings = jit_executable_mappings();
    const auto initial_aliases = aliases();
    const auto before = codegen::publications();
    const auto initial_descriptors = descriptors();
    constexpr std::array<std::uint8_t, 1> input{0xc3};
    bool rejected = false;
    failure = step;
    try {
      static_cast<void>(arena.append(input));
    } catch (const std::system_error &error) {
      auto expected = ENOMEM;
      if (step == Failure::descriptor) { expected = EMFILE; }
      if (step == Failure::truncate) { expected = ENOSPC; }
      rejected = error.code().value() == expected;
    }
    failure = Failure::none;
    require(rejected && jit_executable_mappings() == mappings && aliases() == initial_aliases &&
                codegen::publications() == before && descriptors() == initial_descriptors,
            "Every failed allocation stage releases all partial resources and keeps its errno");
    std::vector<Image> images;
    append(arena, images, input.size(), input.front());
    require(codegen::publications().count == before.count + 1,
            "The same arena can publish normally after an allocation failure");
  }
}
} // namespace

extern "C" int real_memfd(const char *name, unsigned flags) noexcept asm("__real_memfd_create");
extern "C" int wrap_memfd(const char *name, unsigned flags) noexcept asm("__wrap_memfd_create");

int wrap_memfd(const char *const name, const unsigned flags) noexcept {
  if (failure == Failure::descriptor) {
    errno = EMFILE;
    return -1;
  }
  return real_memfd(name, flags);
}

extern "C" int real_truncate(int descriptor, off_t length) noexcept asm("__real_ftruncate");
extern "C" int wrap_truncate(int descriptor, off_t length) noexcept asm("__wrap_ftruncate");

int wrap_truncate(const int descriptor, const off_t length) noexcept {
  if (failure == Failure::truncate) {
    errno = ENOSPC;
    return -1;
  }
  return real_truncate(descriptor, length);
}

extern "C" void *real_map(void *address,
                          std::size_t size,
                          int protection,
                          int flags,
                          int descriptor,
                          off_t offset) noexcept asm("__real_mmap");
extern "C" void *wrap_map(void *address,
                          std::size_t size,
                          int protection,
                          int flags,
                          int descriptor,
                          off_t offset) noexcept asm("__wrap_mmap");

void *wrap_map(void *const address,
               const std::size_t size,
               const int protection,
               const int flags,
               const int descriptor,
               const off_t offset) noexcept {
  if ((failure == Failure::writable && (protection & PROT_WRITE) != 0) ||
      (failure == Failure::executable && (protection & PROT_EXEC) != 0)) {
    errno = ENOMEM;
    return MAP_FAILED;
  }
  return real_map(address, size, protection, flags, descriptor, offset);
}
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::packing();
    dsp56300::test::empty_input();
    dsp56300::test::failures();
    dsp56300::test::require(dsp56300::test::aliases() == 0,
                            "Every RW and RX alias is released after success and failure");
    std::cout << dsp56300::test::checks << " compact native arena checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
