// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "jit/arena.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>
#if defined(__aarch64__)
#include <bit>
#endif
#if defined(__linux__)
#include <fstream>
#include <string>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

namespace {
using dsp56300::detail::ExecutableArena;
extern "C" std::uint32_t dsp56300_executable_arena_test_enter(const void *) noexcept;
#if defined(__x86_64__) && defined(__linux__)
asm(".pushsection .text\n"
    ".globl dsp56300_executable_arena_test_enter\n"
    ".hidden dsp56300_executable_arena_test_enter\n"
    ".type dsp56300_executable_arena_test_enter,@function\n"
    "dsp56300_executable_arena_test_enter:\n"
    "endbr64\n"
    "jmp *%rdi\n"
    ".size dsp56300_executable_arena_test_enter,.-dsp56300_executable_arena_test_enter\n"
    ".popsection\n");
#elif defined(__aarch64__) && defined(__APPLE__)
asm(".text\n"
    ".globl _dsp56300_executable_arena_test_enter\n"
    "_dsp56300_executable_arena_test_enter:\n"
    "br x0\n");
#elif defined(__aarch64__) && defined(__linux__)
asm(".text\n"
    ".globl dsp56300_executable_arena_test_enter\n"
    ".type dsp56300_executable_arena_test_enter,%function\n"
    "dsp56300_executable_arena_test_enter:\n"
    "br x0\n");
#endif

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

auto return_code(const std::uint32_t value) {
#if defined(__aarch64__)
  constexpr std::uint32_t movz = 0x52800000;
  constexpr std::uint32_t movk = 0x72a00000;
  constexpr std::uint32_t ret = 0xd65f03c0;
  constexpr std::uint32_t nop = 0xd503201f;
  constexpr unsigned immediate_shift = 5;
  constexpr unsigned high_half = 16;
  constexpr unsigned half_mask = 0xffff;
  const std::array words{movz | ((value & half_mask) << immediate_shift),
                         movk | ((value >> high_half) << immediate_shift),
                         ret,
                         nop};
  return std::bit_cast<std::array<std::uint8_t, sizeof(words)>>(words);
#else
  // Native test function: ENDBR64; MOV EAX, immediate; RET (not guest opcodes).
  constexpr std::uint8_t endbr_prefix = 0xf3;
  constexpr std::uint8_t escape = 0x0f;
  constexpr std::uint8_t endbr = 0x1e;
  constexpr std::uint8_t endbr64 = 0xfa;
  constexpr std::uint8_t mov_eax = 0xb8;
  constexpr std::uint8_t ret = 0xc3;
  constexpr unsigned byte_bits = 8;
  constexpr unsigned byte_mask = 0xff;
  return std::array<std::uint8_t, 10>{
      endbr_prefix,
      escape,
      endbr,
      endbr64,
      mov_eax,
      static_cast<std::uint8_t>(value & byte_mask),
      static_cast<std::uint8_t>((value >> byte_bits) & byte_mask),
      static_cast<std::uint8_t>((value >> (2U * byte_bits)) & byte_mask),
      static_cast<std::uint8_t>(value >> (3U * byte_bits)),
      ret};
#endif
}

void mapping_permissions() {
#if defined(__linux__)
  std::ifstream mappings("/proc/self/maps");
  require(mappings.is_open(), "Cannot inspect executable mappings on this Linux host");
  unsigned writable = 0;
  unsigned executable = 0;
  for (std::string line; std::getline(mappings, line);) {
    if (line.find("memfd:dsp56300-jit") == std::string::npos) { continue; }
    const auto start = line.find(' ') + 1;
    const auto permissions = line.substr(start, 4);
    require(permissions == "rw-s" || permissions == "r-xs",
            "JIT storage has separate RW and RX shared aliases, never RWX");
    if (permissions == "rw-s") { ++writable; }
    if (permissions == "r-xs") { ++executable; }
  }
  require(writable == 2 && executable == 2,
          "Two arenas own exactly two writable and two executable mappings");
#elif defined(__APPLE__)
  require(pthread_jit_write_protect_supported_np() != 0,
          "Apple silicon must enforce thread-local JIT write protection");
#endif
}

void packing_and_growth() {
  constexpr unsigned entries = 512;
  constexpr std::size_t arena_bytes = std::size_t{64} << 10;
  constexpr std::uintptr_t alignment = 16;
  ExecutableArena arena;
  std::array<const void *, entries> functions{};
  for (unsigned index = 0; index < entries; ++index) {
    functions[index] = arena.append(return_code(index));
    const auto address = reinterpret_cast<std::uintptr_t>(functions[index]);
    const auto first = reinterpret_cast<std::uintptr_t>(functions[0]);
    require(address - first == index * alignment,
            "Small immutable entries share contiguous aligned arena storage");
  }
  std::vector<std::uint8_t> large(2 * arena_bytes);
  constexpr unsigned result = 0x12345678;
  const auto code = return_code(result);
  std::ranges::copy(code, large.begin());
  const auto *large_function = arena.append(large);
  require(dsp56300_executable_arena_test_enter(large_function) == result,
          "Large native entry executes correctly");
  for (unsigned index = 0; index < entries; ++index) {
    require(dsp56300_executable_arena_test_enter(functions[index]) == index,
            "Every previous entry remains unchanged and executable after arena growth");
  }
  mapping_permissions();
  bool rejected = false;
  try {
    static_cast<void>(arena.append({}));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Empty code must be refused");
  const auto published = dsp56300::test::codegen::publications();
  require(published.count == entries + 1, "Observe every successful publication exactly once");
  const auto images = dsp56300::test::codegen::published_images();
  require(images.size() == published.count && images.back().entry == large_function &&
              images.back().bytes == large.size(),
          "Publication observation retains the actual executable entry and image size");
}
}

int main() {
  try {
    dsp56300::test::codegen::track_images();
    packing_and_growth();
    std::cout << "Aligned compact immutable native entries, growth and W^X aliases passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
