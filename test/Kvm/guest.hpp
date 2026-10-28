// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>

namespace dsp56300::test::kvm {
enum class RegisterTransfer : std::uint8_t { automatic, ioctls, shared };

class Unavailable : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct Exit {
  std::uint32_t reason = 0;
  std::uint64_t result = 0, rip = 0, address = 0;
  std::uint32_t length = 0;
  bool write = false;
};

// Experimental x86-64 test guest, not a public execution backend or guest OS.
// Guest addresses and host addresses refer to the same backing pages, but have
// different numerical values. Never dereference a guest pointer on the host.
class Guest {
public:
  static constexpr std::size_t memory_size = std::size_t{2} * 1024 * 1024;
  static constexpr std::size_t code_address = 0x10000, halt_address = 0x30000;
  static constexpr std::size_t cpu_address = 0x20000, context_address = 0x21000;

  explicit Guest(RegisterTransfer transfer = RegisterTransfer::automatic);
  ~Guest();
  Guest(const Guest &) = delete;
  Guest &operator=(const Guest &) = delete;
  Guest(Guest &&) = delete;
  Guest &operator=(Guest &&) = delete;
  [[nodiscard]] void *address(std::size_t offset, std::size_t length);
  [[nodiscard]] bool shared_registers() const noexcept;
  void prepare(std::span<const std::uint8_t> code);
  [[nodiscard]] Exit enter();
  [[nodiscard]] Exit run(std::span<const std::uint8_t> code);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace dsp56300::test::kvm
