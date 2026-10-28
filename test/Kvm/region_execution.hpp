// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "direct.hpp"
#include "guest.hpp"
#include "jit_emitter.hpp"

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <linux/kvm.h>
#include <memory>
#include <span>
#include <stdexcept>

namespace dsp56300::test::kvm::region {
class DirectRegion {
public:
  explicit DirectRegion(const std::span<const std::uint8_t> code) : code_(code) {}

  [[nodiscard]] detail::Cpu &cpu() noexcept { return cpu_; }

  [[nodiscard]] detail::NativeContext &context() noexcept { return context_; }

  [[nodiscard]] std::uint32_t enter() noexcept { return code_.enter(cpu_, context_); }

private:
  detail::Cpu cpu_;
  Direct code_;
  detail::NativeContext context_;
};

class ProtectedRegion {
public:
  ProtectedRegion(const std::span<const std::uint8_t> code, const RegisterTransfer transfer)
      : guest_(transfer) {
    cpu_ = std::construct_at(
        static_cast<detail::Cpu *>(guest_.address(Guest::cpu_address, sizeof(detail::Cpu))));
    context_ = std::construct_at(static_cast<detail::NativeContext *>(
        guest_.address(Guest::context_address, sizeof(detail::NativeContext))));
    guest_.prepare(code);
  }

  [[nodiscard]] detail::Cpu &cpu() noexcept { return *cpu_; }

  [[nodiscard]] detail::NativeContext &context() noexcept { return *context_; }

  [[nodiscard]] bool shared_registers() const noexcept { return guest_.shared_registers(); }

  [[nodiscard]] std::uint32_t enter() {
    const auto exit = guest_.enter();
    if (exit.reason != KVM_EXIT_HLT || exit.rip != Guest::halt_address + 1 ||
        exit.result > std::numeric_limits<std::uint32_t>::max()) {
      throw std::runtime_error("Translated KVM region did not return to its halt boundary");
    }
    return static_cast<std::uint32_t>(exit.result);
  }

private:
  Guest guest_;
  detail::Cpu *cpu_ = nullptr;
  detail::NativeContext *context_ = nullptr;
};

// Experimental execution contract. This is not the public consumer VM backend.
template <typename Runtime>
concept RegionExecutor = requires(Runtime &runtime) {
  { runtime.cpu() } -> std::same_as<detail::Cpu &>;
  { runtime.context() } -> std::same_as<detail::NativeContext &>;
  { runtime.enter() } -> std::same_as<std::uint32_t>;
};

static_assert(RegionExecutor<DirectRegion> && RegionExecutor<ProtectedRegion>);
} // namespace dsp56300::test::kvm::region
