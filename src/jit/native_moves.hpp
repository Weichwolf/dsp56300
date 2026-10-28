// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "instruction.hpp"

#include <array>

namespace dsp56300::detail {
struct NativeMove {
  AddressForm address;
  unsigned reg = 4;
  Space space = Space::x;
  bool load = false;

  [[nodiscard]] bool immediate() const noexcept { return address.mode == 6 && address.index == 4; }
};

struct NativeMoves {
  std::array<NativeMove, 2> transfers;
  unsigned count = 0;
};

struct NativeRegisterMove {
  unsigned source = 0, destination = 0;

  [[nodiscard]] bool valid() const noexcept { return source != 0; }
};

struct NativeAddressImmediate {
  unsigned destination = 0;
  Word value = 0;

  [[nodiscard]] bool valid() const noexcept { return destination != 0; }
};

// Compile-time transfer description, independent of the host instruction set.
[[nodiscard]] bool pure_address_update(const Instruction &instruction);
[[nodiscard]] NativeAddressImmediate plan_address_immediate(const Instruction &instruction);
[[nodiscard]] NativeMoves plan_moves(const Instruction &instruction);
[[nodiscard]] NativeRegisterMove plan_register_move(const Instruction &instruction);
} // namespace dsp56300::detail
