// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"

#include <array>
#include <optional>

namespace dsp56300::detail {
inline constexpr unsigned displacement_mode = 8;

[[nodiscard]] constexpr bool addressed_move(const Operation kind) noexcept {
  return kind == Operation::move_displacement || kind == Operation::move_control_effective ||
         kind == Operation::move_control_short;
}

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
  bool long_pair = false;
};

struct NativeRegisterMove {
  unsigned source = 0, destination = 0;

  [[nodiscard]] bool valid() const noexcept { return source != 0; }
};

struct NativeRegisterImmediate {
  unsigned destination = 0;
  Word value = 0;

  [[nodiscard]] bool valid() const noexcept { return destination != 0; }
};

// Compile-time transfer description, independent of the host instruction set.
[[nodiscard]] std::optional<AddressForm> plan_address_update(const Instruction &instruction);
[[nodiscard]] NativeRegisterImmediate plan_register_immediate(const Instruction &instruction);
[[nodiscard]] NativeMoves plan_moves(const Instruction &instruction);
[[nodiscard]] NativeRegisterMove plan_register_move(const Instruction &instruction);
[[nodiscard]] NativeRegisterMove plan_register_move(const Instruction &instruction, bool partial);
} // namespace dsp56300::detail
