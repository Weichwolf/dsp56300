// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"
#include "instruction.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <optional>

namespace dsp56300 {
using namespace detail;

namespace {
constexpr Word unsupported_numeric_modes =
    compatibility_mode | multiply_mode | arithmetic_16 | arithmetic_saturation;
constexpr std::array x_registers{x0_register, x1_register, a_register, b_register};
constexpr std::array y_registers{y0_register, y1_register, a_register, b_register};
constexpr Word store_to_register = Word{1} << 15;
constexpr Word effective_address_field = Word{1} << 14;
constexpr Word y_space_field = Word{1} << 19;
constexpr Word parallel_pair_field = Word{1} << 23;

AddressForm address_form(const Instruction &decoded) noexcept {
  return {.mode = (decoded.opcode >> 11) & 7,
          .index = (decoded.opcode >> 8) & 7,
          .extra = decoded.extension};
}

std::uint64_t signed_long(const std::uint64_t value) noexcept {
  constexpr auto sign = std::uint64_t{1} << (double_word_bits - 1);
  return (value & sign) != 0 ? value | ~long_mask : value;
}
}

std::uint64_t Core::Impl::transfer_long(const std::uint64_t value,
                                        const std::optional<unsigned> source_register) {
  static_cast<void>(transfer_word(value, source_register));
  const unsigned shift = padding_bits + ((cpu.reg.sr & scale_down) != 0 ? 1U : 0U) -
                         ((cpu.reg.sr & scale_up) != 0 ? 1U : 0U);
  const auto scaled = std::bit_cast<std::int64_t>(value) >> shift;
  constexpr auto minimum = -(std::int64_t{1} << (double_word_bits - 1));
  constexpr auto maximum = (std::int64_t{1} << (double_word_bits - 1)) - 1;
  return static_cast<std::uint64_t>(std::clamp(scaled, minimum, maximum)) & long_mask;
}

bool Core::Impl::numeric_mode_supported(const Word ignored_modes) {
  if ((cpu.reg.sr & (unsupported_numeric_modes & ~ignored_modes)) != 0 ||
      (cpu.reg.sr & (scale_up | scale_down)) == (scale_up | scale_down)) {
    fail(FaultKind::unsupported_mode, Space::program, current_pc, cpu.reg.sr);
    return false;
  }
  return true;
}

void Core::Impl::parallel(const Instruction &decoded) {
  constexpr Word condition_mask = 0xffe000;
  constexpr Word condition_group = 0x202000;
  if ((decoded.opcode & condition_mask) == condition_group) {
    conditional_alu(decoded.opcode);
    return;
  }
  const bool supported = (cpu.reg.sr & arithmetic_saturation) != 0
                             ? numeric_alu_supported(decoded.opcode & byte_mask)
                             : numeric_mode_supported();
  if (!supported) { return; }
  if ((decoded.opcode & byte_mask) == 0) {
    moves(decoded);
    return;
  }
  latched = {cpu.reg.a, cpu.reg.b, cpu.reg.x, cpu.reg.y};
  moves(decoded);
  if (fault.kind == FaultKind::none) { alu(decoded.opcode & byte_mask); }
}

void Core::Impl::register_move(const Word opcode) {
  if ((opcode & register_group_mask) == no_move_group) {
    const auto value = read_move_register((opcode >> 13) & five_bit_mask);
    write_move_register((opcode >> 8) & five_bit_mask, value);
    return;
  }
  const auto target = (opcode >> 16) & five_bit_mask;
  auto value = (opcode >> 8) & byte_mask;
  if (target == x0_register || target == x1_register || target == y0_register ||
      target == y1_register || target == a_register || target == b_register) {
    value <<= 16;
  }
  write_move_register(target, value);
}

void Core::Impl::paired_move(const Word opcode) {
  const auto xreg = x_registers[(opcode >> 18) & 3];
  const auto yreg = y_registers[(opcode >> 16) & 3];
  const auto xmode = (opcode >> 11) & 3;
  const auto ymode = (opcode >> 20) & 3;
  const auto index = (opcode >> 8) & 7;
  const auto yindex = ((opcode >> 13) & 3) + (index < 4 ? 4U : 0U);
  const auto xa = effective({.mode = xmode == 0 ? 4 : xmode, .index = index});
  const auto ya = effective({.mode = ymode == 0 ? 4 : ymode, .index = yindex});
  const bool load = (opcode & store_to_register) != 0;
  const bool yload = (opcode & (1U << 22)) != 0;
  const auto xv = load ? read_memory(Space::x, xa) : read_move_register(xreg);
  if (xa >= io_begin && ya >= io_begin) {
    if (load) {
      write_move_register(xreg, xv);
    } else {
      write_memory(Space::x, xa, xv);
    }
    return;
  }
  const auto yv = yload ? read_memory(Space::y, ya) : read_move_register(yreg);
  if (load) {
    write_move_register(xreg, xv);
  } else {
    write_memory(Space::x, xa, xv);
  }
  if (yload) {
    write_move_register(yreg, yv);
  } else {
    write_memory(Space::y, ya, yv);
  }
}

void Core::Impl::memory_move(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  const auto form = address_form(decoded);
  const auto space = (opcode & y_space_field) != 0 ? Space::y : Space::x;
  const unsigned reg = ((opcode >> 16) & 7) | ((opcode >> 17) & high_register_mask);
  const bool extended = (opcode & effective_address_field) != 0;
  const bool load = (opcode & store_to_register) != 0;
  const auto address = extended ? effective(form) : (opcode >> 8) & six_bit_mask;
  if (fault.kind != FaultKind::none) { return; }
  if (!load) {
    write_memory(space, address, read_move_register(reg));
    return;
  }
  if (extended && form.mode == 6 && form.index == 4) {
    write_move_register(reg, decoded.extension);
  } else {
    const auto value = read_memory(space, address);
    if (fault.kind == FaultKind::none) { write_move_register(reg, value); }
  }
}

void Core::Impl::displacement_move(const Instruction &decoded) {
  if (!numeric_mode_supported(arithmetic_saturation)) { return; }
  const auto opcode = decoded.opcode;
  const bool long_form = decoded.words == 2;
  const auto reg = opcode & (long_form ? six_bit_mask : Word{15});
  const bool valid = (reg >= x0_register && reg < m_register_base + agu_registers) ||
                     reg == ep_register || reg == vba_register || reg == sc_register ||
                     reg >= sz_register;
  if (!valid) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const auto index = (opcode >> 8) & 7U;
  Word displacement = decoded.extension;
  if (!long_form) {
    constexpr Word upper_displacement = 0x7e;
    constexpr Word sign = 0x40;
    constexpr Word short_mask = 0x7f;
    displacement = ((opcode >> 10) & upper_displacement) | ((opcode >> 6) & 1U);
    if ((displacement & sign) != 0) { displacement |= word_mask & ~short_mask; }
  }
  const auto offset = (cpu.reg.m[index] & modifier_mask) == 0
                          ? static_cast<std::int32_t>(displacement)
                          : signed_word(displacement);
  const auto address = update_address(
      {.index = index, .offset = offset, .positive_operand = signed_word(displacement) > 0});
  if (fault.kind != FaultKind::none) { return; }
  const auto space = ((opcode >> (long_form ? 16U : 5U)) & 1U) != 0 ? Space::y : Space::x;
  const bool load = ((opcode >> (long_form ? 6U : 4U)) & 1U) != 0;
  if (load) {
    const auto value = read_memory(space, address);
    if (fault.kind == FaultKind::none) { write_move_register(reg, value); }
  } else {
    const auto value = read_move_register(reg);
    if (fault.kind == FaultKind::none) { write_memory(space, address, value); }
  }
}

void Core::Impl::load_long(const Instruction &decoded, const std::uint64_t value) {
  const auto selector = ((decoded.opcode >> 16) & 3) | ((decoded.opcode >> 17) & 4);
  auto update = move_publication() ? long_destination(selector) : MoveWriteback{};
  capture_move(update);
  switch (selector) {
    case 0: cpu.reg.a = (cpu.reg.a & ~(long_mask << padding_bits)) | (value << padding_bits); break;
    case 1: cpu.reg.b = (cpu.reg.b & ~(long_mask << padding_bits)) | (value << padding_bits); break;
    case 2: cpu.reg.x = value; break;
    case 3: cpu.reg.y = value; break;
    case 4: cpu.reg.a = signed_long(value) << padding_bits; break;
    case 5: cpu.reg.b = signed_long(value) << padding_bits; break;
    case 6:
      write_move_register(a_register, static_cast<Word>(value >> word_bits));
      write_move_register(b_register, static_cast<Word>(value) & word_mask);
      break;
    case 7:
      write_move_register(b_register, static_cast<Word>(value >> word_bits));
      write_move_register(a_register, static_cast<Word>(value) & word_mask);
      break;
    default: break;
  }
  publish_move(update);
}

void Core::Impl::long_move(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  const auto selector = ((opcode >> 16) & 3) | ((opcode >> 17) & 4);
  const bool load = (opcode & store_to_register) != 0;
  const bool extended = (opcode & effective_address_field) != 0;
  const auto form = address_form(decoded);
  if (extended && form.mode == 6 && form.index != 0) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, form.index);
    return;
  }
  const auto address = extended ? effective(form) : (opcode >> 8) & six_bit_mask;
  if (fault.kind != FaultKind::none) { return; }
  if (load) {
    const auto high = read_memory(Space::x, address);
    if (fault.kind != FaultKind::none) { return; }
    const auto low = read_memory(Space::y, address);
    if (fault.kind != FaultKind::none) { return; }
    load_long(decoded, (std::uint64_t{high} << word_bits) | low);
    return;
  }
  if (selector >= 6) {
    const auto first = read_move_register(selector == 6 ? a_register : b_register);
    const auto second = read_move_register(selector == 6 ? b_register : a_register);
    write_memory(Space::x, address, first);
    if (fault.kind != FaultKind::none) { return; }
    write_memory(Space::y, address, second);
    return;
  }
  std::uint64_t value = 0;
  if (selector == 2) {
    value = cpu.reg.x;
  } else if (selector == 3) {
    value = cpu.reg.y;
  } else {
    const auto accumulator = (selector & 1) == 0 ? cpu.reg.a : cpu.reg.b;
    value = accumulator >> padding_bits;
    if (selector >= 4) { value = transfer_long(accumulator, a_register + (selector & 1U)); }
  }
  write_memory(Space::x, address, static_cast<Word>(value >> word_bits) & word_mask);
  if (fault.kind != FaultKind::none) { return; }
  write_memory(Space::y, address, static_cast<Word>(value) & word_mask);
}

void Core::Impl::mixed_move(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  const auto form = address_form(decoded);
  const bool yspace = (opcode & effective_address_field) != 0;
  const bool load = (opcode & store_to_register) != 0;
  if (form.mode == 6 && form.index == 4 && !load) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, form.index);
    return;
  }
  const unsigned source_acc = (opcode & (1U << (yspace ? 19 : 17))) != 0 ? b_register : a_register;
  const unsigned second_target =
      yspace ? x0_register + ((opcode >> 18) & 1) : y0_register + ((opcode >> 16) & 1);
  const unsigned first_target =
      yspace ? y_registers[(opcode >> 16) & 3] : x_registers[(opcode >> 18) & 3];
  const auto address = effective(form);
  if (fault.kind != FaultKind::none) { return; }
  const auto secondary = read_move_register(source_acc);
  const auto space = yspace ? Space::y : Space::x;
  if (!load) {
    write_memory(space, address, read_move_register(first_target));
  } else if (form.mode == 6 && form.index == 4) {
    write_move_register(first_target, decoded.extension);
  } else {
    const auto value = read_memory(space, address);
    if (fault.kind != FaultKind::none) { return; }
    write_move_register(first_target, value);
  }
  if (fault.kind == FaultKind::none) { write_move_register(second_target, secondary); }
}

void Core::Impl::class_two_move(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  const auto form = address_form(decoded);
  if (form.mode == 6) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, opcode);
    return;
  }
  const bool yspace = (opcode & store_to_register) != 0;
  const unsigned accumulator = (opcode & (1U << 16)) != 0 ? b_register : a_register;
  const auto address = effective(form);
  if (fault.kind != FaultKind::none) { return; }
  const auto value = read_move_register(accumulator);
  const auto operand = read_move_register(yspace ? y0_register : x0_register);
  write_memory(yspace ? Space::y : Space::x, address, value);
  if (fault.kind == FaultKind::none) { write_move_register(accumulator, operand); }
}

void Core::Impl::moves(const Instruction &decoded) {
  const auto opcode = decoded.opcode;
  if ((opcode & upper_move_mask) == no_move_group) { return; }
  if ((opcode & update_group_mask) == update_group) {
    static_cast<void>(effective({.mode = (opcode >> 11) & 3, .index = (opcode >> 8) & 7}));
    return;
  }
  if ((opcode & parallel_group_mask) == immediate_group) {
    register_move(opcode);
    return;
  }
  if ((opcode & parallel_pair_field) != 0) {
    paired_move(opcode);
    return;
  }
  if ((opcode & memory_group_mask) == memory_group) {
    if ((opcode & long_selector_mask) == 0) {
      long_move(decoded);
    } else {
      memory_move(decoded);
    }
    return;
  }
  if ((opcode & pair_group_mask) == pair_group) {
    mixed_move(decoded);
    return;
  }
  if ((opcode & class_two_mask) == class_two) {
    class_two_move(decoded);
    return;
  }
  fail(FaultKind::unsupported_instruction, Space::program, current_pc, opcode);
}
} // namespace dsp56300
