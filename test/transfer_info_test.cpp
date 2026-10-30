// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"
#include "dsp56300/transfers.hpp"
#include "support.hpp"
#include "transfer_info_support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test::transfer_info_test {
namespace {
constexpr Word address_update_prefix = 0x204000;
constexpr unsigned invalid_kind = 0xff;
constexpr unsigned register_count = 28, split_shift = 17;

void scalar_case(const unsigned ordinal) {
  const unsigned selector = 4 + (ordinal % register_count);
  const unsigned address = (ordinal / register_count) & 63U;
  const bool read = ((ordinal / (register_count * 64U)) & 1U) != 0;
  const bool y = ((ordinal / (register_count * 128U)) & 1U) != 0;
  const bool extended = ((ordinal / (register_count * 256U)) & 1U) != 0;
  const auto space = y ? TransferSpace::y : TransferSpace::x;
  const auto mode = address / 8;
  const auto index = address % 8;
  const Word opcode = 0x400000U | ((selector & 24U) << split_shift) | ((selector & 7U) << 16) |
                      (y ? 0x80000U : 0U) | (read ? 0x8000U : 0U) | (extended ? 0x4000U : 0U) |
                      (address << 8);
  if (extended && mode == 6 && index != 0 && (!read || index != 4)) {
    unsupported(opcode);
    return;
  }
  auto operand = absolute(space, address);
  if (extended) {
    operand = mode == 6 && index == 4 ? extension() : ea(space, {.mode = mode, .index = index});
  }
  const unsigned words = extended && mode == 6 ? 2U : 1U;
  verify(
      opcode,
      expected(InstructionKind::parallel,
               words,
               {directed({.source = operand, .destination = reg(registers.at(selector))}, read)}));
}
} // namespace

void scalar() {
  constexpr unsigned combinations = register_count * 64U * 2U * 2U * 2U;
  for (unsigned ordinal = 0; ordinal < combinations; ++ordinal) { scalar_case(ordinal); }
}

void register_moves() {
  constexpr std::array literals{Word{0}, Word{0x80}, Word{0xff}};
  for (unsigned target = 4; target < 32; ++target) {
    for (const auto literal : literals) {
      const bool fractional = target < 8 || target == 14 || target == 15;
      verify(0x200000U | (target << 16) | (literal << 8),
             expected(InstructionKind::parallel,
                      1,
                      {{.source = immediate(literal, fractional ? 16 : 0),
                        .destination = reg(registers.at(target))}}));
    }
    for (unsigned source = 4; source < 32; ++source) {
      for (const Word alu : {Word{0}, Word{0x40}, Word{0xd0}}) {
        auto wanted = expected(
            InstructionKind::parallel,
            1,
            {{.source = reg(registers.at(source)), .destination = reg(registers.at(target))}});
        wanted.parallel_alu = alu;
        verify(0x200000U | (source << 13) | (target << 8) | alu, wanted);
      }
    }
  }
}

void address_updates() {
  for (unsigned mode = 0; mode < 4; ++mode) {
    for (unsigned index = 0; index < 8; ++index) {
      auto wanted = expected(InstructionKind::parallel, 1, {});
      wanted.address_update = {.kind = modes.at(mode), .index = static_cast<std::uint8_t>(index)};
      verify(address_update_prefix | (mode << 11) | (index << 8), wanted);
    }
  }
  constexpr std::array input_kinds{AddressKind::post_decrement_offset,
                                   AddressKind::post_increment_offset,
                                   AddressKind::post_decrement,
                                   AddressKind::post_increment,
                                   AddressKind::indirect,
                                   AddressKind::indexed,
                                   AddressKind::pre_decrement,
                                   AddressKind::displacement,
                                   AddressKind::displacement_extension};
  // Independent EA-expression inputs, not a physical pipeline interlock oracle.
  constexpr std::array n_read{true, true, false, false, false, true, false, false, false};
  constexpr std::array m_read{true, true, true, true, false, true, true, true, true};
  constexpr std::array update{true, true, true, true, false, false, true, false, false};
  for (unsigned kind = 0; kind < input_kinds.size(); ++kind) {
    for (unsigned index = 0; index < 8; ++index) {
      const auto mask = static_cast<std::uint8_t>(1U << index);
      const AddressInputs wanted{.r = mask,
                                 .n = n_read.at(kind) ? mask : std::uint8_t{0},
                                 .m = m_read.at(kind) ? mask : std::uint8_t{0},
                                 .updated_r = update.at(kind) ? mask : std::uint8_t{0}};
      require(address_inputs({.kind = input_kinds.at(kind),
                              .index = static_cast<std::uint8_t>(index)}) == wanted,
              "Every R/N/M input and address-update role follows the independent EA table");
    }
  }
  require(address_inputs({}) == AddressInputs{} &&
              address_inputs({.kind = AddressKind::absolute_extension}) == AddressInputs{} &&
              !address_inputs({.kind = AddressKind::indirect, .index = 8}) &&
              !address_inputs({.kind = static_cast<AddressKind>(invalid_kind)}),
          "Absolute/extension addresses have no AGU inputs; malformed descriptors fail visibly");
}
} // namespace dsp56300::test::transfer_info_test

int main() {
  try {
    using namespace dsp56300::test::transfer_info_test;
    scalar();
    register_moves();
    address_updates();
    paired();
    mixed();
    long_moves();
    control();
    program();
    peripheral();
    displacement();
    literals();
    std::cout << dsp56300::test::checks << " data transfer assertions passed across " << cases
              << " independently described encodings\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
