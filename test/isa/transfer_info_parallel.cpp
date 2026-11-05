// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "instruction_decoder.hpp"
#include "transfer_info_support.hpp"
#include "transfers.hpp"

#include <array>

namespace dsp56300::test::transfer_info_test {
namespace {
constexpr unsigned x_shift = 18, y_mode_shift = 20, split_shift = 17, y_acc_shift = 19;
constexpr std::array xregs{RegisterId::x0, RegisterId::x1, RegisterId::a, RegisterId::b};
constexpr std::array yregs{RegisterId::y0, RegisterId::y1, RegisterId::a, RegisterId::b};

void pair_case(const unsigned ordinal) {
  const auto xr = ordinal & 3U;
  const auto yr = (ordinal >> 2) & 3U;
  const auto xi = (ordinal >> 4) & 7U;
  const auto yi = ((ordinal >> 7) & 3U) + (xi < 4 ? 4U : 0U);
  const auto xm = (ordinal >> 9) & 3U;
  const auto ym = (ordinal >> 11) & 3U;
  const bool xl = (ordinal & (1U << 13)) != 0;
  const bool yl = (ordinal & (1U << 14)) != 0;
  const Word opcode = 0x800000U | (xr << x_shift) | (yr << 16) | (xi << 8) | ((yi % 4) << 13) |
                      (xm << 11) | (ym << y_mode_shift) | (xl ? 0x8000U : 0U) |
                      (yl ? 0x400000U : 0U);
  const auto xmemory = ea(TransferSpace::x, {.mode = xm == 0 ? 4U : xm, .index = xi});
  const auto ymemory = ea(TransferSpace::y, {.mode = ym == 0 ? 4U : ym, .index = yi});
  verify(opcode,
         expected(InstructionKind::parallel,
                  1,
                  {directed({.source = xmemory, .destination = reg(xregs.at(xr))}, xl),
                   directed({.source = ymemory, .destination = reg(yregs.at(yr))}, yl)}));
}

void long_case(const unsigned ordinal) {
  constexpr std::array views{RegisterId::a10,
                             RegisterId::b10,
                             RegisterId::x,
                             RegisterId::y,
                             RegisterId::a,
                             RegisterId::b,
                             RegisterId::ab,
                             RegisterId::ba};
  const auto view = ordinal & 7U;
  const auto field = (ordinal >> 3) & 63U;
  const bool load = (ordinal & (1U << 9)) != 0;
  const bool extended = (ordinal & (1U << 10)) != 0;
  const auto mode = field / 8;
  const auto index = field % 8;
  const Word opcode = 0x400000U | ((view & 3U) << 16) | ((view & 4U) << split_shift) |
                      (field << 8) | (load ? 0x8000U : 0U) | (extended ? 0x4000U : 0U);
  if (extended && mode == 6 && index != 0) {
    unsupported(opcode);
    return;
  }
  const auto operand = extended ? ea(TransferSpace::long_pair, {.mode = mode, .index = index})
                                : absolute(TransferSpace::long_pair, field);
  verify(opcode,
         expected(InstructionKind::parallel,
                  extended && mode == 6 ? 2U : 1U,
                  {directed({.source = operand, .destination = reg(views.at(view))}, load)}));
}

void mixed_case(const unsigned ordinal) {
  const auto first = ordinal & 3U;
  const bool accumulator_b = (ordinal & 4U) != 0;
  const auto secondary = (ordinal >> 3) & 1U;
  const bool load = (ordinal & 16U) != 0;
  const bool y = (ordinal & 32U) != 0;
  const auto field = (ordinal >> 6) & 63U;
  const auto mode = field / 8;
  const auto index = field % 8;
  const Word opcode = 0x100000U | (first << (y ? 16U : x_shift)) |
                      (accumulator_b ? 1U << (y ? y_acc_shift : split_shift) : 0U) |
                      (secondary << (y ? x_shift : 16U)) | (load ? 0x8000U : 0U) |
                      (y ? 0x4000U : 0U) | (field << 8);
  if (mode == 6 && index != 0 && (!load || index != 4)) {
    unsupported(opcode);
    return;
  }
  const auto operand = mode == 6 && index == 4 ? extension()
                                               : ea(y ? TransferSpace::y : TransferSpace::x,
                                                    {.mode = mode, .index = index});
  const auto target = y ? yregs.at(first) : xregs.at(first);
  const auto other = y ? xregs.at(secondary) : yregs.at(secondary);
  verify(opcode,
         expected(InstructionKind::parallel,
                  mode == 6 ? 2U : 1U,
                  {directed({.source = operand, .destination = reg(target)}, load),
                   {.source = reg(accumulator_b ? RegisterId::b : RegisterId::a),
                    .destination = reg(other)}}));
}

void class_two_case(const unsigned ordinal) {
  const auto field = ordinal & 63U;
  const bool b = (ordinal & 64U) != 0;
  const bool y = (ordinal & 128U) != 0;
  const auto mode = field / 8;
  const auto index = field % 8;
  const Word opcode = 0x080000U | (b ? 0x10000U : 0U) | (y ? 0x8000U : 0U) | (field << 8);
  if (mode == 6) {
    unsupported(opcode);
    return;
  }
  const auto accumulator = reg(b ? RegisterId::b : RegisterId::a);
  verify(
      opcode,
      expected(InstructionKind::parallel,
               1,
               {{.source = accumulator,
                 .destination =
                     ea(y ? TransferSpace::y : TransferSpace::x, {.mode = mode, .index = index})},
                {.source = reg(y ? RegisterId::y0 : RegisterId::x0), .destination = accumulator}}));
}
} // namespace

void paired() {
  for (unsigned ordinal = 0; ordinal < 32768; ++ordinal) { pair_case(ordinal); }
}

void long_moves() {
  for (unsigned ordinal = 0; ordinal < 2048; ++ordinal) { long_case(ordinal); }
}

void mixed() {
  for (unsigned ordinal = 0; ordinal < 4096; ++ordinal) { mixed_case(ordinal); }
  for (unsigned ordinal = 0; ordinal < 256; ++ordinal) { class_two_case(ordinal); }
}
} // namespace dsp56300::test::transfer_info_test
