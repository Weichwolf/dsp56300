// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/transfers.hpp"
#include "fields.hpp"
#include "transfer_info.hpp"

#include <array>

namespace dsp56300::detail::transfer_info {
namespace {
constexpr unsigned split_shift = 17, x_pair_shift = 18, y_pair_shift = 20, y_acc_shift = 19;
constexpr Word pair_bit = 0x800000, y_load_bit = 0x400000;
constexpr Word load_bit = 0x8000, effective_bit = 0x4000, y_bit = 0x80000;
constexpr std::array xregs{RegisterId::x0, RegisterId::x1, RegisterId::a, RegisterId::b};
constexpr std::array yregs{RegisterId::y0, RegisterId::y1, RegisterId::a, RegisterId::b};

void directed(Builder &builder, const EncodedTransfer transfer, const bool load) noexcept {
  builder.move(
      load ? transfer
           : EncodedTransfer{.source = transfer.destination, .destination = transfer.source});
}

void register_move(Builder &builder, const Word opcode) noexcept {
  if ((opcode & register_group_mask) == no_move_group) {
    builder.move({.source = builder.reg((opcode >> 13) & five_bit_mask),
                  .destination = builder.reg((opcode >> 8) & five_bit_mask)});
    return;
  }
  const auto selector = (opcode >> 16) & five_bit_mask;
  const auto target = builder.reg(selector);
  const bool fractional = target.reg == RegisterId::x0 || target.reg == RegisterId::x1 ||
                          target.reg == RegisterId::y0 || target.reg == RegisterId::y1 ||
                          target.reg == RegisterId::a || target.reg == RegisterId::b;
  builder.move({.source = Builder::literal(
                    {.value = (opcode >> 8) & byte_mask, .shift = fractional ? 16U : 0U}),
                .destination = target});
}

void scalar(Builder &builder, const Word opcode) noexcept {
  const bool load = (opcode & load_bit) != 0;
  const auto space = (opcode & y_bit) != 0 ? TransferSpace::y : TransferSpace::x;
  const auto selector = ((opcode >> 16) & 7U) | ((opcode >> split_shift) & high_register_mask);
  const auto memory = (opcode & effective_bit) != 0
                          ? builder.effective(space, effective_encoding(opcode), load)
                          : Builder::memory(space, {.literal = (opcode >> 8) & six_bit_mask});
  directed(builder, {.source = memory, .destination = builder.reg(selector)}, load);
}

void paired(Builder &builder, const Word opcode) noexcept {
  const auto xmode = (opcode >> 11) & 3U;
  const auto ymode = (opcode >> y_pair_shift) & 3U;
  const auto index = (opcode >> 8) & 7U;
  const auto yindex = ((opcode >> 13) & 3U) + (index < 4 ? 4U : 0U);
  const auto xmemory =
      builder.effective(TransferSpace::x, {.mode = xmode == 0 ? 4U : xmode, .index = index});
  const auto ymemory =
      builder.effective(TransferSpace::y, {.mode = ymode == 0 ? 4U : ymode, .index = yindex});
  directed(builder,
           {.source = xmemory, .destination = Builder::view(xregs[(opcode >> x_pair_shift) & 3U])},
           (opcode & load_bit) != 0);
  directed(builder,
           {.source = ymemory, .destination = Builder::view(yregs[(opcode >> 16) & 3U])},
           (opcode & y_load_bit) != 0);
}

void long_move(Builder &builder, const Word opcode) noexcept {
  constexpr std::array views{RegisterId::a10,
                             RegisterId::b10,
                             RegisterId::x,
                             RegisterId::y,
                             RegisterId::a,
                             RegisterId::b,
                             RegisterId::ab,
                             RegisterId::ba};
  const auto selector = ((opcode >> 16) & 3U) | ((opcode >> split_shift) & 4U);
  const auto memory =
      (opcode & effective_bit) != 0
          ? builder.effective(TransferSpace::long_pair, effective_encoding(opcode))
          : Builder::memory(TransferSpace::long_pair, {.literal = (opcode >> 8) & six_bit_mask});
  directed(builder,
           {.source = memory, .destination = Builder::view(views[selector])},
           (opcode & load_bit) != 0);
}

void mixed(Builder &builder, const Word opcode) noexcept {
  const bool y = (opcode & effective_bit) != 0;
  const bool load = (opcode & load_bit) != 0;
  const auto accumulator =
      (opcode & (1U << (y ? y_acc_shift : split_shift))) != 0 ? RegisterId::b : RegisterId::a;
  const auto secondary =
      y ? ((opcode >> x_pair_shift) & 1U) + x0_register : ((opcode >> 16) & 1U) + y0_register;
  const auto target = y ? yregs[(opcode >> 16) & 3U] : xregs[(opcode >> x_pair_shift) & 3U];
  const auto memory =
      builder.effective(y ? TransferSpace::y : TransferSpace::x, effective_encoding(opcode), load);
  directed(builder, {.source = memory, .destination = Builder::view(target)}, load);
  builder.move({.source = Builder::view(accumulator), .destination = builder.reg(secondary)});
}

void class_two_move(Builder &builder, const Word opcode) noexcept {
  const auto encoded = effective_encoding(opcode);
  if (encoded.mode == 6) {
    builder.valid = false;
    return;
  }
  const bool y = (opcode & load_bit) != 0;
  const auto accumulator =
      Builder::view((opcode & (1U << 16)) != 0 ? RegisterId::b : RegisterId::a);
  builder.move(
      {.source = accumulator,
       .destination = builder.effective(y ? TransferSpace::y : TransferSpace::x, encoded)});
  builder.move(
      {.source = Builder::view(y ? RegisterId::y0 : RegisterId::x0), .destination = accumulator});
}
} // namespace

void parallel(Builder &builder, const Word opcode) noexcept {
  constexpr Word conditional_mask = 0xffe000;
  constexpr Word conditional = 0x202000;
  builder.result.parallel_alu = opcode & byte_mask;
  if ((opcode & upper_move_mask) == no_move_group) { return; }
  if ((opcode & conditional_mask) == conditional) {
    // IFcc/IFcc.U encodes no data MOVE; condition bits are not register operands.
    return;
  }
  if ((opcode & update_group_mask) == update_group) {
    const auto operand = builder.effective(
        TransferSpace::x, {.mode = (opcode >> 11) & 3U, .index = (opcode >> 8) & 7U});
    builder.result.address_update = operand.address;
  } else if ((opcode & parallel_group_mask) == immediate_group) {
    register_move(builder, opcode);
  } else if ((opcode & pair_bit) != 0) {
    paired(builder, opcode);
  } else if ((opcode & memory_group_mask) == memory_group) {
    if ((opcode & long_selector_mask) == 0) {
      long_move(builder, opcode);
    } else {
      scalar(builder, opcode);
    }
  } else if ((opcode & pair_group_mask) == pair_group) {
    mixed(builder, opcode);
  } else if ((opcode & class_two_mask) == class_two) {
    class_two_move(builder, opcode);
  } else {
    builder.valid = false;
  }
}
} // namespace dsp56300::detail::transfer_info
