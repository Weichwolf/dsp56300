// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "instruction_decoder.hpp"
#include "transfer_info_support.hpp"
#include "transfers.hpp"

namespace dsp56300::test::transfer_info_test {
namespace {
constexpr Word load_x0_post = 0x44d800;
constexpr Word store_x0_post = 0x445800;
constexpr Word read_program_x0 = 0x07f084;
constexpr Word write_program_x0 = 0x077084;
constexpr Word immediate_m0 = 0x05ffa0;
constexpr Word x0_to_m0 = 0x04c4a0;
constexpr Word m0_to_x0 = 0x0444a0;
constexpr Word fractional_x0 = 0x248000;
constexpr Word integer_a1 = 0x2c8000;
constexpr Word a_to_x1 = 0x21c500;
constexpr Word raw_long_store = 0x401200;
constexpr Word raw_long_load = 0x429200;
constexpr Word paired_post_load = 0xf09800;
constexpr Word add_without_move = 0x200040;
constexpr Word conditional_add = 0x202040;
constexpr Word byte_ones = 0xff;
constexpr Word long_address = 0x12;
constexpr Word low_x_read = 0x044494;
constexpr Word high_y_write = 0x09c43f;
constexpr Word low_y_read = 0x045321;
constexpr Word port_to_program = 0x085a43;
constexpr Word extension_to_port = 0x07b481;
constexpr Word low_x_address = 0xffff94;
constexpr Word high_y_address = 0xffffff;
constexpr Word low_y_address = 0xffff81;
constexpr Word high_x_address = 0xffffc3;

void peripheral_literals() {
  verify(low_x_read,
         expected(InstructionKind::move_peripheral_register,
                  1,
                  {{.source = absolute(TransferSpace::x, low_x_address),
                    .destination = reg(RegisterId::x0)}}));
  verify(high_y_write,
         expected(InstructionKind::move_peripheral_register,
                  1,
                  {{.source = reg(RegisterId::x0),
                    .destination = absolute(TransferSpace::y, high_y_address)}}));
  verify(low_y_read,
         expected(InstructionKind::move_peripheral_register,
                  1,
                  {{.source = absolute(TransferSpace::y, low_y_address),
                    .destination = reg(RegisterId::r3)}}));
  verify(port_to_program,
         expected(InstructionKind::move_peripheral_program,
                  1,
                  {{.source = absolute(TransferSpace::x, high_x_address),
                    .destination = ea(TransferSpace::program, {.mode = 3, .index = 2})}}));
  verify(extension_to_port,
         expected(
             InstructionKind::move_peripheral_memory,
             2,
             {{.source = extension(), .destination = absolute(TransferSpace::y, low_y_address)}}));
}
} // namespace

void literals() {
  const auto x_post = ea(TransferSpace::x, {.mode = 3, .index = 0});
  const auto program_extension = ea(TransferSpace::program, {.mode = 6, .index = 0});
  verify(load_x0_post,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = x_post, .destination = reg(RegisterId::x0)}}));
  verify(store_x0_post,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = reg(RegisterId::x0), .destination = x_post}}));
  verify(read_program_x0,
         expected(InstructionKind::move_program_effective,
                  2,
                  {{.source = program_extension, .destination = reg(RegisterId::x0)}}));
  verify(write_program_x0,
         expected(InstructionKind::move_program_effective,
                  2,
                  {{.source = reg(RegisterId::x0), .destination = program_extension}}));
  verify(immediate_m0,
         expected(InstructionKind::move_control_immediate,
                  1,
                  {{.source = immediate(byte_ones), .destination = reg(RegisterId::m0)}}));
  verify(x0_to_m0,
         expected(InstructionKind::move_control_register,
                  1,
                  {{.source = reg(RegisterId::x0), .destination = reg(RegisterId::m0)}}));
  verify(m0_to_x0,
         expected(InstructionKind::move_control_register,
                  1,
                  {{.source = reg(RegisterId::m0), .destination = reg(RegisterId::x0)}}));
  verify(fractional_x0,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = immediate(0x80, 16), .destination = reg(RegisterId::x0)}}));
  verify(integer_a1,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = immediate(0x80), .destination = reg(RegisterId::a1)}}));
  verify(a_to_x1,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = reg(RegisterId::a), .destination = reg(RegisterId::x1)}}));
  verify(raw_long_store,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = reg(RegisterId::a10),
                    .destination = absolute(TransferSpace::long_pair, long_address)}}));
  verify(raw_long_load,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = absolute(TransferSpace::long_pair, long_address),
                    .destination = reg(RegisterId::x)}}));
  verify(paired_post_load,
         expected(InstructionKind::parallel,
                  1,
                  {{.source = x_post, .destination = reg(RegisterId::x0)},
                   {.source = ea(TransferSpace::y, {.mode = 3, .index = 4}),
                    .destination = reg(RegisterId::y0)}}));
  verify(0, expected(InstructionKind::nop, 1, {}));
  auto no_move = expected(InstructionKind::parallel, 1, {});
  no_move.parallel_alu = 0x40;
  verify(add_without_move, no_move);
  verify(conditional_add, no_move);
  peripheral_literals();
  for (const Word opcode : {Word{7},
                            Word{0x1000000},
                            Word{0xffffffff},
                            Word{0x200400},
                            Word{0x84},
                            Word{0x86},
                            Word{0x0604a0}}) {
    unsupported(opcode);
  }
}
} // namespace dsp56300::test::transfer_info_test
