// SPDX-License-Identifier: GPL-3.0-only
#include "jit/code.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
void encoding() {
  using namespace detail::x64;
  Code code;
  code.vector(VectorOp::add, {.destination = Xmm::x9, .left = Xmm::x14, .right = Xmm::x15});
  code.vector_unary(VectorOp::copy, Xmm::x10, Xmm::x11);
  constexpr unsigned signs = 0xa0;
  constexpr unsigned high = 0xaa;
  constexpr unsigned sign_bit = 31;
  code.vector_immediate(
      VectorOp::shuffle_dword, {.destination = Xmm::x8, .left = Xmm::x0, .right = Xmm::x15}, signs);
  code.vector_immediate(
      VectorOp::blend_dword, {.destination = Xmm::x9, .left = Xmm::x15, .right = Xmm::x14}, high);
  code.vector_shift(Xmm::x9, Xmm::x15, VectorShift::arithmetic, sign_bit, true);
  constexpr unsigned near = 32;
  constexpr unsigned far = 64;
  code.vector_load(Xmm::x12, {.base = Reg::r12, .offset = near});
  code.vector_store({.base = Reg::r13, .offset = far}, Xmm::x15);
  code.vector_broadcast(Xmm::x14, {.base = Reg::r14, .offset = far});
  code.vector_broadcast(Xmm::x13, {.base = Reg::r15, .offset = near}, true);
  code.vector_extract(Xmm::x14, Xmm::x15, 1);
  code.vector_scalar(Reg::r11, Xmm::x14, 1);
  code.vector_load_dword(Xmm::x9, {.base = Reg::r13, .offset = near});
  code.vector_insert_dword(Xmm::x9, {.base = Reg::r13, .index = Reg::r11, .offset = near}, 3);
  code.vector_pack(Xmm::x14, Xmm::x9, Xmm::x15);
  code.vector(VectorOp::shuffle_bytes,
              {.destination = Xmm::x9, .left = Xmm::x14, .right = Xmm::x15});
  code.vector(VectorOp::shift_right_qword,
              {.destination = Xmm::x9, .left = Xmm::x9, .right = Xmm::x14});
  code.vector_finish();
  // Independent LLVM 19 assembler encodings, with VEX3 and high registers.
  // VPADDQ; VMOVDQA; VPSHUFD; VPBLENDD; VPSRAD; two VMOVDQU;
  // VPBROADCASTQ/D; VEXTRACTI128; VPEXTRQ; VMOVD; VPINSRD; VSHUFPS; VPSHUFB; VPSRLQ; VZEROUPPER.
  constexpr std::array<std::uint8_t, 98> expected{
      0xc4, 0x41, 0x0d, 0xd4, 0xcf, 0xc4, 0x41, 0x7d, 0x6f, 0xd3, 0xc4, 0x41, 0x7d, 0x70,
      0xc7, 0xa0, 0xc4, 0x43, 0x05, 0x02, 0xce, 0xaa, 0xc4, 0xc1, 0x35, 0x72, 0xe7, 0x1f,
      0xc4, 0x41, 0x7e, 0x6f, 0x64, 0x24, 0x20, 0xc4, 0x41, 0x7e, 0x7f, 0x7d, 0x40, 0xc4,
      0x42, 0x7d, 0x59, 0x76, 0x40, 0xc4, 0x42, 0x7d, 0x58, 0x6f, 0x20, 0xc4, 0x43, 0x7d,
      0x39, 0xfe, 0x01, 0xc4, 0x43, 0xf9, 0x16, 0xf3, 0x01, 0xc4, 0x41, 0x79, 0x6e, 0x4d,
      0x20, 0xc4, 0x03, 0x31, 0x22, 0x4c, 0x9d, 0x20, 0x03, 0xc4, 0x41, 0x30, 0xc6, 0xf7,
      0x88, 0xc4, 0x42, 0x0d, 0x00, 0xcf, 0xc4, 0x41, 0x35, 0xd3, 0xce, 0xc5, 0xf8, 0x77};
  require(
      std::ranges::equal(code.bytes(), expected),
      "AVX2 opcode maps, inverted fields, widths, SIB and immediates match independent assembler");
}
}
}

int main() {
  try {
    dsp56300::test::encoding();
    std::cout << "Independent AVX2 emission encodings passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
