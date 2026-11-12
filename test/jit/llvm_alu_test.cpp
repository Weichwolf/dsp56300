// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/alu.hpp"
#include "jit/llvm/builder.hpp"
#include "jit/llvm/runtime.hpp"
#include "native_sequence_model.hpp"
#include "saturated_shift_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <string>
#include <vector>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using enum isa::Opcode;
using V = LLVMValueRef;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr unsigned x_perturb_shift = 18, y_perturb_shift = 23;
constexpr std::array arithmetic{
    tst_a,    tst_b,    clr_a,    clr_b,    rnd_a,    rnd_b,    abs_a,    abs_b,    neg_a,
    neg_b,    asr_a,    asr_b,    asl_a,    asl_b,    tfr_a_b,  tfr_b_a,  tfr_x0_a, tfr_x0_b,
    tfr_x1_a, tfr_x1_b, tfr_y0_a, tfr_y0_b, tfr_y1_a, tfr_y1_b, add_a_b,  add_b_a,  add_x_a,
    add_x_b,  add_y_a,  add_y_b,  add_x0_a, add_x0_b, add_x1_a, add_x1_b, add_y0_a, add_y0_b,
    add_y1_a, add_y1_b, sub_a_b,  sub_b_a,  sub_x_a,  sub_x_b,  sub_y_a,  sub_y_b,  sub_x0_a,
    sub_x0_b, sub_x1_a, sub_x1_b, sub_y0_a, sub_y0_b, sub_y1_a, sub_y1_b, cmp_a_b,  cmp_b_a,
    cmp_x0_a, cmp_x0_b, cmp_x1_a, cmp_x1_b, cmp_y0_a, cmp_y0_b, cmp_y1_a, cmp_y1_b, adc_x_a,
    adc_x_b,  adc_y_a,  adc_y_b,  sbc_x_a,  sbc_x_b,  sbc_y_a,  sbc_y_b};
constexpr std::array products{mpy_register_a,
                              mpy_register_b,
                              mpyr_register_a,
                              mpyr_register_b,
                              mac_register_a,
                              mac_register_b,
                              macr_register_a,
                              macr_register_b};
constexpr std::array<std::uint64_t, 12> edges{0,
                                              1,
                                              accumulator_mask,
                                              sign56,
                                              sign56 - 1,
                                              sign56 + 1,
                                              0x007fffff800000,
                                              0x00800000800000,
                                              0xff7fffff800000,
                                              0x00000000800000,
                                              0xffffffffff0001,
                                              0x123456789abcde};

struct Case {
  Word opcode = 0;
  bool logic = false;
  bfu_model::Input input{};
};

std::vector<Case> cases() {
  std::vector<Case> result{{}};
  for (const auto opcode : arithmetic) { result.push_back({.opcode = static_cast<Word>(opcode)}); }
  for (const auto opcode : products) {
    for (Word selector = 0; selector < 8; ++selector) {
      for (const Word sign : {0U, 4U}) {
        result.push_back({.opcode = static_cast<Word>(opcode) | (selector << 4) | sign});
      }
    }
  }
  for (const auto function : {bfu_model::Function::bit_and,
                              bfu_model::Function::bit_or,
                              bfu_model::Function::bit_xor,
                              bfu_model::Function::complement,
                              bfu_model::Function::left,
                              bfu_model::Function::right}) {
    for (unsigned source = 0; source < 4; ++source) {
      for (const bool destination : {false, true}) {
        const bfu_model::Input input{
            .function = function, .source = source, .destination_b = destination};
        result.push_back({.opcode = bfu_model::opcode(input), .logic = true, .input = input});
      }
    }
  }
  return result;
}

V address(detail::llvm19::Builder &ir, V cpu, const std::size_t offset) {
  return ir.CreateGEP(ir.getInt8Ty(), cpu, ir.getInt64(offsetof(detail::Cpu, reg) + offset));
}

void emit_case(detail::llvm19::Builder &ir, V cpu, const Case &input, const Word mode) {
  constexpr std::array fields{offsetof(Registers, a),
                              offsetof(Registers, b),
                              offsetof(Registers, x),
                              offsetof(Registers, y)};
  detail::llvm19::Banks before{};
  for (unsigned i = 0; i < fields.size(); ++i) {
    before[i] = ir.CreateLoad(ir.getInt64Ty(), address(ir, cpu, fields[i]));
  }
  auto after = before;
  auto *status = ir.CreateLoad(ir.getInt32Ty(), address(ir, cpu, offsetof(Registers, sr)));
  require(detail::llvm19::supported_alu(input.opcode), "Authored ALU case must be eligible");
  detail::llvm19::lower_alu(ir, input.opcode, mode, before, after, status);
  for (unsigned i = 0; i < fields.size(); ++i) {
    ir.CreateStore(after[i], address(ir, cpu, fields[i]));
  }
  ir.CreateStore(status, address(ir, cpu, offsetof(Registers, sr)));
  LLVMBuildRet(ir.get(), ir.getInt32(1));
}

const void *compile(detail::LlvmCompiler &compiler, const std::vector<Case> &inputs, Word mode) {
  detail::LlvmModule module("alu_matrix_" + std::to_string(mode));
  auto *context = module.context();
  auto *pointer = LLVMPointerTypeInContext(context, 0);
  std::array arguments{pointer, pointer};
  auto *type =
      LLVMFunctionType(LLVMInt32TypeInContext(context), arguments.data(), arguments.size(), 0);
  auto *function = LLVMAddFunction(module.code(), module.name().c_str(), type);
  auto *entry = LLVMAppendBasicBlockInContext(context, function, "entry");
  auto *invalid = LLVMAppendBasicBlockInContext(context, function, "invalid");
  detail::llvm19::Builder ir(context, entry);
  auto *cpu = LLVMGetParam(function, 0);
  auto *selector = ir.CreateLoad(ir.getInt32Ty(), address(ir, cpu, offsetof(Registers, pc)));
  auto *select = LLVMBuildSwitch(ir.get(), selector, invalid, static_cast<unsigned>(inputs.size()));
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    auto *block = LLVMAppendBasicBlockInContext(context, function, "case");
    LLVMAddCase(select, ir.getInt32(index), block);
    ir.SetInsertPoint(block);
    emit_case(ir, cpu, inputs[index], mode);
  }
  ir.SetInsertPoint(invalid);
  LLVMBuildRet(ir.get(), ir.getInt32(0));
  return compiler.install(module);
}

Registers initial(Word mode, Word flags, std::size_t seed) {
  auto result = initial_state();
  result.sr = mode | flags;
  result.a = edges[seed];
  result.b = edges[(seed + 3) % edges.size()];
  result.x =
      (edges[(seed + 7) % edges.size()] ^ (std::uint64_t{flags} << x_perturb_shift)) & long_mask;
  result.y =
      (edges[(seed + 5) % edges.size()] ^ (std::uint64_t{flags} << y_perturb_shift)) & long_mask;
  return result;
}

void check(const void *entry, const Case &input, Registers old) {
  auto expected = old;
  const auto form = input.opcode & status_byte;
  if (input.logic) {
    bfu_model::apply(expected, old, input.input);
  } else if (input.opcode == static_cast<Word>(asl_a) || input.opcode == static_cast<Word>(asl_b)) {
    const bool destination = input.opcode == static_cast<Word>(asl_b);
    shift_model::apply(
        expected, old, {.left = true, .source_b = destination, .destination_b = destination});
  } else {
    sequence_model::alu(expected, old, form);
  }
  old.a <<= detail::padding_bits;
  old.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = old};
  detail::NativeContext owner;
  const auto retired = detail::enter_block(entry, cpu, owner);
  auto actual = cpu.reg;
  actual.a >>= detail::padding_bits;
  actual.b >>= detail::padding_bits;
  if (actual != expected) {
    std::cerr << "opcode=" << std::hex << input.opcode << " sr=" << old.sr << " A/B/X/Y=" << old.a
              << '/' << old.b << '/' << old.x << '/' << old.y << " expected=" << expected.a << '/'
              << expected.b << '/' << expected.sr << " actual=" << actual.a << '/' << actual.b
              << '/' << actual.sr << '\n';
  }
  require(retired == 1 && actual == expected && cpu.instructions == 0 && cpu.cycles == 0,
          "Optimized semantic IR matches independent raw56 and bit-truth models");
}

void matrix() {
  const auto inputs = cases();
  detail::LlvmCompiler compiler;
  for (const auto mode : modes) {
    const auto *entry = compile(compiler, inputs, mode);
    for (std::size_t index = 0; index < inputs.size(); ++index) {
      for (Word flags = 0; flags <= status_byte; ++flags) {
        for (std::size_t seed = 0; seed < edges.size(); ++seed) {
          auto state = initial(mode, flags, seed);
          state.pc = static_cast<Word>(index);
          check(entry, inputs[index], state);
        }
      }
    }
  }
}
}

int main() try { matrix(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
