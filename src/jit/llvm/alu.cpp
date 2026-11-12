// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/alu.hpp"

#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "jit/llvm/builder.hpp"

#include <array>
#include <cstdint>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <optional>
#include <string_view>

namespace dsp56300::detail::llvm19 {
namespace {
using V = LLVMValueRef;
constexpr Word alu_mask = 0xf7, product_bit = 0x80;
constexpr unsigned bus_shift = word_bits + padding_bits, integer_shift = 55;

enum AluCode : std::uint8_t {
  test = 0x03,
  rounded = 0x11,
  clear = 0x13,
  complement = 0x17,
  shift_right = 0x22,
  word_right = 0x23,
  absolute = 0x26,
  shift_left = 0x32,
  word_left = 0x33,
  negate = 0x36
};

bool transfer(const Word code) {
  const auto group = (code >> 4) & 7U;
  return (code & product_bit) == 0 && (code & 7U) == 1 && (group == 0 || group >= 4);
}

bool logical(const Word code) {
  const auto kind = code & 7U;
  return (code & product_bit) == 0 &&
         ((code & alu_mask) == complement ||
          (((code >> 4) & 7U) >= 4 && (kind == 2 || kind == 3 || kind == 6)));
}

struct Result {
  V value = nullptr, overflowed = nullptr, carried = nullptr;
  bool compare = false;
};

class Alu {
public:
  Alu(Builder &ir, const Word mode, V &status) : ir_(ir), mode_(mode), status_(status) {}

  void lower(Word code, const Banks &before, Banks &after);
  void shift(Word opcode, Banks &banks);

private:
  V boolean(const bool value) { return LLVMConstInt(ir_.getIntNTy(1), value ? 1U : 0U, 0); }

  V flag(V test, const Word mask) {
    return ir_.CreateSelect(test, ir_.getInt32(mask), ir_.getInt32(0));
  }

  V signed_compare(const LLVMIntPredicate predicate, V left, V right) {
    return LLVMBuildICmp(ir_.get(), predicate, left, right, "");
  }

  V signed_right(V value, const unsigned count) {
    return LLVMBuildAShr(ir_.get(), value, ir_.getInt64(count), "");
  }

  V overflowed(V left, V right, V result, const bool subtract) {
    auto *opposite = ir_.CreateXor(left, right);
    auto *bits =
        ir_.CreateAnd(subtract ? opposite : ir_.CreateNot(opposite), ir_.CreateXor(left, result));
    return signed_compare(LLVMIntSLT, bits, ir_.getInt64(0));
  }

  Result arithmetic(V left, V right, const bool subtract) {
    const std::string_view name = subtract ? "llvm.ssub.with.overflow" : "llvm.sadd.with.overflow";
    auto *function = LLVMGetBasicBlockParent(LLVMGetInsertBlock(ir_.get()));
    auto *module = LLVMGetGlobalParent(function);
    auto *type = LLVMTypeOf(left);
    auto *intrinsic = LLVMGetIntrinsicDeclaration(
        module, LLVMLookupIntrinsicID(name.data(), name.size()), &type, 1);
    std::array inputs{left, right};
    auto *operation = LLVMBuildCall2(ir_.get(),
                                     LLVMGlobalGetValueType(intrinsic),
                                     intrinsic,
                                     inputs.data(),
                                     static_cast<unsigned>(inputs.size()),
                                     "");
    return {.value = LLVMBuildExtractValue(ir_.get(), operation, 0, ""),
            .overflowed = LLVMBuildExtractValue(ir_.get(), operation, 1, "")};
  }

  V operand(const Banks &before, const unsigned selector, const bool destination_b) {
    if (selector < 2) { return before[destination_b ? 0U : 1U]; }
    auto *bank = before[selector == 2 || selector == 4 || selector == 6 ? 2U : 3U];
    if (selector < 4) {
      return ir_.CreateShl(
          ir_.CreateSExt(ir_.CreateTrunc(bank, ir_.getIntNTy(double_word_bits)), ir_.getInt64Ty()),
          ir_.getInt64(padding_bits));
    }
    if (selector >= 6) { bank = ir_.CreateLShr(bank, ir_.getInt64(word_bits)); }
    return ir_.CreateShl(
        ir_.CreateSExt(ir_.CreateTrunc(bank, ir_.getIntNTy(word_bits)), ir_.getInt64Ty()),
        ir_.getInt64(bus_shift));
  }

  V product_word(const Banks &before, const unsigned selector) {
    auto *bank = before[selector < 2 ? 2U : 3U];
    if ((selector & 1U) != 0) { bank = ir_.CreateLShr(bank, ir_.getInt64(word_bits)); }
    return ir_.CreateSExt(ir_.CreateTrunc(bank, ir_.getIntNTy(word_bits)), ir_.getInt64Ty());
  }

  Result product(Word code, const Banks &before, unsigned destination);
  Result sum(Word code, const Banks &before, unsigned destination);
  std::optional<Result> unary(Word code, V value);
  void rounding(Result &result);
  void flags(const Result &result);
  void logic(Word code, const Banks &before, Banks &after, unsigned destination);
  void word_shift(Word code, const Banks &before, Banks &after, unsigned destination);
  void word_result(V value, Banks &after, unsigned destination, V carried = nullptr);

  Builder &ir_;
  const Word mode_;
  V &status_;
};

Result Alu::product(const Word code, const Banks &before, const unsigned destination) {
  constexpr std::array<std::array<unsigned, 2>, 8> pairs{
      {{0, 0}, {2, 2}, {1, 0}, {3, 2}, {0, 3}, {2, 0}, {1, 2}, {3, 1}}};
  const auto pair = pairs[(code >> 4) & 7U];
  auto *term =
      ir_.CreateShl(ir_.CreateMul(product_word(before, pair[0]), product_word(before, pair[1])),
                    ir_.getInt64(padding_bits + 1));
  if ((code & 4U) != 0) { term = ir_.CreateSub(ir_.getInt64(0), term); }
  auto *initial = (code & 2U) != 0 ? before[destination] : ir_.getInt64(0);
  return arithmetic(initial, term, false);
}

Result Alu::sum(const Word code, const Banks &before, const unsigned destination) {
  const auto group = (code >> 4) & 7U;
  const auto kind = code & 7U;
  const bool carried = (group == 2 || group == 3) && (kind == 1 || kind == 5);
  const bool subtract = kind == 4 || kind == 5;
  auto *left = before[destination];
  auto *right = operand(before, group, destination != 0);
  auto first = arithmetic(left, right, subtract);
  auto *intermediate = first.value;
  auto *input = carried ? ir_.CreateShl(ir_.CreateZExt(ir_.CreateAnd(status_, ir_.getInt32(carry)),
                                                       ir_.getInt64Ty()),
                                        ir_.getInt64(padding_bits))
                        : ir_.getInt64(0);
  auto *value = subtract ? ir_.CreateSub(intermediate, input) : ir_.CreateAdd(intermediate, input);
  auto *carry_out = subtract ? ir_.CreateOr(ir_.CreateICmpULT(left, right),
                                            ir_.CreateICmpULT(intermediate, input))
                             : ir_.CreateOr(ir_.CreateICmpULT(intermediate, left),
                                            ir_.CreateICmpULT(value, intermediate));
  return {.value = value,
          .overflowed = carried ? overflowed(left, right, value, subtract) : first.overflowed,
          .carried = carry_out,
          .compare = kind == 5 && !carried};
}

std::optional<Result> Alu::unary(const Word code, V value) {
  switch (code & alu_mask) {
    case test: return Result{.value = value, .compare = true};
    case clear: return Result{.value = ir_.getInt64(0)};
    case rounded: return Result{.value = value};
    case absolute:
    case negate: {
      auto *negated = ir_.CreateSub(ir_.getInt64(0), value);
      auto *result = (code & alu_mask) == negate
                         ? negated
                         : ir_.CreateSelect(
                               signed_compare(LLVMIntSLT, value, ir_.getInt64(0)), negated, value);
      return Result{.value = result,
                    .overflowed =
                        ir_.CreateICmpEQ(value, ir_.getInt64(std::uint64_t{1} << host_sign_bit))};
    }
    case shift_right:
      return Result{.value = ir_.CreateAnd(signed_right(value, 1), ir_.getInt64(padding_mask)),
                    .carried = ir_.CreateICmpNE(
                        ir_.CreateAnd(value, ir_.getInt64(1U << padding_bits)), ir_.getInt64(0))};
    case shift_left: {
      auto *result = ir_.CreateShl(value, ir_.getInt64(1));
      return Result{.value = result,
                    .overflowed =
                        signed_compare(LLVMIntSLT, ir_.CreateXor(value, result), ir_.getInt64(0)),
                    .carried = signed_compare(LLVMIntSLT, value, ir_.getInt64(0))};
    }
    default: return {};
  }
}

void Alu::rounding(Result &result) {
  const auto cleared =
      bus_shift + ((mode_ & scale_down) != 0 ? 1U : 0U) - ((mode_ & scale_up) != 0 ? 1U : 0U);
  const auto half = std::uint64_t{1} << (cleared - 1);
  auto *bias = ir_.getInt64(half);
  if ((mode_ & rounding_mode) == 0) {
    auto *odd = ir_.CreateAnd(ir_.CreateLShr(result.value, ir_.getInt64(cleared)), ir_.getInt64(1));
    bias = ir_.CreateAdd(ir_.getInt64(half - (1U << padding_bits)),
                         ir_.CreateShl(odd, ir_.getInt64(padding_bits)));
  }
  const auto rounded = arithmetic(result.value, bias, false);
  auto *added = rounded.value;
  auto *event = rounded.overflowed;
  result.overflowed =
      ir_.CreateXor(result.overflowed == nullptr ? boolean(false) : result.overflowed, event);
  result.value = ir_.CreateAnd(added, ir_.getInt64(~((std::uint64_t{1} << cleared) - 1)));
}

void Alu::flags(const Result &result) {
  const unsigned shift =
      integer_shift + ((mode_ & scale_down) != 0 ? 1U : 0U) - ((mode_ & scale_up) != 0 ? 1U : 0U);
  auto *integer = ir_.CreateLShr(result.value, ir_.getInt64(shift));
  auto *extended =
      ir_.CreateAnd(ir_.CreateICmpNE(integer, ir_.getInt64(0)),
                    ir_.CreateICmpNE(integer, ir_.getInt64(~std::uint64_t{0} >> shift)));
  auto *adjacent = ir_.CreateXor(integer, ir_.CreateLShr(result.value, ir_.getInt64(shift - 1)));
  auto *unnormal = ir_.CreateICmpEQ(ir_.CreateAnd(adjacent, ir_.getInt64(1)), ir_.getInt64(0));
  auto *bits =
      ir_.CreateOr(flag(ir_.CreateICmpEQ(result.value, ir_.getInt64(0)), zero),
                   flag(signed_compare(LLVMIntSLT, result.value, ir_.getInt64(0)), negative));
  bits = ir_.CreateOr(bits, ir_.CreateOr(flag(extended, extension), flag(unnormal, unnormalized)));
  bits = ir_.CreateOr(
      bits,
      flag(result.overflowed == nullptr ? boolean(false) : result.overflowed, overflow | limit));
  Word changed = overflow | zero | negative | unnormalized | extension;
  if (result.carried != nullptr) {
    changed |= carry;
    bits = ir_.CreateOr(bits, flag(result.carried, carry));
  }
  status_ = ir_.CreateOr(ir_.CreateAnd(status_, ir_.getInt32(~changed)), bits);
}

void Alu::logic(const Word code, const Banks &before, Banks &after, const unsigned destination) {
  auto *left = ir_.CreateTrunc(ir_.CreateLShr(before[destination], ir_.getInt64(bus_shift)),
                               ir_.getInt32Ty());
  left = ir_.CreateAnd(left, ir_.getInt32(word_mask));
  auto *right = ir_.CreateTrunc(
      ir_.CreateLShr(operand(before, (code >> 4) & 7U, destination != 0), ir_.getInt64(bus_shift)),
      ir_.getInt32Ty());
  right = ir_.CreateAnd(right, ir_.getInt32(word_mask));
  const auto kind = code & 7U;
  V value = nullptr;
  if ((code & alu_mask) == complement) {
    value = ir_.CreateXor(left, ir_.getInt32(word_mask));
  } else if (kind == 6) {
    value = ir_.CreateAnd(left, right);
  } else if (kind == 2) {
    value = ir_.CreateOr(left, right);
  } else {
    value = ir_.CreateXor(left, right);
  }
  word_result(value, after, destination);
}

void Alu::word_result(V value, Banks &after, const unsigned destination, V carried) {
  constexpr auto mask = std::uint64_t{word_mask} << bus_shift;
  after[destination] =
      ir_.CreateOr(ir_.CreateAnd(after[destination], ir_.getInt64(~mask)),
                   ir_.CreateShl(ir_.CreateZExt(value, ir_.getInt64Ty()), ir_.getInt64(bus_shift)));
  auto *bits = ir_.CreateOr(
      flag(ir_.CreateICmpEQ(value, ir_.getInt32(0)), zero),
      flag(ir_.CreateICmpNE(ir_.CreateAnd(value, ir_.getInt32(sign24_bit)), ir_.getInt32(0)),
           negative));
  Word changed = overflow | zero | negative;
  if (carried != nullptr) {
    changed |= carry;
    bits = ir_.CreateOr(bits, flag(carried, carry));
  }
  status_ = ir_.CreateOr(ir_.CreateAnd(status_, ir_.getInt32(~changed)), bits);
}

void Alu::word_shift(const Word code,
                     const Banks &before,
                     Banks &after,
                     const unsigned destination) {
  auto *word =
      ir_.CreateAnd(ir_.CreateTrunc(ir_.CreateLShr(before[destination], ir_.getInt64(bus_shift)),
                                    ir_.getInt32Ty()),
                    ir_.getInt32(word_mask));
  const bool left = (code & alu_mask) == word_left;
  auto *value = left ? ir_.CreateAnd(ir_.CreateShl(word, ir_.getInt32(1)), ir_.getInt32(word_mask))
                     : ir_.CreateLShr(word, ir_.getInt32(1));
  auto *carried =
      ir_.CreateICmpNE(ir_.CreateAnd(word, ir_.getInt32(left ? sign24_bit : 1U)), ir_.getInt32(0));
  word_result(value, after, destination, carried);
}

void Alu::shift(const Word opcode, Banks &banks) {
  constexpr Word count_mask = 63;
  constexpr Word source_bit = 1U << 7;
  constexpr Word left_bit = 1U << 8;
  const auto count = (opcode >> 1) & count_mask;
  auto *value = banks[(opcode & source_bit) != 0 ? 1U : 0U];
  Result result{.value = value, .carried = boolean(false)};
  if (count != 0 && (opcode & left_bit) != 0) {
    result.value = ir_.CreateShl(value, ir_.getInt64(count));
    result.carried = ir_.CreateICmpNE(
        ir_.CreateAnd(ir_.CreateLShr(value, ir_.getInt64(host_sign_bit + 1 - count)),
                      ir_.getInt64(1)),
        ir_.getInt64(0));
    auto *lost = signed_right(value, host_sign_bit - count);
    result.overflowed = ir_.CreateAnd(ir_.CreateICmpNE(lost, ir_.getInt64(0)),
                                      ir_.CreateICmpNE(lost, ir_.getInt64(~std::uint64_t{0})));
  } else if (count != 0) {
    result.value = ir_.CreateAnd(signed_right(value, count), ir_.getInt64(padding_mask));
    result.carried = ir_.CreateICmpNE(
        ir_.CreateAnd(ir_.CreateLShr(value, ir_.getInt64(padding_bits + count - 1)),
                      ir_.getInt64(1)),
        ir_.getInt64(0));
  }
  flags(result);
  banks[(opcode & 1U) != 0 ? 1U : 0U] = result.value;
}

void Alu::lower(const Word code, const Banks &before, Banks &after) {
  if (code == 0) { return; }
  const auto destination = (code & 8U) != 0 ? 1U : 0U;
  if (transfer(code)) {
    after[destination] = operand(before, (code >> 4) & 7U, destination != 0);
    return;
  }
  if (logical(code)) {
    logic(code, before, after, destination);
    return;
  }
  if ((code & alu_mask) == word_left || (code & alu_mask) == word_right) {
    word_shift(code, before, after, destination);
    return;
  }
  auto operation = unary(code, before[destination]);
  if (!operation) {
    operation = (code & product_bit) != 0 ? product(code, before, destination)
                                          : sum(code, before, destination);
  }
  if ((code & alu_mask) == rounded || ((code & product_bit) != 0 && (code & 1U) != 0)) {
    rounding(*operation);
  }
  flags(*operation);
  if (!operation->compare) { after[destination] = operation->value; }
}
}

bool supported_alu(const Word opcode) {
  const auto code = opcode & byte_mask;
  const auto form = code & alu_mask;
  if (code == 0 || (code & product_bit) != 0 || transfer(code) || logical(code)) { return true; }
  if (form == test || form == clear || form == rounded || form == absolute || form == negate ||
      form == shift_right || form == shift_left || form == word_right || form == word_left) {
    return true;
  }
  const auto group = (code >> 4) & 7U;
  const auto kind = code & 7U;
  return (group >= 1 && (kind == 0 || kind == 4)) || ((group == 0 || group >= 4) && kind == 5) ||
         ((group == 2 || group == 3) && (kind == 1 || kind == 5));
}

void lower_alu(
    Builder &ir, const Word opcode, const Word mode, const Banks &before, Banks &after, V &status) {
  Alu(ir, mode, status).lower(opcode & byte_mask, before, after);
}

void lower_arithmetic_shift(
    Builder &ir, const Word opcode, const Word mode, Banks &banks, V &status) {
  Alu(ir, mode, status).shift(opcode, banks);
}

V transfer_word(Builder &ir, V value, const Word mode, V &status) {
  const unsigned shift =
      bus_shift + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
  auto *word = LLVMBuildAShr(ir.get(), value, ir.getInt64(shift), "");
  auto *minimum = ir.getInt64(static_cast<std::uint64_t>(static_cast<std::int64_t>(minimum_word)));
  auto *maximum = ir.getInt64(maximum_word);
  auto *low = LLVMBuildICmp(ir.get(), LLVMIntSLT, word, minimum, "");
  auto *high = LLVMBuildICmp(ir.get(), LLVMIntSGT, word, maximum, "");
  auto *limited = ir.CreateOr(low, high);
  word = ir.CreateSelect(low, minimum, ir.CreateSelect(high, maximum, word));
  const auto growth_shift = shift + word_bits - 3;
  auto *adjacent = ir.CreateXor(ir.CreateLShr(value, ir.getInt64(growth_shift)),
                                ir.CreateLShr(value, ir.getInt64(growth_shift + 1)));
  auto *growth_bit = ir.CreateShl(
      ir.CreateTrunc(ir.CreateAnd(adjacent, ir.getInt64(1)), ir.getInt32Ty()), ir.getInt32(7));
  status = ir.CreateOr(
      status,
      ir.CreateOr(growth_bit, ir.CreateSelect(limited, ir.getInt32(limit), ir.getInt32(0))));
  return ir.CreateAnd(ir.CreateTrunc(word, ir.getInt32Ty()), ir.getInt32(word_mask));
}
}
