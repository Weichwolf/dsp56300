// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/region.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/alu.hpp"
#include "jit/llvm/builder.hpp"
#include "jit/llvm/region_plan.hpp"
#include "jit/llvm/runtime.hpp"
#include "jit/move.hpp"
#include "translation.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace dsp56300::detail {
namespace {
using V = LLVMValueRef;
constexpr std::array<std::size_t, 4> bank_fields{
    offsetof(Registers, a), offsetof(Registers, b), offsetof(Registers, x), offsetof(Registers, y)};
constexpr Word stack_mask = stack_entries - 1, pointer_mask = 63;
constexpr unsigned bus_shift = word_bits + padding_bits;

class RegionLifter {
public:
  RegionLifter(LlvmModule &module, const RegionPlan &plan, const void *fallback)
      : context_(module.context()),
        function_(make_function(module)),
        ir_(context_, LLVMAppendBasicBlockInContext(context_, function_, "entry")),
        plan_(plan),
        cpu_(LLVMGetParam(function_, 0)),
        owner_(LLVMGetParam(function_, 1)) {
    generate(fallback);
  }

private:
  struct Bounds {
    V first = nullptr, last = nullptr;
  };

  V boolean(const bool value) { return LLVMConstInt(ir_.getIntNTy(1), value ? 1U : 0U, 0); }

  static LLVMValueRef make_function(LlvmModule &module) {
    auto *pointer = LLVMPointerTypeInContext(module.context(), 0);
    std::array arguments{pointer, pointer};
    auto *type = LLVMFunctionType(
        LLVMInt32TypeInContext(module.context()), arguments.data(), arguments.size(), 0);
    auto *function = LLVMAddFunction(module.code(), module.name().c_str(), type);
    return function;
  }

  V address(V base, const std::size_t offset) {
    return ir_.CreateGEP(ir_.getInt8Ty(), base, ir_.getInt64(offset));
  }

  V load(const unsigned bits, const std::size_t offset, V base = nullptr) {
    return ir_.CreateLoad(ir_.getIntNTy(bits), address(base == nullptr ? cpu_ : base, offset));
  }

  void store(V value, const std::size_t offset, V base = nullptr) {
    ir_.CreateStore(value, address(base == nullptr ? cpu_ : base, offset));
  }

  V reg(const unsigned bits, const std::size_t offset) {
    return load(bits, offsetof(Cpu, reg) + offset);
  }

  void put(V value, const std::size_t offset) { store(value, offsetof(Cpu, reg) + offset); }

  V minimum(V a, V b) { return ir_.CreateSelect(ir_.CreateICmpULT(a, b), a, b); }

  void require(V condition) { eligible_ = ir_.CreateAnd(eligible_, condition); }

  V stack(V index) {
    auto *bytes =
        ir_.CreateMul(ir_.CreateZExt(index, ir_.getInt64Ty()), ir_.getInt64(sizeof(std::uint64_t)));
    auto *base = address(cpu_, offsetof(Cpu, reg) + offsetof(Registers, stack));
    return ir_.CreateLoad(ir_.getInt64Ty(), ir_.CreateGEP(ir_.getInt8Ty(), base, bytes));
  }

  void admission() {
    initial_cycles_ = load(64, offsetof(Cpu, cycles));
    initial_instructions_ = load(64, offsetof(Cpu, instructions));
    status_ = reg(32, offsetof(Registers, sr));
    lc_ = reg(32, offsetof(Registers, lc));
    sp_ = reg(32, offsetof(Registers, sp));
    sc_ = reg(32, offsetof(Registers, sc));
    saved_ = stack(ir_.CreateAnd(sp_, ir_.getInt32(stack_mask)));
    auto *deadline = load(64, offsetof(NativeContext, cycle_deadline), owner_);
    auto *credit = ir_.CreateSelect(ir_.CreateICmpUGE(deadline, initial_cycles_),
                                    ir_.CreateSub(deadline, initial_cycles_),
                                    ir_.getInt64(0));
    auto *budget = load(32, offsetof(NativeContext, budget), owner_);
    turns_ =
        minimum(ir_.CreateZExt(ir_.CreateUDiv(budget, ir_.getInt32(plan_.count)), ir_.getInt64Ty()),
                ir_.CreateUDiv(credit, ir_.getInt64(plan_.cost)));
    turns_ = minimum(
        turns_,
        ir_.CreateUDiv(ir_.CreateSub(ir_.getInt64(std::numeric_limits<std::uint64_t>::max()),
                                     initial_instructions_),
                       ir_.getInt64(plan_.count)));
    finite_count_ = ir_.CreateSelect(ir_.CreateICmpEQ(lc_, ir_.getInt32(0)),
                                     ir_.getInt64(address_words),
                                     ir_.CreateZExt(lc_, ir_.getInt64Ty()));
    forever_ =
        ir_.CreateICmpNE(ir_.CreateAnd(status_, ir_.getInt32(forever_flag)), ir_.getInt32(0));
    turns_ = plan_.batch ? ir_.CreateSelect(forever_, turns_, minimum(turns_, finite_count_))
                         : minimum(turns_, ir_.getInt64(1));
    eligible_ = ir_.CreateICmpUGT(turns_, ir_.getInt64(0));
    require(ir_.CreateICmpEQ(ir_.CreateAnd(status_, ir_.getInt32(numeric_mode_mask)),
                             ir_.getInt32(plan_.operations.front().mode)));
    require(ir_.CreateICmpEQ(reg(32, offsetof(Registers, pc)),
                             ir_.getInt32(plan_.operations.front().pc)));
    auto *active_la = ir_.CreateSelect(
        ir_.CreateICmpNE(ir_.CreateAnd(status_, ir_.getInt32(loop_flag)), ir_.getInt32(0)),
        reg(32, offsetof(Registers, la)),
        ir_.getInt32(inactive_loop));
    require(ir_.CreateICmpEQ(active_la, ir_.getInt32(plan_.operations.front().loop_address)));
    if (at_loop_end(plan_.operations.back())) { require(ir_.CreateICmpUGE(sc_, ir_.getInt32(2))); }
    if (plan_.batch) {
      require(ir_.CreateICmpEQ(ir_.CreateLShr(saved_, ir_.getInt64(word_bits)),
                               ir_.getInt64(plan_.operations.front().pc)));
    }
    require(ir_.CreateICmpEQ(load(32, offsetof(Cpu, repeat_remaining)), ir_.getInt32(0)));
    for (const auto offset : {offsetof(Cpu, processing), offsetof(Cpu, debug_flags)}) {
      require(ir_.CreateICmpEQ(load(8, offset), ir_.getInt8(0)));
    }
    for (unsigned index = 0; index < agu_registers; ++index) {
      cursors_[index] = reg(32, offsetof(Registers, r) + (index * sizeof(Word)));
      strides_[index] = reg(32, offsetof(Registers, n) + (index * sizeof(Word)));
      if (!plan_.agu[index]) { continue; }
      auto *modifier = ir_.CreateAnd(reg(32, offsetof(Registers, m) + (index * sizeof(Word))),
                                     ir_.getInt32(modifier_mask));
      auto *linear = ir_.CreateICmpEQ(modifier, ir_.getInt32(modifier_mask));
      if (plan_.batch) { require(linear); }
      auto *mask = ir_.CreateAnd(modifier, ir_.getInt32(multiple_wrap_mask));
      auto *ring = ir_.CreateAnd(
          ir_.CreateICmpEQ(
              ir_.CreateAnd(modifier, ir_.getInt32(multiple_wrap_bit | multiple_reserved_bit)),
              ir_.getInt32(multiple_wrap_bit)),
          ir_.CreateAnd(ir_.CreateICmpNE(mask, ir_.getInt32(0)),
                        ir_.CreateICmpEQ(ir_.CreateAnd(mask, ir_.CreateAdd(mask, ir_.getInt32(1))),
                                         ir_.getInt32(0))));
      require(ir_.CreateOr(linear, ring));
      masks_[index] = ir_.CreateSelect(linear, ir_.getInt32(word_mask), mask);
      origins_[index] = ir_.CreateAnd(cursors_[index], ir_.CreateNot(masks_[index]));
    }
  }

  V known_words(const MemoryWindow &window, const Bounds bounds, const bool writing) {
    if (window.known.empty()) { return boolean(true); }
    auto *prior = LLVMGetInsertBlock(ir_.get());
    auto *condition = LLVMAppendBasicBlockInContext(context_, function_, "known_condition");
    auto *body = LLVMAppendBasicBlockInContext(context_, function_, "known_body");
    auto *next = LLVMAppendBasicBlockInContext(context_, function_, "known_next");
    auto *done = LLVMAppendBasicBlockInContext(context_, function_, "known_done");
    LLVMBuildBr(ir_.get(), condition);
    ir_.SetInsertPoint(condition);
    auto *index = LLVMBuildPhi(ir_.get(), ir_.getInt64Ty(), "known_index");
    auto *first = bounds.first;
    LLVMAddIncoming(index, &first, &prior, 1);
    LLVMBuildCondBr(ir_.get(), ir_.CreateICmpULT(index, bounds.last), body, done);
    ir_.SetInsertPoint(body);
    auto *base = LLVMConstIntToPtr(ir_.getInt64(std::bit_cast<std::uintptr_t>(window.known.data())),
                                   ir_.getPtrTy());
    auto *value = ir_.CreateLoad(ir_.getInt8Ty(), ir_.CreateGEP(ir_.getInt8Ty(), base, index));
    // Ordinary stores normalize knowledge to one. Only canonical destination
    // bytes can omit that metadata write on the optimized path.
    auto *valid =
        writing ? ir_.CreateICmpEQ(value, ir_.getInt8(1)) : ir_.CreateICmpNE(value, ir_.getInt8(0));
    LLVMBuildCondBr(ir_.get(), valid, next, done);
    ir_.SetInsertPoint(next);
    auto *following = ir_.CreateAdd(index, ir_.getInt64(1));
    LLVMAddIncoming(index, &following, &next, 1);
    LLVMBuildBr(ir_.get(), condition);
    ir_.SetInsertPoint(done);
    auto *known = LLVMBuildPhi(ir_.get(), ir_.getIntNTy(1), "known_range");
    auto *success = boolean(true);
    auto *failure = boolean(false);
    LLVMAddIncoming(known, &success, &condition, 1);
    LLVMAddIncoming(known, &failure, &body, 1);
    return known;
  }

  V select_window(const std::span<const MemoryWindow> windows,
                  const Bounds bounds,
                  const bool writing) {
    auto *base = LLVMConstNull(ir_.getPtrTy());
    auto *accepted = boolean(false);
    for (const auto &window : windows) {
      if (!permitted(window, writing)) { continue; }
      auto *begin = ir_.getInt64(window.address);
      auto *end = ir_.getInt64(window.address + window.words.size());
      auto *matches = ir_.CreateAnd(ir_.CreateICmpUGE(bounds.first, begin),
                                    ir_.CreateICmpULT(bounds.last, end));
      matches = ir_.CreateAnd(matches, ir_.CreateICmpULT(bounds.first, end));
      const Bounds range{
          .first = ir_.CreateSelect(matches, ir_.CreateSub(bounds.first, begin), ir_.getInt64(0)),
          .last =
              ir_.CreateSelect(matches,
                               ir_.CreateAdd(ir_.CreateSub(bounds.last, begin), ir_.getInt64(1)),
                               ir_.getInt64(0))};
      auto *admitted = ir_.CreateAnd(matches, known_words(window, range, writing));
      auto *biased = LLVMBuildIntToPtr(
          ir_.get(),
          ir_.CreateSub(ir_.getInt64(std::bit_cast<std::uintptr_t>(window.words.data())),
                        ir_.getInt64(std::uint64_t{window.address} * sizeof(Word))),
          ir_.getPtrTy(),
          "");
      base = ir_.CreateSelect(admitted, biased, base);
      accepted = ir_.CreateOr(accepted, admitted);
    }
    require(accepted);
    return base;
  }

  void batch_memory_guards() {
    for (unsigned index = 0; index < agu_registers; ++index) {
      for (unsigned bank = 0; bank < 2; ++bank) {
        const auto &extent = plan_.extents[bank][index];
        if (!extent.accessed) { continue; }
        auto *origin = ir_.CreateZExt(cursors_[index], ir_.getInt64Ty());
        auto *first =
            ir_.CreateAdd(origin, ir_.getInt64(static_cast<std::uint64_t>(extent.minimum)));
        auto *last =
            ir_.CreateAdd(origin, ir_.getInt64(static_cast<std::uint64_t>(extent.maximum)));
        auto *travel = ir_.CreateMul(ir_.CreateSub(turns_, ir_.getInt64(1)),
                                     ir_.getInt64(static_cast<std::uint64_t>(plan_.deltas[index])));
        if (plan_.deltas[index] < 0) {
          first = ir_.CreateAdd(first, travel);
        } else {
          last = ir_.CreateAdd(last, travel);
        }
        memory_bases_[bank][index] =
            select_window(plan_.windows[bank], {.first = first, .last = last}, extent.writing);
      }
    }
  }

  V advanced(V cursor, const AddressForm form) {
    auto *step = form.mode < 2 || form.mode == 5 ? strides_[form.index] : ir_.getInt32(1);
    auto *next = form.mode == 0 || form.mode == 2 || form.mode == 7 ? ir_.CreateSub(cursor, step)
                                                                    : ir_.CreateAdd(cursor, step);
    return ir_.CreateOr(origins_[form.index], ir_.CreateAnd(next, masks_[form.index]));
  }

  V effective(std::array<V, agu_registers> &cursors, const AddressForm form) {
    auto *prior = cursors[form.index];
    if (form.mode == 4) { return prior; }
    auto *next = advanced(prior, form);
    if (form.mode != 5) { cursors[form.index] = next; }
    return form.mode == 5 || form.mode == 7 ? next : prior;
  }

  void memory_guards() {
    if (plan_.batch) {
      batch_memory_guards();
      return;
    }
    auto proof = cursors_;
    std::array<std::array<Bounds, agu_registers>, 2> bounds{};
    for (const auto &step : plan_.steps) {
      V paired = nullptr;
      for (const auto &move : std::span(step.memory.transfers).first(step.memory.count)) {
        auto *at =
            step.memory.long_pair && paired != nullptr ? paired : effective(proof, move.address);
        paired = at;
        auto &range = bounds[move.space == Space::x ? 0U : 1U][move.address.index];
        auto *wide = ir_.CreateZExt(at, ir_.getInt64Ty());
        range.first = range.first == nullptr ? wide : minimum(range.first, wide);
        range.last = range.last == nullptr
                         ? wide
                         : ir_.CreateSelect(ir_.CreateICmpUGT(wide, range.last), wide, range.last);
      }
    }
    for (unsigned bank = 0; bank < 2; ++bank) {
      for (unsigned index = 0; index < agu_registers; ++index) {
        if (!plan_.extents[bank][index].accessed) { continue; }
        memory_bases_[bank][index] = select_window(
            plan_.windows[bank], bounds[bank][index], plan_.extents[bank][index].writing);
      }
    }
  }

  V memory(const NativeMove &move, V at) {
    const auto bank = move.space == Space::x ? 0U : 1U;
    return ir_.CreateGEP(ir_.getInt32Ty(),
                         memory_bases_[bank][move.address.index],
                         ir_.CreateZExt(at, ir_.getInt64Ty()));
  }

  V read_register(const std::array<V, 4> &values, const unsigned number) {
    if (number >= n_register_base) { return strides_[number - n_register_base]; }
    if (number >= r_register_base) { return cursors_[number - r_register_base]; }
    if (number == a_register || number == b_register) {
      return llvm19::transfer_word(
          ir_, values[number - a_register], plan_.operations.front().mode, status_);
    }
    auto *value = values[number < y0_register ? 2U : 3U];
    return ir_.CreateAnd(
        ir_.CreateTrunc(ir_.CreateLShr(value, ir_.getInt64(std::uint64_t{number & 1U} * word_bits)),
                        ir_.getInt32Ty()),
        ir_.getInt32(word_mask));
  }

  void write_register(const unsigned number, V word) {
    if (number >= n_register_base) {
      strides_[number - n_register_base] = word;
      return;
    }
    if (number >= r_register_base) {
      cursors_[number - r_register_base] = word;
      return;
    }
    if (number == a_register || number == b_register) {
      banks_[number - a_register] = ir_.CreateShl(
          ir_.CreateSExt(ir_.CreateTrunc(word, ir_.getIntNTy(word_bits)), ir_.getInt64Ty()),
          ir_.getInt64(bus_shift));
      return;
    }
    const auto bank = number < y0_register ? 2U : 3U;
    const auto shift = (number & 1U) * word_bits;
    auto *mask = ir_.getInt64(~(std::uint64_t{word_mask} << shift));
    banks_[bank] =
        ir_.CreateOr(ir_.CreateAnd(banks_[bank], mask),
                     ir_.CreateShl(ir_.CreateZExt(word, ir_.getInt64Ty()), ir_.getInt64(shift)));
  }

  V transfer_long(V value) {
    static_cast<void>(llvm19::transfer_word(ir_, value, plan_.operations.front().mode, status_));
    const auto mode = plan_.operations.front().mode;
    const unsigned shift =
        padding_bits + ((mode & scale_down) != 0 ? 1U : 0U) - ((mode & scale_up) != 0 ? 1U : 0U);
    auto *scaled = LLVMBuildAShr(ir_.get(), value, ir_.getInt64(shift), "");
    constexpr auto minimum = -(std::int64_t{1} << (double_word_bits - 1));
    constexpr auto maximum = (std::int64_t{1} << (double_word_bits - 1)) - 1;
    auto *low = ir_.getInt64(static_cast<std::uint64_t>(minimum));
    auto *high = ir_.getInt64(maximum);
    scaled = ir_.CreateSelect(
        LLVMBuildICmp(ir_.get(), LLVMIntSLT, scaled, low, ""),
        low,
        ir_.CreateSelect(LLVMBuildICmp(ir_.get(), LLVMIntSGT, scaled, high, ""), high, scaled));
    return ir_.CreateAnd(scaled, ir_.getInt64(long_mask));
  }

  V long_value(const unsigned selector) {
    if (selector >= 6) {
      auto *high = read_register(banks_, selector == 6 ? a_register : b_register);
      auto *low = read_register(banks_, selector == 6 ? b_register : a_register);
      return ir_.CreateOr(
          ir_.CreateShl(ir_.CreateZExt(high, ir_.getInt64Ty()), ir_.getInt64(word_bits)),
          ir_.CreateZExt(low, ir_.getInt64Ty()));
    }
    if (selector == 2 || selector == 3) { return banks_[selector]; }
    auto *value = banks_[selector & 1U];
    return selector >= 4 ? transfer_long(value)
                         : ir_.CreateAnd(ir_.CreateLShr(value, ir_.getInt64(padding_bits)),
                                         ir_.getInt64(long_mask));
  }

  void load_long(const unsigned selector, V value) {
    if (selector == 2 || selector == 3) {
      banks_[selector] = value;
      return;
    }
    if (selector >= 6) {
      write_register(
          selector == 6 ? a_register : b_register,
          ir_.CreateTrunc(ir_.CreateLShr(value, ir_.getInt64(word_bits)), ir_.getInt32Ty()));
      write_register(
          selector == 6 ? b_register : a_register,
          ir_.CreateAnd(ir_.CreateTrunc(value, ir_.getInt32Ty()), ir_.getInt32(word_mask)));
      return;
    }
    auto *&destination = banks_[selector & 1U];
    if (selector >= 4) {
      destination = ir_.CreateShl(
          ir_.CreateSExt(ir_.CreateTrunc(value, ir_.getIntNTy(double_word_bits)), ir_.getInt64Ty()),
          ir_.getInt64(padding_bits));
    } else {
      destination =
          ir_.CreateOr(ir_.CreateAnd(destination, ir_.getInt64(~(long_mask << padding_bits))),
                       ir_.CreateShl(value, ir_.getInt64(padding_bits)));
    }
  }

  void long_transfer(const NativeMoves &moves, const Word opcode) {
    const auto selector = ((opcode >> 16) & 3U) | ((opcode >> 17) & 4U);
    auto *at = effective(cursors_, moves.transfers[0].address);
    auto *x = memory(moves.transfers[0], at);
    auto *y = memory(moves.transfers[1], at);
    if (moves.transfers[0].load) {
      auto *high = ir_.CreateAnd(ir_.CreateLoad(ir_.getInt32Ty(), x), ir_.getInt32(word_mask));
      auto *low = ir_.CreateAnd(ir_.CreateLoad(ir_.getInt32Ty(), y), ir_.getInt32(word_mask));
      load_long(selector,
                ir_.CreateOr(
                    ir_.CreateShl(ir_.CreateZExt(high, ir_.getInt64Ty()), ir_.getInt64(word_bits)),
                    ir_.CreateZExt(low, ir_.getInt64Ty())));
    } else {
      auto *value = long_value(selector);
      ir_.CreateStore(
          ir_.CreateTrunc(ir_.CreateLShr(value, ir_.getInt64(word_bits)), ir_.getInt32Ty()), x);
      ir_.CreateStore(
          ir_.CreateAnd(ir_.CreateTrunc(value, ir_.getInt32Ty()), ir_.getInt32(word_mask)), y);
    }
  }

  void transfers(const NativeMoves &moves, const Word opcode) {
    if (moves.long_pair) {
      long_transfer(moves, opcode);
      return;
    }
    const auto before = banks_;
    std::array<V, 2> words{};
    std::array<V, 2> pointers{};
    for (unsigned index = 0; index < moves.count; ++index) {
      const auto &move = moves.transfers[index];
      pointers[index] = memory(move, effective(cursors_, move.address));
      words[index] = move.load ? ir_.CreateAnd(ir_.CreateLoad(ir_.getInt32Ty(), pointers[index]),
                                               ir_.getInt32(word_mask))
                               : read_register(before, move.reg);
    }
    for (unsigned index = 0; index < moves.count; ++index) {
      const auto &move = moves.transfers[index];
      if (move.load) {
        write_register(move.reg, words[index]);
      } else {
        ir_.CreateStore(words[index], pointers[index]);
      }
    }
  }

  V condition(const unsigned selector) {
    const auto test = [&](const Word mask) {
      return ir_.CreateICmpNE(ir_.CreateAnd(status_, ir_.getInt32(mask)), ir_.getInt32(0));
    };
    auto *ge = ir_.CreateICmpEQ(test(negative), test(overflow));
    const std::array truth{ir_.CreateNot(test(carry)),
                           ge,
                           ir_.CreateNot(test(zero)),
                           ir_.CreateNot(test(negative)),
                           ir_.CreateAnd(ir_.CreateNot(test(zero)), test(unnormalized | extension)),
                           ir_.CreateNot(test(extension)),
                           ir_.CreateNot(test(limit)),
                           ir_.CreateAnd(ge, ir_.CreateNot(test(zero)))};
    auto *selected = truth[selector & 7U];
    return (selector & 8U) != 0 ? ir_.CreateNot(selected) : selected;
  }

  V next_pc() {
    const auto &last = plan_.operations.back();
    auto *following = ir_.getInt32((last.pc + last.instruction.words) & word_mask);
    if (!plan_.control.valid) { return following; }
    auto *target = plan_.control.reg < agu_registers
                       ? cursors_[plan_.control.reg]
                       : ir_.getInt32(plan_.control.target & word_mask);
    return plan_.control.selector == 16
               ? target
               : ir_.CreateSelect(condition(plan_.control.selector), target, following);
  }

  void finish() {
    for (unsigned bank = 0; bank < banks_.size(); ++bank) { put(banks_[bank], bank_fields[bank]); }
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (plan_.extents[0][index].accessed || plan_.extents[1][index].accessed ||
          plan_.changed_r[index]) {
        put(ir_.CreateAnd(cursors_[index], ir_.getInt32(word_mask)),
            offsetof(Registers, r) + (index * sizeof(Word)));
      }
    }
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (plan_.changed_n[index]) {
        put(strides_[index], offsetof(Registers, n) + (index * sizeof(Word)));
      }
    }
    put(status_, offsetof(Registers, sr));
    auto *retired = ir_.CreateMul(turns_, ir_.getInt64(plan_.count));
    store(ir_.CreateAdd(initial_instructions_, retired), offsetof(Cpu, instructions));
    store(ir_.CreateAdd(initial_cycles_, ir_.CreateMul(turns_, ir_.getInt64(plan_.cost))),
          offsetof(Cpu, cycles));
    store(ir_.getInt8(0), offsetof(Cpu, repeat_completed));
    store(ir_.getInt8(0), offsetof(Cpu, interrupt_inhibited));
    auto *pc = next_pc();
    put(pc, offsetof(Registers, pc));
    if (!at_loop_end(plan_.operations.back())) {
      LLVMBuildRet(ir_.get(), ir_.CreateTrunc(retired, ir_.getInt32Ty()));
      return;
    }
    auto *loop_finish = LLVMAppendBasicBlockInContext(context_, function_, "loop_finish");
    auto *done = LLVMAppendBasicBlockInContext(context_, function_, "done");
    LLVMBuildCondBr(
        ir_.get(),
        ir_.CreateICmpEQ(pc, ir_.getInt32((plan_.operations.back().loop_address + 1) & word_mask)),
        loop_finish,
        done);
    ir_.SetInsertPoint(loop_finish);
    auto *restore = LLVMAppendBasicBlockInContext(context_, function_, "restore_loop");
    put(ir_.CreateTrunc(ir_.CreateLShr(saved_, ir_.getInt64(word_bits)), ir_.getInt32Ty()),
        offsetof(Registers, pc));
    put(ir_.CreateAnd(ir_.CreateSub(lc_, ir_.CreateTrunc(turns_, ir_.getInt32Ty())),
                      ir_.getInt32(word_mask)),
        offsetof(Registers, lc));
    auto *final = ir_.CreateAnd(ir_.CreateNot(forever_), ir_.CreateICmpEQ(turns_, finite_count_));
    LLVMBuildCondBr(ir_.get(), final, restore, done);
    ir_.SetInsertPoint(restore);
    auto *previous =
        stack(ir_.CreateAnd(ir_.CreateSub(sp_, ir_.getInt32(1)), ir_.getInt32(stack_mask)));
    put(ir_.CreateAnd(ir_.CreateSub(sp_, ir_.getInt32(2)), ir_.getInt32(pointer_mask)),
        offsetof(Registers, sp));
    put(ir_.CreateSub(sc_, ir_.getInt32(2)), offsetof(Registers, sc));
    put(ir_.CreateTrunc(ir_.CreateLShr(previous, ir_.getInt64(word_bits)), ir_.getInt32Ty()),
        offsetof(Registers, la));
    put(ir_.CreateAnd(ir_.CreateTrunc(previous, ir_.getInt32Ty()), ir_.getInt32(word_mask)),
        offsetof(Registers, lc));
    constexpr Word loop_bits = loop_flag | forever_flag;
    put(ir_.CreateOr(
            ir_.CreateAnd(status_, ir_.getInt32(word_mask ^ loop_bits)),
            ir_.CreateAnd(ir_.CreateTrunc(saved_, ir_.getInt32Ty()), ir_.getInt32(loop_bits))),
        offsetof(Registers, sr));
    put(ir_.getInt32((plan_.operations.back().loop_address + 1) & word_mask),
        offsetof(Registers, pc));
    store(ir_.getInt8(static_cast<unsigned>(NativeBoundary::context)),
          offsetof(NativeContext, boundary),
          owner_);
    LLVMBuildBr(ir_.get(), done);
    ir_.SetInsertPoint(done);
    LLVMBuildRet(ir_.get(), ir_.CreateTrunc(retired, ir_.getInt32Ty()));
  }

  void generate(const void *fallback) {
    auto *windows = LLVMAppendBasicBlockInContext(context_, function_, "windows");
    auto *start = LLVMAppendBasicBlockInContext(context_, function_, "admitted");
    auto *miss = LLVMAppendBasicBlockInContext(context_, function_, "ordinary");
    auto *loop = LLVMAppendBasicBlockInContext(context_, function_, "loop");
    auto *complete = LLVMAppendBasicBlockInContext(context_, function_, "complete");
    admission();
    LLVMBuildCondBr(ir_.get(), eligible_, windows, miss);
    ir_.SetInsertPoint(windows);
    memory_guards();
    LLVMBuildCondBr(ir_.get(), eligible_, start, miss);
    ir_.SetInsertPoint(miss);
    std::array types{ir_.getPtrTy(), ir_.getPtrTy(), ir_.getPtrTy()};
    auto *helper_type = LLVMFunctionType(ir_.getInt32Ty(), types.data(), types.size(), 0);
    auto *helper = LLVMConstIntToPtr(ir_.getInt64(std::bit_cast<std::uintptr_t>(&enter_block)),
                                     ir_.getPtrTy());
    std::array arguments{
        LLVMConstIntToPtr(ir_.getInt64(std::bit_cast<std::uintptr_t>(fallback)), ir_.getPtrTy()),
        cpu_,
        owner_};
    LLVMBuildRet(
        ir_.get(),
        LLVMBuildCall2(ir_.get(), helper_type, helper, arguments.data(), arguments.size(), ""));
    ir_.SetInsertPoint(start);
    std::array<V, 4> initial{};
    for (unsigned bank = 0; bank < initial.size(); ++bank) {
      initial[bank] = reg(64, bank_fields[bank]);
    }
    LLVMBuildBr(ir_.get(), loop);
    ir_.SetInsertPoint(loop);
    std::array<V, 4> values{};
    for (unsigned bank = 0; bank < values.size(); ++bank) {
      values[bank] = LLVMBuildPhi(ir_.get(), ir_.getInt64Ty(), "bank");
      LLVMAddIncoming(values[bank], &initial[bank], &start, 1);
    }
    auto *status = LLVMBuildPhi(ir_.get(), ir_.getInt32Ty(), "status");
    LLVMAddIncoming(status, &status_, &start, 1);
    std::array<V, agu_registers> addresses{};
    std::array<V, agu_registers> strides{};
    for (unsigned index = 0; index < agu_registers; ++index) {
      addresses[index] = LLVMBuildPhi(ir_.get(), ir_.getInt32Ty(), "cursor");
      LLVMAddIncoming(addresses[index], &cursors_[index], &start, 1);
      strides[index] = LLVMBuildPhi(ir_.get(), ir_.getInt32Ty(), "stride");
      LLVMAddIncoming(strides[index], &strides_[index], &start, 1);
    }
    auto *remaining = LLVMBuildPhi(ir_.get(), ir_.getInt64Ty(), "remaining");
    LLVMAddIncoming(remaining, &turns_, &start, 1);
    banks_ = values;
    cursors_ = addresses;
    strides_ = strides;
    status_ = status;
    for (std::size_t index = 0; index < plan_.steps.size(); ++index) {
      const auto before = banks_;
      const auto &step = plan_.steps[index];
      if (step.registers.valid()) {
        write_register(step.registers.destination, read_register(before, step.registers.source));
      }
      const auto &instruction = plan_.operations[index].instruction;
      if (instruction.operation == Operation::arithmetic_shift_immediate) {
        llvm19::lower_arithmetic_shift(
            ir_, instruction.opcode, plan_.operations[index].mode, banks_, status_);
        continue;
      }
      if (plan_.control.valid && index + 1 == plan_.steps.size()) { continue; }
      transfers(step.memory, instruction.opcode);
      llvm19::lower_alu(ir_,
                        plan_.operations[index].instruction.opcode,
                        plan_.operations[index].mode,
                        before,
                        banks_,
                        status_);
    }
    for (unsigned bank = 0; bank < values.size(); ++bank) {
      LLVMAddIncoming(values[bank], &banks_[bank], &loop, 1);
    }
    for (unsigned index = 0; index < agu_registers; ++index) {
      LLVMAddIncoming(addresses[index], &cursors_[index], &loop, 1);
      LLVMAddIncoming(strides[index], &strides_[index], &loop, 1);
    }
    LLVMAddIncoming(status, &status_, &loop, 1);
    auto *left = ir_.CreateSub(remaining, ir_.getInt64(1));
    LLVMAddIncoming(remaining, &left, &loop, 1);
    LLVMBuildCondBr(ir_.get(), ir_.CreateICmpEQ(left, ir_.getInt64(0)), complete, loop);
    ir_.SetInsertPoint(complete);
    finish();
  }

  LLVMContextRef context_;
  LLVMValueRef function_;
  llvm19::Builder ir_;
  const RegionPlan &plan_;
  V cpu_, owner_, initial_cycles_ = nullptr, initial_instructions_ = nullptr;
  V status_ = nullptr, lc_ = nullptr, sp_ = nullptr, sc_ = nullptr, saved_ = nullptr;
  V turns_ = nullptr, forever_ = nullptr, finite_count_ = nullptr, eligible_ = nullptr;
  std::array<V, 4> banks_{};
  std::array<V, agu_registers> cursors_{}, strides_{}, masks_{}, origins_{};
  std::array<std::array<V, agu_registers>, 2> memory_bases_{};
};
}

bool region_candidate(const std::span<const BlockInstruction> operations,
                      const JitRuntime &runtime,
                      const bool complete_loop) {
  const auto plan = plan_region(operations, runtime);
  if (!plan || (!plan->batch && !plan->computation)) { return false; }
  // A partial execution grant can resume inside a previously prepared DO body.
  // Keep its ordinary suffix; do not translate each suffix into another LLVM loop.
  return complete_loop || plan->control.valid || !at_loop_end(operations.back());
}

std::optional<PreparedRegion> compile_region(std::unique_ptr<LlvmCompiler> &compiler,
                                             const std::span<const BlockInstruction> operations,
                                             const JitRuntime &runtime,
                                             const void *fallback) {
  const auto plan = plan_region(operations, runtime);
  if (!plan) { return {}; }
  if (!compiler) { compiler = std::make_unique<LlvmCompiler>(); }
  const auto before = compiler->native_bytes();
  LlvmModule module("dsp_region_" + std::to_string(std::bit_cast<std::uintptr_t>(fallback)));
  const RegionLifter lift(module, *plan, fallback);
  const auto *entry = compiler->install(module);
  return PreparedRegion{.entry = entry, .native_bytes = compiler->native_bytes() - before};
}
}
