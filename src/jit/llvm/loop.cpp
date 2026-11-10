// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/loop.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/alu.hpp"
#include "jit/llvm/builder.hpp"
#include "jit/llvm/runtime.hpp"
#include "jit/move.hpp"
#include "jit/resident.hpp"

#include <algorithm>
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
constexpr Word supported_modes = rounding_mode | scale_down | scale_up;

bool data_register(const unsigned reg) {
  return (reg >= x0_register && reg <= y1_register) || reg == a_register || reg == b_register;
}

struct LoopPlan {
  struct Step {
    NativeMoves memory;
    NativeRegisterMove registers;
  };

  std::span<const BlockInstruction> operations;
  std::vector<Step> steps;
  std::array<JitMemory, 2> memory;
  std::array<std::array<Word, agu_registers>, 2> accesses{};
  std::array<Word, agu_registers> deltas{};
  Word cost = 0, count = 0;
};

bool append_moves(LoopPlan &plan, const NativeMoves &moves) {
  Word changed_addresses = 0;
  for (const auto &move : std::span(moves.transfers).first(moves.count)) {
    if (!data_register(move.reg) || (move.address.mode != 3 && move.address.mode != 4) ||
        (changed_addresses & (1U << move.address.index)) != 0) {
      return false;
    }
    const auto bank = move.space == Space::x ? 0U : 1U;
    const auto &memory = plan.memory[bank];
    if (memory.data == nullptr || (!move.load && memory.aliases_program)) { return false; }
    auto &extent = plan.accesses[bank][move.address.index];
    extent = std::max(extent, plan.deltas[move.address.index] + 1);
    if (move.address.mode == 3) { ++plan.deltas[move.address.index]; }
    changed_addresses |= 1U << move.address.index;
  }
  return true;
}

std::optional<LoopPlan> plan_loop(const std::span<const BlockInstruction> operations,
                                  const JitRuntime &runtime) {
  if (operations.empty() || !at_loop_end(operations.back())) { return {}; }
  const auto mode = operations.front().mode;
  if ((mode & ~supported_modes) != 0 ||
      (mode & (scale_down | scale_up)) == (scale_down | scale_up)) {
    return {};
  }
  LoopPlan plan{.operations = operations, .steps = {}, .memory = resident_runtime(runtime).memory};
  auto pc = operations.front().pc;
  for (const auto &operation : operations) {
    const auto &instruction = operation.instruction;
    if (operation.pc != pc++ || operation.mode != mode ||
        operation.loop_address != operations.back().loop_address || instruction.words != 1 ||
        (instruction.operation != Operation::parallel && instruction.operation != Operation::nop) ||
        !llvm19::supported_alu(instruction.opcode)) {
      return {};
    }
    auto moves = plan_moves(instruction);
    const auto registers = plan_register_move(instruction);
    if (moves.long_pair || !append_moves(plan, moves)) { return {}; }
    if (registers.valid() &&
        (!data_register(registers.source) || !data_register(registers.destination))) {
      return {};
    }
    if (instruction.operation == Operation::parallel && moves.count == 0 && !registers.valid() &&
        (instruction.opcode & upper_move_mask) != no_move_group) {
      return {};
    }
    plan.steps.push_back({.memory = moves, .registers = registers});
    plan.cost += instruction.cycles;
    ++plan.count;
  }
  return plan;
}

class LoopLifter {
public:
  LoopLifter(LlvmModule &module, const LoopPlan &plan, const void *fallback)
      : context_(module.context()),
        function_(make_function(module)),
        ir_(context_, LLVMAppendBasicBlockInContext(context_, function_, "entry")),
        plan_(plan),
        cpu_(LLVMGetParam(function_, 0)),
        owner_(LLVMGetParam(function_, 1)) {
    generate(fallback);
  }

private:
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
    turns_ = ir_.CreateSelect(forever_, turns_, minimum(turns_, finite_count_));
    eligible_ = ir_.CreateICmpUGT(turns_, ir_.getInt64(0));
    require(ir_.CreateICmpEQ(ir_.CreateAnd(status_, ir_.getInt32(numeric_mode_mask | loop_flag)),
                             ir_.getInt32(loop_flag | plan_.operations.front().mode)));
    require(ir_.CreateICmpEQ(reg(32, offsetof(Registers, pc)),
                             ir_.getInt32(plan_.operations.front().pc)));
    require(ir_.CreateICmpEQ(reg(32, offsetof(Registers, la)),
                             ir_.getInt32(plan_.operations.back().loop_address)));
    require(ir_.CreateICmpEQ(ir_.CreateLShr(saved_, ir_.getInt64(word_bits)),
                             ir_.getInt64(plan_.operations.front().pc)));
    require(ir_.CreateICmpUGE(sc_, ir_.getInt32(2)));
    require(ir_.CreateICmpEQ(load(32, offsetof(Cpu, repeat_remaining)), ir_.getInt32(0)));
    for (const auto offset : {offsetof(Cpu, processing), offsetof(Cpu, debug_flags)}) {
      require(ir_.CreateICmpEQ(load(8, offset), ir_.getInt8(0)));
    }
    memory_guards();
  }

  void memory_guards() {
    for (unsigned index = 0; index < agu_registers; ++index) {
      cursors_[index] = reg(32, offsetof(Registers, r) + (index * sizeof(Word)));
      for (unsigned bank = 0; bank < 2; ++bank) {
        const auto extent = plan_.accesses[bank][index];
        if (extent == 0) { continue; }
        auto *last =
            ir_.CreateAdd(ir_.CreateZExt(cursors_[index], ir_.getInt64Ty()),
                          ir_.CreateAdd(ir_.getInt64(extent - 1),
                                        ir_.CreateMul(ir_.CreateSub(turns_, ir_.getInt64(1)),
                                                      ir_.getInt64(plan_.deltas[index]))));
        const auto bound = std::min(plan_.memory[bank].words, std::size_t{io_begin});
        require(ir_.CreateICmpULT(last, ir_.getInt64(bound)));
        auto *modifier = reg(32, offsetof(Registers, m) + (index * sizeof(Word)));
        require(ir_.CreateICmpEQ(ir_.CreateAnd(modifier, ir_.getInt32(modifier_mask)),
                                 ir_.getInt32(modifier_mask)));
      }
    }
  }

  V memory(const NativeMove &move) {
    const auto bank = move.space == Space::x ? 0U : 1U;
    auto *base = LLVMConstIntToPtr(
        ir_.getInt64(std::bit_cast<std::uintptr_t>(plan_.memory[bank].data)), ir_.getPtrTy());
    return ir_.CreateGEP(
        ir_.getInt32Ty(), base, ir_.CreateZExt(cursors_[move.address.index], ir_.getInt64Ty()));
  }

  V read_register(const std::array<V, 4> &values, const unsigned number) {
    if (number >= a_register) {
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
    if (number >= a_register) {
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

  void transfers(const NativeMoves &moves) {
    const auto before = banks_;
    std::array<V, 2> words{};
    std::array<V, 2> pointers{};
    for (unsigned index = 0; index < moves.count; ++index) {
      const auto &move = moves.transfers[index];
      pointers[index] = memory(move);
      words[index] = move.load ? ir_.CreateAnd(ir_.CreateLoad(ir_.getInt32Ty(), pointers[index]),
                                               ir_.getInt32(word_mask))
                               : read_register(before, move.reg);
      if (move.address.mode == 3) {
        cursors_[move.address.index] = ir_.CreateAdd(cursors_[move.address.index], ir_.getInt32(1));
      }
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

  void finish() {
    for (unsigned bank = 0; bank < banks_.size(); ++bank) { put(banks_[bank], bank_fields[bank]); }
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (plan_.deltas[index] != 0) {
        put(cursors_[index], offsetof(Registers, r) + (index * sizeof(Word)));
      }
    }
    put(status_, offsetof(Registers, sr));
    auto *retired = ir_.CreateMul(turns_, ir_.getInt64(plan_.count));
    store(ir_.CreateAdd(initial_instructions_, retired), offsetof(Cpu, instructions));
    store(ir_.CreateAdd(initial_cycles_, ir_.CreateMul(turns_, ir_.getInt64(plan_.cost))),
          offsetof(Cpu, cycles));
    store(ir_.getInt8(0), offsetof(Cpu, repeat_completed));
    store(ir_.getInt8(0), offsetof(Cpu, interrupt_inhibited));
    auto *done = LLVMAppendBasicBlockInContext(context_, function_, "done");
    auto *restore = LLVMAppendBasicBlockInContext(context_, function_, "restore_loop");
    put(ir_.getInt32(plan_.operations.front().pc), offsetof(Registers, pc));
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
    auto *start = LLVMAppendBasicBlockInContext(context_, function_, "admitted");
    auto *miss = LLVMAppendBasicBlockInContext(context_, function_, "ordinary");
    auto *loop = LLVMAppendBasicBlockInContext(context_, function_, "loop");
    auto *complete = LLVMAppendBasicBlockInContext(context_, function_, "complete");
    admission();
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
    for (unsigned index = 0; index < agu_registers; ++index) {
      addresses[index] = LLVMBuildPhi(ir_.get(), ir_.getInt32Ty(), "cursor");
      LLVMAddIncoming(addresses[index], &cursors_[index], &start, 1);
    }
    auto *remaining = LLVMBuildPhi(ir_.get(), ir_.getInt64Ty(), "remaining");
    LLVMAddIncoming(remaining, &turns_, &start, 1);
    banks_ = values;
    cursors_ = addresses;
    status_ = status;
    for (std::size_t index = 0; index < plan_.steps.size(); ++index) {
      const auto before = banks_;
      const auto &step = plan_.steps[index];
      if (step.registers.valid()) {
        write_register(step.registers.destination, read_register(before, step.registers.source));
      }
      transfers(step.memory);
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
  const LoopPlan &plan_;
  V cpu_, owner_, initial_cycles_ = nullptr, initial_instructions_ = nullptr;
  V status_ = nullptr, lc_ = nullptr, sp_ = nullptr, sc_ = nullptr, saved_ = nullptr;
  V turns_ = nullptr, forever_ = nullptr, finite_count_ = nullptr, eligible_ = nullptr;
  std::array<V, 4> banks_{};
  std::array<V, agu_registers> cursors_{};
};
}

bool loop_candidate(const std::span<const BlockInstruction> operations, const JitRuntime &runtime) {
  return plan_loop(operations, runtime).has_value();
}

std::optional<PreparedLoop> compile_loop(std::unique_ptr<LlvmCompiler> &compiler,
                                         const std::span<const BlockInstruction> operations,
                                         const JitRuntime &runtime,
                                         const void *fallback) {
  const auto plan = plan_loop(operations, runtime);
  if (!plan) { return {}; }
  if (!compiler) { compiler = std::make_unique<LlvmCompiler>(); }
  const auto before = compiler->native_bytes();
  LlvmModule module("dsp_loop_" + std::to_string(std::bit_cast<std::uintptr_t>(fallback)));
  const LoopLifter lift(module, *plan, fallback);
  const auto *entry = compiler->install(module);
  return PreparedLoop{.entry = entry, .native_bytes = compiler->native_bytes() - before};
}
}
