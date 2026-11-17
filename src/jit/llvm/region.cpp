// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/region.hpp"

#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/alu.hpp"
#include "jit/llvm/builder.hpp"
#include "jit/llvm/region_memory.hpp"
#include "jit/llvm/region_plan.hpp"
#include "jit/llvm/runtime.hpp"
#include "jit/move.hpp"
#include "translation.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <llvm/IR/Constant.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Value.h>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
using V = LLVMValueRef;
constexpr std::array<std::size_t, 4> bank_fields{
    offsetof(Registers, a), offsetof(Registers, b), offsetof(Registers, x), offsetof(Registers, y)};
constexpr Word stack_mask = stack_entries - 1, pointer_mask = 63;
constexpr unsigned bus_shift = word_bits + padding_bits;

std::optional<std::size_t> admission_prefix(const RegionPlan &plan) {
  std::size_t count = 0;
  for (std::size_t index = 0; index < plan.steps.size(); ++index) {
    if (plan.steps[index].guards != 0) { count = index + 1; }
  }
  for (const auto &step : std::span(plan.steps).first(count)) {
    if (step.repeat_end != 0) { return {}; }
    if (std::ranges::any_of(std::span(step.memory.transfers).first(step.memory.count),
                            [](const auto &move) { return !move.load; })) {
      return {};
    }
  }
  return count;
}

enum class BatchAddressing : std::uint8_t { automatic, linear, general };

[[nodiscard]] bool split_flags(const RegionPlan &plan) {
  return plan.batch && plan.computation &&
         std::ranges::none_of(plan.operations, [](const auto &operation) {
           const auto &instruction = operation.instruction;
           return instruction.operation != Operation::nop &&
                  instruction.operation != Operation::bit_memory &&
                  instruction.operation != Operation::arithmetic_shift_immediate &&
                  (instruction.operation != Operation::parallel ||
                   llvm19::flag_inputs(instruction.opcode) != 0);
         });
}

class RegionLifter {
public:
  RegionLifter(LlvmModule &module,
               const RegionPlan &plan,
               const void *fallback,
               const std::string_view entry = {},
               const bool admission_only = false,
               const std::size_t admission_steps = 0,
               const BatchAddressing addressing = BatchAddressing::automatic)
      : context_(module.context()),
        function_(make_function(module, entry, admission_only)),
        ir_(context_, LLVMAppendBasicBlockInContext(context_, function_, "entry")),
        plan_(plan),
        addressing_(addressing),
        dynamic_(fallback == nullptr),
        self_loop_(
            !admission_only && dynamic_ && !plan_.batch &&
            (captured_loop() || (region_independent(plan_.operations) && plan_.control.valid &&
                                 (plan_.control.reg < agu_registers ||
                                  plan_.control.target == plan_.operations.front().pc)))),
        cpu_(LLVMGetParam(function_, 0)),
        owner_(LLVMGetParam(function_, 1)) {
    for (unsigned bank = 0; bank < bindings_.size(); ++bank) {
      bindings_[bank].resize(plan_.windows[bank].size());
    }
    if ((plan_.batch || !plan_.repeats.empty()) && addressing_ == BatchAddressing::automatic &&
        std::ranges::any_of(plan_.agu, [](bool used) { return used; })) {
      generate_versions(module, fallback, admission_only, admission_steps);
    } else if (admission_only) {
      generate_admission(admission_steps);
    } else if (self_loop_) {
      generate_self_loop();
    } else {
      generate(fallback);
    }
  }

private:
  struct Bounds {
    V first = nullptr, last = nullptr;
  };

  using Ranges = std::array<std::array<Bounds, agu_registers>, 2>;

  [[nodiscard]] bool captured_loop() const {
    return at_loop_end(plan_.operations.back()) && !plan_.control.valid;
  }

  V boolean(const bool value) { return LLVMConstInt(ir_.getIntNTy(1), value ? 1U : 0U, 0); }

  static LLVMValueRef
  make_function(LlvmModule &module, const std::string_view entry, const bool admission_only) {
    auto *pointer = LLVMPointerTypeInContext(module.context(), 0);
    std::array arguments{pointer, pointer};
    auto *type = LLVMFunctionType((admission_only ? LLVMInt64TypeInContext(module.context())
                                                  : LLVMInt32TypeInContext(module.context())),
                                  arguments.data(),
                                  arguments.size(),
                                  0);
    auto *function = LLVMAddFunction(
        module.code(), entry.empty() ? module.name().c_str() : std::string(entry).c_str(), type);
    return function;
  }

  void generate_versions(LlvmModule &module,
                         const void *fallback,
                         const bool admission_only,
                         const std::size_t admission_steps) {
    const std::string name = LLVMGetValueName(function_);
    const RegionLifter linear(module,
                              plan_,
                              fallback,
                              name + ".linear",
                              admission_only,
                              admission_steps,
                              BatchAddressing::linear);
    const RegionLifter general(module,
                               plan_,
                               fallback,
                               name + ".general",
                               admission_only,
                               admission_steps,
                               BatchAddressing::general);
    auto *all_linear = boolean(true);
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (!plan_.agu[index]) { continue; }
      auto *modifier = ir_.CreateAnd(reg(32, offsetof(Registers, m) + (index * sizeof(Word))),
                                     ir_.getInt32(modifier_mask));
      all_linear =
          ir_.CreateAnd(all_linear, ir_.CreateICmpEQ(modifier, ir_.getInt32(modifier_mask)));
    }
    auto *first = LLVMAppendBasicBlockInContext(context_, function_, "linear");
    auto *second = LLVMAppendBasicBlockInContext(context_, function_, "general");
    LLVMBuildCondBr(ir_.get(), all_linear, first, second);
    for (const auto [block, target] :
         {std::pair{first, linear.function_}, std::pair{second, general.function_}}) {
      LLVMSetLinkage(target, LLVMInternalLinkage);
      LLVMAddAttributeAtIndex(
          target,
          static_cast<LLVMAttributeIndex>(LLVMAttributeFunctionIndex),
          LLVMCreateEnumAttribute(
              context_, LLVMGetEnumAttributeKindForName("alwaysinline", 12), 0));
      ir_.SetInsertPoint(block);
      std::array arguments{cpu_, owner_};
      LLVMBuildRet(ir_.get(),
                   LLVMBuildCall2(ir_.get(),
                                  LLVMGlobalGetValueType(target),
                                  target,
                                  arguments.data(),
                                  arguments.size(),
                                  ""));
    }
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

  V stack_address(V index) {
    auto *bytes =
        ir_.CreateMul(ir_.CreateZExt(index, ir_.getInt64Ty()), ir_.getInt64(sizeof(std::uint64_t)));
    auto *base = address(cpu_, offsetof(Cpu, reg) + offsetof(Registers, stack));
    return ir_.CreateGEP(ir_.getInt8Ty(), base, bytes);
  }

  V stack(V index) { return ir_.CreateLoad(ir_.getInt64Ty(), stack_address(index)); }

  void stack_admission() {
    if (plan_.depth != 0) {
      const auto added = 2U * (plan_.depth + (plan_.setup ? 1U : 0U));
      require(added < stack_entries
                  ? ir_.CreateICmpUGE(ir_.getInt32(stack_entries - 1 - added), sc_)
                  : boolean(false));
    }
  }

  void stack_state() {
    const auto base = plan_.setup ? 2U : 0U;
    for (unsigned offset = base + 1; offset <= base + (2U * plan_.depth); ++offset) {
      frames_[offset] =
          stack(ir_.CreateAnd(ir_.CreateAdd(sp_, ir_.getInt32(offset)), ir_.getInt32(stack_mask)));
    }
  }

  void admission() {
    initial_cycles_ = load(64, offsetof(Cpu, cycles));
    initial_instructions_ = load(64, offsetof(Cpu, instructions));
    status_ = reg(32, offsetof(Registers, sr));
    initial_status_ = status_;
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
    if (plan_.batch || captured_loop()) {
      turns_ = ir_.CreateSelect(forever_, turns_, minimum(turns_, finite_count_));
    } else if (!self_loop_) {
      turns_ = minimum(turns_, ir_.getInt64(1));
    }
    eligible_ = ir_.CreateICmpUGT(turns_, ir_.getInt64(0));
    if (plan_.setup) {
      const auto count = (std::uint64_t{plan_.count} * plan_.setup_count) + 1;
      const auto cost =
          (std::uint64_t{plan_.cost} * plan_.setup_count) + plan_.setup->instruction.cycles;
      turns_ = ir_.getInt64(plan_.setup_count);
      eligible_ = ir_.CreateAnd(ir_.CreateICmpUGE(budget, ir_.getInt32(count)),
                                ir_.CreateICmpUGE(credit, ir_.getInt64(cost)));
      require(ir_.CreateICmpUGE(ir_.getInt64(std::numeric_limits<std::uint64_t>::max() - count),
                                initial_instructions_));
      require(ir_.CreateICmpUGE(ir_.getInt32(stack_entries - 3), sc_));
    }
    stack_admission();
    stack_state();
    require(ir_.CreateICmpEQ(ir_.CreateAnd(status_, ir_.getInt32(numeric_mode_mask)),
                             ir_.getInt32(plan_.operations.front().mode)));
    require(ir_.CreateICmpEQ(
        reg(32, offsetof(Registers, pc)),
        ir_.getInt32(plan_.setup ? plan_.setup->pc : plan_.operations.front().pc)));
    auto *active_la = ir_.CreateSelect(
        ir_.CreateICmpNE(ir_.CreateAnd(status_, ir_.getInt32(loop_flag)), ir_.getInt32(0)),
        reg(32, offsetof(Registers, la)),
        ir_.getInt32(inactive_loop));
    const auto &last = plan_.operations.back();
    const auto final_word = (last.pc + last.instruction.words - 1) & word_mask;
    const bool uncaptured =
        last.loop_address == inactive_loop && plan_.operations.front().pc <= final_word;
    const auto first_pc = plan_.setup ? plan_.setup->pc : plan_.operations.front().pc;
    auto *outside = ir_.CreateOr(ir_.CreateICmpULT(active_la, ir_.getInt32(first_pc)),
                                 ir_.CreateICmpUGT(active_la, ir_.getInt32(final_word)));
    if (!plan_.setup && uncaptured && !region_independent(plan_.operations)) {
      endpoint_ = ir_.CreateICmpEQ(active_la, ir_.getInt32(final_word));
      require(ir_.CreateOr(outside, endpoint_));
      require(ir_.CreateOr(ir_.CreateNot(endpoint_), ir_.CreateICmpUGE(sc_, ir_.getInt32(2))));
    } else if (plan_.setup || uncaptured || region_independent(plan_.operations)) {
      require(outside);
    } else {
      require(ir_.CreateICmpEQ(active_la, ir_.getInt32(plan_.operations.front().loop_address)));
    }
    if (!plan_.setup && at_loop_end(plan_.operations.back())) {
      require(ir_.CreateICmpUGE(sc_, ir_.getInt32(2)));
    }
    if ((plan_.batch || (self_loop_ && captured_loop())) && !plan_.setup) {
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
      if (plan_.batch && addressing_ == BatchAddressing::linear) { require(linear); }
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

  V known_words(const RegionWindow &window,
                const Bounds bounds,
                const bool writing,
                V dynamic_known) {
    if (!window.tracked) { return boolean(true); }
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
    auto *base = dynamic_
                     ? dynamic_known
                     : LLVMConstIntToPtr(ir_.getInt64(std::bit_cast<std::uintptr_t>(window.known)),
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

  struct Binding {
    V data = nullptr, known = nullptr, valid = nullptr;
  };

  Binding binding(const unsigned bank, const unsigned index, const bool writing) {
    auto &cached = bindings_[bank][index][writing ? 1U : 0U];
    if (cached.valid != nullptr) { return cached; }
    const auto &window = plan_.windows[bank][index];
    const bool explicit_windows = plan_.windowed[bank];
    const auto flags = (window.writable ? region_writable : 0U) |
                       (window.tracked ? region_tracked : 0U) |
                       (window.aliases_program ? region_program_alias : 0U) |
                       (explicit_windows ? region_windowed : 0U) | (writing ? region_writing : 0U);
    std::array values{ir_.getInt32(bank),
                      ir_.getInt32(index),
                      ir_.getInt32(static_cast<Word>(plan_.windows[bank].size())),
                      ir_.getInt32(window.address),
                      ir_.getInt32(static_cast<Word>(window.size)),
                      ir_.getInt32(flags)};
    auto *layout = LLVMConstStructInContext(context_, values.data(), values.size(), 0);
    auto *module = LLVMGetGlobalParent(function_);
    auto *description = LLVMAddGlobal(module, LLVMTypeOf(layout), "region.memory");
    LLVMSetLinkage(description, LLVMPrivateLinkage);
    LLVMSetGlobalConstant(description, 1);
    LLVMSetInitializer(description, layout);
    LLVMSetAlignment(description, alignof(RegionMemory));
    auto *prior = LLVMGetInsertBlock(ir_.get());
    LLVMPositionBuilderBefore(ir_.get(),
                              LLVMGetFirstInstruction(LLVMGetEntryBasicBlock(function_)));
    auto *data = LLVMBuildAlloca(ir_.get(), ir_.getPtrTy(), "data");
    auto *known = LLVMBuildAlloca(ir_.get(), ir_.getPtrTy(), "known");
    ir_.SetInsertPoint(prior);
    ir_.CreateStore(LLVMConstNull(ir_.getPtrTy()), data);
    ir_.CreateStore(LLVMConstNull(ir_.getPtrTy()), known);
    std::array types{
        ir_.getPtrTy(), ir_.getPtrTy(), ir_.getPtrTy(), ir_.getPtrTy(), ir_.getPtrTy()};
    auto *type = LLVMFunctionType(ir_.getInt32Ty(), types.data(), types.size(), 0);
    auto *helper = LLVMGetNamedFunction(module, "dsp56300_region_memory");
    if (helper == nullptr) { helper = LLVMAddFunction(module, "dsp56300_region_memory", type); }
    std::array arguments{cpu_, owner_, description, data, known};
    auto *valid = LLVMBuildCall2(ir_.get(), type, helper, arguments.data(), arguments.size(), "");
    cached = {.data = ir_.CreateLoad(ir_.getPtrTy(), data),
              .known = ir_.CreateLoad(ir_.getPtrTy(), known),
              .valid = ir_.CreateICmpNE(valid, ir_.getInt32(0))};
    return cached;
  }

  [[nodiscard]] std::vector<unsigned> window_order(const unsigned bank, const bool writing) const {
    const auto &windows = plan_.windows[bank];
    std::vector<unsigned> order;
    bool disjoint = true;
    for (unsigned index = 0; index < windows.size(); ++index) {
      const auto &window = windows[index];
      if (!permitted(window, writing)) { continue; }
      for (const auto prior : order) {
        const auto &other = windows[prior];
        if (window.address < other.address + other.size &&
            other.address < window.address + window.size) {
          disjoint = false;
        }
      }
      order.push_back(index);
    }
    if (!disjoint) { std::ranges::reverse(order); }
    return order;
  }

  V select_window(const unsigned bank, const Bounds bounds, const bool writing) {
    const auto order = window_order(bank, writing);
    if (dynamic_) {
      for (const auto index : order) { static_cast<void>(binding(bank, index, writing)); }
    }
    auto *done = LLVMAppendBasicBlockInContext(context_, function_, "window_selected");
    std::vector<V> pointers;
    std::vector<V> acceptance;
    std::vector<LLVMBasicBlockRef> predecessors;
    for (const auto index : order) {
      const auto &window = plan_.windows[bank][index];
      auto *begin = ir_.getInt64(window.address);
      auto *end = ir_.getInt64(window.address + window.size);
      auto *matches = ir_.CreateAnd(ir_.CreateICmpUGE(bounds.first, begin),
                                    ir_.CreateICmpULT(bounds.last, end));
      matches = ir_.CreateAnd(matches, ir_.CreateICmpULT(bounds.first, end));
      const auto bound = dynamic_ ? binding(bank, index, writing) : Binding{};
      if (dynamic_) { matches = ir_.CreateAnd(matches, bound.valid); }
      auto *inspect = LLVMAppendBasicBlockInContext(context_, function_, "window_known");
      auto *selected = LLVMAppendBasicBlockInContext(context_, function_, "window_match");
      auto *next = LLVMAppendBasicBlockInContext(context_, function_, "window_next");
      LLVMBuildCondBr(ir_.get(), matches, inspect, next);
      ir_.SetInsertPoint(inspect);
      const Bounds range{.first = ir_.CreateSub(bounds.first, begin),
                         .last = ir_.CreateAdd(ir_.CreateSub(bounds.last, begin), ir_.getInt64(1))};
      auto *known = known_words(window, range, writing, bound.known);
      LLVMBuildCondBr(ir_.get(), known, selected, next);
      ir_.SetInsertPoint(selected);
      auto *pointer = dynamic_ ? LLVMBuildPtrToInt(ir_.get(), bound.data, ir_.getInt64Ty(), "")
                               : ir_.getInt64(std::bit_cast<std::uintptr_t>(window.data));
      pointers.push_back(LLVMBuildIntToPtr(
          ir_.get(),
          ir_.CreateSub(pointer, ir_.getInt64(std::uint64_t{window.address} * sizeof(Word))),
          ir_.getPtrTy(),
          ""));
      acceptance.push_back(boolean(true));
      predecessors.push_back(LLVMGetInsertBlock(ir_.get()));
      LLVMBuildBr(ir_.get(), done);
      ir_.SetInsertPoint(next);
    }
    pointers.push_back(LLVMConstNull(ir_.getPtrTy()));
    acceptance.push_back(boolean(false));
    predecessors.push_back(LLVMGetInsertBlock(ir_.get()));
    LLVMBuildBr(ir_.get(), done);
    ir_.SetInsertPoint(done);
    auto *base = LLVMBuildPhi(ir_.get(), ir_.getPtrTy(), "window_base");
    auto *accepted = LLVMBuildPhi(ir_.get(), ir_.getIntNTy(1), "window_accepted");
    LLVMAddIncoming(
        base, pointers.data(), predecessors.data(), static_cast<unsigned>(pointers.size()));
    LLVMAddIncoming(
        accepted, acceptance.data(), predecessors.data(), static_cast<unsigned>(acceptance.size()));
    require(accepted);
    return base;
  }

  Bounds address_bounds(Bounds range, const unsigned index) {
    if (plan_.agu[index] && addressing_ != BatchAddressing::linear) {
      auto *linear = ir_.CreateICmpEQ(masks_[index], ir_.getInt32(word_mask));
      auto *begin = ir_.CreateZExt(origins_[index], ir_.getInt64Ty());
      auto *end = ir_.CreateAdd(begin, ir_.CreateZExt(masks_[index], ir_.getInt64Ty()));
      range.first = ir_.CreateSelect(linear, range.first, begin);
      range.last = ir_.CreateSelect(linear, range.last, end);
    }
    return range;
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
        memory_bases_[bank][index] = select_window(
            bank, address_bounds({.first = first, .last = last}, index), extent.writing);
      }
    }
  }

  V advanced(V cursor, const AddressForm form) {
    auto *step = form.mode < 2 || form.mode == 5 ? strides_[form.index] : ir_.getInt32(1);
    auto *next = form.mode == 0 || form.mode == 2 || form.mode == 7 ? ir_.CreateSub(cursor, step)
                                                                    : ir_.CreateAdd(cursor, step);
    // The single-operation batch has already proved every accessed address in range.
    // Its final post-update may wrap; finish() masks the committed register.
    if (split_flags_ && loop_count_ == 1 && addressing_ == BatchAddressing::linear) {
      LLVMSetNSW(next, 1);
      if (form.mode == 3) { LLVMSetNUW(next, 1); }
      return next;
    }
    return ir_.CreateOr(origins_[form.index], ir_.CreateAnd(next, masks_[form.index]));
  }

  V effective(std::array<V, agu_registers> &cursors, const AddressForm form) {
    if (form.mode == 6) { return ir_.getInt32(form.extra & word_mask); }
    auto *prior = cursors[form.index];
    if (form.mode == 4) { return prior; }
    auto *next = advanced(prior, form);
    if (form.mode != 5) { cursors[form.index] = next; }
    return form.mode == 5 || form.mode == 7 ? next : prior;
  }

  void extend(Bounds &range, const Bounds next) {
    range.first = range.first == nullptr ? next.first : minimum(range.first, next.first);
    range.last =
        range.last == nullptr
            ? next.last
            : ir_.CreateSelect(ir_.CreateICmpUGT(next.last, range.last), next.last, range.last);
  }

  void repeat_bounds(const Word roots,
                     const RegionPlan::Repeat &repeat,
                     std::array<V, agu_registers> &proof,
                     Ranges &bounds) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      if ((roots & (1U << index)) == 0) { continue; }
      for (unsigned bank = 0; bank < 2; ++bank) {
        const auto &extent = repeat.extents[bank][index];
        if (!extent.accessed) { continue; }
        auto *origin = ir_.CreateZExt(proof[index], ir_.getInt64Ty());
        const Bounds range{.first = ir_.CreateAdd(
                               origin, ir_.getInt64(static_cast<std::uint64_t>(extent.minimum))),
                           .last = ir_.CreateAdd(
                               origin, ir_.getInt64(static_cast<std::uint64_t>(extent.maximum)))};
        extend(bounds[bank][index], address_bounds(range, index));
      }
      if (repeat.deltas[index] == 0) { continue; }
      auto *next = ir_.CreateAdd(proof[index],
                                 ir_.getInt32(static_cast<std::uint64_t>(repeat.deltas[index])));
      proof[index] = ir_.CreateOr(origins_[index], ir_.CreateAnd(next, masks_[index]));
    }
  }

  auto memory_bounds(const Word roots, const std::span<const RegionPlan::Step> steps) {
    auto proof = cursors_;
    Ranges bounds{};
    const auto first = static_cast<std::size_t>(steps.data() - plan_.steps.data());
    for (auto index = first; index < first + steps.size(); ++index) {
      const auto &step = plan_.steps[index];
      if (step.repeat_end != 0) {
        repeat_bounds(roots, plan_.repeats.at(index), proof, bounds);
        index = step.repeat_end - 1;
        continue;
      }
      if (step.update) { static_cast<void>(effective(proof, *step.update)); }
      V paired = nullptr;
      for (const auto &move : std::span(step.memory.transfers).first(step.memory.count)) {
        if (move.address.mode == 6 || (roots & (1U << move.address.index)) == 0) { continue; }
        auto *at =
            step.memory.long_pair && paired != nullptr ? paired : effective(proof, move.address);
        paired = at;
        auto &range = bounds[move.space == Space::x ? 0U : 1U][move.address.index];
        auto *wide = ir_.CreateZExt(at, ir_.getInt64Ty());
        extend(range, {.first = wide, .last = wide});
      }
    }
    return bounds;
  }

  void memory_guards(const Word roots, const std::span<const RegionPlan::Step> steps) {
    const auto bounds = memory_bounds(roots, steps);
    for (unsigned bank = 0; bank < 2; ++bank) {
      for (unsigned index = 0; index < agu_registers; ++index) {
        if ((roots & (1U << index)) == 0 || !plan_.extents[bank][index].accessed) { continue; }
        memory_bases_[bank][index] =
            select_window(bank, bounds[bank][index], plan_.extents[bank][index].writing);
      }
    }
  }

  void initial_memory_guards() {
    for (unsigned bank = 0; bank < 2; ++bank) {
      for (const auto &[at, writing] : plan_.absolute[bank]) {
        auto *value = ir_.getInt64(at);
        absolute_bases_[bank][at] = select_window(bank, {.first = value, .last = value}, writing);
      }
    }
    if (plan_.batch) {
      batch_memory_guards();
      return;
    }
    Word roots = 0;
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (!plan_.deferred[index]) { roots |= 1U << index; }
    }
    memory_guards(roots, plan_.steps);
  }

  V memory(const NativeMove &move, V at) {
    const auto bank = move.space == Space::x ? 0U : 1U;
    auto *base = move.address.mode == 6 ? absolute_bases_[bank].at(move.address.extra & word_mask)
                                        : memory_bases_[bank][move.address.index];
    return ir_.CreateGEP(ir_.getInt32Ty(), base, ir_.CreateZExt(at, ir_.getInt64Ty()));
  }

  V read_register(const std::array<V, 4> &values, const unsigned number) {
    if (number >= n_register_base) { return strides_[number - n_register_base]; }
    if (number >= r_register_base) { return cursors_[number - r_register_base]; }
    if (number == a_register || number == b_register) {
      return llvm19::transfer_word(
          ir_, values[number - a_register], plan_.operations.front().mode, status_);
    }
    if (number >= a0_register && number <= b1_register) {
      constexpr std::array shifts{padding_bits, accumulator_bits, bus_shift};
      const auto part = (number - a0_register) / 2;
      auto *value = values[(number - a0_register) & 1U];
      auto *shift = ir_.getInt64(shifts[part]);
      auto *word =
          part == 1 ? LLVMBuildAShr(ir_.get(), value, shift, "") : ir_.CreateLShr(value, shift);
      return ir_.CreateAnd(ir_.CreateTrunc(word, ir_.getInt32Ty()), ir_.getInt32(word_mask));
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
      const auto index = number - r_register_base;
      cursors_[index] = word;
      if (plan_.agu[index]) { origins_[index] = ir_.CreateAnd(word, ir_.CreateNot(masks_[index])); }
      return;
    }
    if (number == a_register || number == b_register) {
      banks_[number - a_register] = ir_.CreateShl(
          ir_.CreateSExt(ir_.CreateTrunc(word, ir_.getIntNTy(word_bits)), ir_.getInt64Ty()),
          ir_.getInt64(bus_shift));
      return;
    }
    if (number >= a0_register && number <= b1_register) {
      constexpr std::array shifts{padding_bits, accumulator_bits, bus_shift};
      const auto shift = shifts[(number - a0_register) / 2];
      auto *&value = banks_[(number - a0_register) & 1U];
      value =
          ir_.CreateOr(ir_.CreateAnd(value, ir_.getInt64(~(std::uint64_t{word_mask} << shift))),
                       ir_.CreateShl(ir_.CreateZExt(word, ir_.getInt64Ty()), ir_.getInt64(shift)));
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
      if (move.immediate()) {
        words[index] = ir_.getInt32(move.address.extra & word_mask);
        continue;
      }
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

  V next_pc() {
    const auto &last = plan_.operations.back();
    auto *following = ir_.getInt32((last.pc + last.instruction.words) & word_mask);
    if (!plan_.control.valid) { return following; }
    auto *target = plan_.control.reg < agu_registers
                       ? cursors_[plan_.control.reg]
                       : ir_.getInt32(plan_.control.target & word_mask);
    return plan_.control.selector == 16
               ? target
               : ir_.CreateSelect(
                     llvm19::condition(ir_, status_, plan_.control.selector), target, following);
  }

  void finish() {
    for (unsigned bank = 0; bank < banks_.size(); ++bank) { put(banks_[bank], bank_fields[bank]); }
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (plan_.agu[index] || plan_.extents[0][index].accessed ||
          plan_.extents[1][index].accessed || plan_.definitions[index].has_value()) {
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
    for (unsigned offset = 0; offset < frames_.size(); ++offset) {
      if (frames_[offset] == nullptr) { continue; }
      auto *slot =
          ir_.CreateAnd(ir_.CreateAdd(sp_, ir_.getInt32(offset)), ir_.getInt32(stack_mask));
      ir_.CreateStore(frames_[offset], stack_address(slot));
    }
    auto *retired = ir_.CreateMul(turns_, ir_.getInt64(plan_.count));
    auto *cycles = ir_.CreateMul(turns_, ir_.getInt64(plan_.cost));
    if (plan_.setup) {
      retired = ir_.CreateAdd(retired, ir_.getInt64(1));
      cycles = ir_.CreateAdd(cycles, ir_.getInt64(plan_.setup->instruction.cycles));
    }
    store(ir_.CreateAdd(initial_instructions_, retired), offsetof(Cpu, instructions));
    store(ir_.CreateAdd(initial_cycles_, cycles), offsetof(Cpu, cycles));
    store(ir_.getInt8(0), offsetof(Cpu, repeat_completed));
    store(ir_.getInt8(0), offsetof(Cpu, interrupt_inhibited));
    auto *pc = next_pc();
    put(pc, offsetof(Registers, pc));
    if (plan_.setup) {
      const auto frame = [&](const unsigned advance, V high, V low) {
        auto *index =
            ir_.CreateAnd(ir_.CreateAdd(sp_, ir_.getInt32(advance)), ir_.getInt32(stack_mask));
        auto *value = ir_.CreateOr(
            ir_.CreateShl(ir_.CreateZExt(high, ir_.getInt64Ty()), ir_.getInt64(word_bits)),
            ir_.CreateZExt(low, ir_.getInt64Ty()));
        ir_.CreateStore(value, stack_address(index));
      };
      frame(1, reg(32, offsetof(Registers, la)), lc_);
      frame(2, ir_.getInt32(plan_.operations.front().pc), initial_status_);
      store(ir_.getInt8(static_cast<unsigned>(NativeBoundary::context)),
            offsetof(NativeContext, boundary),
            owner_);
      LLVMBuildRet(ir_.get(), ir_.CreateTrunc(retired, ir_.getInt32Ty()));
      return;
    }
    if (!at_loop_end(plan_.operations.back()) && endpoint_ == nullptr) {
      LLVMBuildRet(ir_.get(), ir_.CreateTrunc(retired, ir_.getInt32Ty()));
      return;
    }
    auto *loop_finish = LLVMAppendBasicBlockInContext(context_, function_, "loop_finish");
    auto *done = LLVMAppendBasicBlockInContext(context_, function_, "done");
    const auto &last = plan_.operations.back();
    auto *following = ir_.getInt32((last.pc + last.instruction.words) & word_mask);
    auto *closing = ir_.CreateICmpEQ(pc, following);
    if (endpoint_ != nullptr) { closing = ir_.CreateAnd(closing, endpoint_); }
    LLVMBuildCondBr(ir_.get(), closing, loop_finish, done);
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
    put(following, offsetof(Registers, pc));
    store(ir_.getInt8(static_cast<unsigned>(NativeBoundary::context)),
          offsetof(NativeContext, boundary),
          owner_);
    LLVMBuildBr(ir_.get(), done);
    ir_.SetInsertPoint(done);
    LLVMBuildRet(ir_.get(), ir_.CreateTrunc(retired, ir_.getInt32Ty()));
  }

  void bit_transfer(const NativeMove &move, const BitAction action, const Word opcode) {
    auto *pointer = memory(move, effective(cursors_, move.address));
    auto *original = ir_.CreateLoad(ir_.getInt32Ty(), pointer);
    const auto bit = opcode & five_bit_mask;
    auto *previous = ir_.CreateAnd(ir_.CreateLShr(original, ir_.getInt32(bit)), ir_.getInt32(1));
    status_ = ir_.CreateOr(ir_.CreateAnd(status_, ir_.getInt32(~carry)), previous);
    if (action == BitAction::test) { return; }
    auto *word = ir_.CreateAnd(original, ir_.getInt32(word_mask));
    auto *mask = ir_.getInt32(Word{1} << bit);
    V value = nullptr;
    switch (action) {
      case BitAction::clear: value = ir_.CreateAnd(word, ir_.CreateNot(mask)); break;
      case BitAction::set: value = ir_.CreateOr(word, mask); break;
      case BitAction::change: value = ir_.CreateXor(word, mask); break;
      case BitAction::test:
      default: throw std::logic_error("Invalid modifying region bit action");
    }
    ir_.CreateStore(value, pointer);
  }

  void body(LLVMBasicBlockRef miss) { body(miss, plan_.steps.size()); }

  struct Frame {
    V address, count;
    unsigned depth = 0;
  };

  void body(LLVMBasicBlockRef miss, const std::size_t count) {
    auto *remaining = plan_.setup ? ir_.getInt32(plan_.setup_count) : lc_;
    if (iteration_ != nullptr && (plan_.setup || captured_loop())) {
      remaining =
          ir_.CreateAnd(ir_.CreateSub(remaining, ir_.CreateTrunc(iteration_, ir_.getInt32Ty())),
                        ir_.getInt32(word_mask));
    }
    body(miss,
         0,
         count,
         {.address = plan_.setup ? ir_.getInt32(plan_.operations.back().loop_address)
                                 : reg(32, offsetof(Registers, la)),
          .count = remaining});
  }

  void body(LLVMBasicBlockRef miss,
            const std::size_t first,
            const std::size_t count,
            const Frame parent) {
    for (auto index = first; index < count; ++index) {
      const auto before = banks_;
      const auto &step = plan_.steps[index];
      if (step.repeat_end != 0) {
        repeat(miss, index, parent);
        index = step.repeat_end - 1;
        continue;
      }
      if (step.update) { static_cast<void>(effective(cursors_, *step.update)); }
      if (step.registers.valid()) {
        write_register(step.registers.destination, read_register(before, step.registers.source));
      }
      if (step.immediate.valid()) {
        write_register(step.immediate.destination, ir_.getInt32(step.immediate.value));
      }
      const auto &instruction = plan_.operations[index].instruction;
      if (instruction.operation == Operation::bit_memory) {
        bit_transfer(step.memory.transfers[0], bit_action(instruction.opcode), instruction.opcode);
      } else if (instruction.operation == Operation::arithmetic_shift_immediate) {
        llvm19::lower_arithmetic_shift(
            ir_, instruction.opcode, plan_.operations[index].mode, banks_, status_);
      } else if (!plan_.control.valid || index + 1 != plan_.steps.size()) {
        transfers(step.memory, instruction.opcode);
        llvm19::lower_alu(
            ir_, instruction.opcode, plan_.operations[index].mode, before, banks_, status_);
      }
      if (step.guards == 0) { continue; }
      memory_guards(step.guards, std::span(plan_.steps).subspan(index + 1));
      auto *ready = LLVMAppendBasicBlockInContext(context_, function_, "computed_address");
      LLVMBuildCondBr(ir_.get(), eligible_, ready, miss);
      ir_.SetInsertPoint(ready);
    }
  }

  struct Live {
    std::array<V, 4> banks;
    std::array<V, agu_registers> cursors, strides;
    V status, limited = nullptr, grown = nullptr;
    std::array<V, stack_entries> frames{};
  };

  [[nodiscard]] Live live() {
    Live state{.banks = banks_,
               .cursors = cursors_,
               .strides = strides_,
               .status = status_,
               .frames = frames_};
    if (split_flags_) {
      state.limited =
          ir_.CreateICmpNE(ir_.CreateAnd(status_, ir_.getInt32(limit)), ir_.getInt32(0));
      state.grown = ir_.CreateICmpNE(ir_.CreateAnd(status_, ir_.getInt32(growth)), ir_.getInt32(0));
      state.status = ir_.CreateAnd(status_, ir_.getInt32(~(limit | growth)));
    }
    return state;
  }

  void restore_live(const Live &state) {
    banks_ = state.banks;
    cursors_ = state.cursors;
    strides_ = state.strides;
    status_ = state.status;
    frames_ = state.frames;
    if (state.limited != nullptr) {
      status_ = ir_.CreateOr(
          status_,
          ir_.CreateOr(ir_.CreateSelect(state.limited, ir_.getInt32(limit), ir_.getInt32(0)),
                       ir_.CreateSelect(state.grown, ir_.getInt32(growth), ir_.getInt32(0))));
    }
  }

  Live live_phis(const Live &initial, LLVMBasicBlockRef predecessor) {
    const auto phi = [&](V value, const char *name) {
      auto *result = LLVMBuildPhi(ir_.get(), LLVMTypeOf(value), name);
      LLVMAddIncoming(result, &value, &predecessor, 1);
      return result;
    };
    Live result{};
    for (unsigned index = 0; index < result.banks.size(); ++index) {
      result.banks[index] = phi(initial.banks[index], "bank");
    }
    for (unsigned index = 0; index < agu_registers; ++index) {
      result.cursors[index] = phi(initial.cursors[index], "cursor");
      result.strides[index] = phi(initial.strides[index], "stride");
    }
    result.status = split_flags_ ? initial.status : phi(initial.status, "status");
    for (unsigned index = 0; index < result.frames.size(); ++index) {
      if (initial.frames[index] != nullptr) {
        result.frames[index] = phi(initial.frames[index], "frame");
      }
    }
    if (initial.limited != nullptr) {
      result.limited = phi(initial.limited, "limited");
      result.grown = phi(initial.grown, "grown");
    }
    return result;
  }

  static void live_backedge(const Live &phis, const Live &after, LLVMBasicBlockRef predecessor) {
    const auto incoming = [&](V phi, V value) { LLVMAddIncoming(phi, &value, &predecessor, 1); };
    for (unsigned index = 0; index < phis.banks.size(); ++index) {
      incoming(phis.banks[index], after.banks[index]);
    }
    for (unsigned index = 0; index < agu_registers; ++index) {
      incoming(phis.cursors[index], after.cursors[index]);
      incoming(phis.strides[index], after.strides[index]);
    }
    for (unsigned index = 0; index < phis.frames.size(); ++index) {
      if (phis.frames[index] != nullptr) { incoming(phis.frames[index], after.frames[index]); }
    }
    if (phis.limited != nullptr) {
      incoming(phis.limited, after.limited);
      incoming(phis.grown, after.grown);
    } else {
      incoming(phis.status, after.status);
    }
  }

  void repeat(LLVMBasicBlockRef miss, const std::size_t index, const Frame parent) {
    const auto &step = plan_.steps[index];
    const auto first = index + 1;
    const auto &last = plan_.operations[step.repeat_end - 1];
    const auto frame = [&](const unsigned offset, V high, V low) {
      const auto depth = 2U * (parent.depth + (plan_.setup ? 1U : 0U));
      auto *value = ir_.CreateOr(
          ir_.CreateShl(ir_.CreateZExt(high, ir_.getInt64Ty()), ir_.getInt64(word_bits)),
          ir_.CreateZExt(low, ir_.getInt64Ty()));
      frames_[depth + offset] = value;
    };
    auto *status = status_;
    if (parent.depth != 0 || plan_.setup) {
      status =
          ir_.CreateOr(ir_.CreateAnd(status, ir_.getInt32(~forever_flag)), ir_.getInt32(loop_flag));
    }
    frame(1, parent.address, parent.count);
    frame(2, ir_.getInt32(plan_.operations[first].pc), status);
    auto leaf = plan_;
    leaf.operations = plan_.operations.subspan(first, step.repeat_end - first);
    leaf.count = static_cast<Word>(leaf.operations.size());
    leaf.batch = true;
    const bool split = split_flags(leaf);
    repeated_body(ir_.getInt64(step.repetitions), split, leaf.count, [&](V iteration) {
      body(miss,
           first,
           step.repeat_end,
           {.address = ir_.getInt32(last.pc + last.instruction.words - 1),
            .count = ir_.CreateSub(ir_.getInt32(step.repetitions),
                                   ir_.CreateTrunc(iteration, ir_.getInt32Ty())),
            .depth = parent.depth + 1});
    });
    if (split && leaf.count == 1 && addressing_ == BatchAddressing::linear) {
      for (const auto &move :
           std::span(plan_.steps[first].memory.transfers).first(plan_.steps[first].memory.count)) {
        const auto form = move.address;
        if (form.mode == 4 || form.mode == 5 || form.mode == 6) { continue; }
        cursors_[form.index] = ir_.CreateAnd(cursors_[form.index], ir_.getInt32(word_mask));
      }
    }
  }

  template <class Body> void repeated_body(V turns, const bool split, Word count, Body emit) {
    const auto prior_split = split_flags_;
    const auto prior_count = loop_count_;
    split_flags_ = split;
    loop_count_ = count;
    const auto initial = live();
    auto *start = LLVMGetInsertBlock(ir_.get());
    auto *loop = LLVMAppendBasicBlockInContext(context_, function_, "repeat");
    auto *done = LLVMAppendBasicBlockInContext(context_, function_, "repeat_done");
    auto *iterations = split_flags_ ? ir_.CreateSub(turns, ir_.getInt64(1)) : turns;
    if (split_flags_) {
      LLVMBuildCondBr(ir_.get(), ir_.CreateICmpEQ(iterations, ir_.getInt64(0)), done, loop);
    } else {
      LLVMBuildBr(ir_.get(), loop);
    }
    ir_.SetInsertPoint(loop);
    const auto phis = live_phis(initial, start);
    auto *remaining = LLVMBuildPhi(ir_.get(), ir_.getInt64Ty(), "repeat_remaining");
    LLVMAddIncoming(remaining, &iterations, &start, 1);
    restore_live(phis);
    emit(ir_.CreateSub(iterations, remaining));
    const auto after = live();
    auto *backedge = LLVMGetInsertBlock(ir_.get());
    live_backedge(phis, after, backedge);
    auto *left = ir_.CreateSub(remaining, ir_.getInt64(1));
    LLVMAddIncoming(remaining, &left, &backedge, 1);
    unroll_short_loop(
        LLVMBuildCondBr(ir_.get(), ir_.CreateICmpEQ(left, ir_.getInt64(0)), done, loop), count);
    ir_.SetInsertPoint(done);
    if (split_flags_) {
      const auto final = live_phis(initial, start);
      live_backedge(final, after, backedge);
      restore_live(final);
      emit(ir_.CreateSub(turns, ir_.getInt64(1)));
    } else {
      restore_live(after);
    }
    split_flags_ = prior_split;
    loop_count_ = prior_count;
  }

  void self_loop_bindings() {
    for (unsigned bank = 0; bank < bindings_.size(); ++bank) {
      for (const bool writing : {false, true}) {
        if (!std::ranges::any_of(plan_.extents[bank],
                                 [writing](const auto &extent) {
                                   return extent.accessed && extent.writing == writing;
                                 }) &&
            !std::ranges::any_of(plan_.absolute[bank], [writing](const auto &entry) {
              return entry.second == writing;
            })) {
          continue;
        }
        for (const auto index : window_order(bank, writing)) {
          static_cast<void>(binding(bank, index, writing));
        }
      }
    }
  }

  void self_loop_finish(const Live &state, V completed) {
    restore_live(state);
    turns_ = completed;
    finish();
  }

  void generate_self_loop() {
    auto *start = LLVMAppendBasicBlockInContext(context_, function_, "self_loop_start");
    auto *loop = LLVMAppendBasicBlockInContext(context_, function_, "self_loop_loop");
    auto *step = LLVMAppendBasicBlockInContext(context_, function_, "self_loop_body");
    auto *guard_miss = LLVMAppendBasicBlockInContext(context_, function_, "self_loop_guard_miss");
    auto *prior = LLVMAppendBasicBlockInContext(context_, function_, "self_loop_prior");
    auto *complete = LLVMAppendBasicBlockInContext(context_, function_, "self_loop_complete");
    auto *miss = LLVMAppendBasicBlockInContext(context_, function_, "ordinary");
    admission();
    auto *limit = turns_;
    LLVMBuildCondBr(ir_.get(), eligible_, start, miss);
    ir_.SetInsertPoint(start);
    self_loop_bindings();
    for (unsigned bank = 0; bank < banks_.size(); ++bank) {
      banks_[bank] = reg(64, bank_fields[bank]);
    }
    const auto initial = live();
    LLVMBuildBr(ir_.get(), loop);
    ir_.SetInsertPoint(loop);
    const auto phis = live_phis(initial, start);
    restore_live(phis);
    auto *completed = LLVMBuildPhi(ir_.get(), ir_.getInt64Ty(), "completed");
    auto *zero = ir_.getInt64(0);
    LLVMAddIncoming(completed, &zero, &start, 1);
    for (unsigned index = 0; index < agu_registers; ++index) {
      if (plan_.agu[index]) {
        origins_[index] = ir_.CreateAnd(cursors_[index], ir_.CreateNot(masks_[index]));
      }
    }
    eligible_ = boolean(true);
    initial_memory_guards();
    LLVMBuildCondBr(ir_.get(), eligible_, step, guard_miss);
    ir_.SetInsertPoint(step);
    iteration_ = completed;
    body(guard_miss);
    const auto after = live();
    auto *next = ir_.CreateAdd(completed, ir_.getInt64(1));
    auto *again = ir_.CreateICmpULT(next, limit);
    if (!captured_loop()) {
      again = ir_.CreateAnd(again,
                            ir_.CreateICmpEQ(next_pc(), ir_.getInt32(plan_.operations.front().pc)));
    }
    auto *backedge = LLVMGetInsertBlock(ir_.get());
    live_backedge(phis, after, backedge);
    LLVMAddIncoming(completed, &next, &backedge, 1);
    LLVMBuildCondBr(ir_.get(), again, loop, complete);
    ir_.SetInsertPoint(complete);
    self_loop_finish(after, next);
    ir_.SetInsertPoint(guard_miss);
    LLVMBuildCondBr(ir_.get(), ir_.CreateICmpNE(completed, ir_.getInt64(0)), prior, miss);
    ir_.SetInsertPoint(prior);
    self_loop_finish(phis, completed);
    ir_.SetInsertPoint(miss);
    LLVMBuildRet(ir_.get(), ir_.getInt32(0));
  }

  void generate_admission(const std::size_t prefix_steps) {
    auto *windows = LLVMAppendBasicBlockInContext(context_, function_, "windows");
    auto *start = LLVMAppendBasicBlockInContext(context_, function_, "admitted");
    auto *miss = LLVMAppendBasicBlockInContext(context_, function_, "refused");
    admission();
    LLVMBuildCondBr(ir_.get(), eligible_, windows, miss);
    ir_.SetInsertPoint(windows);
    initial_memory_guards();
    LLVMBuildCondBr(ir_.get(), eligible_, start, miss);
    ir_.SetInsertPoint(start);
    if (prefix_steps != 0) {
      for (unsigned bank = 0; bank < banks_.size(); ++bank) {
        banks_[bank] = reg(64, bank_fields[bank]);
      }
      body(miss, prefix_steps);
    }
    auto *cycles = ir_.CreateMul(turns_, ir_.getInt64(plan_.cost));
    if (plan_.setup) {
      cycles = ir_.CreateAdd(cycles, ir_.getInt64(plan_.setup->instruction.cycles));
    }
    LLVMBuildRet(ir_.get(), cycles);
    ir_.SetInsertPoint(miss);
    LLVMBuildRet(ir_.get(), ir_.getInt64(0));
  }

  void unroll_short_loop(V branch, const Word count) {
    if (!split_flags_ || count != 1) { return; }
    auto &context = *llvm::unwrap(context_);
    auto *option = llvm::MDNode::get(
        context,
        {llvm::MDString::get(context, "llvm.loop.unroll.count"),
         llvm::ConstantAsMetadata::get(llvm::unwrap<llvm::Constant>(ir_.getInt32(4)))});
    auto *loop = llvm::MDNode::getDistinct(context, {nullptr, option});
    loop->replaceOperandWith(0, loop);
    llvm::unwrap<llvm::Instruction>(branch)->setMetadata(llvm::LLVMContext::MD_loop, loop);
  }

  void generate(const void *fallback) {
    auto *windows = LLVMAppendBasicBlockInContext(context_, function_, "windows");
    auto *start = LLVMAppendBasicBlockInContext(context_, function_, "admitted");
    auto *miss = LLVMAppendBasicBlockInContext(context_, function_, "ordinary");
    admission();
    LLVMBuildCondBr(ir_.get(), eligible_, windows, miss);
    ir_.SetInsertPoint(windows);
    initial_memory_guards();
    LLVMBuildCondBr(ir_.get(), eligible_, start, miss);
    ir_.SetInsertPoint(miss);
    if (fallback == nullptr) {
      LLVMBuildRet(ir_.get(), ir_.getInt32(0));
    } else {
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
    }
    ir_.SetInsertPoint(start);
    for (unsigned bank = 0; bank < banks_.size(); ++bank) {
      banks_[bank] = reg(64, bank_fields[bank]);
    }
    repeated_body(turns_, split_flags(plan_), plan_.count, [&](V iteration) {
      iteration_ = iteration;
      body(miss);
    });
    finish();
  }

  LLVMContextRef context_;
  LLVMValueRef function_;
  llvm19::Builder ir_;
  const RegionPlan &plan_;
  BatchAddressing addressing_;
  bool dynamic_, self_loop_, split_flags_ = false;
  Word loop_count_ = 0;
  V cpu_, owner_, initial_cycles_ = nullptr, initial_instructions_ = nullptr;
  V status_ = nullptr, initial_status_ = nullptr, lc_ = nullptr, sp_ = nullptr, sc_ = nullptr,
    saved_ = nullptr;
  V turns_ = nullptr, forever_ = nullptr, finite_count_ = nullptr, eligible_ = nullptr;
  V endpoint_ = nullptr;
  V iteration_ = nullptr;
  std::array<V, stack_entries> frames_{};
  std::array<V, 4> banks_{};
  std::array<V, agu_registers> cursors_{}, strides_{}, masks_{}, origins_{};
  std::array<std::array<V, agu_registers>, 2> memory_bases_{};
  std::array<std::map<Word, V>, 2> absolute_bases_;
  std::array<std::vector<std::array<Binding, 2>>, 2> bindings_;
};
}

bool region_independent(const std::span<const BlockInstruction> operations) {
  if (operations.empty()) { return false; }
  const auto &last = operations.back();
  const auto final_word = (last.pc + last.instruction.words - 1) & word_mask;
  if (at_loop_end(last) || operations.front().pc > final_word) { return false; }
  const auto kind = last.instruction.operation;
  return kind == Operation::jump_short || kind == Operation::jump_effective ||
         kind == Operation::conditional_jump || kind == Operation::relative_branch;
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

bool lift_region(LlvmModule &module,
                 const std::span<const BlockInstruction> operations,
                 const JitRuntime &runtime) {
  if (module.code() == nullptr) { throw std::logic_error("Region module was already transferred"); }
  if (LLVMGetNamedFunction(module.code(), module.name().c_str()) != nullptr) {
    throw std::invalid_argument("Region module already has its entry");
  }
  const auto plan = plan_region(operations, runtime);
  if (!plan) { return false; }
  const RegionLifter lift(module, *plan, nullptr);
  return true;
}

bool lift_region(LlvmModule &module,
                 const std::span<const BlockInstruction> operations,
                 const std::span<const RegionLayout> memory,
                 const std::string_view entry,
                 Word *properties) {
  if (module.code() == nullptr) { throw std::logic_error("Region module was already transferred"); }
  const auto name = entry.empty() ? module.name() : std::string(entry);
  if (LLVMGetNamedFunction(module.code(), name.c_str()) != nullptr) {
    throw std::invalid_argument("Region module already has its entry");
  }
  const auto plan = plan_region(operations, memory);
  if (!plan) { return false; }
  if (properties != nullptr) {
    *properties = (plan->batch ? 1U : 0U) | (split_flags(*plan) ? 2U : 0U);
  }
  const RegionLifter lift(module, *plan, nullptr, name);
  return true;
}

bool lift_region_admission(LlvmModule &module,
                           const std::span<const BlockInstruction> operations,
                           const std::span<const RegionLayout> memory,
                           const std::string_view entry) {
  if (module.code() == nullptr) { throw std::logic_error("Region module was already transferred"); }
  const auto name = entry.empty() ? module.name() : std::string(entry);
  if (LLVMGetNamedFunction(module.code(), name.c_str()) != nullptr) {
    throw std::invalid_argument("Region module already has its entry");
  }
  const auto plan = plan_region(operations, memory);
  if (!plan) { return false; }
  const auto prefix = admission_prefix(*plan);
  if (!prefix) { return false; }
  const RegionLifter generated(module, *plan, nullptr, name, true, *prefix);
  return true;
}
}
