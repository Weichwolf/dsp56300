// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/image.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/bindings.hpp"
#include "jit/llvm/image_regions.hpp"
#include "jit/llvm/origins.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/region_plan.hpp"
#include "jit/llvm/runtime.hpp"
#include "jit/pair_read_loop.hpp"
#include "semantic.hpp"
#include "semantic_ir.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <llvm/ADT/StringExtras.h>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/InstrTypes.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Alignment.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr unsigned flow_shift = 32, page_shift = 12;
constexpr Word page_words = Word{1} << page_shift, page_mask = page_words - 1;
constexpr std::uint32_t hot_edge_weight = 2000;
using Value = llvm::Value;
using Block = llvm::BasicBlock;

void cold_edges(llvm::Function &function, const std::set<Block *> &cold) {
  llvm::MDBuilder metadata(function.getContext());
  for (auto &block : function) {
    auto *terminator = block.getTerminator();
    if ((!llvm::isa<llvm::BranchInst>(terminator) && !llvm::isa<llvm::SwitchInst>(terminator)) ||
        terminator->getNumSuccessors() < 2) {
      continue;
    }
    std::vector<std::uint32_t> weights;
    weights.reserve(terminator->getNumSuccessors());
    for (unsigned index = 0; index < terminator->getNumSuccessors(); ++index) {
      weights.push_back(cold.contains(terminator->getSuccessor(index)) ? 1U : hot_edge_weight);
    }
    if (std::ranges::find(weights, 1U) != weights.end()) {
      terminator->setMetadata(llvm::LLVMContext::MD_prof, metadata.createBranchWeights(weights));
    }
  }
}

Word memory_flags(const RegionLayout &region) {
  return (region.writable ? memory_writable : 0U) | (region.tracked ? memory_tracked : 0U) |
         (region.aliases_program ? memory_program_alias : 0U);
}

llvm::Function *semantic_function(llvm::Module &module, const bool vector) {
  const auto *abi = module.getGlobalVariable("dsp56300_semantic_abi");
  if (abi == nullptr || !abi->isConstant() || !abi->hasInitializer()) {
    throw std::invalid_argument("DSP image requires semantic ABI metadata");
  }
  const llvm::Constant *data = abi->getInitializer();
  if (data->getType()->isStructTy() && data->getNumOperands() == 1) {
    data = llvm::cast<llvm::Constant>(data->getOperand(0));
  }
  if (!data->getType()->isArrayTy() ||
      data->getType()->getArrayNumElements() != semantic_abi_fields ||
      !data->getType()->getArrayElementType()->isIntegerTy(32)) {
    throw std::invalid_argument("DSP image semantic ABI metadata has an incompatible shape");
  }
  for (unsigned index = 0; index < semantic_abi_fields; ++index) {
    const auto *field = llvm::dyn_cast_or_null<llvm::ConstantInt>(data->getAggregateElement(index));
    if (field == nullptr || field->getZExtValue() != dsp56300_semantic_abi[index]) {
      throw std::invalid_argument("DSP image semantic ABI differs from this compiler");
    }
  }
  auto *function =
      module.getFunction(vector ? "dsp56300_semantic_vector_execute" : "dsp56300_semantic_execute");
  if (function == nullptr || function->isDeclaration()) {
    throw std::invalid_argument("DSP image requires its complete semantic implementation");
  }
  auto *type = function->getFunctionType();
  const unsigned last = vector ? 1 : 2;
  if (!type->getReturnType()->isIntegerTy(64) || type->isVarArg() ||
      type->getNumParams() != last + 1 || !type->getParamType(0)->isPointerTy() ||
      !type->getParamType(last)->isPointerTy() ||
      (!vector && !type->getParamType(1)->isIntegerTy(32)) ||
      type->getParamType(0)->getPointerAddressSpace() != 0 ||
      type->getParamType(last)->getPointerAddressSpace() != 0) {
    throw std::invalid_argument("DSP image semantic function has an incompatible ABI");
  }
  return function;
}

class ImageLifter {
public:
  ImageLifter(LlvmModule &module,
              const ImagePlan &plan,
              const std::string_view entry,
              const bool vector = false,
              const std::optional<std::span<const RegionLayout>> memory = std::nullopt,
              const std::optional<std::span<const Word>> region_modes = std::nullopt,
              const std::uint64_t variant = 0,
              const bool direct = false,
              const bool paired = false)
      : module_(*llvm::unwrap(module.code())),
        locations_(module_, variant),
        ir_(module_.getContext()),
        name_(entry.empty() ? module.name() : std::string(entry)),
        plan_(plan),
        edges_(plan.edges()),
        semantic_(semantic_function(module_, vector)),
        memory_(memory),
        variant_(variant),
        direct_(direct) {
    if (module_.getNamedValue(name_) != nullptr) {
      throw std::invalid_argument("DSP image entry already exists");
    }
    constexpr std::array complete{RegionLayout{.space = Space::x, .first = 0, .size = io_begin},
                                  RegionLayout{.space = Space::y, .first = 0, .size = io_begin}};
    if (!vector && direct_) {
      prepare_regions(module, complete, region_modes);
    } else if (!vector && memory_) {
      prepare_regions(module, *memory_, region_modes);
    }
    if (paired) { prepare_paired_reads(); }
    for (const auto head : plan_.units()) {
      units_.emplace(head, function(name_ + ".unit." + llvm::utohexstr(head, true), true));
    }
    for (const auto &[pc, instruction] : plan_.instructions) {
      static_cast<void>(instruction);
      auto found = units_.upper_bound(pc);
      --found;
      targets_.emplace(pc, found->second);
    }
    dispatch_ = function(name_ + ".dispatch", true);
    dispatch_->addFnAttr(llvm::Attribute::AlwaysInline);
    table_ = table();
    emit_dispatch();
    for (const auto &unit : units_) { emit_unit(unit.second, unit.first); }
    emit_entry();
  }

private:
  struct FastRegion {
    llvm::GlobalVariable *site;
    Word last_pc, last_words, mode, properties;
    std::vector<Word> resume;
    Word minimum_instructions = 0, minimum_cycles = 0;
  };

  enum class RegionForm : std::uint8_t { ordinary, repeat, sequence };

  static std::vector<std::vector<BlockInstruction>>
  sequential_regions(const std::vector<std::vector<BlockInstruction>> &ordinary) {
    constexpr Word no_loop = word_mask + 1U;
    constexpr std::size_t maximum_operations = 24;
    std::map<Word, const std::vector<BlockInstruction> *> entries;
    for (const auto &operations : ordinary) {
      if (operations.back().loop_address == no_loop) {
        entries.emplace(operations.front().pc, &operations);
      }
    }
    std::vector<std::vector<BlockInstruction>> result;
    for (const auto &entry : entries) {
      const auto *prefix = entry.second;
      const auto &tail = prefix->back();
      const auto next = tail.pc + tail.instruction.words;
      const auto suffix = entries.find(next);
      if (suffix == entries.end() || prefix->size() + suffix->second->size() > maximum_operations) {
        continue;
      }
      auto sequence = *prefix;
      sequence.insert(sequence.end(), suffix->second->begin(), suffix->second->end());
      result.push_back(std::move(sequence));
    }
    return result;
  }

  void prepare_regions(LlvmModule &module,
                       const std::span<const RegionLayout> memory,
                       const std::optional<std::span<const Word>> selected) {
    constexpr std::array modes{0U,
                               rounding_mode,
                               scale_down,
                               scale_up,
                               rounding_mode | scale_down,
                               rounding_mode | scale_up};
    const auto add = [&](auto operations, const RegionForm form) {
      const bool linear_only = form == RegionForm::repeat;
      constexpr std::array suffixes{"", ".linear_edge", ".sequence"};
      const auto *suffix = suffixes[static_cast<std::size_t>(form)];
      Word minimum_instructions = 0;
      Word minimum_cycles = 0;
      if (form == RegionForm::sequence) {
        const auto planned = plan_region(operations, memory);
        if (!planned) { return; }
        minimum_instructions = planned->count;
        minimum_cycles = planned->cost;
      }
      for (const auto mode : modes) {
        if (selected && std::ranges::find(*selected, mode) == selected->end()) { continue; }
        for (auto &operation : operations) { operation.mode = mode; }
        const auto first = operations.front().pc;
        const auto &last = operations.back();
        const auto name = name_ + ".region." + llvm::utohexstr(first, true) + "." +
                          llvm::utohexstr(last.loop_address, true) + "." +
                          llvm::utohexstr(mode, true) + suffix;
        Word properties = 0;
        std::vector<Word> resume;
        if (!lift_region(module,
                         operations,
                         memory,
                         name,
                         &properties,
                         variant_,
                         direct_,
                         linear_only,
                         &resume,
                         /*admitted_site=*/true)) {
          continue;
        }
        auto *function = module_.getFunction(name);
        function->setLinkage(llvm::GlobalValue::InternalLinkage);
        function->addFnAttr(llvm::Attribute::AlwaysInline);
        const std::array<llvm::Constant *, 2> fields{function, ir_.getInt32(last.pc)};
        auto *site =
            constant(name + ".site", llvm::ConstantStruct::getAnon(module_.getContext(), fields));
        site->setAlignment(llvm::Align(alignof(SemanticRegion)));
        regions_.emplace(first,
                         FastRegion{.site = site,
                                    .last_pc = last.pc,
                                    .last_words = last.instruction.words,
                                    .mode = mode,
                                    .properties = properties,
                                    .resume = std::move(resume),
                                    .minimum_instructions = minimum_instructions,
                                    .minimum_cycles = minimum_cycles});
      }
    };
    if (direct_) {
      for (auto &operations : image_composed_regions(plan_, memory)) {
        add(std::move(operations), RegionForm::repeat);
      }
    }
    auto ordinary = image_regions(plan_, memory);
    if (direct_) {
      for (auto &operations : sequential_regions(ordinary)) {
        add(std::move(operations), RegionForm::sequence);
      }
    }
    for (auto &operations : ordinary) { add(std::move(operations), RegionForm::ordinary); }
  }

  void commit_region(Value *outcome, const FastRegion &region, const Word pc) {
    auto *count = ir_.CreateLoad(ir_.getInt32Ty(), count_);
    auto *window = ir_.CreateLoad(ir_.getInt64Ty(), window_);
    auto *retired = ir_.CreateTrunc(outcome, ir_.getInt32Ty());
    auto *next = ir_.CreateTrunc(ir_.CreateLShr(outcome, flow_shift), ir_.getInt32Ty());
    auto *cost = ir_.CreateLoad(ir_.getInt64Ty(), region_cost_);
    ir_.CreateStore(ir_.CreateAdd(count, retired), count_);
    ir_.CreateStore(ir_.CreateSub(window, cost), window_);
    ir_.CreateStore(next, pc_);
    const auto site = pc | (std::uint64_t{region.properties} << word_bits);
    retirement(retired, true, site);
    const auto end = (region.last_pc + region.last_words) & word_mask;
    std::set<Word> targets{pc, end};
    targets.insert(region.resume.begin(), region.resume.end());
    for (const auto target : edges_.at(region.last_pc)) {
      if (targets_.contains(target)) { targets.insert(target); }
    }
    auto *transfer = ir_.CreateSwitch(
        next, exits_.at(ImageExit::unprepared), static_cast<unsigned>(targets.size()));
    for (const auto target : targets) {
      transfer->addCase(ir_.getInt32(target), successor(target));
    }
  }

  void prepare_paired_reads() {
    constexpr Word setup_words = 2;
    for (const auto &[pc, instruction] : plan_.instructions) {
      if (instruction.operation != Operation::loop_immediate || instruction.words != setup_words ||
          pc > word_mask - setup_words) {
        continue;
      }
      const auto body = pc + setup_words;
      if (const auto loop = paired_port_read(plan_, body)) { prepare_paired_read(body, *loop); }
    }
  }

  void prepare_paired_read(const Word pc, const PairedPortRead &loop) {
    const auto &edges = loop.edges;
    const std::array<Word, sizeof(SemanticPairedPortRead) / sizeof(Word)> words{
        pc,
        loop.end,
        loop.destination_register,
        loop.retired,
        static_cast<Word>(loop.data.space),
        loop.data.address,
        static_cast<Word>(loop.strobe.space),
        loop.strobe.address,
        loop.bit,
        loop.cycles,
        loop.reads[0],
        loop.reads[1],
        edges[0],
        edges[1],
        edges[2],
        edges[3]};
    auto *data = llvm::ConstantDataArray::get(module_.getContext(), words);
    auto *contract = new llvm::GlobalVariable(module_,
                                              data->getType(),
                                              true,
                                              llvm::GlobalValue::PrivateLinkage,
                                              data,
                                              name_ + ".paired_read." + llvm::utohexstr(pc, true));
    contract->setAlignment(llvm::Align(alignof(SemanticPairedPortRead)));
    paired_reads_.emplace(pc, contract);
  }

  void emit_paired_read(const Word pc, Value *count, Value *window) {
    const auto selected = paired_reads_.find(pc);
    if (selected == paired_reads_.end()) { return; }
    auto *adapter = module_.getFunction("dsp56300_semantic_paired_read");
    if (adapter == nullptr || adapter->isDeclaration() || adapter->arg_size() != 5 ||
        !adapter->getReturnType()->isIntegerTy(64)) {
      throw std::invalid_argument("DSP image lacks the paired I/O ABI");
    }
    auto *commit = block(active_, "paired.complete");
    auto *precise = block(active_, "paired.precise");
    auto *result = ir_.CreateCall(adapter,
                                  {active_->getArg(0),
                                   ir_.CreateSub(active_->getArg(2), count),
                                   selected->second,
                                   window,
                                   region_cost_});
    auto *retired = ir_.CreateTrunc(result, ir_.getInt32Ty());
    ir_.CreateCondBr(ir_.CreateIsNotNull(retired), commit, precise);
    ir_.SetInsertPoint(commit);
    auto *cost = ir_.CreateLoad(ir_.getInt64Ty(), region_cost_);
    ir_.CreateStore(ir_.CreateAdd(count, retired), count_);
    ir_.CreateStore(ir_.CreateSub(window, cost), window_);
    ir_.CreateStore(ir_.getInt32(pc), pc_);
    auto *yield = ir_.CreateTrunc(ir_.CreateLShr(result, flow_shift), ir_.getInt1Ty());
    ir_.CreateCondBr(yield, exits_.at(ImageExit::owner), successor(pc));
    ir_.SetInsertPoint(precise);
  }

  struct RegionGrant {
    Value *count;
    Value *window;
  };

  void emit_regions(const Word pc, RegionGrant grant) {
    const auto [first, last] = regions_.equal_range(pc);
    if (first == last) { return; }
    auto *adapter = module_.getFunction("dsp56300_semantic_region");
    if (adapter == nullptr || adapter->isDeclaration() || adapter->arg_size() != 5 ||
        !adapter->getReturnType()->isIntegerTy(64)) {
      throw std::invalid_argument("DSP image lacks its private region admission ABI");
    }
    auto *mode = module_.getFunction("dsp56300_semantic_numeric_mode");
    if (mode == nullptr || mode->isDeclaration() || mode->arg_size() != 1 ||
        !mode->getReturnType()->isIntegerTy(32)) {
      throw std::invalid_argument("DSP image lacks its private numeric-mode ABI");
    }
    std::map<Word, std::vector<const FastRegion *>> candidates;
    for (auto found = first; found != last; ++found) {
      candidates[found->second.mode].push_back(&found->second);
    }
    auto *precise = block(active_, "region.precise");
    cold_.insert(precise);
    auto *selection = ir_.CreateSwitch(ir_.CreateCall(mode, {active_->getArg(0)}),
                                       precise,
                                       static_cast<unsigned>(candidates.size()));
    for (const auto &[numeric, regions] : candidates) {
      auto *selected = block(active_, "region.mode");
      selection->addCase(ir_.getInt32(numeric), selected);
      ir_.SetInsertPoint(selected);
      for (const auto *region : regions) {
        auto *continued = block(active_, "region.complete");
        auto *miss = region == regions.back() ? precise : block(active_, "region.next");
        cold_.insert(miss);
        auto *budget = ir_.CreateSub(active_->getArg(2), grant.count);
        if (region->minimum_cycles != 0) {
          auto *ready = block(active_, "region.grant");
          auto *fits =
              ir_.CreateAnd(ir_.CreateICmpUGE(budget, ir_.getInt32(region->minimum_instructions)),
                            ir_.CreateICmpUGE(grant.window, ir_.getInt64(region->minimum_cycles)));
          ir_.CreateCondBr(fits, ready, miss);
          ir_.SetInsertPoint(ready);
        }
        auto *outcome = ir_.CreateCall(
            adapter, {active_->getArg(0), budget, region->site, grant.window, region_cost_});
        ir_.CreateCondBr(
            ir_.CreateICmpNE(ir_.CreateTrunc(outcome, ir_.getInt32Ty()), ir_.getInt32(0)),
            continued,
            miss);
        ir_.SetInsertPoint(continued);
        commit_region(outcome, *region, pc);
        ir_.SetInsertPoint(miss);
      }
    }
    ir_.SetInsertPoint(precise);
  }

  llvm::Function *function(const std::string &name, const bool internal) {
    auto *pointer = ir_.getPtrTy();
    std::vector<llvm::Type *> parameters{
        pointer, ir_.getInt32Ty(), ir_.getInt32Ty(), ir_.getInt64Ty()};
    if (internal) { parameters.push_back(ir_.getInt32Ty()); }
    auto *type = llvm::FunctionType::get(ir_.getInt64Ty(), parameters, false);
    auto *result = llvm::Function::Create(type,
                                          internal ? llvm::GlobalValue::InternalLinkage
                                                   : llvm::GlobalValue::ExternalLinkage,
                                          name,
                                          module_);
    attributes(result);
    locations_.function(*result);
    if (internal) { result->setCallingConv(llvm::CallingConv::PreserveNone); }
    return result;
  }

  void attributes(llvm::Function *result) {
    result->addFnAttr("dsp56300-image-part", "1");
    result->addFnAttr(llvm::Attribute::NoUnwind);
    result->addFnAttr(llvm::Attribute::StackProtectStrong);
    for (const auto *const key : {"target-cpu", "target-features", "frame-pointer"}) {
      if (semantic_->hasFnAttribute(key)) { result->addFnAttr(semantic_->getFnAttribute(key)); }
    }
  }

  Block *block(llvm::Function *function, const std::string &name) {
    return Block::Create(module_.getContext(), name, function);
  }

  llvm::GlobalVariable *constant(const std::string &name, llvm::Constant *value) {
    if (module_.getNamedValue(name) != nullptr) {
      throw std::invalid_argument("DSP image constant name already exists");
    }
    auto *global =
        llvm::cast<llvm::GlobalVariable>(module_.getOrInsertGlobal(name, value->getType()));
    global->setConstant(true);
    global->setLinkage(llvm::GlobalValue::PrivateLinkage);
    global->setInitializer(value);
    global->setUnnamedAddr(llvm::GlobalValue::UnnamedAddr::Global);
    return global;
  }

  llvm::GlobalVariable *table() {
    const auto last = plan_.instructions.rbegin()->first;
    const auto pages = (last >> page_shift) + 1;
    auto *pointer = ir_.getPtrTy();
    auto *empty = llvm::ConstantPointerNull::get(pointer);
    std::vector<llvm::Constant *> top(pages, empty);
    std::map<Word, std::vector<llvm::Constant *>> contents;
    for (const auto &[pc, target] : targets_) {
      auto [found, added] = contents.try_emplace(pc >> page_shift);
      if (added) {
        const auto length = (pc >> page_shift) + 1 == pages ? (last & page_mask) + 1 : page_words;
        found->second.assign(length, empty);
      }
      found->second[pc & page_mask] = target;
    }
    for (const auto &[page, targets] : contents) {
      auto *array =
          llvm::ConstantArray::get(llvm::ArrayType::get(pointer, targets.size()), targets);
      top[page] = constant(name_ + ".targets." + llvm::utohexstr(page, true), array);
    }
    return constant(name_ + ".pages",
                    llvm::ConstantArray::get(llvm::ArrayType::get(pointer, pages), top));
  }

  void tail(llvm::Function *function,
            Value *owner,
            Value *pc,
            Value *grant,
            Value *window,
            Value *count) {
    auto *call = ir_.CreateCall(function, {owner, pc, grant, window, count});
    call->setCallingConv(llvm::CallingConv::PreserveNone);
    call->setTailCallKind(llvm::CallInst::TCK_MustTail);
    ir_.CreateRet(call);
  }

  void result(Value *count, const ImageExit reason) {
    ir_.CreateRet(ir_.CreateOr(ir_.CreateZExt(count, ir_.getInt64Ty()),
                               ir_.getInt64(static_cast<std::uint64_t>(reason) << flow_shift)));
  }

  void emit_dispatch() {
    ir_.SetCurrentDebugLocation(Origins::generated(*dispatch_));
    auto *entry = block(dispatch_, "entry");
    auto *time = block(dispatch_, "time");
    auto *range = block(dispatch_, "range");
    auto *page = block(dispatch_, "page");
    auto *slot = block(dispatch_, "slot");
    auto *execute = block(dispatch_, "execute");
    auto *budget = block(dispatch_, "budget");
    auto *deadline = block(dispatch_, "deadline");
    auto *missing = block(dispatch_, "unprepared");
    auto *pc = dispatch_->getArg(1);
    auto *count = dispatch_->getArg(4);
    ir_.SetInsertPoint(entry);
    ir_.CreateCondBr(ir_.CreateICmpULT(count, dispatch_->getArg(2)), time, budget);
    ir_.SetInsertPoint(time);
    ir_.CreateCondBr(ir_.CreateICmpNE(dispatch_->getArg(3), ir_.getInt64(0)), range, deadline);
    ir_.SetInsertPoint(range);
    ir_.CreateCondBr(
        ir_.CreateICmpULE(pc, ir_.getInt32(plan_.instructions.rbegin()->first)), page, missing);
    ir_.SetInsertPoint(page);
    auto *page_slot = ir_.CreateInBoundsGEP(
        table_->getValueType(), table_, {ir_.getInt32(0), ir_.CreateLShr(pc, page_shift)});
    auto *pointer = ir_.CreateLoad(ir_.getPtrTy(), page_slot);
    auto *empty = llvm::ConstantPointerNull::get(ir_.getPtrTy());
    ir_.CreateCondBr(ir_.CreateICmpNE(pointer, empty), slot, missing);
    ir_.SetInsertPoint(slot);
    auto *target_slot =
        ir_.CreateInBoundsGEP(ir_.getPtrTy(), pointer, ir_.CreateAnd(pc, ir_.getInt32(page_mask)));
    auto *target = ir_.CreateLoad(ir_.getPtrTy(), target_slot);
    ir_.CreateCondBr(ir_.CreateICmpNE(target, empty), execute, missing);
    ir_.SetInsertPoint(execute);
    auto *call = ir_.CreateCall(
        dispatch_->getFunctionType(),
        target,
        {dispatch_->getArg(0), pc, dispatch_->getArg(2), dispatch_->getArg(3), count});
    call->setCallingConv(llvm::CallingConv::PreserveNone);
    call->setTailCallKind(llvm::CallInst::TCK_MustTail);
    ir_.CreateRet(call);
    ir_.SetInsertPoint(budget);
    result(count, ImageExit::budget);
    ir_.SetInsertPoint(deadline);
    result(count, ImageExit::deadline);
    ir_.SetInsertPoint(missing);
    result(count, ImageExit::unprepared);
    cold_edges(*dispatch_, {budget, deadline, missing});
  }

  void emit_entry() {
    auto *entry = function(name_, false);
    ir_.SetCurrentDebugLocation(Origins::generated(*entry));
    entry->addFnAttr("dsp56300-image", "1");
    ir_.SetInsertPoint(block(entry, "entry"));
    auto *call = ir_.CreateCall(
        dispatch_,
        {entry->getArg(0), entry->getArg(1), entry->getArg(2), entry->getArg(3), ir_.getInt32(0)});
    call->setCallingConv(dispatch_->getCallingConv());
    call->setTailCallKind(llvm::CallInst::TCK_Tail);
    ir_.CreateRet(call);
  }

  llvm::GlobalVariable *descriptor(const std::string &name, const Instruction &operation) {
    const std::array<llvm::Constant *, 5> fields{
        ir_.getInt32(operation.opcode),
        ir_.getInt32(operation.extension),
        ir_.getInt8(static_cast<std::uint8_t>(operation.operation)),
        ir_.getInt8(operation.words),
        ir_.getInt8(operation.cycles)};
    auto *value = llvm::ConstantStruct::getAnon(module_.getContext(), fields);
    auto *global = constant(name + ".op", value);
    global->setAlignment(llvm::Align(alignof(Instruction)));
    return global;
  }

  llvm::Function *semantic_variant(const Instruction &operation) {
    const auto key = std::pair(operation.opcode, operation.extension);
    const auto found = semantic_variants_.find(key);
    if (found != semantic_variants_.end()) { return found->second; }
    const auto *const kind = semantic_->arg_size() == 3 ? "ordinary." : "vector.";
    const auto name = module_.getModuleIdentifier() + ".semantic." + kind +
                      llvm::utohexstr(key.first, true) + "." + llvm::utohexstr(key.second, true);
    auto *type =
        llvm::FunctionType::get(ir_.getInt64Ty(), {ir_.getPtrTy(), ir_.getInt32Ty()}, false);
    if (auto *existing = module_.getNamedValue(name)) {
      auto *function = llvm::dyn_cast<llvm::Function>(existing);
      if (function == nullptr || function->isDeclaration() || function->getFunctionType() != type ||
          function->getCallingConv() != llvm::CallingConv::C ||
          !function->hasFnAttribute("dsp56300-encoding") ||
          function->getFnAttribute("dsp56300-encoding").getValueAsString() !=
              semantic_->getName()) {
        throw std::invalid_argument("DSP image encoding name conflicts with an existing symbol");
      }
      semantic_variants_.emplace(key, function);
      return function;
    }
    auto *result = llvm::Function::Create(type, llvm::GlobalValue::InternalLinkage, name, module_);
    attributes(result);
    result->addFnAttr("dsp56300-encoding", semantic_->getName());
    const llvm::IRBuilderBase::InsertPointGuard insertion(ir_);
    ir_.SetCurrentDebugLocation({});
    ir_.SetInsertPoint(block(result, "entry"));
    std::vector<Value *> arguments{result->getArg(0)};
    if (semantic_->arg_size() == 3) { arguments.push_back(result->getArg(1)); }
    arguments.push_back(descriptor(name, operation));
    ir_.CreateRet(ir_.CreateCall(semantic_, arguments));
    semantic_variants_.emplace(key, result);
    return result;
  }

  Block *successor(const Word pc) {
    const auto site = sites_.find(pc);
    if (site != sites_.end()) { return site->second; }
    const auto target = targets_.find(pc);
    if (target == targets_.end()) { return exits_.at(ImageExit::unprepared); }
    const auto [link, added] = links_.try_emplace(pc, nullptr);
    if (added) {
      const llvm::IRBuilderBase::InsertPointGuard insertion(ir_);
      link->second = block(active_, "link." + llvm::utohexstr(pc, true));
      ir_.SetInsertPoint(link->second);
      tail(target->second,
           active_->getArg(0),
           ir_.getInt32(pc),
           active_->getArg(2),
           ir_.CreateLoad(ir_.getInt64Ty(), window_),
           ir_.CreateLoad(ir_.getInt32Ty(), count_));
    }
    return link->second;
  }

  void emit_instruction(const Word pc, const Instruction &operation) {
    ir_.SetCurrentDebugLocation(Origins::instruction(*active_, pc));
    ir_.SetInsertPoint(sites_.at(pc));
    auto *time = block(active_, "time");
    auto *execute = block(active_, "execute");
    auto *advance = block(active_, "advance");
    auto *halt = block(active_, "halt");
    auto *owner = block(active_, "owner");
    cold_.insert(halt);
    auto *sequential = block(active_, "sequential");
    auto *branched = block(active_, "branch");
    auto *count = ir_.CreateLoad(ir_.getInt32Ty(), count_);
    ir_.CreateCondBr(
        ir_.CreateICmpULT(count, active_->getArg(2)), time, exits_.at(ImageExit::budget));
    ir_.SetInsertPoint(time);
    auto *window = ir_.CreateLoad(ir_.getInt64Ty(), window_);
    ir_.CreateCondBr(
        ir_.CreateICmpNE(window, ir_.getInt64(0)), execute, exits_.at(ImageExit::deadline));
    ir_.SetInsertPoint(execute);
    memory_contracts();
    emit_paired_read(pc, count, window);
    emit_regions(pc, {.count = count, .window = window});
    auto *outcome =
        ir_.CreateCall(semantic_variant(operation), {active_->getArg(0), ir_.getInt32(pc)});
    auto *next = ir_.CreateTrunc(outcome, ir_.getInt32Ty());
    auto *flow = ir_.CreateTrunc(ir_.CreateLShr(outcome, flow_shift), ir_.getInt8Ty());
    constexpr unsigned opcode_shift = 24;
    constexpr unsigned kind_shift = 48;
    const auto site = static_cast<std::uint64_t>(pc) |
                      (static_cast<std::uint64_t>(operation.opcode) << opcode_shift) |
                      (static_cast<std::uint64_t>(operation.operation) << kind_shift);
    auto *retired = ir_.CreateAdd(count, ir_.getInt32(1));
    ir_.CreateCondBr(
        ir_.CreateICmpUGE(flow, ir_.getInt8(static_cast<unsigned>(SemanticFlow::sequential))),
        advance,
        halt);
    ir_.SetInsertPoint(halt);
    ir_.CreateCondBr(
        ir_.CreateICmpEQ(flow, ir_.getInt8(static_cast<unsigned>(SemanticFlow::owner))),
        owner,
        exits_.at(ImageExit::fault));
    ir_.SetInsertPoint(owner);
    static_cast<void>(emit_retirement(*owner, active_->getArg(0), ir_.getInt32(1), false, site));
    result(retired, ImageExit::owner);
    ir_.SetInsertPoint(advance);
    ir_.CreateStore(retired, count_);
    ir_.CreateStore(next, pc_);
    auto *cost = ir_.getInt64(operation.cycles);
    ir_.CreateStore(ir_.CreateSelect(ir_.CreateICmpUGT(window, cost),
                                     ir_.CreateSub(window, cost),
                                     ir_.getInt64(0)),
                    window_);
    retirement(ir_.getInt32(1), false, site);
    ir_.CreateCondBr(
        ir_.CreateICmpEQ(flow, ir_.getInt8(static_cast<unsigned>(SemanticFlow::sequential))),
        sequential,
        branched);
    ir_.SetInsertPoint(branched);
    auto *transfer = ir_.CreateSwitch(next, exits_.at(ImageExit::unprepared));
    for (const auto target : edges_.at(pc)) {
      if (targets_.contains(target)) { transfer->addCase(ir_.getInt32(target), successor(target)); }
    }
    ir_.SetInsertPoint(sequential);
    const auto expected = (pc + operation.words) & word_mask;
    auto *assume = llvm::Intrinsic::getDeclaration(&module_, llvm::Intrinsic::assume);
    ir_.CreateCall(assume, ir_.CreateICmpEQ(next, ir_.getInt32(expected)));
    ir_.CreateBr(successor(expected));
  }

  void memory_contract(const SemanticMemory &memory) {
    auto *function = module_.getFunction("dsp56300_semantic_memory");
    if (function == nullptr || function->isDeclaration() ||
        !function->getReturnType()->isVoidTy() || function->arg_size() != 2 ||
        !function->getFunctionType()->getParamType(0)->isPointerTy() ||
        !function->getFunctionType()->getParamType(1)->isPointerTy()) {
      throw std::invalid_argument("DSP image lacks its private memory contract ABI");
    }
    const std::array<llvm::Constant *, 7> fields{ir_.getInt32(memory.space),
                                                 ir_.getInt32(memory.index),
                                                 ir_.getInt32(memory.regions),
                                                 ir_.getInt32(memory.first),
                                                 ir_.getInt32(memory.words),
                                                 ir_.getInt32(memory.flags),
                                                 ir_.getInt32(memory.external)};
    auto *value = llvm::ConstantStruct::getAnon(module_.getContext(), fields);
    const auto name = active_->getName().str() + ".memory." + std::to_string(memory.space) + "." +
                      std::to_string(memory.index);
    auto *description = module_.getGlobalVariable(name, true);
    if (description == nullptr) {
      description = constant(name, value);
    } else if (!description->isConstant() || description->getInitializer() != value) {
      throw std::invalid_argument("DSP memory contract conflicts with an existing constant");
    }
    description->setAlignment(llvm::Align(alignof(SemanticMemory)));
    ir_.CreateCall(function, {active_->getArg(0), description});
  }

  void memory_contracts() {
    if (!memory_ || direct_) { return; }
    const auto regions = *memory_;
    const bool external = std::ranges::none_of(
        regions, [](const auto &region) { return region.space == Space::program; });
    for (const auto space : {Space::program, Space::x, Space::y}) {
      if (external && space == Space::program) { continue; }
      const auto count =
          static_cast<Word>(std::ranges::count(regions, space, &RegionLayout::space));
      Word index = 0;
      if (count == 0) {
        memory_contract({.space = static_cast<Word>(space),
                         .index = 0,
                         .regions = 0,
                         .first = 0,
                         .words = 0,
                         .flags = 0,
                         .external = external ? 1U : 0U});
      }
      for (const auto &region : regions) {
        if (region.space != space) { continue; }
        memory_contract({.space = static_cast<Word>(space),
                         .index = index++,
                         .regions = count,
                         .first = region.first,
                         .words = static_cast<Word>(region.size),
                         .flags = memory_flags(region),
                         .external = external ? 1U : 0U});
      }
    }
  }

  void retirement(Value *count, const bool region, const std::uint64_t site) {
    if (auto *ready =
            emit_retirement(*ir_.GetInsertBlock(), active_->getArg(0), count, region, site)) {
      auto *continued = block(active_, "observed");
      ir_.CreateCondBr(ready, continued, exits_.at(ImageExit::owner));
      ir_.SetInsertPoint(continued);
    }
  }

  void emit_unit(llvm::Function *unit, const Word unit_head) {
    active_ = unit;
    ir_.SetCurrentDebugLocation(Origins::generated(*unit));
    sites_.clear();
    links_.clear();
    exits_.clear();
    cold_.clear();
    auto *entry = block(unit, "entry");
    local_dispatch_ = block(unit, "dispatch");
    ir_.SetInsertPoint(entry);
    count_ = ir_.CreateAlloca(ir_.getInt32Ty());
    window_ = ir_.CreateAlloca(ir_.getInt64Ty());
    pc_ = ir_.CreateAlloca(ir_.getInt32Ty());
    region_cost_ = ir_.CreateAlloca(ir_.getInt64Ty());
    ir_.CreateStore(unit->getArg(4), count_);
    ir_.CreateStore(unit->getArg(3), window_);
    ir_.CreateStore(unit->getArg(1), pc_);
    for (const auto reason : {ImageExit::owner,
                              ImageExit::fault,
                              ImageExit::budget,
                              ImageExit::deadline,
                              ImageExit::unprepared}) {
      auto *exit = block(unit, "exit");
      exits_.emplace(reason, exit);
      cold_.insert(exit);
    }
    constexpr auto unit_entry = std::uint64_t{1} << 26;
    retirement(ir_.getInt32(0), true, unit_head | unit_entry);
    ir_.CreateBr(local_dispatch_);
    for (const auto &[pc, target] : targets_) {
      if (target == unit) { sites_.emplace(pc, block(unit, "pc." + llvm::utohexstr(pc, true))); }
    }
    ir_.SetInsertPoint(local_dispatch_);
    auto *switch_pc =
        ir_.CreateSwitch(ir_.CreateLoad(ir_.getInt32Ty(), pc_), exits_.at(ImageExit::unprepared));
    for (const auto &[pc, site] : sites_) { switch_pc->addCase(ir_.getInt32(pc), site); }
    for (const auto &[pc, site] : sites_) {
      static_cast<void>(site);
      emit_instruction(pc, plan_.instructions.at(pc));
    }
    for (const auto &[reason, exit] : exits_) {
      ir_.SetInsertPoint(exit);
      auto *count = ir_.CreateLoad(ir_.getInt32Ty(), count_);
      if (reason == ImageExit::unprepared) {
        tail(dispatch_,
             unit->getArg(0),
             ir_.CreateLoad(ir_.getInt32Ty(), pc_),
             unit->getArg(2),
             ir_.CreateLoad(ir_.getInt64Ty(), window_),
             count);
      } else {
        result(count, reason);
      }
    }
    cold_edges(*unit, cold_);
  }

  llvm::Module &module_;
  Origins locations_;
  llvm::IRBuilder<> ir_;
  std::string name_;
  const ImagePlan &plan_;
  const std::map<Word, std::vector<Word>> edges_;
  llvm::Function *semantic_, *dispatch_ = nullptr, *active_ = nullptr;
  const std::optional<std::span<const RegionLayout>> memory_;
  const std::uint64_t variant_;
  const bool direct_;
  llvm::GlobalVariable *table_ = nullptr;
  std::map<Word, llvm::Function *> units_, targets_;
  std::map<std::pair<Word, Word>, llvm::Function *> semantic_variants_;
  std::multimap<Word, FastRegion> regions_;
  std::map<Word, llvm::GlobalVariable *> paired_reads_;
  std::map<Word, Block *> sites_, links_;
  std::map<ImageExit, Block *> exits_;
  std::set<Block *> cold_;
  Block *local_dispatch_ = nullptr;
  Value *count_ = nullptr, *window_ = nullptr, *pc_ = nullptr, *region_cost_ = nullptr;
};
}

void lift_image(LlvmModule &module,
                const std::span<const CodeSpan> code,
                const std::span<const Word> entries) {
  if (module.code() == nullptr) { throw std::logic_error("Image module was already transferred"); }
  const auto plan = plan_image(code, entries);
  lift_image(module, plan);
}

void lift_image(LlvmModule &module,
                const ImagePlan &plan,
                const std::string_view entry,
                const std::optional<std::span<const RegionLayout>> memory,
                const std::optional<std::span<const Word>> region_modes,
                const std::uint64_t variant,
                const bool direct,
                const bool paired) {
  if (module.code() == nullptr) { throw std::logic_error("Image module was already transferred"); }
  if (plan.instructions.empty()) { throw std::invalid_argument("Image has no instructions"); }
  const ImageLifter lifter(
      module, plan, entry, false, memory, region_modes, variant, direct, paired);
}

void lift_vector(LlvmModule &module,
                 const ImagePlan &plan,
                 const std::string_view entry,
                 const std::uint64_t variant,
                 const bool direct) {
  if (module.code() == nullptr) { throw std::logic_error("Image module was already transferred"); }
  if (plan.instructions.empty()) {
    throw std::invalid_argument("Vector image has no instructions");
  }
  const ImageLifter lifter(module, plan, entry, true, std::nullopt, std::nullopt, variant, direct);
}

ImageArtifact lower_images(const std::string_view name,
                           const std::span<const ImageTranslation> translations,
                           const std::span<const CallbackBinding> callbacks,
                           const CompileOptions::MemoryMode mode) {
  if (name.empty() || name.find('\0') != std::string_view::npos || translations.empty() ||
      translations.front().name != name) {
    throw std::invalid_argument("DSP semantic image requires its complete primary entry");
  }
  ImageArtifact result{.name = std::string(name), .entries = {}, .abis = {}, .bitcode = {}};
  result.entries.reserve(translations.size());
  std::set<std::string_view> names;
  for (const auto &translation : translations) {
    if (translation.name.empty() || translation.name.find('\0') != std::string_view::npos ||
        translation.plan.instructions.empty() || !names.insert(translation.name).second) {
      throw std::invalid_argument("Invalid or duplicate DSP semantic image translation");
    }
    result.entries.emplace_back(translation.name);
  }
  LlvmModule module(result.name);
  module.link_bitcode(semantic_ir());
  configure_memory(module, mode);
  mark_io(module);
  declare_callbacks(module, callbacks);
  const bool paired = std::ranges::any_of(callbacks, [](const CallbackBinding &binding) {
    return binding.role == CallbackRole::paired_read;
  });
  for (const auto &translation : translations) {
    if (translation.vector) {
      lift_vector(module,
                  translation.plan,
                  translation.name,
                  translation.variant,
                  mode == CompileOptions::MemoryMode::direct);
    } else {
      lift_image(module,
                 translation.plan,
                 translation.name,
                 translation.memory,
                 translation.region_modes,
                 translation.variant,
                 mode == CompileOptions::MemoryMode::direct,
                 paired);
    }
  }
  specialize_image(module, result.entries);
  inline_io(module);
  for (std::size_t entry = 0; entry < result.entries.size(); ++entry) {
    const auto *function = llvm::unwrap(module.code())->getFunction(result.entries[entry]);
    std::string abi;
    llvm::raw_string_ostream stream(abi);
    stream << "llvm19;cc=" << function->getCallingConv() << ";type=";
    function->getFunctionType()->print(stream);
    constexpr std::array attributes{llvm::Attribute::SExt,
                                    llvm::Attribute::ZExt,
                                    llvm::Attribute::InReg,
                                    llvm::Attribute::StructRet,
                                    llvm::Attribute::ByVal,
                                    llvm::Attribute::ByRef,
                                    llvm::Attribute::InAlloca,
                                    llvm::Attribute::Preallocated,
                                    llvm::Attribute::Nest,
                                    llvm::Attribute::SwiftSelf,
                                    llvm::Attribute::SwiftError,
                                    llvm::Attribute::SwiftAsync,
                                    llvm::Attribute::Returned};
    for (unsigned index = 0; index <= function->arg_size(); ++index) {
      for (const auto kind : attributes) {
        const auto attribute = function->getAttributes().getAttributeAtIndex(index, kind);
        if (attribute.isValid()) { stream << ';' << index << ':' << attribute.getAsString(); }
      }
    }
    result.abis.push_back(std::move(abi));
  }
  result.bitcode = module.bitcode();
  return result;
}
}
