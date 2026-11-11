// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/image.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/bindings.hpp"
#include "jit/llvm/runtime.hpp"
#include "semantic.hpp"
#include "semantic_ir.hpp"

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
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Alignment.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <map>
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
using Value = llvm::Value;
using Block = llvm::BasicBlock;

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
              const bool vector = false)
      : module_(*llvm::unwrap(module.code())),
        ir_(module_.getContext()),
        name_(entry.empty() ? module.name() : std::string(entry)),
        plan_(plan),
        edges_(plan.edges()),
        semantic_(semantic_function(module_, vector)) {
    if (module_.getNamedValue(name_) != nullptr) {
      throw std::invalid_argument("DSP image entry already exists");
    }
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
    for (const auto &unit : units_) { emit_unit(unit.second); }
    emit_entry();
  }

private:
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
    call->setTailCallKind(llvm::CallInst::TCK_MustTail);
    ir_.CreateRet(call);
  }

  void result(Value *count, const ImageExit reason) {
    ir_.CreateRet(ir_.CreateOr(ir_.CreateZExt(count, ir_.getInt64Ty()),
                               ir_.getInt64(static_cast<std::uint64_t>(reason) << flow_shift)));
  }

  void emit_dispatch() {
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
    call->setTailCallKind(llvm::CallInst::TCK_MustTail);
    ir_.CreateRet(call);
    ir_.SetInsertPoint(budget);
    result(count, ImageExit::budget);
    ir_.SetInsertPoint(deadline);
    result(count, ImageExit::deadline);
    ir_.SetInsertPoint(missing);
    result(count, ImageExit::unprepared);
  }

  void emit_entry() {
    auto *entry = function(name_, false);
    entry->addFnAttr("dsp56300-image", "1");
    ir_.SetInsertPoint(block(entry, "entry"));
    auto *call = ir_.CreateCall(
        dispatch_,
        {entry->getArg(0), entry->getArg(1), entry->getArg(2), entry->getArg(3), ir_.getInt32(0)});
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
    ir_.SetInsertPoint(sites_.at(pc));
    auto *time = block(active_, "time");
    auto *execute = block(active_, "execute");
    auto *commit = block(active_, "commit");
    auto *continued = block(active_, "continued");
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
    auto *outcome =
        ir_.CreateCall(semantic_variant(operation), {active_->getArg(0), ir_.getInt32(pc)});
    auto *next = ir_.CreateTrunc(outcome, ir_.getInt32Ty());
    auto *flow = ir_.CreateTrunc(ir_.CreateLShr(outcome, flow_shift), ir_.getInt8Ty());
    ir_.CreateStore(next, pc_);
    ir_.CreateCondBr(
        ir_.CreateICmpEQ(flow, ir_.getInt8(static_cast<unsigned>(SemanticFlow::fault))),
        exits_.at(ImageExit::fault),
        commit);
    ir_.SetInsertPoint(commit);
    ir_.CreateStore(ir_.CreateAdd(count, ir_.getInt32(1)), count_);
    auto *cost = ir_.getInt64(operation.cycles);
    ir_.CreateStore(ir_.CreateSelect(ir_.CreateICmpUGT(window, cost),
                                     ir_.CreateSub(window, cost),
                                     ir_.getInt64(0)),
                    window_);
    ir_.CreateCondBr(
        ir_.CreateICmpEQ(flow, ir_.getInt8(static_cast<unsigned>(SemanticFlow::owner))),
        exits_.at(ImageExit::owner),
        continued);
    ir_.SetInsertPoint(continued);
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

  void emit_unit(llvm::Function *unit) {
    active_ = unit;
    sites_.clear();
    links_.clear();
    exits_.clear();
    auto *entry = block(unit, "entry");
    local_dispatch_ = block(unit, "dispatch");
    ir_.SetInsertPoint(entry);
    count_ = ir_.CreateAlloca(ir_.getInt32Ty());
    window_ = ir_.CreateAlloca(ir_.getInt64Ty());
    pc_ = ir_.CreateAlloca(ir_.getInt32Ty());
    ir_.CreateStore(unit->getArg(4), count_);
    ir_.CreateStore(unit->getArg(3), window_);
    ir_.CreateStore(unit->getArg(1), pc_);
    ir_.CreateBr(local_dispatch_);
    for (const auto reason : {ImageExit::owner,
                              ImageExit::fault,
                              ImageExit::budget,
                              ImageExit::deadline,
                              ImageExit::unprepared}) {
      exits_.emplace(reason, block(unit, "exit"));
    }
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
  }

  llvm::Module &module_;
  llvm::IRBuilder<> ir_;
  std::string name_;
  const ImagePlan &plan_;
  const std::map<Word, std::vector<Word>> edges_;
  llvm::Function *semantic_, *dispatch_ = nullptr, *active_ = nullptr;
  llvm::GlobalVariable *table_ = nullptr;
  std::map<Word, llvm::Function *> units_, targets_;
  std::map<std::pair<Word, Word>, llvm::Function *> semantic_variants_;
  std::map<Word, Block *> sites_, links_;
  std::map<ImageExit, Block *> exits_;
  Block *local_dispatch_ = nullptr;
  Value *count_ = nullptr, *window_ = nullptr, *pc_ = nullptr;
};
}

void lift_image(LlvmModule &module,
                const std::span<const CodeSpan> code,
                const std::span<const Word> entries) {
  if (module.code() == nullptr) { throw std::logic_error("Image module was already transferred"); }
  const auto plan = plan_image(code, entries);
  lift_image(module, plan);
}

void lift_image(LlvmModule &module, const ImagePlan &plan, const std::string_view entry) {
  if (module.code() == nullptr) { throw std::logic_error("Image module was already transferred"); }
  if (plan.instructions.empty()) { throw std::invalid_argument("Image has no instructions"); }
  const ImageLifter lifter(module, plan, entry);
}

void lift_vector(LlvmModule &module, const ImagePlan &plan, const std::string_view entry) {
  if (module.code() == nullptr) { throw std::logic_error("Image module was already transferred"); }
  if (plan.instructions.empty()) {
    throw std::invalid_argument("Vector image has no instructions");
  }
  const ImageLifter lifter(module, plan, entry, true);
}

ImageArtifact lower_images(const std::string_view name,
                           const std::span<const ImageTranslation> translations) {
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
  mark_io(module);
  for (const auto &translation : translations) {
    if (translation.vector) {
      lift_vector(module, translation.plan, translation.name);
    } else {
      lift_image(module, translation.plan, translation.name);
    }
  }
  specialize_image(module, result.entries);
  inline_io(module);
  for (std::size_t entry = 0; entry < result.entries.size(); ++entry) {
    const auto *function = llvm::unwrap(module.code())->getFunction(result.entries[entry]);
    std::string abi;
    llvm::raw_string_ostream stream(abi);
    stream << (translations[entry].vector ? "event;" : "entry;")
           << "cc=" << function->getCallingConv() << ";type=";
    function->getFunctionType()->print(stream);
    stream << ";noexcept="
           << (function->hasFnAttribute(llvm::Attribute::NoUnwind) ? "true" : "false")
           << ";ret=" << function->getAttributes().getRetAttrs().getAsString();
    for (unsigned index = 0; index < function->arg_size(); ++index) {
      stream << ";arg" << index << '='
             << function->getAttributes().getParamAttrs(index).getAsString();
    }
    result.abis.push_back(std::move(abi));
  }
  result.bitcode = module.bitcode();
  return result;
}
}
