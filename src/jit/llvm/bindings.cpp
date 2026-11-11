// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/bindings.hpp"

#include "jit/llvm/runtime.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <llvm/ADT/StringRef.h>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Alignment.h>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr std::array names{"dsp56300_io_read",
                           "dsp56300_io_write",
                           "dsp56300_io_bit",
                           "dsp56300_io_reset",
                           "dsp56300_io_pending",
                           "dsp56300_io_revision"};
constexpr std::array abi_kinds{llvm::Attribute::ZExt,
                               llvm::Attribute::SExt,
                               llvm::Attribute::InReg,
                               llvm::Attribute::ByVal,
                               llvm::Attribute::ByRef,
                               llvm::Attribute::StructRet,
                               llvm::Attribute::InAlloca,
                               llvm::Attribute::Nest,
                               llvm::Attribute::Returned,
                               llvm::Attribute::SwiftSelf,
                               llvm::Attribute::SwiftError,
                               llvm::Attribute::Preallocated};

bool accepts_parameters(const llvm::Function &source, const llvm::Function &target) {
  constexpr std::array guarantees{
      llvm::Attribute::NoUndef, llvm::Attribute::NonNull, llvm::Attribute::NoAlias};
  for (unsigned index = 0; index < source.arg_size(); ++index) {
    for (const auto kind : guarantees) {
      if (target.hasParamAttribute(index, kind) && !source.hasParamAttribute(index, kind)) {
        return false;
      }
    }
    if (target.getParamAlign(index).valueOrOne() > source.getParamAlign(index).valueOrOne()) {
      return false;
    }
    const auto bytes = source.getParamDereferenceableBytes(index);
    const auto nullable = source.getParamDereferenceableOrNullBytes(index);
    const auto nonnull = source.hasParamAttribute(index, llvm::Attribute::NonNull)
                             ? std::max(bytes, nullable)
                             : bytes;
    if (target.getParamDereferenceableBytes(index) > nonnull ||
        target.getParamDereferenceableOrNullBytes(index) > std::max(bytes, nullable)) {
      return false;
    }
  }
  return true;
}

bool same_abi(const llvm::Function &source, const llvm::Function &target) {
  if (source.getFunctionType() != target.getFunctionType() ||
      source.getCallingConv() != target.getCallingConv() || target.isVarArg() ||
      target.getCallingConv() != llvm::CallingConv::C ||
      !target.hasFnAttribute(llvm::Attribute::NoUnwind) ||
      target.hasFnAttribute(llvm::Attribute::NoReturn) ||
      target.hasFnAttribute(llvm::Attribute::ReturnsTwice) ||
      target.hasFnAttribute(llvm::Attribute::Naked) ||
      target.hasFnAttribute(llvm::Attribute::OptimizeNone) ||
      target.hasParamAttribute(0, llvm::Attribute::NoAlias)) {
    return false;
  }
  if (!accepts_parameters(source, target)) { return false; }
  for (unsigned index = 0; index <= source.arg_size(); ++index) {
    for (const auto kind : abi_kinds) {
      if (source.getAttributes().getAttributeAtIndex(index, kind) !=
          target.getAttributes().getAttributeAtIndex(index, kind)) {
        return false;
      }
    }
  }
  return true;
}
}

std::vector<std::byte> fuse_io(const std::span<const std::byte> semantics,
                               const std::span<const std::byte> host,
                               const std::span<const IoBinding> bindings) {
  if (semantics.empty() || host.empty() || bindings.empty()) {
    throw std::invalid_argument("DSP Io fusion requires semantic/host IR and selected roles");
  }
  LlvmModule module("dsp56300_io_fusion");
  module.link_bitcode(semantics);
  module.link_bitcode(host);
  auto &native = *llvm::unwrap(module.code());
  std::set<IoRole> selected;
  std::vector<std::pair<llvm::Function *, llvm::Function *>> targets;
  for (const auto &binding : bindings) {
    const auto role = static_cast<std::size_t>(binding.role);
    const llvm::StringRef name(binding.name);
    if (role >= names.size() || name.empty() || name.contains('\0') ||
        !selected.insert(binding.role).second) {
      throw std::invalid_argument("Invalid or duplicate DSP public Io binding role");
    }
    auto *source = native.getFunction(names[role]);
    auto *target = native.getFunction(name);
    if (source == nullptr || source->isDeclaration() || target == nullptr ||
        target->isDeclaration() || !same_abi(*source, *target) ||
        std::ranges::find(names, name) != names.end()) {
      throw std::invalid_argument("DSP public Io binding has incompatible ABI: " + name.str());
    }
    targets.emplace_back(source, target);
  }
  for (const auto &[source, target] : targets) {
    source->deleteBody();
    source->setAttributes(target->getAttributes());
    source->removeFnAttr(llvm::Attribute::AlwaysInline);
    source->addFnAttr(llvm::Attribute::NoInline);
    llvm::IRBuilder<> body(llvm::BasicBlock::Create(native.getContext(), "entry", source));
    std::vector<llvm::Value *> arguments;
    for (auto &argument : source->args()) { arguments.push_back(&argument); }
    auto *call = body.CreateCall(target, arguments);
    call->setAttributes(target->getAttributes());
    body.CreateRet(call);
  }
  return module.bitcode();
}

void inline_io(LlvmModule &module) {
  auto &native = *llvm::unwrap(module.code());
  for (const auto *name : names) {
    auto *function = native.getFunction(name);
    if (function == nullptr || function->isDeclaration()) { continue; }
    function->removeFnAttr(llvm::Attribute::NoInline);
    function->addFnAttr(llvm::Attribute::AlwaysInline);
  }
}
}
