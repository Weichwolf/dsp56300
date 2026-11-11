// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/bindings.hpp"

#include "dsp56300/core.hpp"
#include "jit/llvm/runtime.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <llvm/ADT/StringRef.h>
#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Alignment.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/MemoryBufferRef.h>
#include <llvm/Transforms/Utils/ModuleUtils.h>
#include <memory>
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
                               llvm::Attribute::SwiftAsync,
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

namespace {
struct CallbackTarget {
  llvm::Function *source, *target;
};

void wrap_callback(const CallbackTarget binding) {
  auto &source = *binding.source;
  auto &target = *binding.target;
  source.deleteBody();
  llvm::IRBuilder<> body(llvm::BasicBlock::Create(source.getContext(), "entry", &source));
  std::vector<llvm::Value *> arguments;
  for (auto &argument : source.args()) { arguments.push_back(&argument); }
  auto *call = body.CreateCall(&target, arguments);
  call->setAttributes(target.getAttributes());
  body.CreateRet(call);
}

llvm::Function *declaration(llvm::Function &source, const llvm::StringRef symbol) {
  auto &module = *source.getParent();
  auto *target = module.getFunction(symbol);
  if (target != nullptr) {
    if (!target->isDeclaration() || !same_abi(source, *target)) {
      throw std::invalid_argument("DSP callback collides with another definition/ABI: " +
                                  symbol.str());
    }
    return target;
  }
  if (module.getNamedValue(symbol) != nullptr) {
    throw std::invalid_argument("DSP callback collides with a non-function: " + symbol.str());
  }
  target = llvm::Function::Create(
      source.getFunctionType(), llvm::GlobalValue::ExternalLinkage, symbol, module);
  target->setCallingConv(source.getCallingConv());
  std::vector<llvm::AttributeSet> parameters;
  parameters.reserve(source.arg_size());
  for (unsigned index = 0; index < source.arg_size(); ++index) {
    parameters.push_back(source.getAttributes().getParamAttrs(index));
  }
  target->setAttributes(llvm::AttributeList::get(
      source.getContext(), {}, source.getAttributes().getRetAttrs(), parameters));
  target->addFnAttr(llvm::Attribute::NoUnwind);
  target->addFnAttr("dsp56300-callback");
  return target;
}

void validate_host(LlvmModule &module,
                   const std::span<const std::byte> bytes,
                   const std::span<const CallbackBinding> bindings) {
  const llvm::StringRef data(reinterpret_cast<const char *>(bytes.data()), bytes.size());
  auto parsed = llvm::parseBitcodeFile(llvm::MemoryBufferRef(data, "dsp-public-callbacks"),
                                       *llvm::unwrap(module.context()));
  if (!parsed) { throw std::invalid_argument(llvm::toString(parsed.takeError())); }
  const auto &source = *llvm::unwrap(module.code());
  for (const auto &binding : bindings) {
    const llvm::StringRef name(binding.symbol);
    const auto *expected = source.getFunction(name);
    const auto *provided = (*parsed)->getFunction(name);
    if (expected == nullptr || !expected->isDeclaration() || provided == nullptr ||
        provided->isDeclaration() || !same_abi(*expected, *provided)) {
      throw std::invalid_argument("DSP callback definition has incompatible ABI: " + name.str());
    }
  }
}
}

void declare_callbacks(LlvmModule &module, const std::span<const CallbackBinding> bindings) {
  auto &native = *llvm::unwrap(module.code());
  std::set<CallbackRole> selected;
  std::vector<CallbackTarget> targets;
  std::vector<llvm::GlobalValue *> imports;
  for (const auto &binding : bindings) {
    const auto role = static_cast<std::size_t>(binding.role);
    const llvm::StringRef name(binding.symbol);
    if (role >= names.size() || name.empty() || name.contains('\0') ||
        !selected.insert(binding.role).second || std::ranges::find(names, name) != names.end()) {
      throw std::invalid_argument("Invalid or duplicate DSP public callback role/name");
    }
    auto *source = native.getFunction(names[role]);
    if (source == nullptr || source->isDeclaration()) {
      throw std::invalid_argument("DSP callback requires its core-owned public Io adapter");
    }
    auto *target = declaration(*source, name);
    targets.push_back({.source = source, .target = target});
    imports.push_back(target);
  }
  for (const auto binding : targets) { wrap_callback(binding); }
  if (!imports.empty()) { llvm::appendToCompilerUsed(native, imports); }
}

void link_callbacks(LlvmModule &module,
                    const std::span<const std::byte> host,
                    const std::span<const CallbackBinding> bindings) {
  if (host.empty()) {
    if (bindings.empty()) { return; }
    throw std::invalid_argument("DSP callback definitions are missing");
  }
  // Parse within the same context, validate before merging attributes, then
  // destroy the borrowed-context module before link_bitcode replaces that context.
  validate_host(module, host, bindings);
  module.link_bitcode(host);
  for (const auto &binding : bindings) {
    const auto *target = llvm::unwrap(module.code())->getFunction(binding.symbol);
    if (target == nullptr || target->isDeclaration()) {
      throw std::invalid_argument("DSP callback definition is missing: " +
                                  std::string(binding.symbol));
    }
  }
}

void mark_io(LlvmModule &module) {
  auto &native = *llvm::unwrap(module.code());
  for (const auto *name : names) {
    if (auto *function = native.getFunction(name);
        function != nullptr && !function->isDeclaration()) {
      function->addFnAttr("dsp56300-io-adapter");
    }
  }
}

void inline_io(LlvmModule &module) {
  mark_io(module);
  auto &native = *llvm::unwrap(module.code());
  for (auto &function : native) {
    if (!function.hasFnAttribute("dsp56300-io-adapter")) { continue; }
    function.removeFnAttr(llvm::Attribute::NoInline);
    function.addFnAttr(llvm::Attribute::AlwaysInline);
  }
}
}
