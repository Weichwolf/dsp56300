// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/lifecycle.hpp"

#include "jit/llvm/runtime.hpp"

#include <algorithm>
#include <array>
#include <llvm/ExecutionEngine/Orc/ExecutionUtils.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Object/ObjectFile.h>
#include <llvm/Support/Error.h>
#include <llvm/Transforms/Utils/ModuleUtils.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace dsp56300::detail {
namespace {
constexpr std::array suffixes{".initialize", ".finalize"};

void export_wrapper(LlvmModule &module, const bool final) {
  auto &native = *llvm::unwrap(module.code());
  const auto range = final ? llvm::orc::getDestructors(native) : llvm::orc::getConstructors(native);
  std::vector<llvm::orc::CtorDtorIterator::Element> calls;
  for (const auto call : range) {
    if (call.Func != nullptr) { calls.push_back(call); }
  }
  if (calls.empty()) { return; }
  std::ranges::stable_sort(calls, [final](const auto &left, const auto &right) {
    return final ? left.Priority > right.Priority : left.Priority < right.Priority;
  });
  const auto name = module.name() + suffixes[final ? 1 : 0];
  if (native.getNamedValue(name) != nullptr) {
    throw std::invalid_argument("Native image lifecycle export name is already defined");
  }
  auto &context = native.getContext();
  auto *type = llvm::FunctionType::get(llvm::Type::getVoidTy(context), false);
  auto *wrapper = llvm::Function::Create(type, llvm::GlobalValue::ExternalLinkage, name, native);
  llvm::IRBuilder<> ir(llvm::BasicBlock::Create(context, "entry", wrapper));
  for (const auto &call : calls) { ir.CreateCall(call.Func); }
  ir.CreateRetVoid();
}

void bind_wrapper(llvm::Module &module, const std::string &name, const bool final) {
  auto &context = module.getContext();
  auto *type = llvm::FunctionType::get(llvm::Type::getVoidTy(context), false);
  auto *callee = llvm::Function::Create(type, llvm::GlobalValue::ExternalLinkage, name, module);
  auto *wrapper =
      llvm::Function::Create(type, llvm::GlobalValue::InternalLinkage, "invoke_lifecycle", module);
  llvm::IRBuilder<> ir(llvm::BasicBlock::Create(context, "entry", wrapper));
  ir.CreateCall(callee);
  ir.CreateRetVoid();
  if (final) {
    llvm::appendToGlobalDtors(module, wrapper, 0);
  } else {
    llvm::appendToGlobalCtors(module, wrapper, 0);
  }
}
}

void export_lifecycle(LlvmModule &module) {
  export_wrapper(module, false);
  export_wrapper(module, true);
}

ImageLifecycle object_lifecycle(const llvm::object::ObjectFile &object,
                                const std::string_view entry,
                                const char prefix) {
  ImageLifecycle result;
  const auto base = (prefix == 0 ? std::string{} : std::string(1, prefix)) + std::string(entry);
  for (const auto &symbol : object.symbols()) {
    auto name = symbol.getName();
    if (!name) { throw std::runtime_error(llvm::toString(name.takeError())); }
    const bool initialize = *name == base + suffixes[0];
    const bool finalize = *name == base + suffixes[1];
    if (!initialize && !finalize) { continue; }
    auto type = symbol.getType();
    if (!type) { throw std::runtime_error(llvm::toString(type.takeError())); }
    if (*type != llvm::object::SymbolRef::ST_Function) {
      throw std::invalid_argument("Native image lifecycle symbol is not a function");
    }
    result.initialize |= initialize;
    result.finalize |= finalize;
  }
  return result;
}

void bind_lifecycle(LlvmModule &module,
                    const ImageLifecycle lifecycle,
                    const std::string_view entry) {
  auto &native = *llvm::unwrap(module.code());
  if (lifecycle.initialize) { bind_wrapper(native, std::string(entry) + suffixes[0], false); }
  if (lifecycle.finalize) { bind_wrapper(native, std::string(entry) + suffixes[1], true); }
}
}
