// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/runtime.hpp"
#include "support.hpp"

#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Transforms/Utils/ModuleUtils.h>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using dsp56300::detail::LlvmCompiler;
using dsp56300::detail::LlvmModule;
using dsp56300::test::require;
unsigned initialized = 0, finalized = 0;

void initialize() noexcept {
  ++initialized;
}

void finalize() noexcept {
  ++finalized;
}

void source(LlvmModule &module) {
  auto &native = *llvm::unwrap(module.code());
  auto &context = native.getContext();
  auto *type = llvm::FunctionType::get(llvm::Type::getVoidTy(context), false);
  auto *begin =
      llvm::Function::Create(type, llvm::GlobalValue::ExternalLinkage, "image_begin", native);
  auto *end = llvm::Function::Create(type, llvm::GlobalValue::ExternalLinkage, "image_end", native);
  const auto wrapper = [&](llvm::Function *callback, const char *name) {
    auto *function = llvm::Function::Create(type, llvm::GlobalValue::InternalLinkage, name, native);
    llvm::IRBuilder<> body(llvm::BasicBlock::Create(context, "entry", function));
    body.CreateCall(callback);
    body.CreateRetVoid();
    return function;
  };
  llvm::appendToGlobalCtors(native, wrapper(begin, "initialize_image"), 0);
  llvm::appendToGlobalDtors(native, wrapper(end, "finalize_image"), 0);
  auto *entry =
      llvm::Function::Create(llvm::FunctionType::get(llvm::Type::getInt64Ty(context), false),
                             llvm::GlobalValue::ExternalLinkage,
                             module.name(),
                             native);
  llvm::IRBuilder<> ir(llvm::BasicBlock::Create(context, "entry", entry));
  ir.CreateRet(ir.getInt64(0));
}

void rejected_sanitized_object(LlvmCompiler &compiler) {
  LlvmModule module("sanitized_object");
  source(module);
  const auto object = compiler.object(module);
  const auto before = compiler.native_bytes();
  bool refused = false;
  try {
    static_cast<void>(compiler.install_object(object, module.name()));
  } catch (const std::invalid_argument &error) {
    refused =
        std::string_view(error.what()).contains("ASan native object installation is unsupported");
  }
  require(refused && compiler.native_bytes() == before && initialized == 1 && finalized == 0,
          "Unsupported sanitized object loading fails before materialization or lifecycle effects");
}

void lifecycle() {
  constexpr std::uint64_t secondary_result = 42;
  {
    LlvmCompiler compiler;
    compiler.define("image_begin", std::bit_cast<const void *>(&initialize));
    compiler.define("image_end", std::bit_cast<const void *>(&finalize));
    LlvmModule producer("lifetime_source");
    source(producer);
    const auto bitcode = producer.bitcode();
    for (const bool object : {false, true}) {
      LlvmModule module(object ? "lifetime_object" : "lifetime_ir");
      module.link_bitcode(bitcode);
      auto &native = *llvm::unwrap(module.code());
      require(native.getModuleIdentifier() == module.name(),
              "A shared bitcode buffer cannot erase distinct module identities");
      native.getFunction(producer.name())->setName(module.name());
      const auto secondary_name = module.name() + ".additional";
      auto *secondary = llvm::Function::Create(native.getFunction(module.name())->getFunctionType(),
                                               llvm::GlobalValue::ExternalLinkage,
                                               secondary_name,
                                               native);
      llvm::IRBuilder<> ir(llvm::BasicBlock::Create(native.getContext(), "entry", secondary));
      ir.CreateRet(ir.getInt64(secondary_result));
      constexpr bool sanitized = __has_feature(address_sanitizer);
      if (object && sanitized) { rejected_sanitized_object(compiler); }
      const auto *entry = object && !sanitized
                              ? compiler.install_object(compiler.object(module), module.name())
                              : compiler.install(module);
      require(entry != nullptr && initialized == (object ? 2U : 1U) && finalized == 0,
              "Every linked module initializes exactly once before its native entry is published");
      using Function = std::uint64_t (*)() noexcept;
      const auto bytes = compiler.native_bytes();
      const auto additional = std::bit_cast<Function>(compiler.resolve(secondary_name));
      const auto *same = compiler.resolve(secondary_name);
      require(additional() == secondary_result && initialized == (object ? 2U : 1U) &&
                  finalized == 0 && same == std::bit_cast<const void *>(additional) &&
                  compiler.native_bytes() == bytes,
              "Additional exports resolve without recompiling or repeating module initialization");
    }
  }
  require(initialized == 2 && finalized == 2,
          "The execution owner finalizes every initialized module before releasing its code");
}
}

int main() {
  try {
    lifecycle();
    std::cout << dsp56300::test::checks << " native module lifetime checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
