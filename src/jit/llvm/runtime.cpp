// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/runtime.hpp"

#include "jit/llvm/native.hpp"

#if defined(__CET__)
#include <array>
#endif
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <llvm-c/Analysis.h>
#include <llvm-c/Core.h>
#include <llvm-c/Error.h>
#include <llvm-c/LLJIT.h>
#include <llvm-c/Orc.h>
#include <llvm-c/TargetMachine.h>
#include <llvm-c/Transforms/PassBuilder.h>
#include <llvm-c/Types.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace dsp56300::detail {
namespace {
template <auto dispose> struct LlvmDelete {
  template <typename T> void operator()(T *value) const noexcept { dispose(value); }
};

using Message = std::unique_ptr<char, LlvmDelete<LLVMDisposeMessage>>;
using TargetMachine = std::unique_ptr<std::remove_pointer_t<LLVMTargetMachineRef>,
                                      LlvmDelete<LLVMDisposeTargetMachine>>;

void check(LLVMErrorRef error) {
  if (error == nullptr) { return; }
  const std::unique_ptr<char, LlvmDelete<LLVMDisposeErrorMessage>> text(LLVMGetErrorMessage(error));
  throw std::runtime_error(text.get());
}

void verify(LLVMModuleRef module) {
  char *message = nullptr;
  const auto invalid = LLVMVerifyModule(module, LLVMReturnStatusAction, &message);
  const Message text(message);
  if (invalid != 0) { throw std::runtime_error(text.get()); }
}

TargetMachine native_machine() {
  const Message triple(LLVMGetDefaultTargetTriple());
  const Message cpu(LLVMGetHostCPUName());
  const Message features(LLVMGetHostCPUFeatures());
  LLVMTargetRef target = nullptr;
  char *message = nullptr;
  const auto invalid = LLVMGetTargetFromTriple(triple.get(), &target, &message);
  const Message text(message);
  if (invalid != 0) { throw std::runtime_error(text.get()); }
  TargetMachine result(LLVMCreateTargetMachine(target,
                                               triple.get(),
                                               cpu.get(),
                                               features.get(),
                                               LLVMCodeGenLevelAggressive,
                                               LLVMRelocPIC,
                                               LLVMCodeModelJITDefault));
  if (!result) { throw std::runtime_error("LLVM 19 cannot create the native target machine"); }
  return result;
}
}

struct LlvmCompiler::Impl {
  ~Impl() {
    if (jit == nullptr) { return; }
    if (auto *error = LLVMOrcDisposeLLJIT(jit)) {
      const std::unique_ptr<char, LlvmDelete<LLVMDisposeErrorMessage>> text(
          LLVMGetErrorMessage(error));
      std::cerr << "LLVM JIT disposal failed: " << text.get() << '\n';
      std::terminate();
    }
  }

  LLVMOrcLLJITRef jit = nullptr;
  TargetMachine machine;
};

LlvmModule::LlvmModule(std::string name) : name_(std::move(name)) {
  context_ = LLVMOrcCreateNewThreadSafeContext();
  module_ = LLVMModuleCreateWithNameInContext(name_.c_str(), context());
#if defined(__CET__)
  for (const auto flag_name :
       std::array<std::string_view, 2>{"cf-protection-branch", "cf-protection-return"}) {
    LLVMAddModuleFlag(module_,
                      LLVMModuleFlagBehaviorOverride,
                      flag_name.data(),
                      flag_name.size(),
                      LLVMValueAsMetadata(LLVMConstInt(LLVMInt32TypeInContext(context()), 1, 0)));
  }
#endif
}

LlvmModule::~LlvmModule() {
  if (module_ != nullptr) { LLVMDisposeModule(module_); }
  LLVMOrcDisposeThreadSafeContext(context_);
}

LLVMContextRef LlvmModule::context() const noexcept {
  return LLVMOrcThreadSafeContextGetContext(context_);
}

LLVMOrcThreadSafeModuleRef LlvmModule::release() noexcept {
  return LLVMOrcCreateNewThreadSafeModule(std::exchange(module_, nullptr), context_);
}

LlvmCompiler::LlvmCompiler() : impl_(std::make_unique<Impl>()) {
  weichwolf::llvm19::initialize_native();
  impl_->machine = native_machine();
  auto machine = native_machine();
  auto *builder = LLVMOrcCreateLLJITBuilder();
  LLVMOrcLLJITBuilderSetJITTargetMachineBuilder(
      builder, LLVMOrcJITTargetMachineBuilderCreateFromTargetMachine(machine.release()));
  check(LLVMOrcCreateLLJIT(&impl_->jit, builder));
  LLVMOrcDefinitionGeneratorRef symbols = nullptr;
  check(LLVMOrcCreateDynamicLibrarySearchGeneratorForProcess(
      &symbols, LLVMOrcLLJITGetGlobalPrefix(impl_->jit), nullptr, nullptr));
  LLVMOrcJITDylibAddGenerator(LLVMOrcLLJITGetMainJITDylib(impl_->jit), symbols);
}

LlvmCompiler::~LlvmCompiler() = default;

const void *LlvmCompiler::install(LlvmModule &module) {
  if (module.code() == nullptr) { throw std::logic_error("LLVM module was already transferred"); }
  LLVMSetTarget(module.code(), LLVMOrcLLJITGetTripleString(impl_->jit));
  LLVMSetDataLayout(module.code(), LLVMOrcLLJITGetDataLayoutStr(impl_->jit));
  constexpr bool address_sanitizer = __has_feature(address_sanitizer);
  if constexpr (address_sanitizer) {
    constexpr std::string_view name = "sanitize_address";
    const auto kind = LLVMGetEnumAttributeKindForName(name.data(), name.size());
    auto *const attribute = LLVMCreateEnumAttribute(module.context(), kind, 0);
    for (auto *function = LLVMGetFirstFunction(module.code()); function != nullptr;
         function = LLVMGetNextFunction(function)) {
      if (LLVMIsDeclaration(function) == 0) {
        LLVMAddAttributeAtIndex(
            function, static_cast<LLVMAttributeIndex>(LLVMAttributeFunctionIndex), attribute);
      }
    }
  }
  verify(module.code());
  const std::unique_ptr<std::remove_pointer_t<LLVMPassBuilderOptionsRef>,
                        LlvmDelete<LLVMDisposePassBuilderOptions>>
      options(LLVMCreatePassBuilderOptions());
  LLVMPassBuilderOptionsSetVerifyEach(options.get(), 1);
  check(LLVMRunPasses(module.code(),
                      address_sanitizer ? "default<O3>,asan" : "default<O3>",
                      impl_->machine.get(),
                      options.get()));
  verify(module.code());
  check(LLVMOrcLLJITAddLLVMIRModule(
      impl_->jit, LLVMOrcLLJITGetMainJITDylib(impl_->jit), module.release()));
  LLVMOrcExecutorAddress entry = 0;
  check(LLVMOrcLLJITLookup(impl_->jit, &entry, module.name().c_str()));
  return std::bit_cast<const void *>(static_cast<std::uintptr_t>(entry));
}
}
