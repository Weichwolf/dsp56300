// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"

#include "llvm-c/BitReader.h"
#include "llvm-c/Core.h"
#include "llvm-c/Error.h"
#include "llvm-c/LLJIT.h"
#include "llvm-c/Orc.h"
#include "llvm-c/Support.h"
#include "llvm-c/Target.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Transforms/PassBuilder.h"
#include "llvm-c/Types.h"
#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/Support/Error.h"

#include <bit>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace image_compiler {
template <auto Dispose> struct Delete {
  void operator()(auto *value) const noexcept { Dispose(value); }
};
template <class Ref, auto Dispose>
using Owner = std::unique_ptr<std::remove_pointer_t<Ref>, Delete<Dispose>>;

inline void checked(LLVMErrorRef error) {
  if (error == nullptr) { return; }
  const Owner<char *, LLVMDisposeErrorMessage> message(LLVMGetErrorMessage(error));
  throw std::runtime_error(message.get());
}

inline void dispose(LLVMOrcLLJITRef engine) noexcept {
  if (auto *error = LLVMOrcDisposeLLJIT(engine)) {
    LLVMConsumeError(error);
    std::terminate();
  }
}

class Code {
public:
  explicit Code(const SemanticImage &image) {
    if (LLVMInitializeNativeTarget() != 0 || LLVMInitializeNativeAsmPrinter() != 0 ||
        LLVMLoadLibraryPermanently(nullptr) != 0) {
      throw std::runtime_error("Cannot initialize the native test consumer");
    }
    const Owner<LLVMOrcThreadSafeContextRef, LLVMOrcDisposeThreadSafeContext> context(
        LLVMOrcCreateNewThreadSafeContext());
    const auto bytes = image.bitcode();
    const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> buffer(
        LLVMCreateMemoryBufferWithMemoryRangeCopy(
            reinterpret_cast<const char *>(bytes.data()), bytes.size(), "public-image"));
    LLVMModuleRef raw_module = nullptr;
    const auto parsed = LLVMParseBitcodeInContext2(
        LLVMOrcThreadSafeContextGetContext(context.get()), buffer.get(), &raw_module);
    Owner<LLVMModuleRef, LLVMDisposeModule> module(raw_module);
    if (parsed != 0 || !module) { throw std::runtime_error("Cannot parse public semantic code"); }
    char *raw_error = nullptr;
    LLVMTargetRef architecture = nullptr;
    const auto found =
        LLVMGetTargetFromTriple(LLVMGetTarget(module.get()), &architecture, &raw_error);
    const Owner<char *, LLVMDisposeMessage> message(raw_error);
    if (found != 0) { throw std::runtime_error(message.get()); }
    const Owner<char *, LLVMDisposeMessage> cpu(LLVMGetHostCPUName());
    const Owner<char *, LLVMDisposeMessage> features(LLVMGetHostCPUFeatures());
    const Owner<LLVMTargetMachineRef, LLVMDisposeTargetMachine> machine(
        LLVMCreateTargetMachine(architecture,
                                LLVMGetTarget(module.get()),
                                cpu.get(),
                                features.get(),
                                LLVMCodeGenLevelAggressive,
                                LLVMRelocPIC,
                                LLVMCodeModelSmall));
    if (!machine) { throw std::runtime_error("Cannot create the native consumer target"); }
    const Owner<LLVMPassBuilderOptionsRef, LLVMDisposePassBuilderOptions> options(
        LLVMCreatePassBuilderOptions());
    checked(LLVMRunPasses(module.get(), "default<O3>", machine.get(), options.get()));
    LLVMOrcLLJITRef raw_engine = nullptr;
    checked(LLVMOrcCreateLLJIT(&raw_engine, nullptr));
    engine_.reset(raw_engine);
    auto *thread_safe = LLVMOrcCreateNewThreadSafeModule(module.release(), context.get());
    checked(LLVMOrcLLJITAddLLVMIRModule(
        engine_.get(), LLVMOrcLLJITGetMainJITDylib(engine_.get()), thread_safe));
    for (const auto symbol : image.exports()) {
      static_cast<void>(address(std::string(symbol.name)));
    }
    if (auto error = native().initialize(native().getMainJITDylib())) {
      throw std::runtime_error(llvm::toString(std::move(error)));
    }
  }

  ~Code() {
    if (auto error = native().deinitialize(native().getMainJITDylib())) {
      llvm::consumeError(std::move(error));
      std::terminate();
    }
  }

  Code(const Code &) = delete;
  Code &operator=(const Code &) = delete;

  [[nodiscard]] const void *address(const std::string &name) const {
    LLVMOrcExecutorAddress result = 0;
    checked(LLVMOrcLLJITLookup(engine_.get(), &result, name.c_str()));
    return std::bit_cast<const void *>(result);
  }

private:
  [[nodiscard]] llvm::orc::LLJIT &native() const {
    return *reinterpret_cast<llvm::orc::LLJIT *>(engine_.get());
  }

  Owner<LLVMOrcLLJITRef, dispose> engine_;
};
}

inline NativeImage compile_image(const SemanticImage &image) {
  const auto owner = std::make_shared<image_compiler::Code>(image);
  std::vector<NativeSymbol> symbols;
  for (const auto entry : image.exports()) {
    symbols.push_back({.name = entry.name, .address = owner->address(std::string(entry.name))});
  }
  return Core::bind(image, {.owner = owner, .symbols = symbols, .identity = image.identity()});
}
}
