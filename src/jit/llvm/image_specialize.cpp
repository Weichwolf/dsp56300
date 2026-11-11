// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/image.hpp"
#include "jit/llvm/runtime.hpp"

#include <array>
#include <llvm-c/Error.h>
#include <llvm-c/Transforms/PassBuilder.h>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/InstrTypes.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/StructuralHash.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Transforms/IPO/Internalize.h>
#include <memory>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace dsp56300::detail {
namespace {
bool isa_function(const llvm::Function &function) {
  const auto name = function.getName();
  return name == "dsp56300_semantic_execute" || name == "dsp56300_semantic_vector_execute" ||
         name == "dsp56300_semantic_memory" || name.starts_with("_ZN8dsp563004Core4Impl") ||
         name.starts_with("_ZNK8dsp563004Core4Impl") ||
         name.starts_with("_ZN8dsp563006detail10repeatableE") ||
         (name.starts_with("_ZN8dsp563006detail") && name.contains("permitted_vector_"));
}

std::set<llvm::Function *> semantic_calls(llvm::Module &module) {
  std::set<llvm::Function *> result;
  for (const auto &function : module) {
    if (!function.hasFnAttribute("dsp56300-image-part")) { continue; }
    for (const auto &block : function) {
      for (const auto &instruction : block) {
        const auto *call = llvm::dyn_cast<llvm::CallBase>(&instruction);
        if (call == nullptr) { continue; }
        auto *callee =
            llvm::dyn_cast<llvm::Function>(call->getCalledOperand()->stripPointerCasts());
        if (callee == nullptr || !isa_function(*callee)) { continue; }
        if (callee->isDeclaration()) {
          throw std::invalid_argument("DSP image lacks ISA definition " + callee->getName().str());
        }
        result.insert(callee);
      }
    }
  }
  return result;
}

struct DisposeOptions {
  void operator()(LLVMPassBuilderOptionsRef options) const noexcept {
    LLVMDisposePassBuilderOptions(options);
  }
};

void passes(LlvmModule &module, const char *pipeline) {
  const std::unique_ptr<std::remove_pointer_t<LLVMPassBuilderOptionsRef>, DisposeOptions> options(
      LLVMCreatePassBuilderOptions());
  LLVMPassBuilderOptionsSetVerifyEach(options.get(), 1);
  auto *error = LLVMRunPasses(module.code(), pipeline, nullptr, options.get());
  if (error == nullptr) { return; }
  const std::unique_ptr<char, decltype(&LLVMDisposeErrorMessage)> text(LLVMGetErrorMessage(error),
                                                                       &LLVMDisposeErrorMessage);
  throw std::runtime_error(text.get());
}

llvm::IRHash fingerprint(const llvm::Module &module) {
  std::string diagnostic;
  llvm::raw_string_ostream output(diagnostic);
  if (llvm::verifyModule(module, &output)) { throw std::invalid_argument(diagnostic); }
  return llvm::StructuralHash(module, true);
}
}

void specialize_image(LlvmModule &module) {
  const std::array entries{module.name()};
  specialize_image(module, entries);
}

void specialize_image(LlvmModule &module, const std::span<const std::string> entries) {
  if (module.code() == nullptr) { throw std::logic_error("Image module was already transferred"); }
  auto &native = *llvm::unwrap(module.code());
  if (entries.empty()) { throw std::invalid_argument("DSP image requires prepared entry roots"); }
  std::set<const llvm::GlobalValue *> roots;
  for (const auto &name : entries) {
    const auto *entry = native.getFunction(name);
    if (entry == nullptr || entry->isDeclaration() || !entry->hasFnAttribute("dsp56300-image") ||
        !roots.insert(entry).second) {
      throw std::invalid_argument("Invalid or duplicate lifted DSP image entry: " + name);
    }
  }
  std::string diagnostic;
  llvm::raw_string_ostream output(diagnostic);
  if (llvm::verifyModule(native, &output)) { throw std::invalid_argument(diagnostic); }
  llvm::internalizeModule(native,
                          [&](const llvm::GlobalValue &value) { return roots.contains(&value); });
  passes(module, "globaldce");
  for (;;) {
    const auto frontier = semantic_calls(native);
    if (frontier.empty()) { break; }
    const auto before = fingerprint(native);
    for (auto *function : frontier) {
      if (function->hasFnAttribute(llvm::Attribute::OptimizeNone)) {
        throw std::invalid_argument("DSP image requires optimized own semantic IR");
      }
      function->removeFnAttr(llvm::Attribute::NoInline);
      function->addFnAttr(llvm::Attribute::AlwaysInline);
    }
    passes(module,
           "always-inline,function(instcombine<max-iterations=8;verify-fixpoint>,"
           "simplifycfg),globaldce");
    if (fingerprint(native) == before) {
      throw std::runtime_error("DSP semantic specialization made no progress");
    }
  }
  // Specialize each complete encoding once before replicating its reduced effects
  // into native functions. The final code retains no per-instruction wrapper calls.
  for (auto &function : native) {
    if (function.getName().starts_with(module.name() + ".semantic.")) {
      function.addFnAttr(llvm::Attribute::AlwaysInline);
    }
  }
  passes(module, "always-inline,globaldce");
}
}
