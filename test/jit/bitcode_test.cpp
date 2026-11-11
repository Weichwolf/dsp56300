// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit/llvm/runtime.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <memory>
#include <span>
#include <stdexcept>
#include <type_traits>

namespace {
using dsp56300::detail::LlvmModule;
using dsp56300::test::require;

struct DisposeModule {
  void operator()(LLVMModuleRef module) const noexcept { LLVMDisposeModule(module); }
};

void parsing() {
  const LlvmModule module("parsed_source");
  auto *type = LLVMFunctionType(LLVMVoidTypeInContext(module.context()), nullptr, 0, 0);
  auto *function = LLVMAddFunction(module.code(), "retained_definition", type);
  auto *block = LLVMAppendBasicBlockInContext(module.context(), function, "entry");
  auto *builder = LLVMCreateBuilderInContext(module.context());
  LLVMPositionBuilderAtEnd(builder, block);
  LLVMBuildRetVoid(builder);
  LLVMDisposeBuilder(builder);
  const auto original = module.bitcode();
  const auto *context = module.context();
  constexpr std::array invalid{std::byte{1}};
  for (const auto input : {std::span<const std::byte>{}, std::span<const std::byte>(invalid)}) {
    bool rejected = false;
    try {
      const std::unique_ptr<std::remove_pointer_t<LLVMModuleRef>, DisposeModule> parsed(
          module.parse_bitcode(input));
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected && module.context() == context && module.bitcode() == original,
            "Malformed parsing preserves diagnostics, context and complete parent identity");
  }
  {
    const std::unique_ptr<std::remove_pointer_t<LLVMModuleRef>, DisposeModule> parsed(
        module.parse_bitcode(original));
    require(LLVMGetModuleContext(parsed.get()) == context &&
                LLVMGetNamedFunction(parsed.get(), "retained_definition") != nullptr &&
                module.bitcode() == original,
            "Successful parsing owns a separate module in the original comparison context");
  }
  LlvmModule destination("after_borrowed_parse");
  destination.link_bitcode(original);
  require(LLVMGetNamedFunction(destination.code(), "retained_definition") != nullptr &&
              module.context() == context && module.bitcode() == original,
          "Disposed parse results leave later transactional links valid");
}

void declarations() {
  using namespace dsp56300;
  constexpr std::array words{static_cast<Word>(isa::Opcode::stop)};
  const std::array code{CodeSpan{.address = 0, .words = words}};
  constexpr std::array<Word, 1> entries{0};
  const std::array variants{
      CodeVariant{.id = 7, .code = code, .entries = entries, .failed_fetches = {}}};
  constexpr std::array callbacks{
      CallbackBinding{.role = CallbackRole::read, .symbol = "probe_read"}};
  constexpr std::array invalid_host{std::byte{0}};
  const auto image = Core::lower({.variants = variants},
                                 {.memory = {},
                                  .interrupts = {},
                                  .transitions = {},
                                  .callbacks = callbacks,
                                  .bindings = invalid_host});
  LlvmModule module("public_callback_declaration");
  module.link_bitcode(image.bitcode());
  auto *function = LLVMGetNamedFunction(module.code(), "probe_read");
  require(function != nullptr && LLVMIsDeclaration(function) != 0 &&
              LLVMGetFunctionCallConv(function) == LLVMCCallConv && !image.identity().empty(),
          "Public lowering retains the native callback CC without parsing standalone host input");
}
}

int main() try {
  parsing();
  declarations();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
