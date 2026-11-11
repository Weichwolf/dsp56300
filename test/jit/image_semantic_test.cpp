// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_entry.hpp"
#include "isa.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/image.hpp"
#include "jit/llvm/runtime.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <dlfcn.h>
#include <exception>
#include <iostream>
#include <limits>
#include <llvm-c/Error.h>
#include <llvm-c/LLJIT.h>
#include <llvm-c/TargetMachine.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using namespace dsp56300;
using namespace dsp56300::detail;
using test::require;
unsigned engines = 0, machines = 0;
constexpr unsigned reason_shift = 32;

template <class Function> Function original(const char *const name) noexcept {
  auto *const address = dlsym(RTLD_NEXT, name);
  if (address == nullptr) { std::terminate(); }
  return std::bit_cast<Function>(address);
}

ImageArtifact lower() {
  std::array words{static_cast<Word>(isa::Opcode::nop), static_cast<Word>(isa::Opcode::stop)};
  const auto before = words;
  const std::array code{CodeSpan{.address = 0, .words = words}};
  const std::array<Word, 1> entries{0};
  const auto plan = plan_image(code, entries);
  const auto vector = plan_vector(
      16, {static_cast<Word>(isa::Opcode::nop), static_cast<Word>(isa::Opcode::nop)}, plan, {});
  const std::array<std::string, 3> names{
      "semantic_image", "semantic_image.variant", "semantic_image.vector"};
  const std::array translations{ImageTranslation{.name = names[0], .plan = plan},
                                ImageTranslation{.name = names[1], .plan = plan},
                                ImageTranslation{.name = names[2], .plan = vector, .vector = true}};
  auto artifact = lower_images(names.front(), translations);
  require(engines == 0 && machines == 0,
          "Semantic lowering creates neither an execution engine nor a native target machine");
  require(words == before, "Lowering cannot change the borrowed source words");
  require(artifact.name == names.front() && artifact.entries.size() == names.size() &&
              !artifact.bitcode.empty(),
          "Lowering returns owned pre-O3 bitcode and the complete ordinary/vector exports");
  for (std::size_t index = 0; index < names.size(); ++index) {
    require(artifact.entries[index] == names[index],
            "Each export name is copied into the artifact");
  }
  return artifact;
}

ImageArtifact detached() {
  auto first = lower();
  auto copy = first;
  first.name.clear();
  first.entries.clear();
  first.bitcode.clear();
  return copy;
}

void control(const ImageEntry entry) {
  require(dsp56300_image_enter(entry, nullptr, 0, 0, std::numeric_limits<std::uint64_t>::max()) ==
              (std::uint64_t{2} << reason_shift),
          "Reparsed ordinary/vector roots preserve the no-owner zero-quota boundary");
  require(dsp56300_image_enter(entry, nullptr, 0, 1, 0) == (std::uint64_t{3} << reason_shift),
          "Reparsed ordinary/vector roots preserve the expired-window boundary");
}

void materialize(const ImageArtifact &artifact) {
  LlvmModule module(artifact.name);
  module.link_bitcode(artifact.bitcode);
  for (const auto &entry : artifact.entries) {
    const auto *function = llvm::unwrap(module.code())->getFunction(entry);
    require(function != nullptr && !function->isDeclaration() && function->hasExternalLinkage() &&
                function->hasFnAttribute("dsp56300-image"),
            "Fresh-context import retains every independently named ordinary/vector definition");
  }
  require(engines == 0 && machines == 0,
          "Importing an expired producer's artifact still creates no native compiler");
  LlvmCompiler compiler;
  require(engines == 1 && machines != 0,
          "The independent observer detects the later native materializer's engine and machines");
  const auto object = compiler.object(module);
  require(!object.empty() && compiler.native_bytes() == 0,
          "Final native O3 and object emission are separate from semantic lowering");
  control(std::bit_cast<ImageEntry>(compiler.install_object(object, artifact.name)));
  for (const auto &entry : artifact.entries) {
    control(std::bit_cast<ImageEntry>(compiler.resolve(entry)));
  }
}

void invalid_translations() {
  constexpr std::array words{static_cast<Word>(isa::Opcode::nop)};
  const std::array code{CodeSpan{.address = 0, .words = words}};
  constexpr std::array<Word, 1> entries{0};
  const auto plan = plan_image(code, entries);
  const ImagePlan empty;
  const std::array valid{ImageTranslation{.name = "first", .plan = plan}};
  const std::array duplicate{valid.front(), valid.front()};
  const std::string embedded("first\0other", sizeof("first\0other") - 1);
  const std::array nul{valid.front(), ImageTranslation{.name = embedded, .plan = plan}};
  const std::array incomplete{valid.front(), ImageTranslation{.name = "other", .plan = empty}};
  const auto rejects = [](const std::string_view name,
                          const std::span<const ImageTranslation> translations) {
    bool rejected = false;
    try {
      static_cast<void>(lower_images(name, translations));
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Incomplete, duplicate or ambiguous semantic exports fail during lowering");
  };
  rejects("first", {});
  rejects("", valid);
  rejects("other", valid);
  rejects("first", duplicate);
  rejects("first", nul);
  rejects("first", incomplete);
}
}

// Observe our linked compiler's real LLVM API calls, without production counters
// or changes to LLVM. The positive materialization control verifies interception.
extern "C" LLVMErrorRef LLVMOrcCreateLLJIT(LLVMOrcLLJITRef *const result,
                                           LLVMOrcLLJITBuilderRef builder) {
  ++engines;
  return original<decltype(&LLVMOrcCreateLLJIT)>("LLVMOrcCreateLLJIT")(result, builder);
}

extern "C" LLVMTargetMachineRef LLVMCreateTargetMachine(LLVMTargetRef target,
                                                        const char *triple,
                                                        const char *cpu,
                                                        const char *features,
                                                        LLVMCodeGenOptLevel level,
                                                        LLVMRelocMode reloc,
                                                        LLVMCodeModel model) {
  ++machines;
  return original<decltype(&LLVMCreateTargetMachine)>("LLVMCreateTargetMachine")(
      target, triple, cpu, features, level, reloc, model);
}

int main() {
  try {
    invalid_translations();
    const auto artifact = detached();
    materialize(artifact);
    std::cout << test::checks << " owned semantic image lowering checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
