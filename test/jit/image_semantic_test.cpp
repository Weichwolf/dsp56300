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
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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

SemanticImage public_lower() {
  constexpr std::array words{static_cast<Word>(isa::Opcode::nop),
                             static_cast<Word>(isa::Opcode::stop)};
  const std::array code{CodeSpan{.address = 0, .words = words}};
  constexpr std::array<Word, 1> entries{0};
  const std::array variants{CodeVariant{.id = 1, .code = code, .entries = entries}};
  auto image = Core::lower({.variants = variants}, {});
  require(engines == 0 && machines == 0 && !image.bitcode().empty(),
          "Public lowering creates neither an execution engine nor a native target machine");
  return image;
}

void unused_declarations() {
  constexpr std::array words{static_cast<Word>(isa::Opcode::stop)};
  const std::array code{CodeSpan{.address = 0, .words = words}};
  constexpr std::array<Word, 1> entries{0};
  const std::array variants{CodeVariant{.id = 2, .code = code, .entries = entries}};
  constexpr std::array callbacks{
      CallbackBinding{.role = CallbackRole::read, .symbol = "unused_read"},
      CallbackBinding{.role = CallbackRole::bit, .symbol = "unused_bit"}};
  constexpr std::array invalid_host{std::byte{0}};
  const auto image = Core::lower({.variants = variants},
                                 {.memory = {},
                                  .interrupts = {},
                                  .transitions = {},
                                  .callbacks = callbacks,
                                  .bindings = invalid_host});
  require(engines == 0 && machines == 0 && image.exports().size() == 1,
          "Even unused selected roles lower without definitions, Core state or native compilation");
  LlvmModule module(std::string(image.exports().front().name));
  module.link_bitcode(image.bitcode());
  for (const auto &binding : callbacks) {
    const auto *function = llvm::unwrap(module.code())->getFunction(binding.symbol);
    require(function != nullptr && function->isDeclaration() && function->doesNotThrow(),
            "Unused public callback contracts remain typed external declarations in detached IR");
  }
  auto renamed = callbacks;
  renamed[0].symbol = "other_read";
  const auto other = Core::lower(
      {.variants = variants},
      {.memory = {}, .interrupts = {}, .transitions = {}, .callbacks = renamed, .bindings = {}});
  require(image.identity() != other.identity() &&
              image.exports().front().name != other.exports().front().name,
          "Selected callback contracts participate in component identity and symbol namespace");
}

void materialize(const ImageArtifact &artifact, const SemanticImage &guest) {
  LlvmModule module(artifact.name);
  module.link_bitcode(artifact.bitcode);
  module.link_bitcode(guest.bitcode());
  for (const auto &entry : artifact.entries) {
    const auto *function = llvm::unwrap(module.code())->getFunction(entry);
    require(function != nullptr && !function->isDeclaration() && function->hasExternalLinkage() &&
                function->hasFnAttribute("dsp56300-image"),
            "Fresh-context import retains every independently named ordinary/vector definition");
  }
  require(engines == 0 && machines == 0,
          "Importing an expired producer's artifact still creates no native compiler");
  auto compiler = std::make_shared<LlvmCompiler>();
  require(engines == 1 && machines != 0,
          "The independent observer detects the later native materializer's engine and machines");
  const auto object = compiler->object(module);
  require(!object.empty() && compiler->native_bytes() == 0,
          "Final native O3 and object emission are separate from semantic lowering");
  control(std::bit_cast<ImageEntry>(compiler->install_object(object, artifact.name)));
  for (const auto &entry : artifact.entries) {
    control(std::bit_cast<ImageEntry>(compiler->resolve(entry)));
  }
  std::vector<NativeSymbol> addresses;
  for (const auto &entry : guest.exports()) {
    addresses.push_back(
        {.name = entry.name, .address = compiler->resolve(std::string(entry.name))});
  }
  const auto before_engines = engines;
  const auto before_machines = machines;
  const auto native = Core::bind(guest,
                                 {.owner = compiler,
                                  .symbols = addresses,
                                  .identity = guest.identity(),
                                  .native_bytes = compiler->native_bytes()});
  require(engines == before_engines && machines == before_machines &&
              native.report().prepared_entries == 2 &&
              native.report().native_bytes == compiler->native_bytes(),
          "Public binding retains checked code without creating another compiler or owner");
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
    unused_declarations();
    const auto guest = public_lower();
    const auto artifact = detached();
    materialize(artifact, guest);
    std::cout << test::checks << " owned semantic image lowering checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
