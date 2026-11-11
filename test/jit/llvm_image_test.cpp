// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/image.hpp"
#include "jit/llvm/runtime.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <llvm/IR/Constant.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/Module.h>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace dsp56300;
using namespace dsp56300::detail;
using test::require;
template <isa::Opcode operation> constexpr Word opcode = static_cast<Word>(operation);
constexpr Word helper = 8, high = 0x4000;
constexpr unsigned reason_shift = 32;
constexpr std::array first{opcode<isa::Opcode::jmp_absolute_short> | helper,
                           opcode<isa::Opcode::stop>};
constexpr std::array body{
    opcode<isa::Opcode::nop>, opcode<isa::Opcode::nop>, opcode<isa::Opcode::rts>};
constexpr std::array distant{opcode<isa::Opcode::stop>};
const std::array code{CodeSpan{.address = 0, .words = first},
                      CodeSpan{.address = helper, .words = body},
                      CodeSpan{.address = high, .words = distant}};
constexpr std::array<Word, 1> entries{0};

std::vector<std::byte> read(const char *const path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) { throw std::runtime_error("Cannot open semantic bitcode input"); }
  const std::streamoff size = file.tellg();
  if (size <= 0) { throw std::runtime_error("Empty semantic bitcode input"); }
  std::vector<std::byte> bytes(static_cast<std::size_t>(size));
  file.seekg(0);
  file.read(reinterpret_cast<char *>(bytes.data()), size);
  if (!file) { throw std::runtime_error("Incomplete semantic bitcode input"); }
  return bytes;
}

void control(const ImageEntry entry) {
  constexpr auto unlimited = std::numeric_limits<std::uint64_t>::max();
  constexpr auto invalid = std::numeric_limits<Word>::max();
  for (const Word pc : {0U, 1U, helper, helper + 1, helper + 2, high, 20U, 0x1000U, invalid}) {
    require(entry(nullptr, pc, 0, unlimited) == (std::uint64_t{2} << reason_shift),
            "Zero quota returns before touching even a valid instruction's owner");
    require(entry(nullptr, pc, 1, 0) == (std::uint64_t{3} << reason_shift),
            "An expired cycle window precedes code lookup without touching the owner");
  }
  for (const Word pc : {2U, 20U, 0x1000U, high + 1, invalid}) {
    require(entry(nullptr, pc, 1, 1) == (std::uint64_t{4} << reason_shift),
            "Unknown starts, null pages and out-of-range PCs fail before ISA execution");
  }
}

void materialization(const std::span<const std::byte> semantics) {
  LlvmModule module("native_image_control");
  module.link_bitcode(semantics);
  lift_image(module, code, entries);
  specialize_image(module);
  LlvmCompiler first_owner;
  const auto object = first_owner.object(module);
  require(!object.empty() && first_owner.native_bytes() == 0 && module.code() != nullptr,
          "Whole-image compilation exports a relocatable object without binding a Core");
  const auto first_entry =
      std::bit_cast<ImageEntry>(first_owner.install_object(object, module.name()));
  control(first_entry);
  {
    LlvmCompiler second_owner;
    control(std::bit_cast<ImageEntry>(second_owner.install_object(object, module.name())));
  }
  control(first_entry);
  LlvmCompiler direct_owner;
  control(std::bit_cast<ImageEntry>(direct_owner.install(module)));
  require(module.code() == nullptr, "Direct installation consumes the lifted semantic module");
}

void escaped_semantic(const std::span<const std::byte> semantics) {
  constexpr auto *canonical = "dsp56300_semantic_execute";
  constexpr auto *escaped = "\001dsp56300_semantic_execute";
  LlvmModule producer("escaped_semantic_producer");
  producer.link_bitcode(semantics);
  auto &source = *llvm::unwrap(producer.code());
  auto *entry = source.getFunction(canonical);
  require(entry != nullptr && !entry->isDeclaration(), "The producer retains the real ISA body");
  entry->setName(escaped);
  LlvmModule consumer("escaped_semantic_image");
  consumer.link_bitcode(producer.bitcode());
  auto &native = *llvm::unwrap(consumer.code());
  require(native.getFunction(canonical) != nullptr &&
              !native.getFunction(canonical)->isDeclaration() &&
              native.getFunction(escaped) == nullptr,
          "A target asm-label escape must import as the stable compiler semantic ABI symbol");
  lift_image(consumer, code, entries);
  specialize_image(consumer);
  LlvmCompiler compiler;
  control(std::bit_cast<ImageEntry>(compiler.install(consumer)));
  llvm::Function::Create(
      entry->getFunctionType(), llvm::GlobalValue::ExternalLinkage, canonical, source);
  LlvmModule rejected("ambiguous_semantic_image");
  const auto before = rejected.bitcode();
  bool failed = false;
  try {
    rejected.link_bitcode(producer.bitcode());
  } catch (const std::runtime_error &error) {
    failed = std::string_view(error.what()).contains("Ambiguous canonical and escaped");
  }
  require(failed, "Ambiguous semantic ABI aliases must fail for their exact naming conflict");
  require(rejected.bitcode() == before,
          "A failed semantic ABI import must retain the complete original compiler module");
}

void unprepared_source(const std::span<const std::byte> semantics) {
  LlvmModule module("invalid_image_source");
  module.link_bitcode(semantics);
  const auto before = module.bitcode();
  constexpr std::array jump{opcode<isa::Opcode::jmp_absolute_short> | (helper + 1)};
  const std::array missing{CodeSpan{.address = 0, .words = jump}};
  bool rejected = false;
  try {
    lift_image(module, missing, entries);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && module.bitcode() == before,
          "An undeclared encoded target fails before the semantic module is modified");
  rejected = false;
  try {
    specialize_image(module);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && module.bitcode() == before,
          "An unlifted module cannot be mistaken for a complete native image");
}

void rejected_abi(const std::span<const std::byte> semantics) {
  for (const bool missing : {false, true}) {
    LlvmModule module("invalid_image_abi");
    module.link_bitcode(semantics);
    auto &native = *llvm::unwrap(module.code());
    if (missing) {
      auto *function = native.getFunction("dsp56300_semantic_execute");
      require(function != nullptr, "The input contains the real semantic entry");
      function->deleteBody();
    } else {
      auto *metadata = native.getGlobalVariable("dsp56300_semantic_abi");
      require(metadata != nullptr, "The input contains the real semantic ABI metadata");
      metadata->setInitializer(llvm::Constant::getNullValue(metadata->getValueType()));
    }
    const auto before = module.bitcode();
    bool rejected = false;
    try {
      lift_image(module, code, entries);
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected && module.bitcode() == before,
            "Missing semantics and incompatible layouts fail before image emission");
  }
}
}

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::invalid_argument("Expected the normal semantic bitcode input"); }
    const auto semantics = read(argv[1]);
    unprepared_source(semantics);
    rejected_abi(semantics);
    escaped_semantic(semantics);
    materialization(semantics);
    std::cout << test::checks
              << " native image compiler/control checks passed; no DSP ISA "
                 "execution is claimed by this control-only test.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
