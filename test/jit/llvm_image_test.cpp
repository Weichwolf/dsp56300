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
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constant.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/InstrTypes.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
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
    require(dsp56300_image_enter(entry, nullptr, pc, 0, unlimited) ==
                (std::uint64_t{2} << reason_shift),
            "Zero quota returns before touching even a valid instruction's owner");
    require(dsp56300_image_enter(entry, nullptr, pc, 1, 0) == (std::uint64_t{3} << reason_shift),
            "An expired cycle window precedes code lookup without touching the owner");
  }
  for (const Word pc : {2U, 20U, 0x1000U, high + 1, invalid}) {
    require(dsp56300_image_enter(entry, nullptr, pc, 1, 1) == (std::uint64_t{4} << reason_shift),
            "Unknown starts, null pages and out-of-range PCs fail before ISA execution");
  }
}

void direct_native_edges(const LlvmModule &module) {
  bool linked = false;
  for (const auto &function : *llvm::unwrap(module.code())) {
    for (const auto &block : function) {
      for (const auto &instruction : block) {
        const auto *call = llvm::dyn_cast<llvm::CallInst>(&instruction);
        const auto *callee = call != nullptr ? call->getCalledFunction() : nullptr;
        if (callee == nullptr || callee == &function ||
            !callee->getName().starts_with(module.name() + ".unit.")) {
          continue;
        }
        require(call->isMustTailCall() && llvm::isa<llvm::ConstantInt>(call->getArgOperand(1)),
                "Known native successors tail-transfer directly with their established PC");
        linked = true;
      }
    }
  }
  require(linked, "Ordinary static edges connect native blocks without the dynamic target table");
}

bool legacy_cache_access(const llvm::Instruction &instruction) {
  const auto *const access = instruction.getMetadata(llvm::LLVMContext::MD_tbaa);
  const auto *const base = access != nullptr && access->getNumOperands() != 0
                               ? llvm::dyn_cast<llvm::MDNode>(access->getOperand(0).get())
                               : nullptr;
  const auto *const name = base != nullptr && base->getNumOperands() != 0
                               ? llvm::dyn_cast<llvm::MDString>(base->getOperand(0).get())
                               : nullptr;
  return name != nullptr && name->getString().contains("detail3Jit4ImplE");
}

void prepared_program_store(const std::span<const std::byte> semantics) {
  constexpr Word destination = 48;
  constexpr std::array stores{opcode<isa::Opcode::movem_absolute_short> | (destination << 8) | 4U,
                              opcode<isa::Opcode::stop>};
  const std::array source{CodeSpan{.address = 0, .words = stores}};
  LlvmModule module("prepared_program_store");
  module.link_bitcode(semantics);
  lift_image(module, source, entries);
  specialize_image(module);
  LlvmCompiler compiler;
  require(!compiler.object(module).empty(), "Prepared program stores reach native object emission");
  for (const auto &function : *llvm::unwrap(module.code())) {
    for (const auto &block : function) {
      for (const auto &instruction : block) {
        const auto *call = llvm::dyn_cast<llvm::CallBase>(&instruction);
        const auto *callee = call != nullptr ? call->getCalledFunction() : nullptr;
        require(callee == nullptr || !callee->getName().contains("3Jit10invalidate"),
                "Prepared program stores cannot invalidate the unused ordinary native cache");
        if (legacy_cache_access(instruction)) {
          llvm::errs() << "Unexpected ordinary-cache access in " << function.getName() << ':'
                       << block.getName() << ": " << instruction << '\n';
        }
        require(!legacy_cache_access(instruction),
                "Prepared program stores cannot access inlined ordinary native cache state");
      }
    }
  }
}

void materialization(const std::span<const std::byte> semantics) {
  LlvmModule module("native_image_control");
  module.link_bitcode(semantics);
  require(llvm::unwrap(module.code())->getModuleIdentifier() == module.name(),
          "Semantic import preserves the unique native module initialization identity");
  lift_image(module, code, entries);
  direct_native_edges(module);
  specialize_image(module);
  for (const auto &function : *llvm::unwrap(module.code())) {
    require(!function.getName().starts_with(module.name() + ".semantic."),
            "Specialized effects are fully inlined; no per-instruction wrapper survives");
    for (const auto &block : function) {
      for (const auto &instruction : block) {
        const auto *call = llvm::dyn_cast<llvm::CallBase>(&instruction);
        const auto *callee = call != nullptr ? call->getCalledFunction() : nullptr;
        require(callee == nullptr || !callee->getName().contains("_Zdl"),
                "NOP, ordinary jumps/returns and STOP cannot deallocate instruction caches");
      }
    }
  }
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

std::size_t shared_encodings(const LlvmModule &module) {
  std::size_t count = 0;
  for (const auto &function : *llvm::unwrap(module.code())) {
    if (function.hasFnAttribute("dsp56300-encoding")) { ++count; }
  }
  return count;
}

void combined_images(const std::span<const std::byte> semantics) {
  LlvmModule module("combined_images");
  module.link_bitcode(semantics);
  const auto plan = plan_image(code, entries);
  const std::array roots{module.name(),
                         module.name() + ".other",
                         module.name() + ".vector.first",
                         module.name() + ".vector.second"};
  lift_image(module, plan);
  const auto ordinary = shared_encodings(module);
  require(ordinary != 0, "An ordinary image creates explicit shared encoding definitions");
  lift_image(module, plan, roots[1]);
  require(shared_encodings(module) == ordinary,
          "Another code view reuses every identical ordinary encoding before expansion");
  constexpr Word vector_address = 32;
  const auto vector_plan =
      plan_vector(vector_address, {opcode<isa::Opcode::nop>, opcode<isa::Opcode::nop>}, plan);
  lift_vector(module, vector_plan, roots[2]);
  require(shared_encodings(module) == ordinary + 1,
          "Inserted-vector NOP effects remain distinct from ordinary NOP effects");
  lift_vector(module, vector_plan, roots[3]);
  require(shared_encodings(module) == ordinary + 1,
          "Another inserted-vector root reuses its matching encoding definition");
  specialize_image(module, roots);
  const auto &native = *llvm::unwrap(module.code());
  for (const auto &name : roots) {
    const auto *function = native.getFunction(name);
    require(function != nullptr && !function->isDeclaration() && !function->hasLocalLinkage(),
            "All declared ordinary and vector roots survive joint specialization");
  }
  for (const auto &function : native) {
    require(!function.hasFnAttribute("dsp56300-encoding"),
            "Shared encoding definitions disappear into complete native image functions");
  }
  LlvmCompiler exported;
  const auto object = exported.object(module);
  control(std::bit_cast<ImageEntry>(exported.install_object(object, roots[0])));
  for (std::size_t index = 1; index < roots.size(); ++index) {
    control(std::bit_cast<ImageEntry>(exported.resolve(roots[index])));
  }
  LlvmCompiler direct;
  control(std::bit_cast<ImageEntry>(direct.install(module)));
  for (std::size_t index = 1; index < roots.size(); ++index) {
    control(std::bit_cast<ImageEntry>(direct.resolve(roots[index])));
  }
}

void invalid_image_roots(const std::span<const std::byte> semantics) {
  LlvmModule module("invalid_image_roots");
  module.link_bitcode(semantics);
  lift_image(module, code, entries);
  auto &native = *llvm::unwrap(module.code());
  auto *type = llvm::FunctionType::get(llvm::Type::getVoidTy(native.getContext()), false);
  auto *host =
      llvm::Function::Create(type, llvm::GlobalValue::ExternalLinkage, "host_root", native);
  llvm::IRBuilder<> ir(llvm::BasicBlock::Create(native.getContext(), "entry", host));
  ir.CreateRetVoid();
  auto *declaration =
      llvm::Function::Create(type, llvm::GlobalValue::ExternalLinkage, "declared_root", native);
  declaration->addFnAttr("dsp56300-image", "1");
  const std::array<std::vector<std::string>, 5> rejected{std::vector<std::string>{},
                                                         {"unknown_root"},
                                                         {module.name(), module.name()},
                                                         {"host_root"},
                                                         {"declared_root"}};
  const auto before = module.bitcode();
  for (const auto &roots : rejected) {
    bool failed = false;
    try {
      specialize_image(module, roots);
    } catch (const std::invalid_argument &) { failed = true; }
    require(failed && module.bitcode() == before,
            "Invalid image-root inventories fail transactionally before internalization");
  }
  specialize_image(module);
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
  for (unsigned variant = 0; variant < 3; ++variant) {
    LlvmModule module("invalid_image_abi");
    module.link_bitcode(semantics);
    auto &native = *llvm::unwrap(module.code());
    if (variant == 1) {
      auto *function = native.getFunction("dsp56300_semantic_execute");
      require(function != nullptr, "The input contains the real semantic entry");
      function->deleteBody();
    } else if (variant == 2) {
      auto *previous = native.getFunction("dsp56300_semantic_execute");
      require(previous != nullptr, "The current private semantic ABI exists before replacement");
      previous->setName("retired_semantic_abi");
      auto *entry_block = llvm::BasicBlock::Create(native.getContext(), "entry");
      llvm::IRBuilder<> ir(entry_block);
      auto *type = llvm::FunctionType::get(ir.getInt64Ty(), {ir.getPtrTy(), ir.getPtrTy()}, false);
      auto *function = llvm::Function::Create(
          type, llvm::GlobalValue::ExternalLinkage, "dsp56300_semantic_execute", native);
      entry_block->insertInto(function);
      ir.CreateRet(ir.getInt64(0));
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
            "Missing semantics, incompatible layouts and retired signatures fail before image "
            "emission");
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
    combined_images(semantics);
    invalid_image_roots(semantics);
    prepared_program_store(semantics);
    std::cout << test::checks
              << " native image compiler/control checks passed; no DSP ISA "
                 "execution is claimed by this control-only test.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
