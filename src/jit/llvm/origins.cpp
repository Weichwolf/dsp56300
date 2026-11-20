// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/origins.hpp"

#include "dsp56300/memory.hpp"

#include <cstdint>
#include <llvm/BinaryFormat/Dwarf.h>
#include <llvm/IR/DebugInfoMetadata.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Casting.h>
#include <string>

namespace dsp56300::detail {
namespace {
llvm::DICompileUnit *unit(llvm::Module &module) {
  const auto *units = module.getNamedMetadata("llvm.dbg.cu");
  return units != nullptr && units->getNumOperands() != 0
             ? llvm::dyn_cast<llvm::DICompileUnit>(units->getOperand(0))
             : nullptr;
}
}

Origins::Origins(llvm::Module &module, const std::uint64_t variant)
    : builder_(module, true, unit(module)),
      file_(builder_.createFile("core-variant-" + std::to_string(variant) + ".S", "")) {
  if (unit(module) == nullptr) {
    builder_.createCompileUnit(llvm::dwarf::DW_LANG_Mips_Assembler,
                               file_,
                               "dsp56300 LLVM",
                               true,
                               "",
                               0,
                               "",
                               llvm::DICompileUnit::LineTablesOnly);
  }
  if (module.getModuleFlag("Debug Info Version") == nullptr) {
    module.addModuleFlag(llvm::Module::Warning, "Debug Info Version", llvm::DEBUG_METADATA_VERSION);
  }
  if (module.getModuleFlag("Dwarf Version") == nullptr) {
    constexpr unsigned dwarf_version = 4;
    module.addModuleFlag(llvm::Module::Warning, "Dwarf Version", dwarf_version);
  }
}

Origins::~Origins() {
  builder_.finalize();
}

void Origins::function(llvm::Function &function) {
  function.setSubprogram(
      builder_.createFunction(file_,
                              function.getName(),
                              function.getName(),
                              file_,
                              0,
                              builder_.createSubroutineType(builder_.getOrCreateTypeArray({})),
                              0,
                              llvm::DINode::FlagZero,
                              llvm::DISubprogram::SPFlagDefinition));
}

llvm::DILocation *Origins::generated(const llvm::Function &function) {
  return llvm::DILocation::get(function.getContext(), 0, 0, function.getSubprogram());
}

llvm::DILocation *Origins::instruction(const llvm::Function &function, const Word pc) {
  return llvm::DILocation::get(function.getContext(), pc + 1, 0, function.getSubprogram());
}
}
