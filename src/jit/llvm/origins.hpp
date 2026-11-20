// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"

#include <cstdint>
#include <llvm/IR/DIBuilder.h>

namespace llvm {
class DILocation;
class Function;
class Module;
}

namespace dsp56300::detail {
// Source attribution is line-table metadata, never an executed instruction.
class Origins {
public:
  Origins(llvm::Module &module, std::uint64_t variant);
  ~Origins();
  Origins(const Origins &) = delete;
  Origins &operator=(const Origins &) = delete;
  void function(llvm::Function &function);
  [[nodiscard]] static llvm::DILocation *generated(const llvm::Function &function);
  [[nodiscard]] static llvm::DILocation *instruction(const llvm::Function &function, Word pc);

private:
  llvm::DIBuilder builder_;
  llvm::DIFile *file_;
};
}
