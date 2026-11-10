// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <llvm-c/Target.h>
#include <mutex>
#include <stdexcept>

// Identical private definition in both static core libraries: one process-wide
// target registration, not one flag per ISA. Separate loaded DSOs need a shared
// runtime owner instead. Never call LLVMShutdown from an individual core.
namespace weichwolf::llvm19 {
inline std::once_flag native_target_once;

inline void initialize_native() {
  std::call_once(native_target_once, [] {
    if (LLVMInitializeNativeTarget() != 0 || LLVMInitializeNativeAsmPrinter() != 0 ||
        LLVMInitializeNativeAsmParser() != 0) {
      throw std::runtime_error("LLVM 19 native target initialization failed");
    }
  });
}
}
