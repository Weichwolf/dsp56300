// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "jit/llvm/runtime.hpp"

#include <string_view>

namespace llvm::object {
class ObjectFile;
}

namespace dsp56300::detail {
struct ImageLifecycle {
  bool initialize = false, finalize = false;
};

void export_lifecycle(LlvmModule &module);
[[nodiscard]] ImageLifecycle
object_lifecycle(const llvm::object::ObjectFile &object, std::string_view entry, char prefix);
void bind_lifecycle(LlvmModule &module, ImageLifecycle lifecycle, std::string_view entry);
}
