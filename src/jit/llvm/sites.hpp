// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <vector>

namespace llvm::orc {
class LLJIT;
}

namespace dsp56300::detail {
// Cold native-provider data; binding copies it before execution begins.
struct NativeSites {
  void attach(llvm::orc::LLJIT &jit);
  [[nodiscard]] NativeSymbols::FaultMap map() const noexcept;
  std::vector<NativeSymbols::TextRange> text;
  std::vector<NativeSymbols::Site> sites;
};
}
