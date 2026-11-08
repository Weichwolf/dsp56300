// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/code.hpp"

namespace dsp56300::detail {
struct NativeIoOperand {
  Space space = Space::x;
  BusAction action = BusAction::read;
  x64::Reg address = x64::Reg::cx;
};

struct NativeIoExits {
  x64::Label &fallback;
  x64::Label &failure;
};

// RDX carries the request value and receives the result. The caller publishes
// its preceding state; this bridge performs only the typed access, not the ISA.
void native_io_access(x64::Code &code,
                      const BlockInstruction &operation,
                      NativeIoOperand operand,
                      NativeIoExits exits);
}
