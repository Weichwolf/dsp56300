// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/memory.hpp"
#include "native_region.hpp"
#include "x64.hpp"

#include <cstddef>

namespace dsp56300::detail {
inline constexpr std::size_t window_count_offset = 0, window_flag_offset = sizeof(Word);

void emit_region_loop_guard(x64::Code &code,
                            x64::Reg cpu,
                            const NativeRegion &region,
                            x64::Label &failed);
void emit_region_address_guards(x64::Code &code,
                                x64::Reg cpu,
                                const NativeRegion &region,
                                x64::Label &failed);

// Read-only for guest/context state. R12=Cpu, R13=NativeContext; the caller
// provides eight private stack bytes. On success [RSP] holds >=2 iterations.
void emit_region_window(x64::Code &code,
                        const NativeRegion &region,
                        Word operations,
                        x64::Label &failed);
} // namespace dsp56300::detail
