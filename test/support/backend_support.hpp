// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <concepts>
#include <cstdint>

namespace dsp56300 {
// Test workloads share one ordinary consumer, executing the production native core.
template <class Implementation>
concept CoreImplementation = std::constructible_from<Implementation, Buffers, Io *, TestOptions> &&
                             requires(Implementation &core,
                                      const Implementation &view,
                                      const Registers &registers,
                                      const std::uint32_t instructions) {
                               { core.set_registers(registers) } -> std::same_as<void>;
                               { view.registers() } -> std::same_as<Registers>;
                               { view.progress() } noexcept -> std::same_as<Progress>;
                               { core.run(instructions) } -> std::same_as<RunResult>;
                             };
}
