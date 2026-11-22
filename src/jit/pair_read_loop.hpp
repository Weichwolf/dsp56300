// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/memory.hpp"
#include "jit/image_plan.hpp"

#include <array>
#include <optional>

namespace dsp56300::detail {
struct PortAddress {
  Space space;
  Word address;
  bool operator==(const PortAddress &) const = default;
};

struct PairedPortRead {
  PortAddress data, strobe;
  Word end, bit;
  unsigned destination_register, retired, cycles;
  std::array<unsigned, 2> reads;
  std::array<unsigned, 4> edges;
  bool operator==(const PairedPortRead &) const = default;
};

// Recognize one finite DO body that packs two port reads into an indexed Y
// word. The image owns the opcodes. The consumer must independently admit
// the actual device effects over the returned strobe and time contract.
[[nodiscard]] std::optional<PairedPortRead> paired_port_read(const ImagePlan &image,
                                                             Word body) noexcept;

} // namespace dsp56300::detail
