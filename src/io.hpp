// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <cstdint>

// Public-Io signatures only: firmware semantics retain faults and completed effects.
extern "C" {
bool dsp56300_io_read(dsp56300::Io &io,
                      dsp56300::Space space,
                      dsp56300::Word address,
                      dsp56300::Word pc,
                      dsp56300::Word &value) noexcept;
bool dsp56300_io_write(dsp56300::Io &io,
                       dsp56300::Space space,
                       dsp56300::Word address,
                       dsp56300::Word pc,
                       dsp56300::Word value) noexcept;
bool dsp56300_io_bit(dsp56300::Io &io,
                     dsp56300::Space space,
                     dsp56300::Word address,
                     dsp56300::Word pc,
                     dsp56300::Word bit,
                     dsp56300::BitAction action,
                     bool &previous) noexcept;
bool dsp56300_io_reset(dsp56300::Io &io, dsp56300::Word pc) noexcept;
bool dsp56300_io_pending(const dsp56300::Io &io, dsp56300::Word status) noexcept;
std::uint64_t dsp56300_io_revision(const dsp56300::Io &io) noexcept;
}
