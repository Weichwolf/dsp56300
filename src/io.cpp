// SPDX-License-Identifier: GPL-3.0-only
#include "io.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <cstddef>
#include <cstdint>

#ifdef DSP56300_BINDING_IR
#define DSP56300_IO_ENTRY [[gnu::noinline]]
#else
#define DSP56300_IO_ENTRY [[gnu::always_inline]]
#endif

extern "C" DSP56300_IO_ENTRY bool dsp56300_io_read(dsp56300::Io &io,
                                                   const dsp56300::Space space,
                                                   const dsp56300::Word address,
                                                   const dsp56300::Word pc,
                                                   dsp56300::Word &value) noexcept {
  return io.read(space, address, pc, value);
}

extern "C" DSP56300_IO_ENTRY bool dsp56300_io_write(dsp56300::Io &io,
                                                    const dsp56300::Space space,
                                                    const dsp56300::Word address,
                                                    const dsp56300::Word pc,
                                                    const dsp56300::Word value) noexcept {
  return io.write(space, address, pc, value);
}

extern "C" DSP56300_IO_ENTRY bool dsp56300_io_bit(dsp56300::Io &io,
                                                  const dsp56300::Space space,
                                                  const dsp56300::Word address,
                                                  const dsp56300::Word pc,
                                                  const dsp56300::Word bit,
                                                  const dsp56300::BitAction action,
                                                  bool &previous) noexcept {
  return io.bit(space, address, pc, bit, action, previous);
}

extern "C" DSP56300_IO_ENTRY bool dsp56300_io_reset(dsp56300::Io &io,
                                                    const dsp56300::Word pc) noexcept {
  return io.reset_peripherals(pc);
}

extern "C" DSP56300_IO_ENTRY bool dsp56300_io_pending(const dsp56300::Io &io,
                                                      const dsp56300::Word status) noexcept {
  return io.interrupt_pending(status);
}

extern "C" DSP56300_IO_ENTRY std::uint64_t dsp56300_io_revision(const dsp56300::Io &io) noexcept {
  return io.program_revision();
}

extern "C" DSP56300_IO_ENTRY const dsp56300::MemoryWindow *
dsp56300_io_windows(dsp56300::Io &io,
                    const dsp56300::Space space,
                    const dsp56300::Word mode,
                    std::size_t &count) noexcept {
  const auto windows = io.memory_windows(space, mode);
  count = windows.size();
  return windows.data();
}

#undef DSP56300_IO_ENTRY
