// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_io_fixture.hpp"

#include <cstddef>
#include <cstdint>

namespace {
dsp56300::test::ImageIo &device(dsp56300::Io *io, const dsp56300::test::IoEvent role) noexcept {
  auto &result = *static_cast<dsp56300::test::ImageIo *>(io);
  ++result.bound_calls[static_cast<unsigned>(role)];
  return result;
}

const dsp56300::test::ImageIo &device(const dsp56300::Io *io,
                                      const dsp56300::test::IoEvent role) noexcept {
  const auto &result = *static_cast<const dsp56300::test::ImageIo *>(io);
  ++result.bound_calls[static_cast<unsigned>(role)];
  return result;
}
}

extern "C" bool image_io_read(dsp56300::Io *io,
                              dsp56300::Space space,
                              dsp56300::Word address,
                              dsp56300::Word pc,
                              dsp56300::Word &value) noexcept {
  return device(io, dsp56300::test::IoEvent::read).read_body(space, address, pc, value);
}

extern "C" bool image_io_write(dsp56300::Io *io,
                               dsp56300::Space space,
                               dsp56300::Word address,
                               dsp56300::Word pc,
                               dsp56300::Word value) noexcept {
  return device(io, dsp56300::test::IoEvent::write).write_body(space, address, pc, value);
}

extern "C" bool image_io_bit(dsp56300::Io *io,
                             dsp56300::Space space,
                             dsp56300::Word address,
                             dsp56300::Word pc,
                             dsp56300::Word bit,
                             dsp56300::BitAction action,
                             bool &previous) noexcept {
  return device(io, dsp56300::test::IoEvent::bit)
      .bit_body(space, address, pc, bit, action, previous);
}

extern "C" bool image_io_reset(dsp56300::Io *io, dsp56300::Word pc) noexcept {
  return device(io, dsp56300::test::IoEvent::reset).reset_body(pc);
}

extern "C" bool image_io_pending(const dsp56300::Io *io, dsp56300::Word status) noexcept {
  return device(io, dsp56300::test::IoEvent::pending).pending_body(status);
}

extern "C" std::uint64_t image_io_revision(const dsp56300::Io *io) noexcept {
  return device(io, dsp56300::test::IoEvent::revision).revision_body();
}

extern "C" const dsp56300::MemoryWindow *image_io_windows(dsp56300::Io *io,
                                                          const dsp56300::Space space,
                                                          const dsp56300::Word mode,
                                                          std::size_t &count) noexcept {
  const auto windows = device(io, dsp56300::test::IoEvent::windows).windows_body(space, mode);
  count = windows.size();
  return windows.data();
}
