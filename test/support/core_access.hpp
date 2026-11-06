// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string_view>

namespace dsp56300 {
enum class Backend : std::uint8_t { jit };

struct TestOptions {
  Backend backend = Backend::jit;
  Word cacheable_begin = address_words, cacheable_end = address_words;
  unsigned cache_sector_words = 128;
};

// Ordinary public-API consumer. No privileged access to the core implementation.
class TestCore : public Core {
public:
  explicit TestCore(Memory memory, Io *io = nullptr, TestOptions options = {})
      : Core(configure(memory, options), io), buffers_(memory.storage()) {}

  explicit TestCore(Buffers buffers, Io *io = nullptr, TestOptions options = {})
      : TestCore(Memory::separate(buffers), io, options) {}

  explicit TestCore(Io &bus, TestOptions options = {})
      : TestCore(Memory::external(), &bus, options) {}

  [[nodiscard]] Progress progress() const noexcept { return state().progress; }

  [[nodiscard]] Fault fault() const noexcept { return state().failure; }

  [[nodiscard]] ExecutionState processing_state() const noexcept { return state().execution; }

  [[nodiscard]] Word read(Space space, Word address) const {
    const std::array regions{buffers_.program, buffers_.x, buffers_.y};
    const auto words = regions.at(static_cast<std::size_t>(space));
    if (address >= words.size()) {
      throw std::out_of_range("Test memory read exceeds its borrowed view");
    }
    return words[address] & word_mask;
  }

  void write(Space space, Word address, Word value) { load(space, address, std::span(&value, 1)); }

  void clear_code_cache() { load(Space::program, 0, {}); }

private:
  static Memory configure(const Memory memory, const TestOptions options) {
    auto buffers = memory.storage();
    buffers.cache = {.begin = options.cacheable_begin,
                     .end = options.cacheable_end,
                     .sector_words = options.cache_sector_words};
    return buffers.program.empty() ? Memory::external(buffers.cache) : Memory::separate(buffers);
  }

  Buffers buffers_;
};
}

namespace dsp56300::test {
constexpr std::string_view backend_name(const Backend /*backend*/) {
  return "jit";
}

}
