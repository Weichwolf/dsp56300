// SPDX-License-Identifier: GPL-3.0-only
#include "direct.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>

namespace dsp56300::test::native {
Direct::Direct(const std::span<const std::uint8_t> code) {
  constexpr std::size_t maximum_code_size = std::size_t{1} << 20;
  if (code.empty() || code.size() > maximum_code_size) {
    throw std::out_of_range("Direct test code size");
  }
  arena_ = std::make_unique<detail::ExecutableArena>();
  entry_ = arena_->append(code);
}

Direct::~Direct() = default;

std::uint32_t Direct::enter(detail::Cpu &cpu, detail::NativeContext &context) const noexcept {
  return detail::enter_block(entry_, cpu, context);
}
}
