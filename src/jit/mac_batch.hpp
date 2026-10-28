// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/simd.hpp"
#include "jit.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::detail {
inline constexpr unsigned mac_batch_width = SimdLayout<64, 256>::lanes;

constexpr BatchBackend select_mac_batch(const BatchBackend requested, const bool avx2) noexcept {
  if (requested == BatchBackend::disabled) { return BatchBackend::disabled; }
  return requested != BatchBackend::scalar && avx2 ? BatchBackend::avx2 : BatchBackend::scalar;
}

struct MacBatchPlan {
  std::array<unsigned, 2> indices{};
  bool destination_b = false, subtract = false;
};

struct MacBatchInput {
  std::span<const Word> x, y;
  bool subtract = false;
};

struct MacBatchState {
  std::int64_t accumulator = 0;
  Word x = 0, y = 0;
  bool limited = false, overflowed = false;
};

[[nodiscard]] std::optional<MacBatchPlan> plan_mac_batch(const NativeOp &operation);

namespace mac_batch_math {
inline constexpr std::int64_t sign = std::int64_t{1} << (accumulator_bits - 1);
inline constexpr std::int64_t margin =
    mac_batch_width * (std::int64_t{1} << (double_word_bits - 1));
inline constexpr std::uint64_t safe_width = static_cast<std::uint64_t>((2 * (sign - margin)) - 1);

constexpr bool outside(const std::int64_t value) noexcept {
  return static_cast<std::uint64_t>(value + sign) > accumulator_mask;
}

constexpr std::int64_t wrap(const std::int64_t value) noexcept {
  const auto raw = static_cast<std::uint64_t>(value) & accumulator_mask;
  return std::bit_cast<std::int64_t>(raw << padding_bits) >> padding_bits;
}
}

inline void consume_mac_products(MacBatchState &state,
                                 std::array<std::int64_t, mac_batch_width> products,
                                 const bool subtract) noexcept {
  if (subtract) {
    for (auto &product : products) { product = -product; }
  }
  const auto first = state.accumulator + products[0];
  const auto second = first + products[1];
  const auto third = second + products[2];
  const auto fourth = third + products[3];
  // Each signed24 fractional product has magnitude <= 2^47. In this interval
  // no ordered prefix of four additions/subtractions can leave signed56.
  if (static_cast<std::uint64_t>(state.accumulator + mac_batch_math::sign -
                                 mac_batch_math::margin) <= mac_batch_math::safe_width) {
    state.accumulator = fourth;
    state.overflowed = false;
    return;
  }
  state.limited = state.limited || mac_batch_math::outside(first) ||
                  mac_batch_math::outside(second) || mac_batch_math::outside(third) ||
                  mac_batch_math::outside(fourth);
  state.overflowed = mac_batch_math::outside(mac_batch_math::wrap(third) + products[3]);
  state.accumulator = mac_batch_math::wrap(fourth);
}

void scalar_mac_batch(MacBatchState &state, const MacBatchInput &input) noexcept;
void avx2_mac_batch(MacBatchState &state, const MacBatchInput &input) noexcept;
[[nodiscard]] bool avx2_mac_available() noexcept;
[[nodiscard]] Word execute_mac_batch(Cpu &cpu,
                                     NativeContext &context,
                                     const NativeOp &operation,
                                     BatchBackend backend) noexcept;
} // namespace dsp56300::detail
