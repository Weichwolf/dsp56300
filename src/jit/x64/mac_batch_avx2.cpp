// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "mac_batch.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dsp56300::detail {
bool avx2_mac_available() noexcept {
#if defined(__x86_64__)
  return __builtin_cpu_supports("avx2");
#else
  return false;
#endif
}

#if defined(__x86_64__)
namespace {
using Raw = Word __attribute__((ext_vector_type(mac_batch_width)));
using Signed = std::int32_t __attribute__((ext_vector_type(mac_batch_width)));
using Products = std::int64_t __attribute__((ext_vector_type(mac_batch_width)));

[[gnu::target("avx2")]] Products operands(const Word *const words, const Word old) noexcept {
  Raw lanes{};
  std::memcpy(&lanes, words, sizeof(lanes));
  lanes = __builtin_shufflevector(lanes, lanes, 0, 0, 1, 2);
  lanes[0] = old;
  const auto values = std::bit_cast<Signed>(lanes << padding_bits) >> padding_bits;
  return __builtin_convertvector(values, Products);
}
}

[[gnu::target("avx2")]] void avx2_mac_batch(MacBatchState &state,
                                            const MacBatchInput &input) noexcept {
  auto current = state;
  const auto streams = input;
  for (std::size_t index = 0; index < streams.x.size(); index += mac_batch_width) {
    const auto x = operands(streams.x.data() + index, current.x);
    const auto y = operands(streams.y.data() + index, current.y);
    const auto products = x * y * 2;
    std::array<std::int64_t, mac_batch_width> values{};
    std::memcpy(values.data(), &products, sizeof(products));
    consume_mac_products(current, values, streams.subtract);
    current.x = streams.x[index + 3] & word_mask;
    current.y = streams.y[index + 3] & word_mask;
  }
  state = current;
}
#else
void avx2_mac_batch(MacBatchState &state, const MacBatchInput &input) noexcept {
  scalar_mac_batch(state, input);
}
#endif
} // namespace dsp56300::detail
