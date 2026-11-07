// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/memory.hpp"
#include "mac_batch.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dsp56300::detail {
bool native_mac_available() noexcept {
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

[[gnu::target("avx2")]] Products extend(const Raw lanes) noexcept {
  const auto values = std::bit_cast<Signed>(lanes << padding_bits) >> padding_bits;
  return __builtin_convertvector(values, Products);
}

[[gnu::target("avx2")]] Products operands(const Word *const words) noexcept {
  Raw lanes{};
  std::memcpy(&lanes, words, sizeof(lanes));
  return extend(lanes);
}

[[gnu::target("avx2")]] Products first_operands(const Word *const words, const Word old) noexcept {
  Raw lanes{};
  std::memcpy(&lanes, words, sizeof(lanes));
  lanes = __builtin_shufflevector(lanes, lanes, 0, 0, 1, 2);
  lanes[0] = old;
  return extend(lanes);
}

[[gnu::target("avx2")]] void
reduce(MacBatchState &state, const MacBatchInput &input, const unsigned count) noexcept {
  auto sums = first_operands(input.x.data(), state.x) * first_operands(input.y.data(), state.y);
  for (unsigned offset = mac_batch_width; offset < count; offset += mac_batch_width) {
    sums += operands(input.x.data() + offset - 1) * operands(input.y.data() + offset - 1);
  }
  const auto delta = 2 * __builtin_reduce_add(sums);
  state.accumulator += input.subtract ? -delta : delta;
  state.overflowed = false;
}

[[gnu::target("avx2")]] void prefixes(MacBatchState &state, const MacBatchInput &input) noexcept {
  auto current = state;
  const auto streams = input;
  for (std::size_t index = 0; index < streams.x.size(); index += mac_batch_width) {
    const auto x = first_operands(streams.x.data() + index, current.x);
    const auto y = first_operands(streams.y.data() + index, current.y);
    const auto products = x * y * 2;
    std::array<std::int64_t, mac_batch_width> values{};
    std::memcpy(values.data(), &products, sizeof(products));
    consume_mac_products(current, values, streams.subtract);
    current.x = streams.x[index + 3] & word_mask;
    current.y = streams.y[index + 3] & word_mask;
  }
  state = current;
}
}

[[gnu::target("avx2"), gnu::aligned(64)]] void
native_mac_batch(MacBatchState &state, const MacBatchInput &input) noexcept {
  auto current = state;
  const auto streams = input;
  for (std::size_t index = 0; index < streams.x.size();) {
    auto count = mac_batch_math::reduction_width(current.accumulator, streams.x.size() - index);
    if (count != mac_batch_width) {
      reduce(current,
             {.x = streams.x.subspan(index, count),
              .y = streams.y.subspan(index, count),
              .subtract = streams.subtract},
             count);
    } else {
      count = static_cast<unsigned>(
          std::min<std::size_t>(mac_reduction_width, streams.x.size() - index));
      prefixes(current,
               {.x = streams.x.subspan(index, count),
                .y = streams.y.subspan(index, count),
                .subtract = streams.subtract});
    }
    index += count;
    current.x = streams.x[index - 1] & word_mask;
    current.y = streams.y[index - 1] & word_mask;
  }
  state = current;
}
#endif
} // namespace dsp56300::detail
