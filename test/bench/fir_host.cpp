// SPDX-License-Identifier: GPL-3.0-only
#include "fir_host.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fir_host_kernel.hpp"
#include "support.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace dsp56300::test::fir_host {
namespace {
constexpr unsigned sign48_position = 47;
constexpr Word changed_flags = flag_v | flag_z | flag_n | flag_e | flag_u;
constexpr Word loop_body = start + 3, after_program = start + 7;

void products_kernel(Kernel &state,
                     const std::span<const Word> data,
                     const std::span<const Word> coefficients) {
  // Intermediate N/Z/U/E/V are dead; L is sticky across every individual MAC.
  // Eight low padding bits turn signed-56 wrap and overflow into native ADD/OF.
  // Both spans are checked before entering assembly; no hidden vector overread.
  asm volatile("1:\n\t"
               "movslq (%[data],%[index],4), %[left]\n\t"
               "movslq (%[coefficients],%[index],4), %[right]\n\t"
               "shlq %[word_extension], %[left]\n\t"
               "shlq %[word_extension], %[right]\n\t"
               "sarq %[product_extension], %[left]\n\t"
               "sarq %[word_extension], %[right]\n\t"
               "imulq %[right], %[left]\n\t"
               "addq %[left], %[sum]\n\t"
               "seto %[overflow]\n\t"
               "movzbl %[overflow], %k[left]\n\t"
               "orq %[left], %[sticky]\n\t"
               "incq %[index]\n\t"
               "cmpq %[count], %[index]\n\t"
               "jb 1b"
               : [sum] "+&r"(state.accumulated.sum),
                 [sticky] "+&r"(state.accumulated.overflow),
                 [index] "+&r"(state.index),
                 [left] "=&r"(state.left),
                 [right] "=&r"(state.right),
                 [overflow] "=&q"(state.overflow)
               : [data] "r"(data.data()),
                 [coefficients] "r"(coefficients.data()),
                 [count] "r"(data.size()),
                 [word_extension] "i"(64 - word_bits),
                 [product_extension] "i"(64 - word_bits - (64 - accumulator_bits) - 1)
               : "cc", "memory");
}

Accumulator products(const std::span<const Word> data,
                     const std::span<const Word> coefficients,
                     const HostIsa host_isa) {
  Kernel state;
  constexpr std::size_t vector_words = 4;
  if (host_isa == HostIsa::avx2 && data.size() >= vector_words) {
    vector_products(state, data, coefficients);
  }
  if (state.index < data.size()) { products_kernel(state, data, coefficients); }
  state.accumulated.sum >>= 64 - accumulator_bits;
  return state.accumulated;
}

unsigned scaling(const Word status) {
  return word_bits + ((status & mode_down) != 0 ? 1U : 0U) - ((status & mode_up) != 0 ? 1U : 0U);
}

std::int64_t rounded(const Accumulator accumulated, const Word status) {
  const auto sum = accumulated.sum;
  const auto bits = scaling(status);
  const auto unit = std::uint64_t{1} << bits;
  const auto raw = static_cast<std::uint64_t>(sum);
  const auto remainder = raw & (unit - 1);
  const bool increment =
      remainder > unit / 2 ||
      (remainder == unit / 2 && ((status & mode_rm) != 0 || ((raw >> bits) & 1) != 0));
  return ((sum >> bits) + (increment ? 1 : 0)) * static_cast<std::int64_t>(unit);
}

Word finish(Registers &state, const Accumulator accumulated) {
  const auto full = rounded(accumulated, state.sr);
  const auto raw = static_cast<std::uint64_t>(full) & accumulator_mask;
  const auto value = signed56(raw);
  const auto bits = scaling(state.sr);
  const auto position = bits + sign48_position - word_bits;
  const auto boundary = std::int64_t{1} << position;
  const bool overflow =
      full < -static_cast<std::int64_t>(sign56) || full >= static_cast<std::int64_t>(sign56);
  auto status = state.sr & ~changed_flags;
  if (value == 0) { status |= flag_z; }
  if (value < 0) { status |= flag_n; }
  if (value < -boundary || value >= boundary) { status |= flag_e; }
  if ((((raw >> position) ^ (raw >> (position - 1))) & 1) == 0) { status |= flag_u; }
  if (overflow) { status |= flag_v; }
  const auto whole = value >> bits;
  constexpr auto minimum = -static_cast<std::int64_t>(sign24);
  constexpr auto maximum = static_cast<std::int64_t>(sign24 - 1);
  if (accumulated.overflow != 0 || overflow || whole < minimum || whole > maximum) {
    status |= flag_l;
  }
  const auto growth = bits + word_bits - 3;
  if ((((raw >> growth) ^ (raw >> (growth + 1))) & 1) != 0) { status |= flag_s; }
  state.a = raw;
  state.sr = status;
  return static_cast<Word>(std::clamp(whole, minimum, maximum)) & word_mask;
}
} // namespace

bool avx2_supported() noexcept {
  return __builtin_cpu_supports("avx2");
}

Word filter(Registers &state,
            const std::span<const Word> data,
            const std::span<const Word> coefficients,
            const HostIsa host_isa) {
  if (data.empty() || data.size() != coefficients.size() || data.size() > word_mask) {
    throw std::invalid_argument("FIR requires equally sized nonempty bounded inputs");
  }
  if (host_isa == HostIsa::avx2 && !avx2_supported()) {
    throw std::invalid_argument("AVX2 host execution requires qualified runtime support");
  }
  return finish(state, products(data, coefficients, host_isa));
}

Result execute(const Registers &initial,
               const std::span<const Word> data,
               const std::span<const Word> coefficients,
               const HostIsa host_isa) {
  Result result{.registers = initial};
  auto &state = result.registers;
  result.output = filter(state, data, coefficients, host_isa);
  const auto frame = initial.sp % stack_entries;
  state.stack[(frame + 1) % stack_entries] = (std::uint64_t{initial.la} << word_bits) | initial.lc;
  state.stack[(frame + 2) % stack_entries] =
      (std::uint64_t{loop_body} << word_bits) | ((initial.sr & ~changed_flags) | flag_z | flag_u);
  state.x = (initial.x & ~std::uint64_t{word_mask}) | (data.back() & word_mask);
  state.y = (initial.y & ~std::uint64_t{word_mask}) | (coefficients.back() & word_mask);
  const auto count = static_cast<Word>(data.size());
  state.r[0] = (initial.r[0] + count) & word_mask;
  state.r[4] = (initial.r[4] + count) & word_mask;
  state.pc = after_program;
  return result;
}
} // namespace dsp56300::test::fir_host
