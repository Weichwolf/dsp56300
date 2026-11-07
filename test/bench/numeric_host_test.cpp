// SPDX-License-Identifier: GPL-3.0-only
// Independent quotient/remainder arithmetic and complete guarded bank memories.
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "iir_program.hpp"
#include "iir_stream_program.hpp"
#include "nonlinear_stream_program.hpp"
#include "numeric_host_asm.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::int64_t scale = std::int64_t{1} << word_bits;
constexpr std::int64_t unit = scale / 2;
constexpr std::int64_t boundary = unit * scale;
constexpr Word guard = 0xabfedcba, negative_half = 0xc00000;
constexpr Word input_poison = 0xab000000, coefficient_poison = 0xcd000000,
               state_poison = 0xef000000;
constexpr std::array<Word, 16> values{0,
                                      1,
                                      3,
                                      5,
                                      sign24,
                                      sign24 - 1,
                                      word_mask,
                                      0x400000,
                                      0xc00000,
                                      0x200000,
                                      0xe00000,
                                      2048,
                                      6144,
                                      0xfff800,
                                      0x123456,
                                      guard};

std::int64_t signed_word(const Word value) {
  const auto word = value % (word_mask + 1);
  return word < sign24 ? word : std::int64_t{word} - scale;
}

Word word(const std::int64_t value) {
  return static_cast<Word>(std::clamp(value, -unit, unit - 1)) & word_mask;
}

struct Quantizer {
  unsigned bits;
  bool rm;

  [[nodiscard]] std::int64_t rounded(const std::int64_t raw) const {
    const auto quantum = std::int64_t{1} << bits;
    auto quotient = raw / quantum;
    auto residue = raw % quantum;
    if (residue < 0) {
      --quotient;
      residue += quantum;
    }
    if (residue > quantum / 2 || (residue == quantum / 2 && (rm || quotient % 2 != 0))) {
      ++quotient;
    }
    return quotient;
  }

  [[nodiscard]] Word transfer(const std::int64_t raw) const { return word(rounded(raw)); }

  [[nodiscard]] Word driven(const std::int64_t raw) const {
    const auto quantum = std::int64_t{1} << bits;
    auto saturated = rounded(raw) * quantum;
    if (saturated >= boundary) { saturated = boundary - scale; }
    saturated = std::max(saturated, -boundary);
    auto result = saturated / quantum;
    if (saturated % quantum < 0) { --result; }
    return word(result);
  }
};

struct Bank {
  std::vector<Word> input, coefficients, state, output;
  Word progress = guard;
};

Bank seed(const bool nonlinear, const std::size_t phase) {
  const auto inputs = nonlinear ? nonlinear_stream::input_words : iir_stream::input_words;
  const auto coefficients =
      nonlinear ? nonlinear_stream::coefficient_words : iir_stream::coefficient_words;
  const auto states = nonlinear ? nonlinear_stream::state_words : iir_stream::state_words;
  Bank bank{.input = std::vector<Word>(inputs + 2, guard),
            .coefficients = std::vector<Word>(coefficients + 2, guard),
            .state = std::vector<Word>(states + 2, guard),
            .output = std::vector<Word>(iir_stream::output_words + 2, guard)};
  for (std::size_t index = 0; index < inputs; ++index) {
    bank.input[index + 1] = values[(index + phase) % values.size()] | input_poison;
  }
  for (std::size_t index = 0; index < coefficients; ++index) {
    bank.coefficients[index + 1] = values[(index + 3 * phase) % values.size()] | coefficient_poison;
  }
  for (std::size_t index = 0; index < states; ++index) {
    bank.state[index + 1] = values[(index * 3 + phase) % values.size()] | state_poison;
  }
  if (phase == values.size()) {
    std::fill(bank.input.begin() + 1, bank.input.end() - 1, sign24 | input_poison);
    std::fill(bank.state.begin() + 1, bank.state.end() - 1, Word{5} | state_poison);
    for (std::size_t index = 0; index < coefficients; ++index) {
      bank.coefficients[index + 1] = (index % 2 == 0 ? sign24 : negative_half) | coefficient_poison;
    }
  }
  return bank;
}

void nonlinear_frame(Bank &bank, const Quantizer quantizer, const std::size_t frame) {
  using namespace nonlinear_stream;
  for (std::size_t channel = 0; channel < channels; ++channel) {
    auto sample = signed_word(bank.input[1 + ((frame % signal_period) * channels) + channel]);
    for (std::size_t stage = 0; stage < stages; ++stage) {
      const auto slot = 1 + (channel * stages) + stage;
      const auto gain = signed_word(bank.coefficients[1 + (2 * stage)]);
      const auto feedback = signed_word(bank.coefficients[2 + (2 * stage)]);
      const auto product = std::clamp(2 * sample * gain, -boundary, boundary - 1);
      const auto value = quantizer.driven(product + (2 * signed_word(bank.state[slot]) * feedback));
      bank.state[slot] = value;
      sample = signed_word(value);
    }
    bank.output[1 + ((frame * channels + channel) % output_words)] = word(sample);
  }
}

void iir_frame(Bank &bank, const Quantizer quantizer, const std::size_t frame) {
  using namespace iir_stream;
  std::int64_t sum = 0;
  for (std::size_t channel = 0; channel < channels; ++channel) {
    auto sample = signed_word(bank.input[1 + ((frame % signal_period) * channels) + channel]);
    for (std::size_t stage = 0; stage < sections; ++stage) {
      const auto slot = 1 + (2 * (channel * sections + stage));
      const auto *const weights =
          bank.coefficients.data() + 1 + (stage * iir::coefficients_per_section);
      const auto output = quantizer.transfer((signed_word(bank.state[slot]) * scale) +
                                             (2 * sample * signed_word(weights[0])));
      const auto result = signed_word(output);
      const auto first = (signed_word(bank.state[slot + 1]) * scale) +
                         (2 * sample * signed_word(weights[1])) -
                         (2 * result * signed_word(weights[2]));
      const auto second =
          (2 * sample * signed_word(weights[3])) - (2 * result * signed_word(weights[4]));
      bank.state[slot] = quantizer.transfer(first);
      bank.state[slot + 1] = quantizer.transfer(second);
      sample = result;
    }
    sum += sample;
  }
  bank.output[1 + (frame % output_words)] = quantizer.transfer(sum * (scale / 128));
}

void check(const bool nonlinear,
           const Quantizer quantizer,
           const std::size_t phase,
           const std::size_t first,
           const std::span<const std::size_t> grants) {
  auto actual = seed(nonlinear, phase);
  auto expected = actual;
  const auto input = actual.input;
  const auto coefficients = actual.coefficients;
  std::size_t total = first;
  for (const auto grant : grants) { total += grant; }
  auto frame = first;
  for (const auto count : grants) {
    const numeric_host::Job job{.input = actual.input.data() + 1,
                                .coefficients = actual.coefficients.data() + 1,
                                .state = actual.state.data() + 1,
                                .output = actual.output.data() + 1,
                                .progress = &actual.progress,
                                .first = frame,
                                .frames = count,
                                .total = total};
    for (std::size_t index = 0; index < count; ++index, ++frame) {
      if (nonlinear) {
        nonlinear_frame(expected, quantizer, frame);
        expected.progress = static_cast<Word>(total - frame);
      } else {
        iir_frame(expected, quantizer, frame);
      }
    }
    if (nonlinear) {
      numeric_host::nonlinear_asm(job, quantizer.bits, quantizer.rm);
    } else {
      numeric_host::iir_asm(job, quantizer.bits, quantizer.rm);
    }
    require(actual.state == expected.state && actual.output == expected.output &&
                actual.progress == expected.progress && actual.input == input &&
                actual.coefficients == coefficients,
            "ASM bank matches independent quantization, all histories, ring outputs and guards");
  }
}

void saturation_contract() {
  for (const Word sample : values) {
    for (const Word gain : values) {
      const auto raw = 2 * signed_word(sample) * signed_word(gain);
      const auto expected = saturation_model::apply(
          static_cast<std::uint64_t>(raw) & accumulator_mask, {.status = saturation_model::mode});
      require(signed56(expected.value) == std::clamp(raw, -boundary, boundary - 1),
              "Unrounded product saturation preserves fractional low bits");
      for (const bool rm : {false, true}) {
        for (const unsigned bits : {word_bits - 1, word_bits, word_bits + 1}) {
          const Quantizer quantizer{.bits = bits, .rm = rm};
          const auto quantum = std::int64_t{1} << bits;
          const auto value = quantizer.rounded(raw) * quantum;
          const auto limited =
              saturation_model::apply(static_cast<std::uint64_t>(value) & accumulator_mask,
                                      {.status = saturation_model::mode, .rounded = true});
          const auto canonical = signed56(limited.value);
          auto quotient = canonical / quantum;
          if (canonical % quantum < 0) { --quotient; }
          require(quantizer.driven(raw) == word(quotient),
                  "Rounded SM limiting precedes scaled word transfer");
        }
      }
    }
  }
}

void run() {
  saturation_contract();
  constexpr std::array<std::size_t, 6> grants{0, 1, 2, 0, 31, 3};
  for (const bool nonlinear : {false, true}) {
    for (const bool rm : {false, true}) {
      for (const unsigned bits : {word_bits - 1, word_bits, word_bits + 1}) {
        for (std::size_t phase = 0; phase <= values.size(); ++phase) {
          for (const std::size_t first : {0U, 31U, 255U, 1023U}) {
            check(nonlinear, {.bits = bits, .rm = rm}, phase, first, grants);
          }
        }
      }
    }
  }
  std::cout << checks << " independent numeric ASM bank/guard checks passed\n";
}
}
}

int main() {
  try {
    dsp56300::test::run();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
