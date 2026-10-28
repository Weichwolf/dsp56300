// SPDX-License-Identifier: GPL-3.0-only
// Independent radix-2 integer recurrence and direct complex DFT qualification.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "fft_model.hpp"
#include "fft_program.hpp"
#include "fft_twiddles.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <exception>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test::fft {
namespace {
constexpr Word ram_words = 0x4000, p_words = 2048;
constexpr Word initial_flags = flag_c | flag_v | flag_z | flag_n | flag_e | flag_u;
constexpr Word guard = 0x0badf0, output_poison = 0x13579b;
constexpr double maximum_dft_error = 32.0;

double verify_dft(const std::vector<Sample> &input, const Model &model) {
  const auto size = static_cast<Word>(input.size());
  double maximum = 0;
  for (Word bin = 0; bin < size; ++bin) {
    std::complex<double> expected{};
    for (Word index = 0; index < size; ++index) {
      const auto angle = -2 * std::numbers::pi * static_cast<double>(index * bin) / size;
      expected += std::complex<double>(static_cast<double>(input[index][0]),
                                       static_cast<double>(input[index][1])) *
                  std::complex<double>(std::cos(angle), std::sin(angle));
    }
    expected /= size;
    const auto &actual = model.data[reversed(bin, size)];
    maximum = std::max({maximum,
                        std::abs(expected.real() - static_cast<double>(actual[0])),
                        std::abs(expected.imag() - static_cast<double>(actual[1]))});
  }
  require(maximum <= maximum_dft_error, "Fixed FFT agrees with independent normalized DFT");
  return maximum;
}

struct Configuration {
  Word size = 0;
  unsigned signal = 0;
  bool rm = false, sticky = false;
};

struct Witness {
  std::uint64_t clips = 0, ties = 0;
  double maximum_error = 0;
};

void memory(BinaryCase &item, const std::vector<Sample> &input, const Model &model) {
  const auto size = static_cast<Word>(input.size());
  for (const auto space : {Space::x, Space::y}) {
    const auto part = space == Space::x ? 0U : 1U;
    std::vector<Word> before;
    std::vector<Word> after;
    std::vector<Word> output;
    std::vector<Word> factors;
    before.reserve(size);
    after.reserve(size);
    output.reserve(size);
    factors.reserve(size / 2);
    for (Word index = 0; index < size; ++index) {
      before.push_back(word(input[index][part]));
      after.push_back(word(model.data[index][part]));
      output.push_back(word(model.data[reversed(index, size)][part]));
    }
    for (Word index = 0; index < size / 2; ++index) {
      factors.push_back(word(coefficient(index * maximum_size / size)[part]));
    }
    item.before.push_back({.words = std::move(before), .address = data_base, .space = space});
    item.before.push_back({.words = std::move(factors), .address = twiddle_base, .space = space});
    item.before.push_back(
        {.words = std::vector<Word>(size, output_poison), .address = output_base, .space = space});
    for (const auto address : {data_base - 1,
                               data_base + size,
                               twiddle_base - 1,
                               twiddle_base + (size / 2),
                               output_base - 1,
                               output_base + size}) {
      item.before.push_back({.words = {guard}, .address = address, .space = space});
    }
    item.after.push_back({.words = std::move(after), .address = data_base, .space = space});
    item.after.push_back({.words = std::move(output), .address = output_base, .space = space});
  }
}

BinaryCase fixture(const Configuration config, Witness &witness) {
  const auto program = make_program(config.size);
  const auto input = input_signal({.size = config.size, .kind = config.signal});
  BinaryCase item;
  item.name = "Own FFT DIF N=" + std::to_string(config.size) +
              " signal=" + std::to_string(config.signal) +
              " RM=" + std::to_string(static_cast<unsigned>(config.rm)) +
              " sticky=" + std::to_string(static_cast<unsigned>(config.sticky));
  item.memory_words = {p_words, ram_words, ram_words};
  item.initial = register_model::seed(0);
  item.initial.sp = item.initial.sc = 0;
  item.initial.sr =
      initial_flags | (config.rm ? mode_rm : 0) | (config.sticky ? flag_s | flag_l : 0);
  const auto model = transform(input, item.initial.sr);
  witness.clips += model.clips;
  witness.ties += model.ties;
  if (config.signal < 4 || config.signal == 6) {
    require(model.clips == 0, "Low amplitude DFT qualification never clips");
    witness.maximum_error = std::max(witness.maximum_error, verify_dft(input, model));
  }
  item.expected = item.initial;
  item.expected.a = raw_accumulator(model.a);
  item.expected.b = raw_accumulator(model.b);
  item.expected.x = (std::uint64_t{word_mask / 2} << word_bits) | word(model.data.back()[0]);
  item.expected.y = word(model.data.back()[1]);
  for (const auto index : {0U, 4U}) {
    item.expected.r[index] = data_base;
    item.expected.n[index] = config.size / 2;
  }
  for (const auto index : {1U, 5U}) {
    item.expected.r[index] = output_base + config.size;
    item.expected.n[index] = 1;
  }
  for (const auto index : {2U, 6U}) {
    item.expected.r[index] = twiddle_base + config.size / 2;
    item.expected.n[index] = config.size / 2;
  }
  item.expected.stack[1] = (std::uint64_t{item.initial.la} << word_bits) | item.initial.lc;
  item.expected.stack[2] = (std::uint64_t{program.copy_body} << word_bits) | model.status;
  item.expected.stack[3] = (std::uint64_t{program.stages.back().outer_end} << word_bits) | 1;
  item.expected.stack[4] =
      (std::uint64_t{program.stages.back().inner_body} << word_bits) | model.last_inner_status;
  item.expected.pc = start + static_cast<Word>(program.words.size());
  item.expected.sr = model.status;
  item.progress = program.progress;
  item.dispatches = item.budget = static_cast<Word>(program.progress.instructions);
  item.before.push_back({.words = program.words, .address = start});
  memory(item, input, model);
  return item;
}

void generate(const char *const path) {
  std::vector<BinaryCase> cases;
  Witness witness;
  for (const Word size : {16U, 64U, 256U, 1024U}) {
    for (unsigned signal = 0; signal < 7; ++signal) {
      for (const bool rm : {false, true}) {
        for (const bool sticky : {false, true}) {
          cases.push_back(
              fixture({.size = size, .signal = signal, .rm = rm, .sticky = sticky}, witness));
        }
      }
    }
  }
  require(witness.clips != 0 && witness.ties != 0, "Directed FFT cases exercise clipping and ties");
  std::cout << cases.size() << " independent FFT fixtures, " << witness.clips << " clips, "
            << witness.ties << " rounding ties, maximum DFT error " << witness.maximum_error
            << '\n';
  write_fixture(path, std::move(cases));
}
} // namespace
} // namespace dsp56300::test::fft

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_fft OUTPUT.d56"); }
    dsp56300::test::fft::generate(argv[1]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
