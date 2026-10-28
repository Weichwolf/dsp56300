// SPDX-License-Identifier: GPL-3.0-only
// Independent overlapping window/FFT/power expectations and direct DFT probes.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "fft_model.hpp"
#include "fft_program.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "spectral_stream_program.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test::spectral_stream {
namespace {
constexpr Word sentinel = 0x5a36c9;
constexpr std::int64_t amplitude_bound = fft::half_scale / 16;
constexpr double maximum_dft_error = 32.0, maximum_power_error = 4.0;
using Samples = std::vector<fft::Sample>;
using Inputs = std::vector<Samples>;

struct Input {
  Word hops = 0, mode = 0, flags = 0;
};

struct Point {
  Word step = 0, channel = 0;
};

fft::Sample signal(const Point input) {
  const auto phase = ((input.step * (input.channel + 3)) + (input.channel * 11)) % size;
  auto value = fft::coefficient(phase);
  for (std::size_t part = 0; part < value.size(); ++part) {
    value[part] /= 32;
    const auto dither = static_cast<std::int64_t>((input.step + input.channel + part) % 7) - 3;
    value[part] += dither;
  }
  return value;
}

std::vector<Word> weights() {
  std::vector<Word> values;
  values.reserve(size);
  for (Word index = 0; index < size; ++index) {
    values.push_back(static_cast<Word>((fft::half_scale - fft::coefficient(index)[0]) / 2));
  }
  return values;
}

Inputs inputs(const Input input) {
  Inputs result;
  result.reserve(channels);
  for (Word channel = 0; channel < channels; ++channel) {
    result.emplace_back(history_words + (input.hops * hop));
    for (Word frame = 0; frame < input.hops * hop; ++frame) {
      result.back().at(frame + history_words) = signal({.step = frame, .channel = channel});
    }
  }
  return result;
}

void setup(BinaryCase &item, const Input input, const Program &code, const Inputs &data) {
  item.initial = initial_state();
  item.initial.sr = input.mode | input.flags;
  item.initial.m[1] = power_words - 1;
  item.initial.n[7] = input.hops;
  item.memory_words = {
      2048, input_base + (channels * input_stride), input_base + (channels * input_stride)};
  item.budget = 1 + static_cast<Word>(input.hops * code.hop_progress.instructions);
  item.dispatches = item.budget;
  item.progress = {.instructions = item.budget,
                   .estimated_cycles = 5 + (input.hops * code.hop_progress.estimated_cycles)};
  item.before = {
      {.words = code.code.words, .address = start},
      {.words = weights(), .address = window_base, .space = Space::x},
      {.words = std::vector<Word>(power_words, sentinel), .address = power_base, .space = Space::x},
      {.words = {power_base}, .address = power_cursor_address, .space = Space::x},
      {.words = {sentinel}, .address = progress_address, .space = Space::x}};
  std::vector<Word> cursors;
  cursors.reserve(channels);
  for (Word channel = 0; channel < channels; ++channel) {
    cursors.push_back(input_base + (channel * input_stride));
  }
  item.before.push_back({.words = std::move(cursors), .address = cursor_base, .space = Space::x});
  for (const auto address : {cursor_base - 1,
                             cursor_base + channels,
                             progress_address + 1,
                             window_base - 1,
                             window_base + size,
                             power_base - 1,
                             power_base + power_words}) {
    item.before.push_back({.words = {sentinel}, .address = address, .space = Space::x});
  }
  for (const auto space : {Space::x, Space::y}) {
    const auto part = space == Space::x ? 0U : 1U;
    std::vector<Word> factors;
    factors.reserve(size / 2);
    for (Word index = 0; index < size / 2; ++index) {
      factors.push_back(fft::word(fft::coefficient(index)[part]));
    }
    item.before.push_back(
        {.words = std::move(factors), .address = fft::twiddle_base, .space = space});
    for (const auto base : {fft::data_base, fft::output_base}) {
      item.before.push_back(
          {.words = std::vector<Word>(size, sentinel), .address = base, .space = space});
      for (const auto address : {base - 1, base + size}) {
        item.before.push_back({.words = {sentinel}, .address = address, .space = space});
      }
    }
    for (const auto address : {fft::twiddle_base - 1, fft::twiddle_base + (size / 2)}) {
      item.before.push_back({.words = {sentinel}, .address = address, .space = space});
    }
    for (Word channel = 0; channel < channels; ++channel) {
      std::vector<Word> words;
      words.reserve(data.at(channel).size());
      for (const auto value : data.at(channel)) { words.push_back(fft::word(value[part])); }
      const auto base = input_base + (channel * input_stride);
      const auto count = static_cast<Word>(words.size());
      item.before.push_back({.words = std::move(words), .address = base, .space = space});
      for (const auto address : {base - 1, base + count}) {
        item.before.push_back({.words = {sentinel}, .address = address, .space = space});
      }
    }
  }
}

Samples windowed(const Samples &input, const Point point, const Word mode) {
  const auto window = weights();
  Samples result(size);
  for (Word index = 0; index < size; ++index) {
    const auto sample = input.at((point.step * hop) + index);
    for (std::size_t part = 0; part < sample.size(); ++part) {
      const auto value = fft::nearest({.numerator = sample[part] * window.at(index),
                                       .divisor = fft::half_scale,
                                       .rm = mode != 0});
      require(value >= -amplitude_bound && value <= amplitude_bound,
              "Windowed components stay inside the declared no-growth/no-clipping domain");
      result[index][part] = value;
    }
  }
  return result;
}

struct Witness {
  double dft_error = 0, power_error = 0;
  std::uint64_t probes = 0, ties = 0;
};

void direct_dft(const Samples &input,
                const fft::Model &model,
                const Point point,
                Witness &witness) {
  const auto tone = point.channel + 3;
  const std::array<Word, 8> bins{
      0, 1, tone, size - tone, size - tone - 1, size - tone + 1, size / 2, size - 1};
  for (const auto bin : bins) {
    std::complex<double> reference{};
    for (Word index = 0; index < size; ++index) {
      const auto angle = -2 * std::numbers::pi * static_cast<double>(index * bin) / size;
      reference += std::complex<double>{static_cast<double>(input[index][0]),
                                        static_cast<double>(input[index][1])} *
                   std::complex<double>{std::cos(angle), std::sin(angle)};
    }
    reference /= size;
    const auto actual = model.data.at(fft::reversed(bin, size));
    witness.dft_error = std::max({witness.dft_error,
                                  std::abs(reference.real() - static_cast<double>(actual[0])),
                                  std::abs(reference.imag() - static_cast<double>(actual[1]))});
    const auto expected_power = std::norm(reference) / (2 * static_cast<double>(fft::half_scale));
    const auto rounded_power =
        fft::nearest({.numerator = (actual[0] * actual[0]) + (actual[1] * actual[1]),
                      .divisor = fft::accumulator_unit,
                      .rm = model.rm});
    witness.power_error = std::max(witness.power_error,
                                   std::abs(expected_power - static_cast<double>(rounded_power)));
    ++witness.probes;
  }
  require(witness.dft_error <= maximum_dft_error && witness.power_error <= maximum_power_error,
          "Quantized streaming FFT and mean IQ power agree with independent direct DFT probes");
}

struct Model {
  Registers state;
  fft::Model transform{};
  std::vector<Word> power = std::vector<Word>(power_words, sentinel);
  Word hop_status = 0, fft_status = 0;
};

void spectrum(Model &model, const Samples &input, const Point point, Witness &witness) {
  auto samples = windowed(input, point, model.state.sr & mode_rm);
  model.state.b = fft::raw_accumulator(samples.back()[1]);
  sequence_model::flags(model.state, {.full = signed56(model.state.b)});
  model.transform = fft::transform(samples, model.state.sr);
  require(model.transform.clips == 0 &&
              (model.transform.status & (flag_s | flag_l)) == (model.state.sr & (flag_s | flag_l)),
          "The bounded streaming FFT introduces no new transfer growth or clipping");
  witness.ties += model.transform.ties;
  model.fft_status = model.transform.status | fft::loop_flag;
  model.state.sr = model.fft_status;
  model.state.b = fft::raw_accumulator(model.transform.b);
  constexpr Word short_hops = 3;
  if (input.size() <= history_words + (short_hops * hop) || point.step == 0 ||
      point.step == hops_per_second || point.step + 1 == stream_hops) {
    direct_dft(samples, model.transform, point, witness);
  }
  constexpr Word shift = 0x22;
  constexpr Word round = 0x11;
  for (Word bin = 0; bin < size; ++bin) {
    const auto value = model.transform.data.at(fft::reversed(bin, size));
    model.state.a = static_cast<std::uint64_t>(2 * ((value[0] * value[0]) + (value[1] * value[1])));
    sequence_model::alu(model.state, model.state, shift);
    sequence_model::alu(model.state, model.state, round);
    const auto power = register_model::read(model.state, 14);
    const auto position = ((((point.step * channels) + point.channel) * size) + bin) % power_words;
    model.power.at(position) = power;
  }
}

Model analyze(const Input input, const Inputs &data, Witness &witness) {
  Model model{.state = initial_state()};
  model.state.sr = input.mode | input.flags | fft::loop_flag;
  for (Word step = 0; step < input.hops; ++step) {
    model.hop_status = model.state.sr;
    for (Word channel = 0; channel < channels; ++channel) {
      spectrum(model, data.at(channel), {.step = step, .channel = channel}, witness);
    }
  }
  return model;
}

void finish(BinaryCase &item, Model model, const Program &code) {
  auto &state = item.expected;
  state = item.initial;
  const auto count = item.initial.n[7];
  state.a = model.state.a;
  state.b = model.state.b;
  state.x = (std::uint64_t{sign24 - 1} << word_bits) | 1;
  state.y = fft::word(model.transform.data.back()[1]);
  state.sr = model.state.sr & ~fft::loop_flag;
  state.pc = code.hop_end + 1;
  state.r[0] = state.r[4] = fft::output_base + size;
  state.r[1] = power_base + (((count * channels) * size) % power_words);
  state.r[3] = state.r[7] = cursor_base + channels;
  state.r[5] = fft::output_base + size;
  for (const auto index : {0U, 2U, 4U, 6U}) { state.n[index] = size / 2; }
  for (const auto index : {1U, 5U}) { state.n[index] = 1; }
  state.r[2] = state.r[6] = fft::twiddle_base + (size / 2);
  state.stack[2] = (std::uint64_t{code.hop_body} << word_bits) | item.initial.sr;
  state.stack[3] = (std::uint64_t{code.hop_end} << word_bits) | 1;
  state.stack[4] = (std::uint64_t{code.channel_body} << word_bits) | model.hop_status;
  state.stack[5] = (std::uint64_t{code.channel_end} << word_bits) | 1;
  state.stack[6] = (std::uint64_t{code.power_body} << word_bits) | model.fft_status;
  state.stack[7] = (std::uint64_t{code.transform.stages.back().outer_end} << word_bits) | 1;
  state.stack[8] = (std::uint64_t{code.transform.stages.back().inner_body} << word_bits) |
                   model.transform.last_inner_status;
  std::vector<Word> cursors;
  cursors.reserve(channels);
  for (Word channel = 0; channel < channels; ++channel) {
    cursors.push_back(input_base + (channel * input_stride) + (count * hop));
  }
  item.after = {{.words = std::move(cursors), .address = cursor_base, .space = Space::x},
                {.words = {state.r[1]}, .address = power_cursor_address, .space = Space::x},
                {.words = {1}, .address = progress_address, .space = Space::x},
                {.words = std::move(model.power), .address = power_base, .space = Space::x}};
  for (const auto space : {Space::x, Space::y}) {
    const auto part = space == Space::x ? 0U : 1U;
    std::vector<Word> scratch;
    std::vector<Word> output;
    scratch.reserve(size);
    output.reserve(size);
    for (Word index = 0; index < size; ++index) {
      scratch.push_back(fft::word(model.transform.data.at(index)[part]));
      output.push_back(fft::word(model.transform.data.at(fft::reversed(index, size))[part]));
    }
    item.after.push_back({.words = std::move(scratch), .address = fft::data_base, .space = space});
    item.after.push_back({.words = std::move(output), .address = fft::output_base, .space = space});
  }
}

BinaryCase fixture(const Input input, Witness &witness) {
  if (input.hops == 0 || input.hops > stream_hops) {
    throw std::invalid_argument("Invalid spectral stream length");
  }
  BinaryCase item;
  const auto code = program();
  const auto data = inputs(input);
  setup(item, input, code, data);
  finish(item, analyze(input, data, witness), code);
  item.name = "Continuous Hann/FFT/power channels=15 size=1024 rate=48000 hop=480 hops=" +
              std::to_string(input.hops) + " mode=" + std::to_string(input.mode) +
              " flags=" + std::to_string(input.flags);
  return item;
}
} // namespace
} // namespace dsp56300::test::spectral_stream

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_spectral_stream full|check FILE.d56");
    }
    using namespace dsp56300::test;
    spectral_stream::Witness witness;
    std::vector<BinaryCase> cases;
    if (std::string_view{argv[1]} == "full") {
      cases.push_back(spectral_stream::fixture({.hops = spectral_stream::stream_hops}, witness));
    } else if (std::string_view{argv[1]} == "check") {
      for (const auto mode : {0U, mode_rm}) {
        for (const auto flags : {0U, flag_c | flag_s | flag_l}) {
          for (const auto hops : {1U, 2U, 3U}) {
            cases.push_back(
                spectral_stream::fixture({.hops = hops, .mode = mode, .flags = flags}, witness));
          }
        }
      }
    } else {
      throw std::invalid_argument("Select full or check stream fixture");
    }
    write_fixture(argv[2], std::move(cases));
    std::cout << checks << " independent checks, " << witness.probes << " direct DFT/power probes, "
              << witness.ties << " FFT ties; max DFT error=" << witness.dft_error
              << " max power error=" << witness.power_error << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
