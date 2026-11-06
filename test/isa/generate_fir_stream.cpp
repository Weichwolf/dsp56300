// SPDX-License-Identifier: GPL-3.0-only
// Independent prefix-moment convolution and direct-dot checks for persistent FIR.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fir_stream_program.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test::fir_stream {
namespace {
constexpr Word sentinel = 0x5a36c9, coefficient_unit = 128, signal_period = 4096;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::int64_t maximum_sample = (sign24 / 16) + 3;
constexpr std::int64_t coefficient_sum =
    std::int64_t{coefficient_unit} * ((taps + 1) / 2) * ((taps + 1) / 2);
constexpr auto maximum_raw = 2 * maximum_sample * coefficient_sum;
static_assert(maximum_raw < (std::int64_t{1} << (2 * word_bits - 4)));
static_assert(input_stride > stream_frames + taps);

struct Input {
  Word frames = 0, mode = 0, flags = 0;
};

struct Point {
  Word frame = 0, channel = 0;
};

Word signal(const Point input) {
  const Word phase = ((input.frame * (input.channel + 1)) + (input.channel * 7)) % signal_period;
  const auto triangle = phase < signal_period / 2 ? phase : signal_period - phase;
  const auto tone = (static_cast<std::int64_t>(triangle) - (signal_period / 4)) * (sign24 / 16384);
  const auto dither = static_cast<std::int64_t>((input.frame + (input.channel * 3)) % 7) - 3;
  return static_cast<Word>(tone + dither) & word_mask;
}

Word coefficient(const Word index) {
  return coefficient_unit * std::min(index + 1, taps - index);
}

struct Prefix {
  std::vector<std::int64_t> sum, moment;

  explicit Prefix(const std::vector<Word> &data) : sum(data.size() + 1), moment(data.size() + 1) {
    for (std::size_t index = 0; index < data.size(); ++index) {
      const auto value = signed24(data[index]);
      sum[index + 1] = sum[index] + value;
      moment[index + 1] = moment[index] + (static_cast<std::int64_t>(index) * value);
    }
  }

  [[nodiscard]] std::int64_t convolution(const Word frame) const {
    const std::size_t first = frame;
    const auto middle = first + ((taps + 1) / 2);
    const auto last = first + taps;
    const auto ascending = moment[middle] - moment[first] -
                           ((static_cast<std::int64_t>(first) - 1) * (sum[middle] - sum[first]));
    const auto descending = (static_cast<std::int64_t>(last) * (sum[last] - sum[middle])) -
                            (moment[last] - moment[middle]);
    return std::int64_t{2} * coefficient_unit * (ascending + descending);
  }
};

void verify_dot(const std::vector<Word> &data, const Point point, const std::int64_t expected) {
  std::int64_t value = 0;
  for (Word tap = 0; tap < taps; ++tap) {
    value += 2 * signed24(data.at(point.frame + tap)) * coefficient(tap);
  }
  require(value == expected,
          "Prefix-moment convolution matches an independent ordered dot product");
}

struct Model {
  Registers state;
  Word frame_status = 0, last_sample = 0;
  std::vector<Word> output = std::vector<Word>(output_words, sentinel);
};

void output(Model &model, const Point point, const std::int64_t value) {
  require(value >= -maximum_raw && value <= maximum_raw,
          "All FIR sums and positive-coefficient prefixes stay inside the proven signed56 domain");
  model.state.a = static_cast<std::uint64_t>(value) & accumulator_mask;
  sequence_model::alu(model.state, model.state, rounded);
  const auto word = register_model::read(model.state, 14);
  require(
      (model.state.sr & (flag_s | flag_l)) == (model.frame_status & (flag_s | flag_l)),
      "Low-amplitude outputs introduce neither growth nor limit sticky flags in any scale mode");
  model.output.at(((point.frame * channels) + point.channel) % output_words) = word;
}

void setup(BinaryCase &item, const Input input, const Program &code) {
  item.initial = initial_state();
  item.initial.sr = input.mode | input.flags;
  item.initial.r[1] = output_base;
  item.initial.r[4] = coefficient_base;
  item.initial.n[0] = taps - 1;
  item.initial.n[4] = taps;
  item.initial.n[7] = input.frames;
  item.initial.m[1] = output_words - 1;
  item.memory_words = {1024, input_base + (channels * input_stride), 2048};
  item.budget = 1 + (input.frames * frame_instructions);
  item.dispatches = item.budget;
  item.progress = {.instructions = item.budget,
                   .estimated_cycles = 5 + (std::uint64_t{input.frames} * frame_cycles)};
  std::vector<Word> weights;
  weights.reserve(taps);
  for (Word tap = 0; tap < taps; ++tap) { weights.push_back(coefficient(tap)); }
  std::vector<Word> cursors;
  cursors.reserve(channels);
  for (Word channel = 0; channel < channels; ++channel) {
    cursors.push_back(input_base + (channel * input_stride));
  }
  item.before = {{.words = code.words, .address = start},
                 {.words = std::move(weights), .address = coefficient_base, .space = Space::y},
                 {.words = std::move(cursors), .address = cursor_base, .space = Space::x},
                 {.words = std::vector<Word>(output_words, sentinel),
                  .address = output_base,
                  .space = Space::x},
                 {.words = {sentinel}, .address = progress_address, .space = Space::x}};
  for (const auto address : {cursor_base - 1,
                             cursor_base + channels,
                             progress_address - 1,
                             progress_address + 1,
                             output_base - 1,
                             output_base + output_words}) {
    item.before.push_back({.words = {sentinel}, .address = address, .space = Space::x});
  }
  for (const auto address : {coefficient_base - 1, coefficient_base + taps}) {
    item.before.push_back({.words = {sentinel}, .address = address, .space = Space::y});
  }
}

Model convolution(BinaryCase &item, const Input input) {
  Model model{.state = item.initial};
  model.state.sr |= loop_flag;
  std::vector<std::vector<Word>> data;
  std::vector<Prefix> prefixes;
  for (Word channel = 0; channel < channels; ++channel) {
    data.emplace_back(input.frames + taps - 1);
    for (Word frame = 0; frame < input.frames; ++frame) {
      data.back().at(frame + taps - 1) = signal({.frame = frame, .channel = channel});
    }
    prefixes.emplace_back(data.back());
    const auto base = input_base + (channel * input_stride);
    item.before.push_back({.words = data.back(), .address = base, .space = Space::x});
    for (const auto address : {base - 1, base + input.frames + taps - 1}) {
      item.before.push_back({.words = {sentinel}, .address = address, .space = Space::x});
    }
  }
  constexpr Word short_limit = 513;
  for (Word frame = 0; frame < input.frames; ++frame) {
    model.frame_status = model.state.sr;
    for (Word channel = 0; channel < channels; ++channel) {
      const auto value = prefixes.at(channel).convolution(frame);
      const Point point{.frame = frame, .channel = channel};
      if (input.frames <= short_limit || frame <= 1 || frame == taps - 1 || frame == taps ||
          frame == input.frames / 2 || frame + 1 == input.frames) {
        verify_dot(data.at(channel), point, value);
      }
      output(model, point, value);
    }
  }
  model.last_sample = data.back().at(input.frames + taps - 2);
  return model;
}

void finish(BinaryCase &item, Model model, const Program &code) {
  const auto frames = item.initial.n[7];
  auto &state = item.expected;
  state = item.initial;
  state.a = model.state.a;
  state.sr = model.state.sr & ~loop_flag;
  state.x = (std::uint64_t{1} << word_bits) | model.last_sample;
  state.y = coefficient(taps - 1);
  state.r[0] = input_base + ((channels - 1) * input_stride) + frames;
  state.r[1] = output_base + ((frames * channels) % output_words);
  state.r[2] = state.r[3] = cursor_base + channels;
  state.pc = code.frame_end + 1;
  state.stack[2] = (std::uint64_t{code.frame_body} << word_bits) | item.initial.sr;
  state.stack[3] = (std::uint64_t{code.frame_end} << word_bits) | 1;
  state.stack[4] = (std::uint64_t{code.channel_body} << word_bits) | model.frame_status;
  state.stack[5] = (std::uint64_t{code.channel_end} << word_bits) | 1;
  state.stack[6] =
      (std::uint64_t{code.mac_pc} << word_bits) | item.initial.sr | loop_flag | flag_z | flag_u;
  std::vector<Word> cursors;
  cursors.reserve(channels);
  for (Word channel = 0; channel < channels; ++channel) {
    cursors.push_back(input_base + (channel * input_stride) + frames);
  }
  item.after = {{.words = std::move(cursors), .address = cursor_base, .space = Space::x},
                {.words = {1}, .address = progress_address, .space = Space::x},
                {.words = std::move(model.output), .address = output_base, .space = Space::x}};
}

BinaryCase fixture(const Input input) {
  if (input.frames == 0 || input.frames > stream_frames) {
    throw std::invalid_argument("Invalid FIR stream length");
  }
  BinaryCase item;
  const auto code = program();
  setup(item, input, code);
  finish(item, convolution(item, input), code);
  item.name =
      "Continuous FIR bank channels=11 taps=507 rate=48000 frames=" + std::to_string(input.frames) +
      " mode=" + std::to_string(input.mode) + " flags=" + std::to_string(input.flags);
  return item;
}
} // namespace
} // namespace dsp56300::test::fir_stream

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_fir_stream full|check FILE.d56");
    }
    using namespace dsp56300::test;
    std::vector<BinaryCase> cases;
    if (std::string_view{argv[1]} == "full") {
      cases.push_back(fir_stream::fixture({.frames = fir_stream::stream_frames}));
    } else if (std::string_view{argv[1]} == "check") {
      constexpr std::array<dsp56300::Word, 5> lengths{1, 7, 65, 129, 513};
      for (const auto mode : fir_stream::modes) {
        for (const auto flags : {0U, flag_c | flag_s | flag_l}) {
          for (const auto frames : lengths) {
            cases.push_back(fir_stream::fixture({.frames = frames, .mode = mode, .flags = flags}));
          }
        }
      }
    } else {
      throw std::invalid_argument("Select full or check stream fixture");
    }
    write_fixture(argv[2], std::move(cases));
    std::cout << checks
              << " independent FIR convolution checks passed; stream expectations written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
