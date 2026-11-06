// SPDX-License-Identifier: GPL-3.0-only
// Algebraic expectations for a persistent 92-channel, four-biquad filter bank.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "iir_model.hpp"
#include "iir_program.hpp"
#include "iir_stream_program.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

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

namespace dsp56300::test::iir_stream {
namespace {
constexpr Word sentinel = 0x5a36c9;
constexpr auto unit = iir::unit;
constexpr std::int64_t state_unit = std::int64_t{1} << word_bits;
constexpr std::array<iir::Coefficients, sections> filters{
    {{9 * (unit / 64), 9 * (unit / 32), 9 * (unit / 64), -unit / 2, unit / 16},
     {25 * (unit / 256), 25 * (unit / 128), 25 * (unit / 256), -3 * (unit / 4), 9 * (unit / 64)},
     {unit / 16, unit / 8, unit / 16, -unit, unit / 4},
     {81 * (unit / 1024),
      81 * (unit / 512),
      81 * (unit / 1024),
      -7 * (unit / 8),
      49 * (unit / 256)}}};
constexpr std::int64_t maximum_datum = unit / 8;

Word signal(const Word frame, const Word channel) {
  const Word phase = ((frame * ((channel % 11) + 1)) + (channel * 3)) % signal_period;
  const auto triangle = phase < signal_period / 2
                            ? static_cast<std::int64_t>(phase)
                            : static_cast<std::int64_t>(signal_period - phase);
  const auto value = (triangle - signal_period / 4) * (unit / 2048);
  return static_cast<Word>(value) & word_mask;
}

std::vector<Word> input_signal() {
  std::vector<Word> words;
  words.reserve(input_words);
  for (Word frame = 0; frame < signal_period; ++frame) {
    for (Word channel = 0; channel < channels; ++channel) {
      words.push_back(signal(frame, channel));
    }
  }
  return words;
}

std::vector<Word> coefficients() {
  std::vector<Word> words;
  for (const auto &filter : filters) {
    // Physical stream order follows the scheduled old-operand MAC operations.
    for (const auto index : {0U, 1U, 3U, 2U, 4U}) {
      words.push_back(static_cast<Word>(filter.at(index)) & word_mask);
    }
  }
  return words;
}

struct Model {
  std::array<iir::Section, std::size_t{channels} * sections> history{};
  std::vector<Word> output = std::vector<Word>(output_words, sentinel);
  Registers mix;
  Word last_output = 0, last_old_second = 0;
};

void bounded(const Word value) {
  const auto number = signed24(value);
  if (number < -maximum_datum || number > maximum_datum) {
    throw std::logic_error("The low-amplitude stream must not set transfer growth or limit flags");
  }
}

struct Frame {
  Word index = 0, mode = 0;
};

void frame(Model &model, const Frame request) {
  const auto index = request.index;
  const auto mode = request.mode;
  std::int64_t sum = 0;
  for (Word channel = 0; channel < channels; ++channel) {
    auto value = signal(index % signal_period, channel);
    for (Word stage = 0; stage < sections; ++stage) {
      auto &state = model.history.at((channel * sections) + stage);
      model.last_old_second = state.second;
      value = iir::sample(state, value, filters.at(stage), mode);
      bounded(value);
      bounded(state.first);
      bounded(state.second);
    }
    model.last_output = value;
    sum += signed24(value);
  }
  model.mix.b = static_cast<std::uint64_t>(sum * state_unit) & accumulator_mask;
  model.mix.sr = mode | iir::loop_flag;
  constexpr unsigned shifts = 7;
  constexpr Word shift_b = 0x2a;
  constexpr Word round_b = 0x19;
  for (unsigned shift = 0; shift < shifts; ++shift) {
    sequence_model::alu(model.mix, model.mix, shift_b);
  }
  sequence_model::alu(model.mix, model.mix, round_b);
  const auto output = register_model::read(model.mix, 15);
  bounded(output);
  model.output.at(index % output_words) = output;
}

struct Input {
  Word frames = 0, mode = 0;
};

void setup(BinaryCase &item, const Input input, const Program &code) {
  const auto frames = input.frames;
  const auto mode = input.mode;
  item.initial = initial_state();
  item.initial.sr = mode;
  item.initial.r[0] = input_base;
  item.initial.r[1] = output_base;
  item.initial.r[4] = coefficient_base;
  item.initial.m[0] = input_words - 1;
  item.initial.m[1] = output_words - 1;
  item.initial.m[4] = coefficient_words - 1;
  item.initial.n[7] = frames;
  item.memory_words = {1024, input_base + input_words + 1, 1024};
  item.budget = 1 + (frames * frame_instructions);
  item.dispatches = item.budget;
  item.progress = {.instructions = item.budget,
                   .estimated_cycles = 5 + (std::uint64_t{frames} * frame_cycles)};
  item.before = {
      {.words = code.words, .address = start},
      {.words = input_signal(), .address = input_base, .space = Space::x},
      {.words = std::vector<Word>(state_words), .address = state_base, .space = Space::x},
      {.words = std::vector<Word>(output_words, sentinel),
       .address = output_base,
       .space = Space::x},
      {.words = coefficients(), .address = coefficient_base, .space = Space::y}};
  constexpr std::array spaces{Space::x, Space::x, Space::y};
  constexpr std::array begins{state_base, input_base, coefficient_base};
  constexpr std::array sizes{state_words, input_words, coefficient_words};
  for (std::size_t index = 0; index < spaces.size(); ++index) {
    item.before.push_back(
        {.words = {sentinel}, .address = begins[index] - 1, .space = spaces[index]});
    item.before.push_back(
        {.words = {sentinel}, .address = begins[index] + sizes[index], .space = spaces[index]});
  }
  for (const auto address : {output_base - 1, output_base + output_words}) {
    item.before.push_back({.words = {sentinel}, .address = address, .space = Space::x});
  }
}

void finish(BinaryCase &item, Model model, const Word frames, const Program &code) {
  auto &state = item.expected;
  state = item.initial;
  state.a = static_cast<std::uint64_t>(signed24(model.history.back().second) * state_unit) &
            accumulator_mask;
  state.b = model.mix.b;
  state.x = (std::uint64_t{model.last_output} << word_bits) | model.last_output;
  state.y = (std::uint64_t{model.last_old_second} << word_bits) |
            (static_cast<Word>(filters.back()[4]) & word_mask);
  state.sr = model.mix.sr & ~iir::loop_flag;
  state.pc = code.frame_end + 1;
  state.r[0] = input_base + ((frames % signal_period) * channels);
  state.r[1] = output_base + (frames % output_words);
  state.r[2] = state.r[3] = state_base + state_words;
  state.stack[2] = (std::uint64_t{code.frame_body} << word_bits) | item.initial.sr;
  state.stack[3] = (std::uint64_t{code.frame_end} << word_bits) | 1;
  state.stack[4] = (std::uint64_t{code.channel_body} << word_bits) | item.initial.sr |
                   iir::loop_flag | flag_z | flag_u;
  std::vector<Word> history;
  for (const auto entry : model.history) {
    history.insert(history.end(), {entry.first, entry.second});
  }
  item.after = {{.words = std::move(history), .address = state_base, .space = Space::x},
                {.words = std::move(model.output), .address = output_base, .space = Space::x}};
}

BinaryCase fixture(const Input request) {
  const auto frames = request.frames;
  const auto mode = request.mode;
  if (frames == 0 || frames > stream_frames) {
    throw std::invalid_argument("Stream length must be in [1,96000]");
  }
  BinaryCase item;
  const auto code = program();
  setup(item, request, code);
  Model model;
  for (Word index = 0; index < frames; ++index) { frame(model, {.index = index, .mode = mode}); }
  finish(item, std::move(model), frames, code);
  item.name =
      "Continuous IIR bank channels=92 sections=4 rate=48000 frames=" + std::to_string(frames) +
      " mode=" + std::to_string(mode);
  return item;
}
} // namespace
} // namespace dsp56300::test::iir_stream

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_iir_stream full|check FILE.d56");
    }
    using namespace dsp56300::test;
    std::vector<BinaryCase> cases;
    if (std::string_view{argv[1]} == "full") {
      cases.push_back(iir_stream::fixture({.frames = iir_stream::stream_frames}));
    } else if (std::string_view{argv[1]} == "check") {
      constexpr std::array<dsp56300::Word, 4> lengths{1, 7, 257, 1025};
      for (const auto mode : {0U, mode_rm}) {
        for (const auto frames : lengths) {
          cases.push_back(iir_stream::fixture({.frames = frames, .mode = mode}));
        }
      }
    } else {
      throw std::invalid_argument("Select full or check stream fixture");
    }
    write_fixture(argv[2], std::move(cases));
    std::cout << "Independent continuous IIR stream expectations written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
