// SPDX-License-Identifier: GPL-3.0-only
// Independent continuous driven-feedback expectations; never links an emulator.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "nonlinear_stream_model.hpp"
#include "nonlinear_stream_program.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
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

namespace dsp56300::test::nonlinear_stream {
namespace {
constexpr Word sentinel = 0x5a36c9;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};

void validate_math() {
  constexpr std::array<Word, 9> values{
      0, 1, 3, sign24 - 1, sign24, sign24 + 1, word_mask, sign24 / 2, 3 * sign24 / 2};
  for (const auto mode : modes) {
    for (const auto coefficients : weights) {
      for (const auto input : values) {
        for (const auto previous : values) {
          for (const Word flags : {0U, status_byte}) {
            const auto status = mode | sm | flags;
            const auto expected = sample({.sample = input,
                                          .previous = previous,
                                          .status = status,
                                          .coefficients = coefficients});
            Registers state;
            state.sr = status;
            state.x = (std::uint64_t{previous} << word_bits) | input;
            state.y =
                (std::uint64_t{static_cast<Word>(coefficients.feedback) & word_mask} << word_bits) |
                (static_cast<Word>(coefficients.gain) & word_mask);
            constexpr Word mpy = 0x2000d0;
            constexpr Word macr = 0x2000f3;
            saturation_model::alu(state, state, mpy);
            saturation_model::alu(state, state, macr);
            const auto output = register_model::read(state, 14);
            require(output == expected.value && state.sr == expected.status &&
                        state.a ==
                            (static_cast<std::uint64_t>(expected.accumulator) & accumulator_mask),
                    "Numerical feedback quantization matches independent raw56 "
                    "instruction/transfer models");
          }
        }
      }
    }
  }
}

std::vector<Word> inputs() {
  std::vector<Word> result;
  result.reserve(input_words);
  for (Word frame = 0; frame < signal_period; ++frame) {
    for (Word channel = 0; channel < channels; ++channel) {
      result.push_back(signal({.frame = frame, .channel = channel}));
    }
  }
  return result;
}

std::vector<Word> histories() {
  std::vector<Word> result;
  result.reserve(state_words);
  for (Word channel = 0; channel < channels; ++channel) {
    for (Word stage = 0; stage < stages; ++stage) { result.push_back(history(channel, stage)); }
  }
  return result;
}

std::vector<Word> coefficients() {
  std::vector<Word> result;
  for (const auto entry : weights) {
    result.push_back(static_cast<Word>(entry.gain) & word_mask);
    result.push_back(static_cast<Word>(entry.feedback) & word_mask);
  }
  return result;
}

struct Input {
  Word frames = 0, mode = 0, flags = 0;
};

void setup(BinaryCase &item, const Input input, const Program &code) {
  item.initial = initial_state();
  item.initial.sr = input.mode | input.flags;
  item.initial.r[0] = input_base;
  item.initial.r[1] = output_base;
  item.initial.m[0] = input_words - 1;
  item.initial.m[1] = output_words - 1;
  item.initial.n[7] = input.frames;
  item.memory_words = {1024, input_base + input_words + 1, 1024};
  item.budget = 1 + (input.frames * frame_instructions);
  item.dispatches = item.budget;
  item.progress = {.instructions = item.budget,
                   .estimated_cycles = 5 + (std::uint64_t{input.frames} * frame_cycles)};
  item.before = {{.words = code.words, .address = start},
                 {.words = inputs(), .address = input_base, .space = Space::x},
                 {.words = histories(), .address = state_base, .space = Space::x},
                 {.words = std::vector<Word>(output_words, sentinel),
                  .address = output_base,
                  .space = Space::x},
                 {.words = {sentinel}, .address = progress_address, .space = Space::x},
                 {.words = coefficients(), .address = coefficient_base, .space = Space::y}};
  constexpr std::array spaces{Space::x, Space::x, Space::x, Space::x, Space::y};
  constexpr std::array begins{
      state_base, input_base, output_base, progress_address, coefficient_base};
  constexpr std::array lengths{state_words, input_words, output_words, Word{1}, coefficient_words};
  for (std::size_t index = 0; index < spaces.size(); ++index) {
    item.before.push_back(
        {.words = {sentinel}, .address = begins[index] - 1, .space = spaces[index]});
    item.before.push_back(
        {.words = {sentinel}, .address = begins[index] + lengths[index], .space = spaces[index]});
  }
}

struct Counts {
  std::uint64_t positive = 0, negative = 0, canonical = 0, ties = 0;
};

struct Model {
  std::vector<Word> history = histories();
  std::vector<Word> output = std::vector<Word>(output_words, sentinel);
  Result last;
  Counts counts;
  Word frame_status = 0;
};

void frame(Model &model, const Word index) {
  model.last.status |= sm;
  model.frame_status = model.last.status;
  for (Word channel = 0; channel < channels; ++channel) {
    auto value = signal({.frame = index, .channel = channel});
    for (Word stage = 0; stage < stages; ++stage) {
      auto &previous = model.history.at((channel * stages) + stage);
      model.last = sample({.sample = value,
                           .previous = previous,
                           .status = model.last.status,
                           .coefficients = weights[stage]});
      if (model.last.limited) {
        ++(model.last.accumulator < 0 ? model.counts.negative : model.counts.positive);
      } else {
        ++model.counts.canonical;
      }
      model.counts.ties += model.last.tie ? 1U : 0U;
      value = model.last.value;
      previous = value;
    }
    model.output.at(((index * channels) + channel) % output_words) = value;
  }
  model.last.status &= ~sm;
}

void finish(BinaryCase &item, Model model, const Input input, const Program &code) {
  auto &state = item.expected;
  state = item.initial;
  state.a = static_cast<std::uint64_t>(model.last.accumulator) & accumulator_mask;
  state.x = (std::uint64_t{1} << word_bits) | model.last.value;
  state.y = (std::uint64_t{static_cast<Word>(weights.back().feedback) & word_mask} << word_bits) |
            (static_cast<Word>(weights.back().gain) & word_mask);
  state.sr = model.last.status & ~loop_flag;
  state.pc = code.frame_end + 1;
  state.r[0] = input_base + (input.frames % signal_period) * channels;
  state.r[1] = output_base + (input.frames * channels) % output_words;
  state.r[2] = state.r[3] = state_base + state_words;
  state.r[4] = coefficient_base + coefficient_words;
  state.stack[1] = (std::uint64_t{item.initial.la} << word_bits) | item.initial.lc;
  state.stack[2] = (std::uint64_t{code.frame_body} << word_bits) | item.initial.sr;
  state.stack[3] = (std::uint64_t{code.frame_end} << word_bits) | 1;
  state.stack[4] = (std::uint64_t{code.channel_body} << word_bits) | model.frame_status;
  item.after = {{.words = std::move(model.history), .address = state_base, .space = Space::x},
                {.words = std::move(model.output), .address = output_base, .space = Space::x},
                {.words = {1}, .address = progress_address, .space = Space::x}};
  if (input.frames == stream_frames) {
    require(model.counts.positive != 0 && model.counts.negative != 0 && model.counts.canonical != 0,
            "The full nonlinear stream contains positive/negative limiting and ordinary results");
    std::cout << "Stage results: positive=" << model.counts.positive
              << " negative=" << model.counts.negative << " canonical=" << model.counts.canonical
              << " ties=" << model.counts.ties << '\n';
  }
}

BinaryCase fixture(const Input input) {
  if (input.frames == 0 || input.frames > stream_frames) {
    throw std::invalid_argument("Nonlinear stream length must be in [1,96000]");
  }
  BinaryCase item;
  const auto code = program();
  setup(item, input, code);
  Model model;
  model.last.status = item.initial.sr | loop_flag;
  for (Word index = 0; index < input.frames; ++index) { frame(model, index); }
  finish(item, std::move(model), input, code);
  item.name = "Continuous nonlinear feedback channels=204 stages=4 rate=48000 frames=" +
              std::to_string(input.frames) + " mode=" + std::to_string(input.mode) +
              " flags=" + std::to_string(input.flags);
  return item;
}
} // namespace
} // namespace dsp56300::test::nonlinear_stream

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("Usage: dsp56300_generate_nonlinear_stream full|check FILE.d56");
    }
    using namespace dsp56300::test;
    using dsp56300::Word;
    nonlinear_stream::validate_math();
    std::vector<BinaryCase> cases;
    if (std::string_view{argv[1]} == "full") {
      cases.push_back(nonlinear_stream::fixture({.frames = nonlinear_stream::stream_frames}));
    } else if (std::string_view{argv[1]} == "check") {
      constexpr std::array<Word, 4> lengths{1, 7, 33, 257};
      for (const auto mode : nonlinear_stream::modes) {
        for (const Word flags : {0U, status_byte}) {
          for (const auto frames : lengths) {
            cases.push_back(
                nonlinear_stream::fixture({.frames = frames, .mode = mode, .flags = flags}));
          }
        }
      }
    } else {
      throw std::invalid_argument("Select full or check stream fixture");
    }
    write_fixture(argv[2], std::move(cases));
    std::cout << "Independent continuous nonlinear stream expectations written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
