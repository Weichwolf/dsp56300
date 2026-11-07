// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::field_model {
enum class Kind : std::uint8_t { extract, extract_unsigned, insert, merge, leading };

struct Input {
  Kind kind = Kind::extract;
  unsigned controller = 4, source = 4;
  Word width = 1, offset = 0;
  bool immediate = false, source_b = false, destination_b = false;
};

struct Part {
  unsigned selector;
  bool main = true;
};

inline constexpr Word ignored = 0xfc0fc0;
inline constexpr unsigned width_position = 12;

inline Word control(const Input input) {
  return ignored | (input.width << width_position) | input.offset;
}

inline Word opcode(const Input input) {
  const Word destination = input.destination_b ? 1U : 0U;
  constexpr Word extract_register = 0x0c1a00;
  constexpr Word extract_immediate = 0x0c1800;
  constexpr Word insert_register = 0x0c1b00;
  constexpr Word insert_immediate = 0x0c1900;
  constexpr Word merge = 0x0c1b80;
  constexpr Word leading = 0x0c1e00;
  switch (input.kind) {
    case Kind::extract:
    case Kind::extract_unsigned:
      return (input.immediate ? extract_immediate : extract_register) |
             (input.kind == Kind::extract_unsigned ? 0x80U : 0U) | (input.source_b ? 16U : 0U) |
             destination | (input.immediate ? 0U : input.controller * 2);
    case Kind::insert:
      return (input.immediate ? insert_immediate : insert_register) | (input.source * 16) |
             destination | (input.immediate ? 0U : input.controller * 2);
    case Kind::merge: return merge | (input.controller * 2) | destination;
    case Kind::leading: return leading | (input.source_b ? 2U : 0U) | destination;
    default: throw std::logic_error("Unknown authored bit-field operation");
  }
}

template <class State> auto &storage(State &state, const unsigned selector) {
  if (selector < 4) { return selector == 2 ? state.a : state.b; }
  return (selector & 1U) == 0 ? state.x : state.y;
}

inline unsigned position(const Part part) {
  if (part.selector < 4) { return part.main ? word_bits : 0U; }
  return part.selector < 6 ? 0U : word_bits;
}

inline Word read(const Registers &state, const Part part) {
  return static_cast<Word>(storage(state, part.selector) >> position(part)) & word_mask;
}

inline void put(Registers &state, const Part part, const Word value) {
  auto &target = storage(state, part.selector);
  const auto shift = position(part);
  target = (target & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
}

inline auto bits(std::uint64_t value) {
  std::array<bool, accumulator_bits> result{};
  for (auto &bit : result) {
    bit = value % 2 != 0;
    value /= 2;
  }
  return result;
}

inline void logical_flags(Registers &state, const Word result) {
  state.sr &= ~(flag_n | flag_z | flag_v);
  if (result >= sign24) { state.sr |= flag_n; }
  if (result == 0) { state.sr |= flag_z; }
}

inline Fault failure(const FaultKind kind, const Word value) {
  return {.kind = kind, .space = Space::program, .address = start, .value = value, .pc = start};
}

inline void leading(Registers &state, const Input input) {
  const auto raw = input.source_b ? state.b : state.a;
  const auto source = bits(raw);
  int count = 9;
  for (std::size_t index = source.size(); index != 0; --index) {
    if (source[index - 1] != source.back()) { break; }
    --count;
  }
  if (raw == 0) { count = 0; }
  (input.destination_b ? state.b : state.a) =
      static_cast<std::uint64_t>(static_cast<std::int64_t>(count) *
                                 (std::int64_t{1} << word_bits)) &
      accumulator_mask;
  logical_flags(state, static_cast<Word>(count) & word_mask);
}

struct Window {
  unsigned width, offset;
};

inline std::uint64_t number(const std::array<bool, accumulator_bits> &value) {
  std::uint64_t raw = 0;
  for (std::size_t bit = value.size(); bit != 0; --bit) {
    raw = (raw * 2) + (value[bit - 1] ? 1U : 0U);
  }
  return raw;
}

inline std::uint64_t transform(const Registers &state, const Input input, const Window window) {
  auto result = bits(input.destination_b ? state.b : state.a);
  if (input.kind == Kind::extract || input.kind == Kind::extract_unsigned) {
    const auto source = bits(input.source_b ? state.b : state.a);
    for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
      result[bit] = bit < window.width
                        ? source[window.offset + bit]
                        : input.kind == Kind::extract && source[window.offset + window.width - 1];
    }
  } else {
    const auto source =
        bits(read(state,
                  {.selector = input.kind == Kind::merge ? input.controller : input.source,
                   .main = input.kind == Kind::merge}));
    if (input.kind == Kind::merge) {
      for (unsigned bit = 0; bit < width_position; ++bit) {
        result[word_bits + width_position + bit] = source[bit];
      }
    } else {
      for (unsigned bit = 0; bit < window.width; ++bit) {
        result[window.offset + bit] = source[bit];
      }
    }
  }
  return number(result);
}

inline Fault validate(const Input input, const Word descriptor) {
  if (input.kind == Kind::merge) { return {}; }
  const auto width = (descriptor >> width_position) & 63U;
  const auto offset = descriptor & 63U;
  if (input.kind == Kind::insert) {
    if (input.source < 2 || width > word_bits || offset + width > accumulator_bits) {
      return failure(FaultKind::invalid_instruction, opcode(input));
    }
  } else {
    if (offset + width > accumulator_bits) {
      return failure(FaultKind::invalid_instruction, descriptor);
    }
    if (width == 0 && input.kind == Kind::extract) {
      return failure(FaultKind::unsupported_mode, descriptor);
    }
  }
  return {};
}

inline Fault apply(Registers &state, const Input input) {
  constexpr Word unsupported = 0x026000;
  if ((state.sr & unsupported) != 0 ||
      (state.sr & (mode_up | mode_down)) == (mode_up | mode_down)) {
    return failure(FaultKind::unsupported_mode, state.sr);
  }
  if (input.kind == Kind::leading) {
    leading(state, input);
    return {};
  }
  if (!input.immediate && input.controller < 2) {
    return failure(FaultKind::invalid_instruction, opcode(input));
  }
  const auto descriptor =
      input.immediate ? control(input) : read(state, {.selector = input.controller});
  const auto fault = validate(input, descriptor);
  if (fault.kind != FaultKind::none) { return fault; }
  const auto raw = transform(
      state, input, {.width = (descriptor >> width_position) & 63U, .offset = descriptor & 63U});
  (input.destination_b ? state.b : state.a) = raw;
  if (input.kind == Kind::merge) {
    logical_flags(state, static_cast<Word>(raw >> word_bits) & word_mask);
  } else {
    sequence_model::flags(state, {.full = signed56(raw), .carry_changed = true});
  }
  return {};
}
}
