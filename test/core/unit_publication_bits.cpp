// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "unit_publication_support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <tuple>

namespace dsp56300::test::unit_publication_test {
namespace {
struct BitCase {
  unsigned number = 4, bit = 0, action = 0;
  Word mode = 0;
  bool zero = false;
};

Word changed(const Word original, const BitCase input) {
  const Word unit = Word{1} << input.bit;
  const bool previous = (original / unit) % 2 != 0;
  if ((input.action == 0 || input.action == 2) && previous) { return original - unit; }
  if ((input.action == 1 || input.action == 2) && !previous) { return original + unit; }
  return original;
}

Registers initial(const BitCase input) {
  auto state = seed();
  state.sr = input.mode | flag_c | flag_s | flag_l;
  if (input.zero) {
    state.sr = input.mode;
    state.a = 0;
    state.b = 0;
    state.x = 0;
    state.y = 0;
    state.n.fill(0);
    state.stack.fill(0);
  }
  return state;
}

UnitWriteback expected(const BitCase input, const Word opcode, Registers &state) {
  const auto before = state;
  const bool modify = input.action != 3;
  const bool inplace = input.number == register_model::reg_ssh && modify;
  const auto slot = state.sp % stack_entries;
  Word flags = 0;
  if (input.number == 14 || input.number == 15) {
    auto probe = state;
    probe.sr &= ~(flag_s | flag_l);
    static_cast<void>(register_model::read(probe, input.number));
    flags = probe.sr & (flag_s | flag_l);
  }
  const auto original = inplace ? static_cast<Word>(state.stack[slot] >> word_bits)
                                : register_model::read(state, input.number);
  if (modify) {
    const auto value = changed(original, input);
    if (inplace) {
      state.stack[slot] = (std::uint64_t{value} << word_bits) | (state.stack[slot] & word_mask);
    } else {
      register_model::write(state, input.number, value);
    }
  }
  if (input.number != register_model::reg_sr || !modify) {
    state.sr = (state.sr & ~flag_c) | (((original >> input.bit) % 2) != 0 ? flag_c : 0);
    flags |= flag_c;
  }
  UnitWriteback result{.unit = RegisterUnit::bit, .opcode = opcode};
  if (modify) {
    result.updates.at(result.count++) =
        inplace ? field(RegisterStorage::stack, middle, before.stack[slot], state.stack[slot], slot)
                : destination(before, state, input.number);
  } else if (input.number == register_model::reg_ssh) {
    result.updates.at(result.count++) =
        field(RegisterStorage::sp, stack_pointer_mask, before.sp, state.sp);
    result.updates.at(result.count++) =
        field(RegisterStorage::sc, stack_counter_mask, before.sc, state.sc);
  }
  if (flags != 0) {
    result.updates.at(result.count++) = field(RegisterStorage::sr, flags, before.sr, state.sr);
  }
  auto fields = std::span(result.updates).first(result.count);
  std::ranges::sort(fields, [](const RegisterUpdate &left, const RegisterUpdate &right) {
    return std::tie(left.storage, left.index) < std::tie(right.storage, right.index);
  });
  return result;
}

void bit(const Backend backend, const BitCase input) {
  constexpr std::array<Word, 4> prefixes{0x0ac040, 0x0ac060, 0x0bc040, 0x0bc060};
  const Word opcode = prefixes.at(input.action) | (input.number << 8) | input.bit;
  Bus bus;
  Core core(bus, {.backend = backend});
  auto state = initial(input);
  core.set_registers(state);
  static_cast<void>(core.begin_instruction({.address = boot, .opcode = opcode}, policy));
  held(core, state);
  const auto result = expected(input, opcode, state);
  publish(core, result, state);
  retire(core);
  ++state.pc;
  require(core.registers() == state && core.fault().kind == FaultKind::none && bus.calls == 0 &&
              core.progress() == Progress{.instructions = 1, .estimated_cycles = 2},
          "All original register/bit operations retain independent full data/CCR/stack state");
}

void variants(const Backend backend, const BitCase input) {
  for (const bool zero : {false, true}) {
    for (unsigned bit_index = 0; bit_index < word_bits; ++bit_index) {
      for (unsigned action = 0; action < 4; ++action) {
        bit(backend,
            {.number = input.number,
             .bit = bit_index,
             .action = action,
             .mode = input.mode,
             .zero = zero});
      }
    }
  }
}
}

void bits() {
  for (const auto backend : backends) {
    for (unsigned number = 4; number < 64; ++number) {
      if (!register_model::valid(number)) { continue; }
      for (const Word mode : {0U, mode_down, mode_up}) {
        if (mode != 0 && number != 14 && number != 15) { continue; }
        variants(backend, {.number = number, .mode = mode});
      }
    }
  }
}
} // namespace dsp56300::test::unit_publication_test
