// SPDX-License-Identifier: GPL-3.0-only
#include "address_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word low = 0x4000, high = 0x8000, port = 0xffff80;
constexpr Word lf = Word{1} << 15, loop_bits = Word{3} << 15;
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr Word loads = (Word{1} << 15) | (Word{1} << 22);
constexpr Word add = static_cast<Word>(isa::Opcode::add_x0_a);
constexpr Word negate = static_cast<Word>(isa::Opcode::neg_b);
constexpr Word prior_la = 0x900, prior_lc = 7;
constexpr Word x_pattern = 0xab100000, y_pattern = 0xcd200000, port_value = 0x345678;
constexpr Word x_high = 0x123456, y_high = 0x654321, b_word = 0x300000;
constexpr unsigned y_post_shift = 20;
constexpr std::size_t half = 16, size = half * 2;
using Data = std::array<Word, size>;
using Known = std::array<std::uint8_t, size>;

struct Input {
  Word count = 1, modifier = word_mask;
  unsigned post = 3;
  bool split = false, high_window = false, readonly = false, shared = false, alu = false;
  bool accumulators = false;
};

class Bus final : public Io {
public:
  explicit Bus(const Input input) {
    reset();
    const auto bind = [](Data &data, Known &known) {
      return std::array{MemoryWindow{.address = low,
                                     .words = std::span(data).first(half),
                                     .known = std::span(known).first(half),
                                     .writable = true},
                        MemoryWindow{.address = high,
                                     .words = std::span(data).last(half),
                                     .known = std::span(known).last(half),
                                     .writable = true}};
    };
    windows[0] = bind(x, kx);
    windows[1] = bind(input.shared ? x : y, ky);
    windows[1][1].writable = !input.readonly;
    program[start] = pair | loads | (input.alu ? add & status_byte : 0U);
    program[start + 1] = pair | (input.post << 11) | (input.post << y_post_shift) |
                         (input.alu ? negate & status_byte : 0U);
    if (input.accumulators) {
      constexpr Word registers = (Word{2} << 18) | (Word{3} << 16);
      program[start] |= registers;
      program[start + 1] |= registers;
    }
  }

  void reset() {
    for (std::size_t index = 0; index < size; ++index) {
      x[index] = x_pattern + static_cast<Word>(index);
      y[index] = y_pattern + static_cast<Word>(index);
    }
    kx.fill(2);
    ky.fill(3);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program.size() ? std::optional{program[address]} : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    if (space == Space::program) { return {}; }
    return windows[space == Space::x ? 0U : 1U];
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (space == Space::program) {
      const auto word = inspect_program(address);
      if (!word) { return false; }
      value = *word;
      return true;
    }
    for (const auto &window : windows[space == Space::x ? 0U : 1U]) {
      if (address < window.address || address - window.address >= window.words.size()) { continue; }
      const auto index = address - window.address;
      if (!window.known.empty() && window.known[index] == 0) {
        ++reads;
        return false;
      }
      value = window.words[index];
      return true;
    }
    ++reads;
    if (address != port || owner == nullptr) { return false; }
    value = port_value;
    owner->request_yield();
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    if (space == Space::program) {
      ++writes;
      return false;
    }
    for (const auto &window : windows[space == Space::x ? 0U : 1U]) {
      if (address < window.address || address - window.address >= window.words.size()) { continue; }
      if (!window.writable) {
        ++writes;
        return false;
      }
      const auto index = address - window.address;
      window.words[index] = value & word_mask;
      if (!window.known.empty()) { window.known[index] = 1; }
      return true;
    }
    ++writes;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  std::array<Word, memory_words> program{};
  Data x{}, y{};
  Known kx{}, ky{};
  std::array<std::array<MemoryWindow, 2>, 2> windows;
  Core *owner = nullptr;
  unsigned reads = 0, writes = 0; // Exceptional accesses; ordinary RAM has no observer effects.
};

Registers seed(const Input input) {
  auto state = initial_state();
  state.r[0] = state.r[4] = (input.high_window ? high : low) + (input.modifier == 15 ? 14U : 3U);
  state.n[0] = state.n[4] = 2;
  state.m[0] = state.m[4] = input.modifier;
  state.x = std::uint64_t{x_high} << word_bits;
  state.y = std::uint64_t{y_high} << word_bits;
  state.b = std::uint64_t{b_word} << word_bits;
  state.sr = lf | flag_s;
  state.la = start + 1;
  state.lc = input.count;
  state.sp = state.sc = 2;
  state.stack[2] = std::uint64_t{start} << word_bits;
  state.stack[1] = (std::uint64_t{prior_la} << word_bits) | prior_lc;
  return state;
}

std::size_t offset(const Word address) {
  return address >= high ? half + address - high : address - low;
}

struct Model {
  Registers state;
  Data x, y;
  Known kx, ky;
};

void step(Model &model, const Input input) {
  auto &state = model.state;
  const auto before = state;
  const auto xi = offset(state.r[0]);
  const auto yi = offset(state.r[4]);
  const bool load = state.pc == start;
  if (load) {
    const auto xv = model.x.at(xi) & word_mask;
    const auto yv = (input.shared ? model.x : model.y).at(yi) & word_mask;
    if (input.accumulators) {
      state.a = std::uint64_t{xv} << word_bits;
      state.b = std::uint64_t{yv} << word_bits;
    } else {
      state.x = (state.x & ~std::uint64_t{word_mask}) | xv;
      state.y = (state.y & ~std::uint64_t{word_mask}) | yv;
    }
  } else {
    for (const auto index : {0U, 4U}) {
      state.r[index] = address_model::updated({.modifier = state.m[index],
                                               .cursor = state.r[index],
                                               .offset = state.n[index],
                                               .mode = input.post});
    }
    model.x.at(xi) =
        static_cast<Word>(input.accumulators ? state.a >> word_bits : state.x) & word_mask;
    model.kx.at(xi) = 1;
    if (input.readonly && input.high_window) {
      ++state.pc;
      return;
    }
    (input.shared ? model.x : model.y).at(yi) =
        static_cast<Word>(input.accumulators ? state.b >> word_bits : state.y) & word_mask;
    model.ky.at(yi) = 1;
  }
  if (input.alu) { sequence_model::alu(state, before, load ? add : negate); }
  ++state.pc;
  if (load) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
  } else {
    state.sp = state.sc = 0;
    state.la = prior_la;
    state.lc = prior_lc;
    state.sr &= ~loop_bits;
  }
}

void check(const Input input) {
  Bus bus(input);
  Core core(Memory::external(), &bus);
  auto warm = input;
  warm.high_window = false;
  warm.count = 1;
  core.set_registers(seed(warm));
  require(core.run(2).retired == 2, "Warm the immutable entry in a writable window");
  bus.reset();
  const auto initial = seed(input);
  Model model{.state = initial, .x = bus.x, .y = bus.y, .kx = bus.kx, .ky = bus.ky};
  core.set_registers(initial);
  const Word total = input.count * 2;
  const Word first = input.split ? 1U : total;
  for (const auto grant : {first, total - first}) {
    if (grant == 0) { continue; }
    for (Word index = 0; index < grant; ++index) { step(model, input); }
    const auto before = core.state().progress;
    const auto result =
        input.split ? core.run_until(before.estimated_cycles + grant) : core.run(grant);
    const bool refused = input.readonly && input.high_window;
    const Word retired = refused ? grant - 1 : grant;
    const bool matched =
        core.registers() == model.state && bus.x == model.x && bus.y == model.y &&
        bus.kx == model.kx && bus.ky == model.ky && result.retired == retired &&
        result.progress == Progress{.instructions = before.instructions + retired,
                                    .estimated_cycles = before.estimated_cycles + retired};
    if (!matched) {
      const auto actual = core.registers();
      std::cerr << "count=" << input.count << " modifier=" << input.modifier
                << " post=" << input.post << " split=" << input.split
                << " high=" << input.high_window << " readonly=" << input.readonly
                << " shared=" << input.shared << " alu=" << input.alu
                << " accumulators=" << input.accumulators << " grant=" << grant
                << " retired=" << result.retired << " registers=" << (actual == model.state)
                << " x=" << (bus.x == model.x) << " y=" << (bus.y == model.y)
                << " kx=" << (bus.kx == model.kx) << " ky=" << (bus.ky == model.ky)
                << " sr=" << actual.sr << '/' << model.state.sr << " a=" << actual.a << '/'
                << model.state.a << " b=" << actual.b << '/' << model.state.b << '\n';
    }
    require(matched,
            "Window reuse preserves independently modeled full state, data and known-byte writes");
    require(result.fault.kind == (refused ? FaultKind::memory_write : FaultKind::none) &&
                (!refused || (result.fault.pc == start + 1 && result.fault.space == Space::y &&
                              result.fault.address == initial.r[4])) &&
                bus.reads == 0 && bus.writes == (refused ? 1U : 0U),
            "A read witness never grants write permission or hides exceptional accesses");
  }
}

void owner_boundary() {
  Bus bus({});
  Core core(Memory::external(), &bus);
  bus.owner = &core;
  auto state = seed({});
  core.set_registers(state);
  require(core.run(2).retired == 2, "Warm ordinary window witnesses before a real I/O access");
  bus.reset();
  state.r[4] = port;
  core.set_registers(state);
  const auto result = core.run(2);
  state.x |= bus.x[3] & word_mask;
  state.y |= port_value;
  state.pc = start + 1;
  require(result.retired == 1 && result.yielded && result.fault.kind == FaultKind::none &&
              core.registers() == state && bus.reads == 1 && bus.writes == 0,
          "A real observer completes its load and yields before any cached successor");
  const auto resumed = core.run(1);
  require(resumed.retired == 0 && resumed.fault.kind == FaultKind::io_write &&
              resumed.fault.address == port && bus.reads == 1 && bus.writes == 1 &&
              bus.x[3] == ((x_pattern + 3) & word_mask) && bus.kx[3] == 1,
          "Resumption obtains real permissions and never reuses a witness across an owner call");
}

void unknown_word() {
  Bus bus({});
  Core core(Memory::external(), &bus);
  const auto initial = seed({});
  core.set_registers(initial);
  require(core.run(2).retired == 2, "Warm both witnesses before rejecting an unknown word");
  bus.reset();
  bus.ky[3] = 0;
  const auto x = bus.x;
  const auto y = bus.y;
  core.set_registers(initial);
  const auto before = core.state().progress;
  const auto result = core.run(2);
  require(result.retired == 0 && result.progress == before &&
              result.fault == Fault{.kind = FaultKind::memory_read,
                                    .space = Space::y,
                                    .address = low + 3,
                                    .pc = start} &&
              bus.x == x && bus.y == y && bus.kx[3] == 2 && bus.ky[3] == 0 &&
              core.registers().r == initial.r && bus.reads == 1 && bus.writes == 0,
          "An unknown second load keeps the real fault and never reaches a cached store");
  require(core.run(2).retired == 0 && bus.reads == 1,
          "A failed witness remains fail-stop without repeating its observer");
}

void unsupported_modifier() {
  constexpr Word reserved = 0x8000;
  Bus bus({});
  Core core(Memory::external(), &bus);
  auto initial = seed({});
  initial.m[0] = reserved;
  const auto x = bus.x;
  const auto y = bus.y;
  core.set_registers(initial);
  const auto result = core.run(2);
  require(result.retired == 1 && result.fault.kind == FaultKind::unsupported_mode &&
              result.fault.pc == start + 1 && result.fault.value == reserved && bus.x == x &&
              bus.y == y && bus.kx[3] == 2 && bus.ky[3] == 3 &&
              core.registers().r[0] == initial.r[0],
          "Reusing RAM permission never bypasses the independent AGU modifier guard");
}

void metadata_alias() {
  constexpr Word repeated = 0x02020202;
  constexpr Word marked = 0x01020202;
  Bus bus({});
  bus.y.fill(repeated);
  bus.windows[0][0].known = {reinterpret_cast<std::uint8_t *>(bus.y.data()), half};
  Core core(Memory::external(), &bus);
  auto expected = seed({});
  expected.x |= bus.x[3] & word_mask;
  expected.y |= repeated & word_mask;
  expected.r[0] = expected.r[4] = low + 4;
  expected.pc = start + 2;
  expected.sp = expected.sc = 0;
  expected.la = prior_la;
  expected.lc = prior_lc;
  expected.sr &= ~loop_bits;
  core.set_registers(seed({}));
  const auto result = core.run(2);
  require(result.retired == 2 && result.fault.kind == FaultKind::none &&
              core.registers() == expected && bus.x[3] == ((x_pattern + 3) & word_mask) &&
              bus.y[0] == marked && bus.y[3] == (repeated & word_mask) && bus.kx[3] == 2 &&
              bus.ky[3] == 1 && bus.reads == 0 && bus.writes == 0,
          "Cached addresses retain ordered known-byte writes even when metadata aliases data");
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (const dsp56300::Word count : {1U, 2U, 5U}) {
      for (const bool split : {false, true}) {
        for (const bool alu : {false, true}) {
          check({.count = count, .split = split, .alu = alu});
          check({.count = count, .split = split, .high_window = true, .alu = alu});
          check({.count = count, .modifier = 15, .split = split, .alu = alu});
          check({.count = count, .post = 1, .split = split, .alu = alu});
          check({.count = count, .split = split, .shared = true, .alu = alu});
        }
      }
    }
    check({.high_window = true, .readonly = true});
    check({.count = 5, .accumulators = true});
    check({.count = 5, .split = true, .accumulators = true});
    owner_boundary();
    unknown_word();
    unsupported_modifier();
    metadata_alias();
    std::cout << "Window witnesses preserve ordered state, guards, aliases and owner boundaries\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
