// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
constexpr Word pair = static_cast<Word>(isa::Opcode::move_xy);
constexpr unsigned x_register_shift = 18, y_register_shift = 16;
constexpr Word load = pair | (2U << 18) | (3U << 16) | (1U << 15) | (1U << 22);
constexpr Word store = pair | (3U << 18) | (2U << 16) | (3U << 11) | (3U << 20);
constexpr std::size_t extent = 64, split = extent / 2;
constexpr Word x_host_bits = 0xab000000, y_host_bits = 0xcd000000;
constexpr Word io_address = 0xffff80;
constexpr Word callback_value = 0x13579b, ram_value = 0x2468ac;
using Data = std::array<Word, extent>;

struct MemoryIo final : Io {
  MemoryIo()
      : windows_{MemoryWindow{.words = std::span(x).first(split), .known = {}, .writable = true},
                 MemoryWindow{.address = split,
                              .words = std::span(x).subspan(split),
                              .known = known,
                              .writable = true},
                 MemoryWindow{.words = std::span(y).first(split), .known = {}, .writable = true},
                 MemoryWindow{.address = split,
                              .words = std::span(y).subspan(split),
                              .known = known,
                              .writable = true}} {
    program[start] = load;
    program[start + 1] = store;
    known.fill(1);
  }

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    if (space == Space::program) { return {}; }
    return std::span(windows_).subspan(space == Space::x ? 0U : 2U, 2);
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program_words ? std::optional{program[address]} : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (callback_core != nullptr && space == Space::x && address == io_address) {
      ++callbacks;
      callback_pc = callback_core->registers().pc;
      callback_progress = callback_core->progress();
      callback_core->request_yield();
      value = callback_value;
      return true;
    }
    const auto word = inspect_program(address);
    if (space != Space::program || !word) { return false; }
    value = *word;
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    if (callback_core != nullptr && space == Space::x && address == io_address) {
      ++callbacks;
      callback_pc = callback_core->registers().pc;
      callback_progress = callback_core->progress();
      callback_output = value;
      callback_core->request_yield();
      return true;
    }
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  Data x{}, y{};
  std::array<std::uint8_t, split> known{};
  std::array<Word, memory_words> program{};
  std::size_t program_words = memory_words;
  Core *callback_core = nullptr;
  unsigned callbacks = 0;
  Word callback_pc = 0, callback_output = 0;
  Progress callback_progress{};
  std::uint64_t revision = 0;

  std::array<MemoryWindow, 4> windows_;
};

struct ExpectedMemory {
  Data x, y;
};

void owned_loop_access() {
  MemoryIo memory;
  Core core(Memory::external(), &memory);
  memory.callback_core = &core;
  auto initial = register_model::seed(0);
  initial.sr |= Word{1} << 15;
  initial.la = start + 1;
  initial.lc = 5;
  initial.r[0] = io_address;
  initial.r[4] = 4;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  memory.y[4] = ram_value;
  core.set_registers(initial);
  auto expected = initial;
  register_model::write(expected, 14, callback_value);
  register_model::write(expected, 15, memory.y[4]);
  ++expected.pc;
  const auto x = memory.x;
  const auto y = memory.y;
  const auto result = core.run(10);
  const auto actual = core.registers();
  const bool valid = result.fault.kind == FaultKind::none && result.retired == 1 &&
                     result.yielded && memory.callbacks == 1 && memory.callback_pc == start + 1 &&
                     memory.callback_progress == Progress{} && actual == expected &&
                     memory.x == x && memory.y == y;
  if (!valid) {
    std::cerr << "callback: fault=" << static_cast<unsigned>(result.fault.kind)
              << " retired=" << result.retired << " yielded=" << result.yielded
              << " calls=" << memory.callbacks << " observed-PC=" << memory.callback_pc
              << " observed-count=" << memory.callback_progress.instructions
              << " observed-cycles=" << memory.callback_progress.estimated_cycles
              << " PC=" << actual.pc << '/' << expected.pc << " SR=" << actual.sr << '/'
              << expected.sr << " A=" << actual.a << '/' << expected.a << " B=" << actual.b << '/'
              << expected.b << '\n';
  }
  require(valid,
          "A real loop I/O callback observes exact state and yields before the next instruction");
}

void owned_loop_store() {
  MemoryIo memory;
  memory.program[start] = store;
  memory.windows_[2].known = memory.known;
  memory.known.fill(0);
  Core core(Memory::external(), &memory);
  memory.callback_core = &core;
  auto initial = register_model::seed(0);
  initial.sr |= Word{1} << 15;
  initial.la = start + 1;
  initial.lc = 5;
  initial.r[0] = io_address;
  initial.r[4] = 4;
  initial.a = std::uint64_t{callback_value} << word_bits;
  initial.b = std::uint64_t{ram_value} << word_bits;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  core.set_registers(initial);
  auto expected = initial;
  const auto output = register_model::bus_word(expected, initial.b);
  ++expected.r[0];
  ++expected.r[4];
  ++expected.pc;
  const auto x = memory.x;
  auto y = memory.y;
  y[4] = register_model::bus_word(expected, initial.a);
  auto known = memory.known;
  known[4] = 1;
  const auto result = core.run(10);
  require(result.fault.kind == FaultKind::none && result.retired == 1 && result.yielded &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
              memory.callbacks == 1 && memory.callback_pc == start + 1 &&
              memory.callback_progress == Progress{} && memory.callback_output == output &&
              core.registers() == expected && memory.x == x && memory.y == y &&
              memory.known == known,
          "A mixed I/O/RAM store completes both ports and initializes only its granted RAM word");
}

struct CopyRouting {
  bool load_b_first = false, store_b_to_x = true, store_b_to_y = false;
};

void model_step(Registers &state, ExpectedMemory &memory, const CopyRouting routing = {}) {
  auto &x = memory.x;
  auto &y = memory.y;
  if ((state.pc - start) % 2 == 0) {
    register_model::write(state, routing.load_b_first ? 15U : 14U, x[state.r[0]] & word_mask);
    register_model::write(state, routing.load_b_first ? 14U : 15U, y[state.r[4]] & word_mask);
    ++state.pc;
    return;
  }
  const auto to_x = register_model::bus_word(state, routing.store_b_to_x ? state.b : state.a);
  const auto to_y = register_model::bus_word(state, routing.store_b_to_y ? state.b : state.a);
  x[state.r[0]++] = to_x;
  y[state.r[4]++] = to_y;
  ++state.pc;
  if (state.pc - 1 != state.la) { return; }
  if (state.lc > 1) {
    --state.lc;
    state.pc = start;
  } else {
    loop_frames::complete(state);
  }
}

struct PrefixInput {
  Word first = 0, count = 0, mode = 0;
  CopyRouting routing{};
  bool direct = false;
};

void prefixes(const PrefixInput input) {
  const auto first = input.first;
  const auto count = input.count;
  const auto mode = input.mode;
  MemoryIo memory;
  constexpr Word register_fields = (3U << x_register_shift) | (3U << y_register_shift);
  const auto &routing = input.routing;
  memory.program[start] = (load & ~register_fields) |
                          ((routing.load_b_first ? 3U : 2U) << x_register_shift) |
                          ((routing.load_b_first ? 2U : 3U) << y_register_shift);
  memory.program[start + 1] = (store & ~register_fields) |
                              ((routing.store_b_to_x ? 3U : 2U) << x_register_shift) |
                              ((routing.store_b_to_y ? 3U : 2U) << y_register_shift);
  Core core(input.direct
                ? Memory::separate({.program = memory.program, .x = memory.x, .y = memory.y})
                : Memory::external(),
            &memory);
  auto initial = register_model::seed(mode);
  initial.sr |= Word{1} << 15;
  initial.la = start + 1;
  initial.lc = count;
  initial.r[0] = initial.r[4] = first;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  constexpr std::array<Word, 7> values{0, 1, sign24, word_mask, sign24 - 1, 0x123456, 0xabcdef};
  for (unsigned warmth = 0; warmth < 2; ++warmth) {
    for (Word prefix = 0; prefix <= count * 2; ++prefix) {
      for (std::size_t index = 0; index < extent; ++index) {
        memory.x[index] = values[index % values.size()] | x_host_bits;
        memory.y[index] = values[(index + 3) % values.size()] | y_host_bits;
      }
      ExpectedMemory contents{.x = memory.x, .y = memory.y};
      auto expected = initial;
      for (Word step = 0; step < prefix; ++step) { model_step(expected, contents, routing); }
      core.set_registers(initial);
      const auto before = core.progress();
      const auto result =
          warmth == 0 ? core.run(prefix) : core.run_until(before.estimated_cycles + prefix);
      require(result.fault.kind == FaultKind::none && result.retired == prefix &&
                  result.progress ==
                      Progress{.instructions = before.instructions + prefix,
                               .estimated_cycles = before.estimated_cycles + prefix} &&
                  core.registers() == expected && memory.x == contents.x && memory.y == contents.y,
              "Cold and warm short loops preserve every prefix, full state and RAM binding");
    }
  }
}

Registers loop_state() {
  auto state = register_model::seed(0);
  state.sr |= Word{1} << 15;
  state.la = start + 1;
  state.lc = 5;
  state.r[0] = state.r[4] = 4;
  state.stack[2] = std::uint64_t{start} << word_bits;
  return state;
}

void fragmented_window() {
  MemoryIo memory;
  Core core(Memory::external(), &memory);
  const auto initial = loop_state();
  memory.x.fill(sign24 | x_host_bits);
  memory.y.fill((sign24 - 1) | y_host_bits);
  ExpectedMemory contents{.x = memory.x, .y = memory.y};
  auto expected = initial;
  core.set_registers(initial);
  for (const Word grant : {4U, 3U, 1U, 2U}) {
    for (Word step = 0; step < grant; ++step) { model_step(expected, contents); }
    const auto before = core.progress();
    const auto result = core.run(grant);
    require(result.retired == grant && result.fault == Fault{} && !result.yielded &&
                result.progress == Progress{.instructions = before.instructions + grant,
                                            .estimated_cycles = before.estimated_cycles + grant} &&
                core.registers() == expected && memory.x == contents.x && memory.y == contents.y,
            "Resumed grants retain complete loop state across whole windows and interior prefixes");
  }
}

void mismatched_window_frame() {
  constexpr Word target = start + 8;
  constexpr Word grant = 6;
  MemoryIo memory;
  for (Word pc = target; pc < target + grant; ++pc) { memory.program[pc] = isa::nop.front(); }
  Core core(Memory::external(), &memory);
  auto initial = loop_state();
  initial.stack[2] = std::uint64_t{target} << word_bits;
  for (unsigned warmth = 0; warmth < 2; ++warmth) {
    memory.x.fill(sign24 | x_host_bits);
    memory.y.fill((sign24 - 1) | y_host_bits);
    ExpectedMemory contents{.x = memory.x, .y = memory.y};
    auto expected = initial;
    model_step(expected, contents);
    model_step(expected, contents);
    expected.pc = target + grant - 2;
    core.set_registers(initial);
    const auto before = core.progress();
    const auto result = core.run(grant);
    require(result.retired == grant && result.fault == Fault{} && !result.yielded &&
                result.progress == Progress{.instructions = before.instructions + grant,
                                            .estimated_cycles = before.estimated_cycles + grant} &&
                core.registers() == expected && memory.x == contents.x && memory.y == contents.y,
            "A different saved loop start executes one body and follows its actual target");
  }
}

void aliased_copy_prefixes() {
  constexpr Word count = 8;
  std::array<Word, memory_words> program{};
  program[start] = load;
  program[start + 1] = store;
  const auto source = program;
  Data shared{};
  Core core(Memory::separate({.program = program, .x = shared, .y = shared}));
  auto initial = loop_state();
  initial.lc = count;
  initial.r[4] = initial.r[0] + 1;
  constexpr std::array<Word, 7> values{0, 1, sign24, word_mask, sign24 - 1, 0x123456, 0xabcdef};
  for (unsigned warmth = 0; warmth < 2; ++warmth) {
    for (Word prefix = 0; prefix <= count * 2; ++prefix) {
      for (std::size_t index = 0; index < shared.size(); ++index) {
        shared[index] = values[index % values.size()] | x_host_bits;
      }
      auto contents = shared;
      auto expected = initial;
      for (Word step = 0; step < prefix; ++step) {
        if ((expected.pc - start) % 2 == 0) {
          const auto x = contents[expected.r[0]] & word_mask;
          const auto y = contents[expected.r[4]] & word_mask;
          register_model::write(expected, 14, x);
          register_model::write(expected, 15, y);
          ++expected.pc;
        } else {
          const auto a = register_model::bus_word(expected, expected.a);
          const auto b = register_model::bus_word(expected, expected.b);
          contents[expected.r[0]++] = b;
          contents[expected.r[4]++] = a;
          ++expected.pc;
          if (expected.lc > 1) {
            --expected.lc;
            expected.pc = start;
          } else {
            loop_frames::complete(expected);
          }
        }
      }
      core.set_registers(initial);
      const auto before = core.progress();
      const auto result = core.run(prefix);
      require(result.retired == prefix && result.fault == Fault{} &&
                  result.progress ==
                      Progress{.instructions = before.instructions + prefix,
                               .estimated_cycles = before.estimated_cycles + prefix} &&
                  core.registers() == expected && shared == contents && program == source,
              "Overlapping Harvard views retain ordered reads, stores and every loop prefix");
    }
  }
}

Registers window_state(MemoryIo &memory) {
  constexpr Word instructions = 4;
  memory.program[start + 2] = load;
  memory.program[start + 3] = store;
  memory.program_words = start + instructions;
  auto state = loop_state();
  state.la = start + instructions + 1;
  return state;
}

void rebound_windows() {
  MemoryIo memory;
  MemoryIo other;
  Core core(Memory::external(), &memory);
  const auto initial = window_state(memory);
  for (const bool alternate : {false, true, false, true}) {
    auto &selected = alternate ? other : memory;
    auto &inactive = alternate ? memory : other;
    memory.windows_[0].words = std::span(selected.x).first(split);
    memory.windows_[2].words = std::span(selected.y).first(split);
    ++memory.revision;
    for (Word prefix = 0; prefix <= memory.program_words - start; ++prefix) {
      memory.x.fill(callback_value | x_host_bits);
      memory.y.fill(ram_value | y_host_bits);
      other.x.fill(sign24 | x_host_bits);
      other.y.fill(word_mask | y_host_bits);
      ExpectedMemory contents{.x = selected.x, .y = selected.y};
      const ExpectedMemory untouched{.x = inactive.x, .y = inactive.y};
      auto expected = initial;
      for (Word step = 0; step < prefix; ++step) { model_step(expected, contents); }
      core.set_registers(initial);
      const auto before = core.progress();
      const auto result = core.run(prefix);
      require(
          result.fault.kind == FaultKind::none && result.retired == prefix &&
              result.progress == Progress{.instructions = before.instructions + prefix,
                                          .estimated_cycles = before.estimated_cycles + prefix} &&
              core.registers() == expected && selected.x == contents.x &&
              selected.y == contents.y && inactive.x == untouched.x && inactive.y == untouched.y,
          "Returning immutable entries preserve every prefix after RAM window rebinding");
    }
  }
}

void revoked_windows() {
  enum class Revocation : std::uint8_t { unknown, read_only, shortened, program_alias };
  for (const auto revocation : {Revocation::unknown,
                                Revocation::read_only,
                                Revocation::shortened,
                                Revocation::program_alias}) {
    MemoryIo memory;
    Core core(Memory::external(), &memory);
    const auto initial = window_state(memory);
    core.set_registers(initial);
    require(core.run(4).retired == 4, "Prime an initialized writable RAM entry");
    switch (revocation) {
      case Revocation::unknown:
        memory.windows_[0].known = memory.known;
        memory.known[4] = 0;
        break;
      case Revocation::read_only: memory.windows_[0].writable = false; break;
      case Revocation::shortened: memory.windows_[0].words = std::span(memory.x).first(4); break;
      case Revocation::program_alias: memory.windows_[0].aliases_program = true; break;
      default: throw std::runtime_error("Invalid authored revocation");
    }
    ++memory.revision;
    core.set_registers(initial);
    const auto x = memory.x;
    const auto y = memory.y;
    const auto result = core.run(4);
    const bool load_fault =
        revocation == Revocation::unknown || revocation == Revocation::shortened;
    require(result.retired == (load_fault ? 0U : 1U) &&
                result.fault.kind ==
                    (load_fault ? FaultKind::memory_read : FaultKind::memory_write) &&
                result.fault.space == Space::x && result.fault.address == initial.r[0] &&
                result.fault.pc == start + (load_fault ? 0U : 1U) && memory.x == x && memory.y == y,
            "A retained initialized entry cannot survive knowledge, bound or write revocation");
  }
}

void rejected_windows() {
  constexpr Word first = 4;
  for (const bool executable : {false, true}) {
    MemoryIo memory;
    memory.windows_[0].writable = executable;
    memory.windows_[0].aliases_program = executable;
    Core core(Memory::external(), &memory);
    auto initial = initial_state();
    initial.sr = Word{1} << 15;
    initial.la = start + 1;
    initial.lc = 5;
    initial.r[0] = initial.r[4] = first;
    memory.x[first] = 1;
    memory.y[first] = sign24;
    core.set_registers(initial);
    const auto result = core.run(10);
    require(result.retired == 1 && result.fault.kind == FaultKind::memory_write &&
                result.fault.space == Space::x && result.fault.pc == start + 1 &&
                result.fault.address == first && memory.x[first] == 1 && memory.y[first] == sign24,
            "Read-only and executable-alias windows cannot acquire an unchecked writable view");
  }
  MemoryIo memory;
  memory.windows_[0].known = memory.known;
  memory.known[first] = 0;
  Core core(Memory::external(), &memory);
  auto initial = initial_state();
  initial.sr = Word{1} << 15;
  initial.la = start + 1;
  initial.lc = 5;
  initial.r[0] = initial.r[4] = first;
  core.set_registers(initial);
  const auto result = core.run(10);
  auto expected = initial;
  ++expected.pc; // The failed instruction was fetched; its fault keeps the original PC.
  require(result.retired == 0 && result.fault.kind == FaultKind::memory_read &&
              result.fault.space == Space::x && result.fault.pc == start &&
              result.fault.address == first && core.registers() == expected,
          "A tracked unknown input is a precise fault, never an initialized RAM prefix");
}

}
}

int main() try {
  dsp56300::test::rebound_windows();
  dsp56300::test::revoked_windows();
  dsp56300::test::owned_loop_access();
  dsp56300::test::owned_loop_store();
  dsp56300::test::rejected_windows();
  dsp56300::test::fragmented_window();
  dsp56300::test::mismatched_window_frame();
  dsp56300::test::aliased_copy_prefixes();
  constexpr dsp56300::Word tail_rows = 5;
  dsp56300::test::prefixes(
      {.first = static_cast<dsp56300::Word>(dsp56300::test::extent) - tail_rows,
       .count = tail_rows,
       .direct = true});
  for (const bool load_b_first : {false, true}) {
    for (const bool store_b_to_x : {false, true}) {
      for (const bool store_b_to_y : {false, true}) {
        for (const dsp56300::Word count : {4U, 5U, 6U, 8U, 9U}) {
          for (const bool direct : {false, true}) {
            dsp56300::test::prefixes({.first = 4,
                                      .count = count,
                                      .mode = 0,
                                      .routing = {.load_b_first = load_b_first,
                                                  .store_b_to_x = store_b_to_x,
                                                  .store_b_to_y = store_b_to_y},
                                      .direct = direct});
          }
        }
      }
    }
  }
  for (const dsp56300::Word first : {1U, 4U, 5U, 28U, 36U}) {
    for (const dsp56300::Word count : {1U, 2U, 3U, 5U, 8U}) {
      for (const auto mode :
           {0U, dsp56300::test::mode_down, dsp56300::test::mode_up, dsp56300::test::mode_rm}) {
        dsp56300::test::prefixes({.first = first, .count = count, .mode = mode});
      }
    }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
