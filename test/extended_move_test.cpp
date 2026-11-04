// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "move_model.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
namespace {
using move_model::Form;
using move_model::Kind;
constexpr Word round_a = 0x200011;
constexpr Word io_begin = 0xffff80, mac_a = 0x2000d2, mac_b = 0x2000da, adc_x_a = 0x200021;
constexpr Word linear_high = 0xabffff;
constexpr Word location = 384, input_high = 0xab923456, input_low = 0xcd765432;
constexpr std::uint64_t a_value = 0x01123456800000, b_value = 0xfe800000000001;

std::vector<Form> forms() {
  std::vector<Form> result;
  for (unsigned selector = 0; selector < 8; ++selector) {
    for (const bool load : {false, true}) {
      result.push_back({.selector = selector, .mode = 3, .load = load});
    }
  }
  for (const bool y : {false, true}) {
    for (unsigned selector = 0; selector < 4; ++selector) {
      for (const bool source_b : {false, true}) {
        for (const bool second_high : {false, true}) {
          for (const bool load : {false, true}) {
            result.push_back({.kind = Kind::mixed_one,
                              .selector = selector,
                              .mode = 3,
                              .load = load,
                              .y = y,
                              .source_b = source_b,
                              .second_high = second_high});
          }
        }
      }
      if (selector < 2) {
        result.push_back(
            {.kind = Kind::mixed_two, .selector = selector, .mode = 3, .load = false, .y = y});
      }
    }
  }
  return result;
}

class Observer final : public Io {
public:
  bool read(const Space space,
            const Word actual_address,
            const Word actual_pc,
            Word &value) noexcept override {
    value = calls == 0 ? input_high : input_low;
    return access(space, actual_address, actual_pc, false, 0);
  }

  bool write(const Space space,
             const Word actual_address,
             const Word actual_pc,
             const Word value) noexcept override {
    return access(space, actual_address, actual_pc, true, value);
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  Core *core = nullptr;
  Registers expected;
  Progress progress;
  move_model::Output output;
  Word address = 0, pc = start, replacement = 0;
  unsigned calls = 0, fail_at = 0;
  bool long_word = true, load = true, valid = true;
  Space first_space = Space::x;

private:
  bool access(const Space space,
              const Word actual_address,
              const Word actual_pc,
              const bool write,
              const Word value) noexcept {
    ++calls;
    const auto expected_space = long_word && calls == 2 ? Space::y : first_space;
    const auto expected_value = calls == 2 ? output.second : output.first;
    valid = valid && calls <= (long_word ? 2U : 1U) && space == expected_space &&
            actual_address == address && actual_pc == pc && write != load &&
            (!write || value == expected_value) && core->registers() == expected &&
            core->progress() == progress;
    try {
      bool rejected = false;
      try {
        static_cast<void>(core->run(1));
      } catch (const std::logic_error &) { rejected = true; }
      valid = valid && rejected;
      if (replacement != 0) { core->write(Space::program, replacement, adc_x_a); }
    } catch (...) { valid = false; }
    return calls != fail_at;
  }
};

Registers initial(const Word address) {
  auto state = register_model::seed(0);
  state.a = a_value;
  state.b = b_value;
  state.sr = flag_c;
  state.r[0] = address;
  state.m[0] = linear_high;
  return state;
}

void configure(Observer &io,
               const Form form,
               const Registers &before,
               const Word address,
               const move_model::Output output,
               const unsigned fail_at) {
  io.expected = before;
  static_cast<void>(move_model::address(io.expected, form));
  if (form.kind == Kind::long_word) {
    if (!form.load) { static_cast<void>(move_model::store_long(io.expected, form.selector)); }
  } else {
    const auto accumulator = form.source_b ? 15U : 14U;
    if (form.kind == Kind::mixed_two) {
      static_cast<void>(register_model::read(io.expected, 14U + form.selector));
    } else {
      static_cast<void>(register_model::read(io.expected, accumulator));
      if (!form.load) {
        static_cast<void>(register_model::read(io.expected,
                                               form.y ? move_model::yregs.at(form.selector)
                                                      : move_model::xregs.at(form.selector)));
      }
    }
  }
  io.address = address;
  io.output = output;
  io.long_word = form.kind == Kind::long_word;
  io.load = form.load && form.kind != Kind::mixed_two;
  io.first_space = form.y ? Space::y : Space::x;
  io.calls = 0;
  io.fail_at = fail_at;
  io.valid = true;
}

Fault expected_fault(const Observer &io) {
  if (io.fail_at == 0) { return {}; }
  Word value = 0;
  if (!io.load) { value = io.fail_at == 2 ? io.output.second : io.output.first; }
  return {.kind = io.load ? FaultKind::io_read : FaultKind::io_write,
          .space = io.fail_at == 2 ? Space::y : io.first_space,
          .address = io.address,
          .value = value,
          .pc = io.pc};
}

void port_case(Core &core, Observer &io, const Form form, const Word port, const unsigned fail_at) {
  core.reset();
  auto state = initial(port);
  core.set_registers(state);
  auto expected = state;
  const auto output = move_model::apply(expected, form);
  configure(io, form, state, port, output, fail_at);
  ++io.expected.pc;
  ++expected.pc;
  if (fail_at != 0) { expected = io.expected; }
  const auto result = core.run(1);
  const unsigned successful_calls = io.long_word ? 2U : 1U;
  const auto expected_calls = fail_at == 0 ? successful_calls : fail_at;
  require(io.valid && io.calls == expected_calls && core.registers() == expected,
          "Long/mixed I/O observes ordered old sources, canonical state and successful-only "
          "destinations");
  require(result.fault == expected_fault(io) && result.dispatches == (fail_at == 0 ? 1U : 0U) &&
              result.progress ==
                  (fail_at == 0 ? Progress{.instructions = 1, .estimated_cycles = 1} : Progress{}),
          "Long/mixed I/O failure has exact half/fault/retirement accounting");
  if (fail_at != 0) {
    require(
        core.run(2).dispatches == 0 && core.registers() == expected && io.calls == fail_at,
        "Rejected long/mixed I/O never executes the later half or replays the stopped instruction");
  }
  require(core.read(io.first_space, port) == 0,
          "Caller RAM spanning the entire DSP address range cannot bypass I/O callbacks");
}

void extension_wrap(Core &core, Form form) {
  if (form.kind == Kind::mixed_two) { return; }
  core.reset();
  form.mode = 6;
  form.index = 0;
  form.extension = location;
  const std::array tail{adc_x_a, round_a};
  core.write(Space::program, word_mask, move_model::opcode(form));
  core.write(Space::program, 0, location);
  core.load(Space::program, 1, tail);
  const move_model::Data data;
  core.write(Space::x, location, data.high);
  core.write(Space::y, location, form.kind == Kind::long_word ? data.low : data.high);
  auto state = initial(location);
  state.pc = word_mask;
  core.set_registers(state);
  auto expected = state;
  const auto output = move_model::apply(expected, form);
  expected.pc = 1;
  for (const auto operation : tail) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  const auto result = core.run(3);
  require(result.dispatches == 3 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 3, .estimated_cycles = 4} &&
              core.registers() == expected,
          "Long/mixed P extensions wrap at 24 bits and resume exact native arithmetic");
  if (output.stores != 0) {
    require(core.read(output.space, location) == output.first,
            "A wrapped extension retains the exact first data-memory store");
    if (output.stores == 2) {
      require(core.read(Space::y, location) == output.second,
              "A wrapped long extension retains the exact second data-memory store");
    }
  }
}

void ports(const Backend backend) {
  const MemoryArena arena;
  Observer io;
  Core core(arena.buffers(), &io, {.backend = backend});
  io.core = &core;
  for (const auto form : forms()) {
    core.write(Space::program, start, move_model::opcode(form));
    for (unsigned fail_at = 0; fail_at <= (form.kind == Kind::long_word ? 2U : 1U); ++fail_at) {
      for (Word port = io_begin; port <= word_mask; ++port) {
        port_case(core, io, form, port, fail_at);
      }
    }
    extension_wrap(core, form);
  }
}

void native_boundary(const Backend backend, const Word alu, const Form form, const bool accept) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Observer io;
  Core core({.program = p, .x = x, .y = y}, &io, {.backend = backend});
  io.core = &core;
  const std::array program{mac_a, mac_b, move_model::opcode(form, alu), Word{0}};
  core.load(Space::program, start, program);
  const auto state = initial(io_begin);
  core.set_registers(state);
  auto before = state;
  for (const auto operation : {mac_a, mac_b}) {
    const auto old = before;
    sequence_model::alu(before, old, operation);
  }
  before.pc = start + 2;
  auto expected = before;
  const auto output = move_model::apply(expected, form);
  configure(io, form, before, io_begin, output, accept ? 0U : 1U);
  io.progress = {.instructions = 2, .estimated_cycles = 2};
  io.pc = start + 2;
  io.replacement = start + 3;
  ++io.expected.pc;
  ++expected.pc;
  if (accept) {
    if (alu != 0) { sequence_model::alu(expected, before, alu); }
    const auto old = expected;
    sequence_model::alu(expected, old, adc_x_a);
    ++expected.pc;
  } else {
    expected = io.expected;
  }
  const auto result = core.run(4);
  require(io.valid && core.registers() == expected && result.fault == expected_fault(io) &&
              result.dispatches == (accept ? 4U : 2U),
          "Dirty native A/B materialize before long/mixed callbacks; changed successors consume "
          "new data");
  if (backend == Backend::jit) {
    const bool emitted_nop = form.kind == Kind::long_word && !form.load && alu == 0;
    const auto native = accept ? 3U + (alu != 0 || emitted_nop ? 1U : 0U) : 2U;
    require(core.jit_statistics().native_alu_instructions == native &&
                core.jit_statistics().native_move_instructions == 0,
            "Native arithmetic/NOP retirement surrounds genuine I/O callbacks, with no native "
            "RAM transfer claimed for a peripheral side exit");
  }
}

void memory_failure(const Backend backend, const Form form, const bool second) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  const auto xsize = !second && !form.y ? 1U : memory_words;
  const auto ysize = second || form.y ? 1U : memory_words;
  Core core({.program = p, .x = std::span(x).first(xsize), .y = std::span(y).first(ysize)},
            nullptr,
            {.backend = backend});
  auto state = initial(location);
  core.write(Space::program, start, move_model::opcode(form));
  core.set_registers(state);
  auto expected = state;
  const auto output = move_model::apply(expected, form);
  Observer io;
  configure(io, form, state, location, output, 1);
  ++io.expected.pc;
  const auto result = core.run(1);
  const auto failed_space = second || form.y ? Space::y : Space::x;
  const bool load = form.kind != Kind::mixed_two && form.load;
  const auto value = second ? output.second : output.first;
  require(result.dispatches == 0 && result.progress == Progress{} &&
              result.fault == Fault{.kind = load ? FaultKind::memory_read : FaultKind::memory_write,
                                    .space = failed_space,
                                    .address = location,
                                    .value = load ? 0U : value,
                                    .pc = start} &&
              core.registers() == io.expected,
          "Missing long/mixed RAM stops without fabricated destination data or secondary commits");
  if (second && !load) {
    require(core.read(Space::x, location) == output.first,
            "A failed Y half preserves the already accepted X write");
  }
}

void program_alias(const Backend backend, const Form form) {
  if (form.load && form.kind != Kind::mixed_two) { return; }
  constexpr Word negate = 0x200036;
  constexpr Word clear = 0x200013;
  const auto pair = (std::uint64_t{clear} << word_bits) | negate;
  std::array<Word, memory_words> p{};
  Core core({.program = p, .x = p, .y = p}, nullptr, {.backend = backend});
  auto state = initial(start + 1);
  state.x = (std::uint64_t{negate} << word_bits) | negate;
  state.y = state.x;
  state.a = std::uint64_t{negate} << word_bits;
  state.b = state.a;
  if (form.kind == Kind::long_word) {
    state.x = pair;
    state.y = pair;
    state.a = pair;
    state.b = pair;
    if (form.selector >= 6) {
      state.a = std::uint64_t{form.selector == 6 ? clear : negate} << word_bits;
      state.b = std::uint64_t{form.selector == 6 ? negate : clear} << word_bits;
    }
  }
  const std::array program{move_model::opcode(form), Word{0}, adc_x_a};
  core.load(Space::program, start, program);
  core.set_registers(state);
  auto expected = state;
  const auto output = move_model::apply(expected, form);
  const auto replacement = output.stores == 2 ? output.second : output.first;
  require(replacement == negate, "The independent alias model constructs the intended successor");
  ++expected.pc;
  for (const auto operation : {negate, adc_x_a}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  const auto result = core.run(3);
  require(
      result.dispatches == 3 && result.fault.kind == FaultKind::none &&
          core.registers() == expected && core.read(Space::program, start + 1) == negate,
      "Long/mixed guest stores invalidate P aliases; the second long half wins a shared X/Y word");
  if (backend == Backend::jit) {
    require(core.jit_statistics().native_alu_instructions >= 2,
            "The alias witness executes the replacement and its consumer as native arithmetic");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::ports(backend);
      for (const auto form : dsp56300::test::forms()) {
        for (const dsp56300::Word alu : {0U, 0x45U}) {
          for (const bool accept : {false, true}) {
            dsp56300::test::native_boundary(backend, alu, form, accept);
          }
        }
        dsp56300::test::program_alias(backend, form);
        dsp56300::test::memory_failure(backend, form, false);
        if (form.kind == dsp56300::test::Kind::long_word) {
          dsp56300::test::memory_failure(backend, form, true);
        }
      }
    }
    std::cout << dsp56300::test::checks
              << " extended MOVE transfer and native-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << dsp56300::test::checks << " checks before failure: " << error.what() << '\n';
    return 1;
  }
}
