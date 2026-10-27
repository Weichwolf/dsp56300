// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word base = 0x0ac0c0, io_begin = 0xffff80, count_ports = 128;
constexpr Word high = 0x123456, low = 0x654321, low_shifted = 0xca8642;
constexpr Word clear_a = 0x200013, negate_a = 0x200036, target = 384;
constexpr Word ignored_modifier = 0xab8000, alias_stride = 16;
constexpr std::uint64_t accumulator =
    (std::uint64_t{0x80} << 48) | (std::uint64_t{high} << word_bits) | low;

struct Event {
  Space space = Space::x;
  Word address = 0, value = 0;
  bool accept = true;
};

class Script final : public Io {
public:
  explicit Script(std::vector<Event> expected) : expected_(std::move(expected)) {}

  void bind(Core &owner, const Registers &visible, const bool mutate) {
    core_ = &owner;
    visible_ = visible;
    mutate_ = mutate;
  }

  [[nodiscard]] bool reentry_rejected() const { return reject_reentry_; }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    valid_ = false;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    valid_ = false;
    return false;
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    if (core_ == nullptr || next_ == expected_.size()) {
      valid_ = false;
      return false;
    }
    const auto &expected = expected_[next_++];
    valid_ = valid_ && space == expected.space && address == expected.address &&
             value == expected.value && pc == start && core_->registers() == visible_;
    if (mutate_ && next_ == 1) {
      try {
        core_->write(Space::program, start + 1, negate_a);
        static_cast<void>(core_->run(1));
        valid_ = false;
      } catch (const std::logic_error &) { reject_reentry_ = true; } catch (...) {
        valid_ = false;
      }
    }
    return valid_ && expected.accept;
  }

  [[nodiscard]] bool complete() const { return valid_ && next_ == expected_.size(); }

private:
  Core *core_ = nullptr;
  Registers visible_;
  bool mutate_ = false, reject_reentry_ = false;
  std::vector<Event> expected_;
  std::size_t next_ = 0;
  bool valid_ = true;
};

struct PortInput {
  Word address = io_begin;
  unsigned reject = 0;
  bool source_b = false, insert_one = false, absolute = false, mutate = false;
};

void check_result(Core &core,
                  const Script &io,
                  const PortInput input,
                  const RunResult &result,
                  const Registers &visible) {
  const auto shifted = low_shifted + (input.insert_one ? 1U : 0U);
  if (input.reject != 0) {
    require(result.dispatches == 0 && result.progress == Progress{} &&
                result.fault == Fault{.kind = FaultKind::io_write,
                                      .space = input.reject == 1 ? Space::x : Space::y,
                                      .address = input.address,
                                      .value = input.reject == 1 ? high : shifted,
                                      .pc = start},
            "VSL rejected I/O identifies the failing half and does not retire");
    require(core.run(1).dispatches == 0 && io.complete(), "VSL fail-stop never replays I/O");
    return;
  }
  auto expected = visible;
  if (input.mutate) {
    // The successor was replaced from CLR A by NEG A inside the first callback.
    expected.a = static_cast<std::uint64_t>(-signed56(expected.a)) & accumulator_mask;
    sequence_model::flags(expected, {.full = signed56(expected.a)});
    ++expected.pc;
  }
  require(result.dispatches == (input.mutate ? 2U : 1U) && result.fault.kind == FaultKind::none &&
              result.progress ==
                  Progress{.instructions = input.mutate ? 2U : 1U,
                           .estimated_cycles = input.absolute || input.mutate ? 2U : 1U} &&
              core.registers() == expected && core.interruptible(),
          "VSL I/O has exact retirement, preserved source/flags, and fresh successor execution");
  if (input.mutate) { require(io.reentry_rejected(), "VSL callbacks cannot reenter execution"); }
}

void port_case(const Backend backend, const PortInput input) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  std::vector<Event> requests{
      {.space = Space::x, .address = input.address, .value = high, .accept = input.reject != 1}};
  const auto shifted = low_shifted + (input.insert_one ? 1U : 0U);
  if (input.reject != 1) {
    requests.push_back({.space = Space::y,
                        .address = input.address,
                        .value = shifted,
                        .accept = input.reject != 2});
  }
  Script io(std::move(requests));
  Core core({.program = p, .x = x, .y = y}, &io, {.backend = backend});
  const auto field = input.absolute ? 6U : 3U;
  const Word instruction =
      base | (field << 11) | (input.source_b ? 0x010000U : 0U) | (input.insert_one ? 16U : 0U);
  core.write(Space::program, start, instruction);
  core.write(Space::program, start + 1, input.absolute ? input.address : clear_a);
  auto initial = register_model::seed(0);
  initial.sr = status_byte;
  (input.source_b ? initial.b : initial.a) = accumulator;
  initial.r[0] = input.absolute ? target : input.address;
  if (input.absolute) { initial.m[0] = ignored_modifier; }
  core.set_registers(initial);
  auto visible = initial;
  visible.pc = start + (input.absolute ? 2U : 1U);
  if (!input.absolute) { visible.r[0] = (input.address + 1) & word_mask; }
  io.bind(core, visible, input.mutate);
  const auto result = core.run(input.mutate ? 2U : 1U);
  require(io.complete(),
          "VSL reports each raw X/Y store once, in order, with visible AGU/CCR state");
  check_result(core, io, input, result, visible);
}

void ports(const Backend backend) {
  for (Word index = 0; index < count_ports; ++index) {
    for (const bool source_b : {false, true}) {
      for (const bool insert_one : {false, true}) {
        for (const bool absolute : {false, true}) {
          port_case(backend,
                    {.address = io_begin + index,
                     .source_b = source_b,
                     .insert_one = insert_one,
                     .absolute = absolute});
        }
      }
    }
  }
  for (const unsigned reject : {1U, 2U}) {
    port_case(backend, {.address = word_mask, .reject = reject});
  }
  port_case(backend, {.address = io_begin, .mutate = true});
}

void ram_faults(const Backend backend) {
  for (const bool fail_x : {false, true}) {
    std::array<Word, memory_words> p{};
    std::array<Word, memory_words> x{};
    std::array<Word, memory_words> y{};
    const std::span xs{x.data(), fail_x ? target : memory_words};
    const std::span ys{y.data(), fail_x ? memory_words : target};
    Script io({});
    Core core({.program = p, .x = xs, .y = ys}, &io, {.backend = backend});
    core.write(Space::program, start, base | (3U << 11));
    auto initial = register_model::seed(0);
    initial.a = accumulator;
    initial.r[0] = target;
    core.set_registers(initial);
    const auto result = core.run(1);
    require(result.dispatches == 0 && result.progress == Progress{} &&
                result.fault == Fault{.kind = FaultKind::memory_write,
                                      .space = fail_x ? Space::x : Space::y,
                                      .address = target,
                                      .value = fail_x ? high : low_shifted,
                                      .pc = start} &&
                io.complete(),
            "VSL RAM span failure has exact context without peripheral requests");
    require(x[target] == (fail_x ? 0U : high) && y[target] == 0 &&
                core.registers().r[0] == target + 1,
            "VSL exposes one address update and prior X commit on a failing Y store");
    require(core.run(1).dispatches == 0, "A faulted VSL is not replayed");
  }
}

void invalid_before_io(const Backend backend) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Script io({});
  Core core({.program = p, .x = x, .y = y}, &io, {.backend = backend});
  for (const bool immediate : {false, true}) {
    core.reset();
    const auto instruction = immediate ? base | (6U << 11) | (4U << 8) : base | (3U << 11);
    const std::array program{instruction, io_begin};
    core.load(Space::program, start, program);
    auto initial = register_model::seed(0);
    initial.r[0] = io_begin;
    initial.m[0] = ignored_modifier;
    core.set_registers(initial);
    const auto result = core.run(1);
    require(
        result.dispatches == 0 &&
            result.fault.kind ==
                (immediate ? FaultKind::invalid_instruction : FaultKind::unsupported_mode) &&
            io.complete() && core.registers().r == initial.r,
        "VSL illegal immediate destination/unsupported modifier rejects before I/O or AGU commit");
  }
}

enum class Alias : std::uint8_t { x_y, x_program, y_program, all };

void alias_store(const Backend backend, const Alias alias, const Word displacement) {
  std::array<Word, memory_words + alias_stride> shared{};
  std::array<Word, memory_words> separate_p{};
  std::array<Word, memory_words> separate_x{};
  std::array<Word, memory_words> separate_y{};
  const bool xp = alias == Alias::x_program || alias == Alias::all;
  const bool yp = alias == Alias::y_program || alias == Alias::all;
  const bool xy = alias == Alias::x_y;
  const std::span shared_program{shared.data(), memory_words};
  const std::span shared_data{shared.data() + displacement, memory_words};
  const auto p = xp || yp ? shared_program : std::span{separate_p};
  const auto x = xp || xy ? shared_data : std::span{separate_x};
  const auto y = yp || xy ? shared_data : std::span{separate_y};
  Core core({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  const std::array program{base | (4U << 11), clear_a};
  core.load(Space::program, start, program);
  auto initial = initial_state();
  // X writes NEG A; Y writes NOP. Aliasing all spaces makes the Y store win.
  initial.a = std::uint64_t{negate_a} << word_bits;
  initial.r[0] = xp || yp ? start + 1 - displacement : target;
  core.set_registers(initial);
  const auto result = core.run(2);
  const auto next = xp && !yp ? negate_a : 0U;
  if (xp || yp) {
    require(p[start + 1] == next, "VSL P aliases receive raw stores in defined X-then-Y order");
  }
  auto expected = initial;
  sequence_model::alu(expected, initial, xp || yp ? next : clear_a);
  expected.pc = start + 2;
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
              core.registers() == expected,
          "VSL aliased stores invalidate the translated successor before executing it");
  if (xy) {
    require(shared_data[target] == 0, "The later Y half is visible in aliased X/Y memory");
  }
  if (backend == Backend::jit && (xp || yp)) {
    require(core.jit_statistics().invalidations != 0, "VSL alias stores invalidate native code");
  }
}

void alias_extension(const Backend backend, const bool y_alias) {
  // Place the cached successor and its extension on different decode pages.
  constexpr Word extension = 1024;
  constexpr Word entry = extension - 2;
  constexpr Word immediate_a = 0x56f400;
  std::array<Word, extension + 1> shared{};
  std::array<Word, extension + 1> other{};
  Core core({.program = shared,
             .x = y_alias ? std::span{other} : std::span{shared},
             .y = y_alias ? std::span{shared} : std::span{other}},
            nullptr,
            {.backend = backend});
  const std::array program{base | (4U << 11), immediate_a, Word{1}};
  core.load(Space::program, entry, program);
  auto initial = initial_state();
  initial.a = std::uint64_t{2} << word_bits;
  initial.r[0] = extension;
  initial.pc = extension - 1;
  core.set_registers(initial);
  require(core.run(1).dispatches == 1,
          "VSL extension-alias successor is cached before modification");
  initial.pc = entry;
  core.set_registers(initial);
  const auto result = core.run(2);
  auto expected = initial;
  expected.a = std::uint64_t{y_alias ? 0U : 2U} << word_bits;
  expected.pc = extension + 1;
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 3, .estimated_cycles = 5} &&
              core.registers() == expected && shared[extension] == (y_alias ? 0U : 2U),
          "VSL invalidates an aliased extension and its preceding opcode across a decode-page "
          "boundary");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::ports(backend);
      dsp56300::test::ram_faults(backend);
      dsp56300::test::invalid_before_io(backend);
      for (const bool y_alias : {false, true}) {
        dsp56300::test::alias_extension(backend, y_alias);
      }
      for (const auto alias : {dsp56300::test::Alias::x_y,
                               dsp56300::test::Alias::x_program,
                               dsp56300::test::Alias::y_program,
                               dsp56300::test::Alias::all}) {
        for (const dsp56300::Word displacement : {0U, dsp56300::test::alias_stride}) {
          dsp56300::test::alias_store(backend, alias, displacement);
        }
      }
    }
    std::cout << dsp56300::test::checks << " VSL host-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
