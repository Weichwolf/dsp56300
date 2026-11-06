// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16;
constexpr Word do_indirect = 0x064000, rep_indirect = 0x064020, relative_bit = 0x10;
constexpr Word io_begin = 0xffff80, source_count = 3, payload = 0xab000003;
constexpr Word poison_body = 0x000001, outer_la = 0x70, outer_lc = 7;
constexpr Word rep_register = 0x06c420, rep_short = 0x060320, rep_effective = 0x066020;
constexpr Word repeat_one = 0x0601a0, data_address = 0x180;

struct Request {
  Space space = Space::x;
  Word port = io_begin;
  bool repeat = false, relative = false, accept = true, replace = false;
};

class CountIo final : public Io {
public:
  explicit CountIo(const Request request) : request_(request) {}

  void attach(TestCore &core) { core_ = &core; }

  [[nodiscard]] bool observed() const { return observed_; }

  [[nodiscard]] unsigned calls() const { return calls_; }

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    ++calls_;
    const auto state = core_->registers();
    const Word expected_sp = request_.repeat ? 2U : 3U;
    const Word expected_sc = request_.repeat ? 2U : 3U;
    const Word next = start + (request_.repeat ? 1U : 2U);
    observed_ = space == request_.space && address == request_.port && pc == start &&
                state.pc == next && state.sp == expected_sp && state.sc == expected_sc &&
                state.lc == outer_lc && state.r[0] == ((request_.port + 1) & word_mask);
    if (!request_.repeat) {
      observed_ =
          observed_ && state.stack[3] == ((std::uint64_t{outer_la} << word_bits) | outer_lc);
    }
    if (request_.replace) {
      try {
        core_->write(Space::program, next, 0);
      } catch (const std::exception &) { observed_ = false; }
    }
    value = payload;
    return request_.accept;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
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

private:
  Request request_;
  TestCore *core_ = nullptr;
  bool observed_ = false;
  unsigned calls_ = 0;
};

struct Machine {
  Machine(const Backend backend, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, {.backend = backend}) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  TestCore core;
};

void port_count(const Backend backend, const Request request) {
  CountIo io(request);
  Machine machine(backend, &io);
  io.attach(machine.core);
  const Word opcode = (request.repeat ? rep_indirect : do_indirect) |
                      (request.relative ? relative_bit : 0U) | (3U << 11) |
                      (request.space == Space::y ? Word{1} << 6 : 0U);
  const Word last = start + 4;
  if (request.repeat) {
    machine.core.load(Space::program, start, std::array{opcode, poison_body, 0U, 0U});
  } else {
    machine.core.load(
        Space::program,
        start,
        std::array{opcode, request.relative ? last - start : last, poison_body, 0U, 0U});
  }
  auto initial = register_model::seed(0);
  initial.sr = flag_c;
  initial.la = outer_la;
  initial.lc = outer_lc;
  initial.r[0] = request.port;
  machine.core.set_registers(initial);
  const auto entry = machine.core.run(1);
  require(io.calls() == 1 && io.observed(),
          "Loop count reads one real host response at the specified EA/first-push boundary");
  if (!request.accept) {
    require(entry.retired == 0 && entry.progress == Progress{} &&
                entry.fault == Fault{.kind = FaultKind::io_read,
                                     .space = request.space,
                                     .address = request.port,
                                     .pc = start},
            "Rejected count response cannot retire or fabricate a count");
    return;
  }
  require(entry.retired == 1 && entry.fault.kind == FaultKind::none &&
              machine.core.registers().lc == source_count &&
              machine.core.interrupt_ready() != request.repeat,
          "Loop count response is masked to 24 bits with correct REP interrupt exclusion");
  if (request.replace) {
    const auto next = machine.core.run(1);
    require(
        next.retired == 1 && next.fault.kind == FaultKind::none && io.calls() == 1,
        "Count callback can invalidate the fetched block's successor before its first execution");
  }
}

void ports(const Backend backend) {
  for (const auto space : {Space::x, Space::y}) {
    for (Word port = io_begin; port <= word_mask; ++port) {
      port_count(backend, {.space = space, .port = port, .replace = true});
      port_count(backend, {.space = space, .port = port, .relative = true, .replace = true});
      port_count(backend, {.space = space, .port = port, .repeat = true, .replace = true});
    }
    for (const bool repeat : {false, true}) {
      port_count(backend, {.space = space, .repeat = repeat, .accept = false});
    }
  }
}

void sequential_repeats(const Backend backend) {
  Machine machine(backend);
  const std::array program{repeat_one, 0U, rep_register, 0U, rep_short, 0U, rep_effective, 0U, 0U};
  machine.core.load(Space::program, start, program);
  machine.core.write(Space::x, 3, 2);
  machine.core.write(Space::x, data_address, 2);
  auto state = register_model::seed(0);
  state.x = 2;
  state.r[0] = data_address;
  machine.core.set_registers(state);
  for (unsigned part = 0; part < 4; ++part) {
    const auto header = machine.core.run(1);
    require(header.retired == 1 && header.fault.kind == FaultKind::none &&
                !machine.core.interrupt_ready(),
            "Every REP form starts an uninterruptible interval");
    const auto body = machine.core.run(part == 0 ? 1U : 2U);
    require(body.fault.kind == FaultKind::none && machine.core.registers().lc == state.lc &&
                machine.core.interrupt_ready() == (part == 3),
            "Consecutive immediate/register/short/EA REP excludes IRQs until the whole chain ends");
  }
}

void warm_endings(const Backend backend) {
  Machine machine(backend);
  machine.core.load(Space::program, start, std::array{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U});
  for (const bool infinite : {false, true, false, true}) {
    for (const Word count : {0U, 1U, 2U, word_mask}) {
      for (const Word budget : {1U, 2U, 7U}) {
        auto initial = register_model::seed(0);
        initial.sr = lf | (infinite ? fv : 0U) | flag_c;
        initial.la = start;
        initial.lc = count;
        initial.stack[2] = std::uint64_t{start} << word_bits;
        initial.stack[1] = (std::uint64_t{outer_la} << word_bits) | outer_lc;
        auto expected = initial;
        if (!infinite && count != 0 && budget >= count) {
          expected.pc = start + 1 + budget - count;
          expected.sr &= ~(lf | fv);
          expected.la = outer_la;
          expected.lc = outer_lc;
          expected.sp = 0;
          expected.sc = 0;
        } else {
          expected.lc = (count + address_words - budget) % address_words;
        }
        machine.core.set_registers(initial);
        const auto before = machine.core.progress();
        const auto result = machine.core.run(budget);
        require(
            result.retired == budget && result.fault.kind == FaultKind::none &&
                machine.core.registers() == expected &&
                result.progress.instructions - before.instructions == budget &&
                result.progress.estimated_cycles - before.estimated_cycles == budget,
            "Warm shared native key observes current FV and LC zero/one without exceeding budget");
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend : {dsp56300::Backend::jit}) {
      dsp56300::test::ports(backend);
      dsp56300::test::sequential_repeats(backend);
      dsp56300::test::warm_endings(backend);
    }
    std::cout << dsp56300::test::checks << " loop host-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
