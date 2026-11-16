// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word lf = Word{1} << 15, fv = Word{1} << 16;
constexpr Word do_indirect = isa::opcode<isa::Opcode::do_effective>()[0];
constexpr Word rep_indirect = isa::opcode<isa::Opcode::rep_effective>()[0];
constexpr Word relative_bit = 0x10;
constexpr Word io_begin = 0xffff80, source_count = 3, payload = 0xab000003;
constexpr Word poison_body = 0x000001, outer_la = 0x70, outer_lc = 7;
constexpr Word rep_register = isa::opcode<isa::Opcode::rep_register>()[0] | (4U << 8);
constexpr Word rep_short = isa::opcode<isa::Opcode::rep_absolute_short>()[0] | (3U << 8);
constexpr Word rep_effective = isa::opcode<isa::Opcode::rep_effective>()[0] | (4U << 11);
constexpr Word repeat_one = isa::opcode<isa::Opcode::rep_immediate>()[0] | (1U << 8);
constexpr Word data_address = 0x180;
constexpr std::size_t sequential_variant = 12, endings_variant = 13;

struct Request {
  Space space = Space::x;
  Word port = io_begin;
  bool repeat = false, relative = false, accept = true, replace = false;
};

class CountIo final : public Io {
public:
  explicit CountIo(const Request request) : request_(request) {}

  void attach(Core &core) { core_ = &core; }

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
        core_->load(Space::program, next, isa::nop);
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
  Core *core_ = nullptr;
  bool observed_ = false;
  unsigned calls_ = 0;
};

struct Machine {
  explicit Machine(Io *io = nullptr) : core(Memory::separate({.program = p, .x = x, .y = y}), io) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

constexpr std::array repeat_chain{repeat_one,
                                  isa::nop[0],
                                  rep_register,
                                  isa::nop[0],
                                  rep_short,
                                  isa::nop[0],
                                  rep_effective,
                                  isa::nop[0],
                                  isa::nop[0]};
constexpr std::array<Word, 8> ending_program{};

std::vector<Word> program(const Request request) {
  const Word opcode = (request.repeat ? rep_indirect : do_indirect) |
                      (request.relative ? relative_bit : 0U) | (3U << 11) |
                      (request.space == Space::y ? Word{1} << 6 : 0U);
  if (request.repeat) { return {opcode, isa::nop[0], isa::nop[0], isa::nop[0]}; }
  constexpr Word last = start + 4;
  return {opcode, request.relative ? last - start : last, isa::nop[0], isa::nop[0], isa::nop[0]};
}

std::size_t variant_id(const Request request) {
  const std::size_t space = request.space == Space::y ? 1U : 0U;
  std::size_t kind = request.relative ? 1U : 0U;
  if (request.repeat) { kind = 2; }
  return (space * 3U + kind) * 2U;
}

NativeImage prepare() {
  std::array<std::vector<Word>, 6> words;
  std::array<CodeSpan, endings_variant + 1> programs{};
  std::array<Word, endings_variant + 1> entries{};
  std::vector<CodeTransition> transitions;
  for (const auto space : {Space::x, Space::y}) {
    for (unsigned kind = 0; kind < 3; ++kind) {
      const Request request{.space = space, .repeat = kind == 2, .relative = kind == 1};
      const auto variant = variant_id(request);
      auto &source = words[variant / 2];
      source = program(request);
      const auto header = request.repeat ? 1U : 2U;
      programs[variant] = {.address = start, .words = std::span(source).first(header)};
      programs[variant + 1] = {.address = start, .words = source};
      entries[variant] = start;
      entries[variant + 1] = start + header;
      transitions.push_back({.from = variant, .to = variant + 1, .entry = start + header});
    }
  }
  programs[sequential_variant] = {.address = start, .words = repeat_chain};
  programs[endings_variant] = {.address = start, .words = ending_program};
  entries[sequential_variant] = entries[endings_variant] = start;
  return compile_programs(programs, entries, transitions);
}

void port_count(const NativeImage &image, const Request request) {
  CountIo io(request);
  Machine machine(&io);
  io.attach(machine.core);
  auto words = program(request);
  words[request.repeat ? 1U : 2U] = poison_body;
  machine.core.load(Space::program, start, words);
  const auto variant = variant_id(request);
  machine.core.install(image, variant);
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
              machine.core.registers().lc == source_count,
          "Loop count response is masked to 24 bits without fabricating a retirement");
  if (request.replace) {
    require(entry.yielded && machine.p[start + (request.repeat ? 1U : 2U)] == isa::nop[0],
            "The actual count callback writes the declared successor before handing off");
    machine.core.install(image, variant + 1);
    require(machine.core.interrupt_ready() != request.repeat,
            "After binding the real successor, only REP excludes interrupts");
    const auto next = machine.core.run(1);
    require(
        next.retired == 1 && next.fault.kind == FaultKind::none && io.calls() == 1,
        "Count callback can invalidate the fetched block's successor before its first execution");
  }
}

void ports(const NativeImage &image) {
  for (const auto space : {Space::x, Space::y}) {
    for (Word port = io_begin; port <= word_mask; ++port) {
      port_count(image, {.space = space, .port = port, .replace = true});
      port_count(image, {.space = space, .port = port, .relative = true, .replace = true});
      port_count(image, {.space = space, .port = port, .repeat = true, .replace = true});
    }
    for (const bool repeat : {false, true}) {
      port_count(image, {.space = space, .repeat = repeat, .accept = false});
    }
  }
}

void sequential_repeats(const NativeImage &image) {
  Machine machine;
  machine.core.load(Space::program, start, repeat_chain);
  machine.x[3] = 2;
  machine.x[data_address] = 2;
  machine.core.install(image, sequential_variant);
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

void warm_endings(const NativeImage &image) {
  Machine machine;
  machine.core.load(Space::program, start, ending_program);
  machine.core.install(image, endings_variant);
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
    const auto image = dsp56300::test::prepare();
    dsp56300::test::ports(image);
    dsp56300::test::sequential_repeats(image);
    dsp56300::test::warm_endings(image);
    std::cout << dsp56300::test::checks << " loop host-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
