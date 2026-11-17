// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "installed_program.hpp"
#include "isa.hpp"
#include "loop_frames.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
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

constexpr Word suffix_first = start + 1, suffix_last = suffix_first + 3, suffix_literal = 0x654321;
constexpr Word suffix_linear = 0xffff;
constexpr std::array suffix_program{
    isa::opcode<isa::Opcode::jmp_absolute_short>()[0] | suffix_first,
    isa::opcode<isa::Opcode::move_x_effective>()[0] | (3U << 11),
    isa::nop[0],
    isa::opcode<isa::Opcode::move_x_effective>()[0] | (6U << 11) | (4U << 8) | (1U << 15),
    suffix_literal,
    isa::stop[0]};

void suffix_case(const NativeImage &image,
                 const Registers &initial,
                 Word quota,
                 const bool deadline) {
  Machine machine;
  machine.core.load(Space::program, start, suffix_program);
  machine.core.set_registers(initial);
  machine.core.install(image, 0);
  auto expected = initial;
  std::array<Word, memory_words> x{};
  auto available = suffix_first + 3 - initial.pc;
  if ((initial.sr & lf) != 0 && initial.la >= initial.pc && initial.la < suffix_first + 2) {
    available = std::min(available, initial.la - initial.pc + 1);
  }
  quota = std::min(quota, available);
  const bool stored = initial.pc == suffix_first && quota != 0;
  const bool extended = initial.pc + quota > suffix_first + 2;
  if (stored) {
    x.at(initial.r[0]) = static_cast<Word>(initial.x) & word_mask;
    ++expected.r[0];
  }
  if (extended) { expected.x = (initial.x & (long_mask ^ word_mask)) | suffix_literal; }
  expected.pc += quota + (extended ? 1U : 0U);
  const Progress progress{.instructions = quota, .estimated_cycles = quota + (extended ? 1U : 0U)};
  if ((initial.sr & lf) != 0 && quota != 0 && expected.pc - 1 == initial.la) {
    if (initial.lc != 1 || (initial.sr & fv) != 0) {
      expected.lc = (initial.lc - 1) & word_mask;
      expected.pc = start;
    } else {
      loop_frames::complete(expected);
    }
  }
  const auto result =
      deadline ? machine.core.run_until(progress.estimated_cycles) : machine.core.run(quota);
  require(
      result.retired == progress.instructions && result.progress == progress &&
          result.fault == Fault{} && !result.yielded && machine.core.registers() == expected &&
          machine.x == x && machine.y == std::array<Word, memory_words>{},
      "DO suffix preserves registers, RAM, frames and exact instruction/deadline/cycle boundary");
}

void suffix_endings() {
  constexpr std::array<std::array<Word, 7>, 13> points{{{0, 0, 0, suffix_last, 0, 0, 2},
                                                        {fv, 1, 2, suffix_last, 0, 1, 2},
                                                        {lf, 0, 63, suffix_last, lf, 0, 4},
                                                        {lf, 1, 0, suffix_last, lf | fv, 0, 4},
                                                        {lf, 2, 2, suffix_last, 0, 0, 2},
                                                        {lf | fv, 1, 63, suffix_last, 0, 1, 2},
                                                        {lf, word_mask, 2, suffix_last, fv, 1, 4},
                                                        {lf, 1, 63, suffix_first, lf, 0, 4},
                                                        {lf, 2, 0, suffix_first + 1, 0, 0, 2},
                                                        {lf, 1, 2, suffix_first + 2, 0, 0, 2},
                                                        {lf, 1, 63, outer_la, 0, 0, 2},
                                                        {lf | fv, 0, 0, suffix_last, lf, 0, 4},
                                                        {lf, 1, 0, suffix_first + 1, fv, 1, 4}}};
  Machine arena;
  const auto image =
      compile_program(suffix_program, start, {.program = arena.p, .x = arena.x, .y = arena.y});
  for (const Word mode :
       {0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up}) {
    for (const auto &[flags, count, pointer, last, outer, offset, depth] : points) {
      auto state = register_model::seed(mode);
      state.sr = mode | flags | flag_c;
      state.omr = 0;
      state.pc = suffix_first + offset;
      state.la = last;
      state.lc = count;
      state.sp = pointer;
      state.sc = depth;
      state.r[0] = 1;
      state.m.fill(suffix_linear);
      state.stack[state.sp % stack_entries] = (std::uint64_t{start} << word_bits) | outer;
      state.stack[((state.sp + loop_frames::pointer_mask) & loop_frames::pointer_mask) %
                  stack_entries] = (std::uint64_t{outer_la} << word_bits) | outer_lc;
      for (const Word quota : {0U, 1U, 2U, 3U}) {
        for (const bool deadline : {false, true}) { suffix_case(image, state, quota, deadline); }
      }
    }
  }
}

constexpr Word complete_body = start + 2, complete_last = complete_body + 1;
constexpr Word complete_count = 3;
constexpr unsigned complete_instructions = 1 + (2 * complete_count);
constexpr unsigned complete_cycles = 5 + (2 * complete_count);

void complete_do_case(const NativeImage &image,
                      const std::span<const Word> code,
                      Registers expected,
                      const unsigned limit,
                      const bool deadline) {
  Machine machine;
  machine.core.load(Space::program, start, code);
  machine.core.install(image, 0);
  machine.core.set_registers(expected);
  const auto retired = deadline && limit != 0 ? 1 + (limit > 5 ? limit - 5 : 0) : limit;
  std::array<Word, memory_words> memory{};
  if (retired != 0) {
    const auto push = [&](const Word high, const Word low) {
      expected.sp = (expected.sp + 1) & loop_frames::pointer_mask;
      ++expected.sc;
      expected.stack[expected.sp % stack_entries] = (std::uint64_t{high} << word_bits) | low;
    };
    push(expected.la, expected.lc);
    expected.lc = complete_count;
    push(complete_body, expected.sr);
    expected.la = complete_last;
    expected.sr = (expected.sr & ~fv) | lf;
    expected.pc = complete_body;
    for (unsigned index = 1; index < retired; ++index) {
      if (expected.pc == complete_body) {
        memory.at(expected.r[0]++) = static_cast<Word>(expected.x) & word_mask;
      }
      ++expected.pc;
      if (expected.pc != complete_last + 1) { continue; }
      if (expected.lc == 1) {
        loop_frames::complete(expected);
      } else {
        --expected.lc;
        expected.pc = complete_body;
      }
    }
  }
  const auto result = deadline ? machine.core.run_until(limit) : machine.core.run(limit);
  const Progress progress{.instructions = retired,
                          .estimated_cycles = retired == 0 ? 0U : retired + 4};
  require(result.retired == retired && result.progress == progress && result.fault == Fault{} &&
              !result.yielded && machine.core.registers() == expected && machine.x == memory &&
              machine.y == std::array<Word, memory_words>{},
          "Complete DO preserves all RAM, frames and every instruction/time cut");
}

Registers loop_case_state(const unsigned index, const std::span<const Word> depths) {
  constexpr std::array modes{
      0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
  constexpr std::array pointers{0U, 15U, 63U};
  const auto mode = modes[index % modes.size()];
  auto state = register_model::seed(mode);
  state.pc = start;
  state.sr = mode | (static_cast<Word>((index / modes.size()) % 4) << 15) | flag_c;
  state.omr = 0;
  state.la = outer_la;
  state.lc = outer_lc;
  state.sp = pointers[(index / (6 * 4)) % pointers.size()];
  state.sc = depths[(index / (6 * 4 * 3)) % depths.size()];
  state.r[0] = 1;
  state.m.fill(suffix_linear);
  return state;
}

void complete_do() {
  constexpr std::array depths{0U, 2U, static_cast<Word>(stack_entries - 3)};
  constexpr unsigned states = 6 * 4 * 3 * 3;
  for (const bool relative : {false, true}) {
    const std::array code{isa::opcode<isa::Opcode::do_immediate>()[0] | (complete_count << 8) |
                              (relative ? relative_bit : 0U),
                          relative ? complete_last - start : complete_last,
                          isa::opcode<isa::Opcode::move_x_effective>()[0] | (3U << 11),
                          isa::nop[0],
                          isa::stop[0]};
    Machine arena;
    const auto image =
        compile_program(code, start, {.program = arena.p, .x = arena.x, .y = arena.y});
    for (unsigned index = 0; index < states; ++index) {
      const auto state = loop_case_state(index, depths);
      for (const bool deadline : {false, true}) {
        for (unsigned limit = 0; limit <= (deadline ? complete_cycles : complete_instructions);
             ++limit) {
          complete_do_case(image, code, state, limit, deadline);
        }
      }
    }
  }
}

unsigned nested_step(Registers &state,
                     std::array<Word, memory_words> &memory,
                     const std::span<const Word> code) {
  const auto pc = state.pc;
  const auto offset = pc - start;
  const bool setup = offset < 6 && (offset % 2) == 0;
  if (setup) {
    const auto push = [&](const Word high, const Word low) {
      state.sp = (state.sp + 1) & loop_frames::pointer_mask;
      ++state.sc;
      state.stack[state.sp % stack_entries] = (std::uint64_t{high} << word_bits) | low;
    };
    push(state.la, state.lc);
    state.lc = offset == 2 ? 3U : 2U;
    push(pc + 2, state.sr);
    state.sr = (state.sr & ~fv) | lf;
    state.la = start + 9 - (offset / 2);
    if (offset == 4 && (code[6] & status_byte) != 0) { --state.la; }
    state.pc += 2;
  } else {
    const auto before = state;
    if (offset == 6) { memory.at(state.r[0]++) = static_cast<Word>(state.x) & word_mask; }
    sequence_model::alu(state, before, code[offset]);
    ++state.pc;
  }
  if ((state.sr & lf) != 0 && pc + (setup ? 1U : 0U) == state.la && state.pc == state.la + 1) {
    if (state.lc == 1 && (state.sr & fv) == 0) {
      loop_frames::complete(state);
    } else {
      state.lc = (state.lc - 1) & word_mask;
      state.pc = static_cast<Word>(state.stack[state.sp % stack_entries] >> word_bits);
    }
  }
  return setup ? 5U : 1U;
}

void nested_case(const NativeImage &image,
                 const std::span<const Word> code,
                 Registers expected,
                 const unsigned limit,
                 const bool deadline) {
  Machine machine;
  machine.core.load(Space::program, start, code);
  machine.core.install(image, 0);
  machine.core.set_registers(expected);
  std::array<Word, memory_words> memory{};
  Progress progress;
  while ((deadline ? progress.estimated_cycles : progress.instructions) < limit &&
         expected.pc != start + 10) {
    progress.estimated_cycles += nested_step(expected, memory, code);
    ++progress.instructions;
  }
  const auto result = deadline ? machine.core.run_until(limit) : machine.core.run(limit);
  require(result.retired == progress.instructions && result.progress == progress &&
              result.fault == Fault{} && !result.yielded && machine.core.registers() == expected &&
              machine.x == memory && machine.y == std::array<Word, memory_words>{},
          "Nested DO trees preserve every register, scratch frame, RAM word and partial bound");
}

void nested_bounds(const NativeImage &image,
                   const std::span<const Word> code,
                   const Registers &state,
                   const bool captured) {
  auto initial = state;
  std::array<Word, memory_words> scratch{};
  if (captured) {
    static_cast<void>(nested_step(initial, scratch, code));
    initial.sr |= state.sr & fv;
    if ((state.sr & lf) != 0) { initial.lc = 0; }
  }
  const unsigned nested_instructions = (code[6] & status_byte) != 0 ? 35U : 41U;
  const auto nested_cycles = nested_instructions + (4U * (1U + 2U + (2U * 3U)));
  for (const bool deadline : {false, true}) {
    const auto maximum = deadline ? nested_cycles - (captured ? 5U : 0U)
                                  : nested_instructions - (captured ? 1U : 0U);
    for (unsigned limit = 0; limit <= maximum; ++limit) {
      nested_case(image, code, initial, limit, deadline);
    }
  }
}

auto nested_code(const bool relative, const bool arithmetic) {
  const auto loop = [&](const Word count) {
    return isa::opcode<isa::Opcode::do_immediate>()[0] | (count << 8) |
           (relative ? relative_bit : 0U);
  };
  const auto last = [&](const Word offset, const Word distance) {
    return relative ? distance : start + offset + distance;
  };
  return std::array{
      loop(2),
      last(0, 9),
      loop(3),
      last(2, 6),
      loop(2),
      last(4, arithmetic ? 2U : 3U),
      isa::opcode<isa::Opcode::move_x_effective>()[0] | (3U << 11) |
          (arithmetic ? isa::opcode<isa::Opcode::mac_register_a>()[0] & status_byte : 0U),
      isa::nop[0],
      arithmetic ? isa::opcode<isa::Opcode::rnd_a>()[0] : isa::nop[0],
      isa::nop[0],
      isa::stop[0]};
}

void nested_do() {
  constexpr std::array depths{0U, 2U, 9U};
  constexpr unsigned states = 6 * 4 * 3 * 3;
  for (const bool relative : {false, true}) {
    for (const bool arithmetic : {false, true}) {
      const auto code = nested_code(relative, arithmetic);
      Machine arena;
      const auto image =
          compile_program(code, start, {.program = arena.p, .x = arena.x, .y = arena.y});
      for (unsigned index = 0; index < states; ++index) {
        auto state = loop_case_state(index, depths);
        constexpr std::array accumulators{std::uint64_t{0}, sign56 - 1, sign56};
        state.a = accumulators[(index / (6 * 4)) % accumulators.size()];
        for (const bool captured : {false, true}) { nested_bounds(image, code, state, captured); }
      }
    }
  }
}

void nested_late_address() {
  constexpr Word loop = isa::opcode<isa::Opcode::do_immediate>()[0] | (2U << 8);
  constexpr Word store = isa::opcode<isa::Opcode::move_x_effective>()[0] | (3U << 11);
  constexpr Word reset = parallel_none | (4U << 13) | (16U << 8);
  const std::array code{
      loop, start + 7, loop, start + 5, store, isa::nop[0], reset, isa::nop[0], isa::stop[0]};
  Machine machine;
  const auto image =
      compile_program(code, start, {.program = machine.p, .x = machine.x, .y = machine.y});
  machine.core.load(Space::program, start, code);
  machine.core.install(image, 0);
  auto expected = initial_state();
  expected.omr = 0;
  expected.m.fill(suffix_linear);
  expected.x = 1;
  expected.r[0] = memory_words - 1;
  machine.core.set_registers(expected);
  expected.pc = start + 5;
  expected.sr = lf;
  expected.la = start + 5;
  expected.lc = 1;
  expected.sp = expected.sc = 4;
  expected.r[0] = memory_words + 1;
  expected.stack[1] = 0;
  expected.stack[2] = std::uint64_t{start + 2} << word_bits;
  expected.stack[3] = (std::uint64_t{start + 7} << word_bits) | 2U;
  expected.stack[4] = (std::uint64_t{start + 4} << word_bits) | lf;
  const auto result = machine.core.run(100);
  std::array<Word, memory_words> memory{};
  memory.back() = 1;
  require(result.retired == 4 &&
              result.progress == Progress{.instructions = 4, .estimated_cycles = 12} &&
              result.fault == Fault{.kind = FaultKind::memory_write,
                                    .space = Space::x,
                                    .address = memory_words,
                                    .value = 1,
                                    .pc = start + 4} &&
              machine.core.registers() == expected && machine.x == memory &&
              machine.y == std::array<Word, memory_words>{},
          "Nested admission proves every repeated RAM access before a later R definition");
}

} // namespace
} // namespace dsp56300::test

int main() {
  try {
    const auto image = dsp56300::test::prepare();
    dsp56300::test::ports(image);
    dsp56300::test::sequential_repeats(image);
    dsp56300::test::warm_endings(image);
    dsp56300::test::suffix_endings();
    dsp56300::test::complete_do();
    dsp56300::test::nested_do();
    dsp56300::test::nested_late_address();
    std::cout << dsp56300::test::checks << " loop host-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
