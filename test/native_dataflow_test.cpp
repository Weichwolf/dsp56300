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
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word mac_a = 0x2000d2, mac_b = 0x2000fa;
constexpr Word add_b_a = 0x200010, add_a_b = 0x200018;
constexpr Word io_add_load_a = 0x56e010, io_address = 0xffff80, datum = 0xfedcba;
constexpr std::array native_chain{mac_a,
                                  mac_b,
                                  add_b_a,
                                  add_a_b,
                                  Word{0x200036},
                                  Word{0x20002e},
                                  Word{0x200011},
                                  Word{0x200005},
                                  Word{0x200069}};

struct Machine {
  explicit Machine(const Options options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

void model_alu(Registers &state, const Word opcode) {
  const auto old = state;
  sequence_model::alu(state, old, opcode);
  ++state.pc;
}

void prefixes(const Options options) {
  Machine machine(options);
  machine.core.load(Space::program, start, native_chain);
  auto initial = register_model::seed(mode_rm | mode_up | flag_l | flag_s);
  initial.sr &= ~flag_c;
  machine.core.set_registers(initial);
  require(machine.core.run(static_cast<Word>(native_chain.size())).dispatches ==
              native_chain.size(),
          "Warm the complete native register chain");
  for (std::size_t length = 1; length <= native_chain.size(); ++length) {
    auto expected = initial;
    for (const auto opcode : std::span(native_chain.data(), length)) {
      model_alu(expected, opcode);
    }
    machine.core.set_registers(initial);
    const auto before = machine.core.progress();
    const auto result = machine.core.run(static_cast<Word>(length));
    require(result.dispatches == length && result.fault.kind == FaultKind::none &&
                machine.core.registers() == expected,
            "Every shorter warm-block prefix exposes complete independently modeled state");
    require(result.progress.instructions == before.instructions + length &&
                result.progress.estimated_cycles == before.estimated_cycles + length,
            "Register reuse preserves exact partial-budget instruction and cycle counts");
    require(machine.core.interruptible(),
            "Native register reuse retains stopped interrupt boundaries");
  }
  auto expected = initial;
  for (const auto opcode : native_chain) { model_alu(expected, opcode); }
  machine.core.set_registers(initial);
  require(machine.core.run(3).dispatches == 3 && machine.core.run(6).dispatches == 6 &&
              machine.core.registers() == expected,
          "A partitioned chain reloads registers at a new native entry");
}

struct HelperInput {
  Options options;
  unsigned source = 4, target = 14;
};

void helper_writes(const HelperInput input) {
  Machine machine(input.options);
  const Word move = parallel_none | (input.source << 13) | (input.target << 8);
  const std::array program{mac_a, mac_b, move, add_b_a, add_a_b};
  machine.core.load(Space::program, start, program);
  const auto initial = register_model::seed(mode_down | flag_s);
  auto expected = initial;
  model_alu(expected, mac_a);
  model_alu(expected, mac_b);
  const auto value = register_model::read(expected, input.source);
  register_model::write(expected, input.target, value);
  ++expected.pc;
  model_alu(expected, add_b_a);
  model_alu(expected, add_a_b);
  machine.core.set_registers(initial);
  const auto result = machine.core.run(static_cast<Word>(program.size()));
  require(result.dispatches == program.size() && machine.core.registers() == expected &&
              machine.x == std::array<Word, memory_words>{} && machine.y == machine.x,
          "General partial/full accumulator writes invalidate reused native values");
  if (input.options.backend == Backend::jit && input.options.optimize) {
    const auto statistics = machine.core.jit_statistics();
    const bool native_move = (input.source < 8 || input.source >= 14) && input.target >= 14;
    require(statistics.native_alu_instructions == (native_move ? 5U : 4U) &&
                statistics.helper_instructions == (native_move ? 0U : 1U) &&
                statistics.native_move_instructions == (native_move ? 1U : 0U),
            "Coherency crosses actual native register transfers or the qualified general helper");
  }
}

class Observer final : public Io {
public:
  Core *core = nullptr;
  Registers expected;
  bool invalidate = false, accept = true, observed = false;
  unsigned reads = 0;

  struct Request {
    Space space;
    Word address, pc;
  };

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    return observe({.space = space, .address = address, .pc = pc}, value);
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
  bool observe(const Request request, Word &value) noexcept {
    ++reads;
    observed = core != nullptr && request.space == Space::x && request.address == io_address &&
               request.pc == start + 2 && core->registers() == expected;
    if (!observed) { return false; }
    if (invalidate) { core->write(Space::program, start + 3, 0); }
    value = datum;
    return accept;
  }
};

struct CallbackInput {
  Options options;
  bool invalidate = false, accept = true, overflow_first = false;
};

void callback(const CallbackInput input) {
  Observer observer;
  Machine machine(input.options, &observer);
  observer.core = &machine.core;
  observer.invalidate = input.invalidate;
  observer.accept = input.accept;
  const std::array program{mac_a, mac_b, io_add_load_a, add_a_b};
  machine.core.load(Space::program, start, program);
  auto initial = register_model::seed(flag_s | mode_up);
  if (input.overflow_first) {
    initial.a = sign56;
    initial.x = 1;
    initial.y = word_mask;
  }
  initial.r[0] = io_address;
  auto expected = initial;
  model_alu(expected, mac_a);
  model_alu(expected, mac_b);
  observer.expected = expected;
  // Memory callbacks see the instruction's sequential PC before the transfer.
  ++observer.expected.pc;
  machine.core.set_registers(initial);
  const auto result = machine.core.run(static_cast<Word>(program.size()));
  require(observer.observed && observer.reads == 1,
          "I/O observes fully materialized accumulator state after preceding native instructions");
  if (!input.accept) {
    require(result.dispatches == 2 && result.progress.instructions == 2 &&
                result.fault.kind == FaultKind::io_read && result.fault.pc == start + 2,
            "A rejected callback cannot retire its native ALU or successor");
    return;
  }
  const auto old = expected;
  register_model::write(expected, 14, datum);
  sequence_model::alu(expected, old, io_add_load_a);
  ++expected.pc;
  model_alu(expected, input.invalidate ? 0 : add_a_b);
  require(result.dispatches == program.size() && result.fault.kind == FaultKind::none &&
              machine.core.registers() == expected,
          "Slow move fallback retains old inputs and invalidates overwritten accumulator caches");
  if (input.options.backend == Backend::jit && input.invalidate) {
    require(machine.core.jit_statistics().invalidations == 1,
            "Callback code invalidation leaves canonical state before entering replacement code");
  }
}

void loop_exit(const Options options, const unsigned written) {
  Machine machine(options);
  constexpr Word do_once = 0x060180;
  constexpr Word loop_flag = Word{1} << 15;
  std::vector<Word> body;
  if ((written & 1U) != 0) { body.push_back(mac_a); }
  if ((written & 2U) != 0) { body.push_back(mac_b); }
  std::vector<Word> program{do_once, 0};
  program.insert(program.end(), body.begin(), body.end());
  program.push_back(0);
  program[1] = start + static_cast<Word>(program.size()) - 1;
  machine.core.load(Space::program, start, program);
  const auto initial = register_model::seed(mode_down | flag_s);
  auto expected = initial;
  expected.stack[3] = (std::uint64_t{initial.la} << word_bits) | initial.lc;
  expected.stack[4] = (std::uint64_t{start + 2} << word_bits) | initial.sr;
  expected.pc = start + 2;
  expected.sr |= loop_flag;
  for (const auto opcode : body) { model_alu(expected, opcode); }
  ++expected.pc;
  expected.sr &= ~loop_flag;
  machine.core.set_registers(initial);
  const auto budget = static_cast<Word>(body.size() + 2);
  const auto result = machine.core.run(budget);
  require(
      result.dispatches == budget && result.fault.kind == FaultKind::none &&
          result.progress == Progress{.instructions = budget, .estimated_cycles = budget + 4} &&
          machine.core.registers() == expected && machine.core.interruptible(),
      "Loop-ending exits materialize any combination of updated accumulators and restore frames");
  if (options.backend == Backend::jit && options.optimize) {
    require(machine.core.jit_statistics().native_alu_instructions == body.size() + 1 &&
                machine.core.jit_statistics().helper_instructions == 1,
            "Native DO setup and body materialization retain authoritative general final cleanup");
  }
}

void fault_after_native(const Options options) {
  Machine machine(options);
  const std::array program{mac_a, mac_b, io_add_load_a};
  machine.core.load(Space::program, start, program);
  auto initial = register_model::seed(mode_up);
  initial.r[0] = memory_words;
  auto preceding = initial;
  model_alu(preceding, mac_a);
  model_alu(preceding, mac_b);
  machine.core.set_registers(initial);
  const auto result = machine.core.run(3);
  require(result.dispatches == 2 && result.progress.instructions == 2 &&
              result.fault.kind == FaultKind::memory_read && result.fault.pc == start + 2 &&
              result.fault.address == memory_words,
          "A native memory guard fault materializes only previously retired instructions");
  require(machine.core.registers().a == 0 && machine.core.registers().b == preceding.b,
          "A fault exit preserves the general helper's partial result without a stale cache flush");
}

void check(const Backend backend, const bool optimize) {
  for (const Word length : {1U, 2U, 3U, 8U, 64U}) {
    prefixes({.backend = backend, .instructions_per_block = length, .optimize = optimize});
  }
  const Options options{.backend = backend, .optimize = optimize};
  for (const unsigned source : {4U, 6U, 8U, 9U, 14U, 15U}) {
    for (unsigned target = 8; target <= 15; ++target) {
      helper_writes({.options = options, .source = source, .target = target});
    }
  }
  callback({.options = options});
  callback({.options = options, .invalidate = true});
  callback({.options = options, .accept = false});
  callback({.options = options, .overflow_first = true});
  callback({.options = options, .invalidate = true, .overflow_first = true});
  callback({.options = options, .accept = false, .overflow_first = true});
  for (unsigned written = 0; written < 4; ++written) { loop_exit(options, written); }
  fault_after_native(options);
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::check(backend, true);
    }
    dsp56300::test::check(dsp56300::Backend::jit, false);
    std::cout << dsp56300::test::checks << " native dataflow boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
