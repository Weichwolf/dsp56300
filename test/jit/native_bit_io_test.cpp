// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace dsp56300::test {
namespace {
using isa::Opcode;
constexpr Word entry = 0x100, target = 0x80, selector = 6, address_register = 3;
constexpr Word low_address = 0xffff86, high_address = 0xffffc6, absolute_ea = 0x30;
constexpr Word datum = 0x69a53c, initial_status = 0x300, carry_status = 0x301;
constexpr Word complete_status = 0xc0037f, cleared_status = 0x315;
constexpr std::uint64_t accumulator_a = 0x123456789abc, accumulator_b = 0xabcdef012345;
constexpr unsigned bit_count = 24, warm = 32, run_count = 5000;
enum class Form : std::uint8_t { low, high, absolute, indirect };
enum class Kind : std::uint8_t {
  clear,
  set,
  change,
  test,
  jump_clear,
  jump_set,
  relative_clear,
  relative_set
};

struct Shape {
  Form form = Form::low;
  Kind kind = Kind::test;
  Space space = Space::x;
  Word bit = 0;
};

bool branches(const Shape shape) {
  return shape.kind >= Kind::jump_clear;
}

Word address(const Shape shape) {
  return shape.form == Form::low ? low_address : high_address;
}

unsigned length(const Shape shape) {
  return branches(shape) || shape.form == Form::absolute ? 2U : 1U;
}

unsigned cycles(const Shape shape) {
  if (branches(shape)) { return shape.kind >= Kind::relative_clear ? 5U : 4U; }
  return shape.form == Form::absolute ? 3U : 2U;
}

BitAction action(const Shape shape) {
  constexpr std::array actions{
      BitAction::clear, BitAction::set, BitAction::change, BitAction::test};
  return branches(shape) ? BitAction::test : actions[static_cast<unsigned>(shape.kind)];
}

Word encoding(const Shape shape) {
  constexpr std::array low{Opcode::bclr_port_low,
                           Opcode::bset_port_low,
                           Opcode::bchg_port_low,
                           Opcode::btst_port_low,
                           Opcode::jclr_port_low,
                           Opcode::jset_port_low,
                           Opcode::brclr_port_low,
                           Opcode::brset_port_low};
  constexpr std::array high{Opcode::bclr_port_high,
                            Opcode::bset_port_high,
                            Opcode::bchg_port_high,
                            Opcode::btst_port_high,
                            Opcode::jclr_port_high,
                            Opcode::jset_port_high,
                            Opcode::brclr_port_high,
                            Opcode::brset_port_high};
  constexpr std::array effective{Opcode::bclr_effective,
                                 Opcode::bset_effective,
                                 Opcode::bchg_effective,
                                 Opcode::btst_effective,
                                 Opcode::jclr_effective,
                                 Opcode::jset_effective,
                                 Opcode::brclr_effective,
                                 Opcode::brset_effective};
  const auto index = static_cast<unsigned>(shape.kind);
  auto name = effective[index];
  auto operand = shape.form == Form::absolute ? absolute_ea : 0x20U | address_register;
  if (shape.form == Form::low || shape.form == Form::high) {
    name = (shape.form == Form::low ? low : high)[index];
    operand = selector;
  }
  return static_cast<Word>(name) | (operand << 8U) | (shape.space == Space::y ? 64U : 0U) |
         shape.bit;
}

Word extension(const Shape shape) {
  if (!branches(shape)) { return address(shape); }
  return shape.kind >= Kind::relative_clear ? (target + word_mask + 1U - entry) & word_mask
                                            : target;
}

Registers seed(const Shape shape = {}) {
  Registers state{.pc = entry, .sr = initial_status, .lc = 9, .sp = 5, .sc = 2};
  state.a = accumulator_a;
  state.b = accumulator_b;
  state.m.fill(word_mask);
  state.r[address_register] = address(shape);
  return state;
}

Registers expected(Registers state, const Shape shape, const bool previous) {
  state.pc += length(shape);
  if (branches(shape)) {
    const bool wanted = shape.kind == Kind::jump_set || shape.kind == Kind::relative_set;
    if (wanted == previous) { state.pc = target; }
  } else {
    state.sr = ((state.sr / 2) * 2) + static_cast<unsigned>(previous);
  }
  return state;
}

Word expected_word(const Word before, const Shape shape, const bool previous) {
  if (shape.kind == Kind::clear || shape.kind == Kind::change) {
    if (previous) { return before - (Word{1} << shape.bit); }
  }
  if (shape.kind == Kind::set || shape.kind == Kind::change) {
    if (!previous) { return before + (Word{1} << shape.bit); }
  }
  return before;
}

struct Request {
  Space space = Space::x;
  Word address = 0, pc = 0, bit = 0;
  BitAction action = BitAction::test;
  bool operator==(const Request &) const = default;
};

struct Port final : Io {
  Core *core = nullptr;
  std::array<Word, entry + 8> program{};
  std::array<Word, 1> x{}, y{};
  Registers observed;
  Progress prefix;
  Request request;
  Word word = 0;
  unsigned calls = 0;
  std::uint64_t revision = 0;
  bool refuse = false, yield = false, replace = false;

  [[nodiscard]] std::optional<Word> inspect_program(const Word pc) const noexcept override {
    return pc < program.size() ? std::optional{program[pc]} : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  bool read(const Space space, const Word at, Word /*pc*/, Word &value) noexcept override {
    if (space != Space::program || at >= program.size()) { return false; }
    value = program[at];
    return true;
  }

  bool write(Space /*space*/, Word /*at*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(const Space space,
           const Word at,
           const Word pc,
           const Word bit,
           const BitAction operation,
           bool &previous) noexcept override {
    ++calls;
    request = {.space = space, .address = at, .pc = pc, .bit = bit, .action = operation};
    if (core != nullptr) {
      observed = core->registers();
      prefix = core->progress();
    }
    if (refuse || bit >= bit_count) { return false; }
    const auto unit = Word{1} << bit;
    previous = (word & unit) != 0;
    switch (operation) {
      case BitAction::test: break;
      case BitAction::clear: word &= ~unit; break;
      case BitAction::set: word |= unit; break;
      case BitAction::change: word ^= unit; break;
      default: return false;
    }
    if (replace) {
      program[entry + 1] = isa::nop.front();
      ++revision;
      replace = false;
    }
    if (yield && core != nullptr) { core->request_yield(); }
    return true;
  }

  void install(const Shape shape) {
    program[entry] = encoding(shape);
    if (length(shape) == 2) { program[entry + 1] = extension(shape); }
  }
};

void one(const Shape shape, const Word status, const bool previous) {
  Port port;
  port.install(shape);
  const auto unit = Word{1} << shape.bit;
  port.word = (datum & ~unit) | (previous ? unit : 0U);
  const auto word = expected_word(port.word, shape, previous);
  Core core(Memory::separate({.program = port.program, .x = port.x, .y = port.y}), &port);
  port.core = &core;
  auto initial = seed(shape);
  initial.sr = status;
  core.set_registers(initial);
  const auto result = core.run(1);
  auto observed = initial;
  observed.pc += length(shape);
  const Request request{.space = shape.space,
                        .address = address(shape),
                        .pc = entry,
                        .bit = shape.bit,
                        .action = action(shape)};
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = cycles(shape)} &&
              core.registers() == expected(initial, shape, previous) && port.word == word &&
              port.calls == 1 && port.request == request && port.observed == observed &&
              port.prefix == Progress{},
          "Native peripheral bits preserve literal state/cycles, real callback metadata, prefix "
          "and single consumption");
}

void shape_cases(Shape shape) {
  for (unsigned bit = 0; bit < bit_count; ++bit) {
    shape.bit = bit;
    for (const auto status : {initial_status, carry_status, complete_status}) {
      for (const bool previous : {false, true}) { one(shape, status, previous); }
    }
  }
}

void operations() {
  for (const auto form : {Form::low, Form::high, Form::absolute, Form::indirect}) {
    for (const auto space : {Space::x, Space::y}) {
      for (unsigned kind = 0; kind < 8; ++kind) {
        const Shape shape{.form = form, .kind = static_cast<Kind>(kind), .space = space};
        if (branches(shape) && form == Form::absolute) { continue; }
        shape_cases(shape);
      }
    }
  }
}

std::uint32_t forbidden(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

void bridge(void *owner, detail::NativeAccess &access) noexcept {
  detail::access_bit_io(static_cast<Port *>(owner), access);
}

void no_instruction_helper() {
  for (unsigned kind = 0; kind < 8; ++kind) {
    const Shape shape{.form = Form::high, .kind = static_cast<Kind>(kind), .bit = bit_count - 1};
    Port port;
    port.install(shape);
    detail::Cpu cpu;
    cpu.reg = seed(shape);
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    unsigned helpers = 0;
    Fault failure;
    detail::Jit native(cpu,
                       port.program,
                       {.owner = &helpers, .execute = forbidden, .memory = {}},
                       {.owner = &port, .access = bridge, .failure = &failure});
    require(
        native.run(1) == 1 && helpers == 0 && port.calls == 1 && failure.kind == FaultKind::none,
        "Peripheral bits invoke only the declared I/O access, never the complete opcode helper");
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(cpu.reg == expected(seed(shape), shape, false) && cpu.cycles == cycles(shape),
            "The direct I/O path has independently expected state even without an opcode executor");
    cpu.reg = seed(shape);
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    port.refuse = true;
    const auto previous_cycles = cpu.cycles;
    require(native.run(1) == 0 && helpers == 0 && port.calls == 2 &&
                cpu.cycles == previous_cycles &&
                failure == Fault{.kind = FaultKind::io_bit,
                                 .space = Space::x,
                                 .address = high_address,
                                 .value = bit_count - 1,
                                 .pc = entry} &&
                cpu.reg.pc == entry + length(shape),
            "The typed access failure reaches its owner exactly without opcode execution or "
            "retirement");
  }
}

void prefix_exits(const bool refuse) {
  const Shape shape{.kind = Kind::set, .space = Space::y, .bit = bit_count - 1};
  Port port;
  port.install(shape);
  port.program[entry] = isa::opcode<Opcode::clr_a>().front();
  port.program[entry + 1] = encoding(shape);
  port.program[entry + 2] = isa::opcode<Opcode::clr_b>().front();
  port.refuse = refuse;
  port.yield = !refuse;
  Core core(Memory::separate({.program = port.program, .x = port.x, .y = port.y}), &port);
  port.core = &core;
  auto initial = seed(shape);
  initial.sr = carry_status;
  core.set_registers(initial);
  const auto result = core.run(3);
  auto truth = initial;
  truth.a = 0;
  truth.sr = refuse ? cleared_status : cleared_status - 1;
  truth.pc += 2;
  require(port.calls == 1 && core.registers() == truth && port.observed.a == 0 &&
              port.observed.sr == cleared_status &&
              port.prefix == Progress{.instructions = 1, .estimated_cycles = 1},
          "A real callback observes the complete preceding ALU state before its bit effect");
  if (refuse) {
    require(
        result.retired == 1 &&
            result.fault == Fault{.kind = FaultKind::io_bit,
                                  .space = Space::y,
                                  .address = low_address,
                                  .value = bit_count - 1,
                                  .pc = entry + 1} &&
            core.run(3).retired == 0 && port.calls == 1,
        "A failed bit access preserves its exact fault and prefix without a replay or successor");
  } else {
    require(result.retired == 2 && result.yielded && result.progress.estimated_cycles == 3 &&
                result.fault.kind == FaultKind::none && core.run(1).retired == 1 &&
                port.calls == 1 && core.registers().b == 0,
            "A successful requested yield completes its real bit once before foreground resumes");
  }
}

void branch_exits() {
  const Shape shape{.form = Form::high, .kind = Kind::jump_clear, .space = Space::y};
  for (const bool refuse : {false, true}) {
    Port port;
    port.install(shape);
    port.program[target] = isa::opcode<Opcode::clr_b>().front();
    port.refuse = refuse;
    port.yield = true;
    Core core(Memory::separate({.program = port.program, .x = port.x, .y = port.y}), &port);
    port.core = &core;
    core.set_registers(seed(shape));
    const auto result = core.run(3);
    auto truth = seed(shape);
    truth.pc = refuse ? entry + 2 : target;
    require(core.registers() == truth && port.calls == 1 && port.prefix == Progress{},
            "A branch callback retains the actual pre-effect prefix and cannot execute its target "
            "early");
    if (refuse) {
      require(result.retired == 0 && result.progress == Progress{} &&
                  result.fault == Fault{.kind = FaultKind::io_bit,
                                        .space = Space::y,
                                        .address = high_address,
                                        .pc = entry},
              "A refused branch never commits its target, carry or base cycles");
    } else {
      require(result.retired == 1 && result.yielded && result.progress.estimated_cycles == 4 &&
                  result.fault.kind == FaultKind::none && core.run(1).retired == 1 &&
                  core.registers().b == 0 && port.calls == 1,
              "A requested branch yield commits its taken target once before native linking can "
              "resume");
    }
  }
}

void revision_exit() {
  const Shape shape{.kind = Kind::set};
  Port port;
  port.install(shape);
  port.program[entry + 1] = isa::opcode<Opcode::clr_b>().front();
  port.replace = true;
  Core core(Memory::external(), &port);
  port.core = &core;
  core.set_registers(seed(shape));
  const auto result = core.run(2);
  auto truth = seed(shape);
  truth.pc += 2;
  require(result.retired == 2 && result.fault.kind == FaultKind::none &&
              core.registers() == truth && port.calls == 1 && port.word == 1 &&
              port.revision == 1 && result.progress.estimated_cycles == 3,
          "An actual I/O code-revision change leaves stale successors before executing the "
          "replacement NOP");
}

void repeat_and_deadlines() {
  Port repeated;
  const Shape toggle{.kind = Kind::change};
  repeated.program[entry] = isa::opcode<Opcode::rep_immediate>().front() | (3U << 8U);
  repeated.program[entry + 1] = encoding(toggle);
  Core repeat(Memory::separate({.program = repeated.program, .x = repeated.x, .y = repeated.y}),
              &repeated);
  repeated.core = &repeat;
  repeat.set_registers(seed(toggle));
  require(repeat.run(4).retired == 4 && repeated.calls == 3 && repeated.word == 1 &&
              repeat.registers().pc == entry + 2 && repeat.registers().lc == 9 &&
              repeat.registers().sr == initial_status && repeat.progress().estimated_cycles == 11,
          "Peripheral REP retains its actual callback count, carry and restored LC on the "
          "established path");
  Port port;
  const Shape branch{.form = Form::high, .kind = Kind::jump_clear};
  port.install(branch);
  port.program[entry + 1] = entry;
  Core core(Memory::separate({.program = port.program, .x = port.x, .y = port.y}), &port);
  port.core = &core;
  core.set_registers(seed(branch));
  constexpr unsigned last_deadline = 40;
  for (unsigned deadline = 1; deadline <= last_deadline; ++deadline) {
    const auto result = core.run_until(deadline);
    const auto calls = (deadline + 3) / 4;
    require(result.fault.kind == FaultKind::none && core.registers() == seed(branch) &&
                port.calls == calls &&
                result.progress ==
                    Progress{.instructions = calls, .estimated_cycles = 4ULL * calls},
            "A real port-poll loop stops at its first whole instruction at every deadline");
  }
  require(core.run(warm).retired == warm, "Warm the retained native peripheral loop");
  const auto images = codegen::publications();
  const auto calls = port.calls;
  const auto progress = core.progress();
  require(core.run(run_count).retired == run_count && port.calls == calls + run_count &&
              core.progress().estimated_cycles == progress.estimated_cycles + 4ULL * run_count &&
              images == codegen::publications(),
          "Warm I/O execution preserves real effects without retranslating retained code");
}
}
}

int main() {
  try {
    dsp56300::test::operations();
    dsp56300::test::no_instruction_helper();
    dsp56300::test::prefix_exits(false);
    dsp56300::test::prefix_exits(true);
    dsp56300::test::branch_exits();
    dsp56300::test::revision_exit();
    dsp56300::test::repeat_and_deadlines();
    std::cout << "Native peripheral bits, callback prefixes, faults, yields and revisions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
