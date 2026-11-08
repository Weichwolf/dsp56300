// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace dsp56300::test {
namespace {
constexpr Word entry = 0x100, datum = 0xe5a391, direction = 0x8000;
constexpr unsigned first_pointer = 16, last_offset = 31;
constexpr Word callback_status = 0x315, initial_callback_status = 0x301;
constexpr Word sum_status = 0x318;

struct Shape {
  unsigned reg = 4, port = 6;
  bool high = true, y = false, write = false;
};

Word encoding(const Shape shape) {
  auto base = isa::Opcode::movep_register_high;
  if (!shape.high) {
    base = shape.y ? isa::Opcode::movep_register_low_extended : isa::Opcode::movep_register_low;
  }
  const auto selector = shape.high ? shape.port : (shape.port & 31U) | ((shape.port & 32U) << 1U);
  return static_cast<Word>(base) | (shape.reg << 8U) | selector |
         (shape.high && shape.y ? 1U << 16U : 0U) | (shape.write ? direction : 0U);
}

Word address(const Shape shape) {
  constexpr Word high_begin = 0xffffc0;
  constexpr Word low_begin = 0xffff80;
  return (shape.high ? high_begin : low_begin) + shape.port;
}

Registers seed(const Word mode = 0) {
  auto state = register_model::seed(mode);
  state.pc = entry;
  return state;
}

struct Access {
  Space space = Space::x;
  Word address = 0, pc = 0, value = 0;
  bool write = false;
};

struct Port final : Io {
  std::array<Word, entry + 8> program{};
  Shape shape;
  Core *core = nullptr;
  Registers observed;
  Progress prefix;
  Word value = 0, request_pc = 0;
  unsigned calls = 0;
  std::uint64_t revision = 0;
  bool refuse = false, yield = false, replace = false, valid = true;

  explicit Port(const Shape input) : shape(input) { program[entry] = encoding(shape); }

  [[nodiscard]] std::optional<Word> inspect_program(const Word pc) const noexcept override {
    return pc < program.size() ? std::optional{program[pc]} : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  bool observe(const Access request) noexcept {
    ++calls;
    request_pc = request.pc;
    if (request.write) { value = request.value; }
    valid = valid && request.space == (shape.y ? Space::y : Space::x) &&
            request.address == address(shape) && request.write == shape.write;
    if (core != nullptr) {
      observed = core->registers();
      prefix = core->progress();
    }
    if (refuse || !valid) { return false; }
    if (replace) {
      program[request.pc + 1] = isa::nop.front();
      ++revision;
      replace = false;
    }
    if (yield && core != nullptr) { core->request_yield(); }
    return true;
  }

  bool read(const Space space, const Word at, const Word pc, Word &word) noexcept override {
    if (space == Space::program) {
      const auto result = inspect_program(at);
      if (!result) { return false; }
      word = *result;
      return true;
    }
    if (!observe({.space = space, .address = at, .pc = pc})) { return false; }
    // The core must expose only the 24-bit bus word, including callback loads.
    constexpr Word upper_bits = 0xa5000000;
    word = datum | upper_bits;
    return true;
  }

  bool write(const Space space, const Word at, const Word pc, const Word word) noexcept override {
    return observe({.space = space, .address = at, .pc = pc, .value = word, .write = true});
  }

  bool bit(Space /*space*/,
           Word /*at*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

Registers expected(Registers state, const Shape shape, const bool accepted, Word &value) {
  if (shape.write) {
    value = register_model::read(state, shape.reg);
  } else if (accepted) {
    register_model::write(state, shape.reg, datum);
  }
  ++state.pc;
  return state;
}

void public_case(const Shape shape, const Word mode, const bool refuse = false) {
  Port port(shape);
  port.refuse = refuse;
  Core core(Memory::external(), &port);
  port.core = &core;
  for (unsigned invocation = 0; invocation < (refuse ? 1U : 2U); ++invocation) {
    const auto initial = seed(mode);
    core.set_registers(initial);
    Word value = 0;
    const auto wanted = expected(initial, shape, !refuse, value);
    auto visible = wanted;
    if (!shape.write) {
      visible = initial;
      ++visible.pc;
    }
    const auto before = core.progress();
    const auto result = core.run(1);
    require(port.valid && port.calls == invocation + 1 && port.request_pc == entry &&
                port.observed == visible && port.prefix == before && core.registers() == wanted &&
                (!shape.write || port.value == value),
            "Native MOVEP preserves complete register mapping and callback-visible prefix/state");
    if (refuse) {
      require(
          result.retired == 0 && result.progress == before &&
              result.fault == Fault{.kind = shape.write ? FaultKind::io_write : FaultKind::io_read,
                                    .space = shape.y ? Space::y : Space::x,
                                    .address = address(shape),
                                    .value = value,
                                    .pc = entry},
          "Refused MOVEP retains the original fault, sticky transfer flags and unretired prefix");
    } else {
      require(result.retired == 1 && result.fault.kind == FaultKind::none && !result.yielded &&
                  result.progress == Progress{.instructions = before.instructions + 1,
                                              .estimated_cycles = before.estimated_cycles + 1},
              "Cold/warm MOVEP retires one real port operation with its one-cycle base cost");
    }
  }
}

std::uint32_t forbidden(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

void bridge(void *owner, detail::NativeAccess &access) noexcept {
  auto &port = *static_cast<Port *>(owner);
  const bool write = access.action == detail::BusAction::write;
  const bool accepted = write ? port.write(access.space, access.address, access.pc, access.value)
                              : port.read(access.space, access.address, access.pc, access.value);
  if (!accepted) {
    access.failure = {.kind = write ? FaultKind::io_write : FaultKind::io_read,
                      .space = access.space,
                      .address = access.address,
                      .value = access.value,
                      .pc = access.pc};
  }
}

void no_instruction_helper(const Shape shape) {
  Port port(shape);
  detail::Cpu cpu;
  cpu.reg = seed();
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  unsigned helpers = 0;
  Fault failure;
  detail::Jit native(cpu,
                     port.program,
                     {.owner = &helpers, .execute = forbidden, .moves = forbidden, .memory = {}},
                     {.owner = &port, .access = bridge, .failure = &failure});
  require(native.run(1) == 1 && helpers == 0 && port.calls == 1 && port.valid &&
              failure.kind == FaultKind::none && cpu.instructions == 1 && cpu.cycles == 1,
          "MOVEP invokes only typed I/O, without a full-instruction or transfer executor");
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  Word value = 0;
  require(cpu.reg == expected(seed(), shape, true, value) && (!shape.write || port.value == value),
          "A missing opcode executor cannot conceal wrong native transfer state");
}

void prefix_exit(const bool refuse) {
  const Shape shape{.reg = first_pointer, .y = true};
  Port port(shape);
  port.program[entry] = isa::opcode<isa::Opcode::clr_a>().front();
  port.program[entry + 1] = encoding(shape);
  port.program[entry + 2] = isa::opcode<isa::Opcode::clr_b>().front();
  port.refuse = refuse;
  port.yield = !refuse;
  Core core(Memory::external(), &port);
  port.core = &core;
  auto state = seed();
  state.sr = initial_callback_status;
  core.set_registers(state);
  const auto result = core.run(3);
  state.a = 0;
  state.pc += 2;
  state.sr = callback_status;
  require(port.calls == 1 && port.observed == state &&
              port.prefix == Progress{.instructions = 1, .estimated_cycles = 1},
          "The native port observes preceding ALU flags, accumulators, progress and sequential PC");
  if (!refuse) { state.r[0] = datum; }
  require(core.registers() == state && result.retired == (refuse ? 1U : 2U) &&
              result.progress.estimated_cycles == (refuse ? 1U : 2U),
          "Faults and yields preserve the entire completed prefix before the next instruction");
  if (refuse) {
    require(result.fault.kind == FaultKind::io_read && core.run(3).retired == 0 && port.calls == 1,
            "A failed native port instruction is never replayed");
  } else {
    require(result.yielded && result.fault.kind == FaultKind::none && core.run(1).retired == 1 &&
                core.registers().b == 0 && port.calls == 1,
            "A real yield commits its port once and resumes at its untouched successor");
  }
}

void native_continuation() {
  const Shape shape;
  Port port(shape);
  port.program[entry] = isa::opcode<isa::Opcode::clr_a>().front();
  port.program[entry + 1] = encoding(shape);
  port.program[entry + 2] = isa::opcode<isa::Opcode::add_x0_a>().front();
  Core core(Memory::external(), &port);
  port.core = &core;
  auto state = seed();
  state.sr = initial_callback_status;
  core.set_registers(state);
  const auto result = core.run(3);
  state.a = 0;
  state.pc += 2;
  state.sr = callback_status;
  require(port.observed == state && port.prefix.instructions == 1,
          "A continuing port observes the preceding native arithmetic state");
  register_model::write(state, shape.reg, datum);
  register_model::write(state, 14, datum); // Zero plus signed X0 has this exact aligned sum.
  state.sr = sum_status;
  ++state.pc;
  require(result.retired == 3 && result.fault.kind == FaultKind::none && !result.yielded &&
              core.registers() == state && port.calls == 1 &&
              result.progress == Progress{.instructions = 3, .estimated_cycles = 3},
          "Native arithmetic immediately consumes the new port value with exact flags and costs");
}

void revision_and_repeat() {
  const Shape shape{.reg = 24, .write = true};
  Port port(shape);
  port.replace = true;
  port.program[entry + 1] = isa::opcode<isa::Opcode::clr_b>().front();
  Core core(Memory::external(), &port);
  port.core = &core;
  core.set_registers(seed());
  const auto result = core.run(2);
  auto state = seed();
  state.pc += 2;
  require(result.retired == 2 && result.fault.kind == FaultKind::none &&
              core.registers() == state && port.calls == 1 && port.revision == 1,
          "A genuine I/O program revision retires stale successors before continuing");
  Port repeated(shape);
  repeated.program[entry] = isa::opcode<isa::Opcode::rep_immediate>().front() | (3U << 8U);
  repeated.program[entry + 1] = encoding(shape);
  Core repeat(Memory::external(), &repeated);
  repeated.core = &repeat;
  repeat.set_registers(seed());
  require(repeat.run(4).retired == 4 && repeated.calls == 3 && repeat.registers().pc == entry + 2 &&
              repeat.registers().lc == seed().lc,
          "The established REP path keeps actual port effects and restored loop state");
}
}
}

int main() {
  try {
    using namespace dsp56300;
    using namespace dsp56300::test;
    for (unsigned reg = 4; reg <= last_offset; ++reg) {
      if (reg > 7 && reg < 14) { continue; }
      for (const bool high : {false, true}) {
        for (const bool y : {false, true}) {
          for (const bool write : {false, true}) {
            for (const unsigned index : {0U, 6U, 63U}) {
              const Shape shape{.reg = reg, .port = index, .high = high, .y = y, .write = write};
              no_instruction_helper(shape);
              for (const Word mode : {0U, mode_down, mode_up}) {
                public_case(shape, mode);
                public_case(shape, mode, true);
              }
            }
          }
        }
      }
    }
    prefix_exit(false);
    prefix_exit(true);
    native_continuation();
    revision_and_repeat();
    std::cout << "Native MOVEP registers, ports, scaling, callback state, faults, yields and "
                 "revisions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
