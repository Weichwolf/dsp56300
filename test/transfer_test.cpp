// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word payload = 0x765432, incoming = 0xab765432;
constexpr Word low_port = 0xffff80, high_port = 0xffffc0;
constexpr Word ram_address = 0x180, negate = 0x200036;
constexpr Word mode_sa = Word{1} << 17;

struct Event {
  Space space = Space::x;
  Word address = 0, pc = start, value = 0;
  bool write = false, accept = true;
};

class Script final : public Io {
public:
  explicit Script(std::vector<Event> expected) : expected_(std::move(expected)) {}

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    return transfer({.space = space, .address = address, .pc = pc}, &value);
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    return transfer({.space = space, .address = address, .pc = pc, .value = value, .write = true},
                    nullptr);
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

  [[nodiscard]] bool complete() const { return valid_ && next_ == expected_.size(); }

private:
  bool transfer(const Event request, Word *const value) noexcept {
    if (next_ == expected_.size()) {
      valid_ = false;
      return false;
    }
    const auto &expected = expected_[next_++];
    valid_ = valid_ && request.space == expected.space && request.address == expected.address &&
             request.pc == expected.pc && request.write == expected.write &&
             (!request.write || request.value == expected.value);
    if (value != nullptr) { *value = expected.value; }
    return valid_ && expected.accept;
  }

  std::vector<Event> expected_;
  std::size_t next_ = 0;
  bool valid_ = true;
};

struct Machine {
  Machine(const Backend backend, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, {.backend = backend}) {
    core.set_registers(initial_state());
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

struct PortCase {
  Word index = 0;
  bool high = false, y = false, write = false;
};

Word port_opcode(const PortCase input) {
  constexpr Word high_base = 0x084400;
  constexpr Word low_x_base = 0x044480;
  constexpr Word low_y_base = 0x044420;
  if (input.high) {
    return high_base | (input.y ? 0x10000U : 0U) | (input.write ? 0x8000U : 0U) | input.index;
  }
  const Word packed = (input.index & 31U) | ((input.index & 32U) << 1);
  return (input.y ? low_y_base : low_x_base) | (input.write ? 0x8000U : 0U) | packed;
}

void register_port(const Backend backend, const PortCase input) {
  const Word location = (input.high ? high_port : low_port) + input.index;
  Script io({{.space = input.y ? Space::y : Space::x,
              .address = location,
              .value = input.write ? payload : incoming,
              .write = input.write}});
  Machine machine(backend, &io);
  machine.core.write(Space::program, start, port_opcode(input));
  auto state = initial_state();
  constexpr std::uint64_t high_word = 0x123456;
  state.x = (high_word << word_bits) | (input.write ? payload : 1U);
  state.sr = status_byte;
  machine.core.set_registers(state);
  const auto result = machine.core.run(1);
  ++state.pc;
  state.x = (high_word << word_bits) | payload;
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
              machine.core.registers() == state && io.complete(),
          "MOVEP maps every X/Y high/low port and preserves full state");
}

void vector_base_register(const Backend backend) {
  // FM5.4.4.4: VBA[7:0] always read as zero. Read back through native MOVEC.
  constexpr Word bank = 0x123400;
  constexpr Word previous_bank = 0x654300;
  constexpr Word byte_values = 256;
  constexpr Word move_vba = 0x05f430;
  constexpr Word vba_to_y0 = 0x0446b0;
  constexpr Word preserved_y1 = 0xabcdef;
  for (Word low = 0; low < byte_values; ++low) {
    Machine machine(backend);
    const std::array program{move_vba, bank + low, vba_to_y0};
    machine.core.load(Space::program, start, program);
    auto expected = initial_state();
    expected.vba = previous_bank;
    expected.y = std::uint64_t{preserved_y1} << word_bits;
    machine.core.set_registers(expected);
    const auto result = machine.core.run(2);
    expected.vba = ((bank + low) / byte_values) * byte_values;
    expected.y |= expected.vba;
    expected.pc += program.size();
    require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
                machine.core.progress().instructions == 2 && machine.core.registers() == expected,
            "VBA ignores every written low byte and native MOVEC reads the aligned bank");
  }
}

void register_ports(const Backend backend) {
  for (Word index = 0; index < 64; ++index) {
    for (const bool high : {false, true}) {
      for (const bool y : {false, true}) {
        for (const bool write : {false, true}) {
          register_port(backend, {.index = index, .high = high, .y = y, .write = write});
        }
      }
    }
  }
}

struct MemoryCase {
  Word opcode = 0, cycles = 2, modes = 0;
  Space peripheral_space = Space::x, memory_space = Space::x;
  bool write = false, immediate = false;
};

void memory_port(const Backend backend, const MemoryCase input) {
  Script io({{.space = input.peripheral_space,
              .address = low_port + 5,
              .value = payload,
              .write = input.write}});
  Machine machine(backend, &io);
  std::vector<Word> program{input.opcode};
  if (input.immediate) { program.push_back(payload); }
  machine.core.load(Space::program, start, program);
  if (input.write && !input.immediate) {
    machine.core.write(input.memory_space, ram_address, payload);
  }
  auto state = initial_state();
  state.r[0] = ram_address;
  state.sr = input.modes;
  machine.core.set_registers(state);
  const auto result = machine.core.run(1);
  state.pc += static_cast<Word>(program.size());
  if (!input.immediate) { ++state.r[0]; }
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none && io.complete() &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = input.cycles} &&
              machine.core.registers() == state,
          "MOVEP memory/immediate directions retain EA updates and base costs");
  if (!input.immediate) {
    require(machine.core.read(input.memory_space, ram_address) == payload,
            "MOVEP moves the expected memory word");
  }
}

void fast_memory_port(const Backend backend, const MemoryCase input, const Word slot) {
  constexpr Word vector = 0x54;
  Script io({{.space = input.peripheral_space,
              .address = low_port + 5,
              .pc = vector + slot,
              .value = payload,
              .write = input.write}});
  Machine machine(backend, &io);
  std::array<Word, 3> program{};
  program[slot] = input.opcode;
  if (input.immediate) { program[slot + 1] = payload; }
  machine.core.load(Space::program, vector, program);
  if (input.write && !input.immediate) {
    machine.core.write(input.memory_space, ram_address, payload);
  }
  auto state = initial_state();
  state.r[0] = ram_address;
  state.sr = status_byte | input.modes;
  machine.core.set_registers(state);
  machine.core.request_interrupt(vector, 0, {.address = start, .opcode = 0});
  const bool rejected = input.immediate && slot != 0;
  const auto count = input.immediate && slot == 0 ? 1U : 2U;
  const auto result = machine.core.run(count);
  if (rejected) {
    require(result.fault == Fault{.kind = FaultKind::unsupported_instruction,
                                  .space = Space::program,
                                  .address = vector + slot,
                                  .value = input.opcode,
                                  .pc = vector + slot} &&
                result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                machine.core.registers() == state && !io.complete(),
            "A two-word MOVEP cannot begin in the second fast slot or perform its I/O");
    return;
  }
  if (!input.immediate) { ++state.r[0]; }
  const auto cycles = input.cycles + (input.immediate ? 0U : 1U);
  require(result.dispatches == count && result.fault.kind == FaultKind::none && io.complete() &&
              result.progress == Progress{.instructions = count, .estimated_cycles = cycles} &&
              machine.core.registers() == state && !machine.core.interruptible(),
          "Fast MOVEP retains foreground PC, stack, flags, real I/O and manual base costs");
  if (!input.immediate) {
    require(machine.core.read(input.memory_space, ram_address) == payload,
            "Fast MOVEP transfers the actual word and retains its native address update");
  }
  ++state.pc;
  const auto resumed = machine.core.run(1);
  require(resumed.dispatches == 1 && resumed.fault.kind == FaultKind::none &&
              machine.core.registers() == state && machine.core.interruptible() && io.complete(),
          "Fast MOVEP resumes the foreground without an RTI or repeated peripheral effect");
}

void staged_fast_immediate(const Backend backend, const Word modes) {
  constexpr Word vector = 0x54;
  constexpr Word opcode = 0x07f405;
  Script io({{.address = low_port + 5, .pc = vector, .value = payload, .write = true}});
  Machine machine(backend, &io);
  auto state = initial_state();
  state.sr = modes;
  machine.core.set_registers(state);
  machine.core.request_interrupt(vector, 0, {.address = start, .opcode = 0});
  const auto first =
      machine.core.begin_instruction({.address = vector, .opcode = opcode, .extension = payload});
  if (!first.pending) { throw std::runtime_error("Fast MOVEP has no staged peripheral write"); }
  require(first.pending->effect.kind == EffectKind::write && first.pending->effect.pc == vector &&
              !io.complete() && machine.core.registers() == state &&
              machine.core.progress() == Progress{},
          "Captured fast MOVEP waits for its actual vector-addressed peripheral effect");
  static_cast<void>(machine.core.advance_instruction(first.pending.value().ticket));
  const auto after_write = machine.core.checkpoint();
  const auto finish = [&] {
    unsigned effects = 0;
    while (const auto pending = machine.core.pending_instruction()) {
      require(++effects <= 16, "Fast MOVEP has a bounded native retirement suffix");
      const auto result = machine.core.advance_instruction(pending->ticket);
      require(result.fault.kind == FaultKind::none, "Fast MOVEP retirement must remain successful");
    }
    require(io.complete() && machine.core.registers() == state &&
                machine.core.progress() == Progress{.instructions = 1, .estimated_cycles = 2} &&
                !machine.core.interruptible(),
            "Fast MOVEP retires one instruction without changing foreground PC or stack");
  };
  finish();
  machine.core.restore(after_write);
  finish();
}

void memory_ports(const Backend backend) {
  constexpr std::array<MemoryCase, 10> cases{
      {{.opcode = 0x075805},
       {.opcode = 0x07d805, .write = true},
       {.opcode = 0x075845, .memory_space = Space::y},
       {.opcode = 0x07d845, .memory_space = Space::y, .write = true},
       {.opcode = 0x071885, .peripheral_space = Space::y},
       {.opcode = 0x079885, .peripheral_space = Space::y, .write = true},
       {.opcode = 0x009805, .cycles = 6, .memory_space = Space::program},
       {.opcode = 0x00d805, .cycles = 6, .memory_space = Space::program, .write = true},
       {.opcode = 0x07f405, .write = true, .immediate = true},
       {.opcode = 0x07b485, .peripheral_space = Space::y, .write = true, .immediate = true}}};
  constexpr std::array modes{Word{0}, mode_sa, mode_sa | mode_down, mode_sa | mode_up};
  for (auto item : cases) {
    for (const auto status : modes) {
      item.modes = status;
      memory_port(backend, item);
      fast_memory_port(backend, item, 0);
      fast_memory_port(backend, item, 1);
    }
  }
}

void self_modify(const Backend backend, const bool peripheral) {
  Script io({{.address = high_port + 5, .value = 0}});
  Machine machine(backend, &io);
  constexpr Word store_register = 0x076084;
  constexpr Word load_peripheral = 0x086045;
  const std::array program{peripheral ? load_peripheral : store_register, negate};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.a = 3;
  state.r[0] = start + 1;
  machine.core.set_registers(state);
  const auto result = machine.core.run(2);
  state.pc = start + 2;
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
              machine.core.registers() == state &&
              machine.core.read(Space::program, start + 1) == 0,
          "Guest MOVEM/MOVEP P writes replace a planned native successor");
  if (peripheral) { require(io.complete(), "Peripheral-to-P transfer consumes exactly one event"); }
  if (backend == Backend::jit) {
    require(machine.core.jit_statistics().invalidations == 1,
            "Guest program changes invalidate executing blocks after return");
  }
}

void mode_change(const Backend backend) {
  Machine machine(backend);
  constexpr Word load_sr = 0x04c4b9;
  constexpr Word round = 0x200011;
  const std::array program{load_sr, round};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = mode_rm;
  state.a = std::uint64_t{1} << (word_bits - 1);
  machine.core.set_registers(state);
  const auto result = machine.core.run(2);
  state.pc = start + 2;
  state.a = std::uint64_t{1} << word_bits;
  state.sr = mode_rm | flag_u;
  require(result.dispatches == 2 && result.fault.kind == FaultKind::none &&
              machine.core.registers() == state,
          "Guest MOVEC SR changes exit a block before mode-specialized arithmetic");
}

void repeat_port(const Backend backend) {
  const Event event{.address = high_port + 5, .pc = start + 1, .value = payload, .write = true};
  Script io({event, event, event});
  Machine machine(backend, &io);
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word write_port = 0x08c405;
  const std::array program{repeat_three, write_port};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = payload;
  state.lc = 7;
  machine.core.set_registers(state);
  const auto result = machine.core.run(4);
  state.pc = start + 2;
  require(result.dispatches == 4 && result.fault.kind == FaultKind::none && io.complete() &&
              result.progress == Progress{.instructions = 4, .estimated_cycles = 8} &&
              machine.core.registers() == state && machine.core.interruptible(),
          "REP repeats a single-word MOVEP, restores LC and admits interrupts afterward");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto backend :
         {dsp56300::Backend::reference, dsp56300::Backend::cached, dsp56300::Backend::jit}) {
      dsp56300::test::vector_base_register(backend);
      dsp56300::test::register_ports(backend);
      dsp56300::test::memory_ports(backend);
      for (const auto modes : {0U, dsp56300::test::mode_sa}) {
        dsp56300::test::staged_fast_immediate(backend, modes);
      }
      dsp56300::test::self_modify(backend, false);
      dsp56300::test::self_modify(backend, true);
      dsp56300::test::mode_change(backend);
      dsp56300::test::repeat_port(backend);
    }
    std::cout << dsp56300::test::checks << " transfer host-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
