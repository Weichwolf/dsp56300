// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_jit_state.hpp"
#include "a64_loop_state.hpp"
#include "a64_native.hpp"
#include "a64_vector_bus.hpp"
#include "address_model.hpp"
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "loop_control_model.hpp"
#include "register_model.hpp"
#include "return_model.hpp"
#include "selection_model.hpp"
#include "support.hpp"
#include "vector_model.hpp"
#include "vector_state.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using State = vector_model::State;
using Phase = vector_model::Phase;
constexpr std::array phases{Phase::pending, Phase::first, Phase::second};
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::array pointers{0U, 15U, 31U, 63U};
constexpr Word sm = Word{1} << 20;

void expect(const a64::Program &program, const State &initial, const State &active) {
  for (const Word budget : {0U, 1U, 8U}) {
    const auto &expected = budget == 0 ? initial : active;
    loop_model::Runner runner(initial);
    const auto retired = budget != 0 && expected.fault.kind == FaultKind::none ? 1U : 0U;
    require(runner.run(program, budget) == retired,
            "A vector entry retires exactly one complete slot or its precise fault prefix");
    runner.expect(expected);
  }
}

void scalar(const Word mode) {
  for (Word code = 0; code <= status_byte; ++code) {
    const a64::Instruction input{
        .opcode = vector_model::parallel | code, .pc = vector_model::base, .mode = mode};
    const a64::Program program(input, a64::Emission::vector);
    for (const auto phase : phases) {
      auto initial = vector_model::seed(mode | status_byte, phase);
      initial.vector.words = {input.opcode, input.opcode};
      auto expected = initial;
      vector_model::alu(expected, input);
      expect(program, initial, expected);
    }
  }
}

void nop() {
  const a64::Instruction input{.pc = vector_model::base};
  const a64::Program program(input, a64::Emission::vector);
  for (const auto phase : phases) {
    for (unsigned bit = 0; bit < word_bits; ++bit) {
      for (const bool inhibited : {false, true}) {
        auto initial = vector_model::seed((Word{1} << bit) | vector_model::parallel, phase);
        initial.inhibited = inhibited;
        initial.reg.sc = 0;
        initial.reg.pc = vector_model::base + 1;
        initial.vector.resume = initial.reg.pc;
        auto expected = initial;
        vector_model::alu(expected, input);
        expect(program, initial, expected);
      }
    }
  }
}

void short_calls() {
  const vector_model::Call form;
  const auto input = vector_model::instruction(form, 0);
  const a64::Program program(input, a64::Emission::vector);
  for (const auto phase : phases) {
    for (unsigned level = 0; level < 4; ++level) {
      for (const Word pointer : pointers) {
        for (const Word stack : {0U, 14U, 15U, 16U}) {
          auto initial = vector_model::seed(word_mask, phase);
          initial.reg.sp = pointer;
          initial.reg.sc = stack;
          initial.vector.level = level;
          auto expected = initial;
          vector_model::call(expected, form, input);
          expect(program, initial, expected);
        }
      }
    }
  }
}

void frame_bits() {
  const vector_model::Call form;
  const auto input = vector_model::instruction(form, 0);
  const a64::Program program(input, a64::Emission::vector);
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    for (const Word value : {Word{1} << bit, word_mask ^ (Word{1} << bit)}) {
      for (const auto phase : phases) {
        auto initial = vector_model::seed(value, phase);
        initial.vector.resume = value;
        auto expected = initial;
        vector_model::call(expected, form, input);
        expect(program, initial, expected);
      }
    }
  }
}

void effective_calls() {
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned index = 0; index < agu_registers; ++index) {
      const vector_model::Call form{.mode = mode, .index = index, .effective = true};
      const auto input = vector_model::instruction(form, 0);
      const a64::Program program(input, a64::Emission::vector);
      for (const auto phase : phases) {
        for (const Word modifier : {0U, 3U, 0x8003U, 0x8000U, 0xc001U, address_model::linear}) {
          for (const Word stack : {0U, 15U}) {
            auto initial = vector_model::seed(word_mask, phase);
            initial.reg.sc = stack;
            initial.reg.m.at(index) = modifier;
            auto expected = initial;
            vector_model::call(expected, form, input);
            expect(program, initial, expected);
          }
        }
      }
    }
  }
}

void prohibited() {
  constexpr Word jump = 0x0c0000;
  constexpr Word trap = 6;
  constexpr Word enddo = 0x8c;
  constexpr Word repeat = 0x0603a0;
  constexpr Word return_interrupt = 4;
  constexpr Word immediate = 0x0140c0;
  const std::array inputs{
      a64::Instruction{.opcode = jump, .pc = vector_model::base, .cycles = 3},
      a64::Instruction{.opcode = trap, .pc = vector_model::base, .cycles = 9},
      a64::Instruction{.opcode = enddo, .pc = vector_model::base},
      a64::Instruction{.opcode = repeat, .pc = vector_model::base, .cycles = 5},
      a64::Instruction{.opcode = return_interrupt, .pc = vector_model::base, .cycles = 3},
      a64::Instruction{.opcode = immediate, .pc = vector_model::base, .words = 2, .cycles = 2}};
  for (const auto input : inputs) {
    const a64::Program program(input, a64::Emission::vector);
    for (const auto phase : phases) {
      const auto initial = vector_model::seed(word_mask, phase);
      auto expected = initial;
      require(!vector_model::begin(expected, input, false), "Authored vector restriction faults");
      expect(program, initial, expected);
    }
  }
}

void two_word_move() {
  constexpr Word immediate_x0 = 0x44f400;
  const a64::Instruction input{.opcode = immediate_x0,
                               .extension = register_model::datum,
                               .pc = vector_model::base,
                               .words = 2,
                               .cycles = 2};
  const a64::Program program(input, a64::Emission::vector);
  for (const auto phase : phases) {
    const auto initial = vector_model::seed(status_byte, phase);
    auto expected = initial;
    if (vector_model::begin(expected, input)) {
      register_model::write(expected.reg, 4, input.extension);
      vector_model::finish(expected, input);
    }
    expect(program, initial, expected);
  }
}

bool transfer_permitted(const unsigned reg, const bool load) {
  if (!load) { return reg != register_model::reg_ssh; }
  return reg != register_model::reg_sc && reg != register_model::reg_sp &&
         reg != register_model::reg_ssh && reg != register_model::reg_ssl &&
         reg != register_model::reg_la && reg != register_model::reg_lc;
}

struct Transfer {
  unsigned reg = 4;
  bool load = true, effective = false;
  Word mode = 0;
};

void program_move(const Transfer form, const Phase phase) {
  constexpr Word absolute = 0x070000;
  constexpr Word effective = 0x074080;
  constexpr Word direction = 0x8000;
  constexpr Word short_address = 31;
  constexpr Word effective_mode = 3;
  constexpr Word datum = 0xab923456;
  const auto address = form.effective ? vector_model::base : short_address;
  const a64::Instruction input{.opcode = (form.effective ? effective | (effective_mode << 11)
                                                         : absolute | (short_address << 8)) |
                                         (form.load ? direction : 0U) | form.reg,
                               .pc = vector_model::base,
                               .mode = form.mode,
                               .cycles = 6};
  const a64::Program program(input, a64::Emission::vector);
  for (const Word budget : {0U, 1U, 8U}) {
    std::array<Word, memory_words> memory{};
    memory[address] = datum;
    auto wanted_memory = memory;
    auto initial = vector_model::seed(form.mode | status_byte, phase);
    initial.reg.r[0] = address;
    auto expected = initial;
    if (budget != 0 &&
        vector_model::begin(expected, input, transfer_permitted(form.reg, form.load)) &&
        loop_model::numeric(expected, {.pc = input.pc})) {
      if (!register_model::valid(form.reg)) {
        loop_model::fail(expected, {.pc = input.pc}, FaultKind::invalid_instruction, input.opcode);
      } else {
        if (form.effective) { ++expected.reg.r[0]; }
        if (form.load) {
          register_model::write(expected.reg, form.reg, datum & word_mask);
        } else {
          wanted_memory[address] = register_model::read(expected.reg, form.reg);
          expected.program_written = true;
        }
        vector_model::finish(expected, input);
      }
    }
    loop_model::Runner runner(initial);
    runner.bind({}, {.program = {.data = memory.data(), .words = memory.size()}});
    const auto retired = budget != 0 && expected.fault.kind == FaultKind::none ? 1U : 0U;
    require(runner.run(program, budget) == retired,
            "Vector MOVEM whitelist precedes numeric checks, addressing and register effects");
    runner.expect(expected);
    require(memory == wanted_memory, "Vector MOVEM changes only its exact authored program word");
  }
}

void program_registers() {
  for (unsigned reg = 0; reg < 64; ++reg) {
    for (const bool load : {false, true}) {
      for (const bool effective : {false, true}) {
        for (const Word mode : {0U, loop_model::cm, sm}) {
          for (const auto phase : phases) {
            program_move({.reg = reg, .load = load, .effective = effective, .mode = mode}, phase);
          }
        }
      }
    }
  }
}

bool peripheral_model(State &state,
                      a64::VectorAccess &event,
                      const a64::Instruction input,
                      const unsigned reg) {
  if (!vector_model::begin(state, input, transfer_permitted(reg, !event.write))) { return false; }
  if (!register_model::valid(reg)) {
    loop_model::fail(state, {.pc = input.pc}, FaultKind::invalid_instruction, input.opcode);
    return false;
  }
  if (event.write) { event.value = register_model::read(state.reg, reg); }
  event.visible = state;
  if (event.accept) {
    if (!event.write) { register_model::write(state.reg, reg, event.value & word_mask); }
    vector_model::finish(state, input);
  } else {
    state.fault = {.kind = event.write ? FaultKind::io_write : FaultKind::io_read,
                   .space = Space::x,
                   .address = event.address,
                   .value = event.write ? event.value : 0U,
                   .pc = input.pc};
    state.boundary = detail::NativeBoundary::owner;
  }
  return true;
}

void peripheral_move(const unsigned reg, const bool load, const Phase phase, const bool accept) {
  constexpr Word peripheral = 0x084000;
  constexpr Word direction = 0x8000;
  constexpr Word port = 0xffffc0;
  constexpr Word datum = 0xab923456;
  const a64::Instruction input{.opcode = peripheral | (reg << 8) | (load ? 0U : direction),
                               .pc = vector_model::base};
  const a64::Program program(input, a64::Emission::vector);
  for (const Word budget : {0U, 1U, 8U}) {
    const auto initial = vector_model::seed(status_byte, phase);
    auto expected = initial;
    a64::VectorAccess event{.visible = initial,
                            .address = port,
                            .pc = input.pc,
                            .value = datum,
                            .write = !load,
                            .accept = accept};
    const bool accessed = budget != 0 && peripheral_model(expected, event, input, reg);
    loop_model::Runner runner(initial);
    a64::VectorBus bus(runner, event);
    runner.bind({}, bus.binding());
    const auto retired = budget != 0 && expected.fault.kind == FaultKind::none ? 1U : 0U;
    require(runner.run(program, budget) == retired,
            "Vector MOVEP preserves precise callback rejection before slot completion");
    runner.expect(expected);
    bus.expect(accessed ? 1U : 0U);
  }
}

void peripheral_registers() {
  for (unsigned reg = 0; reg < 64; ++reg) {
    for (const bool load : {false, true}) {
      for (const auto phase : phases) {
        for (const bool accept : {false, true}) { peripheral_move(reg, load, phase, accept); }
      }
    }
  }
}

enum class ResetOwner : std::uint8_t { missing, reject, accept };

void reset_probe(const a64::Program &program,
                 const a64::Instruction input,
                 const State &initial,
                 const ResetOwner owner,
                 const Word budget) {
  auto visible = initial;
  require(vector_model::begin(visible, input), "RESET is permitted in either vector slot");
  auto expected = budget == 0 ? initial : visible;
  if (budget != 0) {
    if (owner == ResetOwner::accept) {
      vector_model::finish(expected, input);
    } else {
      loop_model::fail(expected, {.pc = input.pc}, FaultKind::host_event, input.opcode);
    }
  }
  loop_model::Runner runner(initial);
  a64::VectorReset io(runner,
                      {.visible = visible, .pc = input.pc, .accept = owner == ResetOwner::accept});
  if (owner != ResetOwner::missing) { runner.peripherals(&io); }
  require(runner.run(program, budget) == (budget != 0 && owner == ResetOwner::accept ? 1U : 0U),
          "Vector RESET rejects before slot completion and bypasses ordinary loop cleanup");
  runner.expect(expected);
  io.expect(budget != 0 && owner != ResetOwner::missing ? 1U : 0U);
}

void reset_vectors() {
  constexpr Word reset = 0x84;
  const a64::Instruction input{.opcode = reset, .pc = vector_model::base, .cycles = 7};
  const a64::Program program(input, a64::Emission::vector);
  for (const auto phase : phases) {
    auto initial = vector_model::seed(word_mask, phase);
    initial.inhibited = false;
    initial.reg.sc = 0;
    for (const auto owner : {ResetOwner::missing, ResetOwner::reject, ResetOwner::accept}) {
      for (const Word budget : {0U, 1U, 8U}) {
        reset_probe(program, input, initial, owner, budget);
      }
    }
  }
}

void request_debug(State &state, const unsigned form) {
  if (form != 0 &&
      !selection_model::test_condition({.status = state.reg.sr, .selector = form - 1})) {
    return;
  }
  constexpr Word interrupt = 2;
  constexpr Word software = 4;
  constexpr Word pending = 8;
  if ((state.debug_flags & interrupt) == 0) {
    state.debug_flags = static_cast<std::uint8_t>(state.debug_flags | software);
    state.processing = ExecutionState::debugging;
    return;
  }
  if ((state.debug_flags & pending) == 0) {
    state.debug_vector = (state.reg.vba / 256) * 256 + 6;
    state.debug_flags = static_cast<std::uint8_t>(state.debug_flags | pending);
  }
  state.processing = ExecutionState::running;
}

void debug_vectors() {
  constexpr Word debug = 0x200;
  constexpr Word conditional = 0x300;
  constexpr unsigned forms = 17;
  for (unsigned form = 0; form < forms; ++form) {
    const a64::Instruction input{.opcode = form == 0 ? debug : conditional + form - 1,
                                 .pc = vector_model::base,
                                 .cycles = form == 0 ? 1U : 5U};
    const a64::Program program(input, a64::Emission::vector);
    for (Word flags = 0; flags <= status_byte; ++flags) {
      for (const auto phase : phases) {
        auto initial = vector_model::seed(flags | loop_model::lf | loop_model::cm, phase);
        initial.debug_flags = static_cast<std::uint8_t>(flags);
        initial.inhibited = false;
        auto expected = initial;
        require(vector_model::begin(expected, input), "DEBUG is permitted in either vector slot");
        request_debug(expected, form);
        vector_model::finish(expected, input);
        expect(program, initial, expected);
      }
    }
  }
}

void vector_return() {
  constexpr Word rti = 4;
  const vector_model::Call form;
  const auto input = vector_model::instruction(form, 0);
  for (unsigned level = 0; level < 4; ++level) {
    const auto initial = vector_model::seed(word_mask, Phase::pending);
    auto expected = initial;
    expected.vector.level = level;
    a64::Machine machine(expected);
    vector_model::call(expected, form, input);
    require(machine.jit().run_vector(
                {.instruction = detail::decode(input.opcode), .pc = input.pc}) == 1,
            "A native vector call owns a genuine resume/SR stack frame");
    machine.expect(expected);
    const auto return_pc = expected.reg.pc;
    return_model::apply(expected.reg);
    expected.inhibited = true;
    expected.completed = false;
    loop_model::retire(expected, 3);
    require(machine.jit().run_captured({.instruction = detail::decode(rti), .pc = return_pc}) == 1,
            "Native RTI returns from the actual vector frame with restored status and inhibition");
    machine.expect(expected);
  }
}

void entry_roles() {
  constexpr Word add = 0x200040;
  auto initial = vector_model::seed(status_byte, Phase::first);
  initial.reg.pc = vector_model::base;
  a64::Machine machine(initial);
  const auto before = codegen::publications();
  const auto decoded = detail::decode(add);
  const a64::Instruction input{.opcode = add, .pc = vector_model::base};
  for (unsigned pass = 0; pass < 2; ++pass) {
    machine.initialize(initial);
    auto expected = initial;
    a64_alu_model::apply(expected.reg, add & status_byte);
    ++expected.reg.pc;
    expected.completed = expected.inhibited = false;
    loop_model::retire(expected, 1);
    require(machine.jit().run_captured({.instruction = decoded, .pc = input.pc}) == 1,
            "An ordinary captured entry advances its own PC");
    machine.expect(expected);
    machine.initialize(initial);
    expected = initial;
    vector_model::alu(expected, input);
    require(machine.jit().run_vector({.instruction = decoded, .pc = input.pc}) == 1,
            "An identical instruction in a vector entry preserves the foreground PC");
    machine.expect(expected);
    require(codegen::publications().count == before.count + 2,
            "Ordinary and vector roles retain distinct immutable translations");
  }
}

void factory() {
  constexpr Word add = 0x200040;
  constexpr Word clear = 0x200013;
  const auto initial = vector_model::seed(status_byte, Phase::first);
  a64::Machine machine(initial);
  const auto before = codegen::publications();
  for (const Word opcode : {add, clear, add, clear}) {
    for (const auto phase : phases) {
      auto state = vector_model::seed(status_byte, phase);
      state.vector.resume ^= opcode;
      state.reg.pc ^= opcode;
      const a64::Instruction input{.opcode = opcode, .pc = vector_model::base};
      auto expected = state;
      vector_model::alu(expected, input);
      machine.initialize(state);
      const auto decoded = detail::decode(opcode);
      require(machine.jit().run_vector({.instruction = decoded, .pc = input.pc}) == 1,
              "The real Jit vector factory executes pending, first and second slots");
      machine.expect(expected);
    }
  }
  require(codegen::publications().count == before.count + 2,
          "Vector slots retain genuine source variants independently of foreground/resume PCs");
}

void pair() {
  const std::array inputs{a64::Instruction{.opcode = 0x200040, .pc = vector_model::base},
                          a64::Instruction{.opcode = 0x200013, .pc = vector_model::base + 1}};
  auto expected = vector_model::seed(status_byte, Phase::pending);
  expected.vector.words = {inputs[0].opcode, inputs[1].opcode};
  a64::Machine machine(expected);
  for (const auto input : inputs) {
    vector_model::alu(expected, input);
    require(machine.jit().run_vector(
                {.instruction = detail::decode(input.opcode), .pc = input.pc}) == 1,
            "Two genuine native vector slots share the captured latch and resume normally");
    machine.expect(expected);
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (const auto scale : scales) {
      for (const auto saturation : {0U, sm}) { scalar(scale | saturation); }
    }
    nop();
    short_calls();
    frame_bits();
    effective_calls();
    prohibited();
    two_word_move();
    program_registers();
    peripheral_registers();
    reset_vectors();
    debug_vectors();
    vector_return();
    entry_roles();
    factory();
    pair();
    std::cout << checks << " independent native A64 vector checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
