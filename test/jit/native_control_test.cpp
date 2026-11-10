// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "loop_frames.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using isa::Opcode;
constexpr Word entry = 0x180;
constexpr Word destination = 0x100;
constexpr unsigned always = 16;
constexpr unsigned status_count = 128;
constexpr unsigned pointer_range = 64;
constexpr Word modulus = word_mask + 1;
constexpr unsigned offset_register = 3;
constexpr std::uint64_t accumulator_a_pattern = 0x123456789a;
constexpr std::uint64_t accumulator_b_pattern = 0x9876543210;
constexpr std::uint64_t x_pattern = 0x123456789abc;
constexpr std::uint64_t y_pattern = 0xfedcba987654;
constexpr std::uint64_t frame_pattern = 0xabcdeffedcba;
constexpr Word initial_status = 0x300;
constexpr Word return_status = 0xc0037f;
constexpr Word saved_status = 0x654321;
constexpr Word wrapped_target = 0xffff80;
enum class Form : std::uint8_t {
  short_address,
  absolute,
  relative_short,
  relative_long,
  relative_register,
  register_indirect
};
constexpr std::array forms{Form::short_address,
                           Form::absolute,
                           Form::relative_short,
                           Form::relative_long,
                           Form::relative_register,
                           Form::register_indirect};

struct Encoding {
  Word opcode = 0, extra = 0, origin = entry, target = destination;
  unsigned words = 1, cycles = 3, selector = always;
  unsigned target_register = offset_register;
  bool call = false, register_target = false;
};

Word opcode(const Opcode name) {
  return static_cast<Word>(name);
}

Encoding extended_relative(Encoding result, const Form form) {
  const bool reg = form == Form::relative_register;
  const bool conditional = result.selector != always;
  result.opcode |= (conditional ? result.selector : 0U) | (reg ? offset_register << 8U : 0U);
  result.extra = (result.target + modulus - result.origin) % modulus;
  result.words = reg ? 1U : 2U;
  result.cycles = !reg && (conditional != result.call) ? 5U : 4U;
  return result;
}

Encoding make(const Form form,
              const bool call,
              const unsigned selector = always,
              const Word origin = entry,
              const Word target = destination) {
  const bool conditional = selector != always;
  const auto displacement = (target + modulus - origin) % modulus;
  Encoding result{.origin = origin, .target = target, .selector = selector, .call = call};
  constexpr std::array<std::array<Opcode, 4>, 6> names{{{Opcode::jmp_absolute_short,
                                                         Opcode::jsr_absolute_short,
                                                         Opcode::jcc_absolute_short,
                                                         Opcode::jscc_absolute_short},
                                                        {Opcode::jmp_effective,
                                                         Opcode::jsr_effective,
                                                         Opcode::jcc_effective,
                                                         Opcode::jscc_effective},
                                                        {Opcode::bra_relative_short,
                                                         Opcode::bsr_relative_short,
                                                         Opcode::bcc_relative_short,
                                                         Opcode::bscc_relative_short},
                                                        {Opcode::bra_relative_long,
                                                         Opcode::bsr_relative_long,
                                                         Opcode::bcc_relative_long,
                                                         Opcode::bscc_relative_long},
                                                        {Opcode::bra_relative_register,
                                                         Opcode::bsr_relative_register,
                                                         Opcode::bcc_relative_register,
                                                         Opcode::bscc_relative_register},
                                                        {Opcode::jmp_effective,
                                                         Opcode::jsr_effective,
                                                         Opcode::jcc_effective,
                                                         Opcode::jscc_effective}}};
  const auto choice = (conditional ? 2U : 0U) + (call ? 1U : 0U);
  result.opcode = opcode(names.at(static_cast<unsigned>(form)).at(choice));
  if (form == Form::short_address) {
    result.opcode |= target | (conditional ? selector << 12U : 0U);
    result.cycles = conditional ? 4U : 3U;
  } else if (form == Form::absolute) {
    constexpr Word absolute_ea = 0x30;
    result.opcode |= (absolute_ea << 8U) | (conditional ? selector : 0U);
    result.extra = target;
    result.words = 2;
    result.cycles = 4;
  } else if (form == Form::register_indirect) {
    result.opcode |= (4U << 11U) | (offset_register << 8U) | (conditional ? selector : 0U);
    result.cycles = conditional ? 4U : 3U;
    result.register_target = true;
  } else if (form == Form::relative_short) {
    constexpr Word low = 31;
    constexpr Word upper = 0x1e0;
    result.opcode |= (displacement & low) | ((displacement & upper) << 1U) |
                     (conditional ? selector << 12U : 0U);
    result.cycles = 4;
  } else {
    return extended_relative(result, form);
  }
  return result;
}

bool selected(const Word status, const Encoding shape) {
  const auto selector = shape.selector;
  if (selector == always) { return true; }
  const bool c = (status & 1U) != 0;
  const bool v = (status & 2U) != 0;
  const bool z = (status & 4U) != 0;
  const bool n = (status & 8U) != 0;
  const bool u = (status & 16U) != 0;
  const bool e = (status & 32U) != 0;
  const bool l = (status & 64U) != 0;
  const std::array truth{!c,
                         n == v,
                         !z,
                         !n,
                         !z && (u || e),
                         !e,
                         !l,
                         !z && n == v,
                         c,
                         n != v,
                         z,
                         n,
                         z || (!u && !e),
                         e,
                         l,
                         z || n != v};
  return truth.at(selector);
}

Registers seed(const Encoding shape, Registers result) {
  result.pc = shape.origin;
  result.sc = shape.call && !selected(result.sr, shape) ? static_cast<Word>(stack_entries - 1) : 2U;
  result.a = accumulator_a_pattern;
  result.b = accumulator_b_pattern;
  result.x = x_pattern;
  result.y = y_pattern;
  result.m.fill(word_mask);
  const auto index = shape.register_target ? shape.target_register : offset_register;
  result.r[index] =
      shape.register_target ? shape.target : (shape.target + modulus - shape.origin) % modulus;
  result.stack.fill(frame_pattern);
  return result;
}

Registers predict(Registers state, const Encoding shape) {
  const auto next = shape.origin + shape.words;
  const bool taken = selected(state.sr, shape);
  if (taken && shape.call) {
    state.sp = (state.sp + 1) % pointer_range;
    ++state.sc;
    state.stack.at(state.sp % stack_entries) = (std::uint64_t{next} << word_bits) | state.sr;
  }
  state.pc = taken ? shape.target : next;
  return state;
}

Registers loop_prediction(const Registers &state, const Encoding shape) {
  auto result = predict(state, shape);
  if (result.pc != shape.origin + shape.words) { return result; }
  if (result.lc == 1 && (result.sr & detail::forever_flag) == 0) {
    loop_frames::complete(result);
  } else {
    result.lc = (result.lc - 1) & word_mask;
    result.pc = static_cast<Word>(result.stack[result.sp % stack_entries] >> word_bits);
  }
  return result;
}

std::uint32_t forbidden(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

void branches(const Encoding shape) {
  std::array<Word, entry + 4> program{};
  program[shape.origin] = shape.opcode;
  if (shape.words == 2) { program[shape.origin + 1] = shape.extra; }
  Core core(Memory::unified(program));
  detail::Cpu cpu;
  unsigned helper_calls = 0;
  detail::Jit native(cpu, program, {.owner = &helper_calls, .execute = forbidden, .memory = {}});
  constexpr std::array modes{Word{0x300}, Word{0xc00300}};
  constexpr std::array pointers{Word{0}, Word{15}, Word{31}, Word{63}};
  for (const auto mode : modes) {
    for (Word flags = 0; flags < status_count; ++flags) {
      for (const auto pointer : pointers) {
        auto selected_shape = shape;
        if (shape.register_target) {
          constexpr std::array targets{destination, entry + 1, word_mask, 0U};
          selected_shape.target = targets[flags % targets.size()];
        }
        const auto initial = seed(selected_shape, {.sr = mode | flags, .sp = pointer});
        const auto expected = predict(initial, selected_shape);
        core.set_registers(initial);
        const auto before = core.progress();
        const auto result = core.run(1);
        require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                    core.registers() == expected &&
                    result.progress.instructions == before.instructions + 1 &&
                    result.progress.estimated_cycles == before.estimated_cycles + shape.cycles,
                "Public control flow preserves independent full state, flags, stack and cycles");
        cpu.reg = initial;
        cpu.reg.a <<= detail::padding_bits;
        cpu.reg.b <<= detail::padding_bits;
        const auto previous_cycles = cpu.cycles;
        require(native.run(1) == 1 && helper_calls == 0,
                "Eligible control flow executes natively with the general helper forbidden");
        cpu.reg.a >>= detail::padding_bits;
        cpu.reg.b >>= detail::padding_bits;
        require(cpu.reg == expected && cpu.cycles == previous_cycles + shape.cycles,
                "Native-only control flow independently matches the public result");
      }
    }
  }
}

void register_targets() {
  for (unsigned index = 0; index < agu_registers; ++index) {
    for (const bool call : {false, true}) {
      for (unsigned selector = 0; selector <= always; ++selector) {
        auto shape = make(Form::register_indirect, call, selector);
        shape.opcode = (shape.opcode & ~(7U << 8U)) | (index << 8U);
        shape.target_register = index;
        branches(shape);
      }
    }
  }
}

void check_loop_branch(Core &core,
                       const Encoding shape,
                       detail::Jit &native,
                       detail::Cpu &cpu,
                       const Registers &initial,
                       const unsigned &helper_calls) {
  const auto expected = loop_prediction(initial, shape);
  core.set_registers(initial);
  const auto before = core.progress();
  const auto result =
      (initial.sr & 1U) != 0 ? core.run_until(before.estimated_cycles + 1) : core.run(1);
  require(result.retired == 1 && result.fault == Fault{} && core.registers() == expected &&
              result.progress ==
                  Progress{.instructions = before.instructions + 1,
                           .estimated_cycles = before.estimated_cycles + shape.cycles},
          "DO-tail branches preserve taken, fallthrough, next-target and parent frames");
  cpu.reg = initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  const auto cycles = cpu.cycles;
  const auto instructions = cpu.instructions;
  if (!loop_frames::admitted(initial)) {
    const auto unchanged = cpu.reg;
    require(native.run(1) == 0 && helper_calls == 0 && cpu.reg == unchanged &&
                cpu.cycles == cycles && cpu.instructions == instructions,
            "Unproved zero/forever DO tails decline without partial control effects");
    return;
  }
  require(native.run(1) == 1 && helper_calls == 0,
          "Eligible DO-tail branches execute with the general ISA helper forbidden");
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(cpu.reg == expected && cpu.cycles == cycles + shape.cycles &&
              cpu.instructions == instructions + 1,
          "Native DO-tail retirement independently preserves complete state and time");
}

void loop_branches(const Encoding shape) {
  std::array<Word, entry + 4> program{};
  program[shape.origin] = shape.opcode;
  if (shape.words == 2) { program[shape.origin + 1] = shape.extra; }
  Core core(Memory::unified(program));
  detail::Cpu cpu;
  unsigned helper_calls = 0;
  detail::Jit native(cpu, program, {.owner = &helper_calls, .execute = forbidden, .memory = {}});
  constexpr std::array pointers{Word{0}, Word{15}, Word{31}, Word{63}};
  constexpr Word parent_lc = 7;
  constexpr Word parent_la = 0x12345;
  for (const Word count : {3U, 1U, 0U}) {
    for (Word flags = 0; flags < status_count * 2; ++flags) {
      for (const auto pointer : pointers) {
        for (const bool forever_loop : {false, true}) {
          auto initial =
              seed(shape, {.sr = initial_status | detail::loop_flag | flags, .sp = pointer});
          if (forever_loop) { initial.sr |= detail::forever_flag; }
          initial.lc = count;
          initial.la = shape.origin + shape.words - 1;
          initial.stack[pointer % stack_entries] =
              (std::uint64_t{entry - 3} << word_bits) | detail::loop_flag | detail::forever_flag;
          initial.stack[(pointer + pointer_range - 1) % stack_entries] =
              (std::uint64_t{parent_la} << word_bits) | parent_lc;
          check_loop_branch(core, shape, native, cpu, initial, helper_calls);
        }
      }
    }
  }
}

void closed_step(Registers &state, Progress &progress, const Encoding shape) {
  if (state.pc == shape.origin) {
    state = loop_prediction(state, shape);
    progress.estimated_cycles += shape.cycles;
  } else {
    ++state.pc;
    ++progress.estimated_cycles;
  }
  ++progress.instructions;
}

void closed_loop_branches(const bool taken) {
  constexpr unsigned selector = 0;
  const auto shape = make(Form::register_indirect, false, selector, entry + 2, entry);
  std::array<Word, entry + 4> program{};
  program[entry] = isa::nop.front();
  program[entry + 1] = isa::nop.front();
  program[entry + 2] = shape.opcode;
  Core core(Memory::unified(program));
  detail::Cpu cpu;
  unsigned helper_calls = 0;
  detail::Jit native(cpu, program, {.owner = &helper_calls, .execute = forbidden, .memory = {}});
  auto initial =
      seed(shape, {.sr = initial_status | detail::loop_flag | (taken ? 0U : flag_c), .sp = 2});
  initial.pc = entry;
  initial.la = entry + 2;
  initial.lc = 3;
  initial.stack[2] = (std::uint64_t{entry} << word_bits);
  initial.stack[1] = std::uint64_t{7} << word_bits;
  constexpr Word total = 9;
  constexpr Word cycles_per_turn = 6;
  for (const bool deadline : {false, true}) {
    const auto limit = deadline ? total / 3 * cycles_per_turn : total;
    for (Word grant = 0; grant <= limit; ++grant) {
      auto expected = initial;
      Progress wanted;
      while (deadline ? wanted.estimated_cycles < grant : wanted.instructions < grant) {
        closed_step(expected, wanted, shape);
      }
      core.set_registers(initial);
      const auto before = core.progress();
      const auto result =
          deadline ? core.run_until(before.estimated_cycles + grant) : core.run(grant);
      require(
          result.retired == wanted.instructions && result.fault == Fault{} &&
              core.registers() == expected &&
              result.progress ==
                  Progress{.instructions = before.instructions + wanted.instructions,
                           .estimated_cycles = before.estimated_cycles + wanted.estimated_cycles},
          "Closed DO bodies preserve every instruction/deadline prefix around control edges");
      if (deadline) { continue; }
      cpu.reg = initial;
      cpu.reg.a <<= detail::padding_bits;
      cpu.reg.b <<= detail::padding_bits;
      const auto native_cycles = cpu.cycles;
      const auto native_instructions = cpu.instructions;
      require(native.run(grant) == wanted.instructions && helper_calls == 0,
              "Closed control bodies link without the generic executor at any prefix");
      cpu.reg.a >>= detail::padding_bits;
      cpu.reg.b >>= detail::padding_bits;
      require(cpu.reg == expected && cpu.cycles == native_cycles + wanted.estimated_cycles &&
                  cpu.instructions == native_instructions + wanted.instructions,
              "Native closed-loop control retains complete exact state and progress");
    }
  }
}

void returns() {
  std::array<Word, entry + 2> program{};
  program[entry] = isa::rts.front();
  Core core(Memory::unified(program));
  detail::Cpu cpu;
  unsigned helper_calls = 0;
  detail::Jit native(cpu, program, {.owner = &helper_calls, .execute = forbidden, .memory = {}});
  for (Word pointer = 0; pointer < pointer_range; ++pointer) {
    const Encoding shape;
    auto initial = seed(shape, {.sr = return_status, .sp = pointer});
    initial.stack.at(pointer % stack_entries) =
        (std::uint64_t{destination} << word_bits) | saved_status;
    auto expected = initial;
    expected.pc = destination;
    expected.sp = (pointer + pointer_range - 1) % pointer_range;
    --expected.sc;
    core.set_registers(initial);
    const auto result = core.run(1);
    require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                core.registers() == expected,
            "RTS pops every six-bit stack position without restoring saved status");
    cpu.reg = initial;
    cpu.reg.a <<= detail::padding_bits;
    cpu.reg.b <<= detail::padding_bits;
    require(native.run(1) == 1 && helper_calls == 0, "RTS never needs an instruction helper");
  }
}

void stack_faults() {
  for (const bool returning : {false, true}) {
    std::array<Word, entry + 2> program{};
    const auto shape = make(Form::short_address, true);
    program[entry] = returning ? isa::rts.front() : shape.opcode;
    Core core(Memory::unified(program));
    for (const auto pointer : {0U, 15U, 63U}) {
      core.reset();
      auto initial = seed(shape, {.sr = initial_status, .sp = pointer});
      initial.sc = returning ? 0U : static_cast<Word>(stack_entries - 1);
      core.set_registers(initial);
      auto expected = initial;
      expected.pc = returning ? entry + 1 : destination;
      const auto result = core.run(1);
      require(result.retired == 0 && result.fault.kind == FaultKind::stack &&
                  result.fault.pc == entry && result.fault.address == entry &&
                  result.fault.space == Space::program && result.fault.value == pointer &&
                  core.registers() == expected && result.progress.instructions == 0 &&
                  result.progress.estimated_cycles == 0,
              "Stack faults retain the original partial PC and unchanged stack without retirement");
      detail::Cpu cpu;
      cpu.reg = initial;
      cpu.reg.a <<= detail::padding_bits;
      cpu.reg.b <<= detail::padding_bits;
      cpu.interrupt_inhibited = true;
      cpu.repeat_completed = true;
      unsigned helper_calls = 0;
      Fault failure;
      detail::Jit native(cpu,
                         program,
                         {.owner = &helper_calls, .execute = forbidden, .memory = {}},
                         {.failure = &failure});
      require(native.run(1) == 0 && helper_calls == 0 && failure.kind == FaultKind::stack &&
                  failure.pc == entry && failure.address == entry && failure.value == pointer &&
                  cpu.repeat_completed && !cpu.interrupt_inhibited && cpu.instructions == 0 &&
                  cpu.cycles == 0,
              "Native stack faults preserve event gates and never call the general executor");
    }
  }
}

void live_flags_and_loop_frames() {
  constexpr unsigned equal_condition = 10;
  {
    std::array<Word, entry + 4> program{};
    program[entry] = opcode(Opcode::clr_a);
    program[entry + 1] =
        opcode(Opcode::jscc_absolute_short) | (equal_condition << 12U) | destination;
    Core core(Memory::unified(program));
    auto initial = seed(make(Form::short_address, true), {.sr = initial_status | flag_c});
    initial.sc = 0;
    core.set_registers(initial);
    const auto result = core.run(2);
    constexpr Word expected_status = 0x315; // CLR: Z/U set, V/N/E clear, C preserved.
    require(result.fault.kind == FaultKind::none && result.retired == 2 &&
                core.registers().pc == destination && core.registers().a == 0 &&
                core.registers().sr == expected_status && core.registers().sp == 1 &&
                core.registers().sc == 1 &&
                core.registers().stack[1] ==
                    ((std::uint64_t{entry + 2} << word_bits) | expected_status) &&
                result.progress.estimated_cycles == 5,
            "A native call consumes live preceding ALU flags and saves their full status");
  }
  {
    std::array<Word, entry + 5> program{};
    program[entry] = opcode(Opcode::do_immediate) | (3U << 8U);
    program[entry + 1] = entry + 3;
    program[entry + 2] = opcode(Opcode::jsr_absolute_short) | destination;
    program[destination] = isa::rts.front();
    program[entry + 3] = isa::nop.front();
    program[entry + 4] = isa::stop.front();
    Core core(Memory::unified(program));
    Registers initial;
    initial.pc = entry;
    initial.sr = initial_status;
    core.set_registers(initial);
    constexpr std::uint64_t loop_cycles = 36;
    const auto result = core.run(100);
    require(result.fault.kind == FaultKind::none && result.state == ExecutionState::stopped &&
                result.retired == 11 && result.progress.estimated_cycles == loop_cycles &&
                core.registers().pc == entry + 5 && core.registers().sp == 0 &&
                core.registers().sc == 0 && core.registers().lc == 0 &&
                core.registers().sr == initial_status,
            "Calls nested inside a real DO loop retain and restore its two parent frames");
  }
}

void arithmetic_control_prefixes() {
  constexpr Word prefix = 3;
  constexpr Word cleared_flags = flag_z | flag_u;
  constexpr Word changed_flags = flag_z | flag_u | flag_v | flag_n | flag_e;
  for (const auto form : forms) {
    for (const bool call : {false, true}) {
      for (unsigned selector = 0; selector <= always; ++selector) {
        const auto shape = make(form, call, selector);
        std::array<Word, entry + 4> program{};
        program[entry - prefix] = opcode(Opcode::clr_a);
        program[entry - 2] = opcode(Opcode::clr_b);
        program[entry - 1] = isa::nop.front();
        program[entry] = shape.opcode;
        if (shape.words == 2) { program[entry + 1] = shape.extra; }
        Core core(Memory::unified(program));
        for (const Word carry : {0U, flag_c}) {
          auto initial = seed(shape, {.sr = initial_status | flag_l | flag_s | carry});
          initial.pc = entry - prefix;
          initial.sc = 2;
          auto before_branch = initial;
          before_branch.a = before_branch.b = 0;
          before_branch.sr = (initial.sr & ~changed_flags) | cleared_flags;
          const auto expected = predict(before_branch, shape);
          core.set_registers(initial);
          const auto before = core.progress();
          const auto result = core.run(4);
          require(result.retired == 4 && result.fault.kind == FaultKind::none &&
                      core.registers() == expected &&
                      result.progress == Progress{.instructions = before.instructions + 4,
                                                  .estimated_cycles = before.estimated_cycles +
                                                                      prefix + shape.cycles},
                  "Packed native branch/call exits retain live ALU flags, stack and exact cycles");
        }
      }
    }
  }
}

void stack_fault_prefixes() {
  constexpr Word cleared_status = 0x315;
  for (const bool returning : {false, true}) {
    std::array<Word, entry + 5> program{};
    const auto shape = make(Form::short_address, true);
    program[entry] = opcode(Opcode::clr_a);
    program[entry + 1] = isa::nop.front();
    program[entry + 2] = isa::nop.front();
    program[entry + 3] = returning ? isa::rts.front() : shape.opcode;
    for (const auto pointer : {0U, 15U, 63U}) {
      auto initial = seed(shape, {.sr = initial_status | flag_c, .sp = pointer});
      initial.sc = returning ? 0U : static_cast<Word>(stack_entries - 1);
      auto expected = initial;
      expected.a = 0;
      expected.sr = cleared_status;
      expected.pc = returning ? entry + 4 : destination;
      Core core(Memory::unified(program));
      core.set_registers(initial);
      const auto result = core.run(4);
      require(result.retired == 3 && result.fault.kind == FaultKind::stack &&
                  result.fault.pc == entry + 3 && result.fault.address == entry + 3 &&
                  result.fault.value == pointer && core.registers() == expected &&
                  result.progress == Progress{.instructions = 3, .estimated_cycles = 3},
              "A failed native call/return publishes exactly its preceding ALU and clock prefix");
    }
  }
}

void warm_calls() {
  std::array<Word, entry + 2> program{};
  constexpr Word outer = 32;
  constexpr Word inner = 64;
  constexpr unsigned warm_steps = 60;
  constexpr unsigned run_steps = 6000;
  constexpr std::uint64_t run_cycles = 16000;
  program[0] = opcode(Opcode::jsr_absolute_short) | outer;
  program[1] = opcode(Opcode::jmp_absolute_short);
  program[outer] = opcode(Opcode::jsr_absolute_short) | inner;
  program[outer + 1] = isa::rts.front();
  program[inner] = isa::nop.front();
  program[inner + 1] = isa::rts.front();
  Core core(Memory::unified(program));
  for (unsigned step = 0; step < warm_steps; ++step) {
    require(core.run(1).retired == 1, "Warm every actual control and deadline continuation entry");
  }
  const auto images = codegen::publications();
  const auto before = core.progress();
  const auto result = core.run_until(before.estimated_cycles + run_cycles);
  require(result.fault.kind == FaultKind::none && result.retired == run_steps &&
              core.registers().pc == 0 && core.registers().sp == 0 && core.registers().sc == 0 &&
              result.progress.estimated_cycles == before.estimated_cycles + run_cycles &&
              codegen::publications() == images,
          "Warm nested calls stay linked through exact deadlines without replacing native code");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    using namespace dsp56300::test;
    closed_loop_branches(false);
    closed_loop_branches(true);
    register_targets();
    for (const auto form : forms) {
      for (unsigned selector = 0; selector <= always; ++selector) {
        loop_branches(make(form, false, selector));
        const auto words = form == Form::absolute || form == Form::relative_long ? 2U : 1U;
        loop_branches(make(form, false, selector, entry, entry + words));
      }
    }
    for (const auto form : forms) {
      for (const bool call : {false, true}) {
        for (unsigned selector = 0; selector <= always; ++selector) {
          branches(make(form, call, selector));
        }
      }
    }
    for (const auto form : {Form::relative_short, Form::relative_long, Form::relative_register}) {
      branches(make(form, false, always, 0, wrapped_target));
    }
    constexpr dsp56300::Word forward = entry + 64;
    constexpr dsp56300::Word distant = 0x123456;
    for (const auto form : forms) {
      branches(make(form, false, always, entry, forward));
      branches(make(form, true, always, entry, forward));
    }
    branches(make(Form::absolute, false, always, entry, distant));
    branches(make(Form::absolute, true, always, entry, distant));
    returns();
    stack_faults();
    arithmetic_control_prefixes();
    stack_fault_prefixes();
    warm_calls();
    live_flags_and_loop_frames();
    std::cout << checks << " native/public control checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
