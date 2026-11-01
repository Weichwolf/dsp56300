// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
using dsp56300::test::staged_test::Bus;
using dsp56300::test::staged_test::pending;
constexpr Word vector = 0x40, foreground = 0x100;
constexpr Word payload = 0x765432, incoming = 0xab765432, outgoing = 0xabcdef;
constexpr Word high_half = 0x123456, initial_status = 0x95, foreground_move = 0x377700;
constexpr Word foreground_value = 0x77, register_mask = 0x3f00;
constexpr unsigned effect_bound = 32;
constexpr std::array policies{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates,
                              InstructionPublication::pcu_updates};

struct Form {
  Word opcode, address;
  Space space;
  bool write;
};

// FM13 MOVEP register encodings: X/Y, low/high I/O groups, both directions.
// Low index 32 and Y index 31 exercise opposite sides of the split q field.
constexpr std::array forms{
    Form{.opcode = 0x0444c0, .address = 0xffffa0, .space = Space::x, .write = false},
    Form{.opcode = 0x04c4c0, .address = 0xffffa0, .space = Space::x, .write = true},
    Form{.opcode = 0x04443f, .address = 0xffff9f, .space = Space::y, .write = false},
    Form{.opcode = 0x04c43f, .address = 0xffff9f, .space = Space::y, .write = true},
    Form{.opcode = 0x084400, .address = 0xffffc0, .space = Space::x, .write = false},
    Form{.opcode = 0x08c400, .address = 0xffffc0, .space = Space::x, .write = true},
    Form{.opcode = 0x09443f, .address = 0xffffff, .space = Space::y, .write = false},
    Form{.opcode = 0x09c43f, .address = 0xffffff, .space = Space::y, .write = true}};

Registers initial() {
  auto result = dsp56300::test::initial_state();
  result.pc = foreground;
  result.x = (std::uint64_t{high_half} << dsp56300::test::word_bits) | outgoing;
  result.sr = initial_status;
  result.sp = 2;
  result.sc = 2;
  result.stack.at(1) = result.x;
  result.stack.at(2) = result.x + 1;
  result.la = foreground + 8;
  result.lc = effect_bound;
  return result;
}

Bus script(const Form form, const Word slot, const bool accept = true) {
  Bus bus;
  bus.pc = vector;
  bus.events = {
      {.space = Space::program, .address = vector, .value = slot == 0 ? form.opcode : 0},
      {.space = Space::program, .address = vector + 1, .value = slot == 1 ? form.opcode : 0},
      {.kind = form.write ? EffectKind::write : EffectKind::read,
       .space = form.space,
       .address = form.address,
       .value = form.write ? outgoing : incoming,
       .accept = accept}};
  return bus;
}

void finish(Core &core) {
  for (unsigned effect = 0; effect < effect_bound; ++effect) {
    const auto result = core.advance_instruction(pending(core).ticket);
    require(result.fault.kind == FaultKind::none, "Legal fast MOVEP has no native fault");
    if (result.retired) { return; }
  }
  throw std::runtime_error("Fast MOVEP exceeded its bounded native effects");
}

void reach_operand(
    Core &core, Bus &bus, const Form form, const Word slot, const InstructionPublication policy) {
  const auto first = core.begin_fetch(policy);
  require(first.fault.kind == FaultKind::none, "The first vector begins without fault");
  if (slot == 1) {
    finish(core);
    require(bus.calls == 2 && core.registers() == initial(),
            "A first-slot NOP retires only after accepting the complete vector pair");
    bus.pc = vector + 1;
    static_cast<void>(core.begin_fetch(policy));
  }
  for (unsigned effect = 0; effect < effect_bound; ++effect) {
    require(core.fault().kind == FaultKind::none,
            "Legal register MOVEP must reach its actual peripheral operand");
    const auto selected = pending(core);
    if (selected.effect.kind == (form.write ? EffectKind::write : EffectKind::read) &&
        selected.effect.space != Space::program) {
      require(selected.effect.space == form.space && selected.effect.address == form.address &&
                  selected.effect.pc == vector + slot && bus.calls == 2 && bus.accepted == 2 &&
                  core.registers() == initial(),
              "Both accepted vector words precede the precise vector-addressed operand");
      return;
    }
    static_cast<void>(core.advance_instruction(selected.ticket));
  }
  throw std::runtime_error("Fast MOVEP did not offer its actual peripheral operand");
}

void complete(
    Core &core, Bus &bus, const Form form, const Word slot, const InstructionPublication policy) {
  finish(core);
  if (slot == 0) {
    static_cast<void>(core.begin_fetch(policy));
    finish(core);
  }
  auto expected = initial();
  if (!form.write) {
    expected.x = (std::uint64_t{high_half} << dsp56300::test::word_bits) | payload;
  }
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = 2} &&
              core.fault().kind == FaultKind::none && bus.exact && bus.calls == 3 &&
              bus.accepted == 3 && !core.instruction_fetch().exception_vector,
          "Fast MOVEP retains the full foreground/stack/CCR and performs its operand once");
  if (form.write) {
    require((form.space == Space::x ? bus.x_written : bus.y_written) == outgoing,
            "The actual peripheral write contains the original X0 word");
  }
  bus.pc = foreground;
  bus.events.push_back({.space = Space::program, .address = foreground, .value = foreground_move});
  static_cast<void>(core.begin_fetch(policy));
  finish(core);
  expected.r[7] = foreground_value;
  ++expected.pc;
  require(core.registers() == expected && core.progress().instructions == 3 && bus.exact &&
              bus.calls == 4 && bus.accepted == 4,
          "The actual foreground instruction resumes without RTI or duplicate peripheral I/O");
}

void staged(const Backend backend,
            const InstructionPublication policy,
            const Form form,
            const Word slot) {
  auto bus = script(form, slot);
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = foreground_move});
  reach_operand(core, bus, form, slot, policy);
  auto before_bus = bus;
  Core before(before_bus,
              {.backend = backend == Backend::reference ? Backend::jit : Backend::reference});
  before.restore(core.checkpoint());
  static_cast<void>(core.advance_instruction(pending(core).ticket));
  require(bus.exact && bus.calls == 3 && bus.accepted == 3,
          "One native advance accepts exactly one real peripheral effect");
  auto after_bus = bus;
  Core after(after_bus, {.backend = Backend::cached});
  after.restore(core.checkpoint());
  complete(core, bus, form, slot, policy);
  complete(before, before_bus, form, slot, policy);
  complete(after, after_bus, form, slot, policy);
  require(core.registers() == before.registers() && core.registers() == after.registers() &&
              core.progress() == before.progress() && core.progress() == after.progress(),
          "Fresh owners on both sides of the operand retain native continuation exactly");
}

void refused(const Backend backend,
             const InstructionPublication policy,
             const Form form,
             const Word slot) {
  auto bus = script(form, slot, false);
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = foreground_move});
  reach_operand(core, bus, form, slot, policy);
  const auto selected = pending(core);
  const auto result = core.advance_instruction(selected.ticket);
  require(
      result.fault == Fault{.kind = form.write ? FaultKind::memory_write : FaultKind::memory_read,
                            .space = form.space,
                            .address = form.address,
                            .value = form.write ? outgoing : 0,
                            .pc = vector + slot} &&
          !result.retired && core.registers() == initial() &&
          core.progress() == Progress{.instructions = slot, .estimated_cycles = slot} &&
          bus.exact && bus.calls == 3 && bus.accepted == 2 && !core.instruction_outcome(),
      "Refused register MOVEP reports the precise operand without retirement or stack mutation");
  dsp56300::test::staged_test::rejected(
      [&] { static_cast<void>(core.advance_instruction(selected.ticket)); });
}

void restricted(const Backend backend,
                const Word slot,
                const InstructionPublication policy,
                Form form,
                const Word reg) {
  form.opcode = (form.opcode & ~register_mask) | (reg << 8);
  auto bus = script(form, slot);
  Core core(bus, {.backend = backend});
  core.set_registers(initial());
  core.request_interrupt(vector, 0, {.address = foreground, .opcode = foreground_move});
  static_cast<void>(core.begin_fetch(policy));
  if (slot == 1) {
    finish(core);
    static_cast<void>(core.begin_fetch(policy));
  }
  for (unsigned effect = 0; effect < effect_bound && core.fault().kind == FaultKind::none;
       ++effect) {
    static_cast<void>(core.advance_instruction(pending(core).ticket));
  }
  require(core.fault() == Fault{.kind = FaultKind::unsupported_instruction,
                                .space = Space::program,
                                .address = vector + slot,
                                .value = form.opcode,
                                .pc = vector + slot} &&
              core.registers() == initial() &&
              core.progress() == Progress{.instructions = slot, .estimated_cycles = slot} &&
              bus.exact && bus.calls == 2 && bus.accepted == 2 && !core.instruction_outcome(),
          "FM A.3.7 prohibits SSH source and SC/SP/SSH/SSL/LA/LC destinations before any operand");
}

void atomic(const Backend backend, const Form form, const Word slot) {
  std::array<Word, dsp56300::test::memory_words> p{};
  std::array<Word, dsp56300::test::memory_words> x{};
  std::array<Word, dsp56300::test::memory_words> y{};
  p.at(vector + slot) = form.opcode;
  p.at(foreground) = foreground_move;
  auto bus = script(form, slot);
  bus.pc = vector + slot;
  bus.events.erase(bus.events.begin(), bus.events.begin() + 2);
  Core core({.program = p, .x = x, .y = y}, &bus, {.backend = backend});
  core.set_registers(initial());
  core.interrupt(vector, 0);
  auto expected = initial();
  if (!form.write) {
    expected.x = (std::uint64_t{high_half} << dsp56300::test::word_bits) | payload;
  }
  require(core.fault().kind == FaultKind::none &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = 2} &&
              core.registers() == expected && bus.exact && bus.calls == 1 && bus.accepted == 1,
          "Atomic entry executes either register-MOVEP slot with exactly one actual operand");
  const auto result = core.run(1);
  expected.r[7] = foreground_value;
  ++expected.pc;
  require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
              core.registers() == expected && core.progress().instructions == 3 && bus.exact &&
              bus.calls == 1 && bus.accepted == 1,
          "Atomic entry resumes its actual foreground without another peripheral transaction");
}

unsigned restrictions(const Backend backend,
                      const InstructionPublication policy,
                      const Form form,
                      const Word slot) {
  // FM A.3.7 control destinations: SC, SP, SSH, SSL, LA, LC.
  constexpr std::array<Word, 6> destinations{49, 59, 60, 61, 62, 63};
  constexpr Word ssh = 60;
  if (form.write) {
    restricted(backend, slot, policy, form, ssh);
    return 1;
  }
  for (const auto reg : destinations) { restricted(backend, slot, policy, form, reg); }
  return destinations.size();
}
}

int main() {
  try {
    unsigned positive = 0;
    unsigned refusal = 0;
    unsigned guards = 0;
    unsigned atomic_cases = 0;
    for (const auto backend : dsp56300::test::staged_test::backends) {
      for (const auto policy : policies) {
        for (const auto form : forms) {
          for (const Word slot : {0U, 1U}) {
            staged(backend, policy, form, slot);
            ++positive;
            refused(backend, policy, form, slot);
            ++refusal;
            guards += restrictions(backend, policy, form, slot);
          }
        }
      }
      for (const auto form : forms) {
        for (const Word slot : {0U, 1U}) {
          atomic(backend, form, slot);
          ++atomic_cases;
        }
      }
    }
    std::cout << positive << " register-MOVEP vectors, " << refusal << " refusals, " << guards
              << " restrictions, " << atomic_cases << " atomic entries passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
