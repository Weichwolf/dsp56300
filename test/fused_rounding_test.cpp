// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace dsp56300::test {
namespace {
enum class Kind : std::uint8_t { fused, mac, round };
enum class Method : std::uint8_t { ordinary, bounded, captured };
constexpr Word down = 0x400, rm = 0x200000, changed = 0x3e;
constexpr std::uint64_t wrapped = 0x7fffffff000000, other = 0x123456789abcde;
constexpr std::array policies{InstructionPublication::retirement,
                              InstructionPublication::address_updates,
                              InstructionPublication::move_updates,
                              InstructionPublication::alu_updates,
                              InstructionPublication::unit_updates,
                              InstructionPublication::pcu_updates};
constexpr std::array backends{
    Backend::reference, Backend::cached, Backend::jit, Backend::automatic};

struct Input {
  Kind kind;
  Word incoming;
  bool biased, destination_b;
};

Word opcode(const Input input) {
  constexpr Word mac = 0x2000d2;
  constexpr Word rnd = 0x200011;
  const auto code = input.kind == Kind::round ? rnd : mac | (input.kind == Kind::fused ? 1U : 0U);
  return code | (input.destination_b ? 8U : 0U);
}

void captured(Core &cpu, const Input input, const InstructionPublication policy) {
  auto result = cpu.begin_instruction({.address = start, .opcode = opcode(input)}, policy);
  constexpr unsigned maximum_effects = 16;
  for (unsigned index = 0; index < maximum_effects && result.pending; ++index) {
    const auto pending = *result.pending;
    const auto &effect = pending.effect;
    require(effect.kind != EffectKind::read && effect.kind != EffectKind::write &&
                effect.kind != EffectKind::bit && effect.kind != EffectKind::reset,
            "The captured authored arithmetic has no fabricated owner data effects");
    if (input.kind == Kind::fused && effect.kind == EffectKind::alu_update) {
      if (!effect.alu_update) { throw std::logic_error("Missing actual ALU publication payload"); }
      for (const auto &update :
           std::span(effect.alu_update->updates).first(effect.alu_update->count)) {
        require(update.storage != RegisterStorage::sr || (update.mask & flag_l) == 0,
                "A cancelled MACR overflow cannot publish a spurious sticky-L writer");
      }
    }
    result = cpu.advance_instruction(pending.ticket);
  }
  require(result.retired && result.fault.kind == FaultKind::none,
          "Every actual captured policy reaches genuine arithmetic retirement");
}

void check(const Input input,
           const Backend backend,
           const Method method,
           const InstructionPublication policy = InstructionPublication::retirement) {
  std::array<Word, memory_words> p{};
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  p[start] = opcode(input);
  Core cpu({.program = p, .x = x, .y = y}, nullptr, {.backend = backend});
  auto before = initial_state();
  before.sr = down | input.incoming | (input.biased ? rm : 0U);
  before.x = 1;
  before.y = sign24;
  const auto value = input.kind == Kind::round ? wrapped : sign56;
  before.a = input.destination_b ? other : value;
  before.b = input.destination_b ? value : other;
  cpu.set_registers(before);
  auto wanted = before;
  ++wanted.pc;
  const auto output = input.kind == Kind::mac ? wrapped : sign56;
  (input.destination_b ? wanted.b : wanted.a) = output;
  constexpr Word fused_flags = 0x38;
  constexpr Word mac_flags = 0x72;
  constexpr Word round_flags = 0x7a;
  Word flags = fused_flags;
  if (input.kind == Kind::mac) { flags = mac_flags; }
  if (input.kind == Kind::round) { flags = round_flags; }
  wanted.sr = (before.sr & ~changed) | flags;
  if (method == Method::captured) {
    captured(cpu, input, policy);
  } else {
    const auto run = method == Method::bounded
                         ? cpu.run(RunBudget{.instructions = 1, .cycles = 1}).execution
                         : cpu.run(1);
    require(run.dispatches == 1 && run.fault.kind == FaultKind::none,
            "The actual authored instruction retires without a synthetic result");
    if (cpu.backend() == Backend::jit) {
      require(cpu.jit_statistics().native_alu_instructions == 1 &&
                  cpu.jit_statistics().helper_instructions == 0,
              "Default and explicit JIT execution retain actual native MACR/MAC/RND lowering");
    }
  }
  require(cpu.registers() == wanted &&
              cpu.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Literal fused cancellation and independent MAC/RND controls preserve complete state");
}

void cases() {
  for (const auto kind : {Kind::fused, Kind::mac, Kind::round}) {
    for (const Word incoming : {0U, flag_c | flag_v, status_byte}) {
      for (const bool biased : {false, true}) {
        for (const bool destination_b : {false, true}) {
          const Input input{
              .kind = kind, .incoming = incoming, .biased = biased, .destination_b = destination_b};
          for (const auto backend : backends) {
            check(input, backend, Method::ordinary);
            check(input, backend, Method::bounded);
            for (const auto policy : policies) { check(input, backend, Method::captured, policy); }
          }
        }
      }
    }
  }
}
}
}

int main() {
  try {
    dsp56300::test::cases();
    std::cout << dsp56300::test::checks << " literal rounded-product checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
