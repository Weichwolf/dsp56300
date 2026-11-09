// SPDX-License-Identifier: GPL-3.0-only
// ABI-only entries isolate native context routing from instruction decoding.
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/index.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word target = 0xfedcba, inactive = 0x1000000;
constexpr unsigned loop_shift = 7;
constexpr std::array<unsigned, 7> positions{10, 11, 13, 14, 17, 20, 21};
constexpr std::array<Word, 4> loops{0, 0x123456, word_mask, inactive};

struct SwitchInput {
  Word status = 0, loop = 0, pc = target;
  unsigned first = 0, last = 0;
  bool budget_correct = true;
};

std::uint32_t change_context(detail::Cpu *cpu, detail::NativeContext *context) noexcept {
  auto &input = *static_cast<SwitchInput *>(context->runtime.owner);
  ++input.first;
  input.budget_correct &= context->budget == 5;
  cpu->reg.sr = input.status;
  cpu->reg.la = input.loop;
  cpu->reg.pc = input.pc;
  ++cpu->instructions;
  context->boundary = detail::NativeBoundary::context;
  return 1;
}

std::uint32_t stop_context(detail::Cpu *cpu, detail::NativeContext *context) noexcept {
  auto &input = *static_cast<SwitchInput *>(context->runtime.owner);
  ++input.last;
  input.budget_correct &= context->budget == 4;
  ++cpu->instructions;
  context->boundary = detail::NativeBoundary::owner;
  return 1;
}

struct Case {
  Word mode = 0, loop = 0;
  bool active = false, known = true;
};

Word status(const Case point) {
  Word status = point.active ? detail::loop_flag : 0;
  for (unsigned bit = 0; bit < positions.size(); ++bit) {
    if ((point.mode & (1U << bit)) != 0) { status |= 1U << positions[bit]; }
  }
  return status;
}

void check(const Case point) {
  const auto mode = status(point);
  SwitchInput input{.status = mode, .loop = point.loop};
  detail::Cpu cpu;
  detail::NativeContext context;
  context.runtime.owner = &input;
  detail::NativeTable destinations(3);
  detail::NativeTable catalog(4);
  const auto key = ((point.active ? point.loop : inactive) << loop_shift) | point.mode;
  if (point.known) {
    destinations.insert(target, std::bit_cast<const void *>(&stop_context));
    catalog.insert(key, static_cast<const void *>(destinations.index().data()));
  }
  const auto retired = detail::enter_blocks(std::bit_cast<const void *>(&change_context),
                                            cpu,
                                            context,
                                            5,
                                            destinations.index(),
                                            &catalog.index());
  const bool linked = point.known && (!point.active || point.loop <= word_mask);
  const unsigned expected = linked ? 2U : 1U;
  require(retired == expected && cpu.instructions == expected && input.first == 1 &&
              input.last == (linked ? 1U : 0U) && input.budget_correct && cpu.reg.pc == target &&
              cpu.reg.sr == mode && cpu.reg.la == point.loop &&
              context.boundary ==
                  (linked ? detail::NativeBoundary::owner : detail::NativeBoundary::context),
          "Known contexts stay native; unknown contexts return without executing another entry");
}

constexpr Word sequence_stop = 21, unmapped_loop = 0xabcdef;
enum class Ending : std::uint8_t { owner, missing_context, missing_pc };

struct Sequence {
  std::array<Case, 5> routes{};
  unsigned size = 0, calls = 0;
  Word budget = 0;
  Ending ending = Ending::owner;
  bool correct = true;
};

void next_context(Registers &reg, const Sequence &sequence, const unsigned next) {
  const auto &route = sequence.routes[next];
  reg.sr = status(route);
  reg.la = route.loop;
  reg.pc = target;
  if (sequence.calls != sequence_stop - 1) { return; }
  if (sequence.ending == Ending::missing_context) {
    reg.sr = detail::loop_flag;
    reg.la = unmapped_loop;
  }
  if (sequence.ending == Ending::missing_pc) { reg.pc = target - 1; }
}

template <unsigned Index>
std::uint32_t route_sequence(detail::Cpu *cpu, detail::NativeContext *context) noexcept {
  auto &sequence = *static_cast<Sequence *>(context->runtime.owner);
  if (context->budget == 0) { return 0; }
  const auto &route = sequence.routes[Index];
  sequence.correct &= cpu->reg.sr == status(route) && cpu->reg.la == route.loop &&
                      cpu->reg.pc == target &&
                      context->budget == sequence.budget - sequence.calls &&
                      context->boundary == detail::NativeBoundary::none;
  ++sequence.calls;
  ++cpu->instructions;
  cpu->cycles += Index + 1;
  ++cpu->reg.r[Index];
  next_context(cpu->reg, sequence, (Index + 1) % sequence.size);
  context->boundary = sequence.calls == sequence_stop ? detail::NativeBoundary::owner
                                                      : detail::NativeBoundary::context;
  return 1;
}

void sequence_case(Sequence sequence) {
  const std::array entries{std::bit_cast<const void *>(&route_sequence<0>),
                           std::bit_cast<const void *>(&route_sequence<1>),
                           std::bit_cast<const void *>(&route_sequence<2>),
                           std::bit_cast<const void *>(&route_sequence<3>),
                           std::bit_cast<const void *>(&route_sequence<4>)};
  detail::NativeTable catalog(4);
  std::array destinations{detail::NativeTable{3},
                          detail::NativeTable{3},
                          detail::NativeTable{3},
                          detail::NativeTable{3},
                          detail::NativeTable{3}};
  for (unsigned index = 0; index < sequence.size; ++index) {
    const auto &route = sequence.routes[index];
    destinations[index].insert(target, entries[index]);
    const auto key = ((route.active ? route.loop : inactive) << loop_shift) | route.mode;
    catalog.insert(key, static_cast<const void *>(destinations[index].index().data()));
  }
  detail::Cpu cpu;
  cpu.reg.x = word_mask;
  cpu.reg.y = word_mask - 1;
  cpu.reg.a = word_mask - 2;
  cpu.reg.b = word_mask - 3;
  cpu.reg.r.fill(16);
  next_context(cpu.reg, sequence, 0);
  auto expected = cpu.reg;
  const Word stop = sequence_stop - (sequence.ending == Ending::owner ? 0U : 1U);
  const auto count = std::min(sequence.budget, stop);
  std::uint64_t cycles = 0;
  for (Word step = 0; step < count; ++step) {
    const auto index = step % sequence.size;
    ++expected.r[index];
    cycles += index + 1;
    auto prefix = sequence;
    prefix.calls = step + 1;
    next_context(expected, prefix, (index + 1) % sequence.size);
  }
  detail::NativeContext context{.runtime = {.owner = &sequence, .memory = {}}};
  const auto retired = detail::enter_blocks(
      entries[0], cpu, context, sequence.budget, destinations[0].index(), &catalog.index());
  auto boundary = count == 0 ? detail::NativeBoundary::none : detail::NativeBoundary::context;
  if (count == sequence_stop) { boundary = detail::NativeBoundary::owner; }
  if (count == stop && sequence.budget > stop && sequence.ending == Ending::missing_pc) {
    boundary = detail::NativeBoundary::none;
  }
  require(retired == count && sequence.calls == count && sequence.correct &&
              cpu.instructions == count && cpu.cycles == cycles && cpu.reg == expected &&
              context.boundary == boundary,
          "Repeated full context identities preserve exact routes, owner exits and every prefix");
}

void repeated_contexts() {
  constexpr Word same_loop = 0x123456;
  constexpr Word other_loop = 0x654321;
  for (unsigned bit = 0; bit < positions.size(); ++bit) {
    for (const Word mode : {0U, (1U << loop_shift) - 1}) {
      for (const unsigned size : {2U, 3U, 4U, 5U}) {
        for (const auto ending : {Ending::owner, Ending::missing_context, Ending::missing_pc}) {
          for (const Word budget : {0U, 1U, 9U, sequence_stop - 1, sequence_stop + 4}) {
            sequence_case(
                {.routes = {Case{.mode = mode, .loop = same_loop, .active = true},
                            Case{.mode = mode ^ (1U << bit), .loop = same_loop, .active = true},
                            Case{.mode = mode, .loop = word_mask, .active = false},
                            Case{.mode = mode, .loop = word_mask, .active = true},
                            Case{.mode = mode, .loop = other_loop, .active = true}},
                 .size = size,
                 .budget = budget,
                 .ending = ending});
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
    for (dsp56300::Word mode = 0; mode < (1U << dsp56300::test::loop_shift); ++mode) {
      for (const auto loop : dsp56300::test::loops) {
        for (const bool active : {false, true}) {
          for (const bool known : {false, true}) {
            dsp56300::test::check({.mode = mode, .loop = loop, .active = active, .known = known});
          }
        }
      }
    }
    dsp56300::test::repeated_contexts();
    std::cout
        << "Native context routing: all numeric bits, loop identities and owner exits passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
