// SPDX-License-Identifier: GPL-3.0-only
// ABI-only entries isolate native context routing from instruction decoding.
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/index.hpp"
#include "support.hpp"

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

void check(const Case point) {
  Word status = point.active ? detail::loop_flag : 0;
  for (unsigned bit = 0; bit < positions.size(); ++bit) {
    if ((point.mode & (1U << bit)) != 0) { status |= 1U << positions[bit]; }
  }
  SwitchInput input{.status = status, .loop = point.loop};
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
              cpu.reg.sr == status && cpu.reg.la == point.loop &&
              context.boundary ==
                  (linked ? detail::NativeBoundary::owner : detail::NativeBoundary::context),
          "Known contexts stay native; unknown contexts return without executing another entry");
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
    std::cout
        << "Native context routing: all numeric bits, loop identities and owner exits passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
