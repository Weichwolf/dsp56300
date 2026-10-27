// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "guest.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <linux/kvm.h>
#include <memory>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::array chain{Word{0x2000d0},
                           Word{0x2000d2},
                           Word{0x2000fa},
                           Word{0x200010},
                           Word{0x200018},
                           Word{0x200036},
                           Word{0x20002e},
                           Word{0x200011},
                           Word{0x200005},
                           Word{0x200069}};
constexpr std::array inputs{Word{0}, Word{1}, Word{0x7fffff}, Word{0x800000}, Word{0xffffff}};
constexpr std::uint64_t unmapped_address = 2 * kvm::Guest::memory_size;
constexpr int skipped = 77;
constexpr std::uint8_t halt_opcode = 0xf4;

std::vector<detail::NativeOp> operations(const std::span<const Word> program, const Word mode) {
  std::vector<detail::NativeOp> result;
  for (const auto opcode : program) {
    result.push_back({.instruction = detail::decode(opcode),
                      .pc = start + static_cast<Word>(result.size()),
                      .mode = mode});
  }
  return result;
}

void check_case(kvm::Guest &guest, const Registers &initial, const std::size_t count) {
  auto *cpu = std::construct_at(
      static_cast<detail::Cpu *>(guest.address(kvm::Guest::cpu_address, sizeof(detail::Cpu))));
  cpu->reg = initial;
  cpu->reg.a <<= detail::padding_bits;
  cpu->reg.b <<= detail::padding_bits;
  cpu->instructions = 8;
  cpu->cycles = 16;
  cpu->repeat_completed = true;
  cpu->interrupt_inhibited = true;
  std::construct_at(static_cast<detail::NativeContext *>(
      guest.address(kvm::Guest::context_address, sizeof(detail::NativeContext))));
  // No moves or helpers: all context pointers remain null, with no host address
  // embedded in the arithmetic code or its guest-visible runtime context.
  const auto ops =
      operations(std::span(chain).first(count), initial.sr & detail::numeric_mode_mask);
  detail::JitEmitter emitter({}, true);
  const auto code = emitter.generate(ops);
  auto expected = initial;
  for (const auto opcode : std::span(chain).first(count)) {
    const auto old = expected;
    sequence_model::alu(expected, old, opcode);
    ++expected.pc;
  }
  const auto exit = guest.run(code);
  require(exit.reason == KVM_EXIT_HLT && exit.result == count &&
              exit.rip == kvm::Guest::halt_address + 1,
          "Generated DSP code returns its exact retirement count to the KVM halt boundary");
  auto actual = cpu->reg;
  actual.a >>= detail::padding_bits;
  actual.b >>= detail::padding_bits;
  require(actual == expected, "All DSP registers match the independent arithmetic model in KVM");
  require(cpu->instructions == 8 + count && cpu->cycles == 16 + count &&
              cpu->repeat_remaining == 0 && cpu->repeat_pc == 0 && cpu->repeat_lc == 0 &&
              cpu->repeat_opcode == 0 && !cpu->repeat_loaded && !cpu->repeat_completed &&
              !cpu->interrupt_inhibited,
          "Guest retirement updates shared CPU progress and preserves hidden continuation state");
}

void arithmetic(kvm::Guest &guest) {
  for (const auto scale : {Word{0}, mode_down, mode_up}) {
    for (const auto rounding : {Word{0}, mode_rm}) {
      for (const auto x : inputs) {
        for (const auto y : inputs) {
          auto initial = register_model::seed(scale | rounding | flag_l | flag_s | flag_c);
          initial.x = (std::uint64_t{x} << word_bits) | y;
          initial.y = (std::uint64_t{y} << word_bits) | x;
          for (std::size_t count = 1; count <= chain.size(); ++count) {
            check_case(guest, initial, count);
          }
        }
      }
    }
  }
}

void unmapped_memory() {
  // Separate guest: do not complete the MMIO transaction with fabricated data.
  kvm::Guest guest;
  detail::x64::Code code;
  code.constant(detail::x64::Reg::ax, unmapped_address);
  code.load(detail::x64::Reg::ax, detail::x64::Reg::ax, 0);
  code.byte(halt_opcode);
  const auto exit = guest.run(code.bytes());
  require(exit.reason == KVM_EXIT_MMIO && exit.address == unmapped_address &&
              exit.length == sizeof(std::uint64_t) && !exit.write,
          "An unbacked guest physical read exits without reading unrelated host memory");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::kvm::Guest guest;
    dsp56300::test::arithmetic(guest);
    dsp56300::test::unmapped_memory();
    std::cout << dsp56300::test::checks << " KVM translation experiment checks passed\n";
  } catch (const dsp56300::test::kvm::Unavailable &error) {
    std::cerr << "KVM experiment unavailable: " << error.what() << '\n';
    return dsp56300::test::skipped;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
