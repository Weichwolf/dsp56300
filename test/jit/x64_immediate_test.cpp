// SPDX-License-Identifier: GPL-3.0-only
// Independent host bit-circuit/flag expectations for compact native immediates.
#include "architecture.hpp"
#include "jit.hpp"
#include "native/direct.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "x64.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <vector>

namespace dsp56300::test {
namespace {
using detail::x64::Code;
using detail::x64::Reg;
using detail::x64::Width;
constexpr unsigned byte_bits = 8, group_add = 0, group_and = 4, group_sub = 5, group_cmp = 7;
constexpr std::uint64_t cf = 1, pf = 4, af = 16, zf = 64, sf = 128, of = 2048;
constexpr std::uint64_t flags_mask = cf | pf | af | zf | sf | of;
constexpr std::uint64_t sign32 = std::uint64_t{1} << 31, boundary32 = std::uint64_t{1} << 32;
constexpr std::uint64_t sign64 = std::uint64_t{1} << 63;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr std::uint8_t marker = 0xa5;
constexpr auto owner_offset =
    offsetof(detail::NativeContext, runtime) + offsetof(detail::JitRuntime, owner);
constexpr std::array widths{Width::byte, Width::word, Width::dword, Width::qword};
constexpr std::array<std::uint64_t, 15> values{0,
                                               1,
                                               2,
                                               127,
                                               128,
                                               255,
                                               256,
                                               0x7fff,
                                               0x8000,
                                               sign32,
                                               boundary32 - 1,
                                               boundary32,
                                               sign64 - 1,
                                               sign64,
                                               maximum};
constexpr std::array<std::uint32_t, 14> immediates{0,
                                                   1,
                                                   127,
                                                   128,
                                                   255,
                                                   256,
                                                   0x7fff,
                                                   0x8000,
                                                   0xffff,
                                                   0x7fffffff,
                                                   0x80000000,
                                                   0xffffff7f,
                                                   0xffffff80,
                                                   0xffffffff};

constexpr unsigned byte_mask_host = 255, maximum_short = 127;
constexpr std::uint32_t negative_byte = 0xffffff80, below_negative_byte = 0xffffff7f;
constexpr auto maximum32 = std::numeric_limits<std::uint32_t>::max();
constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
constexpr std::array<std::uint8_t, 11> save{
    0x53, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x56};
constexpr std::array<std::uint8_t, 12> restore{
    0x5e, 0x41, 0x5f, 0x41, 0x5e, 0x41, 0x5d, 0x41, 0x5c, 0x5d, 0x5b, 0xc3};
constexpr std::array<std::uint8_t, 1> push_immediate{0x68}, pop_flags{0x9d};
constexpr std::array<std::uint8_t, 2> capture_flags{0x9c, 0x58};

struct Arithmetic {
  unsigned group;
  Width width;
  std::uint32_t immediate;
  std::uint64_t incoming;
};

struct Operands {
  std::uint64_t before, operand;
};

struct Probe {
  std::uint64_t input = 0, result = 0, flags = 0;
  std::array<std::uint8_t, 512> memory{};
};

Code entry() {
  Code code;
  code.raw(landing); // ENDBR64.
  // Save nonvolatile GPRs and the context pointer; RSP is never a test destination.
  code.raw(save);
  return code;
}

void flags(Code &code, const std::uint64_t incoming) {
  code.raw(push_immediate); // PUSH imm32; POPFQ, setting only defined user arithmetic bits.
  code.u32(static_cast<std::uint32_t>(incoming | 2));
  code.raw(pop_flags);
}

void finish(Code &code, const Reg result) {
  const auto pointer = result == Reg::r10 ? Reg::r11 : Reg::r10;
  code.load(pointer, Reg::sp, 0);
  code.load(pointer, pointer, owner_offset);
  code.store(pointer, offsetof(Probe, result), result);
  code.raw(capture_flags); // PUSHFQ; POP RAX, after preserving the complete result.
  code.store(pointer, offsetof(Probe, flags), Reg::ax);
  code.constant(Reg::ax, 0);
  code.raw(restore);
}

struct Expected {
  std::uint64_t result = 0, flags = 0, defined_flags = flags_mask;
};

Expected sum_bits(const Arithmetic spec, const Operands input) {
  const auto bits = static_cast<unsigned>(spec.width) * byte_bits;
  const bool subtract = spec.group == 3 || spec.group == 5 || spec.group == 7;
  bool carry = subtract ? (spec.group != 3 || (spec.incoming & cf) == 0)
                        : (spec.group == 2 && (spec.incoming & cf) != 0);
  Expected expected;
  for (unsigned bit = 0; bit < bits; ++bit) {
    const bool carry_in = carry;
    const auto left = static_cast<unsigned>((input.before >> bit) & 1);
    const auto right = static_cast<unsigned>((input.operand >> bit) & 1) ^ unsigned { subtract };
    const auto sum = left + right + unsigned{carry};
    expected.result |= std::uint64_t{sum & 1U} << bit;
    carry = sum >= 2;
    if (bit == 3 && (carry != subtract)) { expected.flags |= af; }
    if (bit + 1 == bits && carry_in != carry) { expected.flags |= of; }
  }
  if (carry != subtract) { expected.flags |= cf; }
  return expected;
}

Expected arithmetic(const Arithmetic spec, const std::uint64_t before) {
  const auto bits = static_cast<unsigned>(spec.width) * byte_bits;
  const auto mask = bits == 64 ? maximum : (std::uint64_t{1} << bits) - 1;
  auto operand = std::uint64_t{spec.immediate};
  if (spec.width == Width::qword && operand >= sign32) { operand |= maximum ^ (boundary32 - 1); }
  Expected expected;
  const bool logical = spec.group == 1 || spec.group == 4 || spec.group == 6;
  if (logical) {
    if (spec.group == 1) { expected.result = (before | operand) & mask; }
    if (spec.group == 4) { expected.result = (before & operand) & mask; }
    if (spec.group == 6) { expected.result = (before ^ operand) & mask; }
    expected.defined_flags &= ~af; // AF is undefined after host logical operations.
  } else {
    expected = sum_bits(spec, {.before = before, .operand = operand});
  }
  if ((std::popcount(expected.result & byte_mask_host) & 1) == 0) { expected.flags |= pf; }
  if (expected.result == 0) { expected.flags |= zf; }
  if (((expected.result >> (bits - 1)) & 1) != 0) { expected.flags |= sf; }
  if (spec.group == group_cmp) {
    expected.result = before;
  } else if (spec.width == Width::byte || spec.width == Width::word) {
    expected.result |= before & ~mask;
  }
  return expected;
}

void check(const native::Direct &code, Probe &probe, const Expected expected) {
  detail::Cpu cpu;
  cpu.reg = register_model::seed(0);
  const auto original = cpu.reg;
  detail::NativeContext context{.runtime = {.owner = &probe, .memory = {}}};
  require(code.enter(cpu, context) == 0 && probe.result == expected.result &&
              (probe.flags & expected.defined_flags) == expected.flags,
          "Native bytes preserve independent full-register result and every defined host flag");
  require(cpu.reg == original && cpu.instructions == 0 && cpu.cycles == 0 &&
              context.runtime.owner == &probe,
          "The encoding probe preserves DSP state and the execution context");
}

void constants(const Reg reg) {
  constexpr std::array constants{std::uint64_t{0},
                                 std::uint64_t{1},
                                 sign32 - 1,
                                 sign32,
                                 boundary32 - 1,
                                 boundary32,
                                 sign64 - 1,
                                 sign64,
                                 maximum - sign32,
                                 maximum - sign32 + 1,
                                 maximum};
  for (const auto value : constants) {
    for (const auto incoming : {std::uint64_t{0}, cf, of | zf, flags_mask}) {
      auto code = entry();
      code.constant(reg, maximum);
      flags(code, incoming);
      code.constant(reg, value);
      finish(code, reg);
      const native::Direct executable(code.bytes());
      Probe probe;
      check(executable, probe, {.result = value, .flags = incoming});
    }
  }
}

void register_case(const Reg reg, const Arithmetic spec) {
  auto code = entry();
  code.load(reg, Reg::si, owner_offset);
  code.load(reg, reg, offsetof(Probe, input));
  flags(code, spec.incoming);
  code.immediate(spec.group, reg, spec.immediate, spec.width);
  finish(code, reg);
  const native::Direct executable(code.bytes());
  for (const auto before : values) {
    Probe probe{.input = before};
    check(executable, probe, arithmetic(spec, before));
  }
}

void registers() {
  for (unsigned index = 0; index < 16; ++index) {
    const auto reg = static_cast<Reg>(index);
    if (reg == Reg::sp) { continue; }
    constants(reg);
    for (const auto width : widths) {
      for (unsigned group = 0; group < 8; ++group) {
        for (const auto immediate : immediates) {
          for (const auto incoming : {std::uint64_t{0}, flags_mask}) {
            register_case(
                reg,
                {.group = group, .width = width, .immediate = immediate, .incoming = incoming});
          }
        }
      }
    }
  }
}

std::uint64_t read(const Probe &probe, const std::size_t offset) {
  std::uint64_t result = 0;
  for (std::size_t byte = 0; byte < sizeof(result); ++byte) {
    result |= std::uint64_t{probe.memory.at(offset + byte)} << (byte * byte_bits);
  }
  return result;
}

void write(Probe &probe, const std::size_t offset, const std::uint64_t value) {
  for (std::size_t byte = 0; byte < sizeof(value); ++byte) {
    probe.memory.at(offset + byte) = static_cast<std::uint8_t>(value >> (byte * byte_bits));
  }
}

void memory_case(const Reg base, const Arithmetic spec, const std::size_t offset) {
  auto code = entry();
  code.load(base, Reg::si, owner_offset);
  code.immediate(group_add, base, offsetof(Probe, memory));
  flags(code, spec.incoming);
  code.memory_immediate(spec.group, base, offset, spec.width, spec.immediate);
  code.load(Reg::ax, base, offset);
  finish(code, Reg::ax);
  const native::Direct executable(code.bytes());
  for (const auto before : values) {
    Probe probe;
    probe.memory.fill(marker);
    write(probe, offset, before);
    auto expected = probe;
    auto result = arithmetic(spec, before);
    // A 32-bit memory operation preserves its adjacent high dword.
    if (spec.width == Width::dword && spec.group != group_cmp) {
      result.result |= before & ~(boundary32 - 1);
    }
    write(expected, offset, result.result);
    check(executable, probe, result);
    require(probe.memory == expected.memory && read(probe, offset) == result.result,
            "Memory ALU widths, both displacement sizes and all adjacent canary bytes are exact");
  }
}

void memory_shape(const Reg base, const Width width, const unsigned group) {
  constexpr std::array<std::size_t, 3> offsets{0, 127, 128};
  for (const auto immediate : immediates) {
    for (const auto incoming : {std::uint64_t{0}, flags_mask}) {
      for (const auto offset : offsets) {
        memory_case(base,
                    {.group = group, .width = width, .immediate = immediate, .incoming = incoming},
                    offset);
      }
    }
  }
}

void memory() {
  for (const auto base : {Reg::ax, Reg::r12, Reg::r13}) {
    for (const auto width : widths) {
      for (unsigned group = 0; group < 8; ++group) { memory_shape(base, width, group); }
    }
  }
}

void literal() {
  Code code;
  code.constant(Reg::ax, maximum_short);
  code.constant(Reg::r8, boundary32 - 1);
  code.constant(Reg::r9, maximum - sign32 + 1);
  code.constant(Reg::r10, boundary32);
  code.immediate(group_add, Reg::ax, maximum_short);
  code.immediate(group_add, Reg::ax, 128);
  code.immediate(group_and, Reg::r9, negative_byte, Width::dword);
  code.immediate(group_sub, Reg::cx, below_negative_byte, Width::dword);
  code.memory_immediate(group_add, Reg::r12, 0, Width::qword, maximum32);
  code.memory_immediate(group_sub, Reg::sp, 128, Width::word, negative_byte);
  code.memory_immediate(group_cmp, Reg::di, 0, Width::byte, byte_mask_host);
  // LLVM MC independently checked these immediates; existing ModRM address forms
  // deliberately keep their explicit zero displacements/byte-address REX prefix.
  const std::vector<std::uint8_t> expected{
      0xb8, 0x7f, 0x00, 0x00, 0x00, 0x41, 0xb8, 0xff, 0xff, 0xff, 0xff, 0x49, 0xc7, 0xc1,
      0x00, 0x00, 0x00, 0x80, 0x49, 0xba, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
      0x48, 0x83, 0xc0, 0x7f, 0x48, 0x05, 0x80, 0x00, 0x00, 0x00, 0x41, 0x83, 0xe1, 0x80,
      0x81, 0xe9, 0x7f, 0xff, 0xff, 0xff, 0x49, 0x83, 0x44, 0x24, 0x00, 0xff, 0x66, 0x83,
      0xac, 0x24, 0x80, 0x00, 0x00, 0x00, 0x80, 0x40, 0x80, 0x7f, 0x00, 0xff};
  require(code.bytes() == expected,
          "Compact constants, sign boundaries and operand widths match independent literal bytes");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::literal();
    dsp56300::test::registers();
    dsp56300::test::memory();
    std::cout << dsp56300::test::checks << " independent compact-x64 checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
