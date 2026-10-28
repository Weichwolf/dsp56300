// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/backend.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/simd.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native/direct.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace dsp56300::test {
namespace {
using detail::x64::Reg;
using detail::x64::Xmm;
static_assert(CoreImplementation<Core>);
static_assert(CoreImplementation<BackendCore<Backend::reference>>);
static_assert(CoreImplementation<BackendCore<Backend::cached>>);
static_assert(CoreImplementation<BackendCore<Backend::jit>>);
static_assert(sizeof(BackendCore<Backend::jit>) == sizeof(Core));
static_assert(SimdLayout<32, 128>::lanes == 4);
static_assert(SimdLayout<64, 256>::lanes == 4);
static_assert(SimdShape<16, 128> && !SimdShape<detail::word_bits, 128> && !SimdShape<0, 128>);
static_assert(!detail::x64::ScalarTransferLane<16>);

struct WrongRunResult : BackendCore<Backend::reference> {
  using BackendCore::BackendCore;
  std::uint64_t run(std::uint32_t budget);
};

static_assert(!CoreImplementation<WrongRunResult>);
constexpr std::array snapshots{Xmm::x3, Xmm::x4, Xmm::x5, Xmm::x6};
constexpr std::array temporaries{
    Reg::ax, Reg::cx, Reg::dx, Reg::si, Reg::r8, Reg::r9, Reg::r10, Reg::r11};
constexpr Word port = 0xffff80, datum = 0xfedcba;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array data_registers{4U, 5U, 6U, 7U, 14U, 15U};
constexpr std::array io_operations{
    0x44e0d2U, 0x45e0a2U, 0x46e0d2U, 0x47e0b2U, 0x56e018U, 0x57e010U};
constexpr std::array<std::uint8_t, 4> entry_landing{0xf3, 0x0f, 0x1e, 0xfa};
constexpr std::uint8_t return_opcode = 0xc3;

template <unsigned LaneBits>
  requires detail::x64::ScalarTransferLane<LaneBits>
void transfers(const detail::x64::RegisterPair registers, const Xmm snapshot) {
  detail::x64::Code code;
  code.raw(entry_landing);
  code.load(registers.source, Reg::di, offsetof(detail::Cpu, reg) + offsetof(Registers, a));
  code.packed_from<LaneBits>(snapshot, registers.source);
  code.packed_to<LaneBits>(registers.destination, snapshot);
  code.store(Reg::di, offsetof(detail::Cpu, reg) + offsetof(Registers, x), registers.destination);
  code.packed_load_lane<LaneBits>(
      snapshot, Reg::di, offsetof(detail::Cpu, reg) + offsetof(Registers, b));
  code.packed_to<LaneBits>(registers.destination, snapshot);
  code.store(Reg::di, offsetof(detail::Cpu, reg) + offsetof(Registers, y), registers.destination);
  code.raw({return_opcode});
  const native::Direct executable(code.bytes());
  for (unsigned bit = 0; bit < 64; ++bit) {
    for (const auto value : {std::uint64_t{1} << bit, ~(std::uint64_t{1} << bit)}) {
      detail::Cpu cpu;
      cpu.reg.a = value;
      cpu.reg.b = ~value;
      detail::NativeContext context;
      static_cast<void>(executable.enter(cpu, context));
      constexpr auto mask = LaneBits == 64 ? std::numeric_limits<std::uint64_t>::max()
                                           : std::numeric_limits<std::uint32_t>::max();
      require(cpu.reg.x == (value & mask) && cpu.reg.y == (~value & mask) && cpu.reg.a == value &&
                  cpu.reg.b == ~value,
              "SSE2 lanes preserve their width and zero extension across GPRs and memory");
    }
  }
}

void admission() {
  constexpr Word no_snapshot = 0x2000d2;
  constexpr Word load_x_mac = 0x44d8d2;
  constexpr Word load_a_tfr_b = 0x56d809;
  constexpr Word register_x_to_a_tfr_b = 0x208e09;
  for (const Word opcode : {no_snapshot, load_x_mac, load_a_tfr_b, register_x_to_a_tfr_b}) {
    const std::array operations{
        detail::NativeOp{.instruction = detail::decode(opcode), .pc = start}};
    for (const bool optimize : {false, true}) {
      detail::JitEmitter emitter({}, optimize);
      static_cast<void>(emitter.generate(operations));
      require(emitter.cached_inputs() == (optimize && opcode != no_snapshot),
              "SIMD snapshot admission follows overwritten live operands, not application names");
    }
  }
}

template <Backend Kind> void typed_backend() {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  constexpr auto conflicting = Kind == Backend::jit ? Backend::reference : Backend::jit;
  BackendCore<Kind> core({.program = p, .x = x, .y = y}, nullptr, {.backend = conflicting});
  constexpr Word mac = 0x2000d2;
  constexpr Word shift = 0x200022;
  constexpr Word address = 16;
  constexpr Word unmasked = 0xff123456;
  const std::array program{mac};
  core.load(Space::program, start, program);
  core.write(Space::x, address, unmasked & word_mask);
  bool rejected = false;
  try {
    core.write(Space::x, address, unmasked);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && core.read(Space::x, address) == (unmasked & word_mask),
          "Typed cores reject over-wide host words without changing memory");
  for (const auto opcode : {mac, shift}) {
    core.write(Space::program, start, opcode);
    const auto initial = register_model::seed(mode_rm | flag_s | flag_c);
    auto expected = initial;
    sequence_model::alu(expected, initial, opcode);
    ++expected.pc;
    core.set_registers(initial);
    const auto result = core.run(1);
    require(result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                core.registers() == expected && core.progress() == result.progress,
            "Every typed backend shares exact execution and program-mutation contracts");
  }
  const auto statistics = core.jit_statistics();
  require(statistics.native_alu_instructions == (Kind == Backend::jit ? 2U : 0U),
          "The core type selects its implementation even when Options.backend disagrees");
  core.clear_code_cache();
  core.reset();
  require(core.progress() == Progress{} && core.fault().kind == FaultKind::none &&
              core.read(Space::x, address) == (unmasked & word_mask),
          "Typed backend reset clears execution state while retaining owned host memory");
}

class Observer final : public Io {
public:
  Core *core = nullptr;
  Registers expected;
  bool accept = true, invalidate = false, observed = false;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    observed = core != nullptr && space == Space::x && address == port && pc == start + 2 &&
               core->registers() == expected;
    if (observed && accept && invalidate) { core->write(Space::program, start + 3, 0); }
    value = datum;
    asm volatile("pxor %%xmm3, %%xmm3\n\tpcmpeqd %%xmm4, %%xmm4\n\t"
                 "pxor %%xmm5, %%xmm5\n\tpcmpeqd %%xmm6, %%xmm6"
                 :
                 :
                 : "xmm3", "xmm4", "xmm5", "xmm6");
    return observed && accept;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

struct CallbackInput {
  Options options;
  std::size_t operation = 0;
  Word mode = 0;
  unsigned action = 0;
};

void callback(const CallbackInput input) {
  std::array<Word, memory_words> p{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  Observer observer;
  observer.accept = input.action != 2;
  observer.invalidate = input.action == 1;
  Core core({.program = p, .x = x, .y = y}, &observer, input.options);
  observer.core = &core;
  const auto opcode = io_operations.at(input.operation);
  constexpr Word mac_a = 0x2000d2;
  constexpr Word mac_b = 0x2000fa;
  constexpr Word round_b = 0x200019;
  const std::array program{mac_a, mac_b, opcode, round_b};
  core.load(Space::program, start, program);
  auto initial = register_model::seed(input.mode | flag_s | flag_l | flag_c);
  initial.r[0] = port;
  auto expected = initial;
  for (const auto operation : {mac_a, mac_b}) {
    const auto old = expected;
    sequence_model::alu(expected, old, operation);
    ++expected.pc;
  }
  observer.expected = expected;
  ++observer.expected.pc;
  core.set_registers(initial);
  const auto result = core.run(4);
  require(observer.observed, "An I/O helper observes exact state before all six bank overwrites");
  if (!observer.accept) {
    require(result.dispatches == 2 && result.progress.instructions == 2 &&
                result.fault.kind == FaultKind::io_read && result.fault.pc == start + 2,
            "A rejected helper does not consume undefined SIMD snapshots or retire its ALU");
    return;
  }
  auto old = expected;
  register_model::write(expected, data_registers.at(input.operation), datum);
  sequence_model::alu(expected, old, opcode);
  ++expected.pc;
  old = expected;
  sequence_model::alu(expected, old, observer.invalidate ? 0U : round_b);
  ++expected.pc;
  require(result.dispatches == 4 && result.progress.instructions == 4 &&
              result.fault.kind == FaultKind::none && core.registers() == expected,
          "All live banks reload pre-transfer inputs after SIMD-clobbering helpers");
  if (input.options.backend == Backend::jit && input.options.optimize) {
    const auto statistics = core.jit_statistics();
    require(statistics.native_alu_instructions == 4 && statistics.helper_instructions == 0,
            "The snapshot helper test really resumes native ALU execution");
  }
}

void callbacks() {
  for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
    for (const bool optimize : {false, true}) {
      if (backend != Backend::jit && !optimize) { continue; }
      for (std::size_t operation = 0; operation < io_operations.size(); ++operation) {
        for (const auto mode : modes) {
          for (unsigned action = 0; action < 3; ++action) {
            callback({.options = {.backend = backend, .optimize = optimize},
                      .operation = operation,
                      .mode = mode,
                      .action = action});
          }
        }
      }
    }
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const auto source : dsp56300::test::temporaries) {
      for (const auto destination : dsp56300::test::temporaries) {
        for (const auto snapshot : dsp56300::test::snapshots) {
          dsp56300::test::transfers<32>({.destination = destination, .source = source}, snapshot);
          dsp56300::test::transfers<64>({.destination = destination, .source = source}, snapshot);
        }
      }
    }
    dsp56300::test::admission();
    dsp56300::test::typed_backend<dsp56300::Backend::reference>();
    dsp56300::test::typed_backend<dsp56300::Backend::cached>();
    dsp56300::test::typed_backend<dsp56300::Backend::jit>();
    dsp56300::test::callbacks();
    std::cout << dsp56300::test::checks << " native input snapshot checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
