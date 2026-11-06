// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit_emitter.hpp"
#include "native/direct.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"
#include "translation.hpp"
#include "x64.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word loop_flag = Word{1} << 15, port_address = 0xffff80, datum = 0xfedcba;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::uint64_t boundary32 = std::uint64_t{1} << 32;
constexpr std::uint64_t boundary63 = std::uint64_t{1} << 63;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();

struct Step {
  Word opcode = 0, extension = 0;
  bool immediate = false;
};

constexpr Step mac_a{.opcode = 0x2000d3}, mac_b{.opcode = 0x2000fb};
constexpr Step immediate{.opcode = 0x0140c0, .extension = 0xff1234, .immediate = true};
constexpr Step round_a{.opcode = 0x200011}, io_mac{.opcode = 0x44e0d3};
constexpr Step asr_a{.opcode = 0x200022}, asr_b{.opcode = 0x20002a};

Word words(const Step step) {
  return step.immediate ? 2U : 1U;
}

Word ending(const std::span<const Step> steps, const Word first) {
  auto next = first;
  for (const auto step : steps) { next += words(step); }
  return (next - 1) & word_mask;
}

std::vector<detail::NativeOp>
operations(const std::span<const Step> steps, const Word mode, const Word first, const bool loop) {
  std::vector<detail::NativeOp> result;
  auto pc = first;
  for (const auto step : steps) {
    auto decoded = detail::decode(step.opcode);
    decoded.extension = step.extension;
    require(decoded.words == words(step) && decoded.cycles == words(step),
            "Compiler input matches the independently specified word/base-cost shape");
    result.push_back({.instruction = decoded,
                      .pc = pc,
                      .mode = mode,
                      .loop_address = loop ? ending(steps, first) : detail::inactive_loop});
    pc = (pc + words(step)) & word_mask;
  }
  return result;
}

void arithmetic(Registers &state, const Step step) {
  auto old = state;
  if (step.immediate) { old.x = step.extension; }
  sequence_model::alu(state, old, step.immediate ? 0x40U : step.opcode);
  state.pc = (state.pc + words(step)) & word_mask;
}

struct Expected {
  Registers state;
  Progress progress{};
  Word retired = 0;
};

Expected model(const std::span<const Step> steps, const Registers &initial, const Word budget) {
  Expected result{.state = initial};
  if (steps.empty()) { return result; }
  const bool loop = (initial.sr & loop_flag) != 0;
  while (budget - result.retired >= steps.size()) {
    for (std::size_t index = 0; index < steps.size(); ++index) {
      const bool last = loop && index + 1 == steps.size();
      if (last && result.state.lc <= 1) { return result; }
      arithmetic(result.state, steps[index]);
      if (last) {
        --result.state.lc;
        result.state.pc =
            static_cast<Word>(result.state.stack[result.state.sp % stack_entries] >> word_bits);
      }
      ++result.retired;
      ++result.progress.instructions;
      result.progress.estimated_cycles += words(steps[index]);
    }
    if (!loop || result.state.pc != initial.pc) { return result; }
  }
  return result;
}

std::uint64_t modulo_sum(const std::uint64_t before, const std::uint64_t increment) {
  const auto room = maximum - before;
  return increment <= room ? before + increment : increment - room - 1;
}

struct Input {
  Progress progress;
  Word first = start, target = start, lc = 0, sp = 2, budget = 0;
};

void execute_case(const native::Direct &code,
                  const std::span<const Step> steps,
                  const Word mode,
                  const bool loop,
                  const Input input) {
  auto initial = register_model::seed(mode | flag_l | flag_s | flag_c | (loop ? loop_flag : 0));
  initial.pc = input.first;
  initial.la = ending(steps, input.first);
  initial.lc = input.lc;
  initial.sp = input.sp;
  initial.sc = 0;
  initial.stack[input.sp % stack_entries] = (std::uint64_t{input.target} << word_bits) | 3;
  const auto expected = model(steps, initial, input.budget);
  detail::Cpu cpu;
  cpu.reg = initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  cpu.instructions = input.progress.instructions;
  cpu.cycles = input.progress.estimated_cycles;
  cpu.repeat_completed = cpu.interrupt_inhibited = true;
  detail::NativeContext context;
  context.budget = input.budget;
  const auto retired = code.enter(cpu, context);
  auto actual = cpu.reg;
  actual.a >>= detail::padding_bits;
  actual.b >>= detail::padding_bits;
  require(retired == expected.retired && actual == expected.state,
          "Packed/scalar exits preserve every register, wrap, frame and partial-loop prefix");
  require(cpu.instructions ==
                  modulo_sum(input.progress.instructions, expected.progress.instructions) &&
              cpu.cycles ==
                  modulo_sum(input.progress.estimated_cycles, expected.progress.estimated_cycles),
          "Independent qword lanes retain carries, wrap and mixed one/two-cycle instruction costs");
  require(cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0) &&
              cpu.repeat_remaining == 0 && !cpu.repeat_loaded &&
              cpu.processing == ExecutionState::running && cpu.debug_flags == 0,
          "Zero-retirement exits retain exclusion state; successful prefixes clear it");
}

void generated(const std::span<const Step> steps, const Word mode, const bool loop) {
  constexpr std::array seeds{
      Progress{},
      Progress{.instructions = boundary32 - 2, .estimated_cycles = boundary63 - 3},
      Progress{.instructions = maximum - 2, .estimated_cycles = boundary32 - 1},
      Progress{.instructions = boundary63 - 1, .estimated_cycles = maximum - 3}};
  for (const auto first : {start, word_mask - 1}) {
    const auto ops = operations(steps, mode, first, loop);
    detail::JitEmitter emitter({});
    const auto bytes = emitter.generate(ops);
    const bool expected_admission = !steps.empty() && (loop || steps.size() >= 4);
    require(emitter.packed_retirement() == expected_admission,
            "Only the specified straight-run or native-loop admission selects packed retirement");
    require(emitter.cached_status() == expected_admission,
            "Status holding shares the qualified native-only admission");
    const native::Direct code(bytes);
    const auto size = static_cast<Word>(steps.size());
    for (const auto progress : seeds) {
      for (const auto lc : {0U, 1U, 2U, 7U}) {
        for (const auto sp : {0U, 15U, 31U}) {
          for (const auto target : {first, (first + 7) & word_mask}) {
            for (const auto budget : {size, size * 3}) {
              execute_case(code,
                           steps,
                           mode,
                           loop,
                           {.progress = progress,
                            .first = first,
                            .target = target,
                            .lc = lc,
                            .sp = sp,
                            .budget = budget});
            }
          }
        }
      }
    }
  }
}

void counter_lanes() {
  const std::array long_body{mac_a, mac_b, immediate, mac_a, round_a, mac_b, immediate, mac_a};
  const std::array mixed{immediate, mac_a, immediate, mac_b};
  const std::array shifts{asr_a, Step{.opcode = 0x200021}, asr_b, Step{.opcode = 0x200035}};
  for (const auto mode : modes) {
    generated({}, mode, false);
    for (const auto size : {std::size_t{1}, std::size_t{3}, std::size_t{4}, long_body.size()}) {
      for (const bool loop : {false, true}) {
        generated(std::span(long_body).first(size), mode, loop);
      }
    }
    generated(mixed, mode, false);
    generated(mixed, mode, true);
    generated(std::span(shifts).first(1), mode, false);
    generated(std::span(shifts).first(1), mode, true);
    generated(shifts, mode, false);
    generated(shifts, mode, true);
  }
}

void admission_guards() {
  constexpr Word shift = 0x200032;
  constexpr Word aliased_store = 0x4460d3;
  const std::array body{mac_a, mac_b, mac_a, mac_b};
  const auto ops = operations(body, 0, start, false);
  const auto check = [](const std::span<const detail::NativeOp> input,
                        const detail::JitRuntime runtime) {
    detail::JitEmitter emitter(runtime);
    static_cast<void>(emitter.generate(input));
    require(!emitter.packed_retirement(),
            "General helpers, unsupported modes, invalid costs and aliased stores reject packing");
    require(!emitter.cached_status(), "Rejected native regions retain canonical status in memory");
  };
  auto changed = ops;
  changed[1].instruction = detail::decode(shift); // General ASL helper.
  check(changed, {});
  changed = ops;
  for (auto &operation : changed) { operation.mode = detail::arithmetic_16; }
  check(changed, {});
  changed = ops;
  changed[1].instruction.cycles = 0;
  check(changed, {});
  changed = ops;
  for (auto &operation : changed) { operation.instruction = detail::decode(aliased_store); }
  detail::JitRuntime runtime;
  runtime.memory[0].aliases_program = true;
  check(changed, runtime);
}

void encodings() {
  using detail::x64::Packed;
  using detail::x64::Reg;
  using detail::x64::Xmm;
  detail::x64::Code code;
  constexpr unsigned qword_sign = 63;
  code.packed(Packed::bit_xor, Xmm::x0, Xmm::x0);
  code.packed(Packed::equal_dword, Xmm::x1, Xmm::x1);
  code.packed_shift_right(Xmm::x1, qword_sign);
  code.packed(Packed::add_qword, Xmm::x0, Xmm::x1);
  code.packed_load(Xmm::x2, Reg::r12, offsetof(detail::Cpu, instructions));
  code.packed(Packed::add_qword, Xmm::x2, Xmm::x0);
  code.packed_store(Reg::r12, offsetof(detail::Cpu, instructions), Xmm::x2);
  code.packed_from(Xmm::x0, Reg::r15);
  code.packed(Packed::unpack_low_qword, Xmm::x0, Xmm::x0);
  // Independent literal encodings; also checked with LLVM MC during authoring.
  const std::vector<std::uint8_t> expected{
      0x66, 0x0f, 0xef, 0xc0, 0x66, 0x0f, 0x76, 0xc9, 0x66, 0x0f, 0x73, 0xd1, 0x3f,
      0x66, 0x0f, 0xd4, 0xc1, 0xf3, 0x41, 0x0f, 0x6f, 0x94, 0x24, 0x28, 0x01, 0x00,
      0x00, 0x66, 0x0f, 0xd4, 0xd0, 0xf3, 0x41, 0x0f, 0x7f, 0x94, 0x24, 0x28, 0x01,
      0x00, 0x00, 0x66, 0x41, 0x0f, 0x6e, 0xc7, 0x66, 0x0f, 0x6c, 0xc0};
  require(code.bytes() == expected,
          "SSE2 qword/unaligned encodings match the independent byte record");
}

void low32_packing() {
  using detail::x64::Packed;
  using detail::x64::Reg;
  using detail::x64::Width;
  using detail::x64::Xmm;
  constexpr std::uint32_t sign32 = 1U << 31;
  constexpr auto maximum32 = std::numeric_limits<std::uint32_t>::max();
  constexpr std::uint64_t high_marker = 0x1234567800000000;
  constexpr Word adjacent_marker = 123;
  constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
  constexpr std::array<std::uint8_t, 2> save{0x41, 0x57};
  constexpr std::array<std::uint8_t, 3> restore{0x41, 0x5f, 0xc3};
  for (const auto value : {0U, 1U, sign32, maximum32}) {
    detail::x64::Code code;
    code.raw(landing); // ENDBR64.
    code.raw(save);    // Preserve the caller's R15.
    code.constant(Reg::r15, high_marker | value);
    code.packed_from(Xmm::x0, Reg::r15);
    code.packed(Packed::unpack_low_qword, Xmm::x0, Xmm::x0);
    code.packed_store(Reg::di, offsetof(detail::Cpu, instructions), Xmm::x0);
    code.mov({.destination = Reg::ax, .source = Reg::r15}, Width::dword);
    code.raw(restore); // Restore R15 and return.
    const native::Direct executable(code.bytes());
    detail::Cpu cpu;
    const auto registers = register_model::seed(0);
    cpu.reg = registers;
    cpu.repeat_remaining = adjacent_marker;
    detail::NativeContext context;
    require(
        executable.enter(cpu, context) == value && cpu.instructions == value && cpu.cycles == value,
        "MOVD zero-extends every count sign boundary before duplication into independent qwords");
    require(cpu.reg == registers && cpu.repeat_remaining == adjacent_marker,
            "Unaligned packed counter stores preserve both neighboring architectural fields");
  }
}

void packed_copies() {
  using detail::x64::Code;
  using detail::x64::Packed;
  using detail::x64::Reg;
  using detail::x64::Xmm;
  constexpr std::array registers{Xmm::x0, Xmm::x1, Xmm::x2, Xmm::x3, Xmm::x4, Xmm::x5, Xmm::x6};
  constexpr std::uint32_t arithmetic_mask = 0x8d5;
  constexpr std::uint64_t high = 0x89abcdef01234567;
  constexpr std::array<std::uint8_t, 4> landing{0xf3, 0x0f, 0x1e, 0xfa};
  constexpr std::array<std::uint8_t, 1> push_flags{0x68};
  constexpr std::array<std::uint8_t, 1> pop_flags{0x9d};
  constexpr std::array<std::uint8_t, 2> capture{0x9c, 0x58};
  constexpr std::array<std::uint8_t, 1> returned{0xc3};
  for (const auto source : registers) {
    for (const auto destination : registers) {
      Code encoded;
      encoded.packed(Packed::copy, destination, source);
      const auto modrm =
          0xc0U | (static_cast<unsigned>(destination) << 3) | static_cast<unsigned>(source);
      const std::vector<std::uint8_t> expected_encoding{
          0x66, 0x0f, 0x6f, static_cast<std::uint8_t>(modrm)};
      require(encoded.bytes() == expected_encoding,
              "SSE2 register copies match independent MOVDQA literal encodings");
      for (const auto incoming : {0U, arithmetic_mask}) {
        Code code;
        code.raw(landing);
        code.packed_load(source, Reg::di, offsetof(detail::Cpu, instructions));
        if (source != destination) { code.packed(Packed::bit_xor, destination, destination); }
        code.raw(push_flags); // PUSH imm32, POPFQ: defined user arithmetic flags only.
        code.u32(incoming | 2U);
        code.raw(pop_flags);
        code.packed(Packed::copy, destination, source);
        code.raw(capture); // PUSHFQ, POP RAX, before any flag-changing instruction.
        code.immediate(4, Reg::ax, arithmetic_mask);
        code.store(Reg::di, offsetof(detail::Cpu, reg) + offsetof(Registers, x), Reg::ax);
        code.packed_store(Reg::di, offsetof(detail::Cpu, instructions), destination);
        code.packed_to(Reg::ax, source);
        code.store(Reg::di, offsetof(detail::Cpu, reg) + offsetof(Registers, y), Reg::ax);
        code.constant(Reg::ax, 0);
        code.raw(returned);
        const native::Direct executable(code.bytes());
        detail::Cpu cpu;
        auto expected = register_model::seed(0);
        cpu.reg = expected;
        cpu.instructions = maximum;
        cpu.cycles = high;
        expected.x = incoming;
        expected.y = maximum;
        detail::NativeContext context;
        require(executable.enter(cpu, context) == 0 && cpu.instructions == maximum &&
                    cpu.cycles == high && cpu.reg == expected,
                "Every snapshot/cache permutation preserves both qword lanes, source and "
                "arithmetic flags");
      }
    }
  }
}

struct Snapshot {
  Registers state;
  Progress progress;
  Word pc = 0;
};

class Observer final : public Io {
public:
  std::array<Snapshot, 2> expected;
  TestCore *core = nullptr;
  Word patch_pc = 0;
  int reject = -1;
  unsigned reads = 0;
  bool valid = true, invalidate = false;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    const auto index = reads++;
    auto *const cpu = core;
    if (cpu == nullptr || index >= expected.size()) {
      valid = false;
      return false;
    }
    const auto &snapshot = expected[index];
    valid = valid && space == Space::x && address == port_address && pc == snapshot.pc &&
            cpu->registers() == snapshot.state && cpu->progress() == snapshot.progress;
    const bool accepted = reject != static_cast<int>(index);
    if (accepted) {
      value = datum;
      if (index == 0 && invalidate) { cpu->write(Space::program, patch_pc, 0); }
    }
    // Deliberately destroy the caller-saved SIMD temporaries before returning.
    asm volatile("pcmpeqd %%xmm0, %%xmm0\n\tpxor %%xmm1, %%xmm1\n\tpcmpeqd %%xmm2, %%xmm2\n\t"
                 "pxor %%xmm3, %%xmm3\n\tpcmpeqd %%xmm4, %%xmm4\n\tpxor %%xmm5, %%xmm5\n\t"
                 "pcmpeqd %%xmm6, %%xmm6"
                 :
                 :
                 : "xmm0", "xmm1", "xmm2", "xmm3", "xmm4", "xmm5", "xmm6");
    return valid && accepted;
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

struct Machine {
  explicit Machine(Io *io) : core({.program = p, .x = x, .y = y}, io, {.backend = Backend::jit}) {}

  std::array<Word, memory_words> p{}, x{}, y{};
  TestCore core;
};

Expected callback_expectations(Observer &observer, const Registers &initial, const Progress base) {
  std::array body{
      mac_a, mac_b, immediate, mac_a, asr_a, io_mac, mac_b, asr_b, io_mac, round_a, mac_b};
  Expected result{.state = initial, .progress = base};
  unsigned access = 0;
  for (std::size_t index = 0; index < body.size(); ++index) {
    const auto step = body[index];
    const bool io = step.opcode == io_mac.opcode;
    auto old = result.state;
    if (io) {
      auto observed = result.state;
      ++observed.pc;
      observer.expected[access++] = {
          .state = observed, .progress = result.progress, .pc = result.state.pc};
      register_model::write(result.state, 4, datum);
    }
    if (step.immediate) { old.x = step.extension; }
    sequence_model::alu(result.state, old, step.immediate ? 0x40U : step.opcode);
    result.state.pc += words(step);
    ++result.retired;
    ++result.progress.instructions;
    result.progress.estimated_cycles += words(step);
    if (io && access == 1 && observer.invalidate) { body[index + 1] = {}; }
  }
  return result;
}

struct CallbackInput {
  Word mode = 0;
  int reject = -1;
  bool invalidate = false;
};

void callback_case(const CallbackInput input) {
  Observer observer;
  Machine machine(&observer);
  observer.core = &machine.core;
  observer.reject = input.reject;
  observer.invalidate = input.invalidate;
  const std::array body{
      mac_a, mac_b, immediate, mac_a, asr_a, io_mac, mac_b, asr_b, io_mac, round_a, mac_b};
  Word cursor = start;
  for (const auto step : body) {
    machine.core.write(Space::program, cursor++, step.opcode);
    if (step.immediate) { machine.core.write(Space::program, cursor++, step.extension); }
  }
  auto initial = register_model::seed(input.mode | flag_l | flag_s | flag_c);
  initial.r[0] = 7;
  machine.x[7] = datum;
  machine.core.set_registers(initial);
  const auto count = static_cast<Word>(body.size());
  require(machine.core.run(count).retired == count && observer.reads == 0,
          "Warm the complete native RAM path before reusing its I/O slow branches");
  initial.r[0] = port_address;
  machine.core.set_registers(initial);
  const auto base = machine.core.progress();
  const auto expected = callback_expectations(observer, initial, base);
  observer.patch_pc = observer.expected[0].pc + 1;
  const auto result = machine.core.run(count);
  require(observer.valid &&
              observer.reads == (input.reject < 0 ? 2U : static_cast<unsigned>(input.reject) + 1),
          "Every I/O callback sees canonical PC/registers and both precise progress lanes");
  if (input.reject >= 0) {
    const auto &stopped = observer.expected[static_cast<std::size_t>(input.reject)];
    const auto state = machine.core.registers();
    require(result.fault.kind == FaultKind::io_read && result.fault.pc == stopped.pc &&
                result.progress == stopped.progress &&
                result.retired == stopped.progress.instructions - base.instructions &&
                state.a == stopped.state.a && state.b == stopped.state.b &&
                state.sr == stopped.state.sr,
            "A rejected slow move retains prior arithmetic/progress and cannot retire its ALU");
    require(machine.core.run(count).retired == 0 && machine.core.progress() == stopped.progress,
            "Faulted SIMD/helper boundaries cannot replay work or invent retirement");
  } else {
    require(result.fault.kind == FaultKind::none && result.retired == count &&
                result.progress == expected.progress && machine.core.registers() == expected.state,
            "SIMD clobbers and optional P invalidation preserve every completed result and count");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::encodings();
    dsp56300::test::low32_packing();
    dsp56300::test::packed_copies();
    dsp56300::test::counter_lanes();
    dsp56300::test::admission_guards();
    for (const auto mode : dsp56300::test::modes) {
      for (const int reject : {-1, 0, 1}) {
        for (const bool invalidate : {false, true}) {
          dsp56300::test::callback_case({.mode = mode, .reject = reject, .invalidate = invalidate});
        }
      }
    }
    std::cout << dsp56300::test::checks << " packed-retirement/PC/callback checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
