// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace dsp56300::test {
namespace {
constexpr Word mac_pair_bits = 0x50;
constexpr Word mac = std::to_underlying(isa::Opcode::mac_register_a) | mac_pair_bits;
constexpr Word clear = std::to_underlying(isa::Opcode::clr_a);
constexpr Word negate = std::to_underlying(isa::Opcode::neg_a);
constexpr Word x0_read_r0_fields = 0x00a000;
constexpr Word alu_field_mask = 0xff;
constexpr Word io_load = std::to_underlying(isa::Opcode::move_x_effective) | x0_read_r0_fields;
constexpr Word io_load_clear = io_load | (std::to_underlying(isa::Opcode::clr_a) & alu_field_mask);
constexpr Word io_load_add = io_load | (std::to_underlying(isa::Opcode::add_x0_a) & alu_field_mask);
constexpr Word io_load_subtract =
    io_load | (std::to_underlying(isa::Opcode::sub_x0_a) & alu_field_mask);
constexpr Word io_load_compare =
    io_load | (std::to_underlying(isa::Opcode::cmp_x0_a) & alu_field_mask);
constexpr Word io_load_adc = io_load | (std::to_underlying(isa::Opcode::adc_x_a) & alu_field_mask);
constexpr Word io_load_sbc = io_load | (std::to_underlying(isa::Opcode::sbc_x_a) & alu_field_mask);
constexpr Word io_address = 0xffff80;

struct Machine {
  explicit Machine(const TestOptions options, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, options) {
    core.set_registers(initial_state());
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  TestCore core;
};

void exact_budget() {
  Machine machine({.backend = Backend::jit});
  std::array<Word, 16> program{};
  program.fill(mac);
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = 1;
  state.y = 1;
  machine.core.set_registers(state);
  const auto published = codegen::publications();
  require(machine.core.run(0).retired == 0 && codegen::publications() == published,
          "Zero budget does not compile or execute");
  require(machine.core.run(8).retired == 8 && machine.core.registers().a == 16,
          "A generated block retires exactly its guest budget");
  machine.core.set_registers(state);
  require(machine.core.run(3).retired == 3 && machine.core.registers().a == 6 &&
              machine.core.registers().pc == start + 3,
          "A short tail never enters an oversized cached block");
}

void numeric_modes() {
  Machine machine({.backend = Backend::jit});
  machine.core.write(Space::program, start, mac);
  auto state = initial_state();
  constexpr unsigned extension_bit = 46;
  state.a = std::uint64_t{1} << extension_bit;
  const auto published = codegen::publications();
  machine.core.set_registers(state);
  require(machine.core.run(1).retired == 1 && machine.core.registers().sr == 0,
          "Native flags use the unscaled extension boundary");
  state.sr = mode_up;
  machine.core.set_registers(state);
  require(machine.core.run(1).retired == 1 && machine.core.registers().sr == (mode_up | flag_e),
          "A changed scaling mode selects a separately guarded translation");
  require(codegen::publications().count == published.count + 2,
          "Numeric modes have distinct translation keys");
}

void rounding_modes() {
  Machine machine({.backend = Backend::jit});
  constexpr Word round_accumulator = std::to_underlying(isa::Opcode::rnd_a);
  machine.core.write(Space::program, start, round_accumulator);
  auto state = initial_state();
  state.a = std::uint64_t{1} << (word_bits - 1);
  const auto published = codegen::publications();
  machine.core.set_registers(state);
  require(machine.core.run(1).retired == 1 && machine.core.registers().a == 0,
          "Native convergent rounding sends a half tie to even zero");
  state.sr = mode_rm;
  machine.core.set_registers(state);
  require(machine.core.run(1).retired == 1 &&
              machine.core.registers().a == (std::uint64_t{1} << word_bits),
          "A changed RM selects two's-complement rounding at the same PC");
  require(codegen::publications().count == published.count + 2,
          "Both rounding modes execute their separately keyed native translation");
}

struct NativeSum {
  Word opcode = 0, status = 0;
  bool extended = false;
};

void check_sum(Machine &machine, const NativeSum input) {
  const auto words = input.extended ? 2U : 1U;
  const std::array program{input.opcode, Word{0}};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.sr = input.status | flag_l | flag_s;
  auto expected = state;
  expected.sr |= flag_u | flag_z;
  expected.pc = start + words;
  machine.core.set_registers(state);

  const auto result = machine.core.run(1);
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              machine.core.registers() == expected,
          "All addition forms execute with qualified zero-result flags");
}

void native_sums() {
  Machine machine({.backend = Backend::jit});
  constexpr std::array<Word, 23> opcodes{0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x14,
                                         0x24, 0x34, 0x44, 0x54, 0x64, 0x74, 0x05, 0x45,
                                         0x55, 0x65, 0x75, 0x21, 0x31, 0x25, 0x35};
  constexpr std::array<Word, 6> immediates{std::to_underlying(isa::Opcode::add_immediate_short),
                                           0x014084,
                                           std::to_underlying(isa::Opcode::cmp_immediate_short),
                                           0x0140c0,
                                           0x0140c4,
                                           0x0140c5};
  constexpr Word long_immediate_bit = 0x40;
  for (const Word destination : {Word{0}, Word{8}}) {
    for (const Word mode : {Word{0}, mode_down, mode_up}) {
      for (const auto opcode : opcodes) {
        check_sum(machine, {.opcode = parallel_none | opcode | destination, .status = mode});
      }
      for (const auto opcode : immediates) {
        check_sum(machine,
                  {.opcode = opcode | destination,
                   .status = mode,
                   .extended = (opcode & long_immediate_bit) != 0});
      }
    }
  }
}

void unsupported_sum_modes() {
  constexpr std::array<Word, 5> modes{0x2000, 0x4000, 0x20000, 0x100000, mode_up | mode_down};
  constexpr std::array<Word, 4> opcodes{std::to_underlying(isa::Opcode::add_x0_a),
                                        std::to_underlying(isa::Opcode::sbc_x_a),
                                        std::to_underlying(isa::Opcode::add_immediate_short),
                                        std::to_underlying(isa::Opcode::cmp_immediate_short)};
  for (const auto mode : modes) {
    for (const auto opcode : opcodes) {
      Machine machine({.backend = Backend::jit});
      machine.core.write(Space::program, start, opcode);
      auto state = initial_state();
      state.sr = mode;
      machine.core.set_registers(state);
      const auto result = machine.core.run(1);
      constexpr Word cmp_immediate = std::to_underlying(isa::Opcode::cmp_immediate_short);
      if (mode == saturation_model::mode) {
        auto expected = state;
        auto old = state;
        const bool immediate = opcode < parallel_none;
        if (immediate) { old.x = 0; }
        const Word operation = opcode == cmp_immediate ? 0x45U : 0x40U;
        saturation_model::alu(expected, old, immediate ? operation : opcode);
        ++expected.pc;
        require(result.retired == 1 && result.fault.kind == FaultKind::none &&
                    result.progress == Progress{.instructions = 1, .estimated_cycles = 1} &&
                    machine.core.registers() == expected,
                "SM sums and comparisons execute independently calculated saturation semantics");
        continue;
      }
      require(result.retired == 0 && result.progress == Progress{} &&
                  result.fault.kind == FaultKind::unsupported_mode && result.fault.pc == start,
              "Unsupported addition modes fail through the general guard before retirement");
    }
  }
}

void native_carry_chains() {
  constexpr Word adc_x_a = std::to_underlying(isa::Opcode::adc_x_a);
  constexpr Word adc_y_b = std::to_underlying(isa::Opcode::adc_y_b);
  constexpr Word adc_x_b = std::to_underlying(isa::Opcode::adc_x_b);
  constexpr Word sbc_x_b = std::to_underlying(isa::Opcode::sbc_x_b);
  constexpr Word sbc_y_b = std::to_underlying(isa::Opcode::sbc_y_b);
  constexpr Word sbc_x_a = std::to_underlying(isa::Opcode::sbc_x_a);

  struct Chain {
    std::array<Word, 4> program;
    std::uint64_t initial_a = 0, expected_a = 0, expected_b = 0;
    Word y = 0;
  };

  const std::array chains{
      Chain{.program = {adc_x_a, adc_y_b, sbc_x_b, sbc_y_b}, .initial_a = accumulator_mask, .y = 1},
      Chain{.program = {sbc_x_a, sbc_y_b, adc_x_b, adc_y_b},
            .expected_a = accumulator_mask,
            .expected_b = 2}};
  for (const auto &chain : chains) {
    Machine machine({.backend = Backend::jit});
    machine.core.load(Space::program, start, chain.program);
    auto state = initial_state();
    state.a = chain.initial_a;
    state.x = 1;
    state.y = chain.y;
    state.sr = flag_s | flag_l;
    auto expected = state;
    expected.a = chain.expected_a;
    expected.b = chain.expected_b;
    expected.pc = start + 4;
    expected.sr |= flag_u | (chain.expected_b == 0 ? flag_z : 0);
    machine.core.set_registers(state);
    require(machine.core.run(4).retired == 4 && machine.core.registers() == expected,
            "Native multi-precision chains consume the preceding instruction's Carry/Borrow");

    machine.core.set_registers(state);
    require(machine.core.run(1).retired == 1 && machine.core.run(3).retired == 3 &&
                machine.core.registers() == expected,
            "Partitioned carry chains retain flags across native entries");
    require(machine.core.progress() == Progress{.instructions = 8, .estimated_cycles = 8},
            "Short carry-chain runs remain native with exact retirement accounting");
  }
}

void host_program_write() {
  Machine machine({.backend = Backend::jit});
  machine.core.write(Space::program, start, mac);
  auto state = initial_state();
  state.x = 1;
  state.y = 1;
  machine.core.set_registers(state);
  require(machine.core.run(1).retired == 1 && machine.core.registers().a == 2,
          "Native code is populated");
  machine.core.write(Space::program, start, clear);
  machine.core.set_registers(state);
  require(machine.core.run(1).retired == 1 && machine.core.registers().a == 0,
          "Host program writes discard the previous native operation");

  machine.core.reset();
}

class ProgramWriter final : public Io {
public:
  TestCore *core = nullptr;
  unsigned reads = 0;
  bool reentry_rejected = false;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (space != Space::x || address != io_address || pc != start || core == nullptr) {
      return false;
    }
    ++reads;
    try {
      static_cast<void>(core->run(0));
    } catch (const std::logic_error &) { reentry_rejected = true; }
    core->write(Space::program, start, negate);
    core->write(Space::program, start + 1, 0);
    value = 1;
    return true;
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

void callback_program_write(const Word opcode) {
  ProgramWriter io;
  Machine machine({.backend = Backend::jit}, &io);
  io.core = &machine.core;
  const std::array program{opcode, mac};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.r[0] = io_address;
  state.a = 3;
  state.y = 1;
  machine.core.set_registers(state);
  const auto result = machine.core.run(2);
  require(result.retired == 2 && result.fault.kind == FaultKind::none && io.reads == 1,
          "An I/O callback may invalidate code currently executing");
  require(io.reentry_rejected, "Callbacks cannot reenter the execution engine");
  const auto actual = machine.core.registers();
  require(actual.a == (opcode == io_load_clear ? 0U : 3U) && actual.pc == start + 2 &&
              actual.x == 1,
          "The fetched operation completes and the modified successor is fetched afresh");
}

void memory_fault() {
  for (const auto opcode :
       {io_load_clear, io_load_add, io_load_subtract, io_load_compare, io_load_adc, io_load_sbc}) {
    Machine machine({.backend = Backend::jit});
    machine.core.write(Space::program, start, opcode);
    auto state = initial_state();
    state.r[0] = memory_words;
    machine.core.set_registers(state);
    const auto result = machine.core.run(1);
    require(result.retired == 0 && result.fault.kind == FaultKind::memory_read &&
                result.fault.address == memory_words && result.fault.pc == start,
            "The instrumented move helper retains span checks before native retirement");
  }
}

void sum_loop_boundary() {
  Machine machine({.backend = Backend::jit});
  constexpr Word do_three = 0x060380;
  constexpr Word add_x0 = std::to_underlying(isa::Opcode::add_x0_a);
  const std::array program{do_three, start + 3, add_x0, add_x0};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = 1;
  state.la = 7;
  state.lc = 9;
  machine.core.set_registers(state);
  require(machine.core.run(7).retired == 7, "DO and six additions retire");
  const auto actual = machine.core.registers();
  require(actual.a == (std::uint64_t{6} << word_bits) && actual.sr == flag_u &&
              actual.la == state.la && actual.lc == state.lc && actual.sp == 0 && actual.sc == 0 &&
              actual.pc == start + 4,
          "Native addition preserves DO cleanup and full-precision result flags");
}

void sum_repeat() {
  Machine machine({.backend = Backend::jit});
  constexpr Word repeat_count_bits = 4U << 8;
  constexpr Word repeat_four = std::to_underlying(isa::Opcode::rep_immediate) | repeat_count_bits;
  constexpr Word adc_x = std::to_underlying(isa::Opcode::adc_x_a);
  const std::array program{repeat_four, adc_x};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = 1;
  state.lc = 7;
  state.sr = flag_c;
  machine.core.set_registers(state);
  require(machine.core.run(5).retired == 5 && machine.core.registers().a == 5 &&
              machine.core.registers().sr == flag_u && machine.core.registers().lc == state.lc &&
              machine.core.registers().pc == start + 2 && machine.core.interrupt_ready(),
          "Repeated ADC reevaluates Carry and restores LC across the REP continuation");
}

void interrupt_callback_boundary() {
  for (const auto backend : {Backend::jit}) {
    ProgramWriter io;
    Machine machine({.backend = backend}, &io);
    io.core = &machine.core;
    const std::array program{io_load_clear, Word{0}};
    machine.core.load(Space::program, start, program);
    auto state = initial_state();
    state.r[0] = io_address;
    state.pc = start + 4;
    machine.core.set_registers(state);
    machine.core.interrupt(start, 0);
    require(io.reads == 1 && io.reentry_rejected && machine.core.registers().pc == state.pc,
            "Interrupt callbacks obey the same stopped-boundary contract");
    require(machine.core.run(0).retired == 0,
            "Interrupt entry releases its execution guard on return");
  }
}

void loop_boundary() {
  Machine machine({.backend = Backend::jit});
  constexpr Word do_three = 0x060380;
  const std::array program{do_three, start + 3, mac, mac};
  machine.core.load(Space::program, start, program);
  auto state = initial_state();
  state.x = 1;
  state.y = 1;
  constexpr Word old_loop_address = 0x70;
  state.la = old_loop_address;
  state.lc = 7;
  machine.core.set_registers(state);
  require(machine.core.run(7).retired == 7, "DO and six body operations retire");
  const auto actual = machine.core.registers();
  require(actual.a == 12 && actual.la == state.la && actual.lc == state.lc &&
              actual.pc == start + 4 && actual.sp == 0 && actual.sc == 0,
          "Native execution stops before loop cleanup and preserves loop restoration");
}

void loop_cleanup_fault() {
  constexpr Word loop_flag = Word{1} << 15;
  for (const auto backend : {Backend::jit}) {
    Machine machine({.backend = backend});
    machine.core.write(Space::program, start, mac);
    auto state = initial_state();
    state.la = start;
    state.lc = 1;
    state.sr = loop_flag;
    machine.core.set_registers(state);
    const auto result = machine.core.run(1);
    require(result.retired == 0 && result.progress == Progress{} &&
                result.fault.kind == FaultKind::stack,
            "Failed loop cleanup does not retire or consume instruction base cycles");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::exact_budget();
    dsp56300::test::numeric_modes();
    dsp56300::test::rounding_modes();
    dsp56300::test::native_sums();
    dsp56300::test::unsupported_sum_modes();
    dsp56300::test::native_carry_chains();
    dsp56300::test::host_program_write();
    dsp56300::test::callback_program_write(dsp56300::test::io_load_clear);
    dsp56300::test::callback_program_write(dsp56300::test::io_load);
    dsp56300::test::callback_program_write(dsp56300::test::io_load_add);
    dsp56300::test::callback_program_write(dsp56300::test::io_load_subtract);
    dsp56300::test::callback_program_write(dsp56300::test::io_load_compare);
    dsp56300::test::memory_fault();
    dsp56300::test::interrupt_callback_boundary();
    dsp56300::test::loop_boundary();
    dsp56300::test::sum_loop_boundary();
    dsp56300::test::sum_repeat();
    dsp56300::test::loop_cleanup_fault();
    std::cout << dsp56300::test::checks << " native boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
