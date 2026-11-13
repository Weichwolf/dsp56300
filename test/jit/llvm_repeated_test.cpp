// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/region_memory.hpp"
#include "jit/llvm/runtime.hpp"
#include "saturated_shift_model.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word entry = 16, length = 7, stop = entry + length, base = 0x4000;
constexpr Word half = 32, words = half * 2, linear = 0xffff, ring = 0x8007;
constexpr Word descriptor = 0, target = 24, output = 32, control = 48, host_bits = 0xab000000;
constexpr Word jump_cycles = 3, turn_cycles = length + jump_cycles - 1, stop_cycles = 10;
constexpr Word peripheral_first = 0xffff80, device_port = peripheral_first, device_word = 0x654321;
constexpr Word high_register_mask = 0x18, high_register_shift = 17;
constexpr Word r1_register = 17, r3_register = 19;
constexpr Word shift_left_a = 0x32, move_x0_r0 = 0x209000;
constexpr Word move_x0_n0 = 0x209800;
constexpr Word data_pattern = 0x123000, rebased_cursor = 16, origin_data = 0x123456, outer_la = 300;
constexpr std::uint64_t initial_x = 0x123456654321, initial_y = 0x654321123456;
constexpr std::uint64_t initial_a = 0x123456789abcde, initial_b = 0x654321abcdef01;
constexpr std::uint64_t stack_pattern = 0x123456001100;
constexpr std::uint64_t max_progress = std::numeric_limits<std::uint64_t>::max();
constexpr std::array modes{0U,
                           test::mode_down,
                           test::mode_up,
                           test::mode_rm,
                           test::mode_rm | test::mode_down,
                           test::mode_rm | test::mode_up};
using Data = std::array<Word, words>;
using Known = std::array<std::uint8_t, words>;

enum class Variation : std::uint8_t {
  ordinary,
  missing_source,
  missing_derived,
  unmapped_derived,
  missing_late,
  noncanonical,
  alias_pointer,
  alias_data,
  exit,
  unmapped_write,
  io_after_turns
};

enum class Branch : std::uint8_t { reg, short_address, absolute };

struct Case {
  Word mode = 0;
  Word flags = test::status_byte;
  unsigned condition = 16;
  Branch branch = Branch::reg;
  Word jump_target = entry;
  bool rings = false, reset_origin = false;
  bool arithmetic = true, tracked = true;
  bool stride_updates = false, negative_stride = false;
  Variation variation = Variation::ordinary;
};

constexpr bool extended(const Case input) {
  return input.branch == Branch::absolute;
}

constexpr Word stop_at(const Case input) {
  return stop + (extended(input) ? 1U : 0U);
}

constexpr Word tail_cost(const Case input) {
  return input.condition < 16 || extended(input) ? 4U : jump_cycles;
}

constexpr Word body_cost(const Case input) {
  return length + tail_cost(input) - 1;
}

bool selected(const Registers &reg, const unsigned condition) {
  const bool c = (reg.sr & 1U) != 0;
  const bool v = (reg.sr & 2U) != 0;
  const bool z = (reg.sr & 4U) != 0;
  const bool n = (reg.sr & 8U) != 0;
  const bool u = (reg.sr & 16U) != 0;
  const bool e = (reg.sr & 32U) != 0;
  const bool l = (reg.sr & 64U) != 0;
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
  return condition >= truth.size() || truth[condition];
}

Word branch_word(const Case input) {
  if (input.branch == Branch::short_address) {
    return input.condition < 16
               ? static_cast<Word>(isa::Opcode::jcc_absolute_short) | (input.condition << 12) |
                     input.jump_target
               : static_cast<Word>(isa::Opcode::jmp_absolute_short) | input.jump_target;
  }
  const auto opcode = input.condition < 16
                          ? static_cast<Word>(isa::Opcode::jcc_effective) | input.condition
                          : static_cast<Word>(isa::Opcode::jmp_effective);
  return opcode | (extended(input) ? 6U << 11 : (4U << 11) | (3U << 8));
}

struct Grant {
  Word quota = std::numeric_limits<Word>::max();
  std::uint64_t deadline = max_progress, initial_instructions = 0, initial_cycles = 0;
};

Word move(const unsigned reg, const unsigned root, const unsigned mode, const bool load) {
  constexpr Word fields = (7U << 16) | (high_register_mask << high_register_shift);
  return (static_cast<Word>(isa::Opcode::move_x_effective) & ~fields) | ((reg & 7U) << 16) |
         ((reg & high_register_mask) << high_register_shift) | (root << 8) | (mode << 11) |
         (load ? 1U << 15 : 0U);
}

struct Fixture final : Io {
  explicit Fixture(const Case input) : input(input) {
    program.fill(static_cast<Word>(isa::Opcode::nop));
    program[entry] = move(r1_register, 0, input.stride_updates ? 1U : 3U, true);
    program[entry + 1] = move(5, 0, 4, true) | (input.arithmetic ? shift_left_a : 0U);
    program[entry + 2] = move(4, 1, 3, true);
    program[entry + 3] = move(4, 2, 3, false);
    if (input.reset_origin) { program[entry + 4] = move_x0_r0; }
    if (input.stride_updates) { program[entry + 4] = move_x0_n0; }
    program[entry + 5] = move(r3_register, 4, 3, true);
    program[entry + 6] = branch_word(input);
    if (extended(input)) { program[entry + 7] = input.jump_target; }
    program[stop_at(input)] = static_cast<Word>(isa::Opcode::stop);
    known.fill(1);
    for (Word index = 0; index < words; ++index) {
      data[index] = host_bits | (data_pattern + index);
    }
    for (Word index = 0; index < 8; ++index) {
      data[index] = host_bits | (base + target + ((index * 3) & 7));
      data[control + index] = host_bits | input.jump_target;
    }
    for (Word index = 0; index < 16; ++index) {
      data[control + index] = host_bits | input.jump_target;
    }
    if (input.reset_origin) {
      data[0] = base + target;
      data[16] = base + target;
      data[rebased_cursor + 1] = origin_data;
      data[target] = base + 16;
    }
    if (input.stride_updates) {
      for (Word index = 0; index < 8; ++index) {
        data[target + index] = input.negative_stride ? word_mask - (index & 1U) : 2U + (index & 3U);
      }
    }
    switch (input.variation) {
      case Variation::io_after_turns: data[2] = device_port; break;
      case Variation::missing_source: known[2] = 0; break;
      case Variation::missing_derived: known[target + 6] = 0; break;
      case Variation::unmapped_derived: data[2] = base + words; break;
      case Variation::missing_late: known[control + 2] = 0; break;
      case Variation::noncanonical: known[output + 2] = 0x80; break;
      case Variation::exit: data[control + 3] = stop_at(input); break;
      case Variation::ordinary:
      case Variation::alias_pointer:
      case Variation::alias_data:
      case Variation::unmapped_write: break;
      default: throw std::logic_error("Invalid authored case");
    }
    views = {MemoryWindow{.address = base,
                          .words = std::span(data).first(half),
                          .known = std::span(known).first(half),
                          .writable = true},
             MemoryWindow{.address = base + half,
                          .words = std::span(data).subspan(half),
                          .known = std::span(known).subspan(half),
                          .writable = true}};
    if (!input.tracked) {
      for (auto &view : views) { view.known = {}; }
    }
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word at) const noexcept override {
    return at < program.size() ? std::optional{program[at]} : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    return space == Space::x ? std::span<const MemoryWindow>(views)
                             : std::span<const MemoryWindow>{};
  }

  bool read(const Space space, const Word at, Word /*pc*/, Word &value) noexcept override {
    observe();
    if (input.variation != Variation::io_after_turns || space != Space::x || at != device_port) {
      return false;
    }
    value = device_word;
    if (owner != nullptr) { owner->request_yield(); }
    return true;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    observe();
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    observe();
    return false;
  }

  void observe() noexcept {
    ++callbacks;
    if (owner != nullptr) {
      observation = owner->state();
      observed_data = data;
    }
  }

  detail::JitRuntime runtime() { return {.memory = {}, .windows = {views, {}}}; }

  Case input;
  std::array<Word, 32> program{};
  Data data{};
  Known known{};
  std::array<MemoryWindow, 2> views;
  Core *owner = nullptr;
  std::optional<State> observation;
  Data observed_data{};
  unsigned callbacks = 0;
};

Registers seed(const Case input) {
  auto state = test::initial_state();
  state.pc = entry;
  state.sr = input.mode | input.flags;
  state.x = initial_x;
  state.y = initial_y;
  state.a = initial_a;
  state.b = initial_b;
  state.r[0] = base + descriptor;
  if (input.stride_updates) { state.n[0] = input.negative_stride ? word_mask : 1U; }
  state.r[1] = base + target;
  Word destination = output;
  if (input.variation == Variation::alias_pointer) { destination = 1; }
  if (input.variation == Variation::alias_data) { destination = target + 1; }
  if (input.variation == Variation::unmapped_write) { destination = words - 2; }
  state.r[2] = base + destination;
  state.r[3] = entry;
  state.r[4] = base + control;
  state.la = outer_la;
  state.lc = 9;
  state.sp = 14;
  state.sc = 2;
  for (unsigned index = 0; index < state.stack.size(); ++index) {
    state.stack[index] = stack_pattern + index;
  }
  if (input.rings) { state.m[0] = state.m[1] = state.m[2] = state.m[4] = ring; }
  return state;
}

struct Expected {
  Registers reg;
  Data data;
  Known known;
  Fault failure{};
  bool yielded = false;
  std::optional<State> observation = std::nullopt;
  Data observed_data{};
  Word retired = 0;
  std::uint64_t cycles = 0;
  ExecutionState state = ExecutionState::running;
};

Word post(Registers &reg, const unsigned root, const Word step = 1) {
  const auto at = reg.r[root];
  reg.r[root] =
      (reg.m[root] & linear) == linear ? (at + step) & word_mask : (at & ~7U) | ((at + step) & 7U);
  return at;
}

void observe(Expected &model, const bool guarded) {
  if (guarded) { return; }
  model.observation =
      State{.registers = model.reg,
            .failure = {},
            .execution = model.state,
            .progress = {.instructions = model.retired, .estimated_cycles = model.cycles}};
  model.observed_data = model.data;
}

bool read_word(Expected &model, const Case input, const Word at, Word &value, const bool guarded) {
  if (input.variation == Variation::io_after_turns && at == device_port && !guarded) {
    observe(model, guarded);
    value = device_word;
    model.yielded = true;
    return true;
  }
  if (at < base || at - base >= words || (input.tracked && model.known[at - base] == 0)) {
    observe(model, guarded);
    model.failure = {.kind = at >= peripheral_first ? FaultKind::io_read : FaultKind::memory_read,
                     .space = Space::x,
                     .address = at,
                     .pc = model.reg.pc - 1};
    return false;
  }
  value = model.data[at - base] & word_mask;
  return true;
}

bool store_word(Expected &model, const Case input, const bool guarded) {
  const auto at = post(model.reg, 2);
  const auto value = static_cast<Word>(model.reg.x) & word_mask;
  if (at < base || at - base >= words) {
    observe(model, guarded);
    model.failure = {.kind = at >= peripheral_first ? FaultKind::io_write : FaultKind::memory_write,
                     .space = Space::x,
                     .address = at,
                     .value = value,
                     .pc = model.reg.pc - 1};
    return false;
  }
  if (guarded && input.tracked && model.known[at - base] != 1) { return false; }
  model.data[at - base] = value;
  if (input.tracked) { model.known[at - base] = 1; }
  return true;
}

bool instruction(Expected &model, const Case input, const bool guarded) {
  const auto old = model.reg;
  const auto pc = model.reg.pc++;
  if (pc == stop_at(input)) {
    model.state = ExecutionState::stopped;
    return true;
  }
  Word value = 0;
  switch (pc - entry) {
    case 0:
      return read_word(model,
                       input,
                       post(model.reg, 0, input.stride_updates ? model.reg.n[0] : 1U),
                       model.reg.r[1],
                       guarded);
    case 1:
      if (!read_word(model, input, model.reg.r[0], value, guarded)) { return false; }
      model.reg.x = (model.reg.x & word_mask) | (std::uint64_t{value} << test::word_bits);
      if (input.arithmetic) { test::shift_model::apply(model.reg, old, {.left = true}); }
      return true;
    case 2:
      if (!read_word(model, input, post(model.reg, 1), value, guarded)) { return false; }
      model.reg.x = (model.reg.x & ~std::uint64_t{word_mask}) | value;
      return true;
    case 3: return store_word(model, input, guarded);
    case 4:
      if (input.stride_updates) { model.reg.n[0] = static_cast<Word>(model.reg.x) & word_mask; }
      if (input.reset_origin) { model.reg.r[0] = static_cast<Word>(model.reg.x) & word_mask; }
      return true;
    case 5: return read_word(model, input, post(model.reg, 4), model.reg.r[3], guarded);
    case 6:
      if (extended(input)) { ++model.reg.pc; }
      if (selected(model.reg, input.condition)) {
        model.reg.pc = input.branch == Branch::reg ? model.reg.r[3] : input.jump_target;
      }
      return true;
    default: throw std::logic_error("Invalid authored instruction address");
  }
}

bool complete_turn(Expected &model, const Case input) {
  for (Word index = 0; index < length; ++index) {
    if (!instruction(model, input, true)) { return false; }
  }
  return true;
}

Word cost(const Case input, const Word pc) {
  if (pc == stop_at(input)) { return stop_cycles; }
  return pc == entry + length - 1 ? tail_cost(input) : 1U;
}

Expected complete_turns(const Fixture &fixture, const Grant grant) {
  Expected result{.reg = seed(fixture.input), .data = fixture.data, .known = fixture.known};
  if (grant.initial_cycles >= grant.deadline) { return result; }
  while (grant.quota - result.retired >= length &&
         grant.deadline - grant.initial_cycles - result.cycles >= body_cost(fixture.input) &&
         max_progress - grant.initial_instructions - result.retired >= length) {
    auto next = result;
    if (!complete_turn(next, fixture.input)) { break; }
    next.retired += length;
    next.cycles += body_cost(fixture.input);
    result = next;
    if (result.reg.pc != entry) { break; }
  }
  return result;
}

Expected prefix(const Fixture &fixture, const Grant grant) {
  Expected result{.reg = seed(fixture.input), .data = fixture.data, .known = fixture.known};
  while (result.retired < grant.quota && result.cycles < grant.deadline &&
         result.state == ExecutionState::running && !result.yielded) {
    const auto pc = result.reg.pc;
    if (!instruction(result, fixture.input, false)) { break; }
    ++result.retired;
    result.cycles += cost(fixture.input, pc);
  }
  return result;
}

const void *region(detail::LlvmCompiler &compiler, Fixture &fixture, const std::string &name) {
  const auto runtime = fixture.runtime();
  const auto operations = detail::parse_block(fixture.program, seed(fixture.input), runtime);
  require(operations.size() == length, "Authored complete loop has exactly seven instructions");
  detail::LlvmModule module(name);
  require(detail::lift_region(module, operations, runtime),
          "Complete authored loop has guarded SSA");
  compiler.define("dsp56300_region_memory",
                  std::bit_cast<const void *>(&detail::dsp56300_region_memory));
  return compiler.install(module);
}

void direct(const void *code, const Case input, const Grant grant) {
  Fixture fixture(input);
  const auto wanted = complete_turns(fixture, grant);
  auto reg = seed(input);
  reg.a <<= detail::padding_bits;
  reg.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = reg,
                  .instructions = grant.initial_instructions,
                  .cycles = grant.initial_cycles,
                  .repeat_completed = true,
                  .interrupt_inhibited = true};
  detail::NativeContext context{
      .runtime = fixture.runtime(), .budget = grant.quota, .cycle_deadline = grant.deadline};
  const auto retired = detail::enter_block(code, cpu, context);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  if (retired != wanted.retired || cpu.reg != wanted.reg || fixture.data != wanted.data ||
      fixture.known != wanted.known ||
      cpu.instructions != grant.initial_instructions + wanted.retired ||
      cpu.cycles != grant.initial_cycles + wanted.cycles) {
    std::cerr << "DIRECT mode=" << input.mode << " rings=" << input.rings
              << " origin=" << input.reset_origin
              << " variant=" << static_cast<unsigned>(input.variation) << " quota=" << grant.quota
              << " deadline=" << grant.deadline << " retired=" << retired << '/' << wanted.retired
              << '\n';
    throw std::runtime_error("Direct repeated SSA differs from the independently executed model");
  }
  require(cpu.repeat_completed == (retired == 0) && cpu.interrupt_inhibited == (retired == 0) &&
              cpu.processing == ExecutionState::running && context.failure == Fault{} &&
              context.boundary == detail::NativeBoundary::none && fixture.callbacks == 0,
          "Guard misses commit only prior complete iterations without callbacks or stale flags");
}

NativeImage image(Fixture &fixture) {
  const std::array code{
      CodeSpan{.address = entry,
               .words = std::span(fixture.program)
                            .subspan(entry, length + 1 + (extended(fixture.input) ? 1U : 0U))}};
  const std::array entries{entry, stop_at(fixture.input)};
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
  const std::array layout{
      RegionLayout{
          .space = Space::x, .first = base, .size = half, .tracked = fixture.input.tracked},
      RegionLayout{
          .space = Space::x, .first = base + half, .size = half, .tracked = fixture.input.tracked}};
  const std::array layouts{VariantLayout{.id = 0, .memory = layout}};
  return Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
}

void public_prefix(const NativeImage &code, const Case input, const Grant grant, const bool timed) {
  Fixture fixture(input);
  const auto wanted = prefix(fixture, grant);
  Core core(Memory::external(), &fixture);
  fixture.owner = &core;
  core.set_registers(seed(input));
  core.install(code, 0);
  const auto result = timed ? core.run_until(grant.deadline) : core.run(grant.quota);
  if (result.retired != wanted.retired || result.fault != wanted.failure ||
      core.registers() != wanted.reg || fixture.data != wanted.data ||
      fixture.known != wanted.known || result.state != wanted.state ||
      result.progress !=
          Progress{.instructions = wanted.retired, .estimated_cycles = wanted.cycles}) {
    std::cerr << "PUBLIC mode=" << input.mode << " rings=" << input.rings
              << " origin=" << input.reset_origin
              << " variant=" << static_cast<unsigned>(input.variation) << " quota=" << grant.quota
              << " deadline=" << grant.deadline << " retired=" << result.retired << '/'
              << wanted.retired << " cycles=" << result.progress.estimated_cycles << '/'
              << wanted.cycles << " fault kind/address=" << static_cast<unsigned>(result.fault.kind)
              << '/' << result.fault.address
              << " expected=" << static_cast<unsigned>(wanted.failure.kind) << '/'
              << wanted.failure.address << " stride=" << input.stride_updates
              << " negative=" << input.negative_stride << '\n';
    throw std::runtime_error("Public repeated execution differs from the independent prefix model");
  }
  require(result.yielded == wanted.yielded && fixture.callbacks == (wanted.observation ? 1U : 0U) &&
              fixture.observation.has_value() == wanted.observation.has_value(),
          "Only the original access is visible and its real yield is retained");
  if (fixture.observation && wanted.observation) {
    require(
        fixture.observation->registers == wanted.observation->registers &&
            fixture.observation->failure == wanted.observation->failure &&
            fixture.observed_data == wanted.observed_data &&
            fixture.observation->progress == wanted.observation->progress,
        "The callback sees exactly committed iterations and the actual original partial prefix");
  }
}

void branches(const Branch form) {
  unsigned checked = 0;
  for (unsigned condition = 0; condition <= 16; ++condition) {
    for (const bool exits : {false, true}) {
      Case input{.condition = condition, .branch = form, .arithmetic = false};
      if (exits) { input.jump_target = stop_at(input); }
      Fixture source(input);
      const auto compiled = image(source);
      detail::LlvmCompiler compiler;
      const auto *native = region(compiler, source, "authored_conditional_loop");
      for (const Word flags : {0U, test::status_byte, 8U, 16U}) {
        input.flags = flags;
        for (Word bound = 0; bound <= 3 * body_cost(input) + 2; ++bound) {
          const Grant quota{.quota = bound};
          const Grant deadline{.deadline = bound};
          direct(native, input, quota);
          direct(native, input, deadline);
          public_prefix(compiled, input, quota, false);
          public_prefix(compiled, input, deadline, true);
          checked += 4;
        }
      }
    }
  }
  std::cout << "independent branch models passed: " << checked << " prefix checks\n";
}

void conditional_faults(const Branch form) {
  Case input{.condition = 3, .branch = form, .variation = Variation::missing_derived};
  Fixture source(input);
  const auto compiled = image(source);
  detail::LlvmCompiler compiler;
  const auto *native = region(compiler, source, "authored_changed_condition_fault");
  const auto complete = prefix(source, {.quota = 100});
  require(
      complete.retired == 2 * length + 2 && complete.failure.pc == entry + 2,
      "The authored third-turn fault occurs after shifting N and before its conditional branch");
  for (const bool rings : {false, true}) {
    input.rings = rings;
    for (Word bound = 0; bound <= 4 * body_cost(input) + 2; ++bound) {
      const Grant quota{.quota = bound};
      const Grant deadline{.deadline = bound};
      direct(native, input, quota);
      direct(native, input, deadline);
      public_prefix(compiled, input, quota, false);
      public_prefix(compiled, input, deadline, true);
    }
  }
  std::cout << "conditional guard-miss rollback passed\n";
}

unsigned profile_cases(const NativeImage &compiled, const Case profile) {
  constexpr std::array variations{Variation::ordinary,
                                  Variation::missing_source,
                                  Variation::missing_derived,
                                  Variation::unmapped_derived,
                                  Variation::missing_late,
                                  Variation::noncanonical,
                                  Variation::alias_pointer,
                                  Variation::alias_data,
                                  Variation::exit,
                                  Variation::unmapped_write,
                                  Variation::io_after_turns};
  unsigned checked = 0;
  Fixture source(profile);
  detail::LlvmCompiler compiler;
  const auto *native = region(compiler, source, "authored_profile");
  for (const bool rings : {false, true}) {
    for (const auto variation : variations) {
      auto input = profile;
      input.rings = rings;
      input.variation = variation;
      for (Word bound = 0; bound <= 8 * turn_cycles; ++bound) {
        const Grant quota{.quota = bound};
        const Grant deadline{.deadline = bound};
        direct(native, input, quota);
        direct(native, input, deadline);
        public_prefix(compiled, input, quota, false);
        public_prefix(compiled, input, deadline, true);
        checked += 4;
      }
    }
  }
  for (const std::uint64_t room : {0U, 6U, 7U, 14U, 15U}) {
    direct(native, profile, {.quota = 3 * length, .initial_instructions = max_progress - room});
    direct(native,
           profile,
           {.quota = 3 * length,
            .deadline = max_progress,
            .initial_instructions = max_progress - room,
            .initial_cycles = max_progress - stop_cycles});
  }
  direct(native, profile, {.quota = 3 * length, .deadline = 1, .initial_cycles = 2});
  return checked;
}

void strides() {
  constexpr std::array variations{Variation::ordinary,
                                  Variation::missing_late,
                                  Variation::noncanonical,
                                  Variation::alias_pointer,
                                  Variation::exit};
  unsigned checked = 0;
  for (const bool tracked : {false, true}) {
    Fixture source({.tracked = tracked, .stride_updates = true});
    const auto compiled = image(source);
    for (const auto mode : modes) {
      Fixture region_source({.mode = mode, .tracked = tracked, .stride_updates = true});
      detail::LlvmCompiler compiler;
      const auto *native = region(compiler, region_source, "authored_mutable_stride");
      for (const bool negative : {false, true}) {
        for (const bool rings : {false, true}) {
          for (const auto variation : variations) {
            const Case input{.mode = mode,
                             .rings = rings,
                             .tracked = tracked,
                             .stride_updates = true,
                             .negative_stride = negative,
                             .variation = variation};
            for (Word bound = 0; bound <= 4 * turn_cycles; ++bound) {
              const Grant quota{.quota = bound};
              const Grant deadline{.deadline = bound};
              direct(native, input, quota);
              direct(native, input, deadline);
              public_prefix(compiled, input, quota, false);
              public_prefix(compiled, input, deadline, true);
              checked += 4;
            }
          }
        }
      }
    }
  }
  std::cout << "mutable stride models passed: " << checked << " prefix checks\n";
}

void cases() {
  unsigned checked = 0;
  for (const bool tracked : {false, true}) {
    for (const bool origin : {false, true}) {
      Fixture source({.reset_origin = origin, .tracked = tracked});
      const auto compiled = image(source);
      for (const auto mode : modes) {
        checked +=
            profile_cases(compiled, {.mode = mode, .reset_origin = origin, .tracked = tracked});
      }
    }
  }
  std::cout << "independent repeated models passed: " << checked
            << " prefix checks plus counter limits\n";
}

}

int main(const int argc, char **argv) try {
  const std::string group = argc == 1 ? "all" : argv[1];
  constexpr std::array names{"branches-reg", "branches-short", "branches-absolute"};
  require(argc <= 2 && (group == "all" || group == "cases" || group == "branches" ||
                        group == names[0] || group == names[1] || group == names[2]),
          "Unknown repeated-region test group");
  if (group == "all" || group == "cases") {
    cases();
    strides();
  }
  for (const auto form : {Branch::reg, Branch::short_address, Branch::absolute}) {
    if (group == "all" || group == "branches" || group == names[static_cast<unsigned>(form)]) {
      branches(form);
      conditional_faults(form);
    }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
