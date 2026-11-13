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
#include "loop_frames.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
using enum isa::Opcode;
constexpr Word entry = 8, rom = 0x4000, high = 0xc000, lf = 1U << 15, forever = 1U << 16;
constexpr Word loads = static_cast<Word>(move_x_effective) | (1U << 15) | (3U << 11);
constexpr Word stores = static_cast<Word>(move_x_effective) | (3U << 11) | (1U << 8);
constexpr std::size_t words = 16;
constexpr Word host_bits = 0xab000000;
constexpr Word y_bus = 1U << 19;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
using Data = std::array<std::array<Word, words>, 3>;

struct Input {
  Word source = rom + 1, destination = high + 2, count = 3, flags = 0;
  std::size_t known_index = words;
  std::uint8_t knowledge = 1;
};

struct Forms {
  Word load = 3, store = 3;
};

struct Grant {
  Word instructions;
  std::uint64_t cycles = std::numeric_limits<std::uint64_t>::max();
};

struct Fixture final : Io {
  explicit Fixture(const bool aliases = false, const Forms forms = {})
      : windows{MemoryWindow{.address = 0, .words = data[0], .known = {}, .writable = true},
                MemoryWindow{.address = rom, .words = data[1], .known = {}},
                MemoryWindow{.address = high,
                             .words = data[2],
                             .known = known,
                             .writable = true,
                             .aliases_program = aliases}},
        load_mode(forms.load),
        store_mode(forms.store) {
    program[entry - 2] = static_cast<Word>(do_immediate) | (3U << 8);
    program[entry - 1] = entry + 1;
    program[entry] = (loads & ~(7U << 11)) | (forms.load << 11);
    program[entry + 1] = (stores & ~(7U << 11)) | (forms.store << 11);
    program[entry + 2] = static_cast<Word>(rts);
    reset();
  }

  void reset(const Input input = {}) {
    known.fill(1);
    if (input.known_index < known.size()) { known[input.known_index] = input.knowledge; }
    for (std::size_t bank = 0; bank < data.size(); ++bank) {
      for (std::size_t index = 0; index < words; ++index) {
        data[bank][index] = host_bits | (static_cast<Word>(bank + 1) << word_bits / 2) |
                            static_cast<Word>(index + 1);
      }
    }
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    return address < program.size() ? std::optional{program[address]} : std::nullopt;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    return space == Space::x ? std::span<const MemoryWindow>(windows)
                             : std::span<const MemoryWindow>{};
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    if (space != Space::program || address >= program.size()) { return false; }
    value = program[address];
    return true;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  [[nodiscard]] detail::JitRuntime runtime() { return {.memory = {}, .windows = {windows, {}}}; }

  std::array<Word, 32> program{};
  Data data{};
  std::array<std::uint8_t, words> known{};
  std::array<MemoryWindow, 3> windows;
  Word load_mode, store_mode;
};

Registers initial(const Input input, const Word mode) {
  auto result = register_model::seed(mode);
  result.pc = entry;
  result.la = entry + 1;
  result.lc = input.count;
  result.sr = mode | lf | input.flags;
  result.r[0] = input.source;
  result.r[1] = input.destination;
  result.m.fill(word_mask);
  result.stack[2] = std::uint64_t{entry} << word_bits;
  return result;
}

struct Model {
  Registers reg;
  Data data;
  std::array<std::uint8_t, words> known;
  Fault failure;
  Word retired = 0;
  std::uint64_t cycles = 0;
};

std::optional<std::pair<std::size_t, std::size_t>> location(const Word address) {
  constexpr std::array bases{0U, rom, high};
  for (std::size_t bank = 0; bank < bases.size(); ++bank) {
    if (address >= bases[bank] && address - bases[bank] < words) {
      return std::pair{bank, std::size_t{address - bases[bank]}};
    }
  }
  return {};
}

Word effective(Registers &reg, const Fixture &fixture, const bool load) {
  const auto mode = load ? fixture.load_mode : fixture.store_mode;
  auto &cursor = reg.r[load ? 0U : 1U];
  if (mode == 7) { cursor = (cursor - 1) & word_mask; }
  const auto address = cursor;
  if (mode == 3) { cursor = (cursor + 1) & word_mask; }
  if (mode == 2) { cursor = (cursor - 1) & word_mask; }
  return address;
}

void step(Model &model, const Fixture &fixture) {
  const auto pc = model.reg.pc;
  const bool load = pc == entry;
  const auto address = effective(model.reg, fixture, load);
  const auto position = location(address);
  bool accepted = position.has_value();
  const auto value = static_cast<Word>(model.reg.x) & word_mask;
  if (position) {
    const auto [bank, index] = *position;
    accepted = load ? bank != 2 || model.known[index] != 0
                    : bank != 1 && (bank != 2 || !fixture.windows[2].aliases_program);
    if (accepted && load) {
      model.reg.x =
          (model.reg.x & ~std::uint64_t{word_mask}) | (model.data[bank][index] & word_mask);
    } else if (accepted) {
      model.data[bank][index] = value;
      if (bank == 2) { model.known[index] = 1; }
    }
  }
  ++model.reg.pc;
  if (!accepted) {
    model.failure = {.kind = load ? FaultKind::memory_read : FaultKind::memory_write,
                     .space = Space::x,
                     .address = address,
                     .value = load ? 0U : value,
                     .pc = pc};
    return;
  }
  ++model.retired;
  model.cycles += (load ? fixture.load_mode : fixture.store_mode) == 7 ? 2U : 1U;
  if (load) { return; }
  if ((model.reg.sr & forever) != 0 || model.reg.lc != 1) {
    model.reg.lc = (model.reg.lc - 1) & word_mask;
    model.reg.pc = entry;
  } else {
    loop_frames::complete(model.reg);
  }
}

Model expected(const Fixture &fixture, const Registers &state, const Grant grant) {
  Model result{.reg = state, .data = fixture.data, .known = fixture.known, .failure = {}};
  for (Word i = 0; i < grant.instructions && result.failure == Fault{}; ++i) {
    if (result.cycles >= grant.cycles) { break; }
    step(result, fixture);
  }
  return result;
}

unsigned fallback_calls = 0;

std::uint32_t fallback(detail::Cpu * /*cpu*/, detail::NativeContext * /*owner*/) noexcept {
  ++fallback_calls;
  return 0;
}

void direct(Fixture &fixture, const Input input, const Word mode) {
  fixture.reset(input);
  const auto seed = initial(input, mode);
  const auto runtime = fixture.runtime();
  const auto operations = detail::parse_block(fixture.program, seed, runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  const auto image =
      detail::compile_region(compiler, operations, runtime, std::bit_cast<const void *>(&fallback));
  if (!image) { throw std::runtime_error("Declared windows must admit a whole copy loop"); }
  for (Word flags = 0; flags <= status_byte; ++flags) {
    fixture.reset(input);
    auto state = seed;
    state.sr |= flags;
    const auto total = input.count != 0 && (input.flags & forever) == 0 ? input.count * 2 : 6U;
    const auto wanted = expected(fixture, state, {.instructions = total});
    auto shifted = state;
    shifted.a <<= detail::padding_bits;
    shifted.b <<= detail::padding_bits;
    detail::Cpu cpu{.reg = shifted};
    detail::NativeContext owner{.runtime = {}, .budget = total, .cycle_deadline = wanted.cycles};
    const auto before = fallback_calls;
    const auto retired = detail::enter_block(image->entry, cpu, owner);
    cpu.reg.a >>= detail::padding_bits;
    cpu.reg.b >>= detail::padding_bits;
    require(retired == total && fallback_calls == before && cpu.reg == wanted.reg &&
                fixture.data == wanted.data && fixture.known == wanted.known &&
                cpu.instructions == total && cpu.cycles == wanted.cycles,
            "LLVM selects actual per-cursor windows and executes their body without fallback");
  }
}

void prefixes(
    Core &core, Fixture &fixture, const Input input, const Word mode, const bool deadline = false) {
  const auto total = input.count != 0 && (input.flags & forever) == 0 ? input.count * 2 : 6U;
  const auto limit =
      deadline
          ? (total / 2) * ((fixture.load_mode == 7 ? 2U : 1U) + (fixture.store_mode == 7 ? 2U : 1U))
          : total;
  for (Word prefix = 0; prefix <= limit; ++prefix) {
    fixture.reset(input);
    const auto state = initial(input, mode);
    const auto wanted = deadline
                            ? expected(fixture, state, {.instructions = total, .cycles = prefix})
                            : expected(fixture, state, {.instructions = prefix});
    const bool restart = core.state().failure != Fault{};
    if (restart) { core.reset(); }
    core.set_registers(state);
    if (restart) {
      constexpr std::array entries{entry};
      require(core.prepare(entries).prepared_entries == 1,
              "Reprepare after an explicit fault reset");
    }
    const auto before = core.progress();
    const auto result =
        deadline ? core.run_until(before.estimated_cycles + prefix) : core.run(prefix);
    if (core.registers() != wanted.reg || result.fault != wanted.failure) {
      std::cerr << "forms/deadline=" << fixture.load_mode << '/' << fixture.store_mode << '/'
                << deadline << " retired/cycles/yield/fault=" << result.retired << '/'
                << result.progress.estimated_cycles - before.estimated_cycles << '/'
                << result.yielded << '/' << static_cast<unsigned>(result.fault.kind)
                << " fault-space/address/pc=" << static_cast<unsigned>(result.fault.space) << '/'
                << result.fault.address << '/' << result.fault.pc
                << " expected retired/cycles=" << wanted.retired << '/' << wanted.cycles << '\n'
                << "source/destination/count/flags/mode/prefix=" << input.source << '/'
                << input.destination << '/' << input.count << '/' << input.flags << '/' << mode
                << '/' << prefix << " actual PC/R0/R1=" << core.registers().pc << '/'
                << core.registers().r[0] << '/' << core.registers().r[1]
                << " expected=" << wanted.reg.pc << '/' << wanted.reg.r[0] << '/' << wanted.reg.r[1]
                << '\n';
    }
    require(result.retired == wanted.retired && result.fault == wanted.failure &&
                core.registers() == wanted.reg && fixture.data == wanted.data &&
                fixture.known == wanted.known &&
                result.progress ==
                    Progress{.instructions = before.instructions + wanted.retired,
                             .estimated_cycles = before.estimated_cycles + wanted.cycles},
            "Window admission and exact prefixes preserve permissions, aliases, frames and data");
  }
}

void cases(const Word mode) {
  Fixture fixture;
  direct(fixture, {}, mode);
  direct(fixture, {.source = high + 1, .destination = 2}, mode);
  direct(fixture, {.source = 1, .destination = 2}, mode);
  direct(
      fixture, {.source = high + 1, .destination = 2, .known_index = 2, .knowledge = 0x80}, mode);
  Core core(Memory::external(), &fixture);
  core.set_registers(initial({}, mode));
  constexpr std::array entries{entry};
  const auto state = core.state();
  const auto data_before = fixture.data;
  const auto known_before = fixture.known;
  const auto prepared = core.prepare(entries);
  const auto again = core.prepare(entries);
  require(prepared.prepared_entries == 1 && prepared.native_bytes > 0 &&
              prepared.failure == Fault{} && again.prepared_entries == 1 &&
              again.native_bytes == 0 && core.registers() == state.registers &&
              core.progress() == state.progress && fixture.data == data_before &&
              fixture.known == known_before,
          "Prepare a windowed loop once without performing any guest execution");
  constexpr std::array inputs{
      Input{},
      Input{.source = rom + 1, .destination = 2},
      Input{.source = high + 1, .destination = 2},
      Input{.source = 1, .destination = 2},
      Input{.destination = rom + 2},
      Input{.source = rom + words - 1},
      Input{.destination = high + words - 1},
      Input{.count = 1},
      Input{.count = 0},
      Input{.flags = forever | status_byte},
      Input{.known_index = 2, .knowledge = 0},
      Input{.known_index = 2, .knowledge = 0x80},
      Input{.source = high + 1, .destination = 2, .known_index = 2, .knowledge = 0},
      Input{.source = high + 1, .destination = 2, .known_index = 2, .knowledge = 0x80}};
  for (const auto input : inputs) { prefixes(core, fixture, input, mode); }
  Fixture aliased(true);
  Core alias_core(Memory::external(), &aliased);
  alias_core.set_registers(initial({}, mode));
  require(alias_core.prepare(entries).prepared_entries == 1,
          "Prepare before selecting the runtime write destination");
  prefixes(alias_core, aliased, {}, mode);
}

void signed_cases(const Word mode) {
  for (const Word from : {2U, 3U, 4U, 7U}) {
    for (const Word to : {2U, 3U, 4U, 7U}) {
      Fixture fixture(false, {.load = from, .store = to});
      constexpr Input middle{.source = rom + 7, .destination = high + 8};
      direct(fixture, middle, mode);
      direct(fixture, {.source = 7, .destination = 8}, mode);
      direct(fixture, {.source = from == 7 ? 1U : 0U, .count = 1}, mode);
      Core core(Memory::external(), &fixture);
      core.set_registers(initial(middle, mode));
      constexpr std::array entries{entry};
      require(core.prepare(entries).prepared_entries == 1,
              "Prepare a signed window loop through the public core API");
      constexpr std::array inputs{
          middle,
          Input{.source = high + 7, .destination = 8},
          Input{.source = 7, .destination = 8},
          Input{.source = rom, .destination = high},
          Input{.source = rom + words - 1, .destination = high + words - 1},
          Input{.source = rom + 7, .destination = high + 8, .count = 1},
          Input{.source = rom + 7, .destination = high + 8, .count = 0},
          Input{.source = rom + 7, .destination = high + 8, .flags = forever | status_byte},
          Input{.source = rom + 7, .destination = high + 8, .known_index = 7, .knowledge = 0},
          Input{.source = rom + 7, .destination = high + 8, .known_index = 7, .knowledge = 0x80},
          Input{.source = rom + 7, .destination = rom + 8}};
      for (const auto input : inputs) {
        prefixes(core, fixture, input, mode);
        prefixes(core, fixture, input, mode, true);
      }
    }
  }
}

void knowledge_alias() {
  Fixture fixture;
  fixture.windows[2].known = {reinterpret_cast<std::uint8_t *>(fixture.data[2].data()), words};
  Core core(Memory::external(), &fixture);
  core.set_registers(initial({}, 0));
  constexpr std::array entries{entry};
  const auto report = core.prepare(entries);
  require(report.failure == Fault{} && report.prepared_entries == 0 && report.native_bytes == 0,
          "A store aliasing knowledge bytes cannot use invariant initialization admission");
}

void cross_space_cursor(const Forms forms) {
  std::array<Word, 32> program{};
  std::array<Word, words> x{};
  std::array<Word, words> y{};
  std::array<std::uint8_t, words> known{};
  known.fill(1);
  for (std::size_t i = 0; i < words; ++i) { x[i] = static_cast<Word>(i + 1); }
  program[entry] = (loads & ~(7U << 11)) | (forms.load << 11);
  program[entry + 1] = (stores & ~((7U << 8) | (7U << 11))) | y_bus | (forms.store << 11);
  const MemoryWindow from{.address = rom, .words = x, .known = {}};
  const MemoryWindow to{.address = rom + 1, .words = y, .known = known, .writable = true};
  const detail::JitRuntime runtime{.memory = {},
                                   .windows = {std::span(&from, 1), std::span(&to, 1)}};
  const auto state = initial({.source = rom + 7}, 0);
  const auto operations = detail::parse_block(program, state, runtime);
  std::unique_ptr<detail::LlvmCompiler> compiler;
  const auto image =
      detail::compile_region(compiler, operations, runtime, std::bit_cast<const void *>(&fallback));
  if (!image) { throw std::runtime_error("One cursor can address both declared Harvard spaces"); }
  auto wanted = state;
  std::array<Word, words> expected_y{};
  for (unsigned iteration = 0; iteration < 3; ++iteration) {
    if (forms.load == 7) { --wanted.r[0]; }
    const auto value = x.at(wanted.r[0] - rom);
    wanted.x = (wanted.x & ~std::uint64_t{word_mask}) | value;
    if (forms.load == 3) { ++wanted.r[0]; }
    if (forms.load == 2) { --wanted.r[0]; }
    if (forms.store == 7) { --wanted.r[0]; }
    expected_y.at(wanted.r[0] - rom - 1) = value;
    if (forms.store == 3) { ++wanted.r[0]; }
    if (forms.store == 2) { --wanted.r[0]; }
  }
  loop_frames::complete(wanted);
  wanted.pc = entry + 2;
  auto shifted = state;
  shifted.a <<= detail::padding_bits;
  shifted.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = shifted};
  const auto cycles = 3 * ((forms.load == 7 ? 2U : 1U) + (forms.store == 7 ? 2U : 1U));
  detail::NativeContext owner{.runtime = {}, .budget = 6, .cycle_deadline = cycles};
  const auto before = fallback_calls;
  const auto retired = detail::enter_block(image->entry, cpu, owner);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(retired == 6 && fallback_calls == before && cpu.reg == wanted && y == expected_y &&
              cpu.instructions == 6 && cpu.cycles == cycles,
          "Shared cursors preserve signed prefix extents, net movement and predecrement costs");
  shifted.m[0] = 3;
  detail::Cpu nonlinear{.reg = shifted};
  const auto rejected_before = fallback_calls;
  const auto actual = detail::enter_block(image->entry, nonlinear, owner);
  if (forms.load == 4 && forms.store == 4) {
    wanted.m[0] = 3;
    nonlinear.reg.a >>= detail::padding_bits;
    nonlinear.reg.b >>= detail::padding_bits;
    require(
        actual == 6 && fallback_calls == rejected_before && nonlinear.reg == wanted &&
            y == expected_y && nonlinear.instructions == 6 && nonlinear.cycles == cycles,
        "Pure indirect transfers preserve exact execution independently of the unused modifier");
    return;
  }
  require(actual == 0 && fallback_calls == rejected_before + 1 && nonlinear.reg == shifted &&
              nonlinear.cycles == 0 && nonlinear.instructions == 0,
          "Nonlinear modifiers retain exact ordinary execution without speculative memory work");
}

const void *link_region(detail::LlvmCompiler &compiler, Fixture &fixture, const Registers &seed) {
  const auto runtime = fixture.runtime();
  const auto operations = detail::parse_block(fixture.program, seed, runtime);
  detail::LlvmModule module("owned_window_region");
  require(detail::lift_region(module, operations, runtime),
          "A complete guarded region must lower independently of native ISA fallback code");
  compiler.define("dsp56300_region_memory",
                  std::bit_cast<const void *>(&detail::dsp56300_region_memory));
  if constexpr (__has_feature(address_sanitizer)) { return compiler.install(module); }
  const auto object = compiler.object(module);
  require(!object.empty(), "A real relocatable object rejects embedded absolute host pointers");
  return compiler.install_object(object, module.name());
}

void linked_owner(const void *entry_point, Fixture &fixture, const Registers &seed) {
  const auto wanted = expected(fixture, seed, {.instructions = 6});
  auto shifted = seed;
  shifted.a <<= detail::padding_bits;
  shifted.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = shifted};
  detail::NativeContext owner{
      .runtime = fixture.runtime(), .budget = 6, .cycle_deadline = wanted.cycles};
  const auto retired = detail::enter_block(entry_point, cpu, owner);
  cpu.reg.a >>= detail::padding_bits;
  cpu.reg.b >>= detail::padding_bits;
  require(retired == 6 && cpu.reg == wanted.reg && cpu.instructions == wanted.retired &&
              cpu.cycles == wanted.cycles && fixture.data == wanted.data &&
              fixture.known == wanted.known && cpu.repeat_remaining == 0 && !cpu.repeat_loaded &&
              !cpu.repeat_completed && !cpu.interrupt_inhibited && cpu.debug_flags == 0 &&
              cpu.processing == ExecutionState::running && owner.failure == Fault{} &&
              owner.boundary == detail::NativeBoundary::context && !owner.program_written,
          "One retained region uses each current owner's RAM and preserves complete copy effects");
}

void linked_miss(const void *entry_point,
                 Fixture &fixture,
                 const Registers &seed,
                 const Grant grant = {.instructions = 6, .cycles = 6}) {
  auto shifted = seed;
  shifted.a <<= detail::padding_bits;
  shifted.b <<= detail::padding_bits;
  detail::Cpu cpu{.reg = shifted};
  detail::NativeContext owner{
      .runtime = fixture.runtime(), .budget = grant.instructions, .cycle_deadline = grant.cycles};
  const auto data = fixture.data;
  const auto known = fixture.known;
  require(detail::enter_block(entry_point, cpu, owner) == 0 && cpu.reg == shifted &&
              cpu.instructions == 0 && cpu.cycles == 0 && fixture.data == data &&
              fixture.known == known && owner.failure == Fault{} &&
              owner.boundary == detail::NativeBoundary::none,
          "A rejected region returns unchanged state and RAM without entering any ISA fallback");
}

void linkable_windows() {
  Fixture original;
  Fixture independent;
  const auto seed = initial({}, 0);
  detail::LlvmCompiler compiler;
  const auto *const region = link_region(compiler, original, seed);
  const auto native_bytes = compiler.native_bytes();
  for (Word flags = 0; flags <= status_byte; ++flags) {
    original.reset();
    independent.reset();
    constexpr Word tag = 0x231;
    for (auto &word : independent.data[1]) { word ^= tag; }
    auto state = seed;
    state.sr |= flags;
    linked_owner(region, original, state);
    linked_owner(region, independent, state);
  }
  independent.reset();
  linked_miss(region, independent, seed, {.instructions = 0, .cycles = 6});
  linked_miss(region, independent, seed, {.instructions = 1, .cycles = 6});
  linked_miss(region, independent, seed, {.instructions = 6, .cycles = 0});
  linked_miss(region, independent, seed, {.instructions = 6, .cycles = 1});
  independent.windows[2].writable = false;
  linked_miss(region, independent, seed);
  independent.windows[2].writable = true;
  independent.windows[2].aliases_program = true;
  linked_miss(region, independent, seed);
  independent.windows[2].aliases_program = false;
  independent.known[2] = 0;
  linked_miss(region, independent, seed);
  independent.known[2] = 2;
  linked_miss(region, independent, seed);
  independent.reset();
  constexpr Word canonical_bytes = 0x01010101;
  independent.data[2].fill(canonical_bytes);
  independent.windows[2].known = {reinterpret_cast<std::uint8_t *>(independent.data[2].data()),
                                  words};
  linked_miss(region, independent, seed);
  require(compiler.native_bytes() == native_bytes,
          "Independent owners and rejected contexts do not compile or materialize more code");
}
}

int main() try {
  linkable_windows();
  for (const auto mode : modes) { cases(mode); }
  for (const auto mode : modes) { signed_cases(mode); }
  knowledge_alias();
  for (const Word from : {2U, 3U, 4U, 7U}) {
    for (const Word to : {2U, 3U, 4U, 7U}) { cross_space_cursor({.load = from, .store = to}); }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
