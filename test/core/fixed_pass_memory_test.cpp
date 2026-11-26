// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fields.hpp"
#include "isa.hpp"
#include "jit/fixed_pass.hpp"
#include "jit/fixed_pass_memory.hpp"
#include "jit/image_plan.hpp"
#include "reject_io.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#if defined(__APPLE__)
#include <sys/fcntl.h>
#else
#include <fcntl.h>
#endif
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {
using dsp56300::CodeSpan;
using dsp56300::CodeVariant;
using dsp56300::CompileOptions;
using dsp56300::Core;
using dsp56300::Fault;
using dsp56300::FaultKind;
using dsp56300::FixedPassDeclaration;
using dsp56300::Memory;
using dsp56300::MemoryWindow;
using dsp56300::RegionLayout;
using dsp56300::Registers;
using dsp56300::Space;
using dsp56300::StableRead;
using dsp56300::VariantLayout;
using dsp56300::Word;
using dsp56300::detail::capture_pass_memory;
using dsp56300::detail::fixed_pass_effects;
using dsp56300::detail::FixedPassCost;
using dsp56300::detail::PassCell;
using dsp56300::detail::PassEffects;
using dsp56300::detail::PassRead;
using dsp56300::detail::PassWindows;
using dsp56300::detail::PassWord;
using dsp56300::detail::plan_image;
using dsp56300::detail::same_pass_memory;
using dsp56300::test::require;

constexpr Word head = 0x100;
constexpr Word dynamic_cursor = 0x3c;
constexpr Word ram_flag = 6;
constexpr Word ram_initial = 0x23;
constexpr Word ram_modified = 0x42;
constexpr Word reset = 0x84;
constexpr Word bit_address_mask = 0x3f00;
constexpr unsigned bit_address_shift = 8;
constexpr Word from_x_to_r1 = 0x61bc00;
constexpr Word indexed_read = 0x44d900;
constexpr Word indexed_write = 0x445900;
constexpr Word set_x6_bit4 = 0x0a0624;
constexpr Word clear_x6_bit4 = 0x0a0604;
constexpr Word jump_head = 0x0c0000 | head;
constexpr Word to_a_from_port = 0x084e2f;
constexpr Word to_port_from_a = 0x08ce2f;
constexpr PassCell device{.space = Space::x, .address = 0xffffef};
constexpr Word port_sample = 0x20, observed_port_bits = 0x800020;
constexpr std::size_t data_words = 64;
constexpr std::array fixture{
    from_x_to_r1, indexed_read, set_x6_bit4, clear_x6_bit4, to_a_from_port, jump_head};
constexpr std::array path{head, head + 1, head + 2, head + 3, head + 4, head + 5};

class StablePort final : public dsp56300::test::RejectIo {
public:
  explicit StablePort(const Word read_pc = head + 4) noexcept : read_pc_(read_pc) {}

  void until(const std::uint64_t cycle) noexcept { horizon_ = cycle; }

  void allow_internal_events_until(const std::uint64_t cycle) noexcept { extended_ = cycle; }

  void enable_sample() noexcept { sample_available_ = true; }

  void set_value(const Word word) noexcept { value_ = word; }

  void check_execution_guard(Core &core) noexcept { probe_ = &core; }

  [[nodiscard]] bool rejected_nested_execution() const noexcept { return rejected_; }

  [[nodiscard]] bool stable_read_value(const StableRead port, Word &value) const noexcept override {
    if (!sample_available_ || port != device) { return false; }
    value = value_;
    return true;
  }

  [[nodiscard]] std::uint64_t
  stable_reads_until(const std::span<const StableRead> ports) const noexcept override {
    return ports.size() == 1 && ports.front() == device ? horizon_ : 0;
  }

  [[nodiscard]] std::uint64_t
  stable_masked_reads_until(const std::span<const StableRead> ports,
                            const std::span<const Word> masks,
                            const std::span<const StableRead> stores) const noexcept override {
    if (probe_ != nullptr) {
      try {
        static_cast<void>(probe_->run(0));
      } catch (const std::logic_error &) { rejected_ = true; }
    }
    return ports.size() == 1 && ports.front() == device && masks.size() == 1 &&
                   masks.front() == observed_port_bits && stores.empty()
               ? extended_
               : 0;
  }

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (space != device.space || address != device.address || pc != read_pc_) { return false; }
    ++reads;
    value = value_;
    return true;
  }

private:
  std::uint64_t horizon_ = 0;
  std::uint64_t extended_ = 0;
  const Word read_pc_;
  Word value_ = port_sample;
  Core *probe_ = nullptr;
  mutable bool rejected_ = false;
  bool sample_available_ = false;
};

auto image(const std::span<const Word> words) {
  return plan_image(std::array{CodeSpan{.address = head, .words = words}}, std::array{head});
}

void masked_device_read() {
  constexpr Word logical_and = static_cast<Word>(dsp56300::isa::Opcode::and_immediate_long);
  constexpr std::array words{to_a_from_port, logical_and, port_sample, jump_head};
  static constexpr std::array masked_path{head, head + 1, head + 3};
  const auto compiled = fixed_pass_effects(image(words), masked_path, std::array{device});
  if (!compiled) { throw std::runtime_error("The masked device pass must compile"); }
  require(compiled->device_masks == std::vector<Word>{observed_port_bits},
          "AND drops unused device bits while retaining a full accumulator load's sign");

  const std::array code{CodeSpan{.address = head, .words = words}};
  constexpr std::array entries{head};
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
  constexpr std::array layout{
      RegionLayout{.space = Space::program, .first = 0, .size = head + words.size()},
      RegionLayout{.space = Space::x, .first = 0, .size = data_words},
      RegionLayout{.space = Space::y, .first = 0, .size = data_words}};
  const std::array memory{VariantLayout{.id = 0, .memory = layout}};
  static constexpr std::array ports{device};
  constexpr std::array passes{
      FixedPassDeclaration{.variant = 0, .path = masked_path, .device_reads = ports}};
  const auto native = Core::compile({.variants = variants},
                                    {.memory = memory,
                                     .interrupts = {},
                                     .transitions = {},
                                     .callbacks = {},
                                     .bindings = {},
                                     .fixed_passes = passes});

  std::array<Word, head + words.size()> program{};
  std::array<Word, data_words> x{};
  std::array<Word, data_words> y{};
  StablePort port(head);
  port.enable_sample();
  port.until(std::numeric_limits<std::uint64_t>::max());
  Core core(Memory::separate({.program = program, .x = x, .y = y}).with_canonical_data(), &port);
  core.load(Space::program, head, words);
  core.set_registers(Registers{.pc = head});
  core.install(native, 0);

  constexpr auto batch_size = static_cast<std::uint32_t>(masked_path.size() * 4);
  const auto warm = core.run(batch_size);
  require(warm.fault == Fault{} && warm.retired == batch_size,
          "Initial device sample finishes the logical AND sequence");
  const auto before = core.registers();
  const auto read_calls = port.reads;
  port.set_value(port_sample | 1U);
  const auto unchanged = core.run(batch_size);
  require(unchanged.fault == Fault{} && unchanged.retired == batch_size &&
              core.registers() == before && port.reads == read_calls,
          "A changed unused device bit keeps the proof reusable");

  const auto local_deadline = unchanged.progress.estimated_cycles + (3 * compiled->cost.cycles);
  port.allow_internal_events_until(local_deadline + (7 * compiled->cost.cycles));
  port.until(local_deadline);
  const auto bounded = core.run_until(local_deadline);
  require(bounded.fault == Fault{} && !bounded.yielded &&
              bounded.retired == 3 * compiled->cost.instructions &&
              bounded.progress.estimated_cycles == local_deadline && core.registers() == before &&
              port.reads == read_calls,
          "A proven recurrence ends at the normal run deadline");

  const auto expected_cycle = local_deadline + (4 * compiled->cost.cycles);
  port.until(local_deadline + 1);
  port.check_execution_guard(core);
  const auto skipped = core.run_until(
      {.base_cycle_deadline = expected_cycle + 1, .precise_deadline = local_deadline + 1});
  require(port.rejected_nested_execution(), "A retained pass rejects reentry from its I/O grant");
  require(skipped.fault == Fault{} && skipped.retired == batch_size &&
              skipped.progress.estimated_cycles == expected_cycle && core.registers() == before &&
              port.reads == read_calls,
          "An extended grant retires complete passes before its own limit");
  port.set_value(port_sample | 0x800000U);
  port.allow_internal_events_until(expected_cycle + (5 * compiled->cost.cycles));
  const auto stale =
      core.run_until({.base_cycle_deadline = expected_cycle + (5 * compiled->cost.cycles),
                      .precise_deadline = expected_cycle});
  require(stale.retired == 0 && stale.progress == skipped.progress && port.reads == read_calls,
          "A changed observed bit rejects a retained pass at the precise deadline");
  port.until(std::numeric_limits<std::uint64_t>::max());
  const auto changed = core.run(batch_size);
  require(changed.fault == Fault{} && changed.retired == batch_size &&
              core.registers().a != before.a && port.reads > read_calls,
          "A changed sign bit invalidates the proof and updates the accumulator extension");

  core.install(native, 0);
  const auto handoff_start = core.progress();
  const auto handoff_registers = core.registers();
  const auto event_deadline = handoff_start.estimated_cycles + (7 * compiled->cost.cycles) + 1;
  const auto precise_deadline = handoff_start.estimated_cycles + (3 * compiled->cost.cycles);
  port.until(precise_deadline);
  port.allow_internal_events_until(event_deadline);
  const auto handed_off =
      core.run_until({.base_cycle_deadline = event_deadline, .precise_deadline = precise_deadline});
  require(handed_off.fault == Fault{} && !handed_off.yielded &&
              handed_off.retired == 7 * compiled->cost.instructions &&
              handed_off.progress.estimated_cycles == event_deadline - 1 &&
              core.registers() == handoff_registers,
          "A pass proved during execution crosses the local event in the same call");
}

void decoded_effects() {
  const auto plan = image(fixture);
  const auto compiled = fixed_pass_effects(plan, path, std::array{device});
  if (!compiled) { throw std::runtime_error("Expected a proven fixed pass"); }
  const auto &effects = *compiled;
  require(effects.cost == FixedPassCost{.instructions = 6, .cycles = 10},
          "Instruction costs come from the actual decoded pass");
  require(effects.linear == 2 && effects.writes.size() == 1 &&
              effects.writes.front() == PassCell{.space = Space::x, .address = ram_flag},
          "Postincrement proves M1 linear and bit changes use one ordinary RAM destination");
  const PassRead indirect{
      .space = Space::x,
      .address = {.offset = 0, .base = PassCell{.space = Space::x, .address = dynamic_cursor}}};
  require(effects.reads.size() == 3 && std::ranges::contains(effects.reads, indirect),
          "Indirect read retains the exact tracked RAM pointer source");
  require(effects.device_reads.size() == 1 && effects.device_reads.front() == device,
          "The validated device read matches its complete bank and address");
  require(!fixed_pass_effects(plan, path, {}), "Undeclared I/O read is refused");
  require(!fixed_pass_effects(
              plan, path, std::array{PassCell{.space = Space::y, .address = device.address}}),
          "A declaration for the other bank cannot admit a device read");
}

void rejects_effects() {
  constexpr std::array short_path{head, head + 1, head + 2, head + 3};
  require(
      !fixed_pass_effects(
          image(std::array{from_x_to_r1, indexed_write, set_x6_bit4, jump_head}), short_path, {}),
      "Dynamically addressed RAM write is outside the finite write set");
  require(!fixed_pass_effects(
              image(std::array{to_port_from_a, 0U, 0U, jump_head}), short_path, std::array{device}),
          "A stable read declaration never admits an output transaction");
  require(!fixed_pass_effects(image(std::array{indexed_read, 0U, 0U, jump_head}), short_path, {}),
          "Unbound dynamic read rejects the candidate");
  constexpr Word change_cursor =
      (set_x6_bit4 & ~bit_address_mask) | (dynamic_cursor << bit_address_shift);
  require(
      !fixed_pass_effects(
          image(std::array{from_x_to_r1, indexed_read, change_cursor, jump_head}), short_path, {}),
      "A RAM write to an address source prevents preflight of its dynamic access");
  require(!fixed_pass_effects(image(std::array{reset, 0U, 0U, jump_head}), short_path, {}),
          "Peripheral reset remains an observable transaction");
}

struct Fixture {
  PassEffects effects{
      .cost = {.instructions = 4, .cycles = 7},
      .reads = {{.space = Space::x,
                 .address = {.offset = 0, .base = PassCell{.space = Space::x, .address = 0}}}},
      .writes = {{.space = Space::y, .address = 0}},
      .device_reads = {},
      .device_masks = {},
      .linear = 2};
  std::array<Word, 4> x{1, 3, 4, 5};
  std::array<Word, 1> y{ram_initial};
  std::array<std::uint8_t, 4> known{1, 1, 1, 1};
  std::array<std::uint8_t, 1> y_known{1};
  std::array<MemoryWindow, 1> x_view{{{.words = x, .known = known}}};
  std::array<MemoryWindow, 1> y_view{{{.words = y, .known = y_known, .writable = true}}};
  Registers registers{};
  std::array<PassWord, 1> captured{};

  bool capture() {
    return capture_pass_memory(effects, registers, PassWindows{x_view, y_view}, captured, true);
  }
};

void snapshot() {
  Fixture f;
  f.registers.m[1] = dsp56300::detail::modifier_mask;
  require(f.capture() && same_pass_memory(f.captured),
          "Writes are recorded from a current owner-only loan using caller-provided storage");
  f.y.front() = ram_modified;
  require(!same_pass_memory(f.captured), "Changed RAM value invalidates the fixed point");
  f.y.front() = ram_initial;
  require(same_pass_memory(f.captured),
          "Several intermediate writes may restore the initial value");
  f.y_known.front() = 0;
  require(!same_pass_memory(f.captured) && !f.capture(),
          "Knowledge bits are part of the RAM state");
}

void retains_dynamic_address_source() {
  Fixture f;
  f.registers.m[1] = dsp56300::detail::modifier_mask;
  f.x[2] = f.x[1];
  std::array<PassWord, 3> cells{};
  dsp56300::detail::PassFootprint footprint{.cells = cells};
  require(
      capture_pass_memory(
          f.effects, f.registers, PassWindows{f.x_view, f.y_view}, f.captured, true, &footprint) &&
          footprint.complete && footprint.used == cells.size() && same_pass_memory(cells),
      "The retained footprint contains the write, addressed read and its address source");
  f.x.front() = 2;
  require(!same_pass_memory(cells),
          "Retargeting a dynamic read invalidates its footprint even for identical data");
}

void rejects_loan() {
  Fixture f;
  f.registers.m[1] = dsp56300::detail::modifier_mask;
  f.x.front() = device.address;
  require(!f.capture(), "Dynamic port address cannot reach device callbacks through ordinary RAM");
  f.x.front() = 1;
  f.known[1] = 0;
  require(!f.capture(), "Unknown dynamic read would enter the external memory fallback");
  f.known[1] = 1;
  f.registers.m[1] = 0;
  require(!f.capture(), "Nonlinear AGU fails before a proof executes");
  f.registers.m[1] = dsp56300::detail::modifier_mask;
  f.y_view.front().aliases_program = true;
  require(!f.capture(), "Writable executable alias would invalidate code");
  f.y_view.front().aliases_program = false;
  f.y_view.front().writable = false;
  require(!f.capture(), "Read-only loan cannot supply a write destination");
  f.y_view.front().writable = true;
  f.y_view.front().words = std::span(f.x).first<1>();
  require(!f.capture(), "Physical alias of another bank cannot change a dynamic read address");
  f.y_view.front().words = f.y;
  f.y_view.front().known = std::span(std::bit_cast<std::uint8_t *>(f.x.data()), 1U);
  require(!f.capture(), "A write's knowledge byte cannot alias the address-source word");
  f.y_view.front().known = f.y_known;
  f.x_view.front().known = std::span(std::bit_cast<std::uint8_t *>(f.y.data()), f.x.size());
  require(!f.capture(), "A store cannot alias an address source's knowledge byte");
}

void execute_with_native_image() {
  const std::array code{CodeSpan{.address = head, .words = fixture}};
  constexpr std::array entries{head};
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
  constexpr std::array layout{
      RegionLayout{.space = Space::program, .first = 0, .size = head + fixture.size()},
      RegionLayout{.space = Space::x, .first = 0, .size = data_words},
      RegionLayout{.space = Space::y, .first = 0, .size = data_words}};
  const std::array memory{VariantLayout{.id = 0, .memory = layout}};
  static constexpr std::array ports{device};
  constexpr std::array passes{
      FixedPassDeclaration{.variant = 0, .path = path, .device_reads = ports}};
  const CompileOptions options{.memory = memory,
                               .interrupts = {},
                               .transitions = {},
                               .callbacks = {},
                               .bindings = {},
                               .fixed_passes = passes};
  const auto saved = Core::lower({.variants = variants}, options);
  require(Core::load({.variants = variants}, options, saved.artifact()).identity() ==
              saved.identity(),
          "The matching declaration restores the proof metadata from cached prepared code");
  auto ordinary = options;
  ordinary.fixed_passes = {};
  bool rejected_scope = false;
  try {
    static_cast<void>(Core::load({.variants = variants}, ordinary, saved.artifact()));
  } catch (const std::invalid_argument &) { rejected_scope = true; }
  require(rejected_scope, "A proof artifact requires its declaration");
  const auto native = Core::compile({.variants = variants}, options);
  const auto effects = fixed_pass_effects(image(fixture), path, std::array{device});
  if (!effects) { throw std::runtime_error("Expected a proven native fixture scope"); }

  std::array<Word, head + fixture.size()> program{};
  std::array<Word, data_words> x{};
  std::array<Word, data_words> y{};
  x[dynamic_cursor] = 1;
  StablePort port;
  Core core(Memory::separate({.program = program, .x = x, .y = y}).with_canonical_data(), &port);
  core.load(Space::program, head, fixture);
  Registers initial{};
  initial.pc = head;
  initial.m.fill(dsp56300::word_mask);
  core.set_registers(initial);
  core.install(native, 0);
  const auto warm = core.run(effects->cost.instructions);
  require(warm.fault == Fault{} && warm.retired == effects->cost.instructions &&
              warm.progress.estimated_cycles == effects->cost.cycles && port.reads == 1,
          "Ordinary installed native code executes the exact device read on a real traversal");

  const auto before = core.registers();
  const std::array x_view{MemoryWindow{.words = x, .known = {}, .writable = true}};
  const std::array y_view{MemoryWindow{.words = y, .known = {}, .writable = true}};
  std::array<PassWord, 1> snapshot{};
  require(capture_pass_memory(*effects, before, PassWindows{x_view, y_view}, snapshot, true),
          "The compiled ordinary-access footprint binds to the real guest RAM");
  const auto run = core.run(effects->cost.instructions);
  require(run.fault == Fault{} && run.retired == effects->cost.instructions &&
              run.progress.estimated_cycles - warm.progress.estimated_cycles ==
                  effects->cost.cycles &&
              core.registers() == before && same_pass_memory(snapshot) && port.reads == 2,
          "The real AOT pass restores complete visible state and owner's writable RAM");

  constexpr unsigned repetitions = 5;
  port.until(std::numeric_limits<std::uint64_t>::max());
  const auto batch = core.run(repetitions * effects->cost.instructions);
  require(batch.fault == Fault{} && batch.retired == repetitions * effects->cost.instructions &&
              batch.progress.estimated_cycles - run.progress.estimated_cycles ==
                  repetitions * effects->cost.cycles &&
              core.registers() == before && same_pass_memory(snapshot) && port.reads == 3,
          "One compiled proof pass accounts for all five traversals in the same grant");

  port.until(batch.progress.estimated_cycles + (2 * effects->cost.cycles));
  const auto limited = core.run(repetitions * effects->cost.instructions);
  require(limited.retired == batch.retired && core.registers() == before && port.reads == 7,
          "Only the earlier stable portion folds before continuing ordinary execution");

  port.until(std::numeric_limits<std::uint64_t>::max());
  const auto exact_deadline = limited.progress.estimated_cycles + (2 * effects->cost.cycles);
  const auto timed = core.run_until(exact_deadline);
  require(timed.retired == 2 * effects->cost.instructions &&
              timed.progress.estimated_cycles == exact_deadline && core.registers() == before &&
              port.reads == 8,
          "The native cycle quota ends at the exact completed pass");

  x[dynamic_cursor] = device.address;
  const auto rejected = core.run(repetitions * effects->cost.instructions);
  require(rejected.retired == 1 && rejected.fault.kind == FaultKind::io_read &&
              rejected.fault.pc == head + 1 && rejected.fault.address == device.address &&
              port.reads == 8 &&
              rejected.progress.estimated_cycles == timed.progress.estimated_cycles + 1,
          "Failed candidate executes the actual fallback access at its precise fault boundary");
  core.reset();
  x[dynamic_cursor] = 1;
  core.set_registers(initial);
  core.install(native, 0);
  port.enable_sample();
  static_cast<void>(core.run(effects->cost.instructions));
  const auto calls = port.reads;
  static_cast<void>(core.run(repetitions * effects->cost.instructions));
  require(port.reads == calls + 1, "The first reusable pass executes one device read");
  static_cast<void>(core.run(repetitions * effects->cost.instructions));
  require(port.reads == calls + 1 &&
              core.progress().estimated_cycles == effects->cost.cycles * (1U + 2U * repetitions),
          "The matching device word and RAM footprint reuse the fixed-point proof");
  x[1] ^= 1U;
  static_cast<void>(core.run(repetitions * effects->cost.instructions));
  require(port.reads > calls + 1, "A changed input RAM word invalidates the fixed-point proof");
}

void backing_identity() {
  constexpr Word y_bank = 1U << 6;
  constexpr auto y_cursor_bit = (dynamic_cursor << bit_address_shift) | y_bank;
  constexpr std::array code_words{from_x_to_r1,
                                  indexed_read,
                                  (set_x6_bit4 & ~bit_address_mask) | y_cursor_bit,
                                  (clear_x6_bit4 & ~bit_address_mask) | y_cursor_bit,
                                  to_a_from_port,
                                  jump_head};
  const std::array code{CodeSpan{.address = head, .words = code_words}};
  constexpr std::array entries{head};
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}}};
  constexpr std::array layout{
      RegionLayout{.space = Space::program, .first = 0, .size = head + code_words.size()},
      RegionLayout{.space = Space::x, .first = 0, .size = data_words},
      RegionLayout{.space = Space::y, .first = 0, .size = data_words}};
  const std::array memory{VariantLayout{.id = 0, .memory = layout}};
  static constexpr std::array ports{device};
  constexpr std::array passes{
      FixedPassDeclaration{.variant = 0, .path = path, .device_reads = ports}};
  const auto native = Core::compile({.variants = variants},
                                    {.memory = memory,
                                     .interrupts = {},
                                     .transitions = {},
                                     .callbacks = {},
                                     .bindings = {},
                                     .fixed_passes = passes});
  const auto effects = fixed_pass_effects(image(code_words), path, ports);
  require(effects && effects->writes == std::vector{PassCell{Space::y, dynamic_cursor}},
          "Fixture writes the Y word at the X address-source offset");

  const auto name = "/dsp56300-pass-" + std::to_string(getpid());
  constexpr unsigned private_mode = 0600;
  const auto backing = shm_open(name.c_str(), O_RDWR | O_CREAT | O_EXCL, private_mode);
  constexpr auto bytes = data_words * sizeof(Word);
  require(backing >= 0 && shm_unlink(name.c_str()) == 0 && ftruncate(backing, bytes) == 0,
          "Create shared DSP test backing");
  const auto unmap = [](void *view) { static_cast<void>(munmap(view, bytes)); };
  const auto map = [&] {
    auto *const view = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, backing, 0);
    require(view != MAP_FAILED, "Map a shared DSP data view");
    return std::unique_ptr<void, decltype(unmap)>(view, unmap);
  };
  auto x_mapping = map();
  auto alias_mapping = map();
  require(close(backing) == 0, "The mapped test backing no longer needs an open descriptor");
  const std::span x(static_cast<Word *>(x_mapping.get()), data_words);
  const std::span alias(static_cast<Word *>(alias_mapping.get()), data_words);
  require(x.data() != alias.data(), "The same backing has two distinct virtual views");
  std::array<Word, data_words> independent{};

  const auto verify = [&](const std::span<Word> y, const bool canonical, const bool fold) {
    x[dynamic_cursor] = 1;
    y[dynamic_cursor] = 1;
    Registers initial{};
    initial.pc = head;
    initial.m.fill(dsp56300::word_mask);
    const std::array x_view{MemoryWindow{.words = x, .known = {}, .writable = true}};
    const std::array y_view{MemoryWindow{.words = y, .known = {}, .writable = true}};
    std::array<PassWord, 1> scratch{};
    require(capture_pass_memory(
                *effects, initial, PassWindows{x_view, y_view}, scratch, canonical) == fold,
            "Only a proved nonalias admits the dynamic-base and write combination");

    std::array<Word, head + code_words.size()> program{};
    StablePort port;
    const auto data = Memory::separate({.program = program, .x = x, .y = y});
    Core core(canonical ? data.with_canonical_data() : data, &port);
    core.load(Space::program, head, code_words);
    core.set_registers(initial);
    core.install(native, 0);
    const auto warm = core.run(effects->cost.instructions);
    const auto before = core.registers();
    require(warm.fault == Fault{} && port.reads == 1, "Run one ordinary warmup pass");
    port.until(std::numeric_limits<std::uint64_t>::max());
    constexpr unsigned passes_to_run = 5;
    const auto run = core.run(passes_to_run * effects->cost.instructions);
    require(run.fault == Fault{} && run.retired == passes_to_run * effects->cost.instructions &&
                run.progress.estimated_cycles ==
                    warm.progress.estimated_cycles + passes_to_run * effects->cost.cycles &&
                core.registers() == before && x[dynamic_cursor] == 1 && y[dynamic_cursor] == 1,
            "Fold and ordinary execution retire identical instructions, cycles and guest state");
    require(port.reads == (fold ? 2U : passes_to_run + 1),
            "A declined proof executes each actual device read in the ordinary image");
  };

  verify(independent, true, true);
  verify(x, true, false);
  verify(alias, false, false);
}
}

int main() try {
  masked_device_read();
  decoded_effects();
  rejects_effects();
  snapshot();
  retains_dynamic_address_source();
  rejects_loan();
  execute_with_native_image();
  backing_identity();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
