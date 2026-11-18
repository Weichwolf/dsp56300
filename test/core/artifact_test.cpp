// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_compiler.hpp"
#include "isa.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "retirement_observer.hpp"
#include "support.hpp"

#include "llvm-c/Error.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Transforms/PassBuilder.h"
#include "llvm-c/Types.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <dlfcn.h>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
unsigned passes = 0;
constexpr Word start = 16, destination = 32, vector = 8, absent = 96;
constexpr Word program_words = 64, data_words = 16;
constexpr Word nop = static_cast<Word>(isa::Opcode::nop);
constexpr Word move = static_cast<Word>(isa::Opcode::move_immediate_r0);
constexpr std::array<Word, 1> initial_entries{start};
constexpr std::array<Word, 1> final_entries{destination};
constexpr std::array<Word, 1> failed{absent};
constexpr std::array interrupts{InterruptRequest{.vector = vector, .level = 0}};
constexpr std::array transitions{CodeTransition{.from = 0, .to = 1, .entry = destination}};

struct Source {
  std::array<Word, 1> jump{static_cast<Word>(isa::Opcode::jmp_absolute_short) | destination};
  std::array<Word, 2> first_vector{nop, move | (2U << 16) | (7U << 8)};
  std::array<Word, 2> second_vector{nop, move | (2U << 16) | (9U << 8)};
  std::array<Word, 2> target{move | (5U << 8), static_cast<Word>(isa::Opcode::stop)};
  const std::array<CodeSpan, 2> first{CodeSpan{.address = start, .words = jump},
                                      CodeSpan{.address = vector, .words = first_vector}};
  const std::array<CodeSpan, 2> second{CodeSpan{.address = destination, .words = target},
                                       CodeSpan{.address = vector, .words = second_vector}};
  const std::array<CodeVariant, 2> variants{
      CodeVariant{.id = 0, .code = first, .entries = initial_entries, .failed_fetches = {}},
      CodeVariant{.id = 1, .code = second, .entries = final_entries, .failed_fetches = failed}};
  std::array<RegionLayout, 3> memory{
      RegionLayout{.space = Space::program, .first = 0, .size = program_words},
      RegionLayout{.space = Space::x, .first = 0, .size = data_words},
      RegionLayout{.space = Space::y, .first = 0, .size = data_words}};
  const std::array<VariantLayout, 2> layouts{VariantLayout{.id = 0, .memory = memory},
                                             VariantLayout{.id = 1, .memory = memory}};

  [[nodiscard]] CodeImage image() const { return {.variants = variants}; }

  [[nodiscard]] CompileOptions options() const {
    return {.memory = layouts,
            .interrupts = interrupts,
            .transitions = transitions,
            .callbacks = {},
            .bindings = {}};
  }
};

void refused_installs(const NativeImage &image, const Source &source) {
  for (const bool bad_code : {false, true}) {
    std::array<Word, program_words> program{};
    std::array<Word, data_words> x{};
    std::array<Word, data_words> y{};
    program[start] = bad_code ? nop : source.jump[0];
    std::ranges::copy(source.first_vector, program.begin() + vector);
    Core core(Memory::separate(
        {.program = program, .x = std::span(x).first(bad_code ? x.size() : x.size() / 2), .y = y}));
    const auto before = core.state();
    bool refused = false;
    try {
      core.install(image, 0);
    } catch (const std::invalid_argument &) { refused = true; }
    const auto after = core.state();
    require(refused && after.registers == before.registers && after.failure == before.failure &&
                after.execution == before.execution && after.progress == before.progress,
            "A restored component still refuses actual code/layout mismatches transactionally");
  }
}

void execution(const NativeImage &image, const Source &source) {
  std::array<Word, program_words> program{};
  std::array<Word, data_words> x{};
  std::array<Word, data_words> y{};
  program[start] = source.jump[0];
  std::ranges::copy(source.first_vector, program.begin() + vector);
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  auto expected = core.registers();
  expected.pc = start;
  core.set_registers(expected);
  core.install(image, 0);
  core.interrupt(vector, 0);
  expected.r[2] = 7;
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 2, .estimated_cycles = 2},
          "A restored vector plan executes the original first captured vector exactly");
  const auto transfer = core.run();
  expected.pc = destination;
  require(transfer.retired == 1 && transfer.yielded && transfer.fault == Fault{} &&
              core.registers() == expected &&
              core.progress() == Progress{.instructions = 3, .estimated_cycles = 5},
          "Restored transition contracts yield after the actual complete JMP");
  core.load(Space::program, destination, source.target);
  core.load(Space::program, vector, source.second_vector);
  core.install(image, 1);
  require(core.foreground_prefix() == 1,
          "Restored prefix metadata retains the complete MOVE before the real STOP boundary");
  core.interrupt(vector, 0);
  expected.r[2] = 9;
  require(core.registers() == expected &&
              core.progress() == Progress{.instructions = 5, .estimated_cycles = 7},
          "The restored target variant selects its own real vector words");
  const auto completed = core.run();
  expected.r[0] = 5;
  expected.pc = destination + 2;
  require(completed.retired == 2 && completed.fault == Fault{} &&
              completed.state == ExecutionState::stopped && core.registers() == expected,
          "Restored ordinary code retains register effects and the original STOP");
  core.reset();
  expected = core.registers();
  expected.pc = absent;
  core.set_registers(expected);
  core.install(image, 1);
  const auto failure = core.run(1);
  require(failure.retired == 0 && core.registers() == expected &&
              failure.fault == Fault{.kind = FaultKind::memory_read,
                                     .space = Space::program,
                                     .address = absent,
                                     .pc = absent},
          "Restored failed-fetch metadata retains the original precise missing-memory fault");
}

void rejected(const Source &source,
              const std::span<const std::byte> bytes,
              const std::string_view reason) {
  const auto before = passes;
  bool refused = false;
  try {
    static_cast<void>(Core::load(source.image(), source.options(), bytes));
  } catch (const std::invalid_argument &error) {
    refused = std::string_view(error.what()).contains(reason);
  }
  require(refused && passes == before,
          "A stale or corrupt semantic artifact refuses explicitly without lowering");
}

void stale(const Source &source, const std::span<const std::byte> artifact) {
  for (const auto size :
       {std::size_t{0}, std::size_t{31}, artifact.size() / 2, artifact.size() - 1}) {
    rejected(source, artifact.first(size), "semantic artifact");
  }
  Source changed_code;
  changed_code.jump[0] = nop;
  rejected(changed_code, artifact, "input identity");
  Source changed_layout;
  changed_layout.memory[1].size /= 2;
  rejected(changed_layout, artifact, "input identity");
  std::vector<std::byte> compiler(artifact.begin(), artifact.end());
  const auto identity = std::as_bytes(std::span(Core::compiler_identity()));
  const auto found = std::ranges::search(compiler, identity);
  require(!found.empty(), "The artifact records its independently available compiler identity");
  *found.begin() ^= std::byte{1};
  constexpr std::size_t digest_bytes = 32;
  const auto body = std::span(compiler).subspan(digest_bytes);
  const auto digest = llvm::SHA256::hash(
      llvm::ArrayRef(reinterpret_cast<const std::uint8_t *>(body.data()), body.size()));
  std::ranges::copy(std::as_bytes(std::span(digest)), compiler.begin());
  rejected(source, compiler, "compiler identity");
}

void roundtrip() {
  const Source source;
  const auto initial = passes;
  require(!Core::compiler_identity().empty() && passes == initial,
          "Compiler identity is available without preparing any component");
  std::vector<std::byte> artifact;
  std::vector<std::byte> bitcode;
  std::vector<std::pair<std::string, std::string>> exports;
  std::string identity;
  {
    const auto lowered = Core::lower(source.image(), source.options());
    artifact.assign(lowered.artifact().begin(), lowered.artifact().end());
    bitcode.assign(lowered.bitcode().begin(), lowered.bitcode().end());
    identity = lowered.identity();
    for (const auto symbol : lowered.exports()) { exports.emplace_back(symbol.name, symbol.abi); }
  }
  require(passes > initial && !artifact.empty(),
          "An independent observer sees real LLVM work during original cold lowering");
  const auto before_load = passes;
  auto transport = artifact;
  const auto loaded = Core::load(source.image(), source.options(), transport);
  std::ranges::fill(transport, std::byte{0});
  require(passes == before_load && loaded.identity() == identity &&
              std::ranges::equal(loaded.artifact(), artifact) &&
              std::ranges::equal(loaded.bitcode(), bitcode) &&
              loaded.exports().size() == exports.size(),
          "Restoration owns complete code and metadata after producer and transport expire");
  for (std::size_t index = 0; index < exports.size(); ++index) {
    require(loaded.exports()[index].name == exports[index].first &&
                loaded.exports()[index].abi == exports[index].second,
            "Every restored native export retains its complete original name and ABI");
  }
  stale(source, artifact);
  const auto native = test::compile_image(loaded);
  require(native.report().prepared_entries == 11,
          "Restoration preserves both ordinary instruction inventories and both fast vector plans");
  refused_installs(native, source);
  execution(native, source);
}

using test::retirement_binding;
using test::RetirementIo;

void register_only_control() {
  constexpr std::size_t code_words = destination - start + 4;
  constexpr std::uint64_t cycles = (2 * 3) + 4 + 10;
  std::array<Word, code_words> code;
  code.fill(static_cast<Word>(isa::Opcode::stop));
  code[0] = move | (3U << 8);
  code[1] = nop;
  code[2] = static_cast<Word>(isa::Opcode::jsr_absolute_short) | destination;
  code[destination - start] = move | (1U << 16) | (5U << 8);
  code[destination - start + 1] = nop;
  code[destination - start + 2] = static_cast<Word>(isa::Opcode::rts);
  const std::array spans{CodeSpan{.address = start, .words = code}};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = initial_entries, .failed_fetches = {}}};
  const Source source;
  const std::array layouts{VariantLayout{.id = 0, .memory = source.memory}};
  const std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  const auto binding = retirement_binding();
  const auto image = Core::compile({.variants = variants},
                                   {.memory = layouts,
                                    .interrupts = {},
                                    .transitions = {},
                                    .callbacks = callbacks,
                                    .bindings = binding});
  std::array<Word, program_words> program{};
  std::array<Word, data_words> x{};
  std::array<Word, data_words> y{};
  std::ranges::copy(code, program.begin() + start);
  RetirementIo io;
  Core core(Memory::separate({.program = program, .x = x, .y = y}), &io);
  auto expected = test::register_model::seed(0);
  expected.pc = start;
  expected.sr = 0;
  expected.sp = program_words - 1;
  expected.sc = 0;
  core.set_registers(expected);
  core.install(image, 0);
  expected.pc = start + 4;
  expected.r[0] = 3;
  expected.r[1] = 5;
  expected.stack[0] = std::uint64_t{start + 3} << test::word_bits;
  const auto result = core.run();
  require(result.retired == 7 &&
              result.progress == Progress{.instructions = 7, .estimated_cycles = cycles} &&
              result.state == ExecutionState::stopped && result.fault == Fault{} &&
              core.registers() == expected,
          "Register-only calls and returns preserve the complete authored state and clock");
  require(io.counts == std::array<std::uint64_t, 2>{6, 1},
          "RAM-free control prefixes retire natively; only STOP uses exact instruction lowering");
}

struct RetirementMachine {
  RetirementMachine(const NativeImage &image,
                    const std::span<const Word> code,
                    const bool fault,
                    const bool no_io = false) {
    std::ranges::copy(code, program.begin() + start);
    for (std::size_t index = 0; index < x.size(); ++index) {
      x[index] = static_cast<Word>(index + 1);
    }
    core = std::make_unique<Core>(Memory::separate({.program = program, .x = x, .y = y}),
                                  no_io ? nullptr : &io);
    auto registers = core->registers();
    registers.pc = start;
    registers.omr = 0;
    registers.r[0] = fault ? data_words + 1 : 0;
    core->set_registers(registers);
    core->install(image, 0);
  }

  std::array<Word, program_words> program{};
  std::array<Word, data_words> x{}, y{};
  RetirementIo io;
  std::unique_ptr<Core> core;
};

void immediate_alu(const Word width, const bool destination_b) {
  using enum isa::Opcode;
  using test::bfu_model::Function;
  constexpr std::array forms{std::pair{add_immediate_short, add_immediate_long},
                             std::pair{sub_immediate_short, sub_immediate_long},
                             std::pair{cmp_immediate_short, cmp_immediate_long},
                             std::pair{or_immediate_short, or_immediate_long},
                             std::pair{eor_immediate_short, eor_immediate_long},
                             std::pair{and_immediate_short, and_immediate_long}};
  constexpr std::array register_ops{add_x0_a, sub_x0_a, cmp_x0_a};
  constexpr std::array logic{Function::bit_or, Function::bit_xor, Function::bit_and};
  constexpr std::array<Word, forms.size()> literals{
      test::sign24 + 1, word_mask, test::sign24 + 1, word_mask - 1, test::sign24 + 1, 0};
  constexpr Word short_operand_mask = 0x3f;
  const auto operand = [&](const std::size_t index) {
    return width == 1 ? literals.at(index) & short_operand_mask : literals.at(index);
  };
  const Word destination_bit = destination_b ? 8U : 0U;
  std::vector<Word> code;
  for (std::size_t index = 0; index < forms.size(); ++index) {
    const auto form = width == 1 ? forms.at(index).first : forms.at(index).second;
    code.push_back(static_cast<Word>(form) | destination_bit |
                   (width == 1 ? operand(index) << 8 : 0U));
    if (width == 2) { code.push_back(operand(index)); }
  }
  code.push_back(static_cast<Word>(stop));
  const std::array spans{CodeSpan{.address = start, .words = code}};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = initial_entries, .failed_fetches = {}}};
  const Source source;
  const std::array layouts{VariantLayout{.id = 0, .memory = source.memory}};
  const std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  const auto binding = retirement_binding();
  const auto image = Core::compile({.variants = variants},
                                   {.memory = layouts,
                                    .interrupts = {},
                                    .transitions = {},
                                    .callbacks = callbacks,
                                    .bindings = binding});
  for (const auto mode : {0U, test::mode_up, test::mode_down | test::mode_rm}) {
    auto expected = test::register_model::seed(mode);
    expected.pc = start;
    expected.sr = mode | test::status_byte;
    expected.a = test::sign56 - 1;
    expected.b = test::sign56 + 1;
    const auto initial = expected;
    for (std::size_t index = 0; index < forms.size(); ++index) {
      auto old = expected;
      if (index < register_ops.size()) {
        old.x = operand(index);
        test::sequence_model::alu(
            expected, old, static_cast<Word>(register_ops.at(index)) | destination_bit);
      } else {
        test::bfu_model::apply(expected,
                               old,
                               {.function = logic.at(index - register_ops.size()),
                                .form = test::bfu_model::Form::long_immediate,
                                .value = operand(index),
                                .destination_b = destination_b});
      }
      expected.pc += width;
      RetirementMachine machine(image, code, false);
      machine.core->set_registers(initial);
      const auto prefix = index + 1;
      const auto result = machine.core->run_until(prefix * width);
      require(result.fault == Fault{} && result.retired == prefix &&
                  result.progress ==
                      Progress{.instructions = prefix, .estimated_cycles = prefix * width} &&
                  machine.core->registers() == expected,
              "Immediate ALU regions preserve every timed prefix and complete accumulator state");
      if (prefix == forms.size()) {
        require(machine.io.counts == std::array<std::uint64_t, 2>{forms.size(), 0},
                "The complete immediate ALU prefix retires through the native region");
      }
    }
  }
}

void retirement_case(const std::array<NativeImage, 2> &images,
                     const std::span<const Word> code,
                     const bool fault,
                     const unsigned limit,
                     const bool deadline,
                     const bool no_io) {
  RetirementMachine before(images[0], code, fault, no_io);
  RetirementMachine after(images[1], code, fault, no_io);
  const auto first = deadline ? before.core->run_until(limit) : before.core->run(limit);
  const auto second = deadline ? after.core->run_until(limit) : after.core->run(limit);
  const auto a = before.core->state();
  const auto b = after.core->state();
  require(a.registers == b.registers && a.failure == b.failure && a.execution == b.execution &&
              a.progress == b.progress && first.retired == second.retired &&
              first.yielded == second.yielded && before.program == after.program &&
              before.x == after.x && before.y == after.y,
          "Retirement observation preserves every register, memory word, fault and boundary");
  require(before.io.unit_entries == 0 &&
              (no_io || second.retired == 0 || after.io.unit_entries != 0) &&
              (limit != 0 || after.io.unit_entries == 0),
          "Only explicitly observed native execution counts unit entries; zero grants enter none");
  require(before.io.batched == 0 && after.io.deferred <= after.io.batched &&
              after.io.batched <= after.io.counts[0] &&
              before.io.counts == std::array<std::uint64_t, 2>{} &&
              (no_io || after.io.counts[0] + after.io.counts[1] == b.progress.instructions),
          "Only an explicit binding observes successful retirement, including partial runs");
  constexpr unsigned complete = 32;
  if (limit == complete && !no_io) {
    require(after.io.counts ==
                (fault ? std::array<std::uint64_t, 2>{0, 1} : std::array<std::uint64_t, 2>{7, 1}),
            "The complete DO region counts setup and six body instructions; STOP stays precise");
    require(after.io.batched == (fault ? 0U : 7U) && after.io.deferred == 0,
            "The optional region token distinguishes actual batched retirement");
    constexpr unsigned opcode_shift = 24;
    constexpr unsigned kind_shift = 48;
    const auto last_pc = fault ? start : start + 4;
    const auto last_opcode = fault ? code.front() : code.back();
    require((after.io.last_site & word_mask) == last_pc &&
                ((after.io.last_site >> opcode_shift) & word_mask) == last_opcode &&
                (after.io.last_site >> kind_shift) != 0,
            "Successful precise retirement carries its original instruction site and kind");
  }
}

void retirement_vectors(const std::span<const std::byte> binding) {
  const Source source;
  const std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  auto options = source.options();
  options.callbacks = callbacks;
  options.bindings = binding;
  const auto image = Core::compile(source.image(), options);
  std::array<Word, program_words> program{};
  std::array<Word, data_words> x{};
  std::array<Word, data_words> y{};
  program[start] = source.jump[0];
  std::ranges::copy(source.first_vector, program.begin() + vector);
  RetirementIo io;
  Core core(Memory::separate({.program = program, .x = x, .y = y}), &io);
  auto registers = core.registers();
  registers.pc = start;
  core.set_registers(registers);
  core.install(image, 0);
  core.interrupt(vector, 0);
  require(io.counts == std::array<std::uint64_t, 2>{0, 2} && core.progress().instructions == 2,
          "Both inserted IRQ instructions retire through the precise observer");
  require(core.run().retired == 1 && io.counts[0] + io.counts[1] == 3,
          "An owner transfer counts its completed instruction before returning");
  core.load(Space::program, destination, source.target);
  core.load(Space::program, vector, source.second_vector);
  core.install(image, 1);
  core.interrupt(vector, 0);
  const auto result = core.run();
  require(result.retired == 2 && result.state == ExecutionState::stopped &&
              core.progress().instructions == 7 && io.counts[0] + io.counts[1] == 7,
          "Variant changes and repeated IRQ admission preserve exact cumulative retirement");
}

void region_mode_hints() {
  constexpr Word mode_up = Word{1} << 11;
  constexpr std::array code{static_cast<Word>(isa::Opcode::ori_control) | (8U << 8),
                            static_cast<Word>(isa::Opcode::do_immediate) | (3U << 8),
                            start + 4,
                            static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) |
                                (3U << 11),
                            nop,
                            static_cast<Word>(isa::Opcode::stop)};
  const std::array spans{CodeSpan{.address = start, .words = code}};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = initial_entries, .failed_fetches = {}}};
  const Source source;
  const std::array layouts{VariantLayout{.id = 0, .memory = source.memory}};
  const std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  const auto binding = retirement_binding();
  CompileOptions options{.memory = layouts,
                         .interrupts = {},
                         .transitions = {},
                         .callbacks = callbacks,
                         .bindings = binding};
  const auto baseline = Core::lower({.variants = variants}, options);
  const auto reference = test::compile_image(baseline, binding);
  RetirementMachine original(reference, code, false);
  require(original.core->run().retired == 9 &&
              original.io.counts == std::array<std::uint64_t, 2>{7, 2},
          "Default preparation admits DO setup and its body after a guest mode change");
  constexpr std::array hinted{Word{0}};
  constexpr std::array admitted{mode_up};
  for (const auto hint :
       {std::span<const Word>{hinted}, std::span<const Word>{admitted}, std::span<const Word>{}}) {
    options.region_modes = hint;
    const auto prepared = Core::lower({.variants = variants}, options);
    bool rejected = false;
    try {
      static_cast<void>(Core::load({.variants = variants}, options, baseline.artifact()));
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected && baseline.identity() != prepared.identity(),
            "Fast-mode hints participate in semantic artifact identity and restoration checks");
    const auto restored = Core::load({.variants = variants}, options, prepared.artifact());
    const auto native = test::compile_image(restored, binding);
    RetirementMachine machine(native, code, false);
    const auto result = machine.core->run();
    const auto expected_counts = !hint.empty() && hint.front() == mode_up
                                     ? std::array<std::uint64_t, 2>{7, 2}
                                     : std::array<std::uint64_t, 2>{0, 9};
    const auto state = machine.core->state();
    const auto expected = original.core->state();
    require(result.retired == 9 && result.state == ExecutionState::stopped &&
                state.registers == expected.registers && state.progress == expected.progress &&
                state.failure == expected.failure && machine.program == original.program &&
                machine.x == original.x && machine.y == original.y &&
                machine.io.counts == expected_counts,
            "Unhinted modes stay exact through general native code; only hinted regions retire");
  }
}

void unit_entries(const std::span<const std::byte> binding) {
  constexpr std::array code{nop, nop, static_cast<Word>(isa::Opcode::stop)};
  const std::array spans{CodeSpan{.address = start, .words = code}};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = initial_entries, .failed_fetches = {}}};
  const std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  const CompileOptions options{.memory = {},
                               .interrupts = {},
                               .transitions = {},
                               .callbacks = callbacks,
                               .bindings = binding};
  const auto image = test::compile_image(Core::lower({.variants = variants}, options), binding);
  RetirementMachine partial(image, code, false);
  for (unsigned prefix = 0; prefix < code.size(); ++prefix) {
    require(partial.core->run(0).retired == 0 && partial.io.unit_entries == prefix,
            "A zero grant does not execute a native entry or duplicate an event");
    require(partial.core->run(1).retired == 1 && partial.io.unit_entries == prefix + 1,
            "Each resumed one-instruction run enters the same logical unit exactly once");
  }
  require(partial.core->run().retired == 0 && partial.io.unit_entries == code.size(),
          "Stopped execution does not manufacture new native entries");
  RetirementMachine whole(image, code, false);
  require(whole.core->run().retired == code.size() && whole.io.unit_entries == 1 &&
              whole.io.counts[0] + whole.io.counts[1] == code.size() &&
              whole.core->state().registers == partial.core->state().registers,
          "A complete straight-line unit enters once independently of its instruction count");
  for (const bool at_entry : {false, true}) {
    RetirementMachine yielded(image, code, false);
    yielded.io.yield_owner = yielded.core.get();
    yielded.io.yield_at_entry = at_entry;
    yielded.io.yield = [](void *owner) noexcept { static_cast<Core *>(owner)->request_yield(); };
    const auto result = yielded.core->run();
    const unsigned completed = at_entry ? 0U : 1U;
    require(result.retired == completed && result.yielded && result.fault == Fault{} &&
                result.state == ExecutionState::running &&
                result.progress.instructions == completed &&
                result.progress.estimated_cycles == completed &&
                yielded.core->registers().pc == start + completed && yielded.io.unit_entries == 1,
            "An observer yield stops before the next instruction at entry or retirement");
    yielded.io.yield = nullptr;
    require(yielded.core->run().retired == code.size() - completed &&
                yielded.core->state().registers == whole.core->state().registers &&
                yielded.io.counts[0] + yielded.io.counts[1] == code.size(),
            "An observer yield resumes without replay or missing retirement");
  }
}

void retirement_observation() {
  constexpr std::array code{static_cast<Word>(isa::Opcode::do_immediate) | (3U << 8),
                            start + 3,
                            static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) |
                                (3U << 11) |
                                (static_cast<Word>(isa::Opcode::adc_x_a) & test::status_byte),
                            nop,
                            static_cast<Word>(isa::Opcode::stop)};
  const std::array spans{CodeSpan{.address = start, .words = code}};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = initial_entries, .failed_fetches = {}}};
  const Source source;
  const std::array layouts{VariantLayout{.id = 0, .memory = source.memory}};
  const std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  const auto binding = retirement_binding();
  retirement_vectors(binding);
  unit_entries(binding);
  const CompileOptions off{
      .memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}};
  auto on = off;
  on.callbacks = callbacks;
  on.bindings = binding;
  const auto unobserved = Core::lower({.variants = variants}, off);
  const auto observed = Core::lower({.variants = variants}, on);
  require(unobserved.identity() != observed.identity(),
          "The optional observer participates in immutable compilation identity");
  bool refused = false;
  try {
    static_cast<void>(Core::load({.variants = variants}, off, observed.artifact()));
  } catch (const std::invalid_argument &) { refused = true; }
  require(refused, "An observed artifact cannot restore under an unobserved contract");
  const auto plain = test::compile_image(unobserved);
  const auto traced =
      test::compile_image(Core::load({.variants = variants}, on, observed.artifact()), binding);
  const std::array images{plain, traced};
  RetirementMachine yielded_region(traced, code, false);
  yielded_region.io.yield_owner = yielded_region.core.get();
  yielded_region.io.yield = [](void *owner) noexcept {
    static_cast<Core *>(owner)->request_yield();
  };
  const auto yielded = yielded_region.core->run();
  constexpr unsigned region_instructions = 7;
  require(yielded.yielded && yielded.retired == region_instructions &&
              yielded.progress.instructions == region_instructions && yielded.fault == Fault{} &&
              yielded_region.core->registers().pc == start + code.size() - 1,
          "A region retirement observer yield preserves its complete admitted prefix");
  yielded_region.io.yield = nullptr;
  require(yielded_region.core->run().retired == 1 &&
              yielded_region.io.counts[0] + yielded_region.io.counts[1] == region_instructions + 1,
          "A yielded region resumes at STOP without redoing its body or observation");
  for (const bool fault : {false, true}) {
    for (const bool deadline : {false, true}) {
      for (unsigned limit = 0; limit <= 12; ++limit) {
        retirement_case(images, code, fault, limit, deadline, false);
      }
      retirement_case(images, code, fault, 32, deadline, false);
      retirement_case(images, code, fault, 32, deadline, true);
    }
  }
}

}

extern "C" LLVMErrorRef LLVMRunPasses(LLVMModuleRef M,
                                      const char *Passes,
                                      LLVMTargetMachineRef TM,
                                      LLVMPassBuilderOptionsRef Options) {
  ++passes;
  auto *address = dlsym(RTLD_NEXT, "LLVMRunPasses");
  if (address == nullptr) { std::terminate(); }
  return std::bit_cast<decltype(&LLVMRunPasses)>(address)(M, Passes, TM, Options);
}

int main() try {
  register_only_control();
  for (const Word width : {1U, 2U}) {
    for (const bool destination_b : {false, true}) { immediate_alu(width, destination_b); }
  }
  roundtrip();
  retirement_observation();
  region_mode_hints();
  std::cout << "Semantic artifacts restore native execution without lowering\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
