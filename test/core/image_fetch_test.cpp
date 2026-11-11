// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);
constexpr Word start = 0x40, absent = 0x380;
constexpr unsigned word_bits = 24;
constexpr std::size_t program_words = 128;
constexpr Word absolute = 0x30, indirect = 4, zero_flag = 4;
constexpr Word nop = op<isa::Opcode::nop>, stop = op<isa::Opcode::stop>;

struct Form {
  std::array<Word, 4> code;
  std::size_t words;
  unsigned cycles;
  bool call;
};

constexpr std::array forms{
    Form{.code = {nop, op<isa::Opcode::jmp_absolute_short> | absent, stop, nop},
         .words = 3,
         .cycles = 3,
         .call = false},
    Form{.code = {nop, op<isa::Opcode::jsr_absolute_short> | absent, stop, nop},
         .words = 3,
         .cycles = 3,
         .call = true},
    Form{.code = {nop, op<isa::Opcode::jmp_effective> | (absolute << 8), absent, stop},
         .words = 4,
         .cycles = 4,
         .call = false},
    Form{.code = {nop, op<isa::Opcode::jsr_effective> | (absolute << 8), absent, stop},
         .words = 4,
         .cycles = 4,
         .call = true},
    Form{.code = {nop, op<isa::Opcode::jmp_effective> | (indirect << 11), stop, nop},
         .words = 3,
         .cycles = 3,
         .call = false},
    Form{.code = {nop, op<isa::Opcode::jeq_absolute_short> | absent, stop, nop},
         .words = 3,
         .cycles = 4,
         .call = false}};

NativeImage compile() {
  std::array<CodeSpan, forms.size()> spans{};
  std::array<CodeVariant, forms.size()> variants{};
  constexpr std::array entries{start};
  std::vector<Word> failures{absent};
  for (std::size_t index = 0; index < forms.size(); ++index) {
    spans[index] = {.address = start,
                    .words = std::span(forms[index].code).first(forms[index].words)};
    variants[index] = {
        .id = index, .code = {&spans[index], 1}, .entries = entries, .failed_fetches = failures};
  }
  const auto result = Core::compile({.variants = variants}, {});
  failures[0] = absent + 1;
  require(result.report().prepared_entries == forms.size() * 3 &&
              result.report().unresolved_entries == 0,
          "Failed-fetch boundaries are owned but never counted as translated instructions");
  return result;
}

Registers initial() {
  auto result = test::register_model::seed(0);
  result.pc = start;
  result.sr = zero_flag;
  result.r[0] = absent;
  return result;
}

Registers expected(const Form &form) {
  auto result = initial();
  result.pc = absent;
  if (form.call) {
    constexpr Word pointer_mask = 63;
    result.sp = (result.sp + 1) & pointer_mask;
    ++result.sc;
    const auto resume = start + static_cast<Word>(form.words) - 1;
    result.stack[result.sp % stack_entries] = (std::uint64_t{resume} << word_bits) | result.sr;
  }
  return result;
}

void check_prefix(const Core &core,
                  const RunResult &result,
                  const Form &form,
                  const unsigned count,
                  const bool failed) {
  auto registers = initial();
  if (count == 1) { ++registers.pc; }
  if (count == 2) { registers = expected(form); }
  const Progress progress{.instructions = count,
                          .estimated_cycles = count < 2 ? count : 1 + form.cycles};
  const Fault failure = failed ? Fault{.kind = FaultKind::memory_read,
                                       .space = Space::program,
                                       .address = absent,
                                       .value = 0,
                                       .pc = absent}
                               : Fault{};
  require(core.registers() == registers && core.progress() == progress && result.retired == count &&
              result.fault == failure && result.state == ExecutionState::running && !result.yielded,
          "Failed fetches preserve independent complete prefixes, quotas and deadlines");
}

void prefixes(const NativeImage &image) {
  for (std::size_t index = 0; index < forms.size(); ++index) {
    const auto &form = forms[index];
    std::array<Word, program_words> program{};
    std::ranges::copy(std::span(form.code).first(form.words), program.begin() + start);
    std::array<Word, 1> x{};
    std::array<Word, 1> y{};
    Core core(Memory::separate({.program = program, .x = x, .y = y}));
    for (const bool timed : {false, true}) {
      const auto end = timed ? form.cycles + 2 : 3;
      for (unsigned limit = 0; limit <= end; ++limit) {
        core.reset();
        core.set_registers(initial());
        core.install(image, index);
        const auto result = timed ? core.run_until(limit) : core.run(limit);
        check_prefix(
            core, result, form, std::min(limit, 2U), timed ? limit > 1 + form.cycles : limit > 2);
      }
    }
  }
}

void untaken(const NativeImage &image) {
  const auto &form = forms.back();
  std::array<Word, program_words> program{};
  std::ranges::copy(std::span(form.code).first(form.words), program.begin() + start);
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  Core core(Memory::separate({.program = program, .x = x, .y = y}));
  auto registers = initial();
  registers.sr = 0;
  core.set_registers(registers);
  core.install(image, forms.size() - 1);
  const auto result = core.run();
  registers.pc = start + static_cast<Word>(form.words);
  constexpr std::uint64_t cycles = 15;
  require(result.retired == 3 && result.fault == Fault{} &&
              result.state == ExecutionState::stopped && core.registers() == registers &&
              core.progress() == Progress{.instructions = 3, .estimated_cycles = cycles},
          "An untaken failed-fetch branch retains its genuine normal STOP continuation");
}

struct Port final : Io {
  std::span<const Word> code;
  bool inspectable = false, readable = false;
  unsigned reads = 0;
  Word observed_pc = 0;

  explicit Port(const std::span<const Word> source) : code(source) {}

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (address >= start && address - start < code.size()) { return code[address - start]; }
    return address == absent && inspectable ? std::optional{nop} : std::nullopt;
  }

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    if (space != Space::program || address != absent || pc != address) { return false; }
    ++reads;
    observed_pc = pc;
    value = stop;
    return readable;
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

void external(const NativeImage &image) {
  const auto &form = forms[3];
  Port io(std::span(form.code).first(form.words));
  Core core(Memory::external(), &io);
  for (const bool readable : {false, true}) {
    core.reset();
    core.set_registers(initial());
    io.readable = readable;
    io.reads = 0;
    core.install(image, 3);
    require(io.reads == 0, "Installation never replays or fabricates the actual failed read");
    const auto result = core.run(3);
    const auto kind = readable ? FaultKind::unprepared_execution : FaultKind::memory_read;
    require(result.retired == 2 &&
                result.fault == Fault{.kind = kind,
                                      .space = Space::program,
                                      .address = absent,
                                      .value = readable ? stop : 0,
                                      .pc = absent} &&
                core.registers() == expected(form) && io.reads == 1 && io.observed_pc == absent,
            "The actual fetch decides the fault; unexpectedly readable code is never executed");
    const auto again = core.run(1);
    require(again.retired == 0 && again.fault == result.fault && io.reads == 1,
            "A retained fetch fault cannot replay its observable read");
  }
  core.reset();
  core.set_registers(initial());
  io.inspectable = true;
  const auto before = core.state();
  bool rejected = false;
  try {
    core.install(image, 3);
  } catch (const std::invalid_argument &error) {
    rejected = std::string_view(error.what()).contains("readable source");
  }
  require(rejected && core.registers() == before.registers && core.progress() == before.progress &&
              core.state().failure == before.failure && io.reads == 1,
          "An inspectable word invalidates the failed-fetch contract without changing state");
}

void vectors() {
  constexpr Word vector = 0x20;
  constexpr unsigned level = 1;
  constexpr std::array caller{nop, stop};
  constexpr std::array sources{std::array{op<isa::Opcode::jsr_effective> | (absolute << 8), absent},
                               std::array{nop, op<isa::Opcode::jsr_absolute_short> | absent}};
  constexpr std::array entries{start};
  constexpr std::array failures{absent};
  constexpr std::array requests{InterruptRequest{.vector = vector, .level = level}};
  std::array<std::array<CodeSpan, 2>, sources.size()> spans{};
  std::array<CodeVariant, sources.size()> variants{};
  for (std::size_t index = 0; index < sources.size(); ++index) {
    spans[index] = {CodeSpan{.address = start, .words = caller},
                    CodeSpan{.address = vector, .words = sources[index]}};
    variants[index] = {
        .id = index, .code = spans[index], .entries = entries, .failed_fetches = failures};
  }
  const auto image = Core::compile(
      {.variants = variants},
      {.memory = {}, .interrupts = requests, .transitions = {}, .callbacks = {}, .bindings = {}});
  for (std::size_t index = 0; index < sources.size(); ++index) {
    std::array<Word, program_words> program{};
    std::ranges::copy(caller, program.begin() + start);
    std::ranges::copy(sources[index], program.begin() + vector);
    std::array<Word, 1> x{};
    std::array<Word, 1> y{};
    Core core(Memory::separate({.program = program, .x = x, .y = y}));
    auto registers = initial();
    registers.sr = 0;
    core.set_registers(registers);
    core.install(image, index);
    core.request_interrupt(vector, level);
    const auto result = core.run(3);
    registers.pc = absent;
    registers.sr = 0x200;
    ++registers.sp;
    ++registers.sc;
    registers.stack[registers.sp % stack_entries] = std::uint64_t{start} << word_bits;
    const auto count = static_cast<std::uint32_t>(index + 1);
    require(core.registers() == registers && result.retired == count &&
                core.progress() == Progress{.instructions = count, .estimated_cycles = 4} &&
                result.fault == Fault{.kind = FaultKind::memory_read,
                                      .space = Space::program,
                                      .address = absent,
                                      .value = 0,
                                      .pc = absent},
            "First- and second-slot vector calls preserve the real long-entry prefix before fault");
  }
}

void invalid_contracts() {
  const auto &form = forms[3];
  const std::array spans{
      CodeSpan{.address = start, .words = std::span(form.code).first(form.words)}};
  constexpr std::array entries{start};
  for (const std::vector<Word> &failures :
       {std::vector<Word>{}, {absent, absent}, {word_mask + 1}, {start}, {start + 2}}) {
    bool rejected = false;
    try {
      const std::array variants{
          CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = failures}};
      static_cast<void>(Core::lower({.variants = variants}, {}));
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Missing, duplicate, wide, code and operand fetch-failure contracts reject");
  }
  const auto &computed = forms[4];
  const std::array indirect_spans{
      CodeSpan{.address = start, .words = std::span(computed.code).first(computed.words)}};
  std::array failures{absent};
  const std::array variants{
      CodeVariant{.id = 0, .code = indirect_spans, .entries = entries, .failed_fetches = failures}};
  const auto first = Core::lower({.variants = variants}, {});
  ++failures[0];
  const auto second = Core::lower({.variants = variants}, {});
  require(first.identity() != second.identity(),
          "Changing an unavailable entry changes native component identity");
}
}

int main() try {
  invalid_contracts();
  const auto image = compile();
  prefixes(image);
  untaken(image);
  external(image);
  vectors();
  std::cout << test::checks << " native failed-fetch contract checks passed.\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
