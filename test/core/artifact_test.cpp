// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_compiler.hpp"
#include "isa.hpp"
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
  roundtrip();
  std::cout << "Semantic artifacts restore native execution without lowering\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
