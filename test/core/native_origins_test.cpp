// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_compiler.hpp"
#include "isa.hpp"
#include "support.hpp"

#include "llvm-c/BitReader.h"
#include "llvm-c/Core.h"
#include "llvm-c/DebugInfo.h"
#include "llvm-c/Target.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Transforms/PassBuilder.h"
#include "llvm-c/Types.h"
#include "llvm/DebugInfo/DIContext.h"
#include "llvm/DebugInfo/DWARF/DWARFContext.h"
#include "llvm/DebugInfo/DWARF/DWARFDebugLine.h"
#include "llvm/DebugInfo/DWARF/DWARFUnit.h"
#include "llvm/Object/ObjectFile.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/MemoryBufferRef.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
using test::image_compiler::checked;
using test::image_compiler::Owner;
using Location = std::pair<std::uint64_t, Word>;
constexpr std::uint64_t high_variant = (std::uint64_t{1} << 48) + 7;
constexpr Word irq = 8;

std::vector<std::byte> object(const SemanticImage &image, const bool strip) {
  const Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
  const auto bytes = image.bitcode();
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> input(
      LLVMCreateMemoryBufferWithMemoryRangeCopy(
          reinterpret_cast<const char *>(bytes.data()), bytes.size(), "public-origins"));
  LLVMModuleRef parsed = nullptr;
  const auto failed = LLVMParseBitcodeInContext2(context.get(), input.get(), &parsed);
  const Owner<LLVMModuleRef, LLVMDisposeModule> module(parsed);
  require(failed == 0 && module, "Public semantic bitcode parses without private Core access");
  if (strip) { static_cast<void>(LLVMStripModuleDebugInfo(module.get())); }
  LLVMTargetRef architecture = nullptr;
  char *raw_error = nullptr;
  const auto found =
      LLVMGetTargetFromTriple(LLVMGetTarget(module.get()), &architecture, &raw_error);
  const Owner<char *, LLVMDisposeMessage> message(raw_error);
  require(found == 0, "The public image names a supported target");
  const Owner<char *, LLVMDisposeMessage> cpu(LLVMGetHostCPUName());
  const Owner<char *, LLVMDisposeMessage> features(LLVMGetHostCPUFeatures());
  const Owner<LLVMTargetMachineRef, LLVMDisposeTargetMachine> machine(
      LLVMCreateTargetMachine(architecture,
                              LLVMGetTarget(module.get()),
                              cpu.get(),
                              features.get(),
                              LLVMCodeGenLevelAggressive,
                              LLVMRelocPIC,
                              LLVMCodeModelSmall));
  require(machine != nullptr, "The independent native provider creates its target machine");
  const Owner<LLVMPassBuilderOptionsRef, LLVMDisposePassBuilderOptions> options(
      LLVMCreatePassBuilderOptions());
  checked(LLVMRunPasses(module.get(), "default<O3>", machine.get(), options.get()));
  LLVMMemoryBufferRef result = nullptr;
  char *raw_emission = nullptr;
  const auto emitted = LLVMTargetMachineEmitToMemoryBuffer(
      machine.get(), module.get(), LLVMObjectFile, &raw_emission, &result);
  const Owner<char *, LLVMDisposeMessage> emission(raw_emission);
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> output(result);
  require(emitted == 0 && output, "The independent provider emits the entire optimized image");
  const auto *first = reinterpret_cast<const std::byte *>(LLVMGetBufferStart(output.get()));
  return {first, first + LLVMGetBufferSize(output.get())};
}

std::optional<std::uint64_t> variant(const std::string_view file) {
  constexpr std::string_view prefix = "core-variant-";
  constexpr std::string_view suffix = ".S";
  if (!file.starts_with(prefix) || !file.ends_with(suffix)) { return {}; }
  const auto number = file.substr(prefix.size(), file.size() - prefix.size() - suffix.size());
  std::uint64_t result = 0;
  const auto parsed = std::from_chars(number.data(), number.data() + number.size(), result);
  require(!number.empty() && parsed.ec == std::errc{} &&
              parsed.ptr == number.data() + number.size(),
          "Variant identities are complete unsigned 64-bit values");
  return result;
}

struct Evidence {
  std::map<std::string, std::string> text;
  std::set<Location> locations;
};

Evidence inspect(const std::span<const std::byte> bytes) {
  auto parsed = llvm::object::ObjectFile::createObjectFile(llvm::MemoryBufferRef(
      llvm::StringRef(reinterpret_cast<const char *>(bytes.data()), bytes.size()), "native"));
  if (!parsed) { throw std::runtime_error(llvm::toString(parsed.takeError())); }
  Evidence result;
  for (const auto section : (*parsed)->sections()) {
    if (!section.isText()) { continue; }
    auto name = section.getName();
    auto contents = section.getContents();
    if (!name) { throw std::runtime_error(llvm::toString(name.takeError())); }
    if (!contents) { throw std::runtime_error(llvm::toString(contents.takeError())); }
    result.text.emplace(name->str(), contents->str());
  }
  const auto dwarf = llvm::DWARFContext::create(**parsed);
  for (const auto &unit : dwarf->compile_units()) {
    const auto *lines = dwarf->getLineTableForUnit(unit.get());
    if (lines == nullptr) { continue; }
    for (const auto &row : lines->Rows) {
      if (row.EndSequence || row.Line == 0) { continue; }
      std::string file;
      require(lines->getFileNameByIndex(
                  row.File, "", llvm::DILineInfoSpecifier::FileLineInfoKind::RawValue, file),
              "Every guest source location has a resolvable file");
      if (const auto id = variant(file)) { result.locations.emplace(*id, row.Line - 1); }
    }
  }
  return result;
}

void check() {
  constexpr auto nop = static_cast<Word>(isa::Opcode::nop);
  constexpr auto load_x =
      static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) | (3U << 11);
  constexpr auto load_y =
      static_cast<Word>(isa::Opcode::move_y_effective) | (1U << 15) | (3U << 11);
  constexpr std::array first{load_x,
                             static_cast<Word>(isa::Opcode::do_immediate) | (3U << 8),
                             Word{4},
                             load_y | (static_cast<Word>(isa::Opcode::adc_y_a) & 0xffU),
                             nop,
                             static_cast<Word>(isa::Opcode::stop)};
  constexpr std::array second{load_y, static_cast<Word>(isa::Opcode::stop)};
  constexpr std::array vector{load_x, load_y};
  const std::array first_spans{CodeSpan{.address = 0, .words = first},
                               CodeSpan{.address = irq, .words = vector}};
  const std::array second_spans{CodeSpan{.address = 0, .words = second},
                                CodeSpan{.address = irq, .words = vector}};
  constexpr std::array entries{Word{0}};
  const std::array variants{
      CodeVariant{.id = 0, .code = first_spans, .entries = entries, .failed_fetches = {}},
      CodeVariant{
          .id = high_variant, .code = second_spans, .entries = entries, .failed_fetches = {}}};
  constexpr std::array regions{RegionLayout{.space = Space::program, .first = 0, .size = 16},
                               RegionLayout{.space = Space::x, .first = 0, .size = 16},
                               RegionLayout{.space = Space::y, .first = 0, .size = 16}};
  const std::array layouts{VariantLayout{.id = 0, .memory = regions},
                           VariantLayout{.id = high_variant, .memory = regions}};
  constexpr std::array interrupts{InterruptRequest{.vector = irq, .level = 0}};
  const CompileOptions options{.memory = layouts,
                               .interrupts = interrupts,
                               .transitions = {},
                               .callbacks = {},
                               .bindings = {}};
  const auto image = Core::lower({.variants = variants}, options);
  const auto attributed = inspect(object(image, false));
  const auto stripped = inspect(object(image, true));
  require(!attributed.text.empty() && attributed.text == stripped.text &&
              stripped.locations.empty(),
          "Source metadata adds no native instructions under the same native O3 pipeline");
  for (const auto location : {Location{0, 0},
                              Location{high_variant, 0},
                              Location{0, 3},
                              Location{0, irq},
                              Location{0, irq + 1}}) {
    require(attributed.locations.contains(location),
            "Native source intervals retain PC0, full variant IDs, loop memory and shared vectors");
  }
  const auto native = Core::compile({.variants = variants}, options);
  for (const auto id : {std::uint64_t{0}, high_variant}) {
    std::array<Word, 16> program{};
    std::array<Word, 16> x{};
    std::array<Word, 16> y{};
    const auto code = id == 0 ? std::span<const Word>(first) : std::span<const Word>(second);
    std::ranges::copy(code, program.begin());
    std::ranges::copy(vector, program.begin() + irq);
    Core core(Memory::separate({.program = program, .x = x, .y = y}));
    auto registers = core.registers();
    registers.omr = 0;
    core.set_registers(registers);
    core.install(native, id);
    const auto result = core.run();
    require(
        result.state == ExecutionState::stopped && result.fault == Fault{} &&
            result.retired == (id == 0 ? 9U : 2U),
        "The standalone provider binds real linked intervals and executes both declared variants");
  }
}
}

int main() try {
  require(LLVMInitializeNativeTarget() == 0 && LLVMInitializeNativeAsmPrinter() == 0,
          "The native target initializes");
  check();
  std::cout << "Public native source attribution passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
