// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_io_fixture.hpp"
#include "isa.hpp"
#include "jit/llvm/bindings.hpp"
#include "jit/llvm/image.hpp"
#include "jit/llvm/runtime.hpp"
#include "semantic_ir.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/InstrTypes.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Casting.h>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace dsp56300;
using namespace dsp56300::detail;
using namespace dsp56300::test;
std::span<const std::byte> active_semantics;
constexpr unsigned x0_register = 4, byte_shift = 8;
constexpr Word initial_x = 0x654321, initial_mask = 0x300, clear_mask = 0xfc;
constexpr std::array program{
    static_cast<Word>(isa::Opcode::movep_register_high) | (x0_register << byte_shift),
    static_cast<Word>(isa::Opcode::movep_register_high) | (x0_register << byte_shift) | 0x8000U,
    static_cast<Word>(isa::Opcode::bset_port_high) | io_bit,
    static_cast<Word>(isa::Opcode::andi_control) | (clear_mask << byte_shift),
    static_cast<Word>(isa::Opcode::reset),
    static_cast<Word>(isa::Opcode::stop)};
constexpr std::array roles{IoBinding{.role = IoRole::read, .name = "image_io_read"},
                           IoBinding{.role = IoRole::write, .name = "image_io_write"},
                           IoBinding{.role = IoRole::bit, .name = "image_io_bit"},
                           IoBinding{.role = IoRole::reset, .name = "image_io_reset"},
                           IoBinding{.role = IoRole::pending, .name = "image_io_pending"},
                           IoBinding{.role = IoRole::revision, .name = "image_io_revision"}};

std::vector<std::byte> read(const char *path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) { throw std::runtime_error("Cannot open semantic/binding input"); }
  const std::streamoff size = file.tellg();
  if (size <= 0) { throw std::runtime_error("Empty semantic/binding input"); }
  std::vector<std::byte> bytes(static_cast<std::size_t>(size));
  file.seekg(0);
  file.read(reinterpret_cast<char *>(bytes.data()), size);
  if (!file) { throw std::runtime_error("Incomplete semantic/binding input"); }
  return bytes;
}

NativeImage compile(const std::span<const std::byte> semantics) {
  active_semantics = semantics;
  const std::array code{CodeSpan{.address = io_entry, .words = program}};
  constexpr std::array entries{io_entry};
  const std::array versions{CodeVariant{.id = 0, .code = code, .entries = entries}};
  return Core::compile({.variants = versions}, {});
}

struct Prefix {
  Registers registers;
  Progress progress;
  Fault failure;
  ExecutionState state;
  std::uint32_t retired;
  bool yielded;
  bool operator==(const Prefix &) const = default;
};

struct Case {
  Word input;
  IoEvent failure, yield;
  bool pending, remap;
  unsigned quota;
};

struct Result {
  State state;
  Word output;
  std::uint64_t revision;
  std::vector<Prefix> prefixes;
  std::array<IoObservation, io_observations> observed;
  std::size_t count;
  std::array<unsigned, io_roles> dynamic_calls, bound_calls;
};

Result execute(const NativeImage &image, const Case &item) {
  ImageIo io;
  std::ranges::copy(program, io.program.begin() + io_entry);
  io.input = item.input;
  io.failure = item.failure;
  io.yield = item.yield;
  io.pending = item.pending;
  io.remap = item.remap;
  Core core(Memory::external(), &io);
  io.core = &core;
  Registers initial;
  initial.pc = io_entry;
  initial.sr = initial_mask;
  initial.x = initial_x;
  initial.m.fill(word_mask);
  core.set_registers(initial);
  core.install(image, 0);
  io.count = 0;
  io.dynamic_calls.fill(0);
  io.bound_calls.fill(0);
  std::vector<Prefix> prefixes;
  std::uint64_t revision = 0;
  for (unsigned run = 0; run < program.size() + 2; ++run) {
    const auto result = core.run(item.quota);
    prefixes.push_back({.registers = core.registers(),
                        .progress = result.progress,
                        .failure = result.fault,
                        .state = result.state,
                        .retired = result.retired,
                        .yielded = result.yielded});
    if (result.fault != Fault{} || result.state != ExecutionState::running) { break; }
    if (io.revision != revision) {
      const auto before = core.state();
      core.install(image, 0);
      require(core.registers() == before.registers && core.progress() == before.progress &&
                  core.state().failure == before.failure,
              "Actual remapping rebinds the prepared code without replay or state restoration");
      revision = io.revision;
    }
  }
  return {.state = core.state(),
          .output = io.output,
          .revision = io.revision,
          .prefixes = std::move(prefixes),
          .observed = io.observed,
          .count = io.count,
          .dynamic_calls = io.dynamic_calls,
          .bound_calls = io.bound_calls};
}

void expected(const Result &result, const Case &item) {
  require(result.count != 0 && result.count < io_observations,
          "The public callback log covers every actual transaction without truncation");
  require(result.revision == (item.remap && item.failure != IoEvent::read ? 1U : 0U),
          "A callback mapping change remains live even when its transaction later rejects");
  const auto observations = std::span(result.observed).first(result.count);
  const auto first = std::ranges::find(observations, IoEvent::read, &IoObservation::kind);
  require(first != observations.end() && first->pc == io_entry && first->address == io_port &&
              first->space == Space::x && first->registers.pc == io_entry + 1 &&
              first->registers.x == initial_x && first->progress.instructions == 0,
          "The first read observes the architectural pre-retirement state and exact access");
  if (item.failure == IoEvent::revision) {
    require(result.state.failure == Fault{} && result.state.execution == ExecutionState::stopped &&
                result.state.registers.pc == io_entry + program.size() &&
                result.state.progress.instructions == program.size() && result.output == 0 &&
                result.state.registers.x == (item.input & word_mask) &&
                (result.state.registers.sr & initial_mask) == 0,
            "Original guest loads, 24-bit masking, status edit, RESET and STOP complete");
  } else {
    constexpr std::array kinds{
        FaultKind::io_read, FaultKind::io_write, FaultKind::io_bit, FaultKind::host_event};
    const auto index = static_cast<unsigned>(item.failure);
    const auto offset = item.failure == IoEvent::reset ? 4U : index;
    const auto location = item.failure == IoEvent::reset ? io_entry + offset : io_port;
    const std::array values{
        0U, item.input & word_mask, io_bit, static_cast<Word>(isa::Opcode::reset)};
    const Fault fault{.kind = kinds.at(index),
                      .space = item.failure == IoEvent::reset ? Space::program : Space::x,
                      .address = location,
                      .value = values.at(index),
                      .pc = io_entry + offset};
    const std::array outputs{
        0U, item.input & word_mask, (item.input & word_mask) | (1U << io_bit), 0U};
    require(result.state.failure == fault && result.output == outputs.at(index),
            "A rejected transaction retains its actual partial effect and exact independent fault");
  }
  for (std::size_t i = 0; i < result.count; ++i) {
    if (result.observed[i].kind == IoEvent::write) {
      require(result.observed[i].value == (item.input & word_mask) &&
                  result.observed[i].registers.x == (item.input & word_mask),
              "A write sees the masked read result in both arguments and live core state");
    }
  }
}

void compare(const Result &reference, const Result &actual, const bool partial) {
  require(reference.prefixes == actual.prefixes && reference.output == actual.output &&
              reference.count == actual.count && reference.observed == actual.observed &&
              reference.revision == actual.revision,
          "Fusion preserves every complete ISA prefix, yield, failure and state-visible Io effect");
  for (unsigned role = 0; role < io_roles; ++role) {
    require(reference.dynamic_calls[role] == actual.dynamic_calls[role] + actual.bound_calls[role],
            "Each role retains its exact number of real callback effects");
    if (role == static_cast<unsigned>(IoEvent::revision)) { continue; }
    const bool selected = !partial || role == static_cast<unsigned>(IoEvent::read);
    require(selected ? actual.dynamic_calls[role] == 0 : actual.bound_calls[role] == 0,
            "Selected roles bypass virtual calls; unselected roles retain the original Io path");
  }
}

void case_variants(const NativeImage &reference,
                   const NativeImage &fused,
                   const NativeImage &partial,
                   const Case item) {
  for (const bool remap : {false, true}) {
    auto candidate = item;
    candidate.remap = remap;
    const auto normal = execute(reference, candidate);
    expected(normal, candidate);
    const auto optimized = execute(fused, candidate);
    expected(optimized, candidate);
    compare(normal, optimized, false);
    compare(normal, execute(partial, candidate), true);
  }
}

void qualification(const NativeImage &reference,
                   const NativeImage &fused,
                   const NativeImage &partial) {
  constexpr std::array events{
      IoEvent::read, IoEvent::write, IoEvent::bit, IoEvent::reset, IoEvent::revision};
  for (const auto input : {0U, word_mask, 0xfffedcbaU, 0x123456U}) {
    for (const auto failure : events) {
      for (const auto yield : events) {
        for (const bool pending : {false, true}) {
          for (unsigned quota = 1; quota <= program.size() + 1; ++quota) {
            case_variants(reference,
                          fused,
                          partial,
                          {.input = input,
                           .failure = failure,
                           .yield = yield,
                           .pending = pending,
                           .remap = false,
                           .quota = quota});
          }
        }
      }
    }
  }
}

void code_quality(const std::span<const std::byte> semantics) {
  LlvmModule module("public_io_code_quality");
  module.link_bitcode(semantics);
  const std::array code{CodeSpan{.address = io_entry, .words = program}};
  constexpr std::array entries{io_entry};
  lift_image(module, code, entries);
  specialize_image(module);
  LlvmCompiler compiler;
  require(!compiler.object(module).empty(), "The fused public Io program reaches native emission");
  for (const auto &function : *llvm::unwrap(module.code())) {
    require(!function.getName().starts_with("dsp56300_io_"),
            "Final O3 removes every temporary public Io adapter definition");
    for (const auto &block : function) {
      for (const auto &instruction : block) {
        const auto *call = llvm::dyn_cast<llvm::CallBase>(&instruction);
        const auto *target = call != nullptr ? call->getCalledFunction() : nullptr;
        require(target == nullptr || !target->getName().starts_with("dsp56300_io_"),
                "Prepared native code has no extra public Io adapter calls");
      }
    }
  }
}

struct BindingInputs {
  std::span<const std::byte> semantics, host;
};

void rejected(const BindingInputs inputs) {
  const auto check = [&](const std::span<const std::byte> input,
                         const std::span<const IoBinding> bindings) {
    bool failed = false;
    try {
      static_cast<void>(fuse_io(inputs.semantics, input, bindings));
    } catch (const std::exception &) { failed = true; }
    require(failed, "Invalid public Io role, definition or scalar ABI fails before publication");
  };
  check({}, roles);
  check(inputs.host, {});
  const std::array duplicate{roles[0], roles[0]};
  check(inputs.host, duplicate);
  const std::array missing{IoBinding{.role = IoRole::read, .name = "absent"}};
  check(inputs.host, missing);
  const std::array wrong{IoBinding{.role = IoRole::read, .name = roles[1].name}};
  check(inputs.host, wrong);
  LlvmModule changed("invalid_io_abi");
  changed.link_bitcode(inputs.host);
  auto *function = llvm::unwrap(changed.code())->getFunction("image_io_read");
  require(function != nullptr, "Independent public binding fixture contains the read definition");
  if (function->hasRetAttribute(llvm::Attribute::ZExt)) {
    function->removeRetAttr(llvm::Attribute::ZExt);
  } else {
    function->addRetAttr(llvm::Attribute::ZExt);
  }
  check(changed.bitcode(), roles);
  LlvmModule aliased("invalid_io_owner");
  aliased.link_bitcode(inputs.host);
  auto *owner = llvm::unwrap(aliased.code())->getFunction("image_io_read");
  require(owner != nullptr, "The public owner binding exists before the aliasing negative control");
  owner->addParamAttr(0, llvm::Attribute::NoAlias);
  check(aliased.bitcode(), roles);
  LlvmModule extent("invalid_io_extent");
  extent.link_bitcode(inputs.host);
  auto *output = llvm::unwrap(extent.code())->getFunction("image_io_read");
  require(output != nullptr, "The public read output exists before its extent negative control");
  output->addParamAttr(4,
                       llvm::Attribute::getWithDereferenceableBytes(
                           llvm::unwrap(extent.code())->getContext(), sizeof(Word) * 2));
  check(extent.bitcode(), roles);
}
}

namespace dsp56300::detail {
// Only replace the immutable semantic provider in this unit-test executable.
// Normal Core::compile/install/run perform every ISA, fault and device effect.
std::span<const std::byte> semantic_ir() noexcept {
  return active_semantics;
}

std::string_view compiler_identity() noexcept {
  return "public-io-fusion-test";
}
}

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 3) {
      throw std::invalid_argument("Expected semantic and public Io bitcode inputs");
    }
    const auto semantics = read(argv[1]);
    const auto host = read(argv[2]);
    rejected({.semantics = semantics, .host = host});
    const auto reference = compile(semantics);
    const auto fused_ir = fuse_io(semantics, host, roles);
    code_quality(fused_ir);
    const auto fused = compile(fused_ir);
    const auto partial_ir = fuse_io(semantics, host, std::span(roles).first(1));
    const auto partial = compile(partial_ir);
    qualification(reference, fused, partial);
    std::cout << test::checks << " public Io fusion/state/effect checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
