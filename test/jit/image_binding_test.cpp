// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "jit/llvm/runtime.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word entry = 16, clear_mask = 0xfc, mask = 0x300;
constexpr std::uint64_t initial_a = 0x2000000;
unsigned resources_created = 0, resources_destroyed = 0;

SemanticImage lower(const bool left) {
  const std::array words{static_cast<Word>(isa::Opcode::andi_control) | (clear_mask << 8),
                         static_cast<Word>(left ? isa::Opcode::asl_a : isa::Opcode::asr_a),
                         static_cast<Word>(isa::Opcode::stop)};
  const auto unchanged = words;
  const std::array code{CodeSpan{.address = entry, .words = words}};
  constexpr std::array entries{entry};
  const std::array variants{
      CodeVariant{.id = 7, .code = code, .entries = entries, .failed_fetches = {}}};
  auto image = Core::lower({.variants = variants}, {});
  require(words == unchanged && !image.bitcode().empty() && image.exports().size() == 1 &&
              !image.identity().empty() && image.exports().front().abi.starts_with("llvm19;"),
          "Public lowering owns complete IR, ABI exports and identity without changing source");
  return image;
}

struct Resource {
  Resource() { ++resources_created; }

  ~Resource() { ++resources_destroyed; }

  Resource(const Resource &) = delete;
  Resource &operator=(const Resource &) = delete;
  detail::LlvmCompiler compiler;
};

struct Images {
  NativeImage first, second;
  std::weak_ptr<Resource> owner;
};

void rejected(const SemanticImage &image, const NativeSymbols &receipt) {
  const auto check = [&](const NativeSymbols &candidate) {
    bool failed = false;
    try {
      static_cast<void>(Core::bind(image, candidate));
    } catch (const std::invalid_argument &) { failed = true; }
    require(failed, "Incomplete identity/owner/export receipts cannot bind native code");
  };
  auto invalid = receipt;
  invalid.owner.reset();
  check(invalid);
  invalid = receipt;
  invalid.identity = "other_component";
  check(invalid);
  invalid = receipt;
  invalid.symbols = {};
  check(invalid);
  auto symbol = receipt.symbols.front();
  symbol.address = nullptr;
  invalid.symbols = std::span(&symbol, 1);
  check(invalid);
  symbol = receipt.symbols.front();
  symbol.name = "unknown_export";
  check(invalid);
  symbol.name = {};
  check(invalid);
  symbol = receipt.symbols.front();
  const std::array duplicate{symbol, symbol};
  invalid.symbols = duplicate;
  check(invalid);
}

Images materialize() {
  auto first = lower(false);
  const auto second = lower(true);
  const auto same = lower(false);
  require(first.identity() == same.identity() && first.identity() != second.identity() &&
              first.exports().front().name != second.exports().front().name,
          "Exact source/contracts give deterministic identities and distinct image namespaces");
  auto copy = first;
  first = {};
  require(!copy.bitcode().empty(),
          "A semantic copy retains source-independent IR and export names");
  detail::LlvmModule module(std::string(copy.exports().front().name));
  module.link_bitcode(copy.bitcode());
  module.link_bitcode(second.bitcode());
  for (const auto &function : *llvm::unwrap(module.code())) {
    if (!function.hasFnAttribute("dsp56300-io-adapter")) { continue; }
    require(!function.hasFnAttribute(llvm::Attribute::NoInline) &&
                function.hasFnAttribute(llvm::Attribute::AlwaysInline),
            "Detached IR needs no private core preparation before generic host O3");
  }
  auto resource = std::make_shared<Resource>();
  const auto object = resource->compiler.object(module);
  for (const auto &function : *llvm::unwrap(module.code())) {
    require(!function.hasFnAttribute("dsp56300-io-adapter"),
            "Final shared O3 removes adapter boundaries even after component symbol renaming");
  }
  const auto *const address_first = resource->compiler.install_object(object, module.name());
  const auto *const address_second =
      resource->compiler.resolve(std::string(second.exports().front().name));
  const std::array symbols_first{
      NativeSymbol{.name = copy.exports().front().name, .address = address_first}};
  const std::array symbols_second{
      NativeSymbol{.name = second.exports().front().name, .address = address_second}};
  const NativeSymbols receipt{.owner = resource,
                              .symbols = symbols_first,
                              .identity = copy.identity(),
                              .native_bytes = resource->compiler.native_bytes()};
  rejected(copy, receipt);
  auto native_first = Core::bind(copy, receipt);
  auto native_second = Core::bind(second,
                                  {.owner = resource,
                                   .symbols = symbols_second,
                                   .identity = second.identity(),
                                   .native_bytes = receipt.native_bytes});
  require(resources_created == 1 && resources_destroyed == 0 && receipt.native_bytes != 0 &&
              native_first.report().native_bytes == receipt.native_bytes &&
              native_second.report().native_bytes == receipt.native_bytes,
          "Both components retain one materializer and report its shared footprint without sums");
  return {.first = std::move(native_first), .second = std::move(native_second), .owner = resource};
}

struct Device final : Io {
  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
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

  bool interrupt_pending(Word status) const noexcept override {
    ++calls;
    valid = valid && (status & mask) == 0 && core != nullptr && core->registers().pc == entry + 1 &&
            core->progress().instructions == 1;
    return false;
  }

  std::array<Word, 64> program{}, x{}, y{};
  Core *core = nullptr;
  mutable unsigned calls = 0;
  mutable bool valid = true;
};

std::unique_ptr<Core> instance(Device &device, const bool left, const NativeImage &image) {
  device.program[entry] = static_cast<Word>(isa::Opcode::andi_control) | (clear_mask << 8);
  device.program[entry + 1] = static_cast<Word>(left ? isa::Opcode::asl_a : isa::Opcode::asr_a);
  device.program[entry + 2] = static_cast<Word>(isa::Opcode::stop);
  auto core = std::make_unique<Core>(
      Memory::separate({.program = device.program, .x = device.x, .y = device.y}), &device);
  device.core = core.get();
  Registers initial;
  initial.pc = entry;
  initial.sr = mask;
  initial.a = initial_a;
  initial.m.fill(word_mask);
  core->set_registers(initial);
  core->install(image, 7);
  return core;
}

void execution(Images images) {
  Device first;
  Device second;
  auto one = instance(first, false, images.first);
  auto two = instance(second, true, images.second);
  images.first = {};
  images.second = {};
  require(!images.owner.expired(), "Installed cores retain code after images and producers expire");
  const auto result = one->run(3);
  require(result.fault == Fault{} && result.retired == 3 &&
              result.state == ExecutionState::stopped && one->registers().a == initial_a / 2 &&
              first.calls == 1 && first.valid,
          "The first bound component executes its exact guest result and default public Io");
  const auto prefix = two->run(2);
  require(prefix.fault == Fault{} && prefix.retired == 2 && two->registers().a == initial_a * 2 &&
              second.calls == 1 && second.valid,
          "The second component preserves its separate state and exact Io prefix");
  one.reset();
  require(!images.owner.expired() && resources_destroyed == 0,
          "Destroying one component cannot release the common code owner");
  const auto final = two->run(1);
  require(final.fault == Fault{} && final.retired == 1 && final.state == ExecutionState::stopped,
          "The surviving component remains executable without its original semantic artifact");
  two.reset();
  require(images.owner.expired() && resources_created == 1 && resources_destroyed == 1,
          "The common code resource releases exactly once after its last installed core");
}
}

int main() {
  try {
    const SemanticImage empty;
    require(empty.bitcode().empty() && empty.exports().empty() && empty.identity().empty(),
            "An empty semantic value owns no code, exports or identity");
    execution(materialize());
    std::cout << test::checks << " public lower/bind/shared-owner checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
