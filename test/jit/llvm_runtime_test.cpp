// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/runtime.hpp"
#include "llvm_owner.hpp"
#include "support.hpp"

#include <array>
#include <barrier>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#if defined(__APPLE__)
#include <llvm/BinaryFormat/MachO.h>
#else
#include <llvm/BinaryFormat/ELF.h>
#endif
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace {
using dsp56300::detail::LlvmCompiler;
using dsp56300::detail::LlvmModule;
using dsp56300::test::require;
using Function = std::uint32_t (*)(std::uint32_t) noexcept;
constexpr std::uint32_t mask = 0xa5a5a5a5;

struct DisposeBuilder {
  void operator()(LLVMBuilderRef builder) const noexcept { LLVMDisposeBuilder(builder); }
};

void lift(LlvmModule &module) {
  auto *const context = module.context();
  auto *const word = LLVMInt32TypeInContext(context);
  std::array types{word};
  auto *const type = LLVMFunctionType(word, types.data(), 1, 0);
  auto *function = LLVMAddFunction(module.code(), module.name().c_str(), type);
  auto *block = LLVMAppendBasicBlockInContext(context, function, "entry");
  const std::unique_ptr<std::remove_pointer_t<LLVMBuilderRef>, DisposeBuilder> builder(
      LLVMCreateBuilderInContext(context));
  LLVMPositionBuilderAtEnd(builder.get(), block);
  auto *input = LLVMGetParam(function, 0);
  auto *offset = LLVMConstInt(word, mask, 0);
  auto *first = LLVMBuildAdd(builder.get(), input, offset, "first");
  auto *second = LLVMBuildSub(builder.get(), first, offset, "second");
  auto *result = LLVMBuildXor(builder.get(), second, offset, "result");
  LLVMBuildRet(builder.get(), result);
}

Function install(LlvmCompiler &compiler, const std::string &name) {
  LlvmModule module(name);
  lift(module);
  const auto function = std::bit_cast<Function>(compiler.install(module));
  require(module.code() == nullptr, "Installation transfers one immutable module owner");
  bool refused = false;
  try {
    static_cast<void>(compiler.install(module));
  } catch (const std::logic_error &) { refused = true; }
  require(refused, "Transferred modules cannot be installed again");
  return function;
}

void check(const Function function) {
  constexpr std::uint32_t domain = 65536;
  for (std::uint32_t value = 0; value < domain; ++value) {
    require(function(value) == (value ^ mask), "O3 native result differs from independent XOR");
  }
  constexpr auto maximum = std::numeric_limits<std::uint32_t>::max();
  for (const auto value : {maximum, maximum - 1, maximum / 2, (maximum / 2) + 1, mask}) {
    require(function(value) == (value ^ mask), "Fixed-width native wrap result is incorrect");
  }
}

void failure_ownership(LlvmCompiler &compiler, const Function retained) {
  LlvmModule invalid("invalid");
  auto *const type = LLVMFunctionType(LLVMInt32TypeInContext(invalid.context()), nullptr, 0, 0);
  auto *function = LLVMAddFunction(invalid.code(), "invalid", type);
  LLVMAppendBasicBlockInContext(invalid.context(), function, "unterminated");
  bool rejected = false;
  try {
    static_cast<void>(compiler.install(invalid));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected && invalid.code() != nullptr,
          "Verifier failure retains caller ownership before any module transfer");
  LlvmModule missing("missing");
  rejected = false;
  try {
    static_cast<void>(compiler.install(missing));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected && missing.code() == nullptr,
          "Lookup failure after transfer retains resources in the compiler owner");
  check(retained);
  check(install(compiler, "after_failure"));
}

void bitcode_linking(LlvmModule &module) {
  const auto before = module.bitcode();
  constexpr std::array corrupt{std::byte{1}};
  for (const auto input :
       {std::span<const std::byte>(corrupt), std::span<const std::byte>(before)}) {
    bool rejected = false;
    try {
      module.link_bitcode(input);
    } catch (const std::runtime_error &) { rejected = true; }
    require(rejected && module.bitcode() == before,
            "Malformed or duplicate bitcode retains the complete original compiler module");
  }
}

void rejected_objects(LlvmCompiler &compiler, const std::span<const std::byte> object) {
  auto foreign = std::vector<std::byte>(object.begin(), object.end());
  constexpr unsigned byte_mask = 0xff;
#if defined(__APPLE__)
  constexpr unsigned machine_offset = 4;
  constexpr auto other_machine = llvm::MachO::CPU_TYPE_X86_64;
#else
  constexpr unsigned machine_offset = 18;
  constexpr auto other_machine = llvm::ELF::EM_AARCH64;
#endif
  foreign.at(machine_offset) = static_cast<std::byte>(other_machine & byte_mask);
  foreign.at(machine_offset + 1) = static_cast<std::byte>((other_machine >> 8U) & byte_mask);
  constexpr std::array corrupt{std::byte{1}};
  for (const auto input :
       {std::span<const std::byte>(corrupt), std::span<const std::byte>(foreign)}) {
    bool rejected = false;
    try {
      static_cast<void>(compiler.install_object(input, "rejected"));
    } catch (const std::runtime_error &) { rejected = true; }
    require(rejected && compiler.native_bytes() == 0,
            "Malformed or foreign native objects fail before materialization");
  }
}

void exported_code() {
  LlvmCompiler compiler;
  LlvmModule original("exported_entry");
  lift(original);
  const auto serialized = original.bitcode();
  LlvmModule imported(original.name());
  imported.link_bitcode(serialized);
  bitcode_linking(imported);
  const auto object = compiler.object(imported);
  require(!object.empty() && original.code() != nullptr && imported.code() != nullptr &&
              compiler.native_bytes() == 0,
          "Bitcode and object export retain both owners without materializing live code");
  rejected_objects(compiler, object);
  const auto first = std::bit_cast<Function>(compiler.install_object(object, original.name()));
  require(compiler.native_bytes() != 0, "Cold installation accounts actual native text");
  check(first);
  LlvmCompiler independent;
  check(std::bit_cast<Function>(independent.install_object(object, original.name())));
  check(first);
  LlvmModule incompatible("incompatible");
  lift(incompatible);
  LLVMSetDataLayout(incompatible.code(), "E-p:64:64");
  bool rejected = false;
  try {
    static_cast<void>(compiler.object(incompatible));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected && incompatible.code() != nullptr,
          "An incompatible native layout fails before transferring compiler ownership");
  check(first);
}

std::uint32_t host_calls = 0;

std::uint32_t host_boundary(const std::uint32_t value) noexcept {
  ++host_calls;
  return value ^ mask;
}

void lift_boundary(LlvmModule &module) {
  auto *const context = module.context();
  std::array types{LLVMInt32TypeInContext(context)};
  auto *type = LLVMFunctionType(types.front(), types.data(), types.size(), 0);
  auto *host = LLVMAddFunction(module.code(), "native_host_boundary", type);
  auto *function = LLVMAddFunction(module.code(), module.name().c_str(), type);
  auto *entry = LLVMAppendBasicBlockInContext(context, function, "entry");
  const std::unique_ptr<std::remove_pointer_t<LLVMBuilderRef>, DisposeBuilder> builder(
      LLVMCreateBuilderInContext(context));
  LLVMPositionBuilderAtEnd(builder.get(), entry);
  std::array values{LLVMGetParam(function, 0)};
  LLVMBuildRet(
      builder.get(),
      LLVMBuildCall2(builder.get(), type, host, values.data(), values.size(), "host_result"));
}

void named_boundaries() {
  LlvmCompiler compiler;
  LlvmModule module("bound_entry");
  lift_boundary(module);
  const auto object = compiler.object(module);
  require(!module.bitcode().empty(), "Native emission retains valid optimized bitcode");
  const auto *address = std::bit_cast<const void *>(&host_boundary);
  constexpr std::array values{0U, 1U, mask, std::numeric_limits<std::uint32_t>::max()};
  compiler.define("native_host_boundary", address);
  const auto bound = std::bit_cast<Function>(compiler.install_object(object, module.name()));
  const auto before = host_calls;
  for (const auto value : values) {
    require(bound(value) == (value ^ mask), "A named native boundary receives exact arguments");
  }
  require(host_calls - before == values.size(), "Native calls preserve every host side effect");
  LlvmCompiler independent;
  independent.define("native_host_boundary", address);
  const auto function = std::bit_cast<Function>(independent.install_object(object, module.name()));
  require(function(mask) == 0, "The same relocatable object binds independently in another owner");
  bool rejected = false;
  try {
    independent.define("native_host_boundary", address);
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected && function(0) == mask, "Duplicate binding retains the existing native target");
}

void bound_pointer_rejection() {
  LlvmCompiler compiler;
  LlvmModule module("instance_pointer");
  auto *context = module.context();
  auto *type = LLVMFunctionType(LLVMInt32TypeInContext(context), nullptr, 0, 0);
  auto *function = LLVMAddFunction(module.code(), module.name().c_str(), type);
  auto *entry = LLVMAppendBasicBlockInContext(context, function, "entry");
  const std::unique_ptr<std::remove_pointer_t<LLVMBuilderRef>, DisposeBuilder> builder(
      LLVMCreateBuilderInContext(context));
  LLVMPositionBuilderAtEnd(builder.get(), entry);
  auto *pointer = LLVMConstIntToPtr(LLVMConstInt(LLVMInt64TypeInContext(context), 4096, 0),
                                    LLVMPointerTypeInContext(context, 0));
  LLVMBuildRet(
      builder.get(),
      LLVMBuildLoad2(builder.get(), LLVMInt32TypeInContext(context), pointer, "captured_instance"));
  const auto before = module.bitcode();
  bool rejected = false;
  try {
    static_cast<void>(compiler.object(module));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected && module.bitcode() == before && compiler.native_bytes() == 0,
          "An absolute instance pointer cannot enter a relocatable artifact");
}

void independent_owners() {
  require(dsp56300::test::llvm_flag_a() == dsp56300::test::llvm_flag_b(),
          "Separate static core archives must share exactly one target initializer flag");
  std::array<std::unique_ptr<LlvmCompiler>, 2> owners;
  std::array<std::exception_ptr, 2> failures;
  std::barrier gate(2);
  const auto create = [&](const unsigned index) {
    gate.arrive_and_wait();
    try {
      owners[index] = index == 0 ? dsp56300::test::llvm_owner_a() : dsp56300::test::llvm_owner_b();
    } catch (...) { failures[index] = std::current_exception(); }
  };
  {
    const std::jthread first(create, 0);
    const std::jthread second(create, 1);
  }
  for (const auto &failure : failures) {
    if (failure) { std::rethrow_exception(failure); }
  }
  const auto first = install(*owners[0], "same_entry");
  const auto second = install(*owners[1], "same_entry");
  check(first);
  check(second);
  failure_ownership(*owners[1], second);
  owners[0].reset();
  check(second);
}
}

int main() try {
  independent_owners();
  exported_code();
  named_boundaries();
  bound_pointer_rejection();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
