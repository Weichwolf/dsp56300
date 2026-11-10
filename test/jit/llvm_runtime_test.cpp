// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/runtime.hpp"
#include "llvm_owner.hpp"
#include "support.hpp"

#include <array>
#include <barrier>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

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

int main() try { independent_owners(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
