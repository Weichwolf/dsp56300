// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "image_compiler.hpp"

#include "llvm-c/BitWriter.h"
#include "llvm-c/Core.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test {
struct RetirementIo final : Io {
  std::array<std::uint64_t, 2> counts{};
  std::uint64_t last_site = 0;

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
};

inline std::vector<std::byte> retirement_binding() {
  using test::image_compiler::Owner;
  const Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
  const Owner<LLVMModuleRef, LLVMDisposeModule> module(
      LLVMModuleCreateWithNameInContext("retirement-consumer", context.get()));
  const Owner<LLVMBuilderRef, LLVMDisposeBuilder> ir(LLVMCreateBuilderInContext(context.get()));
  auto *byte = LLVMInt8TypeInContext(context.get());
  auto *wide = LLVMInt64TypeInContext(context.get());
  std::array parameters{LLVMPointerTypeInContext(context.get(), 0),
                        LLVMInt32TypeInContext(context.get()),
                        LLVMInt1TypeInContext(context.get()),
                        LLVMInt64TypeInContext(context.get())};
  auto *type = LLVMFunctionType(
      LLVMVoidTypeInContext(context.get()), parameters.data(), parameters.size(), 0);
  auto *function = LLVMAddFunction(module.get(), "record_retired", type);
  for (const auto [name, index] :
       {std::pair{"nounwind", static_cast<LLVMAttributeIndex>(LLVMAttributeFunctionIndex)},
        std::pair{"zeroext", 3U}}) {
    LLVMAddAttributeAtIndex(function,
                            index,
                            LLVMCreateEnumAttribute(context.get(),
                                                    LLVMGetEnumAttributeKindForName(
                                                        name, std::string_view(name).size()),
                                                    0));
  }
  auto *entry = LLVMAppendBasicBlockInContext(context.get(), function, "entry");
  auto *update = LLVMAppendBasicBlockInContext(context.get(), function, "update");
  auto *done = LLVMAppendBasicBlockInContext(context.get(), function, "done");
  LLVMPositionBuilderAtEnd(ir.get(), entry);
  auto *owner = LLVMGetParam(function, 0);
  LLVMBuildCondBr(ir.get(), LLVMBuildIsNull(ir.get(), owner, "absent"), done, update);
  LLVMPositionBuilderAtEnd(ir.get(), update);
  RetirementIo probe;
  const auto offset = reinterpret_cast<std::uintptr_t>(&probe.counts) -
                      reinterpret_cast<std::uintptr_t>(static_cast<Io *>(&probe));
  auto *index = LLVMBuildSelect(ir.get(),
                                LLVMGetParam(function, 2),
                                LLVMConstInt(wide, offset, 0),
                                LLVMConstInt(wide, offset + sizeof(std::uint64_t), 0),
                                "counter.offset");
  auto *address = LLVMBuildGEP2(ir.get(), byte, owner, &index, 1, "counter");
  auto *value = LLVMBuildLoad2(ir.get(), wide, address, "previous");
  auto *count = LLVMBuildZExt(ir.get(), LLVMGetParam(function, 1), wide, "count");
  LLVMBuildStore(ir.get(), LLVMBuildAdd(ir.get(), value, count, "retired"), address);
  auto *site_offset = LLVMConstInt(wide,
                                   reinterpret_cast<std::uintptr_t>(&probe.last_site) -
                                       reinterpret_cast<std::uintptr_t>(static_cast<Io *>(&probe)),
                                   0);
  auto *site_address = LLVMBuildGEP2(ir.get(), byte, owner, &site_offset, 1, "last.site");
  auto *previous_site = LLVMBuildLoad2(ir.get(), wide, site_address, "previous.site");
  auto *site = LLVMBuildSelect(ir.get(),
                               LLVMGetParam(function, 2),
                               previous_site,
                               LLVMGetParam(function, 3),
                               "precise.site");
  LLVMBuildStore(ir.get(), site, site_address);
  LLVMBuildBr(ir.get(), done);
  LLVMPositionBuilderAtEnd(ir.get(), done);
  LLVMBuildRetVoid(ir.get());
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> bytes(
      LLVMWriteBitcodeToMemoryBuffer(module.get()));
  const auto *first = reinterpret_cast<const std::byte *>(LLVMGetBufferStart(bytes.get()));
  return {first, first + LLVMGetBufferSize(bytes.get())};
}

}
