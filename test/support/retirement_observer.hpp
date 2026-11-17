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
  std::uint64_t last_site = 0, batched = 0, deferred = 0, unit_entries = 0;
  void *yield_owner = nullptr;
  void (*yield)(void *) noexcept = nullptr;
  bool yield_at_entry = false;

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

inline void retirement_yield(LLVMBuilderRef ir,
                             LLVMValueRef function,
                             const RetirementIo &probe,
                             LLVMValueRef is_entry,
                             LLVMBasicBlockRef done) {
  auto *context = LLVMGetModuleContext(LLVMGetGlobalParent(function));
  auto *owner = LLVMGetParam(function, 0);
  auto *byte = LLVMInt8TypeInContext(context);
  auto *wide = LLVMInt64TypeInContext(context);
  auto *pointer = LLVMPointerTypeInContext(context, 0);
  const auto member = [&](const auto *field) {
    auto *displacement =
        LLVMConstInt(wide,
                     reinterpret_cast<std::uintptr_t>(field) -
                         reinterpret_cast<std::uintptr_t>(static_cast<const Io *>(&probe)),
                     0);
    return LLVMBuildGEP2(ir, byte, owner, &displacement, 1, "yield.field");
  };
  auto *target = LLVMBuildLoad2(ir, pointer, member(&probe.yield), "yield.target");
  auto *at_entry = LLVMBuildLoad2(ir, byte, member(&probe.yield_at_entry), "yield.at.entry");
  auto *selected_entry =
      LLVMBuildICmp(ir, LLVMIntNE, at_entry, LLVMConstInt(byte, 0, 0), "entry.selected");
  auto *selected = LLVMBuildICmp(ir, LLVMIntEQ, is_entry, selected_entry, "yield.selected");
  auto *invoke = LLVMAppendBasicBlockInContext(context, function, "yield");
  LLVMBuildCondBr(
      ir,
      LLVMBuildAnd(ir, selected, LLVMBuildIsNotNull(ir, target, "yield.enabled"), "yield.ready"),
      invoke,
      done);
  LLVMPositionBuilderAtEnd(ir, invoke);
  auto *argument = LLVMBuildLoad2(ir, pointer, member(&probe.yield_owner), "yield.owner");
  auto *yield_type = LLVMFunctionType(LLVMVoidTypeInContext(context), &pointer, 1, 0);
  LLVMBuildCall2(ir, yield_type, target, &argument, 1, "");
  LLVMBuildBr(ir, done);
}

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
  auto *entry_offset = LLVMConstInt(wide,
                                    reinterpret_cast<std::uintptr_t>(&probe.unit_entries) -
                                        reinterpret_cast<std::uintptr_t>(static_cast<Io *>(&probe)),
                                    0);
  auto *entry_address = LLVMBuildGEP2(ir.get(), byte, owner, &entry_offset, 1, "unit.entries");
  auto *entry_count = LLVMBuildLoad2(ir.get(), wide, entry_address, "previous.entries");
  constexpr unsigned unit_entry_shift = 26;
  auto *entry_bit = LLVMBuildAnd(ir.get(),
                                 LLVMGetParam(function, 3),
                                 LLVMConstInt(wide, std::uint64_t{1} << unit_entry_shift, 0),
                                 "entry.bit");
  auto *is_entry = LLVMBuildAnd(
      ir.get(),
      LLVMGetParam(function, 2),
      LLVMBuildICmp(ir.get(), LLVMIntNE, entry_bit, LLVMConstInt(wide, 0, 0), "entry.marker"),
      "is.entry");
  LLVMBuildStore(
      ir.get(),
      LLVMBuildAdd(
          ir.get(), entry_count, LLVMBuildZExt(ir.get(), is_entry, wide, "entered"), "entries"),
      entry_address);
  constexpr unsigned batch_shift = 24;
  constexpr unsigned deferred_shift = 25;
  for (const auto [counter, shift] :
       {std::pair{&probe.batched, batch_shift}, std::pair{&probe.deferred, deferred_shift}}) {
    auto *bit = LLVMBuildAnd(ir.get(),
                             LLVMGetParam(function, 3),
                             LLVMConstInt(wide, std::uint64_t{1} << shift, 0),
                             "property.bit");
    auto *selected = LLVMBuildAnd(
        ir.get(),
        LLVMGetParam(function, 2),
        LLVMBuildICmp(ir.get(), LLVMIntNE, bit, LLVMConstInt(wide, 0, 0), "is.property"),
        "property");
    auto *property_offset =
        LLVMConstInt(wide,
                     reinterpret_cast<std::uintptr_t>(counter) -
                         reinterpret_cast<std::uintptr_t>(static_cast<Io *>(&probe)),
                     0);
    auto *property_address =
        LLVMBuildGEP2(ir.get(), byte, owner, &property_offset, 1, "property.counter");
    auto *property_previous = LLVMBuildLoad2(ir.get(), wide, property_address, "previous.property");
    auto *increment =
        LLVMBuildSelect(ir.get(), selected, count, LLVMConstInt(wide, 0, 0), "property.increment");
    LLVMBuildStore(ir.get(),
                   LLVMBuildAdd(ir.get(), property_previous, increment, "retired.property"),
                   property_address);
  }
  retirement_yield(ir.get(), function, probe, is_entry, done);
  LLVMPositionBuilderAtEnd(ir.get(), done);
  LLVMBuildRetVoid(ir.get());
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> bytes(
      LLVMWriteBitcodeToMemoryBuffer(module.get()));
  const auto *first = reinterpret_cast<const std::byte *>(LLVMGetBufferStart(bytes.get()));
  return {first, first + LLVMGetBufferSize(bytes.get())};
}

}
