// SPDX-License-Identifier: GPL-3.0-only
#include "jit/llvm/runtime.hpp"

#include "jit/llvm/bindings.hpp"
#include "jit/llvm/lifecycle.hpp"
#include "jit/llvm/native.hpp"

#if defined(__CET__)
#include <array>
#endif
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <llvm-c/Analysis.h>
#include <llvm-c/BitReader.h>
#include <llvm-c/BitWriter.h>
#include <llvm-c/Core.h>
#include <llvm-c/Error.h>
#include <llvm-c/LLJIT.h>
#include <llvm-c/Linker.h>
#include <llvm-c/Object.h>
#include <llvm-c/Orc.h>
#include <llvm-c/TargetMachine.h>
#include <llvm-c/Transforms/PassBuilder.h>
#include <llvm-c/Types.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/Object/ObjectFile.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/MemoryBufferRef.h>
#include <llvm/TargetParser/Triple.h>
#include <memory>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {
template <auto dispose> struct LlvmDelete {
  template <typename T> void operator()(T *value) const noexcept { dispose(value); }
};

using Message = std::unique_ptr<char, LlvmDelete<LLVMDisposeMessage>>;
using Buffer = std::unique_ptr<std::remove_pointer_t<LLVMMemoryBufferRef>,
                               LlvmDelete<LLVMDisposeMemoryBuffer>>;
using Module = std::unique_ptr<std::remove_pointer_t<LLVMModuleRef>, LlvmDelete<LLVMDisposeModule>>;
using TargetMachine = std::unique_ptr<std::remove_pointer_t<LLVMTargetMachineRef>,
                                      LlvmDelete<LLVMDisposeTargetMachine>>;

void check(LLVMErrorRef error) {
  if (error == nullptr) { return; }
  const std::unique_ptr<char, LlvmDelete<LLVMDisposeErrorMessage>> text(LLVMGetErrorMessage(error));
  throw std::runtime_error(text.get());
}

void check(llvm::Error error) {
  if (error) { throw std::runtime_error(llvm::toString(std::move(error))); }
}

llvm::orc::LLJIT &native_jit(LLVMOrcLLJITRef jit) noexcept {
  // LLVM 19 OrcV2CBindings.cpp represents this C handle as a direct LLJIT pointer.
  // Use its C++ lifecycle API, which has no corresponding C entry points.
  return *reinterpret_cast<llvm::orc::LLJIT *>(jit);
}

void verify(LLVMModuleRef module) {
  char *message = nullptr;
  const auto invalid = LLVMVerifyModule(module, LLVMReturnStatusAction, &message);
  const Message text(message);
  if (invalid != 0) { throw std::runtime_error(text.get()); }
}

void canonicalize_semantic_entry(LLVMModuleRef module) {
  // Clang's Mach-O asm label carries an LLVM mangling escape.
  for (const auto *name : {"dsp56300_semantic_execute",
                           "dsp56300_semantic_vector_execute",
                           "dsp56300_semantic_memory"}) {
    const auto alias = std::string(1, '\001') + name;
    auto *escaped = LLVMGetNamedFunction(module, alias.c_str());
    if (escaped == nullptr) { continue; }
    if (LLVMGetNamedFunction(module, name) != nullptr) {
      throw std::runtime_error("Ambiguous canonical and escaped DSP semantic entry symbols");
    }
    LLVMSetValueName2(escaped, name, std::string_view(name).size());
  }
}

Buffer buffer(const std::span<const std::byte> bytes) {
  return Buffer(LLVMCreateMemoryBufferWithMemoryRangeCopy(
      reinterpret_cast<const char *>(bytes.data()), bytes.size(), "native_compiler_input"));
}

std::vector<std::byte> bytes(LLVMMemoryBufferRef buffer) {
  const auto *first = reinterpret_cast<const std::byte *>(LLVMGetBufferStart(buffer));
  return {first, first + LLVMGetBufferSize(buffer)};
}

bool absolute_pointer(LLVMValueRef value, std::set<LLVMValueRef> &visited) {
  if (LLVMIsConstant(value) == 0 || !visited.insert(value).second) { return false; }
  if (LLVMIsAConstantExpr(value) != nullptr && LLVMGetConstOpcode(value) == LLVMIntToPtr) {
    auto *integer = LLVMGetOperand(value, 0);
    if (LLVMIsAConstantInt(integer) != nullptr && LLVMIsNull(integer) == 0) { return true; }
  }
  const auto count = LLVMGetNumOperands(value);
  for (int index = 0; index < count; ++index) {
    if (absolute_pointer(LLVMGetOperand(value, static_cast<unsigned>(index)), visited)) {
      return true;
    }
  }
  return false;
}

void relocatable(LLVMModuleRef module) {
  std::set<LLVMValueRef> visited;
  const auto require = [&](LLVMValueRef value) {
    if (absolute_pointer(value, visited)) {
      throw std::runtime_error("Native object contains an absolute host pointer");
    }
  };
  for (auto *global = LLVMGetFirstGlobal(module); global != nullptr;
       global = LLVMGetNextGlobal(global)) {
    require(global);
  }
  for (auto *function = LLVMGetFirstFunction(module); function != nullptr;
       function = LLVMGetNextFunction(function)) {
    for (auto *block = LLVMGetFirstBasicBlock(function); block != nullptr;
         block = LLVMGetNextBasicBlock(block)) {
      for (auto *instruction = LLVMGetFirstInstruction(block); instruction != nullptr;
           instruction = LLVMGetNextInstruction(instruction)) {
        const auto count = LLVMGetNumOperands(instruction);
        if (LLVMGetInstructionOpcode(instruction) == LLVMIntToPtr) {
          auto *integer = LLVMGetOperand(instruction, 0);
          if (LLVMIsAConstantInt(integer) != nullptr && LLVMIsNull(integer) == 0) {
            throw std::runtime_error("Native object contains an absolute host pointer");
          }
        }
        for (int index = 0; index < count; ++index) {
          require(LLVMGetOperand(instruction, static_cast<unsigned>(index)));
        }
      }
    }
  }
}

struct TargetIdentity {
  std::string_view triple, layout;
};

void compatible(LLVMModuleRef module, const TargetIdentity identity) {
  const std::string_view supplied(LLVMGetTarget(module));
  if (!supplied.empty()) {
    const llvm::Triple source(supplied);
    const llvm::Triple target(identity.triple);
    if (source.getArch() != target.getArch() ||
        source.getObjectFormat() != target.getObjectFormat()) {
      throw std::runtime_error("LLVM module target does not match the native compiler");
    }
  }
  const std::string_view data_layout(LLVMGetDataLayoutStr(module));
  if (!data_layout.empty() && data_layout != identity.layout) {
    throw std::runtime_error("LLVM module data layout does not match the native compiler");
  }
}

class Diagnostics {
public:
  explicit Diagnostics(LLVMContextRef context)
      : context_(context),
        previous_(LLVMContextGetDiagnosticHandler(context)),
        previous_owner_(LLVMContextGetDiagnosticContext(context)) {
    LLVMContextSetDiagnosticHandler(context_, receive, this);
  }

  ~Diagnostics() { LLVMContextSetDiagnosticHandler(context_, previous_, previous_owner_); }

  Diagnostics(const Diagnostics &) = delete;
  Diagnostics &operator=(const Diagnostics &) = delete;

  void check(const bool failed, const char *fallback) const {
    if (exception_) { std::rethrow_exception(exception_); }
    if (failed || diagnosed_) { throw std::runtime_error(text_.empty() ? fallback : text_); }
  }

private:
  static void receive(LLVMDiagnosticInfoRef info, void *owner) noexcept {
    const auto severity = LLVMGetDiagInfoSeverity(info);
    if (severity != LLVMDSWarning && severity != LLVMDSError) { return; }
    const auto &self = *static_cast<const Diagnostics *>(owner);
    self.diagnosed_ = true;
    try {
      const Message message(LLVMGetDiagInfoDescription(info));
      if (message) { self.text_ += message.get(); }
    } catch (...) { self.exception_ = std::current_exception(); }
  }

  LLVMContextRef context_;
  LLVMDiagnosticHandler previous_;
  void *previous_owner_;
  mutable std::string text_;
  mutable std::exception_ptr exception_;
  mutable bool diagnosed_ = false;
};

ImageLifecycle compatible_object(LLVMMemoryBufferRef input,
                                 const char *native_triple,
                                 const std::string_view entry,
                                 const char prefix) {
  const llvm::StringRef data(LLVMGetBufferStart(input), LLVMGetBufferSize(input));
  auto object = llvm::object::ObjectFile::createObjectFile(
      llvm::MemoryBufferRef(data, "native_compiler_object"));
  if (!object) { throw std::runtime_error(llvm::toString(object.takeError())); }
  const auto source = (*object)->makeTriple();
  const llvm::Triple target(native_triple);
  if (source.getArch() != target.getArch() ||
      source.getObjectFormat() != target.getObjectFormat() ||
      (*object)->getBytesInAddress() != sizeof(void *) ||
      (*object)->isLittleEndian() != (std::endian::native == std::endian::little)) {
    throw std::runtime_error("LLVM object target does not match the native compiler");
  }
  return object_lifecycle(**object, entry, prefix);
}

TargetMachine native_machine() {
  const Message triple(LLVMGetDefaultTargetTriple());
  const Message cpu(LLVMGetHostCPUName());
  const Message features(LLVMGetHostCPUFeatures());
  LLVMTargetRef target = nullptr;
  char *message = nullptr;
  const auto invalid = LLVMGetTargetFromTriple(triple.get(), &target, &message);
  const Message text(message);
  if (invalid != 0) { throw std::runtime_error(text.get()); }
  TargetMachine result(LLVMCreateTargetMachine(target,
                                               triple.get(),
                                               cpu.get(),
                                               features.get(),
                                               LLVMCodeGenLevelAggressive,
                                               LLVMRelocPIC,
                                               LLVMCodeModelSmall));
  if (!result) { throw std::runtime_error("LLVM 19 cannot create the native target machine"); }
  return result;
}
}

struct LlvmCompiler::Impl {
  static LLVMErrorRef object(void *owner, LLVMMemoryBufferRef *buffer) noexcept {
    char *message = nullptr;
    const std::unique_ptr<std::remove_pointer_t<LLVMBinaryRef>, LlvmDelete<LLVMDisposeBinary>>
        binary(LLVMCreateBinary(*buffer, nullptr, &message));
    const Message error(message);
    if (!binary) {
      LLVMDisposeMemoryBuffer(*buffer);
      *buffer = nullptr;
      return LLVMCreateStringError(error.get());
    }
    const std::unique_ptr<std::remove_pointer_t<LLVMSectionIteratorRef>,
                          LlvmDelete<LLVMDisposeSectionIterator>>
        sections(LLVMObjectFileCopySectionIterator(binary.get()));
    auto &self = *static_cast<Impl *>(owner);
    if (!sections) { return nullptr; }
    while (LLVMObjectFileIsSectionIteratorAtEnd(binary.get(), sections.get()) == 0) {
      const auto *section_name = LLVMGetSectionName(sections.get());
      const std::string_view name(section_name == nullptr ? "" : section_name);
      if (name == ".text" || name.starts_with(".text.") || name == "__text" || name == ".ltext" ||
          name.starts_with(".ltext.")) {
        self.text_bytes += LLVMGetSectionSize(sections.get());
      }
      LLVMMoveToNextSection(sections.get());
    }
    return nullptr;
  }

  ~Impl() {
    if (jit == nullptr) { return; }
    auto &native = native_jit(jit);
    if (auto error = native.deinitialize(native.getMainJITDylib())) {
      std::cerr << "LLVM JIT finalization failed: " << llvm::toString(std::move(error)) << '\n';
      std::terminate();
    }
    if (auto *error = LLVMOrcDisposeLLJIT(jit)) {
      const std::unique_ptr<char, LlvmDelete<LLVMDisposeErrorMessage>> text(
          LLVMGetErrorMessage(error));
      std::cerr << "LLVM JIT disposal failed: " << text.get() << '\n';
      std::terminate();
    }
  }

  void prepare(LlvmModule &module) const;

  [[nodiscard]] const void *lookup(const std::string &name) const {
    LLVMOrcExecutorAddress entry = 0;
    check(LLVMOrcLLJITLookup(jit, &entry, name.c_str()));
    auto &native = native_jit(jit);
    check(native.initialize(native.getMainJITDylib()));
    return std::bit_cast<const void *>(static_cast<std::uintptr_t>(entry));
  }

  LLVMOrcLLJITRef jit = nullptr;
  TargetMachine machine;
  std::uint64_t text_bytes = 0;
};

LlvmModule::LlvmModule(std::string name) : name_(std::move(name)) {
  context_ = LLVMOrcCreateNewThreadSafeContext();
  module_ = LLVMModuleCreateWithNameInContext(name_.c_str(), context());
#if defined(__CET__)
  for (const auto flag_name :
       std::array<std::string_view, 2>{"cf-protection-branch", "cf-protection-return"}) {
    LLVMAddModuleFlag(module_,
                      LLVMModuleFlagBehaviorOverride,
                      flag_name.data(),
                      flag_name.size(),
                      LLVMValueAsMetadata(LLVMConstInt(LLVMInt32TypeInContext(context()), 1, 0)));
  }
#endif
}

LlvmModule::~LlvmModule() {
  if (module_ != nullptr) { LLVMDisposeModule(module_); }
  LLVMOrcDisposeThreadSafeContext(context_);
}

LLVMContextRef LlvmModule::context() const noexcept {
  return LLVMOrcThreadSafeContextGetContext(context_);
}

std::vector<std::byte> LlvmModule::bitcode() const {
  if (module_ == nullptr) { throw std::logic_error("LLVM module was already transferred"); }
  verify(module_);
  const Buffer output(LLVMWriteBitcodeToMemoryBuffer(module_));
  if (!output) { throw std::runtime_error("LLVM 19 cannot export semantic bitcode"); }
  return bytes(output.get());
}

LLVMModuleRef LlvmModule::parse_bitcode(const std::span<const std::byte> source) const {
  if (module_ == nullptr) { throw std::logic_error("LLVM module was already transferred"); }
  const Diagnostics diagnostics(context());
  const auto input = buffer(source);
  LLVMModuleRef raw = nullptr;
  const auto failed = LLVMParseBitcodeInContext2(context(), input.get(), &raw);
  Module parsed(raw);
  try {
    diagnostics.check(failed != 0 || !parsed, "LLVM 19 cannot parse callback bitcode");
  } catch (const std::runtime_error &error) { throw std::invalid_argument(error.what()); }
  return parsed.release();
}

void LlvmModule::link_bitcode(const std::span<const std::byte> source) {
  if (module_ == nullptr) { throw std::logic_error("LLVM module was already transferred"); }
  LlvmModule candidate(name_);
  const Diagnostics diagnostics(candidate.context());
  const auto original = buffer(bitcode());
  LLVMModuleRef parsed = nullptr;
  auto failed = LLVMParseBitcodeInContext2(candidate.context(), original.get(), &parsed);
  Module retained(parsed);
  diagnostics.check(failed != 0 || !retained, "LLVM 19 cannot retain compiler bitcode");
  LLVMDisposeModule(std::exchange(candidate.module_, retained.release()));
  const auto input = buffer(source);
  parsed = nullptr;
  failed = LLVMParseBitcodeInContext2(candidate.context(), input.get(), &parsed);
  Module incoming(parsed);
  diagnostics.check(failed != 0 || !incoming, "LLVM 19 cannot parse semantic bitcode");
  verify(incoming.get());
  canonicalize_semantic_entry(incoming.get());
  const std::string_view target(LLVMGetTarget(candidate.module_));
  const std::string_view layout(LLVMGetDataLayoutStr(candidate.module_));
  if (!target.empty() || !layout.empty()) {
    compatible(incoming.get(),
               {.triple = target.empty() ? LLVMGetTarget(incoming.get()) : target,
                .layout = layout.empty() ? LLVMGetDataLayoutStr(incoming.get()) : layout});
  }
  if (!target.empty() && std::string_view(LLVMGetTarget(incoming.get())).empty()) {
    LLVMSetTarget(incoming.get(), LLVMGetTarget(candidate.module_));
  }
  if (!layout.empty() && std::string_view(LLVMGetDataLayoutStr(incoming.get())).empty()) {
    LLVMSetDataLayout(incoming.get(), LLVMGetDataLayoutStr(candidate.module_));
  }
  diagnostics.check(LLVMLinkModules2(candidate.module_, incoming.release()) != 0,
                    "LLVM 19 cannot link semantic bitcode");
  LLVMSetModuleIdentifier(candidate.module_, name_.data(), name_.size());
  verify(candidate.module_);
  std::swap(module_, candidate.module_);
  std::swap(context_, candidate.context_);
}

LLVMOrcThreadSafeModuleRef LlvmModule::release() noexcept {
  return LLVMOrcCreateNewThreadSafeModule(std::exchange(module_, nullptr), context_);
}

LlvmCompiler::LlvmCompiler() : impl_(std::make_unique<Impl>()) {
  weichwolf::llvm19::initialize_native();
  impl_->machine = native_machine();
  auto machine = native_machine();
  auto *builder = LLVMOrcCreateLLJITBuilder();
  LLVMOrcLLJITBuilderSetJITTargetMachineBuilder(
      builder, LLVMOrcJITTargetMachineBuilderCreateFromTargetMachine(machine.release()));
  check(LLVMOrcCreateLLJIT(&impl_->jit, builder));
  LLVMOrcObjectTransformLayerSetTransform(
      LLVMOrcLLJITGetObjTransformLayer(impl_->jit), Impl::object, impl_.get());
  LLVMOrcDefinitionGeneratorRef symbols = nullptr;
  check(LLVMOrcCreateDynamicLibrarySearchGeneratorForProcess(
      &symbols, LLVMOrcLLJITGetGlobalPrefix(impl_->jit), nullptr, nullptr));
  LLVMOrcJITDylibAddGenerator(LLVMOrcLLJITGetMainJITDylib(impl_->jit), symbols);
}

LlvmCompiler::~LlvmCompiler() = default;

void LlvmCompiler::define(const std::string &name, const void *address) {
  if (name.empty() || address == nullptr) {
    throw std::invalid_argument("A native binding needs a name and function address");
  }
  LLVMOrcCSymbolMapPair symbol{.Name = LLVMOrcLLJITMangleAndIntern(impl_->jit, name.c_str()),
                               .Sym = {.Address = std::bit_cast<std::uintptr_t>(address),
                                       .Flags = {.GenericFlags = LLVMJITSymbolGenericFlagsExported |
                                                                 LLVMJITSymbolGenericFlagsCallable,
                                                 .TargetFlags = 0}}};
  auto *definition = LLVMOrcAbsoluteSymbols(&symbol, 1);
  auto *error = LLVMOrcJITDylibDefine(LLVMOrcLLJITGetMainJITDylib(impl_->jit), definition);
  if (error != nullptr) {
    LLVMOrcDisposeMaterializationUnit(definition);
    check(error);
  }
}

std::uint64_t LlvmCompiler::native_bytes() const noexcept {
  return impl_->text_bytes;
}

void LlvmCompiler::Impl::prepare(LlvmModule &module) const {
  if (module.code() == nullptr) { throw std::logic_error("LLVM module was already transferred"); }
  compatible(
      module.code(),
      {.triple = LLVMOrcLLJITGetTripleString(jit), .layout = LLVMOrcLLJITGetDataLayoutStr(jit)});
  finalize_io(module);
  LLVMSetTarget(module.code(), LLVMOrcLLJITGetTripleString(jit));
  LLVMSetDataLayout(module.code(), LLVMOrcLLJITGetDataLayoutStr(jit));
  constexpr bool address_sanitizer = __has_feature(address_sanitizer);
  if constexpr (address_sanitizer) {
    constexpr std::string_view name = "sanitize_address";
    const auto kind = LLVMGetEnumAttributeKindForName(name.data(), name.size());
    auto *const attribute = LLVMCreateEnumAttribute(module.context(), kind, 0);
    for (auto *function = LLVMGetFirstFunction(module.code()); function != nullptr;
         function = LLVMGetNextFunction(function)) {
      if (LLVMIsDeclaration(function) == 0) {
        LLVMAddAttributeAtIndex(
            function, static_cast<LLVMAttributeIndex>(LLVMAttributeFunctionIndex), attribute);
      }
    }
  }
  verify(module.code());
  const std::unique_ptr<std::remove_pointer_t<LLVMPassBuilderOptionsRef>,
                        LlvmDelete<LLVMDisposePassBuilderOptions>>
      options(LLVMCreatePassBuilderOptions());
  check(LLVMRunPasses(module.code(),
                      address_sanitizer ? "default<O3>,asan" : "default<O3>",
                      machine.get(),
                      options.get()));
  verify(module.code());
}

const void *LlvmCompiler::install(LlvmModule &module) {
  impl_->prepare(module);
  check(LLVMOrcLLJITAddLLVMIRModule(
      impl_->jit, LLVMOrcLLJITGetMainJITDylib(impl_->jit), module.release()));
  return impl_->lookup(module.name());
}

const void *LlvmCompiler::resolve(const std::string &name) const {
  return impl_->lookup(name);
}

std::vector<std::byte> LlvmCompiler::object(LlvmModule &module) {
  if (module.code() == nullptr) { throw std::logic_error("LLVM module was already transferred"); }
  relocatable(module.code());
  impl_->prepare(module);
  export_lifecycle(module);
  verify(module.code());
  relocatable(module.code());
  char *message = nullptr;
  LLVMMemoryBufferRef exported = nullptr;
  const auto failed = LLVMTargetMachineEmitToMemoryBuffer(
      impl_->machine.get(), module.code(), LLVMObjectFile, &message, &exported);
  const Message error(message);
  const Buffer output(exported);
  if (failed != 0 || !output) {
    throw std::runtime_error(error ? error.get() : "LLVM 19 cannot export native code");
  }
  return bytes(output.get());
}

const void *LlvmCompiler::install_object(const std::span<const std::byte> code,
                                         const std::string &entry) {
  if constexpr (__has_feature(address_sanitizer)) {
    throw std::invalid_argument(
        "ASan native object installation is unsupported; use IR installation");
  }
  auto input = buffer(code);
  const auto lifecycle = compatible_object(input.get(),
                                           LLVMOrcLLJITGetTripleString(impl_->jit),
                                           entry,
                                           LLVMOrcLLJITGetGlobalPrefix(impl_->jit));
  check(LLVMOrcLLJITAddObjectFile(
      impl_->jit, LLVMOrcLLJITGetMainJITDylib(impl_->jit), input.release()));
  if (lifecycle.initialize || lifecycle.finalize) {
    LlvmModule shim(entry + ".lifetime");
    bind_lifecycle(shim, lifecycle, entry);
    impl_->prepare(shim);
    check(LLVMOrcLLJITAddLLVMIRModule(
        impl_->jit, LLVMOrcLLJITGetMainJITDylib(impl_->jit), shim.release()));
  }
  return impl_->lookup(entry);
}
}
