// SPDX-License-Identifier: GPL-3.0-only
#include "support.hpp"

#include <cstddef>
#include <exception>
#include <iostream>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/BinaryFormat/Magic.h>
#include <llvm/Object/Archive.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/MemoryBufferRef.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
using dsp56300::test::require;

using Buffer = std::unique_ptr<LLVMOpaqueMemoryBuffer, decltype(&LLVMDisposeMemoryBuffer)>;

Buffer read(const char *path) {
  LLVMMemoryBufferRef value = nullptr;
  char *message = nullptr;
  if (LLVMCreateMemoryBufferWithContentsOfFile(path, &value, &message) != 0) {
    const std::string error(message);
    LLVMDisposeMessage(message);
    throw std::runtime_error(error);
  }
  return {value, LLVMDisposeMemoryBuffer};
}

void verify(const char *path) {
  const auto source = read(path);
  const llvm::StringRef bytes(LLVMGetBufferStart(source.get()), LLVMGetBufferSize(source.get()));
  require(llvm::identify_magic(bytes) == llvm::file_magic::archive,
          "The consumer receives a static core archive");
  auto error = llvm::Error::success();
  const llvm::object::Archive archive(llvm::MemoryBufferRef(bytes, path), error);
  if (error) { throw std::runtime_error(llvm::toString(std::move(error))); }
  std::size_t blobs = 0;
  for (const auto &child : archive.children(error)) {
    auto name = child.getName();
    if (!name) { throw std::runtime_error(llvm::toString(name.takeError())); }
    if (!name->ends_with("data.cpp.o")) { continue; }
    auto buffer = child.getBuffer();
    if (!buffer) { throw std::runtime_error(llvm::toString(buffer.takeError())); }
    const auto magic = llvm::identify_magic(*buffer);
    require(magic == llvm::file_magic::elf_relocatable || magic == llvm::file_magic::macho_object,
            "The semantic blob is native object data, independent of files at consumer link time");
    ++blobs;
  }
  if (error) { throw std::runtime_error(llvm::toString(std::move(error))); }
  require(blobs == 1, "The core archive contains exactly one semantic blob object");
}
}

int main(int argc, char **argv) try {
  require(argc == 2, "Pass the consumer core archive");
  verify(argv[1]);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
