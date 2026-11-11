// SPDX-License-Identifier: GPL-3.0-only
#include "support.hpp"

#include <cstddef>
#include <exception>
#include <iostream>
#include <llvm/BinaryFormat/Magic.h>
#include <llvm/Object/Archive.h>
#include <llvm/Object/Binary.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/Error.h>
#include <stdexcept>
#include <utility>

namespace {
using dsp56300::test::require;

void verify(const char *path) {
  auto binary = llvm::object::createBinary(path);
  if (!binary) { throw std::runtime_error(llvm::toString(binary.takeError())); }
  const auto *archive = llvm::dyn_cast<llvm::object::Archive>(binary->getBinary());
  require(archive != nullptr, "The consumer receives a static core archive");
  auto error = llvm::Error::success();
  std::size_t blobs = 0;
  for (const auto &child : archive->children(error)) {
    auto name = child.getName();
    if (!name) { throw std::runtime_error(llvm::toString(name.takeError())); }
    if (!name->ends_with("data.cpp.o")) { continue; }
    auto buffer = child.getMemoryBufferRef();
    if (!buffer) { throw std::runtime_error(llvm::toString(buffer.takeError())); }
    const auto magic = llvm::identify_magic(buffer->getBuffer());
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
