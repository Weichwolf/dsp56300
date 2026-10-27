// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dsp56300::test {
namespace {
using Memory = std::array<std::vector<Word>, 3>;

void patch(Memory &memory, const std::vector<MemoryBlock> &blocks) {
  for (const auto &block : blocks) {
    auto &space = memory.at(static_cast<std::size_t>(block.space));
    if (block.address > space.size() || block.words.size() > space.size() - block.address) {
      throw std::runtime_error("Fixture block outside memory span");
    }
    std::ranges::copy(block.words, space.begin() + block.address);
  }
}

void check(const BinaryCase &item, const Backend backend) {
  Memory memory;
  for (std::size_t space = 0; space < memory.size(); ++space) {
    memory[space].resize(item.memory_words[space]);
  }
  patch(memory, item.before);
  auto expected = memory;
  patch(expected, item.after);
  Core core({.program = memory[0], .x = memory[1], .y = memory[2]}, nullptr, {.backend = backend});
  core.set_registers(item.initial);
  const auto result = core.run(item.budget);
  require(result.dispatches == item.dispatches, "Retired dispatch count");
  require(result.progress == item.progress, "Instruction count and documented base cost");
  require(result.fault == item.fault, "Expected fault context");
  require(core.interruptible() == item.interruptible, "Interrupt admission boundary");
  // Failed dispatches do not promise a committed architectural state.
  if (item.fault.kind == FaultKind::none) {
    require(core.registers() == item.expected, "Complete architectural register state");
    require(memory == expected, "All P/X/Y memory, including locations outside expected writes");
  }
}

void run(const std::filesystem::path &path) {
  const auto cases = read_fixture(path);
  require(!cases.empty(), "Fixture contains cases");
  for (const auto backend : {Backend::reference, Backend::cached}) {
    for (std::size_t index = 0; index < cases.size(); ++index) {
      try {
        check(cases[index], backend);
      } catch (const std::exception &error) {
        const std::string engine = backend == Backend::reference ? "reference" : "cached";
        throw std::runtime_error(path.string() + ": case " + std::to_string(index) + " (" +
                                 cases[index].name + ", " + engine + "): " + error.what());
      }
    }
  }
  std::cout << path.filename().string() << ": " << cases.size() << " cases per backend\n";
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc < 2) { throw std::runtime_error("Usage: dsp56300_fixture_test FILE.d56 [...]"); }
    for (int index = 1; index < argc; ++index) { dsp56300::test::run(argv[index]); }
    std::cout << dsp56300::test::checks << " binary fixture checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
