// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "fixture_memory.hpp"
#include "support.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace dsp56300::test {
namespace {
void check(const BinaryCase &item, const Backend backend) {
  auto memory = initial_memory(item);
  auto expected = memory;
  patch(expected, item.after);
  Core core({.program = memory[0], .x = memory[1], .y = memory[2]}, nullptr, {.backend = backend});
  core.set_registers(item.initial);
  const auto result = core.run(item.budget);
  require(result.dispatches == item.dispatches, "Retired dispatch count");
  require(result.progress == item.progress, "Instruction count and documented base cost");
  require(result.fault == item.fault, "Expected fault context");
  if (backend == Backend::jit) {
    const auto statistics = core.jit_statistics();
    require(statistics.native_alu_instructions + statistics.helper_instructions ==
                result.dispatches,
            "Native/helper retirement accounting");
  }
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
  for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
    for (std::size_t index = 0; index < cases.size(); ++index) {
      try {
        check(cases[index], backend);
      } catch (const std::exception &error) {
        const std::string engine{backend_name(backend)};
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
