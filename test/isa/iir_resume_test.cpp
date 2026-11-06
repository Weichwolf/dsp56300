// SPDX-License-Identifier: GPL-3.0-only
// Resume the original IIR at independently predicted instruction boundaries.
#include "backend_support.hpp"
#include "binary_fixture.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fixture_memory.hpp"
#include "support.hpp"

#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace dsp56300::test {
namespace {
constexpr std::size_t expected_groups = 288, expected_observations = 6624;

std::string_view scenario(const BinaryCase &item) {
  const auto separator = item.name.rfind(" budget=");
  if (separator == std::string::npos) {
    throw std::runtime_error("IIR scenario has no budget tag");
  }
  return std::string_view{item.name}.substr(0, separator);
}

struct MemoryViews {
  const FixtureMemory &current, &initial;
};

template <CoreImplementation Implementation>
void observe_prefix(Implementation &core,
                    const MemoryViews memory,
                    const BinaryCase &item,
                    const Word previous) {
  auto expected = memory.initial;
  patch(expected, item.after);
  const auto retired = item.budget - previous;
  const auto result = core.run(retired);
  require(result.retired == retired && result.fault.kind == FaultKind::none &&
              result.progress == item.progress && result.state == ExecutionState::running,
          "Resumed IIR retains exact cumulative progress and dispatch boundaries");
  require(core.registers() == item.expected && memory.current == expected,
          "Every resumed IIR observation matches independent registers, flags, frames and all RAM");
  require(core.interrupt_ready() == item.interrupt_ready,
          "A stopped IIR prefix preserves the declared interrupt contract");
}

template <CoreImplementation Implementation>
void group(const std::span<const BinaryCase> items, const Backend backend) {
  const auto initial = initial_memory(items.front());
  auto memory = initial;
  Implementation core(
      {.program = memory[0], .x = memory[1], .y = memory[2]}, nullptr, {.backend = backend});
  core.set_registers(items.front().initial);
  Word previous = 0;
  for (const auto &item : items) {
    require(item.initial == items.front().initial && initial_memory(item) == initial &&
                item.budget >= previous,
            "Resume scenarios share initialization and ordered independent observations");
    try {
      observe_prefix(core, {.current = memory, .initial = initial}, item, previous);
    } catch (const std::exception &error) {
      throw std::runtime_error(item.name + ": " + std::string{backend_name(backend)} + ": " +
                               error.what());
    }
    previous = item.budget;
  }
  require((core.registers().sr & (Word{1} << 15)) == 0,
          "The last resumed instruction completes the loop and restores the outer frame");
}

template <CoreImplementation Implementation>
void check(const std::span<const BinaryCase> cases, const Backend backend) {
  std::size_t groups = 0;
  std::size_t observations = 0;
  for (std::size_t first = 0; first < cases.size(); ++first) {
    if (cases[first].budget != 0) { continue; }
    const auto name = scenario(cases[first]);
    auto end = first + 1;
    while (end < cases.size() && scenario(cases[end]) == name) { ++end; }
    group<Implementation>(cases.subspan(first, end - first), backend);
    ++groups;
    observations += end - first;
    first = end - 1;
  }
  require(groups == expected_groups && observations == expected_observations,
          "Every declared IIR mode/context/design/signal/schedule resume scenario runs");
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_iir_resume_test FILE.d56"); }
    const auto cases = dsp56300::test::read_fixture(argv[1]);
    dsp56300::test::check<dsp56300::TestCore>(cases, dsp56300::Backend::jit);
    dsp56300::test::check<dsp56300::TestCore>(cases, dsp56300::Backend::jit);
    dsp56300::test::check<dsp56300::TestCore>(cases, dsp56300::Backend::jit);
    std::cout << "864 original IIR resume scenarios, 19872 observations; " << dsp56300::test::checks
              << " checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
