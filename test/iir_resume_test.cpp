// SPDX-License-Identifier: GPL-3.0-only
// Resume the original IIR at independently predicted instruction checkpoints.
#include "binary_fixture.hpp"
#include "dsp56300/backend.hpp"
#include "dsp56300/core.hpp"
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
constexpr std::size_t expected_groups = 288, expected_checkpoints = 6624;

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
void checkpoint(Implementation &core,
                const MemoryViews memory,
                const BinaryCase &item,
                const Word previous) {
  auto expected = memory.initial;
  patch(expected, item.after);
  const auto dispatches = item.budget - previous;
  const auto result = core.run(dispatches);
  require(result.dispatches == dispatches && result.fault.kind == FaultKind::none &&
              result.progress == item.progress && result.state == ProcessingState::running,
          "Resumed IIR retains exact cumulative progress and dispatch boundaries");
  require(core.registers() == item.expected && memory.current == expected,
          "Every resumed IIR checkpoint matches independent registers, flags, frames and all RAM");
  require(core.interruptible() == item.interruptible && core.debug_status() == item.debug,
          "A stopped IIR prefix preserves the declared interrupt/debug contract");
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
            "Resume scenarios share initialization and ordered independent checkpoints");
    try {
      checkpoint(core, {.current = memory, .initial = initial}, item, previous);
    } catch (const std::exception &error) {
      throw std::runtime_error(item.name + ": " + std::string{backend_name(backend)} + ": " +
                               error.what());
    }
    previous = item.budget;
  }
  require((core.registers().sr & (Word{1} << 15)) == 0,
          "The last resumed instruction completes the loop and restores the outer frame");
  if (backend == Backend::jit) {
    const auto statistics = core.jit_statistics();
    require(statistics.native_alu_instructions + statistics.helper_instructions == previous,
            "Resumed native/helper counters charge each instruction exactly once");
    require(statistics.batch_instructions == 0,
            "Causal multi-instruction IIR loops never enter the independent-product MAC batch");
  }
}

template <CoreImplementation Implementation>
void check(const std::span<const BinaryCase> cases, const Backend backend) {
  std::size_t groups = 0;
  std::size_t checkpoints = 0;
  for (std::size_t first = 0; first < cases.size(); ++first) {
    if (cases[first].budget != 0) { continue; }
    const auto name = scenario(cases[first]);
    auto end = first + 1;
    while (end < cases.size() && scenario(cases[end]) == name) { ++end; }
    group<Implementation>(cases.subspan(first, end - first), backend);
    ++groups;
    checkpoints += end - first;
    first = end - 1;
  }
  require(groups == expected_groups && checkpoints == expected_checkpoints,
          "Every declared IIR mode/context/design/signal/schedule resume scenario runs");
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_iir_resume_test FILE.d56"); }
    const auto cases = dsp56300::test::read_fixture(argv[1]);
    dsp56300::test::check<dsp56300::BackendCore<dsp56300::Backend::reference>>(
        cases, dsp56300::Backend::reference);
    dsp56300::test::check<dsp56300::BackendCore<dsp56300::Backend::cached>>(
        cases, dsp56300::Backend::cached);
    dsp56300::test::check<dsp56300::BackendCore<dsp56300::Backend::jit>>(cases,
                                                                         dsp56300::Backend::jit);
    std::cout << "864 original IIR resume scenarios, 19872 checkpoints; " << dsp56300::test::checks
              << " checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
