// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "external_run_fixture.hpp"
#include "memory_window_workloads.hpp"
#include "support.hpp"

#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string_view>

namespace dsp56300::test {
namespace {
constexpr Word instructions = Word{1} << 20;
constexpr unsigned samples = 3;
constexpr double million = 1000000.0;

void measure(const external_run::Workload input, const bool mapped) {
  window_workload::Memory memory(input, mapped);
  TestCore core(memory);
  const auto initial = window_workload::initial(input, instructions);
  const auto expected = window_workload::expected(input, instructions);
  core.set_registers(initial);
  require(core.run(window_workload::body).retired == window_workload::body,
          "Warm the genuine generated memory body before measuring");
  for (unsigned sample = 0; sample < samples; ++sample) {
    core.set_registers(initial);
    const auto prior = core.progress();

    const auto reads = memory.reads;
    const auto begin = std::chrono::steady_clock::now();
    const auto result = core.run(instructions);
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

    const auto callbacks = memory.reads - reads;

    require(result.retired == instructions && result.fault.kind == FaultKind::none &&
                core.registers() == expected &&
                result.progress ==
                    Progress{.instructions = prior.instructions + instructions,
                             .estimated_cycles = prior.estimated_cycles + instructions} &&
                callbacks == (mapped ? 0U : instructions),
            "Measured native memory kernels match independent full-state arithmetic and actual "
            "callback counts");
    std::cout << input.name << " mapped=" << mapped << " sample=" << sample
              << " instructions=" << instructions << " callbacks=" << callbacks

              << " seconds=" << std::setprecision(std::numeric_limits<double>::max_digits10)
              << seconds << " guest-mips=" << (instructions / seconds) / million << '\n';
  }
}
}
}

int main(const int count, char **arguments) {
  try {
    using namespace dsp56300::test;
    require(count == 2, "Expected all or one authored memory-kernel selector");
    const auto selected = std::string_view(arguments[1]);
    unsigned executed = 0;
    for (const auto input : external_run::workloads) {
      if (selected != "all" && selected != input.name) { continue; }
      ++executed;
      measure(input, false);
      measure(input, true);
    }
    require(executed != 0, "Unknown authored memory-kernel selector");
    std::cout
        << "selected=" << executed << " total=" << external_run::workloads.size()
        << " skipped=" << external_run::workloads.size() - executed
        << " scope=RAM-read-microkernels;no-complete-application-or-instrument-throughput-claim\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
