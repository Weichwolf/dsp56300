// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"
#include "native_repeat_fixture.hpp"
#include "support.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace dsp56300::test::native_repeat {
namespace {
constexpr unsigned rounds = 16, samples = 3;
constexpr double million = 1000000.0;

void measure(const external_run::Workload input) {
  Bus bus;
  bus.program[start] = repeat(0);
  bus.program[body] = input.opcode;
  TestCore core(bus);
  const auto wanted = expected(input, {.retired = zero_count});
  for (unsigned sample = 0; sample <= samples; ++sample) {
    double seconds = 0;

    unsigned reads = 0;
    for (unsigned round = 0; round < rounds; ++round) {
      core.set_registers(seed(input));
      require(core.run(1).retired == 1, "Benchmark executes real REP setup outside timing");
      const auto before = core.progress();

      const auto program_reads = bus.program_reads;
      const auto begin = std::chrono::steady_clock::now();
      const auto result = core.run(zero_count);
      seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

      reads += bus.program_reads - program_reads;
      require(result.retired == zero_count && result.fault.kind == FaultKind::none &&
                  core.registers() == wanted &&
                  core.progress() ==
                      Progress{.instructions = before.instructions + zero_count,
                               .estimated_cycles = before.estimated_cycles + zero_count},
              "Timed REP execution retains the shared independent full-state and cycle oracle");
    }
    if (sample == 0) { continue; }
    constexpr auto count = std::uint64_t{rounds} * zero_count;
    std::cout << input.name << " sample=" << sample << " instructions=" << count
              << " seconds=" << seconds << " guest-mips=" << count / seconds / million
              << " reads=" << reads << '\n';
  }
}
}
}

int main(const int argc, const char *const *const argv) {
  try {
    if (argc > 2) { throw std::invalid_argument("Select all or one authored domain"); }
    const std::string_view selected = argc == 2 ? argv[1] : "all";
    unsigned executed = 0;
    for (const auto input : dsp56300::test::external_run::workloads) {
      if (selected == "all" || selected == input.name) {
        dsp56300::test::native_repeat::measure(input);
        ++executed;
      }
    }
    if (executed == 0) { throw std::invalid_argument("Unknown authored domain"); }
    std::cout << "domains=" << executed
              << " skipped=" << dsp56300::test::external_run::workloads.size() - executed << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "REP benchmark: " << error.what() << '\n';
    return 1;
  }
}
