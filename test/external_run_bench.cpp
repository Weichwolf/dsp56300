// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {
using namespace dsp56300::test::external_run;

struct Mode {
  std::string_view name;
  bool staged, single_step;
};

void report(const Workload sample) {
  constexpr std::array modes{
      Mode{.name = "synchronous-batch", .staged = false, .single_step = false},
      Mode{.name = "synchronous-step", .staged = false, .single_step = true},
      Mode{.name = "staged", .staged = true, .single_step = true}};
  for (const auto mode : modes) {
    const auto result =
        workload(sample, dsp56300::Backend::reference, mode.staged, mode.single_step);
    std::cout << sample.name << " mode=" << mode.name << " instructions=" << result.instructions
              << " actual-P-reads=" << result.reads << " seconds=" << result.seconds << '\n';
  }
}

unsigned execute(const std::string_view selected) {
  unsigned executed = 0;
  for (const auto sample : workloads) {
    if (selected != "all" && selected != sample.name) { continue; }
    ++executed;
    report(sample);
  }
  return executed;
}
}

int main(const int count, char **arguments) {
  try {
    if (count > 2) {
      throw std::invalid_argument("Expected all or one authored external-Io workload name");
    }
    const auto selected = count == 2 ? std::string_view(arguments[1]) : std::string_view("all");
    const auto executed = execute(selected);
    if (executed == 0) {
      throw std::invalid_argument("Unknown authored external-Io workload selector");
    }
    std::cout << "selected-workloads=" << executed << " total=" << workloads.size()
              << " skipped=" << workloads.size() - executed
              << " diagnostic-only,no-realtime-qualification\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
