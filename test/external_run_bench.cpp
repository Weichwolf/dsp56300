// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace {
using namespace dsp56300::test::external_run;

struct Selection {
  std::string_view workload, mode;
};

struct Mode {
  std::string_view name;
  bool staged, single_step, captured;
};

constexpr std::array modes{
    Mode{.name = "synchronous-batch", .staged = false, .single_step = false, .captured = false},
    Mode{.name = "synchronous-step", .staged = false, .single_step = true, .captured = false},
    Mode{.name = "staged", .staged = true, .single_step = true, .captured = false},
    Mode{.name = "captured-pcu", .staged = true, .single_step = true, .captured = true}};

void report(const Workload sample, const std::string_view selected) {
  for (const auto mode : modes) {
    if (selected != "all" && selected != mode.name) { continue; }
    const auto result =
        mode.captured
            ? captured_workload(sample, dsp56300::Backend::reference)
            : workload(sample, dsp56300::Backend::reference, mode.staged, mode.single_step);
    std::cout << sample.name << " mode=" << mode.name << " instructions=" << result.instructions
              << " actual-P-reads=" << result.reads
              << " seconds=" << std::setprecision(std::numeric_limits<double>::max_digits10)
              << result.seconds << '\n';
  }
}

unsigned execute(const Selection &selected) {
  unsigned executed = 0;
  for (const auto sample : workloads) {
    if (selected.workload != "all" && selected.workload != sample.name) { continue; }
    ++executed;
    report(sample, selected.mode);
  }
  return executed;
}
}

int main(const int count, char **arguments) {
  try {
    if (count > 3) {
      throw std::invalid_argument(
          "Expected [all|workload] [all|mode] for authored external-Io cases");
    }
    const auto selected = count >= 2 ? std::string_view(arguments[1]) : std::string_view("all");
    const auto mode = count == 3 ? std::string_view(arguments[2]) : std::string_view("all");
    if (mode != "all" && std::ranges::find(modes, mode, &Mode::name) == modes.end()) {
      throw std::invalid_argument("Unknown authored external-Io execution mode");
    }
    const auto executed = execute({.workload = selected, .mode = mode});
    if (executed == 0) {
      throw std::invalid_argument("Unknown authored external-Io workload selector");
    }
    const auto selected_modes = mode == "all" ? modes.size() : 1U;
    std::cout << "selected-workloads=" << executed << " total=" << workloads.size()
              << " skipped=" << workloads.size() - executed << " selected-modes=" << selected_modes
              << " total-modes=" << modes.size()
              << " skipped-modes=" << modes.size() - selected_modes
              << " diagnostic-only,no-realtime-qualification\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
