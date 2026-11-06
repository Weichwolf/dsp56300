// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "external_run_fixture.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace {
using namespace dsp56300::test::external_run;

struct Selection {
  std::string_view workload, mode;
  unsigned repetitions = 1;
};

struct Mode {
  std::string_view name;
  bool single_step, captured;
  bool cache_enabled = false;
};

constexpr std::array modes{
    Mode{.name = "synchronous-batch", .single_step = false, .captured = false},
    Mode{.name = "synchronous-step", .single_step = true, .captured = false},

    Mode{.name = "captured", .single_step = true, .captured = true},
    Mode{.name = "captured-ce", .single_step = true, .captured = true, .cache_enabled = true}};

void report(const Workload sample, const std::string_view selected, const unsigned repetitions) {
  for (const auto mode : modes) {
    if (selected != "all" && selected != mode.name) { continue; }
    Timing result{.seconds = 0, .instructions = 0, .reads = 0};
    for (unsigned repetition = 0; repetition < repetitions; ++repetition) {
      const auto measured =
          mode.captured ? captured_workload(sample, dsp56300::Backend::jit, mode.cache_enabled)
                        : workload(sample, dsp56300::Backend::jit, mode.single_step);
      result.seconds += measured.seconds;
      result.instructions += measured.instructions;
      result.reads += measured.reads;
    }
    std::cout << sample.name << " mode=" << mode.name << " instructions=" << result.instructions
              << " actual-P-reads=" << result.reads
              << " seconds=" << std::setprecision(std::numeric_limits<double>::max_digits10)
              << result.seconds << '\n';
  }
}

unsigned repetition_count(const std::string_view text) {
  constexpr unsigned maximum_repetitions = 256;
  unsigned value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value == 0 ||
      value > maximum_repetitions) {
    throw std::invalid_argument("Expected an external-Io repetition count in [1,256]");
  }
  return value;
}

unsigned execute(const Selection &selected) {
  unsigned executed = 0;
  for (const auto sample : workloads) {
    if (selected.workload != "all" && selected.workload != sample.name) { continue; }
    ++executed;
    report(sample, selected.mode, selected.repetitions);
  }
  return executed;
}
}

int main(const int count, char **arguments) {
  try {
    if (count > 4) {
      throw std::invalid_argument(
          "Expected [all|workload] [all|mode] [repetitions] for authored external-Io cases");
    }
    const auto selected = count >= 2 ? std::string_view(arguments[1]) : std::string_view("all");
    const auto mode = count >= 3 ? std::string_view(arguments[2]) : std::string_view("all");
    if (mode != "all" && std::ranges::find(modes, mode, &Mode::name) == modes.end()) {
      throw std::invalid_argument("Unknown authored external-Io execution mode");
    }
    const auto repetitions = count == 4 ? repetition_count(arguments[3]) : 1U;
    const auto executed = execute({.workload = selected, .mode = mode, .repetitions = repetitions});
    if (executed == 0) {
      throw std::invalid_argument("Unknown authored external-Io workload selector");
    }
    const auto selected_modes = mode == "all" ? modes.size() : 1U;
    std::cout << "selected-workloads=" << executed << " total=" << workloads.size()
              << " skipped=" << workloads.size() - executed << " selected-modes=" << selected_modes
              << " total-modes=" << modes.size()
              << " skipped-modes=" << modes.size() - selected_modes
              << " repetitions=" << repetitions
              << " backend=automatic diagnostic-only,no-realtime-qualification\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
