// SPDX-License-Identifier: GPL-3.0-only
#include "native_long_load_fixture.hpp"

#include <charconv>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace {
unsigned repetitions(const std::string_view text) {
  constexpr unsigned maximum = 256;
  unsigned result = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || result == 0 ||
      result > maximum) {
    throw std::invalid_argument("Expected repetitions in [1,256]");
  }
  return result;
}
}

int main(const int count, char **arguments) {
  try {
    using namespace dsp56300::test::long_load;
    if (count > 4) {
      throw std::invalid_argument("Expected [all|domain] [all|long|word] [repetitions]");
    }
    const auto domain = count >= 2 ? std::string_view(arguments[1]) : std::string_view("all");
    const auto mode = count >= 3 ? std::string_view(arguments[2]) : std::string_view("all");
    const auto repeat = count == 4 ? repetitions(arguments[3]) : 1U;
    if (mode != "all" && mode != "long" && mode != "word") {
      throw std::invalid_argument("Unknown long-load mode");
    }
    unsigned selected = 0;
    for (const auto name : domains) {
      if (domain != "all" && domain != name) { continue; }
      for (const bool use_long : {false, true}) {
        const std::string_view label = use_long ? "long" : "word";
        if (mode != "all" && mode != label) { continue; }
        const auto result = workload(name, use_long, repeat);
        std::cout << name << " mode=" << label << " instructions=" << result.instructions
                  << " native=" << result.native << " moves=" << result.moves
                  << " helpers=" << result.helpers
                  << " seconds=" << std::setprecision(std::numeric_limits<double>::max_digits10)
                  << result.seconds << '\n';
        ++selected;
      }
    }
    if (selected == 0) { throw std::invalid_argument("Unknown long-load domain"); }
    std::cout << "selected=" << selected << " total=" << domains.size() * 2 << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
