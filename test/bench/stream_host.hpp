// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "binary_fixture.hpp"

#include <string_view>
#include <vector>

namespace dsp56300::test {
struct HostStreamTimes {
  double scalar_seconds = 0, avx2_seconds = 0;
  std::string_view scope = "complete-state-and-memory";
  std::string_view implementation = "handwritten-asm";
};

[[nodiscard]] HostStreamTimes measure_fir_host(const BinaryCase &item);
void verify_fir_host(const std::vector<BinaryCase> &cases);

[[nodiscard]] HostStreamTimes measure_numeric_host(std::string_view name, const BinaryCase &item);
void verify_numeric_host(std::string_view name, const std::vector<BinaryCase> &cases);
[[nodiscard]] HostStreamTimes measure_spectral_host(const BinaryCase &item);
void verify_spectral_host(const std::vector<BinaryCase> &cases);
}
