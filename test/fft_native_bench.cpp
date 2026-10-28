// SPDX-License-Identifier: GPL-3.0-only
// Native algorithm lower bound; this executable never runs a DSP instruction.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "fft_native.hpp"
#include "fft_program.hpp"
#include "fft_twiddles.hpp"
#include "fixture_memory.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <ratio>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace dsp56300::test::fft::native {
namespace {
using Clock = std::chrono::steady_clock;
constexpr unsigned warmups = 64, trials = 9, default_replays = 4096;
constexpr unsigned maximum_workers = 64, maximum_replays = 1U << 20;
constexpr std::string_view name_prefix = "Own FFT DIF N=";

unsigned number(const std::string_view text) {
  unsigned value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw std::invalid_argument("Invalid unsigned argument");
  }
  return value;
}

Word size_of(const BinaryCase &item) {
  require(item.name.starts_with(name_prefix), "Native experiment requires an own FFT fixture");
  const auto end = item.name.find(' ', name_prefix.size());
  const auto value =
      number(std::string_view(item.name).substr(name_prefix.size(), end - name_prefix.size()));
  require(value >= 16 && value <= maximum_size && std::has_single_bit(value), "Qualified FFT size");
  require((item.initial.sr & (mode_up | mode_down)) == 0, "Native FFT requires ordinary scaling");
  return value;
}

class Fixture {
public:
  explicit Fixture(const BinaryCase &item) : rm_((item.initial.sr & mode_rm) != 0) {
    const auto size = size_of(item);
    const auto initial = initial_memory(item);
    auto expected = initial;
    patch(expected, item.after);
    plan_ = make_plan({.real = std::span(initial[1]).subspan(twiddle_base, size / 2),
                       .imaginary = std::span(initial[2]).subspan(twiddle_base, size / 2)});
    for (Word index = 0; index < size; ++index) {
      initial_.real.push_back(signed24(initial[1][data_base + index]));
      initial_.imaginary.push_back(signed24(initial[2][data_base + index]));
      expected_[0].push_back(expected[1][data_base + index]);
      expected_[1].push_back(expected[2][data_base + index]);
      expected_[2].push_back(expected[1][output_base + index]);
      expected_[3].push_back(expected[2][output_base + index]);
    }
    initial_.output_real.resize(size);
    initial_.output_imaginary.resize(size);
    data_ = initial_;
  }

  void prepare() {
    std::ranges::copy(initial_.real, data_.real.begin());
    std::ranges::copy(initial_.imaginary, data_.imaginary.begin());
  }

  void execute(Pool &pool) { pool.run(data_, plan_, rm_); }

  void validate() const {
    const auto encoded = [](const std::int64_t value) {
      return static_cast<Word>(value) & word_mask;
    };
    require(std::ranges::equal(data_.real, expected_[0], std::equal_to{}, encoded) &&
                std::ranges::equal(data_.imaginary, expected_[1], std::equal_to{}, encoded) &&
                std::ranges::equal(data_.output_real, expected_[2], std::equal_to{}, encoded) &&
                std::ranges::equal(data_.output_imaginary, expected_[3], std::equal_to{}, encoded),
            "Native algorithm matches all independently expected FFT and output words");
  }

private:
  bool rm_;
  Data initial_, data_;
  std::vector<Stage> plan_;
  std::array<std::vector<Word>, 4> expected_;
};

bool avx2_available() noexcept {
  return __builtin_cpu_supports("avx2");
}

double nanoseconds(const Clock::time_point begin, const Clock::time_point end) {
  return std::chrono::duration<double, std::nano>(end - begin).count();
}

struct Mode {
  unsigned workers = 1, replays = default_replays;
  bool avx2 = false;
};

void benchmark(const BinaryCase &item, const Mode mode) {
  Pool pool(mode.workers, mode.avx2);
  Fixture fixture(item);
  for (unsigned index = 0; index < warmups; ++index) {
    fixture.prepare();
    fixture.execute(pool);
  }
  fixture.validate();
  std::array<double, trials> timings{};
  for (auto &trial : timings) {
    double elapsed = 0;
    for (unsigned index = 0; index < mode.replays; ++index) {
      fixture.prepare();
      const auto before = Clock::now();
      fixture.execute(pool);
      const auto after = Clock::now();
      elapsed += nanoseconds(before, after);
    }
    fixture.validate();
    trial = elapsed / mode.replays;
  }
  std::ranges::sort(timings);
  std::cout << (mode.avx2 ? "avx2" : "scalar") << " workers=" << mode.workers
            << " native algorithm median " << timings[trials / 2] << " ns/FFT, range "
            << timings.front() << ".." << timings.back() << '\n';
}

void verify(const std::vector<BinaryCase> &cases) {
  for (const bool avx2 : {false, true}) {
    if (avx2 && !avx2_available()) { continue; }
    for (const unsigned workers : {1U, 2U, 3U, 6U}) {
      Pool pool(workers, avx2);
      for (const auto &item : cases) {
        Fixture fixture(item);
        fixture.execute(pool);
        fixture.validate();
      }
    }
  }
  std::cout << checks << " native FFT algorithm checks passed; AVX2="
            << static_cast<unsigned>(avx2_available()) << '\n';
}

void run(const int argc, const char *const *const argv) {
  if (argc == 3 && std::string_view(argv[1]) == "--verify") {
    verify(read_fixture(argv[2]));
    return;
  }
  if (argc < 3 || argc > 5) {
    throw std::runtime_error(
        "Usage: dsp56300_fft_native_bench FILE CASE [WORKERS [REPLAYS]] or --verify FILE");
  }
  const auto cases = read_fixture(argv[1]);
  const auto &item = cases.at(number(argv[2]));
  const auto workers = argc >= 4 ? number(argv[3]) : 1U;
  const auto replays = argc == 5 ? number(argv[4]) : default_replays;
  if (workers == 0 || workers > maximum_workers || replays == 0 || replays > maximum_replays) {
    throw std::invalid_argument("Workers must be in [1,64]; replays in [1,1048576]");
  }
  std::cout << std::fixed << std::setprecision(2) << item.name << '\n'
            << "Native algorithm only: no DSP registers, events, timing or emulator execution.\n"
            << "64 warmups, median of 9 x " << replays
            << " calls; input restoration excluded, stage synchronization included.\n";
  benchmark(item, {.workers = workers, .replays = replays});
  if (avx2_available()) { benchmark(item, {.workers = workers, .replays = replays, .avx2 = true}); }
}
} // namespace
} // namespace dsp56300::test::fft::native

int main(const int argc, const char *const *const argv) {
  try {
    dsp56300::test::fft::native::run(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
