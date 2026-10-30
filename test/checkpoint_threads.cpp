// SPDX-License-Identifier: GPL-3.0-only
#include "checkpoint_support.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <latch>
#include <thread>
#include <vector>

namespace dsp56300::test::checkpoint_test {
void shared_checkpoint() {
  constexpr Word cache_enable = Word{1} << 19;
  constexpr Word external = 1024;
  Machine source(Backend::reference);
  auto source_state = seed();
  source_state.sr = cache_enable;
  source_state.pc = external;
  source.core.set_registers(source_state);
  source.core.write(Space::program, external, add_x0);
  step(source.core);
  add(source_state);
  source.core.set_registers(source_state);
  source.core.write(Space::program, external, 0);
  const auto saved = source.core.checkpoint();
  const auto memory = source.memory;
  auto expected = source_state;
  add(expected);
  ++expected.pc;
  std::latch started(1);
  std::array<bool, 3> valid{};
  const auto resume = [&](const std::size_t index) {
    started.wait();
    try {
      Machine target(backends[index]);
      target.memory = memory;
      bool exact = true;
      for (unsigned suffix = 0; suffix < 8; ++suffix) {
        target.core.restore(saved);
        const auto result = target.core.run(1);
        exact = exact && result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                target.core.registers() == expected &&
                result.progress == Progress{.instructions = 2, .estimated_cycles = 2};
      }
      valid[index] = exact;
    } catch (const std::exception &) { valid[index] = false; }
  };
  {
    std::vector<std::jthread> threads;
    threads.reserve(backends.size());
    try {
      for (std::size_t index = 0; index < backends.size(); ++index) {
        threads.emplace_back(resume, index);
      }
    } catch (...) {
      started.count_down();
      throw;
    }
    started.count_down();
  }
  for (const bool exact : valid) {
    require(exact, "Independent CPU owners concurrently restore one immutable checkpoint");
  }
  require(source.core.registers() == source_state &&
              source.core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "Shared snapshot use never mutates its original CPU");
}
} // namespace dsp56300::test::checkpoint_test
