// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "native_long_load_fixture.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>

namespace dsp56300::test::long_load {
Timing workload(const std::string_view domain, const bool use_long, const unsigned repetitions) {
  constexpr std::size_t pairs = 1024;
  constexpr unsigned intervals = 8;
  constexpr unsigned maximum_repetitions = 256;
  if (repetitions == 0 || repetitions > maximum_repetitions) {
    throw std::invalid_argument("Expected long-load repetitions in [1,256]");
  }
  const auto *const selected = std::ranges::find(domains, domain);
  if (selected == domains.end()) { throw std::invalid_argument("Unknown long-load workload"); }
  const auto domain_index = static_cast<std::size_t>(selected - domains.begin());
  constexpr std::array<Word, 3> operations{0x2000d2, 0x200040, 0x200043};
  constexpr std::array<unsigned, 3> selectors{2, 0, 6};
  const auto alu = operations.at(domain_index);
  const Input input{.selector = selectors.at(domain_index)};
  constexpr Word scalar_load = 0x44e000;
  std::array<Word, pairs * 2> program{};
  for (std::size_t pair = 0; pair < pairs; ++pair) {
    program.at(pair * 2) = use_long ? opcode(input) : scalar_load;
    program.at((pair * 2) + 1) = alu;
  }
  std::array<Word, words> x{};
  std::array<Word, words> y{};
  x.at(address) = static_cast<Word>(input.value >> word_bits);
  y.at(address) = static_cast<Word>(input.value & word_mask);
  auto expected = seed(input);
  for (std::size_t pair = 0; pair < pairs; ++pair) {
    if (use_long) {
      expected_load(expected, input);
    } else {
      expected.x = (expected.x & (std::uint64_t{word_mask} << word_bits)) | x.at(address);
      ++expected.pc;
    }
    const auto old = expected;
    expected_alu(expected, old, alu);
    ++expected.pc;
  }
  Core cpu({.program = program, .x = x, .y = y}, nullptr, {.backend = Backend::jit});
  Timing result;
  for (unsigned sample = 0; sample <= repetitions * intervals; ++sample) {
    cpu.set_registers(seed(input));
    const auto old = cpu.jit_statistics();
    const auto before = std::chrono::steady_clock::now();
    const auto actual = cpu.run(static_cast<std::uint32_t>(program.size()));
    const auto after = std::chrono::steady_clock::now();
    require(actual.dispatches == program.size() && actual.fault.kind == FaultKind::none &&
                cpu.registers() == expected,
            "Shared long-load workload checks complete independent architectural state");
    if (sample == 0) { continue; }
    result.seconds += std::chrono::duration<double>(after - before).count();
    const auto current = cpu.jit_statistics();
    result.instructions += actual.dispatches;
    result.native += current.native_alu_instructions - old.native_alu_instructions;
    result.moves += current.native_move_instructions - old.native_move_instructions;
    result.helpers += current.helper_instructions - old.helper_instructions;
  }
  return result;
}
} // namespace dsp56300::test::long_load
