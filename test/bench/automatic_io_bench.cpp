// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "external_run_fixture.hpp"
#include "native_sequence_model.hpp"
#include "support.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace {
using namespace dsp56300;
using namespace dsp56300::test;
constexpr unsigned body_words = 32;
constexpr std::uint32_t instruction_count = std::uint32_t{1} << 24;
constexpr unsigned samples = 4;
constexpr double million = 1000000.0;

class CodeMemory final : public Io {
public:
  std::array<Word, body_words> program{};
  unsigned accesses = 0;

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (address < start || address - start >= program.size()) { return std::nullopt; }
    return program[address - start];
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    ++accesses;
    const auto known = inspect_program(address);
    if (space != Space::program || !known) { return false; }
    value = *known;
    return true;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    ++accesses;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    ++accesses;
    return false;
  }
};

Registers expected_result(Registers expected, const external_run::Workload workload) {
  for (std::uint32_t index = 0; index < instruction_count; ++index) {
    const auto before = expected;
    if (workload.name == "communications-xor") {
      bfu_model::apply(expected, before, {.function = bfu_model::Function::bit_xor});
    } else {
      sequence_model::alu(expected, before, workload.opcode);
    }
  }
  expected.pc = start;
  expected.lc = 1;
  return expected;
}

void measure(const external_run::Workload workload) {
  CodeMemory memory;
  memory.program.fill(workload.opcode);
  TestCore core(memory); // Ordinary construction: no backend or compiler selection.
  auto initial = initial_state();
  initial.x = workload.x;
  initial.y = workload.y;
  initial.sr = 0x8000;
  initial.la = start + body_words - 1;
  initial.lc = instruction_count / body_words + 1;
  initial.sp = initial.sc = 2;
  initial.stack[2] = std::uint64_t{start} << word_bits;
  const auto expected = expected_result(initial, workload);
  for (unsigned sample = 0; sample < samples; ++sample) {
    core.set_registers(initial);
    const auto prior = core.progress();

    const auto begin = std::chrono::steady_clock::now();
    const auto result = core.run(instruction_count);
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

    require(result.retired == instruction_count && !result.yielded &&
                result.fault.kind == FaultKind::none && core.registers() == expected &&
                result.progress ==
                    Progress{.instructions = prior.instructions + instruction_count,
                             .estimated_cycles = prior.estimated_cycles + instruction_count} &&
                memory.accesses == 0,
            "Automatic sparse-code execution matches independent full-state domain arithmetic and "
            "DO accounting");
    std::cout << workload.name << " sample=" << sample
              << " phase=" << (sample == 0 ? "cold" : "warm")
              << " instructions=" << instruction_count

              << " seconds=" << std::setprecision(std::numeric_limits<double>::max_digits10)
              << seconds << " guest-mips=" << (instruction_count / seconds) / million << '\n';
  }
  std::cout << "scope=core-only,pure-register-domains,external-program-view;"
               "not-connected-instrument-or-complete-application-throughput\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected one independently authored domain selector");
    const auto selected = std::string_view(arguments[1]);
    for (const auto workload : external_run::workloads) {
      if (selected != workload.name) { continue; }
      measure(workload);
      return 0;
    }
    throw std::invalid_argument("Unknown independently authored domain");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
