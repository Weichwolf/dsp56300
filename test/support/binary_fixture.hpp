// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace dsp56300::test {
struct MemoryBlock {
  std::vector<Word> words;
  Word address = 0;
  Space space = Space::program;
};

enum class FixtureVersion : std::uint8_t { legacy = 1, processing = 2, debugging = 3 };

struct ResetAction {
  Word pc = 0;
  bool accepted = false;
  bool operator==(const ResetAction &) const = default;
};

struct BinaryCase {
  Registers initial, expected;
  Progress progress{.instructions = 1, .estimated_cycles = 1};
  Fault fault;
  std::vector<MemoryBlock> before, after;
  std::vector<ResetAction> reset_actions;
  std::string name;
  std::array<Word, 3> memory_words{1024, 1024, 1024};
  Word budget = 1, dispatches = 1;
  bool interruptible = true;
  ProcessingState state = ProcessingState::running;
  bool debug_interrupt_mode = false;
  DebugStatus debug;
};

// Versioned, little-endian fields; never a dump of compiler-dependent structures.
void write_fixture(const std::filesystem::path &path,
                   std::vector<BinaryCase> cases,
                   FixtureVersion version = FixtureVersion::legacy);
[[nodiscard]] std::vector<BinaryCase> read_fixture(const std::filesystem::path &path);
} // namespace dsp56300::test
