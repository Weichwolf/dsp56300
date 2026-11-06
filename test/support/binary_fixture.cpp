// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::uint32_t format_magic = 0x46363544; // ASCII D56F
constexpr std::uint32_t legacy_version = 1, processing_version = 2, debugging_version = 3;
constexpr std::uint32_t maximum_reset_actions = 65536;
constexpr std::uint32_t maximum_cases = 65536, maximum_blocks = 1024, maximum_name = 256;
constexpr unsigned byte_bits = 8;

class Writer {
public:
  explicit Writer(const std::filesystem::path &path) : stream_(path, std::ios::binary) {
    stream_.exceptions(std::ios::badbit | std::ios::failbit);
  }

  template <std::unsigned_integral Value> void scalar(Value &value) {
    std::array<char, sizeof(Value)> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      bytes[i] =
          static_cast<char>((value >> (i * byte_bits)) & std::numeric_limits<unsigned char>::max());
    }
    stream_.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }

  void text(std::string &value) {
    auto length = static_cast<std::uint32_t>(value.size());
    count(length, maximum_name);
    stream_.write(value.data(), static_cast<std::streamsize>(length));
  }

  void count(std::uint32_t &value, const std::uint32_t maximum) {
    if (value > maximum) { throw std::runtime_error("Fixture count exceeds format limit"); }
    scalar(value);
  }

  void finish() { stream_.close(); }

private:
  std::ofstream stream_;
};

class Reader {
public:
  explicit Reader(const std::filesystem::path &path) : stream_(path, std::ios::binary) {
    stream_.exceptions(std::ios::badbit | std::ios::failbit);
  }

  template <std::unsigned_integral Value> void scalar(Value &value) {
    std::array<unsigned char, sizeof(Value)> bytes{};
    for (auto &byte : bytes) {
      char input = 0;
      stream_.get(input);
      byte = static_cast<unsigned char>(input);
    }
    value = 0;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      value |= static_cast<Value>(bytes[i]) << (i * byte_bits);
    }
  }

  void text(std::string &value) {
    std::uint32_t length = 0;
    count(length, maximum_name);
    value.resize(length);
    stream_.read(value.data(), static_cast<std::streamsize>(length));
  }

  void count(std::uint32_t &value, const std::uint32_t maximum) {
    scalar(value);
    if (value > maximum) { throw std::runtime_error("Fixture count exceeds format limit"); }
  }

  void finish() {
    // peek may set eofbit; failbit still identifies an actual I/O failure.
    if (stream_.peek() != std::char_traits<char>::eof()) {
      throw std::runtime_error("Trailing bytes after fixture cases");
    }
  }

private:
  std::ifstream stream_;
};

template <typename Archive>
concept FixtureArchive =
    requires(Archive &archive, std::uint32_t &word, std::uint64_t &wide, std::string &text) {
      { archive.scalar(word) } -> std::same_as<void>;
      { archive.scalar(wide) } -> std::same_as<void>;
      { archive.text(text) } -> std::same_as<void>;
      { archive.count(word, maximum_cases) } -> std::same_as<void>;
      { archive.finish() } -> std::same_as<void>;
    };

template <FixtureArchive Archive> void registers(Archive &archive, Registers &state) {
  for (auto *value : {&state.x, &state.y, &state.a, &state.b}) { archive.scalar(*value); }
  for (auto *bank : {&state.r, &state.n, &state.m}) {
    for (auto &value : *bank) { archive.scalar(value); }
  }
  for (auto *value : {&state.pc,
                      &state.sr,
                      &state.omr,
                      &state.la,
                      &state.lc,
                      &state.sp,
                      &state.sc,
                      &state.sz,
                      &state.ep,
                      &state.vba}) {
    archive.scalar(*value);
  }
  for (auto &value : state.stack) { archive.scalar(value); }
}

template <FixtureArchive Archive> void blocks(Archive &archive, std::vector<MemoryBlock> &memory) {
  auto size = static_cast<std::uint32_t>(memory.size());
  archive.count(size, maximum_blocks);
  memory.resize(size);
  for (auto &block : memory) {
    auto space = static_cast<std::uint32_t>(block.space);
    archive.scalar(space);
    if (space > static_cast<std::uint32_t>(Space::y)) {
      throw std::runtime_error("Invalid fixture space");
    }
    block.space = static_cast<Space>(space);
    archive.scalar(block.address);
    auto words = static_cast<std::uint32_t>(block.words.size());
    archive.count(words, static_cast<std::uint32_t>(address_words));
    block.words.resize(words);
    for (auto &word : block.words) {
      archive.scalar(word);
      if (word > word_mask) { throw std::runtime_error("Fixture memory word exceeds 24 bits"); }
    }
  }
}

template <FixtureArchive Archive>
void processing(Archive &archive, BinaryCase &item, const std::uint32_t version) {
  auto state = static_cast<std::uint32_t>(item.state);
  const auto maximum_state =
      version == debugging_version ? ExecutionState::debugging : ExecutionState::stopped;
  archive.count(state, static_cast<std::uint32_t>(maximum_state));
  item.state = static_cast<ExecutionState>(state);
  auto count = static_cast<std::uint32_t>(item.reset_actions.size());
  archive.count(count, maximum_reset_actions);
  item.reset_actions.resize(count);
  for (auto &action : item.reset_actions) {
    archive.scalar(action.pc);
    if (action.pc > word_mask) { throw std::runtime_error("Fixture RESET PC exceeds 24 bits"); }
    std::uint32_t accepted = action.accepted ? 1U : 0U;
    archive.count(accepted, 1);
    action.accepted = accepted != 0;
  }
}

template <FixtureArchive Archive> void debugging(Archive &archive, BinaryCase &item) {
  for (auto *value :
       {&item.debug_interrupt_mode, &item.debug.interrupt_mode, &item.debug.software_occurrence}) {
    std::uint32_t bit = *value ? 1U : 0U;
    archive.count(bit, 1);
    *value = bit != 0;
  }
}

template <FixtureArchive Archive>
void test_case(Archive &archive, BinaryCase &item, const std::uint32_t version) {
  if (version == legacy_version &&
      (item.state != ExecutionState::running || !item.reset_actions.empty())) {
    throw std::runtime_error("Processing expectations require fixture version 2");
  }
  if (version < debugging_version && (item.debug_interrupt_mode || item.debug != DebugStatus{})) {
    throw std::runtime_error("Debug expectations require fixture version 3");
  }
  archive.text(item.name);
  for (auto &size : item.memory_words) {
    archive.scalar(size);
    if (size == 0 || size > address_words) {
      throw std::runtime_error("Invalid fixture memory size");
    }
  }
  archive.scalar(item.budget);
  archive.scalar(item.retired);
  archive.scalar(item.progress.instructions);
  archive.scalar(item.progress.estimated_cycles);
  auto kind = static_cast<std::uint32_t>(item.fault.kind);
  auto space = static_cast<std::uint32_t>(item.fault.space);
  archive.scalar(kind);
  archive.scalar(space);
  const auto maximum_fault =
      version == legacy_version ? FaultKind::invalid_instruction : FaultKind::host_event;
  if (kind > static_cast<std::uint32_t>(maximum_fault) ||
      space > static_cast<std::uint32_t>(Space::y)) {
    throw std::runtime_error("Invalid fixture fault");
  }
  item.fault.kind = static_cast<FaultKind>(kind);
  item.fault.space = static_cast<Space>(space);
  archive.scalar(item.fault.address);
  archive.scalar(item.fault.value);
  archive.scalar(item.fault.pc);
  std::uint32_t interrupt_ready = item.interrupt_ready ? 1U : 0U;
  archive.count(interrupt_ready, 1);
  item.interrupt_ready = interrupt_ready != 0;
  if (version >= processing_version) { processing(archive, item, version); }
  if (version == debugging_version) { debugging(archive, item); }
  registers(archive, item.initial);
  registers(archive, item.expected);
  blocks(archive, item.before);
  blocks(archive, item.after);
}

template <FixtureArchive Archive>
void contents(Archive &archive, std::vector<BinaryCase> &cases, const FixtureVersion schema) {
  auto magic = format_magic;
  auto version = static_cast<std::uint32_t>(schema);
  archive.scalar(magic);
  archive.scalar(version);
  if (magic != format_magic || (version != legacy_version && version != processing_version &&
                                version != debugging_version)) {
    throw std::runtime_error("Unsupported DSP fixture magic/version");
  }
  auto count = static_cast<std::uint32_t>(cases.size());
  archive.count(count, maximum_cases);
  cases.resize(count);
  for (auto &item : cases) { test_case(archive, item, version); }
  archive.finish();
}
} // namespace

void write_fixture(const std::filesystem::path &path,
                   std::vector<BinaryCase> cases,
                   const FixtureVersion version) {
  Writer writer(path);
  contents(writer, cases, version);
}

std::vector<BinaryCase> read_fixture(const std::filesystem::path &path) {
  Reader reader(path);
  std::vector<BinaryCase> cases;
  contents(reader, cases, FixtureVersion::legacy);
  return cases;
}
} // namespace dsp56300::test
