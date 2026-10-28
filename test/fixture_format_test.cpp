// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr std::size_t legacy_bytes = 676, state_offset = 76;
constexpr std::uint32_t magic = 0x46363544, excessive_count = 65537;
constexpr Word action_pc = 0x123456;

void word(std::vector<char> &bytes, const std::size_t offset, const std::uint32_t value) {
  for (unsigned byte = 0; byte < 4; ++byte) {
    constexpr Word octet_mask = 255;
    bytes.at(offset + byte) = static_cast<char>((value >> (byte * 8)) & octet_mask);
  }
}

std::vector<char> golden(const FixtureVersion version) {
  const bool processing = version != FixtureVersion::legacy;
  // Authored from the documented wire coordinates, never sizeof/native dumps.
  std::vector<char> result(legacy_bytes, 0);
  word(result, 0, magic);
  word(result, 4, static_cast<std::uint32_t>(version));
  word(result, 8, 1);
  for (const std::size_t offset : {16U, 20U, 24U}) { word(result, offset, 1024); }
  for (const std::size_t offset : {28U, 32U, 36U, 44U, 72U}) { word(result, offset, 1); }
  if (processing) {
    result.insert(result.begin() + state_offset, 16, 0);
    word(result, state_offset, 1);
    word(result, state_offset + 4, 1);
    word(result, state_offset + 8, action_pc);
    word(result, state_offset + 12, 1);
  }
  if (version == FixtureVersion::debugging) {
    constexpr std::size_t debug_offset = state_offset + 16;
    constexpr std::size_t software_offset = debug_offset + 8;
    result.insert(result.begin() + debug_offset, 12, 0);
    word(result, state_offset, 3);
    word(result, software_offset, 1);
  }
  return result;
}

void put(const std::filesystem::path &path, const std::vector<char> &bytes) {
  std::ofstream output(path, std::ios::binary);
  output.exceptions(std::ios::badbit | std::ios::failbit);
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

std::vector<char> get(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  input.exceptions(std::ios::badbit);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void round_trip(const std::filesystem::path &path) {
  for (const auto version :
       {FixtureVersion::legacy, FixtureVersion::processing, FixtureVersion::debugging}) {
    const bool processing = version != FixtureVersion::legacy;
    BinaryCase item;
    if (processing) {
      item.state = ProcessingState::waiting;
      item.reset_actions = {{.pc = action_pc, .accepted = true}};
    }
    if (version == FixtureVersion::debugging) {
      item.state = ProcessingState::debugging;
      item.debug.software_occurrence = true;
    }
    write_fixture(path, {item}, version);
    const auto expected = golden(version);
    require(get(path) == expected, "Fixture wire bytes equal the independently authored golden");
    put(path, expected);
    const auto decoded = read_fixture(path);
    require(decoded.size() == 1 && decoded[0].initial == item.initial &&
                decoded[0].expected == item.expected && decoded[0].progress == item.progress &&
                decoded[0].fault == item.fault && decoded[0].state == item.state &&
                decoded[0].reset_actions == item.reset_actions &&
                decoded[0].debug_interrupt_mode == item.debug_interrupt_mode &&
                decoded[0].debug == item.debug,
            "Both schema versions decode independent golden records");
  }
  BinaryCase item;
  constexpr std::size_t maximum_actions = 65536;
  item.reset_actions.assign(maximum_actions, {.pc = action_pc, .accepted = false});
  item.fault.kind = FaultKind::host_event;
  write_fixture(path, {item}, FixtureVersion::processing);
  const auto decoded = read_fixture(path);
  require(decoded[0].reset_actions == item.reset_actions && decoded[0].fault == item.fault,
          "Version 2 admits its exact callback-count boundary and appended event fault");
}

void rejected(const std::filesystem::path &path, const std::vector<char> &bytes) {
  put(path, bytes);
  bool failed = false;
  try {
    static_cast<void>(read_fixture(path));
  } catch (const std::exception &) { failed = true; }
  require(failed, "Malformed fixture is rejected before it can supply expectations");
}

struct Corruption {
  std::size_t offset = 0;
  std::uint32_t value = 0;
};

void invalid_records(const std::filesystem::path &path) {
  constexpr std::array legacy{
      Corruption{.offset = 0},
      Corruption{.offset = 4, .value = 4},
      Corruption{.offset = 8, .value = excessive_count},
      Corruption{.offset = 16},
      Corruption{.offset = 16, .value = address_words + 1},
      Corruption{.offset = 52, .value = static_cast<Word>(FaultKind::host_event)},
      Corruption{.offset = 56, .value = 3},
      Corruption{.offset = 72, .value = 2}};
  for (const auto input : legacy) {
    auto bytes = golden(FixtureVersion::legacy);
    word(bytes, input.offset, input.value);
    rejected(path, bytes);
  }
  constexpr std::array processing{
      Corruption{.offset = 76, .value = 3},
      Corruption{.offset = 80, .value = excessive_count},
      Corruption{.offset = 84, .value = address_words},
      Corruption{.offset = 88, .value = 2},
      Corruption{.offset = 52, .value = static_cast<Word>(FaultKind::host_event) + 1}};
  for (const auto input : processing) {
    auto bytes = golden(FixtureVersion::processing);
    word(bytes, input.offset, input.value);
    rejected(path, bytes);
  }
  const auto complete = golden(FixtureVersion::processing);
  for (const std::size_t length : {0U, 1U, 7U, 11U, 12U, 20U, 75U, 76U, 80U, 83U, 84U, 88U, 91U}) {
    rejected(path, {complete.begin(), complete.begin() + static_cast<std::ptrdiff_t>(length)});
  }
  auto incomplete = complete;
  incomplete.pop_back();
  rejected(path, incomplete);
  auto trailing = complete;
  trailing.push_back(0);
  rejected(path, trailing);
}

void invalid_debug_records(const std::filesystem::path &path) {
  constexpr std::size_t debug_offset = state_offset + 16;
  constexpr std::array invalid{Corruption{.offset = state_offset, .value = 4},
                               Corruption{.offset = debug_offset, .value = 2},
                               Corruption{.offset = debug_offset + 4, .value = 2},
                               Corruption{.offset = debug_offset + 8, .value = 2}};
  for (const auto corruption : invalid) {
    auto bytes = golden(FixtureVersion::debugging);
    word(bytes, corruption.offset, corruption.value);
    rejected(path, bytes);
  }
  for (const auto version : {FixtureVersion::legacy, FixtureVersion::processing}) {
    for (unsigned choice = 0; choice < 3; ++choice) {
      BinaryCase item;
      if (choice == 0) { item.debug_interrupt_mode = true; }
      if (choice == 1) { item.debug.interrupt_mode = true; }
      if (choice == 2) { item.debug.software_occurrence = true; }
      bool failed = false;
      try {
        write_fixture(path, {item}, version);
      } catch (const std::exception &) { failed = true; }
      require(failed, "Older schema cannot silently omit debug control/status expectations");
    }
  }
}

void reject_legacy_extensions(const std::filesystem::path &path) {
  for (unsigned choice = 0; choice < 3; ++choice) {
    BinaryCase item;
    if (choice == 0) { item.state = ProcessingState::waiting; }
    if (choice == 1) { item.reset_actions = {{.pc = action_pc}}; }
    if (choice == 2) { item.fault.kind = FaultKind::host_event; }
    bool failed = false;
    try {
      write_fixture(path, {item});
    } catch (const std::exception &) { failed = true; }
    require(failed, "Legacy writer cannot silently drop version 2 expectations");
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_fixture_format_test TEMP_FILE"); }
    const std::filesystem::path path(argv[1]);
    dsp56300::test::round_trip(path);
    dsp56300::test::invalid_records(path);
    dsp56300::test::invalid_debug_records(path);
    dsp56300::test::reject_legacy_extensions(path);
    std::filesystem::remove(path);
    std::cout << dsp56300::test::checks << " independent fixture format checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
