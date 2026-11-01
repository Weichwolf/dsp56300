// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <map>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace dsp56300::test::external_run {
inline constexpr Word origin = 0x100, datum = 0x123456, other = 0x654321;
inline constexpr Word load_x0 = 0x44f000, immediate_x0 = 0x44f400;
inline constexpr Word store_x0 = 0x447000, program_store = 0x077084;
inline constexpr Word load_post = 0x44d800, store_post = 0x445800;
inline constexpr Word address = 0x80, port = 0xffff80;
inline constexpr Word ce = Word{1} << 19, be = Word{1} << 10;
inline constexpr std::array backends{Backend::reference, Backend::cached, Backend::jit};
enum class Operation : unsigned char { read, write, bit, reset };

struct Event {
  Operation operation;
  Space space = Space::program;
  Word address = 0, pc = 0, value = 0;
  BitAction action = BitAction::test;
  bool operator==(const Event &) const = default;
};

class Bus final : public Io {
public:
  using Key = std::pair<Space, Word>;
  std::map<Key, Word> memory;
  std::vector<Event> events;
  std::optional<Key> refuse_read;
  std::optional<Key> refuse_write;
  bool refuse_bit = false, refuse_reset = false;
  bool record = true;
  std::size_t reads = 0, writes = 0, bits = 0, resets = 0;

  bool read(Space space, Word index, Word pc, Word &value) noexcept override;
  bool write(Space space, Word index, Word pc, Word value) noexcept override;
  bool bit(Space space, Word index, Word pc, Word bit, BitAction action, bool &previous) noexcept
      override;
  bool reset_peripherals(Word pc) noexcept override;
  void program(Word first, std::initializer_list<Word> words);
};

struct Workload {
  std::string_view name;
  Word opcode;
  std::uint64_t x, y;
};

inline constexpr std::array workloads{
    Workload{.name = "audio-mac", .opcode = 0x2000d2, .x = 0x100000, .y = 0x100000},
    Workload{.name = "image-add", .opcode = 0x200040, .x = 1, .y = 0},
    Workload{.name = "communications-xor", .opcode = 0x200043, .x = 0x123456, .y = 0}};

struct Timing {
  double seconds;
  std::uint64_t instructions;
  std::size_t reads;
};

[[nodiscard]] Timing workload(Workload input, Backend backend, bool staged, bool single_step);
[[nodiscard]] Timing captured_workload(Workload input, Backend backend);
void cache_checks();
void continuation_checks();
}
