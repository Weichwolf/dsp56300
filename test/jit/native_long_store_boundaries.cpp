// SPDX-License-Identifier: GPL-3.0-only
#include "core_access.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_long_store_fixture.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300::test::native_long {
namespace {
class Memory final : public Io {
public:
  bool read(const Space space, const Word target, Word /*pc*/, Word &value) noexcept override {
    const auto code = inspect_program(target);
    if (space != Space::program || !code) { return false; }
    value = *code;
    return true;
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word target) const noexcept override {
    return target == 0 ? std::optional<Word>(instruction) : std::nullopt;
  }

  bool
  write(const Space space, const Word target, const Word pc, const Word value) noexcept override {
    struct Access {
      Space space;
      Word address, pc, value;
    };

    const Access access{.space = space, .address = target, .pc = pc, .value = value};
    const auto bank = writes++;
    if (bank >= data.size()) { return false; }
    valid = valid && access.space == (bank == 0 ? Space::x : Space::y) && access.address == port &&
            access.pc == 0;
    data.at(bank) = access.value;
    return writes != fail_at;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  std::span<const MemoryWindow> memory_windows(const Space space, Word /*mode*/) noexcept override {
    return mapped ? std::span<const MemoryWindow>(windows.at(space == Space::x ? 0 : 1))
                  : std::span<const MemoryWindow>{};
  }

  std::array<Word, 2> data{};
  std::array<std::uint8_t, 2> known{};
  std::array<std::array<MemoryWindow, 1>, 2> windows{
      std::array{MemoryWindow{.address = address,
                              .words = std::span(data).first(1),
                              .known = std::span(known).first(1),
                              .writable = true}},
      std::array{MemoryWindow{.address = address,
                              .words = std::span(data).last(1),
                              .known = std::span(known).last(1),
                              .writable = true}}};
  Word port = 0xffff80;
  Word instruction = 0;
  unsigned writes = 0, fail_at = 0;
  bool mapped = false, valid = true;
};

void peripheral(const unsigned failure) {
  const Input input{.mode = 3};
  std::array<Word, 1> program{opcode(input)};
  Memory memory;
  memory.fail_at = failure;
  std::array<Word, 1> x{};
  std::array<Word, 1> y{};
  TestCore cpu({.program = program, .x = x, .y = y}, &memory, {.backend = Backend::jit});
  auto initial = seed(input);
  initial.r[0] = memory.port;
  cpu.set_registers(initial);
  auto final = expected(input);
  final.state.r[0] = memory.port + 1;
  const auto run = cpu.run(1);
  const auto retired = failure == 0 ? 1U : 0U;
  require(run.retired == retired && cpu.registers() == final.state && memory.valid &&
              memory.writes == (failure == 1 ? 1U : 2U) && memory.data[0] == final.data[0] &&
              memory.data[1] == (failure == 1 ? 0U : final.data[1]) &&
              cpu.progress() == Progress{.instructions = retired, .estimated_cycles = retired} &&
              run.fault.kind == (failure == 0 ? FaultKind::none : FaultKind::io_write),
          "Long-store side exits preserve ordered callbacks, first fault and one AGU update");
}

void windows(const unsigned selector) {
  const Input input{.selector = selector, .mode = 3};
  Memory memory;
  memory.mapped = true;
  memory.instruction = opcode(input);
  TestCore cpu(memory, {.backend = Backend::jit});
  cpu.set_registers(seed(input));
  const auto final = expected(input);
  const auto run = cpu.run(1);
  require(
      run.retired == 1 && run.fault.kind == FaultKind::none && cpu.registers() == final.state &&
          memory.data == final.data && memory.known == std::array<std::uint8_t, 2>{1, 1} &&
          memory.writes == 0,
      "Sparse long stores map both host pointers from one guest EA and publish both known bits");
}

void shared_data() {
  const Input input{.selector = 2, .mode = 3};
  std::array<Word, 1> program{opcode(input)};
  std::array<Word, words> data{};
  auto final_data = data;
  const auto final = expected(input);
  final_data.at(address) = final.data[1];
  TestCore cpu({.program = program, .x = data, .y = data}, nullptr, {.backend = Backend::jit});
  cpu.set_registers(seed(input));
  const auto run = cpu.run(1);
  require(run.retired == 1 && run.fault.kind == FaultKind::none && cpu.registers() == final.state &&
              data == final_data,
          "Long X/Y host aliases capture old data before ordered stores; the Y half wins");
}
}

void boundary_cases() {
  for (unsigned failure = 0; failure < 3; ++failure) { peripheral(failure); }
  for (unsigned selector = 0; selector < 8; ++selector) { windows(selector); }
  shared_data();
}
} // namespace dsp56300::test::native_long
