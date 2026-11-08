// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
constexpr Word origin = 0x800, destination = 0x100, other = 0x120, datum = 0x123456;
constexpr Word absolute_mode = 6, x0_register = 4;
constexpr Word store =
    isa::opcode<isa::Opcode::movem_effective>().front() | (absolute_mode << 11) | x0_register;
constexpr Word jump = isa::opcode<isa::Opcode::jmp_absolute_short>().front() | origin;

class Program final : public Io {
public:
  std::array<Word, 3> code{store, destination, jump};
  std::array<Word, origin> ram{};

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (address >= origin && address - origin < code.size()) { return code[address - origin]; }
    return address < ram.size() ? std::optional{ram[address]} : std::nullopt;
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    const auto word = inspect_program(address);
    if (space != Space::program || !word) { return false; }
    value = *word;
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    if (space != Space::program) { return false; }
    if (address >= origin && address - origin < code.size()) {
      code[address - origin] = value;
      return true;
    }
    if (address >= ram.size()) { return false; }
    ram[address] = value;
    return true;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }
};

Registers initialize(Core &core, const Word value = datum) {
  auto state = core.registers();
  state.pc = origin;
  state.x = value;
  core.set_registers(state);
  return state;
}

void writes_beside_code() {
  Program memory;
  Core core(Memory::external(), &memory);
  require(core.run(1).fault.kind == FaultKind::none, "Observe lower code before a real CPU reset");
  core.reset();
  const auto expected = initialize(core);
  const auto cold = core.run(2);
  require(cold.fault.kind == FaultKind::none && cold.retired == 2 && core.registers() == expected &&
              memory.ram[destination] == datum,
          "MOVEM writes the actual P destination and JMP returns with unchanged registers");
  const auto before = dsp56300::test::codegen::publications();
  constexpr unsigned passes = 128;
  for (unsigned pass = 0; pass < passes; ++pass) {
    const auto result = core.run(2);
    require(result.fault.kind == FaultKind::none && result.retired == 2 &&
                core.registers() == expected && memory.ram[destination] == datum,
            "Every repeated loader transaction preserves its exact architectural effects");
  }
  const auto after = dsp56300::test::codegen::publications();
  std::cout << "loader publications: " << before.count << " -> " << after.count
            << "; bytes: " << before.bytes << " -> " << after.bytes << '\n';
  require(before == after, "P writes outside observed code must retain translated loader entries");
}

void changed_code() {
  Program memory;
  memory.code[1] = origin + 1;
  Core core(Memory::external(), &memory);
  const auto expected = initialize(core, other);
  const auto first = core.run(2);
  require(first.fault.kind == FaultKind::none && first.retired == 2 &&
              core.registers() == expected && memory.code[1] == other && memory.ram[other] == 0,
          "MOVEM changes its own translated extension before returning to its entry");
  const auto before = dsp56300::test::codegen::publications();
  const auto second = core.run(2);
  require(second.fault.kind == FaultKind::none && second.retired == 2 &&
              core.registers() == expected && memory.ram[other] == other &&
              dsp56300::test::codegen::publications() != before,
          "The changed translated extension controls the next actual store destination");
}

void self_write() {
  for (const auto replacement : {isa::nop.front(), isa::stop.front()}) {
    Program memory;
    memory.code[1] = origin + 2;
    Core core(Memory::external(), &memory);
    auto expected = initialize(core, replacement);
    expected.pc = origin + 3;
    const auto result = core.run(2);
    const auto state =
        replacement == isa::stop.front() ? ExecutionState::stopped : ExecutionState::running;
    require(result.fault.kind == FaultKind::none && result.retired == 2 && result.state == state &&
                core.registers() == expected && memory.code[2] == replacement,
            "A guest write to the active block replaces JMP with the actual new instruction");
  }
}

}

int main() try {
  writes_beside_code();
  changed_code();
  self_write();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
