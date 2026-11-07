// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fixture_memory.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/a64/emitter.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace dsp56300::test {
namespace {
std::uint32_t forbidden(void *const owner,
                        const detail::BlockInstruction * /*operation*/) noexcept {
  ++*static_cast<unsigned *>(owner);
  return 0;
}

void check(const BinaryCase &item) {
  require(item.budget == 1, "Transfer fixture must exercise one complete instruction");
  auto memory = initial_memory(item);
  auto expected_memory = memory;
  patch(expected_memory, item.after);
  const auto address = item.initial.pc;
  auto instruction = detail::decode(memory[0].at(address));
  if (instruction.words == 2) { instruction.extension = memory[0].at((address + 1) & word_mask); }
  const std::array operations{
      detail::BlockInstruction{.instruction = instruction,
                               .pc = address,
                               .mode = item.initial.sr & detail::numeric_mode_mask}};
  detail::a64::Emitter emitter;
  detail::ExecutableArena arena;
  const auto *entry = arena.append(emitter.generate(operations));
  detail::Cpu cpu;
  cpu.reg = item.initial;
  cpu.reg.a <<= detail::padding_bits;
  cpu.reg.b <<= detail::padding_bits;
  unsigned helpers = 0;
  detail::NativeContext context{
      .runtime = {.owner = &helpers,
                  .execute = forbidden,
                  .moves = forbidden,
                  .memory = {{{.data = memory[1].data(), .words = memory[1].size()},
                              {.data = memory[2].data(), .words = memory[2].size()}}}},
      .budget = 1,
      .bus = {.program = {.data = memory[0].data(), .words = memory[0].size()}}};
  const auto retired = detail::enter_block(entry, cpu, context);
  require(
      retired == item.retired && cpu.instructions == item.progress.instructions &&
          cpu.cycles == item.progress.estimated_cycles && helpers == 0,
      "Native transfer retirement/base costs must match independent fixtures without ISA helpers");
  if (context.failure != item.fault) {
    std::cerr << std::hex << "opcode=" << instruction.opcode
              << " fault=" << static_cast<unsigned>(context.failure.kind)
              << " expected=" << static_cast<unsigned>(item.fault.kind)
              << " address=" << context.failure.address
              << " expected_address=" << item.fault.address << " value=" << context.failure.value
              << " expected_value=" << item.fault.value << std::dec << '\n';
  }
  require(context.failure == item.fault && cpu.processing == item.state,
          "Native transfer fault identity and execution state follow independent fixtures");
  if (item.fault.kind == FaultKind::none) {
    auto expected = item.expected;
    expected.a <<= detail::padding_bits;
    expected.b <<= detail::padding_bits;
    require(cpu.reg == expected && memory == expected_memory,
            "Every native transfer preserves complete registers, stack and all P/X/Y memory");
  }
}
}
}

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::invalid_argument("Expected one transfer fixture path"); }
    const auto cases = dsp56300::test::read_fixture(argv[1]);
    dsp56300::test::require(!cases.empty(), "Transfer fixture must contain cases");
    for (std::size_t index = 0; index < cases.size(); ++index) {
      try {
        dsp56300::test::check(cases[index]);
      } catch (const std::exception &error) {
        throw std::runtime_error("Transfer fixture " + std::to_string(index) + " (" +
                                 cases[index].name + "): " + error.what());
      }
    }
    std::cout << cases.size() << " independent transfer fixtures passed directly through ARM64\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
