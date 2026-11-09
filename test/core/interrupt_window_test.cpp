// SPDX-License-Identifier: GPL-3.0-only
#include "codegen_observer.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace dsp56300;
using dsp56300::test::require;
constexpr Word body = 128, masked = 0x300, mask_byte = 0xfc;
constexpr Word unmask = isa::opcode<isa::Opcode::andi_control>().front() | (mask_byte << 8);

class Requests final : public Io {
public:
  bool pending = true;
  mutable unsigned queries = 0;

  [[nodiscard]] bool interrupt_pending(const Word status) const noexcept override {
    ++queries;
    return pending && ((status >> 8U) & 3U) == 0;
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
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

void mask_boundary(const bool pending) {
  std::vector<Word> ram(body, isa::opcode<isa::Opcode::add_x0_a>().front());
  ram.push_back(unmask);
  ram.push_back(isa::nop.front());
  ram.push_back(isa::stop.front());
  Requests requests;
  requests.pending = pending;
  Core core(Memory::unified(ram), &requests);
  auto expected = core.registers();
  expected.x = 1;
  expected.sr = masked;
  core.set_registers(expected);
  expected.a = std::uint64_t{body} << dsp56300::test::word_bits;
  expected.sr = dsp56300::test::flag_u;
  expected.pc = pending ? body + 1 : body + 3;
  const auto before = dsp56300::test::codegen::publications();
  const auto result = core.run();
  require(result.fault.kind == FaultKind::none && result.retired == expected.pc &&
              core.registers() == expected && result.yielded == pending &&
              result.state == (pending ? ExecutionState::running : ExecutionState::stopped),
          "A pending IRQ returns exactly after unmasking, without executing the next instruction");
  require(requests.queries == 1, "Only the actual interrupt-mask write queries the peripheral");
  const auto after = dsp56300::test::codegen::publications();
  constexpr unsigned maximum_entries = 4;
  require(after.count - before.count <= maximum_entries,
          "A masked request preserves complete native regions instead of stepping their suffixes");
  if (pending) {
    require(core.interrupt_ready() && requests.pending,
            "The query neither admits nor acknowledges the actual request");
    requests.pending = false;
    const auto resumed = core.run();
    expected.pc += 2;
    require(resumed.fault.kind == FaultKind::none && resumed.retired == 2 &&
                resumed.state == ExecutionState::stopped && !resumed.yielded &&
                core.registers() == expected && requests.queries == 1,
            "A cleared request resumes the exact remaining prefix without per-instruction polls");
  }
}

void unchanged_mask() {
  constexpr Word all_flags = 0xff;
  constexpr Word ccr_field = 1;
  constexpr Word set_flags =
      isa::opcode<isa::Opcode::ori_control>().front() | (all_flags << 8) | ccr_field;
  constexpr Word clear_flags = isa::opcode<isa::Opcode::andi_control>().front() | ccr_field;
  auto ram = std::array{set_flags, clear_flags, isa::stop.front()};
  Requests requests;
  Core core(Memory::unified(ram), &requests);
  auto expected = core.registers();
  expected.sr = masked;
  core.set_registers(expected);
  expected.pc = ram.size();
  const auto result = core.run();
  require(result.retired == ram.size() && result.fault.kind == FaultKind::none &&
              result.state == ExecutionState::stopped && !result.yielded &&
              core.registers() == expected && requests.queries == 0,
          "CCR writes preserve the interrupt mask and do not poll pending requests");
}

void captured_mask() {
  auto ram = std::array{unmask, isa::stop.front()};
  Requests requests;
  Core core(Memory::unified(ram), &requests);
  auto expected = core.registers();
  expected.sr = masked;
  core.set_registers(expected);
  expected.sr = 0;
  expected.pc = 1;
  const auto result = core.run_instruction({.address = 0, .opcode = unmask});
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              result.state == ExecutionState::running && result.yielded &&
              core.registers() == expected && requests.queries == 1 && requests.pending,
          "Captured native execution reports the same completed mask-change boundary");
}
}

int main() try {
  mask_boundary(true);
  mask_boundary(false);
  unchanged_mask();
  captured_mask();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
