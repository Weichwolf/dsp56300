// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "control_byte_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <exception>
#include <iostream>
#include <optional>

namespace dsp56300::test {
namespace {
class ModeIo final : public Io {
public:
  explicit ModeIo(const Word opcode) : opcode_(opcode) {}

  void bind(Core &core) { core_ = &core; }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (address == start) { return opcode_; }
    if (address == start + 1) {
      constexpr Word add = 0x200040;
      constexpr Word clear = 0x200013;
      return core_->registers().omr == 0 ? add : clear;
    }
    return std::nullopt;
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    ++reads_;
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    ++reads_;
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    ++reads_;
    return false;
  }

  void expect() const {
    require(reads_ == 0, "Known program views cause no fabricated bus accesses");
  }

private:
  Word opcode_;
  Core *core_ = nullptr;
  unsigned reads_ = 0;
};

void mode_byte(const byte_model::Field field) {
  const byte_model::Input input{.field = field, .immediate = 1, .inclusive_or = true};
  ModeIo io(byte_model::opcode(input));
  Core core(Memory::external(), &io);
  io.bind(core);
  auto initial = register_model::seed(0);
  initial.sr = initial.omr = 0;
  core.set_registers(initial);
  auto expected = initial;
  byte_model::apply(expected, input);
  ++expected.pc;
  const auto first = core.run(8);
  require(
      first.retired == 1 && first.yielded && first.fault.kind == FaultKind::none &&
          first.progress == Progress{.instructions = 1, .estimated_cycles = 3} &&
          core.registers() == expected,
      "OMR byte writes return after their actual instruction before a changed mapping executes");
  constexpr Word clear_byte = 0x13;
  a64_alu_model::apply(expected, clear_byte);
  ++expected.pc;
  const auto second = core.run(1);
  require(second.retired == 1 && !second.yielded && second.fault.kind == FaultKind::none &&
              second.progress == Progress{.instructions = 2, .estimated_cycles = 4} &&
              core.registers() == expected,
          "The next owner call executes the genuine changed program view");
  io.expect();
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    mode_byte(byte_model::Field::com);
    mode_byte(byte_model::Field::eom);
    std::cout << checks << " ARM64 public owner boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
