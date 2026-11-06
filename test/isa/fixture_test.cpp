// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fixture_memory.hpp"
#include "support.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>

namespace dsp56300::test {
namespace {
class ResetScript final : public Io {
public:
  explicit ResetScript(const std::span<const ResetAction> actions) : actions_(actions) {}

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

  bool reset_peripherals(const Word pc) noexcept override {
    if (used_ == actions_.size()) {
      matched_ = false;
      return false;
    }
    const auto action = actions_[used_++];
    matched_ = matched_ && pc == action.pc;
    return action.accepted;
  }

  [[nodiscard]] bool complete() const noexcept { return matched_ && used_ == actions_.size(); }

private:
  std::span<const ResetAction> actions_;
  std::size_t used_ = 0;
  bool matched_ = true;
};

void check(const BinaryCase &item) {
  auto memory = initial_memory(item);
  auto expected = memory;
  patch(expected, item.after);
  ResetScript script(item.reset_actions);
  Core core(Memory::separate({.program = memory[0], .x = memory[1], .y = memory[2]}),
            item.reset_actions.empty() ? nullptr : &script);
  require(!item.debug_interrupt_mode, "Fixture must use the supported non-JTAG instrument profile");
  core.set_registers(item.initial);
  const auto result = core.run(item.budget);
  require(result.retired == item.retired, "Retired dispatch count");
  require(result.progress == item.progress, "Instruction count and documented base cost");
  require(result.fault == item.fault, "Expected fault context");
  require(result.state == item.state && core.state().execution == item.state,
          "Explicit running/WAIT/STOP/debug state");
  require(script.complete(), "RESET callback PC/order/count follows explicit host script");

  require(core.interrupt_ready() == item.interrupt_ready, "Interrupt admission boundary");
  // Failed instructions do not promise a committed architectural state.
  if (item.fault.kind == FaultKind::none) {
    require(core.registers() == item.expected, "Complete architectural register state");
    require(memory == expected, "All P/X/Y memory, including locations outside expected writes");
  }
}

void run(const std::filesystem::path &path) {
  const auto cases = read_fixture(path);
  require(!cases.empty(), "Fixture contains cases");
  for (std::size_t index = 0; index < cases.size(); ++index) {
    try {
      check(cases[index]);
    } catch (const std::exception &error) {
      throw std::runtime_error(path.string() + ": case " + std::to_string(index) + " (" +
                               cases[index].name + "): " + error.what());
    }
  }
  std::cout << path.filename().string() << ": " << cases.size()
            << " cases through public Core API\n";
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc < 2) { throw std::runtime_error("Usage: dsp56300_fixture_test FILE.d56 [...]"); }
    for (int index = 1; index < argc; ++index) { dsp56300::test::run(argv[index]); }
    std::cout << dsp56300::test::checks << " binary fixture checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
