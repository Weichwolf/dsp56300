// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fixture_memory.hpp"
#include "isa.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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

void check(const BinaryCase &item, const NativeImage &image, const std::uint64_t variant) {
  auto memory = initial_memory(item);
  auto expected = memory;
  patch(expected, item.after);
  ResetScript script(item.reset_actions);
  Core core(Memory::separate({.program = memory[0], .x = memory[1], .y = memory[2]}),
            item.reset_actions.empty() ? nullptr : &script);
  require(!item.debug_interrupt_mode, "Fixture must use the supported non-JTAG instrument profile");
  core.install(image, variant);
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

void declare_end_nop(BinaryCase &item) {
  const auto memory = initial_memory(item);
  for (auto &block : item.before) {
    if (block.space != Space::program || block.address != item.initial.pc) { continue; }
    const auto following = std::size_t{block.address} + block.words.size();
    require(following < memory[0].size() && memory[0][following] == isa::nop[0],
            "The declared successor must be an actual resident NOP, not invented padding");
    block.words.push_back(memory[0][following]);
    return;
  }
  throw std::runtime_error("Missing authored program for the declared resident successor");
}

template <isa::Opcode operation> constexpr Word op = static_cast<Word>(operation);

// Authored reserved source words, not a classifier of arithmetic input values.
constexpr auto cold_encodings = [] {
  std::array<Word, 28> words{};
  std::size_t next = 0;
  for (const auto prefix : {op<isa::Opcode::asl_register>,
                            op<isa::Opcode::asr_register>,
                            op<isa::Opcode::lsl_register>,
                            op<isa::Opcode::lsr_register>,
                            op<isa::Opcode::extract_register>,
                            op<isa::Opcode::extractu_register>,
                            op<isa::Opcode::insert_register> | (4U << 4),
                            op<isa::Opcode::merge_register>}) {
    for (const Word selector : {0U, 1U}) { words[next++] = prefix | (selector << 1); }
  }
  for (const Word selector : {0U, 1U}) {
    for (const Word destination : {0U, 1U}) {
      words[next++] = op<isa::Opcode::normf_register> | (selector << 1) | destination;
    }
    words[next++] = op<isa::Opcode::insert_immediate> | (selector << 4);
  }
  for (const Word selector : {1U, 2U, 3U}) {
    for (const Word destination : {0U, 1U}) {
      words[next++] = op<isa::Opcode::cmpu_register> | (selector << 1) | destination;
    }
  }
  return words;
}();

bool cold_candidate(const BinaryCase &item) {
  if (item.fault.kind != FaultKind::invalid_instruction) { return false; }
  const auto source = executable_source(item);
  return item.budget == 1 && item.retired == 0 && item.progress == Progress{} &&
         !source.second.empty() && item.fault.space == Space::program &&
         item.fault.pc == source.first && item.fault.address == source.first &&
         item.fault.value == source.second.front() &&
         std::ranges::find(cold_encodings, source.second.front()) != cold_encodings.end();
}

void refuse_sources(const std::span<const BinaryCase> cases) {
  const auto sources = fixture_catalogue(cases);
  for (const auto &[source, id] : sources) {
    const std::array spans{CodeSpan{.address = source.first, .words = source.second}};
    const std::array entries{source.first};
    const std::array variants{
        CodeVariant{.id = id, .code = spans, .entries = entries, .failed_fetches = {}}};
    bool rejected = false;
    try {
      static_cast<void>(Core::lower(
          {.variants = variants},
          {.memory = {}, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}}));
    } catch (const std::invalid_argument &error) {
      const std::string_view reason(error.what());
      require(reason.contains(std::format("P:${:06x}", source.first)) &&
                  reason.contains("unsupported instruction encoding"),
              "Cold source refusal identifies the actual address and unsupported encoding");
      rejected = true;
    }
    require(rejected, "Invalid source cannot form a partially executable prepared image");
  }
  std::cout << cases.size() << " invalid-encoding inputs represented by " << sources.size()
            << " distinct cold source refusals\n";
}

void run(const std::filesystem::path &path,
         const bool resident_successor,
         const bool runtime_errors) {
  auto input = read_fixture(path);
  if (runtime_errors) {
    std::erase_if(input, [](const BinaryCase &item) {
      return item.fault.kind != FaultKind::invalid_instruction || cold_candidate(item);
    });
    std::cout << path.filename().string() << ": native runtime-error subset\n";
  }
  require(!input.empty(), "Fixture contains cases");
  const auto refused = std::ranges::stable_partition(
      input, [](const BinaryCase &item) { return !cold_candidate(item); });
  const std::span cases(input.begin(), refused.begin());
  if (!refused.empty()) { refuse_sources(refused); }
  require(!cases.empty(), "Fixture retains executable cases after cold source refusals");
  // These counted-loop fixtures end at a real zero-initialized P word. Declare
  // that specific NOP so completed REP admission has prepared next-op metadata.
  if (resident_successor) {
    for (auto &item : cases) { declare_end_nop(item); }
  }
  const auto catalogue = fixture_catalogue(cases);
  std::cout << path.filename().string() << ": preparing " << catalogue.size()
            << " authored program variants\n"
            << std::flush;
  const auto started = std::chrono::steady_clock::now();
  const auto image = compile_fixture(catalogue);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  std::cout << path.filename().string()
            << ": preparation=" << std::chrono::duration<double>(elapsed).count() << "s\n"
            << std::flush;
  for (std::size_t index = 0; index < cases.size(); ++index) {
    try {
      check(cases[index], image, catalogue.at(executable_source(cases[index])));
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
    bool successor = false;
    bool runtime_errors = false;
    int first = 1;
    for (; first < argc; ++first) {
      const std::string_view argument(argv[first]);
      if (argument == "--declare-end-nop") {
        successor = true;
      } else if (argument == "--runtime-errors") {
        runtime_errors = true;
      } else {
        break;
      }
    }
    if (argc <= first) {
      throw std::runtime_error(
          "Usage: dsp56300_fixture_test [--declare-end-nop] [--runtime-errors] FILE.d56 [...]");
    }
    for (int index = first; index < argc; ++index) {
      dsp56300::test::run(argv[index], successor, runtime_errors);
    }
    std::cout << dsp56300::test::checks << " binary fixture checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
