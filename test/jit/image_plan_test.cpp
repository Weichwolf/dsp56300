// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit/image_plan.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <exception>
#include <format>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
using namespace dsp56300;
using detail::CodeSpan;
using detail::plan_image;
using test::require;
template <isa::Opcode operation> constexpr Word opcode = static_cast<Word>(operation);
constexpr Word start = 0x100, helper = 0x800;
constexpr Word absolute = opcode<isa::Opcode::jmp_effective> | (0x30U << 8);
constexpr Word reserved = 0x000007; // Deliberate reserved encoding, not an ISA operation.

void rejected(const std::span<const CodeSpan> spans,
              const std::span<const Word> entries,
              const Word address,
              const std::string_view reason) {
  bool caught = false;
  try {
    static_cast<void>(plan_image(spans, entries));
  } catch (const std::invalid_argument &error) {
    const std::string_view text(error.what());
    caught = text.contains(std::format("P:${:06x}", address)) && text.contains(reason);
  }
  require(caught, "Invalid image sources fail with the affected program address and reason");
}

void explicit_streams() {
  const std::array call{opcode<isa::Opcode::jsr_absolute_short> | helper};
  const std::array body{opcode<isa::Opcode::nop>, opcode<isa::Opcode::rts>};
  const std::array dead{opcode<isa::Opcode::nop>, opcode<isa::Opcode::stop>};
  const std::array<CodeSpan, 3> spans{{{.address = helper, .words = body},
                                       {.address = start, .words = call},
                                       {.address = start + 8, .words = dead}}};
  constexpr std::array entries{start, helper, start};
  const auto plan = plan_image(spans, entries);
  const std::array<Word, 5> expected{start, start + 8, start + 9, helper, helper + 1};
  require(plan.instructions.size() == expected.size() &&
              plan.entries == std::vector<Word>{start, helper},
          "Inventory covers every declared instruction, including code unreachable from entries");
  std::size_t index = 0;
  for (const auto &site : plan.instructions) {
    require(site.first == expected[index++],
            "Instruction starts are unique and canonically sorted");
  }
  require(plan.find(start + 1) == nullptr && plan.find(start + 8) != nullptr &&
              plan.find(word_mask + 1) == nullptr,
          "Saved return addresses and readable inline tables are not inferred executable code");
  auto reverse = spans;
  std::ranges::reverse(reverse);
  require(plan_image(reverse, entries).instructions == plan.instructions,
          "Source ordering does not change the executable inventory");
}

void alternate_streams() {
  std::array long_jump{absolute, opcode<isa::Opcode::nop>};
  const std::array zero{opcode<isa::Opcode::stop>};
  const std::array<CodeSpan, 2> spans{
      {{.address = start, .words = long_jump}, {.address = 0, .words = zero}}};
  constexpr std::array entry{start};
  const auto ordinary = plan_image(spans, entry);
  require(ordinary.instructions.size() == 2 && ordinary.find(start + 1) == nullptr &&
              ordinary.find(start)->words == 2,
          "Extension words remain operands even when their values encode valid instructions");
  const std::array alternate{CodeSpan{.address = start, .words = long_jump},
                             CodeSpan{.address = start + 1, .words = std::span(long_jump).last(1)},
                             CodeSpan{.address = 0, .words = zero},
                             CodeSpan{.address = start, .words = long_jump}};
  const auto explicit_alternate = plan_image(alternate, entry);
  require(explicit_alternate.instructions.size() == 3 &&
              explicit_alternate.find(start + 1)->operation == detail::Operation::nop,
          "An explicitly declared overlapping stream adds its own valid starts once");
  long_jump[1] = helper;
  require(ordinary.find(start)->extension == 0,
          "The inventory owns instruction descriptors rather than borrowing mutable source words");
  rejected(spans, entry, start, "encoded target");
}

void input_faults() {
  constexpr std::array entry{start};
  constexpr std::array nop{opcode<isa::Opcode::nop>};
  constexpr std::array unknown{reserved};
  constexpr std::array wide{word_mask + 1};
  constexpr std::array jump{absolute};
  rejected(std::array{CodeSpan{.address = start, .words = unknown}}, entry, start, "unsupported");
  rejected(std::array{CodeSpan{.address = start, .words = wide}}, entry, start, "24 bits");
  rejected(std::array{CodeSpan{.address = start, .words = jump}}, entry, start, "extension");
  rejected(std::array{CodeSpan{.address = start, .words = {}}}, entry, start, "empty");
  rejected(std::array{CodeSpan{.address = word_mask + 1, .words = nop}},
           entry,
           word_mask + 1,
           "out-of-range");
  constexpr std::array absent{start + 1};
  rejected(std::array{CodeSpan{.address = start, .words = nop}}, absent, start + 1, "entry");
  constexpr std::array stop{opcode<isa::Opcode::stop>};
  rejected(std::array{CodeSpan{.address = start, .words = nop},
                      CodeSpan{.address = start, .words = stop}},
           entry,
           start,
           "disagree");
  const std::array broken{absolute, helper};
  rejected(std::array{CodeSpan{.address = start, .words = broken}}, entry, start, "encoded target");
  const std::array into_operand{absolute, start + 1};
  rejected(std::array{CodeSpan{.address = start, .words = into_operand}},
           entry,
           start,
           "encoded target");
}

void overwritten_operands() {
  constexpr std::array original{absolute, Word{0}};
  constexpr std::array replacement{opcode<isa::Opcode::stop>};
  constexpr std::array destination{opcode<isa::Opcode::stop>};
  const CodeSpan before{.address = start, .words = original};
  const CodeSpan after{.address = start + 1, .words = replacement};
  const CodeSpan target{.address = 0, .words = destination};
  constexpr std::array old_entry{start};
  constexpr std::array new_entry{start + 1};
  rejected(std::array{before, after, target}, old_entry, start + 1, "disagree");
  const auto old_variant = plan_image(std::array{before, target}, old_entry);
  const auto new_variant = plan_image(std::array{after}, new_entry);
  require(old_variant.find(start + 1) == nullptr && new_variant.find(start) == nullptr &&
              old_variant.find(start)->extension == 0 &&
              new_variant.find(start + 1)->operation == detail::Operation::stop,
          "Distinct code versions cannot be merged merely because their opcode starts differ");
}

void wrapping_and_extent() {
  constexpr std::array jump{absolute, Word{1}};
  constexpr std::array target{opcode<isa::Opcode::stop>};
  constexpr std::array entry{word_mask};
  const auto wrapped = plan_image(std::array{CodeSpan{.address = word_mask, .words = jump},
                                             CodeSpan{.address = 1, .words = target}},
                                  entry);
  require(wrapped.instructions.size() == 2 && wrapped.find(word_mask)->words == 2 &&
              wrapped.find(word_mask)->extension == 1 && wrapped.find(0) == nullptr,
          "A declared stream crossing the program boundary preserves wrapped operands");
  constexpr std::size_t length = 4097;
  std::vector<Word> program(length, opcode<isa::Opcode::nop>);
  program.back() = opcode<isa::Opcode::stop>;
  constexpr std::array first{start};
  const auto full = plan_image(std::array{CodeSpan{.address = start, .words = program}}, first);
  require(full.instructions.size() == length &&
              full.instructions.rbegin()->first == start + length - 1,
          "Every instruction is inventoried without an arbitrary region-length limit");
}
}

int main() {
  try {
    explicit_streams();
    alternate_streams();
    input_faults();
    overwritten_operands();
    wrapping_and_extent();
    std::cout << test::checks << " explicit executable-inventory checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
