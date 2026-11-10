// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "fetch.hpp"
#include "instruction.hpp"
#include "isa.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/preparation.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
template <isa::Opcode operation> constexpr Word opcode = static_cast<Word>(operation);
constexpr Word target = 0x123456;
constexpr Word unrecognized = 0x000007; // Reserved encoding; deliberately no ISA operation.
constexpr Word helper = 24, helper_return = helper + 1, absolute_ea = 0x30;

struct CodeView {
  std::map<Word, Word> words;
  std::size_t inspections = 0, executions = 0;

  static std::optional<Word> inspect(void *owner, const Word address) noexcept {
    auto &self = *static_cast<CodeView *>(owner);
    ++self.inspections;
    const auto found = self.words.find(address);
    return found == self.words.end() ? std::nullopt : std::optional(found->second);
  }

  static Word execute(void *owner, const detail::BlockInstruction * /*operation*/) noexcept {
    ++static_cast<CodeView *>(owner)->executions;
    return 0;
  }

  [[nodiscard]] detail::JitRuntime runtime() {
    return {
        .owner = this, .execute = &execute, .moves = &execute, .program = &inspect, .memory = {}};
  }
};

void reachable_graph() {
  CodeView view{.words = {{0, opcode<isa::Opcode::jmp_absolute_short> | 8U},
                          {1, unrecognized},
                          {8, opcode<isa::Opcode::neg_a>},
                          {9, opcode<isa::Opcode::jeq_absolute_short> | 16U},
                          {10, opcode<isa::Opcode::jsr_absolute_short> | helper},
                          {11, opcode<isa::Opcode::jmp_absolute_short> | 8U},
                          {16, opcode<isa::Opcode::rts>},
                          {helper, opcode<isa::Opcode::wait>},
                          {helper_return, opcode<isa::Opcode::rts>}}};
  const auto before = view.words;
  constexpr std::array<Word, 2> entries{0, 0};
  const auto graph = detail::preparation_graph({}, view.runtime(), entries);
  constexpr std::array<Word, 8> expected{0, 8, 9, 10, 16, 11, 24, 25};
  require(graph.failure == Fault{} && graph.instructions.size() == expected.size(),
          "Direct reachability deduplicates seeds, loop edges and call targets");
  for (std::size_t i = 0; i < expected.size(); ++i) {
    require(graph.instructions[i].pc == expected[i] &&
                graph.instructions[i].instruction.opcode == before.at(expected[i]),
            "Discovery follows explicit edges, never unconnected data or padding");
  }
  require(graph.unresolved == std::vector<Word>{16, helper, helper_return} &&
              view.inspections == expected.size() && view.executions == 0 && view.words == before,
          "Dynamic returns and wake continuations stay open without guest or device execution");
  const auto empty = detail::preparation_graph({}, view.runtime(), {});
  require(empty.instructions.empty() && empty.unresolved.empty() && empty.failure == Fault{} &&
              view.inspections == expected.size(),
          "An empty entry set never scans the program");
}

struct BranchInput {
  Word opcode = 0, extension = 0, pc = 0;
};

detail::Successors flow(const BranchInput input) {
  auto decoded = detail::decode(input.opcode);
  require(decoded.operation != detail::Operation::unsupported, "Authored branch must decode");
  decoded.extension = input.extension;
  return detail::successors({.instruction = decoded, .pc = input.pc});
}

void absolute_edges() {
  constexpr Word pc = word_mask;
  constexpr Word short_target = 0x345;
  for (const bool call : {false, true}) {
    const auto short_form =
        call ? opcode<isa::Opcode::jsr_absolute_short> : opcode<isa::Opcode::jmp_absolute_short>;
    const auto effective =
        call ? opcode<isa::Opcode::jsr_effective> : opcode<isa::Opcode::jmp_effective>;
    const auto direct = flow({.opcode = short_form | short_target, .extension = 0, .pc = pc});
    const auto extended = flow({.opcode = effective | (0x30U << 8), .extension = target, .pc = pc});
    const auto indirect = flow({.opcode = effective | (4U << 11), .extension = 0, .pc = pc});
    require(!direct.indirect && direct.count == (call ? 2U : 1U) &&
                direct.direct[call ? 1U : 0U] == short_target && (!call || direct.direct[0] == 0) &&
                !extended.indirect && extended.count == (call ? 2U : 1U) &&
                extended.direct[call ? 1U : 0U] == target && (!call || extended.direct[0] == 1) &&
                indirect.indirect && indirect.count == (call ? 1U : 0U) &&
                (!call || indirect.direct[0] == 0),
            "Absolute jumps have no fallthrough; calls retain wrapped return PCs and indirects");
    for (Word condition = 0; condition < 16; ++condition) {
      const auto conditional =
          call ? opcode<isa::Opcode::jscc_absolute_short> : opcode<isa::Opcode::jcc_absolute_short>;
      const auto branch = flow(
          {.opcode = conditional | (condition << 12) | short_target, .extension = 0, .pc = pc});
      require(branch.count == 2 && !branch.indirect && branch.direct[0] == 0 &&
                  branch.direct[1] == short_target,
              "Every conditional absolute transfer retains both possible successors");
    }
  }
}

void short_relative_edges(const Word pc, const bool call) {
  constexpr std::array<Word, 5> offsets{0, 1, 255, 256, 511};
  const auto base =
      call ? opcode<isa::Opcode::bscc_relative_short> : opcode<isa::Opcode::bcc_relative_short>;
  for (Word condition = 0; condition < 16; ++condition) {
    for (const auto offset : offsets) {
      const auto encoded = ((offset & 0x1e0U) << 1) | (offset & 0x1fU);
      const auto branch =
          flow({.opcode = base | (condition << 12) | encoded, .extension = 0, .pc = pc});
      const auto signed_offset = offset < 256 ? offset : word_mask - (511 - offset);
      require(branch.count == 2 && !branch.indirect && branch.direct[0] == ((pc + 1) & word_mask) &&
                  branch.direct[1] == ((pc + signed_offset) & word_mask),
              "Short relative edges use signed nine-bit displacement from instruction PC");
    }
  }
}

void relative_edges() {
  for (const Word pc : {0U, word_mask - 1}) {
    for (const bool call : {false, true}) {
      short_relative_edges(pc, call);
      const auto always =
          call ? opcode<isa::Opcode::bsr_relative_long> : opcode<isa::Opcode::bra_relative_long>;
      const auto direct = flow({.opcode = always, .extension = word_mask, .pc = pc});
      const auto relative_register = call ? opcode<isa::Opcode::bsr_relative_register>
                                          : opcode<isa::Opcode::bra_relative_register>;
      const auto indirect = flow({.opcode = relative_register, .extension = 0, .pc = pc});
      require(direct.count == (call ? 2U : 1U) && !direct.indirect &&
                  direct.direct[call ? 1U : 0U] == ((pc + word_mask) & word_mask) &&
                  (!call || direct.direct[0] == ((pc + 2) & word_mask)) && indirect.indirect &&
                  indirect.count == (call ? 1U : 0U),
              "Long relative offsets wrap while register targets remain unknown");
    }
  }
}

void capture_and_frontiers() {
  constexpr Word entry = word_mask;
  CodeView view{.words = {{entry, opcode<isa::Opcode::jmp_effective> | (absolute_ea << 8)},
                          {0, target},
                          {target, opcode<isa::Opcode::rts>}}};
  const auto before = view.words;
  const auto capture = detail::capture_instruction({}, entry, view.runtime());
  require(capture.failure == Fault{} && capture.instruction.words == 2 &&
              capture.instruction.extension == target && view.inspections == 2,
          "Code capture wraps the extension word without architectural instruction fetch");
  constexpr std::array entries{entry};
  const auto graph = detail::preparation_graph({}, view.runtime(), entries);
  require(graph.failure == Fault{} && graph.instructions.size() == 2 &&
              graph.instructions[1].pc == target && graph.unresolved == std::vector<Word>{target},
          "Wrapped direct jumps retain their independently established destination");
  view.words.erase(0);
  const auto missing = detail::capture_instruction({}, entry, view.runtime());
  const auto unresolved = detail::preparation_graph({}, view.runtime(), entries);
  require(missing.failure == Fault{.kind = FaultKind::memory_read,
                                   .space = Space::program,
                                   .address = 0,
                                   .pc = entry} &&
              unresolved.instructions.empty() && unresolved.failure == Fault{} &&
              unresolved.unresolved == std::vector<Word>{entry} && view.executions == 0,
          "Unavailable extensions retain exact diagnostics and an honest unresolved entry");
  view.words = before;
  view.words[entry] = unrecognized;
  const auto unsupported = detail::preparation_graph({}, view.runtime(), entries);
  require(unsupported.failure == Fault{.kind = FaultKind::unsupported_instruction,
                                       .space = Space::program,
                                       .address = entry,
                                       .value = unrecognized,
                                       .pc = entry},
          "Reachable unrecognized instructions retain exact fault context");
  constexpr std::array<Word, 1> invalid{word_mask + 1};
  bool rejected = false;
  try {
    static_cast<void>(detail::preparation_graph({}, view.runtime(), invalid));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "An invalid entry address cannot silently alias a valid PC");
}

void implicit_and_external_edges() {
  for (const auto instruction : {opcode<isa::Opcode::wait>,
                                 opcode<isa::Opcode::stop>,
                                 opcode<isa::Opcode::trap>,
                                 opcode<isa::Opcode::debug>,
                                 opcode<isa::Opcode::rep_immediate>,
                                 opcode<isa::Opcode::enddo>}) {
    const auto branch = flow({.opcode = instruction, .extension = 0, .pc = word_mask});
    require(branch.indirect && branch.count == 1 && branch.direct[0] == 0,
            "Implicit loop/REP, vector and wake continuations remain explicit frontiers");
  }
  for (const auto instruction : {opcode<isa::Opcode::rts>, opcode<isa::Opcode::rti>}) {
    const auto branch = flow({.opcode = instruction, .extension = 0, .pc = word_mask});
    require(branch.indirect && branch.count == 0, "Returns never invent a sequential successor");
  }
  const auto illegal =
      flow({.opcode = opcode<isa::Opcode::illegal>, .extension = 0, .pc = word_mask});
  require(illegal.indirect && illegal.count == 1 && illegal.direct[0] == 1,
          "ILLEGAL retains its specified PC+2 exception continuation");
  for (const bool relative : {false, true}) {
    const auto instruction =
        relative ? opcode<isa::Opcode::dor_immediate> : opcode<isa::Opcode::do_immediate>;
    const auto loop = flow({.opcode = instruction | (2U << 8), .extension = target, .pc = 8});
    require(loop.indirect && loop.count == 2 && loop.direct[0] == 10 &&
                loop.direct[1] == target + (relative ? 9U : 1U),
            "DO captures body and zero-count exit without inventing an implicit stack edge");
  }
  const auto absolute_bit =
      flow({.opcode = opcode<isa::Opcode::jclr_register>, .extension = target, .pc = word_mask});
  const auto relative_bit =
      flow({.opcode = opcode<isa::Opcode::brclr_register>, .extension = target, .pc = word_mask});
  require(absolute_bit.count == 2 && absolute_bit.direct[0] == 1 &&
              absolute_bit.direct[1] == target && relative_bit.count == 2 &&
              relative_bit.direct[0] == 1 && relative_bit.direct[1] == target - 1,
          "Bit-control edges are known without reading the tested register or device");
}

void owned_metadata_and_long_region() {
  constexpr std::size_t count = 1025;
  std::vector<Word> program(count, opcode<isa::Opcode::nop>);
  program.back() = opcode<isa::Opcode::jmp_absolute_short>;
  const auto original = program;
  constexpr std::array<Word, 2> entries{0, 0};
  const auto graph = detail::preparation_graph(program, {}, entries);
  const auto block = detail::parse_block(program, {}, {});
  require(graph.failure == Fault{} && graph.unresolved.empty() &&
              graph.instructions.size() == count && block.size() == count && program == original,
          "A closed region exceeds historical translation caps and never recompiles a backedge");
  for (std::size_t i = 0; i < count; ++i) {
    require(graph.instructions[i].pc == i && block[i].pc == i &&
                graph.instructions[i].instruction == block[i].instruction &&
                graph.instructions[i].instruction.words == 1 &&
                graph.instructions[i].instruction.cycles == (i + 1 == count ? 3 : 1),
            "Preparation and ordinary capture share independently checked word/cycle metadata");
  }
  const auto absent = detail::capture_instruction(program, static_cast<Word>(count), {});
  require(absent.failure == Fault{.kind = FaultKind::memory_read,
                                  .space = Space::program,
                                  .address = static_cast<Word>(count),
                                  .pc = static_cast<Word>(count)},
          "Owned code grants do not imply initialized words beyond their actual extent");
}
}

int main() try {
  reachable_graph();
  absolute_edges();
  relative_edges();
  capture_and_frontiers();
  implicit_and_external_edges();
  owned_metadata_and_long_region();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
