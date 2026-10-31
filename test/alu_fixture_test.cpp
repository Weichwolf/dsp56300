// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "fixture_memory.hpp"
#include "support.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace dsp56300::test::alu_fixture_test {
namespace {
class Ram final : public Io {
public:
  explicit Ram(FixtureMemory memory) : words(std::move(memory)) {}

  FixtureMemory words;

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    const auto index = static_cast<std::size_t>(space);
    if (index >= words.size() || address >= words[index].size()) { return false; }
    value = words[index][address];
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    const auto index = static_cast<std::size_t>(space);
    if (index >= words.size() || address >= words[index].size()) { return false; }
    words[index][address] = value;
    return true;
  }

  bool bit(const Space space,
           const Word address,
           const Word pc,
           const Word bit,
           const BitAction action,
           bool &previous) noexcept override {
    Word value = 0;
    if (bit >= word_bits || !read(space, address, pc, value)) { return false; }
    const Word mask = Word{1} << bit;
    previous = (value & mask) != 0;
    switch (action) {
      case BitAction::clear: value &= ~mask; break;
      case BitAction::set: value |= mask; break;
      case BitAction::change: value ^= mask; break;
      case BitAction::test: return true;
      default: return false;
    }
    return write(space, address, pc, value);
  }
};

bool interruptible(Core &core, const Ram &ram) {
  try {
    return core.interruptible();
  } catch (const std::logic_error &) {
    // External REP-tail eligibility needs only the real next opcode, not an
    // invented or speculatively fetched extension. Outside owned RAM it is false.
    const auto request = core.instruction_fetch();
    const auto &program = ram.words[0];
    if (request.address >= program.size()) { return false; }
    return core.interruptible_opcode(
        {.address = request.address,
         .opcode = request.latched_opcode.value_or(program[request.address]) & word_mask});
  }
}

void update(Registers &state, const RegisterUpdate field) {
  if (field.storage == RegisterStorage::a || field.storage == RegisterStorage::b) {
    auto &target = field.storage == RegisterStorage::a ? state.a : state.b;
    require((target & field.mask) == field.previous,
            "First native data write retains actual pre-unit bits");
    target = (target & ~field.mask) | field.value;
    return;
  }
  Word *target = nullptr;
  if (field.storage == RegisterStorage::sr) {
    target = &state.sr;
  } else if (field.storage == RegisterStorage::r && field.index < agu_registers) {
    target = &state.r[field.index];
  }
  if (target == nullptr) { throw std::runtime_error("Unexpected native ALU physical storage"); }
  require((*target & field.mask) == field.previous,
          "First native control/AGU write retains actual pre-unit bits");
  *target = static_cast<Word>((*target & ~field.mask) | field.value);
}

bool advance(Core &core) {
  auto pending = core.pending_instruction();
  if (!pending) { throw std::runtime_error("Missing actual staged fixture effect"); }
  const auto before = core.registers();
  const auto progress = core.progress();
  auto after = before;
  const bool alu = pending->effect.kind == EffectKind::alu_update;
  if (alu) {
    const auto result = pending->effect.alu_update;
    if (!result) { throw std::runtime_error("Native ALU effect lost its owned result"); }
    require(result->count > 0 && result->count <= result->updates.size(),
            "Native ALU publication is a bounded actual result");
    for (unsigned index = 0; index < result->count; ++index) {
      const auto field = result->updates.at(index);
      require(field.mask != 0 && (field.previous & ~field.mask) == 0 &&
                  (field.value & ~field.mask) == 0,
              "Masked unit results contain no unrelated bits, including equal-value writers");
      update(after, field);
    }
    pending->effect.alu_update = AluWriteback{};
    pending->computed = {};
  }
  const auto result = core.advance_instruction(pending->ticket);
  if (alu) {
    require(!result.retired && core.registers() == after && core.progress() == progress,
            "Accepting an owned unit publishes only its native bits and never PC/loop/retirement");
  }
  return result.retired;
}

void instruction(Core &core) {
  static_cast<void>(core.begin_fetch(InstructionPublication::alu_updates));
  constexpr unsigned effect_limit = 65;
  for (unsigned count = 0; core.fault().kind == FaultKind::none; ++count) {
    if (count == effect_limit) {
      throw std::runtime_error("Fixture continuation exceeded its bounded effect ledger");
    }
    if (advance(core)) { return; }
  }
}

void check(const BinaryCase &item, const Backend backend) {
  require(item.reset_actions.empty(),
          "Selected ALU archives require no fabricated peripheral RESET acknowledgement");
  Ram ram(initial_memory(item));
  auto expected = ram.words;
  patch(expected, item.after);
  Core core(ram, {.backend = backend});
  core.set_debug_interrupt_mode(item.debug_interrupt_mode);
  core.set_registers(item.initial);
  for (Word retired = 0; retired < item.budget && core.fault().kind == FaultKind::none &&
                         core.processing_state() == ProcessingState::running;
       ++retired) {
    instruction(core);
  }
  require(core.progress() == item.progress && core.progress().instructions == item.dispatches,
          "Independent fixture retirement and documented base costs through real unit publication");
  require(core.fault() == item.fault && core.processing_state() == item.state &&
              core.debug_status() == item.debug,
          "Independent fixture fault and hidden execution/debug state");
  if (core.fault().kind == FaultKind::none) {
    require(core.registers() == item.expected && ram.words == expected,
            "Independently authored fixture oracle covers complete registers, CCR and every P/X/Y "
            "word");
    require(interruptible(core, ram) == item.interruptible,
            "Captured next instruction preserves real REP-tail interrupt eligibility");
  } else {
    require(!core.interruptible(), "Faulted unit execution admits no interrupt");
  }
}

void run(const std::filesystem::path &path) {
  const auto cases = read_fixture(path);
  require(!cases.empty(), "Owned independent unit archive contains actual cases");
  for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
    for (std::size_t index = 0; index < cases.size(); ++index) {
      try {
        check(cases[index], backend);
      } catch (const std::exception &error) {
        throw std::runtime_error(path.string() + ": case " + std::to_string(index) + " (" +
                                 cases[index].name + ", " + std::string(backend_name(backend)) +
                                 "): " + error.what());
      }
    }
  }
  std::cout << path.filename().string() << ": " << cases.size()
            << " cases per staged backend option\n";
}
}
} // namespace dsp56300::test::alu_fixture_test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc < 2) { throw std::runtime_error("Usage: dsp56300_alu_fixture_test FILE.d56 [...]"); }
    for (int index = 1; index < argc; ++index) {
      dsp56300::test::alu_fixture_test::run(argv[index]);
    }
    std::cout << dsp56300::test::checks << " independently expected ALU fixture checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
