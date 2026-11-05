// SPDX-License-Identifier: GPL-3.0-only
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"
#include "fixture_memory.hpp"
#include "support.hpp"
#include "unit_fixture_support.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace dsp56300::test::alu_fixture_test {
namespace {
class Ram final : public Io {
public:
  explicit Ram(FixtureMemory memory, const std::span<const ResetAction> actions = {})
      : words(std::move(memory)), resets(actions) {}

  FixtureMemory words;
  std::span<const ResetAction> resets;
  std::size_t reset_used = 0, reset_requests = 0;
  bool reset_exact = true;

  bool reset_peripherals(const Word pc) noexcept override {
    ++reset_requests;
    // Memory transport can exist while its peripheral RESET participant is absent.
    if (resets.empty()) { return false; }
    if (reset_used == resets.size()) {
      reset_exact = false;
      return false;
    }
    const auto action = resets[reset_used++];
    reset_exact = reset_exact && pc == action.pc;
    return reset_exact && action.accepted;
  }

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
  if (pending->effect.kind == EffectKind::pcu_update) {
    const auto result = pending->effect.pcu_update;
    if (!result || result->count == 0 || result->count > result->updates.size()) {
      throw std::runtime_error("Native PCU effect lost its bounded result");
    }
    for (unsigned index = 0; index < result->count; ++index) {
      unit_fixture_test::apply(after, result->updates.at(index));
    }
    pending->effect.pcu_update = PcuWriteback{};
    pending->computed = {};
    const auto advanced = core.advance_instruction(pending->ticket);
    require(!advanced.retired && core.registers() == after && core.progress() == progress,
            "Only the stored native PCU result publishes its selected register prefix");
    return false;
  }
  if (pending->effect.kind == EffectKind::unit_update) {
    const auto result = pending->effect.unit_update;
    if (!result) { throw std::runtime_error("Native scalar-unit effect lost its owned result"); }
    require(result->count > 0 && result->count <= result->updates.size(),
            "A scalar-unit publication has its bounded actual writer set");
    for (unsigned index = 0; index < result->count; ++index) {
      unit_fixture_test::apply(after, result->updates.at(index));
    }
    pending->effect.unit_update = UnitWriteback{};
    pending->computed = {};
    const auto result_progress = core.advance_instruction(pending->ticket);
    require(!result_progress.retired && core.registers() == after && core.progress() == progress,
            "The accepted native scalar unit publishes only its owned CPU prefix");
    return false;
  }
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

InstructionProgress
begin(Core &core, const Ram &ram, const InstructionPublication policy, const bool captured) {
  constexpr Word cache_enable = Word{1} << 19;
  if (!captured || (core.registers().sr & cache_enable) != 0) { return core.begin_fetch(policy); }
  const auto request = core.instruction_fetch();
  const auto &program = ram.words[0];
  if (request.exception_vector || request.address >= program.size()) {
    return core.begin_fetch(policy);
  }
  const auto opcode = request.latched_opcode.value_or(program[request.address]) & word_mask;
  const auto shape = inspect_instruction(opcode);
  if (!shape) { return core.begin_fetch(policy); }
  InstructionWords words{.address = request.address, .opcode = opcode};
  if (shape->words == 2) {
    const auto next = (request.address + 1) & word_mask;
    if (next >= program.size()) { return core.begin_fetch(policy); }
    words.extension = program[next] & word_mask;
  }
  return core.begin_instruction(words, policy);
}

void instruction(Core &core,
                 const Ram &ram,
                 const InstructionPublication policy,
                 const bool captured) {
  static_cast<void>(begin(core, ram, policy, captured));
  constexpr unsigned effect_limit = 65;
  for (unsigned count = 0; core.fault().kind == FaultKind::none; ++count) {
    if (count == effect_limit) {
      throw std::runtime_error("Fixture continuation exceeded its bounded effect ledger");
    }
    if (advance(core)) { return; }
  }
}

void check(const BinaryCase &item,
           const Backend backend,
           const InstructionPublication policy,
           const bool captured) {
  if (policy != InstructionPublication::pcu_updates) {
    require(item.reset_actions.empty(),
            "Selected ALU archives require no fabricated peripheral RESET acknowledgement");
  }
  Ram ram(initial_memory(item), item.reset_actions);
  auto expected = ram.words;
  patch(expected, item.after);
  Core core(ram, {.backend = backend});
  core.set_debug_interrupt_mode(item.debug_interrupt_mode);
  core.set_registers(item.initial);
  for (Word retired = 0; retired < item.budget && core.fault().kind == FaultKind::none &&
                         core.processing_state() == ProcessingState::running;
       ++retired) {
    instruction(core, ram, policy, captured);
  }
  if (policy == InstructionPublication::pcu_updates) {
    require(ram.reset_exact && ram.reset_used == item.reset_actions.size(),
            "Real RESET callbacks preserve explicit expected PC/order/count and acceptance");
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

void run(const std::filesystem::path &path,
         const InstructionPublication policy,
         const bool captured) {
  const auto cases = read_fixture(path);
  require(!cases.empty(), "Owned independent unit archive contains actual cases");
  for (const auto backend : {Backend::reference, Backend::cached, Backend::jit}) {
    for (std::size_t index = 0; index < cases.size(); ++index) {
      try {
        check(cases[index], backend, policy, captured);
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
    const bool units = std::string_view(argv[1]) == "--units";
    const bool captured = std::string_view(argv[1]) == "--captured-pcu";
    const bool pcu = std::string_view(argv[1]) == "--pcu" || captured;
    auto policy = dsp56300::InstructionPublication::alu_updates;
    if (units) { policy = dsp56300::InstructionPublication::unit_updates; }
    if (pcu) { policy = dsp56300::InstructionPublication::pcu_updates; }
    const int first = units || pcu ? 2 : 1;
    if (first == argc) { throw std::runtime_error("No owned fixture archive supplied"); }
    for (int index = first; index < argc; ++index) {
      dsp56300::test::alu_fixture_test::run(argv[index], policy, captured);
    }
    std::cout << dsp56300::test::checks << " independently expected ALU fixture checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
