// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "staged_support.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace dsp56300::test::program_fetch_test {
namespace {
using staged_test::backends;
using staged_test::Bus;
using staged_test::pending;
using staged_test::rejected;
using staged_test::retire;
constexpr Word immediate_x0 = 0x44f400, payload = 0x800001;
constexpr Word repeat_twice = 0x0602a0, add_x0 = 0x200040, clear_a = 0x200013;
constexpr auto policy = InstructionPublication::alu_updates;

Registers initial(const Word pc) {
  auto state = staged_test::seed();
  state.pc = pc;
  return state;
}

void held(Core &core, const Registers &state) {
  require(core.registers() == state && core.progress() == Progress{},
          "Logical program reads and result publication hold unrelated CPU state and retirement");
}

void boundary(Core &core, const Word pc, const unsigned ordinal) {
  auto state = initial(pc);
  if (ordinal > 2) { state.x = payload; }
  held(core, state);
  const auto action = pending(core);
  if (ordinal < 2) {
    require(action.effect.kind ==
                    (ordinal == 0 ? EffectKind::fetch_opcode : EffectKind::fetch_extension) &&
                action.effect.space == Space::program && action.effect.pc == pc &&
                action.effect.address == (ordinal == 0 ? pc : (pc + 1) & word_mask),
            "Native opcode/extension tickets retain real PC, space and modulo24 address");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else if (ordinal == 2) {
    const MoveWriteback result{
        .selector = 4,
        .count = 1,
        .updates = {
            {{.storage = RegisterStorage::x, .mask = word_mask, .previous = 1, .value = payload}}}};
    require(action.effect.kind == EffectKind::move_update && action.effect.move_update == result,
            "Only a real accepted extension computes the native immediate MOVE");
    static_cast<void>(core.advance_instruction(action.ticket));
  } else {
    retire(core);
    state.pc = (pc + 2) & word_mask;
    require(core.registers() == state && core.progress().instructions == 1 &&
                core.fault().kind == FaultKind::none,
            "Complete independently expected instruction state survives P-address wrap");
  }
}

struct BackendPair {
  Backend from, to;
};

void restore(const Word pc, const BackendPair backend, const unsigned cut) {
  Bus bus;
  bus.pc = pc;
  bus.events = {{.space = Space::program, .address = pc, .value = immediate_x0},
                {.space = Space::program, .address = (pc + 1) & word_mask, .value = payload}};
  Core original(bus, {.backend = backend.from});
  original.set_registers(initial(pc));
  static_cast<void>(original.begin_fetch(policy));
  for (unsigned ordinal = 0; ordinal < cut; ++ordinal) { boundary(original, pc, ordinal); }
  const auto saved = original.checkpoint();
  const auto stale = pending(original);
  Bus resumed_bus = bus;
  Core resumed(resumed_bus, {.backend = backend.to});
  resumed.restore(saved);
  rejected([&] { static_cast<void>(resumed.advance_instruction(stale.ticket)); });
  original.restore(saved);
  rejected([&] { static_cast<void>(original.advance_instruction(stale.ticket)); });
  for (unsigned ordinal = cut; ordinal < 4; ++ordinal) {
    boundary(original, pc, ordinal);
    boundary(resumed, pc, ordinal);
  }
  require(bus.calls == 2 && resumed_bus.calls == 2 && bus.accepted == 2 &&
              resumed_bus.accepted == 2 && bus.exact && resumed_bus.exact,
          "Restoring every read/result boundary never rereads accepted program words");
}

void failure(const Backend backend, const bool extension, const Word pc) {
  Bus bus;
  bus.pc = pc;
  bus.events = {
      {.space = Space::program, .address = pc, .value = immediate_x0, .accept = extension}};
  if (extension) {
    bus.events.push_back(
        {.space = Space::program, .address = (pc + 1) & word_mask, .accept = false});
  }
  Core core(bus, {.backend = backend});
  const auto state = initial(pc);
  core.set_registers(state);
  static_cast<void>(core.begin_fetch(policy));
  if (extension) { boundary(core, pc, 0); }
  const auto request = pending(core);
  const auto failed = core.advance_instruction(request.ticket);
  require(failed.fault == Fault{.kind = FaultKind::memory_read,
                                .space = Space::program,
                                .address = extension ? (pc + 1) & word_mask : pc,
                                .pc = pc} &&
              !failed.retired && !failed.pending,
          "A real refused opcode/extension read produces the exact native memory fault, without "
          "dummy data");
  held(core, state);
  require(bus.calls == (extension ? 2U : 1U) && bus.accepted == (extension ? 1U : 0U) && bus.exact,
          "Failed fetch retains only actually accepted external reads");
}

void complete(Core &core) {
  while (const auto request = core.pending_instruction()) {
    if (core.advance_instruction(request->ticket).retired) { return; }
  }
  throw std::runtime_error("Expected real instruction retirement");
}

void cache_burst_checkpoint(const Backend backend, const InstructionPublication publication) {
  constexpr Word cache_enable = Word{1} << 19;
  constexpr Word burst_enable = Word{1} << 10;
  const Word pc = staged_test::boot;
  Bus bus;
  bus.pc = pc;
  for (Word offset = 0; offset < 4; ++offset) {
    bus.events.push_back({.space = Space::program, .address = pc + offset, .value = 0});
  }
  const Options options{.backend = backend, .cacheable_begin = pc, .cacheable_end = pc + 8};
  Core original(bus, options);
  auto state = initial(pc);
  state.sr |= cache_enable;
  state.omr |= burst_enable;
  original.set_registers(state);
  static_cast<void>(original.begin_fetch(publication));
  for (Word offset = 0; offset < 2; ++offset) {
    const auto action = pending(original);
    require(action.effect.kind == EffectKind::fetch_opcode && action.effect.address == pc + offset,
            "Burst reads real program words in order");
    static_cast<void>(original.advance_instruction(action.ticket));
  }
  const auto saved = original.checkpoint();
  Bus clone_bus = bus;
  Core clone(clone_bus, options);
  clone.restore(saved);
  complete(original);
  complete(clone);
  require(bus.calls == 4 && clone_bus.calls == 4 && bus.exact && clone_bus.exact,
          "Checkpoint continuation never repeats accepted burst reads");
  bus.pc = pc + 1;
  clone_bus.pc = pc + 1;
  static_cast<void>(original.begin_fetch(publication));
  static_cast<void>(clone.begin_fetch(publication));
  require(pending(original).effect.kind != EffectKind::fetch_opcode &&
              pending(clone).effect.kind != EffectKind::fetch_opcode && bus.calls == 4 &&
              clone_bus.calls == 4,
          "A cached next instruction needs no program-bus read after restore");
  complete(original);
  complete(clone);
  static_cast<void>(original.begin_instruction({.address = pc + 2, .opcode = 0}, publication));
  static_cast<void>(clone.begin_instruction({.address = pc + 2, .opcode = 0}, publication));
  require(pending(original).effect.kind != EffectKind::fetch_opcode &&
              pending(clone).effect.kind != EffectKind::fetch_opcode && bus.calls == 4 &&
              clone_bus.calls == 4,
          "Caller-supplied instructions with CE use no implicit program fetch");
  complete(original);
  complete(clone);
  static_cast<void>(original.begin_fetch(publication));
  static_cast<void>(clone.begin_fetch(publication));
  require(pending(original).effect.kind != EffectKind::fetch_opcode &&
              pending(clone).effect.kind != EffectKind::fetch_opcode && bus.calls == 4 &&
              clone_bus.calls == 4,
          "Caller-supplied instructions preserve previously cached words");
  complete(original);
  complete(clone);
}

void cache_burst_failure(const Backend backend, const Word refused) {
  constexpr Word cache_enable = Word{1} << 19;
  constexpr Word burst_enable = Word{1} << 10;
  const Word pc = staged_test::boot;
  Bus bus;
  bus.pc = pc;
  for (Word offset = 0; offset <= refused; ++offset) {
    bus.events.push_back(
        {.space = Space::program, .address = pc + offset, .value = 0, .accept = offset != refused});
  }
  const Options options{.backend = backend, .cacheable_begin = pc, .cacheable_end = pc + 8};
  Core original(bus, options);
  auto state = initial(pc);
  state.sr |= cache_enable;
  state.omr |= burst_enable;
  original.set_registers(state);
  static_cast<void>(original.begin_fetch());
  for (Word offset = 0; offset < refused; ++offset) {
    const auto action = pending(original);
    require(action.effect.address == pc + offset, "Accepted burst reads remain ordered");
    static_cast<void>(original.advance_instruction(action.ticket));
  }
  const auto saved = original.checkpoint();
  Bus clone_bus = bus;
  Core clone(clone_bus, options);
  clone.restore(saved);
  const auto failed = original.advance_instruction(pending(original).ticket);
  const auto clone_failed = clone.advance_instruction(pending(clone).ticket);
  const Fault expected{
      .kind = FaultKind::memory_read, .space = Space::program, .address = pc + refused, .pc = pc};
  require(failed.fault == expected && clone_failed.fault == expected && !failed.pending &&
              !clone_failed.pending && !failed.retired && !clone_failed.retired,
          "Rejected burst reports its exact program address before and after restore");
  require(bus.calls == refused + 1 && clone_bus.calls == refused + 1 && bus.accepted == refused &&
              clone_bus.accepted == refused && bus.exact && clone_bus.exact &&
              original.registers() == state && clone.registers() == state,
          "Rejected burst preserves accepted read count and CPU state across checkpoint");
}

void cache_flush_checkpoint(const Backend backend) {
  constexpr Word cache_enable = Word{1} << 19;
  constexpr Word burst_enable = Word{1} << 10;
  constexpr Word flush_all = 3;
  const Word pc = staged_test::boot;
  Bus bus;
  bus.pc = pc;
  bus.events = {{.space = Space::program, .address = pc, .value = 0},
                {.space = Space::program, .address = pc + 1, .value = flush_all},
                {.space = Space::program, .address = pc + 2, .value = 0},
                {.space = Space::program, .address = pc + 3, .value = 0},
                {.space = Space::program, .address = pc + 2, .value = 0},
                {.space = Space::program, .address = pc + 3, .value = 0}};
  const Options options{.backend = backend, .cacheable_begin = pc, .cacheable_end = pc + 8};
  Core original(bus, options);
  auto state = initial(pc);
  state.sr |= cache_enable;
  state.omr |= burst_enable;
  original.set_registers(state);
  static_cast<void>(original.begin_fetch());
  complete(original);
  static_cast<void>(original.begin_fetch());
  require(pending(original).effect.kind == EffectKind::retire && bus.calls == 4,
          "Flush instruction comes from cached burst without a bus request");
  const auto saved = original.checkpoint();
  Bus clone_bus = bus;
  Core clone(clone_bus, options);
  clone.restore(saved);
  retire(original);
  retire(clone);
  bus.pc = pc + 2;
  clone_bus.pc = pc + 2;
  static_cast<void>(original.begin_fetch());
  static_cast<void>(clone.begin_fetch());
  require(pending(original).effect.address == pc + 2 && pending(clone).effect.address == pc + 2,
          "Flushed program word requests external fetch after checkpoint restore");
  complete(original);
  complete(clone);
  require(bus.calls == 6 && clone_bus.calls == 6 && bus.exact && clone_bus.exact,
          "Flush invalidates cached burst on both checkpoint paths");
}

void cache_program_write(const Backend backend, const bool accepted) {
  constexpr Word cache_enable = Word{1} << 19;
  constexpr Word burst_enable = Word{1} << 10;
  constexpr Word movem_write_x0 = 0x077084;
  const Word pc = staged_test::boot;
  Bus bus;
  bus.pc = pc;
  bus.events = {{.space = Space::program, .address = pc, .value = movem_write_x0},
                {.space = Space::program, .address = pc + 1, .value = pc + 3},
                {.space = Space::program, .address = pc + 2, .value = 0},
                {.space = Space::program, .address = pc + 3, .value = 0},
                {.kind = EffectKind::write,
                 .space = Space::program,
                 .address = pc + 3,
                 .value = add_x0,
                 .accept = accepted}};
  const Options options{.backend = backend, .cacheable_begin = pc, .cacheable_end = pc + 8};
  Core original(bus, options);
  auto state = initial(pc);
  state.sr |= cache_enable;
  state.omr |= burst_enable;
  state.x = add_x0;
  original.set_registers(state);
  static_cast<void>(original.begin_fetch());
  for (unsigned offset = 0; offset < 4; ++offset) {
    static_cast<void>(original.advance_instruction(pending(original).ticket));
  }
  const auto write = pending(original);
  require(write.effect.kind == EffectKind::write && write.effect.address == pc + 3 &&
              write.effect.value == add_x0,
          "PMOVE offers the actual cached program address and data");
  const auto saved = original.checkpoint();
  Bus clone_bus = bus;
  Core clone(clone_bus, options);
  clone.restore(saved);
  const auto result = original.advance_instruction(write.ticket);
  const auto clone_result = clone.advance_instruction(pending(clone).ticket);
  if (!accepted) {
    const Fault expected{.kind = FaultKind::memory_write,
                         .space = Space::program,
                         .address = pc + 3,
                         .value = add_x0,
                         .pc = pc};
    require(result.fault == expected && clone_result.fault == expected && bus.accepted == 4 &&
                clone_bus.accepted == 4 && original.registers() == state &&
                clone.registers() == state,
            "Refused PMOVE fails without publishing CPU state across restore");
    return;
  }
  retire(original);
  retire(clone);
  for (Core *core : {&original, &clone}) {
    static_cast<void>(core->begin_fetch());
    complete(*core);
    static_cast<void>(core->begin_fetch());
    complete(*core);
    require(core->registers().a == (std::uint64_t{add_x0} << word_bits),
            "Accepted PMOVE changes the subsequently executed cached instruction");
  }
  require(bus.calls == 5 && clone_bus.calls == 5 && bus.exact && clone_bus.exact,
          "Accepted PMOVE and later cached fetches do not duplicate bus traffic");
}

void repeat(const Backend backend) {
  Bus bus;
  bus.events = {{.space = Space::program, .address = staged_test::boot, .value = repeat_twice},
                {.space = Space::program, .address = staged_test::boot + 1, .value = add_x0},
                {.space = Space::program, .address = staged_test::boot + 1, .value = clear_a}};
  Core core(bus, {.backend = backend});
  auto state = initial(staged_test::boot);
  core.set_registers(state);
  static_cast<void>(core.begin_fetch(policy));
  complete(core);
  bus.pc = staged_test::boot + 1;
  static_cast<void>(core.begin_fetch(policy));
  complete(core);
  require(bus.calls == 2 && bus.exact,
          "REP and the first body instruction each use their genuine opcode read");
  static_cast<void>(core.begin_fetch(policy));
  require(
      pending(core).effect.kind == EffectKind::alu_update,
      "Subsequent REP execution uses the native opcode latch and performs no speculative P reread");
  complete(core);
  state.a = std::uint64_t{2} << word_bits;
  state.sr = flag_u;
  state.pc += 2;
  require(core.registers() == state && core.progress().instructions == 3 && bus.calls == 2 &&
              bus.exact,
          "Original REP opcode and independent data/CCR/PC/LC state survive a changed potential P "
          "response");
  require(
      core.interruptible_opcode({.address = state.pc, .opcode = immediate_x0}) && bus.calls == 2,
      "REP-tail interrupt eligibility requires only the actual opcode, not an invented extension");
  rejected([&] {
    static_cast<void>(core.interruptible({.address = state.pc, .opcode = immediate_x0}));
  });
}

void invalid_repeat_fetch(const Backend backend,
                          const InstructionPublication publication,
                          const bool known_extension) {
  // FM13-160/A.3.8: a REP body is one instruction word, before any extension read.
  const Word pc = staged_test::boot;
  Bus bus;
  Core original(bus, {.backend = backend});
  original.set_registers(initial(pc));
  static_cast<void>(
      original.begin_instruction({.address = pc, .opcode = repeat_twice}, publication));
  complete(original);
  bus.pc = pc + 1;
  bus.events = {{.space = Space::program, .address = pc + 1, .value = immediate_x0}};
  if (known_extension) {
    bus.events.push_back({.space = Space::program, .address = pc + 2, .value = payload});
  }
  const auto state = original.registers();
  const auto progress = original.progress();
  const auto admitted = original.begin_fetch(publication);
  require(admitted.fault.kind == FaultKind::none &&
              pending(original).effect.kind == EffectKind::fetch_opcode,
          "An unloaded REP body requests its actual first opcode");
  const auto saved = original.checkpoint();
  Bus clone_bus = bus;
  Core clone(clone_bus, {.backend = backend});
  clone.restore(saved);
  for (Core *core : {&original, &clone}) {
    const auto result = core->advance_instruction(pending(*core).ticket);
    const Fault expected{.kind = FaultKind::invalid_instruction,
                         .space = Space::program,
                         .address = pc + 1,
                         .value = immediate_x0,
                         .pc = pc + 1};
    require(result.fault == expected && !result.pending && !result.retired &&
                core->registers() == state && core->progress() == progress,
            "Invalid two-word REP body must fail before requesting even a known extension");
    rejected([&] { static_cast<void>(core->begin_fetch(publication)); });
  }
  require(bus.calls == 1 && bus.accepted == 1 && bus.exact && clone_bus.calls == 1 &&
              clone_bus.accepted == 1 && clone_bus.exact,
          "Both checkpoint owners read only the real invalid opcode, never its extension");
}

void run() {
  for (const auto pc : {staged_test::boot, word_mask}) {
    for (const auto from : backends) {
      for (const auto to : backends) {
        for (unsigned cut = 0; cut < 4; ++cut) { restore(pc, {.from = from, .to = to}, cut); }
      }
      failure(from, false, pc);
      failure(from, true, pc);
    }
  }
  for (const auto backend : backends) { repeat(backend); }
  constexpr std::array publications{InstructionPublication::retirement,
                                    InstructionPublication::address_updates,
                                    InstructionPublication::move_updates,
                                    InstructionPublication::alu_updates,
                                    InstructionPublication::unit_updates,
                                    InstructionPublication::pcu_updates};
  for (const auto backend : backends) {
    for (const auto publication : publications) {
      cache_burst_checkpoint(backend, publication);
      for (const bool known_extension : {false, true}) {
        invalid_repeat_fetch(backend, publication, known_extension);
      }
    }
  }
  for (const auto backend : backends) { cache_flush_checkpoint(backend); }
  for (const auto backend : backends) {
    for (const bool accepted : {false, true}) { cache_program_write(backend, accepted); }
  }
  for (const auto backend : backends) {
    for (Word refused = 0; refused < 4; ++refused) { cache_burst_failure(backend, refused); }
  }
}
}
} // namespace dsp56300::test::program_fetch_test

int main() {
  try {
    dsp56300::test::program_fetch_test::run();
    std::cout << dsp56300::test::checks << " native program fetch assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
