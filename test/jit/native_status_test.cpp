// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "native_sequence_model.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
constexpr Word datum = 0xfedcba, other_datum = 0x123456, port = 0xffff80;
constexpr Word x_address = 7, y_address = 11;
constexpr std::array modes{0U, mode_down, mode_up, mode_rm, mode_rm | mode_down, mode_rm | mode_up};
constexpr std::array<Word, 8> program{
    0x200036, // NEG A.
    0x566021, // ADC X,A with old A -> X:(R0).
    0x576011, // RND A with old B -> X:(R0).
    0xcb8010, // ADD B,A with X:(R0) -> A and Y:(R4) -> B, using both old accumulators.
    0x200021, // ADC X,A.
    0x56e0d2, // MAC X0,Y0,A with X:(R0) -> A, using old A.
    0x200039, // ADC Y,B.
    0x200019  // RND B.
};

struct Observation {
  Registers state;
  Progress progress{};
  Word value = 0;
  bool store = false;
};

struct Expected {
  Registers state;
  std::array<Word, memory_words> x{}, y{};
  std::array<Observation, 4> observations{};
  Progress progress{};
};

Expected model(const Registers &initial, const Word length, const bool io) {
  Expected result{.state = initial};
  result.x[x_address] = datum;
  result.y[y_address] = other_datum;
  std::size_t access = 0;
  for (Word index = 0; index < length; ++index) {
    const auto old = result.state;
    const bool store = index == 1 || index == 2;
    const bool load = index == 3 || index == 5;
    Word value = io ? datum : result.x[x_address];
    if (store) { value = register_model::read(result.state, index == 1 ? 14U : 15U); }
    if (io && (store || load)) {
      auto observed = result.state;
      ++observed.pc;
      result.observations[access++] = {
          .state = observed, .progress = result.progress, .value = value, .store = store};
    }
    if (store && !io) { result.x[x_address] = value; }
    if (load) { register_model::write(result.state, 14, value); }
    if (index == 3) { register_model::write(result.state, 15, result.y[y_address]); }
    sequence_model::alu(result.state, old, program[index]);
    ++result.state.pc;
    ++result.progress.instructions;
    ++result.progress.estimated_cycles;
  }
  return result;
}

struct Machine {
  explicit Machine(const bool optimize, Io *io = nullptr)
      : core({.program = p, .x = x, .y = y}, io, {.backend = Backend::jit, .optimize = optimize}) {
    core.load(Space::program, start, program);
  }

  void prepare(const Registers &state) {
    x.fill(0);
    y.fill(0);
    x[x_address] = datum;
    y[y_address] = other_datum;
    core.set_registers(state);
  }

  std::array<Word, memory_words> p{}, x{}, y{};
  Core core;
};

struct Input {
  Word mode = 0, flags = 0;
  unsigned seed = 0;
  int reject = -1;
};

Registers initial(const Input input) {
  constexpr std::uint64_t growth = std::uint64_t{1} << 46;
  auto state = register_model::seed(input.mode);
  state.sr = input.mode | input.flags;
  state.r[0] = x_address;
  state.r[4] = y_address;
  if (input.seed == 0) {
    state.a = 0;
    state.b = accumulator_mask;
  } else if (input.seed == 1) {
    state.a = sign56;
    state.b = sign56 - 1;
  } else if (input.seed == 2) {
    state.a = growth;
    state.b = sign56 - 1;
  }
  return state;
}

void prefixes(const bool optimize) {
  Machine machine(optimize);
  for (const auto mode : modes) {
    for (Word flags = 0; flags <= status_byte; ++flags) {
      for (unsigned seed = 0; seed < 4; ++seed) {
        const auto state = initial({.mode = mode, .flags = flags, .seed = seed});
        for (Word length = 1; length <= program.size(); ++length) {
          const auto expected = model(state, length, false);
          machine.prepare(state);
          const auto before = machine.core.progress();
          const auto result = machine.core.run(length);
          require(result.dispatches == length && result.fault.kind == FaultKind::none &&
                      machine.core.registers() == expected.state && machine.x == expected.x &&
                      machine.y == expected.y,
                  "Every warmed prefix preserves full SR, clipped stores and both old ALU inputs");
          require(result.progress.instructions == before.instructions + length &&
                      result.progress.estimated_cycles == before.estimated_cycles + length,
                  "Status holding preserves exact retirement across repeated entry boundaries");
        }
      }
    }
  }
  const auto counts = machine.core.jit_statistics();
  require((counts.native_alu_instructions != 0) == optimize &&
              (counts.native_move_instructions != 0) == optimize,
          "The matrix executes actual native ALU/transfers and the optimization-disabled fallback");
}

class Observer final : public Io {
public:
  Core *core = nullptr;
  std::array<Observation, 4> expected;
  Progress base{};
  unsigned accesses = 0;
  int reject = -1;
  bool valid = true;

  bool read(const Space space, const Word address, const Word pc, Word &value) noexcept override {
    value = datum;
    return observe(space, address, pc, value, false);
  }

  bool
  write(const Space space, const Word address, const Word pc, const Word value) noexcept override {
    return observe(space, address, pc, value, true);
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

private:
  bool observe(const Space space,
               const Word address,
               const Word pc,
               const Word value,
               const bool store) noexcept {
    const auto index = accesses++;
    auto *const current = core;
    if (current == nullptr || index >= expected.size()) {
      valid = false;
      return false;
    }
    const auto &snapshot = expected[index];
    const auto progress = current->progress();
    valid = valid && space == Space::x && address == port && pc + 1 == snapshot.state.pc &&
            store == snapshot.store && value == snapshot.value &&
            current->registers() == snapshot.state &&
            progress.instructions == base.instructions + snapshot.progress.instructions &&
            progress.estimated_cycles == base.estimated_cycles + snapshot.progress.estimated_cycles;
    return valid && reject != static_cast<int>(index);
  }
};

void callbacks(const Input input) {
  Observer observer;
  Machine machine(true, &observer);
  auto state = initial(input);
  machine.prepare(state);
  const auto budget = static_cast<Word>(program.size());
  require(machine.core.run(budget).dispatches == budget,
          "Warm every native RAM transfer before selecting its slow helper branch");
  state.r[0] = port;
  machine.prepare(state);
  const auto expected = model(state, budget, true);
  observer.core = &machine.core;
  observer.expected = expected.observations;
  observer.base = machine.core.progress();
  observer.reject = input.reject;
  const auto result = machine.core.run(budget);
  require(observer.valid && observer.accesses ==
                                (input.reject < 0 ? 4U : static_cast<unsigned>(input.reject) + 1),
          "All observers see complete SR after native arithmetic and earlier slow clipping stores");
  if (input.reject < 0) {
    require(
        result.dispatches == budget && result.fault.kind == FaultKind::none &&
            machine.core.registers() == expected.state && machine.x == expected.x &&
            machine.y == expected.y,
        "Slow helpers reload changed S/L bits before subsequent carry and arithmetic consumers");
  } else {
    const auto &stopped = expected.observations[static_cast<std::size_t>(input.reject)];
    const auto progress = machine.core.progress();
    require(
        result.dispatches == stopped.progress.instructions &&
            result.fault.kind == FaultKind::io_write && result.fault.pc + 1 == stopped.state.pc &&
            machine.core.registers() == stopped.state &&
            progress.instructions == observer.base.instructions + stopped.progress.instructions &&
            progress.estimated_cycles ==
                observer.base.estimated_cycles + stopped.progress.estimated_cycles,
        "Every rejected store preserves its exact partial state and newly sticky status");
    require(machine.core.run(budget).dispatches == 0 && machine.core.progress() == progress,
            "A failed status checkpoint cannot replay previously retired work");
  }
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    dsp56300::test::prefixes(true);
    dsp56300::test::prefixes(false);
    for (const auto mode : dsp56300::test::modes) {
      for (unsigned seed = 0; seed < 4; ++seed) {
        for (const int reject : {-1, 0, 1}) {
          dsp56300::test::callbacks({.mode = mode, .seed = seed, .reject = reject});
        }
      }
    }
    std::cout << dsp56300::test::checks << " native status/old-input/callback checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
