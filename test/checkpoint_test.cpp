// SPDX-License-Identifier: GPL-3.0-only
#include "checkpoint_support.hpp"
#include "dsp56300/backend.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace dsp56300::test::checkpoint_test {
namespace {
constexpr Word port = 0xffff80, datum = 0x123456, load_post = 0x44d800;
static_assert(CoreImplementation<Core>);
static_assert(CoreImplementation<BackendCore<Backend::reference>>);
static_assert(CoreImplementation<BackendCore<Backend::cached>>);
static_assert(CoreImplementation<BackendCore<Backend::jit>>);
static_assert(CoreImplementation<BackendCore<Backend::jit, BatchBackend::disabled>>);
static_assert(CoreImplementation<BackendCore<Backend::jit, BatchBackend::scalar>>);
static_assert(CoreImplementation<BackendCore<Backend::jit, BatchBackend::avx2>>);

void lifetime() {
  struct Saved {
    Memory memory;
    Checkpoint cpu;
  };

  const auto saved = [] {
    Machine source(Backend::reference);
    source.core.set_registers(seed());
    source.core.write(Space::program, start, add_x0);
    return Saved{.memory = source.memory, .cpu = source.core.checkpoint()};
  }();
  Machine target(Backend::jit);
  target.memory = saved.memory;
  target.core.restore(saved.cpu);
  step(target.core);
  auto expected = seed();
  add(expected);
  ++expected.pc;
  require(target.core.registers() == expected &&
              target.core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
          "A checkpoint remains usable after its original CPU and buffers are destroyed");
}

template <Backend Kind> void typed_checkpoint() {
  std::array<Word, 4> p{};
  std::array<Word, 4> x{};
  std::array<Word, 4> y{};
  BackendCore<Kind> core({.program = p, .x = x, .y = y});
  auto expected = seed();
  expected.pc = 0;
  core.set_registers(expected);
  core.load(Space::program, 0, std::array{Word{0}, add_x0});
  require(core.run(1).dispatches == 1, "Typed backend executes the actual prefix");
  ++expected.pc;
  const auto saved = core.checkpoint();
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    core.restore(saved);
    require(core.registers() == expected &&
                core.progress() == Progress{.instructions = 1, .estimated_cycles = 1},
            "Typed adapter forwards complete restoration and counters");
    require(core.run(1).dispatches == 1, "Typed backend resumes from its checkpoint");
    auto result = expected;
    add(result);
    ++result.pc;
    require(core.registers() == result &&
                core.progress() == Progress{.instructions = 2, .estimated_cycles = 2},
            "Typed checkpoint suffix has independently specified arithmetic and progress");
  }
}

class Device final : public Io {
public:
  Core *core = nullptr;
  std::optional<Checkpoint> saved;
  unsigned capture_rejections = 0, restore_rejections = 0;
  bool valid = true;

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word &value) noexcept override {
    try {
      if (core == nullptr || !saved) { return false; }
      const auto before = core->registers();
      try {
        static_cast<void>(core->checkpoint());
      } catch (const std::logic_error &) { ++capture_rejections; }
      try {
        core->restore(*saved);
      } catch (const std::logic_error &) { ++restore_rejections; }
      valid = core->registers() == before;
      value = datum;
      return valid;
    } catch (const std::exception &) {
      valid = false;
      return false;
    }
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
}

void callbacks() {
  for (const auto backend : backends) {
    Device device;
    Machine machine(backend, &device);
    device.core = &machine.core;
    auto expected = seed();
    expected.r[0] = port;
    machine.core.set_registers(expected);
    machine.core.load(Space::program, start, std::array{load_post});
    device.saved = machine.core.checkpoint();
    step(machine.core);
    expected.x = datum;
    ++expected.r[0];
    ++expected.pc;
    require(machine.core.registers() == expected && device.valid &&
                device.capture_rejections == 1 && device.restore_rejections == 1,
            "Active callback checkpoint/restore reject without undoing the actual transfer");
  }
}
} // namespace dsp56300::test::checkpoint_test

int main() {
  try {
    using namespace dsp56300::test::checkpoint_test;
    repetition();
    vectors();
    processing();
    cache();
    geometry();
    callbacks();
    lifetime();
    shared_checkpoint();
    alias_offsets();
    typed_checkpoint<dsp56300::Backend::reference>();
    typed_checkpoint<dsp56300::Backend::cached>();
    typed_checkpoint<dsp56300::Backend::jit>();
    std::cout << dsp56300::test::checks << " checkpoint assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
