// SPDX-License-Identifier: GPL-3.0-only
#include "a64_alu_model.hpp"
#include "a64_loop_state.hpp"
#include "a64_native.hpp"
#include "address_model.hpp"
#include "bit_access_model.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/index.hpp"
#include "loop_control_model.hpp"
#include "register_model.hpp"
#include "saturation_model.hpp"
#include "support.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace dsp56300::test {
namespace {
using State = loop_model::State;
using Input = loop_model::Input;
using Kind = loop_model::Kind;
using Operand = loop_model::Operand;
constexpr std::array scales{
    0U, mode_up, mode_down, mode_rm, mode_rm | mode_up, mode_rm | mode_down};
constexpr std::array pointers{0U, 15U, 31U, 63U};
constexpr Word location = 384;

State seed(const Word status) {
  State state;
  state.reg = register_model::seed(status);
  state.reg.sr = status;
  return state;
}

a64::Instruction instruction(const Input input, const State &initial) {
  return {.opcode = loop_model::opcode(input),
          .extension = input.extension,
          .pc = initial.reg.pc,
          .mode = initial.reg.sr,
          .words = loop_model::words(input),
          .cycles = loop_model::cycles(input),
          .loop_address = (initial.reg.sr & loop_model::lf) != 0 ? initial.reg.la : word_mask + 1};
}

void setup(const Input input, State initial) {
  const a64::Program program(instruction(input, initial));
  const loop_model::Site site{.pc = initial.reg.pc, .words = loop_model::words(input)};
  initial.reg.r.at(input.address.index) = input.address.cursor;
  initial.reg.n.at(input.address.index) = input.address.offset;
  initial.reg.m.at(input.address.index) = input.address.modifier;
  std::array<Word, memory_words> x{};
  x.fill(input.memory_value);
  for (const Word budget : {0U, 1U, 2U}) {
    auto expected = initial;
    if (budget != 0) { loop_model::setup(expected, input, site); }
    loop_model::Runner runner(initial);
    runner.bind({{{.data = x.data(), .words = x.size()}, {.data = x.data(), .words = x.size()}}});
    const auto retired = runner.run(program, budget);
    runner.expect(expected);
    require(retired == (budget != 0 && expected.fault.kind == FaultKind::none ? 1U : 0U),
            "DO/REP setup retires exactly once after complete count/stack effects");
  }
}

void setup_counts(const Word status) {
  for (const auto kind : {Kind::absolute,
                          Kind::relative,
                          Kind::repeat,
                          Kind::forever_absolute,
                          Kind::forever_relative}) {
    for (const Word count : {0U, 1U, 3U, 4095U}) {
      auto initial = seed(status);
      if (loop_model::forever({.kind = kind})) { initial.reg.lc = count; }
      for (const Word pointer : pointers) {
        initial.reg.sp = pointer;
        for (const Word stack : {0U, 13U, 14U, 15U, 16U}) {
          initial.reg.sc = stack;
          setup({.kind = kind, .source = count}, initial);
        }
      }
    }
  }
}

void setup_registers(const Word status) {
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    for (unsigned number = 0; number < 64; ++number) {
      for (const Word stack : {0U, 13U, 14U, 15U}) {
        auto initial = seed(status);
        initial.reg.sc = stack;
        initial.reg.sp = pointers.at(number % pointers.size());
        setup({.kind = kind, .operand = Operand::reg, .source = number}, initial);
      }
    }
  }
}

void immediate_bits() {
  constexpr Word count_mask = 4095;
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    for (unsigned bit = 0; bit < 12; ++bit) {
      const auto single = Word{1} << bit;
      setup({.kind = kind, .source = single}, seed(status_byte));
      setup({.kind = kind, .source = count_mask ^ single}, seed(status_byte));
    }
  }
}

void setup_memory() {
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      for (unsigned index = 0; index < agu_registers; ++index) {
        for (const Word modifier : {0U, 3U, 0x8003U, 0x8000U, 0xc001U, address_model::linear}) {
          setup({.kind = kind,
                 .operand = Operand::effective,
                 .address = {.modifier = modifier,
                             .cursor = location,
                             .offset = 1,
                             .mode = mode,
                             .index = index},
                 .y = (index & 1U) != 0},
                seed(status_byte));
        }
      }
    }
    for (const Word count : {0U, 1U, word_mask}) {
      setup({.kind = kind,
             .operand = Operand::short_memory,
             .source = register_model::pointer_mask,
             .memory_value = count},
            seed(0));
    }
    setup({.kind = kind,
           .operand = Operand::effective,
           .address =
               {.modifier = address_model::linear, .cursor = memory_words, .offset = 1, .mode = 3}},
          seed(0));
  }
}

void endings(const Word status) {
  for (const Word pointer : pointers) {
    for (const Word stack : {0U, 1U, 2U, 14U}) {
      auto initial = seed(status);
      initial.reg.sp = pointer;
      initial.reg.sc = stack;
      for (int condition = -1; condition < 16; ++condition) {
        const auto opcode = condition < 0 ? loop_model::enddo
                                          : loop_model::break_base | static_cast<Word>(condition);
        const a64::Program program(
            {.opcode = opcode, .mode = status, .cycles = condition < 0 ? 1U : 5U});
        auto expected = initial;
        loop_model::end(expected, {.pc = start}, condition);
        loop_model::Runner runner(initial);
        const auto retired = runner.run(program, 1);
        runner.expect(expected);
        require(retired == (expected.fault.kind == FaultKind::none ? 1U : 0U),
                "ENDDO/BRK preserve first-fault partial pop state");
      }
    }
  }
}

void endpoint(const Word status) {
  for (const Word count : {0U, 1U, 2U, word_mask}) {
    for (const Word stack : {0U, 1U, 2U}) {
      auto initial = seed(status | loop_model::lf);
      initial.reg.lc = count;
      initial.reg.sc = stack;
      initial.reg.la = start;
      initial.reg.sp = register_model::pointer_mask;
      const a64::Program program({.mode = initial.reg.sr, .loop_address = start});
      auto expected = initial;
      ++expected.reg.pc;
      expected.inhibited = false;
      expected.completed = false;
      loop_model::finish(expected, {.pc = start});
      if (expected.fault.kind == FaultKind::none) { loop_model::retire(expected, 1); }
      loop_model::Runner runner(initial);
      const auto retired = runner.run(program, 1);
      runner.expect(expected);
      require(retired == (expected.fault.kind == FaultKind::none ? 1U : 0U),
              "Loop endpoints preserve zero/forever and final-pop fault semantics");
    }
  }
}

void chain() {
  constexpr Word last = start + 3;
  const Input input{.source = 3, .extension = last};
  const auto initial = seed(status_byte);
  const a64::Program entry(instruction(input, initial));
  const std::array body{
      a64::Instruction{.pc = start + 2, .mode = status_byte, .loop_address = last},
      a64::Instruction{.pc = last, .mode = status_byte, .loop_address = last}};
  const a64::Program loop(body);
  const a64::Program after({.pc = last + 1, .mode = status_byte});
  detail::NativeTable outside(3);
  detail::NativeTable inside(3);
  detail::NativeTable catalog(4);
  outside.insert(last + 1, after.entry());
  inside.insert(start + 2, loop.entry());
  constexpr Word inactive = word_mask + 1;
  catalog.insert(inactive << 7, static_cast<const void *>(outside.index().data()));
  catalog.insert(last << 7, static_cast<const void *>(inside.index().data()));
  for (const Word budget : {0U, 1U, 2U, 3U, 6U, 7U, 8U, 32U}) {
    auto expected = initial;
    Word retired = 0;
    if (budget != 0) {
      loop_model::setup(expected, input);
      ++retired;
    }
    while (retired < budget && expected.reg.pc <= last + 1) {
      expected.boundary = detail::NativeBoundary::none;
      const auto pc = expected.reg.pc;
      ++expected.reg.pc;
      expected.inhibited = false;
      expected.completed = false;
      loop_model::finish(expected, {.pc = pc});
      loop_model::retire(expected, 1);
      ++retired;
    }
    loop_model::Runner runner(initial);
    require(runner.chain(entry, budget, outside.index(), catalog.index()) == retired,
            "Complete native loop bodies and retained context tables chain across DO entry, "
            "backedges and exit");
    runner.expect(expected);
  }
}

class CountBus {
public:
  CountBus(const loop_model::Runner &runner,
           const State &visible,
           const Input input,
           const bool accept)
      : runner_(runner), visible_(visible), input_(input), accept_(accept) {}

  [[nodiscard]] detail::NativeBus binding() noexcept { return {.owner = this, .access = access}; }

  void expect(const unsigned calls) const {
    require(valid_ && calls_ == calls,
            "Loop count bus reads publish exact post-push/post-AGU state before retirement");
  }

private:
  static void access(void *const owner, detail::NativeAccess &request) noexcept {
    auto &self = *static_cast<CountBus *>(owner);
    ++self.calls_;
    auto physical = self.visible_.reg;
    constexpr unsigned padding = 8;
    physical.a <<= padding;
    physical.b <<= padding;
    const auto &cpu = self.runner_.cpu();
    constexpr Word port = 0xffff80;
    self.valid_ = self.valid_ && request.action == detail::BusAction::read && request.value == 0 &&
                  request.address == port && request.pc == start &&
                  request.space == (self.input_.y ? Space::y : Space::x) &&
                  request.failure == Fault{} && cpu.reg == physical &&
                  cpu.instructions == self.visible_.instructions &&
                  cpu.cycles == self.visible_.cycles &&
                  cpu.repeat_remaining == self.visible_.repeat.remaining &&
                  cpu.repeat_loaded == self.visible_.repeat.loaded &&
                  cpu.repeat_completed == self.visible_.completed && !cpu.interrupt_inhibited;
    constexpr Word payload = 0xab000003;
    request.value = payload;
    if (!self.accept_) {
      request.failure = {.kind = FaultKind::io_read,
                         .space = request.space,
                         .address = request.address,
                         .pc = request.pc};
    }
  }

  const loop_model::Runner &runner_;
  State visible_;
  Input input_;
  bool accept_, valid_ = true;
  unsigned calls_ = 0;
};

void io_count(const Input input, const State &initial, const bool accept) {
  constexpr Word port = 0xffff80;
  constexpr Word count = 3;
  const loop_model::Site site{.pc = start, .words = loop_model::words(input)};
  const a64::Program program(instruction(input, initial));
  auto visible = initial;
  visible.reg.pc += loop_model::words(input);
  visible.inhibited = false;
  const bool repeat = input.kind == Kind::repeat;
  if (!repeat) {
    loop_model::push(visible, site, (std::uint64_t{visible.reg.la} << word_bits) | visible.reg.lc);
  }
  const bool accessed = visible.fault.kind == FaultKind::none;
  if (accessed) { ++visible.reg.r[0]; }
  auto expected = visible;
  if (accessed) {
    expected.boundary = detail::NativeBoundary::owner;
    if (!accept) {
      expected.fault = {.kind = FaultKind::io_read,
                        .space = input.y ? Space::y : Space::x,
                        .address = port,
                        .pc = start};
    } else if (repeat) {
      expected.repeat = {.remaining = count,
                         .pc = expected.reg.pc,
                         .lc = initial.reg.lc,
                         .opcode = initial.repeat.opcode};
      expected.reg.lc = count;
    } else {
      expected.reg.lc = count;
      loop_model::push(
          expected, site, (std::uint64_t{expected.reg.pc} << word_bits) | expected.reg.sr);
      if (expected.fault.kind == FaultKind::none) {
        expected.reg.la = loop_model::last(input, site);
        expected.reg.sr = (expected.reg.sr & ~loop_model::fv) | loop_model::lf;
        expected.completed = false;
      }
    }
    if (expected.fault.kind == FaultKind::none) {
      loop_model::retire(expected, loop_model::cycles(input));
    }
  }
  loop_model::Runner runner(initial);
  CountBus bus(runner, visible, input, accept);
  runner.bind({}, bus.binding());
  require(runner.run(program, 1) == (expected.fault.kind == FaultKind::none ? 1U : 0U),
          "Loop count I/O preserves partial setup and failure priority");
  runner.expect(expected);
  bus.expect(accessed ? 1U : 0U);
}

void io_counts() {
  constexpr Word port = 0xffff80;
  for (const auto kind : {Kind::absolute, Kind::relative, Kind::repeat}) {
    for (const Word pointer : pointers) {
      for (const Word count : {0U, 14U, 15U}) {
        for (const bool accept : {false, true}) {
          for (const bool y : {false, true}) {
            auto initial = seed(status_byte);
            initial.reg.r[0] = port;
            initial.reg.sp = pointer;
            initial.reg.sc = count;
            io_count({.kind = kind,
                      .operand = Operand::effective,
                      .address = {.modifier = address_model::linear,
                                  .cursor = port,
                                  .offset = 1,
                                  .mode = 3},
                      .y = y},
                     initial,
                     accept);
          }
        }
      }
    }
  }
}

void nested_end() {
  constexpr Word previous_la = 0x222;
  for (const Word mode : {0U, loop_model::cm}) {
    for (Word count = 0; count <= 4; ++count) {
      auto initial = seed(mode | status_byte | loop_model::lf);
      initial.reg.sp = 4;
      initial.reg.sc = count;
      initial.reg.stack[4] = (std::uint64_t{start + 1} << word_bits) | loop_model::lf;
      initial.reg.stack[3] = (std::uint64_t{start} << word_bits) | 1U;
      initial.reg.stack[2] = (std::uint64_t{start + 8} << word_bits) | status_byte;
      initial.reg.stack[1] = (std::uint64_t{previous_la} << word_bits) | 7U;
      const a64::Program program({.opcode = loop_model::enddo, .mode = initial.reg.sr});
      auto expected = initial;
      loop_model::end(expected, {.pc = start});
      loop_model::Runner runner(initial);
      require(runner.run(program, 1) == (expected.fault.kind == FaultKind::none ? 1U : 0U),
              "ENDDO can reach an outer endpoint; both cleanup stages preserve partial faults");
      runner.expect(expected);
    }
  }
}

void extension_endpoints() {
  constexpr Word immediate_add = 0x0140c0;
  for (const Word pc : {start, word_mask}) {
    for (const Word displacement : {0U, 1U, 2U}) {
      for (const Word count : {1U, 2U}) {
        auto initial = seed(status_byte | loop_model::lf);
        initial.reg.pc = pc;
        initial.reg.la = (pc + displacement) % address_words;
        initial.reg.lc = count;
        const a64::Program program({.opcode = immediate_add,
                                    .extension = 1,
                                    .pc = pc,
                                    .mode = initial.reg.sr,
                                    .words = 2,
                                    .cycles = 2,
                                    .loop_address = initial.reg.la});
        auto expected = initial;
        auto old = initial.reg;
        old.x = 1;
        a64_alu_model::apply(expected.reg, old, 0x40);
        expected.reg.pc = (pc + 2) % address_words;
        expected.inhibited = false;
        expected.completed = false;
        loop_model::finish(expected, {.pc = pc, .words = 2});
        if (expected.fault.kind == FaultKind::none) { loop_model::retire(expected, 2); }
        loop_model::Runner runner(initial);
        require(runner.run(program, 1) == (expected.fault.kind == FaultKind::none ? 1U : 0U),
                "Only the final word of a complete instruction reaches LA, including PC wrap");
        runner.expect(expected);
      }
    }
  }
}

void enable_at_endpoint() {
  const bit_model::Input input{
      .action = BitAction::set, .selector = register_model::reg_sr, .bit = 15};
  for (const Word mode : {0U, loop_model::cm}) {
    for (const Word count : {1U, 2U}) {
      for (const Word stack : {0U, 2U}) {
        auto initial = seed(mode | status_byte);
        initial.reg.la = start;
        initial.reg.lc = count;
        initial.reg.sc = stack;
        const a64::Program program({.opcode = bit_model::opcode(input), .mode = mode, .cycles = 2});
        auto expected = initial;
        require(bit_model::apply_register(expected.reg, input).kind == FaultKind::none,
                "A raw SR bit write executes before endpoint compatibility checks");
        expected.inhibited = false;
        expected.completed = false;
        expected.boundary = detail::NativeBoundary::owner;
        loop_model::finish(expected, {.pc = start});
        if (expected.fault.kind == FaultKind::none) { loop_model::retire(expected, 2); }
        loop_model::Runner runner(initial);
        require(runner.run(program, 1) == (expected.fault.kind == FaultKind::none ? 1U : 0U),
                "The current instruction can enable a loop; cleanup preserves an earlier owner "
                "boundary");
        runner.expect(expected);
      }
    }
  }
}
}
}

int main() {
  try {
    using namespace dsp56300::test;
    for (const auto scale : scales) {
      for (const auto saturation : {0U, saturation_model::mode}) {
        const auto mode = scale | saturation;
        setup_counts(mode | status_byte);
        setup_registers(mode | status_byte);
        endings(mode | status_byte);
        endpoint(mode | status_byte);
        endpoint(mode | status_byte | loop_model::fv);
      }
    }
    for (const dsp56300::Word mode :
         {loop_model::cm, dsp56300::Word{1} << 14, dsp56300::Word{1} << 17, mode_up | mode_down}) {
      setup_counts(mode | status_byte);
      setup_registers(mode | status_byte);
      endpoint(mode | status_byte);
    }
    setup_memory();
    immediate_bits();
    chain();
    io_counts();
    nested_end();
    extension_endpoints();
    enable_at_endpoint();
    std::cout << checks << " independent native A64 hardware-loop checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
