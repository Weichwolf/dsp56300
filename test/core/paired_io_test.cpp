// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_compiler.hpp"
#include "isa.hpp"
#include "memory_arena.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <llvm-c/BitWriter.h>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <span>
#include <string_view>
#include <vector>

namespace {
using namespace dsp56300;
using test::require;
using test::image_compiler::Owner;
template <isa::Opcode opcode> constexpr auto named = static_cast<Word>(opcode);
constexpr Word start = 0x100, data_port = 0xffffda, strobe_port = 0xffff90;
constexpr Word strobe = 11, past_tail = start + 18, iterations = 31;
constexpr Word destination = 4, out = 0x120, byte = 0xff, spanned_words = 1024;
constexpr Word unpack_shift = 32;
constexpr Word turn_cycles = 20, turn_instructions = 15, events_per_turn = 6;
constexpr Word full = 1 + (turn_instructions * iterations);
constexpr auto read_port = named<isa::Opcode::movep_register_high> | (12U << 8U) | 0x1aU;
constexpr auto bit_operand = (16U << 8U) | (1U << 6U) | strobe;
constexpr auto clear = named<isa::Opcode::bclr_port_low> | bit_operand;
constexpr auto set = named<isa::Opcode::bset_port_low> | bit_operand;
constexpr auto save_high =
    (named<isa::Opcode::move_register_x0_x1> & ~((31U << 13U) | (31U << 8U))) | (12U << 13U) |
    (5U << 8U);
constexpr auto write =
    named<isa::Opcode::move_y_effective> | (1U << 20U) | (3U << 11U) | (destination << 8U);
constexpr std::array program{named<isa::Opcode::do_immediate> | (iterations << 8U),
                             past_tail - 1,
                             read_port,
                             clear,
                             named<isa::Opcode::lsl_immediate> | (16U << 1U),
                             named<isa::Opcode::nop>,
                             set,
                             save_high,
                             read_port,
                             named<isa::Opcode::nop>,
                             clear,
                             named<isa::Opcode::and_immediate_long>,
                             byte,
                             named<isa::Opcode::lsl_immediate> | (8U << 1U),
                             named<isa::Opcode::or_x1_a>,
                             set,
                             write,
                             named<isa::Opcode::nop>,
                             named<isa::Opcode::stop>};
static_assert(program.size() == past_tail - start + 1);

struct Storage {
  test::MemoryArena arena;
  Buffers data = arena.buffers();
  std::array<MemoryWindow, 3> windows{
      MemoryWindow{.words = data.program.first(spanned_words), .known = {}, .writable = true},
      MemoryWindow{.words = data.x.first(spanned_words), .known = {}, .writable = true},
      MemoryWindow{.words = data.y.first(spanned_words), .known = {}, .writable = true}};

  [[nodiscard]] Memory loan() const {
    const std::array whole{data.program, data.x, data.y};
    std::array<Memory::Views, 3> views;
    std::array<std::span<const MemoryWindow>, 3> descriptors;
    for (std::size_t i = 0; i < views.size(); ++i) {
      views[i] = {.read = {.data = whole[i], .mirrored_tail = {}},
                  .write = {.data = whole[i], .mirrored_tail = {}}};
      descriptors[i] = std::span(&windows[i], 1);
    }
    return Memory::complete(descriptors, views);
  }
};

struct Stimulus {
  Word seed, quota;
  bool inactive;
};

struct Bus final : Io {
  enum class Action : std::uint8_t { read, clear, set };
  using Entry = std::array<Word, 2>;

  explicit Bus(Storage &memory, const Stimulus input)
      : ram(memory.data.y), seed(input.seed), quota(input.quota), pin(input.inactive) {}

  void event(Action action, Word value) noexcept {
    if (used == log.size()) { std::terminate(); }
    log[used++] = {static_cast<Word>(action), value};
  }

  bool read(Space space, Word address, Word /*pc*/, Word &value) noexcept override {
    if (space != Space::x || address != data_port || index == fail_index) { return false; }
    constexpr Word multiplier = 0x2c8f3;
    value = (seed + multiplier * index++) & word_mask;
    event(Action::read, value);
    return true;
  }

  bool bit(Space space,
           Word address,
           Word /*pc*/,
           Word bit,
           BitAction action,
           bool &previous) noexcept override {
    if (space != Space::y || address != strobe_port || bit != strobe ||
        (action != BitAction::clear && action != BitAction::set)) {
      return false;
    }
    previous = pin;
    pin = action == BitAction::set;
    event(pin ? Action::set : Action::clear, static_cast<Word>(previous));
    return true;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  static std::uint64_t
  batch(Io *owner, const PairedPortTransfer *transfer, std::uint64_t request) noexcept {
    auto &bus = *static_cast<Bus *>(owner);
    constexpr std::array<Word, 2> reads{0, 8};
    constexpr std::array<Word, 4> edges{1, 5, 10, 16};
    if (transfer->data_space != static_cast<Word>(Space::x) ||
        transfer->data_address != data_port ||
        transfer->strobe_space != static_cast<Word>(Space::y) ||
        transfer->strobe_address != strobe_port || transfer->strobe_bit != strobe ||
        transfer->cycles != turn_cycles || transfer->read_cycles != reads ||
        transfer->edge_cycles != edges) {
      std::terminate();
    }
    const auto count = std::min(static_cast<Word>(request >> unpack_shift), bus.quota);
    if (count == 0) { return 0; }
    auto address = static_cast<Word>(request) & word_mask;
    Word result = 0;
    for (Word i = 0; i < count; ++i, ++address) {
      Word high = 0;
      Word low = 0;
      bool previous = false;
      if (!bus.read(Space::x, data_port, 0, high) ||
          !bus.bit(Space::y, strobe_port, 0, strobe, BitAction::clear, previous) ||
          !bus.bit(Space::y, strobe_port, 0, strobe, BitAction::set, previous) ||
          !bus.read(Space::x, data_port, 0, low) ||
          !bus.bit(Space::y, strobe_port, 0, strobe, BitAction::clear, previous) ||
          !bus.bit(Space::y, strobe_port, 0, strobe, BitAction::set, previous)) {
        std::terminate();
      }
      result = ((high & byte) << 16U) | ((low & byte) << 8U);
      bus.ram[address] = result;
    }
    bus.transferred += count;
    bus.quota -= count;
    if (bus.yield_owner) { bus.core->request_yield(); }
    if (bus.revise_program) { ++bus.revision; }
    return (std::uint64_t{count} << unpack_shift) | result;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  decltype(&Bus::batch) target = &Bus::batch;
  Core *core = nullptr;
  bool yield_owner = false, revise_program = false;
  std::uint64_t revision = 0;
  std::array<Entry, std::size_t{iterations} * events_per_turn> log{};
  std::span<Word> ram;
  Word seed, quota, index = 0, transferred = 0, used = 0;
  Word fail_index = word_mask;
  bool pin;
};

std::vector<std::byte> binding() {
  const Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
  const Owner<LLVMModuleRef, LLVMDisposeModule> module(
      LLVMModuleCreateWithNameInContext("authored-pair-bus", context.get()));
  const Owner<LLVMBuilderRef, LLVMDisposeBuilder> ir(LLVMCreateBuilderInContext(context.get()));
  auto *ptr = LLVMPointerTypeInContext(context.get(), 0);
  auto *wide = LLVMInt64TypeInContext(context.get());
  std::array parameters{ptr, ptr, wide};
  auto *type = LLVMFunctionType(wide, parameters.data(), parameters.size(), 0);
  auto *function = LLVMAddFunction(module.get(), "authored_pair", type);
  constexpr std::string_view attribute = "nounwind";
  LLVMAddAttributeAtIndex(
      function,
      static_cast<unsigned>(LLVMAttributeFunctionIndex),
      LLVMCreateEnumAttribute(
          context.get(), LLVMGetEnumAttributeKindForName(attribute.data(), attribute.size()), 0));
  LLVMPositionBuilderAtEnd(ir.get(),
                           LLVMAppendBasicBlockInContext(context.get(), function, "entry"));
  Storage memory;
  const Bus bus(memory, {.seed = 0, .quota = 0, .inactive = false});
  const auto offset = reinterpret_cast<std::uintptr_t>(&bus.target) -
                      reinterpret_cast<std::uintptr_t>(static_cast<const Io *>(&bus));
  auto *index = LLVMConstInt(wide, offset, 0);
  auto *owner = LLVMGetParam(function, 0);
  auto *pointer =
      LLVMBuildGEP2(ir.get(), LLVMInt8TypeInContext(context.get()), owner, &index, 1, "");
  auto *called = LLVMBuildLoad2(ir.get(), ptr, pointer, "");
  std::array arguments{owner, LLVMGetParam(function, 1), LLVMGetParam(function, 2)};
  LLVMBuildRet(ir.get(),
               LLVMBuildCall2(ir.get(), type, called, arguments.data(), arguments.size(), ""));
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> bitcode(
      LLVMWriteBitcodeToMemoryBuffer(module.get()));
  const auto *base = reinterpret_cast<const std::byte *>(LLVMGetBufferStart(bitcode.get()));
  return {base, base + LLVMGetBufferSize(bitcode.get())};
}

NativeImage image(bool optimized) {
  constexpr std::array entries{start};
  constexpr std::array spans{CodeSpan{.address = start, .words = program}};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  constexpr std::array callbacks{
      CallbackBinding{.role = CallbackRole::paired_read, .symbol = "authored_pair"}};
  const auto bitcode = optimized ? binding() : std::vector<std::byte>{};
  return Core::compile(
      {.variants = variants},
      {.memory = {},
       .interrupts = {},
       .transitions = {},
       .callbacks = optimized ? std::span{callbacks}.first(1) : std::span<const CallbackBinding>{},
       .bindings = bitcode,
       .memory_mode = CompileOptions::MemoryMode::direct});
}

struct Machine {
  Machine(const NativeImage &image, const Registers &initial, const Stimulus input)
      : bus(memory, input), core(memory.loan(), &bus) {
    bus.core = &core;
    core.load(Space::program, start, program);
    core.set_registers(initial);
    core.install(image, 0);
  }

  Storage memory;
  Bus bus;
  Core core;
};

void equal(const Machine &control,
           const Machine &candidate,
           const RunResult before,
           const RunResult after) {
  const auto known = control.core.state();
  const auto result = candidate.core.state();
  require(known.registers == result.registers && known.progress == result.progress &&
              known.failure == result.failure && known.execution == result.execution &&
              before.retired == after.retired && before.yielded == after.yielded,
          "Exact core state");
  require(control.bus.used == candidate.bus.used && control.bus.pin == candidate.bus.pin &&
              std::ranges::equal(control.bus.log, candidate.bus.log),
          "Exact observed port effects");
  require(std::ranges::equal(control.memory.windows[2].words, candidate.memory.windows[2].words),
          "Exact RAM and untouched neighboring locations");
}

struct Images {
  const NativeImage &precise;
  const NativeImage &paired;
};

void check_boundary(const Images images, const bool revision) {
  constexpr Word quota = 2;
  constexpr Word seed = 42;
  constexpr Word completed = 1 + (turn_instructions * quota);
  auto init = test::initial_state();
  init.r[destination] = out;
  Machine reference(images.precise, init, {.seed = seed, .quota = 0, .inactive = true});
  Machine candidate(images.paired, init, {.seed = seed, .quota = quota, .inactive = true});
  candidate.bus.yield_owner = !revision;
  candidate.bus.revise_program = revision;
  auto expected = reference.core.run(completed);
  expected.yielded = true; // Same guest prefix with the committed host output notification.
  const auto actual = candidate.core.run(full);
  require(actual.yielded && actual.retired == completed,
          "The paired callback must return at the notified turn boundary");
  equal(reference, candidate, expected, actual);
  candidate.bus.yield_owner = false;
  candidate.bus.revise_program = false;
  if (revision) { candidate.core.install(images.paired, 0); }
  equal(reference,
        candidate,
        reference.core.run(full - completed),
        candidate.core.run(full - completed));
  std::cout << (revision ? "Paired program revision and exact resumption passed\n"
                         : "Paired owner yield boundary and exact resumption passed\n");
}

struct PrefixCase {
  Word status, deadline, quota, seed;
};

bool check_prefix(const Images images, const PrefixCase input) {
  auto initial = test::initial_state();
  initial.r[destination] = out;
  constexpr std::uint64_t accumulator_a = 0xf4'19aa98'31e4a1;
  constexpr std::uint64_t accumulator_b = 0xef'1fc788'573912;
  constexpr std::uint64_t data_x = 0x199456'7bff17;
  initial.a = accumulator_a;
  initial.x = data_x;
  initial.b = accumulator_b;
  initial.sr = input.status | test::flag_c | test::flag_v | test::flag_z | test::flag_n;
  const bool inactive = (input.seed & 1U) != 0;
  Machine reference(
      images.precise, initial, {.seed = input.seed, .quota = 0, .inactive = inactive});
  Machine candidate(
      images.paired, initial, {.seed = input.seed, .quota = input.quota, .inactive = inactive});
  const auto progress = [&](Machine &machine) {
    return input.deadline != 0 ? machine.core.run_until(input.deadline) : machine.core.run(full);
  };
  const auto before = progress(reference);
  const auto after = progress(candidate);
  equal(reference, candidate, before, after);
  require(before.retired <= full, "Prefix cannot pass the terminal body");
  const auto remaining = full - before.retired;
  if (remaining != 0) {
    equal(reference, candidate, reference.core.run(remaining), candidate.core.run(remaining));
  }
  require(reference.core.registers().pc == past_tail &&
              reference.bus.used == iterations * events_per_turn,
          "Original word and strobe count after the terminal DO restore");
  return candidate.bus.transferred != 0;
}

void check_prefixes(const Images images) {
  constexpr std::array times{0U, 1U, 2U, 17U, 22U, 30U, 103U, 302U, 601U};
  constexpr std::array counts{0U, 1U, 7U, iterations};
  constexpr std::array modes{0U, test::mode_up, test::mode_down, test::mode_rm};
  constexpr std::array<Word, 3> seeds{0, 0x91f55a, 0xffffff};
  unsigned fast = 0;
  for (const auto mode : modes) {
    for (const auto time : times) {
      for (const auto quota : counts) {
        for (const auto seed : seeds) {
          fast += unsigned{check_prefix(
              images, {.status = mode, .deadline = time, .quota = quota, .seed = seed})};
        }
      }
    }
  }
  constexpr unsigned intended_coverage = 324;
  require(fast == intended_coverage, "The expected cases execute a native transfer");
}

void check_faults_and_wrap(const Images images) {
  auto initial = test::initial_state();
  initial.r[destination] = out;
  constexpr Word fault_seed = 0xf8de13;
  constexpr Word wrap_seed = 0xff578f;
  for (const auto quota : {1U, 7U, 19U}) {
    for (const auto within_pair : {0U, 1U}) {
      Machine reference(
          images.precise, initial, {.seed = fault_seed, .quota = 0, .inactive = true});
      Machine candidate(
          images.paired, initial, {.seed = fault_seed, .quota = quota, .inactive = true});
      reference.bus.fail_index = candidate.bus.fail_index = quota * 2 + within_pair;
      const auto before = reference.core.run(full);
      const auto after = candidate.core.run(full);
      equal(reference, candidate, before, after);
      require(after.fault.kind == FaultKind::io_read && after.fault.address == data_port &&
                  candidate.bus.transferred == quota,
              "A fault after the last admitted pair preserves its exact instruction and effects");
    }
  }
  initial.r[destination] = out + 9;
  for (const auto modifier : {15U, 31U, 0x801fU}) {
    initial.m[destination] = modifier;
    Machine reference(images.precise, initial, {.seed = wrap_seed, .quota = 0, .inactive = true});
    Machine candidate(
        images.paired, initial, {.seed = wrap_seed, .quota = iterations, .inactive = true});
    const auto before = reference.core.run(full);
    const auto after = candidate.core.run(full);
    equal(reference, candidate, before, after);
    require(after.fault == Fault{} && candidate.bus.transferred == 0,
            "A modulo destination takes the original wrap path");
  }
}
}

int main() try {
  const auto precise = image(false);
  const auto paired = image(true);
  const Images images{.precise = precise, .paired = paired};
  check_prefixes(images);
  check_faults_and_wrap(images);
  check_boundary(images, false);
  check_boundary(images, true);
  std::cout << test::checks << " exact paired-I/O checks passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
