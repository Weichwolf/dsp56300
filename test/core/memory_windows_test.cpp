// SPDX-License-Identifier: GPL-3.0-only
#include "bfu_logic_model.hpp"
#include "dsp56300/core.hpp"
#include "external_run_fixture.hpp"
#include "memory_window_workloads.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace dsp56300::test {
namespace {
constexpr Word low_base = 0x80, high_base = 0x200, rom_base = 0x4000, port = 0xffff90;
constexpr Word datum = 0x9654a1, other = 0x2579b3, complement = 0x200017;
constexpr Word load_post = 0x44d843, store = 0x446000, pair = 0xf09843;
constexpr std::size_t words = 32, rom_words = 16;

struct Storage {
  std::array<Word, words> low{}, high{};
  std::array<Word, rom_words> rom{};
  std::array<std::uint8_t, words> low_known{}, high_known{};
  std::array<MemoryWindow, 3> windows;

  Storage()
      : windows{{{.address = low_base, .words = low, .known = low_known, .writable = true},
                 {.address = high_base, .words = high, .known = high_known, .writable = true},
                 {.address = rom_base, .words = rom, .known = {}}}} {}
};

class Memory final : public Io {
public:
  std::array<Word, memory_words> program{};
  std::array<bool, memory_words> program_known{};
  std::array<Storage, 2> storage;
  std::uint64_t revision = 0;
  unsigned reads = 0, writes = 0;
  bool remap_on_port = false;

  void code(const std::span<const Word> code) {
    program_known.fill(false);
    for (std::size_t index = 0; index < code.size(); ++index) {
      program[start + index] = code[index];
      program_known[start + index] = true;
    }
    ++revision;
  }

  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    if (address >= program.size() || !program_known[address]) { return std::nullopt; }
    return program[address];
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision; }

  [[nodiscard]] std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return storage[space == Space::x ? 0 : 1].windows;
  }

  MemoryWindow *locate(const Space space, const Word address) noexcept {
    for (auto &window : storage[space == Space::x ? 0 : 1].windows) {
      if (address >= window.address && address - window.address < window.words.size()) {
        return &window;
      }
    }
    return nullptr;
  }

  bool read(const Space space, const Word address, Word /*pc*/, Word &value) noexcept override {
    ++reads;
    if (space == Space::program) {
      const auto word = inspect_program(address);
      if (!word) { return false; }
      value = *word;
      return true;
    }
    const auto *window = locate(space, address);
    if (window == nullptr) { return false; }
    const auto index = address - window->address;
    if (!window->known.empty() && window->known[index] == 0) { return false; }
    value = window->words[index];
    return true;
  }

  bool
  write(const Space space, const Word address, Word /*pc*/, const Word value) noexcept override {
    ++writes;
    if (space == Space::program) { return false; }
    if (address == port && remap_on_port) {
      storage[0].windows[0].words = storage[0].high;
      storage[0].windows[0].known = storage[0].high_known;
      ++revision;
      return true;
    }
    auto *window = locate(space, address);
    if (window == nullptr || !window->writable) { return false; }
    const auto index = address - window->address;
    window->words[index] = value;
    if (!window->known.empty()) { window->known[index] = 1; }
    if (window->aliases_program) { ++revision; }
    return true;
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

Word transfer(const unsigned reg, const bool load, const bool y) {
  constexpr Word memory_move = 0x406000;
  constexpr Word high_register_mask = 0x18;
  constexpr Word load_bit = 0x8000;
  constexpr Word y_bit = 0x80000;
  constexpr Word xor_alu = 0x43;
  constexpr unsigned high_register_shift = 17;
  return memory_move | ((reg & 7U) << 16) | ((reg & high_register_mask) << high_register_shift) |
         (load ? load_bit : 0U) | (y ? y_bit : 0U) | xor_alu;
}

void xor_old(Registers &expected, const Registers &old) {
  constexpr auto middle = std::uint64_t{word_mask} << word_bits;
  auto truth = expected;
  bfu_model::apply(truth, old, {.function = bfu_model::Function::bit_xor});
  // The parallel MOVE owns A's extension/low words; the BFU writes A1 only.
  expected.a = (expected.a & ~middle) | (truth.a & middle);
  expected.sr = truth.sr;
  ++expected.pc;
}

struct TransferCase {
  unsigned reg = 4;
  bool load = false, y = false, optimize = false;
  Word mode = 0;
  std::size_t range = 0, index = 0;
};

void one_transfer(Core &core, Memory &memory, const TransferCase input) {
  auto &bank = memory.storage[input.y ? 1 : 0];
  auto &window = bank.windows[input.range];
  auto initial = register_model::seed(input.mode | flag_s | flag_l);
  initial.r[0] = window.address + static_cast<Word>(input.index);
  core.set_registers(initial);
  window.words[input.index] = datum;
  if (!window.known.empty()) { window.known[input.index] = input.load ? 1 : 0; }
  auto expected = initial;
  auto expected_memory = bank;
  if (input.load) {
    register_model::write(expected, input.reg, datum);
  } else {
    const auto wanted = register_model::read(expected, input.reg);
    if (input.range == 0) {
      expected_memory.low[input.index] = wanted;
      expected_memory.low_known[input.index] = 1;
    } else {
      expected_memory.high[input.index] = wanted;
      expected_memory.high_known[input.index] = 1;
    }
  }
  xor_old(expected, initial);
  const auto native = core.jit_statistics().native_move_instructions;
  const auto reads = memory.reads;
  const auto writes = memory.writes;
  const auto result = core.run(1);
  const bool valid = result.dispatches == 1 && result.fault.kind == FaultKind::none &&
                     core.registers() == expected && bank.low == expected_memory.low &&
                     bank.high == expected_memory.high && bank.rom == expected_memory.rom &&
                     bank.low_known == expected_memory.low_known &&
                     bank.high_known == expected_memory.high_known;
  if (!valid) {
    const auto actual = core.registers();
    std::cerr << "window opt=" << input.optimize << " load=" << input.load << " y=" << input.y
              << " reg=" << input.reg << " mode=" << input.mode << " range=" << input.range
              << " index=" << input.index << " retired=" << result.dispatches
              << " fault=" << static_cast<unsigned>(result.fault.kind) << std::hex
              << " opcode=" << memory.program[start] << " pc=" << actual.pc << '/' << expected.pc
              << " a=" << actual.a << '/' << expected.a << " x=" << actual.x << '/' << expected.x
              << " y=" << actual.y << '/' << expected.y << " sr=" << actual.sr << '/' << expected.sr
              << " reads=" << memory.reads << " writes=" << memory.writes << '\n';
  }
  require(
      valid,
      "Sparse windows preserve independent old-input XOR/bus flags and complete word/byte arrays");
  require(core.jit_statistics().native_move_instructions == native + (input.optimize ? 1 : 0) &&
              (!input.optimize || (memory.reads == reads && memory.writes == writes)),
          "Actual native loads/stores avoid all owner callbacks and mark genuine writes");
}

void transfer_cases(Core &core, Memory &memory, TransferCase input) {
  constexpr Word saturation_mode = Word{1} << 20;
  auto &bank = memory.storage[input.y ? 1 : 0];
  for (const Word mode : {0U, mode_down, mode_up, mode_rm, saturation_mode}) {
    input.mode = mode;
    for (const std::size_t range : {0U, 1U, 2U}) {
      if (!input.load && range == 2) { continue; }
      input.range = range;
      for (const auto index : {std::size_t{0}, bank.windows[range].words.size() - 1}) {
        input.index = index;
        one_transfer(core, memory, input);
      }
    }
  }
}

void transfers(const bool optimize) {
  Memory memory;
  Core core(memory, {.optimize = optimize});
  for (const bool y : {false, true}) {
    for (const bool load : {false, true}) {
      for (const unsigned reg : {4U, 5U, 6U, 7U, 14U, 15U}) {
        const std::array program{transfer(reg, load, y)};
        memory.code(program);
        transfer_cases(core, memory, {.reg = reg, .load = load, .y = y, .optimize = optimize});
      }
    }
  }
}

void bounded_unknown_and_dma() {
  Memory memory;
  constexpr std::array program{Word{0x200017}, load_post};
  memory.code(program);
  memory.storage[0].low_known[1] = 1;
  Core core(memory);
  auto initial = register_model::seed(0);
  initial.r[0] = low_base;
  core.set_registers(initial);
  auto expected = initial;
  bfu_model::apply(expected, initial, {.function = bfu_model::Function::complement});
  ++expected.pc;
  const auto stopped = core.run(RunBudget{.instructions = 10, .cycles = 10});
  require(
      stopped.execution.dispatches == 1 && stopped.cycles == 1 &&
          stopped.stop == RunStop::external_effect && core.registers() == expected &&
          stopped.next && stopped.next->opcode == load_post && memory.reads == 0,
      "An unknown native window read stops before its whole EA/ALU/PC effect after a real prefix");
  const auto repeated = core.run(RunBudget{.instructions = 1, .cycles = 1});
  require(repeated.execution.dispatches == 0 && repeated.stop == RunStop::external_effect &&
              core.registers() == expected && memory.reads == 0,
          "Repeated owner-boundary queries neither initialize unknown RAM nor change CPU state");
  const auto compiled = core.jit_statistics().compiled_blocks;
  memory.storage[0].low[0] = datum;
  memory.storage[0].low_known[0] = 1; // A real owner/DMA write needs no code revision.
  const auto old = expected;
  ++expected.r[0];
  register_model::write(expected, 4, datum);
  xor_old(expected, old);
  const auto actual = core.run(RunBudget{.instructions = 1, .cycles = 1});
  require(actual.execution.dispatches == 1 && actual.cycles == 1 && core.registers() == expected &&
              memory.reads == 0 && core.jit_statistics().compiled_blocks == compiled,
          "A warmed known-bit guard sees an actual DMA initialization without stale metadata");
}

void paired_guards() {
  for (const bool store_y : {false, true}) {
    Memory memory;
    constexpr Word y_load = 0x400000;
    const std::array program{store_y ? pair & ~y_load : pair};
    memory.code(program);
    Core core(memory);
    auto initial = register_model::seed(mode_up | flag_l);
    initial.r[0] = low_base;
    initial.r[4] = high_base;
    core.set_registers(initial);
    memory.storage[0].low[0] = datum;
    memory.storage[0].low_known[0] = 1;
    auto stopped = core.run(RunBudget{.instructions = 1, .cycles = 1});
    if (!store_y) {
      require(stopped.execution.dispatches == 0 && stopped.stop == RunStop::external_effect &&
                  core.registers() == initial && memory.reads == 0 && memory.writes == 0,
              "A second-port unknown read preserves the complete first-port/parallel state");
      memory.storage[1].high[0] = other;
      memory.storage[1].high_known[0] = 1;
      stopped = core.run(RunBudget{.instructions = 1, .cycles = 1});
    }
    auto expected = initial;
    ++expected.r[0];
    ++expected.r[4];
    Word y_value = other;
    if (store_y) { y_value = register_model::read(expected, 6); }
    register_model::write(expected, 4, datum);
    if (!store_y) { register_model::write(expected, 6, other); }
    xor_old(expected, initial);
    require(stopped.execution.dispatches == 1 && core.registers() == expected &&
                (store_y ? memory.storage[1].high[0] == y_value : true) &&
                memory.storage[1].high_known[0] == 1 && memory.reads == 0 && memory.writes == 0,
            "Paired sparse windows preserve old X/Y inputs, both AGUs and initialized write state");
  }
}

void aliased_pair() {
  Memory memory;
  constexpr Word x_load = 0x8000;
  const std::array program{pair & ~x_load};
  memory.code(program);
  auto &target = memory.storage[1].windows[1];
  target.words = memory.storage[0].low;
  target.known = memory.storage[0].low_known;
  Core core(memory);
  auto initial = register_model::seed(0);
  initial.r[0] = low_base;
  initial.r[4] = high_base;
  core.set_registers(initial);
  const auto stopped = core.run(RunBudget{.instructions = 1, .cycles = 1});
  require(stopped.execution.dispatches == 0 && stopped.stop == RunStop::external_effect &&
              core.registers() == initial && memory.storage[0].low[0] == 0 &&
              memory.storage[0].low_known[0] == 0 && memory.writes == 0 && memory.reads == 0,
          "A second unknown port cannot initialize the first port's aliased store or commit either "
          "AGU");
  memory.storage[0].low[0] = other;
  memory.storage[0].low_known[0] = 1;
  auto expected = initial;
  ++expected.r[0];
  ++expected.r[4];
  const auto written = register_model::read(expected, 4);
  register_model::write(expected, 6, other);
  xor_old(expected, initial);
  const auto actual = core.run(RunBudget{.instructions = 1, .cycles = 1});
  require(actual.execution.dispatches == 1 && core.registers() == expected &&
              memory.storage[0].low[0] == written && memory.reads == 0 && memory.writes == 0,
          "Both ports capture the genuine old shared word before the native store commits");
}

void callback_rebinding() {
  Memory memory;
  constexpr Word set_r0 = 0x60f400;
  constexpr Word load = 0x44e000;
  const std::array program{store, set_r0, low_base, load};
  memory.code(program);
  memory.remap_on_port = true;
  memory.storage[0].low[0] = datum;
  memory.storage[0].low_known[0] = 1;
  memory.storage[0].high[0] = other;
  memory.storage[0].high_known[0] = 1;
  Core core(memory);
  auto expected = register_model::seed(0);
  expected.r[0] = port;
  core.set_registers(expected);
  expected.r[0] = low_base;
  register_model::write(expected, 4, other);
  expected.pc += static_cast<Word>(program.size());
  const auto actual = core.run(3);
  require(actual.dispatches == 3 && actual.fault.kind == FaultKind::none &&
              core.registers() == expected && memory.reads == 0 && memory.writes == 1 &&
              core.jit_statistics().invalidations != 0,
          "A real peripheral callback remaps storage and invalidates already embedded successor "
          "pointers");
}

void all_known_writes() {
  Memory memory;
  constexpr std::array program{store};
  memory.code(program);
  memory.storage[0].windows[1].known = {};
  Core core(memory);
  auto initial = register_model::seed(0);
  initial.r[0] = low_base;
  core.set_registers(initial);
  require(core.run(1).dispatches == 1 && memory.storage[0].low_known[0] == 1,
          "A tracked direct store initializes exactly its genuine known byte");
  memory.storage[0].low_known[0] = 0;
  initial.r[0] = high_base;
  core.set_registers(initial);
  require(core.run(1).dispatches == 1 && memory.storage[0].high[0] == (initial.x & word_mask) &&
              memory.storage[0].low_known[0] == 0 && memory.writes == 0,
          "An all-known window never reuses a previous window's initialization pointer");
}

void bounded_buffers() {
  std::array<Word, memory_words> program{};
  std::array<Word, memory_words> x{};
  std::array<Word, memory_words> y{};
  constexpr Word address = 7;
  program[start] = load_post;
  x[address] = datum;
  Core core({.program = program, .x = x, .y = y});
  auto initial = register_model::seed(0);
  initial.r[0] = address;
  core.set_registers(initial);
  auto expected = initial;
  ++expected.r[0];
  register_model::write(expected, 4, datum);
  xor_old(expected, initial);
  const auto actual = core.run(RunBudget{.instructions = 1, .cycles = 1});
  require(actual.execution.dispatches == 1 && actual.cycles == 1 && core.registers() == expected &&
              core.jit_statistics().native_move_instructions == 1,
          "Bounded native ordinary-buffer RAM honors its actual load, old ALU input and AGU");
  initial.r[0] = memory_words;
  core.set_registers(initial);
  const auto stopped = core.run(RunBudget{.instructions = 1, .cycles = 1});
  require(stopped.execution.dispatches == 0 && stopped.stop == RunStop::external_effect &&
              core.registers() == initial,
          "An out-of-span buffer access remains a complete-instruction owner boundary");
  const auto refused = core.run(1);
  require(refused.dispatches == 0 && refused.fault.kind == FaultKind::memory_read &&
              refused.fault.space == Space::x && refused.fault.address == memory_words &&
              refused.fault.pc == start,
          "The real ordinary-buffer fallback retains its precise fail-stop read");
}

void refusals() {
  constexpr std::array<Word, 4> addresses{
      low_base - 1, low_base + static_cast<Word>(words), rom_base, port};
  for (const Word address : addresses) {
    Memory memory;
    constexpr std::array program{store};
    memory.code(program);
    Core core(memory);
    auto initial = register_model::seed(0);
    initial.r[0] = address;
    core.set_registers(initial);
    const auto stopped = core.run(RunBudget{.instructions = 1, .cycles = 10});
    require(stopped.execution.dispatches == 0 && stopped.stop == RunStop::external_effect &&
                core.registers() == initial && memory.writes == 0,
            "Hole, ROM and peripheral addresses stop before any bounded native store effect");
    const auto actual = core.run(1);
    require(actual.dispatches == 0 && memory.writes == 1 && actual.fault.address == address &&
                actual.fault.pc == start && actual.fault.space == Space::x &&
                actual.fault.kind ==
                    (address == port ? FaultKind::io_write : FaultKind::memory_write),
            "Ordinary execution retains the genuine owner refusal and exact access context");
  }
}

void rebinding() {
  Memory memory;
  constexpr std::array program{Word{0x44e000}};
  memory.code(program);
  memory.storage[0].low[0] = datum;
  memory.storage[0].low_known[0] = 1;
  memory.storage[0].high[0] = other;
  memory.storage[0].high_known[0] = 1;
  Core core(memory);
  auto initial = register_model::seed(0);
  initial.r[0] = low_base;
  core.set_registers(initial);
  require(core.run(1).dispatches == 1 && (core.registers().x & word_mask) == datum,
          "Warm a genuine first storage mapping");
  auto &window = memory.storage[0].windows[0];
  window.words = memory.storage[0].high;
  window.known = memory.storage[0].high_known;
  ++memory.revision;
  core.set_registers(initial);
  require(core.run(1).dispatches == 1 && (core.registers().x & word_mask) == other,
          "A descriptor/pointer revision invalidates old embedded host addresses");
  window.words = memory.storage[0].low;
  window.known = memory.storage[0].low_known;
  ++memory.revision;
  core.set_registers(initial);
  require(core.run(1).dispatches == 1 && (core.registers().x & word_mask) == datum &&
              memory.reads == 0,
          "A subsequent explicit descriptor revision rebinds the original storage");
}

void executable_alias() {
  Memory memory;
  constexpr std::array program{store, Word{0}};
  memory.code(program);
  auto &window = memory.storage[0].windows[0];
  window.words = std::span(memory.program).subspan(start + 1, 1);
  window.known = std::span(memory.storage[0].low_known).first(1);
  window.known[0] = 1;
  window.aliases_program = true;
  Core core(memory);
  auto initial = register_model::seed(0);
  initial.r[0] = low_base;
  initial.x = complement;
  core.set_registers(initial);
  const auto stopped = core.run(RunBudget{.instructions = 2, .cycles = 2});
  require(stopped.execution.dispatches == 0 && stopped.stop == RunStop::external_effect &&
              core.registers() == initial && memory.writes == 0,
          "An executable alias is never silently written through a direct bounded RAM pointer");
  auto expected = initial;
  ++expected.pc;
  const auto old = expected;
  bfu_model::apply(expected, old, {.function = bfu_model::Function::complement});
  ++expected.pc;
  const auto actual = core.run(2);
  require(actual.dispatches == 2 && core.registers() == expected && memory.writes == 1 &&
              memory.program[start + 1] == complement &&
              core.jit_statistics().native_move_instructions == 0,
          "The true executable-alias write invalidates its already compiled successor once");
}

void domain_kernels() {
  constexpr Word count = 97;
  for (const auto input : external_run::workloads) {
    window_workload::Memory memory(input, true);
    Core core(memory);
    core.set_registers(window_workload::initial(input, count));
    const auto result = core.run(RunBudget{.instructions = count, .cycles = count});
    require(result.execution.dispatches == count && result.cycles == count &&
                result.execution.fault.kind == FaultKind::none &&
                core.registers() == window_workload::expected(input, count) && memory.reads == 0 &&
                core.jit_statistics().native_move_instructions == count,
            "Bounded multi-iteration RAM kernels retain independent audio/image/communications "
            "arithmetic");
  }
}

void malformed() {
  for (unsigned variant = 0; variant < 5; ++variant) {
    Memory memory;
    constexpr std::array<Word, 1> program{0};
    memory.code(program);
    auto &windows = memory.storage[0].windows;
    if (variant == 0) { windows[0].words = {}; }
    if (variant == 1) { windows[0].known = std::span(memory.storage[0].low_known).first(1); }
    if (variant == 2) { windows[0].address = port; }
    if (variant == 3) { windows[1].address = low_base; }
    if (variant == 4) { windows[2].address = 0; }
    Core core(memory);
    core.set_registers(register_model::seed(0));
    const auto result = core.run(1);
    require(result.dispatches == 0 && result.fault.kind == FaultKind::host_event &&
                result.fault.space == Space::x && memory.reads == 0 && memory.writes == 0,
            "Malformed grants fail visibly before compilation or any CPU/data access");
  }
}
}
}

int main() {
  try {
    dsp56300::test::transfers(false);
    dsp56300::test::transfers(true);
    dsp56300::test::bounded_unknown_and_dma();
    dsp56300::test::paired_guards();
    dsp56300::test::aliased_pair();
    dsp56300::test::callback_rebinding();
    dsp56300::test::all_known_writes();
    dsp56300::test::bounded_buffers();
    dsp56300::test::refusals();
    dsp56300::test::rebinding();
    dsp56300::test::executable_alias();
    dsp56300::test::domain_kernels();
    dsp56300::test::malformed();
    std::cout << dsp56300::test::checks << " external window/owner-boundary checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
