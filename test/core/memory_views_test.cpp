// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_compiler.hpp"
#include "isa.hpp"
#include "memory_arena.hpp"
#include "native_faults.hpp"
#include "reject_io.hpp"
#include "support.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fcntl.h>
#include <iostream>
#include <llvm-c/BitReader.h>
#include <llvm-c/Core.h>
#include <llvm-c/Transforms/PassBuilder.h>
#include <llvm-c/Types.h>
#include <span>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <system_error>
#include <unistd.h>

namespace {
using namespace dsp56300;
using test::require;
constexpr Word start = 16, target = 4, value = 0x123456, other = 0x654321;
constexpr std::array code{static_cast<Word>(isa::Opcode::move_x_effective) | (4U << 11),
                          static_cast<Word>(isa::Opcode::move_x_effective) | (4U << 11) |
                              (1U << 15),
                          static_cast<Word>(isa::Opcode::stop)};

void checked(const bool success) {
  if (!success) { throw std::system_error(errno, std::generic_category(), "Test memory mapping"); }
}

struct Backing {
  Backing() {
    static unsigned next = 0;
    const auto name = "/dspviews-" + std::to_string(getpid()) + "-" + std::to_string(next++);
    fd = shm_open(name.c_str(), O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
    checked(fd >= 0);
    if (shm_unlink(name.c_str()) != 0) {
      const auto reason = errno;
      static_cast<void>(close(fd));
      throw std::system_error(reason, std::generic_category(), "Unlink test backing");
    }
  }

  ~Backing() { static_cast<void>(close(fd)); }

  Backing(const Backing &) = delete;
  Backing &operator=(const Backing &) = delete;
  int fd = -1;
};

struct Storage {
  explicit Storage(const bool read_only_x = false) {
    const auto page = sysconf(_SC_PAGESIZE);
    require(page > 0, "The test queries the actual host page size");
    page_ = static_cast<std::size_t>(page);
    checked(ftruncate(file_.fd, static_cast<off_t>(page_ * (views.size() + 1))) == 0);
    const auto r = read_.buffers();
    const auto w = write_.buffers();
    const std::array reads{r.program, r.x, r.y};
    const std::array writes{w.program, w.x, w.y};
    for (std::size_t bank = 0; bank < views.size(); ++bank) {
      for (const bool writing : {false, true}) {
        const auto words = writing ? writes[bank] : reads[bank];
        checked(mprotect(words.data(), words.size_bytes(), PROT_NONE) == 0);
        const auto protection = writing ? PROT_READ | PROT_WRITE : PROT_READ;
        const auto backing = writing && read_only_x && bank == 1 ? views.size() : bank;
        checked(mmap(words.data(),
                     page_,
                     protection,
                     MAP_SHARED | MAP_FIXED,
                     file_.fd,
                     static_cast<off_t>(backing * page_)) != MAP_FAILED);
      }
      views[bank] = {.read = {.data = reads[bank], .mirrored_tail = {}},
                     .write = {.data = writes[bank], .mirrored_tail = {}}};
      regions[bank] = {.address = 0,
                       .words = reads[bank].first(page_ / sizeof(Word)),
                       .known = {},
                       .writable = true};
      descriptors[bank] = std::span(&regions[bank], 1);
    }
    if (read_only_x) {
      regions[1].writable = false;
      checked(pwrite(file_.fd,
                     &other,
                     sizeof(other),
                     static_cast<off_t>(page_ + (target * sizeof(Word)))) ==
              static_cast<ssize_t>(sizeof(other)));
    }
  }

  [[nodiscard]] Memory memory() const { return Memory::complete(descriptors, views); }

  [[nodiscard]] Word word(const Space space, const Word at) const {
    return static_cast<const volatile Word *>(
        views[static_cast<std::size_t>(space)].read.data.data())[at];
  }

  [[nodiscard]] std::size_t words() const { return page_ / sizeof(Word); }

  std::array<Memory::Views, 3> views;
  std::array<MemoryWindow, 3> regions;
  std::array<std::span<const MemoryWindow>, 3> descriptors;

  test::MemoryArena read_, write_;
  Backing file_;
  std::size_t page_ = 0;
};

struct Origin {
  Word pc = start;
  std::uint64_t variant = 0;
};

NativeImage image(const Storage &storage,
                  const bool fixed,
                  const std::span<const Word> program = code,
                  const CompileOptions::MemoryMode mode = CompileOptions::MemoryMode::exact,
                  const Origin origin = {}) {
  const std::array spans{CodeSpan{.address = origin.pc, .words = program}};
  const std::array entries{origin.pc};
  const std::array variants{
      CodeVariant{.id = origin.variant, .code = spans, .entries = entries, .failed_fetches = {}}};
  std::array<RegionLayout, 3> regions{};
  for (std::size_t bank = 0; bank < regions.size(); ++bank) {
    regions[bank] = {.space = static_cast<Space>(bank),
                     .first = 0,
                     .size = storage.words(),
                     .writable = storage.regions[bank].writable};
  }
  const std::array layouts{VariantLayout{.id = origin.variant, .memory = regions}};
  return Core::compile({.variants = variants},
                       {.memory = fixed ? std::span(layouts) : std::span<const VariantLayout>{},
                        .interrupts = {},
                        .transitions = {},
                        .callbacks = {},
                        .bindings = {},
                        .memory_mode = mode});
}

void initialize(Core &core, const NativeImage &native) {
  core.load(Space::program, start, code);
  auto registers = core.registers();
  registers.pc = start;
  registers.omr = 0;
  registers.r[0] = target;
  registers.x = value;
  core.set_registers(registers);
  core.install(native, 0);
}

bool same(const State &a, const State &b) {
  return a.registers == b.registers && a.progress == b.progress && a.execution == b.execution &&
         a.failure == b.failure;
}

void execution(const bool fixed, const bool direct = false) {
  const Storage first;
  const Storage second;
  const auto native =
      image(first,
            fixed,
            code,
            direct ? CompileOptions::MemoryMode::direct : CompileOptions::MemoryMode::exact);
  Core core(first.memory());
  initialize(core, native);
  require(first.word(Space::program, start) == code.front(),
          "Host programming writes through its write alias into a read-only program view");
  require(core.run(1).retired == 1 && first.word(Space::x, target) == value,
          "A guest store reaches the write view while the read view remains host read-only");
  const auto before = core.state();
  bool refused = false;
  try {
    core.set_memory(Memory::external());
  } catch (const std::invalid_argument &) { refused = true; }
  require(refused && same(before, core.state()), "A rejected rebind preserves state and storage");
  Core programmer(second.memory());
  programmer.load(Space::program, start, code);
  programmer.load(Space::x, target, std::array{other});
  core.set_memory(second.memory());
  require(same(before, core.state()), "A valid rebind changes no architectural state or progress");
  core.install(native, 0);
  const auto result = core.run();
  require(result.retired == 2 && result.state == ExecutionState::stopped &&
              result.fault == Fault{} && core.progress().instructions == code.size() &&
              (core.registers().x & word_mask) == other && first.word(Space::x, target) == value,
          "Reinstallation resumes the original instruction boundary against the new backing");
  Core revoked(first.memory());
  initialize(revoked, native);
  require(revoked.run(1).retired == 1, "The revoked binding begins with one real store");
  revoked.set_memory(second.memory());
  const auto unbound = revoked.run(1);
  require(unbound.retired == 0 && unbound.fault.kind == FaultKind::unprepared_execution &&
              unbound.fault.pc == start + 1 && unbound.progress.instructions == 1,
          "Changing storage revokes execution until a matching installation");
  core.reset();
  auto registers = core.registers();
  registers.pc = start + 1;
  registers.omr = 0;
  registers.r[0] = static_cast<Word>(second.words());
  core.set_registers(registers);
  core.install(native, 0);
  const auto failed = core.run(1);
  require(failed.retired == 0 && failed.fault.kind == FaultKind::memory_read &&
              failed.fault.address == second.words() && failed.fault.pc == start + 1,
          "The checked path reports a protected undeclared word without dereferencing the hole");
}

// Inspect only the public artifact, then run the real shared mappings through
// the native provider. No private Core layout or host-address alias assumption.
struct AccessPcs {
  Word read, write;
};

void volatile_accesses(const std::span<const Word> program, const AccessPcs pcs) {
  using test::image_compiler::Owner;
  const std::array spans{CodeSpan{.address = start, .words = program}};
  const std::array entries{start};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const auto lowered = Core::lower({.variants = variants},
                                   {.memory = {},
                                    .interrupts = {},
                                    .transitions = {},
                                    .callbacks = {},
                                    .bindings = {},
                                    .memory_mode = CompileOptions::MemoryMode::direct});
  const Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
  const auto bytes = lowered.bitcode();
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> buffer(
      LLVMCreateMemoryBufferWithMemoryRangeCopy(
          reinterpret_cast<const char *>(bytes.data()), bytes.size(), "direct-alias"));
  LLVMModuleRef parsed = nullptr;
  const auto invalid = LLVMParseBitcodeInContext2(context.get(), buffer.get(), &parsed);
  const Owner<LLVMModuleRef, LLVMDisposeModule> module(parsed);
  require(invalid == 0 && module, "The public direct-memory artifact parses");
  const Owner<LLVMPassBuilderOptionsRef, LLVMDisposePassBuilderOptions> options(
      LLVMCreatePassBuilderOptions());
  test::image_compiler::checked(LLVMRunPasses(module.get(), "default<O3>", nullptr, options.get()));
  unsigned reads = 0;
  unsigned writes = 0;
  for (auto *function = LLVMGetFirstFunction(module.get()); function != nullptr;
       function = LLVMGetNextFunction(function)) {
    for (auto *block = LLVMGetFirstBasicBlock(function); block != nullptr;
         block = LLVMGetNextBasicBlock(block)) {
      for (auto *instruction = LLVMGetFirstInstruction(block); instruction != nullptr;
           instruction = LLVMGetNextInstruction(instruction)) {
        const auto operation = LLVMGetInstructionOpcode(instruction);
        if (operation != LLVMLoad && operation != LLVMStore) { continue; }
        if (LLVMGetVolatile(instruction) == 0) { continue; }
        auto *const type = operation == LLVMLoad ? LLVMTypeOf(instruction)
                                                 : LLVMTypeOf(LLVMGetOperand(instruction, 0));
        require(LLVMGetTypeKind(type) == LLVMIntegerTypeKind && LLVMGetIntTypeWidth(type) == 32,
                "Every direct guest access retains its scalar Word type after O3");
        const auto line = LLVMGetDebugLocLine(instruction);
        reads += static_cast<unsigned>(operation == LLVMLoad && line == pcs.read + 1);
        writes += static_cast<unsigned>(operation == LLVMStore && line == pcs.write + 1);
      }
    }
  }
  require(reads != 0 && writes != 0,
          "O3 retains ordered guest reads and writes at both source instructions");
  require(LLVMGetNamedFunction(module.get(), "dsp56300_memory_direct") == nullptr,
          "The memory-mode marker is eliminated before native execution");
}

void aliases() {
  constexpr Word turns = 256;
  constexpr Word load = static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15);
  constexpr Word store = static_cast<Word>(isa::Opcode::move_x_effective);
  constexpr auto stop = static_cast<Word>(isa::Opcode::stop);
  constexpr std::array reload{store | (4U << 11),
                              static_cast<Word>(isa::Opcode::move_immediate_x0),
                              load | (4U << 11),
                              stop};
  constexpr std::array shifted{static_cast<Word>(isa::Opcode::do_immediate) |
                                   ((turns & 0xffU) << 8) | ((turns >> 8) & 15U),
                               start + 3,
                               load | (3U << 11),
                               store | (3U << 11) | (1U << 8),
                               stop};
  for (const bool loop : {false, true}) {
    const auto program = loop ? std::span<const Word>(shifted) : std::span<const Word>(reload);
    volatile_accesses(program, {.read = start + 2, .write = loop ? start + 3 : start});
    const Storage storage;
    const auto native = image(storage, true, program, CompileOptions::MemoryMode::direct);
    Core core(storage.memory());
    core.load(Space::program, start, program);
    std::array<Word, turns + 1> words{};
    words[0] = value;
    core.load(Space::x, 0, words);
    auto registers = core.registers();
    registers.pc = start;
    registers.omr = 0;
    registers.m.fill(word_mask);
    registers.r[1] = 1;
    registers.x = value;
    core.set_registers(registers);
    core.install(native, 0);
    const auto result = core.run();
    require(result.fault == Fault{} && result.state == ExecutionState::stopped &&
                result.retired == (loop ? (turns * 2) + 2 : reload.size()) &&
                (core.registers().x & word_mask) == value,
            "The native call observes writes through a distinct real read alias");
    if (!loop) { continue; }
    require(core.registers().r[0] == turns && core.registers().r[1] == turns + 1,
            "The shifted loop executes every source and destination increment");
    for (Word index = 0; index <= turns; ++index) {
      require(storage.word(Space::x, index) == value,
              "Forward overlapping copy propagates the seed through every aliased word");
    }
  }
}

void geometry() {
  const Storage storage;
  for (unsigned mutation = 0; mutation != 6; ++mutation) {
    auto views = storage.views;
    auto regions = storage.regions;
    switch (mutation) {
      case 0: views[0].read.data = views[0].read.data.first(address_words - 1); break;
      case 1: views[1].write.data = views[0].write.data; break;
      case 2: views[0].read.mirrored_tail = views[0].read.data.first(1); break;
      case 3: regions[0].words = regions[0].words.subspan(1); break;
      case 4:
        regions[1].known =
            std::span<std::uint8_t>{reinterpret_cast<std::uint8_t *>(regions[1].words.data()), 1};
        break;
      case 5: regions[2].address = word_mask; break;
      default: throw std::logic_error("Unknown authored geometry case");
    }
    std::array<std::span<const MemoryWindow>, 3> descriptors{};
    for (std::size_t bank = 0; bank < regions.size(); ++bank) {
      descriptors[bank] = std::span(&regions[bank], 1);
    }
    bool refused = false;
    try {
      static_cast<void>(Memory::complete(descriptors, views));
    } catch (const std::invalid_argument &) { refused = true; }
    require(refused, "Invalid view extents, aliases, tails and region geometry are rejected cold");
  }
}

struct Discard final : test::RejectIo {
  bool
  write(const Space space, const Word address, const Word pc, const Word input) noexcept override {
    ++stores;
    return space == Space::x && address == target && pc == start && input == value;
  }

  unsigned stores = 0;
};

void read_only(const bool fixed, const bool direct = false) {
  const Storage storage(true);
  const auto native =
      image(storage,
            fixed,
            code,
            direct ? CompileOptions::MemoryMode::direct : CompileOptions::MemoryMode::exact);
  Discard io;
  Core core(storage.memory(), &io);
  initialize(core, native);
  const auto result = core.run();
  require(
      result.retired == code.size() && result.fault == Fault{} &&
          result.state == ExecutionState::stopped && io.stores == (direct ? 0U : 1U) &&
          (core.registers().x & word_mask) == other && storage.word(Space::x, target) == other,
      "Exact callback writes preserve read-only content despite a separate writable scratch view");
}

void native_failure(const bool writing) {
  constexpr std::uint64_t variant = (std::uint64_t{1} << 48) + 7;
  const Storage storage;
  const auto operation =
      static_cast<Word>(isa::Opcode::move_x_effective) | (4U << 11) | (writing ? 0U : 1U << 15);
  const std::array program{
      operation, static_cast<Word>(isa::Opcode::adc_x_a), static_cast<Word>(isa::Opcode::stop)};
  const auto native = image(
      storage, true, program, CompileOptions::MemoryMode::direct, {.pc = 0, .variant = variant});
  Core core(storage.memory());
  core.load(Space::program, 0, program);
  auto registers = core.registers();
  registers.omr = 0;
  registers.r[0] = static_cast<Word>(storage.words());
  registers.x = value;
  core.set_registers(registers);
  core.install(native, variant);
  const test::NativeFaults::InterruptedMask interrupted;
  const auto result = core.run_until(10);
  require(interrupted.restored(),
          "A native escape retains the interrupted mask and removes handler-only blocking");
  require(result.fault.kind == (writing ? FaultKind::memory_write : FaultKind::memory_read) &&
              result.fault.space == Space::x && result.fault.address == storage.words() &&
              result.fault.pc == 0 && result.fault.variant == variant && result.retired == 0 &&
              result.progress == Progress{},
          "A real native read/write fault reports PC0 and the full source variant before later "
          "arithmetic");
  const auto repeated = core.run(1);
  require(repeated.retired == 0 && repeated.fault == result.fault,
          "A terminal escape releases the owner guard but never resumes guest execution");
  bool refused = false;
  try {
    core.reset();
  } catch (const std::logic_error &) { refused = true; }
  require(refused, "A terminal native instance cannot be reset into execution");
}

void repeat_eligibility() {
  const Storage storage;
  constexpr std::array program{static_cast<Word>(isa::Opcode::rep_immediate) | (3U << 8),
                               static_cast<Word>(isa::Opcode::nop),
                               static_cast<Word>(isa::Opcode::nop),
                               static_cast<Word>(isa::Opcode::stop)};
  const auto native = image(storage, true, program);
  Core core(storage.memory());
  core.load(Space::program, start, program);
  auto registers = core.registers();
  registers.pc = start;
  registers.omr = 0;
  core.set_registers(registers);
  core.install(native, 0);
  const auto result = core.run(4);
  require(result.retired == 4 && result.fault == Fault{} && core.registers().pc == start + 2 &&
              core.interrupt_ready(),
          "Post-REP eligibility reads the real next instruction from lent P without an I/O owner");
  registers = core.registers();
  registers.pc = static_cast<Word>(storage.words());
  core.set_registers(registers);
  bool refused = false;
  try {
    static_cast<void>(core.interrupt_ready());
  } catch (const std::logic_error &) { refused = true; }
  require(refused, "Missing post-REP metadata reports an error without reading a protected P hole");
}
}

int main() try {
  const test::NativeFaults faults(Core::host_fault);
  require(!Core::host_fault(0, false, 0),
          "Unowned faults are not claimed outside native execution");
  aliases();
  geometry();
  execution(false);
  execution(true);
  read_only(false);
  read_only(true);
  execution(false, true);
  execution(true, true);
  read_only(false, true);
  read_only(true, true);
  native_failure(false);
  native_failure(true);
  repeat_eligibility();
  std::cout << "Borrowed DSP read/write views passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
