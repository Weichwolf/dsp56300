// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image_compiler.hpp"
#include "isa.hpp"
#include "memory_arena.hpp"
#include "native_faults.hpp"
#include "reject_io.hpp"
#include "retirement_observer.hpp"
#include "support.hpp"

#include <algorithm>
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
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <system_error>
#include <unistd.h>
#include <vector>

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
  explicit Storage(const bool read_only_x = false, const bool shared_views = false) {
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
      if (shared_views) { views[bank].read = views[bank].write; }
      regions[bank] = {.address = 0,
                       .words = views[bank].read.data.first(page_ / sizeof(Word)),
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
                  const Origin origin = {},
                  const std::span<const Word> entries = {}) {
  const std::array spans{CodeSpan{.address = origin.pc, .words = program}};
  const std::array default_entry{origin.pc};
  const std::array variants{
      CodeVariant{.id = origin.variant,
                  .code = spans,
                  .entries = entries.empty() ? std::span<const Word>(default_entry) : entries,
                  .failed_fetches = {}}};
  constexpr std::array spaces{Space::program, Space::x, Space::y};
  std::array<RegionLayout, spaces.size()> regions{};
  for (std::size_t bank = 0; bank < regions.size(); ++bank) {
    regions[bank] = {.space = spaces[bank],
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

void exact_shared_views() {
  const Storage storage(false, true);
  const auto native = image(storage, true);
  Core core(storage.memory());
  initialize(core, native);
  const auto result = core.run();
  require(result.retired == code.size() && result.state == ExecutionState::stopped &&
              result.fault == Fault{} && storage.word(Space::x, target) == value &&
              (core.registers().x & word_mask) == value,
          "A fixed exact native region binds a complete loan with identical read and write views");

  core.reset();
  auto registers = core.registers();
  registers.pc = start;
  registers.omr = 0;
  registers.r[0] = static_cast<Word>(storage.words());
  core.set_registers(registers);
  core.install(native, 0);
  const auto hole = core.run();
  require(hole.retired == 0 && hole.fault.kind == FaultKind::memory_write &&
              hole.fault.address == registers.r[0] && hole.fault.pc == start,
          "A complete exact region reports the first access outside the checked loan");
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

void native_failure(const bool writing, const bool loop = false) {
  constexpr std::uint64_t variant = (std::uint64_t{1} << 48) + 7;
  const Storage storage;
  const auto operation =
      static_cast<Word>(isa::Opcode::move_x_effective) | (4U << 11) | (writing ? 0U : 1U << 15);
  const std::array ordinary{
      operation, static_cast<Word>(isa::Opcode::adc_x_a), static_cast<Word>(isa::Opcode::stop)};
  const std::array region{static_cast<Word>(isa::Opcode::do_immediate) | (4U << 8),
                          Word{4},
                          operation ^ (7U << 11),
                          static_cast<Word>(isa::Opcode::adc_x_a),
                          static_cast<Word>(isa::Opcode::nop),
                          static_cast<Word>(isa::Opcode::stop)};
  const auto program = loop ? std::span<const Word>(region) : std::span<const Word>(ordinary);
  const auto native = image(
      storage, true, program, CompileOptions::MemoryMode::direct, {.pc = 0, .variant = variant});
  Core core(storage.memory());
  core.load(Space::program, 0, program);
  auto registers = core.registers();
  registers.omr = 0;
  registers.r[0] = static_cast<Word>(storage.words()) - (loop ? 1U : 0U);
  registers.m.fill(word_mask);
  registers.x = value;
  core.set_registers(registers);
  core.install(native, variant);
  if (loop) {
    const auto setup = core.run(1);
    require(setup.retired == 1 && setup.fault == Fault{} && core.registers().pc == 2,
            "The DO setup completes at a real owner boundary before entering its memory loop");
  }
  const auto before = core.progress();
  const test::NativeFaults::InterruptedMask interrupted;
  const auto result = core.run_until(50);
  require(interrupted.restored(),
          "A native escape retains the interrupted mask and removes handler-only blocking");
  require(result.fault.kind == (writing ? FaultKind::memory_write : FaultKind::memory_read) &&
              result.fault.space == Space::x && result.fault.address == storage.words() &&
              result.fault.pc == (loop ? 2U : 0U) && result.fault.variant == variant &&
              result.retired == 0 && result.progress == before,
          "A native fault preserves its source variant, actual memory PC and last owner progress");
  const auto repeated = core.run(1);
  require(repeated.retired == 0 && repeated.fault == result.fault,
          "A terminal escape releases the owner guard but never resumes guest execution");
  bool refused = false;
  try {
    core.reset();
  } catch (const std::logic_error &) { refused = true; }
  require(refused, "A terminal native instance cannot be reset into execution");
}

class AuthoredProgram final : public Io {
public:
  [[nodiscard]] std::optional<Word> inspect_program(const Word address) const noexcept override {
    ++inspections;
    if (address >= start && address - start < code.size()) { return code[address - start]; }
    return {};
  }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
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

  mutable unsigned inspections = 0;
};

void mapped_program_source_precedes_io() {
  const Storage storage;
  AuthoredProgram owner;
  const auto native = image(storage, true, code, CompileOptions::MemoryMode::direct);
  Core core(storage.memory(), &owner);
  core.load(Space::program, start, code);
  core.install(native, 0);
  require(owner.inspections == 0,
          "Mapped code satisfies installation without consulting the owner");

  const auto changed = std::array{other};
  core.load(Space::program, start, changed);
  require(owner.inspect_program(start) == code.front() &&
              storage.word(Space::program, start) == other,
          "Owner's ordinary inspection is deliberately stale while protected P contains new code");

  bool refused = false;
  try {
    core.install(native, 0);
  } catch (const std::invalid_argument &error) {
    refused = std::string(error.what()).find("P:$000010") != std::string::npos;
  }
  require(refused && owner.inspections == 1,
          "Installation rejects the actual mapped mismatch without an owner override");
}

void mapped_program_known_boundary() {
  Storage storage;
  constexpr auto split = start + 1;
  const auto words = storage.regions[0].words;
  std::vector<std::uint8_t> known(words.size() - split, 1);
  const std::array program{
      MemoryWindow{.address = 0, .words = words.first(split), .known = {}, .writable = true},
      MemoryWindow{
          .address = split, .words = words.subspan(split), .known = known, .writable = true}};
  storage.descriptors[0] = program;

  AuthoredProgram owner;
  const auto native = image(storage, false);
  Core core(storage.memory(), &owner);
  core.load(Space::program, start, code);
  core.install(native, 0);

  known[1] = 0;
  bool second = false;
  try {
    core.install(native, 0);
  } catch (const std::invalid_argument &error) {
    second = std::string_view(error.what()).ends_with("P:$000012");
  }
  require(second && owner.inspections == 0,
          "The first unknown word in a second mapping fails before consulting an owner");
}

void program_store() {
  const Storage storage;
  constexpr std::array program{static_cast<Word>(isa::Opcode::movem_absolute_short) |
                                   (target << 8) | 4U,
                               static_cast<Word>(isa::Opcode::stop)};
  auto regions = storage.regions;
  regions[0].address = start;
  regions[0].words = storage.views[0].read.data.subspan(start, program.size());
  auto descriptors = storage.descriptors;
  descriptors[0] = std::span(regions.data(), 1);

  struct Loader final : test::RejectIo {
    explicit Loader(Word *destination) : destination(destination) {}

    bool write(Space space, Word address, Word pc, Word word) noexcept override {
      ++stores;
      transfer = {.space = space, .address = address, .pc = pc, .value = word};
      if (transfer.space != Space::program || transfer.address != target) { return false; }
      *static_cast<volatile Word *>(destination) = word;
      loaded = true;
      return true;
    }

    [[nodiscard]] std::optional<Word> inspect_program(Word address) const noexcept override {
      return loaded && address == target
                 ? std::optional<Word>(*static_cast<const volatile Word *>(destination))
                 : std::nullopt;
    }

    Word *destination;

    struct Transfer {
      Space space = Space::program;
      Word address = 0, pc = 0, value = 0;
    } transfer;

    unsigned stores = 0;
    bool loaded = false;
  } io(storage.views[0].write.data.data() + target);

  const auto native = image(storage, false, program, CompileOptions::MemoryMode::direct);
  Core core(Memory::complete(descriptors, storage.views), &io);
  core.load(Space::program, start, program);
  auto registers = core.registers();
  registers.pc = start;
  registers.omr = 0;
  registers.x = value;
  core.set_registers(registers);
  core.install(native, 0);
  require(!io.inspect_program(target), "Original program contents are not predeclared loaded");
  const auto result = core.run();
  require(result.fault == Fault{} && result.state == ExecutionState::stopped &&
              result.retired == program.size() && io.stores == 1 && io.transfer.pc == start &&
              io.inspect_program(target) == value && storage.word(Space::program, target) == value,
          "A Direct P store preserves the consumer's real loader and cold code-knownness effects");
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

void direct_self_loop_empty_window() {
  Storage storage;
  constexpr auto read = static_cast<Word>(isa::Opcode::move_x_effective) | (4U << 11) | (1U << 15);
  constexpr auto adc = static_cast<Word>(isa::Opcode::adc_x_a);
  constexpr auto jump = static_cast<Word>(isa::Opcode::jmp_absolute_short) | start;
  constexpr std::array program{read, adc, jump};
  const auto native = image(storage, false, program, CompileOptions::MemoryMode::direct);
  Core programmer(storage.memory());
  programmer.load(Space::program, start, program);
  programmer.load(Space::x, target, std::array{value});
  const auto configure = [&](Core &core) {
    auto registers = core.registers();
    registers.pc = start;
    registers.omr = 0;
    registers.r[0] = target;
    registers.m.fill(word_mask);
    core.set_registers(registers);
    core.install(native, 0);
  };
  configure(programmer);
  const auto executed = programmer.run(12);
  require(executed.retired == 12 && executed.fault == Fault{} &&
              (programmer.registers().x & word_mask) == value,
          "A protected direct self-loop reads mapped X and completes all four turns");
  storage.descriptors[1] = {};
  const auto x = storage.views[1].read.data.first(storage.words());
  checked(mprotect(x.data(), x.size_bytes(), PROT_NONE) == 0);
  Core blocked(storage.memory());
  configure(blocked);
  const auto result = blocked.run(12);
  require(result.retired == 0 && result.fault.kind == FaultKind::memory_read &&
              result.fault.space == Space::x && result.fault.address == target &&
              result.fault.pc == start,
          "An unmapped direct self-loop read reports the protected address and guest PC");
}

void protected_operands() {
  const Storage storage;
  constexpr auto variant = (std::uint64_t{1} << 48) + 7;
  constexpr Word indirect = 4U << 11;
  constexpr Word load = 1U << 15;
  constexpr Word y = 1U << 6;
  constexpr Word vector_address = 8;
  constexpr Word instruction_limit = 20;
  constexpr auto nop = static_cast<Word>(isa::Opcode::nop);
  constexpr auto stop = static_cast<Word>(isa::Opcode::stop);
  constexpr auto adc = static_cast<Word>(isa::Opcode::adc_x_a);
  constexpr auto bit = static_cast<Word>(isa::Opcode::jset_effective) | indirect;
  constexpr auto do_effective = static_cast<Word>(isa::Opcode::do_effective) | indirect;
  constexpr auto movem = static_cast<Word>(isa::Opcode::movem_effective) | indirect | 4U;
  constexpr auto read_y = static_cast<Word>(isa::Opcode::move_y_effective) | indirect | load;
  constexpr std::array vector{nop, read_y};
  constexpr std::array bit_x{adc, bit, start + 3, stop};
  constexpr std::array bit_y{adc, bit | y, start + 3, stop};
  constexpr std::array loop_count{adc, do_effective, start + 4, nop, adc, stop};
  constexpr std::array load_p{adc, movem | load, stop};
  constexpr std::array store_p{adc, movem, stop};
  constexpr std::array foreground{nop, stop};
  constexpr std::array entry{start};
  constexpr std::array irqs{InterruptRequest{.vector = vector_address, .level = 0}};

  struct Case {
    std::span<const Word> program;
    Space space = Space::x;
    FaultKind operation = FaultKind::memory_read;
    bool irq = false;
  };

  const std::array cases{
      Case{.program = bit_x},
      Case{.program = bit_y, .space = Space::y},
      Case{.program = loop_count},
      Case{.program = load_p, .space = Space::program},
      Case{.program = store_p, .space = Space::program, .operation = FaultKind::memory_write},
      Case{.program = foreground, .space = Space::y, .irq = true}};
  for (const auto &test : cases) {
    const auto extra =
        test.irq ? std::span<const InterruptRequest>(irqs) : std::span<const InterruptRequest>{};
    const std::array spans{CodeSpan{.address = start, .words = test.program},
                           CodeSpan{.address = vector_address, .words = vector}};
    const std::array images{CodeVariant{.id = variant,
                                        .code = std::span(spans).first(test.irq ? 2U : 1U),
                                        .entries = entry,
                                        .failed_fetches = {}}};
    const auto native = Core::compile({.variants = images},
                                      {.memory = {},
                                       .interrupts = extra,
                                       .transitions = {},
                                       .callbacks = {},
                                       .bindings = {},
                                       .memory_mode = CompileOptions::MemoryMode::direct});
    Core core(storage.memory());
    core.load(Space::program, start, test.program);
    if (test.irq) { core.load(Space::program, vector_address, vector); }
    auto registers = core.registers();
    registers.pc = start;
    registers.omr = 0;
    registers.sr = 0;
    registers.r[0] = static_cast<Word>(storage.words());
    registers.m.fill(word_mask);
    registers.x = value;
    core.set_registers(registers);
    core.install(native, variant);
    if (test.irq) { core.request_interrupt(vector_address, 0); }
    const Fault expected{.kind = test.operation,
                         .space = test.space,
                         .address = registers.r[0],
                         .pc = (test.irq ? vector_address : start) + 1,
                         .variant = variant};
    require(core.run(instruction_limit).fault == expected && core.run(1).fault == expected,
            "A protected opcode operand reports its actual source and stops further execution");
  }
}

class DescendingIo final : public test::RejectIo {
public:
  // The core callback fixes the order of the address and issuing PC.
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  bool read(const Space space, const Word address, const Word pc, Word &result) noexcept override {
    constexpr Word peripheral_first = 0xffff80;
    if (space != Space::x || address < peripheral_first || reads == addresses.size()) {
      return false;
    }
    addresses[reads] = address;
    pcs[reads++] = pc;
    result = address;
    return true;
  }

  std::array<Word, 2> addresses{}, pcs{};
};

constexpr Word read_stream = start + 8;
constexpr Word stream_read_count = 5;

void repeated_unit_reads(const Storage &storage,
                         const NativeImage &exact_image,
                         const NativeImage &direct_image) {
  constexpr std::array<Word, 16> values{
      0x1, 0x3, 0x5, 0x7, 0x9, 0xb, 0xd, 0xf, 0x11, 0x13, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f};
  constexpr std::array modifiers{0U, 2U, 3U, 7U, 0x8003U, 0x8007U, word_mask};
  constexpr Word first_data = 8;
  constexpr Word last_data = 15;
  Core exact(storage.memory());
  Core direct(storage.memory());
  exact.load(Space::x, first_data - 2, values);
  exact.install(exact_image, 0);
  direct.install(direct_image, 0);
  for (const auto modifier : modifiers) {
    for (auto origin = first_data; origin <= last_data; ++origin) {
      for (Word prefix = 1; prefix <= stream_read_count; ++prefix) {
        const auto execute_prefix = [&](Core &core) {
          auto registers = core.registers();
          registers.pc = read_stream;
          registers.sr = 0;
          registers.r[0] = origin;
          registers.m[0] = modifier;
          registers.x = 0;
          core.set_registers(registers);
          const auto result = core.run(prefix);
          require(result.retired == prefix && result.fault == Fault{},
                  "Each unit-step memory prefix reaches its actual next instruction");
          return core.registers();
        };
        const auto checked = execute_prefix(exact);
        require(checked == execute_prefix(direct),
                "Protected repeated loads preserve wraps, read values and address state");
      }
    }
  }
}

void crossing_zero() {
  constexpr auto xread = static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15);
  constexpr auto setup = static_cast<Word>(isa::Opcode::do_immediate) | (3U << 8);
  constexpr auto adc = static_cast<Word>(isa::Opcode::adc_x_a);
  constexpr auto stop = static_cast<Word>(isa::Opcode::stop);
  constexpr auto outer = static_cast<Word>(isa::Opcode::do_immediate) | (1U << 8);
  constexpr Word child = start + 5;
  constexpr auto finish = child + 1;
  // An indexed read keeps the outer DO on the dynamically addressed region.
  constexpr auto increment = 3U << 11;
  constexpr auto decrement = 2U << 11;
  const std::array program{outer,
                           finish,
                           xread | (5U << 11),
                           setup,
                           child,
                           xread | decrement,
                           adc,
                           stop,
                           xread | increment,
                           xread | increment,
                           xread | decrement,
                           xread | increment,
                           xread | decrement,
                           stop};
  const std::array entries{start, read_stream};
  constexpr std::array<Word, 4> source{0x001, 0x142, 0x243, 0x344};
  const Storage storage;
  Core programmer(storage.memory());
  programmer.load(Space::program, start, program);
  programmer.load(Space::x, 0, source);
  const auto checked =
      image(storage, false, program, CompileOptions::MemoryMode::exact, {}, entries);
  const auto direct =
      image(storage, false, program, CompileOptions::MemoryMode::direct, {}, entries);
  constexpr Word instruction_count = 8;
  for (Word origin = 0; origin < source.size(); ++origin) {
    DescendingIo checked_io;
    DescendingIo direct_io;
    Core reference(storage.memory(), &checked_io);
    Core product(storage.memory(), &direct_io);
    const auto prepare = [origin](Core &core, const NativeImage &native) {
      auto registers = core.registers();
      registers.pc = start;
      registers.sr = 0;
      registers.omr = 0;
      registers.r[0] = origin;
      registers.m.fill(word_mask);
      core.set_registers(registers);
      core.install(native, 0);
      return core.run();
    };
    const auto expected = prepare(reference, checked);
    const auto result = prepare(product, direct);
    require(expected.fault == Fault{} && expected.retired == instruction_count,
            "Execute the complete exact witness");
    require(result.fault == Fault{} && result.retired == expected.retired &&
                product.state().progress == reference.state().progress &&
                product.registers() == reference.registers(),
            "Direct and exact execution agree at the zero-address wrap");
    const auto count = origin < 2 ? 2U - origin : 0U;
    require(direct_io.addresses == checked_io.addresses && direct_io.pcs == checked_io.pcs &&
                direct_io.reads == checked_io.reads && direct_io.reads == count,
            "Only descending children crossing the peripheral boundary reach the I/O owner");
    if (count != 0) {
      require(direct_io.pcs[0] == child && direct_io.addresses[0] == word_mask,
              "Report the original descending DO child and the real wrapped address");
    }
  }
  repeated_unit_reads(storage, checked, direct);
}

NativeImage observed_direct_image(const std::span<const Word> program) {
  const std::array spans{CodeSpan{.address = start, .words = program}};
  constexpr std::array entries{start};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  constexpr std::array modes{0U};
  constexpr std::array callbacks{
      CallbackBinding{.role = CallbackRole::retired, .symbol = "record_retired"}};
  const auto binding = test::retirement_binding();
  return Core::compile({.variants = variants},
                       {.memory = {},
                        .interrupts = {},
                        .transitions = {},
                        .callbacks = callbacks,
                        .bindings = binding,
                        .memory_mode = CompileOptions::MemoryMode::direct,
                        .region_modes = modes});
}

void grouped_conditional_transfers() {
  constexpr Word equal = 10U << 12;
  constexpr Word zero = 1U << 2;
  constexpr auto nop = static_cast<Word>(isa::Opcode::nop);
  constexpr auto stop = static_cast<Word>(isa::Opcode::stop);
  constexpr auto read_x =
      static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15) | (3U << 11) | (2U << 8);
  constexpr std::array forms{0x020000U | equal, 0x030102U | equal, 0x020902U | equal};
  for (const auto transfer : forms) {
    const Storage storage;
    const std::array program{nop, transfer, read_x, stop};
    Core reference(storage.memory());
    reference.load(Space::program, start, program);
    reference.load(Space::x, target, std::array{value, other});
    reference.install(image(storage, false, program), 0);
    test::RetirementIo trace;
    Core product(storage.memory(), &trace);
    product.install(observed_direct_image(program), 0);
    for (const auto status : {0U, zero}) {
      for (const auto deadline : {1U, 2U, 3U, 4U}) {
        Registers seed{};
        seed.pc = start;
        seed.sr = status;
        seed.a = value;
        seed.b = other;
        seed.r[1] = target + 1;
        seed.r[2] = target;
        seed.m.fill(word_mask);
        trace.counts = {};
        for (auto *core : {&reference, &product}) {
          core->reset();
          core->set_registers(seed);
          require(core->run_until(deadline).fault == Fault{},
                  "Reach a conditional transfer boundary");
        }
        require(same(reference.state(), product.state()),
                "Preserve conditional accumulator and address transfers at each deadline");
        if (deadline >= 2) {
          require(trace.counts[0] == 2, "Retire the conditional transfer at the end of its region");
        }
      }
    }
  }
}

void grouped_short_immediate_moves() {
  constexpr auto nop = static_cast<Word>(isa::Opcode::nop);
  constexpr auto stop = static_cast<Word>(isa::Opcode::stop);
  constexpr auto add_x0 = static_cast<Word>(isa::Opcode::add_x0_a) & 0xffU;
  constexpr Word move_x0 = 0x248000 | add_x0;
  constexpr Word move_a1 = 0x2cff00;
  constexpr Word turns = 4;
  constexpr Registers seed{.x = value, .a = 0x752654'000103ULL, .pc = start};
  constexpr std::array linear{nop, move_x0, nop, move_a1, stop};
  constexpr std::array loop{
      static_cast<Word>(isa::Opcode::do_immediate) | (turns << 8), start + 3, nop, move_x0, stop};
  for (const auto program : {std::span<const Word>(linear), std::span<const Word>(loop)}) {
    const Storage storage;
    Core reference(storage.memory());
    reference.load(Space::program, start, program);
    reference.install(image(storage, false, program), 0);
    test::RetirementIo trace;
    Core product(storage.memory(), &trace);
    product.install(observed_direct_image(program), 0);
    for (const auto deadline : {0U, 1U, 2U, 3U, 4U, 6U, 11U}) {
      trace.counts = {};
      for (auto *core : {&reference, &product}) {
        core->reset();
        core->set_registers(seed);
        require(core->run_until(deadline).fault == Fault{},
                "Reach a short immediate MOVE boundary");
      }
      require(same(reference.state(), product.state()),
              "Preserve the ALU's input before the parallel MOVE at each deadline");
      require(reference.run().fault == Fault{} && product.run().fault == Fault{} &&
                  same(reference.state(), product.state()),
              "Resume short immediate sequences and DO loops at the exact guest boundary");
      if (deadline == 0) {
        require(trace.counts[0] >= program.size() - 1,
                "Retire short data immediate MOVEs as part of a native region");
      }
    }
  }
}

void grouped_single_wrap(const bool unit_steps) {
  const Storage storage;
  constexpr Word turns = 8;
  constexpr Word body = start + 2;
  constexpr Word ring_first = 128;
  constexpr Word modifier = 0xab000c;
  constexpr auto update = static_cast<Word>(isa::Opcode::move_address_update) | (1U << 11);
  constexpr auto load_x = static_cast<Word>(isa::Opcode::move_x_effective) | (1U << 15);
  constexpr auto postincrement = load_x | (3U << 11) | (1U << 16);
  constexpr auto indexed = load_x | (5U << 11);
  constexpr auto predecrement = load_x | (7U << 11);
  const std::array program{static_cast<Word>(isa::Opcode::do_immediate) | (turns << 8),
                           body + 2,
                           unit_steps ? postincrement : update,
                           unit_steps ? predecrement : indexed,
                           static_cast<Word>(isa::Opcode::add_x0_a),
                           static_cast<Word>(isa::Opcode::stop)};
  std::array<Word, 256> data{};
  for (std::size_t i = 0; i < data.size(); ++i) { data[i] = static_cast<Word>(i); }
  Core programmer(storage.memory());
  programmer.load(Space::program, start, program);
  programmer.load(Space::x, 0, data);
  const auto exact = image(storage, false, program);
  const auto direct = observed_direct_image(program);
  test::RetirementIo trace;
  Core reference(storage.memory());
  Core product(storage.memory(), &trace);
  reference.install(exact, 0);
  product.install(direct, 0);
  for (const auto stride : {3U, word_mask - 1}) {
    for (const auto deadline : {1U, 4U, 11U, 23U, 34U}) {
      Registers seed{};
      seed.pc = start;
      seed.r[0] = ring_first + 11;
      seed.n[0] = stride;
      seed.m.fill(word_mask);
      seed.m[0] = modifier;
      trace.counts = {};
      for (auto *core : {&reference, &product}) {
        core->reset();
        core->set_registers(seed);
        require(core->run_until(deadline).fault == Fault{}, "Reach the requested cycle boundary");
      }
      require(same(reference.state(), product.state()),
              "Preserve the interrupted single-wrap loop");
      require(reference.run().fault == Fault{} && product.run().fault == Fault{} &&
                  same(reference.state(), product.state()),
              "Resume the reads and address updates at the original guest boundary");
      require(trace.counts[0] != 0, "Retire the valid single-wrap body as a native region");
    }
  }
}

void grouped_modifier_and_mixed_products() {
  constexpr Word movec = static_cast<Word>(isa::Opcode::movec_register);
  constexpr Word n1 = 25;
  constexpr Word n2 = 26;
  constexpr Word r5 = 21;
  constexpr Word m2 = 2;
  constexpr std::array program{movec | (n1 << 8) | m2,
                               movec | (1U << 15) | (n2 << 8) | m2,
                               static_cast<Word>(isa::Opcode::mpy_su) | 0x14U,
                               movec | (r5 << 8) | m2,
                               static_cast<Word>(isa::Opcode::mac_su) | 0x6aU,
                               static_cast<Word>(isa::Opcode::stop)};
  const Storage storage;
  test::RetirementIo trace;
  Core reference(storage.memory());
  Core product(storage.memory(), &trace);
  reference.load(Space::program, start, program);
  reference.install(image(storage, false, program), 0);
  product.install(observed_direct_image(program), 0);
  Registers seed{};
  constexpr auto x_operands = 0x800001ff'ffffULL;
  constexpr auto y_operands = 0x7fffff00'003dULL;
  constexpr Word old_modifier = 0x8a0007;
  constexpr Word new_modifier = 0x91abcd;
  seed.pc = start;
  seed.x = x_operands;
  seed.y = y_operands;
  seed.m[m2] = old_modifier;
  seed.n[2] = new_modifier;
  for (const auto deadline : {0U, 1U, 3U, 7U}) {
    trace.counts = {};
    for (auto *core : {&reference, &product}) {
      core->reset();
      core->set_registers(seed);
      require(core->run_until(deadline).fault == Fault{}, "Reach the modifier transfer boundary");
    }
    require(same(reference.state(), product.state()), "Preserve state at the exact cycle boundary");
    require(reference.run().fault == Fault{} && product.run().fault == Fault{} &&
                same(reference.state(), product.state()),
            "Resume mixed arithmetic with the live modifier and original flags");
    require(product.registers().n[1] == seed.m[m2] && product.registers().r[5] == seed.n[2],
            "Read both the entry modifier and its updated value");
    if (deadline == 0) {
      require(trace.counts[0] == program.size() - 1,
              "Retire mixed arithmetic and modifier moves together");
    }
  }
}

void grouped_power_products() {
  constexpr auto nop = static_cast<Word>(isa::Opcode::nop);
  constexpr auto stop = static_cast<Word>(isa::Opcode::stop);
  constexpr auto multiply = static_cast<Word>(isa::Opcode::mpy_power);
  constexpr Word minimum_power = 23U << 8;
  constexpr Word maximum_power = 1U << 8;
  constexpr Word rounded_subtract = 7U;
  constexpr Word destination_b = 1U << 3;
  constexpr std::array program{nop,
                               multiply | maximum_power | rounded_subtract,
                               nop,
                               multiply | minimum_power | destination_b,
                               stop};
  constexpr std::array invalid{nop, multiply, stop};
  constexpr Registers seed{
      .x = 0x18'7caa'fd7f01ULL, .y = 0xff'ffe1'34b23fULL, .a = 0x7f'ffff'fffed1'41ULL, .pc = start};
  const Storage storage;
  Core reference(storage.memory());
  test::RetirementIo trace;
  Core product(storage.memory(), &trace);
  for (const auto words : {std::span<const Word>(program), std::span<const Word>(invalid)}) {
    reference.load(Space::program, start, words);
    reference.install(image(storage, false, words), 0);
    product.install(observed_direct_image(words), 0);
    for (const auto deadline : {0U, 1U, 2U, 3U, 4U}) {
      trace.counts = {};
      for (auto *core : {&reference, &product}) {
        core->reset();
        core->set_registers(seed);
        static_cast<void>(core->run_until(deadline));
      }
      require(same(reference.state(), product.state()),
              "Preserve the power-product cycle boundary");
      const auto expected = reference.run();
      const auto actual = product.run();
      require(actual.fault == expected.fault && same(reference.state(), product.state()),
              "Preserve the power-product result, flags and invalid coefficients");
      if (deadline == 0 && words.size() == program.size()) {
        require(trace.counts[0] == program.size() - 1, "Retire power products in native groups");
      }
    }
  }
}

void grouped_displacement_moves() {
  constexpr auto short_load = static_cast<Word>(isa::Opcode::move_displacement_short);
  constexpr auto long_move = static_cast<Word>(isa::Opcode::move_displacement_long);
  constexpr Word short_offset = 0x79;
  constexpr Word long_offset = 77;
  constexpr Word y_on_r1 = (1U << 16) | (1U << 8);
  constexpr std::array program{short_load | ((short_offset & 0x7eU) << 10) |
                                   ((short_offset & 1U) << 6) | (1U << 4) | 4U,
                               static_cast<Word>(isa::Opcode::add_x0_a),
                               long_move | y_on_r1 | 12U,
                               long_offset,
                               long_move | y_on_r1 | (1U << 6) | 6U,
                               long_offset,
                               static_cast<Word>(isa::Opcode::stop)};
  const Storage checked_storage;
  const Storage direct_storage;
  test::RetirementIo trace;
  Core reference(checked_storage.memory());
  Core product(direct_storage.memory(), &trace);
  std::array<Word, 512> data{};
  for (std::size_t i = 0; i < data.size(); ++i) { data[i] = static_cast<Word>(i + 1); }
  for (auto *core : {&reference, &product}) {
    core->load(Space::program, start, program);
    core->load(Space::x, 0, data);
  }
  const auto exact_image = image(checked_storage, false, program);
  const auto direct_image = observed_direct_image(program);
  reference.install(exact_image, 0);
  product.install(direct_image, 0);
  constexpr std::array modifiers{word_mask, 0x801fU, 22U};
  constexpr Word source = 0x105;
  constexpr Word destination = 0x131;
  for (const auto modifier : modifiers) {
    for (const auto deadline : {0U, 2U, 5U}) {
      trace.counts = {};
      for (auto *core : {&reference, &product}) {
        core->reset();
        Registers seed{};
        seed.pc = start;
        seed.r[0] = source;
        seed.r[1] = destination;
        seed.m[0] = modifier;
        seed.m[1] = modifier;
        core->set_registers(seed);
        require(core->run_until(deadline).fault == Fault{}, "Reach a displacement cycle boundary");
      }
      require(same(reference.state(), product.state()), "Preserve the displacement move prefix");
      require(reference.run().fault == Fault{} && product.run().fault == Fault{} &&
                  same(reference.state(), product.state()),
              "Resume displacement moves with the same operands and result");
      require(product.registers().r[0] == source && product.registers().r[1] == destination,
              "Displacement addressing leaves both cursor registers unchanged");
      require(std::ranges::equal(checked_storage.regions[2].words, direct_storage.regions[2].words),
              "Store the same word at the same bank address");
      if (deadline == 0) {
        require(trace.counts[0] == 4, "Retire both displacement widths as one native region");
      }
    }
  }
  constexpr Word peripheral_offset = 6;
  for (const auto modifier : {0U, word_mask}) {
    Core checked_fault(checked_storage.memory());
    Core direct_fault(direct_storage.memory());
    checked_fault.install(exact_image, 0);
    direct_fault.install(direct_image, 0);
    Registers seed{};
    seed.pc = start;
    seed.r[0] = modifier == 0 ? source : peripheral_offset;
    seed.m[0] = modifier;
    checked_fault.set_registers(seed);
    direct_fault.set_registers(seed);
    const auto expected = checked_fault.run();
    const auto actual = direct_fault.run();
    require(expected.fault != Fault{} && actual.fault == expected.fault && actual.retired == 0 &&
                actual.fault.pc == start,
            "Keep the original address and PC on protected and peripheral accesses");
  }
}

void grouped_modifier_memory() {
  constexpr Word effective = static_cast<Word>(isa::Opcode::movec_effective);
  constexpr Word absolute = static_cast<Word>(isa::Opcode::movec_absolute_short);
  constexpr Word register_move = static_cast<Word>(isa::Opcode::movec_register);
  constexpr Word literal = 0x8f001f;
  constexpr Word m2 = 2;
  constexpr Word m3 = 3;
  constexpr Word n1 = 25;
  constexpr Word load = 1U << 15;
  constexpr Word destination = 11;
  constexpr Word source = 12;
  constexpr std::array program{effective | load | (6U << 11) | (4U << 8) | m2,
                               literal,
                               absolute | (1U << 6) | (destination << 8) | m2,
                               effective | load | (3U << 11) | m3,
                               register_move | (n1 << 8) | m3,
                               static_cast<Word>(isa::Opcode::stop)};
  const Storage exact_storage;
  const Storage direct_storage;
  test::RetirementIo trace;
  Core reference(exact_storage.memory());
  Core product(direct_storage.memory(), &trace);
  for (auto *core : {&reference, &product}) {
    core->load(Space::program, start, program);
    constexpr std::array data{value};
    core->load(Space::x, source, data);
  }
  reference.install(image(exact_storage, false, program), 0);
  product.install(observed_direct_image(program), 0);
  for (const auto deadline : {0U, 1U, 3U, 5U}) {
    trace.counts = {};
    for (auto *core : {&reference, &product}) {
      core->reset();
      Registers seed{};
      seed.pc = start;
      seed.r[0] = source;
      seed.m[0] = word_mask;
      core->set_registers(seed);
      require(core->run_until(deadline).fault == Fault{}, "Reach the MOVEC memory cycle boundary");
    }
    require(same(reference.state(), product.state()), "Preserve MOVEC state before resume");
    require(reference.run().fault == Fault{} && product.run().fault == Fault{} &&
                same(reference.state(), product.state()),
            "Keep modifier values and cursor updates exact across memory transfers");
    require(direct_storage.word(Space::y, destination) == literal &&
                product.registers().r[0] == source + 1 && product.registers().n[1] == value,
            "Store the active modifier and feed the loaded modifier to the register move");
    if (deadline == 0) {
      require(trace.counts[0] == 4, "Retire all modifier transfer forms in one region");
    }
  }
}
}

int main() try {
  const test::NativeFaults faults(Core::host_fault);
  grouped_conditional_transfers();
  grouped_short_immediate_moves();
  grouped_single_wrap(false);
  grouped_single_wrap(true);
  grouped_modifier_and_mixed_products();
  grouped_power_products();
  grouped_displacement_moves();
  grouped_modifier_memory();
  crossing_zero();
  direct_self_loop_empty_window();
  protected_operands();
  require(!Core::host_fault(0, false, 0),
          "Unowned faults are not claimed outside native execution");
  aliases();
  geometry();
  execution(false);
  execution(true);
  exact_shared_views();
  read_only(false);
  read_only(true);
  execution(false, true);
  execution(true, true);
  read_only(false, true);
  read_only(true, true);
  native_failure(false);
  native_failure(true);
  native_failure(false, true);
  native_failure(true, true);
  mapped_program_source_precedes_io();
  mapped_program_known_boundary();
  program_store();
  repeat_eligibility();
  std::cout << "Borrowed DSP read/write views passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
