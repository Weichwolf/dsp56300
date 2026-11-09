// SPDX-License-Identifier: GPL-3.0-only
#include "jit.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "jit/index.hpp"
#include "jit/resident_region.hpp"
#include "translation.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>
#if defined(__aarch64__)
#include "jit/resident.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#endif
#if defined(__x86_64__) && defined(__linux__)
#include "mac_batch.hpp"
#endif

namespace dsp56300::detail {
namespace {
struct Mapping {
  explicit Mapping(const std::uint64_t context) : key(context) {}

  std::uint64_t key;
  NativeTable table{3};
};

enum class EntryKind : std::uint8_t { ordinary, repeat, vector };

struct Block {
  std::vector<BlockInstruction> operations;
  const void *entry = nullptr;
  std::unique_ptr<ResidentRegion> region;
#if defined(__aarch64__)
  EntryKind kind = EntryKind::ordinary;
  std::array<JitMemory, 2> memory{};
#endif
};
}

struct Jit::Impl {
  Impl(Cpu &state,
       const std::span<const Word> supplied,
       const JitRuntime runtime,
       const NativeBus bus,
       const NativeEvents events)
      : cpu(state),
        program(supplied),
        context{
            .runtime = runtime, .exit_requested = &exit_requested, .bus = bus, .events = events} {}

  [[nodiscard]] Block *compile();
#if defined(__aarch64__)
  [[nodiscard]] Block *reuse(std::span<const BlockInstruction> operations, EntryKind kind) const;
#endif
  [[nodiscard]] Block *retain(std::unique_ptr<Block> block);
  [[nodiscard]] const void *prepare();
  [[nodiscard]] std::uint32_t enter(const Block &block, Word budget);
  [[nodiscard]] std::uint32_t finish(std::uint32_t retired);
  Cpu &cpu;
  std::span<const Word> program;
  std::unordered_map<std::uint64_t, Block *> blocks;
  std::unordered_map<std::uint64_t, std::unique_ptr<Mapping>> mappings;
  Mapping *current = nullptr;
  NativeTable catalog{4};
  std::optional<BlockInstruction> rejected_repeat;
  bool invalidated = false;
  bool exit_requested = false;
  NativeContext context;
  ExecutableArena arena;
  std::vector<std::unique_ptr<Block>> storage;
  std::array<std::vector<NativeWindow>, 2> windows;
};

Block *Jit::Impl::compile() {
  auto block = std::make_unique<Block>();
#if defined(__aarch64__)
  const bool callbacks = (context.events.io != nullptr || context.bus.external) &&
                         active_loop_address(cpu.reg) == inactive_loop;
#else
  constexpr bool callbacks = false;
#endif
  block->operations = parse_block(program, cpu.reg, context.runtime, callbacks);
  if (block->operations.empty()) { return nullptr; }
#if defined(__aarch64__)
  block->region = resident_region_plan(program, cpu.reg, context.runtime, block->operations);
  if (!block->region) {
    if (auto *existing = reuse(block->operations, EntryKind::ordinary)) { return existing; }
  }
#endif
  auto bytes = emit_block(block->operations, context.runtime, cpu.reg.m);
#if defined(__x86_64__)
  block->region = resident_region_plan(program, cpu.reg, context.runtime, block->operations);
#endif
  if (block->region) {
    auto &region = *block->region;
    const auto full_width = 1U << (region.native.size() - 1);
    for (unsigned index = 0; index < region.native.size(); ++index) {
      const unsigned lanes = 1U << index;
      // Slot zero batches child columns; the other slots batch parents.
      if (region.nested && ((lanes == 1 && (region.inner_iterations < full_width ||
                                            region.inner_iterations % full_width != 0)) ||
                            (lanes == 2 && region.inner_iterations > full_width))) {
        continue;
      }
      region.native[index] = arena.append(emit_resident_region(region, lanes));
    }
    bytes = select_resident_region(region, bytes);
  }
#if defined(__x86_64__)
  const std::span<const BlockInstruction> admission = block->region && block->region->nested
                                                          ? std::span<const BlockInstruction>{}
                                                          : block->operations;
#else
  const std::span<const BlockInstruction> admission;
#endif
  block->entry = arena.append(admit_cycles(bytes, admission));
  return retain(std::move(block));
}

#if defined(__aarch64__)
Block *Jit::Impl::reuse(const std::span<const BlockInstruction> operations,
                        const EntryKind kind) const {
  const auto memory = resident_runtime(context.runtime).memory;
  const auto same_memory = [](const JitMemory &left, const JitMemory &right) {
    return left.data == right.data && left.words == right.words &&
           left.aliases_program == right.aliases_program;
  };
  for (const auto &block : storage) {
    // Nested regions retain child code beyond this operation span.
    if (block->kind != kind || block->region ||
        !std::ranges::equal(block->memory, memory, same_memory)) {
      continue;
    }
    const auto equal = [](const BlockInstruction &left, const BlockInstruction &right) {
      return left.instruction == right.instruction && left.pc == right.pc &&
             left.mode == right.mode && left.loop_address == right.loop_address;
    };
    if (std::ranges::equal(block->operations, operations, equal)) { return block.get(); }
  }
  return nullptr;
}
#endif

Jit::Jit(Cpu &cpu,
         const std::span<const Word> program,
         const JitRuntime runtime,
         const NativeBus bus,
         const NativeEvents events)
    : impl_(std::make_unique<Impl>(cpu, program, runtime, bus, events)) {}

Jit::~Jit() = default;

bool Jit::available() noexcept {
#if defined(__x86_64__) && defined(__linux__)
  return native_mac_available();
#elif defined(__aarch64__) && defined(__APPLE__)
  return true;
#else
  return false;
#endif
}

bool Jit::exit_requested() const noexcept {
  return impl_->exit_requested;
}

void Jit::request_yield() noexcept {
  impl_->exit_requested = true;
}

void Jit::invalidate() noexcept {
  // Leave the active entry before invalidating its lookup. Published code and
  // embedded operation metadata remain owned until Core destruction.
  if ((!impl_->blocks.empty() || !impl_->mappings.empty()) && !impl_->invalidated) {

    impl_->invalidated = true;
    impl_->exit_requested = true;
  }
}

bool Jit::bind_windows(const std::array<std::span<const MemoryWindow>, 2> windows) noexcept {
#if defined(__aarch64__)
  std::array<std::vector<NativeWindow>, 2> prepared;
  try {
    for (std::size_t bank = 0; bank < windows.size(); ++bank) {
      auto &native = prepared[bank];
      native.reserve(windows[bank].size());
      for (const auto &window : windows[bank]) {
        native.push_back({.address = window.address,
                          .words = window.words.size(),
                          .data = window.words.data(),
                          .known = window.known.empty() ? nullptr : window.known.data(),
                          .writable = window.writable,
                          .aliases_program = window.aliases_program});
      }
    }
  } catch (const std::exception &) { return false; }
  impl_->windows.swap(prepared);
  for (std::size_t bank = 0; bank < windows.size(); ++bank) {
    const auto &native = impl_->windows[bank];
    impl_->context.bus.windows[bank] = {.data = native.data(), .count = native.size()};
  }
#endif
  impl_->context.runtime.windows = windows;
  invalidate();
  return true;
}

void Jit::reset() {
  impl_->blocks.clear();
  impl_->mappings.clear();
  impl_->catalog.clear();
  impl_->current = nullptr;
  impl_->rejected_repeat.reset();
  impl_->invalidated = false;
  impl_->exit_requested = false;
}

Block *Jit::Impl::retain(std::unique_ptr<Block> block) {
#if defined(__aarch64__)
  block->memory = resident_runtime(context.runtime).memory;
#endif
  auto *const entry = block.get();
  storage.push_back(std::move(block));
  return entry;
}

const void *Jit::Impl::prepare() {
  if (invalidated) {
    blocks.clear();
    mappings.clear();
    catalog.clear();
    current = nullptr;
    invalidated = false;
  }
  exit_requested = false;
  if (cpu.reg.pc > word_mask) { return nullptr; }
  const auto key = translation_key(cpu.reg) >> word_bits;
  if (current == nullptr || current->key != key) {
    auto &mapping = mappings[key];
    if (!mapping) { mapping = std::make_unique<Mapping>(key); }
    current = mapping.get();
    if (key <= std::numeric_limits<std::uint32_t>::max()) {
      catalog.insert(static_cast<std::uint32_t>(key),
                     static_cast<const void *>(current->table.index().data()));
    }
  }
  if (const auto *entry = current->table.find(cpu.reg.pc)) { return entry; }
  const auto *retained = compile();
  if (retained == nullptr) { return nullptr; }
  current->table.insert(cpu.reg.pc, retained->entry);
  return retained->entry;
}

std::uint32_t Jit::Impl::enter(const Block &block, const Word budget) {
  context.budget = budget;
  context.boundary = NativeBoundary::none;
  context.failure = {};
  return finish(enter_block(block.entry, cpu, context));
}

std::uint32_t Jit::Impl::finish(const std::uint32_t retired) {
  if (context.bus.failure != nullptr && context.bus.failure->kind == FaultKind::none &&
      context.failure.kind != FaultKind::none) {
    *context.bus.failure = context.failure;
  }
  if (context.program_written) {
    context.program_written = false;
    invalidated = true;
    exit_requested = true;
  }
  return retired;
}

std::uint32_t Jit::run(const std::uint32_t budget) {
  auto &self = *impl_;
  if (budget == 0) { return 0; }
  const auto *entry = self.prepare();
  if (entry == nullptr) { return 0; }
  self.context.boundary = NativeBoundary::none;
  self.context.failure = {};
  return self.finish(enter_blocks(
      entry, self.cpu, self.context, budget, self.current->table.index(), &self.catalog.index()));
}

void Jit::set_cycle_deadline(const std::uint64_t deadline) noexcept {
  impl_->context.cycle_deadline = deadline;
}

std::uint32_t Jit::run_repeat(const BlockInstruction operation, const std::uint32_t budget) {
  auto &self = *impl_;
  if (budget == 0 || self.cpu.repeat_remaining == 0) { return 0; }
  if (self.invalidated) {
    self.blocks.clear();
    self.mappings.clear();
    self.catalog.clear();
    self.current = nullptr;

    self.invalidated = false;
  }
  self.exit_requested = false;
  if (self.rejected_repeat && self.rejected_repeat->instruction == operation.instruction &&
      self.rejected_repeat->mode == operation.mode && self.rejected_repeat->pc == operation.pc &&
      self.rejected_repeat->loop_address == operation.loop_address) {
    return 0;
  }
  constexpr auto repeat_tag = std::uint64_t{1} << 63;
  const auto key = translation_key(self.cpu.reg) | repeat_tag;
  auto found = self.blocks.find(key);
  // A loaded REP body survives backing-P mutation. Reusing the same PC for a
  // later REP must still distinguish its genuine new opcode from that latch.
  if (found == self.blocks.end() ||
      found->second->operations.front().instruction != operation.instruction) {
#if defined(__aarch64__)
    if (auto *existing = self.reuse(std::span(&operation, 1), EntryKind::repeat)) {
      self.blocks.insert_or_assign(key, existing);
      return self.enter(*existing, budget);
    }
#endif
    auto block = std::make_unique<Block>();
    block->operations.push_back(operation);
#if defined(__aarch64__)
    block->kind = EntryKind::repeat;
#endif
    const auto bytes = emit_repeat(block->operations.front(), self.context.runtime, self.cpu.reg.m);
    if (bytes.empty()) {
      self.rejected_repeat = operation;
      return 0;
    }
    block->entry = self.arena.append(admit_cycles(bytes));

    self.blocks.insert_or_assign(key, self.retain(std::move(block)));

    found = self.blocks.find(key);
  }
  return self.enter(*found->second, budget);
}

std::uint32_t Jit::run_captured(BlockInstruction operation) {
  auto &self = *impl_;
  if (self.cpu.repeat_remaining != 0) {
#if !defined(__aarch64__)
    // The final REP at a DO endpoint still needs the genuine frame cleanup.
    if (at_loop_end(operation) && self.cpu.repeat_remaining == 1) { return 0; }
    operation.loop_address = inactive_loop;
#endif
    return run_repeat(operation, 1);
  }
  if (self.invalidated) {
    self.blocks.clear();
    self.mappings.clear();
    self.catalog.clear();
    self.current = nullptr;

    self.invalidated = false;
  }
  self.exit_requested = false;
  constexpr auto captured_tag = std::uint64_t{1} << 62;
  const auto key = translation_key(self.cpu.reg) | captured_tag;
  auto found = self.blocks.find(key);
  if (found == self.blocks.end() ||
      found->second->operations.front().instruction != operation.instruction) {
#if defined(__aarch64__)
    if (auto *existing = self.reuse(std::span(&operation, 1), EntryKind::ordinary)) {
      self.blocks.insert_or_assign(key, existing);
      return self.enter(*existing, 1);
    }
#endif
    auto block = std::make_unique<Block>();
    block->operations.push_back(operation);
    const auto bytes = emit_block(block->operations, self.context.runtime, self.cpu.reg.m);
    block->entry = self.arena.append(admit_cycles(bytes));

    self.blocks.insert_or_assign(key, self.retain(std::move(block)));

    found = self.blocks.find(key);
  }
  return self.enter(*found->second, 1);
}

#if defined(__aarch64__)
std::uint32_t Jit::run_vector(const BlockInstruction operation) {
  auto &self = *impl_;
  if (self.invalidated) {
    self.blocks.clear();
    self.mappings.clear();
    self.catalog.clear();
    self.current = nullptr;
    self.invalidated = false;
  }
  self.exit_requested = false;
  constexpr auto vector_tag = std::uint64_t{1} << 61;
  const auto key =
      (translation_key(self.cpu.reg) & ~std::uint64_t{word_mask}) | operation.pc | vector_tag;
  auto found = self.blocks.find(key);
  if (found == self.blocks.end() ||
      found->second->operations.front().instruction != operation.instruction) {
    auto *block = self.reuse(std::span(&operation, 1), EntryKind::vector);
    if (block == nullptr) {
      auto created = std::make_unique<Block>();
      created->kind = EntryKind::vector;
      created->operations.push_back(operation);
      created->entry = self.arena.append(admit_cycles(emit_vector(operation)));
      block = self.retain(std::move(created));
    }
    found = self.blocks.insert_or_assign(key, block).first;
  }
  return self.enter(*found->second, 1);
}
#endif

} // namespace dsp56300::detail
