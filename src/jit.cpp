// SPDX-License-Identifier: GPL-3.0-only
#include "jit.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "jit/emitter.hpp"
#include "jit/index.hpp"
#include "jit/region.hpp"
#include "mac_batch.hpp"
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

namespace dsp56300::detail {
namespace {
struct Mapping {
  explicit Mapping(const std::uint64_t context) : key(context) {}

  std::uint64_t key;
  NativeTable table{3};
};

struct Block {
  std::vector<BlockInstruction> operations;
  const void *entry = nullptr;
  std::unique_ptr<NativeRegion> region;
};
}

struct Jit::Impl {
  Impl(Cpu &state, const std::span<const Word> supplied, const JitRuntime runtime)
      : cpu(state),
        program(supplied),
        context{.runtime = runtime, .exit_requested = &exit_requested} {}

  [[nodiscard]] std::unique_ptr<Block> compile();
  [[nodiscard]] Block *retain(std::unique_ptr<Block> block);
  [[nodiscard]] const void *prepare();
  [[nodiscard]] std::uint32_t enter(const Block &block, Word budget);
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
};

std::unique_ptr<Block> Jit::Impl::compile() {
  auto block = std::make_unique<Block>();
  block->operations = parse_block(program, cpu.reg, context.runtime);
  if (block->operations.empty()) { return nullptr; }
  auto bytes = emit_block(block->operations, context.runtime, cpu.reg.m);
  block->region = plan_region(program, cpu.reg, context.runtime, block->operations);
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
      region.native[index] = arena.append(emit_region(region, lanes));
    }
    bytes = select_region(region, bytes);
  }
  block->entry = arena.append(bytes);
  return block;
}

Jit::Jit(Cpu &cpu, const std::span<const Word> program, const JitRuntime runtime)
    : impl_(std::make_unique<Impl>(cpu, program, runtime)) {}

Jit::~Jit() = default;

bool Jit::available() noexcept {
#if defined(__x86_64__) && defined(__linux__)
  return avx2_mac_available();
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

void Jit::bind_windows(const std::array<std::span<const MemoryWindow>, 2> windows) noexcept {
  impl_->context.runtime.windows = windows;
  invalidate();
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
  auto block = compile();
  if (!block) { return nullptr; }
  const auto *retained = retain(std::move(block));
  current->table.insert(cpu.reg.pc, retained->entry);
  return retained->entry;
}

std::uint32_t Jit::Impl::enter(const Block &block, const Word budget) {
  context.budget = budget;
  return enter_block(block.entry, cpu, context);
}

std::uint32_t Jit::run(const std::uint32_t budget) {
  auto &self = *impl_;
  if (budget == 0) { return 0; }
  const auto *entry = self.prepare();
  if (entry == nullptr) { return 0; }
  self.context.boundary = NativeBoundary::none;
  return enter_blocks(
      entry, self.cpu, self.context, budget, self.current->table.index(), &self.catalog.index());
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
    auto block = std::make_unique<Block>();
    block->operations.push_back(operation);
    BlockEmitter emitter(self.context.runtime, self.cpu.reg.m);
    const auto bytes = emitter.generate_repeat(block->operations.front());
    if (bytes.empty()) {
      self.rejected_repeat = operation;
      return 0;
    }
    block->entry = self.arena.append(bytes);

    self.blocks.insert_or_assign(key, self.retain(std::move(block)));

    found = self.blocks.find(key);
  }
  return self.enter(*found->second, budget);
}

std::uint32_t Jit::run_captured(BlockInstruction operation) {
  auto &self = *impl_;
  if (self.cpu.repeat_remaining != 0) {
    // The final REP at a DO endpoint still needs the genuine frame cleanup.
    if (at_loop_end(operation) && self.cpu.repeat_remaining == 1) { return 0; }
    operation.loop_address = inactive_loop;
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
    auto block = std::make_unique<Block>();
    block->operations.push_back(operation);
    BlockEmitter emitter(self.context.runtime, self.cpu.reg.m);
    const auto bytes = emitter.generate(block->operations);
    block->entry = self.arena.append(bytes);

    self.blocks.insert_or_assign(key, self.retain(std::move(block)));

    found = self.blocks.find(key);
  }
  return self.enter(*found->second, 1);
}

} // namespace dsp56300::detail
