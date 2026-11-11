// SPDX-License-Identifier: GPL-3.0-only
#include "jit.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "io.hpp"
#include "jit/arena.hpp"
#include "jit/block.hpp"
#include "jit/index.hpp"
#include "jit/llvm/region.hpp"
#include "jit/llvm/runtime.hpp"
#include "jit/preparation.hpp"
#include "jit/resident_region.hpp"
#include "translation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>
#if defined(__aarch64__)
#include "jit/resident.hpp"

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

bool whole_loop_entry(const Registers &state) {
  return (state.sr & loop_flag) != 0 && state.sc >= 2 &&
         state.stack[state.sp % stack_entries] >> word_bits == state.pc;
}

struct Block {
  std::vector<BlockInstruction> operations;
  const void *entry = nullptr;
  const void *direct = nullptr;
  std::unique_ptr<ResidentRegion> region;
  std::uint64_t native_bytes = 0;
  bool optimized = false;
#if defined(__aarch64__)
  EntryKind kind = EntryKind::ordinary;
  std::array<JitMemory, 2> memory{};
#endif
};

#if defined(__aarch64__)
bool callback_boundaries(const NativeContext &context, const Registers &state) noexcept {
  return (context.events.io != nullptr || context.bus.external) &&
         active_loop_address(state) == inactive_loop;
}
#endif
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

  [[nodiscard]] Block *compile(const Registers &state, bool requested = false);
  void compile_resident(Block &block);
  [[nodiscard]] Mapping &mapping(const Registers &state);
  [[nodiscard]] Block *reuse_region(const Registers &state) const;
  void bind_region(Block &block);
  void clear_invalidated();
  [[nodiscard]] bool current_view(std::optional<std::uint64_t> revision) const;
  [[nodiscard]] std::optional<PreparedRegion>
  prepare_entry(const Registers &state, std::optional<std::uint64_t> revision, bool complete_loop);
#if defined(__aarch64__)
  [[nodiscard]] Block *reuse(std::span<const BlockInstruction> operations, EntryKind kind) const;
#endif
  [[nodiscard]] Word first_budget(const Block &block, Word budget) const;
  [[nodiscard]] Block *retain(std::unique_ptr<Block> block);
  [[nodiscard]] const void *prepare(Word &budget);
  [[nodiscard]] std::uint32_t enter(const Block &block, Word budget);
  [[nodiscard]] std::uint32_t finish(std::uint32_t retired);
  Cpu &cpu;
  std::span<const Word> program;
  std::unordered_map<std::uint64_t, Block *> blocks;
  std::unordered_map<std::uint64_t, Block *> shared_regions;
  std::unordered_map<std::uint64_t, std::unique_ptr<Mapping>> mappings;
  Mapping *current = nullptr;
  NativeTable catalog{4};
  std::optional<BlockInstruction> rejected_repeat;
  bool invalidated = false;
  bool exit_requested = false;
  NativeContext context;
  std::unique_ptr<LlvmCompiler> compiler;
  ExecutableArena arena;
  std::vector<std::unique_ptr<Block>> storage;
  std::array<std::vector<NativeWindow>, 2> windows;
#if defined(__aarch64__)
  std::unordered_multimap<Word, Block *> reusable;
#endif
};

Block *Jit::Impl::compile(const Registers &state, const bool requested) {
  if (auto *shared = reuse_region(state)) { return shared; }
  auto block = std::make_unique<Block>();
#if defined(__aarch64__)
  const bool callbacks = !requested && callback_boundaries(context, state);
#else
  constexpr bool callbacks = false;
#endif
  block->operations = parse_block(program, state, context.runtime, callbacks);
  if (block->operations.empty()) { return nullptr; }
#if defined(__aarch64__)
  if (!requested) {
    block->region = resident_region_plan(program, state, context.runtime, block->operations);
  }
  if (!requested && !block->region) {
    if (auto *existing = reuse(block->operations, EntryKind::ordinary)) { return existing; }
  }
#endif
  auto bytes = emit_block(block->operations, context.runtime, state.m);
#if defined(__x86_64__)
  if (!requested) {
    block->region = resident_region_plan(program, state, context.runtime, block->operations);
  }
#endif
  if (block->region) {
    compile_resident(*block);
    bytes = select_resident_region(*block->region, bytes);
  }
  const std::span<const BlockInstruction> admission = block->region && block->region->nested
                                                          ? std::span<const BlockInstruction>{}
                                                          : block->operations;
  std::size_t body_offset = 0;
  const auto native = admit_cycles(bytes, admission, &body_offset);
  block->native_bytes += native.size();
  block->entry = arena.append(native);
  if (!block->region) {
    block->direct = static_cast<const std::uint8_t *>(block->entry) + body_offset;
  }
  if (requested) {
    if (const auto compiled =
            compile_region(compiler, block->operations, context.runtime, block->entry)) {
      block->entry = compiled->entry;
      block->native_bytes += compiled->native_bytes;
      block->optimized = true;
      block->direct = nullptr;
    }
  }
  return retain(std::move(block));
}

void Jit::Impl::compile_resident(Block &block) {
  auto &region = *block.region;
  const auto full_width = 1U << (region.native.size() - 1);
  for (unsigned index = 0; index < region.native.size(); ++index) {
    const unsigned lanes = 1U << index;
    // Slot zero batches child columns; the other slots batch parents.
    if (region.nested && ((lanes == 1 && (region.inner_iterations < full_width ||
                                          region.inner_iterations % full_width != 0)) ||
                          (lanes == 2 && region.inner_iterations > full_width))) {
      continue;
    }
    const auto native = emit_resident_region(region, lanes);
    block.native_bytes += native.size();
    region.native[index] = arena.append(native);
  }
}

#if defined(__aarch64__)
Block *Jit::Impl::reuse(const std::span<const BlockInstruction> operations,
                        const EntryKind kind) const {
  const auto memory = resident_runtime(context.runtime).memory;
  const auto same_memory = [](const JitMemory &left, const JitMemory &right) {
    return left.data == right.data && left.words == right.words &&
           left.aliases_program == right.aliases_program;
  };
  const auto range = reusable.equal_range(operations.front().pc);
  for (auto item = range.first; item != range.second; ++item) {
    auto *block = item->second;
    // Nested regions retain child code beyond this operation span.
    if (block->kind != kind || block->region ||
        !std::ranges::equal(block->memory, memory, same_memory)) {
      continue;
    }
    const auto equal = [](const BlockInstruction &left, const BlockInstruction &right) {
      return left.instruction == right.instruction && left.pc == right.pc &&
             left.mode == right.mode && left.loop_address == right.loop_address;
    };
    if (std::ranges::equal(block->operations, operations, equal)) { return block; }
  }
  return nullptr;
}

#endif

Word Jit::Impl::first_budget(const Block &block, const Word budget) const {
  auto credit = context.cycle_deadline - cpu.cycles;
  Word count = 0;
  for (const auto &operation : block.operations) {
    const auto cost = std::max(1U, unsigned{operation.instruction.cycles});
    ++count;
    if (count == budget || cost >= credit) { break; }
    credit -= cost;
  }
  return count;
}

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
  impl_->shared_regions.clear();
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
  if (entry->optimized && region_independent(entry->operations)) { bind_region(*entry); }
#if defined(__aarch64__)
  if (!entry->region && !entry->optimized) {
    reusable.emplace(entry->operations.front().pc, entry);
  }
#endif
  return entry;
}

const void *Jit::Impl::prepare(Word &budget) {
  clear_invalidated();
  exit_requested = false;
  if (cpu.reg.pc > word_mask) { return nullptr; }
  current = &mapping(cpu.reg);
  if (const auto *entry = current->table.find(cpu.reg.pc)) { return entry; }
  const auto *retained = compile(cpu.reg);
  if (retained == nullptr) { return nullptr; }
  current->table.insert(cpu.reg.pc, retained->entry);
  // Exact cold grants avoid suffix compilation; warm lookup retains native admission.
  if (retained->direct != nullptr &&
      context.cycle_deadline != std::numeric_limits<std::uint64_t>::max() &&
      context.cycle_deadline > cpu.cycles) {
    budget = first_budget(*retained, budget);
    return retained->direct;
  }
  return retained->entry;
}

void Jit::Impl::clear_invalidated() {
  if (invalidated) {
    blocks.clear();
    shared_regions.clear();
    mappings.clear();
    catalog.clear();
    current = nullptr;
    invalidated = false;
  }
}

namespace {
std::uint64_t region_key(const Registers &state) {
  return state.pc | (std::uint64_t{state.sr & numeric_mode_mask} << word_bits);
}

bool outside_region(const Registers &state, const std::span<const BlockInstruction> operations) {
  const auto &last = operations.back();
  const auto end = (last.pc + last.instruction.words - 1) & word_mask;
  const auto la = active_loop_address(state);
  return la < operations.front().pc || la > end;
}

bool same_region(const std::span<const BlockInstruction> left,
                 const std::span<const BlockInstruction> right) {
  if (right.empty() || at_loop_end(right.back())) { return false; }
  return std::ranges::equal(left, right, [](const auto &a, const auto &b) {
    return a.pc == b.pc && a.mode == b.mode && a.instruction == b.instruction;
  });
}
}

Block *Jit::Impl::reuse_region(const Registers &state) const {
  const auto known = shared_regions.find(region_key(state));
  if (known == shared_regions.end() || !outside_region(state, known->second->operations)) {
    return nullptr;
  }
  return known->second;
}

void Jit::Impl::bind_region(Block &block) {
  const auto &first = block.operations.front();
  Registers state;
  state.pc = first.pc;
  state.sr = first.mode;
  shared_regions.insert_or_assign(region_key(state), &block);
  // Tables and the shared index describe only the current program/grant view.
  // Upgrade matching ordinary bindings while the execution owner is stopped.
  for (const auto &[key, binding] : mappings) {
    constexpr unsigned numeric_key_bits = 7;
    const auto la = static_cast<Word>(key >> numeric_key_bits);
    const auto &last = block.operations.back();
    const auto end = (last.pc + last.instruction.words - 1) & word_mask;
    if (la >= first.pc && la <= end) { continue; }
    const auto *entry = binding->table.find(first.pc);
    if (entry == nullptr) { continue; }
    const auto known = std::ranges::find_if(
        storage, [entry](const auto &candidate) { return candidate->entry == entry; });
    if (known != storage.end() && !(*known)->optimized &&
        same_region(block.operations, (*known)->operations)) {
      binding->table.insert(first.pc, block.entry);
    }
  }
}

Mapping &Jit::Impl::mapping(const Registers &state) {
  const auto key = translation_key(state) >> word_bits;
  if (current == nullptr || current->key != key) {
    auto &mapping = mappings[key];
    if (!mapping) { mapping = std::make_unique<Mapping>(key); }
    current = mapping.get();
    if (key <= std::numeric_limits<std::uint32_t>::max()) {
      catalog.insert(static_cast<std::uint32_t>(key),
                     static_cast<const void *>(current->table.index().data()));
    }
  }
  return *current;
}

bool Jit::Impl::current_view(const std::optional<std::uint64_t> revision) const {
  return !revision ||
         (context.events.io != nullptr && dsp56300_io_revision(context.events.io) == *revision);
}

std::optional<PreparedRegion> Jit::Impl::prepare_entry(const Registers &state,
                                                       const std::optional<std::uint64_t> revision,
                                                       const bool complete_loop) {
  auto &binding = mapping(state);
  const auto *entry = binding.table.find(state.pc);
  const auto known = std::ranges::find_if(storage, [&](const auto &block) {
    return entry != nullptr && block->entry == entry && block->optimized;
  });
  if (known != storage.end()) { return PreparedRegion{.entry = entry}; }
  if (auto *shared = reuse_region(state)) {
    binding.table.insert(state.pc, shared->entry);
    return PreparedRegion{.entry = shared->entry};
  }
  const auto operations = parse_block(program, state, context.runtime);
  if (!region_candidate(operations, context.runtime, complete_loop)) { return {}; }
  const auto *compiled = compile(state, true);
  if (compiled == nullptr || !compiled->optimized) { return {}; }
  if (!current_view(revision)) {
    invalidated = true;
    return {};
  }
  binding.table.insert(state.pc, compiled->entry);
  return PreparedRegion{.entry = compiled->entry, .native_bytes = compiled->native_bytes};
}

PreparationReport Jit::prepare(const std::span<const Word> established_entries,
                               const Registers &state,
                               const std::optional<std::uint64_t> revision) {
  auto &self = *impl_;
  self.clear_invalidated();
  const auto graph = preparation_graph(self.program, self.context.runtime, established_entries);
  PreparationReport report{.failure = graph.failure};
  std::set<Word> unresolved(graph.unresolved.begin(), graph.unresolved.end());
  std::set<std::uint64_t> visited;
  std::set<std::uint64_t> installed;
  const auto install = [&](const Registers &context, const bool complete_loop) {
    if (!visited.insert(translation_key(context)).second) { return; }
    const auto entry = self.prepare_entry(context, revision, complete_loop);
    if (!entry) {
      unresolved.insert(context.pc);
      return;
    }
    if (installed.insert(region_key(context)).second) { ++report.prepared_entries; }
    report.native_bytes += entry->native_bytes;
  };
  if (!self.current_view(revision)) {
    report.failure = {.kind = FaultKind::host_event,
                      .space = Space::program,
                      .address = state.pc,
                      .pc = state.pc};
    return report;
  }
  install(state, whole_loop_entry(state));
  for (const auto pc : graph.entries) {
    auto context = state;
    context.pc = pc;
    context.sr &= ~(loop_flag | forever_flag);
    install(context, false);
  }
  for (const auto &operation : graph.instructions) {
    const auto kind = operation.instruction.operation;
    if (kind != Operation::loop_immediate && kind != Operation::loop_register &&
        kind != Operation::loop_short && kind != Operation::loop_effective &&
        kind != Operation::loop_forever) {
      continue;
    }
    const auto &decoded = operation.instruction;
    const bool relative =
        kind == Operation::loop_forever ? (decoded.opcode & 1U) == 0 : (decoded.opcode & 16U) != 0;
    auto context = state;
    context.pc = (operation.pc + decoded.words) & word_mask;
    context.la = relative ? (operation.pc + decoded.extension) & word_mask : decoded.extension;
    context.sr = (context.sr & ~(loop_flag | forever_flag)) | loop_flag;
    install(context, true);
  }
  report.unresolved_entries = unresolved.size();
  if (!self.current_view(revision)) {
    self.invalidated = true;
    report.prepared_entries = 0;
    report.failure = {.kind = FaultKind::host_event,
                      .space = Space::program,
                      .address = state.pc,
                      .pc = state.pc};
  }
  return report;
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

std::uint32_t Jit::run(std::uint32_t budget) {
  auto &self = *impl_;
  if (budget == 0) { return 0; }
  const auto *entry = self.prepare(budget);
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
  self.clear_invalidated();
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
  self.clear_invalidated();
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
  self.clear_invalidated();
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
