// SPDX-License-Identifier: GPL-3.0-only
#include "jit.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit_emitter.hpp"
#include "mac_batch.hpp"
#include "translation.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <sys/mman.h>
#include <system_error>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dsp56300::detail {
namespace {

// A SysV tail bridge enters untyped generated code without pretending it has
// Clang's compiler-emitted function-type metadata. No sanitizer is disabled.
#if defined(__x86_64__) && defined(__linux__)
extern "C" std::uint32_t dsp56300_enter_generated(const void *, Cpu *, NativeContext *) noexcept;
asm(".pushsection .text\n"
    ".globl dsp56300_enter_generated\n"
    ".hidden dsp56300_enter_generated\n"
    ".type dsp56300_enter_generated,@function\n"
    "dsp56300_enter_generated:\n"
    "endbr64\n"
    "mov %rdi,%rax\n"
    "mov %rsi,%rdi\n"
    "mov %rdx,%rsi\n"
    "jmp *%rax\n"
    ".size dsp56300_enter_generated,.-dsp56300_enter_generated\n"
    ".popsection\n");
#endif

std::size_t aligned_size(const std::size_t size, const std::size_t alignment) {
  const auto padding = (alignment - (size % alignment)) % alignment;
  if (size > std::numeric_limits<std::size_t>::max() - padding) {
    throw std::length_error("Native code size overflow");
  }
  return size + padding;
}

class Executable {
public:
  explicit Executable(const std::span<const std::uint8_t> code,
                      const std::span<const std::uint8_t> precise = {}) {
    const auto page = sysconf(_SC_PAGESIZE);
    if (page <= 0) { throw std::runtime_error("Cannot determine executable page size"); }
    const auto page_size = static_cast<std::size_t>(page);
    constexpr std::size_t entry_alignment = 16;
    const auto offset = precise.empty() ? 0 : aligned_size(code.size(), entry_alignment);
    if (precise.size() > std::numeric_limits<std::size_t>::max() - offset) {
      throw std::length_error("Native code size overflow");
    }
    size_ = aligned_size(precise.empty() ? code.size() : offset + precise.size(), page_size);
    pointers_[0] = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (pointers_[0] == MAP_FAILED) {
      throw std::system_error(errno, std::generic_category(), "JIT mmap");
    }
    pointers_[1] = static_cast<std::byte *>(pointers_[0]) + offset;
    std::memcpy(pointers_[0], code.data(), code.size());
    if (!precise.empty()) { std::memcpy(pointers_[1], precise.data(), precise.size()); }
    if (mprotect(pointers_[0], size_, PROT_READ | PROT_EXEC) != 0) {
      const auto error = errno;
      munmap(pointers_[0], size_);
      throw std::system_error(error, std::generic_category(), "JIT mprotect");
    }
  }

  ~Executable() { munmap(pointers_[0], size_); }

  Executable(const Executable &) = delete;
  Executable &operator=(const Executable &) = delete;

  [[nodiscard]] std::size_t size() const noexcept { return size_; }

  [[nodiscard]] std::uint32_t
  enter(Cpu &cpu, NativeContext &context, const bool precise = false) const noexcept {
#if defined(__x86_64__) && defined(__linux__)
    return dsp56300_enter_generated(pointers_[precise ? 1U : 0U], &cpu, &context);
#else
    static_cast<void>(cpu);
    static_cast<void>(context);
    static_cast<void>(precise);
    return 0;
#endif
  }

private:
  std::array<void *, 2> pointers_{};
  std::size_t size_ = 0;
};

bool terminates(const Operation operation) {
  return !repeatable(operation) || operation == Operation::conditional_transfer ||
         operation == Operation::illegal || operation == Operation::debug ||
         operation == Operation::conditional_debug;
}

struct Block {
  std::vector<NativeOp> operations;
  std::unique_ptr<Executable> code;
};
}

struct Jit::Impl {
  Impl(Cpu &state, const std::span<const Word> supplied, const JitRuntime runtime)
      : cpu(state),
        program(supplied),
        context{.runtime = runtime, .exit_requested = &exit_requested} {}

  [[nodiscard]] std::optional<Word> word(const Word address) const noexcept {
    if (context.runtime.program != nullptr) {
      return context.runtime.program(context.runtime.owner, address);
    }
    return address < program.size() ? std::optional(program[address] & word_mask) : std::nullopt;
  }

  [[nodiscard]] std::unique_ptr<Block> compile() const;
  [[nodiscard]] Block *retain(std::unique_ptr<Block> block);
  [[nodiscard]] Block *prepare();
  [[nodiscard]] std::uint32_t enter(const Block &block, Word budget);
  Cpu &cpu;
  std::span<const Word> program;
  // The dispatch index borrows entries; ownership survives replacement and reset.
  std::unordered_map<std::uint64_t, Block *> blocks;
  std::optional<NativeOp> rejected_repeat;
  bool invalidated = false;
  bool exit_requested = false;
  NativeContext context;
  std::vector<std::unique_ptr<Block>> storage;
};

std::unique_ptr<Block> Jit::Impl::compile() const {
  auto block = std::make_unique<Block>();
  auto pc = cpu.reg.pc;
  const auto mode = cpu.reg.sr & numeric_mode_mask;
  const auto loop_address = active_loop_address(cpu.reg);
  while (true) {
    const auto opcode = word(pc);
    if (!opcode) { break; }
    auto decoded = decode(*opcode);
    if (decoded.words == 2) {
      const auto following = word((pc + 1) & word_mask);
      if (!following) { break; }
      decoded.extension = *following;
    }
    block->operations.push_back(
        {.instruction = decoded, .pc = pc, .mode = mode, .loop_address = loop_address});
    if (terminates(decoded.operation) || at_loop_end(block->operations.back())) { break; }
    if (context.runtime.program == nullptr && decoded.words >= program.size() - pc) { break; }
    const auto next = (pc + decoded.words) & word_mask;
    if (next <= pc) { break; }
    pc = next;
  }
  if (block->operations.empty()) { return nullptr; }
  JitEmitter emitter(context.runtime, cpu.reg.m);
  const auto bytes = emitter.generate(block->operations);
  std::vector<std::uint8_t> checked;
  if (block->operations.size() > 1) {
    JitEmitter precise(context.runtime, cpu.reg.m);
    checked = precise.generate(block->operations, ExecutionBounds::precise);
  }
  block->code = std::make_unique<Executable>(bytes, checked);
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
  if (!impl_->blocks.empty() && !impl_->invalidated) {

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
  impl_->rejected_repeat.reset();
  impl_->invalidated = false;
  impl_->exit_requested = false;
}

Block *Jit::Impl::retain(std::unique_ptr<Block> block) {
  auto *const entry = block.get();
  storage.push_back(std::move(block));
  return entry;
}

Block *Jit::Impl::prepare() {
  if (invalidated) {
    blocks.clear();

    invalidated = false;
  }
  exit_requested = false;
  const auto key = translation_key(cpu.reg);
  const auto found = blocks.find(key);
  if (found != blocks.end()) { return found->second; }
  auto block = compile();
  if (!block) { return nullptr; }
  auto *const entry = retain(std::move(block));
  blocks.emplace(key, entry);
  return entry;
}

std::uint32_t Jit::Impl::enter(const Block &block, const Word budget) {
  context.budget = budget;
  return block.code->enter(cpu, context, budget < block.operations.size());
}

std::uint32_t Jit::run(const std::uint32_t budget) {
  auto &self = *impl_;
  if (budget == 0) { return 0; }
  const auto *const block = self.prepare();
  return block != nullptr ? self.enter(*block, budget) : 0;
}

std::uint32_t Jit::run_repeat(const NativeOp operation, const std::uint32_t budget) {
  auto &self = *impl_;
  if (budget == 0 || self.cpu.repeat_remaining == 0) { return 0; }
  if (self.invalidated) {
    self.blocks.clear();

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
    JitEmitter emitter(self.context.runtime, self.cpu.reg.m);
    const auto bytes = emitter.generate_repeat(block->operations.front());
    if (bytes.empty()) {
      self.rejected_repeat = operation;
      return 0;
    }
    block->code = std::make_unique<Executable>(bytes);

    self.blocks.insert_or_assign(key, self.retain(std::move(block)));

    found = self.blocks.find(key);
  }
  return self.enter(*found->second, budget);
}

std::uint32_t Jit::run_captured(NativeOp operation) {
  auto &self = *impl_;
  if (self.cpu.repeat_remaining != 0) {
    // The final REP at a DO endpoint still needs the genuine frame cleanup.
    if (at_loop_end(operation) && self.cpu.repeat_remaining == 1) { return 0; }
    operation.loop_address = inactive_loop;
    return run_repeat(operation, 1);
  }
  if (self.invalidated) {
    self.blocks.clear();

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
    JitEmitter emitter(self.context.runtime, self.cpu.reg.m);
    const auto bytes = emitter.generate(block->operations);
    block->code = std::make_unique<Executable>(bytes);

    self.blocks.insert_or_assign(key, self.retain(std::move(block)));

    found = self.blocks.find(key);
  }
  return self.enter(*found->second, 1);
}

} // namespace dsp56300::detail
