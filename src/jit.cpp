// SPDX-License-Identifier: GPL-3.0-only
#include "jit.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "jit_emitter.hpp"
#include "mac_batch.hpp"
#include "translation.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

class Executable {
public:
  explicit Executable(const std::span<const std::uint8_t> code) {
    const auto page = sysconf(_SC_PAGESIZE);
    if (page <= 0) { throw std::runtime_error("Cannot determine executable page size"); }
    const auto page_size = static_cast<std::size_t>(page);
    size_ = ((code.size() + page_size - 1) / page_size) * page_size;
    pointer_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (pointer_ == MAP_FAILED) {
      throw std::system_error(errno, std::generic_category(), "JIT mmap");
    }
    std::memcpy(pointer_, code.data(), code.size());
    if (mprotect(pointer_, size_, PROT_READ | PROT_EXEC) != 0) {
      const auto error = errno;
      munmap(pointer_, size_);
      throw std::system_error(error, std::generic_category(), "JIT mprotect");
    }
  }

  ~Executable() { munmap(pointer_, size_); }

  Executable(const Executable &) = delete;
  Executable &operator=(const Executable &) = delete;

  [[nodiscard]] std::size_t size() const noexcept { return size_; }

  [[nodiscard]] std::uint32_t enter(Cpu &cpu, NativeContext &context) const noexcept {
#if defined(__x86_64__) && defined(__linux__)
    return dsp56300_enter_generated(pointer_, &cpu, &context);
#else
    static_cast<void>(cpu);
    static_cast<void>(context);
    return 0;
#endif
  }

private:
  void *pointer_ = nullptr;
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
  std::vector<std::unique_ptr<Block>> tails;
  Word requested_limit = 0;
  bool terminated = false;
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

  [[nodiscard]] std::unique_ptr<Block> compile(Word block_limit) const;
  [[nodiscard]] Block *prepare(Word budget);
  [[nodiscard]] Block *prefix(Block &block, Word instructions) const;
  [[nodiscard]] std::uint32_t enter(const Block &block, Word budget);
  Cpu &cpu;
  std::span<const Word> program;
  std::unordered_map<std::uint64_t, std::unique_ptr<Block>> blocks;
  std::optional<NativeOp> rejected_repeat;
  bool invalidated = false;
  bool exit_requested = false;
  NativeContext context;
};

std::unique_ptr<Block> Jit::Impl::compile(const Word block_limit) const {
  auto block = std::make_unique<Block>();
  block->requested_limit = block_limit;
  auto pc = cpu.reg.pc;
  const auto mode = cpu.reg.sr & numeric_mode_mask;
  const auto loop_address = active_loop_address(cpu.reg);
  for (Word index = 0; index < block_limit; ++index) {
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
    if (terminates(decoded.operation) || at_loop_end(block->operations.back())) {
      block->terminated = true;
      break;
    }
    if (context.runtime.program == nullptr && decoded.words >= program.size() - pc) {
      block->terminated = true;
      break;
    }
    pc = (pc + decoded.words) & word_mask;
  }
  if (block->operations.empty()) { return nullptr; }
  JitEmitter emitter(context.runtime, cpu.reg.m);
  const auto bytes = emitter.generate(block->operations);
  block->code = std::make_unique<Executable>(bytes);

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
  // Defer destruction until generated code and all embedded metadata have returned.
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

Block *Jit::Impl::prepare(const Word budget) {
  if (invalidated) {
    blocks.clear();

    invalidated = false;
  }
  exit_requested = false;
  const auto key = translation_key(cpu.reg);
  auto found = blocks.find(key);
  constexpr auto translation_limit = static_cast<Word>(address_words);
  const auto compile_limit = std::min(budget, translation_limit);
  if (found == blocks.end() ||
      (!found->second->terminated && found->second->requested_limit < compile_limit)) {
    auto block = compile(compile_limit);
    if (!block) { return nullptr; }

    blocks.insert_or_assign(key, std::move(block));

    found = blocks.find(key);
  }
  return found->second.get();
}

Block *Jit::Impl::prefix(Block &block, const Word instructions) const {
  if (instructions >= block.operations.size()) { return &block; }
  for (const auto &tail : block.tails) {
    if (tail->requested_limit == instructions) { return tail.get(); }
  }
  auto tail = compile(instructions);
  if (!tail) { return nullptr; }

  block.tails.push_back(std::move(tail));
  return block.tails.back().get();
}

std::uint32_t Jit::Impl::enter(const Block &block, const Word budget) {
  context.budget = budget;
  const auto retired = block.code->enter(cpu, context);

  return retired;
}

std::uint32_t Jit::run(const std::uint32_t budget) {
  auto &self = *impl_;
  auto *block = self.prepare(budget);
  if (block == nullptr) { return 0; }
  block = self.prefix(*block, budget);
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

    self.blocks.insert_or_assign(key, std::move(block));

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

    self.blocks.insert_or_assign(key, std::move(block));

    found = self.blocks.find(key);
  }
  return self.enter(*found->second, 1);
}

} // namespace dsp56300::detail
