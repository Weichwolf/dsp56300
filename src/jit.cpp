// SPDX-License-Identifier: GPL-3.0-only
#include "jit.hpp"

#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "instruction_boundary.hpp"
#include "jit_emitter.hpp"
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
constexpr std::size_t cache_limit = std::size_t{8} << 20;

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
  std::vector<Word> prefix_cycles{0};
  std::vector<std::unique_ptr<Block>> tails;
  std::size_t storage_bytes = 0;
  Word requested_limit = 0, pure = 0;
  bool terminated = false;
};
}

struct Jit::Impl {
  Impl(Cpu &state,
       const std::span<const Word> supplied,
       const Options settings,
       const JitRuntime runtime)
      : cpu(state),
        program(supplied),
        options(settings),
        context{.runtime = runtime,
                .exit_requested = &exit_requested,
                .native_moves = &statistics.native_move_instructions,
                .batch_instructions = &statistics.batch_instructions,
                .avx2_instructions = &statistics.avx2_instructions,
                .batch_calls = &statistics.batch_calls} {}

  [[nodiscard]] std::optional<Word> word(const Word address) const noexcept {
    if (context.runtime.program != nullptr) {
      return context.runtime.program(context.runtime.owner, address);
    }
    return address < program.size() ? std::optional(program[address] & word_mask) : std::nullopt;
  }

  [[nodiscard]] std::unique_ptr<Block> compile(Word block_limit) const;
  [[nodiscard]] Block *prepare(Word budget);
  [[nodiscard]] Block *prefix(Block &block, Word instructions);
  void retain_current(Block &block);
  [[nodiscard]] std::uint32_t enter(const Block &block, Word budget);
  Cpu &cpu;
  std::span<const Word> program;
  Options options;
  std::unordered_map<std::uint64_t, std::unique_ptr<Block>> blocks;
  JitStatistics statistics;
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
  JitEmitter emitter(context.runtime, options.optimize, options.batch_backend, cpu.reg.m);
  const auto bytes = emitter.generate(block->operations);
  if (bytes.size() > cache_limit / 2) { return nullptr; }
  block->code = std::make_unique<Executable>(bytes);
  block->storage_bytes = block->code->size();
  for (std::size_t index = 0; index < block->operations.size(); ++index) {
    const auto &instruction = block->operations[index].instruction;
    block->prefix_cycles.push_back(block->prefix_cycles.back() + instruction.cycles);
    if (block->pure == index &&
        (!requires_owner(instruction) || emitter.direct_memory(block->operations[index]))) {
      ++block->pure;
    }
  }
  return block;
}

Jit::Jit(Cpu &cpu,
         const std::span<const Word> program,
         const Options options,
         const JitRuntime runtime)
    : impl_(std::make_unique<Impl>(cpu, program, options, runtime)) {}

Jit::~Jit() = default;

bool Jit::available() noexcept {
#if defined(__x86_64__) && defined(__linux__)
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
  // Defer destruction until generated code and all embedded metadata have returned.
  if (!impl_->blocks.empty() && !impl_->invalidated) {
    ++impl_->statistics.invalidations;
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
  impl_->statistics = {};
  impl_->invalidated = false;
  impl_->exit_requested = false;
}

void Jit::helper_retired() noexcept {
  ++impl_->statistics.helper_instructions;
}

JitStatistics Jit::statistics() const noexcept {
  return impl_->statistics;
}

Block *Jit::Impl::prepare(const Word budget) {
  if (invalidated) {
    blocks.clear();
    statistics.executable_bytes = 0;
    invalidated = false;
  }
  exit_requested = false;
  const auto key = translation_key(cpu.reg);
  auto found = blocks.find(key);
  const auto compile_limit = std::min(budget, options.instructions_per_block);
  if (found == blocks.end() ||
      (!found->second->terminated && found->second->requested_limit < compile_limit)) {
    auto block = compile(compile_limit);
    if (!block) { return nullptr; }
    if (statistics.executable_bytes + block->storage_bytes > cache_limit) {
      blocks.clear();
      statistics.executable_bytes = 0;
      found = blocks.end();
    }
    if (found != blocks.end()) { statistics.executable_bytes -= found->second->storage_bytes; }
    statistics.executable_bytes += block->storage_bytes;
    blocks.insert_or_assign(key, std::move(block));
    ++statistics.compiled_blocks;
    found = blocks.find(key);
  }
  return found->second.get();
}

void Jit::Impl::retain_current(Block &block) {
  auto current = blocks.extract(translation_key(cpu.reg));
  blocks.clear();
  blocks.insert(std::move(current));
  block.tails.clear();
  block.storage_bytes = block.code->size();
  statistics.executable_bytes = block.storage_bytes;
}

Block *Jit::Impl::prefix(Block &block, const Word instructions) {
  if (instructions >= block.operations.size()) { return &block; }
  for (const auto &tail : block.tails) {
    if (tail->requested_limit == instructions) { return tail.get(); }
  }
  auto tail = compile(instructions);
  if (!tail) { return nullptr; }
  if (statistics.executable_bytes + tail->storage_bytes > cache_limit) { retain_current(block); }
  block.storage_bytes += tail->storage_bytes;
  statistics.executable_bytes += tail->storage_bytes;
  ++statistics.compiled_blocks;
  block.tails.push_back(std::move(tail));
  return block.tails.back().get();
}

std::uint32_t Jit::Impl::enter(const Block &block, const Word budget) {
  const auto helpers = statistics.helper_instructions;
  context.budget = budget;
  const auto retired = block.code->enter(cpu, context);
  statistics.native_alu_instructions += retired - (statistics.helper_instructions - helpers);
  return retired;
}

std::uint32_t Jit::run(const std::uint32_t budget) {
  auto &self = *impl_;
  auto *block = self.prepare(budget);
  if (block == nullptr) { return 0; }
  block = self.prefix(*block, budget);
  return block != nullptr ? self.enter(*block, budget) : 0;
}

std::uint32_t Jit::run_bounded(const std::uint32_t instructions, const std::uint64_t cycles) {
  auto &self = *impl_;
  auto *block = self.prepare(instructions);
  if (block == nullptr || block->pure == 0) { return 0; }
  const auto count = static_cast<Word>(block->operations.size());
  Word allowed = 0;
  auto remaining_cycles = cycles;
  if (block->pure == count && at_loop_end(block->operations.back())) {
    const auto body_cycles = block->prefix_cycles.back();
    if (body_cycles == 0) { return 0; }
    const auto repeats = std::min(std::uint64_t{instructions / count}, cycles / body_cycles);
    allowed = static_cast<Word>(repeats * count);
    remaining_cycles -= repeats * body_cycles;
  }
  const auto tail_limit = std::min(instructions - allowed, block->pure);
  Word tail = 0;
  while (tail < tail_limit && block->prefix_cycles[tail + 1] <= remaining_cycles) { ++tail; }
  allowed += tail;
  if (allowed == 0) { return 0; }
  block = self.prefix(*block, std::min(allowed, count));
  return block != nullptr ? self.enter(*block, allowed) : 0;
}
} // namespace dsp56300::detail
