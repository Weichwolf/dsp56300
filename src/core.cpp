// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"
#include "io.hpp"
#include "memory_views.hpp"
#include "native_call.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>

namespace dsp56300 {

namespace {

class DeadlineScope {
public:
  DeadlineScope(std::uint64_t &limit, const std::uint64_t deadline) : limit_(limit) {
    limit_ = deadline;
  }

  ~DeadlineScope() { limit_ = std::numeric_limits<std::uint64_t>::max(); }

  DeadlineScope(const DeadlineScope &) = delete;
  DeadlineScope &operator=(const DeadlineScope &) = delete;

private:
  std::uint64_t &limit_;
};

Buffers bind_memory(const Memory memory, Io *const io, const bool complete) {
  if (memory.storage().program.empty() && io == nullptr && !complete) {
    throw std::invalid_argument("External DSP memory requires an I/O owner");
  }
  return memory.storage();
}

Buffers validate(const Buffers buffers, const bool external) {
  if (!external && (buffers.program.empty() || buffers.x.empty() || buffers.y.empty() ||
                    buffers.program.size() > address_words || buffers.x.size() > address_words ||
                    buffers.y.size() > address_words)) {
    throw std::invalid_argument("DSP requires nonempty memory spans of at most 2^24 words");
  }
  if (buffers.cache.begin > buffers.cache.end || buffers.cache.end > address_words ||
      (buffers.cache.sector_words != 128 && buffers.cache.sector_words != 256)) {
    throw std::invalid_argument("Invalid DSP instruction cache interval or sector size");
  }
  return buffers;
}
}

Core::Impl::Impl(const Buffers supplied,
                 Io *const host_io,
                 const bool external_memory,
                 const std::array<std::span<const MemoryWindow>, 3> region_views,
                 const std::array<Memory::Views, 3> address_views)
    : io(host_io),
      regions(region_views),
      buffers(validate(supplied, external_memory)),
      views(address_views),
      external(external_memory) {
  aliases_program[1] = overlaps_program(buffers.x);
  aliases_program[2] = overlaps_program(buffers.y);
}

bool Core::Impl::overlaps_program(const std::span<const Word> memory) const {
  const auto program = reinterpret_cast<std::uintptr_t>(buffers.program.data());
  const auto other = reinterpret_cast<std::uintptr_t>(memory.data());
  return other < program ? program - other < memory.size_bytes()
                         : other - program < buffers.program.size_bytes();
}

void Core::Impl::invalidate_program_word(const Word *const location) {
  const auto program = reinterpret_cast<std::uintptr_t>(buffers.program.data());
  const auto address = reinterpret_cast<std::uintptr_t>(location);
  if (address < program || address - program >= buffers.program.size_bytes()) { return; }
  image_word_written({.address = static_cast<Word>((address - program) / sizeof(Word)),
                      .value = *location & word_mask});
}

std::span<Word> Core::Impl::area(const Space space) const {
  switch (space) {
    case Space::program: return buffers.program;
    case Space::x: return buffers.x;
    case Space::y: return buffers.y;
    default: throw std::invalid_argument("Invalid DSP memory space");
  }
}

void Core::Impl::fail(const FaultKind kind,
                      const Space space,
                      const Word address,
                      const Word value) {
  if (fault.kind == FaultKind::none) {
    fault = {.kind = kind, .space = space, .address = address, .value = value, .pc = current_pc};
  }
}

Word Core::Impl::read_memory(const Space space, const Word address) {
  if (detail::dsp56300_memory_direct()) {
    if (space == Space::program || address < detail::io_begin) {
      return static_cast<const volatile Word *>(
                 views[static_cast<std::size_t>(space)].read.data.data())[address] &
             word_mask;
    }
  } else if (external) {
    return read_external(space, address);
  }
  if (space != Space::program && address >= detail::io_begin) {
    Word value = 0;
    if (io == nullptr || !dsp56300_io_read(*io, space, address, current_pc, value)) {
      fail(FaultKind::io_read, space, address);
    }
    image_io_completed();
    return value & word_mask;
  }
  const auto memory = area(space);
  if (address >= memory.size()) {
    fail(FaultKind::memory_read, space, address);
    return 0;
  }
  if (space == Space::program && cacheable(address) && icache) {
    if (const auto cached = icache->peek(address)) { return *cached; }
  }
  return memory[address] & word_mask;
}

void Core::Impl::write_memory(const Space space, const Word address, const Word value) {
  if (detail::dsp56300_memory_direct()) {
    // Undeclared P stores retain the consumer's loader/code-change effects.
    if (space == Space::program && io != nullptr && memory_window(space, address) == nullptr) {
      write_external(space, address, value);
      return;
    }
    if (space == Space::program || address < detail::io_begin) {
      static_cast<volatile Word *>(
          views[static_cast<std::size_t>(space)].write.data.data())[address] = value & word_mask;
      if (space == Space::program) {
        image_word_written({.address = address, .value = value & word_mask});
      }
      return;
    }
  } else if (external) {
    write_external(space, address, value);
    return;
  }
  if (space != Space::program && address >= detail::io_begin) {
    if (io == nullptr || !dsp56300_io_write(*io, space, address, current_pc, value & word_mask)) {
      fail(FaultKind::io_write, space, address, value);
    }
    image_io_completed();
    return;
  }
  const auto memory = area(space);
  if (address >= memory.size()) {
    fail(FaultKind::memory_write, space, address, value);
    return;
  }
  memory[address] = value & word_mask;
  if (space == Space::program && cacheable(address) && icache) {
    icache->write({.address = address, .value = value});
  }
  if (aliases_program[static_cast<std::size_t>(space)]) {
    invalidate_program_word(&memory[address]);
  }
}

Core::Core(const Memory memory, Io *const io)
    : impl_(std::make_unique<Impl>(bind_memory(memory, io, !memory.views_[0].read.data.empty()),
                                   io,
                                   memory.storage().program.empty(),
                                   memory.regions_,
                                   memory.views_)) {
  reset();
}

void Core::set_memory(const Memory memory) {
  auto &self = *impl_;
  if (self.executing || self.terminal_fault) {
    throw std::logic_error("Memory binding requires a usable stopped DSP owner");
  }
  const Impl replacement(bind_memory(memory, self.io, !memory.views_[0].read.data.empty()),
                         self.io,
                         memory.storage().program.empty(),
                         memory.regions_,
                         memory.views_);
  self.buffers = replacement.buffers;
  self.regions = replacement.regions;
  self.views = replacement.views;
  self.external = replacement.external;
  self.aliases_program = replacement.aliases_program;
  self.native_call = std::unique_ptr<detail::NativeCall>{};
  self.image.reset();
  self.active_image = nullptr;
  self.next_images = nullptr;
  self.image_ready = false;
  self.image_storage_admitted = false;
  self.image_windows = {};
  self.icache.reset();
}

State Core::state() const {
  return {.registers = registers(),
          .failure = fault(),
          .execution = processing_state(),
          .progress = progress()};
}

void Core::load(const Space space, const Word address, const std::span<const Word> words) {
  if (impl_->complete_memory()) {
    impl_->load_views(space, address, words);
    return;
  }
  const auto memory = impl_->area(space);
  if (address > memory.size() || words.size() > memory.size() - address) {
    throw std::out_of_range("DSP host load outside memory span");
  }
  if (std::ranges::any_of(words, [](const Word word) { return word > word_mask; })) {
    throw std::invalid_argument("DSP memory values must fit in 24 bits");
  }
  for (std::size_t index = 0; index < words.size(); ++index) {
    memory[address + index] = words[index];
    if (space == Space::program) {
      impl_->image_word_written(
          {.address = address + static_cast<Word>(index), .value = words[index]});
    }
  }
  if (space != Space::program && impl_->aliases_program[static_cast<std::size_t>(space)]) {
    for (std::size_t index = 0; index < words.size(); ++index) {
      impl_->invalidate_program_word(&memory[address + index]);
    }
  }
}

Core::~Core() = default;

void Core::reset() {
  if (impl_->executing || impl_->terminal_fault) {
    throw std::logic_error("Reset requires a usable stopped DSP owner");
  }

  impl_->cpu = {};
  impl_->vector = {};
  impl_->next_images = nullptr;
  impl_->debug_vector = 0;
  impl_->cpu.processing = ExecutionState::running;
  impl_->icache.reset();
  impl_->cpu.reg.m.fill(word_mask);
  impl_->fault = {};
}

Progress Core::progress() const noexcept {
  return {.instructions = impl_->cpu.instructions, .estimated_cycles = impl_->cpu.cycles};
}

Fault Core::fault() const noexcept {
  return impl_->fault;
}

RunResult Core::run(const std::uint32_t instruction_budget) {

  const detail::ExecutionGuard guard(impl_->executing);
  impl_->yield_requested = false;
  if (impl_->image) { return impl_->run_image(instruction_budget); }
  RunResult result;

  if (instruction_budget != 0 && impl_->cpu.cycles < impl_->cycle_deadline &&
      fault().kind == FaultKind::none && impl_->cpu.processing == ExecutionState::running) {
    impl_->unprepared(impl_->cpu.reg.pc);
  }
  result.progress = progress();
  result.fault = fault();
  result.state = processing_state();
  result.yielded = impl_->yield_requested;
  return result;
}

RunResult Core::run_until(const std::uint64_t base_cycle_deadline) {
  if (impl_->executing) { throw std::logic_error("Recursive DSP execution is not permitted"); }
  const auto bound = detail::instruction_cycle_bound();
  if (base_cycle_deadline > std::numeric_limits<std::uint64_t>::max() - (bound - 1U)) {
    throw std::overflow_error("DSP deadline leaves no representable instruction boundary");
  }
  if (impl_->cpu.cycles >= base_cycle_deadline) {
    return {.state = processing_state(), .progress = progress(), .fault = fault()};
  }
  const DeadlineScope scope(impl_->cycle_deadline, base_cycle_deadline);
  return run(std::numeric_limits<std::uint32_t>::max());
}

bool Core::interrupt_ready() const {
  if (impl_->next_images != nullptr) { return false; }
  if ((impl_->cpu.processing != ExecutionState::running &&
       impl_->cpu.processing != ExecutionState::waiting) ||
      (impl_->cpu.debug_flags & detail::debug_interrupt_pending) != 0 ||
      fault().kind != FaultKind::none || impl_->vector.phase != Impl::VectorPhase::none ||
      impl_->cpu.repeat_remaining != 0 || impl_->cpu.interrupt_inhibited) {
    return false;
  }
  if (!impl_->cpu.repeat_completed) { return true; }
  const auto pc = impl_->cpu.reg.pc;
  Word opcode = 0;
  if (impl_->external) {
    const auto known = impl_->known_program_word(pc);
    if (!known) {
      throw std::logic_error("Interrupt eligibility requires known next-instruction metadata");
    }
    opcode = *known;
  } else {
    if (pc >= impl_->buffers.program.size()) { return false; }
    opcode = impl_->buffers.program[pc] & word_mask;
  }
  if (impl_->cacheable(pc) && impl_->icache) {
    if (const auto cached = impl_->icache->peek(pc)) { opcode = *cached; }
  }
  if (impl_->image) {
    const auto found = impl_->active_image->instructions.find(pc);
    if (found == impl_->active_image->instructions.end() || found->second.opcode != opcode) {
      throw std::logic_error("IRQ eligibility has no matching prepared DSP instruction");
    }
    return !detail::is_repeat(found->second.operation);
  }
  return !detail::is_repeat(detail::decode(opcode).operation);
}

} // namespace dsp56300
