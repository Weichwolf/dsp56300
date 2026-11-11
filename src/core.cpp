// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/memory.hpp"
#include "execution_guard.hpp"
#include "instruction.hpp"
#include "io.hpp"
#include "jit.hpp"

#include <algorithm>
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
  DeadlineScope(std::uint64_t &limit, detail::Jit &jit, const std::uint64_t deadline)
      : limit_(limit), jit_(jit) {
    limit_ = deadline;
    jit_.set_cycle_deadline(deadline);
  }

  ~DeadlineScope() {
    limit_ = std::numeric_limits<std::uint64_t>::max();
    jit_.set_cycle_deadline(limit_);
  }

  DeadlineScope(const DeadlineScope &) = delete;
  DeadlineScope &operator=(const DeadlineScope &) = delete;

private:
  std::uint64_t &limit_;
  detail::Jit &jit_;
};

Buffers bind_memory(const Memory memory, Io *const io) {
  if (memory.storage().program.empty() && io == nullptr) {
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
  if (!detail::Jit::available()) {
    throw std::invalid_argument("Native translation requires macOS ARM64 or Linux x86-64");
  }
  return buffers;
}
}

Core::Impl::Impl(const Buffers supplied, Io *const host_io, const bool external_memory)
    : io(host_io),
      buffers(validate(supplied, external_memory)),
      external(external_memory),
      code_pages(external_memory ? std::make_unique<CodePages>() : nullptr) {
  aliases_program[1] = overlaps_program(buffers.x);
  aliases_program[2] = overlaps_program(buffers.y);

  jit = std::make_unique<detail::Jit>(
      cpu,
      buffers.program,
      detail::JitRuntime{.owner = this,
                         .latched = &latched,
#if !defined(__aarch64__)
                         .execute = jit_execute,
                         .moves = jit_moves,
#endif
                         .program = external ? jit_program : nullptr,
                         .memory = {{{.data = buffers.x.data(),
                                      .words = buffers.x.size(),
                                      .aliases_program = aliases_program[1]},
                                     {.data = buffers.y.data(),
                                      .words = buffers.y.size(),
                                      .aliases_program = aliases_program[2]}}},
                         .cacheable_program = buffers.cache.begin < buffers.cache.end},
      detail::NativeBus{
          .program = {.data = buffers.program.data(), .words = buffers.program.size()},
          .cache = buffers.cache,
          .owner = this,
          .access = jit_access,
          .failure = &fault,
          .external = external},
      detail::NativeEvents{.vector = &vector,
                           .debug_vector = &debug_vector,
                           .io = io,
                           .cache_owner = this,
                           .cache = jit_cache});
}

bool Core::Impl::overlaps_program(const std::span<const Word> memory) const {
  const auto program = reinterpret_cast<std::uintptr_t>(buffers.program.data());
  const auto other = reinterpret_cast<std::uintptr_t>(memory.data());
  return other < program ? program - other < memory.size_bytes()
                         : other - program < buffers.program.size_bytes();
}

void Core::Impl::invalidate_program_word(const Word *const location, const bool installed) {
  const auto program = reinterpret_cast<std::uintptr_t>(buffers.program.data());
  const auto address = reinterpret_cast<std::uintptr_t>(location);
  if (address < program || address - program >= buffers.program.size_bytes()) { return; }
  image_word_written({.address = static_cast<Word>((address - program) / sizeof(Word)),
                      .value = *location & word_mask});
  if (!installed && jit) { jit->invalidate(); }
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

  if (external) { return read_external(space, address); }
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
  const bool installed = image != nullptr;
  if (external) {
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
    invalidate_program_word(&memory[address], installed);
  }
}

detail::Instruction Core::Impl::instruction(const Word pc) {
  auto decoded = detail::decode(fetch_program(pc));
  if (decoded.words == 2) { decoded.extension = fetch_program((pc + 1) & word_mask); }
  return decoded;
}

void Core::Impl::step() {
  if (debug_ready()) { admit_debug(); }
  if (exception_ready()) {
    step_vector();
    return;
  }
  current_pc = cpu.reg.pc;
  const auto decoded =
      cpu.repeat_remaining != 0 && cpu.repeat_loaded ? repeated : instruction(current_pc);
  if (fault.kind != FaultKind::none) { return; }
  step_decoded(decoded);
}

void Core::Impl::step_decoded(const detail::Instruction &decoded) {
#if defined(__aarch64__)
  step_native(decoded);
#else
  step_semantic(decoded);
#endif
}

Core::Core(const Memory memory, Io *const io)
    : impl_(std::make_unique<Impl>(bind_memory(memory, io), io, memory.storage().program.empty())) {
  reset();
}

State Core::state() const {
  return {.registers = registers(),
          .failure = fault(),
          .execution = processing_state(),
          .progress = progress()};
}

void Core::load(const Space space, const Word address, const std::span<const Word> words) {
  const bool installed = impl_->image != nullptr;
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
  if (space == Space::program) {
    clear_code_cache();
  } else if (impl_->aliases_program[static_cast<std::size_t>(space)]) {
    for (std::size_t index = 0; index < words.size(); ++index) {
      impl_->invalidate_program_word(&memory[address + index], installed);
    }
  }
}

RunResult Core::step() {
  return run(1);
}

Core::~Core() = default;

void Core::reset() {
  if (impl_->executing) { throw std::logic_error("Reset requires stopped DSP execution"); }

  impl_->cpu = {};
  impl_->vector = {};
  impl_->next_image = nullptr;
  impl_->debug_vector = 0;
  impl_->cpu.processing = ExecutionState::running;
  impl_->icache.reset();
  impl_->cpu.reg.m.fill(word_mask);
  impl_->fault = {};
  clear_code_cache();
  if (impl_->jit) { impl_->jit->reset(); }
}

Progress Core::progress() const noexcept {
  return {.instructions = impl_->cpu.instructions, .estimated_cycles = impl_->cpu.cycles};
}

PreparationReport Core::prepare(const std::span<const Word> established_entries) {
  const auto pc = impl_->cpu.reg.pc;
  if (impl_->executing) {
    return {.failure = {
                .kind = FaultKind::host_event, .space = Space::program, .address = pc, .pc = pc}};
  }
  const detail::ExecutionGuard guard(impl_->executing);
  if (impl_->image) {
    PreparationReport report{.prepared_entries = impl_->active_image->instructions.size(),
                             .native_bytes = impl_->image->report.native_bytes,
                             .failure = {}};
    for (const auto entry : established_entries) {
      if (!impl_->image_ready || !impl_->active_image->instructions.contains(entry)) {
        ++report.unresolved_entries;
        report.failure = {.kind = FaultKind::unprepared_execution,
                          .space = Space::program,
                          .address = entry,
                          .pc = pc};
      }
    }
    return report;
  }
  const auto revision =
      impl_->external ? std::optional(dsp56300_io_revision(*impl_->io)) : std::nullopt;
  if (revision && (!impl_->windows_bound || *revision != impl_->program_revision ||
                   impl_->program_omr != impl_->cpu.reg.omr)) {
    if (const auto error = impl_->bind_memory_windows(); error.kind != FaultKind::none) {
      return {.failure = error};
    }
    impl_->jit->invalidate();
    impl_->program_revision = *revision;
    impl_->program_omr = impl_->cpu.reg.omr;
  }
  return impl_->jit->prepare(established_entries, impl_->cpu.reg, revision);
}

Fault Core::fault() const noexcept {
  return impl_->fault;
}

RunResult Core::run(const std::uint32_t instruction_budget) {

  const detail::ExecutionGuard guard(impl_->executing);
  impl_->yield_requested = false;
  if (impl_->image) { return impl_->run_image(instruction_budget); }
  RunResult result;

  impl_->synchronize_program_view();
  impl_->yield_requested = false;
  while (result.retired < instruction_budget && impl_->cpu.cycles < impl_->cycle_deadline &&
         !impl_->yield_requested && fault().kind == FaultKind::none &&
         impl_->cpu.processing == ExecutionState::running) {
    const auto previous_status = impl_->cpu.reg.sr;
    if (impl_->vector.phase == Impl::VectorPhase::none && !impl_->instruction_cache_active() &&
        (impl_->cpu.debug_flags & detail::debug_interrupt_pending) == 0) {
      const auto remaining = instruction_budget - result.retired;
      const auto retired = impl_->cpu.repeat_remaining != 0 ? impl_->run_repeat_native(remaining)
                                                            : impl_->jit->run(remaining);
      result.retired += retired;
#if defined(__aarch64__)
      impl_->synchronize_cache_mode();
      impl_->synchronize_program_view();
      impl_->check_interrupt_window(previous_status);
#endif
      if (retired != 0 || fault().kind != FaultKind::none ||
          impl_->cpu.cycles >= impl_->cycle_deadline) {
        continue;
      }
    }
    impl_->step();
    impl_->synchronize_program_view();
    impl_->check_interrupt_window(previous_status);
    if (fault().kind == FaultKind::none) { ++result.retired; }
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
  const DeadlineScope scope(impl_->cycle_deadline, *impl_->jit, base_cycle_deadline);
  return run(std::numeric_limits<std::uint32_t>::max());
}

bool Core::interrupt_ready() const {
  if (impl_->next_image != nullptr) { return false; }
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
    const auto known = Impl::jit_program(impl_.get(), pc);
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

void Core::clear_code_cache() {
  impl_->windows_bound = false;
  if (impl_->code_pages) { impl_->code_pages->reset(); }
  if (impl_->jit) { impl_->jit->invalidate(); }
}
} // namespace dsp56300
