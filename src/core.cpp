// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "instruction.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>

namespace dsp56300 {
namespace {
Buffers validate(const Buffers buffers, const Options options) {
  if (buffers.program.empty() || buffers.x.empty() || buffers.y.empty() ||
      buffers.program.size() > address_words || buffers.x.size() > address_words ||
      buffers.y.size() > address_words) {
    throw std::invalid_argument("DSP requires nonempty memory spans of at most 2^24 words");
  }
  if (options.backend == Backend::jit) {
    throw std::invalid_argument("The independent x64 JIT has not yet been implemented");
  }
  return buffers;
}
}

Core::Impl::Impl(const Buffers supplied, Io *const host_io, const Options settings)
    : io(host_io), buffers(validate(supplied, settings)), options(settings) {}

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
  if (space != Space::program && address >= detail::io_begin) {
    Word value = 0;
    if (io == nullptr || !io->read(space, address, current_pc, value)) {
      fail(FaultKind::io_read, space, address);
    }
    return value & word_mask;
  }
  const auto memory = area(space);
  if (address >= memory.size()) {
    fail(FaultKind::memory_read, space, address);
    return 0;
  }
  return memory[address] & word_mask;
}

void Core::Impl::write_memory(const Space space, const Word address, const Word value) {
  if (space != Space::program && address >= detail::io_begin) {
    if (io == nullptr || !io->write(space, address, current_pc, value & word_mask)) {
      fail(FaultKind::io_write, space, address, value);
    }
    return;
  }
  const auto memory = area(space);
  if (address >= memory.size()) {
    fail(FaultKind::memory_write, space, address, value);
    return;
  }
  memory[address] = value & word_mask;
  if (space == Space::program) {
    pages[address / page_words].reset();
    if (address != 0) { pages[(address - 1) / page_words].reset(); }
  }
}

detail::Instruction Core::Impl::instruction(const Word pc) {
  const auto opcode = read_memory(Space::program, pc);
  if (options.backend == Backend::cached && pages[pc / page_words]) {
    const auto &cached = (*pages[pc / page_words])[pc % page_words];
    if (cached.operation != detail::Operation::unsupported && cached.opcode == opcode &&
        (cached.words == 1 ||
         cached.extension == read_memory(Space::program, (pc + 1) & word_mask))) {
      return cached;
    }
  }
  auto decoded = detail::decode(opcode);
  if (decoded.words == 2) { decoded.extension = read_memory(Space::program, (pc + 1) & word_mask); }
  if (options.backend == Backend::reference) { return decoded; }
  auto &page = pages[pc / page_words];
  if (!page) { page = std::make_unique<Page>(); }
  auto &cached = (*page)[pc % page_words];
  if (cached != decoded) { cached = decoded; }
  return cached;
}

void Core::Impl::step() {
  current_pc = cpu.reg.pc;
  const auto decoded =
      cpu.repeat_remaining != 0 && cpu.repeat_loaded ? repeated : instruction(current_pc);
  if (fault.kind != FaultKind::none) { return; }
  const bool repeating = cpu.repeat_remaining != 0;
  if (repeating && (decoded.words != 1 || (decoded.operation != detail::Operation::parallel &&
                                           decoded.operation != detail::Operation::nop))) {
    fail(FaultKind::invalid_instruction, Space::program, current_pc, decoded.opcode);
    return;
  }
  current_words = decoded.words;
  cpu.interrupt_inhibited = false;
  cpu.reg.pc = (current_pc + decoded.words) & word_mask;
  execute(decoded);
  if (fault.kind != FaultKind::none) { return; }
  ++cpu.instructions;
  cpu.cycles += decoded.cycles;
  if (repeating) {
    cpu.repeat_opcode = decoded.opcode;
    repeated = decoded;
    cpu.repeat_loaded = true;
    --cpu.repeat_remaining;
    if (cpu.repeat_remaining != 0) {
      cpu.reg.lc = cpu.repeat_remaining;
      cpu.reg.pc = current_pc;
      return;
    }
    cpu.reg.lc = cpu.repeat_lc;
    cpu.repeat_completed = true;
    cpu.repeat_loaded = false;
  } else if (decoded.operation != detail::Operation::repeat_immediate) {
    cpu.repeat_completed = false;
  }
  finish_loop();
}

Core::Core(const Buffers buffers, Io *const io, const Options options)
    : impl_(std::make_unique<Impl>(buffers, io, options)) {
  reset();
}

Core::~Core() = default;

void Core::reset() {
  impl_->cpu = {};
  impl_->cpu.reg.m.fill(word_mask);
  impl_->fault = {};
  clear_code_cache();
}

Progress Core::progress() const noexcept {
  return {.instructions = impl_->cpu.instructions, .estimated_cycles = impl_->cpu.cycles};
}

Fault Core::fault() const noexcept {
  return impl_->fault;
}

Word Core::read(const Space space, const Word address) const {
  const auto memory = impl_->area(space);
  if (address >= memory.size()) { throw std::out_of_range("DSP host read outside memory span"); }
  return memory[address] & word_mask;
}

void Core::write(const Space space, const Word address, const Word value) {
  load(space, address, std::span(&value, 1));
}

void Core::load(const Space space, const Word address, const std::span<const Word> words) {
  const auto memory = impl_->area(space);
  if (address > memory.size() || words.size() > memory.size() - address) {
    throw std::out_of_range("DSP host load outside memory span");
  }
  if (std::ranges::any_of(words, [](const Word word) { return word > word_mask; })) {
    throw std::invalid_argument("DSP memory values must fit in 24 bits");
  }
  for (std::size_t index = 0; index < words.size(); ++index) {
    memory[address + index] = words[index];
  }
  if (space == Space::program) { clear_code_cache(); }
}

RunResult Core::run(const std::uint32_t dispatches) {
  RunResult result;
  while (result.dispatches < dispatches && fault().kind == FaultKind::none) {
    impl_->step();
    if (fault().kind == FaultKind::none) { ++result.dispatches; }
  }
  result.progress = progress();
  result.fault = fault();
  return result;
}

bool Core::interruptible() const {
  if (fault().kind != FaultKind::none || impl_->cpu.repeat_remaining != 0 ||
      impl_->cpu.interrupt_inhibited) {
    return false;
  }
  if (!impl_->cpu.repeat_completed) { return true; }
  const auto pc = impl_->cpu.reg.pc;
  if (pc >= impl_->buffers.program.size()) { return false; }
  return detail::decode(impl_->buffers.program[pc] & word_mask).operation !=
         detail::Operation::repeat_immediate;
}

void Core::clear_code_cache() {
  for (auto &page : impl_->pages) { page.reset(); }
}
} // namespace dsp56300
