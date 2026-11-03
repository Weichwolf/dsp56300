// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"

#include <concepts>
#include <cstdint>
#include <optional>
#include <span>

namespace dsp56300 {
// Structural integration contract. Numerical correctness, ownership, invalidation
// and event ordering still need independent behavioral qualification.
template <class Implementation>
concept CoreImplementation = std::constructible_from<Implementation, Buffers, Io *, Options> &&
                             requires(Implementation &core,
                                      const Implementation &view,
                                      const Registers &state,
                                      const Checkpoint &checkpoint,
                                      const InstructionWords instruction,
                                      const EffectTicket &ticket,
                                      const Space space,
                                      const Word address,
                                      const Word value,
                                      const std::span<const Word> words,
                                      const std::uint32_t budget,
                                      const unsigned level,
                                      const bool enabled) {
                               { core.reset() } -> std::same_as<void>;
                               { view.registers() } -> std::same_as<Registers>;
                               { core.set_registers(state) } -> std::same_as<void>;
                               { view.checkpoint() } -> std::same_as<Checkpoint>;
                               { view.candidate_checkpoint(ticket) } -> std::same_as<Checkpoint>;
                               { core.restore(checkpoint) } -> std::same_as<void>;
                               { view.instruction_fetch() } -> std::same_as<InstructionFetch>;
                               {
                                 core.begin_instruction(instruction)
                               } -> std::same_as<InstructionProgress>;
                               {
                                 view.pending_instruction()
                               } -> std::same_as<std::optional<PendingInstruction>>;
                               {
                                 core.advance_instruction(ticket)
                               } -> std::same_as<InstructionProgress>;
                               { view.interruptible(instruction) } -> std::same_as<bool>;
                               {
                                 core.request_interrupt(address, level, instruction)
                               } -> std::same_as<void>;
                               { view.progress() } noexcept -> std::same_as<Progress>;
                               { view.fault() } noexcept -> std::same_as<Fault>;
                               { view.jit_statistics() } noexcept -> std::same_as<JitStatistics>;
                               { view.read(space, address) } -> std::same_as<Word>;
                               { core.write(space, address, value) } -> std::same_as<void>;
                               { core.load(space, address, words) } -> std::same_as<void>;
                               { core.run(budget) } -> std::same_as<RunResult>;
                               {
                                 view.processing_state()
                               } noexcept -> std::same_as<ProcessingState>;
                               { view.debug_status() } noexcept -> std::same_as<DebugStatus>;
                               { core.set_debug_interrupt_mode(enabled) } -> std::same_as<void>;
                               { core.request_debug() } -> std::same_as<void>;
                               { core.resume_from_debug() } -> std::same_as<void>;
                               { core.resume_from_stop() } -> std::same_as<void>;
                               { view.interruptible() } -> std::same_as<bool>;
                               { core.interrupt(address, level) } -> std::same_as<void>;
                               { core.clear_code_cache() } -> std::same_as<void>;
                             };

template <Backend Kind, BatchBackend Batch = BatchBackend::automatic>
  requires((Kind == Backend::reference || Kind == Backend::cached || Kind == Backend::jit ||
            Kind == Backend::automatic) &&
           (Batch == BatchBackend::automatic || Batch == BatchBackend::disabled ||
            Batch == BatchBackend::scalar || Batch == BatchBackend::avx2))
class BackendCore {
public:
  static constexpr Backend backend = Kind;

  explicit BackendCore(const Buffers buffers, Io *const io = nullptr, const Options options = {})
      : core_(buffers, io, select(options)) {}

  explicit BackendCore(Io &bus, const Options options = {}) : core_(bus, select(options)) {}

  void reset() { core_.reset(); }

  [[nodiscard]] Registers registers() const { return core_.registers(); }

  void set_registers(const Registers &state) { core_.set_registers(state); }

  [[nodiscard]] Checkpoint checkpoint() const { return core_.checkpoint(); }

  [[nodiscard]] Checkpoint candidate_checkpoint(const EffectTicket &ticket) const {
    return core_.candidate_checkpoint(ticket);
  }

  void restore(const Checkpoint &checkpoint) { core_.restore(checkpoint); }

  [[nodiscard]] InstructionFetch instruction_fetch() const { return core_.instruction_fetch(); }

  [[nodiscard]] InstructionProgress begin_instruction(const InstructionWords words) {
    return core_.begin_instruction(words);
  }

  [[nodiscard]] std::optional<PendingInstruction> pending_instruction() const {
    return core_.pending_instruction();
  }

  [[nodiscard]] InstructionProgress advance_instruction(const EffectTicket &ticket) {
    return core_.advance_instruction(ticket);
  }

  [[nodiscard]] bool interruptible(const InstructionWords words) const {
    return core_.interruptible(words);
  }

  void request_interrupt(const Word vector, const unsigned level, const InstructionWords words) {
    core_.request_interrupt(vector, level, words);
  }

  [[nodiscard]] Progress progress() const noexcept { return core_.progress(); }

  [[nodiscard]] Fault fault() const noexcept { return core_.fault(); }

  [[nodiscard]] JitStatistics jit_statistics() const noexcept { return core_.jit_statistics(); }

  [[nodiscard]] Word read(const Space space, const Word address) const {
    return core_.read(space, address);
  }

  void write(const Space space, const Word address, const Word value) {
    core_.write(space, address, value);
  }

  void load(const Space space, const Word address, const std::span<const Word> words) {
    core_.load(space, address, words);
  }

  [[nodiscard]] RunResult run(const std::uint32_t budget) { return core_.run(budget); }

  [[nodiscard]] ProcessingState processing_state() const noexcept {
    return core_.processing_state();
  }

  [[nodiscard]] DebugStatus debug_status() const noexcept { return core_.debug_status(); }

  void set_debug_interrupt_mode(const bool enabled) { core_.set_debug_interrupt_mode(enabled); }

  void request_debug() { core_.request_debug(); }

  void resume_from_debug() { core_.resume_from_debug(); }

  void resume_from_stop() { core_.resume_from_stop(); }

  [[nodiscard]] bool interruptible() const { return core_.interruptible(); }

  void interrupt(const Word vector, const unsigned level) { core_.interrupt(vector, level); }

  void clear_code_cache() { core_.clear_code_cache(); }

private:
  static Options select(Options options) {
    options.backend = Kind;
    if constexpr (Batch != BatchBackend::automatic) { options.batch_backend = Batch; }
    return options;
  }

  Core core_;
};
} // namespace dsp56300
