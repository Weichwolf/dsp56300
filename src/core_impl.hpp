// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "architecture.hpp"
#include "bit_instruction.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image.hpp"
#include "instruction.hpp"
#include "jit.hpp"
#include "jit/fixed_pass_memory.hpp"
#include "program_cache.hpp"
#include "semantic.hpp"
#include "vector_state.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace dsp56300::detail {
class NativeCall;
}

namespace dsp56300 {
struct Core::Impl {
  using VectorPhase = detail::VectorPhase;
  using Vector = detail::VectorState;

  [[nodiscard]] bool exception_ready() const noexcept {
    return vector.phase != VectorPhase::none && cpu.repeat_remaining == 0;
  }

  [[nodiscard]] bool debug_ready() const noexcept {
    return (cpu.debug_flags & detail::debug_interrupt_pending) != 0 && cpu.repeat_remaining == 0 &&
           vector.phase == VectorPhase::none;
  }

  void request_debug(bool software);
  void execute_debug(const detail::Instruction &decoded);
  void admit_debug();

  [[nodiscard]] bool instruction_cache_active() const noexcept {
    return (cpu.reg.sr & detail::cache_enable) != 0 && buffers.cache.begin < buffers.cache.end;
  }

  [[nodiscard]] bool cacheable(const Word address) const noexcept {
    return (cpu.reg.sr & detail::cache_enable) != 0 && address >= buffers.cache.begin &&
           address < buffers.cache.end;
  }

  [[nodiscard]] detail::ProgramCache &program_cache();

  void synchronize_cache_mode() {
    if ((cpu.reg.sr & detail::cache_enable) == 0) { icache.reset(); }
  }

  [[nodiscard]] Word fetch_program(Word address);
  void check_interrupt_window(Word previous_status) noexcept;
  [[nodiscard]] Word read_external(Space space, Word address);
  void write_external(Space space, Word address, Word value);
  void execute_processing(const detail::Instruction &decoded);
  void execute_cache(const detail::Instruction &decoded);
  void execute_exception(const detail::Instruction &decoded);
  void step_vector_semantic(const detail::Instruction &decoded);
  void call_vector(const detail::Instruction &decoded);

  explicit Impl(Buffers supplied,
                Io *host_io,
                bool external_memory = false,
                std::array<std::span<const MemoryWindow>, 3> region_views = {},
                std::array<Memory::Views, 3> address_views = {});

  void load_views(Space space, Word address, std::span<const Word> words);

  [[nodiscard]] bool complete_memory() const noexcept { return !views[0].read.data.empty(); }

  void retire_instruction(const unsigned cycles) {
    ++cpu.instructions;
    cpu.cycles += cycles;
  }

  [[nodiscard]] std::span<Word> area(Space space) const;
  [[gnu::cold]] void fail(FaultKind kind, Space space, Word address, Word value = 0);
  [[nodiscard]] Word read_memory(Space space, Word address);
  void write_memory(Space space, Word address, Word value);
  [[nodiscard]] bool overlaps_program(std::span<const Word> memory) const;
  void invalidate_program_word(const Word *location);
  void step_semantic(const detail::Instruction &decoded);
  void assume_image_admission() const noexcept;
  [[nodiscard]] static Word semantic_numeric_mode(void *owner) noexcept
      asm("dsp56300_semantic_numeric_mode");
  [[nodiscard]] static bool
  semantic_retired(void *owner, std::uint32_t count, bool region, std::uint64_t site) noexcept
      asm("dsp56300_semantic_retired");
  static void semantic_memory(void *owner, const detail::SemanticMemory &memory) noexcept
      asm("dsp56300_semantic_memory");
  [[nodiscard]] static std::uint64_t semantic_region(void *owner,
                                                     std::uint32_t budget,
                                                     const detail::SemanticRegion &region,
                                                     std::uint64_t window,
                                                     std::uint64_t &cycles) noexcept
      asm("dsp56300_semantic_region");
  [[nodiscard]] static std::uint64_t
  semantic_paired_read(void *owner,
                       std::uint32_t budget,
                       const detail::SemanticPairedPortRead &read,
                       std::uint64_t window,
                       std::uint64_t &cycles) noexcept asm("dsp56300_semantic_paired_read");
  [[nodiscard]] bool plain_foreground() const noexcept;
  [[nodiscard]] static bool valid_registers(const Registers &state) noexcept;
  [[nodiscard]] static std::uint32_t
  semantic_foreground(void *owner, Word pc, std::uint32_t quota, std::uint64_t window) noexcept
      asm("dsp56300_semantic_foreground");
  using FixedPassEntry = std::uint64_t (*)(void *) noexcept;
  [[nodiscard]] static std::uint64_t semantic_fixed_pass(void *owner,
                                                         Word head,
                                                         FixedPassEntry proof,
                                                         std::uint64_t window,
                                                         detail::PassWord *scratch,
                                                         std::uint32_t quota,
                                                         detail::PassCommit &commit) noexcept
      asm("dsp56300_semantic_fixed_pass");
  [[nodiscard]] static std::uint64_t checked_fixed_pass(void *owner,
                                                        Word head,
                                                        FixedPassEntry proof,
                                                        std::uint64_t window,
                                                        detail::PassWord *scratch,
                                                        std::uint32_t quota,
                                                        detail::PassCommit &commit,
                                                        std::uint64_t retained_horizon) noexcept;
  [[nodiscard]] RunResult advance_proven_pass(std::uint32_t quota);
  [[nodiscard]] static detail::SemanticResult
  semantic_execute(void *owner, const detail::Instruction &decoded) noexcept;
  // Stable private LLVM boundary; the packed result does not depend on struct-return ABI.
  [[nodiscard]] static std::uint64_t
  semantic_entry(void *owner, Word pc, const detail::Instruction &decoded) noexcept
      asm("dsp56300_semantic_execute");
  [[nodiscard]] static std::uint64_t
  semantic_vector_entry(void *owner, const detail::Instruction &decoded) noexcept
      asm("dsp56300_semantic_vector_execute");
  [[nodiscard]] std::optional<Word> known_program_word(Word address) const;
  [[nodiscard]] RunResult run_image(std::uint32_t quota);
  [[nodiscard]] std::uint64_t
  call_image(detail::ImageEntry entry, Word pc, std::uint32_t instructions, std::uint64_t cycles);
  [[nodiscard]] detail::ImageEntry image_vector();
  [[nodiscard]] bool image_interrupt(Word address, unsigned level);
  [[nodiscard]] bool image_transition(bool eligible);
  [[nodiscard]] Word image_address(bool vector_fetch) const noexcept;
  void image_word_written(detail::ProgramCache::Write transfer) noexcept;
  void image_io_completed() noexcept;
  void unprepared(Word address, Word value = 0) noexcept;
  [[nodiscard]] const MemoryWindow *memory_window(Space space, Word address) const noexcept;
  [[nodiscard]] bool write_window(Space space, Word address, Word value);
  void execute_operation(const detail::Instruction &decoded);
  void execute_return(const detail::Instruction &decoded);
  void execute_immediate(const detail::Instruction &decoded);
  void execute_iteration(const detail::Instruction &decoded);
  void normalize_fast(Word opcode);
  void viterbi_shift(const detail::Instruction &decoded);
  void execute_address_calculation(const detail::Instruction &decoded);
  void execute_product(const detail::Instruction &decoded);
  void execute_unit(const detail::Instruction &decoded);
  void compare_unsigned(Word opcode);
  void execute_bit_field(const detail::Instruction &decoded);
  [[nodiscard]] Word bit_field_control(const detail::Instruction &decoded);
  void extract_field(const detail::Instruction &decoded, Word control);
  void insert_field(const detail::Instruction &decoded, Word control);
  void merge_field(const detail::Instruction &decoded, Word source);
  void count_leading_bits(Word opcode);
  void execute_jump(const detail::Instruction &decoded);
  void execute_branch(const detail::Instruction &decoded);
  void execute_bit(const detail::Instruction &decoded);
  [[nodiscard]] bool access_bit(const detail::Instruction &decoded, BitAction action);
  [[nodiscard]] bool register_bit(const detail::BitAccess &access);
  [[nodiscard]] Word bit_address(const detail::BitOperand &operand,
                                 const detail::Instruction &decoded);
  void conditional_transfer(Word opcode);
  void conditional_alu(Word opcode);
  [[nodiscard]] Word control_address(const detail::Instruction &decoded);
  void parallel(const detail::Instruction &decoded);
  void moves(const detail::Instruction &decoded);
  void register_move(Word opcode);
  void paired_move(Word opcode);
  void memory_move(const detail::Instruction &decoded);
  void displacement_move(const detail::Instruction &decoded);
  void load_long(const detail::Instruction &decoded, std::uint64_t value);
  void long_move(const detail::Instruction &decoded);
  void mixed_move(const detail::Instruction &decoded);
  void class_two_move(const detail::Instruction &decoded);
  void control_move(const detail::Instruction &decoded);
  void control_register_move(Word opcode);
  void program_move(const detail::Instruction &decoded);
  void peripheral_move(const detail::Instruction &decoded);
  void peripheral_register_move(const detail::Instruction &decoded);
  void alu(Word opcode);
  void multiply(Word opcode);
  [[nodiscard]] bool unary(Word opcode);
  [[nodiscard]] bool logic(Word opcode);
  [[nodiscard]] bool addition(Word opcode);
  [[nodiscard]] bool shifted_addition(Word opcode);
  void add_operands(detail::AddOperands input);
  [[nodiscard]] bool word_shift(Word opcode);
  [[nodiscard]] bool numeric_mode_supported(Word ignored_modes = 0);
  [[nodiscard]] bool numeric_alu_supported(Word opcode);
  bool limit_arithmetic(std::uint64_t &value, bool rounded = false) const;
  void saturate_arithmetic(std::uint64_t &value, bool rounded = false);
  void execute_shift(const detail::Instruction &decoded);
  void arithmetic_shift(std::uint64_t value, detail::ShiftControl control);
  void logical_shift(detail::ShiftControl control, bool rotate = false);
  [[nodiscard]] Word read_register(unsigned number);
  void write_register(unsigned number, Word value);
  [[nodiscard]] Word effective(detail::AddressForm form);
  [[nodiscard]] Word update_address(detail::AddressUpdate update);
  [[nodiscard]] Word transfer_word(std::uint64_t value);
  [[nodiscard]] std::uint64_t transfer_long(std::uint64_t value);
  [[nodiscard]] std::uint64_t source(unsigned selector, bool destination_b) const;
  void arithmetic_flags(std::uint64_t value, bool overflowed);
  void logical_flags(Word value);
  void round(std::uint64_t &value);
  void push(Word high, Word low);
  [[nodiscard]] std::uint64_t pop();
  void finish_loop(Word last);
  void restore_loop();
  void execute_loop(const detail::Instruction &decoded);
  [[nodiscard]] Word loop_count(const detail::Instruction &decoded);

  // Fixed passes start between instructions with no active REP. Operand
  // latches are initialized by each instruction before use.
  struct FixedPassState {
    detail::Cpu cpu;
    Vector vector;
    Word current_pc, debug_vector;
  };

  static constexpr std::size_t fixed_pass_observations = 64;

  struct FixedPassMemo {
    FixedPassState state;
    std::array<detail::PassWord, fixed_pass_observations> cells;
    std::array<Word, fixed_pass_observations> device_values;
    FixedPassEntry proof;
    std::uint64_t revision;
    std::size_t cell_count;
  };

  detail::Cpu cpu;
  std::optional<FixedPassMemo> fixed_pass_memo;
  std::optional<std::uint64_t> fixed_pass_handoff_cycle;
  bool fixed_pass_handoff_pending = false;
  std::uint64_t proven_deadline = 0;
  Io *io;
  std::unique_ptr<detail::ProgramCache> icache;
  std::unique_ptr<detail::NativeCall> native_call;
  std::uint64_t cycle_deadline = std::numeric_limits<std::uint64_t>::max();
  const NativeImage::Impl::Variant *active_image = nullptr;
  const std::vector<std::uint64_t> *next_images = nullptr;
  std::uint64_t image_revision = 0;
  std::shared_ptr<const NativeImage::Impl> image;
  std::array<std::uint64_t, 4> latched{};
  std::array<std::span<const MemoryWindow>, 2> image_windows{};
  std::array<std::span<const MemoryWindow>, 3> regions;
  Buffers buffers;
  std::array<Memory::Views, 3> views;
  Fault fault;
  Word current_pc = 0;
  Word debug_vector = 0;
  Word program_omr = 0;
  detail::Instruction repeated;
  Vector vector;
  bool executing = false;
  bool external = false;
  bool yield_requested = false;
  bool image_ready = false;
  bool image_storage_admitted = false;
  bool terminal_fault = false;
  std::array<bool, 3> aliases_program{true, false, false};
  bool canonical_data = false;
};
} // namespace dsp56300
