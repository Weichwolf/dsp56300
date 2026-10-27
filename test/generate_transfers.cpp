// SPDX-License-Identifier: GPL-3.0-only
// Independent bus/register expectations; this generator never executes a DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;

enum RegisterId : std::uint8_t {
  reg_r_last = 23,
  reg_n_base = 24,
  end_modifiers = 40,
  reg_ep = 42,
  reg_vba = 48,
  reg_sc = 49,
  reg_sz = 56,
  reg_sr = 57,
  reg_omr = 58,
  reg_sp = 59,
  reg_ssh = 60,
  reg_ssl = 61,
  reg_la = 62,
  reg_lc = 63
};

constexpr unsigned extension_shift = 48;
constexpr Word pointer_mask = 63, extension_fill = 0xffff00;
constexpr std::uint64_t seed_accumulator_a = 0x7e345678abcdef;
constexpr std::uint64_t seed_accumulator_b = 0x813456abcdef12;
constexpr std::uint64_t seed_x = 0x345678fedcba, seed_y = 0x876543123456;
constexpr Word frame_high = 0x123400, frame_low = 0x567800, loop_address = 0xabc;
constexpr Word program_base = 0x070000, control_memory_base = 0x050020;
constexpr Word control_immediate_base = 0x0500a0, class_two_postincrement = 0x081f00;
constexpr Word cursor = 0x180, datum = 0xfedcba;
constexpr std::array<unsigned, 19> controls{32,
                                            33,
                                            34,
                                            35,
                                            36,
                                            37,
                                            38,
                                            39,
                                            reg_ep,
                                            reg_vba,
                                            reg_sc,
                                            reg_sz,
                                            reg_sr,
                                            reg_omr,
                                            reg_sp,
                                            reg_ssh,
                                            reg_ssl,
                                            reg_la,
                                            reg_lc};
constexpr std::array modes{Word{0},
                           mode_down,
                           mode_up,
                           flag_s | flag_l,
                           mode_down | flag_s | flag_l,
                           mode_up | flag_s | flag_l};

bool valid(const unsigned number) {
  return (number >= 4 && number < end_modifiers) || number == reg_ep || number == reg_vba ||
         number == reg_sc || (number >= reg_sz && number < 64);
}

Registers seed(const Word mode) {
  auto result = initial_state();
  result.a = seed_accumulator_a;
  result.b = seed_accumulator_b;
  result.x = seed_x;
  result.y = seed_y;
  result.sr = mode | (status_byte & ~(flag_s | flag_l));
  result.omr = 5;
  result.ep = cursor;
  result.vba = frame_high;
  result.sz = loop_address;
  result.la = loop_address;
  result.lc = 7;
  result.sp = 2;
  result.sc = 2;
  for (std::size_t index = 0; index < agu_registers; ++index) {
    result.r[index] = cursor + static_cast<Word>(index);
    result.n[index] = 3;
  }
  for (std::size_t index = 0; index < stack_entries; ++index) {
    result.stack[index] = (std::uint64_t{frame_high + index} << word_bits) | (frame_low + index);
  }
  return result;
}

Word bus_word(Registers &state, const std::uint64_t raw) {
  const unsigned bits =
      word_bits + ((state.sr & mode_down) != 0 ? 1U : 0U) - ((state.sr & mode_up) != 0 ? 1U : 0U);
  const auto unit = std::int64_t{1} << bits;
  const auto value = signed56(raw);
  auto whole = value / unit;
  if (value % unit < 0) { --whole; }
  const unsigned growth_bit = bits + word_bits - 3;
  if (((raw >> growth_bit) & 1) != ((raw >> (growth_bit + 1)) & 1)) { state.sr |= flag_s; }
  const auto maximum = static_cast<std::int64_t>(sign24 - 1);
  const auto minimum = -static_cast<std::int64_t>(sign24);
  if (whole < minimum || whole > maximum) { state.sr |= flag_l; }
  return static_cast<Word>(std::clamp(whole, minimum, maximum)) & word_mask;
}

Word read(Registers &state, const unsigned number) {
  if (number >= 16 && number < reg_n_base) { return state.r.at(number - 16); }
  if (number >= reg_n_base && number < 32) { return state.n.at(number - reg_n_base); }
  if (number >= 32 && number < end_modifiers) { return state.m.at(number - 32); }
  switch (number) {
    case 4: return static_cast<Word>(state.x) & word_mask;
    case 5: return static_cast<Word>(state.x >> word_bits);
    case 6: return static_cast<Word>(state.y) & word_mask;
    case 7: return static_cast<Word>(state.y >> word_bits);
    case 8: return static_cast<Word>(state.a) & word_mask;
    case 9: return static_cast<Word>(state.b) & word_mask;
    case 10:
    case 11: {
      const auto top = static_cast<Word>((number == 10 ? state.a : state.b) >> extension_shift);
      return top < 128 ? top : top | extension_fill;
    }
    case 12: return static_cast<Word>(state.a >> word_bits) & word_mask;
    case 13: return static_cast<Word>(state.b >> word_bits) & word_mask;
    case 14: return bus_word(state, state.a);
    case 15: return bus_word(state, state.b);
    case reg_ep: return state.ep;
    case reg_vba: return state.vba;
    case reg_sc: return state.sc;
    case reg_sz: return state.sz;
    case reg_sr: return state.sr;
    case reg_omr: return state.omr;
    case reg_sp: return state.sp;
    case reg_ssh: {
      const auto value = static_cast<Word>(state.stack.at(state.sp % stack_entries) >> word_bits);
      state.sp = (state.sp + pointer_mask) % 64;
      --state.sc;
      return value;
    }
    case reg_ssl: return static_cast<Word>(state.stack.at(state.sp % stack_entries)) & word_mask;
    case reg_la: return state.la;
    case reg_lc: return state.lc;
    default: throw std::logic_error("Invalid model source register");
  }
}

void write(Registers &state, const unsigned number, const Word value) {
  if (number >= 16 && number < reg_n_base) {
    state.r.at(number - 16) = value;
    return;
  }
  if (number >= reg_n_base && number < 32) {
    state.n.at(number - reg_n_base) = value;
    return;
  }
  if (number >= 32 && number < end_modifiers) {
    state.m.at(number - 32) = value;
    return;
  }
  if (number >= 4 && number <= 7) {
    auto &bank = number < 6 ? state.x : state.y;
    const unsigned shift = number % 2 == 0 ? 0U : word_bits;
    bank = (bank & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
    return;
  }
  if (number >= 8 && number <= 13) {
    auto &bank = number % 2 == 0 ? state.a : state.b;
    unsigned shift = word_bits;
    if (number < 10) {
      shift = 0;
    } else if (number < 12) {
      shift = extension_shift;
    }
    const auto mask = number < 10 || number >= 12 ? std::uint64_t{word_mask} : std::uint64_t{0xff};
    bank = (bank & ~(mask << shift)) | ((std::uint64_t{value} & mask) << shift);
    return;
  }
  switch (number) {
    case 14:
    case 15:
      (number == 14 ? state.a : state.b) =
          static_cast<std::uint64_t>(signed24(value) * (std::int64_t{1} << word_bits)) &
          accumulator_mask;
      break;
    case reg_ep: state.ep = value; break;
    case reg_vba: state.vba = value; break;
    case reg_sc: state.sc = value % 32; break;
    case reg_sz: state.sz = value; break;
    case reg_sr: state.sr = value; break;
    case reg_omr: state.omr = value; break;
    case reg_sp: state.sp = value % 64; break;
    case reg_ssh:
      state.sp = (state.sp + 1) % 64;
      ++state.sc;
      state.stack.at(state.sp % stack_entries) =
          (std::uint64_t{value} << word_bits) |
          (state.stack.at(state.sp % stack_entries) & word_mask);
      break;
    case reg_ssl:
      state.stack.at(state.sp % stack_entries) =
          ((state.stack.at(state.sp % stack_entries) >> word_bits) << word_bits) | value;
      break;
    case reg_la: state.la = value; break;
    case reg_lc: state.lc = value; break;
    default: throw std::logic_error("Invalid model destination register");
  }
}

void append(Cases &cases, BinaryCase item, std::vector<Word> program) {
  item.name = "independent transfer opcode " + std::to_string(program.at(0));
  item.expected.pc = start + static_cast<Word>(program.size());
  item.before.push_back({.words = std::move(program), .address = start});
  cases.push_back(std::move(item));
}

struct RegisterMove {
  Word mode = 0;
  unsigned control = 32, other = 4;
  bool load = false;
};

void register_move(Cases &cases, const RegisterMove input) {
  BinaryCase item;
  item.initial = seed(input.mode);
  item.expected = item.initial;
  const auto value = read(item.expected, input.load ? input.other : input.control);
  write(item.expected, input.load ? input.control : input.other, value);
  const Word opcode = 0x0440a0 | (input.load ? 0x8000U : 0U) |
                      (static_cast<Word>(input.other) << 8) | (input.control - 32);
  append(cases, std::move(item), {opcode});
}

void register_moves(Cases &cases) {
  for (const Word mode : modes) {
    for (const auto control : controls) {
      for (unsigned other = 4; other < 64; ++other) {
        if (!valid(other)) { continue; }
        for (const bool load : {false, true}) {
          register_move(cases, {.mode = mode, .control = control, .other = other, .load = load});
        }
      }
    }
  }
}

struct Address {
  Word mode = 4, index = 7;
  bool short_address = false, immediate = false;
};

Word address(BinaryCase &item, const Address form) {
  if (form.short_address) { return 12; }
  if (form.mode == 6) { return cursor; }
  auto &reg = item.expected.r.at(form.index);
  switch (form.mode) {
    case 0: reg -= 3; return cursor;
    case 1: reg += 3; return cursor;
    case 2: --reg; return cursor;
    case 3: ++reg; return cursor;
    case 4: return cursor;
    case 5: return cursor + 3;
    case 7: --reg; return cursor - 1;
    default: throw std::logic_error("Invalid model addressing mode");
  }
}

struct MemoryMove {
  Address form{};
  unsigned reg = 32;
  Word mode = 0;
  Space space = Space::x;
  bool load = false;
};

void memory_move(Cases &cases, const MemoryMove input) {
  BinaryCase item;
  item.initial = seed(input.mode);
  item.initial.r.at(input.form.index) = cursor;
  item.expected = item.initial;
  const auto location = address(item, input.form);
  const bool program = input.space == Space::program;
  Word opcode = program ? program_base : control_memory_base;
  opcode |= input.load ? 0x8000U : 0U;
  opcode |= program ? input.reg : input.reg - 32;
  if (input.space == Space::y) { opcode |= 0x40U; }
  if (input.form.short_address) {
    opcode |= location << 8;
  } else {
    opcode |= 0x4000U | (input.form.mode << 11) | (input.form.index << 8);
    if (program) { opcode |= 0x80U; }
  }
  item.progress.estimated_cycles = program ? 6 : 1;
  if (!input.form.short_address &&
      (input.form.mode == 5 || input.form.mode == 6 || input.form.mode == 7)) {
    ++item.progress.estimated_cycles;
  }
  std::vector<Word> words{opcode};
  if (input.form.mode == 6 && !input.form.short_address) {
    words.push_back(input.form.immediate ? datum : cursor);
  }
  if (input.load) {
    write(item.expected, input.reg, datum);
    if (!input.form.immediate) {
      item.before.push_back({.words = {datum}, .address = location, .space = input.space});
    }
  } else {
    const auto value = read(item.expected, input.reg);
    item.after.push_back({.words = {value}, .address = location, .space = input.space});
  }
  append(cases, std::move(item), std::move(words));
}

void memory_variants(Cases &cases, const MemoryMove input) {
  for (const bool load : {false, true}) {
    auto variant = input;
    variant.load = load;
    variant.form.short_address = true;
    memory_move(cases, variant);
    variant.form.short_address = false;
    for (Word mode = 0; mode < 8; ++mode) {
      variant.form.mode = mode;
      variant.form.index = mode == 6 ? 0 : 7;
      // Same-source AGU update ordering needs separate hardware qualification.
      if (input.reg == reg_r_last && mode != 6) { continue; }
      memory_move(cases, variant);
    }
    if (load && input.space != Space::program) {
      variant.form = {.mode = 6, .index = 4, .immediate = true};
      memory_move(cases, variant);
    }
  }
}

void memory_moves(Cases &cases) {
  for (const Word mode : modes) {
    for (const auto reg : controls) {
      for (const auto space : {Space::x, Space::y}) {
        memory_variants(cases, {.reg = reg, .mode = mode, .space = space});
      }
    }
    for (unsigned reg = 4; reg < 64; ++reg) {
      if (valid(reg)) {
        memory_variants(cases, {.reg = reg, .mode = mode, .space = Space::program});
      }
    }
  }
}

void immediate_moves(Cases &cases) {
  for (const auto reg : controls) {
    for (const Word value : {0U, 1U, 0x7fU, 0x80U, 0xffU}) {
      BinaryCase item;
      item.initial = seed(0);
      item.expected = item.initial;
      write(item.expected, reg, value);
      append(cases, std::move(item), {control_immediate_base | (value << 8) | (reg - 32)});
    }
  }
}

void class_two_cases(Cases &cases) {
  for (const bool y : {false, true}) {
    for (const bool b : {false, true}) {
      BinaryCase item;
      item.initial = seed(0);
      item.expected = item.initial;
      const auto reg = b ? 15U : 14U;
      const auto value = read(item.expected, reg);
      write(item.expected, reg, read(item.expected, y ? 6 : 4));
      item.expected.r[7] = cursor + 8;
      item.after.push_back(
          {.words = {value}, .address = cursor + 7, .space = y ? Space::y : Space::x});
      append(cases,
             std::move(item),
             {class_two_postincrement | (b ? 0x10000U : 0U) | (y ? 0x8000U : 0U)});
    }
  }
}

struct FaultCase {
  std::vector<Word> program;
  Fault fault;
  Word cursor_value = cursor;
};

void fault_case(Cases &cases, FaultCase input) {
  BinaryCase item;
  item.initial = seed(0);
  item.initial.r[0] = input.cursor_value;
  item.initial.sc = 0;
  item.initial.sp = 0;
  item.expected = item.initial;
  item.dispatches = 0;
  item.progress = {};
  item.interruptible = false;
  item.fault = input.fault;
  item.fault.pc = start;
  if (item.fault.kind == FaultKind::invalid_instruction) {
    item.fault.space = Space::program;
    item.fault.address = start;
    item.fault.value = input.program.at(0);
  }
  append(cases, std::move(item), std::move(input.program));
}

void fault_cases(Cases &cases) {
  constexpr std::array<Word, 8> invalid{
      0x0500a8, 0x04c0a0, 0x05f120, 0x07f484, 0x07f184, 0x078000, 0x08b005, 0x08b445};
  for (const auto opcode : invalid) {
    fault_case(cases,
               {.program = {opcode, cursor}, .fault = {.kind = FaultKind::invalid_instruction}});
  }
  constexpr Word read_program = 0x07e084;
  constexpr Word write_program = 0x076084;
  fault_case(cases,
             {.program = {read_program},
              .fault = {.kind = FaultKind::memory_read, .address = memory_words},
              .cursor_value = memory_words});
  fault_case(cases,
             {.program = {write_program},
              .fault = {.kind = FaultKind::memory_write, .address = memory_words, .value = datum},
              .cursor_value = memory_words});
  constexpr Word pop_to_x0 = 0x0444bc;
  fault_case(cases,
             {.program = {pop_to_x0}, .fault = {.kind = FaultKind::stack, .address = start}});
  constexpr std::array<Word, 8> io{
      0x084405, 0x08c405, 0x094405, 0x09c405, 0x044485, 0x04c485, 0x044425, 0x04c425};
  for (std::size_t index = 0; index < io.size(); ++index) {
    const bool y = (index & 2U) != 0;
    const bool write = (index & 1U) != 0;
    constexpr Word high = 0xffffc5;
    constexpr Word low = 0xffff85;
    fault_case(cases,
               {.program = {io[index]},
                .fault = {.kind = write ? FaultKind::io_write : FaultKind::io_read,
                          .space = y ? Space::y : Space::x,
                          .address = index < 4 ? high : low,
                          .value = write ? datum : 0}});
  }
}
} // namespace
} // namespace dsp56300::test

int main(const int argc, const char *const *const argv) {
  try {
    if (argc != 2) { throw std::runtime_error("Usage: dsp56300_generate_transfers OUTPUT.d56"); }
    std::vector<dsp56300::test::BinaryCase> cases;
    dsp56300::test::register_moves(cases);
    dsp56300::test::memory_moves(cases);
    dsp56300::test::immediate_moves(cases);
    dsp56300::test::class_two_cases(cases);
    dsp56300::test::fault_cases(cases);
    const auto count = cases.size();
    dsp56300::test::write_fixture(argv[1], std::move(cases));
    std::cout << count << " independent transfer cases written\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
