// SPDX-License-Identifier: GPL-3.0-only
// Independent bus/register expectations; this generator never executes a DSP core.
#include "binary_fixture.hpp"
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dsp56300::test {
namespace {
using Cases = std::vector<BinaryCase>;
using namespace register_model;

constexpr Word program_base = 0x070000, control_memory_base = 0x050020;
constexpr Word control_immediate_base = 0x0500a0, class_two_postincrement = 0x081f00;
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
