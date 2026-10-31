// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "register_model.hpp"
#include "unit_publication_support.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::unit_publication_test {
namespace {
std::uint64_t accumulator_mask_for(const unsigned number) {
  if (number < 10) { return word_mask; }
  if (number < 12) { return upper; }
  if (number < 14) { return middle; }
  return accumulator_mask;
}

RegisterUpdate
data_destination(const Registers &before, const Registers &after, const unsigned number) {
  if (number < 8) {
    return field(number < 6 ? RegisterStorage::x : RegisterStorage::y,
                 number % 2 == 0 ? std::uint64_t{word_mask} : middle,
                 number < 6 ? before.x : before.y,
                 number < 6 ? after.x : after.y);
  }
  return field(number % 2 == 0 ? RegisterStorage::a : RegisterStorage::b,
               accumulator_mask_for(number),
               number % 2 == 0 ? before.a : before.b,
               number % 2 == 0 ? after.a : after.b);
}
}

RegisterUpdate destination(const Registers &before, const Registers &after, const unsigned number) {
  if (number < 16) { return data_destination(before, after, number); }
  if (number < register_model::end_modifiers) {
    const auto index = number % agu_registers;
    if (number < register_model::reg_n_base) {
      return field(RegisterStorage::r, word_mask, before.r[index], after.r[index], index);
    }
    if (number < control_base) {
      return field(RegisterStorage::n, word_mask, before.n[index], after.n[index], index);
    }
    return field(RegisterStorage::m, word_mask, before.m[index], after.m[index], index);
  }
  switch (number) {
    case register_model::reg_ep: return field(RegisterStorage::ep, word_mask, before.ep, after.ep);
    case register_model::reg_vba:
      return field(RegisterStorage::vba, word_mask, before.vba, after.vba);
    case register_model::reg_sc:
      return field(RegisterStorage::sc, stack_counter_mask, before.sc, after.sc);
    case register_model::reg_sz: return field(RegisterStorage::sz, word_mask, before.sz, after.sz);
    case register_model::reg_sr: return field(RegisterStorage::sr, word_mask, before.sr, after.sr);
    case register_model::reg_omr:
      return field(RegisterStorage::omr, word_mask, before.omr, after.omr);
    case register_model::reg_sp:
      return field(RegisterStorage::sp, stack_pointer_mask, before.sp, after.sp);
    case register_model::reg_ssl:
      return field(RegisterStorage::stack,
                   word_mask,
                   before.stack[before.sp % stack_entries],
                   after.stack[before.sp % stack_entries],
                   before.sp % stack_entries);
    case register_model::reg_la: return field(RegisterStorage::la, word_mask, before.la, after.la);
    case register_model::reg_lc: return field(RegisterStorage::lc, word_mask, before.lc, after.lc);
    default: throw std::logic_error("No independent unit destination expectation");
  }
}
} // namespace dsp56300::test::unit_publication_test
