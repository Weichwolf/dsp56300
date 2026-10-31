// SPDX-License-Identifier: GPL-3.0-only
#include "move_storage.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "fields.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace dsp56300::detail {
namespace {
template <typename State, typename Visitor>
void storage(State &state, const RegisterUpdate update, const Visitor &visit) {
  switch (update.storage) {
    case RegisterStorage::x: visit(state.x, 0U); return;
    case RegisterStorage::y: visit(state.y, 0U); return;
    case RegisterStorage::a: visit(state.a, padding_bits); return;
    case RegisterStorage::b: visit(state.b, padding_bits); return;
    case RegisterStorage::r: visit(state.r.at(update.index), 0U); return;
    case RegisterStorage::n: visit(state.n.at(update.index), 0U); return;
    case RegisterStorage::m: visit(state.m.at(update.index), 0U); return;
    case RegisterStorage::sr: visit(state.sr, 0U); return;
    case RegisterStorage::omr: visit(state.omr, 0U); return;
    case RegisterStorage::la: visit(state.la, 0U); return;
    case RegisterStorage::lc: visit(state.lc, 0U); return;
    case RegisterStorage::sp: visit(state.sp, 0U); return;
    case RegisterStorage::sc: visit(state.sc, 0U); return;
    case RegisterStorage::sz: visit(state.sz, 0U); return;
    case RegisterStorage::ep: visit(state.ep, 0U); return;
    case RegisterStorage::vba: visit(state.vba, 0U); return;
    case RegisterStorage::stack: visit(state.stack.at(update.index), 0U); return;
    case RegisterStorage::pc: visit(state.pc, 0U); return;
    default: throw std::logic_error("Unknown native MOVE storage");
  }
}
}

std::uint64_t move_storage(const Registers &state, const RegisterUpdate update) {
  std::uint64_t value = 0;
  storage(state, update, [&](const auto &field, const unsigned shift) { value = field >> shift; });
  return value;
}

void merge_move_storage(Registers &state, const RegisterUpdate update) {
  storage(state, update, [&](auto &field, const unsigned shift) {
    using Field = std::remove_reference_t<decltype(field)>;
    const auto mask = update.mask << shift;
    field = static_cast<Field>((field & ~mask) | ((update.value & update.mask) << shift));
  });
}
} // namespace dsp56300::detail

namespace dsp56300 {
using namespace detail;

MoveWriteback Core::Impl::move_destination(const unsigned number) const {
  MoveWriteback result{.selector = static_cast<std::uint8_t>(number)};
  const auto add = [&](const RegisterStorage field,
                       const std::uint64_t mask = word_mask,
                       const unsigned index = 0) {
    result.updates.at(result.count++) = {
        .storage = field, .index = static_cast<std::uint8_t>(index), .mask = mask};
  };
  if (number >= r_register_base && number < n_register_base) {
    add(RegisterStorage::r, word_mask, number - r_register_base);
  } else if (number >= n_register_base && number < m_register_base) {
    add(RegisterStorage::n, word_mask, number - n_register_base);
  } else if (number >= m_register_base && number < m_register_base + agu_registers) {
    add(RegisterStorage::m, word_mask, number - m_register_base);
  } else {
    constexpr auto main = std::uint64_t{word_mask} << word_bits;
    constexpr auto high = accumulator_mask ^ long_mask;
    switch (number) {
      case x0_register: add(RegisterStorage::x); break;
      case x1_register: add(RegisterStorage::x, main); break;
      case y0_register: add(RegisterStorage::y); break;
      case y1_register: add(RegisterStorage::y, main); break;
      case a0_register: add(RegisterStorage::a); break;
      case b0_register: add(RegisterStorage::b); break;
      case a1_register: add(RegisterStorage::a, main); break;
      case b1_register: add(RegisterStorage::b, main); break;
      case a2_register: add(RegisterStorage::a, high); break;
      case b2_register: add(RegisterStorage::b, high); break;
      case a_register: add(RegisterStorage::a, accumulator_mask); break;
      case b_register: add(RegisterStorage::b, accumulator_mask); break;
      case ep_register: add(RegisterStorage::ep); break;
      case vba_register: add(RegisterStorage::vba); break;
      case sc_register: add(RegisterStorage::sc, (stack_entries * 2) - 1); break;
      case sz_register: add(RegisterStorage::sz); break;
      case sr_register: add(RegisterStorage::sr); break;
      case omr_register: add(RegisterStorage::omr); break;
      case sp_register: add(RegisterStorage::sp, (stack_entries * 4) - 1); break;
      case la_register: add(RegisterStorage::la); break;
      case lc_register: add(RegisterStorage::lc); break;
      case ssh_register:
        add(RegisterStorage::sp, (stack_entries * 4) - 1);
        add(RegisterStorage::sc, (stack_entries * 2) - 1);
        add(RegisterStorage::stack, long_mask, (cpu.reg.sp + 1) & (stack_entries - 1));
        break;
      case ssl_register:
        add(RegisterStorage::stack, word_mask, cpu.reg.sp & (stack_entries - 1));
        break;
      default: break;
    }
  }
  return result;
}

MoveWriteback Core::Impl::long_destination(const unsigned selector) {
  MoveWriteback result{.producer = MoveProducer::long_load,
                       .selector = static_cast<std::uint8_t>(selector)};
  if (selector >= 6) { return result; } // AB/BA use two real register writes.
  constexpr std::array fields{RegisterStorage::a,
                              RegisterStorage::b,
                              RegisterStorage::x,
                              RegisterStorage::y,
                              RegisterStorage::a,
                              RegisterStorage::b};
  result.count = 1;
  result.updates[0] = {.storage = fields.at(selector),
                       .mask = selector >= 4 ? accumulator_mask : long_mask};
  return result;
}
} // namespace dsp56300
