// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "support.hpp"

#include <cstdint>
#include <stdexcept>

namespace dsp56300::test::unit_fixture_test {
inline void apply(Registers &state, const RegisterUpdate field) {
  std::uint64_t *wide = nullptr;
  Word *word = nullptr;
  switch (field.storage) {
    case RegisterStorage::x: wide = &state.x; break;
    case RegisterStorage::y: wide = &state.y; break;
    case RegisterStorage::a: wide = &state.a; break;
    case RegisterStorage::b: wide = &state.b; break;
    case RegisterStorage::stack: wide = &state.stack.at(field.index); break;
    case RegisterStorage::r: word = &state.r.at(field.index); break;
    case RegisterStorage::n: word = &state.n.at(field.index); break;
    case RegisterStorage::m: word = &state.m.at(field.index); break;
    case RegisterStorage::sr: word = &state.sr; break;
    case RegisterStorage::pc: word = &state.pc; break;
    case RegisterStorage::omr: word = &state.omr; break;
    case RegisterStorage::la: word = &state.la; break;
    case RegisterStorage::lc: word = &state.lc; break;
    case RegisterStorage::sp: word = &state.sp; break;
    case RegisterStorage::sc: word = &state.sc; break;
    case RegisterStorage::sz: word = &state.sz; break;
    case RegisterStorage::ep: word = &state.ep; break;
    case RegisterStorage::vba: word = &state.vba; break;
    default: throw std::runtime_error("Unknown native scalar-unit physical storage");
  }
  const auto before = wide != nullptr ? *wide : *word;
  require(field.mask != 0 && (before & field.mask) == field.previous &&
              (field.previous & ~field.mask) == 0 && (field.value & ~field.mask) == 0,
          "Every scalar-unit field retains its actual first-written bits and bounded result");
  const auto after = (before & ~field.mask) | field.value;
  if (wide != nullptr) {
    *wide = after;
  } else {
    *word = static_cast<Word>(after);
  }
}
} // namespace dsp56300::test::unit_fixture_test
