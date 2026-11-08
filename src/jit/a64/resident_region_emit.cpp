// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit.hpp"
#include "jit/a64/code.hpp"
#include "jit/a64/neon.hpp"
#include "jit/a64/resident_emit.hpp"
#include "jit/resident_region.hpp"
#include "jit/value_graph.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace dsp56300::detail::a64 {
void RegionEmitter::save() {
  const auto saved_roots = (std::min(root_registers.size(), region_.roots.size()) + 1) / 2;
  for (std::size_t pair = 0; pair < saved_roots; ++pair) {
    code_.push(root_registers[2 * pair], root_registers[(2 * pair) + 1]);
  }
  constexpr unsigned first_saved = 8;
  constexpr unsigned last_saved = 16;
  for (unsigned reg = first_saved; reg < last_saved; reg += 2) {
    neon_.push_doubles(static_cast<Vec>(reg), static_cast<Vec>(reg + 1));
  }
  for (std::size_t root = 0; root < std::min(root_registers.size(), region_.roots.size()); ++root) {
    code_.load(root_registers[root],
               Reg::x0,
               cursor_offset(root, offsetof(RegionCursor, pointer)),
               Access::doubleword);
  }
}

void RegionEmitter::restore() {
  constexpr unsigned first_saved = 8;
  constexpr unsigned last_saved = 16;
  for (auto reg = last_saved; reg != first_saved; reg -= 2) {
    neon_.pop_doubles(static_cast<Vec>(reg - 2), static_cast<Vec>(reg - 1));
  }
  const auto saved_roots = (std::min(root_registers.size(), region_.roots.size()) + 1) / 2;
  for (auto pair = saved_roots; pair != 0; --pair) {
    code_.pop(root_registers[2 * (pair - 1)], root_registers[(2 * pair) - 1]);
  }
}

void RegionEmitter::save_events(const bool preceding) {
  const auto growth =
      preceding ? offsetof(RegionFrame, preceding_growth) : offsetof(RegionFrame, growth);
  const auto limits =
      preceding ? offsetof(RegionFrame, preceding_limits) : offsetof(RegionFrame, limits);
  for (unsigned part = 0; part < 2; ++part) {
    auto growth_value = growth_register + part;
    auto limit_value = limit_register + part;
    if (packed_events()) {
      growth_value = first_scratch;
      limit_value = second_scratch;
      if (part < parts_) {
        neon_.extend(growth_value, growth_register, part != 0);
        neon_.extend(limit_value, limit_register, part != 0);
        if (lanes_ == 1) {
          neon_.insert(growth_value, Reg::zr, 1);
          neon_.insert(limit_value, Reg::zr, 1);
        }
      } else {
        neon_.zero(growth_value);
        neon_.zero(limit_value);
      }
    }
    neon_.store(growth_value, Reg::x0, growth + (std::size_t{part} * 16));
    neon_.store(limit_value, Reg::x0, limits + (std::size_t{part} * 16));
  }
}

void RegionEmitter::emit_body() {
  for (current_ = 0; current_ < region_.graph.nodes.size(); ++current_) {
    for (auto &node : owners_) {
      if (node != no_value && last_[node] < current_) {
        locations_[node] = no_value;
        node = no_value;
      }
    }
    if (needed_[current_]) { emit(region_.graph.nodes[current_]); }
  }
}

std::vector<std::uint8_t> RegionEmitter::generate() {
  save();
  cache_strides();
  code_.load(Reg::x3, Reg::x0, offsetof(RegionFrame, constants), Access::doubleword);
  code_.load(Reg::x1, Reg::x0, offsetof(RegionFrame, iterations));
  code_.constant(Reg::x9, 1);
  neon_.broadcast(one_register, Reg::x9);
  for (unsigned part = 0; part < 2; ++part) {
    neon_.zero(growth_register + part);
    neon_.zero(limit_register + part);
  }
  Label outer;
  Label inner;
  Label complete;
  const auto inner_trips = region_.inner_iterations / (inner_vector_ ? lanes_ : 1U);
  code_.bind(outer);
  if (region_.nested && !inner_vector_) {
    Label continuing;
    code_.constant(Reg::x9, lanes_);
    code_.compare(Reg::x1, Reg::x9);
    code_.branch(Condition::not_equal, continuing);
    save_events(true);
    code_.bind(continuing);
  }
  if (inner_trips > 1) { code_.constant(Reg::x2, inner_trips); }
  code_.bind(inner);
  emit_body();
  advance(true);
  if (inner_trips > 1) {
    code_.constant(Reg::x9, 1);
    code_.alu(Alu::subtract_flags, Reg::x2, Reg::x2, Reg::x9);
    code_.branch_far(Condition::not_equal, inner);
  }
  if (inner_vector_) {
    Label continuing;
    code_.constant(Reg::x9, 2);
    code_.compare(Reg::x1, Reg::x9);
    code_.branch(Condition::not_equal, continuing);
    save_events(true);
    outputs(lanes_ - 1, true);
    code_.bind(continuing);
  }
  code_.constant(Reg::x9, inner_vector_ ? 1U : lanes_);
  code_.alu(Alu::subtract_flags, Reg::x1, Reg::x1, Reg::x9);
  code_.branch_far(Condition::equal, complete);
  advance(false);
  code_.branch(outer);
  code_.bind(complete);
  inputs_.fill(no_value);
  for (unsigned index = 0; index < region_.graph.registers.size(); ++index) {
    if ((region_.graph.changed_registers & (Word{1} << index)) != 0) {
      publish(region_.graph.registers[index],
              {.offset = offsetof(RegionFrame, results) + (index * sizeof(std::uint64_t)),
               .lane = lanes_ - 1});
    }
  }
  outputs(lanes_ - 1, false);
  if (region_.nested && !inner_vector_) { outputs(lanes_ - 2, true); }
  save_events(false);
  restore();
  code_.return_();
  return code_.bytes();
}
}

namespace dsp56300::detail {
std::vector<std::uint8_t> emit_resident_region(ResidentRegion &region, const unsigned lanes) {
  return a64::RegionEmitter(region, lanes).generate();
}

std::vector<std::uint8_t> select_resident_region(ResidentRegion &region,
                                                 const std::span<const std::uint8_t> fallback) {
  using namespace a64;
  Code code;
  Label ordinary;
  Label rejected;
  constexpr Word minimum_work = 128;
  code.load(Reg::x9, Reg::x0, offsetof(Registers, lc));
  code.constant(Reg::x10, ((minimum_work - 1) / region.graph.instructions) + 1);
  code.compare(Reg::x9, Reg::x10);
  code.branch_far(Condition::no_carry, rejected);
  code.load(Reg::x9, Reg::x1, offsetof(NativeContext, budget));
  code.constant(Reg::x10, minimum_work);
  code.compare(Reg::x9, Reg::x10);
  code.branch_far(Condition::no_carry, rejected);
  code.push(Reg::fp, Reg::lr);
  code.add(Reg::fp, Reg::sp, 0, Width::x64);
  code.push(Reg::x19, Reg::x20);
  code.move(Reg::x19, Reg::x0);
  code.move(Reg::x20, Reg::x1);
  code.constant(Reg::x2, reinterpret_cast<std::uintptr_t>(&region));
  code.constant(Reg::x16, std::bit_cast<std::uintptr_t>(&run_resident_region));
  code.call(Reg::x16);
  code.branch_zero(Reg::x0, ordinary);
  code.pop(Reg::x19, Reg::x20);
  code.pop(Reg::fp, Reg::lr);
  code.return_();
  code.bind(ordinary);
  code.move(Reg::x0, Reg::x19);
  code.move(Reg::x1, Reg::x20);
  code.pop(Reg::x19, Reg::x20);
  code.pop(Reg::fp, Reg::lr);
  code.bind(rejected);
  code.append(fallback);
  return code.bytes();
}
}
