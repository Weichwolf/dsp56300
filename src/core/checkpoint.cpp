// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"

#include "architecture.hpp"
#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "instruction.hpp"
#include "program_cache.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>

namespace dsp56300 {
namespace {
struct MemoryLayout {
  std::array<std::size_t, 3> sizes{};
  std::array<std::optional<std::int64_t>, 3> offsets{};
  bool operator==(const MemoryLayout &) const = default;
};

std::optional<std::int64_t> overlap(const std::span<const Word> left,
                                    const std::span<const Word> right) {
  const auto first = reinterpret_cast<std::uintptr_t>(left.data());
  const auto second = reinterpret_cast<std::uintptr_t>(right.data());
  if (second >= first) {
    if (second - first >= left.size_bytes()) { return std::nullopt; }
    return static_cast<std::int64_t>(second - first);
  }
  if (first - second >= right.size_bytes()) { return std::nullopt; }
  return -static_cast<std::int64_t>(first - second);
}

MemoryLayout layout(const Buffers buffers) {
  return {.sizes = {buffers.program.size(), buffers.x.size(), buffers.y.size()},
          .offsets = {overlap(buffers.program, buffers.x),
                      overlap(buffers.program, buffers.y),
                      overlap(buffers.x, buffers.y)}};
}
}

struct Checkpoint::State {
  detail::Cpu cpu;
  std::array<std::uint64_t, 4> latched;
  Word current_pc;
  unsigned current_words;
  detail::Instruction repeated;
  Fault fault;
  Core::Impl::Vector vector;
  Word debug_vector;
  std::optional<detail::ProgramCache> cache;
  MemoryLayout memory;
  Word cacheable_begin;
  Word cacheable_end;
  unsigned cache_sector_words;
};

Checkpoint::Checkpoint(std::shared_ptr<const State> state) : state_(std::move(state)) {}

Checkpoint::~Checkpoint() = default;

Checkpoint Core::checkpoint() const {
  if (impl_->executing) { throw std::logic_error("Checkpoint capture requires stopped execution"); }
  const auto cache = impl_->icache ? std::optional(*impl_->icache) : std::nullopt;
  return Checkpoint(std::make_shared<const Checkpoint::State>(
      Checkpoint::State{.cpu = impl_->cpu,
                        .latched = impl_->latched,
                        .current_pc = impl_->current_pc,
                        .current_words = impl_->current_words,
                        .repeated = impl_->repeated,
                        .fault = impl_->fault,
                        .vector = impl_->vector,
                        .debug_vector = impl_->debug_vector,
                        .cache = cache,
                        .memory = layout(impl_->buffers),
                        .cacheable_begin = impl_->options.cacheable_begin,
                        .cacheable_end = impl_->options.cacheable_end,
                        .cache_sector_words = impl_->options.cache_sector_words}));
}

void Core::restore(const Checkpoint &checkpoint) {
  if (impl_->executing) { throw std::logic_error("Checkpoint restore requires stopped execution"); }
  if (!checkpoint.state_) { throw std::invalid_argument("Cannot restore an empty checkpoint"); }
  const auto &state = *checkpoint.state_;
  if (state.memory != layout(impl_->buffers) ||
      state.cacheable_begin != impl_->options.cacheable_begin ||
      state.cacheable_end != impl_->options.cacheable_end ||
      state.cache_sector_words != impl_->options.cache_sector_words) {
    throw std::invalid_argument("Checkpoint memory aliases, sizes or cache geometry differ");
  }
  // Allocate before mutating the destination. Remaining state transfers cannot fail.
  auto cache = state.cache ? std::make_unique<detail::ProgramCache>(*state.cache) : nullptr;
  clear_code_cache();
  impl_->cpu = state.cpu;
  impl_->latched = state.latched;
  impl_->current_pc = state.current_pc;
  impl_->current_words = state.current_words;
  impl_->repeated = state.repeated;
  impl_->fault = state.fault;
  impl_->vector = state.vector;
  impl_->debug_vector = state.debug_vector;
  impl_->icache = std::move(cache);
}
} // namespace dsp56300
