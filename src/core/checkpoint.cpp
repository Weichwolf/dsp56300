// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/checkpoint.hpp"

#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "execution_state.hpp"
#include "staged.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
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
  detail::ExecutionState execution;
  std::optional<detail::StagedState> staged;
  bool external = false;
  MemoryLayout memory;
  Word cacheable_begin;
  Word cacheable_end;
  unsigned cache_sector_words;
};

Checkpoint::Checkpoint(std::shared_ptr<const State> state) : state_(std::move(state)) {}

Checkpoint::~Checkpoint() = default;

Checkpoint Core::checkpoint() const {
  if (impl_->executing) { throw std::logic_error("Checkpoint capture requires stopped execution"); }
  return Checkpoint(std::make_shared<const Checkpoint::State>(Checkpoint::State{
      .execution = impl_->execution_state(),
      .staged = impl_->staged_active() ? std::optional(*impl_->staged) : std::nullopt,
      .external = impl_->external,
      .memory = layout(impl_->buffers),
      .cacheable_begin = impl_->options.cacheable_begin,
      .cacheable_end = impl_->options.cacheable_end,
      .cache_sector_words = impl_->options.cache_sector_words}));
}

void Core::restore(const Checkpoint &checkpoint) {
  if (impl_->executing) { throw std::logic_error("Checkpoint restore requires stopped execution"); }
  if (!checkpoint.state_) { throw std::invalid_argument("Cannot restore an empty checkpoint"); }
  const auto &state = *checkpoint.state_;
  if (state.external != impl_->external || state.memory != layout(impl_->buffers) ||
      state.cacheable_begin != impl_->options.cacheable_begin ||
      state.cacheable_end != impl_->options.cacheable_end ||
      state.cache_sector_words != impl_->options.cache_sector_words) {
    throw std::invalid_argument("Checkpoint memory aliases, sizes or cache geometry differ");
  }
  auto staged = state.staged ? std::make_unique<detail::StagedState>(*state.staged) : nullptr;
  auto owner = impl_->effect_owner;
  if (staged && !owner) { owner = std::make_shared<const bool>(false); }
  if (impl_->effect_epoch == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Instruction effect generations exhausted");
  }
  impl_->restore_execution(state.execution);
  clear_code_cache();
  ++impl_->effect_epoch;
  impl_->effect_owner = std::move(owner);
  impl_->staged = std::move(staged);
}
} // namespace dsp56300
