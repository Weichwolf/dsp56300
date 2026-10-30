// SPDX-License-Identifier: GPL-3.0-only
#include "execution_state.hpp"

#include "core_impl.hpp"
#include "dsp56300/core.hpp"
#include "program_cache.hpp"

#include <memory>
#include <optional>
#include <utility>

namespace dsp56300 {
detail::ExecutionState Core::Impl::execution_state() const {
  return {.cpu = cpu,
          .latched = latched,
          .current_pc = current_pc,
          .current_words = current_words,
          .repeated = repeated,
          .fault = fault,
          .vector = vector,
          .debug_vector = debug_vector,
          .cache = icache ? std::optional(*icache) : std::nullopt};
}

void Core::Impl::restore_execution(const detail::ExecutionState &state) {
  auto cache = state.cache ? std::make_unique<detail::ProgramCache>(*state.cache) : nullptr;
  cpu = state.cpu;
  latched = state.latched;
  current_pc = state.current_pc;
  current_words = state.current_words;
  repeated = state.repeated;
  fault = state.fault;
  vector = state.vector;
  debug_vector = state.debug_vector;
  icache = std::move(cache);
}
} // namespace dsp56300
