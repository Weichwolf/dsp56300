// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "a64_loop_state.hpp"
#include "cache_model.hpp"
#include "cache_state.hpp"
#include "dsp56300/memory.hpp"
#include "program_cache.hpp"
#include "support.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dsp56300::test::a64 {
struct CacheSeed {
  unsigned width = 128, locks = 0;
};

class CacheStorage {
public:
  CacheStorage(loop_model::Runner &runner, const CacheSeed input)
      : model_(input.width), actual_(input.width), runner_(runner) {
    const auto width = input.width;
    const auto locks = input.locks;
    std::array<Word, memory_size> memory{};
    for (Word index = 0; index < memory_size; ++index) { memory[index] = (index * 3) + 1; }
    for (Word tag = 0; tag < cache_model::slots; ++tag) {
      for (const Word offset : {0U, 63U, 64U, width - 1}) {
        const auto address = (tag * width) + offset;
        require(actual_.fetch(address, memory, false, memory.size()) ==
                    model_.fetch(address, memory, false, memory.size()),
                "Native cache fixtures start with independently authored resident words");
      }
      if ((locks & (1U << tag)) != 0) {
        actual_.lock(tag * width, true);
        model_.lock(tag * width, true);
      }
    }
    for (std::size_t slot = 0; slot < cache_model::slots; ++slot) {
      const auto contents = actual_.contents(slot);
      before_[slot].assign(contents.begin(), contents.end());
    }
    runner_.cache(this, storage);
  }

  void visible(const loop_model::State &state) { visible_ = state; }

  void expect(const unsigned calls) const {
    require(valid_ && calls_ == calls,
            "Only valid enabled native controls obtain cache storage, with canonical CPU state");
    for (std::size_t slot = 0; slot < cache_model::slots; ++slot) {
      const auto &observed = actual_.sectors()[slot];
      const auto &expected = model_.sectors[slot];
      require(observed.tag == expected.tag && observed.locked == expected.locked,
              "Native cache tags and lock bits match the independent map");
      for (Word offset = 0; offset < model_.width; ++offset) {
        require(observed.valid.test(offset) == expected.words.contains(offset),
                "Every native cache validity bit matches the independent map");
      }
      require(std::ranges::equal(actual_.contents(slot), before_[slot]),
              "Cache controls preserve physical cached payload words");
      if (slot != 0) {
        const auto &order = actual_.order();
        require(model_.sectors[order[slot - 1]].age > model_.sectors[order[slot]].age,
                "Native LRU permutation agrees with independent access ages");
      }
    }
  }

  [[nodiscard]] cache_model::Model &model() { return model_; }

private:
  static detail::CacheState *storage(void *const owner) noexcept {
    auto &self = *static_cast<CacheStorage *>(owner);
    ++self.calls_;
    auto physical = self.visible_.reg;
    constexpr unsigned padding = 8;
    physical.a <<= padding;
    physical.b <<= padding;
    const auto &cpu = self.runner_.cpu();
    self.valid_ =
        self.valid_ && cpu.reg == physical && cpu.instructions == self.visible_.instructions &&
        cpu.cycles == self.visible_.cycles &&
        cpu.repeat_remaining == self.visible_.repeat.remaining &&
        cpu.repeat_pc == self.visible_.repeat.pc && cpu.repeat_lc == self.visible_.repeat.lc &&
        cpu.repeat_opcode == self.visible_.repeat.opcode &&
        cpu.repeat_loaded == self.visible_.repeat.loaded &&
        cpu.repeat_completed == self.visible_.completed && !cpu.interrupt_inhibited &&
        cpu.processing == self.visible_.processing && cpu.debug_flags == self.visible_.debug_flags;
    return &self.actual_.state();
  }

  static constexpr Word memory_size = 4096;
  cache_model::Model model_;
  detail::ProgramCache actual_;
  loop_model::Runner &runner_;
  loop_model::State visible_;
  std::array<std::vector<Word>, cache_model::slots> before_;
  unsigned calls_ = 0;
  bool valid_ = true;
};
}
