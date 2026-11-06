// SPDX-License-Identifier: GPL-3.0-only
#include "cache_model.hpp"
#include "dsp56300/memory.hpp"
#include "program_cache.hpp"
#include "support.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

namespace dsp56300::test {
namespace {
constexpr Word memory_size = 4096, poison = 0x5ac39e;

struct Pair {
  explicit Pair(const unsigned words) : actual(words), model(words), memory(memory_size) {
    for (Word index = 0; index < memory_size; ++index) { memory[index] = index * 3; }
    compare();
  }

  detail::ProgramCache actual;
  cache_model::Model model;
  std::vector<Word> memory;

  void compare() const {
    const auto &sectors = actual.sectors();
    const auto &order = actual.order();
    for (std::size_t slot = 0; slot < cache_model::slots; ++slot) {
      const auto &expected = model.sectors[slot];
      const auto &observed = sectors[slot];
      require(observed.tag == expected.tag && observed.locked == expected.locked,
              "Architectural cache tags and locks follow independent sector map");
      for (Word offset = 0; offset < model.width; ++offset) {
        const auto word = expected.words.find(offset);
        require(observed.valid.test(offset) == (word != expected.words.end()),
                "Every cache word has the independent valid bit");
        if (word != expected.words.end()) {
          require(actual.contents(slot)[offset] == word->second, "Every valid cache word is exact");
        }
      }
      if (slot != 0) {
        require(model.sectors[order[slot - 1]].age > model.sectors[order[slot]].age,
                "Cache MRU/LRU order follows independent access ages");
      }
    }
  }

  void fetch(const Word address, const bool burst = false, const Word end = memory_size) {
    const auto expected = model.fetch(address, memory, burst, end);
    require(actual.fetch(address, memory, burst, end) == expected,
            "Cache fetch equals independent map result");
    compare();
  }

  void lock(const Word address, const bool locked) {
    model.lock(address, locked);
    actual.lock(address, locked);
    compare();
  }

  void write(const cache_model::Write transfer) {
    const auto address = transfer.address;
    model.write(transfer);
    actual.write({.address = address, .value = transfer.value});
    require(actual.peek(address) == model.peek(address), "PMOVE hit data is exact");
    compare();
  }

  void flush(const bool unlocked_only) {
    model.flush(unlocked_only);
    actual.flush(unlocked_only);
    compare();
  }
};

void fetch_and_replace(const unsigned width) {
  Pair pair(width);
  for (const bool burst : {false, true}) {
    for (Word tag = 0; tag < 12; ++tag) {
      for (const Word offset : {0U, 1U, 2U, 3U, width / 2, width - 1}) {
        const auto address = (tag * width) + offset;
        pair.fetch(address, burst);
        pair.memory[address] ^= poison;
        pair.fetch(address, burst);
      }
    }
    for (Word tag = 12; tag > 0; --tag) { pair.fetch(((tag - 1) * width) + 5, burst); }
    pair.flush(true);
  }
  pair.fetch(memory_size);
  pair.fetch(memory_size - 1, true);
  pair.fetch(memory_size - 3, true, memory_size - 2);
}

void lock_masks(const unsigned width) {
  for (unsigned mask = 0; mask < 256; ++mask) {
    Pair pair(width);
    for (Word tag = 0; tag < cache_model::slots; ++tag) {
      pair.fetch(tag * width);
      if ((mask & (1U << tag)) != 0) { pair.lock(tag * width, true); }
    }
    pair.flush(true);
    for (Word tag = 0; tag < cache_model::slots; ++tag) {
      pair.memory[static_cast<std::size_t>(tag) * width] ^= poison;
      pair.fetch(tag * width);
    }
    pair.model.free();
    pair.actual.free();
    pair.compare();
    pair.flush(false);
  }
}

void all_locked(const unsigned width) {
  Pair pair(width);
  for (Word tag = 0; tag < cache_model::slots; ++tag) { pair.lock(tag * width, true); }
  pair.lock(width * 9, true);
  pair.lock(width * 9, false);
  pair.fetch(width * 9, true);
  pair.write({.address = width * 9, .value = poison});
  pair.lock(width * 4, false);
  pair.fetch(width * 9, true);
  pair.lock(width * 2, true);
  pair.flush(true);
  pair.flush(false);
}

void program_data(const unsigned width) {
  Pair pair(width);
  for (Word offset = 0; offset < width; ++offset) {
    pair.write({.address = offset, .value = poison});
    pair.lock(0, true);
    pair.write({.address = offset, .value = poison + offset});
    require(pair.actual.peek(offset) == pair.model.peek(offset), "PMOVE lookup preserves state");
    pair.compare();
    pair.fetch(offset);
    pair.memory[offset] = offset;
  }
  pair.lock(word_mask, true);
  pair.write({.address = word_mask, .value = poison});
  require(pair.actual.peek(word_mask) == std::optional<Word>{poison},
          "Top 24-bit sector and offset are bounded");
  pair.compare();
  pair.flush(false);
}

void reader_transfers(const unsigned width) {
  struct Source {
    std::vector<Word> requests;
    std::optional<Word> reject;
  } source;

  const auto read = [](void *context, const Word address) -> std::optional<Word> {
    auto &input = *static_cast<Source *>(context);
    input.requests.push_back(address);
    if (input.reject == address) { return std::nullopt; }
    return 0x1000000U | address;
  };
  detail::ProgramCache cache(width);
  require(cache.fetch(5, 32, true, 32, read, &source) == 5,
          "Reader burst returns masked first word");
  require(source.requests == std::vector<Word>{5, 6, 7}, "Reader observes exact burst order");
  source.requests.clear();
  require(cache.fetch(6, 32, true, 32, read, &source) == 6 && source.requests.empty(),
          "Reader cache hit issues no transfer");
  source.reject = 10;
  require(!cache.fetch(9, 32, true, 32, read, &source), "Reader refusal is observable");
  require(source.requests == std::vector<Word>{9, 10}, "Burst stops at refused word");
  require(cache.peek(9) == 9 && !cache.peek(10), "Only accepted burst words become valid");
  source.requests.clear();
  require(!cache.fetch(32, 32, false, 32, read, &source) && source.requests.empty(),
          "Out-of-range fetch performs no transfer");
}

void replay_burst(const unsigned width) {
  struct Pause : std::exception {};

  struct Source {
    std::vector<Word> accepted;
    std::vector<Word> requested;
    std::size_t cursor = 0;
  } source;

  const auto read = [](void *context, const Word address) -> std::optional<Word> {
    auto &input = *static_cast<Source *>(context);
    if (input.cursor < input.accepted.size()) {
      require(input.accepted[input.cursor++] == address, "Replay consumes accepted reads in order");
      return address + 0x100;
    }
    input.requested.push_back(address);
    throw Pause{};
  };
  const detail::ProgramCache base(width);
  for (const Word expected : {4U, 5U, 6U, 7U}) {
    auto candidate = base;
    source.cursor = 0;
    try {
      static_cast<void>(candidate.fetch(4, 32, true, 32, read, &source));
      require(false, "Incomplete burst must pause");
    } catch (const Pause &) {
      require(source.requested.back() == expected, "Next request advances through burst");
      source.accepted.push_back(expected);
    }
  }
  auto candidate = base;
  source.cursor = 0;
  constexpr Word first_word = 0x104;
  constexpr Word last_word = 0x107;
  require(candidate.fetch(4, 32, true, 32, read, &source) == first_word,
          "Completed replay returns the first fetched word");
  require(source.requested == std::vector<Word>{4, 5, 6, 7},
          "Replay never repeats an external request");
  require(candidate.peek(7) == last_word, "Completed replay contains the last burst word");
}
} // namespace
} // namespace dsp56300::test

int main() {
  try {
    for (const unsigned width : {128U, 256U}) {
      dsp56300::test::fetch_and_replace(width);
      dsp56300::test::lock_masks(width);
      dsp56300::test::all_locked(width);
      dsp56300::test::program_data(width);
      dsp56300::test::reader_transfers(width);
      dsp56300::test::replay_burst(width);
    }
    std::cout << dsp56300::test::checks << " independent cache storage checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
