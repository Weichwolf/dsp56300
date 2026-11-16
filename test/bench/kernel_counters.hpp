// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#if defined(__linux__)
#include <linux/perf_event.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace core_kernels {
struct Counts {
  std::uint64_t cycles = 0, instructions = 0, enabled_ns = 0, running_ns = 0;
};

// Current-thread user-space PMCs, never scaled or mixed with preparation.
class Counters {
public:
  Counters() noexcept {
#if defined(__linux__)
    perf_event_attr event{};
    event.size = sizeof(event);
    event.type = PERF_TYPE_HARDWARE;
    event.config = PERF_COUNT_HW_CPU_CYCLES;
    event.disabled = 1;
    event.exclude_kernel = 1;
    event.exclude_hv = 1;
    event.read_format =
        PERF_FORMAT_GROUP | PERF_FORMAT_TOTAL_TIME_ENABLED | PERF_FORMAT_TOTAL_TIME_RUNNING;
    leader_ = static_cast<int>(syscall(SYS_perf_event_open, &event, 0, -1, -1, 0));
    if (leader_ < 0) { return; }
    event.config = PERF_COUNT_HW_INSTRUCTIONS;
    member_ = static_cast<int>(syscall(SYS_perf_event_open, &event, 0, -1, leader_, 0));
#endif
  }

  ~Counters() {
#if defined(__linux__)
    if (member_ >= 0) { close(member_); }
    if (leader_ >= 0) { close(leader_); }
#endif
  }

  Counters(const Counters &) = delete;
  Counters &operator=(const Counters &) = delete;

  void start() noexcept {
#if defined(__linux__)
    valid_ = member_ >= 0 && ioctl(leader_, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP) == 0 &&
             ioctl(leader_, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP) == 0;
#endif
  }

  [[nodiscard]] std::optional<Counts> stop() const noexcept {
#if defined(__linux__)
    if (!valid_ || ioctl(leader_, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP) != 0) {
      return std::nullopt;
    }
    // nr, time_enabled, time_running, cycles, instructions; group insertion order.
    std::array<std::uint64_t, 5> values{};
    if (read(leader_, values.data(), sizeof(values)) != sizeof(values) || values[0] != 2 ||
        values[1] == 0 || values[1] != values[2]) {
      return std::nullopt;
    }
    return Counts{.cycles = values[3],
                  .instructions = values[4],
                  .enabled_ns = values[1],
                  .running_ns = values[2]};
#else
    return std::nullopt;
#endif
  }

private:
#if defined(__linux__)
  int leader_ = -1, member_ = -1;
  bool valid_ = false;
#endif
};
}
