// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <signal.h> // IWYU pragma: export
#include <stdexcept>
#include <system_error>
#include <ucontext.h>
#include <unistd.h>

namespace dsp56300::test {
// The consumer owns handlers and restores uc_sigmask before the terminal escape.
class NativeFaults {
public:
  using Claim = bool (*)(std::uintptr_t, bool, std::uintptr_t) noexcept;

  class InterruptedMask {
  public:
    InterruptedMask() {
      sigset_t blocked{};
      sigemptyset(&blocked);
      sigaddset(&blocked, SIGUSR1);
      const auto error = pthread_sigmask(SIG_BLOCK, &blocked, &previous_);
      if (error != 0) {
        throw std::system_error(error, std::generic_category(), "Block test signal");
      }
    }

    ~InterruptedMask() {
      if (pthread_sigmask(SIG_SETMASK, &previous_, nullptr) != 0) { _exit(5); }
    }

    InterruptedMask(const InterruptedMask &) = delete;
    InterruptedMask &operator=(const InterruptedMask &) = delete;

    [[nodiscard]] bool restored() const noexcept {
      sigset_t current{};
      return pthread_sigmask(SIG_SETMASK, nullptr, &current) == 0 &&
             sigismember(&current, SIGUSR1) == 1 &&
             sigismember(&current, SIGUSR2) == sigismember(&previous_, SIGUSR2);
    }

  private:
    sigset_t previous_{};
  };

  explicit NativeFaults(Claim claim) : claim_(claim) {
    if (active_ != nullptr) { throw std::logic_error("Test fault handler already installed"); }
    struct sigaction action{};
    action.sa_sigaction = handle;
    action.sa_flags = SA_SIGINFO;
    sigemptyset(&action.sa_mask);
    sigaddset(&action.sa_mask, SIGUSR2);
    for (std::size_t index = 0; index < signals.size(); ++index) {
      if (sigaction(signals[index], &action, &previous_[index]) != 0) {
        const auto reason = errno;
        while (index != 0) {
          --index;
          sigaction(signals[index], &previous_[index], nullptr);
        }
        throw std::system_error(reason, std::generic_category(), "Install test fault handler");
      }
    }
    active_ = this;
  }

  ~NativeFaults() {
    for (std::size_t index = 0; index < signals.size(); ++index) {
      sigaction(signals[index], &previous_[index], nullptr);
    }
    active_ = nullptr;
  }

  NativeFaults(const NativeFaults &) = delete;
  NativeFaults &operator=(const NativeFaults &) = delete;

private:
  static void chain(const int signal, siginfo_t *info, void *context) noexcept {
    const auto &prior = active_->previous_[signal == SIGSEGV ? 0U : 1U];
    if ((prior.sa_flags & SA_SIGINFO) != 0) {
      prior.sa_sigaction(signal, info, context);
      return;
    }
    if (prior.sa_handler == SIG_IGN) { return; }
    if (prior.sa_handler != SIG_DFL) {
      prior.sa_handler(signal);
      return;
    }
    sigaction(signal, &prior, nullptr);
    static_cast<void>(raise(signal));
  }

  static void handle(const int signal, siginfo_t *info, void *context) noexcept {
    const auto *state = static_cast<ucontext_t *>(context);
#if defined(__linux__) && defined(__x86_64__)
    const auto pc = static_cast<std::uintptr_t>(state->uc_mcontext.gregs[REG_RIP]);
    const bool writing = (state->uc_mcontext.gregs[REG_ERR] & 2) != 0;
#elif defined(__APPLE__) && defined(__aarch64__)
    const auto pc =
        static_cast<std::uintptr_t>(__darwin_arm_thread_state64_get_pc(state->uc_mcontext->__ss));
    const bool writing = (state->uc_mcontext->__es.__esr & (1U << 6)) != 0;
#elif defined(__APPLE__) && defined(__x86_64__)
    const auto pc = static_cast<std::uintptr_t>(state->uc_mcontext->__ss.__rip);
    const bool writing = (state->uc_mcontext->__es.__err & 2) != 0;
#else
#error "The native fault fixture needs this host's ucontext register layout"
#endif
    sigset_t handler_mask{};
    const auto saved_errno = errno;
    if (pthread_sigmask(SIG_SETMASK, &state->uc_sigmask, &handler_mask) != 0) { _exit(2); }
    if (active_->claim_(reinterpret_cast<std::uintptr_t>(info->si_addr), writing, pc)) { _exit(3); }
    if (pthread_sigmask(SIG_SETMASK, &handler_mask, nullptr) != 0) { _exit(4); }
    errno = saved_errno;
    chain(signal, info, context);
  }

  static constexpr std::array signals{SIGSEGV, SIGBUS};
  std::array<struct sigaction, 2> previous_{};
  Claim claim_;
  static inline NativeFaults *active_ = nullptr;
};
}
