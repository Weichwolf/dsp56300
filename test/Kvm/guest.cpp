// SPDX-License-Identifier: GPL-3.0-only
#include "guest.hpp"

#include <array>
#include <asm/kvm.h>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/kvm.h>
#include <memory>
#include <span>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <system_error>
#include <unistd.h>

namespace dsp56300::test::kvm {
namespace {
constexpr std::size_t page_size = 4096, page_entries = 512;
constexpr std::size_t pml4_address = page_size, pdpt_address = 2 * page_size;
constexpr std::size_t pd_address = 3 * page_size, stack_address = 0x1f0000 - 8;
constexpr std::uint64_t present_writable = 3, large_page_flags = 0x83;
constexpr std::uint64_t protected_paging = 0x80000011, pae = 1U << 5;
constexpr std::uint64_t long_mode = (1U << 8) | (1U << 10);
constexpr std::uint32_t segment_limit = 0xffffffff;
constexpr std::uint16_t code_selector = 8, data_selector = 16;
constexpr std::uint8_t code_segment_type = 11, data_segment_type = 3;
constexpr std::uint8_t halt_opcode = 0xf4;
constexpr std::uint32_t cpuid_entries = 256;

struct CpuDescription {
  std::uint32_t count = cpuid_entries, padding = 0;
  std::array<kvm_cpuid_entry2, cpuid_entries> entries{};
};

static_assert(offsetof(CpuDescription, entries) == offsetof(kvm_cpuid2, entries));

[[noreturn]] void fail(const char *operation) {
  throw std::system_error(errno, std::generic_category(), operation);
}

int checked(const int result, const char *operation) {
  if (result < 0) { fail(operation); }
  return result;
}

int open_device() {
  const int fd = open("/dev/kvm", O_RDWR | O_CLOEXEC);
  if (fd < 0 && (errno == EACCES || errno == ENOENT || errno == ENODEV)) {
    throw Unavailable(std::system_error(errno, std::generic_category(), "open /dev/kvm").what());
  }
  checked(fd, "open /dev/kvm");
  const int version = ioctl(fd, KVM_GET_API_VERSION, 0);
  if (version != KVM_API_VERSION) {
    const int error = errno;
    close(fd);
    if (version < 0) {
      throw std::system_error(error, std::generic_category(), "KVM_GET_API_VERSION");
    }
    throw std::runtime_error("Unsupported KVM API version");
  }
  return fd;
}

class Descriptor {
public:
  explicit Descriptor(const int fd) : fd_(fd) {}

  ~Descriptor() { close(fd_); }

  Descriptor(const Descriptor &) = delete;
  Descriptor &operator=(const Descriptor &) = delete;
  Descriptor(Descriptor &&) = delete;
  Descriptor &operator=(Descriptor &&) = delete;

  [[nodiscard]] int get() const noexcept { return fd_; }

private:
  int fd_;
};

class Mapping {
public:
  struct Options {
    int fd = -1;
    std::size_t size = 0;
  };

  explicit Mapping(const Options options) : size_(options.size) {
    data_ = mmap(nullptr,
                 size_,
                 PROT_READ | PROT_WRITE,
                 options.fd < 0 ? MAP_PRIVATE | MAP_ANONYMOUS : MAP_SHARED,
                 options.fd,
                 0);
    if (data_ == MAP_FAILED) { fail("mmap KVM memory"); }
  }

  ~Mapping() { munmap(data_, size_); }

  Mapping(const Mapping &) = delete;
  Mapping &operator=(const Mapping &) = delete;
  Mapping(Mapping &&) = delete;
  Mapping &operator=(Mapping &&) = delete;

  [[nodiscard]] void *data() const noexcept { return data_; }

private:
  void *data_ = nullptr;
  std::size_t size_;
};

kvm_segment segment(const bool code) {
  kvm_segment result{};
  result.limit = segment_limit;
  result.selector = code ? code_selector : data_selector;
  result.type = code ? code_segment_type : data_segment_type;
  result.present = 1;
  result.s = 1;
  result.l = code ? 1 : 0;
  result.db = code ? 0 : 1;
  result.g = 1;
  return result;
}

std::size_t run_mapping_size(const int device) {
  return static_cast<std::size_t>(
      checked(ioctl(device, KVM_GET_VCPU_MMAP_SIZE, 0), "KVM_GET_VCPU_MMAP_SIZE"));
}

void configure_cpu(const int device, const int vcpu) {
  constexpr std::uint32_t paravirtual_features = 0x40000001;
  constexpr std::uint32_t extended_features = 0x80000001;
  constexpr std::uint32_t cpuid_long_mode = 1U << 29;
  CpuDescription model;
  checked(ioctl(device, KVM_GET_SUPPORTED_CPUID, &model), "KVM_GET_SUPPORTED_CPUID");
  if (model.count > model.entries.size()) { throw std::runtime_error("KVM CPUID count overflow"); }
  bool supports_long_mode = false;
  for (auto &entry : std::span(model.entries).first(model.count)) {
    if (entry.function == 1) {
      // No in-kernel local APIC is created by this helper-free experiment.
      constexpr std::uint32_t x2apic = 1U << 21;
      constexpr std::uint32_t tsc_deadline = 1U << 24;
      entry.ecx &= ~(x2apic | tsc_deadline);
    }
    if (entry.function == paravirtual_features) {
      constexpr std::uint32_t pv_unhalt = 1U << 7;
      entry.eax &= ~pv_unhalt;
    }
    if (entry.function == extended_features) {
      supports_long_mode = (entry.edx & cpuid_long_mode) != 0;
    }
  }
  if (!supports_long_mode) { throw Unavailable("KVM does not support x86-64 long mode"); }
  checked(ioctl(vcpu, KVM_SET_CPUID2, &model), "KVM_SET_CPUID2");
}
} // namespace

struct Guest::Impl {
  Descriptor device{open_device()};
  Descriptor vm{checked(ioctl(device.get(), KVM_CREATE_VM, 0), "KVM_CREATE_VM")};
  Descriptor vcpu{checked(ioctl(vm.get(), KVM_CREATE_VCPU, 0), "KVM_CREATE_VCPU")};
  Mapping memory{{.size = memory_size}};
  Mapping run_memory{{.fd = vcpu.get(), .size = run_mapping_size(device.get())}};

  Impl() {
    kvm_userspace_memory_region region{};
    region.memory_size = memory_size;
    region.userspace_addr = reinterpret_cast<std::uintptr_t>(memory.data());
    checked(ioctl(vm.get(), KVM_SET_USER_MEMORY_REGION, &region), "KVM_SET_USER_MEMORY_REGION");
    configure_cpu(device.get(), vcpu.get());
    setup_paging();
  }

  void setup_paging() const {
    const auto bytes = std::span(static_cast<std::byte *>(memory.data()), memory_size);
    const auto entry = [&bytes](const std::size_t offset, const std::uint64_t value) {
      std::memcpy(bytes.subspan(offset, sizeof(value)).data(), &value, sizeof(value));
    };
    entry(pml4_address, pdpt_address | present_writable);
    entry(pdpt_address, pd_address | present_writable);
    // Identity-map 1 GiB in guest page tables. KVM backs only the first 2 MiB;
    // accesses beyond that must exit rather than touch unrelated host memory.
    for (std::size_t i = 0; i < page_entries; ++i) {
      entry(pd_address + (i * sizeof(std::uint64_t)), (i * memory_size) | large_page_flags);
    }
    kvm_sregs state{};
    checked(ioctl(vcpu.get(), KVM_GET_SREGS, &state), "KVM_GET_SREGS");
    state.cs = segment(true);
    state.ds = state.es = state.fs = state.gs = state.ss = segment(false);
    state.cr0 = protected_paging;
    state.cr3 = pml4_address;
    state.cr4 = pae;
    state.efer = long_mode;
    checked(ioctl(vcpu.get(), KVM_SET_SREGS, &state), "KVM_SET_SREGS");
  }
};

Guest::Guest() : impl_(std::make_unique<Impl>()) {}

Guest::~Guest() = default;

void *Guest::address(const std::size_t offset, const std::size_t length) {
  if (offset > memory_size || length > memory_size - offset) {
    throw std::out_of_range("KVM test guest memory range");
  }
  return std::span(static_cast<std::byte *>(impl_->memory.data()), memory_size)
      .subspan(offset, length)
      .data();
}

Exit Guest::run(const std::span<const std::uint8_t> code) {
  if (code.size() > cpu_address - code_address) {
    throw std::out_of_range("KVM test guest code range");
  }
  std::memcpy(address(code_address, code.size()), code.data(), code.size());
  std::memcpy(address(halt_address, sizeof(halt_opcode)), &halt_opcode, sizeof(halt_opcode));
  constexpr std::uint64_t return_address = halt_address;
  std::memcpy(
      address(stack_address, sizeof(return_address)), &return_address, sizeof(return_address));
  kvm_regs state{};
  state.rip = code_address;
  state.rsp = stack_address;
  state.rdi = cpu_address;
  state.rsi = context_address;
  state.rflags = 2;
  checked(ioctl(impl_->vcpu.get(), KVM_SET_REGS, &state), "KVM_SET_REGS");
  int result = 0;
  do { result = ioctl(impl_->vcpu.get(), KVM_RUN, 0); } while (result < 0 && errno == EINTR);
  checked(result, "KVM_RUN");
  checked(ioctl(impl_->vcpu.get(), KVM_GET_REGS, &state), "KVM_GET_REGS");
  const auto &run = *static_cast<const kvm_run *>(impl_->run_memory.data());
  Exit exit{.reason = run.exit_reason, .result = state.rax, .rip = state.rip};
  if (run.exit_reason == KVM_EXIT_MMIO) {
    exit.address = run.mmio.phys_addr;
    exit.length = run.mmio.len;
    exit.write = run.mmio.is_write != 0;
  }
  return exit;
}
} // namespace dsp56300::test::kvm
