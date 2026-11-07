// SPDX-License-Identifier: GPL-3.0-only
#include "spectral_host_asm.hpp"

#include "dsp56300/memory.hpp"
#include "fft_host.hpp"
#include "support.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

// These assembler macros are local to each expanded block and purged afterwards.
#define SPECTRAL_VECTOR_ROUND                                                                      \
  ".macro spectral_round value scratch bits rm\n\t"                                                \
  ".if \\rm\n\t"                                                                                   \
  ".if \\bits == 1\n\t"                                                                            \
  "vpaddq %%ymm12,\\value,\\value\n\t"                                                             \
  ".else\n\t"                                                                                      \
  "vpaddq %%ymm13,\\value,\\value\n\t"                                                             \
  ".endif\n\t"                                                                                     \
  ".else\n\t"                                                                                      \
  "vpsrlq $\\bits,\\value,\\scratch\n\t"                                                           \
  "vpand %%ymm12,\\scratch,\\scratch\n\t"                                                          \
  ".if \\bits != 1\n\t"                                                                            \
  "vpaddq %%ymm13,\\scratch,\\scratch\n\t"                                                         \
  ".endif\n\t"                                                                                     \
  "vpaddq \\scratch,\\value,\\value\n\t"                                                           \
  ".endif\n\t"                                                                                     \
  ".if \\bits == 1\n\t"                                                                            \
  "vpsrad $1,\\value,\\value\n\t"                                                                  \
  ".else\n\t"                                                                                      \
  "vpsrad $\\bits,\\value,\\scratch\n\t"                                                           \
  "vpsrlq $\\bits,\\value,\\value\n\t"                                                             \
  "vpblendd $0xaa,\\scratch,\\value,\\value\n\t"                                                   \
  ".endif\n\t"                                                                                     \
  ".endm\n\t"

namespace dsp56300::test::fft::host {
namespace {
constexpr unsigned host_word_bits = 32;

struct alignas(32) Constants {
  std::array<std::int64_t, lane_count> one, bias23, low, high;
};

template <bool Rm>
constexpr Constants constants{.one = {1, 1, 1, 1},
                              .bias23 = {(scale / 2) - (Rm ? 0 : 1),
                                         (scale / 2) - (Rm ? 0 : 1),
                                         (scale / 2) - (Rm ? 0 : 1),
                                         (scale / 2) - (Rm ? 0 : 1)},
                              .low = {-scale, -scale, -scale, -scale},
                              .high = {scale - 1, scale - 1, scale - 1, scale - 1}};

struct Butterfly {
  std::int64_t *real, *imaginary, *upper_real, *upper_imaginary;
  const std::int64_t *weight_real, *weight_imaginary;
  std::size_t count;
};

template <bool Rm>
[[gnu::target("avx2"), gnu::always_inline]] inline void vector_columns(const Butterfly &job) {
  std::size_t offset;
  asm volatile(SPECTRAL_VECTOR_ROUND R"(
    vmovdqa %c[one](%[constants]),%%ymm12
    vmovdqa %c[bias](%[constants]),%%ymm13
    vmovdqa %c[low](%[constants]),%%ymm14
    vmovdqa %c[high](%[constants]),%%ymm15
  1:
    vmovdqu (%[real],%[offset]),%%ymm0
    vmovdqu (%[imaginary],%[offset]),%%ymm1
    vmovdqu (%[upper_real],%[offset]),%%ymm2
    vmovdqu (%[upper_imaginary],%[offset]),%%ymm3
    vpsubq %%ymm2,%%ymm0,%%ymm4
    vpsubq %%ymm3,%%ymm1,%%ymm5
    vpaddq %%ymm2,%%ymm0,%%ymm6
    vpaddq %%ymm3,%%ymm1,%%ymm7
    spectral_round %%ymm6,%%ymm8,1,%c[rm]
    spectral_round %%ymm7,%%ymm8,1,%c[rm]
    vmovdqu %%ymm6,(%[real],%[offset])
    vmovdqu %%ymm7,(%[imaginary],%[offset])
    spectral_round %%ymm4,%%ymm8,1,%c[rm]
    spectral_round %%ymm5,%%ymm8,1,%c[rm]
    vpminsd %%ymm15,%%ymm4,%%ymm4
    vpminsd %%ymm15,%%ymm5,%%ymm5
    vmovdqu (%[weight_real],%[offset]),%%ymm6
    vmovdqu (%[weight_imaginary],%[offset]),%%ymm7
    vpmuldq %%ymm6,%%ymm4,%%ymm0
    vpmuldq %%ymm7,%%ymm5,%%ymm1
    vpsubq %%ymm1,%%ymm0,%%ymm0
    vpmuldq %%ymm7,%%ymm4,%%ymm2
    vpmuldq %%ymm6,%%ymm5,%%ymm3
    vpaddq %%ymm3,%%ymm2,%%ymm2
    spectral_round %%ymm0,%%ymm8,23,%c[rm]
    spectral_round %%ymm2,%%ymm8,23,%c[rm]
    vpmaxsd %%ymm14,%%ymm0,%%ymm0
    vpminsd %%ymm15,%%ymm0,%%ymm0
    vpmaxsd %%ymm14,%%ymm2,%%ymm2
    vpminsd %%ymm15,%%ymm2,%%ymm2
    vmovdqu %%ymm0,(%[upper_real],%[offset])
    vmovdqu %%ymm2,(%[upper_imaginary],%[offset])
    addq $32,%[offset]
    cmpq %[bytes],%[offset]
    jb 1b
    .purgem spectral_round
  )"
               : [offset] "=&r"(offset)
               : "0"(std::size_t{0}),
                 [real] "r"(job.real),
                 [imaginary] "r"(job.imaginary),
                 [upper_real] "r"(job.upper_real),
                 [upper_imaginary] "r"(job.upper_imaginary),
                 [weight_real] "r"(job.weight_real),
                 [weight_imaginary] "r"(job.weight_imaginary),
                 [bytes] "r"(job.count * sizeof(std::int64_t)),
                 [constants] "r"(&constants<Rm>),
                 [one] "i"(offsetof(Constants, one)),
                 [bias] "i"(offsetof(Constants, bias23)),
                 [low] "i"(offsetof(Constants, low)),
                 [high] "i"(offsetof(Constants, high)),
                 [rm] "i"(Rm)
               : "cc",
                 "memory",
                 "ymm0",
                 "ymm1",
                 "ymm2",
                 "ymm3",
                 "ymm4",
                 "ymm5",
                 "ymm6",
                 "ymm7",
                 "ymm8",
                 "ymm12",
                 "ymm13",
                 "ymm14",
                 "ymm15");
}

template <bool Rm> inline void scalar_column(const Butterfly &job) {
  asm volatile(R"(
    .macro spectral_half value temporary
    .if %c[rm]
      addq $1,\value
    .else
      movq \value,\temporary
      sarq $1,\temporary
      andq $1,\temporary
      addq \temporary,\value
    .endif
      sarq $1,\value
    .endm
    .macro spectral_fraction value temporary
    .if %c[rm]
      addq %[half],\value
    .else
      movq \value,\temporary
      sarq $23,\temporary
      andq $1,\temporary
      addq %[bias],\temporary
      addq \temporary,\value
    .endif
      sarq $23,\value
      cmpq %%r10,\value
      cmovgq %%r10,\value
      cmpq %%r11,\value
      cmovlq %%r11,\value
    .endm
    movq (%[real]),%%r8
    movq (%[imaginary]),%%r9
    movq (%[upper_real]),%%r10
    movq (%[upper_imaginary]),%%r11
    leaq (%%r8,%%r10),%%rax
    spectral_half %%rax,%%rbx
    movq %%rax,(%[real])
    leaq (%%r9,%%r11),%%rax
    spectral_half %%rax,%%rbx
    movq %%rax,(%[imaginary])
    subq %%r10,%%r8
    subq %%r11,%%r9
    spectral_half %%r8,%%rbx
    spectral_half %%r9,%%rbx
    movq %[maximum],%%rbx
    cmpq %%rbx,%%r8
    cmovgq %%rbx,%%r8
    cmpq %%rbx,%%r9
    cmovgq %%rbx,%%r9
    movq (%[weight_real]),%%r10
    movq (%[weight_imaginary]),%%r11
    movq %%r8,%%rax
    imulq %%r10,%%rax
    movq %%r9,%%rbx
    imulq %%r11,%%rbx
    subq %%rbx,%%rax
    imulq %%r11,%%r8
    imulq %%r10,%%r9
    leaq (%%r8,%%r9),%%r12
    movq %[maximum],%%r10
    movq %[minimum],%%r11
    spectral_fraction %%rax,%%rbx
    spectral_fraction %%r12,%%rbx
    movq %%rax,(%[upper_real])
    movq %%r12,(%[upper_imaginary])
    .purgem spectral_half
    .purgem spectral_fraction
  )"
               :
               : [real] "r"(job.real),
                 [imaginary] "r"(job.imaginary),
                 [upper_real] "r"(job.upper_real),
                 [upper_imaginary] "r"(job.upper_imaginary),
                 [weight_real] "r"(job.weight_real),
                 [weight_imaginary] "r"(job.weight_imaginary),
                 [rm] "i"(Rm),
                 [maximum] "i"(scale - 1),
                 [minimum] "i"(-scale),
                 [half] "i"(scale / 2),
                 [bias] "i"((scale / 2) - 1)
               : "cc", "memory", "rax", "rbx", "r8", "r9", "r10", "r11", "r12");
}

void permute(Data &data) {
  const auto shift = host_word_bits - static_cast<unsigned>(std::countr_zero(data.real.size()));
  std::size_t offset;
  asm volatile(R"(
  1:
    movl %k[index],%%eax
    bswapl %%eax
    movl %%eax,%%edx
    shrl $4,%%edx
    andl $0x0f0f0f0f,%%edx
    andl $0x0f0f0f0f,%%eax
    shll $4,%%eax
    orl %%edx,%%eax
    movl %%eax,%%edx
    shrl $2,%%edx
    andl $0x33333333,%%edx
    andl $0x33333333,%%eax
    shll $2,%%eax
    orl %%edx,%%eax
    movl %%eax,%%edx
    shrl $1,%%edx
    andl $0x55555555,%%edx
    andl $0x55555555,%%eax
    shll $1,%%eax
    orl %%edx,%%eax
    shrl %%cl,%%eax
    movq (%[real],%%rax,8),%%rdx
    movq (%[imaginary],%%rax,8),%%r8
    movq %%rdx,(%[output_real],%[index],8)
    movq %%r8,(%[output_imaginary],%[index],8)
    incq %[index]
    cmpq %[size],%[index]
    jb 1b
  )"
               : [index] "=&r"(offset)
               : "0"(std::size_t{0}),
                 [real] "r"(data.real.data()),
                 [imaginary] "r"(data.imaginary.data()),
                 [output_real] "r"(data.output_real.data()),
                 [output_imaginary] "r"(data.output_imaginary.data()),
                 [size] "r"(data.real.size()),
                 "c"(shift)
               : "cc", "memory", "rax", "rdx", "r8");
}

template <bool Rm>
[[gnu::target("avx2")]] void stages(Data &data, const std::span<const Stage> plan) {
  for (const auto &stage : plan) {
    for (std::size_t group = 0; group < data.real.size(); group += 2 * stage.half) {
      Butterfly job{.real = data.real.data() + group,
                    .imaginary = data.imaginary.data() + group,
                    .upper_real = data.real.data() + group + stage.half,
                    .upper_imaginary = data.imaginary.data() + group + stage.half,
                    .weight_real = stage.real.data(),
                    .weight_imaginary = stage.imaginary.data(),
                    .count = stage.half};
      if (stage.half >= lane_count) {
        vector_columns<Rm>(job);
      } else {
        for (std::size_t column = 0; column < stage.half; ++column) {
          scalar_column<Rm>(job);
          ++job.real;
          ++job.imaginary;
          ++job.upper_real;
          ++job.upper_imaginary;
          ++job.weight_real;
          ++job.weight_imaginary;
        }
      }
    }
  }
  asm volatile("vzeroupper" ::
                   : "ymm0",
                     "ymm1",
                     "ymm2",
                     "ymm3",
                     "ymm4",
                     "ymm5",
                     "ymm6",
                     "ymm7",
                     "ymm8",
                     "ymm9",
                     "ymm10",
                     "ymm11",
                     "ymm12",
                     "ymm13",
                     "ymm14",
                     "ymm15");
}
}

void transform_asm(Data &data, const std::span<const Stage> plan, const bool rm) {
  if (rm) {
    stages<true>(data, plan);
  } else {
    stages<false>(data, plan);
  }
  permute(data);
}

namespace {
template <bool Rm> [[gnu::target("avx2")]] void window_words(const Window &job) {
  std::size_t offset;
  asm volatile(
      SPECTRAL_VECTOR_ROUND R"(
    vmovdqa %c[one](%[constants]),%%ymm12
    vmovdqa %c[bias](%[constants]),%%ymm13
    vmovdqa %c[low](%[constants]),%%ymm14
    vmovdqa %c[high](%[constants]),%%ymm15
  1:
    vmovdqu (%[real],%[index],4),%%xmm0
    vmovdqu (%[imaginary],%[index],4),%%xmm1
    vmovdqu (%[weights],%[index],4),%%xmm2
    vpslld $8,%%ymm0,%%ymm0
    vpslld $8,%%ymm1,%%ymm1
    vpslld $8,%%ymm2,%%ymm2
    vpsrad $8,%%ymm0,%%ymm0
    vpsrad $8,%%ymm1,%%ymm1
    vpsrad $8,%%ymm2,%%ymm2
    vpmovsxdq %%xmm0,%%ymm0
    vpmovsxdq %%xmm1,%%ymm1
    vpmovsxdq %%xmm2,%%ymm2
    vpmuldq %%ymm2,%%ymm0,%%ymm0
    vpmuldq %%ymm2,%%ymm1,%%ymm1
    spectral_round %%ymm0,%%ymm8,23,%c[rm]
    spectral_round %%ymm1,%%ymm8,23,%c[rm]
    vpmaxsd %%ymm14,%%ymm0,%%ymm0
    vpminsd %%ymm15,%%ymm0,%%ymm0
    vpmaxsd %%ymm14,%%ymm1,%%ymm1
    vpminsd %%ymm15,%%ymm1,%%ymm1
    vmovdqu %%ymm0,(%[output_real],%[index],8)
    vmovdqu %%ymm1,(%[output_imaginary],%[index],8)
    addq $4,%[index]
    cmpq %[count],%[index]
    jb 1b
    vzeroupper
    .purgem spectral_round
  )"
      : [index] "=&r"(offset)
      : "0"(std::size_t{0}),
        [real] "r"(job.real),
        [imaginary] "r"(job.imaginary),
        [weights] "r"(job.weights),
        [output_real] "r"(job.output_real),
        [output_imaginary] "r"(job.output_imaginary),
        [count] "r"(job.count),
        [constants] "r"(&constants<Rm>),
        [rm] "i"(Rm),
        [one] "i"(offsetof(Constants, one)),
        [bias] "i"(offsetof(Constants, bias23)),
        [low] "i"(offsetof(Constants, low)),
        [high] "i"(offsetof(Constants, high))
      : "cc", "memory", "ymm0", "ymm1", "ymm2", "ymm8", "ymm12", "ymm13", "ymm14", "ymm15");
}

template <bool Rm> std::size_t power_words(const Power &job) {
  std::size_t offset;
  std::size_t cursor;
  asm volatile(R"(
  1:
    movq %c[real](%[job]),%%r10
    movq (%%r10,%[index],8),%%rax
    andl %[mask],%%eax
    movq %c[stored_real](%[job]),%%r10
    movl %%eax,(%%r10,%[index],4)
    movq %c[imaginary](%[job]),%%r10
    movq (%%r10,%[index],8),%%rax
    andl %[mask],%%eax
    movq %c[stored_imaginary](%[job]),%%r10
    movl %%eax,(%%r10,%[index],4)
    movq %c[output_real](%[job]),%%r10
    movq (%%r10,%[index],8),%%r8
    movl %%r8d,%%eax
    andl %[mask],%%eax
    movq %c[stored_output_real](%[job]),%%r10
    movl %%eax,(%%r10,%[index],4)
    movq %c[output_imaginary](%[job]),%%r10
    movq (%%r10,%[index],8),%%r9
    movl %%r9d,%%eax
    andl %[mask],%%eax
    movq %c[stored_output_imaginary](%[job]),%%r10
    movl %%eax,(%%r10,%[index],4)
    imulq %%r8,%%r8
    imulq %%r9,%%r9
    addq %%r9,%%r8
    .if %c[rm]
      addq %[half],%%r8
    .else
      movq %%r8,%%rax
      shrq $24,%%rax
      andl $1,%%eax
      addq %[bias],%%rax
      addq %%rax,%%r8
    .endif
    shrq $24,%%r8
    movq %[maximum],%%rax
    cmpq %%rax,%%r8
    cmovgq %%rax,%%r8
    movq %c[power](%[job]),%%r10
    movl %%r8d,(%%r10,%[cursor],4)
    incq %[cursor]
    cmpq %[ring_words],%[cursor]
    jb 2f
    xorq %[cursor],%[cursor]
  2:
    incq %[index]
    cmpq %[count],%[index]
    jb 1b
  )"
               : [index] "=&r"(offset), [cursor] "=&r"(cursor)
               : "0"(std::size_t{0}),
                 "1"(job.position),
                 [job] "r"(&job),
                 [count] "r"(job.count),
                 [ring_words] "r"(job.ring_words),
                 [real] "i"(offsetof(Power, real)),
                 [imaginary] "i"(offsetof(Power, imaginary)),
                 [output_real] "i"(offsetof(Power, output_real)),
                 [output_imaginary] "i"(offsetof(Power, output_imaginary)),
                 [stored_real] "i"(offsetof(Power, stored_real)),
                 [stored_imaginary] "i"(offsetof(Power, stored_imaginary)),
                 [stored_output_real] "i"(offsetof(Power, stored_output_real)),
                 [stored_output_imaginary] "i"(offsetof(Power, stored_output_imaginary)),
                 [power] "i"(offsetof(Power, power)),
                 [mask] "i"(word_mask),
                 [rm] "i"(Rm),
                 [half] "i"(scale),
                 [bias] "i"(scale - 1),
                 [maximum] "i"(scale - 1)
               : "cc", "memory", "rax", "r8", "r9", "r10");
  return cursor;
}
}

void window_asm(const Window &job, const bool rm) {
  require(job.count % lane_count == 0, "ASM window needs complete four-word groups");
  if (job.count == 0) { return; }
  if (rm) {
    window_words<true>(job);
  } else {
    window_words<false>(job);
  }
}

std::size_t power_asm(const Power &job, const bool rm) {
  require(job.ring_words != 0 && job.position < job.ring_words, "ASM power ring extent");
  if (job.count == 0) { return job.position; }
  return rm ? power_words<true>(job) : power_words<false>(job);
}

}

#undef SPECTRAL_VECTOR_ROUND
