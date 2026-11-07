// SPDX-License-Identifier: GPL-3.0-only
#include "numeric_host_asm.hpp"

#include "dsp56300/memory.hpp"
#include "iir_stream_program.hpp"
#include "nonlinear_stream_program.hpp"
#include "support.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

// Assembler macros are local to each expanded kernel and purged before returning.
#define NUMERIC_LOAD                                                                               \
  ".macro numeric_load base offset value\n"                                                        \
  "movslq \\offset(\\base),\\value\n"                                                              \
  "shlq $40,\\value\n"                                                                             \
  "sarq $40,\\value\n"                                                                             \
  ".endm\n"

#define NUMERIC_QUANTIZE                                                                           \
  ".macro numeric_quantize value scratch bits rm low high\n"                                       \
  ".if \\rm\n"                                                                                     \
  "addq $(1<<(\\bits-1)),\\value\n"                                                                \
  ".else\n"                                                                                        \
  "movq \\value,\\scratch\n"                                                                       \
  "shrq $\\bits,\\scratch\n"                                                                       \
  "andq $1,\\scratch\n"                                                                            \
  "leaq ((1<<(\\bits-1))-1)(\\value,\\scratch),\\value\n"                                          \
  ".endif\n"                                                                                       \
  "sarq $\\bits,\\value\n"                                                                         \
  "movq $\\low,\\scratch\n"                                                                        \
  "cmpq \\scratch,\\value\n"                                                                       \
  "cmovlq \\scratch,\\value\n"                                                                     \
  "movq $\\high,\\scratch\n"                                                                       \
  "cmpq \\scratch,\\value\n"                                                                       \
  "cmovgq \\scratch,\\value\n"                                                                     \
  ".endm\n"

#define NUMERIC_IIR_STAGE                                                                          \
  ".macro numeric_iir_stage stage bits rm\n"                                                       \
  "numeric_load %%r14,(\\stage*8),%%rax\n"                                                         \
  "shlq $24,%%rax\n"                                                                               \
  "numeric_load %%r9,(\\stage*20),%%rdx\n"                                                         \
  "imulq %%rbx,%%rdx\n"                                                                            \
  "leaq (%%rax,%%rdx,2),%%rax\n"                                                                   \
  "numeric_quantize %%rax,%%rcx,\\bits,\\rm,-8388608,8388607\n"                                    \
  "movq %%rax,%%r8\n"                                                                              \
  "numeric_load %%r14,(\\stage*8+4),%%rax\n"                                                       \
  "shlq $24,%%rax\n"                                                                               \
  "numeric_load %%r9,(\\stage*20+4),%%rdx\n"                                                       \
  "imulq %%rbx,%%rdx\n"                                                                            \
  "leaq (%%rax,%%rdx,2),%%rax\n"                                                                   \
  "numeric_load %%r9,(\\stage*20+8),%%rdx\n"                                                       \
  "imulq %%r8,%%rdx\n"                                                                             \
  "addq %%rdx,%%rdx\n"                                                                             \
  "subq %%rdx,%%rax\n"                                                                             \
  "numeric_quantize %%rax,%%rcx,\\bits,\\rm,-8388608,8388607\n"                                    \
  "andl $16777215,%%eax\n"                                                                         \
  "movl %%eax,(\\stage*8)(%%r14)\n"                                                                \
  "numeric_load %%r9,(\\stage*20+12),%%rax\n"                                                      \
  "imulq %%rbx,%%rax\n"                                                                            \
  "addq %%rax,%%rax\n"                                                                             \
  "numeric_load %%r9,(\\stage*20+16),%%rdx\n"                                                      \
  "imulq %%r8,%%rdx\n"                                                                             \
  "addq %%rdx,%%rdx\n"                                                                             \
  "subq %%rdx,%%rax\n"                                                                             \
  "numeric_quantize %%rax,%%rcx,\\bits,\\rm,-8388608,8388607\n"                                    \
  "andl $16777215,%%eax\n"                                                                         \
  "movl %%eax,(\\stage*8+4)(%%r14)\n"                                                              \
  "movq %%r8,%%rbx\n"                                                                              \
  ".endm\n"

#define NUMERIC_NONLINEAR_STAGE                                                                    \
  ".macro numeric_nonlinear_stage stage bits rm low high\n"                                        \
  "numeric_load %%r9,(\\stage*8),%%rax\n"                                                          \
  "imulq %%rbx,%%rax\n"                                                                            \
  "addq %%rax,%%rax\n"                                                                             \
  "cmpq %%rcx,%%rax\n"                                                                             \
  "cmovgq %%rcx,%%rax\n"                                                                           \
  "numeric_load %%r14,(\\stage*4),%%rdx\n"                                                         \
  "numeric_load %%r9,(\\stage*8+4),%%r8\n"                                                         \
  "imulq %%r8,%%rdx\n"                                                                             \
  "leaq (%%rax,%%rdx,2),%%rax\n"                                                                   \
  "numeric_quantize %%rax,%%r8,\\bits,\\rm,\\low,\\high\n"                                         \
  "movq %%rax,%%rbx\n"                                                                             \
  "andl $16777215,%%eax\n"                                                                         \
  "movl %%eax,(\\stage*4)(%%r14)\n"                                                                \
  ".endm\n"

namespace dsp56300::test::numeric_host {
namespace {
static_assert(iir_stream::sections == 4 && nonlinear_stream::stages == 4);

template <unsigned Bits, bool Rm> void iir_kernel(const Job &job) {
  asm volatile(NUMERIC_LOAD NUMERIC_QUANTIZE NUMERIC_IIR_STAGE R"(
    movq %c[first](%[job]),%%r12
    movq %c[frames](%[job]),%%r13
    addq %%r12,%%r13
    movq %c[coefficients](%[job]),%%r9
    movq %c[output](%[job]),%%r15
  1:
    movq %%r12,%%rsi
    andl $%c[period_mask],%%esi
    imulq $%c[input_bytes],%%rsi,%%rsi
    addq %c[input](%[job]),%%rsi
    movq %c[state](%[job]),%%r14
    xorq %%r10,%%r10
    movl $%c[channels],%%r11d
  2:
    numeric_load %%rsi,0,%%rbx
    .irp stage,0,1,2,3
    numeric_iir_stage \stage,%c[bits],%c[rm]
    .endr
    addq %%rbx,%%r10
    addq $4,%%rsi
    addq $%c[state_bytes],%%r14
    decl %%r11d
    jnz 2b
    movq %%r10,%%rax
    shlq $%c[mix_shift],%%rax
    numeric_quantize %%rax,%%rcx,%c[bits],%c[rm],-8388608,8388607
    andl $16777215,%%eax
    movq %%r12,%%rdx
    andl $%c[output_mask],%%edx
    movl %%eax,(%%r15,%%rdx,4)
    incq %%r12
    cmpq %%r13,%%r12
    jb 1b
    .purgem numeric_iir_stage
    .purgem numeric_quantize
    .purgem numeric_load
  )"
               :
               : [job] "D"(&job),
                 [first] "i"(offsetof(Job, first)),
                 [frames] "i"(offsetof(Job, frames)),
                 [coefficients] "i"(offsetof(Job, coefficients)),
                 [input] "i"(offsetof(Job, input)),
                 [state] "i"(offsetof(Job, state)),
                 [output] "i"(offsetof(Job, output)),
                 [bits] "i"(Bits),
                 [rm] "i"(Rm),
                 [period_mask] "i"(iir_stream::signal_period - 1),
                 [input_bytes] "i"(iir_stream::channels * sizeof(Word)),
                 [channels] "i"(iir_stream::channels),
                 [state_bytes] "i"(std::size_t{iir_stream::sections} * 2 * sizeof(Word)),
                 [mix_shift] "i"(word_bits - 7),
                 [output_mask] "i"(iir_stream::output_words - 1)
               : "cc",
                 "memory",
                 "rax",
                 "rbx",
                 "rcx",
                 "rdx",
                 "rsi",
                 "r8",
                 "r9",
                 "r10",
                 "r11",
                 "r12",
                 "r13",
                 "r14",
                 "r15");
}

template <unsigned Bits, bool Rm> void nonlinear_kernel(const Job &job) {
  constexpr std::int64_t boundary = std::int64_t{sign24} << word_bits;
  constexpr std::int64_t maximum_accumulator = boundary - (std::int64_t{1} << word_bits);
  constexpr std::int64_t maximum_product = boundary - 1;
  constexpr auto low = -std::min(std::int64_t{sign24}, boundary >> Bits);
  constexpr auto high = std::min(std::int64_t{sign24 - 1}, maximum_accumulator >> Bits);
  asm volatile(NUMERIC_LOAD NUMERIC_QUANTIZE NUMERIC_NONLINEAR_STAGE R"(
    movq %c[first](%[job]),%%r12
    movq %c[frames](%[job]),%%r13
    addq %%r12,%%r13
    movq %c[coefficients](%[job]),%%r9
    movq %c[output](%[job]),%%r15
    movabsq $%c[maximum],%%rcx
  1:
    movq %%r12,%%rsi
    andl $%c[period_mask],%%esi
    imulq $%c[input_bytes],%%rsi,%%rsi
    addq %c[input](%[job]),%%rsi
    movq %c[state](%[job]),%%r14
    imulq $%c[channels],%%r12,%%r11
    andl $%c[output_mask],%%r11d
    movl $%c[channels],%%r10d
  2:
    numeric_load %%rsi,0,%%rbx
    .irp stage,0,1,2,3
    numeric_nonlinear_stage \stage,%c[bits],%c[rm],%c[low],%c[high]
    .endr
    movl %%eax,(%%r15,%%r11,4)
    incl %%r11d
    andl $%c[output_mask],%%r11d
    addq $4,%%rsi
    addq $%c[state_bytes],%%r14
    decl %%r10d
    jnz 2b
    movq %c[total](%[job]),%%rax
    subq %%r12,%%rax
    movq %c[progress](%[job]),%%rdx
    movl %%eax,(%%rdx)
    incq %%r12
    cmpq %%r13,%%r12
    jb 1b
    .purgem numeric_nonlinear_stage
    .purgem numeric_quantize
    .purgem numeric_load
  )"
               :
               : [job] "D"(&job),
                 [first] "i"(offsetof(Job, first)),
                 [frames] "i"(offsetof(Job, frames)),
                 [coefficients] "i"(offsetof(Job, coefficients)),
                 [input] "i"(offsetof(Job, input)),
                 [state] "i"(offsetof(Job, state)),
                 [output] "i"(offsetof(Job, output)),
                 [progress] "i"(offsetof(Job, progress)),
                 [total] "i"(offsetof(Job, total)),
                 [maximum] "i"(maximum_product),
                 [low] "i"(low),
                 [high] "i"(high),
                 [bits] "i"(Bits),
                 [rm] "i"(Rm),
                 [period_mask] "i"(nonlinear_stream::signal_period - 1),
                 [input_bytes] "i"(nonlinear_stream::channels * sizeof(Word)),
                 [channels] "i"(nonlinear_stream::channels),
                 [state_bytes] "i"(nonlinear_stream::stages * sizeof(Word)),
                 [output_mask] "i"(nonlinear_stream::output_words - 1)
               : "cc",
                 "memory",
                 "rax",
                 "rbx",
                 "rcx",
                 "rdx",
                 "rsi",
                 "r8",
                 "r9",
                 "r10",
                 "r11",
                 "r12",
                 "r13",
                 "r14",
                 "r15");
}

template <unsigned Bits, bool Nonlinear> void rounding(const Job &job, const bool rm) {
  if constexpr (Nonlinear) {
    if (rm) {
      nonlinear_kernel<Bits, true>(job);
    } else {
      nonlinear_kernel<Bits, false>(job);
    }
  } else {
    if (rm) {
      iir_kernel<Bits, true>(job);
    } else {
      iir_kernel<Bits, false>(job);
    }
  }
}

template <bool Nonlinear> void run(const Job &job, const unsigned bits, const bool rm) {
  require(job.first <= job.total && job.frames <= job.total - job.first,
          "Numeric ASM frame range is bounded");
  require(bits >= word_bits - 1 && bits <= word_bits + 1, "Numeric ASM quantization shift");
  if (job.frames == 0) { return; }
  switch (bits) {
    case word_bits - 1: rounding<word_bits - 1, Nonlinear>(job, rm); break;
    case word_bits: rounding<word_bits, Nonlinear>(job, rm); break;
    case word_bits + 1: rounding<word_bits + 1, Nonlinear>(job, rm); break;
    default: break;
  }
}
}

void iir_asm(const Job &job, const unsigned bits, const bool rm) {
  run<false>(job, bits, rm);
}

void nonlinear_asm(const Job &job, const unsigned bits, const bool rm) {
  run<true>(job, bits, rm);
}
}

#undef NUMERIC_LOAD
#undef NUMERIC_QUANTIZE
#undef NUMERIC_IIR_STAGE
#undef NUMERIC_NONLINEAR_STAGE
