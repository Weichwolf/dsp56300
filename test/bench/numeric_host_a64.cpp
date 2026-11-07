// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "iir_stream_program.hpp"
#include "nonlinear_stream_program.hpp"
#include "numeric_host_asm.hpp"
#include "support.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#define DSP56300_NUMERIC_MACROS                                                                    \
  R"(
  .macro dsp_numeric_load base offset value
    ldr w\value,[\base,#\offset]
    sbfx x\value,x\value,#0,#24
  .endm
  .macro dsp_numeric_quantize bits rm
    .if !\rm
      lsr x1,x0,#\bits
      and x1,x1,#1
      add x0,x0,x1
    .endif
    add x0,x0,x4
    asr x0,x0,#\bits
    cmp x0,x5
    csel x0,x0,x5,ge
    cmp x0,x6
    csel x0,x0,x6,le
  .endm
  .macro dsp_iir_stage stage bits rm
    dsp_numeric_load x14,(\stage*8),0
    lsl x0,x0,#24
    dsp_numeric_load x11,(\stage*20),2
    mul x2,x17,x2
    add x0,x0,x2,lsl #1
    dsp_numeric_quantize \bits,\rm
    mov x3,x0
    dsp_numeric_load x14,(\stage*8+4),0
    lsl x0,x0,#24
    dsp_numeric_load x11,(\stage*20+4),2
    mul x2,x17,x2
    add x0,x0,x2,lsl #1
    dsp_numeric_load x11,(\stage*20+8),2
    mul x2,x3,x2
    sub x0,x0,x2,lsl #1
    dsp_numeric_quantize \bits,\rm
    and w0,w0,#0xffffff
    str w0,[x14,#(\stage*8)]
    dsp_numeric_load x11,(\stage*20+12),0
    mul x0,x17,x0
    lsl x0,x0,#1
    dsp_numeric_load x11,(\stage*20+16),2
    mul x2,x3,x2
    sub x0,x0,x2,lsl #1
    dsp_numeric_quantize \bits,\rm
    and w0,w0,#0xffffff
    str w0,[x14,#(\stage*8+4)]
    mov x17,x3
  .endm
  .macro dsp_nonlinear_stage stage bits rm
    dsp_numeric_load x11,(\stage*8),0
    mul x0,x17,x0
    lsl x0,x0,#1
    cmp x0,x7
    csel x0,x0,x7,le
    dsp_numeric_load x14,(\stage*4),2
    dsp_numeric_load x11,(\stage*8+4),3
    mul x2,x2,x3
    add x0,x0,x2,lsl #1
    dsp_numeric_quantize \bits,\rm
    mov x17,x0
    and w0,w0,#0xffffff
    str w0,[x14,#(\stage*4)]
  .endm
)"

namespace dsp56300::test::numeric_host {
namespace {
template <unsigned Bits, bool Rm> void iir_kernel(const Job &job) {
  asm volatile(DSP56300_NUMERIC_MACROS R"(
    ldr x9,[%[job],#%c[first]]
    ldr x10,[%[job],#%c[frames]]
    add x10,x10,x9
    ldr x11,[%[job],#%c[coefficients]]
    ldr x12,[%[job],#%c[output]]
    mov x4,#%c[bias]
    mov x5,#-8388608
    mov x6,#8388607
  1:
    and x2,x9,#%c[period_mask]
    mov x3,#%c[input_bytes]
    ldr x13,[%[job],#%c[input]]
    madd x13,x2,x3,x13
    ldr x14,[%[job],#%c[state]]
    mov x15,xzr
    mov x16,#%c[channels]
  2:
    dsp_numeric_load x13,0,17
    .irp stage,0,1,2,3
      dsp_iir_stage \stage,%c[bits],%c[rm]
    .endr
    add x15,x15,x17
    add x13,x13,#4
    add x14,x14,#%c[state_bytes]
    subs x16,x16,#1
    b.ne 2b
    lsl x0,x15,#%c[mix_shift]
    dsp_numeric_quantize %c[bits],%c[rm]
    and w0,w0,#0xffffff
    and x2,x9,#%c[output_mask]
    str w0,[x12,x2,lsl #2]
    add x9,x9,#1
    cmp x9,x10
    b.lo 1b
    .purgem dsp_numeric_load
    .purgem dsp_numeric_quantize
    .purgem dsp_iir_stage
    .purgem dsp_nonlinear_stage
  )"
               :
               : [job] "r"(&job),
                 [first] "i"(offsetof(Job, first)),
                 [frames] "i"(offsetof(Job, frames)),
                 [coefficients] "i"(offsetof(Job, coefficients)),
                 [input] "i"(offsetof(Job, input)),
                 [state] "i"(offsetof(Job, state)),
                 [output] "i"(offsetof(Job, output)),
                 [bits] "i"(Bits),
                 [rm] "i"(Rm),
                 [bias] "i"((std::int64_t{1} << (Bits - 1)) - (Rm ? 0 : 1)),
                 [period_mask] "i"(iir_stream::signal_period - 1),
                 [input_bytes] "i"(iir_stream::channels * sizeof(Word)),
                 [channels] "i"(iir_stream::channels),
                 [state_bytes] "i"(std::size_t{iir_stream::sections} * 2 * sizeof(Word)),
                 [mix_shift] "i"(word_bits - 7),
                 [output_mask] "i"(iir_stream::output_words - 1)
               : "cc",
                 "memory",
                 "x0",
                 "x1",
                 "x2",
                 "x3",
                 "x4",
                 "x5",
                 "x6",
                 "x9",
                 "x10",
                 "x11",
                 "x12",
                 "x13",
                 "x14",
                 "x15",
                 "x16",
                 "x17");
}

template <unsigned Bits, bool Rm> void nonlinear_kernel(const Job &job) {
  constexpr std::int64_t boundary = std::int64_t{sign24} << word_bits;
  constexpr auto low = -std::min(std::int64_t{sign24}, boundary >> Bits);
  constexpr auto high =
      std::min(std::int64_t{sign24 - 1}, (boundary - (std::int64_t{1} << word_bits)) >> Bits);
  asm volatile(DSP56300_NUMERIC_MACROS R"(
    ldr x9,[%[job],#%c[first]]
    ldr x10,[%[job],#%c[frames]]
    add x10,x10,x9
    ldr x11,[%[job],#%c[coefficients]]
    ldr x12,[%[job],#%c[output]]
    mov x4,#%c[bias]
    mov x5,#%c[low]
    mov x6,#%c[high]
    mov x7,#%c[maximum]
  1:
    and x2,x9,#%c[period_mask]
    mov x3,#%c[input_bytes]
    ldr x13,[%[job],#%c[input]]
    madd x13,x2,x3,x13
    ldr x14,[%[job],#%c[state]]
    mov x2,#%c[channels]
    mul x15,x9,x2
    and x15,x15,#%c[output_mask]
    mov x16,#%c[channels]
  2:
    dsp_numeric_load x13,0,17
    .irp stage,0,1,2,3
      dsp_nonlinear_stage \stage,%c[bits],%c[rm]
    .endr
    str w0,[x12,x15,lsl #2]
    add x15,x15,#1
    and x15,x15,#%c[output_mask]
    add x13,x13,#4
    add x14,x14,#%c[state_bytes]
    subs x16,x16,#1
    b.ne 2b
    ldr x0,[%[job],#%c[total]]
    sub x0,x0,x9
    ldr x2,[%[job],#%c[progress]]
    str w0,[x2]
    add x9,x9,#1
    cmp x9,x10
    b.lo 1b
    .purgem dsp_numeric_load
    .purgem dsp_numeric_quantize
    .purgem dsp_iir_stage
    .purgem dsp_nonlinear_stage
  )"
               :
               : [job] "r"(&job),
                 [first] "i"(offsetof(Job, first)),
                 [frames] "i"(offsetof(Job, frames)),
                 [coefficients] "i"(offsetof(Job, coefficients)),
                 [input] "i"(offsetof(Job, input)),
                 [state] "i"(offsetof(Job, state)),
                 [output] "i"(offsetof(Job, output)),
                 [progress] "i"(offsetof(Job, progress)),
                 [total] "i"(offsetof(Job, total)),
                 [maximum] "i"(boundary - 1),
                 [low] "i"(low),
                 [high] "i"(high),
                 [bits] "i"(Bits),
                 [rm] "i"(Rm),
                 [bias] "i"((std::int64_t{1} << (Bits - 1)) - (Rm ? 0 : 1)),
                 [period_mask] "i"(nonlinear_stream::signal_period - 1),
                 [input_bytes] "i"(nonlinear_stream::channels * sizeof(Word)),
                 [channels] "i"(nonlinear_stream::channels),
                 [state_bytes] "i"(nonlinear_stream::stages * sizeof(Word)),
                 [output_mask] "i"(nonlinear_stream::output_words - 1)
               : "cc",
                 "memory",
                 "x0",
                 "x1",
                 "x2",
                 "x3",
                 "x4",
                 "x5",
                 "x6",
                 "x7",
                 "x9",
                 "x10",
                 "x11",
                 "x12",
                 "x13",
                 "x14",
                 "x15",
                 "x16",
                 "x17");
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

#undef DSP56300_NUMERIC_MACROS
