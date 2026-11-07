// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "fft_host.hpp"
#include "spectral_host_asm.hpp"
#include "support.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

#define DSP56300_SPECTRAL_MACROS                                                                   \
  R"(
  .macro dsp_spectral_round value bits rm
    .if !\rm
      sshr v20.2d,v\value\().2d,#\bits
      and v20.16b,v20.16b,v21.16b
      add v\value\().2d,v\value\().2d,v20.2d
    .endif
    .if \bits == 1
      .if \rm
        add v\value\().2d,v\value\().2d,v21.2d
      .endif
    .else
      add v\value\().2d,v\value\().2d,v22.2d
    .endif
    sshr v\value\().2d,v\value\().2d,#\bits
  .endm
  .macro dsp_spectral_limit value
    cmgt v20.2d,v\value\().2d,v24.2d
    bit v\value\().16b,v24.16b,v20.16b
    cmgt v20.2d,v23.2d,v\value\().2d
    bit v\value\().16b,v23.16b,v20.16b
  .endm
  .macro dsp_spectral_load value base pair offset
    .if \pair
      ldr q\value,[\base,\offset]
    .else
      ldr d\value,[\base,\offset]
    .endif
  .endm
  .macro dsp_spectral_store value base pair offset
    .if \pair
      str q\value,[\base,\offset]
    .else
      str d\value,[\base,\offset]
    .endif
  .endm
)"

namespace dsp56300::test::fft::host {
namespace {
struct Butterfly {
  std::int64_t *real, *imaginary, *upper_real, *upper_imaginary;
  const std::int64_t *weight_real, *weight_imaginary;
  std::size_t count;
};

template <bool Rm, bool Pair> void columns(const Butterfly &job) {
  std::size_t offset;
  asm volatile(DSP56300_SPECTRAL_MACROS R"(
    mov x9,#1
    dup v21.2d,x9
    mov x9,#%c[bias]
    dup v22.2d,x9
    mov x9,#-8388608
    dup v23.2d,x9
    mov x9,#8388607
    dup v24.2d,x9
  1:
    dsp_spectral_load 0,%[real],%c[pair],%[offset]
    dsp_spectral_load 1,%[imaginary],%c[pair],%[offset]
    dsp_spectral_load 2,%[upper_real],%c[pair],%[offset]
    dsp_spectral_load 3,%[upper_imaginary],%c[pair],%[offset]
    sub v4.2d,v0.2d,v2.2d
    sub v5.2d,v1.2d,v3.2d
    add v0.2d,v0.2d,v2.2d
    add v1.2d,v1.2d,v3.2d
    dsp_spectral_round 0,1,%c[rm]
    dsp_spectral_round 1,1,%c[rm]
    dsp_spectral_store 0,%[real],%c[pair],%[offset]
    dsp_spectral_store 1,%[imaginary],%c[pair],%[offset]
    dsp_spectral_round 4,1,%c[rm]
    dsp_spectral_round 5,1,%c[rm]
    dsp_spectral_limit 4
    dsp_spectral_limit 5
    dsp_spectral_load 6,%[weight_real],%c[pair],%[offset]
    dsp_spectral_load 7,%[weight_imaginary],%c[pair],%[offset]
    xtn v4.2s,v4.2d
    xtn v5.2s,v5.2d
    xtn v6.2s,v6.2d
    xtn v7.2s,v7.2d
    smull v0.2d,v4.2s,v6.2s
    smull v1.2d,v5.2s,v7.2s
    sub v0.2d,v0.2d,v1.2d
    smull v2.2d,v4.2s,v7.2s
    smlal v2.2d,v5.2s,v6.2s
    dsp_spectral_round 0,23,%c[rm]
    dsp_spectral_round 2,23,%c[rm]
    dsp_spectral_limit 0
    dsp_spectral_limit 2
    dsp_spectral_store 0,%[upper_real],%c[pair],%[offset]
    dsp_spectral_store 2,%[upper_imaginary],%c[pair],%[offset]
    add %[offset],%[offset],#%c[stride]
    cmp %[offset],%[bytes]
    b.lo 1b
    .purgem dsp_spectral_round
    .purgem dsp_spectral_limit
    .purgem dsp_spectral_load
    .purgem dsp_spectral_store
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
                 [stride] "i"((Pair ? 2 : 1) * sizeof(std::int64_t)),
                 [pair] "i"(Pair),
                 [rm] "i"(Rm),
                 [bias] "i"((scale / 2) - (Rm ? 0 : 1))
               : "cc",
                 "memory",
                 "x9",
                 "v0",
                 "v1",
                 "v2",
                 "v3",
                 "v4",
                 "v5",
                 "v6",
                 "v7",
                 "v20",
                 "v21",
                 "v22",
                 "v23",
                 "v24");
}

template <bool Rm> void stages(Data &data, const std::span<const Stage> plan) {
  for (const auto &stage : plan) {
    for (std::size_t group = 0; group < data.real.size(); group += 2 * stage.half) {
      const Butterfly job{.real = data.real.data() + group,
                          .imaginary = data.imaginary.data() + group,
                          .upper_real = data.real.data() + group + stage.half,
                          .upper_imaginary = data.imaginary.data() + group + stage.half,
                          .weight_real = stage.real.data(),
                          .weight_imaginary = stage.imaginary.data(),
                          .count = stage.half};
      if (stage.half >= 2) {
        columns<Rm, true>(job);
      } else {
        columns<Rm, false>(job);
      }
    }
  }
}

void permute(Data &data) {
  const auto shift = 32 - static_cast<unsigned>(std::countr_zero(data.real.size()));
  std::size_t index;
  asm volatile(DSP56300_SPECTRAL_MACROS R"(
    1:
      rbit w9,%w[index]
      lsr w9,w9,%w[shift]
      ldr x10,[%[real],x9,lsl #3]
      ldr x11,[%[imaginary],x9,lsl #3]
      str x10,[%[output_real],%[index],lsl #3]
      str x11,[%[output_imaginary],%[index],lsl #3]
      add %[index],%[index],#1
      cmp %[index],%[size]
      b.lo 1b
    .purgem dsp_spectral_round
    .purgem dsp_spectral_limit
    .purgem dsp_spectral_load
    .purgem dsp_spectral_store
  )"
               : [index] "=&r"(index)
               : "0"(std::size_t{0}),
                 [real] "r"(data.real.data()),
                 [imaginary] "r"(data.imaginary.data()),
                 [output_real] "r"(data.output_real.data()),
                 [output_imaginary] "r"(data.output_imaginary.data()),
                 [size] "r"(data.real.size()),
                 [shift] "r"(shift)
               : "cc", "memory", "x9", "x10", "x11");
}

template <bool Rm> void window_words(const Window &job) {
  std::size_t offset;
  asm volatile(DSP56300_SPECTRAL_MACROS R"(
    mov x9,#1
    dup v21.2d,x9
    mov x9,#%c[bias]
    dup v22.2d,x9
    mov x9,#-8388608
    dup v23.2d,x9
    mov x9,#8388607
    dup v24.2d,x9
  1:
    ldr d0,[%[real],%[offset]]
    ldr d1,[%[imaginary],%[offset]]
    ldr d2,[%[weights],%[offset]]
    shl v0.2s,v0.2s,#8
    shl v1.2s,v1.2s,#8
    shl v2.2s,v2.2s,#8
    sshr v0.2s,v0.2s,#8
    sshr v1.2s,v1.2s,#8
    sshr v2.2s,v2.2s,#8
    smull v0.2d,v0.2s,v2.2s
    smull v1.2d,v1.2s,v2.2s
    dsp_spectral_round 0,23,%c[rm]
    dsp_spectral_round 1,23,%c[rm]
    dsp_spectral_limit 0
    dsp_spectral_limit 1
    lsl x9,%[offset],#1
    str q0,[%[output_real],x9]
    str q1,[%[output_imaginary],x9]
    add %[offset],%[offset],#8
    cmp %[offset],%[bytes]
    b.lo 1b
    .purgem dsp_spectral_round
    .purgem dsp_spectral_limit
    .purgem dsp_spectral_load
    .purgem dsp_spectral_store
  )"
               : [offset] "=&r"(offset)
               : "0"(std::size_t{0}),
                 [real] "r"(job.real),
                 [imaginary] "r"(job.imaginary),
                 [weights] "r"(job.weights),
                 [output_real] "r"(job.output_real),
                 [output_imaginary] "r"(job.output_imaginary),
                 [bytes] "r"(job.count * sizeof(Word)),
                 [rm] "i"(Rm),
                 [bias] "i"((scale / 2) - (Rm ? 0 : 1))
               : "cc", "memory", "x9", "v0", "v1", "v2", "v20", "v21", "v22", "v23", "v24");
}

template <bool Rm> std::size_t power_words(const Power &job) {
  std::size_t index;
  auto cursor = job.position;
  asm volatile(DSP56300_SPECTRAL_MACROS R"(
    mov x4,#%c[bias]
    mov x5,#8388607
  1:
    ldr x9,[%[job],#%c[real]]
    ldr x0,[x9,%[index],lsl #3]
    and w0,w0,#0xffffff
    ldr x9,[%[job],#%c[stored_real]]
    str w0,[x9,%[index],lsl #2]
    ldr x9,[%[job],#%c[imaginary]]
    ldr x0,[x9,%[index],lsl #3]
    and w0,w0,#0xffffff
    ldr x9,[%[job],#%c[stored_imaginary]]
    str w0,[x9,%[index],lsl #2]
    ldr x9,[%[job],#%c[output_real]]
    ldr x2,[x9,%[index],lsl #3]
    and w0,w2,#0xffffff
    ldr x9,[%[job],#%c[stored_output_real]]
    str w0,[x9,%[index],lsl #2]
    ldr x9,[%[job],#%c[output_imaginary]]
    ldr x3,[x9,%[index],lsl #3]
    and w0,w3,#0xffffff
    ldr x9,[%[job],#%c[stored_output_imaginary]]
    str w0,[x9,%[index],lsl #2]
    mul x0,x2,x2
    madd x0,x3,x3,x0
    .if !%c[rm]
      lsr x1,x0,#24
      and x1,x1,#1
      add x0,x0,x1
    .endif
    add x0,x0,x4
    lsr x0,x0,#24
    cmp x0,x5
    csel x0,x0,x5,le
    ldr x9,[%[job],#%c[power]]
    str w0,[x9,%[cursor],lsl #2]
    add %[cursor],%[cursor],#1
    cmp %[cursor],%[ring_words]
    csel %[cursor],xzr,%[cursor],hs
    add %[index],%[index],#1
    cmp %[index],%[count]
    b.lo 1b
    .purgem dsp_spectral_round
    .purgem dsp_spectral_limit
    .purgem dsp_spectral_load
    .purgem dsp_spectral_store
  )"
               : [index] "=&r"(index), [cursor] "+&r"(cursor)
               : "0"(std::size_t{0}),
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
                 [rm] "i"(Rm),
                 [bias] "i"(scale - (Rm ? 0 : 1))
               : "cc", "memory", "x0", "x1", "x2", "x3", "x4", "x5", "x9");
  return cursor;
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

#undef DSP56300_SPECTRAL_MACROS
