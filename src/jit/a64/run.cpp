// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "dsp56300/core.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/resident_region.hpp"

#include <cstddef>
#include <cstdint>

#if defined(__APPLE__)
#define DSP56300_A64_NAME(name) "_" name
#else
#define DSP56300_A64_NAME(name) name
#endif

namespace dsp56300::detail {
constexpr unsigned pc_offset = 128, status_offset = 132, loop_offset = 140;
constexpr unsigned budget_offset = 152, boundary_offset = 156;
static_assert(offsetof(Cpu, reg) + offsetof(Registers, pc) == pc_offset);
static_assert(offsetof(Cpu, reg) + offsetof(Registers, sr) == status_offset);
static_assert(offsetof(Cpu, reg) + offsetof(Registers, la) == loop_offset);
static_assert(offsetof(NativeContext, budget) == budget_offset);
static_assert(offsetof(NativeContext, boundary) == boundary_offset);
static_assert(static_cast<unsigned>(NativeBoundary::context) == 2);
static_assert(static_cast<unsigned>(NativeBoundary::owner) == 1);
static_assert(static_cast<unsigned>(NativeBoundary::none) == 0);
static_assert(sizeof(void *) == 8 && native_page_shift == 8);
static_assert(sizeof(NativePage) == native_page_entries * sizeof(void *));

extern "C" void dsp56300_enter_resident_region(const void *, RegionFrame *) noexcept;
extern "C" std::uint32_t dsp56300_enter_generated(const void *, Cpu *, NativeContext *) noexcept;
extern "C" std::uint32_t dsp56300_run_blocks(const void *,
                                             Cpu *,
                                             NativeContext *,
                                             std::uint32_t,
                                             const void *const *,
                                             const void *const *) noexcept;
asm(".text\n.p2align 2\n");
asm(".globl " DSP56300_A64_NAME("dsp56300_enter_generated") "\n");
asm(DSP56300_A64_NAME("dsp56300_enter_generated") ":\n");
asm("mov x16,x0\n"
    "mov x0,x1\n"
    "mov x1,x2\n"
    "br x16\n");
asm(".globl " DSP56300_A64_NAME("dsp56300_enter_resident_region") "\n");
asm(DSP56300_A64_NAME("dsp56300_enter_resident_region") ":\n");
asm("mov x16,x0\n"
    "mov x0,x1\n"
    "br x16\n");
asm(".globl " DSP56300_A64_NAME("dsp56300_run_blocks") "\n");
asm(DSP56300_A64_NAME("dsp56300_run_blocks") ":\n");
asm("stp x29,x30,[sp,#-16]!\n"
    "mov x29,sp\n"
    "stp x19,x20,[sp,#-16]!\n"
    "stp x21,x22,[sp,#-16]!\n"
    "stp x23,x24,[sp,#-16]!\n"
    "mov x16,x0\n"
    "mov x19,x1\n"
    "mov x20,x2\n"
    "mov w21,w3\n"
    "mov x22,x4\n"
    "mov x23,x5\n"
    "mov w24,wzr\n");
asm("Ldsp56300_a64_next:\n"
    "mov x0,x19\n"
    "mov x1,x20\n"
    "str w21,[x20,#152]\n"
    "blr x16\n"
    "cbz w0,Ldsp56300_a64_done\n"
    "add w24,w24,w0\n"
    "sub w21,w21,w0\n"
    "cbz w21,Ldsp56300_a64_done\n"
    "ldrb w9,[x20,#156]\n"
    "cbz w9,Ldsp56300_a64_same\n"
    "cmp w9,#2\n");
asm("b.ne Ldsp56300_a64_done\n"
    "cbz x23,Ldsp56300_a64_done\n"
    "ldr w9,[x19,#132]\n"
    "ubfx w10,w9,#10,#2\n"
    "ubfx w11,w9,#13,#2\n"
    "orr w10,w10,w11,lsl #2\n"
    "ubfx w11,w9,#17,#1\n"
    "orr w10,w10,w11,lsl #4\n"
    "ubfx w11,w9,#20,#2\n"
    "orr w10,w10,w11,lsl #5\n"
    "tbnz w9,#15,Ldsp56300_a64_active\n"
    "mov w9,#0x1000000\n");
asm("b Ldsp56300_a64_key\n"
    "Ldsp56300_a64_active:\n"
    "ldr w9,[x19,#140]\n"
    "lsr w11,w9,#24\n"
    "cbnz w11,Ldsp56300_a64_done\n"
    "Ldsp56300_a64_key:\n"
    "lsl w9,w9,#7\n"
    "orr w9,w9,w10\n"
    "lsr w11,w9,#24\n"
    "ldr x10,[x23,x11,lsl #3]\n"
    "cbz x10,Ldsp56300_a64_done\n"
    "ubfx w11,w9,#16,#8\n");
asm("ldr x10,[x10,x11,lsl #3]\n"
    "cbz x10,Ldsp56300_a64_done\n"
    "ubfx w11,w9,#8,#8\n"
    "ldr x10,[x10,x11,lsl #3]\n"
    "cbz x10,Ldsp56300_a64_done\n"
    "and w11,w9,#255\n"
    "ldr x22,[x10,x11,lsl #3]\n"
    "cbz x22,Ldsp56300_a64_done\n"
    "strb wzr,[x20,#156]\n"
    "Ldsp56300_a64_same:\n"
    "ldr w9,[x19,#128]\n"
    "lsr w11,w9,#24\n");
asm("cbnz w11,Ldsp56300_a64_done\n"
    "lsr w11,w9,#16\n"
    "ldr x10,[x22,x11,lsl #3]\n"
    "cbz x10,Ldsp56300_a64_done\n"
    "ubfx w11,w9,#8,#8\n"
    "ldr x10,[x10,x11,lsl #3]\n"
    "cbz x10,Ldsp56300_a64_done\n"
    "and w11,w9,#255\n"
    "ldr x16,[x10,x11,lsl #3]\n"
    "cbnz x16,Ldsp56300_a64_next\n"
    "Ldsp56300_a64_done:\n"
    "mov w0,w24\n");
asm("ldp x23,x24,[sp],#16\n"
    "ldp x21,x22,[sp],#16\n"
    "ldp x19,x20,[sp],#16\n"
    "ldp x29,x30,[sp],#16\n"
    "ret\n");

bool enter_resident_region(const void *const entry, RegionFrame &frame) noexcept {
  dsp56300_enter_resident_region(entry, &frame);
  return true;
}

std::uint32_t enter_block(const void *const entry, Cpu &cpu, NativeContext &context) noexcept {
  return dsp56300_enter_generated(entry, &cpu, &context);
}

std::uint32_t enter_blocks(const void *const entry,
                           Cpu &cpu,
                           NativeContext &context,
                           const std::uint32_t budget,
                           const NativeIndex &index,
                           const NativeCatalog *const catalog) noexcept {
  return dsp56300_run_blocks(
      entry, &cpu, &context, budget, index.data(), catalog == nullptr ? nullptr : catalog->data());
}
}

#undef DSP56300_A64_NAME
