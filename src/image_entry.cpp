// SPDX-License-Identifier: GPL-3.0-only
#include "image_entry.hpp"

#if defined(__APPLE__)
#define DSP56300_IMAGE_SYMBOL "_dsp56300_image_enter"
#else
#define DSP56300_IMAGE_SYMBOL "dsp56300_image_enter"
#endif

#if defined(__x86_64__)
asm(".text\n"
    ".p2align 4\n"
    ".globl " DSP56300_IMAGE_SYMBOL "\n" DSP56300_IMAGE_SYMBOL ":\n"
    ".cfi_startproc\n"
    "endbr64\n"
    "movq %rdi, %rax\n"
    "movq %rsi, %rdi\n"
    "movl %edx, %esi\n"
    "movl %ecx, %edx\n"
    "movq %r8, %rcx\n"
    "jmp *%rax\n"
    ".cfi_endproc\n");
#elif defined(__aarch64__)
asm(".text\n"
    ".p2align 2\n"
    ".globl " DSP56300_IMAGE_SYMBOL "\n" DSP56300_IMAGE_SYMBOL ":\n"
    ".cfi_startproc\n"
    "bti c\n"
    "mov x16, x0\n"
    "mov x0, x1\n"
    "mov w1, w2\n"
    "mov w2, w3\n"
    "mov x3, x4\n"
    "br x16\n"
    ".cfi_endproc\n");
#else
#error "The native image entry requires an x86-64 or AArch64 host"
#endif
#undef DSP56300_IMAGE_SYMBOL
