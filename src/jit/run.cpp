// SPDX-License-Identifier: GPL-3.0-only
#include "architecture.hpp"
#include "jit.hpp"
#include "jit/block.hpp"
#include "jit/resident_region.hpp"

#include <cstddef>
#include <cstdint>
#if defined(__x86_64__) && defined(__linux__)
#include "dsp56300/core.hpp"
#endif

#define DSP56300_RUN_PC_OFFSET 128
#define DSP56300_RUN_SR_OFFSET 132
#define DSP56300_RUN_LA_OFFSET 140
#define DSP56300_RUN_BUDGET_OFFSET 152
#define DSP56300_RUN_BOUNDARY_OFFSET 156
#define DSP56300_RUN_STRING_VALUE(value) #value
#define DSP56300_RUN_STRING(value) DSP56300_RUN_STRING_VALUE(value)

namespace dsp56300::detail {
#if defined(__x86_64__) && defined(__linux__)
static_assert(offsetof(Cpu, reg) + offsetof(Registers, pc) == DSP56300_RUN_PC_OFFSET);
static_assert(offsetof(Cpu, reg) + offsetof(Registers, sr) == DSP56300_RUN_SR_OFFSET);
static_assert(offsetof(Cpu, reg) + offsetof(Registers, la) == DSP56300_RUN_LA_OFFSET);
static_assert(offsetof(NativeContext, budget) == DSP56300_RUN_BUDGET_OFFSET);
static_assert(offsetof(NativeContext, boundary) == DSP56300_RUN_BOUNDARY_OFFSET);
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
asm(".equ dsp56300_run_pc," DSP56300_RUN_STRING(DSP56300_RUN_PC_OFFSET) "\n");
asm(".equ dsp56300_run_sr," DSP56300_RUN_STRING(DSP56300_RUN_SR_OFFSET) "\n");
asm(".equ dsp56300_run_la," DSP56300_RUN_STRING(DSP56300_RUN_LA_OFFSET) "\n");
asm(".equ dsp56300_run_budget," DSP56300_RUN_STRING(DSP56300_RUN_BUDGET_OFFSET) "\n");
asm(".equ dsp56300_run_boundary," DSP56300_RUN_STRING(DSP56300_RUN_BOUNDARY_OFFSET) "\n");
asm(".pushsection .text\n"
    ".globl dsp56300_enter_generated\n"
    ".hidden dsp56300_enter_generated\n"
    ".type dsp56300_enter_generated,@function\n"
    "dsp56300_enter_generated:\n"
    "endbr64\n"
    "mov %rdi,%rax\n"
    "mov %rsi,%rdi\n"
    "mov %rdx,%rsi\n"
    "jmp *%rax\n"
    ".size dsp56300_enter_generated,.-dsp56300_enter_generated\n"
    ".globl dsp56300_enter_resident_region\n"
    ".hidden dsp56300_enter_resident_region\n"
    ".type dsp56300_enter_resident_region,@function\n"
    "dsp56300_enter_resident_region:\n"
    "endbr64\n"
    "mov %rdi,%rax\n"
    "mov %rsi,%rdi\n"
    "jmp *%rax\n"
    ".size dsp56300_enter_resident_region,.-dsp56300_enter_resident_region\n"
    ".globl dsp56300_run_blocks\n"
    ".hidden dsp56300_run_blocks\n"
    ".type dsp56300_run_blocks,@function\n"
    "dsp56300_run_blocks:\n"
    "endbr64\n"
    "push %rbx\n"
    "push %rbp\n"
    "push %r12\n"
    "push %r13\n"
    "push %r14\n"
    "push %r15\n"
    "sub $8,%rsp\n"
    "mov %rdi,%rax\n"
    "mov %rsi,%r12\n"
    "mov %rdx,%r13\n"
    "mov %r8,%r14\n"
    "mov %r9,%rbp\n"
    "mov %ecx,%r15d\n"
    "xor %ebx,%ebx\n"
    ".Ldsp56300_run_next:\n"
    "mov %r12,%rdi\n"
    "mov %r13,%rsi\n"
    "mov %r15d,dsp56300_run_budget(%r13)\n"
    "call *%rax\n"
    "test %eax,%eax\n"
    "jz .Ldsp56300_run_done\n"
    "add %eax,%ebx\n"
    "sub %eax,%r15d\n"
    "jz .Ldsp56300_run_done\n"
    "cmpb $0,dsp56300_run_boundary(%r13)\n"
    "je .Ldsp56300_run_same_context\n"
    "cmpb $2,dsp56300_run_boundary(%r13)\n"
    "jne .Ldsp56300_run_done\n"
    "test %rbp,%rbp\n"
    "jz .Ldsp56300_run_done\n"
    "mov dsp56300_run_sr(%r12),%eax\n"
    "mov %eax,%edx\n"
    "shr $10,%edx\n"
    "and $3,%edx\n"
    "mov %eax,%ecx\n"
    "shr $11,%ecx\n"
    "and $12,%ecx\n"
    "or %ecx,%edx\n"
    "mov %eax,%ecx\n"
    "shr $13,%ecx\n"
    "and $16,%ecx\n"
    "or %ecx,%edx\n"
    "mov %eax,%ecx\n"
    "shr $15,%ecx\n"
    "and $96,%ecx\n"
    "or %ecx,%edx\n"
    "test $0x8000,%eax\n"
    "jz .Ldsp56300_run_inactive_loop\n"
    "mov dsp56300_run_la(%r12),%eax\n"
    "cmp $0xffffff,%eax\n"
    "ja .Ldsp56300_run_done\n"
    "jmp .Ldsp56300_run_context_key\n"
    ".Ldsp56300_run_inactive_loop:\n"
    "mov $0x1000000,%eax\n"
    ".Ldsp56300_run_context_key:\n"
    "shl $7,%eax\n"
    "or %edx,%eax\n"
    "mov %eax,%edx\n"
    "shr $24,%edx\n"
    "mov (%rbp,%rdx,8),%rdx\n"
    "test %rdx,%rdx\n"
    "jz .Ldsp56300_run_done\n"
    "mov %eax,%ecx\n"
    "shr $16,%ecx\n"
    "and $255,%ecx\n"
    "mov (%rdx,%rcx,8),%rdx\n"
    "test %rdx,%rdx\n"
    "jz .Ldsp56300_run_done\n"
    "mov %eax,%ecx\n"
    "shr $8,%ecx\n"
    "and $255,%ecx\n"
    "mov (%rdx,%rcx,8),%rdx\n"
    "test %rdx,%rdx\n"
    "jz .Ldsp56300_run_done\n"
    "movzbl %al,%eax\n"
    "mov (%rdx,%rax,8),%rdx\n"
    "test %rdx,%rdx\n"
    "jz .Ldsp56300_run_done\n"
    "mov %rdx,%r14\n"
    "movb $0,dsp56300_run_boundary(%r13)\n"
    ".Ldsp56300_run_same_context:\n"
    "mov dsp56300_run_pc(%r12),%eax\n"
    "cmp $0xffffff,%eax\n"
    "ja .Ldsp56300_run_done\n"
    "mov (%r14,%rax,8),%rax\n"
    "test %rax,%rax\n"
    "jnz .Ldsp56300_run_next\n"
    ".Ldsp56300_run_done:\n"
    "mov %ebx,%eax\n"
    "add $8,%rsp\n"
    "pop %r15\n"
    "pop %r14\n"
    "pop %r13\n"
    "pop %r12\n"
    "pop %rbp\n"
    "pop %rbx\n"
    "ret\n"
    ".size dsp56300_run_blocks,.-dsp56300_run_blocks\n"
    ".popsection\n");
#endif

bool enter_resident_region(const void *const entry, RegionFrame &frame) noexcept {
#if defined(__x86_64__) && defined(__linux__)
  dsp56300_enter_resident_region(entry, &frame);
  return true;
#else
  static_cast<void>(entry);
  static_cast<void>(frame);
  return false;
#endif
}

std::uint32_t enter_block(const void *const entry, Cpu &cpu, NativeContext &context) noexcept {
#if defined(__x86_64__) && defined(__linux__)
  return dsp56300_enter_generated(entry, &cpu, &context);
#else
  static_cast<void>(entry);
  static_cast<void>(cpu);
  static_cast<void>(context);
  return 0;
#endif
}

std::uint32_t enter_blocks(const void *const entry,
                           Cpu &cpu,
                           NativeContext &context,
                           const std::uint32_t budget,
                           const NativeIndex &index,
                           const NativeCatalog *const catalog) noexcept {
#if defined(__x86_64__) && defined(__linux__)
  return dsp56300_run_blocks(
      entry, &cpu, &context, budget, index.data(), catalog == nullptr ? nullptr : catalog->data());
#else
  static_cast<void>(entry);
  static_cast<void>(cpu);
  static_cast<void>(context);
  static_cast<void>(budget);
  static_cast<void>(index);
  static_cast<void>(catalog);
  return 0;
#endif
}
}

#undef DSP56300_RUN_STRING
#undef DSP56300_RUN_STRING_VALUE
#undef DSP56300_RUN_BOUNDARY_OFFSET
#undef DSP56300_RUN_BUDGET_OFFSET
#undef DSP56300_RUN_LA_OFFSET
#undef DSP56300_RUN_SR_OFFSET
#undef DSP56300_RUN_PC_OFFSET
