/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BROADCOM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL BROADCOM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * #BRCM_2# */

#ifndef _ASM_MACH_NLM_MMU_H
#define _ASM_MACH_NLM_MMU_H

#include <linux/smp.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/page.h>
#include <asm/pgtable-bits.h>

#ifdef CONFIG_NLM_XLP
#include <asm/mach-netlogic/xlp-mmu.h>
#endif

#define ENTRYLO_PFN_SHIFT _PAGE_GLOBAL_SHIFT

#ifndef __ASSEMBLY__

#define SMALLEST_TLBPAGE_SZ (4UL << 10)
#define LARGEST_TLBPAGE_SZ  (256UL << 20)

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
#define MAX_WIRED_PFN PFN_UP(4UL << 30)
#endif

#define TRUE 1
#define FALSE 0

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
typedef struct
{
	unsigned long entryHi;
	unsigned long entrylo0;
	unsigned long entrylo1;
	int wired;
} tlb_entry_t;
#else
typedef struct
{
	unsigned long vaddr;
	uint64_t paddr0;
	uint64_t paddr1;
	uint32_t pagesize;
	uint32_t attr0;
	uint32_t attr1;
	int wired;
} tlb_info_t;
#endif

#ifdef CONFIG_MAPPED_KERNEL
extern unsigned long __vmalloc_start;
#endif
extern unsigned long long nlm_common_tlb_stats[];

extern void mmu_init(void);

#ifndef CONFIG_NLM_16G_MEM_SUPPORT
extern void setup_tlb(tlb_info_t *tlb);
#endif

/*
 * the following needs an used argument to confirm to the 
 * prototype of functions passed to on_each_cpu()
 */
static inline void nlm_update_tlb_stats(void *arg)
{
	nlm_common_tlb_stats[smp_processor_id()] = nlm_read_os_scratch_2();
}

#define tlbstats_init() nlm_write_os_scratch_2(0ULL)

#ifdef CONFIG_HUGETLBFS
#define entrylo0_mask_init() \
nlm_write_os_scratch_3(~(((1ULL << HUGETLB_PAGE_ORDER) - 1) << ENTRYLO_PFN_SHIFT))
#else
#define entrylo0_mask_init()
#endif

extern void setup_mapped_kernel_tlbs(int index, int secondary_cpu);
extern unsigned long recalculate_max_low_pfn(unsigned long max_low_pfn);

#if defined(CONFIG_NLM_XLP) && defined(CONFIG_NUMA)
extern void __init nlm_numa_bootmem_init(unsigned long);
#endif

#ifndef CONFIG_NLM_XLP
#define disable_pgwalker(flags) (void)flags
#define enable_pgwalker(flags) (void) flags
#endif

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
extern int map_kernel_addrspace(unsigned long vaddr, unsigned long paddr,
				unsigned long max_pfn);
extern void setup_mapped_kernel_pgtable(void);

#define KERNEL_PAGE_ATTR \
	(_CACHE_CACHABLE_COW |_PAGE_DIRTY |  _PAGE_VALID | _PAGE_GLOBAL)
#endif

#endif /* __ASSEMBLY__ */
#endif
