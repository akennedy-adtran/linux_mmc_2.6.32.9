/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#include <asm/mipsregs.h>
#include <asm/mach-netlogic/mmu.h>
#include <asm/mach-netlogic/pgwalker.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/asm-offsets.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/hal/nlm_hal.h>

//#define DEBUG_PGWALKER

#ifdef DEBUG_PGWALKER
#define DEBUG_PRINTK	printk
#else
#define DEBUG_PRINTK(...)
#endif

#define READ_INHIBIT (1 << 31)
#define EXEC_INHIBIT (1 << 30)

#ifndef CONFIG_HUGETLB_PAGE
#ifdef CONFIG_64BIT
static int __initdata tlb_config = (ENABLE_ETLB | ENABLE_128_TLB);
#else
static int __initdata tlb_config = (ENABLE_128_TLB);	
#endif
#else
/* For hugetlb which has two different page sizes,
 * xlp8xx/xlp3xx hardware page walker seems having some issues.
 */
static int __initdata tlb_config = (ENABLE_ETLB | ENABLE_128_TLB);
#endif

int __init disable_etlb(char *str)
{
	tlb_config &= ~ENABLE_ETLB;
	return 1;
}
__setup("disable_etlb", disable_etlb);

int __init disable_128tlb(char *str)
{
	tlb_config &= ~ENABLE_128_TLB;

	return 1;
}
__setup("disable_128tlb", disable_128tlb);

int __init enable_pgwalker_cmdline(char *str)
{
	tlb_config |= ENABLE_PGWALKER;

	return 1;
}
__setup("enable_pgwalker", enable_pgwalker_cmdline);

/*
 * Page Walker
 */

DEFINE_PER_CPU(unsigned long [NR_ADDR_SEGMENTS], pgd_bases);

#ifdef CONFIG_64BIT
static int pgtable_levels = PGD | PMD | PTE;
#else
static int pgtable_levels = PGD | PTE;
#endif

static void pgwalker_init(void)
{
	unsigned int value;
	int i = 0;

	if (!(tlb_config & ENABLE_PGWALKER)) {
		/* Disable hardware page walker */
		write_c0_config6(read_c0_config6() & (~ENABLE_PGWALKER));
		printk("Hardware Page Walker disabled on cpu@%d\n", hard_smp_processor_id());
		return;
	}

	/* Initialize pgd_bases to default values */
	for(i = 0; i < NR_ADDR_SEGMENTS; i++) {
		get_cpu_var(pgd_bases)[i] = (unsigned long)swapper_pg_dir;
		put_cpu_var(pgd_bases);
	}

	/*
	 * hardware page levels information:
	 *
	 * [15:8] no of top-most bits of vaddr used to form
	 *        an index into the pgdirs table
	 * [ 7:4] shift amount by which pfn (page frame number)
	 *        needs to be left shifted for populating the
	 *        entrylo0 and entrylo1 registers
	 * [ 3:0] page table levels used. 32-bit kernels use
	 *        pgd and pte levels, while 64-bits kernels
	 *        use pgd, pmd, and pte
	 */
	value  = ((ffs(NR_ADDR_SEGMENTS) - 1) & 0xff) << 8;
	value |= ENTRYLO_PFN_SHIFT << 4;
	value |= pgtable_levels;
	pgw_register_write_w(PGW_MMU_INFO, value);

#ifdef CONFIG_64BIT
	pgw_register_write_d(PGW_PGD_BASES, (unsigned long long)&(__get_cpu_var(pgd_bases)[0]));
#else
	pgw_register_write_w(PGW_PGD_BASES, (unsigned int)&(__get_cpu_var(pgd_bases)[0]));
#endif

	/* PGD shift and mask information */
	pgw_register_write_w(PGW_PGD_SHIFT, _PGDIR_SHIFT - _PGD_T_LOG2);
	pgw_register_write_w(PGW_PGD_MASK, (_PTRS_PER_PGD - 1) << _PGD_T_LOG2);
#ifdef CONFIG_64BIT
	/*
	 * MIPS Linux currently does not use 4-level page tables
	 * and hence it is not necessary to fill in pud information
	 *
	 * So, just fill in the PMD shift and mask information
	 */
	pgw_register_write_w(PGW_PMD_SHIFT, _PMD_SHIFT - _PMD_T_LOG2);
	pgw_register_write_w(PGW_PMD_MASK, (_PTRS_PER_PMD - 1) << _PMD_T_LOG2);
#endif

	/* PTE shift and mask */
	pgw_register_write_w(PGW_PTE_SHIFT, PAGE_SHIFT - _PTE_T_LOG2);
	pgw_register_write_w(PGW_PTE_MASK, (_PTRS_PER_PTE - 1) << _PTE_T_LOG2);

	/* PUD shift and mask */
	pgw_register_write_w(PGW_PUD_SHIFT, 0);
	pgw_register_write_w(PGW_PUD_MASK, 0);

	get_cpu_var(pgd_bases)[VMALLOC_SEG] = (unsigned long)swapper_pg_dir;

#ifdef MODULE_START
	__get_cpu_var(pgd_bases)[MODULE_SEG] = (unsigned long)swapper_pg_dir;
#endif


	put_cpu_var(pgd_bases);

	dump_pgwalker_config();
	printk("Initialized Hardware Page Walker on cpu@%d\n", hard_smp_processor_id());
}

static void pgwalker_init_mips_compliant(void)
{
	int i = 0;
	uint64_t pwbase_val = 0, pwfield_val = 0, pwsize_val = 0, bd_shift = 0;
	uint32_t pwctl_val = 0;

	if (!(tlb_config & ENABLE_PGWALKER)) {
		/* disable pagewalker (PWCTL_PW_EN field was initialized to 0) */
		printk("Hardware Page Walker disabled on cpu@%d\n", hard_smp_processor_id());
		write_c0_pwctl(pwctl_val);
		return;
	}

	/* Initialize pgd_bases to default values */
	for(i = 0; i < NR_ADDR_SEGMENTS; i++) {
		get_cpu_var(pgd_bases)[i] = (unsigned long)swapper_pg_dir;
		put_cpu_var(pgd_bases);
	}
	pwbase_val = (uint64_t)&(__get_cpu_var(pgd_bases)[0]);

	/* enable page walker */
	pwctl_val |= ((uint32_t)1) << PWCTL_PW_EN_O;

	/* page table selector */
	bd_shift = _PGDIR_SHIFT + ffs(_PTRS_PER_PGD) - 1;
	pwfield_val |= bd_shift << PWFIELD_BD_O; /* corresponds to old HPW_NUM_PAGE_LVL[15:8] */
	pwsize_val |= ((uint64_t)(ffs(NR_ADDR_SEGMENTS) - 1)) << PWSIZE_BD_O;

	/* global directory */
	pwfield_val |= ((uint64_t)(_PGDIR_SHIFT)) << PWFIELD_GD_O;
	pwsize_val |= ((uint64_t)(ffs(_PTRS_PER_PGD) - 1)) << PWSIZE_GD_O;

	/* upper directory (was initialized to 0) */

#ifdef CONFIG_64BIT
	/* middle directory */
	pwfield_val |= ((uint64_t)_PMD_SHIFT) << PWFIELD_MD_O;
	pwsize_val |= ((uint64_t)(ffs(_PTRS_PER_PMD) - 1)) << PWSIZE_MD_O;

#endif
	/* page table index */
	pwfield_val |= ((uint64_t)PAGE_SHIFT) << PWFIELD_PT_O;
	pwsize_val |= ((uint64_t)(ffs(_PTRS_PER_PTE) - 1)) << PWSIZE_PT_O;

#ifdef CONFIG_64BIT
	pwsize_val |= ((uint64_t)1) << PWSIZE_PS_O;
#else
	/* PWSIZE_PS field was initialized to 0 */
#endif

	/* logical right rotate of PTE loaded from memory */
	pwfield_val |= ((uint64_t)ENTRYLO_PFN_SHIFT) << PWFIELD_PTE_O;

	/* PWSIZE_PTE field was initialized to 0 (page table entry spacing) */

	/* write registers*/
	write_c0_pwbase(pwbase_val);
	write_c0_pwfield(pwfield_val);
	write_c0_pwsize(pwsize_val);
	write_c0_pwctl(pwctl_val);

	get_cpu_var(pgd_bases)[VMALLOC_SEG] = (unsigned long)swapper_pg_dir;

#ifdef MODULE_START
	__get_cpu_var(pgd_bases)[MODULE_SEG] = (unsigned long)swapper_pg_dir;
#endif

	put_cpu_var(pgd_bases);

	dump_pgwalker_config();
	printk("Initialized Page Walker on cpu@%d\n", hard_smp_processor_id());
}

void dump_pgwalker_config(void)
{
#ifdef DEBUG_PGWALKER
	int i = 0;
	uint64_t pwbase_val = 0, pwfield_val = 0, pwsize_val = 0;
	uint32_t pwctl_val = 0;

	if (!is_nlm_xlp2xx()) {
		pgw_print_w(PGW_MMU_INFO);
		pgw_print_w(PGW_PGD_SHIFT);
		pgw_print_w(PGW_PGD_MASK);
		pgw_print_w(PGW_PMD_SHIFT);
		pgw_print_w(PGW_PMD_MASK);
		pgw_print_w(PGW_PTE_SHIFT);
		pgw_print_w(PGW_PTE_MASK);
		pgw_print_w(PGW_PUD_SHIFT);
		pgw_print_w(PGW_PUD_MASK);
	} else {
		pgw_print_w(PGW_MMU_SETUP);
	}

	printk("swapper_pg_dir = %lx\n", (unsigned long)swapper_pg_dir);
	for(i = 0; i < NR_ADDR_SEGMENTS; i++) {
		printk("pgd_bases[%d] = 0x%lx\n", i, __get_cpu_var(pgd_bases)[i]);
	}
	if (is_nlm_xlp2xx()) {
		pwbase_val = read_c0_pwbase();
		pwfield_val = read_c0_pwfield();
		pwsize_val = read_c0_pwsize();
		pwctl_val = read_c0_pwctl();
		printk("read pwbase: = %llx\n", pwbase_val);
		printk("read pwfield: = %llx\n", pwfield_val);
		printk("read pwsize: = %llx\n", pwsize_val);
		printk("read pwctl: = %x\n", pwctl_val);
	}
#endif
}

void mmu_init(void)
{
	uint32_t config4_val = 0, value;

	/* Removed pgwalker workaround for XLP8xx A silicon */

	if (is_nlm_xlp2xx()) {
		/* MMU_SETUP:
		 * [3]: using fixed TLB or not.
		 * [4]: PTE format compatible with XLP8XX/XLP4XX/XLP3XX, mostly useful for RI/XI support
		 */
		value = 0;
		if (tlb_config & ENABLE_ETLB)
			value |= 1 << 3;
		value |= 1 << 4;
		pgw_register_write_w(PGW_MMU_SETUP, value);
		DEBUG_PRINTK(KERN_INFO "[%s] PGW_MMU_SETUP = 0x%x\n", __func__, value);

		/* set config4 to kernel page size */
		config4_val = read_c0_config4();
		config4_val &= ~(((uint32_t)0x1f) << CFG4_FTLBPAGESIZE_O); /*clear 5-bit width field*/
		config4_val |= ((PAGE_SHIFT - 10) >> 1) << CFG4_FTLBPAGESIZE_O;
		write_c0_config4(config4_val);
		DEBUG_PRINTK(KERN_INFO "[%s] config4 = 0x%x\n", __func__, config4_val);
	} else {
		/*
	 	* shift right half the number of 1s in
	 	* the pagemask and populate that value
	 	*/
#ifdef CONFIG_HUGETLB_PAGE
		write_c0_config7(PM_DEFAULT_MASK >> (13 + (ffz(PM_DEFAULT_MASK >> 13) / 2))
			| ((PM_HUGE_MASK >> (13 + (ffz(PM_HUGE_MASK >> 13) / 2))) << 8));
#else
		write_c0_config7(PM_DEFAULT_MASK >> (13 + (ffz(PM_DEFAULT_MASK >> 13) / 2)));
#endif

#ifdef DEBUG_PGWALKER
		printk(KERN_INFO "( %s ): write_c0_config7 = %d\n", __FUNCTION__,
			read_c0_config7());
#endif

	}

	/*
	 * Read back TLB entries after configuration
	 */
	current_cpu_data.tlbsize = (read_c0_config6() >> 16 ) & 0xffff;

#ifdef CONFIG_EXEC_INHIBIT
	write_c0_pagegrain(read_c0_pagegrain() | EXEC_INHIBIT);
#endif

#ifdef CONFIG_READ_INHIBIT
	write_c0_pagegrain(read_c0_pagegrain() | READ_INHIBIT);
#endif

	if (is_nlm_xlp2xx()) {
		pgwalker_init_mips_compliant();
	} else {
		pgwalker_init();

		/* Intialize after pgwalker and others are configured! */
		write_c0_config6(read_c0_config6() | tlb_config);
	}
	tlbstats_init();
	entrylo0_mask_init();

}
