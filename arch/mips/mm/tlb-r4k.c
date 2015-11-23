/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 * Copyright (C) 1997, 1998, 1999, 2000 Ralf Baechle ralf@gnu.org
 * Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 2002 MIPS Technologies, Inc.  All rights reserved.
 */
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/system.h>

#ifdef CONFIG_NLM_COMMON
#include <asm/netlogic/mips-exts.h>
#include <asm/mach-netlogic/mmu.h>
#endif

#ifdef CONFIG_HUGETLB_PAGE
/* To work around the RAM TLB issue */
#define XLP_TLB_WORKAROUND
#endif

extern void build_tlb_refill_handler(void);

/*
 * Make sure all entries differ.  If they're not different
 * MIPS32 will take revenge ...
 */
#define UNIQUE_ENTRYHI(idx) (CKSEG0 + ((idx) << (PAGE_SHIFT + 1)))

/* Atomicity and interruptability */
#ifdef CONFIG_MIPS_MT_SMTC

#include <asm/smtc.h>
#include <asm/mipsmtregs.h>

#define ENTER_CRITICAL(flags) \
	{ \
	unsigned int mvpflags; \
	local_irq_save(flags);\
	mvpflags = dvpe()
#define EXIT_CRITICAL(flags) \
	evpe(mvpflags); \
	local_irq_restore(flags); \
	}
#else

#define ENTER_CRITICAL(flags) local_irq_save(flags)
#define EXIT_CRITICAL(flags) local_irq_restore(flags)

#endif /* CONFIG_MIPS_MT_SMTC */

#if defined(CONFIG_CPU_LOONGSON2)
/*
 * LOONGSON2 has a 4 entry itlb which is a subset of dtlb,
 * unfortrunately, itlb is not totally transparent to software.
 */
#define FLUSH_ITLB write_c0_diag(4);

#define FLUSH_ITLB_VM(vma) { if ((vma)->vm_flags & VM_EXEC)  write_c0_diag(4); }

#else

#define FLUSH_ITLB
#define FLUSH_ITLB_VM(vma)

#endif

#ifdef CONFIG_NLM_VMIPS
#define UNIQUE_VMIPS_ENTRYHI(idx)  ((1ULL << 63) + (1ULL << 40) + ((idx) << (PAGE_SHIFT + 1)) + ( 1 << 8))
extern int nlm_vmips_max_wired_entries;
#endif

void local_flush_tlb_all(void)
{
	unsigned long flags, config6_flags;
	unsigned long old_ctx;
	int entry;

	ENTER_CRITICAL(flags);
	disable_pgwalker(config6_flags);
	/* Save old context and create impossible VPN2 value */
	old_ctx = read_c0_entryhi();
	write_c0_entrylo0(0);
	write_c0_entrylo1(0);

	entry = read_c0_wired();

#if defined(CONFIG_MAPPED_KERNEL)
	if (!entry) printk("[%s] flushing entry=%d in MAPPED_KERNEL mode!\n",
			   __FUNCTION__, entry);
#endif
	/* Blast 'em all away. */
	while (entry < current_cpu_data.tlbsize) {
		/* Make sure all entries differ. */
#ifndef CONFIG_NLM_VMIPS
		write_c0_entryhi(UNIQUE_ENTRYHI(entry));
#else
        __write_64bit_c0_register($10, 0, (UNIQUE_VMIPS_ENTRYHI(entry)));
#endif
		write_c0_index(entry);
		mtc0_tlbw_hazard();
		tlb_write_indexed();
		entry++;
	}
	tlbw_use_hazard();
	write_c0_entryhi(old_ctx);
	FLUSH_ITLB;
	enable_pgwalker(config6_flags);
	EXIT_CRITICAL(flags);
}

/* All entries common to a mm share an asid.  To effectively flush
   these entries, we just bump the asid. */
void local_flush_tlb_mm(struct mm_struct *mm)
{
	int cpu;
	unsigned long config6_flags;

	preempt_disable();

	cpu = smp_processor_id();

	disable_pgwalker(config6_flags);
	if (cpu_context(cpu, mm) != 0) {
		drop_mmu_context(mm, cpu);
	}
	enable_pgwalker(config6_flags);
	preempt_enable();
}

void local_flush_tlb_range(struct vm_area_struct *vma, unsigned long start,
	unsigned long end)
{
	struct mm_struct *mm = vma->vm_mm;
	int cpu = smp_processor_id();

	if (cpu_context(cpu, mm) != 0) {
		unsigned long size, flags;
		unsigned long config6_flags;

		ENTER_CRITICAL(flags);
		disable_pgwalker(config6_flags);
		size = (end - start + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
		size = (size + 1) >> 1;
		if (size <= current_cpu_data.tlbsize/2) {
			int oldpid = read_c0_entryhi();
			int newpid = cpu_asid(cpu, mm);

			start &= (PAGE_MASK << 1);
			end += ((PAGE_SIZE << 1) - 1);
			end &= (PAGE_MASK << 1);
			while (start < end) {
				int idx;

				write_c0_entryhi(start | newpid);
				start += (PAGE_SIZE << 1);
				mtc0_tlbw_hazard();
				tlb_probe();
				tlb_probe_hazard();
				idx = read_c0_index();
				write_c0_entrylo0(0);
				write_c0_entrylo1(0);
				if (idx < 0)
					continue;
				/* Make sure all entries differ. */
#ifndef CONFIG_NLM_VMIPS
				write_c0_entryhi(UNIQUE_ENTRYHI(idx));
#else
				__write_64bit_c0_register($10, 0, (UNIQUE_VMIPS_ENTRYHI(idx)));
#endif
				mtc0_tlbw_hazard();
				tlb_write_indexed();
			}
			tlbw_use_hazard();
			write_c0_entryhi(oldpid);
		} else {
			drop_mmu_context(mm, cpu);
		}
		FLUSH_ITLB;
		enable_pgwalker(config6_flags);
		EXIT_CRITICAL(flags);
	}
}

void local_flush_tlb_kernel_range(unsigned long start, unsigned long end)
{
	unsigned long size, flags;
	unsigned long config6_flags;

	ENTER_CRITICAL(flags);
	disable_pgwalker(config6_flags);
	size = (end - start + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
	size = (size + 1) >> 1;
	if (size <= current_cpu_data.tlbsize / 2) {
		int pid = read_c0_entryhi();

		start &= (PAGE_MASK << 1);
		end += ((PAGE_SIZE << 1) - 1);
		end &= (PAGE_MASK << 1);

		while (start < end) {
			int idx;

			write_c0_entryhi(start);
			start += (PAGE_SIZE << 1);
			mtc0_tlbw_hazard();
			tlb_probe();
			tlb_probe_hazard();
			idx = read_c0_index();
			write_c0_entrylo0(0);
			write_c0_entrylo1(0);
			if (idx < 0)
				continue;
			/* Make sure all entries differ. */
#ifndef CONFIG_NLM_VMIPS
			write_c0_entryhi(UNIQUE_ENTRYHI(idx));
#else
			__write_64bit_c0_register($10, 0, (UNIQUE_VMIPS_ENTRYHI(idx)));
#endif
			mtc0_tlbw_hazard();
			tlb_write_indexed();
		}
		tlbw_use_hazard();
		write_c0_entryhi(pid);
	} else {
		local_flush_tlb_all();
	}
	FLUSH_ITLB;
	enable_pgwalker(config6_flags);
	EXIT_CRITICAL(flags);
}

void local_flush_tlb_page(struct vm_area_struct *vma, unsigned long page)
{
	int cpu = smp_processor_id();

	if (cpu_context(cpu, vma->vm_mm) != 0) {
		unsigned long flags, config6_flags;
		int oldpid, newpid, idx;

		newpid = cpu_asid(cpu, vma->vm_mm);
		page &= (PAGE_MASK << 1);
		ENTER_CRITICAL(flags);
		disable_pgwalker(config6_flags);
		oldpid = read_c0_entryhi();
		write_c0_entryhi(page | newpid);
		mtc0_tlbw_hazard();
		tlb_probe();
		tlb_probe_hazard();
		idx = read_c0_index();
		write_c0_entrylo0(0);
		write_c0_entrylo1(0);
		if (idx < 0)
			goto finish;
		/* Make sure all entries differ. */
#ifndef CONFIG_NLM_VMIPS
		write_c0_entryhi(UNIQUE_ENTRYHI(idx));
#else
		__write_64bit_c0_register($10, 0, (UNIQUE_VMIPS_ENTRYHI(idx)));
#endif
		mtc0_tlbw_hazard();
		tlb_write_indexed();
		tlbw_use_hazard();

	finish:
		write_c0_entryhi(oldpid);
		FLUSH_ITLB_VM(vma);
		enable_pgwalker(config6_flags);
		EXIT_CRITICAL(flags);
	}
}

#ifdef CONFIG_HUGETLB_PAGE
asmlinkage void do_hugetlb_invalidate(void)
{
	int oldpid, idx;

	oldpid = read_c0_entryhi();
	tlb_probe();
	tlb_probe_hazard();
	idx = read_c0_index();
	if (idx > 0) {
		int ridx = idx & 0x1fff;

		if (ridx > ((read_c0_config6() >> 6) & 0x3ff)) {
			/* Make sure all entries differ. */
			write_c0_entrylo0(0);
			write_c0_entrylo1(0);
			write_c0_entryhi(UNIQUE_ENTRYHI(idx & 0x1fff));
			mtc0_tlbw_hazard();
			tlb_write_indexed();
			tlbw_use_hazard();
			write_c0_entryhi(oldpid);
		}
	}
}
#endif

/*
 * This one is only used for pages with the global bit set so we don't care
 * much about the ASID.
 */
void local_flush_tlb_one(unsigned long page)
{
	unsigned long flags, config6_flags;
	int oldpid, idx;

	ENTER_CRITICAL(flags);
	disable_pgwalker(config6_flags);
	oldpid = read_c0_entryhi();
	page &= (PAGE_MASK << 1);
	write_c0_entryhi(page);
	mtc0_tlbw_hazard();
	tlb_probe();
	tlb_probe_hazard();
	idx = read_c0_index();
	write_c0_entrylo0(0);
	write_c0_entrylo1(0);
	if (idx >= 0) {
		/* Make sure all entries differ. */
#ifndef CONFIG_NLM_VMIPS
		write_c0_entryhi(UNIQUE_ENTRYHI(idx));
#else
		__write_64bit_c0_register($10, 0, (UNIQUE_VMIPS_ENTRYHI(idx)));
#endif
		mtc0_tlbw_hazard();
		tlb_write_indexed();
		tlbw_use_hazard();
	}
	write_c0_entryhi(oldpid);
	FLUSH_ITLB;
	enable_pgwalker(config6_flags);
	EXIT_CRITICAL(flags);
}

/*
 * We will need multiple versions of update_mmu_cache(), one that just
 * updates the TLB with the new pte(s), and another which also checks
 * for the R4k "end of page" hardware bug and does the needy.
 */
void __update_tlb(struct vm_area_struct * vma, unsigned long address, pte_t pte)
{
	unsigned long flags;
	pgd_t *pgdp;
	pud_t *pudp;
	pmd_t *pmdp;
	pte_t *ptep;
	int idx, pid;
	unsigned long config6_flags;

	/*
	 * Handle debugger faulting in for debugee.
	 */
	if (current->active_mm != vma->vm_mm)
		return;

	ENTER_CRITICAL(flags);
	disable_pgwalker(config6_flags);

	pid = read_c0_entryhi() & ASID_MASK;
	address &= (PAGE_MASK << 1);
	write_c0_entryhi(address | pid);
	pgdp = pgd_offset(vma->vm_mm, address);
	mtc0_tlbw_hazard();
	tlb_probe();
	tlb_probe_hazard();
	pudp = pud_offset(pgdp, address);
	pmdp = pmd_offset(pudp, address);
	idx = read_c0_index();
#ifdef CONFIG_HUGETLB_PAGE
	/* this could be a huge page  */
	if (is_vm_hugetlb_page(vma)) {
		write_c0_pagemask(PM_HUGE_MASK);
		ptep = pte_offset_map(pmdp, address);
		write_c0_entrylo0(pte_to_entrylo(pte_val(*ptep++)));
		write_c0_entrylo1(pte_to_entrylo(pte_val(*ptep)));

		mtc0_tlbw_hazard();
		if (idx < 0)
			tlb_write_random();
		else {
#ifndef XLP_TLB_WORKAROUND
			tlb_write_indexed();
#else
			int ridx = idx & 0x1fff;
			if (ridx > ((read_c0_config6() >> 6) & 0x3ff)) {
				tlb_write_random();
			}
			else
				tlb_write_indexed();
#endif
		}
		write_c0_pagemask(PM_DEFAULT_MASK);
	} else
#endif
	{
		ptep = pte_offset_map(pmdp, address);

#if defined(CONFIG_64BIT_PHYS_ADDR) && defined(CONFIG_CPU_MIPS32)
		write_c0_entrylo0(ptep->pte_high);
		ptep++;
		write_c0_entrylo1(ptep->pte_high);
#else
		write_c0_entrylo0(pte_to_entrylo(pte_val(*ptep++)));
		write_c0_entrylo1(pte_to_entrylo(pte_val(*ptep)));
#endif
		mtc0_tlbw_hazard();
		if (idx < 0)
			tlb_write_random();
		else
			tlb_write_indexed();
	}
	tlbw_use_hazard();
	FLUSH_ITLB_VM(vma);
	enable_pgwalker(config6_flags);
	EXIT_CRITICAL(flags);
}

#if 0
static void r4k_update_mmu_cache_hwbug(struct vm_area_struct * vma,
				       unsigned long address, pte_t pte)
{
	unsigned long flags;
	unsigned int asid;
	pgd_t *pgdp;
	pmd_t *pmdp;
	pte_t *ptep;
	int idx;

	ENTER_CRITICAL(flags);
	address &= (PAGE_MASK << 1);
	asid = read_c0_entryhi() & ASID_MASK;
	write_c0_entryhi(address | asid);
	pgdp = pgd_offset(vma->vm_mm, address);
	mtc0_tlbw_hazard();
	tlb_probe();
	tlb_probe_hazard();
	pmdp = pmd_offset(pgdp, address);
	idx = read_c0_index();
	ptep = pte_offset_map(pmdp, address);
	write_c0_entrylo0(pte_val(*ptep++) >> 6);
	write_c0_entrylo1(pte_val(*ptep) >> 6);
	mtc0_tlbw_hazard();
	if (idx < 0)
		tlb_write_random();
	else
		tlb_write_indexed();
	tlbw_use_hazard();
	EXIT_CRITICAL(flags);
}
#endif

void __init add_wired_entry(unsigned long entrylo0, unsigned long entrylo1,
	unsigned long entryhi, unsigned long pagemask)
{
	unsigned long flags;
	unsigned long wired;
	unsigned long old_pagemask;
	unsigned long old_ctx;
	unsigned long config6_flags;

	ENTER_CRITICAL(flags);
	disable_pgwalker(config6_flags);
	/* Save old context and create impossible VPN2 value */
	old_ctx = read_c0_entryhi();
	old_pagemask = read_c0_pagemask();
	wired = read_c0_wired();
	write_c0_wired(wired + 1);
	write_c0_index(wired);
	tlbw_use_hazard();	/* What is the hazard here? */
	write_c0_pagemask(pagemask);
	write_c0_entryhi(entryhi);
	write_c0_entrylo0(entrylo0);
	write_c0_entrylo1(entrylo1);
	mtc0_tlbw_hazard();
	tlb_write_indexed();
	tlbw_use_hazard();

	write_c0_entryhi(old_ctx);
	tlbw_use_hazard();	/* What is the hazard here? */
	write_c0_pagemask(old_pagemask);
	local_flush_tlb_all();
	enable_pgwalker(config6_flags);
	EXIT_CRITICAL(flags);
}

/*
 * Used for loading TLB entries before trap_init() has started, when we
 * don't actually want to add a wired entry which remains throughout the
 * lifetime of the system
 */

static int temp_tlb_entry __cpuinitdata;

__init int add_temporary_entry(unsigned long entrylo0, unsigned long entrylo1,
			       unsigned long entryhi, unsigned long pagemask)
{
	int ret = 0;
	unsigned long flags;
	unsigned long wired;
	unsigned long old_pagemask;
	unsigned long old_ctx;
	unsigned long config6_flags;

	ENTER_CRITICAL(flags);
	disable_pgwalker(config6_flags);
	/* Save old context and create impossible VPN2 value */
	old_ctx = read_c0_entryhi();
	old_pagemask = read_c0_pagemask();
	wired = read_c0_wired();
	if (--temp_tlb_entry < wired) {
		printk(KERN_WARNING
		       "No TLB space left for add_temporary_entry\n");
		ret = -ENOSPC;
		goto out;
	}

	write_c0_index(temp_tlb_entry);
	write_c0_pagemask(pagemask);
	write_c0_entryhi(entryhi);
	write_c0_entrylo0(entrylo0);
	write_c0_entrylo1(entrylo1);
	mtc0_tlbw_hazard();
	tlb_write_indexed();
	tlbw_use_hazard();

	write_c0_entryhi(old_ctx);
	write_c0_pagemask(old_pagemask);
out:
	enable_pgwalker(config6_flags);
	EXIT_CRITICAL(flags);
	return ret;
}

static void __cpuinit probe_tlb(unsigned long config)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	unsigned int reg __attribute__((unused));

	/*
	 * If this isn't a MIPS32 / MIPS64 compliant CPU.  Config 1 register
	 * is not supported, we assume R4k style.  Cpu probing already figured
	 * out the number of tlb entries.
	 */
	if ((c->processor_id & 0xff0000) == PRID_COMP_LEGACY)
		return;
#ifdef CONFIG_MIPS_MT_SMTC
	/*
	 * If TLB is shared in SMTC system, total size already
	 * has been calculated and written into cpu_data tlbsize
	 */
	if((smtc_status & SMTC_TLB_SHARED) == SMTC_TLB_SHARED)
		return;
#endif /* CONFIG_MIPS_MT_SMTC */

	reg = read_c0_config1();
	if (!((config >> 7) & 7))
		panic("No TLB present");

#if defined(CONFIG_NLM_XLP)
	c->tlbsize = ((read_c0_config6() >> 16 ) & 0xffff) + 1;
#else
	c->tlbsize = ((reg >> 25) & 0x3f) + 1;
#endif

}

static int __cpuinitdata ntlb;
static int __init set_ntlb(char *str)
{
	get_option(&str, &ntlb);
	return 1;
}

__setup("ntlb=", set_ntlb);

#ifdef CONFIG_NLM_COMMON
extern void nlm_common_tlb_init(void);

void nlm_tlb_stats_init(void)
{
	nlm_write_os_scratch_2(0ULL);
}
#endif

void __cpuinit tlb_init(void)
{
	unsigned int config = read_c0_config();

	/*
	 * You should never change this register:
	 *   - On R4600 1.7 the tlbp never hits for pages smaller than
	 *     the value in the c0_pagemask register.
	 *   - The entire mm handling assumes the c0_pagemask register to
	 *     be set to fixed-size pages.
	 */
	probe_tlb(config);
	write_c0_pagemask(PM_DEFAULT_MASK);

#if defined(CONFIG_NLM_VMIPS)
	if(ntlb && ((current_cpu_data.tlbsize-ntlb) < nlm_vmips_max_wired_entries))
		ntlb = current_cpu_data.tlbsize - nlm_vmips_max_wired_entries;
#endif

	if (current_cpu_type() == CPU_R10000 ||
	    current_cpu_type() == CPU_R12000 ||
	    current_cpu_type() == CPU_R14000)
		write_c0_framemask(0);

#if !defined(CONFIG_MAPPED_KERNEL)
	write_c0_wired(0);
	write_c0_framemask(0);
#endif

   if (kernel_uses_smartmips_rixi) {
       /*
        * Enable the no read, no exec bits, and enable large virtual
        * address.
        */
       u32 pg = PG_RIE | PG_XIE;
#ifdef CONFIG_64BIT
       pg |= PG_ELPA;
#endif
       write_c0_pagegrain(pg);
   }

	temp_tlb_entry = current_cpu_data.tlbsize - 1;

        /* From this point on the ARC firmware is dead.  */
	local_flush_tlb_all();

	/* Did I tell you that ARC SUCKS?  */

	if (ntlb) {
		if (ntlb > 1 && ntlb <= current_cpu_data.tlbsize) {
			int wired = current_cpu_data.tlbsize - ntlb;
			write_c0_wired(wired);
			write_c0_index(wired-1);
			printk("Restricting TLB to %d entries\n", ntlb);
		} else
			printk("Ignoring invalid argument ntlb=%d\n", ntlb);
	}

#ifdef CONFIG_NLM_COMMON
	nlm_tlb_stats_init();
#endif

    build_tlb_refill_handler();
}
