/*-
 * Copyright 2009-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
 * MIPS Huge TLB Page Support for Kernel.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2002, Rohit Seth <rohit.seth@intel.com>
 * Copyright 2005, Embedded Alley Solutions, Inc.
 * Matt Porter <mporter@embeddedalley.com>
 * Copyright (C) 2008, 2009 Cavium Networks, Inc.
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/pagemap.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/sysctl.h>
#include <asm/mman.h>
#include <asm/tlb.h>
#include <asm/tlbflush.h>

pte_t *huge_pte_alloc_single(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte = NULL;

	pgd = pgd_offset(mm, addr);
	pud = pud_alloc(mm, pgd, addr);
	if (pud) {
		pmd = (pmd_t *)pmd_alloc(mm, pud, addr);
		if (pmd)
			pte = pte_alloc_map(mm, pmd, addr);
	}

	return pte;
}

/**
 * Given any address, we need to allocate page table entries
 * for all pte's covered by the same huge page. This is needed if
 * any address referencing the huge page faults and the tlb refill handler
 * can refill the tlb entry with correct value.
 *
 * Return any valid pte pointer is fine as later on we still have
 * "addr" to identify the correct huge page.
 */
pte_t *huge_pte_alloc(struct mm_struct *mm, unsigned long addr,
		      unsigned long sz)
{
	pte_t *pte = NULL;
	unsigned long i = 0;
	unsigned long htlb_entries = 1 << HUGETLB_PAGE_ORDER;

	addr &= HPAGE_MASK;
	for (i = 0; i < htlb_entries; i++) {
		pte = huge_pte_alloc_single(mm, addr);
		if (!pte)
			return NULL;
		addr += PAGE_SIZE;
	}
	return pte;
}

pte_t *huge_pte_offset(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte = NULL;

	pgd = pgd_offset(mm, addr);
	if (pgd_present(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (pud_present(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (pmd_present(*pmd))
				pte = pte_offset_map(pmd, addr);
		}
	}
	return pte;
}

/**
 * Fill the pte value to all pte's covered by the same huge page.
 */
void set_huge_pte_at(struct mm_struct *mm, unsigned long addr, pte_t *ptep,
pte_t entry)
{
	unsigned long i;
	unsigned long htlb_entries = 1 << HUGETLB_PAGE_ORDER;
	pte_t entry2;

	entry2 =  __pte(pte_val(entry) + (HPAGE_SIZE >> 1));

	/* for hardware page walker, bit 61 tells hpw it is a hpage */
	entry  = __pte(pte_val(entry)  | (1ULL << 61));
	entry2 = __pte(pte_val(entry2) | (1ULL << 61));

	addr &= HPAGE_MASK;
	for (i = 0; i < htlb_entries; i += 2) {
		ptep = huge_pte_offset(mm, addr);
		set_pte_at(mm, addr, ptep, entry);
		addr += PAGE_SIZE;

		ptep = huge_pte_offset(mm, addr);
		set_pte_at(mm, addr, ptep, entry2);
		addr += PAGE_SIZE;
	}
}

pte_t huge_ptep_get_and_clear(struct mm_struct *mm, unsigned long addr,
				pte_t *ptep)
{
	pte_t entry;
	unsigned long i;
	unsigned long htlb_entries = 1 << HUGETLB_PAGE_ORDER;

	entry = *ptep;

	/* clear bit 61 before giving back to the upper level function */
	entry = __pte(pte_val(entry) & ~(1ULL << 61));

	addr &= HPAGE_MASK;
	for (i = 0; i < htlb_entries; i++) {
		ptep = huge_pte_offset(mm, addr);
		pte_clear(mm, addr, ptep);
		addr += PAGE_SIZE;
	}
	return entry;
}

int huge_pmd_unshare(struct mm_struct *mm, unsigned long *addr, pte_t *ptep)
{
	return 0;
}

/*
 * This function checks for proper alignment of input addr and len parameters.
 */
int is_aligned_hugepage_range(unsigned long addr, unsigned long len)
{
	if (len & ~HPAGE_MASK)
		return -EINVAL;
	if (addr & ~HPAGE_MASK)
		return -EINVAL;
	return 0;
}

struct page *
follow_huge_addr(struct mm_struct *mm, unsigned long address, int write)
{
	return ERR_PTR(-EINVAL);
}

int pmd_huge(pmd_t pmd)
{
	return (pmd_val(pmd) & _PAGE_HUGE) != 0;
}

int pud_huge(pud_t pud)
{
	return (pud_val(pud) & _PAGE_HUGE) != 0;
}

struct page *
follow_huge_pmd(struct mm_struct *mm, unsigned long address,
		pmd_t *pmd, int write)
{
	struct page *page;

	page = pte_page(*(pte_t *)pmd);
	if (page)
		page += ((address & ~HPAGE_MASK) >> PAGE_SHIFT);
	return page;
}

