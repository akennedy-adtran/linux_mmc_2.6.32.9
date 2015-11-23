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
 * Copyright (C) 1994 - 2001, 2003 by Ralf Baechle
 * Copyright (C) 1999, 2000, 2001 Silicon Graphics, Inc.
 */
#ifndef _ASM_PGALLOC_H
#define _ASM_PGALLOC_H

#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/sched.h>

static inline void pmd_populate_kernel(struct mm_struct *mm, pmd_t *pmd,
	pte_t *pte)
{
	set_pmd(pmd, __pmd((unsigned long)pte));
}

static inline void pmd_populate(struct mm_struct *mm, pmd_t *pmd,
	pgtable_t pte)
{
	set_pmd(pmd, __pmd((unsigned long)page_address(pte)));
}
#define pmd_pgtable(pmd) pmd_page(pmd)

/*
 * Initialize a new pmd table with invalid pointers.
 */
extern void pmd_init(unsigned long page, unsigned long pagetable);

#ifdef CONFIG_64BIT

static inline void pud_populate(struct mm_struct *mm, pud_t *pud, pmd_t *pmd)
{
	set_pud(pud, __pud((unsigned long)pmd));
}
#endif

/*
 * Initialize a new pgd / pmd table with invalid pointers.
 */
extern void pgd_init(unsigned long page);

static inline pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *ret, *init;

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
	ret = (pgd_t *) __get_free_pages(GFP_DMA|GFP_KERNEL, PGD_ORDER);
#else
	ret = (pgd_t *) __get_free_pages(GFP_KERNEL, PGD_ORDER);
#endif
	if (ret) {
		init = pgd_offset(&init_mm, 0UL);
		pgd_init((unsigned long)ret);
		memcpy(ret + USER_PTRS_PER_PGD, init + USER_PTRS_PER_PGD,
		       (PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));
	}

	return ret;
}

static inline void pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
	free_pages((unsigned long)pgd, PGD_ORDER);
}

static inline pte_t *pte_alloc_one_kernel(struct mm_struct *mm,
	unsigned long address)
{
	pte_t *pte;

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
	pte = (pte_t *) __get_free_pages(GFP_DMA|GFP_KERNEL|__GFP_REPEAT|__GFP_ZERO, PTE_ORDER);
#else
	pte = (pte_t *) __get_free_pages(GFP_KERNEL|__GFP_REPEAT|__GFP_ZERO, PTE_ORDER);
#endif

	return pte;
}

static inline struct page *pte_alloc_one(struct mm_struct *mm,
	unsigned long address)
{
	struct page *pte;

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
	pte = alloc_pages(GFP_DMA | GFP_KERNEL | __GFP_REPEAT, PTE_ORDER);
#else
	pte = alloc_pages(GFP_KERNEL | __GFP_REPEAT, PTE_ORDER);
#endif
	if (pte) {
		clear_highpage(pte);
		pgtable_page_ctor(pte);
	}
	return pte;
}

static inline void pte_free_kernel(struct mm_struct *mm, pte_t *pte)
{
	free_pages((unsigned long)pte, PTE_ORDER);
}

static inline void pte_free(struct mm_struct *mm, pgtable_t pte)
{
	pgtable_page_dtor(pte);
	__free_pages(pte, PTE_ORDER);
}

#define __pte_free_tlb(tlb,pte,address)			\
do {							\
	pgtable_page_dtor(pte);				\
	tlb_remove_page((tlb), pte);			\
} while (0)

#ifdef CONFIG_64BIT

static inline pmd_t *pmd_alloc_one(struct mm_struct *mm, unsigned long address)
{
	pmd_t *pmd;

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
	pmd = (pmd_t *) __get_free_pages(GFP_DMA|GFP_KERNEL|__GFP_REPEAT, PMD_ORDER);
#else
	pmd = (pmd_t *) __get_free_pages(GFP_KERNEL|__GFP_REPEAT, PMD_ORDER);
#endif
	if (pmd)
		pmd_init((unsigned long)pmd, (unsigned long)invalid_pte_table);
	return pmd;
}

static inline void pmd_free(struct mm_struct *mm, pmd_t *pmd)
{
	free_pages((unsigned long)pmd, PMD_ORDER);
}

#define __pmd_free_tlb(tlb, x, addr)	pmd_free((tlb)->mm, x)

#endif

#define check_pgt_cache()	do { } while (0)

extern void pagetable_init(void);

#endif /* _ASM_PGALLOC_H */
