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
 * Copyright (C) 1999 Ralf Baechle
 * Copyright (C) 1999 Silicon Graphics, Inc.
 */
#include <asm/mipsregs.h>
#include <asm/page.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>

	.macro tlb_do_page_fault, write
	NESTED(tlb_do_page_fault_\write, PT_SIZE, sp)
	SAVE_ALL
	MFC0	a2, CP0_BADVADDR

#ifdef CONFIG_HUGETLB_PAGE
	/* invalidate the tlb entry in fixed tlb */
	jal	do_hugetlb_invalidate
	nop
#endif

	KMODE
	move	a0, sp
	REG_S	a2, PT_BVADDR(sp)
	li	a1, \write
	PTR_LA	ra, ret_from_exception
	j	do_page_fault
	END(tlb_do_page_fault_\write)
	.endm

	tlb_do_page_fault 0
	tlb_do_page_fault 1
#if defined(CONFIG_READ_INHIBIT) || defined(CONFIG_EXEC_INHIBIT)
	tlb_do_page_fault 2
#endif
