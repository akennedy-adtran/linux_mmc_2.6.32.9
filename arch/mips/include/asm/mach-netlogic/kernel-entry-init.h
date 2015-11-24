/*-
 * Copyright (c) 2005-2012 Broadcom Corporation
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

#ifndef __ASM_MACH_NLM_KERNEL_ENTRY_H
#define __ASM_MACH_NLM_KERNEL_ENTRY_H

/* XLP_MERGE_TODO */
#if !defined(CKSSEG)
#ifdef CONFIG_64BIT
#define CKSSEG			0xffffffffc0000000
#else
#define CKSSEG                  0xc0000000
#endif
#endif

#ifdef CONFIG_64BIT
#define LA dla
#define MTC0 dmtc0
#define SW sd
#else
#define LA la
#define MTC0 mtc0
#define SW sw
#endif

#ifdef CONFIG_CPU_XLP
#define JRHB jr.hb 
#define EHB ehb
#else
#define JRHB jr
#define EHB 
#endif

	/*
	 * inputs are the text nasid in t1, data nasid in t2.
	 */
	.macro MAPPED_KERNEL_SETUP_TLB
#ifdef CONFIG_MAPPED_KERNEL
	/*
	 * Drop in 0xffffffffc0000000 in tlbhi, 0+VG in tlblo_0,
	 * 0+DVG in tlblo_1.
	 */
	dli	    t3, CKSSEG
	dmtc0	t3, CP0_ENTRYHI
	li      t1, 0x1f
	MTC0	t1, CP0_ENTRYLO0	# physaddr, VG, cach exlwr
	li	    t2, 0x1
	MTC0	t2, CP0_ENTRYLO1	# physaddr, DVG, cach exlwr
	li	    t1, 0x1fffe000		# MAPPED_KERN_TLBMASK, TLBPGMASK_256M
	mtc0	t1, CP0_PAGEMASK
    mtc0    zero, CP0_INDEX
	tlbwi
	li      t0, 1
    mtc0	t0, CP0_WIRED
	EHB
    LA      v0, mapped_space
	JRHB    v0
	nop
mapped_space:
#else
	mtc0	zero, CP0_WIRED
#endif
	.endm

#endif /* __ASM_MACH_NLM_KERNEL_ENTRY_H */
