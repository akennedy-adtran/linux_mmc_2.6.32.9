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


#ifndef _ASM_NLM_PIC_H
#define _ASM_NLM_PIC_H

#include <asm/netlogic/iomap.h>

#ifndef __ASSEMBLY__
struct pt_regs;
extern void nlm_common_ipi_handler(int irq, struct pt_regs *regs);
extern void nlm_common_msgring_int_handler(unsigned int irq, struct pt_regs *regs);

struct pic_tmask { 
	unsigned int mask; 
	int set; 
	int valid;
};

#endif

#if defined(CONFIG_NLM_XLP)

// can't do floating in the kernel, so use 64 as an approximation 
#define PIC_CLKS_PER_SEC 133333333ULL
#define PIC_CLKS_PER_USEC 133	//(PIC_CLKS_PER_SEC / 1000000)
#define PIC_CLKS_PER_TIMER_TICK (PIC_CLKS_PER_SEC / HZ)

#else
// can't do floating in the kernel, so use 64 as an approximation 
#define PIC_CLKS_PER_SEC 66666666ULL
#define PIC_CLKS_PER_USEC 66	//(PIC_CLKS_PER_SEC / 1000000)
#define PIC_CLKS_PER_TIMER_TICK (PIC_CLKS_PER_SEC / HZ)

#include <asm/netlogic/xlr_pic.h>
#endif /* CONFIG_NLM_XLP */

#endif /* #ifndef _ASM_NLM_PIC_H */
	
