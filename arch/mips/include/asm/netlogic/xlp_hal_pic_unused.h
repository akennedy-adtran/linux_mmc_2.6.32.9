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


#ifndef _XLP_HAL_PIC_H
#define _XLP_HAL_PIC_H

#if !defined __ASSEMBLY__
#include "nlm_hal.h"

static __inline__ int nlm_hal_cpu_id(void)
{
	int cpu;

	__asm__ __volatile__ (
		".set push\n"
		".set noreorder\n"
		".set mips32\n"
		"mfc0 %0, $15, 1\n"
		"andi %0, %0, 0x3ff\n"
		".set pop\n"
		: "=r"(cpu)
		);

	return cpu;
}

typedef volatile unsigned long long pic_reg_t;

static __inline__ pic_reg_t* nlm_hal_pic_offset(void)
{
	uint32_t cpu = nlm_hal_cpu_id();

	return ((pic_reg_t *) (XLP_IO_PIC_OFFSET + NODE_OFFSET(CPU_TO_NODE(cpu))));
}

#ifdef CONFIG_64BIT

static __inline__ void nlm_hal_write_pic_reg(pic_reg_t *base,
		unsigned int offset, unsigned long long value)
{
	base[offset] = value;
}

static __inline__ unsigned long long nlm_hal_read_pic_reg(pic_reg_t *base,
		unsigned int offset)
{
	return ((base)[offset]);
}

#else

static __inline__ void nlm_hal_write_pic_reg(pic_reg_t *base, unsigned int offset, unsigned long long value)
{
        uint32_t lsw, msw;
        uint64_t val;
        uint32_t ls, ms;
        unsigned long flags;

        lsw = (uint32_t) (base+offset);
        msw = (uint32_t) 0xffffffffUL;
        val = (uint64_t)value;

        ls = (uint32_t) (val & 0xffffffff);
        ms = (uint32_t) (val >> 32);

        enable_KX(flags);
        __asm__ __volatile__(".set push\n"
                        ".set noreorder\n"
                        ".set mips64\n"
                        ".set noat\n"
                        "dsll32 $1, %2, 0\n"
                        "dsll32 %1, 0\n"
                        "dsrl32 %1, 0\n"
                        "or $1, $1, %1\n"
                        "dsll32 $8, %4, 0\n"
                        "dsll32 %3, 0\n"
                        "dsrl32 %3, 0\n"
                        "or $8, $8, %3\n"
                        "sd $8, 0($1) \n"
                        ".set at\n"
                        ".set pop\n"
                        :
                        :"r"(val), "r"(lsw), "r"(msw), "r"(ls), "r"(ms)
                        :"$1", "$8");
        disable_KX(flags);
}

static __inline__ unsigned long long nlm_hal_read_pic_reg(pic_reg_t *base, unsigned int offset)
{
        uint32_t lsw, msw;
        uint64_t value = 0;
        uint32_t lo, hi;
        unsigned long flags;

        lsw = (uint32_t) (base+offset);
        msw = (uint32_t) 0xffffffffUL;

        enable_KX(flags);
        __asm__ __volatile__(".set push\n"
                        ".set noreorder\n"
                        ".set mips64\n"
                        ".set noat\n"
                        "dsll32 $1, %3, 0\n"
                        "dsll32 %2, 0\n"
                        "dsrl32 %2, 0\n"
                        "or $1, $1, %2\n"
                        "ld $8, 0($1) \n"
                        "dsrl32 %1, $8, 0\n"
                        "dsll32 $8, $8, 0\n"
                        "dsrl32 %0, $8, 0\n"
                        ".set at\n"
                        ".set pop\n"
                        :"=r"(lo), "=r"(hi)
                        :"r"(lsw), "r"(msw)
                        :"$1", "$8");

        disable_KX(flags);
        value = hi;
        value = (uint64_t) ((value<<32) | lo);
        return (value);
}
#endif


#endif /* __ASSEMBLY__ */

#endif /* _NLM_HAL_PIC_H */
