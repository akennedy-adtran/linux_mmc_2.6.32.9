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

#ifndef _XLR_MACROS_H
#define _XLR_MACROS_H

#ifndef __ASSEMBLY__

#ifdef CONFIG_32BIT
#define XKPHYS        0x8000000000000000ULL
#endif

#define CCA_UNCACHED  0x1000000000000000ULL
#define CCA_CACHED    0x1800000000000000ULL

#define enable_KX(flags)   \
  preempt_disable();       \
 __asm__ __volatile__ (    \
	".set push\n"          \
	".set noat\n"          \
	".set noreorder\n"     \
	"mfc0 %0, $12\n\t"     \
	"ori $1, %0, 0x81\n\t" \
	"xori $1, 1\n\t"       \
	"mtc0 $1, $12\n"       \
    ".set pop\n"           \
    : "=r"(flags) );       \
  preempt_enable();
	
#define disable_KX(flags)  \
 __asm__ __volatile__ (    \
	".set push\n"          \
	"mtc0 %0, $12\n"       \
    ".set pop\n"           \
    :                      \
    : "r"(flags))

static __inline__ uint8_t lb_40bit_phys(uint64_t phys, uint64_t cca)
{
	uint32_t lsw, msw;
	uint8_t value;
	unsigned long flags;

	phys &= 0xffffffffffULL;
	phys |= (XKPHYS | cca);
	lsw = (uint32_t) phys & 0xffffffff;
	msw = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(
		".set push\n"
        ".set noreorder\n"
        ".set mips64\n"
		".set noat\n"
		"dsll32 $1, %2, 0\n"
		"dsll32 %1, 0\n"
		"dsrl32 %1, 0\n"
        "or $1, $1, %1\n"
        "lb %0, 0($1) \n"
		".set at\n"
        ".set pop\n"
        : "=r"(value)
        : "r"(lsw), "r"(msw)

		: "$1"
		);
	disable_KX(flags);

	return value;
}
static __inline__ uint8_t lb_40bit_phys_uncached(uint64_t phys)
{
        return lb_40bit_phys(phys, CCA_UNCACHED);
}
static __inline__ uint8_t lb_40bit_phys_cached(uint64_t phys)
{
        return lb_40bit_phys(phys, CCA_CACHED);
}

static __inline__ void sb_40bit_phys(uint64_t phys, uint64_t cca, uint8_t value)
{
	uint32_t lsw, msw;
	unsigned long flags;

	phys &= 0xffffffffffULL;
	phys |= (XKPHYS | cca);
	lsw = (uint32_t) phys & 0xffffffff;
	msw = (uint32_t) (phys >> 32);
	
	enable_KX(flags);
	__asm__ __volatile__(
		".set push\n"
        ".set noreorder\n"
        ".set mips64\n"
		".set noat\n"
		"dsll32 $1, %2, 0\n"
		"dsll32 %1, 0\n"
		"dsrl32 %1, 0\n"
        "or $1, $1, %1\n"
        "sb %0, 0($1) \n"
		".set at\n"
        ".set pop\n"
		:
        : "r"(value), "r"(lsw), "r"(msw)
		: "$1"
		);
	disable_KX(flags);
}
static __inline__ void sb_40bit_phys_uncached(uint64_t phys, uint8_t value)
{
      sb_40bit_phys(phys, CCA_UNCACHED, value);
}
static __inline__ void sb_40bit_phys_cached(uint64_t phys, uint8_t value)
{
      sb_40bit_phys(phys, CCA_CACHED, value);
}

#endif

#endif
