/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * http://www.gnu.org/licenses/gpl-2.0.txt  
 * or the Broadcom license below:

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
 * #BRCM_4# */

#ifndef _NLM_HAL_MACROS_H
#define _NLM_HAL_MACROS_H

#ifndef __ASSEMBLY__
extern unsigned long xlp_io_base;
extern unsigned long xlp_fmn_base[];
extern unsigned long xlp_nae_base[];
extern unsigned long xlp_mac_base[];
extern unsigned long xlp_poe_base_pcie[];
extern unsigned long xlp_poe_base_pcim[];
extern unsigned long xlp_sys_base[];

extern int nlm_chip_is_xlp3xx;
extern unsigned long xlp_regex_base_pcie;
extern unsigned long xlp_regex_base_pcim;

#ifndef is_nlm_xlp8xx
extern int is_nlm_xlp(unsigned int chipid, unsigned int rev, unsigned int ext);

#define XLP_REVISION_ANY	0xFF
#define is_nlm_xlp8xx()		(is_nlm_xlp(0x8000, XLP_REVISION_ANY, 0) || is_nlm_xlp(0x4000, XLP_REVISION_ANY, 0))
#endif /* is_nlm_xlp8xx */

#endif /* #ifndef __ASSEMBLY__ */

#if defined(NLM_HAL_LINUX_USER) /* Linux User mode */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#define nlm_print			printf
#define nlm_malloc			malloc
#define nlm_free			free
#if (_MIPS_SIM == _MIPS_SIM_ABI32) || (_MIPS_SIM == _MIPS_SIM_NABI32) || \
	(_MIPS_SZLONG == 32)
#define KSEG0				0x80000000UL
#define KSEG1				0xA0000000UL
#define KSEG2				0xC0000000UL
#define KSEG3				0xE0000000UL
#define KSEG0_PHY_BOUNDARY	0x10000000UL	/* 256 MB */
#else
#define KSEG0				(0xffffffff80000000ULL)
#define KSEG1				(0xffffffffA0000000ULL)
#define KSEG2				(0xffffffffC0000000ULL)
#define KSEG3				(0xffffffffE0000000ULL)
#define KSEG0_PHY_BOUNDARY	0x10000000ULL	/* 256 MB */
#endif

/* Adding a new macro for mdelay. */
#define nlm_udelay(n)		usleep(n)
#define nlm_mdelay(n)		usleep(n * 1000) 

#elif defined(NLM_HAL_LINUX_KERNEL) /* Linux Kernel mode */

#include <asm/mipsregs.h>
#ifndef __ASSEMBLY__
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/delay.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0))
#include <linux/slab.h>
#include <asm/atomic.h>
#endif

#define nlm_print			printk
#define nlm_malloc(size)	kmalloc((size), GFP_KERNEL)
#define nlm_free			kfree

static inline unsigned long nlm_spill_alloc(int node, uint64_t size)
{
		struct page *pg;
		pg = alloc_pages_exact_node(node, GFP_KERNEL, get_order(size));
	if (pg == NULL) {
		nlm_print("Spill Mem allocation on node %d failed \n", node);
		return 0;
	}
		return page_to_phys(pg);
}

#if (_MIPS_SIM == _MIPS_SIM_ABI32) || (_MIPS_SIM == _MIPS_SIM_NABI32) || \
	(_MIPS_SZLONG == 32)
#ifndef KSEG0
#define KSEG0				0x80000000UL
#endif
#ifndef KSEG1
#define KSEG1				0xA0000000UL
#endif
#ifndef KSEG2
#define KSEG2				0xC0000000UL
#endif
#ifndef KSEG3
#define KSEG3				0xE0000000UL
#endif
#define KSEG0_PHY_BOUNDARY	0x10000000UL	/* 256 MB */

#else
#define KSEG0				(0xffffffff80000000ULL)
#define KSEG1				(0xffffffffA0000000ULL)
#define KSEG2				(0xffffffffC0000000ULL)
#define KSEG3				(0xffffffffE0000000ULL)
#define KSEG0_PHY_BOUNDARY	0x10000000ULL	/* 256 MB */
#endif

#define nlm_udelay(n)		udelay(n)
#define nlm_mdelay(n)		mdelay(n)

#endif /* __ASSEMBLY__ */

#elif defined(NLM_HAL_NETOS) /* Netos */

#ifndef __ASSEMBLY__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#define nlm_print			printf
#define nlm_malloc			malloc
#define nlm_free			free
#define nlm_udelay(x)		_netos_delay(x) /* Temporary implementation for netos-hyperex */
#define nlm_mdelay(n)		nlm_udelay(n * 1000)

static __inline__ void _netos_delay(unsigned int x)
{
	unsigned long int i;

	/* compilers beyond gcc 4.0 will remove off tight loops
	 * when optimization is enabled. This asm call is
	 * supposedly the standard way to work around this.
	 */
	for (i = 0; i<(1000 * x) ; i++)
		__asm__ __volatile__ ("");
}
#endif /* __ASSEMBLY__ */

#if (_MIPS_SIM == _MIPS_SIM_ABI32) || (_MIPS_SIM == _MIPS_SIM_NABI32) || \
	(_MIPS_SZLONG == 32)
#define KSEG0				0x80000000
#define KSEG1				0xA0000000
#define KSEG2				0xC0000000
#define KSEG3				0xE0000000
#define KSEG0_PHY_BOUNDARY	0x10000000	/* 256 MB */
#else
#define KSEG0				(0xffffffff80000000)
#define KSEG1				(0xffffffffA0000000)
#define KSEG2				(0xffffffffC0000000)
#define KSEG3				(0xffffffffE0000000)
#define KSEG0_PHY_BOUNDARY	0x10000000	/* 256 MB */
#endif

#elif defined(NLM_HAL_UBOOT) /* u-boot */

#include <common.h>
#include <command.h>
#include <malloc.h>

#define nlm_print			printf
#define nlm_malloc			malloc
#define nlm_free			free
#define nlm_udelay(n)		udelay(n)
#define nlm_mdelay(n)		udelay(n * 1000)

#if (_MIPS_SIM == _MIPS_SIM_ABI32) || (_MIPS_SIM == _MIPS_SIM_NABI32) || \
	(_MIPS_SZLONG == 32)
#define KSEG0				0x80000000UL
#define KSEG1				0xA0000000UL
#define KSEG2				0xC0000000UL
#define KSEG3				0xE0000000UL
#define KSEG0_PHY_BOUNDARY	0x10000000UL	/* 256 MB */
#else
#define KSEG0				(0xffffffff80000000ULL)
#define KSEG1				(0xffffffffA0000000ULL)
#define KSEG2				(0xffffffffC0000000ULL)
#define KSEG3				(0xffffffffE0000000ULL)
#define KSEG0_PHY_BOUNDARY	0x10000000ULL	/* 256 MB */
#endif

#elif defined(NLM_HAL_XLOADER) /* x-loader */
#include <common.h>
#define KSEG0				0x80000000UL
#define KSEG1				0xA0000000UL
#define KSEG2				0xC0000000UL
#define KSEG3				0xE0000000UL
#define KSEG0_PHY_BOUNDARY 0x10000000UL    /* 256 MB */

#elif defined(NLM_HAL_NETLBOOT) /* netlboot */
#include <printk.h>

#define nlm_print			printk
#define nlm_malloc			malloc
#define nlm_free			free
#define nlm_udelay(n)		udelay(n)
#define nlm_mdelay(n)		mdelay(n)

#else
#error "Unsupported platform for NL HAL"
#endif	/* NLM_HAL_LINUX_USER */

#define CCA_UNCACHED		2
#define CCA_CACHED			3

#define CRC_POLY_REG_0_SEL	0
#define CRC_POLY_REG_1_SEL	1
#define CRC_POLY_REG_2_SEL	2
#define CRC_POLY_REG_3_SEL	3

#define CRC_ENDIAN_BIT		5
#define CRC_FLIPBITS_BIT	4
#define CRC_DESTINATION_BIT	6

#define CRC_32_INIT_VALUE	0xffffffff
#define CRC_16_INIT_VALUE	0

/*
 * Memory segments (64bit kernel mode addresses)
 */
/* XLP_MERGE_TODO */
#define NLH_XKUSEG			0x0000000000000000
#define NLH_XKSSEG			0x4000000000000000
#ifdef CONFIG_64BIT
#define NLH_XKPHYS			0x8000000000000000
#else
#define NLH_XKPHYS			0x8000000000000000ULL
#endif
#define NLH_XKPHYS_UNCACHED	0x9000000000000000ULL
#define NLH_XKSEG			0xc000000000000000
#define NLH_CKSEG0			0xffffffff80000000
#define NLH_CKSEG1			0xffffffffa0000000
#define NLH_CKSSEG			0xffffffffc0000000
#define NLH_CKSEG3			0xffffffffe0000000

#define SET_MIPS64 .set mips64r2
#define NLM_HAL_THREAD_SIZE (8 << 10)

/* For hal internal debug */
#define nlm_hal_dbg_msg(fmt, args...) nlm_print(fmt, ##args)

#ifndef __STR
#define __STR(x) #x
#endif
#ifndef STR
#define STR(x) __STR(x)
#endif

#ifndef __ASSEMBLY__
#define XLP_BIT_MASK_W(size)	((1 << (size)) - 1)
#define XLP_BIT_MASK_DW(size)	(((unsigned long long) 1 << size) - 1)

#define enable_KX(flags)		\
 __asm__ __volatile__ (			\
		".set push\n"			\
		".set noat\n"			\
		".set noreorder\n"		\
		"mfc0 %0, $12\n\t"	 	\
		"ori $1, %0, 0x81\n\t"	\
		"xori $1, 1\n\t"		\
		"mtc0 $1, $12\n"		\
		".set pop\n"			\
		: "=r"(flags) );

#define disable_KX(flags)		\
 __asm__ __volatile__ (			\
		".set push\n"			\
		"mtc0 %0, $12\n"		\
		".set pop\n"			\
		: : "r"(flags) )

#ifdef CONFIG_64BIT
static __inline__ uint8_t lb_40bit_phys(uint64_t phys, int cca)
{
	uint8_t value = 0;
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, " STR(NLH_XKPHYS) "\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "lb	%0, 0($8)\n" 
						 ".set pop\n":"=r"(value)
						:"r"(phys & 0xffffffffffULL),
						 "r"((uint64_t) cca << 59)
						:"$8");
		return value;
}

static __inline__ uint16_t lh_40bit_phys(uint64_t phys, int cca)
{
	uint16_t value = 0;
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, " STR(NLH_XKPHYS) "\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "lhu	%0, 0($8)\n"
						 ".set pop\n"
						:"=r"(value)
						:"r"(phys & 0xfffffffffeULL),"r"((uint64_t) cca << 59)
						:"$8");
	return value;
}

static __inline__ uint64_t lw_40bit_phys(uint64_t phys, int cca)
{
	uint64_t value = 0;
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, " STR(NLH_XKPHYS) "\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "lw	%0, 0($8)\n"
						 ".set pop\n"
						:"=r"(value)
						:"r"(phys & 0xfffffffffcULL),"r"((uint64_t) cca << 59)
						:"$8");
	return value;
}
static __inline__ uint64_t ld_40bit_phys(uint64_t phys, int cca)
{
	uint64_t value = 0;
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, " STR(NLH_XKPHYS) "\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "ld	%0, 0($8)\n"
						 ".set pop\n"
						:"=r"(value)
						:"r"(phys & 0xfffffffff8ULL),"r"((uint64_t) cca << 59)
						:"$8");
	return value;
}
#else

static __inline__ uint8_t lb_40bit_phys(uint64_t phys, int cca)
{
	uint8_t value = 0;
	uint32_t low, high;
	uint64_t cca64 = ((uint64_t)cca << 59);
	unsigned long flags;

	phys &= 0xffffffffffULL;
	phys |= (NLH_XKPHYS | cca64);
	low   = (uint32_t) phys & 0xffffffff;
	high  = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 ".set noat\n"
						 "dsll32 $1, %2, 0\n"
						 "dsll32 %1, 0\n"
						 "dsrl32 %1, 0\n"
						 "or $1, $1, %1\n"
						 "lb %0, 0($1)\n"
						 ".set at\n"
						 ".set pop\n"
						:"=r"(value)
						:"r"(low), "r"(high)
						:"$1");
	disable_KX(flags);
	return value;
}

static __inline__ uint16_t lh_40bit_phys(uint64_t phys, int cca)
{
	uint16_t value = 0;
	uint32_t low, high;
	uint64_t cca64 = ((uint64_t)cca << 59);
	unsigned long flags;

	phys &= 0xfffffffffeULL;
	phys |= (NLH_XKPHYS | cca64);
	low   = (uint32_t) phys & 0xffffffff;
	high  = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 ".set noat\n"
						 "dsll32 $1, %2, 0\n"
						 "dsll32 %1, 0\n"
						 "dsrl32 %1, 0\n"
						 "or $1, $1, %1\n"
						 "lhu %0, 0($1)\n"
						 ".set at\n"
						 ".set pop\n"
						:"=r"(value)
						:"r"(low), "r"(high)
						:"$1");

	disable_KX(flags);
	return value;
}

static __inline__ uint32_t lw_40bit_phys(uint64_t phys, int cca)
{
	uint32_t value = 0;
	uint32_t low, high;
	uint64_t cca64 = ((uint64_t)cca << 59);
	unsigned long flags;

	phys &= 0xfffffffffcULL;
	phys |= (NLH_XKPHYS | cca64);
	low   = (uint32_t) phys & 0xffffffff;
	high  = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 ".set noat\n"
						 "dsll32 $1, %2, 0\n"
						 "dsll32 %1, 0\n"
						 "dsrl32 %1, 0\n"
						 "or $1, $1, %1\n"
						 "lw %0, 0($1)\n"
						 ".set at\n"
						 ".set pop\n"
						:"=r"(value)
						:"r"(low), "r"(high)
						:"$1");
	disable_KX(flags);
	return value;
}

static __inline__ uint64_t ld_40bit_phys(uint64_t phys, int cca)
{
	uint32_t lsw, msw, high, low;
	uint64_t cca64 = ((uint64_t)cca << 59);
	unsigned long flags;

	phys &= 0xfffffffff8ULL;
	phys |= (NLH_XKPHYS | cca64);
	low   = (uint32_t) phys & 0xffffffff;
	high  = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 ".set noat\n"
						 "dsll32 $1, %3, 0\n"
						 "dsll32 %2, 0\n"
						 "dsrl32 %2, 0\n"
						 "or $1, $1, %2\n"
						 "ld $1, 0($1)\n"
						 "dsrl32 %1, $1 ,0\n"
						 "dsll32 $1, $1 ,0\n"
						 "dsrl32 %0, $1 ,0\n"
						 ".set at\n"
						 ".set pop\n"
						:"=r"(lsw), "=r"(msw)
						:"r"(low), "r"(high)
						:"$1");
	disable_KX(flags);
	return (((unsigned long long)msw << 32) | lsw);
}

#endif /* #ifdef CONFIG_64BIT */

static __inline__ uint8_t lb_40bit_phys_uncached(uint64_t phys)
{
	return lb_40bit_phys(phys, CCA_UNCACHED);
}

static __inline__ uint8_t lb_40bit_phys_cached(uint64_t phys)
{
	return lb_40bit_phys(phys, CCA_CACHED);
}

static __inline__ uint16_t lh_40bit_phys_uncached(uint64_t phys)
{
	return lh_40bit_phys(phys, CCA_UNCACHED);
}

static __inline__ uint16_t lh_40bit_phys_cached(uint64_t phys)
{
	return lh_40bit_phys(phys, CCA_CACHED);
}

static __inline__ uint32_t lw_40bit_phys_uncached(uint64_t phys)
{
	return lw_40bit_phys(phys, CCA_UNCACHED);
}
static __inline__ uint32_t lw_40bit_phys_cached(uint64_t phys)
{
	return lw_40bit_phys(phys, CCA_CACHED);
}
static __inline__ uint64_t ld_40bit_phys_uncached(uint64_t phys)
{
	return ld_40bit_phys(phys, CCA_UNCACHED);
}
static __inline__ uint64_t ld_40bit_phys_cached(uint64_t phys)
{
	return ld_40bit_phys(phys, CCA_CACHED);
}

#ifdef CONFIG_64BIT
static __inline__ void sb_40bit_phys(uint64_t phys, int cca, uint8_t value)
{
 	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, "STR(NLH_XKPHYS)"\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "sb	%0, 0($8)\n"
						 ".set pop\n"
						:
						: "r"(value), "r"(phys & 0xffffffffffULL), "r"((uint64_t)cca << 59)
						: "$8");
}

static __inline__ void sh_40bit_phys(uint64_t phys, int cca, uint32_t value)
{
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, "STR(NLH_XKPHYS)"\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "sh	%0, 0($8)\n"
						 ".set pop\n"
						:
						: "r"(value), "r"(phys & 0xfffffffffeULL), "r"((uint64_t)cca << 59)
						: "$8");
}

static __inline__ void sw_40bit_phys(uint64_t phys, int cca, uint32_t value)
{
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, " STR(NLH_XKPHYS) "\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "sw	%0, 0($8)\n"
						 ".set pop\n"
						:
						: "r"(value), "r"(phys & 0xfffffffffcULL), "r"((uint64_t) cca << 59)
						: "$8");
}

static __inline__ void sd_40bit_phys(uint64_t phys, int cca, uint64_t value)
{
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 "dli	$8, " STR(NLH_XKPHYS) "\n"
						 "or	$8, $8, %2\n"
						 "daddu $8, $8, %1\n"
						 "sd	%0, 0($8)\n"
						 ".set pop\n"
						:
						: "r"(value), "r"(phys & 0xfffffffff8ULL), "r"((uint64_t) cca << 59)
						: "$8");
}

#else

static __inline__ void sb_40bit_phys(uint64_t phys, int cca, uint8_t value)
{
	uint32_t low, high;
	uint64_t cca64 = ((uint64_t)cca << 59);
	unsigned long flags;

	phys &= 0xffffffffffULL;
	phys |= (NLH_XKPHYS | cca64);
	low = (uint32_t) phys & 0xffffffff;
	high = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 ".set noat\n"
						 "dsll32 $1, %2, 0\n"
						 "dsll32 %1, 0\n"
						 "dsrl32 %1, 0\n"
						 "or $1, $1, %1\n"
						 "sb %0, 0($1)\n"
						 ".set at\n"
						 ".set pop\n"
						:
						:"r"(value), "r"(low), "r"(high)
						:"$1");
	disable_KX(flags);
}

static __inline__ void sh_40bit_phys(uint64_t phys, int cca, uint16_t value)
{
	uint32_t low, high;
	uint64_t cca64 = ((uint64_t)cca << 59);
	unsigned long flags;

	phys &= 0xfffffffffeULL;
	phys |= (NLH_XKPHYS | cca64);
	low = (uint32_t) phys & 0xffffffff;
	high = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 ".set noat\n"
						 "dsll32 $1, %2, 0\n"
						 "dsll32 %1, 0\n"
						 "dsrl32 %1, 0\n"
						 "or $1, $1, %1\n"
						 "sh %0, 0($1)\n"
						 ".set at\n"
						 ".set pop\n"
						:
						:"r"(value), "r"(low), "r"(high)
						:"$1");
	disable_KX(flags);
}

static __inline__ void sw_40bit_phys(uint64_t phys, int cca, uint32_t value)
{
	uint32_t low, high;
	uint64_t cca64 = ((uint64_t)cca << 59);
	unsigned long flags;

	phys &= 0xfffffffffcULL;
	phys |= (NLH_XKPHYS | cca64);
	low = (uint32_t) phys & 0xffffffff;
	high = (uint32_t) (phys >> 32);

	enable_KX(flags);
	__asm__ __volatile__(".set push\n"
						 ".set noreorder\n"
						 ".set mips64\n"
						 ".set noat\n"
						 "dsll32 $1, %2, 0\n"
						 "dsll32 %1, 0\n"
						 "dsrl32 %1, 0\n"
						 "or $1, $1, %1\n"
						 "sw %0, 0($1)\n"
						 ".set at\n"
						 ".set pop\n"
						:
						:"r"(value), "r"(low), "r"(high)
						:"$1");
	disable_KX(flags);
}

static __inline__ void sd_40bit_phys(uint64_t phys, int cca, uint64_t value)
{
		uint32_t lsw, msw, high, low;
		uint64_t cca64 = ((uint64_t)cca << 59);
		unsigned long flags;
		phys &= 0xfffffffff8ULL;
		phys |= (NLH_XKPHYS | cca64);
		low   = (uint32_t) phys & 0xffffffff;
		high  = (uint32_t) (phys >> 32);
		lsw   = (uint32_t) value & 0xffffffff;
		msw   = (uint32_t)(value >>32);
		enable_KX(flags);
		__asm__ __volatile__(".set push\n"
							 ".set noreorder\n"
							 ".set mips64\n"
							 ".set noat\n"
							 "dsll32 $1, %3, 0\n"
							 "dsll32 %2, 0\n"
							 "dsrl32 %2, 0\n"
							 "or $1, $1, %2\n"
							 "dsll32 $8, %1, 0\n"
							 "dsll32 %0, 0\n"
							 "dsrl32 %0, 0\n"
							 "or $8, $8, %0\n"
							 "sd $8, 0($1)\n"
							 ".set at\n"
							 ".set pop\n"
							:
							: "r"(lsw), "r"(msw), "r"(low), "r"(high)
							:"$1", "$8");
		disable_KX(flags);
}

#endif /* #ifdef CONFIG_64BIT */

static __inline__ void sb_40bit_phys_uncached(uint64_t phys, uint8_t value)
{
	sb_40bit_phys(phys, CCA_UNCACHED, value);
}
static __inline__ void sb_40bit_phys_cached(uint64_t phys, uint8_t value)
{
	sb_40bit_phys(phys, CCA_CACHED, value);
}

static __inline__ void sh_40bit_phys_uncached(uint64_t phys, uint16_t value)
{
	sh_40bit_phys(phys, CCA_UNCACHED, value);
}
static __inline__ void sh_40bit_phys_cached(uint64_t phys, uint16_t value)
{
	sh_40bit_phys(phys, CCA_CACHED, value);
}

static __inline__ void sw_40bit_phys_uncached(uint64_t phys, uint32_t value)
{
	sw_40bit_phys(phys, CCA_UNCACHED, value);
}
static __inline__ void sw_40bit_phys_cached(uint64_t phys, uint32_t value)
{
	sw_40bit_phys(phys, CCA_CACHED, value);
}
static __inline__ void sd_40bit_phys_uncached(uint64_t phys, uint64_t value)
{
	sd_40bit_phys(phys, CCA_UNCACHED, value);
}
static __inline__ void sd_40bit_phys_cached(uint64_t phys, uint64_t value)
{
	sd_40bit_phys(phys, CCA_CACHED, value);
}

		
#define enable_ELPA()			\
 __asm__ __volatile__ (			\
		".set push\n"			\
		".set noat\n"			\
		".set noreorder\n"		\
		"mfc0 $8, $5, 1\n"		\
		"li $9, 0x20000000\n"	\
		"or $8, $8, $9\n"		\
		"mtc0 $8, $5, 1\n"		\
		".set pop\n"			\
		: : :"$8", "$9")

/*
 * COP2 Reg access macros
 */
#define _read_32bit_cp2_register(source)	\
({ int __res;								\
	__asm__ __volatile__(					\
		".set\tpush\n\t"					\
		".set\treorder\n\t"					\
		"mfc2\t%0,"STR(source)"\n\t"		\
		".set\tpop"							\
		: "=r" (__res));					\
		__res;})

#define _write_32bit_cp2_register(register,value)	\
	__asm__ __volatile__(							\
		"mtc2\t%0,"STR(register)"\n\t"				\
		"nop"										\
		: : "r" (value));

#define _read_32bit_cp2_register_sel(source, sel)	\
({ int __res;										\
	__asm__ __volatile__(							\
		".set\tpush\n\t"							\
		".set mips32\n\t"							\
		"mfc2\t%0,"STR(source)", %1\n\t"			\
		".set\tpop"									\
		: "=r" (__res) : "i" (sel) );				\
		__res;})

#define _write_32bit_cp2_register_sel(reg, value, sel)	\
	__asm__ __volatile__(								\
		".set\tpush\n\t"								\
		".set mips32\n\t"								\
		"mtc2\t%0,"STR(reg)", %1\n\t"					\
		".set\tpop"										\
		: : "r" (value), "i" (sel) );

#ifndef _ABI64

#define _read_64bit_cp2_register_sel(source, sel)	\
({													\
	unsigned int high, low;							\
	__asm__ __volatile__(							\
		".set\tmips64\n\t"							\
		"dmfc2\t$8, "STR(source)","STR(sel)"\n\t"	\
		"dsrl32\t%0, $8, 0\n\t"						\
		"dsll32\t$8, $8, 0\n\t"						\
		"dsrl32\t%1, $8, 0\n\t"						\
		".set\tmips0"								\
		: "=r" (high), "=r"(low): "i"(sel) : "$8");	\
		( (((unsigned long long)high)<<32) | low);	\
})

#define _write_64bit_cp2_register_sel(source, val, sel)		\
do {														\
	unsigned int high = val>>32;							\
	unsigned int low  = val & 0xffffffff;					\
	__asm__ __volatile__(									\
		".set\tmips64\n\t"									\
		"dsll32 $8, %1, 0\n"								\
		"dsll32 $9, %0, 0\n"								\
		"dsrl32 $8, $8, 0\n"								\
		"or	 $8, $8, $9\n"									\
		"dmtc2\t$8, "STR(source)", %2\n\t"					\
		".set\tmips0"										\
		: : "r" (high), "r" (low), "i"(sel): "$8", "$9");	\
} while (0)

#define _read_64bit_cp2_register(source)				\
		_read_64bit_cp2_register_sel(source, 0)

#define _write_64bit_cp2_register(source, val) 			\
		_write_64bit_cp2_register_sel(source, val, 0)

#else /* _ABI64 */

#define _read_64bit_cp2_register(source)	\
({ unsigned long long __res;				\
	__asm__ __volatile__(					\
		".set\tpush\n\t"					\
		".set\treorder\n\t"					\
		".set\tmips64\n\t"					\
		"dmfc2\t%0,"STR(source)"\n\t"		\
		".set\tpop"							\
		: "=r" (__res));					\
		__res;})

#define _write_64bit_cp2_register(register,value)	\
	__asm__ __volatile__(							\
		".set\tpush\n\t"							\
		".set\treorder\n\t"							\
		"dmtc2\t%0,"STR(register)"\n\t"				\
		"nop"										\
		".set\tpop"									\
		: : "r" (value));

#define _read_64bit_cp2_register_sel(source, sel)	\
({ unsigned long long __res;						\
	__asm__ __volatile__(							\
		".set\tpush\n\t"							\
		".set mips64\n\t"							\
		"dmfc2\t%0,"STR(source)", %1\n\t"			\
		".set\tpop"									\
		: "=r" (__res) : "i" (sel) );				\
		__res;})

#define _write_64bit_cp2_register_sel(reg, value, sel)	\
	__asm__ __volatile__(								\
		".set\tpush\n\t"								\
		".set mips64\n\t"								\
		"dmtc2\t%0,"STR(reg)", %1\n\t"					\
		".set\tpop"										\
		: : "r" (value), "i" (sel) );

#endif /* _ABI64 */

typedef enum crc_type {
	NLM_CRC_32 = 0,
	NLM_CRC_16 = 16,
	NLM_CRC_7 = 25
} crc_type_t;

#define REG_STR(x)			"$" #x

#undef USE_64BIT_CRC
#if defined(NLM_HAL_LINUX_KERNEL)
#define USE_64BIT_CRC
#else
#if _MIPS_SIM != _MIPS_SIM_ABI32 && !defined(ABI_32)
#define USE_64BIT_CRC
#endif
#endif /* NLM_HAL_LINUX_KERNEL */

#ifdef USE_64BIT_CRC
typedef uint64_t u_data;
#else
typedef uint32_t u_data;
#endif

#define INIT_CRC_POLY(poly_reg, poly_type, crc_poly) 					\
({																		\
	__asm__ __volatile__ (												\
		".set push\n"													\
		".set noat\n"													\
		"dmtur %0, " REG_STR(poly_reg) "\n"								\
		"ehb\n"															\
		".set pop\n"													\
		: : [poly] "r"((crc_poly << poly_type) & 0x00000000ffffffff));	\
})

#ifdef USE_64BIT_CRC
static __inline__ unsigned int
nlm_crc32_generic(uint64_t data, unsigned int flags_len, unsigned int crc)
{
	unsigned int ret;
	__asm__ __volatile__ (
		".set push\n"
		".set noreorder\n"
		"addiu	$9, %[flags_len], 0\n"
		"dcrc	%[result], %[input], $9\n"
		".set pop\n"
		: [result] "=r"(ret)
		: [input] "r"(data), [flags_len] "r"(flags_len), "0"(crc)
		: "$9"
	);
	return ret;
}
#else
static __inline__ unsigned int
nlm_crc32_generic(unsigned int data, unsigned int flags_len, unsigned int crc)
{
	unsigned int ret;
	__asm__ __volatile__ (
		".set push\n"
		".set noreorder\n"
		"addiu	$9, %[flags_len], 0\n"
		"crc	%[result], %[input], $9\n"
		".set pop\n"
		: [result] "=r"(ret)
		: [input] "r"(data), [flags_len] "r"(flags_len), "0"(crc)
		: "$9"
	);
	return ret;
}
#endif

static __inline__ uint32_t
bit_flip(uint32_t t)
{
	unsigned int ret = 0, x = t;
	__asm__ __volatile__ (
		".set push\n"
		".set mips32\n"
		".set noreorder\n"
		"addiu	%[ret], $0, 0\n"
	 "1: addiu	$9, $0, 32\n"
		"clz	$8, %[val]\n"
		"beq	$8, $9, 3f\n"

		/* set the bit in the correct location of result */
		"addiu	$9, $0, 1\n"
		"sllv	$9, $9, $8\n"
		"or		%[ret], %[ret], $9\n"

		/* reset the bit in the data */
		"addiu	$9, $0, 31\n"
		"subu	$9, $9, $8\n"
		"addiu	$8, $0, 1\n"
		"sllv	$9, $8, $9\n"
		"xor	 %[val], %[val], $9\n"
		"b		1b\n"
		".set pop\n"
	 "3: nop\n"
		: [ret] "=&r"(ret) : [val] "r"(x)
		:"$8", "$9"
		);
	return ret;
}

static __inline__ unsigned int
nlm_crc32_word(int crc_reg, u_data data, unsigned int len, unsigned int crc_init)
{
	/* flip bit */
	unsigned int flags_len = (1 << CRC_FLIPBITS_BIT) | (1 << CRC_DESTINATION_BIT);

	/* set length and CRC poly reg */
	flags_len |= ((len & 0x7) | ((crc_reg & 0x3) << 8));

	return nlm_crc32_generic(data, flags_len, crc_init);
}

static __inline__ uint32_t
nlm_crc16_ibm(int crc_reg, u_data data, unsigned int len, unsigned short crc_init)
{
	/* flip bit */
	unsigned int flags_len = (1 << CRC_FLIPBITS_BIT) | (1 << CRC_DESTINATION_BIT);

	/* set length and CRC poly reg */
	flags_len |= ((len & 0x7) | ((crc_reg & 0x3) << 8));

	return nlm_crc32_generic(data, flags_len, crc_init);
}

static __inline__ uint32_t
nlm_crc7_word(int crc_reg, u_data data, unsigned int len, unsigned short crc_init)
{
	/* flip bit */
	unsigned int flags_len = (1 << CRC_FLIPBITS_BIT) | (1 << CRC_DESTINATION_BIT);

	/* set length and CRC poly reg */
	flags_len |= ((len & 0x7) | ((crc_reg & 0x3) << 8));

	return nlm_crc32_generic(data, flags_len, crc_init);
}

/**
 * This macro loops on a string of data.  It is kept this way to
 * adjust to endianess and use of 64bit instruction, so that each
 * CRC function does not require ifdef.
 */
#ifdef USE_64BIT_CRC
#define LOOP_ON_DATA(c, init, b, len, f) 		\
({												\
	uint64_t data;								\
	uint32_t i, rem = len, __ret = init, l;		\
	for(i = 0; rem > 0;) {						\
		data = ((uint64_t *)buf)[i];			\
		i += 1;									\
		if(rem >= 8) {							\
			rem -= 8;							\
			l = 7;								\
		} else if (rem == 7) {					\
			rem -= 7;							\
			l = 6;								\
		} else if (rem == 6) {					\
			rem -= 6;							\
			l = 5;								\
		} else if (rem == 5) {					\
			rem -= 5;							\
			l = 4;								\
		} else if (rem == 4) {					\
			rem -= 4;							\
			l = 3;								\
		} else if (rem == 3) {					\
			rem -= 3;							\
			l = 2;								\
		} else if (rem == 2) {					\
			rem -= 2;							\
			l = 1;								\
		} else if (rem == 1) {					\
			rem -= 1;							\
			l = 0;								\
		}										\
		__ret = f(crc_reg, data, l, __ret);		\
	}											\
	__ret;										\
})
#else
#define LOOP_ON_DATA(c, init, b, len, f) 		\
({												\
	uint32_t data;								\
	uint32_t i, rem = len, __ret = init, l;		\
	for(i = 0; rem > 0;) {						\
		data = ((uint32_t *)buf)[i];			\
			i += 1;								\
		if(rem >= 4) {							\
			rem -= 4;							\
			l = 3;								\
		} else if (rem == 3) {					\
			rem -= 3;							\
			l = 2;								\
		} else if (rem == 2) {					\
			rem -= 2;							\
			l = 1;								\
		} else if (rem == 1) {					\
			rem -= 1;							\
			l = 0;								\
		}										\
		__ret = f(crc_reg, data, l, __ret);		\
	}											\
	__ret;										\
})
#endif

static __inline__ uint32_t
nlm_crc32(int crc_reg, const unsigned char *buf, unsigned int len, unsigned int crc)
{
	return LOOP_ON_DATA(crc_reg, crc,
			buf, len, nlm_crc32_word) ^ 0xffffffff;
}

static __inline__ uint16_t
nlm_crc16(int crc_reg, const unsigned char *buf, unsigned int len, unsigned short crc)
{
	return (uint16_t)(LOOP_ON_DATA(crc_reg, crc,
			buf, len, nlm_crc16_ibm) & 0xffff);
}

static __inline__ unsigned char
nlm_crc7(int crc_reg, const unsigned char *buf, unsigned int len, unsigned char crc)
{
	return (unsigned char)(LOOP_ON_DATA(crc_reg, crc, buf,
			len, nlm_crc7_word) & 0x7f);
}

#if 0
static __inline__ int num_ones(unsigned long mask)
{
	int nones;

	for (nones = 0; mask; mask >>= 1) {
		if (mask & 0x1)
			++nones;
	}

	return nones;
}
#endif

static __inline__ void write_32bit_cfg_reg(uint32_t *base, unsigned int offset, uint32_t value)
{
	base[offset] = value;
}

static __inline__ uint32_t read_32bit_cfg_reg(uint32_t *base, unsigned int offset)
{
	return ((base)[offset]);
}

static __inline__ void write_64bit_cfg_reg(uint64_t *base, unsigned int offset, uint64_t value)
{
	base[offset] = value;
}

static __inline__ uint64_t read_64bit_cfg_reg(uint64_t *base, unsigned int offset)
{
	return ((base)[offset]);
}

static __inline__ uint32_t xlp_get_field_w(uint32_t word, int lsb, int size)
{
	return ((word >> lsb) & XLP_BIT_MASK_W(size));
}

static __inline__ uint64_t xlp_get_field_dw(uint64_t dword, int lsb, int size)
{
	return ((dword >> lsb) & XLP_BIT_MASK_DW(size));
}

#if !defined(NLM_HAL_LINUX_USER)
static __inline__ int nlm_read_prid(void)
{
		int res = 0;
		__asm__ __volatile__ (		\
			 ".set push\n"			\
			 ".set noat\n"			\
			 ".set noreorder\n"		\
			 "mfc0 %0, $15, 0\n"	\
			 ".set pop\n"			\
			: "=r" (res));
		return res;
}

#else
#define nlm_read_prid			nlm_uaccess_processor_id
#endif

static __inline__ uint32_t nlm_read_ebase(void)
{
		uint32_t res = 0;
		__asm__ __volatile__ (		\
			".set push\n"			\
			".set noat\n"			\
			".set noreorder\n"		\
			"mfc0 %0, $15, 1\n"		\
			".set pop\n"			\
			: "=r" (res));

		return res;
}

/* Linux User Mode */
#if defined(NLM_HAL_LINUX_USER)
#include <nlm_uaccess.h>
#define nlh_read_cfg_reg16(addr)		nlm_uaccess_mem_read16((NLH_XKPHYS_UNCACHED | (addr)))
#define nlh_write_cfg_reg16(addr, val)	nlm_uaccess_mem_write16((NLH_XKPHYS_UNCACHED | (addr)), (val))
#define nlh_read_cfg_reg32(addr)		nlm_uaccess_mem_read32((NLH_XKPHYS_UNCACHED | (addr)))
#define nlh_write_cfg_reg32(addr, val)	nlm_uaccess_mem_write32((NLH_XKPHYS_UNCACHED | (addr)), (val))
#define nlh_read_cfg_reg64(addr)		nlm_uaccess_mem_read64((NLH_XKPHYS_UNCACHED | (addr)))
#define nlh_write_cfg_reg64(addr, val)	nlm_uaccess_mem_write64((NLH_XKPHYS_UNCACHED | (addr)), (val))

/* For Accessing Regex Registers in PCI Memory space */
#define WRITE_REGX_CFG_REG_PCIM(reg, val)	nlh_write_cfg_reg32((xlp_regex_base_pcim + reg), (val))
#define READ_REGX_CFG_REG_PCIM(reg)			nlh_read_cfg_reg32((xlp_regex_base_pcim + reg))

#define nlh_send_msg4(dst, code, data0, data1, data2, data3) \
		nlm_uaccess_msgsnd_4(code, dst, data0, data1, data2, data3)

#define nlh_send_msg3(dst, code, data0, data1, data2) \
		nlm_uaccess_msgsnd_3(code, dst, data0, data1, data2)

#define nlh_send_msg2(dst, code, data0, data1) \
		nlm_uaccess_msgsnd_2(code, dst, data0, data1)

#define nlh_send_msg1(dst, code, data0) \
		nlm_uaccess_msgsnd_1(code, dst, data0)

/* Returns 1 on failure and 0 on success */
#define nlh_recv_msg2(dst, src, size, code, data0, data1)	\
		nlm_uaccess_msgrcv_2(dst, src, size, code, data0, data1)

#define nlh_recv_msg1(dst, src, size, code, data0)	\
		nlm_uaccess_msgrcv_1(dst, src, size, code, data0)

/* NETOS and Linux Kernel Mdoe */
#elif defined(NLM_HAL_NETOS) || defined(NLM_HAL_LINUX_KERNEL) \
	|| defined(NLM_HAL_UBOOT) || defined(NLM_HAL_NETLBOOT) || defined(NLM_HAL_XLOADER)

#define nlh_read_cfg_reg16(addr)		lh_40bit_phys_uncached(addr)
#define nlh_write_cfg_reg16(addr, val)	sh_40bit_phys_uncached(addr, val)
#define nlh_read_cfg_reg32(addr)		lw_40bit_phys_uncached(addr)
#define nlh_write_cfg_reg32(addr, val)	sw_40bit_phys_uncached(addr, val)
#define nlh_read_cfg_reg64(addr)		ld_40bit_phys_uncached(addr)
#define nlh_write_cfg_reg64(addr, val)	sd_40bit_phys_uncached(addr, val)

#define nlh_send_msg4(dst, code, data0, data1, data2, data3) \
	xlp_message_send_4(dst, code, data0, data1, data2, data3)

#define nlh_send_msg3(dst, code, data0, data1, data2) \
	xlp_message_send_3(dst, code, data0, data1, data2)

#define nlh_send_msg2(dst, code, data0, data1) \
	xlp_message_send_2(dst, code, data0, data1)

#define nlh_send_msg1(dst, code, data0) \
	xlp_message_send_1(dst, code, data0)

#define nlh_recv_msg2(dst, src, size, code, data0, data1) \
	xlp_message_receive_2(dst, src, size, code, data0, data1)

#define nlh_recv_msg1(dst, src, size, code, data0) \
	xlp_message_receive_1(dst, src, size, code, data0)

#else
#error "Unsupported platform for NL HAL"

#endif

#if !defined(NLM_HAL_LINUX_USER)
static __inline__ uint32_t nlm_hard_cpuid(void)
{
	return nlm_read_ebase() & 0x3ff;
}
#else
#define nlm_hard_cpuid			nlm_uaccess_hard_cpuid
#endif

static __inline__ uint32_t nlm_node_id(void)
{
	if (is_nlm_xlp8xx())
		return (nlm_hard_cpuid() >> 5) & 0x3;
	return 0;
}

static __inline__ uint32_t nlm_cpu_id(void)
{
	return nlm_hard_cpuid() & 0x1f;
}

#else  /* __ASSEMBLY__ */

#if (_MIPS_SIM == _MIPS_SIM_ABI32) || (_MIPS_SIM == _MIPS_SIM_NABI32) || \
	(_MIPS_SZLONG == 32)
#define LW		lw
#define LA		la
#define SW		sw
#else
#define LW		ld
#define LA		dla
#define SW		sd
#endif

#endif /* __ASSEMBLY__ */

#endif /* #ifndef _NLM_HAL_MACROS_H */
