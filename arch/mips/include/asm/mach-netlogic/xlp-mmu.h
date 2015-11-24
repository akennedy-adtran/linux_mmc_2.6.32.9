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

#ifndef _ASM_MACH_NLM_XLP_MMU_H
#define _ASM_MACH_NLM_XLP_MMU_H

#include <linux/percpu.h>
#include <asm/mipsregs.h>
#include <asm/netlogic/xlp8xx/cpu_control_macros.h>
#include <asm/mach-netlogic/pgwalker.h>

/* 
 * These numbers correspond to Cop0 Config6 reg 
 * bit positions 
 */
#define ENABLE_ETLB        0x4
#define ENABLE_128_TLB     0x20
#define ENABLE_PGWALKER    0x8

#define USER_SEG 0

#ifdef CONFIG_64BIT
#define NR_ADDR_SEGMENTS 8  /* MUST be a power of 2 */
#define MODULE_SEG 7
#define VMALLOC_SEG 6
#else /* CONFIG_32BIT */
#define NR_ADDR_SEGMENTS 2 /* MUST be a power of 2 */
#define VMALLOC_SEG 1
#endif /* CONFIG_64BIT */

extern DEFINE_PER_CPU(unsigned long [NR_ADDR_SEGMENTS], pgd_bases);

extern uint32_t nlm_l1_lock[NR_CPUS/4];

extern int is_nlm_xlp2xx_compat;

static inline void setup_user_pgd(pgd_t *pgd)
{
	get_cpu_var(pgd_bases)[USER_SEG] = (unsigned long) pgd;
	put_cpu_var(pgd_bases);
};


static inline void nlm_lock_l1(uint32_t core)
{
        uint32_t temp;
        __asm__ __volatile__(
                "       .set    noreorder       \n"
                "1:     ll      %1, %2          \n"
                "       bgtz    %1, 2f          \n"
                "       ori     %1, 1           \n"
                "       sc      %1, %0          \n"
                "       beqz    %1, 1b          \n"
                "       nop                     \n"
                "       .subsection 2           \n"
                "2:     ll      %1, %2          \n"
                "       bgtz    %1, 2b          \n"
                "       nop                     \n"
                "       b       1b              \n"
                "        nop                    \n"
                "       .previous               \n"
                "       .set    reorder         \n"
                : "=m" (nlm_l1_lock[core]), "=&r" (temp)
                : "m" (nlm_l1_lock[core])
                : "memory");
}

static inline void nlm_unlock_l1(uint32_t core)
{
        uint32_t temp;

        __asm__ __volatile__(
                "       .set    noreorder 	         \n"
                "1:     ll      %1, %2          	 \n"
                "       sub     %1, 1                    \n"
                "       sc      %1, %0                   \n"
                "       beqz    %1, 2f                   \n"
                "        nop                             \n"
                "       .subsection 2                    \n"
                "2:     b       1b                       \n"
                "        nop                             \n"
                "       .previous                        \n"
                "       .set    reorder                  \n"
                : "=m" (nlm_l1_lock[core]), "=&r" (temp)
                : "m" (nlm_l1_lock[core])
                : "memory");

}

#define NLM_XLP_L1_MAXWAY       2
#define NLM_XLP_L1_MAXINDX      128

static inline void nlm_flush_l1_dcache_line(uint32_t line)
{
         __asm__ __volatile__ (
                "       .set push                       \n"
                "       .set noat                       \n"
                "       .set noreorder                  \n"
                "       li $8, "STR(LSU_DEBUG_DATA0)"   \n"
                "       mtcr $0, $8                     \n"
                "       li $9, "STR(LSU_DEBUG_ADDR)"    \n"
                "       ori %0, %0, 0x1                 \n"
                "       mtcr %0, $9                     \n"
                "1:                                     \n"
                "       mfcr $8, $9                     \n"
                "       andi $8, $8, 0x1                \n"
                "       bnez $8, 1b                     \n"
                "       nop                             \n"
                "       .set pop                        \n"
                : : "r"(line) : "$8" , "$9");
}

static inline void nlm_flush_l1_dcache(void)
{
    uint32_t index, line, max;
    uint32_t cpu = read_c0_ebase() & 0x7f;
    uint32_t thread = cpu & 0x3;

    nlm_lock_l1(cpu >> 2);
    max = (thread + 1) * current_cpu_data.dcache.sets;
    index = thread * current_cpu_data.dcache.sets;

    for(; index < max; index++) {

        line = (index << 5) | (1<<1);
        nlm_flush_l1_dcache_line(line);
        line = (1 << 2) | (index << 5) | (1<<1);
        nlm_flush_l1_dcache_line(line);

        line = (index << 5) | (1<<1) | (0x1 << 14);
        nlm_flush_l1_dcache_line(line);
        line = (1 << 2) | (index << 5) | (1<<1) | (0x1 << 14);
        nlm_flush_l1_dcache_line(line);
    }

    nlm_unlock_l1(cpu >> 2);
}

static __inline__ void pipeline_flush(void)
{
	__asm__ __volatile__ (
		".set push         \n"
		".set arch=xlp     \n"
		"dla      $8, 1f    \n"
		"jr.hb   $8        \n"
		"nop               \n"
		"1: nop            \n"
		".set pop          \n"
		:
		:
		: "$8"
		);
}

#ifdef CONFIG_32BIT
#define disable_pgwalker(flags) (void)flags
#define enable_pgwalker(flags) (void) flags
#else
#define disable_pgwalker(flags)						\
	({ if (!is_nlm_xlp2xx_compat) {				\
	        flags = read_c0_config6();				\
		pipeline_flush();					\
		write_c0_config6(read_c0_config6() & ~ENABLE_PGWALKER);	\
		pipeline_flush();					\
	   } else { 							\
	        flags = read_c0_pwctl();				\
		pipeline_flush();					\
		write_c0_pwctl(flags & ~(1 <<  PWCTL_PW_EN_O));		\
		pipeline_flush();					\
	   }								\
	  })

#define enable_pgwalker(flags)								\
	({ 										\
	    if (!is_nlm_xlp2xx_compat) {						\
		write_c0_config6(read_c0_config6() | (flags & ENABLE_PGWALKER));	\
	    } else {									\
		write_c0_pwctl(read_c0_pwctl() | (flags & (1 << PWCTL_PW_EN_O)));	\
	    }										\
	})

#endif
#endif
