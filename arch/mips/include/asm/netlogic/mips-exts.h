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


#ifndef _ASM_NLM_MIPS_EXTS_H
#define _ASM_NLM_MIPS_EXTS_H

#define NETLOGIC_OSS_SEL_TLB_STATS 0
#define NETLOGIC_OSS_SEL_UNUSED 1
#define NETLOGIC_OSS_SEL_PAGEMASK 2
#define NETLOGIC_OSS_SEL_VADDR 3
#define NETLOGIC_OSS_SEL_PFN0 4
#define NETLOGIC_OSS_SEL_PFN1 5
#define NETLOGIC_OSS_SEL_K0 6
#define NETLOGIC_OSS_SEL_K1 7

#define OS_SCRATCH_REG0	22, 0
#define OS_SCRATCH_REG1	22, 1
#define OS_SCRATCH_REG2	22, 2
#define OS_SCRATCH_REG3	22, 3
#define OS_SCRATCH_REG4	22, 4
#define OS_SCRATCH_REG5	22, 5
#define OS_SCRATCH_REG6	22, 6
#define OS_SCRATCH_REG7	22, 7

#define OS_KGDB_SCRATCH_REG6	$22, 6
#define OS_KGDB_SCRATCH_REG7	$22, 7

#ifndef __ASSEMBLY__

#include <linux/types.h>
//#include <asm/netlogic/interrupt.h>

/* Scratch registers used */
#define NLM_TLB_STATS_SCRATCH_REG_SEL  2
#define NLM_HTLB_PMASK_SCRATCH_REG_SEL 3
#define NLM_CRF_PERF0_SCRATCH_REG_SEL  NLM_PERF0_SCRATCH
#define NLM_CRF_PERF1_SCRATCH_REG_SEL  NLM_PERF1_SCRATCH


#define DMFC0_AT_EIRR 0x40214806
#define DMFC0_AT_EIMR 0x40214807
#define DMTC0_AT_EIRR 0x40a14806
#define DMTC0_AT_EIMR 0x40a14807

/* functions to write to and read from the extended
 * cp0 registers.
 * EIRR : Extended Interrupt Request Register
 *        cp0 register 9 sel 6
 *        bits 0...7 are same as cause register 8...15
 * EIMR : Extended Interrupt Mask Register
 *        cp0 register 9 sel 7
 *        bits 0...7 are same as status register 8...15
 */

static inline __u64 read_64bit_cp0_eirr(void)
{
  __u32 high, low;

  __asm__ __volatile__ (
			".set push\n"
			".set noreorder\n"
			".set noat\n"
			".set mips4\n"

			".word 0x40214806  \n\t"
			"nop               \n\t"
			"dsra32 %0, $1, 0  \n\t"
			"sll    %1, $1, 0  \n\t"

			".set pop\n"

			: "=r" (high), "=r" (low)
			);

  return ( ((__u64)high) << 32) | low;
}

static inline __u64 read_64bit_cp0_eimr(void)
{
  __u32 high, low;

  __asm__ __volatile__ (
			".set push\n"
			".set noreorder\n"
			".set noat\n"
			".set mips4\n"

			".word 0x40214807  \n\t"
			"nop               \n\t"
			"dsra32 %0, $1, 0  \n\t"
			"sll    %1, $1, 0  \n\t"

			".set pop\n"

			: "=r" (high), "=r" (low)
			);

  return ( ((__u64)high) << 32) | low;
}

static inline void write_64bit_cp0_eirr(__u64 value)
{
  __u32 low, high;

  high = value >> 32;
  low  = value & 0xffffffff;

	__asm__ __volatile__ (
	".set push\n"
	".set noreorder\n"
	".set noat\n"
	".set mips4\n\t"

	"dsll32 $2, %1, 0  \n\t"
	"dsll32 $1, %0, 0  \n\t"
	"dsrl32 $2, $2, 0  \n\t"
	"or     $1, $1, $2 \n\t"
	".word  0x40a14806 \n\t"
	"nop               \n\t"

	".set pop\n"

	:
	: "r" (high), "r" (low)
	: "$1", "$2");
}

static inline void write_64bit_cp0_eimr(__u64 value)
{
  __u32 low, high;

  high = value >> 32;
  low  = value & 0xffffffff;

	__asm__ __volatile__ (
	".set push\n"
	".set noreorder\n"
	".set noat\n"
	".set mips4\n\t"

	"dsll32 $2, %1, 0  \n\t"
	"dsll32 $1, %0, 0  \n\t"
	"dsrl32 $2, $2, 0  \n\t"
	"or     $1, $1, $2 \n\t"
	".word  0x40a14807 \n\t"
	"nop               \n\t"

	".set pop\n"

	:
	: "r" (high), "r" (low)
	: "$1", "$2");
}

#ifdef CONFIG_NLM_ATOMICS
static __inline__ int ldadd_w(unsigned int value, volatile int *addr)
{
	unsigned long res;
  __asm__ __volatile__(
		       ".set push\n"
		       ".set noreorder\n"
		       "move $8, %2\n"
		       "move $9, %3\n"
		       //"ldaddw %2, %3\n"
                       ".word 0x71280010\n"
		       "move %0, $8\n"
		       ".set pop\n"
		       :"=r"(res), "+m"(*addr)
		       : "r" (value), "r"((unsigned long)addr)
		       : "$8", "$9"
		       );
  return res;
}

static __inline__ void ldadd_w_no_read(int value, volatile int *addr)
{
	unsigned long res;
  __asm__ __volatile__(
                       ".set push\n"
                       ".set noreorder\n"
                       "move $8, %2\n"
                       "move $9, %3\n"
                       //"ldaddw $8, $9\n"
                       ".word 0x71280010\n"
                       //"move %0, $8\n"
                       ".set pop\n"
                       :"=r"(res), "+m"(*addr)
                       : "r" (value), "r"((unsigned long)addr)
                       : "$8", "$9"
                       );
}

static __inline__ unsigned int ldadd_wu(unsigned int value, volatile unsigned int *addr)
{
	unsigned long res;
  __asm__ __volatile__(
		       ".set push\n"
		       ".set noreorder\n"
		       "move $8, %2\n"
		       "move $9, %3\n"
		       //"ldaddwu $8, $9\n"
                       ".word 0x71280011\n"
		       "move %0, $8\n"
		       ".set pop\n"
		       :"=r"(res), "+m"(*addr)
		       : "r"(value), "r"((unsigned long)addr)
		       : "$8", "$9"
		       );
  return res;
}

static __inline__ void ldadd_wu_no_read(unsigned int value,
					volatile unsigned int *addr)
{
	unsigned long res;
  __asm__ __volatile__(
                       ".set push\n"
                       ".set noreorder\n"
                       "move $8, %2\n"
                       "move $9, %3\n"
                       //"ldaddwu $8, $9\n"
                       ".word 0x71280011\n"
                       //"move %0, $8\n"
                       ".set pop\n"
                       :"=r"(res), "+m"(*addr)
                       : "r"(value), "r"((unsigned long)addr)
                       : "$8", "$9"
                       );
}
#endif /* CONFIG_NLM_ATOMICS */

static __inline__ int hard_smp_processor_id(void)
{
	int cpu;

	__asm__ __volatile__ (
		".set push\n"
		".set noreorder\n"
                ".set mips32\n"
                "mfc0 %0, $15, 1\n"
		"andi %0, %0, 0x3ff\n"
		".set pop\n"
		: "=&r"(cpu)
		);

	return cpu;
}

static __inline__ int netlogic_node_id(void)
{
	return hard_smp_processor_id() >> 5;	
}

static __inline__ int netlogic_cpu_id(void)
{
	return hard_smp_processor_id() >> 2;
}

static __inline__ int netlogic_thr_id(void)
{
	return hard_smp_processor_id() & 0x03;
}

#define CPU_BLOCKID_IFU      0
#define CPU_BLOCKID_ICU      1
#define CPU_BLOCKID_IEU      2
#define CPU_BLOCKID_LSU      3
#define CPU_BLOCKID_MMU      4
#define CPU_BLOCKID_PRF      5

#ifdef CONFIG_NLM_XLR
#define LSU_CERRLOG_REGID    9
#else
#define CPU_BLOCKID_SCU      8

#define ICU_CERRLOG0_REGID   0x10
#define ICU_CERRLOG1_REGID   0x11
#define ICU_CERRLOG2_REGID   0x12

#define LSU_CERRLOG0_REGID   0x08
#define LSU_CERRLOG1_REGID   0x09

#define SCU_CERRLOG0_REGID   0x10
#define SCU_CERRLOG1_REGID   0x11
#define SCU_CERRLOG2_REGID   0x12

#endif

static __inline__ unsigned int read_32bit_nlm_ctrl_reg(int block, int reg)
{
  unsigned int __res;

  __asm__ __volatile__(
		       ".set\tpush\n\t"
		       ".set\tnoreorder\n\t"
		       "move $9, %1\n"
/* 		       "mfcr\t$8, $9\n\t"          */
		       ".word 0x71280018\n"
		       "move %0, $8\n"
		       ".set\tpop"
		       : "=r" (__res) : "r"((block<<8)|reg)
		       : "$8", "$9"
		       );
  return __res;
}

static __inline__ void write_32bit_nlm_ctrl_reg(int block, int reg, unsigned int value)
{
  __asm__ __volatile__(
		       ".set\tpush\n\t"
		       ".set\tnoreorder\n\t"
		       "move $8, %0\n"
		       "move $9, %1\n"
/* 		       "mtcr\t$8, $9\n\t"  */
		       ".word 0x71280019\n"
		       ".set\tpop"
		       :
		       : "r" (value), "r"((block<<8)|reg)
		       : "$8", "$9"
		       );
}

static __inline__ unsigned long long read_64bit_nlm_ctrl_reg(int block, int reg)
{
	unsigned int high, low;

	__asm__ __volatile__(
		".set\tmips64\n\t"
		"move    $9, %2\n"
		/* "mfcr    $8, $9\n" */
		".word   0x71280018\n"
		"dsrl32  %0, $8, 0\n\t"
		"dsll32  $8, $8, 0\n\t"
		"dsrl32  %1, $8, 0\n\t"
		".set mips0"
		: "=r" (high), "=r"(low)
		: "r"((block<<8)|reg)
		: "$8", "$9"
		);

	return ( (((unsigned long long)high)<<32) | low);
}

static __inline__ void write_64bit_nlm_ctrl_reg(int block, int reg,unsigned long long value)
{
	__u32 low, high;
	high = value >> 32;
	low = value & 0xffffffff;

	__asm__ __volatile__(
		".set push\n"
		".set noreorder\n"
		".set mips4\n\t"
		/* Set up "rs" */
		"move $9, %0\n"

		/* Store 64 bit value in "rt" */
		"dsll32 $10, %1, 0  \n\t"
		"dsll32 $8, %2, 0  \n\t"
		"dsrl32 $8, $8, 0  \n\t"
		"or     $8, $10, $8 \n\t"

		".word 0x71280019\n" /* mtcr $8, $9 */

		".set pop\n"

		:  /* No outputs */
		: "r"((block<<8)|reg), "r" (high), "r" (low)
		: "$8", "$9", "$10"
		);
}

typedef struct { volatile int value; } nlm_common_atomic_t;

#ifdef CONFIG_NLM_XLR
static __inline__ int nlm_common_test_and_set(nlm_common_atomic_t *lock)
{
  int oldval = 0;

  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			"move $9, %2\n"
			"li $8, 1\n"
			//"swapw $8, $9\n"
			".word 0x71280014\n"
			"move %1, $8\n"
			".set pop\n"
			: "+m" (lock->value), "=r" (oldval)
			: "r" ((unsigned long)&lock->value)
			: "$8", "$9"
			);
  return (oldval == 0 ? 1/*success*/ : 0/*failure*/);
}
#endif /* CONFIG_NLM_XLR */

#define nlm_write_os_scratch_2(val)	__write_64bit_c0_register($22, 2, val)
#define nlm_read_os_scratch_2()	__read_64bit_c0_register($22, 2)

#define nlm_write_os_scratch_3(val)	__write_64bit_c0_register($22, 3, val)
#define nlm_read_os_scratch_3()	__read_64bit_c0_register($22, 3)
#endif

#ifdef CONFIG_CPU_XLP
#define SET_MIPS64 .set mips64r2
#else
#define SET_MIPS64 .set mips64
#endif

#endif /* _ASM_NLM_MIPS_EXTS_H */
