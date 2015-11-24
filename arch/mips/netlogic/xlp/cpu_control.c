/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/pm.h>

#include <asm/asm.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/netlogic/xlp8xx/cpu_control_macros.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/hal/nlm_hal.h>

#define XLP_ECFG_BASE           0x18000000
#define XLP_SYS_DEV_BASE        0x35000

/* temporary storage space for
 * stack pointers
 */
unsigned long linuxsp[NR_CPUS];

/* Externs
 */
extern unsigned char __stack[];
extern char boot_siblings_start[], boot_siblings_end[];
extern char reset_entry[], reset_entry_end[];

static inline void jump_address(unsigned long entry)
{
	asm volatile (
		".set push   	\n"
		".set noreorder \n"
		"jalr  %0     	\n"
		"nop          	\n"
		".set pop       \n"
		:: "r"(entry)
	);
}

/* Configure LSU */
/* Removed test for xlp8xx_a01_workaround_needed to LSU_DEFEATURE S1RCM */
static inline void config_lsu(void)
{
	uint32_t tmp0, tmp1, tmp2;

/* XLP8xx/4xx/3xx Unaligned access erratum - two possible work-arounds:
 * 1) Enable safe retry mode in SCHED_DEFEATURE (see below)
 * 2) Disable hardware acceleration for unaligned accesses
 * Enable only one of the below LUI opcodes.  For XLP2xx/XLP1xx leave
 * unaligned access enabled.  L2HPE = bit 23.  L1HPE = bit 20.  Both default 0.
 */
	__asm__ __volatile__ (
		".set push\n"
		".set noreorder\n"
		"li		%0, "STR(LSU_DEFEATURE)"\n"
		"mfcr	%1, %0\n"
//		"lui	%2, 0x4080\n"  /* Enable Unaligned Access, enable L2HPE */
		"lui	%2, 0x0080\n"  /* Enable L2HPE */
		"or		%1, %1, %2\n"
		"mtcr	%1, %0\n"
		"ehb\n"

#if 0
		"li		%0, "STR(THREAD_FAIRNESS)"\n"
		"li		%1, 0x8\n"
		"mtcr	%1, %0\n" // Set thread fairness to 8 if DRAM utilization is high
#endif

/* CPU Scheduler Issue Queue 3 erratum - two possible work-arounds:
 * 1) Disable Branch Resolution ALU from accepting ALU operations
 * 2) Equally weight ALU and branch operations in issue queue 3
 * Also options to enable or disable safe retry mode for unaligned access erratum
 * Enable only one of the below four LUI opcodes
 */
		"li		%0, "STR(SCHED_DEFEATURE)"\n"	// Enable only ONE of the following four lines
//		"lui	%1, 0x0100\n"	// Disable BRU accepting ALU ops, don't enable safe retry
//		"lui	%1, 0xF900\n"	// Disable BRU accepting ALU ops, enable safe retry
		"lui	%1, 0x0020\n"	// Equally weight ALU and branch ops in issue Q3, don't enable safe rerty
//		"lui	%1, 0xF820\n"	// Equally weight ALU and branch ops in issue Q3, enable safe rerty
		"mtcr	%1, %0\n"

#if 0
	/* Set up serial access mode for uncached accesses - only matters for multi-node */
		"li		%0, "STR(L2_FEATURE_CTRL0)"\n"
		"mfcr	%1, %0\n"
		"ori	%1, %1, 0x200\n"
		"mtcr	%1, %0\n"
#endif

		"nop\n"
		".set pop\n"
		: "=r" (tmp0), "=r" (tmp1), "=r" (tmp2)
	);
}

static void enable_cores(unsigned int node, unsigned int cores_bitmap)
{
	uint32_t core, value;
	uint64_t sys_mmio;
	uint32_t cbitmap = cores_bitmap;

	/* if n0c0t0, don't pull yourself out of reset! */
	if(node == 0) cbitmap = cbitmap & 0xfe;

//	printk("[%s] node@%d, cores_bitmap = 0x%08x cbitmap = 0x%08x\n",
//	       __func__, node, cores_bitmap, cbitmap);

	sys_mmio = (uint64_t) (XLP_ECFG_BASE + XLP_SYS_DEV_BASE + 0x40000 * node );

	for (core = 0x1; core != (0x1 << 8); core <<= 1) {

		if ( (cbitmap & core) == 0) continue;

		if (!is_nlm_xlp2xx()) {
			/* Enable CPU clock: only needed for xlp8xx/xlp3xx */
			value = nlm_hal_read_32bit_reg(sys_mmio,0x4E) & ~core;
			nlm_hal_write_32bit_reg(sys_mmio, 0x4E, value);
		}

		/* Remove CPU Reset */
		value = nlm_hal_read_32bit_reg(sys_mmio, 0x4B) & ~core;
		nlm_hal_write_32bit_reg(sys_mmio, 0x4B, value);

		/* Poll for CPU to mark itself coherent */
		for(;;) {
			value = nlm_hal_read_32bit_reg(sys_mmio, 0x4D) & core;
			if (!value) break;
		}
	}
}

static inline int num_ones(unsigned long mask)
{
	int ret = 0;

	if (!mask) return 0;
	while ((mask &= (mask - 1))) ret++;
	return (ret + 1);
}

int threads_to_enable = 0;
/*
 * This function is called once for each node. However, it is executed
 * only on "master cpu", mostly on n0c0t0
 *
 * t0_bitmap: bitmap of thread@0 of those cores in the given node, which 
 *            are enabled in node_cpumask
 *
 * cores_bitmap: bitmap of cores (core 0's) which have at least one thread
 *               enabled (considering ALL nodes and cores)
 *
 * threads_to_enable: count bitmap of threads to enable in all enabled cores.
 *
 */
void enable_cpus(unsigned int node, unsigned int node_cpumask)
{
	uint32_t t0_bitmap = 0x0;
	uint32_t t0_positions = 0, index = 0;
	uint32_t cores_bitmap;

	if (hard_smp_processor_id() != 0) {
		printk("[%s]: Running on non n0c0t0 cpu??\n", __func__);
		return;
	}

	/* Setup Exception vectors only the first time around */
	if (!node) {

		config_lsu();

		/* Linux runs out of KSEG2. Setup TLBs
		 * for other threads, by running from
		 * KSEG0. Then, jump back into KSEG2.
		 */
		memcpy((void *)(NMI_BASE + (2<<10)),
		       (void *)&boot_siblings_start,
		       (boot_siblings_end - boot_siblings_start));

		/* setup the reset vector */
		memcpy((void *)(NMI_BASE), (void *)&reset_entry, (reset_entry_end - reset_entry));

	}

	/* bitmap of thread@0 of every core in this node */
	t0_bitmap = node_cpumask & 0x11111111;

	cores_bitmap = 0;
	for (t0_positions = 0, index = 0; t0_positions < 32; t0_positions += 4, index++) {
		 if (t0_bitmap & (1 << t0_positions)){
			cores_bitmap |= (1 << index);
			threads_to_enable |= (num_ones((node_cpumask >> t0_positions) & 0xf) << t0_positions);
		 }
	}

//	printk("node@%d: t0_bitmap = 0x%08x, cores_bitmap = 0x%08x threads_to_enable 0x%x\n", 
//		node, t0_bitmap, cores_bitmap,threads_to_enable);

	enable_cores(node, cores_bitmap);

	if (!node) {
		/* Wakeup threads of n0c0 */
		jump_address(NMI_BASE + (2<<10));
	}

	/* Printks from this point forward can get corrupted */

	return;
}
EXPORT_SYMBOL(enable_cpus);

u32 get_core_dfs(int cpu_num)
{
	u32 core_dfs, dfs;
	uint64_t mmio;

	mmio = (uint64_t)cpu_io_mmio(cpu_num/32,SYS);
	core_dfs = nlm_hal_read_32bit_reg(mmio, SYS_COREDFSDIVCTRL);
	dfs  = SYS_CORE_DFS(core_dfs, (cpu_num >> 2));
	return dfs;
}

EXPORT_SYMBOL(get_core_dfs);
