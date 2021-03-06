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


#include <linux/init.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <asm/mipsregs.h>

#include "op_impl.h"

#define NETLOGIC_PMC_EVENT_MASK			0x3f
#define NETLOGIC_PMC_EVENT(event) 		\
				((event & NETLOGIC_PMC_EVENT_MASK) << 5)
#define NETLOGIC_PMC_DOM_EXL       		(1U << 0)
#define NETLOGIC_PMC_DOM_KERNEL			(1U << 1)
#define NETLOGIC_PMC_DOM_USR			(1U << 3)
#define NETLOGIC_PMC_ENABLE_INT          (1U << 4)
#define NETLOGIC_PMC_THREAD_ID_MASK		0x03
#define NETLOGIC_PMC_THREAD_ID(tid)      	\
				((tid & NETLOGIC_PMC_THREAD_ID_MASK) << 11)
#define NETLOGIC_PMC_COUNT_ALL_THREADS	(1U << 13)

#define XLR_MAX_PERF_COUNTERS 2
#define XLR_MAX_CPU_CORES 8
#define XLR_MAX_CPUS 32

extern struct op_mips_model op_model_xlr;

static struct nlm_common_register_config {
	unsigned int control[XLR_MAX_PERF_COUNTERS];
	unsigned int reset_counter[XLR_MAX_PERF_COUNTERS];
}reg;

/* Compute all of the registers in preparation for enabling profiling.  */
volatile int nlm_common_perf_core_setup[XLR_MAX_CPU_CORES];
spinlock_t nlm_common_perf_lock = SPIN_LOCK_UNLOCKED;
int g_stop_pmc[XLR_MAX_CPUS];

/* 
 * Per core Performance counter overflow mask. This 
 * mask is set to 0xF by the perf counter "owner" 
 * when a performance event counter overflows. This 
 * is used by other threads in the core to call 
 * oprofile_add_sample
 */

volatile int nlm_common_pc_of_mask1[XLR_MAX_CPUS];
volatile int nlm_common_pc_of_mask2[XLR_MAX_CPUS];

/* Check if this thread is the owner for PerfCounters in this core */
int nlm_common_pmc_owner(void)
{
	int cpu_id ;
	unsigned long flags;

	/* Allow only one thread in each core to set perfcounter events */
	spin_lock_irqsave(&nlm_common_perf_lock, flags);
	cpu_id = netlogic_cpu_id();
	if(nlm_common_perf_core_setup[cpu_id] == hard_smp_processor_id()) {
		spin_unlock_irqrestore(&nlm_common_perf_lock, flags);
		return 1;
	}
	spin_unlock_irqrestore(&nlm_common_perf_lock, flags);
	return 0;
}
/* To be called only from perf interrup handler */
int nlm_common_pmc_owner_nolock(void)
{
	int cpu_id, h_id ;
	cpu_id = netlogic_cpu_id();
	h_id = hard_smp_processor_id();

	if(nlm_common_perf_core_setup[cpu_id] == h_id) {
		return 1;
	}
	return 0;

}

/* 
 * Check if perfcounters is already owned 
 * by some other thread in this core 
 */

static int nlm_common_pmc_owned(void)
{
	int cpu_id ;
	unsigned long flags;
	int thr_id;

	/* Allow only thread0 in each core to set perfcounter events */
	spin_lock_irqsave(&nlm_common_perf_lock, flags);
	thr_id = netlogic_thr_id();
	if(thr_id) {
		spin_unlock_irqrestore(&nlm_common_perf_lock, flags);
		return 1;
	}
	cpu_id = netlogic_cpu_id();
	if(nlm_common_perf_core_setup[cpu_id] == -1) {
		nlm_common_perf_core_setup[cpu_id] = hard_smp_processor_id();
		spin_unlock_irqrestore(&nlm_common_perf_lock, flags);
		return 0;
	}
	if(nlm_common_perf_core_setup[cpu_id] == hard_smp_processor_id()) {
		spin_unlock_irqrestore(&nlm_common_perf_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&nlm_common_perf_lock, flags);
	return 1;
}
static void nlm_common_reg_setup(struct op_counter_config *ctr)
{
	unsigned int counters = op_model_xlr.num_counters;
	int i;
	unsigned long flags;

	/* Compute the performance counter control word.  */
	local_irq_save(flags);
	for(i=0; i < counters; i++) {
		reg.control[i] = 0;
		reg.reset_counter[i] = 0;

		if (!ctr[i].enabled)
			continue;

		reg.control[i] = NETLOGIC_PMC_EVENT(ctr[i].event) | 
			NETLOGIC_PMC_ENABLE_INT | 
			NETLOGIC_PMC_COUNT_ALL_THREADS;
		if (ctr[i].kernel)
			reg.control[i] |= NETLOGIC_PMC_DOM_KERNEL;
		if (ctr[i].user)
			reg.control[i] |= NETLOGIC_PMC_DOM_USR;
		if (ctr[i].exl)
			reg.control[i] |= NETLOGIC_PMC_DOM_EXL;

		reg.reset_counter[i] = 0x80000000 - ctr[i].count;
	}
	wmb();
	local_irq_restore(flags);
}

/* Program all of the registers in preparation for enabling profiling.  */

static void netlogic_cpu_setup (void *args)
{
	unsigned long flags;
	/* 
	 * Check if some other thread has already taken 
	 * the ownership of setting perf counters. If not,
	 * set the ownership
	 */
	if(nlm_common_pmc_owned())
		return;

	local_irq_save(flags);
	__write_32bit_c0_register($25, 1, reg.reset_counter[0]);
	__write_32bit_c0_register($25, 3, reg.reset_counter[1]);
	local_irq_restore(flags);
}

static void netlogic_cpu_start(void *args)
{
	int core_start = netlogic_cpu_id() * 4;
	int i;
	unsigned long flags;

	if(nlm_common_pmc_owned()) {
		return;
	}
	/* Start all counters on current CPU */
	local_irq_save(flags);

	__write_32bit_c0_register($25, 0, reg.control[0]);
	__write_32bit_c0_register($25, 2, reg.control[1]);

	for(i=0; i < 4; i++)
		g_stop_pmc[core_start + i] = 0;
	wmb();
	local_irq_restore(flags);
}

static void netlogic_cpu_stop(void *args)
{
	int cpu_id = netlogic_cpu_id();
	int core_start = cpu_id * 4;
	int i;
	unsigned long flags;

	local_irq_save(flags);
	g_stop_pmc[hard_smp_processor_id()] = 1;
	local_irq_restore(flags);

	if(nlm_common_pmc_owned())
		return;
	/* Stop all counters on current CPU */
	local_irq_save(flags);
	__write_32bit_c0_register($25, 0, 0);
	__write_32bit_c0_register($25, 2, 0);
	for(i=0; i < 4; i++) {
		nlm_common_pc_of_mask1[core_start + i] = 0;
		nlm_common_pc_of_mask2[core_start + i] = 0;
	}
	local_irq_restore(flags);
}

/* 
 * This handler is called from count compare timer 
 * interrupt as the perf counter overflow interrupt 
 * shares the same count compare IRQ.
 */

void nlm_common_oprofile_int_handler(int irq, void * dev_id,
	struct pt_regs *regs)
{
	uint32_t counter1, counter2;
	uint32_t control1, control2;
	int i;
	int cpu_id = netlogic_cpu_id() * 4; /* 0, 4, 8 ... 28 */
	int h_id = hard_smp_processor_id();/* 0, 1, 2, 3, 4, .....31 */
	int ret, lcpu;
	int sample1_taken=0;
	int sample2_taken=0;
	extern struct plat_smp_ops *mp_ops;

	if(g_stop_pmc[h_id])
		return;

	if(((ret = nlm_common_pmc_owner_nolock()) == 0)) {
		/* if any counter overflow occured on this core.... */
		if(nlm_common_pc_of_mask1[h_id]) {
			oprofile_add_sample(regs, 0);
		}
		if(nlm_common_pc_of_mask2[h_id]) {
			oprofile_add_sample(regs, 1);
		}
		return;
	}

	control1 = __read_32bit_c0_register ($25, 0);
	control2 = __read_32bit_c0_register ($25, 2);


	counter1 = __read_32bit_c0_register ($25, 1);
	counter2 = __read_32bit_c0_register ($25, 3);

	if (((int)counter1) < 0) {
		__write_32bit_c0_register($25, 0, 0);
		oprofile_add_sample(regs, 0);
		counter1 = reg.reset_counter[0];
		sample1_taken = 1;

		for(i=0; i < 4; i++)
			nlm_common_pc_of_mask1[cpu_id + i] = 1;
		wmb();
		for(i=1; i < 4; i++) {
			lcpu = cpu_number_map(cpu_id+i);
			if(lcpu && cpu_isset(lcpu, cpu_online_map)) {
				mp_ops->send_ipi_single(lcpu, SMP_OPROFILE_IPI);
			}
		}
	}
	if (((int)counter2) < 0) {
		__write_32bit_c0_register($25, 2, 0);
		oprofile_add_sample(regs, 1);
		counter2 = reg.reset_counter[1];
		sample2_taken = 1;

		for(i=0; i < 4; i++)
			nlm_common_pc_of_mask2[cpu_id + i] = 1;
		wmb();
		for(i=1; i < 4; i++) {
			lcpu = cpu_number_map(cpu_id+i);
			if(lcpu && cpu_isset(lcpu, cpu_online_map)) {
				mp_ops->send_ipi_single(lcpu, SMP_OPROFILE_IPI);
			}
		}
	}

	if(sample1_taken) {
		__write_32bit_c0_register($25, 1, counter1);
		__write_32bit_c0_register($25, 0, reg.control[0]);
	}
	if(sample2_taken) {
		__write_32bit_c0_register($25, 3, counter2);
		__write_32bit_c0_register($25, 2, reg.control[1]);
	}

	return ;
}

static void nlm_common_reset_perf_counters(void)
{
	__write_32bit_c0_register($25, 0, 0);
	__write_32bit_c0_register($25, 1, 0);

	__write_32bit_c0_register($25, 2, 0);
	__write_32bit_c0_register($25, 3, 0);

}

static int nlm_common_init(void)
{
	int i;

	for(i=0; i < XLR_MAX_CPU_CORES; i++)
		nlm_common_perf_core_setup[i] = -1;

	for(i=0; i < XLR_MAX_CPUS; i++)
		g_stop_pmc[i] = 1;
	nlm_common_reset_perf_counters();

	return 0;
}

static void nlm_common_exit(void)
{
	nlm_common_reset_perf_counters();
	return;
}


/*
 * The following is assigned to the variable 
 * 'lmodel' in oprofile_arch_init()
 */
struct op_mips_model op_model_xlr = {
	.reg_setup	= nlm_common_reg_setup,
	.cpu_setup	= netlogic_cpu_setup,
	.init		= nlm_common_init,
	.exit		= nlm_common_exit,
	.cpu_start	= netlogic_cpu_start,
	.cpu_stop	= netlogic_cpu_stop,
	.cpu_type	= "mips/xlr",
	.num_counters	= XLR_MAX_PERF_COUNTERS,
};
