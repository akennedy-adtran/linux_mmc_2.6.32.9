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

#define NETLOGIC_PMC_COUNT_ALL_THREADS	(1U << 25)

#define XLP_MAX_PERF_COUNTERS 4
#define XLP_MAX_CPU_CORES 8
#define XLP_MAX_CPUS 32

extern struct op_mips_model op_model_xlp;

static struct nlm_common_register_config {
	unsigned int control[XLP_MAX_PERF_COUNTERS];
	uint64_t reset_counter[XLP_MAX_PERF_COUNTERS];
}reg;

volatile int g_stop_pmc[XLP_MAX_CPUS];


static void nlm_common_reg_setup(struct op_counter_config *ctr)
{
	unsigned int counters = op_model_xlp.num_counters;
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
			NETLOGIC_PMC_ENABLE_INT;
		if (ctr[i].kernel)
			reg.control[i] |= NETLOGIC_PMC_DOM_KERNEL;
		if (ctr[i].user)
			reg.control[i] |= NETLOGIC_PMC_DOM_USR;
		if (ctr[i].exl)
			reg.control[i] |= NETLOGIC_PMC_DOM_EXL;

		reg.reset_counter[i] = 0x8000000000000000ULL - ctr[i].count;
	}
	wmb();
	local_irq_restore(flags);
}

/* Program all of the registers in preparation for enabling profiling.  */

static void netlogic_cpu_setup (void *args)
{
	unsigned long flags;

	local_irq_save(flags);
	__write_64bit_c0_register($25, 1, reg.reset_counter[0]);
	__write_64bit_c0_register($25, 3, reg.reset_counter[1]);
	__write_64bit_c0_register($25, 5, reg.reset_counter[2]);
	__write_64bit_c0_register($25, 7, reg.reset_counter[3]);
	local_irq_restore(flags);
}

static void netlogic_cpu_start(void *args)
{
	unsigned long flags;

	/* Start all counters on current CPU */
	local_irq_save(flags);
	__write_32bit_c0_register($25, 0, reg.control[0]);
	__write_32bit_c0_register($25, 2, reg.control[1]);
	__write_32bit_c0_register($25, 4, reg.control[2]);
	__write_32bit_c0_register($25, 6, reg.control[3]);
	local_irq_restore(flags);

	local_irq_save(flags);
	g_stop_pmc[hard_smp_processor_id()] = 0;
	local_irq_restore(flags);

	wmb();

}

static void netlogic_cpu_stop(void *args)
{
	unsigned long flags;

	local_irq_save(flags);
	g_stop_pmc[hard_smp_processor_id()] = 1;
	local_irq_restore(flags);

	/* Stop all counters on current CPU */
	local_irq_save(flags);
	__write_32bit_c0_register($25, 0, 0);
	__write_32bit_c0_register($25, 2, 0);
	__write_32bit_c0_register($25, 4, 0);
	__write_32bit_c0_register($25, 6, 0);
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
	uint64_t counter1, counter2;
	uint64_t counter3, counter4;
	int h_id = hard_smp_processor_id();/* 0, 1, 2, 3, 4, .....31 */

	if(g_stop_pmc[h_id])
		return;

	counter1 = __read_64bit_c0_register ($25, 1);
	counter2 = __read_64bit_c0_register ($25, 3);
	counter3 = __read_64bit_c0_register ($25, 5);
	counter4 = __read_64bit_c0_register ($25, 7);

	if (((long)counter1) < 0) {
		__write_32bit_c0_register($25, 0, 0);
		oprofile_add_sample(regs, 0);
		counter1 = reg.reset_counter[0];
		__write_64bit_c0_register($25, 1, counter1);
		__write_32bit_c0_register($25, 0, reg.control[0]);

		wmb();
	}
	if (((long)counter2) < 0) {
		__write_32bit_c0_register($25, 2, 0);
		oprofile_add_sample(regs, 1);
		counter2 = reg.reset_counter[1];
		__write_64bit_c0_register($25, 3, counter2);
		__write_32bit_c0_register($25, 2, reg.control[1]);

		wmb();
	}

	if (((long)counter3) < 0) {
		__write_32bit_c0_register($25, 4, 0);
		oprofile_add_sample(regs, 2);
		counter3 = reg.reset_counter[2];
		__write_64bit_c0_register($25, 5, counter3);
		__write_32bit_c0_register($25, 4, reg.control[2]);

		wmb();
	}
	if (((long)counter4) < 0) {
		__write_32bit_c0_register($25, 6, 0);
		oprofile_add_sample(regs, 3);
		counter4 = reg.reset_counter[3];
		__write_64bit_c0_register($25, 7, counter4);
		__write_32bit_c0_register($25, 6, reg.control[3]);

		wmb();
	}
	return ;
}

static void nlm_common_reset_perf_counters(void)
{
	__write_32bit_c0_register($25, 0, 0);
	__write_64bit_c0_register($25, 1, 0);

	__write_32bit_c0_register($25, 2, 0);
	__write_64bit_c0_register($25, 3, 0);

	__write_32bit_c0_register($25, 4, 0);
	__write_64bit_c0_register($25, 5, 0);

	__write_32bit_c0_register($25, 6, 0);
	__write_64bit_c0_register($25, 7, 0);

}

static int nlm_common_init(void)
{
	int i;

	for(i=0; i < XLP_MAX_CPUS; i++)
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
struct op_mips_model op_model_xlp = {
	.reg_setup	= nlm_common_reg_setup,
	.cpu_setup	= netlogic_cpu_setup,
	.init		= nlm_common_init,
	.exit		= nlm_common_exit,
	.cpu_start	= netlogic_cpu_start,
	.cpu_stop	= netlogic_cpu_stop,
	.cpu_type	= "mips/xlp",
	.num_counters	= XLP_MAX_PERF_COUNTERS,
};
