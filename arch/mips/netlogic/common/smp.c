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


#include <asm/netlogic/64bit.h>
#include <asm/addrspace.h>
#include <asm/smp.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/hardirq.h>
#include <linux/module.h>

#include <asm/netlogic/sim.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/xlp_irq.h>

/* ipi statistics counters for debugging */
__u32 ipi_3_counter_tx[NR_CPUS][NR_CPUS];
__u32 ipi_3_counter_rx[NR_CPUS];

extern void save_epc(unsigned long *epc);
extern void smp_call_function_interrupt(void);
static int nlm_common_ipi_stats[NR_CPUS];
static unsigned long nlm_common_ipi_epc[NR_CPUS];

#ifdef CONFIG_NLM_XLR
#define SET_IPI_VECTOR(x, y) x |= y

void core_send_ipi(int logical_cpu, unsigned int action)
{
	int cpu = cpu_logical_map(logical_cpu);
	__u32 tid = cpu & 0x3;
	__u32 pid = (cpu >> 2) & 0x07;
	__u32 ipi = (tid << 16) | (pid << 20);

	if (action & SMP_CALL_FUNCTION) {
		SET_IPI_VECTOR(ipi, IRQ_IPI_SMP_FUNCTION);
	} else if (action & SMP_RESCHEDULE_YOURSELF) {
		SET_IPI_VECTOR(ipi, IRQ_IPI_SMP_RESCHEDULE);
	} else if (action & SMP_CALL_KGDB_HOOK) {
		SET_IPI_VECTOR(ipi, IRQ_IPI_SMP_KGDB);
		/* set NMI also for KGDB */
		SET_IPI_VECTOR(ipi, (1 << 8));
	} else if (action & SMP_OPROFILE_IPI) {
		SET_IPI_VECTOR(ipi, IRQ_IPI_OPROFILE);
	}
#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
	else if (action & SMP_NETRX_IPI) {
		SET_IPI_VECTOR(ipi, IRQ_IPI_NETRX);
	}
#endif				/* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */
	else
		return;

	pic_send_ipi(ipi);
}
#endif

extern __u64 nlm_common_irq_mask;

void nlm_common_smp_finish(void)
{
#if !defined(CONFIG_NLM_XLP)
	nlm_common_msgring_cpu_init();
#endif
}

#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
extern void skb_transfer_finish(void);
#endif				/* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */

#ifdef CONFIG_SMP
/* IRQ_IPI_SMP_FUNCTION Handler */
void nlm_smp_function_ipi_handler(unsigned int irq, struct irq_desc *desc)
{
	nlm_common_ipi_stats[smp_processor_id()]++;
	save_epc(&nlm_common_ipi_epc[smp_processor_id()]);
	smp_call_function_interrupt();
	nlm_common_ipi_stats[smp_processor_id()]--;
}

/* IRQ_IPI_SMP_RESCHEDULE  handler */
void nlm_smp_resched_ipi_handler(unsigned int irq, struct irq_desc *desc)
{
	nlm_common_ipi_stats[smp_processor_id()]++;
	save_epc(&nlm_common_ipi_epc[smp_processor_id()]);

	/* Announce that we are for reschduling */
	set_need_resched();
	nlm_common_ipi_stats[smp_processor_id()]--;
}

#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
/* nlm_ip_flow_ipi_handler HANDLER */
void nlm_ip_flow_ipi_handler(unsigned int irq, struct irq_desc *desc)
{
	nlm_common_ipi_stats[smp_processor_id()]++;
	save_epc(&nlm_common_ipi_epc[smp_processor_id()]);

	/* do_IRQ called irq_enter() before calling this desc->handler */
	skb_transfer_finish();
	nlm_common_ipi_stats[smp_processor_id()]--;

}
#endif /* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */
#endif

void nlm_common_ipi_handler(int irq, struct pt_regs *regs)
{
	nlm_common_ipi_stats[smp_processor_id()]++;
	save_epc(&nlm_common_ipi_epc[smp_processor_id()]);

	if (irq == NLM_IRQ_IPI_SMP_FUNCTION) {
		smp_call_function_interrupt();
	}
#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
	else if (irq == IRQ_IPI_NETRX) {
		irq_enter();

		skb_transfer_finish();

		/* run soft IRQ at the end */
		irq_exit();
	}
#endif	/* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */
	else {
		/* Announce that we are for reschduling */
		set_need_resched();
	}
	nlm_common_ipi_stats[smp_processor_id()]--;
}
