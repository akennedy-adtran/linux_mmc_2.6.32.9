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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/smp.h>

#include <asm/mipsregs.h>
#include <asm/mmu_context.h>
#include <asm/atomic.h>
#include <asm/cacheops.h>

#include <asm/netlogic/xlp.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/interrupt.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/hal/nlm_hal_fmn.h>
#include <linux/module.h>

#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/processor.h>

#include <asm/netlogic/cpumask.h>
#include <linux/nodemask.h>

#include <asm/mach-netlogic/mmu.h>

#include <asm/netlogic/xlp8xx/cpu_control_macros.h>
#include "pic/timer-base.h"

struct smp_boot_info smp_boot;
EXPORT_SYMBOL(smp_boot);
cpumask_t fdt_cpumask;
//cpumask_t fdt_loadermask;
cpumask_t phys_cpu_present_map;
EXPORT_SYMBOL(phys_cpu_present_map);

extern void ptr_smp_boot(unsigned long, unsigned long, unsigned long);
extern void prom_reconfigure_thr_resources(void);
extern unsigned long nlm_common_ebase;
extern void enable_cpus(unsigned int node, unsigned online_mask);
extern void nlm_smp_irq_init(void);
extern void asmlinkage smp_bootstrap(void);
extern void enable_msgconfig_int(void);
extern void xlp_pic_ite_init(const struct cpumask *);

static int node_exist[NLM_MAX_NODES];
/*
 * Input parameter is logical cpu number.
 * Should convert to physical cpu before using it
 */
void nlm_send_ipi_single(int lcpu, unsigned int action)
{
	int phys_cpu = cpu_logical_map(lcpu);
	__u32 node = phys_cpu / 32;
	__u32 ipi = 0;
	__u8 nmi = 0;
	phys_cpu = phys_cpu % 32;	/* This need to be changed for NUMA?? */

	if (action & SMP_CALL_FUNCTION) {
		ipi |= XLP_IRQ_IPI_SMP_FUNCTION_RVEC;
	} else if (action & SMP_RESCHEDULE_YOURSELF) {
		ipi |= XLP_IRQ_IPI_SMP_RESCHEDULE_RVEC;
	} else if (action & SMP_CALL_KGDB_HOOK_RVEC) {
		ipi |= XLP_IRQ_IPI_SMP_KGDB_RVEC;
		/* for KGDB enable NMI also */
		nmi = 1;
	} else
		return;

	smp_mb();

	nlm_hal_pic_send_ipi(nmi, (ipi & 0x3f), node, phys_cpu);
}

void nlm_send_ipi_mask(const struct cpumask * mask, unsigned int action)
{
	int cpu;

	for_each_cpu(cpu, mask){
		nlm_send_ipi_single(cpu, action);
	}
}

#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
extern void nlm_common_timer_setup(void);
#endif

/*
 * Code to run on secondary just after probing the CPU
 */

void xlp_pic_init(u8 node);
static void __cpuinit nlm_init_secondary(void)
{
	int cpu;
	cpu = hard_smp_processor_id();
	/* Time init for this cpu is done in mips_clockevent_init() */
	nlm_smp_irq_init();
	enable_msgconfig_int();

	/* Workaround for XLP A0 Multi-Node bug */
	if ((cpu % NLM_MAX_CPU_PER_NODE) == 0) {
		xlp_pic_init(cpu/NLM_MAX_CPU_PER_NODE);
#if defined CONFIG_NLM_XLP && !defined CONFIG_XLP_REPLACE_R4K_TIMER
		nlm_common_timer_setup();
#endif
	}
}

void nlm_smp_finish(void)
{
}

void reassign_irq_mask(void);
int irq_select_affinity_usr(unsigned int);
void xlp_prog_all_node_ites(void );
void xlp_reprogram_timer_masks(void);

static long __init fmn_config(void *arg)
{
	extern void *fdt;
	printk("Doing XLP FMN init from cpu %d for node %d\n",
			hard_smp_processor_id(), (int)(long)arg);
	nlm_hal_fmn_init(fdt, (int)(long)arg);
	return 0;
}

void __cpuinit nlm_cpus_done(void)
{
	xlp_prog_all_node_ites();
#if defined CONFIG_XLP_REPLACE_R4K_TIMER
	/* irq_select_affinity_usr(XLP_TIMER_IRQ(0, 0)); */
	xlp_reprogram_timer_masks();
#endif
}

/*
 * init/main.c : smp_init ==> cpu_up ==> _cpu_up => __cpu_up (arch/mips/kernel/
 * smp.c) ==> mp_ops->boot_secondary
 * The cpu argument is the bit number from cpu_present_mask (for_each_online_cpu
 * ) => logical cpu id
 */
void __cpuinit nlm_boot_secondary(int cpu, struct task_struct *idle)
{
	unsigned long gp = (unsigned long)task_thread_info(idle);
	unsigned long sp = (unsigned long)__KSTK_TOS(idle);
	int phys_cpu = cpu_logical_map(cpu);

//	printk("nlm_boot_secondary: logical cpu %d physical cpu %d\n", cpu, phys_cpu);

	smp_boot.boot_info[phys_cpu].sp = sp;
	smp_boot.boot_info[phys_cpu].gp = gp;
	smp_boot.boot_info[phys_cpu].fn = (unsigned long)&smp_bootstrap;

	/* barrier */
	__sync();
	smp_boot.boot_info[phys_cpu].ready = 1;

/*  	printk("[%s]: (PROM): sent a wakeup message to cpu %d\n", __FUNCTION__, cpu);  */
}

void __cpuinit nlm_smp_setup(void)
{
	int num_cpus = 0;
	int boot_cpu = hard_smp_processor_id();
	int i = 0;
	char buf[CPUMASK_BUF];

	/* Initialize maps */
	for (i = 0; i < NR_CPUS; i++) {
		__cpu_number_map[i] = NR_CPUS + 1; /* Set to invalid value */
		__cpu_logical_map[i] = NR_CPUS + 1; /* Set to invalid value */
	}

	/* Setup map for master cpu */
	__cpu_number_map[boot_cpu] = 0;
	__cpu_logical_map[0] = boot_cpu;
	num_cpus = 1;
	cpumask_clear(&cpu_possible_map);
	cpumask_clear(&cpu_present_map);
	cpumask_clear(&phys_cpu_present_map);
	cpu_set(0, cpu_possible_map);
	cpu_set(boot_cpu, phys_cpu_present_map);

	node_exist[boot_cpu / NLM_NCPUS_PER_NODE] = (boot_cpu << 16) | 1;
	/* Setup map for other cpus */
	for (i = 0; i < NR_CPUS; i++) {

		if (i == boot_cpu) continue;

		if (cpumask_test_cpu(i, &fdt_cpumask)) {
			__cpu_number_map[i] = num_cpus;
			__cpu_logical_map[num_cpus] = i;
			cpu_data[num_cpus].core = (int) (__cpu_logical_map[num_cpus]/XLP_THREADS_PER_CORE);
			cpu_set(num_cpus, cpu_possible_map);
			cpu_set(i, phys_cpu_present_map);
			num_cpus++;
			if(node_exist[i / NLM_NCPUS_PER_NODE] == 0) 
				node_exist[i/ NLM_NCPUS_PER_NODE] = (i << 16) | 1;
		}
	}

	cpu_present_map = cpu_possible_map;
	cpumask_scnprintf(buf, CPUMASK_BUF, &cpu_present_map);
	printk("Present  CPU map %s\n", buf);
	cpumask_scnprintf(buf, CPUMASK_BUF, &cpu_possible_map);
	printk("Possible CPU map %s\n", buf);
	printk("Detected %d CPU(s)\n", num_cpus);
}

#ifdef CONFIG_XEN
extern int xen_enable_cpus(int node, uint32_t onlinemask);
#define hw_enable_cpus xen_enable_cpus
#else
#define hw_enable_cpus enable_cpus
#endif

//* Removed config_mmu() - only used for XLP8xx A0 */

/*
 * this function requires cpu_present_map
 */
int __cpuinit wakeup_secondary_cpus(void)
{
	cpumask_t mask32;
	int node;

	cpumask_clear(&mask32);
	uint32_to_cpumask(&mask32, 0xffffffff);

	for (node = 0; node < 4; node++) {
		cpumask_t tmpmask, nodemask;
//		cpumask_t tmploader_mask, loadernode_mask;
		unsigned int onlinemask;

		cpumask_clear(&tmpmask);
		cpumask_clear(&nodemask);
//		cpumask_clear(&tmploader_mask);
//		cpumask_clear(&loadernode_mask);

		cpumask_shift_right(&nodemask, &fdt_cpumask, 32 * node);
		cpumask_and(&tmpmask, &nodemask, &mask32);

//		cpumask_shift_right(&loadernode_mask, &fdt_loadermask, 32 * node);
//		cpumask_and(&tmploader_mask, &loadernode_mask, &mask32);

//		if (cpumask_empty(&tmpmask) && cpumask_empty(&tmploader_mask))
		if (cpumask_empty(&tmpmask))
			continue;

//		onlinemask = cpumask_to_uint32(&tmpmask) | cpumask_to_uint32(&tmploader_mask);
		onlinemask = cpumask_to_uint32(&tmpmask);

		hw_enable_cpus(node, onlinemask);

		printk("Enabled cpus (0x%08x) on node@%d\n", onlinemask, node);
	}

	return 0;
}

void nlm_prepare_cpus(unsigned int max_cpus)
{
}

struct plat_smp_ops nlm_smp_ops = {
    .send_ipi_single    = nlm_send_ipi_single,
    .send_ipi_mask      = nlm_send_ipi_mask,
    .init_secondary     = nlm_init_secondary,
    .smp_finish     	= nlm_smp_finish,
    .cpus_done      	= nlm_cpus_done,
    .boot_secondary     = nlm_boot_secondary,
    .smp_setup      	= nlm_smp_setup,
    .prepare_cpus       = nlm_prepare_cpus,
};

void prom_boot_cpus_secondary(void *args)
{
	int cpu = hard_smp_processor_id();

	write_c0_ebase((uint32_t)nlm_common_ebase);

	/* Announce that this cpu is available */
	cpumask_test_and_set_cpu(cpu, &smp_boot.online_map);

	/* Wait for master to signal */
	for (;;) {
		if (smp_boot.boot_info[cpu].ready) break;
	}
	__sync();

	prom_reconfigure_thr_resources();

	setup_mapped_kernel_tlbs(TRUE, FALSE);
	setup_mapped_kernel_tlbs(FALSE, FALSE);

#if 0
	if (cpumask_test_cpu(cpu, &fdt_loadermask)) {
		/* TLBMISS_HANDLER_SETUP */
		write_c0_context(cpu << 26);
		back_to_back_c0_hazard();
		pgd_current[cpu] = (unsigned long)(swapper_pg_dir);
		write_c0_pagemask(PM_DEFAULT_MASK);
	}
#endif

	/* Entry into the kernel (smp_bootstrap) */
	ptr_smp_boot(smp_boot.boot_info[cpu].fn, smp_boot.boot_info[cpu].sp,
		     smp_boot.boot_info[cpu].gp);
}

static int __init nlm_fmn_init(void)
{
	extern void *fdt;
	int node, rv;

	/* Better to do node level fmn initialization */
	if(get_dom_fmn_node_ownership(fdt, 0)) {
		for(node = 0; node < NLM_MAX_NODES; node++) {
			if(!node_exist[node])
				continue;
			rv = work_on_cpu(node_exist[node] >> 16, fmn_config, (void *)(long)node);
			if(rv < 0) 
				panic("Fmn init failed\n");
		}
	} else
		printk("Skipping XLP FMN initialization due to lack of ownership\n");

	return 0;
}
core_initcall(nlm_fmn_init);
