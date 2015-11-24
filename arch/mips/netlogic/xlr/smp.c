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


#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/smp.h>

#include <asm/mipsregs.h>
#include <asm/mmu_context.h>
#include <asm/atomic.h>
//#include <asm/cpumask.h>

#include <asm/netlogic/sim.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/xlr_user_mac.h>

#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/processor.h>

#include <asm/mach-netlogic/mmu.h>

extern volatile cpumask_t cpu_callin_map;
extern void __init phoenix_smp_init(void); 
extern void phoenix_smp_finish(void);

extern void smp_tune_scheduling (void);
extern void ptr_smp_boot(unsigned long, unsigned long, unsigned long);
struct smp_boot_info smp_boot;
extern void prom_reconfigure_thr_resources(void);
extern uint32_t nlm_common_loader_mask;
extern unsigned long nlm_common_ebase;


int phys_proc_id[NR_CPUS]; /* cpuid+thrid of each logical CPU */
cpumask_t phys_cpu_present_map;
extern void asmlinkage smp_bootstrap(void);
extern void core_send_ipi(int cpu, unsigned int action);

void nlm_send_ipi_single(int cpu, unsigned int action)
{
    core_send_ipi(cpu, action);
}

void nlm_send_ipi_mask(const struct cpumask * mask, unsigned int action)
{
    int cpu;
    for_each_cpu(cpu, mask){
        core_send_ipi(cpu, action);
    }
}

/*
 * Code to run on secondary just after probing the CPU
 */
static void __cpuinit nlm_init_secondary(void)
{
    extern void nlm_smp_irq_init(void);

    nlm_smp_irq_init();
    /* Time init for this cpu is done in mips_clockevent_init() */
}

void nlm_smp_finish(void)
{
#if !defined(CONFIG_NLM_XLP)
    nlm_common_msgring_cpu_init();
#endif
}

void nlm_cpus_done(void)
{
}

/* Boot all other cpus in the system, initialize them, and
   bring them into the boot fn */
void nlm_boot_secondary(int logical_cpu, struct task_struct *idle)
{
	unsigned long gp = (unsigned long)task_thread_info(idle);
	unsigned long sp = (unsigned long)__KSTK_TOS(idle);
	int cpu = cpu_logical_map(logical_cpu);

/* 	printk("[%s]: (PROM): waking up phys cpu# %d: address of boot_info=%p (addressof(ready)=%p)\n", */
/* 	       __FUNCTION__, cpu, &(smp_boot.boot_info[cpu]), &((smp_boot.boot_info[cpu]).ready)); */
  
	smp_boot.boot_info[cpu].sp = sp;
	smp_boot.boot_info[cpu].gp = gp;
	smp_boot.boot_info[cpu].fn = (unsigned long)&smp_bootstrap;  
	/* barrier */
	__sync();
	smp_boot.boot_info[cpu].ready = 1;
  
/* 	printk("[%s]: (PROM): sent a wakeup message to cpu %d\n", __FUNCTION__, cpu); */
}

extern void ptr_smp_boot(unsigned long, unsigned long, unsigned long);
struct smp_boot_info smp_boot;
extern void prom_reconfigure_thr_resources(void);
extern uint32_t nlm_common_loader_mask;
extern unsigned long nlm_common_ebase;

unsigned int fast_syscall_cpumask_phy = 0x1;

void __init nlm_smp_setup(void)
{
	int num_cpus = 1;
	__u32 boot_cpu_online_map = 0, boot_cpu = 0x0;


	extern __u32 ipi_3_counter_tx[NR_CPUS][NR_CPUS];
	extern __u32 ipi_3_counter_rx[NR_CPUS];
	int i=0, j=0;

	boot_cpu = hard_smp_processor_id();

	cpus_clear(phys_cpu_present_map);
	/*	cpu_set(0, phys_cpu_present_map);
	__cpu_number_map[0] = 0;
	__cpu_logical_map[0] = 0;  
	dev_tree_en fix , and also not required for the existing case also */

	cpus_clear(cpu_possible_map);
	/* cpu_set(0, cpu_possible_map); */

	/* Initialize the ipi debug stat variables */
	for(i=0;i<NR_CPUS;i++) {
		for(j=0;j<NR_CPUS;j++)
			ipi_3_counter_tx[i][j] = 0;
  
		ipi_3_counter_rx[i] = 0;
	}


	boot_cpu_online_map = smp_boot.online_map;
	printk("(PROM) CPU present map: %x\n", boot_cpu_online_map);

	/* 0th entry in the logical_map should be the bootcpu and all
       others proceeds after that */
	/* Fill the entries for boot cpu */
	boot_cpu_online_map &= (~(1 << boot_cpu));
	cpu_set(boot_cpu, phys_cpu_present_map);
	__cpu_number_map[boot_cpu] = 0;
	__cpu_logical_map[0] = boot_cpu;
	cpu_set(0, cpu_possible_map);

	for(i = 0;i<NR_CPUS;i++) {
		if (boot_cpu_online_map & (1<<i)) {
			cpu_set(i, phys_cpu_present_map);
			__cpu_number_map[i] = num_cpus;
			__cpu_logical_map[num_cpus] = i;
			cpu_set(num_cpus, cpu_possible_map);
			++num_cpus;
		}
	}


	fast_syscall_cpumask_phy = (unsigned int)phys_cpu_present_map.bits[0];

	printk("Phys CPU present map: %lx, possible map %lx\n", 
	       (unsigned long)phys_cpu_present_map.bits[0], 
	       (unsigned long)cpu_possible_map.bits[0]);

	printk("Detected %i Slave CPU(s)\n", num_cpus);
}

void nlm_prepare_cpus(unsigned int max_cpus)
{
}


struct plat_smp_ops nlm_smp_ops = {
    .send_ipi_single    = nlm_send_ipi_single,
    .send_ipi_mask      = nlm_send_ipi_mask,
    .init_secondary     = nlm_init_secondary,
    .smp_finish     = nlm_smp_finish,
    .cpus_done      = nlm_cpus_done,
    .boot_secondary     = nlm_boot_secondary,
    .smp_setup      = nlm_smp_setup,
    .prepare_cpus       = nlm_prepare_cpus,
};



void prom_boot_cpus_secondary(void *args)
{
	int cpu = hard_smp_processor_id();
  
	write_c0_ebase((uint32_t)nlm_common_ebase);
	atomic_add((1<<cpu), (atomic_t *)&smp_boot.online_map);
	for(;;) {
		if (smp_boot.boot_info[cpu].ready) break;
	}
	__sync();

	prom_reconfigure_thr_resources();

        setup_mapped_kernel_tlbs(TRUE, FALSE);
        setup_mapped_kernel_tlbs(FALSE, FALSE);

	ptr_smp_boot(smp_boot.boot_info[cpu].fn, smp_boot.boot_info[cpu].sp, 
		     smp_boot.boot_info[cpu].gp);
}



#ifdef CONFIG_NLM_XLP
/* place holder for boot multiple cpu function */

#endif //CONFIG_NLM_XLP
