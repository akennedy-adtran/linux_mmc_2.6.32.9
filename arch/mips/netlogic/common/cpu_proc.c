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
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/cpumask.h>

#include <asm/netlogic/proc.h>
#include <asm/mach-netlogic/mmu.h>
#include <asm/time.h>

extern struct proc_dir_entry *nlm_root_proc;

#ifndef CONFIG_BTLB_LOADER
extern void nlm_update_tlb_stats(void *ignored);
#endif

struct nlm_cpu_stat {
	unsigned long long msgring_pic_int;
	unsigned long long msgring_int;
	unsigned long long msgring_cycles;
	unsigned long long fp_exp;
	unsigned long long rdhwr_exp;
};

struct nlm_cpu_stat nlm_cpu_stats[NR_CPUS];
__u64 nlm_cp2_exceptions[NR_CPUS];

extern unsigned long long nlm_common_tlb_stats[];


void nlm_cpu_stat_update_rdhwr(void)
{
	int cpu = 0;

	preempt_disable();

	cpu = hard_smp_processor_id();
	nlm_cpu_stats[cpu].rdhwr_exp++;

	preempt_enable();
}

void nlm_cpu_stat_update_fp(void)
{
	int cpu = 0;

	preempt_disable();

	cpu = hard_smp_processor_id();
	nlm_cpu_stats[cpu].fp_exp++;

	preempt_enable();
}

void nlm_cpu_stat_update_msgring_int(void)
{
	int cpu = 0;

	preempt_disable();

	cpu = hard_smp_processor_id();
	nlm_cpu_stats[cpu].msgring_int++;

	preempt_enable();
}

void nlm_cpu_stat_update_msgring_cycles(__u32 cycles)
{
	int cpu = 0;

	preempt_disable();

	cpu = hard_smp_processor_id();
	nlm_cpu_stats[cpu].msgring_cycles += cycles;

	preempt_enable();
}

void nlm_cpu_stat_update_msgring_pic_int(void)
{
	int cpu = 0;

	preempt_disable();

	cpu = hard_smp_processor_id();
	nlm_cpu_stats[cpu].msgring_pic_int++;

	preempt_enable();
}

static int nlm_cpu_proc_read(char *page, char **start, off_t off,
			     int count, int *eof, void *data)
{
	int i = 0;
	int len = 0;
	off_t begin = 0;

#ifndef CONFIG_BTLB_LOADER
	/* Update the TLB stats from other CPUs */
	on_each_cpu(nlm_update_tlb_stats, NULL, 1);
#endif

	len += sprintf(page + len, "CPU Frequency: %u HZ\n", (unsigned int)mips_hpt_frequency);
	if (!proc_pos_check(&begin, &len, off, count))
		goto out;

#ifdef CONFIG_32BIT
        len += sprintf(page + len, "32 Bit ");
#else
        len += sprintf(page + len, "64 Bit ");
#endif
        if (!proc_pos_check(&begin, &len, off, count))
               goto out;

#ifdef CONFIG_CPU_BIG_ENDIAN
        len += sprintf(page + len, "Big Endian ");
#else
        len += sprintf(page + len, "Little Endian ");
#endif
        if (!proc_pos_check(&begin, &len, off, count))
               goto out;

#ifdef CONFIG_MAPPED_KERNEL
        len += sprintf(page + len, "Mapped Kernel.\n");
#else
        len += sprintf(page + len, "Un-Mapped Kernel.\n");
#endif
        if (!proc_pos_check(&begin, &len, off, count))
               goto out;

	for(i=0;i<32;i++) {

		if (!nlm_cp2_exceptions[i]) continue;

			len += sprintf(page + len,
				       "cop2_exp: %03d 0x%016llx\n",
				       i, (unsigned long long)nlm_cp2_exceptions[i]);
			if (!proc_pos_check(&begin, &len, off, count))
				goto out;
	}

	for(i=0;i<32;i++) {

		if (!nlm_cpu_stats[i].msgring_pic_int && !nlm_cpu_stats[i].msgring_int)
			continue;

			len += sprintf(page + len,
				       "msgring: %03d 0x%016llx 0x%016llx 0x%016llx\n",
				       i, nlm_cpu_stats[i].msgring_pic_int,
				       nlm_cpu_stats[i].msgring_int,
				       nlm_cpu_stats[i].msgring_cycles);
			if (!proc_pos_check(&begin, &len, off, count))
				goto out;
	}

	for(i=0;i<32;i++) {

		if (!nlm_cpu_stats[i].fp_exp && !nlm_cpu_stats[i].rdhwr_exp)
			continue;

			len += sprintf(page + len,
				       "cpu_exp: %03d 0x%016llx 0x%016llx\n",
				       i, nlm_cpu_stats[i].fp_exp,
				       nlm_cpu_stats[i].rdhwr_exp);
			if (!proc_pos_check(&begin, &len, off, count))
				goto out;
	}

	for (i = 0; i < 32; i++) {

		if (!nlm_common_tlb_stats[i])
			continue;

		len += sprintf(page + len,
			       "tlb: %03d 0x%016llx \n",
			       i, nlm_common_tlb_stats[i]);
		if (!proc_pos_check(&begin, &len, off, count))
			goto out;
	}

	*eof = 1;

      out:
	*start = page + (off - begin);
	len -= (off - begin);
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
}

static int nlm_cpu_proc_init(void)
{
	struct proc_dir_entry *entry;
#ifdef CONFIG_NLM_XLR
	entry = create_proc_read_entry("xlr_cpu", 0 /* def mode */ ,
				       nlm_root_proc/* parent */ ,
				       nlm_cpu_proc_read
				       /* proc read function */ ,
				       0	/* no client data */
		);
#endif
#ifdef CONFIG_NLM_XLP
	entry = create_proc_read_entry("xlp_cpu", 0 /* def mode */ ,
				       nlm_root_proc/* parent */ ,
				       nlm_cpu_proc_read
				       /* proc read function */ ,
				       0	/* no client data */
		);
#endif
	if (!entry) {
		printk("[%s]: Unable to create proc read entry for cpu!\n",
		       __FUNCTION__);
	}

	return 0;
}

static void nlm_cpu_proc_exit(void)
{
}

module_init(nlm_cpu_proc_init);
module_exit(nlm_cpu_proc_exit);
