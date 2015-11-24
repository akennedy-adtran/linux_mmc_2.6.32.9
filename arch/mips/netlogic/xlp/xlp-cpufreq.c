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
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <asm/netlogic/xlp8xx/cpu_control_macros.h>
#include <asm/netlogic/cpumask.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/xlp_irq.h>

static int get_cpu_freq_masked(int cpu_num, int mask)
{
        uint64_t freq = nlm_hal_cpu_freq();
        const uint64_t khz = 1000;

        do_div(freq, khz);
        return (freq & (mask));
}
#define XLP_FREQ_MASK  (0xfffffff0)
#define XLP_CPU0       0

static spinlock_t freq_lock = SPIN_LOCK_UNLOCKED;
static struct cpumask xlp_affected_cpus[NR_CPUS];
/* The following are the possible devisors
 * The first value need not be 2 always. Read Reset value from
 * CORE_DFS_DIV_VALUE PRM 34.9.7.20
 *
 * This array contains frequency multipliers:
 * Say, initial frequency is 1.2GHz and core_dfs reads 1,
 * that means 1.2 GHz is the max (== 1.2 / ((core_dfs + 1)/2)).
 * Then a range of 1.2GHz through 1.2/(15+1)/2 == 150MHz is available for
 * scaling.
 * Scaling range is limited to core_dfs@startup through xlp_divs[XLP_DIVS]
 */
#define XLP_DIVS 11
static int xlp_divs[XLP_DIVS] = {1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 15};
static struct cpufreq_frequency_table xlp_freq_table[XLP_DIVS + 1];

static int xlp_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, xlp_freq_table);
}

static u64 xlp_syscfg_base[NLM_MAX_CPU_NODE] = { XLP_BDF_BASE(0,6,5),
	XLP_BDF_BASE(0,14,5), XLP_BDF_BASE(0,22,5), XLP_BDF_BASE(0,30,5) };
/*
 * @cpu_num : # of the cpu
 * @dec	: whether to decrement frequency
 * NOTE: If frequency to be decremented, multiplier should be incremented
 */
u32 change_cpu_freq(int cpu, int dec)
{
	u32 val;
	u8 node = cpu / NLM_MAX_CPU_PER_NODE;
	cpu %= NLM_MAX_CPU_PER_NODE;

	/* INC freq --> DEC multiplier */
	u32 reg = (dec == 1) ? 0x52 : 0x53;	/* Increment/Decrement reg */
	val = (0x1 << (cpu >> 2));
	nlm_hal_write_32bit_reg(xlp_syscfg_base[node], reg, val);
	return 0;
}

EXPORT_SYMBOL(change_cpu_freq);

static void setup_affected_cpus(struct cpumask map)
{
	/* Find out the number of cores. Read SYS, reg 0x42 bits 0-7,
	 * PRM 34.9.7.3 */
	u32 idx;
	struct cpumask thr;
	unsigned long flags;

	cpumask_clear(&thr);
	cpumask_set_cpu(0, &thr);
	cpumask_set_cpu(1, &thr);
	cpumask_set_cpu(2, &thr);
	cpumask_set_cpu(3, &thr);

	spin_lock_irqsave(&freq_lock, flags);
	for (idx = 0; idx < NR_CPUS; idx++) {
		cpumask_and(&xlp_affected_cpus[idx], &thr, &map);
		if (((idx + 1) % XLP_THREADS_PER_CORE) == 0) {
			cpumask_shift_left(&thr, &thr, XLP_THREADS_PER_CORE);
		}
	}
	spin_unlock_irqrestore(&freq_lock, flags);
	return;
}

__init static int build_cpufreq_table(struct cpufreq_frequency_table *t)
{
	unsigned long freq;
	u32 divfs = get_core_dfs(XLP_CPU0);
	int i, div_start = -1, j;

	freq = get_cpu_freq_masked(XLP_CPU0, XLP_FREQ_MASK);	// In KHz
	divfs = (divfs >> ((XLP_CPU0 & 0x1f) >> 2)) & 0xf;
	for (i = 0; i < XLP_DIVS; i++) {
		if (xlp_divs[i] == divfs){
			div_start = i;
			break;
		}
	}
	if (div_start == -1) {
		return -EFAULT;
	}
	for (i = 0, j = div_start; j < XLP_DIVS; i++, j++) {
		xlp_freq_table[i].index = j;
		xlp_freq_table[i].frequency = (int)(freq * 2 / (xlp_divs[j] + 1));
	}
	xlp_freq_table[i].frequency = CPUFREQ_TABLE_END;
	xlp_freq_table[i].index = 0;
	return 0;
}

static unsigned int xlp_cpufreq_get_speed(unsigned int cpu)
{
	return get_cpu_freq_masked(cpu, XLP_FREQ_MASK);
}

static int xlp_freq_set(struct cpufreq_frequency_table *from,
		struct cpufreq_frequency_table *to, int cpu, int dec)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&freq_lock, flags);
	for (i = from->index; i != to->index; ){
		fdebug("i = %d\n", i);
		change_cpu_freq(cpu, dec);
		/* To decrement frequency, INC index */
		i = (dec == 1) ? (i + 1) : (i - 1);
	}
	spin_unlock_irqrestore(&freq_lock, flags);
	return 0;
}
static int xlp_cpufreq_set_target(struct cpufreq_policy *policy,
				      unsigned int target_freq,
				      unsigned int relation)
{
	int ret;
	unsigned int i = 0, tmp = 0;
	struct cpufreq_freqs freq;
	u32 cur_freq;
	u32 start, end;

	if (!policy) {
		return -1;
	}
	if (target_freq < policy->min) {
		target_freq = policy->min;
	}
	if (target_freq > policy->max) {
		target_freq = policy->max;
	}

	ret = cpufreq_frequency_table_target(policy, xlp_freq_table,
						target_freq, relation, &i);
	if (ret < 0) {
		printk(KERN_WARNING "Cannot find a target frequency\n");
		return ret;
	}
	start = policy->cpu & (~(NLM_MAX_THREADS_PER_CPU - 1));
	end = start + NLM_MAX_THREADS_PER_CPU;
	cur_freq = get_cpu_freq_masked(start, XLP_FREQ_MASK);
	ret = cpufreq_frequency_table_target(policy, xlp_freq_table,
					     cur_freq, relation, &tmp);
	if (ret < 0) {
		printk(KERN_WARNING "Cannot find a source frequency\n");
		return ret;
	}
	freq.old = xlp_freq_table[tmp].frequency;
	freq.new = xlp_freq_table[i].frequency;
	freq.cpu = policy->cpu;
	freq.flags = 0;
	if (freq.old == freq.new) {
		return 0;
	}
	cpufreq_notify_transition(&freq, CPUFREQ_PRECHANGE);
	ret = xlp_freq_set(&xlp_freq_table[tmp], &xlp_freq_table[i],
			policy->cpu, (freq.old > freq.new));
	cpufreq_notify_transition(&freq, CPUFREQ_POSTCHANGE);
	if (ret != 0) {
		printk(KERN_WARNING "Setting cpufrequency failed\n");
		return -EFAULT;
	}
	return 0;
}

__init static int xlp_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;
	unsigned long flags;

	policy->cur = get_cpu_freq_masked(XLP_CPU0, XLP_FREQ_MASK);
	if (xlp_freq_table == NULL) {
		pr_err("cpufreq: No frequency information for this CPU\n");
		return -ENODEV;
	}
	/* Pick a conservative guess in ns: */
	policy->cpuinfo.transition_latency = 2 * 1000 * 1000;
	spin_lock_irqsave(&freq_lock, flags);
	cpumask_copy(policy->cpus, &xlp_affected_cpus[policy->cpu]);
	spin_unlock_irqrestore(&freq_lock, flags);
	ret = cpufreq_frequency_table_cpuinfo(policy, xlp_freq_table);
	if (ret != 0) {
		pr_err("cpufreq: Failed to configure frequency table: %d\n", ret);
	}
	return ret;
}

__exit static int xlp_cpufreq_driver_exit(struct cpufreq_policy *policy)
{
	return 0;
}

static struct cpufreq_driver xlp_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags          = 0,
	.verify		= xlp_cpufreq_verify_speed,
	.target		= xlp_cpufreq_set_target,
	.get		= xlp_cpufreq_get_speed,
	.init		= xlp_cpufreq_driver_init,
	.exit		= xlp_cpufreq_driver_exit,
	.name		= "xlp-cpufreq",
};

static int __init xlp_cpufreq_init(void)
{
	setup_affected_cpus(cpu_present_map);
	if (build_cpufreq_table(xlp_freq_table) < 0) {
		return -EFAULT;
	}
	return cpufreq_register_driver(&xlp_cpufreq_driver);
}
module_init(xlp_cpufreq_init);
