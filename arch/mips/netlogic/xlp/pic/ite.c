/*-
 * Copyright (c) 2003-2015 Broadcom Corporation
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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/cpumask.h>
#include <linux/nodemask.h>
#include <linux/netdevice.h>

#include <asm/netlogic/iomap.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/hal/nlm_hal_fmn.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/cpumask.h>

/*
 * xlp_ites[node][0-3] = {0x1, 0xffffffff, 0x0000ffff, 0xffff0000};//local only
 */
static struct cpumask xlp_ites[NLM_MAX_NODES][XLP_ITE_ENTRIES];
static DEFINE_SPINLOCK(xlp_ites_lock);

/* Modifications below to read actual programmed values instead of just dumping table */
void dump_all_ites(void)
{
	u8 node, i;
	u32 ite3, ite2, ite1, ite0;
	u64 tmp, reg;
	char buf[140];

	for_each_online_node(node) {
		for (i = 0; i < XLP_ITE_ENTRIES; i++) {
			cpumask_scnprintf(buf, 140, &xlp_ites[node][i]);
			printk(KERN_DEBUG "PIC: Node %d supported CPUMASK (%d) -> %s\n", node, i, buf);
			reg = XLP_PIC_INT_THREADEN23(i);
			tmp = nlh_pic_r64r(node, reg);
			ite3 = tmp >> 32;
			ite2 = tmp & 0xffffffffULL;
			reg = XLP_PIC_INT_THREADEN01(i);
			tmp = nlh_pic_r64r(node, reg);
			ite1 = tmp >> 32;
			ite0 = tmp & 0xffffffffULL;
			printk(KERN_DEBUG "               Programmed ITE (%d) -> %08x,%08x,%08x,%08x\n",
					i, ite3, ite2, ite1, ite0);
		}
	}

	return;
}

/* This function sets the given ITE's cpu bit on node node.
 *
 * @node	: node on which ITE is to be set
 * @cpu		: target cpu id
 * @ite		: ITE index
 * @bitval	: 0 to clear, 1 to set
 */
void xlp_ite_cpu_op(u8 node, u8 cpu, u8 ite, u8 bitval)
{
	unsigned long flags;
	u64 val, reg;
	u8 bit;

	/* No param checking, must be checked before calling.
	 * Target cpu id decices whether to use THREADEN01 or THREADEN23
	 * i.e., if target cpu < 64, use THREADEN01 as base else THREADEN23 */
	reg = (cpu < 64) ? XLP_PIC_INT_THREADEN01(ite) :
					XLP_PIC_INT_THREADEN23(ite);
	bit = cpu % 64;
	spin_lock_irqsave(&xlp_ites_lock, flags);
	val = nlh_pic_r64r(node, reg);
	val = (bitval == 0) ?  (val & ~(1ULL << bit)) : (val | (1ULL << bit));
	nlh_pic_w64r(node, reg, val);
	spin_unlock_irqrestore(&xlp_ites_lock, flags);
}

extern struct cpumask phys_cpu_present_map;

/* This function would program ITE values on node given by the cpumask
 * @cpumask	: cpumask to program on ITE
 * @node	: node on which ITE should be programmed
 * @ite		: ITE to program
 * @scope	: program ITE only on the given node (0) or all nodes (1)
 */
void xlp_cpumask_to_node_ite(const struct cpumask *m, u8 node, u8 ite, u8 scope)
{
	__label__ prog_all;
	struct cpumask t;
	int cpu = (node * NLM_MAX_CPU_PER_NODE), last;

	if (scope != 0) goto prog_all;

	/* When the scope is 0, program node ITEs with target as
	 * local cpus only */
	last = cpu + NLM_MAX_CPU_PER_NODE - 1;
	if (last >= NR_CPUS) return;
	cpumask_and(&t, m, &phys_cpu_present_map);
	for (; cpu <= last; cpu++) {
		cpumask_test_cpu(cpu, &t) ? xlp_ite_cpu_set(node, cpu, ite) : xlp_ite_cpu_clear(node, cpu, ite);
	}
	return;
prog_all:
	/* Here we program the specified ITE in all nodes with the cpumask
	 * passed. */
	/* TBD TODO */
	return;
}

/* Once all CPUs are up, walk through the node mask and program all
 * ITEs in the PICs */
void xlp_prog_all_node_ites(void)
{
	u8 i, node;

	for_each_online_node(node) {
		for (i = 0; i < XLP_ITE_ENTRIES; i++) {
			/* 4 is the ITE that redirects int.s to all cpus */
			xlp_cpumask_to_node_ite(&xlp_ites[node][i], node, i, (i == 4));
		}
	}
	dump_all_ites();
}

/* Checks if a mask spans multiple nodes
 *
 * @mask	: cpumask to check for multiple node span
 */
int xlp_span_multiple_nodes(const struct cpumask *mask)
{
	int l, f;
	f = cpumask_first(mask);
	l = find_last_bit(cpumask_bits(mask), NR_CPUS);
	if ((f/NLM_MAX_CPU_PER_NODE) != (l/NLM_MAX_CPU_PER_NODE)) {
		printk(KERN_DEBUG "PIC: Mask spans from cpu %#x to %#x. Spans across nodes are not supported\n", f, l);
		return -EINVAL;
	}
	return 0;
}

/*
 * In XLP cpu mask for setting affinity of an interrupt cannot span multiple
 * nodes. Although this is not a h/w restriction, the effort to implement
 * this feature does not justify the potential benefit; not only that handling
 * non local interrupts are slightly slower, it could be expensive in terms of
 * memory access and other resource utilization
 *
 * @node	: node to which mask `mask` to be restricted
 * @src		: mask to restrict
 * @dst		: restricted mask (result)
 */
void constrict_mask_to_node(u8 node, struct cpumask *dst, const struct cpumask *src)
{
//	char buf[140];
	int i;

	cpumask_clear(dst);
	for (i = NLM_MAX_CPU_PER_NODE * node;
			i < (NLM_MAX_CPU_PER_NODE *(node + 1)); i++) {
		cpumask_set_cpu(i, dst);
	}
	cpumask_and(dst, dst, &phys_cpu_present_map);
	cpumask_and(dst, dst, src);
	return;
}

/*
 * This function returns closest match cpumask among the supported bitmasks
 * in XLP
 * Logic is moot, need to improve it later.
 *
 * @m	: user supplied cpumask
 */
static int xlp_closest_match_cpumask(u8 node, const struct cpumask *m)
{
	int i;
	char buf[40];
	struct cpumask t, a;
	/* m will be a logical cpu mask.  If all threads are enabled, this will
	   match the physical cpu mask.  If not, this won't match and this function
	   won't work, however the fall-back is to route interrupts to all CPUs so
	   the system will still work, just not necessarily with the desired affinity.
	   So we have to convert the logical cpu mask to a physical mask first. */
	cpumask_clear(&a);
	for_each_cpu(i, m)	// Iterate through logical map, build a corresponding physical map
		cpumask_set_cpu(cpu_logical_map(i), &a);
	cpumask_and(&a, &a, &phys_cpu_present_map);
	constrict_mask_to_node(node, &t, &a);
	cpumask_clear(&a);
	for (i = 0; i < XLP_ITE_ENTRIES; i++) {
		cpumask_and(&a, &xlp_ites[node][i], &phys_cpu_present_map);
		if (cpumask_equal(&t, &a)) {
			cpumask_scnprintf(buf, 40, m);
//			printk(KERN_DEBUG "PIC: Matched ITE #%d for logical cpumask %s\n", i, buf);
			return i;
		}
	}
	cpumask_scnprintf(buf, 40, m);
	printk(KERN_WARNING "PIC: Could not find ITE match for logical cpumask %s\n", buf);
	cpumask_scnprintf(buf, 40, &t);
	printk(KERN_WARNING "                      Calculated physical cpumask %s\n", buf);
	printk(KERN_WARNING "     Using ITE #1 (default all online CPUs)\n");
	return 1; /* if no match, point to all local cpus */
}

/*
 * This function programs the ITE on the IRT entry on all online nodes
 * @m	: CPU mask resulting from xlp_closest_match_cpumask() call
 */
void xlp_set_cpumask_on_node(u8 node, const struct cpumask *m, int irq)
{
	u8 ite;
	u64 val;
	int irt = xlp_irq_to_irt(irq);
	int rvec = xlp_rvec_from_irq(irq);

	//dump_all_ites();
	ite = xlp_closest_match_cpumask(node, m);
	printk(KERN_DEBUG "PIC: Using ITE %d for IRQ %d (IRT %d)\n", ite, irq, irt);
	/* xlp_pic_init() has set default values. Override them */
	val = XLP_IRTENT_ENABLE | XLP_IRTENT_SCH_LCL |
			XLP_IRTENT_RVEC(rvec) | XLP_IRTENT_DB(ite);
	nlh_pic_w64r(node, XLP_PIC_IRT_ENTRY(irt), val);
	//fdebug("Setting up mask: Wrote %#llx to (%d)%#llx\n", val, irt, XLP_PIC_IRT_ENTRY(irt));
	return;
}

/* helper function to create cpumask from unsigned long
 * Easiest way is to create it directly using bitmap_copy.
 * For some reason, this was not successful.
 *
 * @m	: cpumask pointer to populate. Lower 32 bits must be 0
 * @u	: u32 variable pointer with bitmask
 */
void u32_to_cpumask(struct cpumask *m, u32 bm)
{
	u8 bit = 0;

	/* should not clear the mask passed */
	for ( ; bit < sizeof(u32) * BITS_PER_BYTE; bit++) {
		if (bm & (1 << bit)) {
			cpumask_set_cpu(bit, m);
		} else {
			cpumask_clear_cpu(bit, m);
		}
	}
}

/*
 * Initializes PIC ITE entries PRM 9.5.6.26
 * XLP restricts CPU affinity to 8 groups. Though configurable, they are
 * programmed to have the following patterns.
 * 0 =>	Only 0th cpu on the node
 * 1 => All local threads in node; mask = (0xffffffff) on node
 * 2 => cpu0-15 on node; mask = 0x0000ffff & online_cpu_mask on nodes
 * 3 => cpu15-31 on node; mask = 0xffff0000 & online_cpu_mask on node
 * 4 => All cpus on all nodes; i.e., 
 * mask = (0xffffffff_ffffffff_ffffffff_ffffffff & physical online cpu map)
 * These are programmer defined groups and can be changed as warranted.
 * Added 5 => CPUs 0-11
 * Added 6 => CPUs 0-7
 * Added 7 => CPUs 0-3
 * Actual programmed value will take into consideration cpu_online_mask.
 * 
 * There is a major issue that needs addressing when run in multi node mode
 * Number of nodes must be determined and programmed correctly, if a bit in ITE
 * is programmed without physical thread being present, when interrupt is
 * dispatched to that CPU under global scheme, system would hang. Thus this
 * scenario should be avoided. That is why phys_cpu_present_map is used
 *
 * This function simply initializes the xlp_ites entries with proposed
 * CPUmasks.  */
static void xlp_ites_init(void)
{
	u64 bm = 0x1;
	u8 node;
	struct cpumask m;

	cpumask_clear(&m);
	for_each_online_node(node) {
	/* Simply set the static pattern in all */
	bm = 1;
	u32_to_cpumask(&xlp_ites[node][0], bm);
	cpumask_shift_left(&xlp_ites[node][0], &xlp_ites[node][0], NLM_MAX_CPU_PER_NODE * node); /* directs only to cpu0 of node `node` */

	bm = 0xffffffff;
	u32_to_cpumask(&xlp_ites[node][1], bm);
	cpumask_shift_left(&xlp_ites[node][1], &xlp_ites[node][1], NLM_MAX_CPU_PER_NODE * node); /* directs to all cpus of node `node` */
	cpumask_or(&m, &m, &xlp_ites[node][1]);

	bm = 0x0000ffff;
	u32_to_cpumask(&xlp_ites[node][2], bm);
	cpumask_shift_left(&xlp_ites[node][2], &xlp_ites[node][2], NLM_MAX_CPU_PER_NODE * node); /* directs to specified cpus of node `node` */

	bm = 0xffff0000;
	u32_to_cpumask(&xlp_ites[node][3], bm);
	cpumask_shift_left(&xlp_ites[node][3], &xlp_ites[node][3], NLM_MAX_CPU_PER_NODE * node); /* directs to specified cpus of node `node` */

	bm = 0x000000ff;
	u32_to_cpumask(&xlp_ites[node][5], bm);
	cpumask_shift_left(&xlp_ites[node][5], &xlp_ites[node][5], NLM_MAX_CPU_PER_NODE * node); /* directs to specified cpus of node `node` */

	bm = 0x000000f0;
	u32_to_cpumask(&xlp_ites[node][6], bm);
	cpumask_shift_left(&xlp_ites[node][6], &xlp_ites[node][6], NLM_MAX_CPU_PER_NODE * node); /* directs to specified cpus of node `node` */

	bm = 0x0000000f;
	u32_to_cpumask(&xlp_ites[node][7], bm);
	cpumask_shift_left(&xlp_ites[node][7], &xlp_ites[node][7], NLM_MAX_CPU_PER_NODE * node); /* directs to specified cpus of node `node` */

	}
	for_each_online_node(node) {
		cpumask_copy(&xlp_ites[node][4], &m);
	}
//	dump_all_ites();
}

/* Initializes the PIC
 * Mainly sets up the IRT entries default values
 * called from on_chip.c:pic_init()
 * */
void xlp_pic_init(u8 node)
{
	u8 i;
	u64 val;

	if (node == 0) {
		xlp_ites_init();
		/* Program ITEs to direct interrupts only to cpu 0
		 * This is mandatory for PIC timer to come up. */
		xlp_cpumask_to_node_ite(&xlp_ites[node][0], node, 0, 0);
	}
	/* We set the following in IRT entry
	 * 28 : clear to indicate global delivery
	 * 19 : set to indicate DB
	 * 16-18 : 0
	 * 0-15 : 0 => cpu0 */
	for (i = 0; i < XLP_IRT_NUM; i++) {
		val = XLP_IRTENT_SCH_LCL |  XLP_IRTENT_DT |
			XLP_IRTENT_DB(node<<1) | XLP_IRTENT_DTE(0);
		nlh_pic_w64r(node, XLP_PIC_IRT_ENTRY(i), val);
	}
}
