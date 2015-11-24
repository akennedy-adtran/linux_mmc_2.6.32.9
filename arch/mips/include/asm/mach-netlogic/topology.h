/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#ifndef _ASM_MACH_TOPOLOGY_H
#define _ASM_MACH_TOPOLOGY_H

#ifdef CONFIG_NUMA

#include <asm/mmzone.h>

/* FIXME_XLP: only works for all cpus up */
#define cpu_to_node(cpu)	(__cpu_logical_map[cpu] >> 5)
#define hardcpu_to_node(cpu)	(cpu >> 5)

#define cpumask_of_node(node)	(NODE_CPU_MASK(node))

#define parent_node(node)	(node)

#ifdef CONFIG_PCI
#define cpumask_of_pcibus(bus)  (cpu_online_mask)
/* FIXME_XLP: to be implemented */
#define pcibus_to_node(bus)     (0)
#endif

/* sched_domains SD_NODE_INIT for multi-node XLP machines */
/* FIXME_XLP: the number needs to be fine tuned later */
#define SD_NODE_INIT (struct sched_domain) {		\
	.parent			= NULL,			\
	.child			= NULL,			\
	.groups			= NULL,			\
	.min_interval		= 8,			\
	.max_interval		= 32,			\
	.busy_factor		= 32,			\
	.imbalance_pct		= 125,			\
	.cache_nice_tries	= 1,			\
	.flags			= SD_LOAD_BALANCE |	\
				  SD_BALANCE_EXEC,	\
	.last_balance		= jiffies,		\
	.balance_interval	= 1,			\
	.nr_balance_failed	= 0,			\
}

#endif

#include <asm-generic/topology.h>

#endif /* _ASM_MACH_TOPOLOGY_H */
