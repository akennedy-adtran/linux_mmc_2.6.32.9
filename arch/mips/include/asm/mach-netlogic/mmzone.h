/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#ifndef _ASM_MACH_MMZONE_H
#define _ASM_MACH_MMZONE_H

#if defined(CONFIG_NEED_MULTIPLE_NODES) && defined(CONFIG_NUMA)

#include <linux/cpumask.h>

#ifndef NLM_MAX_CPU_NODE
#define NLM_MAX_CPU_NODE	4
#endif

struct nlm_mem_info {
	unsigned long low_pfn;
	unsigned long high_pfn;
	unsigned long map_pfn;
	unsigned long bootmem_size;
};
struct nlm_node_mem_frag {
	unsigned long start_pfn;
	unsigned long end_pfn;
};

#define NLM_MAX_MEM_FRAGS_PER_NODE 16
struct nlm_node_mem_info {
	struct nlm_node_mem_frag mem[NLM_MAX_MEM_FRAGS_PER_NODE];
	int frags;
#ifdef CONFIG_NLM_16G_MEM_SUPPORT
	/* node 0: minimum start pfn about DMA region; other nodes: minimum start pfn */
	int min_start_pfn;
#endif
};

struct nlm_cpu_info {
	struct cpumask mask;
};

extern struct nlm_mem_info __node_mem_data[];

struct nlm_node_data {
	struct pglist_data pg_data;
	struct nlm_cpu_info cpu;
};

extern struct nlm_node_data *__node_data[];

#define NODE_DATA(nid)		(&__node_data[nid]->pg_data)
#define NODE_CPU_MASK(nid)	(&__node_data[nid]->cpu.mask)
#define NODE_MEM_DATA(nid)	(&__node_mem_data[nid])

extern int nlm_get_node(unsigned long pfn);

static inline unsigned int pa_to_nid(unsigned long addr)
{
	unsigned int  i;
	unsigned long pfn = addr >> PAGE_SHIFT;

	/* TODO: Implement this using NODE_DATA */
	for (i = 0; i < NLM_MAX_CPU_NODE; i++) {

		if ((!node_online(i)) || ((NODE_MEM_DATA(i)->low_pfn == 0) && (NODE_MEM_DATA(i)->high_pfn == 0)))
			continue;

		if (pfn >= NODE_MEM_DATA(i)->low_pfn && pfn <= NODE_MEM_DATA(i)->high_pfn)
			return i;
	}

	/* special case: get node ID from dram_map info */
	if (addr == 0)
	{
		i = nlm_get_node(pfn);
		// printk("** special case: returning node %d.\n", i);
		return i;
	}

	/* it should not really reach here */
	printk("Invalid address is %lx\n", addr);
	panic("Invalid address in pa_to_nid\n");
	return 0;
}

#endif

#endif /* _ASM_MACH_MMZONE_H */
