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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/mm.h>
#include <linux/percpu.h>
#include <linux/mmzone.h>
#include <linux/pfn.h>
#include <linux/highmem.h>
#include <linux/swap.h>

#include <asm/addrspace.h>
#include <asm/pgalloc.h>
#include <asm/sections.h>
#include <asm/bootinfo.h>
#include <asm/mach-netlogic/mmu.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/iomap.h>

extern unsigned long setup_zero_pages(void);

struct nlm_node_data *__node_data[NLM_MAX_CPU_NODE];
struct nlm_node_data __node_data_holder[NLM_MAX_CPU_NODE];
struct nlm_mem_info __node_mem_data[MAX_NUMNODES];
EXPORT_SYMBOL(__node_data);
EXPORT_SYMBOL(__node_mem_data);

struct xlp_dram_mapping {
	unsigned long low_pfn;
	unsigned long high_pfn;
	int node;
};
#define NLM_NODES_MAX_DRAM_REGION (NLM_MAX_DRAM_REGION * MAX_NUMNODES)
extern struct xlp_dram_mapping	dram_map[NLM_NODES_MAX_DRAM_REGION];
extern void nlm_get_dram_mapping(void);
extern int hcpu_to_lcpu[];

#ifdef CONFIG_MAPPED_KERNEL
#ifdef CONFIG_64BIT
#define _low_virt_to_phys(addr) ((unsigned long)(addr) & ~CKSSEG)
#else
#define _low_virt_to_phys(addr) ((unsigned long)(addr) & ~KSEG2)
#endif
#endif

static uint8_t _node_map_mem[MAX_NUMNODES][PAGE_SIZE];

extern cpumask_t fdt_cpumask;
static void nlm_init_bootmem_node (unsigned long mapstart, unsigned long min_pfn, unsigned long max_pfn)
{
	int i;

	for (i = 0; i < NLM_MAX_CPU_NODE; i++) {
		unsigned long map_pfn, start_pfn, end_pfn, bootmem_size;
		int j;

		if(!node_online(i))
			continue;

		start_pfn = NODE_MEM_DATA(i)->low_pfn;
		end_pfn   = NODE_MEM_DATA(i)->high_pfn;

		if (start_pfn && start_pfn < min_pfn)
			start_pfn = min_pfn;

		if (end_pfn > max_pfn)
			end_pfn = max_pfn;

		/* in general, never hit the condition */
		if (start_pfn && start_pfn >= end_pfn) {
			NODE_MEM_DATA(i)->map_pfn = 0; /* indicate a bad map_pfn */
			continue;
		}

		if (start_pfn > mapstart)
			map_pfn = start_pfn;
		else
			map_pfn = mapstart;

		if((start_pfn == 0) && (end_pfn == 0)) {
			map_pfn = 
			_low_virt_to_phys(&_node_map_mem[i][0]) >> PAGE_SHIFT;
			__node_data[i] = __va(map_pfn << PAGE_SHIFT);
		} else {
			__node_data[i] = __va(map_pfn << PAGE_SHIFT);
			map_pfn  += PFN_UP(sizeof(struct nlm_node_data));
		}

		NODE_DATA(i)->bdata = &bootmem_node_data[i];
		NODE_DATA(i)->node_start_pfn = start_pfn;
		NODE_DATA(i)->node_spanned_pages = end_pfn - start_pfn;

		/* Set this up with logical cpu map :
		   Assuming here that there are 32 cpus per node
		   */
		cpumask_clear(NODE_CPU_MASK(i));
		for (j = 0; j < 32; j++)
			if (cpumask_test_cpu((j + i * 32), &fdt_cpumask))
				cpumask_set_cpu(hcpu_to_lcpu[(j+i*32)], 
						NODE_CPU_MASK(i));

		NODE_MEM_DATA(i)->map_pfn = map_pfn;

		bootmem_size = init_bootmem_node(NODE_DATA(i), map_pfn, start_pfn, end_pfn);
		NODE_MEM_DATA(i)->bootmem_size = bootmem_size;
	}
}

static void nlm_reserve_bootmem(void)
{
	int i;
	unsigned long size;

	for (i = 0; i < NLM_MAX_CPU_NODE; i++) {

		if(!node_online(i))
			continue;
		if(NODE_DATA(i)->node_spanned_pages == 0)
			continue;
		size = NODE_MEM_DATA(i)->map_pfn - NODE_DATA(i)->node_start_pfn;
		size = PFN_PHYS(size);
		reserve_bootmem_node(NODE_DATA(i), 
			PFN_PHYS(NODE_DATA(i)->node_start_pfn),
			(NODE_MEM_DATA(i)->bootmem_size + size),
			 BOOTMEM_DEFAULT);
	}
}

static int dram_get_node_id (unsigned long start_pfn, unsigned long end_pfn)
{
	int i;

	for (i = 0; i < NLM_MAX_CPU_NODE; i++) {
		if (NODE_MEM_DATA(i)->low_pfn <= start_pfn && end_pfn <= NODE_MEM_DATA(i)->high_pfn)
			return i;
	}

	printk("Invalid start %lx end %lx\n", start_pfn, end_pfn);
	/* should not reach here */
	panic("dram_get_node_id: incorrect memory region\n");
}

/* This is used very early in boot process to build the node mem regions */
int nlm_get_node(unsigned long pfn)
{
	int i;
	for(i=0; i < NLM_NODES_MAX_DRAM_REGION; i++) {
		if((pfn >= dram_map[i].low_pfn) && 
				(pfn <= dram_map[i].high_pfn))
			return dram_map[i].node;
	}
	panic("Invalid PFN Passed: Cannot get node id\n");
}
EXPORT_SYMBOL(nlm_get_node);

/**
 * boot memory initialization for NUMA architecture.
 *
 * The implementation here copies the implementation in
 * arch/mips/kernel/setup, but added numa support.
 */
extern struct nlm_node_mem_info node_mem_info[];
extern void prom_meminit(void);
void __init nlm_numa_bootmem_init(unsigned long reserved_end)
{
	unsigned long mapstart = ~0UL;
	int i;
	int node, seg;

	/* Get the hardware dram region info */
	nlm_get_dram_mapping();

	/*
	 * max_low_pfn is not a number of pages. The number of pages
	 * of the system is given by 'max_low_pfn - min_low_pfn'.
	 */
	min_low_pfn = ~0UL;
	max_low_pfn = 0;

	/*
	 * Find the highest page frame number we have available.
	 */
	for (i = 0; i < boot_mem_map.nr_map; i++) {
		unsigned long start, end;

		if (boot_mem_map.map[i].type != BOOT_MEM_RAM)
			continue;

		start = PFN_UP(boot_mem_map.map[i].addr);
		end = PFN_DOWN(boot_mem_map.map[i].addr
				+ boot_mem_map.map[i].size);

		node = nlm_get_node(start);
		seg = node_mem_info[node].frags;
		node_mem_info[node].mem[seg].start_pfn = start;
		node_mem_info[node].mem[seg].end_pfn = end;
		node_mem_info[node].frags++;

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
		if (node == 0) {
			/* 40 bit physical address, min_start_pfn is the minimum above dma region */
			unsigned long max_dma_pfn = (MAX_DMA_ADDRESS & 0xffffffffffULL) >> PAGE_SHIFT;
			if (seg == 0) {
				if (start >= max_dma_pfn)
					node_mem_info[node].min_start_pfn = start;
			} else if (start > max_dma_pfn) {
				if (node_mem_info[node].min_start_pfn == 0)
					node_mem_info[node].min_start_pfn = start;
				else if (start < node_mem_info[node].min_start_pfn)
					node_mem_info[node].min_start_pfn = start;
			}
		} else {
			if (seg == 0)
				node_mem_info[node].min_start_pfn = start;
			else if (start < node_mem_info[node].min_start_pfn)
				node_mem_info[node].min_start_pfn = start;
		}
#endif

		if (end > max_low_pfn)
			max_low_pfn = end;
		if (start < min_low_pfn)
			min_low_pfn = start;
		if (end <= reserved_end)
			continue;
		if (start >= mapstart)
			continue;
		mapstart = max(reserved_end, start);
	}

	if (min_low_pfn >= max_low_pfn)
		panic("Incorrect memory mapping !!!");
	if (min_low_pfn > ARCH_PFN_OFFSET) {
		pr_info("Wasting %lu bytes for tracking %lu unused pages\n",
			(min_low_pfn - ARCH_PFN_OFFSET) * sizeof(struct page),
			min_low_pfn - ARCH_PFN_OFFSET);
	} else if (min_low_pfn < ARCH_PFN_OFFSET) {
		pr_info("%lu free pages won't be used\n",
			ARCH_PFN_OFFSET - min_low_pfn);
	}
	min_low_pfn = ARCH_PFN_OFFSET;

	/*
	 * Determine low and high memory ranges
	 */
	max_pfn = max_low_pfn;
	if (max_low_pfn > PFN_DOWN(HIGHMEM_START)) {
#ifdef CONFIG_HIGHMEM
		highstart_pfn = PFN_DOWN(HIGHMEM_START);
		highend_pfn = max_low_pfn;
#endif
		max_low_pfn = PFN_DOWN(HIGHMEM_START);
	}

	max_low_pfn = recalculate_max_low_pfn(max_low_pfn);

#ifdef DEBUG_MAPPED_KERNEL
	printk("max_low_pfn = 0x%lx\n", max_low_pfn);
#endif
	setup_mapped_kernel_tlbs(FALSE, TRUE);
	prom_meminit();
	/*
	 * Initialize the boot-time allocator with low memory only.
	 */
	nlm_init_bootmem_node(mapstart, min_low_pfn, max_low_pfn);

	for (i = 0; i < boot_mem_map.nr_map; i++) {
		unsigned long start, end;

		start = PFN_UP(boot_mem_map.map[i].addr);
		end = PFN_DOWN(boot_mem_map.map[i].addr
				+ boot_mem_map.map[i].size);

		if (start <= min_low_pfn)
			start = min_low_pfn;
		if (start >= end)
			continue;

#ifndef CONFIG_HIGHMEM
		if (end > max_low_pfn)
			end = max_low_pfn;

		/*
		 * ... finally, is the area going away?
		 */
		if (end <= start)
			continue;
#endif
		add_active_range(dram_get_node_id(start, end), start, end);
	}

	/*
	 * Register fully available low RAM pages with the bootmem allocator.
	 */
	for (i = 0; i < boot_mem_map.nr_map; i++) {
		unsigned long start, end, size;

		/*
		 * Reserve usable memory.
		 */
		if (boot_mem_map.map[i].type != BOOT_MEM_RAM)
			continue;

		start = PFN_UP(boot_mem_map.map[i].addr);
		end   = PFN_DOWN(boot_mem_map.map[i].addr
				    + boot_mem_map.map[i].size);
		/*
		 * We are rounding up the start address of usable memory
		 * and at the end of the usable range downwards.
		 */
		if (start >= max_low_pfn)
			continue;
		if (start < reserved_end)
			start = reserved_end;
		if (end > max_low_pfn)
			end = max_low_pfn;

		/*
		 * ... finally, is the area going away?
		 */
		if (end <= start)
			continue;
		size = end - start;

		/* Register lowmem ranges */
		free_bootmem(PFN_PHYS(start), size << PAGE_SHIFT);
		memory_present(dram_get_node_id(start, end), start, end);
	}

	/*
	 * Reserve the bootmap memory.
	 */
	nlm_reserve_bootmem();
}

static int __init page_is_ram(unsigned long pagenr)
{
	int i;

	for (i = 0; i < boot_mem_map.nr_map; i++) {
		unsigned long start, end;

		if (boot_mem_map.map[i].type != BOOT_MEM_RAM)
			continue;

		start = PFN_UP(boot_mem_map.map[i].addr);
		end   = PFN_DOWN(boot_mem_map.map[i].addr
				    + boot_mem_map.map[i].size);
		if(pagenr >= start && pagenr <= end)
			return 1;
	}
	return 0;
}

void __init paging_init(void)
{
	unsigned long zones_size[MAX_NR_ZONES] = {0, };
	unsigned node;

	pagetable_init();
#ifdef CONFIG_NLM_16G_MEM_SUPPORT
	setup_mapped_kernel_pgtable();
#endif

#ifdef CONFIG_ZONE_DMA
	zones_size[ZONE_DMA] = MAX_DMA_PFN;
#endif


	for_each_online_node(node) {
		unsigned long start_pfn, end_pfn;

		get_pfn_range_for_nid(node, &start_pfn, &end_pfn);

		if (end_pfn > max_low_pfn)
			max_low_pfn = end_pfn;
	}
	zones_size[ZONE_NORMAL] = max_low_pfn;
	free_area_init_nodes(zones_size);
}

void __init mem_init(void)
{
	unsigned long codesize, reservedpages, datasize, initsize, tmp, ram;
	unsigned int  node;

	high_memory = (void *) __va(max_low_pfn<< PAGE_SHIFT);
	for_each_online_node(node) {
		/*
		 * This will free up the bootmem, ie, slot 0 memory.
		 */
		//if((NODE_MEM_DATA(node)->low_pfn !=0) && 
				//(NODE_MEM_DATA(node)->high_pfn !=0))
			totalram_pages += 
				free_all_bootmem_node(NODE_DATA(node));
	}

	totalram_pages -= setup_zero_pages();   /* This comes from node 0 */

	reservedpages = ram = 0;
	for (tmp = 0; tmp < max_low_pfn; tmp++)
		if (page_is_ram(tmp)) {
			ram++;
			if (PageReserved(pfn_to_page(tmp)))
				reservedpages++;
		}
	num_physpages = ram;

	codesize =  (unsigned long) &_etext - (unsigned long) &_text;
	datasize =  (unsigned long) &_edata - (unsigned long) &_etext;
	initsize =  (unsigned long) &__init_end - (unsigned long) &__init_begin;

	printk(KERN_INFO "Memory: %luk/%luk available (%ldk kernel code, "
		"%ldk reserved, %ldk data, %ldk init, %ldk highmem)\n",
		nr_free_pages() << (PAGE_SHIFT - 10),
		ram << (PAGE_SHIFT - 10),
		codesize >> 10,
		reservedpages << (PAGE_SHIFT - 10),
		datasize >> 10,
		initsize >> 10,
		(unsigned long) (totalhigh_pages << (PAGE_SHIFT - 10)));
}
