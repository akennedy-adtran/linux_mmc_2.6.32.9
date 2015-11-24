
/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#include <linux/kernel.h>
#include <asm/bootinfo.h>
#include <asm/netlogic/bootinfo.h>

void copy_mem_map(struct boot_mem_map *dst, struct nlm_boot_mem_map *src)
{
	int i;

	dst->nr_map = src->nr_map;
	for(i=0; i < dst->nr_map; i++) {
		dst->map[i].addr = (phys_t)src->map[i].addr;
		dst->map[i].size = (phys_t)src->map[i].size;
		dst->map[i].type = (long)src->map[i].type;
	}
}
