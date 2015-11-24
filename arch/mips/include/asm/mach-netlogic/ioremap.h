/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 *	include/asm-mips/mach-generic/ioremap.h
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */
#ifndef __ASM_MACH_GENERIC_IOREMAP_H
#define __ASM_MACH_GENERIC_IOREMAP_H

#include <linux/types.h>

#define ARCH_HAS_VALID_PHYS_ADDR_RANGE

#define NETLOGIC_UNCACHED_START 0x10000000UL
#define NETLOGIC_UNCACHED_END 0x20000000UL

static inline int valid_phys_addr_range(unsigned long addr, size_t count)
{
	/* for now return valid */
	return 1;
}

static inline int valid_mmap_phys_addr_range(unsigned long pfn, size_t size) 
{
	/* for now return valid */
	return 1;
}

//extern inline int valid_phys_addr_range(unsigned long addr, size_t count);
//extern inline int valid_mmap_phys_addr_range(unsigned long pfn, size_t size);
/*
 * Allow physical addresses to be fixed up to help peripherals located
 * outside the low 32-bit range -- generic pass-through version.
 */
static inline phys_t fixup_bigphys_addr(phys_t phys_addr, phys_t size)
{
	return phys_addr;
}

static inline void __iomem *plat_ioremap(phys_t offset, unsigned long size,
	unsigned long flags)
{
	return NULL;
}

static inline int plat_iounmap(const volatile void __iomem *addr)
{
	return 0;
}

#endif /* __ASM_MACH_GENERIC_IOREMAP_H */
