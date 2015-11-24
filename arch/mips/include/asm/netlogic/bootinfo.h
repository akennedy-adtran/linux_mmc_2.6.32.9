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

#ifndef _ASM_NETLOGIC_BOOTINFO_H
#define _ASM_NETLOGIC_BOOTINFO_H

#define LOADER_UBOOT   1
#define LOADER_OTHER   2

/* This is what netlboot passes and linux boot_mem_map is subtly different */
struct nlm_boot_mem_map {
	int nr_map;
	struct nlm_boot_mem_map_entry {
		uint64_t addr;  /* start of memory segment */
		uint64_t size;  /* size of memory segment */
		uint32_t type;          /* type of memory segment */
	} map[BOOT_MEM_MAP_MAX];
};

#define MAX_EXCLUDE 16
struct boot_mem_map_exclude_region {
	uint64_t start;
	uint64_t end;
};
extern void copy_mem_map(struct boot_mem_map *, struct nlm_boot_mem_map *);


#ifdef CONFIG_NLM_XLP
extern struct psb_info *prom_info;
extern struct psb_info prom_info_copy;
extern struct boot_mem_map boot_physaddr_info;

extern int read_prominfo(void);
extern int read_dram_info(void);
extern int read_physaddr_map(void);

extern int wakeup_secondary_cpus(void);
#endif

#endif
