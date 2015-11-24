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

#include <linux/stddef.h>
#include <linux/string.h>

#include <asm/bootinfo.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/bootinfo.h>
#include <xen/interface/xen.h>

static int is_valid_prominfo(struct psb_info *info)
{
	if (!prom_info) 
		return -1;
  
	if ((prom_info->magic_dword & 0xffffffffULL) != 0x900dbeef) 
		return -1;

	if ((prom_info->magic_dword >> 32) != PSB_INFO_VERSION) 
		return -1;

	return 0;
}

int read_cmdline_args(int *argc, char *n_argv[], char *n_envp[])
{
	char **argv, **envp;
	int i;
	int32_t *t_argv;

	*argc = (int)fw_arg0;
	argv = (char **)(unsigned long)(int)fw_arg1;
	envp = (char **)(unsigned long)(int)fw_arg2;

		
	for (i = 0, t_argv = (int32_t *)argv; i < *argc; i++, t_argv++)
		n_argv[i] = (char *)(unsigned long)(*t_argv);

		if (envp != NULL) {
			int32_t *t_envp;

			for (i = 0, t_envp = (int32_t *)envp; *t_envp; i++) {
				n_envp[i] = (char *)(unsigned long)(*t_envp);
				t_envp++;
		}
	}

	return 0;
}

int read_prominfo(void)
{
	prom_info = &prom_info_copy;

	memcpy((void *)prom_info, (void *)(unsigned long)(int)fw_arg3, 
		   sizeof(struct psb_info));
	
	return is_valid_prominfo(prom_info);
}

/* TODO: Need to add right code here for XLP here */
int read_dram_info(void)
{
	struct nlm_boot_mem_map *map;

	if (!prom_info || (!prom_info->psb_mem_map && !prom_info->avail_mem_map)) 
		return -1;

	/* copy the mem_map from bootloader */
	if (sizeof(*prom_info) <= prom_info->size && prom_info->avail_mem_map)
		map = (struct nlm_boot_mem_map *) ((unsigned long)prom_info->avail_mem_map);	
	else
		map = (struct nlm_boot_mem_map *)((unsigned long)prom_info->psb_mem_map);
	
	if (!(map->nr_map > 0 && map->nr_map <= 32))
		return -1;

	copy_mem_map(&prom_map, map);
	
	return 0;
}

/* TODO: Is this valid for XLP ? */
int read_physaddr_map(void)
{
	struct nlm_boot_mem_map *physaddr_map = (struct nlm_boot_mem_map *)
		((unsigned long)prom_info->psb_physaddr_map);

	if (physaddr_map == NULL)
		return -1;

	copy_mem_map(&boot_physaddr_info,  physaddr_map);

	return 0;
}
