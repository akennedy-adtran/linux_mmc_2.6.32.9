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

#if defined(__KERNEL__)
#if !defined(NLM_HAL_UBOOT)
#include <linux/kernel.h>
#endif
#else
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#if !defined(NLM_HAL_NETLBOOT) && !defined(NLM_HAL_NETOS)
#include <sys/mman.h>
#endif
#endif

#include "libfdt.h"
#include "fdt_helper.h"

static int fdt_helper_print  = 1;

extern int printf(const char *format, ...);
#if !defined(__KERNEL__)
#define fdtprint(x...) do {   \
	if (fdt_helper_print) \
		printf(x);    \
} while (0)
#elif defined(NLM_HAL_UBOOT)
#define fdtprint(x...) do {   \
	if (fdt_helper_print) \
		printf(x);    \
} while (0)
#else
#define fdtprint(x...) do {   \
	if (fdt_helper_print) \
		printk(x);    \
} while (0)
#endif

int set_fdt_helper_print(int val)
{
	int old_val = fdt_helper_print;
	fdt_helper_print = val;
	return old_val;
}

#if !defined(__KERNEL__) && !defined(NLM_HAL_NETOS) && !defined(NLM_HAL_NETLBOOT)
void *open_fdt(int fd)
{
	struct stat st;
	if (fstat(fd, &st) < 0) {
		perror("fstat");
		return NULL;
	}

	void *fdt;
	fdt = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if (fdt == MAP_FAILED) {
		perror("mmap");
		return NULL;
	}
	return fdt;
}
#else
void *open_fdt(int fd) { return NULL; }
#endif

static void print_fdt_prop(const char *path, const char *prop,
	enum prop_type type, const void *buf, int len)
{
#if 0
	fdtprint("FDT: parsed %s.%s: ", path, prop);
	if (type == PROP_CELL) {
		const uint32_t *dst = (const uint32_t *)buf;
		int cells = len / sizeof(uint32_t);
		int i;

		fdtprint("cells=%d val=", cells);
		for (i = 0; i < cells; i++) {
			fdtprint("0x%x(%d),", dst[i], dst[i]);
		}
	}
	else {
		fdtprint("len=%d val=%s", len, (const char *)buf);
	}
	fdtprint("\n");
#endif
}

int copy_fdt_prop(void *fdt, const char *path, const char *prop,
	enum prop_type type, void *buf, int len)
{
	int nodeoffset;
	const void *pval;
	int plen;
	int copylen;

	if (len <= 0)  {
		fdtprint("Warning: Len is 0 while copying %s/%s\n", path, prop);
		return -1;
	}

	nodeoffset = fdt_path_offset(fdt, path);
	if (nodeoffset < 0) {
#if 0
		fdtprint("%s: Failed to parse path %s\n",
		         fdt_strerror(nodeoffset), path);
#endif
		return nodeoffset;
	}

	pval = fdt_getprop(fdt, nodeoffset, prop, &plen);
	if (pval == NULL) {
#if 0
		fdtprint("%s: Failed to parse property %s\n",
		         fdt_strerror(plen), prop);
#endif
		return plen;
	}

	if (plen > len) {
		fdtprint("WARNING: buf of %d is insufficient to store %d (%s/%s)\n",
		         len, plen, path, prop);
		copylen = len;
	}
	else {
		copylen = plen;
	}

	if (type == PROP_CELL) {
		const uint32_t *src = (const uint32_t *)pval;
		uint32_t *dst = (uint32_t *)buf;
		int i;
		for (i = 0; i < copylen / sizeof(uint32_t); i++) {
			dst[i] = fdt32_to_cpu(src[i]);
		}
	}
	else {
		memcpy(buf, pval, copylen);
	}

 	print_fdt_prop(path, prop, type, buf, copylen); 

	return copylen;
}
