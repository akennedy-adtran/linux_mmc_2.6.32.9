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

#if !defined(__KERNEL__) && !defined(NLM_HAL_NETLBOOT)
#include <stdint.h>
#endif

/**
 * open_fdt - get pointer to fdt blob for given file descriptor
 * @fd: file descriptor for dtb
 *
 * returns:
 * 	pointer to fdt blob, on success
 * 	NULL, on error
 */
extern void *open_fdt(int fd);


enum prop_type {
	PROP_STR = 0,
	PROP_CELL
};

/**
 * copy_fdt_prop - copies value of given path and property
 * @fdt: pointer to fdt blob
 * @path: path of node to find
 * @prop: property to find
 * @type: type of property
 * @buf: pointer to buffer (copy destination)
 * @len: size of buffer (copy destination)
 *
 * returns:
 * 	bytes copied to buffer, on success
 * 	-FDT_ERR_BADPATH, given path does not begin with '/' or is invalid
 * 	-FDT_ERR_NOTFOUND, node does not exist or does not have named property
 * 	-FDT_ERR_BADMAGIC,
 * 	-FDT_ERR_BADVERSION,
 * 	-FDT_ERR_BADSTATE,
 * 	-FDT_ERR_BADSTRUCTURE,
 * 	-FDT_ERR_TRUNCATED, standard meanings
 */
extern int copy_fdt_prop(void *fdt, const char *path, const char *prop,
	enum prop_type type, void *buf, int len);

static inline int copy_fprop_str(void *fdt,
	const char *path, const char *prop, char *buf, int buflen) {
	return copy_fdt_prop(fdt, path, prop, PROP_STR, (void *)buf, buflen);
}

static inline int copy_fprop_cell(void *fdt,
	const char *path, const char *prop, uint32_t *cells, int numcells) {
	int len = copy_fdt_prop(fdt, path, prop, PROP_CELL,
	                        (void *)cells, numcells * sizeof(uint32_t));
	return len / sizeof(uint32_t);
}

/**
 * set_fdt_helper_print - switch for print output from helper functions
 * @val: zero = off, non-zero = on
 *
 * returns:
 * 	previous setting
 */
extern int set_fdt_helper_print(int val);
