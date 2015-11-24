
/*
 * (C) Copyright 2007
 * Gerald Van Baren, Custom IDEAS, vanbaren@cideas.com
 * Based on code written by:
 *   Pantelis Antoniou <pantelis.antoniou@gmail.com> and
 *   Matthew McClintock <msm@freescale.com>
 * 
 * (C) Copyright 2012
 * Broadcom Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/cpumask.h>

#include <asm/netlogic/proc.h>
#include <asm/mach-netlogic/mmu.h>
#include <asm/time.h>
#include "libfdt.h"
#include "fdt_helper.h"

extern struct proc_dir_entry *nlm_root_proc;
extern void * fdt;

#define MAX_LEVEL	32		// How deeply nested we can go
#define SCRATCHPAD	1024	// Bytes of scratchpad memory

struct fdt_header *working_fdt;

#define isprint(c) (((c) >= 0x20) && ((c) <= 0x7e))

static int is_printable_string(const void *data, int len)
{
	const char *s = data;

	/* zero length is not */
	if (len == 0)
		return 0;

	/* must terminate with zero */
	if (s[len - 1] != '\0')
		return 0;

	/* printable or a null byte (concatenated strings) */
	while (((*s == '\0') || isprint(*s)) && (len > 0)) {
		/*
		 * If we see a null, there are three possibilities:
		 * 1) If len == 1, it is the end of the string, printable
		 * 2) Next character also a null, not printable.
		 * 3) Next character not a null, continue to check.
		 */
		if (s[0] == '\0') {
			if (len == 1)
				return 1;
			if (s[1] == '\0')
				return 0;
		}
		s++;
		len--;
	}

	/* Not the null termination, or not done yet: not printable */
	if (*s != '\0' || (len != 0))
		return 0;

	return 1;
}

static int print_data(char * page, const void *data, int len)
{
	int j, plen = 0;

	/* no data, don't print */
	if (len == 0)
		return plen;

	/*
	 * It is a string, but it may have multiple strings (embedded '\0's).
	 */
	if (is_printable_string(data, len)) {
		plen += sprintf(page + plen, "\"");
		j = 0;
		while (j < len) {
			if (j > 0)
				plen += sprintf(page + plen, "\", \"");
				plen += sprintf(page + plen, "%s", (char *)data);
			j    += strlen(data) + 1;
			data += strlen(data) + 1;
		}
		plen += sprintf(page + plen, "\"");
		return plen;
	}

	if ((len %4) == 0) {
		const u32 *p;

		plen += sprintf(page + plen, "<");
		for (j = 0, p = data; j < len/4; j ++)
			plen += sprintf(page + plen, "0x%x%s", fdt32_to_cpu(p[j]), j < (len/4 - 1) ? " " : "");
		plen += sprintf(page + plen, ">");
	} else { /* anything else... hexdump */
		const u8 *s;

		plen += sprintf(page + plen, "[");
		for (j = 0, s = data; j < len; j++)
			plen += sprintf(page + plen, "%02x%s", s[j], j < len - 1 ? " " : "");
		plen += sprintf(page + plen, "]");
	}

	return plen;
}

static int nlm_fdt_read_ucore(char *page, char **start, off_t off,
				int count, int *eof, void * data)
{
	char path[128];
	int node, src;
	uint32_t blen, wlen;
	uint8_t * buf;
	int plen = 0, windex, _windex;
	off_t begin = 0;
	int srccount = 0;

	working_fdt = (struct fdt_header *)fdt;

	for(node = 0; node < 4; node++) {
		for(src = 0; src < 32; src++) {
			sprintf(path, "/soc/nae@node-%d/ucore/src@%d", node, src);
			if ( fdt_path_offset (working_fdt, path) < 0 ) 
				continue;

			if ( copy_fdt_prop(working_fdt, path, "src-data-bytes", 
				PROP_CELL, &blen, sizeof(uint32_t)) <= 0 )
				continue;

#ifndef __MIPSEL__
			blen = fdt32_to_cpu(blen);
#endif			
			wlen = ((blen+4)/4)*4;
			buf = (uint8_t *)kmalloc(sizeof(uint8_t)*wlen, GFP_KERNEL);
			if ( !buf ) {
				plen += sprintf(page + plen, "Unable to kmalloc 0x%x bytes for node-%d/src-%d\n",
					wlen, node, src);
				continue;
			}
			if ( copy_fdt_prop(working_fdt, path, "src-data",
				PROP_CELL, buf, wlen) <= 0 ) {
				plen += sprintf(page + plen, "Failed to copy source data for node-%d/src-%d\n",
					node, src);
				continue;
			}

			srccount++;

			/* FIXME - do we need to handle little endian - wrap buf[windex] below with call to fdt32_to_cpu? */
			plen += sprintf(page + plen, "/** -- BEGIN uCore src for %s -- **/\n", path);
			for(windex = 0; windex < blen; windex++) {
#ifndef __MIPSEL__
				_windex = windex;
#else
				_windex = (windex & ~(0x3) ) | (3 - (windex & 0x3) );
#endif
				plen += sprintf(page + plen, "%c", buf[_windex]);
			}
			if (!proc_pos_check(&begin, &plen, off, count)) goto out;
			plen += sprintf(page + plen, "/** -- END uCore src for %s -- **/\n", path);
			
		}
	}

	plen += sprintf(page + plen, "/** Total source files: %d **/\n", srccount);

	*eof = 1;

	out:
	*start = page + (off - begin);
	plen -= (off - begin);
	if (plen > count)
		plen = count;
	if (plen < 0)
		plen = 0;

	return plen;
}

static int nlm_fdt_read(char *page, char **start, off_t off,
			     int count, int *eof, void *data)
{
	static char tabs[MAX_LEVEL+1] =
		"\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t"
		"\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
	const void *nodep;	/* property node pointer */
	int  nodeoffset;	/* node offset from libfdt */
	int  nextoffset;	/* next node offset from libfdt */
	uint32_t tag;		/* tag */
	int  len;		/* length of the property */
	int  level = 0;		/* keep track of nesting level */
	const struct fdt_property *fdt_prop;
	int plen = 0;
	const char *pathp = "/";
	int depth = MAX_LEVEL;
	off_t begin = 0;

	working_fdt = (struct fdt_header *)fdt;

	nodeoffset = fdt_path_offset (working_fdt, pathp);
	if (nodeoffset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		plen += sprintf(page + plen, 
			"libfdt fdt_path_offset() returned %s\n",
			fdt_strerror(nodeoffset));
		goto out;
	}

	/*
	 * The user passed in a node path and no property,
	 * print the node and all subnodes.
	 */
	while(level >= 0) {
		if (!proc_pos_check(&begin, &plen, off, count)) goto out;
		tag = fdt_next_tag(working_fdt, nodeoffset, &nextoffset);
		switch(tag) {
		case FDT_BEGIN_NODE:
			pathp = fdt_get_name(working_fdt, nodeoffset, NULL);
			if (level <= depth) {
				if (pathp == NULL)
					pathp = "/* NULL pointer error */";
				if (*pathp == '\0')
					pathp = "/";	/* root is nameless */
				plen += sprintf(page + plen, "%s%s {\n",
					&tabs[MAX_LEVEL - level], pathp);
			}
			level++;
			if (level >= MAX_LEVEL) {
				plen += sprintf(page + plen, "Nested too deep, aborting.\n");
				goto out;
			}
			break;
		case FDT_END_NODE:
			level--;
			if (level <= depth)
				plen += sprintf(page + plen, "%s};\n", &tabs[MAX_LEVEL - level]);
			if (level == 0) {
				level = -1;		/* exit the loop */
			}
			break;
		case FDT_PROP:
			fdt_prop = fdt_offset_ptr(working_fdt, nodeoffset,
					sizeof(*fdt_prop));
			pathp    = fdt_string(working_fdt,
					fdt32_to_cpu(fdt_prop->nameoff));
			len      = fdt32_to_cpu(fdt_prop->len);
			nodep    = fdt_prop->data;
			if (len < 0) {
				plen += sprintf (page + plen, "libfdt fdt_getprop(): %s\n",
					fdt_strerror(len));
				goto out;
			} else if (len == 0) {
				/* the property has no value */
				if (level <= depth)
					plen += sprintf(page + plen, "%s%s;\n",
						&tabs[MAX_LEVEL - level],
						pathp);
			} else {
				if (level <= depth) {
					plen += sprintf(page + plen, "%s%s = ",
						&tabs[MAX_LEVEL - level],
						pathp);
					plen += print_data (page + plen, nodep, len);
					plen += sprintf(page + plen, ";\n");
				}
			}
			break;
		case FDT_NOP:
			plen += sprintf(page + plen, "%s/* NOP */\n", &tabs[MAX_LEVEL - level]);
			break;
		case FDT_END:
			goto good_out;
		default:
			if (level <= depth)
				plen += sprintf(page + plen, "Unknown tag 0x%08X\n", tag);
			goto out;
		}
		nodeoffset = nextoffset;
	}

	good_out:
	*eof = 1;

	out:
	*start = page + (off - begin);
	plen -= (off - begin);
	if (plen > count)
		plen = count;
	if (plen < 0)
		plen = 0;

	return plen;
}

static int nlm_fdt_proc_init(void)
{
	struct proc_dir_entry *fdt_entry;
	struct proc_dir_entry *fdt_uc_entry;
	fdt_entry = create_proc_read_entry(
			"fdt-main", 0,	// def mode
			nlm_root_proc,	// parent
			nlm_fdt_read,	// proc read function
			0);				// no client data
	if (!fdt_entry) {
		printk(KERN_WARNING "[%s]: Unable to create proc read entry for fdt!\n",
				__FUNCTION__);
		return -1;
	}

	fdt_uc_entry = create_proc_read_entry(
			"fdt-ucore", 0,		// def mode
			nlm_root_proc,		// parent
			nlm_fdt_read_ucore,	// proc read function
			0);					// no client data
	if (!fdt_uc_entry) {
		printk(KERN_WARNING "[%s]: Unable to create proc read entry for ucore fdt!\n",
				__FUNCTION__);
		return -1;
	}

	printk(KERN_INFO "[%s]: Created netlogic proc entry for fdt and ucore\n",
			__FUNCTION__);
	return 0;
}

static void nlm_fdt_proc_exit(void)
{
}

module_init(nlm_fdt_proc_init);
module_exit(nlm_fdt_proc_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom XLP FDT proc viewer");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

