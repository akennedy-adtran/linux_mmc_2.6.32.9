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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>


struct proc_dir_entry *msgring_debug;
struct proc_dir_entry *msgring_dbglevel;

static int initstatus = 0;

int msgring_debug_level = 3;
EXPORT_SYMBOL(msgring_debug_level);
extern struct proc_dir_entry *nlm_root_proc;

#define MAX_DEBUG_LEVEL 128


static int msgring_debug_read(char *page, char **start, off_t off, int count, int *eof,
				       void *data) {
	int ret = 0;
	if(off > 0) {
		*eof = 1;
		return 0;
	}

	if(!initstatus) {
		ret = sprintf(page, "%d", msgring_debug_level);
	}
	return ret;
}

static int msgring_debug_write(struct file *filp, const char __user *buf,
					unsigned long count, void *data) {
	int ret = 0;
	int level = 0;
	if(!initstatus) {
		char scratch[10];
		if(count < sizeof(scratch)) {
			__copy_from_user(&scratch, buf, count);
			scratch[count] = '\0';
			level = simple_strtol(scratch, NULL, 0);
			if(level < 0 || level > MAX_DEBUG_LEVEL) {
				ret = -EINVAL;
			} else {
				msgring_debug_level = level;
				ret = count;
			}
		} else {
			ret = -ENOSPC;
		}
	}
	return ret;
}


static int msgring_debug_init(void) {
	msgring_debug = proc_mkdir("msgring_debug", nlm_root_proc);
	if(!msgring_debug) {
		printk(KERN_ERR "unable to create /proc/msgring_debug.\n");
		initstatus = -ENOMEM;
	}

	if(!initstatus) {
		msgring_dbglevel = create_proc_entry("level", 0644, msgring_debug);
		if(!msgring_dbglevel) {
			printk(KERN_ERR "unable to create proc entry: /proc/msgring_debug/level.\n");
			initstatus = -ENOMEM;
			remove_proc_entry("msgring_debug", nlm_root_proc);
		} else {
			msgring_dbglevel->read_proc = msgring_debug_read;
			msgring_dbglevel->write_proc = msgring_debug_write;
		}
	}
	return initstatus;
}

static void msgring_debug_exit(void) {
	if(!initstatus) {
		remove_proc_entry("level", msgring_debug);
		remove_proc_entry("msgring_debug", nlm_root_proc);
	}
}

module_init(msgring_debug_init);
module_exit(msgring_debug_exit);
