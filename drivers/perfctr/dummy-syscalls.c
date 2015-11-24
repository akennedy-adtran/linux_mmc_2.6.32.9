/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: dummy-syscalls.c,v 1.1.2.3 2007-10-31 17:34:40 kmurthy Exp $
 * Virtual per-process performance counters.
 *
 * Copyright (C) 1999-2004  Mikael Pettersson
 */
#include <linux/init.h>
#include <linux/compiler.h> /* for unlikely() in 2.4.18 and older */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/ptrace.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/perfctr.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "cpumask.h"
#include "virtual.h"
#include "compat.h"

/****************************************************************
 *								*
 * Virtual perfctr actual system calls.				*
 *								*
 ****************************************************************/

/* tid is the actual task/thread id (née pid, stored as ->pid),
   pid/tgid is that 2.6 thread group id crap (stored as ->tgid) */

asmlinkage long sys_vperfctr_open(int tid, int creat)
{
   printk ("PERFCTR not configured, unimplemented syscall\n");
   return -EINVAL;
}

asmlinkage long sys_vperfctr_control(int fd,
				     const struct vperfctr_control __user *argp,
				     unsigned int argbytes)
{
   printk ("PERFCTR not configured, unimplemented syscall\n");
   return -EINVAL;
}

asmlinkage long sys_vperfctr_unlink(int fd)
{
   printk ("PERFCTR not configured, unimplemented syscall\n");
   return -EINVAL;
}

asmlinkage long sys_vperfctr_iresume(int fd)
{
   printk ("PERFCTR not configured, unimplemented syscall\n");
   return -EINVAL;
}

asmlinkage long sys_vperfctr_read(int fd, unsigned int cmd, void __user *argp, unsigned int argbytes)
{
   printk ("PERFCTR not configured, unimplemented syscall\n");
   return -EINVAL;
}
