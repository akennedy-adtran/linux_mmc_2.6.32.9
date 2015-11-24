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


#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/init.h>
/*#include <linux/proc_fs.h>*/
/*#include <linux/cpumask.h>*/

#include <asm/uaccess.h>
#include <asm/mman.h>
#include <asm/atomic.h>
#include <asm/smp.h>
#include <asm/bootinfo.h>

#include <asm/netlogic/pic.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/devices.h>
#include <asm/netlogic/nlm_common_rmios_debugger.h>

#define PHXDEB_DIAG(a, b...) //printk(a, ##b)
#define ErrorMsg(a, b...) printk(a, ##b) 
//#define barrier() __asm__ __volatile__(".set mips3\n" "sync": : :"memory")

struct nlm_common_rmios_debugger_struct {
	int cpu;
	int bucket;
	int code;
	int msgsize;
	int stid;
	struct msgrng_msg msg;
	uint64_t buf;
	uint64_t mem_addr;
};
static spinlock_t nlm_common_rmios_debugger_lock[LINUX_RMIOS_VCPU];
static struct __wait_queue_head nlm_common_rmios_debugger_waitq[LINUX_RMIOS_VCPU];
static u32 length[LINUX_RMIOS_VCPU];
static struct nlm_common_rmios_debugger_struct nlm_common_rmios_debugger_data[LINUX_RMIOS_VCPU][2]; /* one pending message allowed. */
static int nlm_common_rmios_data_available[LINUX_RMIOS_VCPU];
static spinlock_t msgrng_lock;


static ssize_t nlm_common_rmios_debugger_read(struct file *filep, char __user *user,
					size_t len, loff_t *offset)
{
	int cpu;
	struct nlm_common_rmios_debugger_struct local;
	if(copy_from_user((void *)&local, (void __user *)user, sizeof(local))) {
		ErrorMsg("Invalid address\n");
		return -EINVAL;
	}
	cpu = local.cpu;

	wait_event_interruptible(nlm_common_rmios_debugger_waitq[cpu], (nlm_common_rmios_data_available[cpu] > 0) /* condition */);

    
	if(copy_to_user((void __user *)user, (void *)&nlm_common_rmios_debugger_data[cpu][0], sizeof(local))){
		return -EINVAL;
	}
	spin_lock_irq(&nlm_common_rmios_debugger_lock[cpu]);
	nlm_common_rmios_data_available[cpu]--;
	if (nlm_common_rmios_data_available[cpu]) {
		nlm_common_rmios_debugger_data[cpu][0].code = 
			nlm_common_rmios_debugger_data[cpu][1].code;
		nlm_common_rmios_debugger_data[cpu][0].msgsize =
			nlm_common_rmios_debugger_data[cpu][1].msgsize;
		nlm_common_rmios_debugger_data[cpu][0].stid =
			nlm_common_rmios_debugger_data[cpu][1].stid;
		memcpy(&(nlm_common_rmios_debugger_data[cpu][0].msg), &(nlm_common_rmios_debugger_data[cpu][1].msg), sizeof(struct msgrng_msg));
	}
	spin_unlock_irq(&nlm_common_rmios_debugger_lock[cpu]);
	return 0;
}

static ssize_t nlm_common_rmios_debugger_write(struct file *filep, const char __user *user,
					size_t len, loff_t *offset)
{
	struct nlm_common_rmios_debugger_struct local;
	struct msgrng_msg msg;
	int bucket;
	int code;
	int msgsize;
	unsigned long flags, msgrng_flags;

	if(copy_from_user((void *)&local, (void __user *)user, sizeof(local))) {
		ErrorMsg("Invalid address\n");
		return -EINVAL;
	}
	bucket = local.bucket;
	code = local.code;
	msgsize = local.msgsize;
	memcpy(&msg, &local.msg, sizeof(msg));
	
	msgrng_access_save(&msgrng_lock, flags, msgrng_flags);

	while ((code = message_send(msgsize, code, bucket, &msg)) != 0) { };     

	PHXDEB_DIAG("Message Sent:   code = %x\n", code);

	msgrng_access_restore(&msgrng_lock, flags, msgrng_flags); 
	return 0;

}

static int nlm_common_rmios_debugger_ioctl(struct inode *inode, struct file *file,
        unsigned int cmd, unsigned long arg)
{
	struct nlm_common_rmios_debugger_struct local;
	struct msgrng_msg msg;
	int bucket;
	int code;
	int msgsize=0;
	int cpu;
	unsigned long flags, msgrng_flags;
	char *buf;
	
	if (copy_from_user(&local, (void __user *)arg, sizeof(local)))
	{
		ErrorMsg("Invalid address\n");
		return -EINVAL;
	}
	cpu = local.cpu;

	switch (cmd) {
		case NLM_RMIOS_DEBUGGER_WRITE: 
			bucket = local.bucket;
			code = local.code;
			msgsize = local.msgsize;
			memcpy(&msg, &local.msg, sizeof(msg));
			msgrng_access_save(&msgrng_lock, flags, msgrng_flags);

			while ((code = message_send(msgsize, code, bucket, &msg)) != 0) { };     

			PHXDEB_DIAG("Message Sent:   code = %x\n", code);

			msgrng_access_restore(&msgrng_lock, flags, msgrng_flags); 
			break;
		case NLM_RMIOS_DEBUGGER_READ:
			wait_event_interruptible(nlm_common_rmios_debugger_waitq[cpu], (nlm_common_rmios_data_available[cpu] > 0) /* condition */);

			spin_lock_irq(&nlm_common_rmios_debugger_lock[cpu]);
			if (copy_to_user((void __user *)arg, (void *)&nlm_common_rmios_debugger_data[cpu][0], sizeof(local))){
				return -EINVAL;
			}
			nlm_common_rmios_data_available[cpu]--;
			if (nlm_common_rmios_data_available[cpu]) {
				nlm_common_rmios_debugger_data[cpu][0].code = 
					nlm_common_rmios_debugger_data[cpu][1].code;
				nlm_common_rmios_debugger_data[cpu][0].msgsize =
					nlm_common_rmios_debugger_data[cpu][1].msgsize;
				nlm_common_rmios_debugger_data[cpu][0].stid =
					nlm_common_rmios_debugger_data[cpu][1].stid;
				memcpy(&(nlm_common_rmios_debugger_data[cpu][0].msg), &(nlm_common_rmios_debugger_data[cpu][1].msg), sizeof(struct msgrng_msg));
			}
			spin_unlock_irq(&nlm_common_rmios_debugger_lock[cpu]);
			break;
		case NLM_RMIOS_DEBUGGER_TX_MEM_WRITE:
			buf = (unsigned char *)(LINUX_RMIOS_TX_BUF_BASE + LINUX_RMIOS_TX_BUF_SIZE * cpu);
			msgsize = local.msgsize;
			if (copy_from_user(buf, (void __user *)(long)(local.buf), msgsize))
			{
				ErrorMsg("Invalid address\n");
				return -EINVAL;
			}
			barrier();
			break;
		case NLM_RMIOS_DEBUGGER_TX_MEM_READ:
			buf = (unsigned char *)(LINUX_RMIOS_TX_BUF_BASE + LINUX_RMIOS_TX_BUF_SIZE * cpu);
			msgsize = local.msgsize;
			if (copy_to_user((void __user *)(long)(local.buf), (void *)buf, msgsize))
			{
				ErrorMsg("Invalid address\n");
				return -EINVAL;
			}
			break;
		case NLM_RMIOS_DEBUGGER_RX_MEM_WRITE:
			buf = (unsigned char *)(LINUX_RMIOS_RX_BUF_BASE + LINUX_RMIOS_RX_BUF_SIZE * cpu);
			msgsize = local.msgsize;
			if (copy_from_user(buf, (void __user *)(long)(local.buf), msgsize))
			{
				ErrorMsg("Invalid address\n");
				return -EINVAL;
			}
			barrier();
			break;
		case NLM_RMIOS_DEBUGGER_RX_MEM_READ:
			buf = (unsigned char *)(LINUX_RMIOS_RX_BUF_BASE + LINUX_RMIOS_RX_BUF_SIZE * cpu);
			msgsize = local.msgsize;
			if (copy_to_user((void __user *)(long)(local.buf), (void *)buf, msgsize))
			{
				ErrorMsg("Invalid address\n");
				return -EINVAL;
			}
			break;
		case NLM_RMIOS_DEBUGGER_MEM_READ:
			buf = (char *)(long)(local.mem_addr);
			msgsize = local.msgsize;
			if (copy_to_user((void __user *)(long)(local.buf), (void *)buf, msgsize))
			{
				ErrorMsg("Invalid address\n");
				return -EINVAL;
			}
			break;
		case NLM_RMIOS_DEBUGGER_MEM_WRITE:
			buf = (char *)(long)(local.mem_addr);
			msgsize = local.msgsize;
			if (copy_from_user(buf, (void __user *)(long)(local.buf), msgsize))
			{
				ErrorMsg("Invalid address\n");
				return -EINVAL;
			}
			barrier();
			break;
		case NLM_RMIOS_DEBUGGER_PIC_IPI:
			{
			nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);	
			netlogic_write_reg(mmio, PIC_IPI, local.cpu);
			}
			break;
		default:
			ErrorMsg("Invalid command\n");
			return -EINVAL;
	}
	return 0;
}

static long nlm_common_rmios_debugger_compat_ioctl(struct file *filp, unsigned int cmd,
                unsigned long arg)
{
        unsigned long ret = -1;
        lock_kernel();
        ret = nlm_common_rmios_debugger_ioctl(NULL, filp, cmd, arg);
        unlock_kernel();
        if(ret){
                printk("%s: ioctl error\n", __FUNCTION__);
                return -EINVAL;
        }
        return ret;
}

static int nlm_common_rmios_debugger_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int nlm_common_rmios_debugger_release(struct inode *inode, struct file *filp)
{
	return 0;
}

struct file_operations nlm_common_rmios_debugger_fops = {
	.owner    = THIS_MODULE,
	.open     = nlm_common_rmios_debugger_open,
	.release  = nlm_common_rmios_debugger_release,
	.read     = nlm_common_rmios_debugger_read,
	.write    = nlm_common_rmios_debugger_write,
	.ioctl    = nlm_common_rmios_debugger_ioctl,
	.compat_ioctl    = nlm_common_rmios_debugger_compat_ioctl, /* 32-bit appn in 64-bit linux goes through this */
};

static int nlm_common_rmios_debugger_major;

static int nlm_common_rmios_debugger_init(void)
{
	int i;
	printk("%s - Phnx Rmios Debugger\n", __FUNCTION__);

	nlm_common_rmios_debugger_major = register_chrdev(XLR_DEBUGGER_MAJOR, NLM_DEB_DEV_NAME, 
			&nlm_common_rmios_debugger_fops);
	if(nlm_common_rmios_debugger_major < 0) {
		printk("%s: register_chrdev() failed\n", __FUNCTION__);
		return nlm_common_rmios_debugger_major;
	}
	printk("Created Device %s with major number %d\n", NLM_DEB_DEV_NAME,
				XLR_DEBUGGER_MAJOR);
	for (i = 0; i < LINUX_RMIOS_VCPU; i++) {
		init_waitqueue_head(&nlm_common_rmios_debugger_waitq[i]);
		spin_lock_init(&nlm_common_rmios_debugger_lock[i]);
	}
	return 0;
}

static void nlm_common_rmios_debugger_exit(void)
{
	printk("%s - Exit called\n", __FUNCTION__);
	unregister_chrdev(nlm_common_rmios_debugger_major, NLM_DEB_DEV_NAME);
}

static void __init reserve_linux_rmios_memory(void)
{
	add_memory_region (LINUX_RMIOS_SHARED_BASE, 0x10000, BOOT_MEM_RESERVED);
}
early_initcall(reserve_linux_rmios_memory);


module_init(nlm_common_rmios_debugger_init);
module_exit(nlm_common_rmios_debugger_exit);

