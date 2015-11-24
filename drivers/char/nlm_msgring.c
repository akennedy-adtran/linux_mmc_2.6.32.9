/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/smp_lock.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/device.h>

#define MSGRING_WAIT_CHRDEV_NAME "nlm_msgring"
static int msgring_major;
static wait_queue_head_t msgring_wq[NR_CPUS];
volatile static int msgring_status[NR_CPUS];
static int msgring_timeout[NR_CPUS];

/* Additions to work with udev */
static struct class *msgring_class;
static struct device *msgring_device;
static dev_t msgring_devt;

/* TODO - fix this hack and do it properly
   Pass parameter directly instead of passing pointer to parameter (works for both 32b and 64b
   userspace.  Driver as written doesn't work for 32b userspace with 64b kernel */
#define NLM_MSGRING_WAIT_IOC		'm'
#define NLM_MSGRING_WAIT_VC			_IOWR(NLM_MSGRING_WAIT_IOC, 1, unsigned int)
#define NLM_MSGRING_WAIT_TIMEOUT	_IOWR(NLM_MSGRING_WAIT_IOC, 2, unsigned int)
extern unsigned int intr_vc_mask;

extern int xlp_register_vc_intr_handler(int vc, int (*handler)(int vc));
extern void xlp_unregister_vc_intr_handler(int (*handler)(int));

/* Called when a CPU VC interrupt triggers and we are the registered handler.  Need to disable
 * further interrupts until read function is called.
 */
static void msgring_event(int vc)
{
	int cpu = hard_smp_processor_id();
	int node = cpu >> 5;
	uint32_t output_q = ((cpu & 0x1f) << 2) + vc;

	nlm_hal_disable_vc_intr(node, output_q);

	if(msgring_status[cpu] != 1)
		return 0;

	msgring_status[cpu] = 0;
	wake_up_interruptible(&msgring_wq[cpu]);
}

/* Enable interrupt and put reading process to sleep */
static ssize_t msgring_read (struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	int cpu = hard_smp_processor_id();
	int node = cpu >> 5;
	uint32_t output_q = ((cpu & 0x1f) << 2) + vc;

	msgring_status[cpu] = 1;

	nlm_hal_enable_vc_intr(node, output_q, LVL_INT_HIGHWM, HWM_NON_EMPTY, TMR_INT_DISABLE, 0);

	if(msgring_timeout[cpu] < 0)
		wait_event_interruptible(msgring_wq[cpu], (msgring_status[cpu] == 0));
	else
		wait_event_interruptible_timeout(msgring_wq[cpu], (msgring_status[cpu] == 0), msgring_timeout[cpu]);

	return count;
}

static int msgring_open (struct inode *inode, struct file *filp)
{
	return 0;
}

static int msgring_release (struct inode *inode, struct file *filp)
{
	return 0;
}

static int msgring_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		   unsigned long arg)
{
	int err = 0, vc = -1;
	int cpu = hard_smp_processor_id();
	int timeout;
	static int registered = 0;

	switch (cmd) {
		case NLM_MSGRING_WAIT_VC:
			if(registered) {
				printk(KERN_NOTICE "[%s] Already registered VC %d - ignoring IOCTL\n",
						__func__, vc);
				break;
			}
			vc = (int)(arg & 0x03UL);
			xlp_register_vc_intr_handler(vc, msgring_event);
			registered = 1;
			break;
		case NLM_MSGRING_WAIT_TIMEOUT:
			timeout = (int)arg;
			msgring_timeout[cpu] = timeout;
			printk(KERN_NOTICE "[%s] Timeout for cpu %d is %d\n",
					__FUNCTION__, cpu, timeout);
			break;
		default:
			printk(KERN_WARNING "[%s] Received ioctl cmd 0x%08x, expecting 0x%08x\n",
					__FUNCTION__, cmd, (unsigned int)NLM_MSGRING_WAIT_VC);
			err = -EINVAL;
	}
	return err;
}

#ifdef CONFIG_COMPAT
static long msgring_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -1;
	lock_kernel();
	ret = msgring_ioctl(NULL,filp,cmd,arg);
	unlock_kernel();
	return (long)ret;
}
#endif /* CONFIG_COMPAT */

static struct file_operations msgring_fops = {
	owner:		THIS_MODULE,
	read:		msgring_read,
	open:  		msgring_open,
	ioctl:		msgring_ioctl,
#ifdef CONFIG_COMPAT
	compat_ioctl:   msgring_compat_ioctl,
#endif
	release:        msgring_release,
};

static int __init msgring_init(void)
{
	int i;

	msgring_major = register_chrdev(0, MSGRING_WAIT_CHRDEV_NAME, &msgring_fops);
	if (msgring_major < 0) {
		printk(KERN_WARNING"[%s] Failed to register %s char device\n", __FUNCTION__, MSGRING_WAIT_CHRDEV_NAME);
		return msgring_major;
	}

	/* Register a class and a device so udev works */
	msgring_class = class_create(THIS_MODULE, MSGRING_WAIT_CHRDEV_NAME);
	if(IS_ERR(msgring_class)) {
		printk(KERN_WARNING "[%s] Failed to create %s class\n", __FUNCTION__, MSGRING_WAIT_CHRDEV_NAME);
		unregister_chrdev(msgring_major, MSGRING_WAIT_CHRDEV_NAME);
		return PTR_ERR(msgring_class);
	}	

	msgring_devt = MKDEV(msgring_major, 0);
	msgring_device = device_create(msgring_class, NULL, msgring_devt, NULL, MSGRING_WAIT_CHRDEV_NAME);
	if(IS_ERR(msgring_device)) {
		printk(KERN_WARNING"[%s] Failed to create %s device\n", __FUNCTION__, MSGRING_WAIT_CHRDEV_NAME);
		class_unregister(msgring_class);
		class_destroy(msgring_class);
		unregister_chrdev(msgring_major, MSGRING_WAIT_CHRDEV_NAME);
		return PTR_ERR(msgring_device);
	}

	/* Don't do this set-up unless we successfully register with the kernel */
	for(i = 0; i < NR_CPUS; i++) {
		init_waitqueue_head(&msgring_wq[i]);
		msgring_timeout[i] = -1;
	}

	printk(KERN_INFO "[%s] Registered nlm_msgring char device major=%d\n",
			__FUNCTION__, msgring_major);

	return 0;
}

static void __exit msgring_exit(void)
{
	xlp_unregister_vc_intr_handler(msgring_event);
	device_destroy(msgring_class, msgring_devt);
	class_unregister(msgring_class);
	class_destroy(msgring_class);
	unregister_chrdev(msgring_major, MSGRING_WAIT_CHRDEV_NAME);
}

module_init(msgring_init);
module_exit(msgring_exit);
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XLP FMN msgwait driver");
