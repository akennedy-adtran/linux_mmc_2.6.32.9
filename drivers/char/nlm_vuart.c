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


#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/device.h>

#ifdef CONFIG_XEN
#include <xen/interface/xen.h>
#include <xen/grant_table.h>
#include <asm/xen/hypercall.h>
#endif

#define VUART_MAJOR 251
#define VUART_NAME "nlm_vuart"
#define CASE1 1
#define CASE2 2

static unsigned int counter = 0;
static char string [128];
static int data;

DECLARE_WAIT_QUEUE_HEAD(vuart_wait);
static int data_not_ready = 0;
#ifdef CONFIG_XEN
extern int gnttab_grant_foreign_access(domid_t domid, unsigned long frame, int readonly);
extern void gnttab_update_service (uint32_t service_for_dom_id, uint32_t start_ref_id,
				uint32_t nr_pages, uint64_t pfn, uint32_t service_id);
#else
static unsigned long uart_sharedcfg_base = (496 << 20);
//static unsigned long uart_sharedcfg_size = (4 << 20);
#endif

static int vuart_open (struct inode *inode, struct file *file) {
	return 0;
}

static int vuart_release (struct inode *inode, struct file *file) {
	return 0;
}

static ssize_t vuart_read (struct file *file, char *buf,
			   size_t count, loff_t *ppos)
{
	int len, err;

	/*
	 * check if we have data - if not, sleep wake up in
	 * interrupt_handler
	 */
	while (data_not_ready)
		interruptible_sleep_on(&vuart_wait);

	if (counter <= 0)
		return 0;

	err = copy_to_user(buf,string,counter);
	if (err != 0)
		return -EFAULT;

	len = counter;
	counter = 0;

	return len;
}

/*
 * write function called when to /dev/vuart is written
 */
static ssize_t vuart_write (struct file *file, const char *buf,
			    size_t count, loff_t *ppos)
{
	int err;

	err = copy_from_user(string,buf,count);
	if (err != 0)
		return -EFAULT;

	counter += count;

	return count;
}

static int vuart_ioctl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg) {
	int retval = 0;

	switch (cmd) {

	case CASE1: /* for writing data to arg */
		if (copy_from_user(&data, (int *)arg, sizeof(int)))
			return -EFAULT;
		break;

	case CASE2: /* for reading data from arg */
		if (copy_to_user((int *)arg, &data, sizeof(int)))
			return -EFAULT;
		break;

	default:
		retval = -EINVAL;
	}

	return retval;
}

static long vuart_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long ret = -1;

	lock_kernel();
	ret = vuart_ioctl(NULL, filp, cmd, arg);
	unlock_kernel();

	if (ret)
		return -ENOIOCTLCMD;

	return ret;
}

#ifdef CONFIG_XEN
#define NUM_VUARTS 32
#define NLM_VUART_ID 100
static unsigned long shared_page_paddr = 0;

static int vuart_mmap(struct file * filp, struct vm_area_struct * vma)
{
	if (!shared_page_paddr) {
		/* Sharing of page */
		int i, ref;
		char* free_page;
		unsigned long mfn;

		/* Each Page is shared by read and write vuarts */
		free_page = (char *) __get_free_pages (GFP_KERNEL, get_order(NUM_VUARTS/2 * PAGE_SIZE));
		mfn = __pa (free_page) >> PAGE_SHIFT;

		for (i = 0; i < NUM_VUARTS/2; i++) {
			ref = gnttab_grant_foreign_access (i, mfn + i, 0);
			gnttab_update_service (i, ref, 1, mfn + i, NLM_VUART_ID);
		}

		shared_page_paddr = (unsigned long) __pa(free_page);
	}

	if (remap_pfn_range (vma, (unsigned long) vma->vm_start,
						 shared_page_paddr >> PAGE_SHIFT,
						 1 << (PAGE_SHIFT + get_order (NUM_VUARTS/2 * PAGE_SIZE)),
						 vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}
#else
static int vuart_mmap(struct file * filp, struct vm_area_struct * vma)
{
	unsigned long vm_size = vma->vm_end - vma->vm_start;

	if (vma->vm_flags & VM_LOCKED)
	{
		printk("%s: VM_LOCKED flag is set \n", __func__);
		return -EPERM;
	}

	vma->vm_flags |= (VM_RESERVED | VM_IO);

        if (remap_pfn_range(vma, vma->vm_start, (uart_sharedcfg_base >> PAGE_SHIFT),
			    vm_size, vma->vm_page_prot))
	{
		printk("%s: remap_pfn_range failed\n", __func__);
		return -EAGAIN;
	}

	printk("[%s]: Linux Loader pbase=%#lx size=%#lx\n", __FUNCTION__, uart_sharedcfg_base, vm_size);

	return 0;
}

static unsigned int vuart_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int mask;

	if (!filp->private_data) return -EINVAL;

	mask = 0;

	return mask;
}
#endif

struct file_operations vuart_fops = {
	.owner	      =	THIS_MODULE,
	.read	      =	vuart_read,
	.write	      =	vuart_write,
	.mmap	      =	vuart_mmap,
#ifndef CONFIG_XEN
	.poll	  =	vuart_poll,
#endif
	.ioctl	      =	vuart_ioctl,
	.compat_ioctl =	vuart_compat_ioctl,
	.open	      =	vuart_open,
	.release      =	vuart_release,
};

static int vuart_init (void)
{
	int ret;

	ret = register_chrdev(VUART_MAJOR, VUART_NAME, &vuart_fops);
	if (ret != 0) {
		printk("[%s] Failed to register char device major=%d\n", __FUNCTION__, VUART_MAJOR);
		return -EIO;
	}

	printk("[%s] Registered char device major=%d\n", __FUNCTION__, VUART_MAJOR);
	return 0;
}

static void vuart_cleanup (void)
{
	printk("cleaning up module\n");
	unregister_chrdev (VUART_MAJOR, VUART_NAME);
}

module_init(vuart_init);
module_exit(vuart_cleanup);
MODULE_AUTHOR("Netlogic");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Virtual UART driver");
