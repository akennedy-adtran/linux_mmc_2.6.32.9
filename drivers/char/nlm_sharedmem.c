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
#include <linux/device.h>

#include <xen/interface/xen.h>
#include <xen/grant_table.h>
#include <asm/xen/hypercall.h>

#define SHAREDMEM_NAME             "nlm_sharedmem"
#define NUM_SHAREDMEMS             (1)
#define NLM_SHAREDMEM_ID           (101)
#define MAX_EXTENTS                (1)
#define MAX_EXTENT_ORDER           (12)
#define MAX_PAGE_REQUEST           (MAX_EXTENTS)
#define DOMID                      (0)
#define DOMAIN_ID_OPTION           (1)
#define MAX_MINOR_NUMBERS          (256)

static unsigned long shared_memory_mfn[MAX_MINOR_NUMBERS] = {0};
static char gnttab_update_done[MAX_MINOR_NUMBERS][MAX_MINOR_NUMBERS] = {{0}}; //[domain_id][minor]
static int num_order[MAX_MINOR_NUMBERS] = {0};
static int domain_id = 1;
static int sharedmem_major_num = 0;

extern int gnttab_grant_foreign_access(domid_t domid, unsigned long frame, int readonly);
extern int gnttab_update_service (uint32_t service_for_dom_id, uint32_t start_ref_id,
								  uint32_t nr_pages, uint32_t pfn, uint32_t service_id);
extern int xen_get_maximum_reservation (uint32_t dom_id);
extern int xen_get_current_reservation (uint32_t dom_id);
extern int xen_increase_max_mem (uint32_t dom_id, uint64_t max_mem);
extern int xen_increase_reservation (uint32_t dom_id, int n_extents, int n_extent_order);
extern xen_pfn_t p2m_host[MAX_EXTENTS];

static int sharedmem_open (struct inode *inode, struct file *file) {
	return 0;
}

static int sharedmem_release (struct inode *inode, struct file *file) {
	return 0;
}

static ssize_t sharedmem_read (struct file *file, char *buf,
							   size_t count, loff_t *ppos) {
	return 0;
}

static ssize_t sharedmem_write (struct file *file, const char *buf,
								size_t count, loff_t *ppos) {
	return 0;
}

static int sharedmem_ioctl(struct inode *inode, struct file *file,
						   unsigned int cmd, unsigned long arg) {
	return 0;
}

static long sharedmem_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {

	/* Will be called during 32 bit user space applications */
	int ret = 0;
	switch ( cmd ) {
	case DOMAIN_ID_OPTION:/* for writing data to arg */
		if (copy_from_user(&domain_id, (int *)arg, sizeof(int)))
			return -EFAULT;
#ifdef DEBUG
		printk ("%s: domain id is %d\n", __func__, domain_id);
#endif
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int sharedmem_mmap(struct file * filep, struct vm_area_struct * vma)
{
	int ref = 0, i, order;
	int minor = iminor (filep->f_dentry->d_inode);

	order = get_order (vma->vm_end - vma->vm_start);

	if (num_order [minor] && order > num_order [minor]) {
		printk ("Order %d > already allocated memory of order %d\n",
				order, num_order[minor]);
		return -EAGAIN;
	}

	if (!shared_memory_mfn[minor]) {
		int req_pages, ret;
		unsigned long mfn;
		int tot_pages, max_pages;

		tot_pages = xen_get_current_reservation (DOMID_SELF);
		max_pages = xen_get_maximum_reservation (DOMID_SELF);

		req_pages = (1 << order);
		ret = xen_increase_max_mem (DOMID, ((req_pages + max_pages) * PAGE_SIZE) >> 10);
		if (ret == 0) {
			max_pages = xen_get_maximum_reservation (DOMID_SELF);
			ret = xen_increase_reservation (DOMID_SELF, MAX_EXTENTS, order);
			if (ret != MAX_EXTENTS) {
				printk ("(%s): xen_increase_reservation failed with error = %d\n", __func__, ret);
				goto err;
			}

			tot_pages = xen_get_current_reservation (DOMID_SELF);
			max_pages = xen_get_maximum_reservation (DOMID_SELF);
			mfn = p2m_host[MAX_EXTENTS - 1];
			shared_memory_mfn[minor] = mfn;
			num_order[minor] = order;
			goto out;
		}
		else {
			printk ("(%s): xen_increase_max_mem failed with error = %d\n", __func__, ret);
			goto err;
		}
	err:
		/* Reduce max_mem to old value */
		xen_increase_max_mem (DOMID, (max_pages * PAGE_SIZE) >> 10);
		return ret;
	}
out:
	if (remap_pfn_range (vma, (unsigned long) vma->vm_start, shared_memory_mfn[minor],
						 1 << (order + PAGE_SHIFT), vma->vm_page_prot))	{
		printk ("%s mapping failed \n", __func__);
		return -EAGAIN;
	}
	if (!gnttab_update_done[domain_id][minor])
		for (i = 0; i < (1 << num_order [minor]); i++) {
			ref = gnttab_grant_foreign_access (domain_id, shared_memory_mfn [minor] + i, 0);
			if (ref < 0)
				printk ("%s Grant foreign access failed ref = %d for page index %d\n", __func__, ref, i);
			if (!i)
			{
				gnttab_update_service (domain_id, ref, 1 << num_order [minor],
						shared_memory_mfn [minor], NLM_SHAREDMEM_ID + minor);
				gnttab_update_done[domain_id][minor] = 1;
			}
		}
	return 0;
}

struct file_operations sharedmem_fops = {
	.owner	      =	THIS_MODULE,
	.read	      =	sharedmem_read,
	.write	      =	sharedmem_write,
	.mmap	      =	sharedmem_mmap,
	.ioctl	      =	sharedmem_ioctl,
	.compat_ioctl =	sharedmem_compat_ioctl,
	.open	      =	sharedmem_open,
	.release      =	sharedmem_release,
};

static int sharedmem_init (void)
{
	int ret;

	ret = register_chrdev (0, SHAREDMEM_NAME, &sharedmem_fops);
	if (ret < 0) {
		printk("[%s] Failed to register packet memory char device major=%d\n",
			   __FUNCTION__, 0);
		return -EIO;
	}
	sharedmem_major_num = ret;
	printk("[%s] Registered nlm_sharedmem char device major=%d\n",
		   __FUNCTION__, sharedmem_major_num);
	

	return 0;
}

static void sharedmem_cleanup (void)
{
	printk("cleaning up module\n");
	unregister_chrdev (sharedmem_major_num, SHAREDMEM_NAME);
}

module_init(sharedmem_init);
module_exit(sharedmem_cleanup);
MODULE_AUTHOR("Netlogic");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Shared Memory driver");
