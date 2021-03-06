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

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/pfn.h>
#include <linux/smp_lock.h>
#include <linux/fdtable.h>
#include <linux/pagemap.h>
#include <asm/mman.h>
#include <asm/io.h>
#include <asm/tlb.h>
#include <asm/pgtable.h>
#include <asm/atomic.h>
#include "nlm_cmem.h"

static int nlm_cmem_driver_major;
static unsigned long app_pagesize = 0;
static unsigned int app_pageorder = 0;

#define NLM_CMEM_DEV_NAME "nlm_cmem"

//#define Message printk
#define Message(x, args...) { }

//#define DBG_ALLOC 1
#ifdef DBG_ALLOC
#define DBG_NALLOCS_INC(x) atomic_inc(x);
#define DBG_NALLOCS_DEC(x) atomic_dec(x);
#else
#define DBG_NALLOCS_INC(x) 
#define DBG_NALLOCS_DEC(x) 
#endif

#define MMINFO_HASH_SIZE 256

struct file_priv_data {
#ifdef DBG_ALLOC
	atomic_t nallocs;
#endif
	struct hlist_head       mhash_head[MMINFO_HASH_SIZE];
	spinlock_t            	hlock[MMINFO_HASH_SIZE];

};

/* vinfo and pinfo keeps track of the pages */
struct pinfo {
	struct page *page; /* pages app-page-size aligned */
	spinlock_t lock; /* protect for this page */
};

struct vaddr_info {
	unsigned long start; /* Originally allocated start*/
	unsigned long end; /* Originally allocated end */
	struct file_priv_data *fpriv;
	struct hlist_node node; /* list of vaddr info */
	atomic_t refcnt;
	void *mminfo;
	struct pinfo pinfo[0]; /* page info */
};

/* This one keeps track of vaddr_info of one mm */
struct mm_info {
	 struct hlist_node mnode;
	 struct hlist_head vhead;
	 struct mm_struct *mm;
	 int refcnt; /* how may vaddress is refering */
	 int hash;
	 int status;
};

static int nlm_cmem_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

static inline unsigned long long calc_hash(unsigned long data)
{
#define CRC_FLIPBITS_BIT 4
#define CRC_DESTINATION_BIT 6
	unsigned long long result = 0xffffffffffffffffULL;
	static unsigned int flag = (0 << 8) | (1 << CRC_DESTINATION_BIT) | (1 << CRC_FLIPBITS_BIT) | 0x7;
	asm volatile (
			".set push\n"
			".set noreorder\n"
			"dcrc   %0, %1, %2\n"
			".set pop\n"
			:"=r"(result)
			: "r"(data), "r"(flag), "0"(result)
		     );
	return result;

}

/* should be called with lock held */
static inline struct mm_info *list_find_mminfo(struct file_priv_data *fpriv, struct mm_struct *mm, int hash)
{
	struct mm_info *tmp;
	struct hlist_node *node;
	hlist_for_each(node, &fpriv->mhash_head[hash]) {
		tmp = (struct mm_info *) ((unsigned long)node -
					((unsigned long)(&((struct mm_info *)0)->mnode)));
		if(tmp->mm == mm) {
			tmp->refcnt++;
			return tmp;
		}
	}
	return NULL;
}

static inline int list_add_vinfo(struct file_priv_data *fpriv, 
		struct vaddr_info *vinfo, struct mm_struct *mm)
{
	int hash ;
	struct mm_info *mminfo, *tmp = NULL;

	hash = calc_hash((unsigned long)mm) %  MMINFO_HASH_SIZE;

	spin_lock(&fpriv->hlock[hash]);
	mminfo = list_find_mminfo(fpriv, mm, hash);
	spin_unlock(&fpriv->hlock[hash]);
	if(!mminfo) {
		mminfo = kmalloc(sizeof(struct mm_info), GFP_KERNEL);
		if(!mminfo) {
			printk("Error : no mem in %s\n", __FUNCTION__);
			spin_unlock(&fpriv->hlock[hash]);
			return -ENOMEM;
		}
		INIT_HLIST_NODE(&mminfo->mnode);
		INIT_HLIST_HEAD(&mminfo->vhead);
		mminfo->mm = mm;
		mminfo->refcnt = 1;
		mminfo->hash = hash;
		mminfo->status = 0;
		/* check whether it is already added or not */
		spin_lock(&fpriv->hlock[hash]);
		if((tmp = list_find_mminfo(fpriv, mm, hash)) == NULL) {
			hlist_add_head(&mminfo->mnode, &fpriv->mhash_head[hash]);
			DBG_NALLOCS_INC(&fpriv->nallocs);
		} else {
			kfree(mminfo);
			mminfo = tmp;
		}
		spin_unlock(&fpriv->hlock[hash]);
	} 

	vinfo->mminfo = (void *)mminfo;

	spin_lock(&fpriv->hlock[hash]);
	hlist_add_head(&vinfo->node, &mminfo->vhead);
	spin_unlock(&fpriv->hlock[hash]);

	return 0;
}

static inline void list_del_vinfo(struct file_priv_data *fpriv, struct vaddr_info *vinfo)
{
	struct mm_info *mminfo = (struct mm_info *)vinfo->mminfo;
	int hash;
	if(!mminfo) {
		printk("Error : invalid pointer %s\n", __FUNCTION__);
		return;
	}
	hash = mminfo->hash;
	
	spin_lock(&fpriv->hlock[hash]);
	__hlist_del(&vinfo->node); 
	mminfo->refcnt--;
	if(mminfo->refcnt == 0)
		__hlist_del(&mminfo->mnode); 
	else
		mminfo = NULL;
	spin_unlock(&fpriv->hlock[hash]);
	if(mminfo) {
		kfree(mminfo);
		DBG_NALLOCS_DEC(&fpriv->nallocs);
	}
	kfree(vinfo);
	DBG_NALLOCS_DEC(&fpriv->nallocs);
}



static struct vaddr_info *create_new_vinfo(struct file_priv_data *fpriv, 
			unsigned long start, unsigned long end, struct mm_struct *mm)
{
	unsigned long size;
	unsigned int npages;
	struct vaddr_info *vinfo;

	npages = (end - start) / app_pagesize;
	size = sizeof(struct vaddr_info) + (npages * sizeof(struct pinfo));
	vinfo = kmalloc(size, GFP_KERNEL);
	if(!vinfo) {
		printk("Error : no mem in %s\n", __FUNCTION__);
		return NULL;
	}

	/* we require this info in the file list for virtual to physical mapping of this 
	 * region during fork for the child process 
	 */
	memset(vinfo, 0, size);
	vinfo->start = start;
	vinfo->end = end;
	vinfo->fpriv = fpriv;
	atomic_set(&vinfo->refcnt, 1) ;
	INIT_HLIST_NODE(&vinfo->node);
	if(list_add_vinfo(fpriv, vinfo, mm) != 0) {
		kfree(vinfo);
		return NULL;
	}
	DBG_NALLOCS_INC(&fpriv->nallocs); 
	return vinfo;
}

/* called from down_write mmap_sem , no need to take spinlock */
static void set_err_status(struct file_priv_data *fpriv, struct mm_struct *mm)
{
	int hash;
	struct mm_info *mminfo;

	hash = calc_hash((unsigned long)mm) %  MMINFO_HASH_SIZE;
	mminfo = list_find_mminfo(fpriv, mm, hash);
	if(mminfo) {
		/* mminfo refcount has been incremented */
		mminfo->refcnt--;
		mminfo->status = -ENOMEM;
	}
}

static int remap_pte_fn(pte_t *pte, struct page *pmd_page,
                      unsigned long addr, void *data)
{
        unsigned long long *args = (unsigned long long *)data;
	unsigned long long pfn = args[0];
	pte_t pval;
	

	pgprot_t prot;
	prot.pgprot = args[1];

	pval = pte_mkspecial(pfn_pte(pfn, prot));
	
	Message("%s addr %lx phys %llx\n", __FUNCTION__, addr, pfn << PAGE_SHIFT);
	set_pte_at(current->mm, addr, pte, pval);
	args[0]++;
        return 0;
}

static int unmap_pte_fn(pte_t *pte, struct page *pmd_page,
                        unsigned long addr, void *data)
{
	Message("%s addr %lx\n", __FUNCTION__, addr);
	set_pte_at(current->mm, addr, pte, __pte(0));
	return 0;
}

#if 0
static int write_protect_pte_fn(pte_t *pte, struct page *pmd_page,
                        unsigned long addr, void *data)
{
        unsigned long long *args = (unsigned long long *)data;
	unsigned long long pfn = args[0];
	/* mm is not used in the below function 
	struct mm_struct *mm = (struct mm_struct *)args[2];  */
	pgprot_t prot;
	unsigned long long elo = pte_to_entrylo(pte_val(*pte));

	prot.pgprot = args[1];

	Message("%s  in pte %lx pmd %lx addr %lx elo %llx paddr %llx write %d mod %d\n", __FUNCTION__, 
			(long)pte, (long)pmd_page, addr, elo, (elo >> 6) << 12, 
			pte_write(*pte), pte_dirty(*pte));

	set_pte_at(mm, addr, pte,  pte_wrprotect(pfn_pte(pfn, prot)));

	Message("%s  out pte %lx pmd %lx addr %lx elo %llx paddr %llx write %d mod %d\n", __FUNCTION__, 
			(long)pte, (long)pmd_page, addr, elo, (elo >> 6) << 12, 
			pte_write(*pte), pte_dirty(*pte));

	args[0]++;
	return 0;
}

static int dump_pte_fn(pte_t *pte, struct page *pmd_page,
                        unsigned long addr, void *data)
{
	unsigned long long elo = pte_to_entrylo(pte_val(*pte));
	Message("%s  pte %lx pmd %lx addr %lx elo %llx paddr %llx write %d mod %d\n", __FUNCTION__, 
			(long)pte, (long)pmd_page, addr, elo, (elo >> 6) << 12, 
			pte_write(*pte), pte_dirty(*pte));
	return 0;
}
#endif

	
/*
 * Called from madvise, Both start and end should be 
 * app-pagesize algined. va is still valid but not pa
 * As the application can coalasce the vaddress from multiple mmap
 * return values(if it is aligned properly) and can call the madvise(dont-need)
 * for the entire range. So we need to use find_vma for each app page size
 */
static inline int nlm_cmem_unmap_pinfo_entry(struct file_priv_data *fpriv, 
			unsigned long start, unsigned long end)
{
	struct vaddr_info *vinfo;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct * vma;
	unsigned int page_idx;
	int rv = -EINVAL;
	unsigned long e;
	struct page *page;

	down_read(&mm->mmap_sem);

	for(; start < end ; ) {

		if((vma = find_vma(mm, start)) == NULL) {
			printk("Error : Invalid start address %s\n", __FUNCTION__);
			goto err_exit;
		}

		/* apply till this vma limit */
		if(end >= vma->vm_end)
			e = vma->vm_end;
		else
			e = end;

		vinfo = vma->vm_private_data;
		if(!((start >= vinfo->start) && (e <= vinfo->end))) {
			printk("Error : Invalid range given %s\n", __FUNCTION__);
			goto err_exit;
		}

		page_idx = (start - vinfo->start) / app_pagesize;

		Message("%s , vs %lx ve %lx s %lx e %lx pageidx %d npages %d p %llx \n", __FUNCTION__, 
				vinfo->start, vinfo->end, start, e, page_idx, 
				(unsigned int)((e - start) / app_pagesize), 
				page_to_phys(vinfo->pinfo[page_idx].page));

		/*  invalidate the pte for this va by setting ptprot bits to 0  */
		rv = apply_to_page_range(mm, start, (e - start), unmap_pte_fn, NULL);
		if(rv) {
			printk("Error : Unmap pte pfn failed %s\n", __FUNCTION__);
			goto err_exit;
		}

		flush_tlb_range(vma, start, e);
	
		for(; start < e; start += app_pagesize, page_idx++) {
			/* If fault handler is accessing this page and allocated a new page for this index
			This should not be the case. Lock is taken just for protection of the page  
			 */
			spin_lock(&vinfo->pinfo[page_idx].lock);
			
			if((page = vinfo->pinfo[page_idx].page) == NULL) {
				spin_unlock(&vinfo->pinfo[page_idx].lock);
				continue;
			}
			vinfo->pinfo[page_idx].page = NULL;
			
			spin_unlock(&vinfo->pinfo[page_idx].lock);
			
			DBG_NALLOCS_DEC(&fpriv->nallocs); 
			free_pages((unsigned long)page_address(page), app_pageorder);
		}
	}
	rv = 0;
err_exit:
	up_read(&mm->mmap_sem);
	return rv;
}		

/* remapping the deleted pages, returns the physical address, One app-pagesize at a time
*/
static inline int nlm_cmem_remap_pinfo_entry(struct file_priv_data *fpriv, unsigned long start, unsigned long long *u_ptr)
{
	struct vaddr_info *vinfo;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct * vma;
	unsigned int page_idx;
	struct vm_fault vmf;
	int rv = -EINVAL;
	unsigned long long page_paddr;

	down_read(&mm->mmap_sem);

	if((vma = find_vma(mm, start)) == NULL) {
		printk("Error : Invalid start address %s\n", __FUNCTION__);
		goto err_exit;
	}

	/* start is already app_pagesize aligned */
	vinfo = vma->vm_private_data;
	page_idx = (start - vinfo->start) / app_pagesize;

	Message("%s , s %lx e %lx pageidx %d npages %d p %lx \n", __FUNCTION__, 
		vinfo->start, vinfo->end, page_idx, 1, (long)vinfo->pinfo[page_idx].page);

	vmf.virtual_address = (void __user *)(start & PAGE_MASK);

	rv = nlm_cmem_vma_fault(vma, &vmf);
	if(rv != VM_FAULT_NOPAGE) {
		rv = -ENOMEM;
		goto err_exit;
	}
		
	page_paddr = page_to_phys(vinfo->pinfo[page_idx].page);
	if(u_ptr)
		copy_to_user((u_ptr), &page_paddr, sizeof(page_paddr));		
	rv = 0;
err_exit:
	up_read(&mm->mmap_sem);
	return rv;
}


/* Called from the child fork post process handler*/
static int nlm_cmem_child_post_atfork_handler(struct file_priv_data *fpriv)
{


	int hash;
	struct mm_info *mminfo;
	struct mm_struct *mm = current->mm;

	Message("%s %d\n", __FUNCTION__, __LINE__);

	/* set by vma open, as the allocator uses internal memory for its database
	 mminfo should be present even if the application did not allocate 
	 any memory 
	 */
	hash = calc_hash((unsigned long)mm) %  MMINFO_HASH_SIZE;
	mminfo = list_find_mminfo(fpriv, mm, hash);
	if(mminfo) {
		/* mminfo refcount has been incremented */
		mminfo->refcnt--;
	       if(mminfo->status == 0)
			return 0;
	}
	return -ENOMEM;
}

static int nlm_cmem_copy_va_pa_info(struct file_priv_data *fpriv, unsigned long start,
		unsigned int nentries, 	unsigned long long *ptr)
{
	unsigned long vas, vae, e;
	unsigned long long pa, tmp;
	unsigned int off, entries = 0;
	struct hlist_node *node;
	struct vaddr_info *vinfo;
	struct vm_area_struct *vma;
	struct mm_struct *mm = current->mm;
	int hash;
	struct mm_info *mminfo;

	hash = calc_hash((unsigned long)mm) %  MMINFO_HASH_SIZE;
	
	down_read(&mm->mmap_sem);

	/* Now we dont need to take the hash lock, as all the vinfo addition and deletion 
	   takes write mmap_sem(mmap, vma_open, vma_close) , 
	 We already taken the read lock */

	mminfo = list_find_mminfo(fpriv, mm, hash);
	if(!mminfo) {
		goto end;
	} else {
		/* mminfo refcount has been incremented */
		mminfo->refcnt--;
	}
	
	hlist_for_each(node, &mminfo->vhead) {
		vinfo = (struct vaddr_info *) ((unsigned long)node -
				((unsigned long)(&((struct vaddr_info *)0)->node)));
		if(start) {
			if(!(start >= vinfo->start && start <= vinfo->end))
				continue;
			vas = start;
		} else
			vas = vinfo->start;
		vae = vinfo->end;
		/* for 2nd loop */
		start = 0;

		for(; vas < vae;) {
			if((vma = find_vma(mm, vas)) == NULL) {
				vas += app_pagesize;
				continue;
			}
			/* apply till this vma limit */
			if(vae >= vma->vm_end)
				e = vma->vm_end;
			else
				e = vae;

			vinfo = vma->vm_private_data;
			off = (vas - vinfo->start) / app_pagesize;
			for(; vas < e; vas += app_pagesize, off++) {

				if(!vinfo->pinfo[off].page)
					pa = 0;
				else
					pa = page_to_phys(vinfo->pinfo[off].page);
				tmp = vas;
				copy_to_user((ptr), &tmp, sizeof(tmp));		
				ptr++;
				copy_to_user((ptr), &pa, sizeof(pa));		
				ptr++;
				entries++;
				if(entries >= nentries)  {
					goto end;
				}
			}
		}
	}
end:
	up_read(&mm->mmap_sem);
	return entries;
}

static int nlm_cmem_driver_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		   unsigned long arg)
{
	int err = 0;
	struct file_priv_data *fpriv =  filp->private_data;
	unsigned long long *ptr = (unsigned long long *) arg;
	unsigned long long start, nentries, end, tmp;

	Message("%s %d, cmd %x, type %d size %d num %d\n", 
		__FUNCTION__, __LINE__, cmd, _IOC_TYPE(cmd), _IOC_SIZE(cmd), _IOC_NR(cmd));

	switch (cmd) {
		case NLM_CMEM_SET_APP_PAGE_SIZE: 
		{
			/* Different applications sharing this driver cannot have different app pagesize,
			 * Ignore the request if it is already set */
			if(app_pagesize)
				break;

			copy_from_user(&tmp, ptr, sizeof(*ptr));
			app_pagesize = tmp;
			if(app_pagesize < PAGE_SIZE) {
				printk("%s, Error : App page size is very low\n", __FUNCTION__); 
				return -EINVAL;
			}

			app_pageorder = get_order(app_pagesize);
			Message("App page size %lx order %d\n", app_pagesize, app_pageorder);
			break;
		}
		case NLM_CMEM_MADVISE_DONT_NEED:
		{
			copy_from_user(&start, ptr, sizeof(*ptr));
			copy_from_user(&end, ptr + 1, sizeof(*ptr));
			err = nlm_cmem_unmap_pinfo_entry(fpriv, start, end);
			break;

		}

		case NLM_CMEM_MADVISE_NEED:
		{
			copy_from_user(&start, ptr, sizeof(*ptr));
			err = nlm_cmem_remap_pinfo_entry(fpriv, start, ptr);
			break;

		}
		case NLM_CMEM_POST_CHILD_ATFORK:
		{
			err = nlm_cmem_child_post_atfork_handler(fpriv);
			break;

		}

		case NLM_CMEM_GET_VA_PA_INFO:
		{
			copy_from_user(&start, ptr, sizeof(*ptr));
			copy_from_user(&nentries, ptr + 1, sizeof(*ptr));
			err = nlm_cmem_copy_va_pa_info(fpriv, start, nentries, ptr);
			break;
		}
		default:
			printk("Invalid cmd in %s\n", __FUNCTION__);
			err = -EINVAL;

	}

	return err;
}

static long nlm_cmem_driver_compat_ioctl(struct file *filp, unsigned int cmd,
	       			unsigned long arg)
{
	Message("%s %d\n", __FUNCTION__, __LINE__);
	return 0;
}

/*
 * Open can be after a atfork handler or it might be from a new session 
 */

static int nlm_cmem_driver_open(struct inode *inode, struct file *filp)
{
	struct file_priv_data *fpriv __attribute__((unused));
	filp->private_data = kmalloc(sizeof(struct file_priv_data), GFP_KERNEL);
	if(filp->private_data == NULL) {
		printk("Error : no mem in %s\n", __FUNCTION__);
		return -ENOMEM;
	}
	memset(filp->private_data, 0, sizeof(struct file_priv_data));
	fpriv = filp->private_data;
	DBG_NALLOCS_INC(&fpriv->nallocs); 
	return 0;
}



static int nlm_cmem_driver_release(struct inode *inode, struct file *filp)
{
	struct file_priv_data *fpriv = filp->private_data;
	if(fpriv) {
		DBG_NALLOCS_DEC(&fpriv->nallocs); 
		#ifdef DBG_ALLOC
		printk("%s %d (nallocs %d)\n", __FUNCTION__, __LINE__, atomic_read(&fpriv->nallocs));
		#endif
		kfree(fpriv);
		filp->private_data = NULL;
	}
	return 0;
}

/*
 * Called when a device mapping is created by a means other than mmap (via fork, munmap, etc.). 
 * Called with write mmap_sem held
*/
static void nlm_cmem_vma_open(struct vm_area_struct *vma)
{
	struct vaddr_info *vinfo, *nvinfo;
	unsigned long start = vma->vm_start;
	unsigned long end = vma->vm_end;
	struct file_priv_data *fpriv;
	unsigned int i = 0, off = 0;
	int rv;

	vinfo = vma->vm_private_data;
	fpriv = vinfo->fpriv;

	Message("%s %d vma %lx start %lx end %lx, pid %d\n", 
	__FUNCTION__, __LINE__, (long)vma, vma->vm_start, vma->vm_end, current->pid);

	/* Ignore if the entry does not match. This will be the case when the library
 	* munmap the unaligned portions after mmap. 
 	* If it matches, it should be by fork or munmap which can cause split_vma. 
 	*/

	/* Also we allocate physical pages for the aligned start end address only.*/
	start = (start + app_pagesize - 1) & ~(app_pagesize - 1);
	end = end &  ~(app_pagesize - 1);
	if((start < vma->vm_start) || (start >= end)) 
		return;

	if(!(start >= vinfo->start && end <= vinfo->end)) 
		return;


	/* The call will be by fork if the current vm does not match with the vma vm. 
	 *  During munmap, split vma can happen, and the kernel calls vma open for the 
	 *  new vinfo. We can ignore this and do the required operation on the immediate
	 *  vma close */
	if(vma->vm_mm == current->mm) {
		atomic_inc(&vinfo->refcnt);
		return; 
	}

	/* if memory is not available, we will continue the fork process, 
	 * later the post atfork handler can return error, and the application
	 * can exit the process */
	nvinfo = create_new_vinfo(fpriv, start, end, vma->vm_mm);
	vma->vm_private_data = nvinfo;
	if(!nvinfo) {
		set_err_status(fpriv, vma->vm_mm);
		return;
	}

	/* find the offset of the original vinfo */
	off = (start - vinfo->start) / app_pagesize;

	for(i = 0; start < end; start += app_pagesize, i++, off++) {
		nvinfo->pinfo[i].page = vinfo->pinfo[off].page;
		if(vinfo->pinfo[off].page)  {
			/* increment the refcount, and mark the pages as non-writable */
			get_page(vinfo->pinfo[off].page);

			rv = apply_to_page_range(current->mm, start, app_pagesize, unmap_pte_fn, NULL);
			if(rv) {
				printk("Error : Write protect pte pfn failed %s\n", __FUNCTION__);
				put_page(vinfo->pinfo[off].page);
				set_err_status(fpriv, vma->vm_mm);
				return;
			}
			
			rv = apply_to_page_range(vma->vm_mm, start, app_pagesize, unmap_pte_fn, NULL);
			if(rv) {
				printk("Error : Write protect pte pfn failed %s\n", __FUNCTION__);
				put_page(vinfo->pinfo[off].page);
				set_err_status(fpriv, vma->vm_mm);
				return;
			}
			DBG_NALLOCS_INC(&fpriv->nallocs);/* for get page */

		} else  {
			rv = apply_to_page_range(vma->vm_mm, start, app_pagesize, unmap_pte_fn, NULL);
			if(rv) {
				printk("Error : Unmap pte pfn failed %s\n", __FUNCTION__);
				set_err_status(fpriv, vma->vm_mm);
				return;
			}
		}
	}
	return;
}
/* Close vma function 
 * Called with write mmap_sem held
 */
static void nlm_cmem_vma_close(struct vm_area_struct *vma)
{
	struct vaddr_info *vinfo;
	unsigned long start = vma->vm_start;
	unsigned long end = vma->vm_end;
	struct file_priv_data *fpriv;
	unsigned int off = 0;

	vinfo = vma->vm_private_data;
	fpriv = vinfo->fpriv;

	Message("%s %d vma %lx start %lx end %lx pid %d\n", 
		__FUNCTION__, __LINE__, (long)vma, vma->vm_start, vma->vm_end, current->pid);

	/* Ignore if the entry does not match. This will be the case when the library
 	* munmap the unaligned portions after mmap. 
 	* If it matches, it should be by the process exit handler or dueto split vma caused by munmap
 	*/

	/* Also we allocate physical pages for the aligned start end address only.*/
	start = (start + app_pagesize - 1) & ~(app_pagesize - 1);
	end = end &  ~(app_pagesize - 1);
	if((start < vma->vm_start) || (start >= end)) 
		return;

	if(!(start >= vinfo->start && end <= vinfo->end)) 
		return;

	off = (start - vinfo->start) / app_pagesize;
	for(; start < end; start += app_pagesize, off++) {
		if(!vinfo->pinfo[off].page)
			continue;
		Message("Deleting pages s %lx page %llx\n", start, page_to_phys(vinfo->pinfo[off].page));
		free_pages((unsigned long)page_address(vinfo->pinfo[off].page), 
				app_pageorder);
		vinfo->pinfo[off].page = NULL;
		DBG_NALLOCS_DEC(&fpriv->nallocs);


	}
	if (!atomic_dec_and_test(&vinfo->refcnt))
                return;

	Message("Deleting the vinfo\n");
	list_del_vinfo(fpriv, vinfo);
}

/*
 * Creates a  page and maps it to user space.
 * Called with mmap_sem read held, to allow multiple faults in the same vma.
 * So we need to protect the allocation.
 */
static int nlm_cmem_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	unsigned long vaddr = (unsigned long)vmf->virtual_address;
	struct vaddr_info *vinfo = vma->vm_private_data;
	unsigned int off;
	struct page *page;
	unsigned long long page_paddr, args[2];
	struct mm_struct *mm = current->mm;
	unsigned long start;
#ifdef DBG_ALLOC
	struct file_priv_data *fpriv = vinfo->fpriv;
#endif
	void *src, *dst;
	int rv;


	Message("%s %d vma %lx start %lx end %lx vaddr %lx\n", 
		__FUNCTION__, __LINE__, (long)vma, vma->vm_start, vma->vm_end, vaddr);

	/* Also we allocate physical pages for the aligned start address only.*/
	start = vaddr &  ~(app_pagesize - 1);
	if(start < vma->vm_start) 
		return VM_FAULT_ERROR;

	if(!(start >= vinfo->start && start <= vinfo->end)) 
		return VM_FAULT_ERROR;

	off = (start - vinfo->start) / app_pagesize;

	/* if it is already allocated by the madvise-need ioctl call. 
	   This page can be freed during this time if madvise-no-need gets called for the same memory(error cases, 
	   for example cpu is accessing a invalid memory which is going to be freed soon ). 
	   So we need to take lock till we done with the page
	 */
	spin_lock(&vinfo->pinfo[off].lock);
	if((page = vinfo->pinfo[off].page) == NULL)  {
		Message("%s %d page null\n", __FUNCTION__, __LINE__);
		page = alloc_pages(GFP_KERNEL, app_pageorder);
		if(!page){
			printk("Error : Couldn't allocate pages for order %d!! \n", app_pageorder);
			rv = VM_FAULT_OOM;
			goto err_exit;
		}

		vinfo->pinfo[off].page = page;
		DBG_NALLOCS_INC(&fpriv->nallocs); 
	} else {
		Message("%s %d page non-null pagecnt %d, pid %d\n", __FUNCTION__, __LINE__, page_count(page), current->pid);

		/* if more than 1 referenced it */
		if(page_count(page) > 1) {
			page = alloc_pages(GFP_KERNEL, app_pageorder);
			if(!page){
				printk("Error : Couldn't allocate pages for order %d!! \n", app_pageorder);
				rv = VM_FAULT_OOM;
				goto err_exit;
			}
			
			src = page_address(vinfo->pinfo[off].page);
			dst = page_address(page);
			memcpy(dst, src, app_pagesize);
			/* free_pages will just decrement the refcount and free the pages only if the refcnt = 0 */	
			free_pages((unsigned long)page_address(vinfo->pinfo[off].page), app_pageorder);
			Message("Copying the pages src %lx dst %lx\n", (unsigned long)src, (unsigned long)dst);
		}
		vinfo->pinfo[off].page = page;
	}

	page_paddr = page_to_phys(page);

	/*  remap the pte for this va by setting ptprot bits  */
	args[0] = page_paddr >> PAGE_SHIFT;
	args[1] = vma->vm_page_prot.pgprot;
	rv = apply_to_page_range(mm, start, app_pagesize, remap_pte_fn, args);
	if(rv) {
		printk("Error : Remap pte pfn failed %s\n", __FUNCTION__);
		rv = VM_FAULT_ERROR;
		goto err_exit;
	}

	flush_tlb_range(vma, start, start + app_pagesize); 
	rv =  VM_FAULT_NOPAGE;
err_exit:
	spin_unlock(&vinfo->pinfo[off].lock);
	return rv;
}
	

static const struct vm_operations_struct nlm_cmem_vm_ops = 
{
        .open = nlm_cmem_vma_open,
        .close = nlm_cmem_vma_close,
	.fault = nlm_cmem_vma_fault
};


static int nlm_cmem_driver_mmap(struct file * file, struct vm_area_struct * vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size  = vma->vm_end - vma->vm_start;
	unsigned long end = vma->vm_end;
	struct file_priv_data *fpriv =  file->private_data;
	unsigned int idx;
	struct vaddr_info *vinfo;

	Message("\n%s %d vma %lx start %lx size %lx\n", __FUNCTION__, __LINE__, (long)vma, start, size);
	/* This call guarantees physically contigous pages for the app_pagesize. 
 	 * The mmap call should always ask for more virtual address spaces depending
 	 * on the kernel-page-size and app-page-size and munmap the extra portions
 	 * so that it can return app_pagesize aligned virtual address.
 	 */ 
	if(app_pagesize < PAGE_SIZE) {
		printk("%s, Error : App page size not set\n", __FUNCTION__);
		return -EINVAL;
	}

	if((size < app_pagesize) || (size % app_pagesize)) {
		printk("%s, Error : Invalid size %ld (app_pagesize %lx)\n",
					__FUNCTION__, size, app_pagesize );
		return -EINVAL;
	}

	/* If the kernel-pagesize < app_pagesize, the start address may not be app_pagesize
 	 * aligned. In that case we allocate physical memory for only from the 
 	 * aligned address. The library should munmap the front portion */
	start = (start + app_pagesize - 1) & ~(app_pagesize - 1);

	/* Also we allocate physical pages for the aligned end address only.*/
	end = end &  ~(app_pagesize - 1);

	if(start >= end) {
		printk("%s, Error : Invalid size after alignment (start %lx end %lx)\n",
				 __FUNCTION__, start, end);
		return -EINVAL;
	}

	/*
	   special vmas that are non-mergable, non-mlock()able
	   VM_SPECIAL = (VM_IO | VM_DONTEXPAND | VM_RESERVED | VM_PFNMAP)
	*/
	vma->vm_flags |= VM_SPECIAL;
	vma->vm_ops = &nlm_cmem_vm_ops;

	/* Allocation is always in multiple chunks of app_pagesize, so that during munmap/madvise
 	 * we can delete the partial memory.  
 	 */
	vinfo = create_new_vinfo(fpriv, start, end, current->mm);
	if(!vinfo)
		return -ENOMEM;

	/* save it in the vma private data also */
	vma->vm_private_data = vinfo;

	Message("%s %d allocating pages for start %lx end %lx, prot %llx\n", 
			__FUNCTION__, __LINE__, start, end, (unsigned long long)vma->vm_page_prot.pgprot);

	for(idx = 0; start < end; start += app_pagesize, idx++) {
		struct page *page;
		unsigned long long page_paddr;
		page = alloc_pages(GFP_KERNEL, app_pageorder);
		if(!page){
			printk("Error : Couldn't allocate pages for order %d!! \n", app_pageorder);
			return -ENOMEM;
		}
		vinfo->pinfo[idx].page = page;
		page_paddr = page_to_phys(page);
		DBG_NALLOCS_INC(&fpriv->nallocs);

		Message("%s %d s %lx p %llx\n", __FUNCTION__, __LINE__, start, page_paddr);

		if(remap_pfn_range(vma, start, page_paddr >> PAGE_SHIFT, 
				app_pagesize, vma->vm_page_prot))
		return -EAGAIN;
	}
	
	return 0;
}

struct file_operations nlm_cmem_driver_fops = {
	.open = nlm_cmem_driver_open,
	.ioctl = nlm_cmem_driver_ioctl,
	.compat_ioctl = nlm_cmem_driver_compat_ioctl,
	.mmap = nlm_cmem_driver_mmap,
	.release = nlm_cmem_driver_release
};

static int __init nlm_cmem_driver_init(void)
{
	nlm_cmem_driver_major = register_chrdev(0, NLM_CMEM_DEV_NAME, &nlm_cmem_driver_fops);
	if (nlm_cmem_driver_major < 0) {
		printk("[%s]: Register chrdev failed\n", __FUNCTION__);
		return nlm_cmem_driver_major;
	}
	return 0;
}

static void __exit nlm_cmem_driver_exit(void)
{
	unregister_chrdev(nlm_cmem_driver_major, NLM_CMEM_DEV_NAME);
	return;
}

module_init(nlm_cmem_driver_init);
module_exit(nlm_cmem_driver_exit);

MODULE_AUTHOR("Netlogic Microsystems");
MODULE_DESCRIPTION("Netlogic XLP Physically Contiguous Mem allocator ");
MODULE_LICENSE("GPL");
