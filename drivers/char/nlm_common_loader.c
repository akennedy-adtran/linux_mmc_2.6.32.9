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
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <asm/netlogic/devices.h>
#include <asm/netlogic/nlm_common_loader.h>
#include <user/netlogic/nlm_common_loader.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/hal/nlm_hal_pic.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/rwsem.h>
#define dbg_msg(fmt, args...) //printk(fmt,##args)

#define Message(fmt, args...) //printk("\n[%s]-[%d] "fmt"\n",__FUNCTION__,__LINE__,##args)

extern int xlr_lib_launch_userapp(int cpu);
extern void wakeup_cpu(unsigned int cpu, unsigned long fn, unsigned long args);
extern void xlr_send_stop_ipi(unsigned long mask);
extern volatile unsigned int xlr_lib_shmem_size;
static int nlm_common_loader_major;
static spinlock_t nlm_common_loader_lock;
static int nlm_common_loader_user = 0;
static int nlm_common_map_sh_mem=0;
static int nlm_common_map_persistent_mem = 0;
static int nlm_common_map_load_addr = 0;
static int nlm_common_map_app_shmem=0;
unsigned char *xlr_persistent_data_start=NULL;
unsigned char *xlr_persistent_data_start_orig=NULL;
static uint64_t xlr_app_shmem_start = 0x0;

unsigned int xlr_persistent_data_size=0;
/* Size of the shared memory b/w Linux userapp and rmios apps */
extern uint32_t nlm_common_app_sh_mem_sz;
extern unsigned long  nlm_common_app_shmem_start;

extern uint64_t nlm_common_loader_kuseg_size, nlm_common_loader_kuseg_start;
extern uint32_t nlm_common_loader_kseg_size, nlm_common_loader_kseg_start;
extern int xlr_loader_support;
extern uint32_t nlm_common_loader_mask;
extern void prom_check_image(void);
unsigned char load_env[32][6];
extern unsigned char *xlr_lib_shmem_start;

struct wakeup_info
{
	int vcpu;
    unsigned long long func;
    unsigned long long data;
};

struct xlr_load_addr
{
	uint64_t phys;
	uint64_t size;
	uint32_t flag;
}xlr_image_load_addr;


struct loader_send_ipi
{
	uint32_t mask;
	uint32_t ipi;
};

struct xlr_lib_shared_mem
{
	uint64_t entries;
	uint64_t tot_size;
	uint64_t addr[32];
	uint64_t size[32];
};
static struct xlr_lib_shared_mem app_shared_mem;

static int nlm_common_loader_open (struct inode *inode, struct file *filp)
{
	uint32_t minor=0;
	if(xlr_loader_support == 0) 
		return -EPERM;

	minor = MINOR(inode->i_rdev);	
	if(minor == XLR_MAP_SLAVE_DEVICE){
		filp->private_data = (void *)inode;
		return 0;
	}

	/* ALLOW ONLY ONE OPEN at a time */
	spin_lock(&nlm_common_loader_lock);
	if(nlm_common_loader_user == 1){
		spin_unlock(&nlm_common_loader_lock);
		return -EAGAIN;
	}
	filp->private_data = (void *)inode;
	nlm_common_loader_user = 1;
	nlm_common_map_load_addr = 0;
	nlm_common_map_persistent_mem = 0;
	nlm_common_map_sh_mem = 0;
	nlm_common_map_app_shmem = 0;

	spin_unlock(&nlm_common_loader_lock);


	return 0;
}

void nlm_common_loader_send_ipi(struct loader_send_ipi *data)
{
	uint32_t i;

	for(i=0; i<32; i++){
		if((data->mask) & (1<<i)){
#if defined(CONFIG_NLM_XLP)
			nlm_hal_pic_send_ipi(0, data->ipi, 0, i);
#else
			uint32_t pid, tid;
			uint32_t val;
			nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);

			pid = i >> 2;
			tid = i % 4;
			val = (pid << 20) | (tid << 16) | data->ipi;
			netlogic_write_reg(mmio, PIC_IPI, val);
#endif /* #if defined(CONFIG_NLM_XLP) */
		}
	}
}

int nlm_common_loader_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int *addr = (unsigned int*)arg;
	struct loader_send_ipi data;
	switch (cmd) {

		/*size of the data structures shared memory - this includes
		 all xlr_vcpu_wakeup_info and other lib data structure.*/
		case NLM_LOADER_IOC_SHMEM_SIZE: 
		{
			Message(" available shared mem [%#x]",xlr_lib_shmem_size);
			if(put_user(xlr_lib_shmem_size, addr)){
				panic("\nput user failed for ioc shmem size\n");
			}
		}
		break;

		/*Next mmap should be for global data structure - 
		 wakeup_info and other structures*/
		case NLM_LOADER_IOC_MMAP_SHMEM:
		{
			nlm_common_map_sh_mem = 1;
			nlm_common_map_persistent_mem = 0;
			nlm_common_map_load_addr = 0;
			nlm_common_map_app_shmem = 0;
		}
		break;

		/*Do an mmap of the passed physical address and size*/
		case NLM_LOADER_IOC_MMAP_LOAD_ADDR:
		{
			if(copy_from_user((void *)&xlr_image_load_addr,
								(const void __user *)addr,
								sizeof(struct xlr_load_addr)))
					panic("\nmmap load addr - copy frm user - failed\n");
			nlm_common_map_load_addr = 1;
			nlm_common_map_persistent_mem = 0;
			nlm_common_map_sh_mem = 0;
			nlm_common_map_app_shmem = 0;
		}
		break;

		/*Wakeup cpu*/
		case NLM_LOADER_IOC_START_IPI: 
		{
			struct wakeup_info tmp;
			if(copy_from_user((void *)&tmp, (const void __user *)addr,
								sizeof(struct wakeup_info)))
				panic("copy from user failed for START IPI\n");
			wakeup_cpu(tmp.vcpu, (unsigned long)tmp.func, 
									(unsigned long)tmp.data);
		}
		break;

		/*Stop cpu*/
		case NLM_LOADER_IOC_STOP_IPI:
		{
			uint32_t vcpu=0;
			if(get_user(vcpu, (uint32_t *)addr))
				panic("\nGetUser Failed for stop ipi\n");	
			/*Below Func Is Library Call.*/
			xlr_send_stop_ipi(vcpu);
		}
		break;

		/*Send ipi to the cpu*/
		case NLM_LOADER_SEND_IPI:
		{
			if(copy_from_user((void *)&data , (const void __user *)arg, 
							sizeof(data))){
					panic("\ncopy_from_user failed for send_ipi\n");
			}
			nlm_common_loader_send_ipi(&data);
		}
		break;

		/*Check if pereistent memory is present*/
		case NLM_LOADER_IOC_LIB_BKP:
		{
			if(xlr_persistent_data_start_orig){
				if(put_user(1, addr)){
					panic("\nput_user failed for lib_bkp\n");
				}
			}
			else{
				if(put_user(0, addr))
					panic("\nput_user failed for lib_bkp\n");
			}
		}
		break;

		/*Store shared memory info*/
		case NLM_LOADER_IOC_STORE_APP_SHMEM_INFO:
		{
			if(copy_from_user((void *)&app_shared_mem, (const void __user *)arg, 
							sizeof(app_shared_mem))){
				panic("copy from User failed for app shared mem info\n");
			}
			/*
			int i=0;
			for(i=0; i<app_shared_mem.entries; i++){
				printk("\nStoring Physical Addr %#llx, Size %#llx\n",
						(unsigned long long)app_shared_mem.addr[i], 
						(unsigned long long)app_shared_mem.size[i]);
			}
			*/
		}
		break;

		/*Retrive shared memory info*/
		case NLM_LOADER_IOC_GET_APP_SHMEM_INFO:
		{
			/*
			int i=0;
			for(i=0; i<app_shared_mem.entries; i++){
				printk("\nRetrive Physical Addr %#llx, Size %#llx\n",
						(unsigned long long)app_shared_mem.addr[i], 
						(unsigned long long)app_shared_mem.size[i]);
			}*/

			if(copy_to_user((void __user *)arg, (const void *)&app_shared_mem,
							sizeof(app_shared_mem))){
				panic("\ncopy_to_user failed for app shared mem info\n");
			}			
		}
		break;

#if 0
		/*Check if app shared memory is already reserved*/
		case NLM_LOADER_IOC_APP_SHMEM_RESERVE:
		{
			if(xlr_app_shmem_start){
				if(put_user(1, addr)){
					panic("\nput_user failed for lib_bkp\n");
				}
			}
			else{
				if(put_user(0, addr)){
					panic("\nput_user failed for lib_bkp\n");
				}
			}	
		}
		break;
		
		/*Get the app shared memory physical address start from user space*/
		case NLM_LOADER_IOC_APP_SHMEM_PHYS:
		{
				if(get_user(xlr_app_shmem_start, (uint64_t *)addr)){
					panic("\nput_user failed for lib_bkp\n");
				}
		}
		break;

		/*Get the app shared memory size*/
		case NLM_LOADER_IOC_APP_SHMEM_SIZE:
		{
			if(put_user(nlm_common_app_sh_mem_sz, addr)){
				panic("\nput user failed for ioc shmem size\n");
			}
		}
		break;

		/*mmap app shared memory in next mmap call*/
		case NLM_LOADER_IOC_MMAP_APP_SHMEM:
		{
			nlm_common_map_app_shmem = 1;
			nlm_common_map_persistent_mem = 0;
			nlm_common_map_sh_mem = 0;
			nlm_common_map_load_addr = 0;
		}
		break;
#endif	

		/*Get the kseg0 base of the data structure's shared memory
		  used bye mmap_to_kseg/kseg_to_mmap*/
		case NLM_LOADER_IOC_SHMEM_KSEG_ADDR:
		{
			if (put_user((long long)(int)(long)xlr_lib_shmem_start, (uint64_t *)addr)){
				panic("\nput user failed for ioc shmem kseg addr\n");
			}
		}
		break;
		
		
		/*Free persistent memory*/
		case NLM_LOADER_IOC_FREE_PERSISTENT_MEM:
		{
			unsigned long addr=0;
			if(!xlr_persistent_data_start_orig){
				printk("\nData Is Not Allocated.\n");
				return -EINVAL;
			}
			addr = (unsigned long)xlr_persistent_data_start;
			while(addr < (unsigned long)
						(xlr_persistent_data_start+xlr_persistent_data_size)){
                ClearPageReserved(virt_to_page((void *)addr));
                addr += PAGE_SIZE;	
			}
			kfree(xlr_persistent_data_start_orig);
			xlr_persistent_data_start_orig = xlr_persistent_data_start = NULL;
		}
		break;

		/*Alloc persistent memory*/
		case NLM_LOADER_IOC_ALLOC_PERSISTENT_MEM:
		{
			unsigned long tmp_addr=0;
			if(get_user(xlr_persistent_data_size,(unsigned int *)addr)){
				panic("\nGetUser Failed for alloc persistent mem\n");	
			}
			if(!xlr_persistent_data_size){
				printk("\nInvalid Len %#x\n", xlr_persistent_data_size);
				return -EINVAL;
			}
			xlr_persistent_data_start_orig = 
				kmalloc(xlr_persistent_data_size+PAGE_SIZE, GFP_KERNEL|GFP_DMA);
			if(!xlr_persistent_data_start_orig)
				return -ENOMEM;
			xlr_persistent_data_start = (unsigned char *)
						(((unsigned long)(xlr_persistent_data_start_orig + 
											PAGE_SIZE)) & ~(PAGE_SIZE-1));
			tmp_addr = (unsigned long)xlr_persistent_data_start;
			while(tmp_addr < (unsigned long)
						(xlr_persistent_data_start+xlr_persistent_data_size)){
                SetPageReserved(virt_to_page((void *)tmp_addr));
                tmp_addr += PAGE_SIZE;	
			}
		}
		break;

		/*mmap persistent memory in next mmap call.*/
		case NLM_LOADER_IOC_MMAP_PERSISTENT_MEM:
		{
			nlm_common_map_persistent_mem = 1;
			nlm_common_map_sh_mem = 0;
			nlm_common_map_load_addr = 0;
			nlm_common_map_app_shmem = 0;
		}
		break;

		/*call lib_launch.c's launch_userapp for kseg app*/
		case NLM_LOADER_IOC_LAUNCH_KSEG:
		{
				uint32_t vcpu=0;
				if(get_user(vcpu, (uint32_t *)addr))
						panic("\nGetUser Failed for launch kseg\n");
				/*Below Func Is Library Call.*/
				xlr_lib_launch_userapp(vcpu);
		}
		break;

		/*store UART info*/
		case NLM_LOADER_STORE_ENV:
		{
				int i;
				unsigned char temp[32][6];
				copy_from_user ((void *)&temp , (const void __user *)addr, sizeof(temp));

				for (i=0; i<32; i++)    {
						if (strcmp(temp[i],"") != 0)
								strcpy(load_env[i],temp[i]);
				}
		}
		break;
		default:
		{
			printk("ioctl(): invalid command=0x%x\n", cmd);
			return -EINVAL;
		}
	}
	return 0;
}

static long nlm_common_loader_compat_ioctl(struct file *filp, unsigned int cmd, 
		unsigned long arg)
{
	unsigned long ret = -1;
	Message("");
	lock_kernel();	
	Message("Got the user space address [%#lx]",arg);
	ret = nlm_common_loader_ioctl(NULL,filp,cmd,(uint32_t)arg);
	unlock_kernel();
	if(ret){
		printk("nlm_common_loader_ioctl returned with an error.");
		return -ENOIOCTLCMD;
	}
	return ret;
}

static int nlm_common_loader_mmap_app_shared_mem(struct file *file,
		struct vm_area_struct *vma)
{
	struct xlr_lib_shared_mem *shmem = &app_shared_mem;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long vaddr = 0;
	int i=0;
	int result = 0;
	unsigned long mmap_size = 0;
	unsigned long size = 0;

	for(i=0; i<shmem->entries; i++){
		if(shmem->addr[i] & (PAGE_SIZE-1)){
			printk("\nMmaping to invalid address ... %#llx\n",
						(unsigned long long)shmem->addr[i]);
			return -EINVAL;
		}
	}

	if (vma->vm_flags & VM_LOCKED) return -EPERM;

	if (offset >= shmem->tot_size) return -ESPIPE;
	
	pgprot_val (vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val (vma->vm_page_prot) |= _CACHE_CACHABLE_COW;

	vaddr = vma->vm_start;
	size = vma->vm_end - vma->vm_start;

	for(i=0; i<shmem->entries && size; i++){
	/*	printk("\nMapping vaddr = %#lx, Physical Addr %#llx, Size %#llx\n",
						(unsigned long)vaddr, 
						(unsigned long long)shmem->addr[i], 
						(unsigned long long)shmem->size[i]);
	*/
		mmap_size = (shmem->size[i] > size) ? size : shmem->size[i];
		result = remap_pfn_range(vma, vaddr, (shmem->addr[i])>>PAGE_SHIFT, 
					mmap_size, vma->vm_page_prot);
		if (result) return -EAGAIN;
		vaddr += shmem->size[i];
		size -= mmap_size;
	}
	return 0;
}

static int nlm_common_loader_map_helper(struct file *file, struct vm_area_struct *vma)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	uint64_t shm_addr;
	unsigned long shm_size;
	unsigned long size = 0;
	int result = 0;
	unsigned long shm_pfn_addr = 0;
	struct inode *i;
	uint32_t minor;
	i = (struct inode *)file->private_data;
	minor = MINOR(i->i_rdev);	
	if(minor == XLR_MAP_SLAVE_DEVICE){
		return nlm_common_loader_mmap_app_shared_mem(file, vma);
	}else if(nlm_common_map_sh_mem){
		shm_addr = (uint64_t)(49<<20);
		shm_size = xlr_lib_shmem_size; 
		shm_pfn_addr = ((uint64_t)shm_addr >> PAGE_SHIFT);
	}else if(nlm_common_map_persistent_mem){
		shm_addr = (uint64_t)virt_to_phys(xlr_persistent_data_start);
		shm_size = xlr_persistent_data_size;
		shm_pfn_addr = ((uint64_t)shm_addr >> PAGE_SHIFT);
	}else if(nlm_common_map_load_addr){
		shm_addr = xlr_image_load_addr.phys;
		shm_size = (unsigned long)xlr_image_load_addr.size;
		shm_pfn_addr = ((uint64_t)shm_addr >> PAGE_SHIFT);
	}else if(nlm_common_map_app_shmem){
		shm_addr = xlr_app_shmem_start;
		shm_size = nlm_common_app_sh_mem_sz;
		shm_pfn_addr = (unsigned long)((uint64_t)shm_addr >> PAGE_SHIFT);
	}else{
		printk("\nInvalid mmap command.\n");
		return -EINVAL;
	}
	dbg_msg("[%s]: shm_addr=%lx, shm_size=%lx, offset = %lx, vm_start=%lx, vm_size=%lx, vm_flags=%lx, vm_page_prot=%lx\n", __FUNCTION__, shm_addr, shm_size, offset, vma->vm_start, vm_size, vma->vm_flags, pgprot_val(vma->vm_page_prot));
	if (!shm_addr) return -ENXIO;

	if(shm_addr & (PAGE_SIZE-1)){
		printk("\nMmaping to invalid address ... %#llx\n",
						(unsigned long long)shm_addr);
		return -EINVAL;
	}
	if (offset >= shm_size) return -ESPIPE;

	if (vma->vm_flags & VM_LOCKED) return -EPERM;

	size = vma->vm_end - vma->vm_start;
	pgprot_val (vma->vm_page_prot) &= ~_CACHE_MASK;
	if(nlm_common_map_load_addr && (xlr_image_load_addr.flag == XLR_MAP_UNCACHED)){
		pgprot_val (vma->vm_page_prot) |= _CACHE_UNCACHED;
	}else{
		pgprot_val (vma->vm_page_prot) |= _CACHE_CACHABLE_COW;
	}
	result = remap_pfn_range(vma, vma->vm_start, shm_pfn_addr, size, 
								vma->vm_page_prot);
	if (result) return -EAGAIN;

	return 0;
}

static int nlm_common_loader_mmap(struct file *file, struct vm_area_struct *vma)
{
	int res;
	
	res = nlm_common_loader_map_helper(file, vma);
	nlm_common_map_app_shmem = 0;
	nlm_common_map_sh_mem = 0;
	nlm_common_map_persistent_mem = 0;
	nlm_common_map_load_addr = 0;
	return res;
}

static int nlm_common_loader_release (struct inode *inode, struct file *filp)
{
	return 0;
}

static int nlm_common_loader_flush(struct file *fp, fl_owner_t id)
{
	uint32_t minor=0;
	struct inode *inode = (struct inode *)(fp->private_data);
	minor = MINOR(inode->i_rdev);
	if(minor == XLR_MAP_SLAVE_DEVICE){
		return 0;
	}
	spin_lock(&nlm_common_loader_lock);
	nlm_common_loader_user = 0;
	spin_unlock(&nlm_common_loader_lock);
	return 0;

}

struct file_operations nlm_common_loader_fops = {
	.mmap     = nlm_common_loader_mmap,
	.open	  = nlm_common_loader_open,
	.ioctl    = nlm_common_loader_ioctl,
	.release  = nlm_common_loader_release,
	.flush = nlm_common_loader_flush,
	.compat_ioctl = nlm_common_loader_compat_ioctl,
};

static int nlm_common_loader_init(void)	
{
	if(xlr_loader_support == 0)
		return -EINVAL;

	spin_lock_init(&nlm_common_loader_lock);

	nlm_common_loader_major = register_chrdev (XLR_APP_LOADER_MAJOR, 
					     NLM_APP_LOADER_CHRDEV_NAME, &nlm_common_loader_fops);

	if (nlm_common_loader_major < 0) {
		printk("[%s]: register_chrdev failed\n", __FUNCTION__);
		return nlm_common_loader_major;
	}

	/* Launch the threads now */
/*
	nlm_common_start_loader_threads();

	printk("Registered xlr app loader driver: nlm_common_loader_major=%d\n", XLR_APP_LOADER_MAJOR);
*/
	return 0;
}

static void nlm_common_loader_exit(void)
{
	/*TODO: Clean up*/
    unregister_chrdev (nlm_common_loader_major, NLM_APP_LOADER_CHRDEV_NAME);
}

module_init (nlm_common_loader_init);
module_exit (nlm_common_loader_exit);

