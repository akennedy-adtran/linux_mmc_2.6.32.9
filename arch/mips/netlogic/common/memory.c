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


#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/irqflags.h>
#include <asm/mipsregs.h>
#include <asm/page.h>
#include <asm/mach-netlogic/mmu.h>

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
#include <linux/bootmem.h>
#include <asm/pgtable.h>
#endif

//#define DEBUG_TLB

/*
 * the following structures and definitions are internal to this
 * file and hence not defined in a header file
 */
typedef struct
{
	unsigned int size;
	unsigned int mask;
} tlbparam_t;

tlbparam_t mipstlbs[] = 
{ {  4 << 10,    0x0},
  { 16 << 10,    0x3},
  { 64 << 10,    0xf},
  {256 << 10,   0x3f},
  {  1 << 20,   0xff},
  {  4 << 20,  0x3ff},
  { 16 << 20,  0xfff},
  { 64 << 20, 0x3fff},
  {256 << 20, 0xffff},
};

#define NTLB (sizeof(mipstlbs)/sizeof(tlbparam_t))
#define PCIDEV_ADDRSPACE_START 0xC0000000	// 3072MB

static uint32_t align_size(uint32_t size)
{
	int i;

	for (i = 0; (i < NTLB - 1) && (size > mipstlbs[i].size); ++i)
		;
	return mipstlbs[i].size;
}

static uint32_t tlb_mask(uint32_t size)
{
	int i;

	size = align_size(size);

	for (i = 0; i < NTLB && mipstlbs[i].size != size; ++i)
		;
	return mipstlbs[i].mask;
}

#ifdef CONFIG_NLM_16G_MEM_SUPPORT
#define page_entrylo(paddr, attr) ((((paddr) >> 12) << 6) | (attr))

/*
 * External Function / APIs
 */

static void setup_tlb(tlb_entry_t *tlb, unsigned long pagesize)
{
	write_c0_pagemask(tlb_mask(pagesize) << 13);
	write_c0_entryhi(tlb->entryHi & ~0x1fff);
	write_c0_entrylo0(tlb->entrylo0);
	write_c0_entrylo1(tlb->entrylo1);

	if (tlb->wired) {
		write_c0_index(read_c0_wired());
		tlb_write_indexed();
		write_c0_wired(read_c0_wired() + 1);
	}
	else {
		tlb_write_random();
	}
}
#else /* ! CONFIG_NLM_16G_MEM_SUPPORT */
#define entrylo(paddr, attr) \
	((((paddr & 0xffffffffffULL) >> 12) << 6) | (attr))

/*
 * External Function / APIs
 */

void setup_tlb(tlb_info_t *tlb)
{
	write_c0_pagemask(tlb_mask(tlb->pagesize) << 13);
	write_c0_entryhi(tlb->vaddr & ~0x1fff);
	write_c0_entrylo0(entrylo(tlb->paddr0, tlb->attr0));
	write_c0_entrylo1(entrylo(tlb->paddr1, tlb->attr1));

	if (tlb->wired) {
		write_c0_index(read_c0_wired());
		tlb_write_indexed();
		write_c0_wired(read_c0_wired() + 1);
	}
	else {
		tlb_write_random();
	}
}
#endif /* CONFIG_NLM_16G_MEM_SUPPORT */

#ifdef CONFIG_MAPPED_KERNEL

/*
 * the following initialization is needed for 32-bit
 * mapped kernels. It must be set 512 MB past the
 * mapped start kseg2 address (0xc0000000).
 * 0xc0000000 + 0x20000000(512MB) = 0xe0000000
 */
unsigned long __vmalloc_start = 0xe0000000;

#endif

#if defined(CONFIG_KSEG2_LOWMEM) && defined(CONFIG_KSEG2_LOWMEM)

#include <asm/barrier.h>

static volatile int max_low_pfn_set = 0;
extern unsigned long max_low_pfn;

#ifdef CONFIG_NLM_16G_MEM_SUPPORT

#ifdef CONFIG_64BIT
#define KERNEL_SEG_START	XKSEG	
#else
#define KERNEL_SEG_START	KSEG2
#endif

#define MIN(x,y)  ((x) < (y) ? (x) : (y))

#undef alloc_bootmem_low

static unsigned long
alloc_bootmem_low(gfp_t gfp_mask, unsigned int order)
{
	return (unsigned long)__alloc_bootmem_low((1 << order) * PAGE_SIZE, SMP_CACHE_BYTES, 0);
}

unsigned long NONWIRED_START = ~0x0;
unsigned long NONWIRED_END = ~0x0;

void setup_mapped_kernel_tlbs(int firstpage, int primary_cpu)
{
	tlb_entry_t tlb;
	unsigned long max_wired_size;
	unsigned long pagesize;
	unsigned long vaddr, paddr;
	unsigned short attr;

	pagesize = LARGEST_TLBPAGE_SZ; /* we set up the largest pages */

	/*
	 * In NetLogic's Linux kernel, the second 256MB of physical
	 * address space is reserved for device configuration and
	 * is not mapped to DRAM (to imply memory as opposed to IO
	 * device space). Hence the attribute of the second part of
	 * the first wired entry is invalid, while the both part of
	 * other wired entries are symmetric. We handle the above
	 * difference through the following unseemly if condition
	 */
	if (firstpage) {
		tlb.entryHi = KERNEL_SEG_START;

		attr = (KERNEL_PAGE_ATTR >> ENTRYLO_PFN_SHIFT);
		tlb.entrylo0 = page_entrylo(0, attr); /* we start at pfn = 0 */

		/* 256MB - 512 MB is IO config (invalid dram) */
		attr = (_PAGE_GLOBAL >> ENTRYLO_PFN_SHIFT);
		tlb.entrylo1 = page_entrylo(0, attr);

		tlb.wired = TRUE;
		setup_tlb(&tlb, pagesize);
	}
	else {
#ifdef CONFIG_NUMA
		int node;
		extern struct nlm_node_mem_info node_mem_info[];
#endif
		/*
		 * the primary cpu reads the memory map and records
		 * the highest page frame number. Secondary cpus
		 * must wait till the variable max_low_pfn is set
		 */
		if (!primary_cpu)
			while (!max_low_pfn_set)
				;
		vaddr = KERNEL_SEG_START + 2 * LARGEST_TLBPAGE_SZ;
		max_wired_size = PFN_PHYS(MIN(max_low_pfn, MAX_WIRED_PFN));
		attr = (KERNEL_PAGE_ATTR >> ENTRYLO_PFN_SHIFT);

		/*
		 * the following loop assumes that pagesize is set to
		 * 256 MB. change the logic if the pagesize ever changes
		 */
		paddr = 2 * LARGEST_TLBPAGE_SZ;
		for (; paddr < max_wired_size;
				paddr += 2 * pagesize, vaddr += 2 * pagesize) {
#ifdef DEBUG_TLB
			printk("(wired entry): vaddr = 0x%lx, paddr = 0x%lx\n", vaddr, paddr);
#endif
			/* Skip 3 - 3.5GB range (PCI device space) */
			if (paddr == PCIDEV_ADDRSPACE_START)
				continue;
			tlb.entryHi = vaddr;
			tlb.entrylo0 = page_entrylo(paddr, attr);
			tlb.entrylo1 = page_entrylo(paddr + pagesize, attr);
			tlb.wired = TRUE;
			setup_tlb(&tlb, pagesize);
		}

#ifdef CONFIG_NUMA
		/* For NUMA, for each node, we have to wire the minimum physical page for that node.
		 * NUMA uses that piece of memory immediately to keep its internal data structure.
		 *
		 * For node 0: the first chunk of memory above the MAX_DMA_ADDRESS needs to be wired
		 * as the kernel will allocate space in that area before the exception handlers etc.
		 * are setup.
		 *
		 * For other nodes, the first chunk of memory available on that node needs to be wired.
		 */
		for (node = 0; node < NLM_MAX_CPU_NODE; node ++) {
			paddr = PFN_PHYS(node_mem_info[node].min_start_pfn);
			if (paddr == 0)
				continue;

			/* for node 0, wire extra lines only if it is not wired yet */
			if (node == 0 && node_mem_info[0].min_start_pfn < MAX_WIRED_PFN)
				continue;

			/* make sure the paddr is aligned also */
			if (paddr % ( 2 * LARGEST_TLBPAGE_SZ))
				paddr -= (paddr % (2 * LARGEST_TLBPAGE_SZ));

			vaddr = KERNEL_SEG_START + paddr;
			tlb.entryHi = vaddr;
			tlb.entrylo0 = page_entrylo(paddr, attr);
			tlb.entrylo1 = page_entrylo(paddr + pagesize, attr);
			tlb.wired = TRUE;
			setup_tlb(&tlb, pagesize);
		}
#endif
	}
}

void setup_mapped_kernel_pgtable(void)
{
	unsigned long vaddr = KERNEL_SEG_START + PFN_PHYS(MIN(max_low_pfn, MAX_WIRED_PFN));
	int retval;

	if (max_low_pfn > MAX_WIRED_PFN) {
		__get_free_pages = alloc_bootmem_low;
		retval = map_kernel_addrspace(vaddr, MAX_WIRED_PFN, max_low_pfn);
		if (retval != 0)
			panic("unable to map kernel addrspace\n");
		NONWIRED_START = vaddr;
		NONWIRED_END = vaddr + (PFN_PHYS(max_low_pfn - MAX_WIRED_PFN));
		__get_free_pages = ____get_free_pages;
	}
#ifdef CONFIG_64BIT
	__vmalloc_start = KERNEL_SEG_START + (1UL << PGDIR_SHIFT);
#else
	__vmalloc_start = vaddr;
#endif
	return;
}

#define FLOOR(addr, alignment) ((addr) & ~((alignment) - 1)) /* alignment must be power of 2 */

unsigned long recalculate_max_low_pfn(unsigned long max_low_pfn)
{
	/* 
	 * truncate max_low_pfn to 512MB boundary as largest tlb
	 * pages are used to minimize the number of wired entries
	 */
	if ((max_low_pfn > PFN_DOWN(LARGEST_TLBPAGE_SZ << 1)) && (max_low_pfn <= MAX_WIRED_PFN))
		max_low_pfn = PFN_DOWN(FLOOR(PFN_PHYS(max_low_pfn), LARGEST_TLBPAGE_SZ << 1));
	max_low_pfn_set = TRUE;
	__sync();

#ifdef DEBUG_TLB
		printk("max_low_pfn = 0x%lx -> max memory address = 0x%llx\n",
				max_low_pfn, ((unsigned long long)(max_low_pfn + 1) << PAGE_SHIFT) - 1);
#endif

	return max_low_pfn;
}

#else /* !CONFIG_NLM_16G_MEM_SUPPORT */

#ifdef CONFIG_64BIT
#define TLB_VADDR		XKSEG	
#else
#define TLB_VADDR		KSEG2
#endif

void tlb_printk(tlb_info_t *tlb, int primary_cpu)
{
#ifdef DEBUG_TLB
	int cpu = hard_smp_processor_id();
	if(primary_cpu) {
		if (tlb->attr1 & (_PAGE_VALID >> ENTRYLO_PFN_SHIFT) )
			printk(KERN_INFO "CPU %2d: TLB wired entry %2d: Pagesize %3dMB, VA 0x%lx PA0 0x%010llx PA1 0x%010llx\n",
				cpu, read_c0_index(), (tlb->pagesize >> 20), (unsigned long)tlb->vaddr,
				(unsigned long long)tlb->paddr0,
				(unsigned long long)tlb->paddr1);
		else
			printk(KERN_INFO "CPU %2d: TLB wired entry %2d: Pagesize %3dMB, VA 0x%lx PA0 0x%010llx\n",
				cpu, read_c0_index(), (tlb->pagesize >> 20), (unsigned long)tlb->vaddr,
				(unsigned long long)tlb->paddr0);
	}
#endif
}

void setup_mapped_kernel_tlbs(int firstpage, int primary_cpu)
{
	tlb_info_t tlb;

	tlb.pagesize = LARGEST_TLBPAGE_SZ; /* we set up the largest pages */

	/*
	 * In NetLogic's Linux kernel, the second 256MB of physical
	 * address space is reserved for device configuration and
	 * is not mapped to DRAM (to imply memory as opposed to IO
	 * device space). Hence the attribute of the second part of
	 * the first wired entry is invalid, while the both part of
	 * other wired entries are symmetric. We handle the above
	 * difference through the following unseemly if condition
	 */
	if (firstpage) {
		tlb.vaddr  = TLB_VADDR;
		tlb.paddr1 = tlb.paddr0 = 0;
		tlb.attr0  = ((_CACHE_CACHABLE_COW |_PAGE_DIRTY |  _PAGE_VALID | _PAGE_GLOBAL) >> ENTRYLO_PFN_SHIFT);
		tlb.attr1  = _PAGE_GLOBAL >> ENTRYLO_PFN_SHIFT;
		tlb.wired  = TRUE;
		tlb_printk(&tlb, primary_cpu);
		setup_tlb(&tlb);
	}
	else {
		/*
		 * the primary cpu reads the memory map and records
		 * the highest page frame number. Secondary cpus
		 * must wait till the variable max_low_pfn is set
		 */
		if (!primary_cpu)
			while (!max_low_pfn_set)
				;

		tlb.attr0 = ((_CACHE_CACHABLE_COW |_PAGE_DIRTY |  _PAGE_VALID | _PAGE_GLOBAL) >> ENTRYLO_PFN_SHIFT);
		tlb.vaddr = TLB_VADDR + 2 * LARGEST_TLBPAGE_SZ; 
		tlb.paddr0 = 2 * LARGEST_TLBPAGE_SZ;
		for (; tlb.paddr0 <= (max_low_pfn << PAGE_SHIFT);
			 tlb.paddr0 += 2 * tlb.pagesize, tlb.vaddr += 2 * tlb.pagesize) {
			/*
			 * Skip 3 - 3.5GB range (PCI device space)
			 */
			if (tlb.paddr0 == PCIDEV_ADDRSPACE_START)
				continue;
			/* Check if only one half of the entry is needed */
			if ((tlb.paddr0 + tlb.pagesize) <= (max_low_pfn << PAGE_SHIFT)) {
				tlb.paddr1 = tlb.paddr0 + tlb.pagesize;
				tlb.attr1 = tlb.attr0;
			} else {
				tlb.paddr1 = 0;
				tlb.attr1 = _PAGE_GLOBAL >> ENTRYLO_PFN_SHIFT;
			}
			tlb.wired = TRUE;
			tlb_printk(&tlb, primary_cpu);
			setup_tlb(&tlb);
		}
		if (primary_cpu)
			__vmalloc_start = tlb.vaddr;
	}
}

unsigned long recalculate_max_low_pfn(unsigned long max_low_pfn)
{
#if 0
	/* 
	 * truncate max_low_pfn to 512MB boundary as largest tlb
	 * pages are used to minimize the number of wired entries
	 */
	if ((max_low_pfn << PAGE_SHIFT) >= (2ULL * LARGEST_TLBPAGE_SZ))
		max_low_pfn = PFN_DOWN((uint64_t)(max_low_pfn << PAGE_SHIFT) & ~((2ULL * LARGEST_TLBPAGE_SZ) - 1));
#endif
	max_low_pfn_set = TRUE;
	__sync();

#ifdef DEBUG_TLB
		printk("max_low_pfn = 0x%lx -> max memory address = 0x%llx\n",
				max_low_pfn, ((unsigned long long)(max_low_pfn + 1) << PAGE_SHIFT) - 1);
#endif

	return max_low_pfn;
}

#endif

#else

void setup_mapped_kernel_tlbs(int index, int secondary_cpu) { }
unsigned long recalculate_max_low_pfn(unsigned long max_low_pfn) {return max_low_pfn;}

#endif /* #if defined(CONFIG_KSEG2_LOWMEM) && defined(CONFIG_KSEG2_LOWMEM) */

/* override of arch/mips/mm/cache.c: __uncached_access */
extern int nlm_common_get_pgprot(unsigned long address);
int __uncached_access(struct file *file, unsigned long addr)
{
	if (file->f_flags & O_SYNC)
		return 1;

	/* check the address region, return uncached pages for IO space and
	   cached page for memory space. */
	return nlm_common_get_pgprot(addr);
}

inline int valid_phys_addr_range(unsigned long addr, size_t count)
{
	/* for now return valid */
	return 1;
}

inline int valid_mmap_phys_addr_range(unsigned long pfn, size_t size) 
{
	/* for now return valid */
	return 1;
}
