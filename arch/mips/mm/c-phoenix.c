/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 * Copyright (C) 1997, 2001 Ralf Baechle (ralf@gnu.org)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include <linux/init.h>
#include <asm/asm.h>
#include <asm/mmu_context.h>
#include <asm/bootinfo.h>
#include <asm/cacheops.h>
#include <asm/cpu.h>
#include <asm/uaccess.h>
#include <linux/smp.h>
#include <linux/kallsyms.h>
#include <linux/mm.h>
#include <linux/module.h>

#ifdef CONFIG_NLM_XLP
#include <asm/mach-netlogic/xlp-mmu.h>
#endif
#include <asm/netlogic/debug.h>

static unsigned int icache_linesz;
static unsigned int icache_lines;

#ifdef CONFIG_NLM_VMIPS
extern void nlm_vmips_temp_xkphys_tlb_add(phys_t start, phys_t end, int *tlbs, int *tlbe);
extern void nlm_vmips_wired_entry_remove(int index);
#endif


#define cacheop(op, base) __asm__ __volatile__ (".set push\n.set mips4\ncache %0, 0(%1)\n.set pop\n" : : "i"(op), "r"(base))

#define cacheop_extable(op, base) do {                    \
  __asm__ __volatile__(                                    \
		       "    .set push                \n"   \
		       "    .set noreorder           \n"   \
		       "    .set mips4               \n"   \
		       "1:  cache %0, 0(%1)           \n"  \
		       "2:  .set pop                 \n"   \
		       "    .section __ex_table,\"a\"\n"   \
		            STR(PTR)"\t1b, 2b\n\t"        \
		       "     .previous               \n"   \
		       : : "i" (op), "r" (base));          \
  } while (0)

#ifdef CONFIG_NLM_XLP

static __inline__ void sync_istream(void)
{
	pipeline_flush();
}

static __inline__ void cacheop_hazard(void)
{
	pipeline_flush();
}

static __inline__ void cacheop_sync_istream(void)
{
	pipeline_flush();
}

#else /* !CONFIG_NLM_XLP */

static __inline__ void sync_istream(void)
{
  __asm__ __volatile__ (
                       ".set push                     \n"
                       ".set noreorder                \n"
		       //                       " la     $8, 1f                \n"
                       //" mtc0   $8, $14               \n"
                       //"eret                          \n"
		       //"1:nop                         \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
		       "nop                           \n"
                       ".set pop                      \n"
                       : : : "$8"
		       );
}

static __inline__ void cacheop_hazard(void)
{
  __asm__ __volatile__ (
                       ".set push                     \n"
                       ".set noreorder                \n"
                       " nop;nop;nop;nop              \n"
                       " nop;nop;nop;nop              \n"
                       ".set pop                      \n"
                       );
}

static __inline__ void cacheop_sync_istream(void)
{
  cacheop_hazard();
  sync_istream();
}

#endif /* !CONFIG_NLM_XLP */

#if 0
#define optimize_thread_flush() do { \
  if ( (cpu_logical_map(smp_processor_id()) & 0x03) != 0) return; \
} while(0)
#else
#define optimize_thread_flush()
#endif

extern unsigned long nlm_common_ebase;
/*****************************************************************************************
 *
 * These routines support Generic Kernel cache flush requirements
 *
 *****************************************************************************************/
void nlm_common_flush_dcache_page(struct page *page)
{
  ClearPageDcacheDirty(page);
}

EXPORT_SYMBOL(nlm_common_flush_dcache_page);

static void nlm_common_local_flush_icache_range(unsigned long start, unsigned long end)
{
  unsigned long addr;

  //dbg_msg("flush icache range, start=%lx, end=%lx\n", start, end);

  for(addr = (start & ~((unsigned long)(icache_linesz - 1))); addr < end;
            addr += icache_linesz) {
    cacheop_extable(Hit_Invalidate_I, addr);
  }

  cacheop_sync_istream();
}

struct flush_icache_range_args {
  unsigned long start;
  unsigned long end;
};
struct flush_icache_range_args_paddr {
  phys_t start;
  phys_t end;
};

static void nlm_common_flush_icache_range_ipi(void *info)
{
  struct flush_icache_range_args *args = info;

  optimize_thread_flush();

  nlm_common_local_flush_icache_range(args->start, args->end);
}

void nlm_common_flush_icache_range(unsigned long start, unsigned long end)
{
  struct flush_icache_range_args args;

#ifdef CONFIG_NLMCOMMON_VM_DEBUG
  dbg_msg("return address: ");
  print_symbol("ra[0]=%s\n", return_address());
#endif

  if ((end - start) > PAGE_SIZE) {
    dbg_msg("flushing more than page size of icache addresses starting @ %lx\n", start);
  }

  args.start = start;
  args.end = end;
  /* TODO: don't even send ipi to non-zero thread ids
   * This may require some changes to smp_call_function interface, for now just avoid
   * redundant cache ops
   */
  on_each_cpu(nlm_common_flush_icache_range_ipi, &args, 1);
}

static void nlm_common_flush_cache_sigtramp_ipi(void *info)
{
  unsigned long addr = (unsigned long)info;

  optimize_thread_flush();

  addr = addr & ~((unsigned long)(icache_linesz - 1));
  cacheop_extable(Hit_Invalidate_I, addr );
  cacheop_sync_istream();
}

static void nlm_common_flush_cache_sigtramp(unsigned long addr)
{
  on_each_cpu(nlm_common_flush_cache_sigtramp_ipi, (void *) addr, 1);
}

/*****************************************************************************************
 *
 * These routines support MIPS specific cache flush requirements.
 * These are called only during bootup or special system calls
 *
 *****************************************************************************************/

static void nlm_common_local_flush_icache(void)
{
  int i=0;
  unsigned long base = CKSEG0;

  //dbg_msg("flushing the whole damn local I-cache\n");

  /* Index Invalidate all the lines and the ways */
  for(i=0;i<icache_lines;i++) {
    cacheop(Index_Invalidate_I, base);
    base += icache_linesz;
  }

  cacheop_sync_istream();

}

static void nlm_common_local_flush_dcache(void)
{
  int i=0;
  unsigned long base = CKSEG0;
  unsigned int lines;

	if (is_nlm_xlp2xx_compat) {
  		//dbg_msg("flushing the whole damn local D-cache\n");

		lines = current_cpu_data.dcache.ways * current_cpu_data.dcache.sets;

		 /* Index Invalidate all the lines and the ways */
		for(i=0;i<lines;i++) {
		  cacheop(Index_Writeback_Inv_D, base);
		  base += current_cpu_data.dcache.linesz;
  		}
	}
	else {
		nlm_flush_l1_dcache();	
	}
  	cacheop_hazard();
}

#ifdef CONFIG_KGDB
void nlm_common_flush_l1_icache_ipi(void *info)
{
	nlm_common_local_flush_icache();
}
#endif

#ifdef CONFIG_KGDB
void nlm_common_flush_l1_caches_ipi(void *info)
#else
static void nlm_common_flush_l1_caches_ipi(void *info)
#endif
{
  optimize_thread_flush();

  nlm_common_local_flush_dcache();
  nlm_common_local_flush_icache();
}

static void nlm_common_flush_l1_caches(void)
{
  //dbg_msg("NASTY CACHE FLUSH: flushing L1 caches on all cpus!\n");
  on_each_cpu(nlm_common_flush_l1_caches_ipi, (void *)NULL, 1);
}

#ifdef CONFIG_NLM_XLP

struct flush_cache_page_args {
	struct vm_area_struct *vma;
	unsigned long addr;
        unsigned long pfn;
};

static inline int has_valid_asid(const struct mm_struct *mm)
{
        return cpu_context(smp_processor_id(), mm);
}

static void local_nlm_flush_cache_mm(void * args)
{
       struct mm_struct *mm = args;

       if (!has_valid_asid(mm))
                return;

	nlm_flush_l1_dcache();
	nlm_common_local_flush_icache();
	cacheop_hazard();

}

static void nlm_flush_cache_mm(struct mm_struct *mm)
{
       if (!cpu_has_dc_aliases)
                return;

       on_each_cpu(local_nlm_flush_cache_mm, mm, 1);
}

static inline void local_nlm_flush_cache_range(void * args)
{
        struct vm_area_struct *vma = args;

        if (!(has_valid_asid(vma->vm_mm)))
                return;

        nlm_flush_l1_dcache();
	nlm_common_local_flush_icache();
        cacheop_hazard();
}

static void nlm_flush_cache_range(struct vm_area_struct *vma,
        unsigned long start, unsigned long end)
{
       if (!cpu_has_dc_aliases)
                return;

       on_each_cpu(local_nlm_flush_cache_range, vma, 1);
}

static inline void local_nlm_flush_cache_page(void *args)
{
       nlm_flush_l1_dcache();
	nlm_common_local_flush_icache();
       cacheop_hazard();
}

static void nlm_flush_cache_page(struct vm_area_struct *vma,
        unsigned long addr, unsigned long pfn)
{
        struct flush_cache_page_args args;

        args.vma = vma;
        args.addr = addr;
        args.pfn = pfn;

       on_each_cpu(local_nlm_flush_cache_page, &args, 1);
}

#endif

/*****************************************************************************************/

static void nlm_common_noflush(void) { /* do nothing */ }

static __init void probe_l1_cache(void)
{
  struct cpuinfo_mips *c = &current_cpu_data;
  unsigned int config1 = read_c0_config1();
  int lsize = 0;
  int icache_size=0, dcache_size=0;

  if ((lsize = ((config1 >> 19) & 7)))
    c->icache.linesz = 2 << lsize;
  else
    c->icache.linesz = lsize;
  c->icache.sets = 64 << ((config1 >> 22) & 7);
  c->icache.ways = 1 + ((config1 >> 16) & 7);

  icache_size = c->icache.sets *
    c->icache.ways *
    c->icache.linesz;
  c->icache.waybit = ffs(icache_size/c->icache.ways) - 1;

  c->dcache.flags = 0;

  if ((lsize = ((config1 >> 10) & 7)))
    c->dcache.linesz = 2 << lsize;
  else
    c->dcache.linesz= lsize;
  c->dcache.sets = 64 << ((config1 >> 13) & 7);
  c->dcache.ways = 1 + ((config1 >> 7) & 7);

  dcache_size = c->dcache.sets *
    c->dcache.ways *
    c->dcache.linesz;
  c->dcache.waybit = ffs(dcache_size/c->dcache.ways) - 1;

  if (smp_processor_id()==0) {
    printk("Primary instruction cache %dkB, %d-way, linesize %d bytes.\n",
	   icache_size >> 10,
	   c->icache.ways, c->icache.linesz);

    printk("Primary data cache %dkB %d-way, linesize %d bytes.\n",
	   dcache_size >> 10, c->dcache.ways, c->dcache.linesz);
  }

}

static __inline__ void install_cerr_handler(void)
{
  extern char except_vec2_generic;

  memcpy((void *)(nlm_common_ebase + 0x100), &except_vec2_generic, 0x80);
}

static void update_kseg0_coherency(void)
{
  int attr = read_c0_config() & CONF_CM_CMASK;

  if (attr != 0x3) {

    nlm_common_local_flush_dcache();
    nlm_common_local_flush_icache();

    change_c0_config(CONF_CM_CMASK, 0x3);

    sync_istream();
  }
  _page_cachable_default = (0x3 << _CACHE_SHIFT);

}

void ld_mmu_xlr(void)
{
	extern void build_clear_page(void);
	extern void build_copy_page(void);
	/* update cpu_data */

	probe_l1_cache();

	if (smp_processor_id()) {

#if 0
		/* flush the exception vector region to make sure
		 * not to execute bootloader's exception code
		 */
		nlm_common_local_flush_icache_range(nlm_common_ebase, nlm_common_ebase + 0x400);
#endif
		nlm_common_local_flush_icache();

		update_kseg0_coherency();

		return;
	}

	/* These values are assumed to be the same for all cores */
	icache_lines = current_cpu_data.icache.ways * current_cpu_data.icache.sets;
	icache_linesz = current_cpu_data.icache.linesz;

	/* When does this function get called? Looks like MIPS has some syscalls
	 * to flush the caches.
	 */
	__flush_cache_all = nlm_common_flush_l1_caches;

	/* flush_cache_all: makes all kernel data coherent.
	 * This gets called just before changing or removing
	 * a mapping in the page-table-mapped kernel segment (kmap).
	 * Physical Cache -> do nothing
	 */
	flush_cache_all = nlm_common_noflush;

	/* flush_icache_range: makes the range of addresses coherent w.r.t I-cache and D-cache
	 * This gets called after the instructions are written to memory
	 * All addresses are valid kernel or mapped user-space virtual addresses
	 */
	flush_icache_range = nlm_common_flush_icache_range;

	/* flush_cache_{mm, range, page}: make these memory locations, that may have been written
	 *                                by a user process, coherent
	 * These get called when virtual->physical translation of a user address space is about
	 * to be changed. These are closely related to TLB coherency (flush_tlb_{mm, range, page})
	 */
#ifdef CONFIG_NLM_XLP
        if ((!is_nlm_xlp2xx_compat) && cpu_has_dc_aliases) {
               flush_cache_mm = nlm_flush_cache_mm;
               flush_cache_range = nlm_flush_cache_range;
               flush_cache_page = nlm_flush_cache_page;
        }
	else 
#endif
	{
		flush_cache_mm = (void (*)(struct mm_struct *))nlm_common_noflush;
		flush_cache_range = (void *) nlm_common_noflush;
		flush_cache_page = (void *) nlm_common_noflush;
	}

	/* flush_icache_page: flush_dcache_page + update_mmu_cache takes care of this
	 *
	 */
	flush_data_cache_page = (void *) nlm_common_noflush;

	/* flush_cache_sigtramp: flush the single I-cache line with the proper fixup code
	 */
	flush_cache_sigtramp = nlm_common_flush_cache_sigtramp;

	/* flush_icache_all: This should get called only for Virtuall Tagged I-Caches
	 */
	flush_icache_all = (void *)nlm_common_noflush;

	local_flush_icache_range = nlm_common_local_flush_icache_range;
	local_flush_data_cache_page	= (void *)nlm_common_noflush;

	__flush_cache_vmap = (void *)nlm_common_noflush;
	__flush_cache_vunmap = (void *)nlm_common_noflush;

	install_cerr_handler();

	build_clear_page();
	build_copy_page();

	nlm_common_local_flush_icache();

	update_kseg0_coherency();
}

#ifdef CONFIG_64BIT
#define cacheop_paddr(op, base) __asm__ __volatile__ ( \
                         ".set push\n"           \
                         ".set noreorder\n"      \
                         ".set mips64\n"          \
                         "dli $8, 0x9800000000000000\n"              \
                         "daddu $8, $8, %1\n"       \
                         "cache %0, 0($8)\n"     \
                         ".set pop\n"            \
                         : : "i"(op), "r"(base) : "$8")

#else
static inline void cacheop_paddr(const unsigned int op, phys_t base)
{
	uint64_t temp_msb, temp_lsb;
	phys_t temp1;

	temp_msb = (uint64_t)(base >> 32);
	temp_lsb = (uint64_t)(base & 0xffffffff);

	__asm__ volatile(
		".set push\n"
		".set noreorder\n"
		".set mips64\n"
		"dli $8,0x9800000000000000\n"
		"dsll32 %0, %2,0\n"
		"or %0,%0,%3\n"
		"daddu $8, $8, %0\n"
		"cache %1, 0($8)\n"
		".set pop\n"
		".set reorder\n"
		: "=&r"(temp1)
		: "i"(op), "r"(temp_msb) , "r"(temp_lsb)
		:"$8"
		);
}
#endif

#define enable_KX(flags)   \
 preempt_disable(); \
 __asm__ __volatile__ (          \
	".set push\n"              \
	".set noat\n"               \
	".set noreorder\n"     \
	"mfc0 %0, $12\n\t"             \
	"ori $1, %0, 0x81\n\t"   \
	"xori $1, 1\n\t"      \
	"mtc0 $1, $12\n"       \
        ".set pop\n"          \
        : "=r"(flags) ); \
  preempt_enable();

#define disable_KX(flags)   \
 __asm__ __volatile__ (          \
	".set push\n"              \
	"mtc0 %0, $12\n"       \
        ".set pop\n"          \
        : : "r"(flags) )


#define SETS_PER_WAY_SHIFT 22
#define SETS_PER_WAY_MASK 0x7
#define CACHELINE_SIZE_BITS 5

static void nlm_common_local_flush_icache_range_paddr(phys_t start, phys_t end)
{
	phys_t addr;
#ifdef CONFIG_32BIT
	unsigned long flags;
	phys_t temp;
#endif
#ifdef CONFIG_NLM_VMIPS
	int tlbs = 0, tlbe = 0;
	nlm_vmips_temp_xkphys_tlb_add(start, end, &tlbs, &tlbe);
#endif

#ifdef CONFIG_NLM_XLP
	int sets_per_way, niter, i;
	uint64_t mask;

	sets_per_way = (read_c0_config1() >> SETS_PER_WAY_SHIFT) & SETS_PER_WAY_MASK;
	niter = sets_per_way + 6 + CACHELINE_SIZE_BITS - PAGE_SHIFT;
	if (niter < 0)
		niter = 0;
	niter = 1 << niter;
	mask = niter - 1;
#endif

#ifdef CONFIG_32BIT
	enable_KX(flags);
#endif
    for (addr = (start & ~(phys_t)(icache_linesz - 1)); addr < end;
                    addr += icache_linesz) {
		cacheop_paddr(Hit_Invalidate_I, addr);
#ifdef CONFIG_NLM_XLP
		for (i = 1; i < niter; ++i)
			cacheop_paddr(Hit_Invalidate_I, (addr & ~(mask << PAGE_SHIFT)) | (i << PAGE_SHIFT));
#endif
    }

#ifdef CONFIG_32BIT
	disable_KX(flags);
#endif

#ifdef CONFIG_NLM_VMIPS
	for(;tlbe >= tlbs; tlbe--)
        nlm_vmips_wired_entry_remove(tlbe);

#endif
	cacheop_sync_istream();
}

static void nlm_common_flush_icache_range_paddr_ipi(void *info)
{
  struct flush_icache_range_args_paddr *args = info;

  optimize_thread_flush();

  nlm_common_local_flush_icache_range_paddr(args->start, args->end);
}

void nlm_common_flush_icache_range_paddr(phys_t start)
{
  struct flush_icache_range_args_paddr args;

#ifdef CONFIG_NLMCOMMON_VM_DEBUG
  dbg_msg("return address: ");
  print_symbol("ra[0]=%s\n", (unsigned long) return_address());
#endif

  args.start = start;
  args.end = start + PAGE_SIZE;
  /* TODO: don't even send ipi to non-zero thread ids
   * This may require some changes to smp_call_function interface, for now just avoid
   * redundant cache ops
   */
  on_each_cpu(nlm_common_flush_icache_range_paddr_ipi, &args, 1);
}


