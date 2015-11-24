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


/*
 * Setup code for Netlogic's XLR-based boards
 */

#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/pm.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>

#include <asm/netlogic/sim.h>
#include <asm/mipsregs.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/xlr_user_mac.h>
#include <asm/netlogic/msgring.h>

#include <asm/netlogic/nlm_pcix_gen_dev.h>
#include <asm/netlogic/bootinfo.h>
#include <asm/netlogic/memory-exclusion.h>

#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/mach-netlogic/mmu.h>
#include <asm/netlogic/xlr_board.h>
#include "ops.h"

#define fdt32_to_cpu(x) be32_to_cpu(x)

#ifdef NLM_BRIDGE_WKAROUND
#include <asm/netlogic/nlm_rw_lock.h>
#include <asm/netlogic/global_shmem.h>
nlm_rwlock_t *nlm_bridge_lock;
EXPORT_SYMBOL(nlm_bridge_lock);
int nlm_enable_br_wrkaround = 0;
unsigned long global_shmem_addr = 0;
EXPORT_SYMBOL(nlm_enable_br_wrkaround);
#endif

#ifdef CONFIG_NLM_XLP
#include <asm/netlogic/hal/nlm_hal_macros.h>
#include <asm/netlogic/hal/nlm_hal_pic.h>
#endif

#define dprintk(fmt, args...) //printk(fmt, ##args)

/* Certain macros for this file
 */

#define TRUE 					1
#define FALSE 					0


#define GPIO_SWRESET_REG 		8

#define PER_CPU_THREAD_SIZE 	(THREAD_SIZE >> 2)
#define TOTAL_THREAD_SIZE       (PER_CPU_THREAD_SIZE * (NR_CPUS - 1))

#define BOOT_LOADER_REGION_SZ 	0x04000000
#define LOADER_KSEG_END 		0x10000000

/* used by the default memory map
 */
#define DEF_PHYMEM_START_ADDR 	0x100000
#define DEF_PHYMEM_SIZE 		0x0ff00000

char cpu_model_info[100]={'X','L','R'};
#define PROCESSOR_ID_MAX_LEN 		6

extern char _end;

/* by default, do not assume u-boot */
int loader_used = LOADER_OTHER; 

/*  Board information filled by board dependent code */
xlr_board_info_t nlm_xlr_board_info;

/* Struct for temp. allocation
 * of sp/gp for secondary CPUs 
 */
struct xlr_stack_pages {
	unsigned long stack[(TOTAL_THREAD_SIZE)/sizeof(long)];
};

struct xlr_stack_pages xlr_stack_pages_temp
__attribute__((__section__(".data.init_task"),
	       __aligned__(THREAD_SIZE)));

extern void prom_pre_boot_secondary_cpus(void *);
extern unsigned int nlm_xlr_uart_in(struct uart_port *p, int offset);
extern void nlm_xlr_uart_out(struct uart_port *p, int offset, int value);

struct proc_dir_entry *nlm_root_proc;
EXPORT_SYMBOL(nlm_root_proc);

unsigned long long nlm_common_tlb_stats[32] __cacheline_aligned;

spinlock_t atomic_lock = SPIN_LOCK_UNLOCKED;
uint32_t xlr_linux_cpu_mask;
unsigned int onlinemask = 0x1;

__u8 xlr_base_mac_addr[6];
static char *hybrid_str = NULL;

/* xls chip family variables */
int chip_is_xls6xx = 0;
int chip_is_xls4xx = 0;
int chip_is_xls2xx = 0;
int chip_is_xls1xx = 0;
int chip_is_xls = 0;
int chip_is_xls_b0 = 0;
int chip_is_xls6xx_b0 = 0;
int chip_is_xls4xx_b0 = 0;
EXPORT_SYMBOL(chip_is_xls6xx);
EXPORT_SYMBOL(chip_is_xls4xx);
EXPORT_SYMBOL(chip_is_xls2xx);
EXPORT_SYMBOL(chip_is_xls1xx);
EXPORT_SYMBOL(chip_is_xls);
EXPORT_SYMBOL(chip_is_xls_b0);
EXPORT_SYMBOL(chip_is_xls6xx_b0);
EXPORT_SYMBOL(chip_is_xls4xx_b0);

int xlp_with_mac_driver = 0;
EXPORT_SYMBOL(xlp_with_mac_driver);

/* Environment Variables
 */
struct environment xlr_bldr_env ;

__u32 xlr_board_major_version = NLM_XLR_BOARD_ARIZONA_I;
__u32 xlr_board_minor_version = 0;

void *nlm_common_psb_shm = 0;
unsigned long nlm_common_psb_shm_size = 0;
static int dyna_exc_index=0;
extern unsigned long _text[];
extern void config_net_init(void);

#ifdef CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID
unsigned long nlm_asid_mask = 0x3f;
unsigned int nlm_shtlb = 1; /* by default shared TLB is enabled */
#endif

/* TODO: This should be removed eventually after taking a look at
   nlm_fs_cpu_max_freq() and nlm_fs_prominfo
   */
struct psb_info prom_info_copy; /* Bootloader prom_info is saved here */


static struct physmap_info {
	int type;
	char *name;
} psb_physmap_info[] =
{
	{ 0x01 , "Memory" },
	{ 0x02 , " *** HOLE ***" },
	{ 0x03 , "Exception Vectors" },
	{ 0x04 , "Bootloader 0" },
	{ 0x05 , "NMI Memory" },
	{ 0x10 , "PCI ECFG Space" },
	{ 0x11 , "PCIX IO Space"    },
	{ 0x12 , "PCIX CFG Space"   },
	{ 0x13 , "PCIX Memory Space"},
	{ 0x14 , "HT IO Space"      },
	{ 0x15 , "HT CFG Space" },
	{ 0x16 , "HT Memory Space" },
	{ 0x17 , "SRAM (QDR) Space" },
	{ 0x18 , "Flash Region(Re-mapped)" },
	{ 0x19 , "PCIE IO Space"    },
	{ 0x1A , "PCIE CFG Space"   },
	{ 0x1B , "PCIE Memory Space"},
	{ 0xff , "Unknown type" }
};



struct boot_mem_map boot_physaddr_info;

/* Maintain in ascending order of 
 * the starting physical addresses 
 */
static struct boot_mem_map_exclude_region dynamic_exclude_regions[] = {
	[0] = {0, 0}, /* PCI Shared Mem Or RMIOS Lib Memory*/
	[1] = {0, 0}, /* PCI Shared Mem Or RMIOS Lib Memory*/
	[7] = {0, 0}, /* Hybrid Mode exclusion*/
	[8] = {0, 0}, /* END of the list - MUST be the last entry always */
};

static char *get_psb_physmap_name(int type)
{
	int i = 0;
	int tsize = sizeof(psb_physmap_info) / sizeof(struct physmap_info);

	for (i = 0; i < tsize; i++)	{
		if ( (psb_physmap_info[i].type == type ) ||
		    (psb_physmap_info[i].type == 0xff ) )
			return psb_physmap_info[i].name;
	}
	return ("Unknown type");
}

/* Return value
 * 	1 ==> IO (or not found)
 *  0 ==> mem 
 */
int nlm_common_get_pgprot(unsigned long address)
{
	/* return 1 if uncached and return 0 if cached access is required */
	/* TODO:
	   We need a actual "physical memory map" to implement this fully.
	   For now, treat anything in 256MB to 512MB as uncached access
	   */
	if((address >= NETLOGIC_UNCACHED_START) && 
			(address < NETLOGIC_UNCACHED_END))
		return 1;

	return 0;
}

#if defined(CONFIG_NLM_XLP_SIM)
const char *DEFAULT_CONSOLE_BOOT_PARAMS = "boot_noi2c mem=255m@1m mem=512m@512m console=ttyS0,115200 ";
#else
const char *DEFAULT_CONSOLE_BOOT_PARAMS = "mem=255m@1m mem=512m@512m console=ttyS0,115200 ";
#endif
const char *DEFAULT_INITRD_BOOT_PARAMS = "rdinit=/sbin/init ";

const char *get_system_type(void)
{
#ifdef CONFIG_NLM_XLP
	return "Netlogic XLP SIM";
#else
	if ( is_xls() )
		return "Netlogic XLS";
	return "Netlogic XLR";
#endif
}

#ifdef CONFIG_SMP
atomic_t cpus_rebooted = ATOMIC_INIT(0);
#endif

static void ptr_linux_exit(void)
{
       nlm_reg_t *mmio;

	/* trigger a chip reset 
	 */
	mmio = netlogic_io_mmio(NETLOGIC_IO_GPIO_OFFSET);
	netlogic_write_reg(mmio, GPIO_SWRESET_REG, 1);
	for ( ; ; ) 
		cpu_wait();
}

void __init bus_error_init(void)
{
}

void prom_reconfigure_thr_resources(void)
{
	unsigned int mmu_setup=0;
	int i=0, count=0, dis_contig=0;
	int value = 0;

	__u32 online_map, thr_mask; 

#ifdef CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID
	uint32_t map;
#endif
	online_map = xlr_linux_cpu_mask;
	
	thr_mask = online_map >> (netlogic_cpu_id()<<2);

#ifdef CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID
	/* netlogic kernel configures this 
	 */

	if (nlm_shtlb && (nlm_asid_mask == 0x3f)) {
		/* Global TLB will work only if all 
		 * the enabled cores have all their
		 * threads owned by Linux. 
		 */
		map = online_map;
		for (i = 0; i < NR_CPUS; i += 4) {
			if ((map & 0xf) && ((map & 0xf) != 0xf)) {
				nlm_asid_mask = 0xff;
				nlm_shtlb = 0;
				printk("Disabling Shared TLB mode\n");
				break;
			}
			map >>= 4;
		}
		if ((nlm_asid_mask == 0x3f) && (netlogic_thr_id() == 0)) {
			mmu_setup = read_32bit_nlm_ctrl_reg(4, 0);
			mmu_setup = mmu_setup | 0x1;
			write_32bit_nlm_ctrl_reg(4, 0, mmu_setup);

			printk("CPU %d: Enabled Shared TLB mode \n", 
					netlogic_cpu_id());
			return;
		}
	}
	
	return;

#endif /* CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID */


	 if (netlogic_thr_id() == 0) { 

		for (i=0;i<4;i++) {
			if (thr_mask & (1<<i)) {
				if (i != count)
					dis_contig = 1;
				count++;
			}
		}

		switch(count) {
			case 1: value = 0x00; break;
			case 2: value = 0x02; break;
			default:
					value = 0x03; break;
		}

		if (dis_contig)
			value = 0x3; 

		mmu_setup = read_32bit_nlm_ctrl_reg(4, 0);
		mmu_setup = mmu_setup & ~0x06;
		mmu_setup |= (value << 1);

		write_32bit_nlm_ctrl_reg(4, 0, mmu_setup);
	} 
}

int xlr_hybrid;

int xlr_console_pci_con_dev = 0;
int xlr_console_pci_con_baud = 0;
int xlr_boot_over_nfs = 0;

unsigned long nlm_common_ebase = 0x0;

static char prop_buf[MAX_PROP_LEN];
extern char _end;
extern void *fdt;
extern void *fdt_init(void *blob);
extern void *simple_alloc_init(char *base, unsigned long heap_size,
		                unsigned long granularity, unsigned long max_allocs);


//#if !defined(CONFIG_NLMCOMMON_MAC)
//struct user_mac_data *user_mac;
struct xlr_user_mac_config xlr_user_mac;
//#endif

static inline void init_default_macaddr(void)
{
	xlr_base_mac_addr[0] = 0x00;
	xlr_base_mac_addr[1] = 0x01;
	xlr_base_mac_addr[2] = 0x02;
	xlr_base_mac_addr[3] = 0x03;
	xlr_base_mac_addr[4] = 0x04;
	xlr_base_mac_addr[5] = 0x05;
}

static void setup_default_configuration(void)
{
	xlr_hybrid = XLR_HYBRID_NONE;
	xlr_user_mac.l4_extract = 0;
	xlr_user_mac.fast_syscall = 1;

	nlm_common_psb_shm = 0;

	init_default_macaddr();
}

void exclude_hybrid_mem_region(void)
{
	dynamic_exclude_regions[dyna_exc_index].start = 1<<20;
	dynamic_exclude_regions[dyna_exc_index].end = 
		(unsigned long long)(((unsigned long)&_text) & 0x1fffffffUL);
	dyna_exc_index++;
}

#ifndef CONFIG_MAPPED_KERNEL
static void xlr_early_hybrid_setup(char *str)
{
	hybrid_str = str;

	if ((strcmp(str, "=rmios_ipsec") == 0)||
			(strcmp(str, "rmios_ipsec") == 0)) {
		exclude_hybrid_mem_region();
	}
	else if ((strcmp(str, "=rmios_tcpip_stack") == 0)||
			(strcmp(str, "rmios_tcpip_stack") == 0)) {
		exclude_hybrid_mem_region();
	}
}
#endif

static int xlr_hybrid_setup(char *str)
{
	uint64_t kernel_start;

	if ((strcmp(str, "=user_mac_xgmac") == 0)||
			(strcmp(str, "user_mac_xgmac") == 0)) {
		if (xlr_board_atx_ii()) {
			xlr_hybrid = XLR_HYBRID_USER_MAC_XGMAC;
			printk("Configured for Hybrid mode with USER_MAC_XGMAC\n");
		} else if ( xlr_board_atx_i()) {
			xlr_hybrid = XLR_HYBRID_USER_MAC_SPI4;
			printk("Configured for Hybrid mode with USER_MAC_SPI4\n");
		}
		else {
			printk(
		"user_mac_xgmac hybrid mode is available only on ATX-II\n");
		}
	}
	else if ((strcmp(str, "=user_mac") == 0)||
			(strcmp(str, "user_mac") == 0)) {
		xlr_hybrid = XLR_HYBRID_USER_MAC;
		printk("Configured for Hybrid mode with USER_MAC\n");
	}
	else if ((strcmp(str, "=rmios_ipsec") == 0) ||
			(strcmp(str, "rmios_ipsec") == 0)) {
		xlr_hybrid = XLR_HYBRID_RMIOS_IPSEC;
		printk("Configured for Hybrid mode with RMIOS IPSEC\n");
	}
	else if ((strcmp(str, "=rmios_tcpip_stack") == 0)||
			(strcmp(str, "rmios_tcpip_stack") == 0)) {
		xlr_hybrid = XLR_HYBRID_RMIOS_TCPIP_STACK;
		kernel_start = (uint64_t)
			(((unsigned long)&_text) & 0x1fffffffUL);
		if (kernel_start < NLM_RMIOS_TCPIP_END) {
			panic("Build kernel with loadaddress above %#x\n",
					NLM_RMIOS_TCPIP_END);
		}
		printk("Configured for Hybrid mode with RMIOS_TCPIP_STACK\n");
	}
	else {
		xlr_hybrid = XLR_HYBRID_NONE;
		printk("Configured for Hybrid mode with None\n");
	}

	return 1;
}


unsigned int __cpuinit get_c0_compare_int(void)
{
    return IRQ_TIMER;
}


void plat_time_init(void)
{
	extern void nlm_common_timer_setup(void);

	mips_hpt_frequency = nlm_xlr_board_info.cpu_freq;

	printk("mips_hpt_frequency = %u\n", mips_hpt_frequency);

	nlm_common_timer_setup();
}

#ifdef CONFIG_NLM_COMMON
int avail_mem_above_4G;
int force_usb __initdata = 0;
static int __init xls_force_usb(char *p)
{
    force_usb = 1;
        return 0;
}
early_param("forceusb", xls_force_usb);


/* This routine is useful when USB is desired on
 * 64-Bit Linux with DRAM mapped >4G. On such systems,
 * since the XLS USB controller is 32-bit, USB is
 * disabled. Use command line option 'forceusb' to
 * enable it; This adjusts the mapped available mem
 * to a max of till 0xFFFFFFFF.
 */
static void __init tweak_avail_dram_map(void) {

    int j=0;
    int nrmap_ctr = (boot_physaddr_info.nr_map - 1);

    avail_mem_above_4G = 0;

    for (j=nrmap_ctr; j>=0; j--) {
        if ((boot_physaddr_info.map[j].addr + boot_physaddr_info.map[j].size)
                > 0x100000000ULL) {
            avail_mem_above_4G++;
#ifdef CONFIG_64BIT
            if (force_usb) {
                printk(KERN_WARNING "[USB]:Re-adjusting Available DRAM map\n");
                if (boot_physaddr_info.map[j].addr > 0x100000000ULL) {
                    boot_physaddr_info.nr_map--;
                }
                else {
                    /* Reclaim whatever we can... */
                    boot_physaddr_info.map[j].size =
                        0x100000000ULL - boot_physaddr_info.map[j].addr;
                }
            }
#endif
        }
    }
}
#endif


void __init plat_mem_setup(void)
{
	extern int panic_timeout;
  
	panic_timeout = 5;  
  
	_machine_restart = (void (*)(char *))ptr_linux_exit;
	_machine_halt    = ptr_linux_exit;
	pm_power_off 	 = ptr_linux_exit;

	tweak_avail_dram_map();

	return;
}  

#ifdef CONFIG_MAPPED_KERNEL
#define secondary_cpus_bootup_func \
	((unsigned long)prom_pre_boot_secondary_cpus - \
	 (unsigned long)LOADADDR + (unsigned long)PHYSADDR)
#else
#define secondary_cpus_bootup_func prom_pre_boot_secondary_cpus
#endif

/* arg 	- arg passed by user
 * name - pointer to the start of name=value string
 * base - conversion base 
 * res 	- converted number is stored here 
 * Note: -
 *	returned value is a 32 bit number always
 * Returns 0 on success, -1 otherwise
 */

static int get_name_value(char *arg, char *name, int base, uint32_t *res)
{
	char *ptr;

	if ((ptr = strstr(arg, name)) == NULL)
		return -1;

	if (!strcmp("app_sh_mem_sz=", name)) {

		printk("WARNING: \"app_sh_mem_sz\"  option  is  deprecated\n");
		printk("WARNING: Use ./userapp shmem option to reserve app "
				 "shared memory\n");
		return -1;
	}
			
	ptr = strrchr(ptr, '=');
	dprintk("ptr after strrchr = %s\n", ptr);
	ptr++;
	*res = (uint32_t)simple_strtol(ptr, (char **)NULL, base);
	return 0;

}

struct nlm_common_name_value_struct {
	char *name;
	uint32_t *val;
};

static struct nlm_common_name_value_struct nlm_common_name_value_args[] = {
	{NULL, NULL}
};


static void parse_cmdline_args(int argc, char *argv[])  
{
	int i, j;
	int ret;
	char *tmp = NULL;

	for (i=1; i<argc; i++) {

		if (argv && argv[i]) {

			if (strcmp(argv[i], "nlm_no_shtlb") == 0) {

#ifdef CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID
				nlm_shtlb = 0;
				nlm_asid_mask = 0xff;
				printk("Disabling Shared TLB Support\n");
#endif
			} else if (strcmp(argv[i],"console=/dev/pci_co0") == 0) {
				xlr_console_pci_con_dev = 1;

			} else if (strcmp(argv[i],"console=pci_co,38400") == 0) {
				xlr_console_pci_con_baud = 1;

			} else if (strcmp(argv[i],"root=/dev/nfs") == 0) {
				xlr_boot_over_nfs = 1;

			} else if (strncmp(argv[i],"xlr_hybrid=",strlen("xlr_hybrid=")) == 0) {
				tmp = argv[i]+strlen("xlr_hybrid=");

			} else if (strcmp(argv[i], "xlp_with_mac_driver=1") == 0) {
				xlp_with_mac_driver = 1;
			} else if (strcmp(argv[i], "loader=uboot") == 0) {
				loader_used = LOADER_UBOOT;
			}
			else {
				j = 0;
				while(nlm_common_name_value_args[j].name != NULL) {
					
					ret = get_name_value(argv[i], 
							nlm_common_name_value_args[j].name, 16, 
							nlm_common_name_value_args[j].val);
					
					if (ret == 0)
						break;
					j++;
				}
			}
		}
	}

#ifdef CONFIG_MAPPED_KERNEL
	exclude_hybrid_mem_region();
#else
	if (tmp) {
		xlr_early_hybrid_setup(tmp);
	}
#endif
}

/* Maintain in ascending order of 
 * the starting physical addresses 
 */
static struct boot_mem_map_exclude_region _exclude_regions[2][MAX_EXCLUDE + 2];

static struct boot_mem_map_exclude_region *exclude_regions = _exclude_regions[1];

static struct boot_mem_map_exclude_region static_exclude_regions[] = {
	[0] = { 0,0},
};

void prom_exclude_pci_shmem(void)
{
	dynamic_exclude_regions[dyna_exc_index].start =
		NLM_PCIX_SHARED_MEM_START;
	dynamic_exclude_regions[dyna_exc_index].end = NLM_PCIX_SHARED_MEM_END; 
	dyna_exc_index++;
	printk("Excluding PCI Shared Memory\n");
}

void sort_dynamic_exclude_region(void)
{
	int i=0;
	int j=0;
	int max=0;
	struct boot_mem_map_exclude_region *list = dynamic_exclude_regions;

	uint64_t start = 0;
	uint64_t end = 0;

	while (list[max].start != 0)
		max++;

	for (i = 0; i < max; i++) {
		for (j = i; j < max; j++) {
			if (list[i].start > list[j].start) {
				start = list[i].start;
				end = list[i].end;
				list[i].start = list[j].start;
				list[i].end = list[j].end;
				list[j].start = start;
				list[j].end = end;
			}
		}
	}
}

static int merge_exclude_regions(struct boot_mem_map_exclude_region *,
                                 struct boot_mem_map_exclude_region *);

void prom_update_exclude_region(void)
{
	int i;

#ifdef CONFIG_NLMCOMMON_PCIX_GEN_DRIVER
	if (xlr_get_pci_mode() == XLR_PCI_DEV_MODE) {
		prom_exclude_pci_shmem();
	}
#endif

	sort_dynamic_exclude_region();
	
	exclude_regions = _exclude_regions[0];
	
	/*
	 * we assume that all exclude regions are sorted
	 * to start with.
	 */
	merge_exclude_regions(exclude_regions, static_exclude_regions);
	merge_exclude_regions(exclude_regions, dynamic_exclude_regions);
	
	dprintk("Final exclude regions ----->\n");
	for (i = 0; exclude_regions[i].start; i++) {
		dprintk("%d: Start 0x%llx End 0x%llx\n", i, 
			exclude_regions[i].start,
			exclude_regions[i].end);
	}
}

/* Reset this only if we find a valid FDT info passed */
int use_default_phymem = TRUE;


void validate_mem_map(void)
{
	struct boot_mem_map *map = &boot_physaddr_info;

	if (!(map->nr_map > 0 && map->nr_map <= 32))
		goto set_default_mmap;
	
	return;

set_default_mmap:
	/* We just set a global flag
	 */
	use_default_phymem = TRUE;
	return;
}

static void prom_add_memory(void)
{
	int i = 0, j = 0;
	__u64 start = 0, end = 0, exc_start = 0, exc_end = 0;
	__u64 pref_backup = 512;

	if (use_default_phymem)
		goto use_default;

	prom_update_exclude_region();
	
	for (i = 0; i < boot_physaddr_info.nr_map; i++) {
		start = boot_physaddr_info.map[i].addr;
		end = boot_physaddr_info.map[i].addr + 
			boot_physaddr_info.map[i].size;

		for (j = 0; j < MAX_EXCLUDE; j++) {
			exc_start = exclude_regions[j].start;
			exc_end = exclude_regions[j].end;
			
			if ((exc_start == 0) && (exc_end == 0)) /* Empty slot */
				continue;

			if (exc_start >= start && exc_start < end) {
				if (exc_start == start) { /* Continuous exclude */
					start = exc_end;
					continue;
				}
				if (boot_physaddr_info.map[i].type == BOOT_MEM_RAM) {

					/*
					 * memcpy/__copy_user prefetch, which
					 * will cause a bus error for
					 * KSEG/KUSEG addrs not backed by RAM.
					 * Hence, reserve some padding for the
					 * prefetch distance.
				 	*/
					if (exc_start-start > pref_backup) {
						add_memory_region(start,
						exc_start-start-pref_backup, 
						(long)boot_physaddr_info.map[i].type);
					}
					start = exc_end;
				}
			} 
			else if ((exc_start < start) && (exc_end > start)) {
				/* Overlapping excludes 
				 */
				start = exc_end;
			}
		}
		if (start != end)
			if (boot_physaddr_info.map[i].type == BOOT_MEM_RAM) {
				if (end-start > pref_backup)
					add_memory_region(start, 
						end-start-pref_backup, 
						(long)boot_physaddr_info.map[i].type);
			}
	}
	
	return;
	
 use_default:
	printk("Using Default Physical Mem Map\n"); 
	/* 255m@1m 
	 */
	add_memory_region (DEF_PHYMEM_START_ADDR, 
			DEF_PHYMEM_SIZE-pref_backup, (long)BOOT_MEM_RAM);
}

static void psb_print_physmap(void)
{
	struct boot_mem_map *physaddr_map = &boot_physaddr_info;
	char *name;
	int i = 0;
	int max;


	max = physaddr_map->nr_map;

	prom_dbg_msg("Physical Address Map\n");
	for (i = 0 ; i <max ; i++) {
		name = get_psb_physmap_name(physaddr_map->map[i].type);		
		if ( i == max-1) {
			prom_dbg_msg("\t%010llx --> %010llx ( %s )\n",
				     (unsigned long long)physaddr_map->map[i].addr,
				     (unsigned long long)(physaddr_map->map[i].addr + physaddr_map->map[i].size),
				     name);
		}
		else {
			prom_dbg_msg("\t%010llx --> %010llx ( %s )\n",
				     (unsigned long long)physaddr_map->map[i].addr,
				     (unsigned long long)(physaddr_map->map[i].addr +
							  physaddr_map->map[i].size -1),
				     name);
		}
	}
}

/* disable dedicated interrupt vector for virtual mips mode */
void disable_divec(void)
{
    int i;
    for (i = 0; i < NR_CPUS; i++)
        cpu_data[i].options &= ~MIPS_CPU_DIVEC;

    return;
}

extern void (*board_nmi_handler_setup)(void );

void __init nlm_nmi_setup (void)
{
	/* setup nmi handler only if KGDB is enabled */
#ifdef CONFIG_KGDB
	void *base;
	extern char nlm_except_vec_nmi;

	printk("Setting up NMI Handler \n");
	base = (void *)(unsigned long)0xffffffffbfc00000ULL;
	memcpy(base, &nlm_except_vec_nmi, 0x80);
#endif
}

/* setup early serial port driver */
#ifdef CONFIG_SERIAL_8250

#ifdef CONFIG_NLM_XLR
#define UART_CLK 66666666
#else
#define UART_CLK 133333333
#endif

static void nlm_early_serial_setup(void)
{
	struct uart_port s;
	extern int __init early_serial_setup(struct uart_port *port);

	memset(&s, 0, sizeof(s));

	s.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
	/* XLP_MERGE_TODO */
	s.iotype = UPIO_NLM;
	/* registers are 4 bytes wide */
	s.regshift = 2;
	/* hardware int 4 - the serial int, is CPU int 6
	 but poll for now */
	s.irq =  PIC_UART_0_IRQ;
	s.uartclk = UART_CLK;
	s.serial_in	= nlm_xlr_uart_in;
	s.serial_out	= nlm_xlr_uart_out;
	s.membase = (unsigned char __iomem *)(DEFAULT_NETLOGIC_IO_BASE+NETLOGIC_IO_UART_0_OFFSET);
	s.mapbase = (DEFAULT_NETLOGIC_IO_BASE+NETLOGIC_IO_UART_0_OFFSET);

	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial setup failed!\n");
	}


}
#else
static void nlm_early_serial_setup(void) {}
#endif

extern struct plat_smp_ops nlm_smp_ops;



static void process_prominfo(void (*(*wakeup))(void *, void *, __u32))
{
#ifdef NLM_BRIDGE_WKAROUND
	if (global_shmem_addr != 0) {
		nlm_bridge_lock = (nlm_rwlock_t *)(unsigned long)
			(global_shmem_addr + BRIDGE_WKAROUND_AREA_OFFSET);
		nlm_enable_br_wrkaround = 1;
		printk("Enabling Bridge Workaround \n");
	}
#endif

	xlr_board_major_version = nlm_xlr_board_info.major;
	xlr_board_minor_version = nlm_xlr_board_info.minor;

	printk("Board Major %d Minor %d \n", xlr_board_major_version, 
			xlr_board_minor_version);

	psb_print_physmap();


	/* update the nlm_common  mac addr */
	xlr_base_mac_addr[0] = nlm_xlr_board_info.mac_addr[0];
	xlr_base_mac_addr[1] = nlm_xlr_board_info.mac_addr[1];
	xlr_base_mac_addr[2] = nlm_xlr_board_info.mac_addr[2];
	xlr_base_mac_addr[3] = nlm_xlr_board_info.mac_addr[3];
	xlr_base_mac_addr[4] = nlm_xlr_board_info.mac_addr[4];
	xlr_base_mac_addr[5] = nlm_xlr_board_info.mac_addr[5];

#ifdef DEBUG
	printk("MAC ADDR BASE: %02x:%02x:%02x:%02x:%02x:%02x\n",
	       xlr_base_mac_addr[0], xlr_base_mac_addr[1], xlr_base_mac_addr[2],
	       xlr_base_mac_addr[3], xlr_base_mac_addr[4], xlr_base_mac_addr[5]);
#endif

}

unsigned long secondary_entry_point;
static void prepare_wakeup(unsigned long wakeup_fun)
{
	void *base;
	extern char nlm_boot_smp_nmi;

	secondary_entry_point = wakeup_fun;

	base = (void *)(unsigned long)0xffffffffbfc00000ULL;
	memcpy(base, &nlm_boot_smp_nmi, 0x80);
	
}

static int wakeup_secondary_cpus(void)
{
	__u32 wakeup_mask;
	int i;
	uint32_t tid, pid, ipi;

	prepare_wakeup((unsigned long)secondary_cpus_bootup_func);

	wakeup_mask = (__u32)xlr_linux_cpu_mask & (~smp_boot.online_map);

	for(i=0; i < NR_CPUS; i++) {
		if(wakeup_mask & (1 << i)) {
			tid = i & 0x3;
			pid = (i >> 2) & 0x7;
			/* Send NMI, IPI should not matter - setting to 0 here*/
			ipi = (tid << 16) | (pid << 20) | (1 << 8);
			pic_send_ipi(ipi);
		}
	}


	return 0;
}

static int build_arcs_cmdline(char *arcs_cmdline, int argc, char *argv[])
{
	int i;

	for (i = 0; i < argc; i++) {
		if (argv[i]) {
			strcat(arcs_cmdline, argv[i]);
			strcat(arcs_cmdline, " ");
		}
	}
	strcat(arcs_cmdline, " ");


#ifdef CONFIG_NLMCOMMON_CONSOLE_OVER_PCI
	if (!(xlr_board_atx_iii() || xlr_board_atx_v()) ||
		!(xlr_console_pci_con_baud && 
		  xlr_console_pci_con_dev))
		strcat(arcs_cmdline, DEFAULT_CONSOLE_BOOT_PARAMS);
#else
	if ((strstr(arcs_cmdline, "console=")) == NULL)
		strcat(arcs_cmdline, DEFAULT_CONSOLE_BOOT_PARAMS);
#endif
	strcat(arcs_cmdline, " ");

#ifdef CONFIG_ROOT_NFS
	if (!xlr_boot_over_nfs)
		strcat(arcs_cmdline, DEFAULT_INITRD_BOOT_PARAMS);
#else
	strcat(arcs_cmdline, DEFAULT_INITRD_BOOT_PARAMS);
#endif
	strcat(arcs_cmdline, " ");

	return 0;
}

char xlr_arcs_cmdline[CL_SIZE];
/* This function will get the following:
   1. Boot args
   2. Physical memory map
   3. CPU online map to be used by Linux
   */
static int xlr_fdt_process(void)
{
	int  domain=0;
	char domstr[32] = "";
	unsigned int  i,j, na, ns, regs[16], entries;
	char macaddress[6];
	uint32_t shmem_addr;

	/* If booted using FDT and U-Boot, all
	 * we get is a pointer to an FDT Blob
	 */
	void *blob = (void *)fw_arg0;
	void *node;

	if(!blob)
		return -1;
	/* Create a region starting from
	 * (_end + 64K) of size 8MB for
	 * the FDT structures. The 64K is
	 * the current page size for XEN
	 */
	simple_alloc_init((char *)((unsigned long)(&_end)+0x10000),
			(8<<20), 32, 128);

	/* Create a local copy of the FDT
	 */
	fdt = fdt_init(blob);
	if(!fdt) {
		printk("FDT Init failed\n");
		return -1;
	}

	/* extract cmdline params
	 */
	node = finddevice("/chosen");
	if (node) {
		if (getprop(node, "bootargs", prop_buf, MAX_PROP_LEN) >= 0) {
			strcat(xlr_arcs_cmdline, prop_buf);
		}
	}

	node = finddevice("/system"); 
	if(node) {
		if (getprop(node, "boardmajor", &nlm_xlr_board_info.major,
					sizeof(nlm_xlr_board_info.major)) >= 0) {
			nlm_xlr_board_info.major = 
				fdt32_to_cpu(nlm_xlr_board_info.major);
		}

		if (getprop(node, "boardminor", &nlm_xlr_board_info.minor,
					sizeof(nlm_xlr_board_info.minor)) >= 0) {
			nlm_xlr_board_info.minor = 
				fdt32_to_cpu(nlm_xlr_board_info.minor);
		}

		if (getprop(node, "cpufrequency", &nlm_xlr_board_info.cpu_freq,
					sizeof(nlm_xlr_board_info.cpu_freq)) >= 0) {
			nlm_xlr_board_info.cpu_freq = 
				fdt32_to_cpu(nlm_xlr_board_info.cpu_freq);
		}

		if (getprop(node, "globalshmem", &shmem_addr,
					sizeof(shmem_addr)) >= 0) {
			global_shmem_addr = (int)fdt32_to_cpu(shmem_addr);
		}

		if (getprop(node, "macaddress", macaddress, 6) >= 0) {
			for(i=0; i < 6; i++) 
				nlm_xlr_board_info.mac_addr[i] = macaddress[i];
		}
	}

	/* extract memory ranges,
	 * add to command line
	 */
	node = finddevice("/doms/dom@0");
	if (node) {
		if (getprop(node, "#address-cells", &na, sizeof(na)) < 0)
			na = 1;
		na = fdt32_to_cpu(na);
		if (na < 1 || na > 2)
			printk("Can't cope with #address-cells == %d.\n\r", na);
		if (getprop(node, "#size-cells", &ns, sizeof(ns)) < 0)
			ns = 1;
		ns = fdt32_to_cpu(ns);
		if (ns < 1 || ns > 2)
			printk("Can't cope with #size-cells == %d.\n\r", ns);
	}

	node = finddevice("/doms/dom@0/memory");
	if (node) {
		entries = (getprop(node, "reg", regs, sizeof(regs))) / sizeof(regs[0]);
		if (!entries || (entries % (na+ns)))
			printk("Invalid Memory Map Specified!\n");

		boot_physaddr_info.nr_map = entries/2;
		j = 0;
		use_default_phymem = FALSE;
		for (i=0; i<entries; i+=2) {
			unsigned long long addr, size;
			addr = fdt32_to_cpu(regs[i]);
			size = fdt32_to_cpu(regs[i + 1]);
			boot_physaddr_info.map[j].addr = addr;
			boot_physaddr_info.map[j].size = size;
			boot_physaddr_info.map[j].type = BOOT_MEM_RAM;
			j++;
			printk("FDT mem[%d] start = %llx size %llx\n",
					j, addr, size);

		}
	}

	printk("FDT Cmdline: %s\n", xlr_arcs_cmdline);

	/* extract CPU online mask for
	 * domain 0 (linux)
	 */
	sprintf(domstr, "/doms/dom@%d/cpu", domain);

	node = finddevice(domstr);
	if (node) {
		if (getprop(node, "onlinemask", &onlinemask,
					sizeof(onlinemask)) < 0)
			return -1;
		onlinemask = fdt32_to_cpu(onlinemask);
	}

	if (!onlinemask)
		/* something went wrong ? */
		onlinemask = 0x1;

	xlr_linux_cpu_mask = onlinemask;
	return 0;
}

void xlr_split_args(int *argc, char *argv[])
{
	char *temp_cmdline = xlr_arcs_cmdline;
	int i=0;

	while(temp_cmdline) {
		argv[i] = (char *)strsep(&temp_cmdline, " ");
		i++;
	}
	*argc = i;
}

/*
 * prom_init is called just after the cpu type is detenetlogicned, from setup_arch()
 */
void __init prom_init(void)
{
	int argc;
	
	char *n_argv[NLM_MAX_ARGS] = {NULL};
	void (*wakeup)(void *, void *, __u32) = NULL;

	setup_mapped_kernel_tlbs(TRUE, TRUE);

	setup_default_configuration();

	xlr_fdt_process();
	/* split the args from arcs_cmdline to argv array */
	xlr_split_args(&argc, n_argv);

	parse_cmdline_args(argc, n_argv);

	build_arcs_cmdline(arcs_cmdline, argc, n_argv);

	set_xls_chip_family_types();

	/* Try to get the board major, minor, mac address etc if no FDT info is 
	   passed - TODO 
	   */
	if(read_board_info(&nlm_xlr_board_info) != 0) {
		printk("read board_info failed\n");
	}

	process_prominfo(&wakeup);

	validate_mem_map();

	nlm_common_ebase = read_c0_ebase() & (~((1 << 12) - 1));

	prom_add_memory();


	smp_boot.online_map = (1 << hard_smp_processor_id());

	wakeup_secondary_cpus();

	if (hybrid_str != NULL)
		xlr_hybrid_setup(hybrid_str);

	config_net_init();

	board_nmi_handler_setup = nlm_nmi_setup;

	on_chip_init();

	prom_reconfigure_thr_resources();

	/* setup early serial port driver */
	nlm_early_serial_setup();

	register_smp_ops(&nlm_smp_ops);
}

void prom_free_prom_memory(void)
{
	/* nothing to free */
}

void read_cp0_regs(void)
{
	printk("[%s]: count = 0x%x, compare = 0x%x\n"
	       "status = 0x%x, cause = 0x%x\n"
	       "eimr = 0x%llx, eirr = 0x%llx\n",
	       __FUNCTION__, 
	       read_c0_count(),
	       read_c0_compare(),
	       read_c0_status(),
	       read_c0_cause(),
	       (unsigned long long)read_64bit_cp0_eimr(),
	       (unsigned long long)read_64bit_cp0_eirr()
		);
}

void static add_region(struct boot_mem_map_exclude_region *x, int *k,
		       uint64_t start, uint64_t end)
{
	if (*k > MAX_EXCLUDE) {
		printk("No of exclude regions = %d; Cannot add more\n", MAX_EXCLUDE);
		return;
	}

	if (start < x[*k-1].end) {
		return;
	}

	x[*k].start = start;
	x[*k].end = end;
	++*k;
}

static int merge_exclude_regions(struct boot_mem_map_exclude_region *x,
				 struct boot_mem_map_exclude_region *y)
{
	static int _index = 0;
	int i, j, k;

	i = j = 0;
	k = 1;

	while (x[i].start != 0 && y[j].start != 0) {
		if (x[i].start < y[j].start) {
			add_region(_exclude_regions[_index], &k, x[i].start, x[i].end);
			++i;
		}
		else {
			add_region(_exclude_regions[_index], &k, y[j].start, y[j].end);
			++j;
		}
	}

	if (x[i].start == 0) {
		while (y[j].start) {
			add_region(_exclude_regions[_index], &k, y[j].start, y[j].end);
			++j;
		}
	}
	else if (y[j].start == 0) {
		while (x[i].start) {
			add_region(_exclude_regions[_index], &k, x[i].start, x[i].end);
			++i;
		}
	}

	exclude_regions = &_exclude_regions[_index][1];
	_index = _index ? 0 : 1;

	return 0;
}


#ifdef CONFIG_EARLY_PRINTK

static void NS16550_putc(char c)
{
	nlm_reg_t *mmio = 
		netlogic_io_mmio(NETLOGIC_IO_UART_0_OFFSET);

	while (netlogic_read_reg(mmio, 0x5) == 0);
	netlogic_write_reg(mmio, 0x0, c);
}

void prom_putchar(char c)
{
	void (*putchar)(char);

	putchar = ((void (*)(char c))(unsigned long)(&NS16550_putc));
	putchar(c);
}
#endif

static int __init nlm_proc_setup(void)
{
	nlm_root_proc = proc_mkdir("netlogic", 0);	
	if (!nlm_root_proc)
		return -ENOMEM;
	
	return 0;
}
rootfs_initcall(nlm_proc_setup);

static int get_xls_proc_name(void)
{
        int processor_id = ((read_c0_prid() & 0xff00) >> 8);
        switch ( processor_id ) {
        case CHIP_PROCESSOR_ID_XLS_608:
        case CHIP_PROCESSOR_ID_XLS_608_B0:
                strcpy(cpu_model_info,"XLS608");
                break;
        case CHIP_PROCESSOR_ID_XLS_408:
        case CHIP_PROCESSOR_ID_XLS_408_B0:
                strcpy(cpu_model_info,"XLS408");
                break;
        case CHIP_PROCESSOR_ID_XLS_404:
        case CHIP_PROCESSOR_ID_XLS_404_B0:
                strcpy(cpu_model_info,"XLS404");
                break;
        case CHIP_PROCESSOR_ID_XLS_208:
                strcpy(cpu_model_info,"XLS208");
                break;
        case CHIP_PROCESSOR_ID_XLS_204:
                strcpy(cpu_model_info,"XLS204");
                break;
        case CHIP_PROCESSOR_ID_XLS_616_B0:
                strcpy(cpu_model_info,"XLS616");
                break;
        case CHIP_PROCESSOR_ID_XLS_416_B0:
                strcpy(cpu_model_info,"XLS416");
                break;
        case CHIP_PROCESSOR_ID_XLS_412_B0:
                strcpy(cpu_model_info,"XLS412");
                break;
        case CHIP_PROCESSOR_ID_XLS_108:
                strcpy(cpu_model_info,"XLS108");
                break;
        case CHIP_PROCESSOR_ID_XLS_104:
                strcpy(cpu_model_info,"XLS104");
                break;
        default:
                strcpy(cpu_model_info,"XLS???");
                return -1;
        }
        return 0;
}

static int get_xlr_proc_name(void )
{
        int processor_id = ((read_c0_prid() & 0xff00) >> 8);
        if ( xlr_revision_c()) {
                switch ( processor_id ) {
                case CHIP_PROCESSOR_ID_XLR_C_308:
                        strcpy(cpu_model_info,"XLR308");
                        break;
                case CHIP_PROCESSOR_ID_XLR_C_508:
                        strcpy(cpu_model_info,"XLR508");
                        break;
                case CHIP_PROCESSOR_ID_XLR_C_516:
                        strcpy(cpu_model_info,"XLR516");
                        break;
                case CHIP_PROCESSOR_ID_XLR_C_532:
                        strcpy(cpu_model_info,"XLR532");
                        break;
                case CHIP_PROCESSOR_ID_XLR_C_716:
                        strcpy(cpu_model_info,"XLR716");
                        break;
                case CHIP_PROCESSOR_ID_XLR_C_732:
                        strcpy(cpu_model_info,"XLR732");
                        break;
                default:
                        strcpy(cpu_model_info,"XLR???");
                        return -1;
                }
        } else {
                switch ( processor_id ) {
                case CHIP_PROCESSOR_ID_XLR_B_308:
                        strcpy(cpu_model_info,"XLR308");
                        break;
                case CHIP_PROCESSOR_ID_XLR_B_508:
                        strcpy(cpu_model_info,"XLR508");
                        break;
                case CHIP_PROCESSOR_ID_XLR_B_516:
                        strcpy(cpu_model_info,"XLR516");
                        break;
                case CHIP_PROCESSOR_ID_XLR_B_532:
                        strcpy(cpu_model_info,"XLR532");
                        break;
                case CHIP_PROCESSOR_ID_XLR_B_716:
                        strcpy(cpu_model_info,"XLR716");
                        break;
                case CHIP_PROCESSOR_ID_XLR_B_732:
                        strcpy(cpu_model_info,"XLR732");
                        break;
                default:
                        strcpy(cpu_model_info,"XLR???");
                        return -1;
                }
        }
        return 0;
}

static int get_xlr_revision(void )
{
        int revision =  xlr_revision();
        switch( revision ) {
        case XLR_REVISION_A0:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev A0");
                break;
        case XLR_REVISION_A1:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev A1");
                break;
        case XLR_REVISION_B0:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev B0");
                break;
        case XLR_REVISION_B1:
                strcpy(cpu_model_info+ PROCESSOR_ID_MAX_LEN," Rev B1");
                break;
        case XLR_REVISION_B2:
                strcpy(cpu_model_info+ PROCESSOR_ID_MAX_LEN," Rev B2");
                break;
        case XLR_REVISION_C0:
                strcpy(cpu_model_info + PROCESSOR_ID_MAX_LEN," Rev C0");
                break;
        case XLR_REVISION_C1:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev C1");
                break;
        case XLR_REVISION_C2:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev C2");
                break;
        case XLR_REVISION_C3:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev C3");
                break;
        case XLR_REVISION_C4:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev C4");
                break;
        default:
                strcpy(cpu_model_info+PROCESSOR_ID_MAX_LEN," Rev ??");
                return -1;
        }
        return 0;
}

char* get_cpu_info()
{
                if (is_xls())
                        get_xls_proc_name();
                else
                        get_xlr_proc_name();

                get_xlr_revision();

		return cpu_model_info;
}
