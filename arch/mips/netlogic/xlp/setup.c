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

/*
 * Setup code for Netlogic's XLP-based boards
 */

#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/pm.h>

#include <asm/netlogic/xlp_irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>

#include <asm/mipsregs.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/nlm_pcix_gen_dev.h>
#include <asm/netlogic/memory-exclusion.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/mach-netlogic/mmu.h>
#include <asm/netlogic/bootinfo.h>
#include <asm/netlogic/cpumask.h>
#include <asm/netlogic/xlp_ici.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/phnx_loader.h>
#include "../boot/ops.h"
#include <asm/netlogic/xlp8xx/cpu_control_macros.h>

#include <asm/netlogic/xlp-hal/iomap.h>

void parse_cmdline(void);
/* Certain macros for this file
 */

#define TRUE						1
#define FALSE						0

#define DEFAULT_LINUX_CPU_MASK		0x1

#define PER_CPU_THREAD_SIZE			(THREAD_SIZE >> 2)
#define TOTAL_THREAD_SIZE			(PER_CPU_THREAD_SIZE * (NR_CPUS - 1))

#define BOOT_LOADER_REGION_SZ		0x04000000
#define LOADER_KSEG_END				0x10000000

#define PROCESSOR_ID_MAX_LEN		6

extern char _end;
extern void *fdt;

EXPORT_SYMBOL(fdt);
EXPORT_SYMBOL(dt_ops);

extern void *fdt_init(void *blob);
extern void *simple_alloc_init(char *base, unsigned long heap_size,
		unsigned long granularity, unsigned long max_allocs);

extern unsigned int xlp_uart_in(struct uart_port *p, int offset);
extern void xlp_uart_out(struct uart_port *p, int offset, int value);

extern void (*board_nmi_handler_setup)(void );
extern unsigned long _text[];
extern cpumask_t fdt_cpumask;
//extern cpumask_t fdt_loadermask;
//int xlp_loader_support = 0;

#ifdef CONFIG_NUMA
int hcpu_to_lcpu[NR_CPUS];
#endif

extern uint32_t nlm_cpu_vc_mask[NLM_MAX_CPU_NODE*NLM_MAX_CPU_PER_NODE];

struct proc_dir_entry *nlm_root_proc;
EXPORT_SYMBOL(nlm_root_proc);

unsigned long nlm_common_ebase = 0x0;

char cpu_model_info[MAX_CPU_REV_LEN] = {'X','L','P'};

static unsigned int xlp_uart_portid = 0;

static char prop_buf[MAX_PROP_LEN];

void *nlm_common_psb_shm = 0;
unsigned long nlm_common_psb_shm_size = 0;

#ifdef CONFIG_NUMA
struct nlm_node_mem_info node_mem_info[NLM_MAX_CPU_NODE];
/* number of nodes */
static int nlm_nodes=1;
#endif

#ifdef CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID
unsigned long nlm_asid_mask = 0x3f;
unsigned int nlm_shtlb = 1; /* by default shared TLB is enabled */
#endif

const char *DEFAULT_CONSOLE_BOOT_PARAMS = "boot_noi2c mem=255m@1m mem=512m@512m console=ttyS0,115200 ";
const char *DEFAULT_INITRD_BOOT_PARAMS = "rdinit=/sbin/init ";

#ifdef CONFIG_SMP
atomic_t cpus_rebooted = ATOMIC_INIT(0);
#endif

unsigned long long nlm_common_tlb_stats[NR_CPUS] __cacheline_aligned;
spinlock_t atomic_lock = SPIN_LOCK_UNLOCKED;

int hwemul = 0;
EXPORT_SYMBOL(hwemul);

/* Used to be a flag here only for xlp8xx a0, and potentially a1.
 * Removing since all of those should be gone from the field */

/* Struct for temp. allocation
 * of sp/gp for secondary CPUs
 */
struct xlp_stack_pages {
	unsigned long stack[(TOTAL_THREAD_SIZE)/sizeof(long)];
};

struct xlp_stack_pages xlp_stack_pages_temp
		__attribute__((__section__(".data.init_task"),
		__aligned__(THREAD_SIZE)));

uint64_t nlm_io_base;
struct nlm_ici_vc_param
{
	int own_credit;
	int shared_credit;
	int txwght;
	int segth;
};

struct nlm_ici_config
{
	int enable_config;
	int total_credits;
	int node_link_mask[NLM_MAX_NODES];
	struct nlm_ici_vc_param gcu_vc[ICI_MAX_GCU_VC];
	struct nlm_ici_vc_param fmn_vc[ICI_MAX_FMN_VC];
	struct nlm_ici_vc_param pic_vc[ICI_MAX_PIC_VC];
} nlm_ici_config;

struct boot_mem_map boot_physaddr_info;
struct xlp_dram_mapping {	  
		unsigned long low_pfn; 
		unsigned long high_pfn;	
		int node;   
};

#define NLM_NODES_MAX_DRAM_REGION (NLM_MAX_DRAM_REGION * NLM_MAX_NODES)
struct xlp_dram_mapping dram_map[NLM_NODES_MAX_DRAM_REGION];	
	  
#define NLM_DRAM_BASE_REG_0		20
#define NLM_DRAM_LIMIT_REG_0	28
#define NLM_DRAM_NODEADDR_XLAT	36
#define HDR_OFFSET				0x100
#define BRIDGE					(0x00<<20) | (0x00<<15) | (0x00<<12)	

int __init nlm_common_get_pgprot(unsigned long address)
{
	int i = 0;

	for (i = 0; i < boot_physaddr_info.nr_map; i++) {
		__u64 start = 0, end = 0;
		long type = 0;

		start = boot_physaddr_info.map[i].addr;
		end = start + boot_physaddr_info.map[i].size;
		type = boot_physaddr_info.map[i].type;

		if (address >= start && address < end) {
			/* Uncached */
			if (type == BOOT_MEM_RESERVED) return 1;
			/* cached */
			if (type == BOOT_MEM_RAM) return 0;
		}
	}

	/* uncached */
	return 1;
}

int __init valid_mmap_nlm_common_addr_range(unsigned long pfn)
{
	int i;
	__u64 end=0;

	for (i = 0; i < boot_physaddr_info.nr_map; i++) {
		end = boot_physaddr_info.map[i].addr + boot_physaddr_info.map[i].size;
		end = end >> PAGE_SHIFT;
		if (pfn <= (unsigned long)end)
			return 1;
	}
	return 0;
}

void __init read_node_bars(int node)
{	 
	int i, idx;
	uint64_t membase = (uint64_t)cpu_io_mmio(node, BRIDGE);

	for (i = 0; i < NLM_MAX_DRAM_REGION; i++) {
		uint64_t base_reg  = nlm_hal_read_32bit_reg(membase, NLM_DRAM_BASE_REG_0 + i);
		uint64_t limit_reg = nlm_hal_read_32bit_reg(membase, NLM_DRAM_LIMIT_REG_0 + i);
		uint32_t node_reg =  nlm_hal_read_32bit_reg(membase, NLM_DRAM_NODEADDR_XLAT + i);

		if(((node_reg >> 1) & 0x3) != node) {
			continue;  
		}  
		if(((limit_reg >> 12) << 20) == 0) {	
			continue;  
		}  

		idx = (node * NLM_MAX_DRAM_REGION) + i;
		dram_map[idx].low_pfn = ((base_reg >> 12) << 20) >> PAGE_SHIFT;	  
		dram_map[idx].high_pfn =
			((limit_reg >> 12) << 20) >> PAGE_SHIFT;
		dram_map[idx].node = node;

		if(dram_map[idx].high_pfn == dram_map[idx].low_pfn){
			continue;
		}

		boot_physaddr_info.map[boot_physaddr_info.nr_map].addr = dram_map[idx].low_pfn << PAGE_SHIFT;     
		boot_physaddr_info.map[boot_physaddr_info.nr_map].size =
				(dram_map[idx].high_pfn - dram_map[idx].low_pfn + (1<<(20-PAGE_SHIFT))) << PAGE_SHIFT;
		boot_physaddr_info.map[boot_physaddr_info.nr_map].type = BOOT_MEM_RAM;
		boot_physaddr_info.nr_map++;
	}
}

void __init nlm_get_dram_mapping(void)
{
	int node;
	boot_physaddr_info.nr_map = 0;

	for(node=0; node < NLM_MAX_NODES; node++) {
		read_node_bars(node);
	}
}


const char *get_system_type(void)
{
	return "Netlogic XLP SoC";
}

static void ptr_linux_exit(void)
{
	// trigger a chip reset
	 nlm_hal_write_sys_reg(netlogic_node_id(), CHIP_RESET, 1);
	 for ( ; ; )
		  cpu_wait();
}

void __init bus_error_init(void)
{
}

#ifdef CONFIG_NLM_XLP
void __init prom_reconfigure_thr_resources(void) {}
#else
void __init prom_reconfigure_thr_resources(void)
{
	unsigned int mmu_setup = 0;
	int i = 0, num_threads = 0, dis_contig = 0;
	int value = 0;
	int cpu = 0;

	/* Configure thread resources only if it is thread_0 of that core */
	if (netlogic_thr_id() != 0) return;

#ifdef CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID
	uint32_t map;

	/* netlogic kernel configures this
	 */
	if (nlm_shtlb && (nlm_asid_mask == 0x3f)) {

		uint32_t online_map = smp_node.onlinemask[0];    /* from fdt */

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
			mmu_setup = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_MMU, 0);
			mmu_setup = mmu_setup | 0x1;
			write_32bit_nlm_ctrl_reg(CPU_BLOCKID_MMU, 0, mmu_setup);

			printk("CPU %d: Enabled Shared TLB mode \n",
					netlogic_cpu_id());
			return;
		}
	}
	return;
#endif /* CONFIG_NLMCOMMON_GLOBAL_TLB_SPLIT_ASID */

	/* cpu has to be thread@0 */
	cpu = hard_smp_processor_id();

	for (i = 0; i < 4; i++) {

		if (!cpumask_test_cpu(cpu + i, &fdt_cpumask)) continue;

		if (i != num_threads)	dis_contig = 1;

		num_threads++;
	}

	switch(num_threads) {
	case 1: value = 0x00; break;
	case 2: value = 0x02; break;
	default:
		value = 0x03; break;
	}

	if (dis_contig)	value = 0x3;

	mmu_setup = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_MMU, 0);
	mmu_setup = mmu_setup & ~0x06;
	mmu_setup |= (value << 1);
	write_32bit_nlm_ctrl_reg(CPU_BLOCKID_MMU, 0, mmu_setup);
}
#endif

#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
unsigned int __cpuinit get_c0_compare_int(void)
{
	return XLP_IRQ_TIMER_RVEC;
}
#endif

/* TODO: Get this from FDT */
void plat_time_init(void)
{
#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
	extern void nlm_common_timer_setup(void);
#endif
	mips_hpt_frequency = (unsigned int) nlm_hal_cpu_freq();
	printk(KERN_DEBUG "mips_hpt_frequency = %u\n", mips_hpt_frequency);
#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
	nlm_common_timer_setup();
#endif
}

void __init plat_mem_setup(void)
{
	extern int panic_timeout;

	panic_timeout		= 5;
	_machine_restart	= (void (*)(char *))ptr_linux_exit;
	_machine_halt		= ptr_linux_exit;
	pm_power_off		= ptr_linux_exit;
	return;
}

struct nlm_common_name_value_struct {
	char *name;
	uint32_t *val;
};

static void prom_add_memory(uint64_t start, uint64_t size) __attribute__((unused));
static void __init prom_add_memory(uint64_t start, uint64_t size)
{
	__u64 pref_backup = 512;

	add_memory_region(
			start, size - pref_backup, /* CHECK! */
			BOOT_MEM_RAM);
	return;
}


void __init nlm_nmi_setup (void)
{
	void *base;
	extern char nlm_except_vec_nmi;

	printk("Setting up NMI Handler \n");
	base = (void *)(unsigned long)0xffffffffbfc00000ULL;
	memcpy(base, &nlm_except_vec_nmi, 0x80);
}

#ifdef CONFIG_SERIAL_8250
/* setup early serial port driver */
extern int early_serial_setup(struct uart_port *port);
static void __init nlm_early_serial_setup(int uart_id)
{
	struct uart_port s;

	memset(&s, 0, sizeof(s));
	s.flags = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
	s.iotype = UPIO_NLM;
	s.type = PORT_16550;
	s.regshift = 2; /* registers are 4 bytes wide */
	/* hardware int 4 - the serial int, is CPU int 6
	 but poll for now */

	/* Replaced simple 133/66 test (likely pre-firefly) with call to HAL function below */
	s.uartclk = nlm_hal_get_ref_clk_freq();

	switch(uart_id){
		default:
		case 0:
			s.irq = XLP_UART_IRQ(0, 0);
			s.membase = (unsigned char __iomem *)
			(DEFAULT_NETLOGIC_IO_BASE + NETLOGIC_IO_UART_0_OFFSET);
			s.mapbase = (DEFAULT_NETLOGIC_IO_BASE +
					NETLOGIC_IO_UART_0_OFFSET);
			s.line = 0;
			break;
		case 1:
			s.irq = XLP_UART_IRQ(0, 1);
			s.membase = (unsigned char __iomem *)
			(DEFAULT_NETLOGIC_IO_BASE + NETLOGIC_IO_UART_1_OFFSET);
			s.mapbase = (DEFAULT_NETLOGIC_IO_BASE +
					NETLOGIC_IO_UART_1_OFFSET);
			s.line = 1;
			break;
	}
	s.serial_in	= xlp_uart_in;
	s.serial_out = xlp_uart_out;
	printk("Setting up early serial access\n");
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial setup failed!\n");
	}
}
#else
static void nlm_early_serial_setup(int uart_id) {}
#endif /* CONFIG_SERIAL_8250 */

extern struct plat_smp_ops nlm_smp_ops;
struct loader_mem_info loader_mem_map[MAX_NUM_LOADER_MEM_BLK];

#define MAX_CPUMASK_CELLS 4
#define fdt32_to_cpu(x) be32_to_cpu(x)

int nlm_get_fdt_app_param(const char *param, void *data, int size)
{
	void *node;
	node = finddevice("/doms/dom@0/app-param");
	if(node) {
		if (getprop(node, param, data, size) > 0)
			return 0;
	}
	return -1;
}
EXPORT_SYMBOL(nlm_get_fdt_app_param);

int nae_rx_vc = -1, nae_fb_vc = -1;
int sae_rx_vc = -1, sae_rx_sync_vc = -1;
int ipsec_async_vc = -1, ipsec_sync_vc = -1;
static void __init parse_fdt_sae_vc_config(void)
{
	void *node;
	void * valid_node;
	int i;
	int num_nodes = 1;

	node = finddevice("/doms/dom@0/cpu");
	if(node) {
		if (getprop(node, "nae-rx-vc", &nae_rx_vc, 4) > 0)
			nae_rx_vc = fdt32_to_cpu(nae_rx_vc);

		if (getprop(node, "nae-fb-vc", &nae_fb_vc, 4) > 0)
			nae_fb_vc = fdt32_to_cpu(nae_fb_vc);

		if (getprop(node, "sae-rx-vc", &sae_rx_vc, 4) > 0) 
			sae_rx_vc = fdt32_to_cpu(sae_rx_vc);
		
		if (getprop(node, "sae-rx-sync-vc", &sae_rx_sync_vc, 4) > 0)
			sae_rx_sync_vc = fdt32_to_cpu(sae_rx_sync_vc);

		if (getprop(node, "ipsec-async-vc", &ipsec_async_vc, 4) > 0)
			ipsec_async_vc = fdt32_to_cpu(ipsec_async_vc);

		if (getprop(node, "ipsec-sync-vc", &ipsec_sync_vc, 4) > 0)
			ipsec_sync_vc = fdt32_to_cpu(ipsec_sync_vc);

		valid_node  = finddevice("/soc/nodes");

		if (getprop(valid_node, "num-nodes", &num_nodes, 4) > 0 )
			num_nodes = fdt32_to_cpu(num_nodes);

		for(i =0 ; i < num_nodes*NLM_MAX_CPU_PER_NODE; i++) {
			if(nlm_cpu_vc_mask[i] & (1 << ipsec_sync_vc)) {
				ipsec_sync_vc = -1;
			}

		}
	}

	return;
}
EXPORT_SYMBOL(nae_rx_vc);
EXPORT_SYMBOL(nae_fb_vc);
EXPORT_SYMBOL(sae_rx_vc);
EXPORT_SYMBOL(sae_rx_sync_vc);
EXPORT_SYMBOL(ipsec_async_vc);
EXPORT_SYMBOL(ipsec_sync_vc);

static void __init ici_read_vc_parameter(void *node, int max_vc, struct nlm_ici_vc_param *vc_index)
{
	uint32_t tmp_data[32];
	int i = 0;

	if(getprop(node, "own_credit", &tmp_data, sizeof(int)*max_vc) <= 0) {
		for(i=0; i<max_vc; i++){
			(vc_index+i)->own_credit = 64;
		}
	}else{
		for(i=0; i<max_vc; i++){
			(vc_index+i)->own_credit = tmp_data[i];
		}
	}

	if(getprop(node, "shared_credit", &tmp_data, sizeof(int)*max_vc) <= 0) {
		for(i=0; i<max_vc; i++){
			(vc_index+i)->shared_credit = 1;
		}
	}else{
		for(i=0; i<max_vc; i++){
			(vc_index+i)->shared_credit = tmp_data[i];
		}
	}

	if(getprop(node, "txwght", &tmp_data, sizeof(int)*max_vc) <= 0) {
		for(i=0; i<max_vc; i++){
			(vc_index+i)->txwght = 64;
		}
	}else{
		for(i=0; i<max_vc; i++){
			(vc_index+i)->txwght = tmp_data[i];
		}
	}

	if(getprop(node, "segth", &tmp_data, sizeof(int)*max_vc) <= 0) {
		for(i=0; i<max_vc; i++){
			(vc_index+i)->segth = 2;
		}
	}else{
		for(i=0; i<max_vc; i++){
			(vc_index+i)->segth = tmp_data[i];
		}
	}
}

#if 0
static void ici_dump_vc_info(const char *header, int max_vc, struct nlm_ici_vc_param *vc)
{
	printk("[=== %s ===]\n", header);
	
	if(max_vc == 8){
		printk("OWN:    %d %d %d %d %d %d %d %d\n",(vc+0)->own_credit, (vc+1)->own_credit, (vc+2)->own_credit, 
				(vc+3)->own_credit,	(vc+4)->own_credit, (vc+5)->own_credit, (vc+6)->own_credit, (vc+7)->own_credit);
		printk("SHARED: %d %d %d %d %d %d %d %d\n",(vc+0)->shared_credit, (vc+1)->shared_credit, (vc+2)->shared_credit, 
				(vc+3)->shared_credit,	(vc+4)->shared_credit, (vc+5)->shared_credit, (vc+6)->shared_credit, 
				(vc+7)->shared_credit);
		printk("TXWGHT: %d %d %d %d %d %d %d %d\n",(vc+0)->txwght, (vc+1)->txwght, (vc+2)->txwght, (vc+3)->txwght,
				(vc+4)->txwght, (vc+5)->txwght, (vc+6)->txwght, (vc+7)->txwght);
		printk("SEGTH:  %d %d %d %d %d %d %d %d\n",(vc+0)->segth, (vc+1)->segth, (vc+2)->segth, (vc+3)->segth,
				(vc+4)->segth, (vc+5)->segth, (vc+6)->segth, (vc+7)->segth);
	}else if (max_vc == 4){
		printk("OWN:    %d %d %d %d\n",(vc+0)->own_credit, (vc+1)->own_credit, (vc+2)->own_credit, (vc+3)->own_credit);
		printk("SHARED: %d %d %d %d \n",(vc+0)->shared_credit, (vc+1)->shared_credit, (vc+2)->shared_credit, 
				(vc+3)->shared_credit);
		printk("TXWGHT: %d %d %d %d\n",(vc+0)->txwght, (vc+1)->txwght, (vc+2)->txwght, (vc+3)->txwght);
		printk("SEGTH:  %d %d %d %d\n",(vc+0)->segth, (vc+1)->segth, (vc+2)->segth, (vc+3)->segth);

	}else if(max_vc == 1){
		printk("OWN:    %d\n",(vc+0)->own_credit);
		printk("SHARED: %d \n",(vc+0)->shared_credit);
		printk("TXWGHT: %d \n",(vc+0)->txwght);
		printk("SEGTH:  %d \n",(vc+0)->segth);
	}
}
#endif

static void __init parse_ici_parameters(void)
{
	char domstr[32] = "";
	void *node;
	int domain = 0;

	sprintf(domstr, "/doms/dom@%d/ici-config", domain);
	node = finddevice(domstr);
	if(node){
			if(getprop(node, "enable", &nlm_ici_config.enable_config, sizeof(int)) <= 0){
				nlm_ici_config.enable_config = 0;
			}
			if(getprop(node, "total_credits", &nlm_ici_config.total_credits, sizeof(int)) <= 0){
				nlm_ici_config.total_credits = 1023;
			}
			if(getprop(node, "node_0_link_mask", &nlm_ici_config.node_link_mask[0], sizeof(int)) <= 0){
				nlm_ici_config.node_link_mask[0] = 0;
			}
			if(getprop(node, "node_1_link_mask", &nlm_ici_config.node_link_mask[1], sizeof(int)) <= 0){
				nlm_ici_config.node_link_mask[1] = 0;
			}
			if(getprop(node, "node_2_link_mask", &nlm_ici_config.node_link_mask[2], sizeof(int)) <= 0){
				nlm_ici_config.node_link_mask[2] = 0;
			}
			if(getprop(node, "node_3_link_mask", &nlm_ici_config.node_link_mask[3], sizeof(int)) <= 0){
				nlm_ici_config.node_link_mask[3] = 0;
			}
	}
	if(!nlm_ici_config.enable_config){
		printk(KERN_DEBUG "ICI config not enabled\n");
		goto end;
	}

	sprintf(domstr, "/doms/dom@%d/ici-config/gcu", domain);
	node = finddevice(domstr);
	if(node){
			ici_read_vc_parameter(node, ICI_MAX_GCU_VC, &nlm_ici_config.gcu_vc[0]);
	}

	sprintf(domstr, "/doms/dom@%d/ici-config/fmn", domain);
	node = finddevice(domstr);
	if(node){
			ici_read_vc_parameter(node, ICI_MAX_FMN_VC, &nlm_ici_config.fmn_vc[0]);
	}

	sprintf(domstr, "/doms/dom@%d/ici-config/pic", domain);
	node = finddevice(domstr);
	if(node){
			ici_read_vc_parameter(node, ICI_MAX_PIC_VC, &nlm_ici_config.pic_vc[0]);
	}
#if 0
	printk("\n:::::::::ICI CONFIGURATION:::::::::\n");
	printk("TotalCredits %d, EnableConfig %d\n",nlm_ici_config.total_credits, nlm_ici_config.enable_config);
	ici_dump_vc_info("GCU", ICI_MAX_GCU_VC, &nlm_ici_config.gcu_vc[0]);
	ici_dump_vc_info("FMN", ICI_MAX_FMN_VC, &nlm_ici_config.fmn_vc[0]);
	ici_dump_vc_info("PIC", ICI_MAX_PIC_VC, &nlm_ici_config.pic_vc[0]);
	printk("LinkMask: Node0: %#x, Node1: %#x, Node2: %#x, Node3: %#x\n", nlm_ici_config.node_link_mask[0],
			nlm_ici_config.node_link_mask[1], nlm_ici_config.node_link_mask[2], nlm_ici_config.node_link_mask[3]);
#endif
end:
	return;
}

static int __init fdt_process(void)
{
	int  domain=0;
	char domstr[32] = "";
	uint32_t tmp, i, na, ns, regs[MAX_PROP_LEN / 4], entries, cpu_cells;
	uint32_t node_vc_mask[NLM_MAX_CPU_NODE] = {0};
	unsigned char buf[30];
	int j, id=0, k;
	uint32_t onlinemask[MAX_CPUMASK_CELLS];
// 	uint32_t linux_loader_mask[MAX_CPUMASK_CELLS] = {0};

	/* If booted using FDT and U-Boot, all
	 * we get is a pointer to an FDT Blob
	 */
	void *blob = (void *)fw_arg0;
	void *node;

	cpumask_clear(&fdt_cpumask);
//	cpumask_clear(&fdt_loadermask);

	if(!blob)
		return -1;
	/* Create a region starting from
	 * (_end + 64K) of size 1MB for
	 * the FDT structures. The 64K is
	 * the current page size for XEN
	 */
	simple_alloc_init((char *)((unsigned long)(&_end)+0x10000),
			(1<<20), 32, 128);

	/* Create a local copy of the FDT
	 */
	fdt = fdt_init(blob);
	if(!fdt)
		return -1;
	printk(KERN_DEBUG "Cached handle for FDT @ %p\n", fdt);

	/* extract cmdline params
	 */
	node = finddevice("/chosen");
	if (node) {
		if (getprop(node, "bootargs", prop_buf, MAX_PROP_LEN) <= 0)
			return -1;
		strcat(arcs_cmdline, prop_buf);
	}

	/* extract memory ranges,
	 * add to command line
	 */
	node = finddevice("/doms/dom@0");
	if (node) {
#ifdef CONFIG_NUMA
		if (getprop(node, "#nodes", &nlm_nodes, sizeof(nlm_nodes)) <= 0)
			nlm_nodes = 1;
		else
			nlm_nodes = fdt32_to_cpu(nlm_nodes);
#endif

		if (getprop(node, "#address-cells", &na, sizeof(na)) <= 0)
			na = 1;
		else
			na = fdt32_to_cpu(na);
		if (na < 1 || na > 2)
			printk(KERN_ERR "Can't cope with #address-cells == %d.\n\r", na);

		if (getprop(node, "#size-cells", &ns, sizeof(ns)) <= 0)
			ns = 1;
		else
			ns = fdt32_to_cpu(ns);
		if (ns < 1 || ns > 2)
			printk(KERN_ERR "Can't cope with #size-cells == %d.\n\r", ns);

		if (getprop(node, "#cpumask-cells", &cpu_cells, sizeof(cpu_cells)) <= 0)
			cpu_cells = 1;
		else
			cpu_cells = fdt32_to_cpu(cpu_cells);

		if (cpu_cells < 1 || cpu_cells > MAX_CPUMASK_CELLS)
			printk(KERN_ERR "Can't cope with #cpumask-cells == %d\n\r", cpu_cells);
	}

	node = finddevice("/doms/dom@0/memory");
	if (node) {
		entries = (getprop(node, "reg", regs, MAX_PROP_LEN)) / sizeof(regs[0]);
		if (!entries || (entries % (na+ns)))
			printk(KERN_ERR "Invalid Memory Map Specified!\n");

		for (i=0; i<entries; i+=na+ns) {
			int base = i;
			uint64_t addr, size;

			addr = fdt32_to_cpu(regs[base++]);
			if (na == 2)
			{
				/* handle 2 address-cells (ie. 64-bits of address) */
				addr <<= 32;
				addr |= fdt32_to_cpu(regs[base++]);
			}

			size = fdt32_to_cpu(regs[base++]);
			if (ns == 2)
			{
				/* handle 2 size-cells (ie. 64-bits of size) */
				size <<= 32;
				size |= fdt32_to_cpu(regs[base++]);
			}

			sprintf(domstr, " mem=%lldm@%lldm ", (size >> 20), (addr >> 20));
			strcat(arcs_cmdline, domstr);
			memset((void *)&domstr, '\0', sizeof(domstr));
		}
	}

	printk("FDT Cmdline: %s\n", arcs_cmdline);

	/*
	 * extract CPU online mask for domain 0 (linux)
	 */
	for (i = 0; i < MAX_CPUMASK_CELLS; i++)
		onlinemask[i] = 0;

	sprintf(domstr, "/doms/dom@%d/cpu", domain);

	node = finddevice(domstr);
	if (node) {
		uint32_t onlinemask_buf[MAX_CPUMASK_CELLS];

		/* Initialize buffers */
		for (i = 0; i < MAX_CPUMASK_CELLS; i++)
			onlinemask_buf[i] = 0;

		/* Parse cpumask from FDT and handle endianness */
		if (getprop(node, "onlinemask", &onlinemask_buf[0], sizeof(uint32_t) * cpu_cells) > 0) {
			for (i = 0; i < cpu_cells; i++) {
				onlinemask_buf[i] = fdt32_to_cpu(onlinemask_buf[i]);

//			printk("FDT: cpu_cells: %d onlinemask[%d]: %08x\n",
//					cpu_cells, i, onlinemask_buf[i]);
		}

			/* Store cpumask in predefined order */
			for (i = 0; i < cpu_cells; i++)
				onlinemask[i] = onlinemask_buf[cpu_cells - 1 - i];

			for (i = 0; i < MAX_CPUMASK_CELLS; i++) {
				int j = 0;

				for (j = 0; j < 32; j++) {
					if ((onlinemask[i] & (1 << j)) == 0)
						continue;
					cpumask_set_cpu((i * 32 + j), &fdt_cpumask);
				}
			}
		}

		/* Removed napi-vc-mask - only one NAPI driver allowed and it should
		 * register via xlp_register_napi_handler
		 */

//		cpumask_scnprintf(buf, CPUMASK_BUF, &fdt_cpumask);
//		printk("fdt_cpumask: %s\n", buf);
	}

#if 0
	sprintf(domstr, "/doms/dom@%d/linuxloader", domain);
	node = finddevice(domstr);
	if (node) {
		uint32_t loadermask_buf[MAX_CPUMASK_CELLS], index;
		char buf[CPUMASK_BUF];

		xlp_loader_support = 1;
		/* Initialize buffers */
		for (i = 0; i < MAX_CPUMASK_CELLS; i++)
			loadermask_buf[i] = 0;

		/* Parse cpumask from FDT and handle endianness */
		if((getprop(node, "loadermask", &loadermask_buf[0], sizeof(uint32_t) * cpu_cells) < 0)){
			xlp_loader_support = 0;
			goto noloadermask;
		}

		for (i = 0; i < cpu_cells; i++) {
			loadermask_buf[i] = fdt32_to_cpu(loadermask_buf[i]);

			printk("FDT: cpu_cells: %d loadermask_buf[%d]: %#08x\n",
					cpu_cells, i, loadermask_buf[i]);
		}

		/* Store cpumask in predefined order */
		for (i = 0; i < cpu_cells; i++){
			linux_loader_mask[i] = loadermask_buf[cpu_cells - 1 - i];
			printk("linux_loader_mask[%d] = %#x\n",i,linux_loader_mask[i]);
		}

		for (i = 0; i < MAX_CPUMASK_CELLS; i++) {
			int j = 0;

			for (j = 0; j < 32; j++) {
				if ((linux_loader_mask[i] & (1 << j)) == 0)
					continue;
				cpumask_set_cpu((i * 32 + j), &fdt_loadermask);
			}
		}

		entries = (getprop(node, "memory", regs, sizeof(regs))) / sizeof(regs[0]);
		if (!entries || (entries % (na+ns))){
			printk(KERN_ERR "Invalid Memory Map Specified!\n");
			xlp_loader_support = 0;
			goto noloadermask;
		}
		for (i=0,index=0; i<entries; i+=4, index++) {
				unsigned long long lsb_addr, msb_addr, lsb_size, msb_size;
				msb_addr = fdt32_to_cpu(regs[i]);
				lsb_addr = fdt32_to_cpu(regs[i + 1]);
				msb_size = fdt32_to_cpu(regs[i + 2]);
				lsb_size = fdt32_to_cpu(regs[i + 3]);
				loader_mem_map[index].start_addr = lsb_addr | (msb_addr << 32);
				loader_mem_map[index].size  = lsb_size | (msb_size << 32);
				printk("LoaderMemory [%#llx] @ [%#llx]\n",loader_mem_map[index].size, loader_mem_map[index].start_addr);
		}
noloadermask:
		cpumask_scnprintf(buf, CPUMASK_BUF, &fdt_loadermask);
		printk("fdt_loadermask: %s\n", buf);
	}
#endif

	node = finddevice("/doms/dom@0/fmn");
	if (node) {
		for (i = 0; i < NLM_MAX_CPU_NODE; i++) {
			sprintf(buf, "node_%d_vc_mask", i);
			memset(&node_vc_mask, 0, sizeof(node_vc_mask));
			if (getprop(node, buf, &node_vc_mask, sizeof(node_vc_mask)) > 0) {
				for (j = 3; j >= 0; j--) {
					tmp = fdt32_to_cpu(node_vc_mask[j]);
					for (k = 0; k < 8; k++) {
						nlm_cpu_vc_mask[id++] = (tmp >> (k * 4)) & 0xf;
					}
				}
			}
		}
	}

	sprintf(domstr, "/doms/dom@%d/uart", domain);
	node = finddevice(domstr);
	if (node) {
		if (getprop(node, "id", &xlp_uart_portid, sizeof(xlp_uart_portid)) <= 0)
			return -1;
	}
	/* Parse the sae async/sync vcs for linux userspace model */
	parse_fdt_sae_vc_config();

	parse_ici_parameters();
	return 0;
}

char* get_cpu_info(void)
{
	struct nlm_netl_proc_info cpu_info;
	nlm_hal_get_cpuinfo(&cpu_info);
	strcpy(cpu_model_info, cpu_info.cpu_info_str);
	return cpu_model_info;
}

#ifdef CONFIG_XEN
extern void xen_init(void);
#else
static void __init xen_init(void) {}
#endif

#ifdef CONFIG_NUMA
static void __init sort_mem_info(struct nlm_node_mem_info *info, unsigned long *spfn,
	unsigned long *epfn)
{
	struct nlm_node_mem_frag *list = info->mem;
	int i,j;

	uint64_t start_pfn = 0;
	uint64_t end_pfn = 0;

	*spfn = *epfn = 0;
	if(info->frags == 0)
		return;

	for(i=0; i < info->frags; i++) {
		for (j = i; j < info->frags; j++) {
			if (list[i].start_pfn > list[j].start_pfn) {
				start_pfn = list[i].start_pfn;
				end_pfn = list[i].end_pfn;
				list[i].start_pfn = list[j].start_pfn;
				list[i].end_pfn = list[j].end_pfn;
				list[j].start_pfn = start_pfn;
				list[j].end_pfn = end_pfn;
			}
		}
	}
	*spfn = list[0].start_pfn;
	*epfn = list[info->frags-1].end_pfn;
}

void __init prom_meminit(void)
{
	int node=0;
	unsigned long start_pfn, end_pfn;
	struct nlm_mem_info *minfo;

	/* sort the node_mem_map */
	for(node=0; node < nlm_nodes; node++) {
		sort_mem_info(&node_mem_info[node], &start_pfn, &end_pfn);
		minfo = NODE_MEM_DATA(node);
		minfo->low_pfn = start_pfn;
		minfo->high_pfn = end_pfn;
	}
}

extern struct nlm_node_data __node_data_holder[];
void __init build_node_cpu_map(void)
{
	int cpu, node,i;

	/* kernel expects all node_data to initialized
 	 * If a node has its own memory, we will overwrite this pointer
 	 */
	for(node=0; node < NLM_MAX_NODES; node++) {
		__node_data[node] = &__node_data_holder[node];
	}

	i=0;
	for_each_cpu(cpu, &fdt_cpumask) {
		node = hardcpu_to_node(cpu);
		hcpu_to_lcpu[cpu] = i;
		if(!node_online(node)) {
			node_set_online(num_online_nodes());
		}
		i++;
	}
	printk(KERN_DEBUG "Number of online nodes = %d\n", num_online_nodes());

}

static void __init nlm_update_ici_credits(int node, int link)
{
	volatile uint32_t *mmio;
	volatile int i = 0;

	mmio = (volatile uint32_t *)(0xffffffffb0000000ULL|(unsigned long)ici_io_mmio(node, link));

	/*Configure GCU vcs*/
	for(i=0; i<ICI_MAX_GCU_VC; i++){
		mmio[ICI_CREDOWN0 + ICI_GCU_VC_START + i] = nlm_ici_config.gcu_vc[i].own_credit;
		mmio[ICI_CREDSHARE0 + ICI_GCU_VC_START + i] = nlm_ici_config.gcu_vc[i].shared_credit;
		mmio[ICI_TXWGHT0 + ICI_GCU_VC_START + i] = nlm_ici_config.gcu_vc[i].txwght;
		mmio[ICI_TXSEGTH0 + ICI_GCU_VC_START + i] = nlm_ici_config.gcu_vc[i].segth;
	}

	/*Configure FMN vcs*/
	for(i=0; i<ICI_MAX_FMN_VC; i++){
		mmio[ICI_CREDOWN0 + ICI_FMN_VC_START + i] = nlm_ici_config.fmn_vc[i].own_credit;
		mmio[ICI_CREDSHARE0 + ICI_FMN_VC_START + i] = nlm_ici_config.fmn_vc[i].shared_credit;
		mmio[ICI_TXWGHT0 + ICI_FMN_VC_START + i] = nlm_ici_config.fmn_vc[i].txwght;
		mmio[ICI_TXSEGTH0 + ICI_FMN_VC_START + i] = nlm_ici_config.fmn_vc[i].segth;
	}

	/*Configure PIC vcs*/
	for(i=0; i<ICI_MAX_PIC_VC; i++){
		mmio[ICI_CREDOWN0 + ICI_PIC_VC_START + i] = nlm_ici_config.pic_vc[i].own_credit;
		mmio[ICI_CREDSHARE0 + ICI_PIC_VC_START + i] = nlm_ici_config.pic_vc[i].shared_credit;
		mmio[ICI_TXWGHT0 + ICI_PIC_VC_START + i] = nlm_ici_config.pic_vc[i].txwght;
		mmio[ICI_TXSEGTH0 + ICI_PIC_VC_START + i] = nlm_ici_config.pic_vc[i].segth;
	}

	/*Configure Total Credits*/
	mmio[ICI_CREDTOT] = nlm_ici_config.total_credits;

	/*Update Credit Load*/
	mmio[ICI_CREDLD] = mmio[ICI_CREDLD] | 0x1;

	printk("Reconfigured ICI Node %d, Link %d\n", node, link);
}

int __init nlm_config_ici(void)
{
	/*configure ICI*/
	int node, link;
	uint32_t active_link;

	if(!nlm_ici_config.enable_config){
		return -1;
	}

	for(node=0; node<NLM_MAX_NODES; node++){
		if(node_online(node)) {
			active_link = nlm_ici_config.node_link_mask[node];
			for(link=0; link<ICI_NUM_LINKS; link++){
				if(active_link & (1<<link)){
					nlm_update_ici_credits(node, link);
				}
			}
		}
	}
}

#endif

void __init prom_init(void)
{
	unsigned int c0status;

#ifdef CONFIG_MAPPED_KERNEL
	setup_mapped_kernel_tlbs(TRUE, TRUE);
#endif

	fdt_process();
	parse_cmdline();

#ifdef CONFIG_NUMA
	build_node_cpu_map();
	nlm_config_ici();
#endif

	xen_init();

	nlm_common_ebase = read_c0_ebase() & (~((1 << 12) - 1));

	// workaround. trap_init enables cop2. But this function gets called before trap_init
	c0status = read_c0_status() | ST0_CU2;
	write_c0_status(c0status);
	cpumask_clear(&phys_cpu_present_map);
	cpumask_clear(&smp_boot.online_map);
	cpumask_set_cpu(hard_smp_processor_id(), &smp_boot.online_map);
	cpumask_set_cpu(hard_smp_processor_id(), &phys_cpu_present_map);

	board_nmi_handler_setup = nlm_nmi_setup;

	on_chip_init();
	nlm_get_dram_mapping();

	prom_reconfigure_thr_resources();

	/* setup early serial port driver */
	nlm_early_serial_setup(xlp_uart_portid);

	register_smp_ops(&nlm_smp_ops);

	wakeup_secondary_cpus();
}

void __init prom_free_prom_memory(void)
{
	/* nothing to free */
}

#if 0
/* No one calls either of these functions */
#ifndef KSEG0
#define KSEG0 0xffffffff80000000ULL
#endif

#define RING_BUFFER_BASE (511 << 20)
#define RING_BUFFER_SIZE (8 << 10)
static void outbyte_ring_buffer(char c)
{
	unsigned long base = RING_BUFFER_BASE + (hard_smp_processor_id() * RING_BUFFER_SIZE);
	char *buf = (char *)KSEG0 + base;
	static int idx = 0;

	buf[idx] = c;
	idx = (idx + 1) % (RING_BUFFER_SIZE);
}

void nlm_early_printk(const char *fmt, ...)
{
	char buf[256];
	va_list args;
	char *str = buf;

	va_start(args, fmt);
	vsnprintf(buf, 256, fmt, args);
	va_end(args);

	while (*str) {
		outbyte_ring_buffer(*str);
		str++;
	}
}
#endif

#ifdef CONFIG_EARLY_PRINTK
void prom_putchar(char c)
{
	nlm_reg_t *mmio;
	switch(xlp_uart_portid){
		default:
		case 0:
			mmio = netlogic_io_mmio(NETLOGIC_IO_UART_0_OFFSET);
			break;
		case 1:
			mmio = netlogic_io_mmio(NETLOGIC_IO_UART_1_OFFSET);
			break;
	}
	while (netlogic_read_reg( mmio, 0x5) == 0);
		netlogic_write_reg( mmio, 0x0, c);
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

#ifdef CONFIG_BLK_DEV_INITRD
extern unsigned long initrd_start, initrd_end;

static int __init initrd_setup(char *str)
{
	char rdarg[64];
	int idx;
	char *tmp, *endptr;
	unsigned long initrd_size;

	/* Make a copy of the initrd argument so we can smash it up here */
	for (idx = 0; idx < sizeof(rdarg)-1; idx++) {
		if (!str[idx] || (str[idx] == ' ')) break;
		rdarg[idx] = str[idx];
	}

	rdarg[idx] = 0;
	str = rdarg;

	/*
	*Initrd location comes in the form "<hex size of ramdisk in bytes>@<location in memory>"
	*  e.g. initrd=size@physaddr.
	*/
	for (tmp = str; *tmp != '@'; tmp++) {
		if (!*tmp) {
			goto fail;
		}
	}
	*tmp = 0;
	tmp++;
	if (!*tmp) {
		goto fail;
	}
	initrd_size = simple_strtoul(str, &endptr, 16);
	if (*endptr) {
		*(tmp-1) = '@';
		goto fail;
	}
	*(tmp-1) = '@';
	initrd_start = simple_strtoul(tmp, &endptr, 16);

	nlm_io_base = CKSEG1ADDR(XLP_DEFAULT_IO_BASE);
#if defined(CONFIG_32BIT) && defined (CONFIG_MAPPED_KERNEL)
	initrd_start = CKSEG2ADDR(initrd_start);
#else
	initrd_start = CKSEG1ADDR(initrd_start);
#endif
	
	if (*endptr) {
		goto fail;
	}
	initrd_end = initrd_start + initrd_size;
	printk("Found initrd of %lx@%lx\n", initrd_size, initrd_start);
	return 1;
	fail:
	printk(KERN_WARNING "Bad initrd argument.  Disabling initrd\n");
	initrd_start = 0;
	initrd_end = 0;
	return 1;
}
#endif

/*
 * Init routine which accepts the variables from u-boot
 */
void  __init parse_cmdline(void)
{
	char *ptr;
	/* Need to find out early whether we've got an initrd.  So scan
	the list looking now */
	for (ptr = arcs_cmdline; *ptr; ptr++) {
		while (*ptr == ' ') {
			ptr++;
		}
		if (!strncmp(ptr, "initrd=", 7)) {
#ifdef CONFIG_BLK_DEV_INITRD
			initrd_setup(ptr+7);
#else
			printk("initrd is disabled in Kernel\n");
#endif
			break;
		} else {
			while (*ptr && (*ptr != ' ')) {
				ptr++;
			}
		}
	}
}
