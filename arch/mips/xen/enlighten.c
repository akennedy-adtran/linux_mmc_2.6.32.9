/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * Core of Xen paravirt_ops implementation.
 *
 * This file contains the xen_paravirt_ops structure itself, and the
 * implementations for:
 * - privileged instructions
 * - interrupt flags
 * - segment operations
 * - booting and setup
 *
 * Jeremy Fitzhardinge <jeremy@xensource.com>, XenSource Inc, 2007
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/preempt.h>
#include <linux/hardirq.h>
#include <linux/percpu.h>
#include <linux/delay.h>
#include <linux/start_kernel.h>
#include <linux/sched.h>
#include <linux/kprobes.h>
#include <linux/bootmem.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/highmem.h>
#include <linux/console.h>

#include <xen/interface/xen.h>
#include <xen/interface/version.h>
#include <xen/interface/physdev.h>
#include <xen/interface/vcpu.h>
#include <xen/features.h>
#include <xen/page.h>
#include <xen/hvc-console.h>

#include <asm/xen/hypercall.h>
#include <asm/xen/hypervisor.h>
#include <asm/fixmap.h>
#include <asm/processor.h>
#include <asm/traps.h>
#include <asm/setup.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <asm/reboot.h>
#include <asm/bootinfo.h>

#include <linux/gfp.h>

EXPORT_SYMBOL_GPL(hypercall_page);

DEFINE_PER_CPU(struct vcpu_info *, xen_vcpu);
DEFINE_PER_CPU(struct vcpu_info, xen_vcpu_info);

enum xen_domain_type xen_domain_type = XEN_PV_DOMAIN;
EXPORT_SYMBOL_GPL(xen_domain_type);

/* unsigned long *machine_to_phys_mapping = (void *)MACH2PHYS_VIRT_START; */
/* EXPORT_SYMBOL(machine_to_phys_mapping); */
unsigned int   machine_to_phys_order;
EXPORT_SYMBOL(machine_to_phys_order);

struct start_info *xen_start_info;
EXPORT_SYMBOL_GPL(xen_start_info);

struct shared_info xen_dummy_shared_info;

/*
 * Point at some empty memory to start with. We map the real shared_info
 * page as soon as fixmap is up and running.
 */
struct shared_info *HYPERVISOR_shared_info = (void *)&xen_dummy_shared_info;

unsigned long get_phys_to_machine(unsigned long pfn)
{
	return pfn;
}
EXPORT_SYMBOL_GPL(get_phys_to_machine);

#if 0
/*
 * Flag to determine whether vcpu info placement is available on all
 * VCPUs.  We assume it is to start with, and then set it to zero on
 * the first failure.  This is because it can succeed on some VCPUs
 * and not others, since it can involve hypervisor memory allocation,
 * or because the guest failed to guarantee all the appropriate
 * constraints on all VCPUs (ie buffer can't cross a page boundary).
 *
 * Note that any particular CPU may be using a placed vcpu structure,
 * but we can only optimise if the all are.
 *
 * 0: not available, 1: available
 */
static int have_vcpu_info_placement = 1;

static void xen_vcpu_setup(int cpu)
{
	struct vcpu_register_vcpu_info info;
	int err;
	struct vcpu_info *vcpup;

	BUG_ON(HYPERVISOR_shared_info == &xen_dummy_shared_info);
	per_cpu(xen_vcpu, cpu) = &HYPERVISOR_shared_info->vcpu_info[cpu];

	if (!have_vcpu_info_placement)
		return;		/* already tested, not available */

	vcpup = &per_cpu(xen_vcpu_info, cpu);

	info.mfn = arbitrary_virt_to_mfn(vcpup);
	info.offset = offset_in_page(vcpup);

	printk(KERN_DEBUG "trying to map vcpu_info %d at %p, mfn %llx, offset %d\n",
	       cpu, vcpup, info.mfn, info.offset);

	/* Check to see if the hypervisor will put the vcpu_info
	   structure where we want it, which allows direct access via
	   a percpu-variable. */
	err = HYPERVISOR_vcpu_op(VCPUOP_register_vcpu_info, cpu, &info);

	if (err) {
		printk(KERN_DEBUG "register_vcpu_info failed: err=%d\n", err);
		have_vcpu_info_placement = 0;
	} else {
		/* This cpu is using the registered vcpu info, even if
		   later ones fail to. */
		per_cpu(xen_vcpu, cpu) = vcpup;

		printk(KERN_DEBUG "cpu %d using vcpu_info at %p\n",
		       cpu, vcpup);
	}
}
#endif

static char *banner = "Booting paravirtualized kernel on Xen\n";

static void __init xen_banner(void)
{
	unsigned version = HYPERVISOR_xen_version(XENVER_version, NULL);
	struct xen_extraversion extra;

	HYPERVISOR_xen_version(XENVER_extraversion, &extra);

	HYPERVISOR_console_io(CONSOLEIO_write, strlen(banner), banner);

	printk(KERN_INFO "Xen version: %d.%d%s%s\n",
	       version >> 16, version & 0xffff, extra.extraversion,
	       xen_initial_domain() ? " (dom0)" : "");
}

void xen_setup_shared_info(void)
{
	HYPERVISOR_shared_info =
		(struct shared_info *) xen_start_info->shared_info;

#ifndef CONFIG_SMP
	/* In UP this is as good a place as any to set up shared info */
	xen_setup_vcpu_info_placement();
#endif
}

#ifndef CONFIG_SMP
/* This is called once we have the cpu_possible_map */
void xen_setup_vcpu_info_placement(void)
{
	int cpu;

	for_each_possible_cpu(cpu)
		xen_vcpu_setup(cpu);

	/* xen_vcpu_setup managed to place the vcpu_info within the
	   percpu area for all cpus, so make use of it */
	if (have_vcpu_info_placement) {
		printk(KERN_INFO "Xen: using vcpu_info placement\n");
	}
}
#endif

static void xen_reboot(int reason)
{
	struct sched_shutdown r = { .reason = reason };

#ifdef CONFIG_SMP
	smp_send_stop();
#endif

	if (HYPERVISOR_sched_op(SCHEDOP_shutdown, &r))
		BUG();
}

static void xen_restart(char *msg)
{
	xen_reboot(SHUTDOWN_reboot);
}

static void xen_machine_halt(void)
{
	xen_reboot(SHUTDOWN_poweroff);
}

struct setup_header {
        __u8    setup_sects;
        __u16   root_flags;
        __u32   syssize;
        __u16   ram_size;
#define RAMDISK_IMAGE_START_MASK        0x07FF
#define RAMDISK_PROMPT_FLAG             0x8000
#define RAMDISK_LOAD_FLAG               0x4000
        __u16   vid_mode;
        __u16   root_dev;
        __u16   boot_flag;
        __u16   jump;
        __u32   header;
        __u16   version;
        __u32   realmode_swtch;
        __u16   start_sys;
        __u16   kernel_version;
        __u8    type_of_loader;
        __u8    loadflags;
#define LOADED_HIGH     (1<<0)
#define QUIET_FLAG      (1<<5)
#define KEEP_SEGMENTS   (1<<6)
#define CAN_USE_HEAP    (1<<7)
        __u16   setup_move_size;
        __u32   code32_start;
        __u32   ramdisk_image;
        __u32   ramdisk_size;
        __u32   bootsect_kludge;
        __u16   heap_end_ptr;
        __u8    ext_loader_ver;
        __u8    ext_loader_type;
        __u32   cmd_line_ptr;
        __u32   initrd_addr_max;
        __u32   kernel_alignment;
        __u8    relocatable_kernel;
        __u8    _pad2[3];
        __u32   cmdline_size;
        __u32   hardware_subarch;
        __u64   hardware_subarch_data;
        __u32   payload_offset;
        __u32   payload_length;
        __u64   setup_data;
} __attribute__((packed));

static struct setup_header boot_params_hdr;

static int __init xen_start_kernel(void)
{
	xen_domain_type = XEN_PV_DOMAIN;

	/* Don't do the full vcpu_info placement stuff until we have a
	   possible map and a non-dummy shared_info. */
	per_cpu(xen_vcpu, 0) = &HYPERVISOR_shared_info->vcpu_info[0];

	/* Poke various useful things into boot_params */
	boot_params_hdr.type_of_loader = (9 << 4) | 0;
	boot_params_hdr.ramdisk_image = xen_start_info->mod_start
		? __pa(xen_start_info->mod_start) : 0;
	boot_params_hdr.ramdisk_size = xen_start_info->mod_len;
	boot_params_hdr.cmd_line_ptr = __pa(xen_start_info->cmd_line);

	xen_start_info->console.domU.mfn = 0;
	xen_start_info->console.domU.evtchn = 0;

	_machine_halt = xen_machine_halt;
	_machine_restart = xen_restart;

	xen_banner();

	HYPERVISOR_set_callbacks(CALLBACKTYPE_dom0_getfreepages, (unsigned long)__get_free_pages);

	return 0;
}

subsys_initcall(xen_start_kernel);
