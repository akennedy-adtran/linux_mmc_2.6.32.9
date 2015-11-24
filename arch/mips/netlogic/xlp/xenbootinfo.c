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

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/pfn.h>

#include <asm/page.h>
#include <asm/bootinfo.h>
#include <asm/netlogic/bootinfo.h>
#include <xen/interface/xen.h>
#include <xen/interface/domctl.h>
#include <asm/xen/hypervisor.h>
#include <asm/xen/hypercall.h>

#include <asm/netlogic/debug.h>
#include <asm/netlogic/xlp.h>

extern char _end[];
extern unsigned int onlinemask;

extern void xen_setup_shared_info(void);

void xen_init(void)
{
	xen_start_info = (struct start_info *)__va((unsigned long) PFN_ALIGN(__pa_symbol(&_end)));

	xen_setup_shared_info();
}

extern void prom_pre_boot_secondary_cpus(void *);

#ifdef CONFIG_MAPPED_KERNEL
#define secondary_cpus_bootup_func \
       ((unsigned long)prom_pre_boot_secondary_cpus - \
        (unsigned long)LOADADDR + (unsigned long)PHYSADDR)
#else
#define secondary_cpus_bootup_func prom_pre_boot_secondary_cpus
#endif

/*
 * adopted from the compilation of bit-twiddling hacks:
 * http://graphics.stanford.edu/~seander/bithacks.html
 * :-)
 */
static int count_ones(int v)
{
	v = v - ((v >> 1) & 0x55555555);
	v = (v & 0x33333333) + ((v >> 2) & 0x33333333);

	return (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
}

static int xc_wakeup_secondary(int dom_id, unsigned long cpumask, unsigned long entry)
{
	struct xen_domctl domctl;
	struct vcpu_guest_context ctxt;
	int vcpu, ret, nr_vcpu;

	domctl.interface_version = XEN_DOMCTL_INTERFACE_VERSION;
	domctl.cmd = XEN_DOMCTL_bootvcpu;
	domctl.domain = dom_id;

	ctxt.reg31 = entry;
	ctxt.cp0_status = 0;
	set_xen_guest_handle(domctl.u.vcpucontext.ctxt, &ctxt);

	nr_vcpu = count_ones(cpumask);

	for (vcpu = 1; vcpu < nr_vcpu; ++vcpu) {
		domctl.u.vcpucontext.vcpu = vcpu;
		ret = privcmd_call(__HYPERVISOR_domctl2, (unsigned long)&domctl, 0, 0, 0, 0);
		if (ret != 0)
			panic("Unable to launch vcpu\n");
	}

	return 0;
}

int xen_enable_cpus(int node, uint32_t onlinemask)
{
	if (node) {
		printk("XEN Multi-Node not available yet!\n");
		return 0;
	}

	return xc_wakeup_secondary(0, onlinemask, (unsigned long)secondary_cpus_bootup_func);
}
