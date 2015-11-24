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
#include <linux/module.h>
#include <linux/cpumask.h>
#include <linux/nodemask.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/xlp.h>
#include <asm/smp.h>

static u64 xlp_pic_base[NLM_MAX_CPU_NODE] = {XLP_BDF_BASE(0,0,4),
	XLP_BDF_BASE(0,8,4), XLP_BDF_BASE(0,16,4), XLP_BDF_BASE(0,24,4)};
u64 __nlh_pic_r64o(u8 nid, u64 offset)
{
	return (nlh_read_cfg_reg64(xlp_pic_base[nid] + offset));
}
EXPORT_SYMBOL(__nlh_pic_r64o);

void __nlh_pic_w64o(u8 nid, u64 offset, u64 val)
{
	nlh_write_cfg_reg64(xlp_pic_base[nid] + offset, val);
}
EXPORT_SYMBOL(__nlh_pic_w64o);

/*
 * __nlm_hal_request_irq
 * This function will return the irt index for any given irq.
 * Note :
 *	We don't care if the irt is enabled or not, if there are
 *	multiple assignments of the same irq or not.
 *	must be called with irt_irq lock taken
 *
 * @irq : irq whose irt entry should be returned
 *
 * return : irt entry index
 */
int __nlm_hal_request_irq(u8 node, int irt, int rvec)
{
        u64 val;

	val = nlh_pic_r64r(node, XLP_PIC_IRT_ENTRY(irt));
	val &= ~(0x3fULL << 20);
	val |= ((((u64)rvec) << 20) | (1ULL << 31));
	nlh_pic_w64r(node, XLP_PIC_IRT_ENTRY(irt), val);
        return 0;
}

/*
 * __nlm_hal_release_irq
 * Note :
 *	must be called with irt_irq lock taken
 *
 * @irq : irq whose irt entry should be returned
 *
 * return : irt entry index
 */
void __nlm_hal_release_irq(u8 node, int irt)
{
        u64 val;

	val = nlh_pic_r64r(node, XLP_PIC_IRT_ENTRY(irt));
	val &= ~(1ULL << 31);
	nlh_pic_w64r(node, XLP_PIC_IRT_ENTRY(irt), val);
        return;
}

void xlp_ack_pic(u8 node, int irt)
{
	/* Order of ack-ing pic_status and pic_int_ack is very important
	 * Otherwise you might end up double the interrupt rate */
	if (irt < 12) { /* Ack status register for WD and Sys.Timers */
		nlh_pic_w64r(node, XLP_PIC_STATUS, 1 << irt);
	}
	/* Currently we have no way of figuring out the source PIC for an
	 * interrupt. So, we restrict interrupt delivery to local node only
	 * Please make sure this is the case while interrupt thread enable
	 * registers (0x94 onwards) */
	nlh_pic_w64r(node, XLP_PIC_INT_ACK, irt);
	return;
}
#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/kernel.h>
#include <linux/module.h>
EXPORT_SYMBOL(__nlm_hal_request_irq);
EXPORT_SYMBOL(__nlm_hal_release_irq);
#endif

void nlm_hal_pic_send_ipi(int nmi, int vec, int node, int cpu)
{
	unsigned long long ipi = (nmi << 31) | (vec << 20) | (node << 17) | (1 << (cpu & 0xf));
	if (cpu > 15) {
		ipi |= 0x10000; // Setting bit 16 to select cpus 16-31
	}
	nlh_pic_w64r(0, XLP_PIC_IPI_CTL, ipi);
}


#if 0
void xlp_ack_pic(int irt)
{
	int i;

	for_each_online_node(i) {
		nlh_pic_w64r(i, 0x50, irt);
	}
	return;
}
#endif

#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
void nlm_hal_pic_update_control(u64 control)
{
	u64 val = nlh_pic_r64r(0, XLP_PIC_CTRL);
	val |= control;
	nlh_pic_w64r(0, XLP_PIC_CTRL, val);
}
#endif

#if 0
static void nlm_hal_pic_write_irt(int irt_num, int en, int nmi, int sch, int vec, int dt, int db, int dte)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	unsigned long long val = (((en & 0x1) << 31) | ((nmi & 0x1) << 29) | ((sch & 0x1) << 28) |
				  ((vec & 0x3f) << 20) | ((dt & 0x1 ) << 19) | ((db & 0x7) << 16) |
				  (dte & 0xffff));

	nlm_hal_write_pic_reg(mmio, PIC_IRT(irt_num), val);
}

static __inline__ unsigned long long nlm_hal_pic_read_control(void)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	return nlm_hal_read_pic_reg(mmio, PIC_CTRL);
}

static __inline__ void nlm_hal_pic_write_control(unsigned long long control)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	nlm_hal_write_pic_reg(mmio, PIC_CTRL, control);
}


static __inline__ void nlm_hal_ack_pic(int irt_num)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	nlm_hal_write_pic_reg(mmio, PIC_INT_ACK, irt_num);

	/* Ack the Status register for Watchdog & System timers */
	if (irt_num < 12) {
		nlm_hal_write_pic_reg(mmio, PIC_STATUS, (1 << irt_num));
	}
}

static __inline__ unsigned long long nlm_hal_pic_read_irt(int irt_num)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	return nlm_hal_read_pic_reg(mmio, PIC_IRT(irt_num));
}

static __inline__ void nlm_hal_pic_write_irt_direct(int irt_num, int en, int nmi, int sch, int vec, int cpu)
{
	nlm_hal_pic_write_irt(irt_num, en, nmi, sch, vec, 1, CPUIDBIT2(cpu), CPUIDBITS01(cpu));
	/* Does not support multi node support yet */
}

static __inline__ unsigned long long nlm_hal_pic_read_timer(int timer)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	return nlm_hal_read_pic_reg(mmio, PIC_SYS_TIMER_COUNTER(timer));
}

static __inline__ void nlm_hal_pic_write_timer(int timer, pic_reg_t value)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	nlm_hal_write_pic_reg(mmio, PIC_SYS_TIMER_COUNTER(timer), value);
}
#endif

