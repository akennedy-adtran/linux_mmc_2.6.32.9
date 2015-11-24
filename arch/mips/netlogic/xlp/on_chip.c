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

/* Removed most FMN handler functionality - only support drivers that register a handler (either
 * NAPI handler or normal interrupt-driven handler).  Currently this is only the NAE driver.
 * Moved all NAPI related code to NAPI driver (NAE driver).  Note these changes have definitely
 * broken the nlm_adma and SRIO drivers.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/netlogic/hal/nlm_hal_fmn.h>
#include <asm/netlogic/xlp_irq.h>

//#define FMN_DEBUG

#ifdef FMN_DEBUG
#define FMN_DEBUG_PRINTK		printk
#else
#define FMN_DEBUG_PRINTK(...)
#endif

unsigned long netlogic_io_base = (unsigned long)(DEFAULT_NETLOGIC_IO_BASE);
EXPORT_SYMBOL(netlogic_io_base);

extern void nlm_cpu_stat_update_msgring_int(void);
extern void nlm_cpu_stat_update_msgring_cycles(__u32 cycles);
extern void nlm_cpu_stat_update_msgring_pic_int(void);

uint32_t nlm_cpu_vc_mask[NLM_MAX_CPU_NODE*NLM_MAX_CPU_PER_NODE] = {0};
EXPORT_SYMBOL(nlm_cpu_vc_mask);

uint32_t nlm_l1_lock[NR_CPUS/4] = {0};

typedef void (*vc_intr_handler_t)(uint32_t vc);
static vc_intr_handler_t vc_intr_handlers[NLM_MAX_VC_PER_THREAD] = {NULL};
static uint32_t vc_intr_mask = 0;
static uint32_t xlp3xx_ack_mask = 0xf << 16;
#define xlp2xx_ack_mask		0xf

/* make this a read/write spinlock */
static spinlock_t fmn_lock;

static nlm_common_atomic_t msgring_registered;


/* Function is called when a CPU VC interrupt handler is ready to receive interrupts.  Must be
 * called before enabling interrupts to receive messages.  Pass the VC to register and a pointer
 * to the handler callback.  The VC will be passed to the callback when it is called to service
 * an interrupt.  Callback function will be called in the interrupt context.
 */
int xlp_register_vc_intr_handler(uint32_t vc, int napi, vc_intr_handler_t handler)
{
	if(vc < 0 || vc >= NLM_MAX_VC_PER_THREAD) {
		printk(KERN_WARNING "[%s] Invalid VC %d\n", __FUNCTION__, vc);
		return -1;
	}

	if(napi && (xlp3xx_ack_mask != (0xf << 16))) {
		printk(KERN_NOTICE "[%s] Already registered a NAPI VC\n", __func__);
		return -1;
	} else if(vc_intr_mask & (1 << vc)) {
		printk(KERN_NOTICE "[%s] VC %d already registered\n", __func__, vc);
		return -1;
	}

	spin_lock(&fmn_lock);
	if(napi)
		xlp3xx_ack_mask = (~(1 << vc) & 0xf) << 16;
	else
		vc_intr_mask |= (1 << vc);
	vc_intr_handlers[vc] = handler;
	spin_unlock(&fmn_lock);

	FMN_DEBUG_PRINTK("[%s] xlp3xx_ack_mask = 0x%x, intr_vc_mask = 0x%x\n",
			__func__, xlp3xx_ack_mask, vc_intr_mask);
	return 0;
}
EXPORT_SYMBOL(xlp_register_vc_intr_handler);

void xlp_unregister_vc_intr_handler(vc_intr_handler_t handler)
{
	uint32_t vc;

	spin_lock(&fmn_lock);
	for (vc = 0; vc < NLM_MAX_VC_PER_THREAD; vc++) {
		if (vc_intr_handlers[vc] == handler) {
			xlp3xx_ack_mask |= 1 << (vc + 16);
			vc_intr_mask &= ~(1 << vc);
			vc_intr_handlers[vc] = NULL;
		}
	}
	spin_unlock(&fmn_lock);

	FMN_DEBUG_PRINTK("[%s] xlp3xx_ack_mask = 0x%x, intr_vc_mask = 0x%x\n",
			__func__, xlp3xx_ack_mask, vc_intr_mask);
}
EXPORT_SYMBOL(xlp_unregister_vc_intr_handler);

/*********************************************************************
 * xlp_msgring_int_handler 
 *
 *  @irq	msgring irq number
 *  @regs	linux systems call back function provide struct pt_regs 
 *  
 ********************************************************************/
void xlp_msgring_int_handler(unsigned int irq, struct pt_regs *regs)
{
//	uint32_t cycles;
	uint32_t vc, msg_int, vc_mask;
	int cpu = hard_smp_processor_id();

#if 0	// Uncomment to see these in /proc/netlogic/xlp_cpu stats
	if (irq == XLP_IRQ_MSGRING_RVEC) {
		/* normal message ring interrupt */
		nlm_cpu_stat_update_msgring_int();
	} else {
		nlm_cpu_stat_update_msgring_pic_int();
	}
#endif

	irq_enter();

//	cycles = read_c0_count();

	/* Get bitmap of VCs that have triggered an interrupt from the CP2 Message Interrupt
	 * register bits 3:0 (XLP2xx) or Message Status 1 register (XLP3xx/4xx/8xx).  XLP2xx
	 * PDM description of this register appears to be incorrect.  Correct behavior appears
	 * to be 1 in a bit position means an interrupt is pending for that CPU VC and 0
	 * means no interrupt is pending.  PDM description for XLP3xx appears correct.
	 *
	 * For XLP2xx, it appears to be necessary to clear all pending interrupts (by writing 1s)
	 * to the msg_int register to enable triggering the next interrupt.  For XLP3xx this does
	 * not appear to be required.  Thus for XLP2xx NAPI RX, the interrupt handler disables
	 * the interrupt (by disabling the interrupt enable in MSG_CONFIG).  For XLP3xx NAPI RX,
	 * NAPI interrupts are not acknowledged until all RX packets have been received.  Note
	 * for XLP3xx msg_status1, must do read-modify-write.  Not necessary for XLP2xx msg_int.
	 */

	if (is_nlm_xlp2xx()) {
		msg_int = xlp_read_msg_int();
		FMN_DEBUG_PRINTK("[%s] CPU %d msg_int = 0x%x\n", __func__, cpu, msg_int);
		vc_mask = msg_int & nlm_cpu_vc_mask[cpu];
	} else {
		msg_int = xlp_read_status1();
		FMN_DEBUG_PRINTK("[%s] CPU %d msg_status1 = 0x%x\n", __func__, cpu, msg_int);
		vc_mask = (~msg_int >> 16) & nlm_cpu_vc_mask[cpu];
	}

	for(vc = 0; vc < NLM_MAX_VC_PER_THREAD; vc++) {
		if((vc_mask & (1 << vc)) && vc_intr_handlers[vc]) {
			FMN_DEBUG_PRINTK("  Calling interrupt handler for VC %d\n", vc);
			vc_intr_handlers[vc](vc);
		}
	}

	/* Ack interrupts - NAPI VC interrupt should be disabled at this time */
	if (is_nlm_xlp2xx()) {
		xlp_write_msg_int(xlp2xx_ack_mask);
	} else {
		uint32_t val = xlp_read_status1();
		xlp_write_status1(val | xlp3xx_ack_mask);
	}

//	nlm_cpu_stat_update_msgring_cycles(read_c0_count() - cycles);

	irq_exit();
}
EXPORT_SYMBOL(xlp_msgring_int_handler);

/* Removed back-up timer - only needed for XLP8xx/XLP4xx A0 - no more in field.
 * Removed xlp_poll_vc0_messages - only used by DMA driver (which is now broken,
 * but isn't used anyway).
 * Removed un/register_xlp_msgring_handler - also only used by DMA driver, srio
 * driver, nlm_crtpto and nlm_cde drivers.  Need to re-write those to new
 * functionality if those hardware accelerators are to be used.
 * Removed nlm_xlp_register_napi_final_handler - functionality now in NAPI-
 * compliant device drivers (e.g. NAE).
 * Removed nlm_nmi_cpus (not used)
 */

/*********************************************************************
 * enable_msgconfig_int 
 *
 ********************************************************************/
void enable_msgconfig_int(void)
{
	unsigned long flags __attribute__((unused));

	/* Need write interrupt vector to cp2 msgconfig register */
	nlm_hal_set_fmn_interrupt(XLP_IRQ_MSGRING_RVEC);
}
/* Removed nlm_usb_init - no callers */
/* Removed xlp_napi_fmn_poll and xlp_napi_setup - moved to NAE driver */

/*********************************************************************
 * on_chip_init
 *  
 ********************************************************************/
extern void xlp_pic_init(u8);
void __init on_chip_init(void)
{
	int node;

	cpu_logical_map(0) = hard_smp_processor_id();
	node = hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE;
	/* Set netlogic_io_base to the run time value */
	spin_lock_init(&fmn_lock);
	msgring_registered.value = 0;
	nlm_hal_init();
	xlp_pic_init(node);
	/* On XLP, MSGRING config register is per hw-thread */
	enable_msgconfig_int();
}
