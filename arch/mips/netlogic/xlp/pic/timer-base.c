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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/bug.h>
#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/time.h>
#include <asm/system.h>
#include <asm/mipsregs.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/xlp.h>
#include "timer-base.h"


DEFINE_SPINLOCK(pit_lock);
/* Must be called with corresponding locks held
 * Enables/Disables timer in PIC_CTRL */
void __xlp_timer_ctrl(u8 node, u8 timer, u8 en)
{
	u64 val;

	val = nlh_pic_r64r(node, XLP_PIC_CTRL);
	switch (en) {
	case 0:
		if (!(val & (1 << (timer + 10)))) { /* EN : bit 10-17 incl */
		return;
		}
		val &= ~(1 << (timer + 10));
		break;
	default:
		if (val & (1 << (timer + 10))) { /* EN : bit 10-17 incl */
			return;
		}
		val |= (1 << (timer + 10));
		break;
	}
	nlh_pic_w64r(node, XLP_PIC_CTRL, val);
}

int xlp_timer_enable(u8 node, u8 timer)
{
	unsigned long flags;

	spin_lock_irqsave(&pit_lock, flags);
	__xlp_timer_ctrl(node, timer, 1);
	spin_unlock_irqrestore(&pit_lock, flags);
	return 0;
}

EXPORT_SYMBOL(xlp_timer_enable);

void xlp_timer_disable(u8 node, u8 timer)
{
	unsigned long flags;

	spin_lock_irqsave(&pit_lock, flags);
	__xlp_timer_ctrl(node, timer, 0);
	spin_unlock_irqrestore(&pit_lock, flags);
}
EXPORT_SYMBOL(xlp_timer_disable);

u64 xlp_timer_regread(u8 node, u32 reg)
{
	u64 val;

	preempt_disable();
	/* read the count */
	val = nlh_pic_r64r(node, reg);
	preempt_enable();
	return val;
}
EXPORT_SYMBOL(xlp_timer_regread);

int xlp_timer_regwrite(u8 node, u8 reg, u64 val)
{
	unsigned long flags;

	/* We have delta (cycles, I assume, in future) for an event
	 * Program a one shot timer with the CED with that many cycles for
	 * interrupt */
	spin_lock_irqsave(&pit_lock, flags);
	/* Simply writing maxval reg will cause count decrement from that val */
	nlh_pic_w64r(node, reg, val);
	spin_unlock_irqrestore(&pit_lock, flags);
	return 0;
}
EXPORT_SYMBOL(xlp_timer_regwrite);

