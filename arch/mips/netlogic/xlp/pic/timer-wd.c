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

#define MY_TIMER	XLP_SYSTIMER2"0"
static u8 timer;
/* Ideally this should be a platform driver;
 * For the time being, just make a dummy driver */
static irqreturn_t xlp_systimer_handler(int irq, void *p)
{
	u8 node = XLP_IRQ_TO_NODE(irq);
	unsigned int xlp_pit_tick_rate = nlm_hal_get_xlp_pit_tick_rate();

	xlp_timer_mvwrite(node, timer, xlp_pit_tick_rate); /* One Hz */
	printk(KERN_DEBUG "in timer handler\n");
	return IRQ_HANDLED;
}

int timer_wd_init(void)
{
	int ret;
	u8 nid;
	unsigned int xlp_pit_tick_rate = nlm_hal_get_xlp_pit_tick_rate();

	for_each_online_node(nid) {
		timer = xlp_str2timer(MY_TIMER);
		xlp_timer_mvwrite(nid, timer, xlp_pit_tick_rate); /* One Hz */
		xlp_timer_enable(nid, timer);
		ret = request_irq(XLP_TIMER_IRQ(nid, timer), xlp_systimer_handler, 0, MY_TIMER, NULL);
		if (ret < 0) {
			return -EINVAL;
		}
	}
	return 0;
}

void timer_wd_exit(void)
{
	u8 nid;

	for_each_online_node(nid) {
		xlp_timer_disable(nid, timer);
		free_irq(XLP_TIMER_IRQ(nid, timer), NULL);
	}
}

module_init(timer_wd_init);
module_exit(timer_wd_exit);
MODULE_LICENSE("GPL");

