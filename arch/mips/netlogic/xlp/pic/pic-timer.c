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
 * Implements the constant frequency timer on PIC
 * Rationale :
 * For dynamic frequency changes, we need a constant frequency timer
 * because, by default, MIPS goes with high resolution timer using
 * count compare based on the frequency of the CPU. (For information on
 * how this is implemented, check clock event timer implementation given
 * in  arch/mips/kernel/cevt-r4k.c.
 * As a solution, if dynamic cpu frequency change is configured, cevt
 * is disabled and PIC timer running at constant frequency is used
 * instead. Although there are 7 sys timers (systimer 0-6) available,
 * we use only two of them
 */

/* Caution : this file cannot be compiled as a module */
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

/* Clocksource functions start START_CS */
static cycle_t xlp_cs_read(struct clocksource *cs)
{
	u8 nid = xlp_str2node(cs->name);
	return (cycles_t) (XLP_PIT_TIMER_MAX - xlp_timer_ctread(nid, xlp_str2timer(cs->name)));
}

static int xlp_cs_enable(struct clocksource *cs)
{
	u8 nid = xlp_str2node(cs->name);
	return xlp_timer_enable(nid, xlp_str2timer(cs->name));
}

static void xlp_cs_disable(struct clocksource *cs)
{
	u8 nid = xlp_str2node(cs->name);
	xlp_timer_disable(nid, xlp_str2timer(cs->name));
}

/* There are two distinct features of timers from 2.6.17 onwards.
 * clock source is the monotonic continuous timer which has nothing to with
 * interrupts, TOD ..etc. We use systimer 6 for this.
 * @133 MHz, one cycle is 7.5 ns => mult = 30 and shift = 2.
 * @125 MHz, one cycle is 8.0 ns => mult = 32 and shift = 2.
 */
static struct clocksource xlp_pit_cs[NLM_MAX_CPU_NODE] = {
	[0 ... NLM_MAX_CPU_NODE-1] = {
	.name		= XLP_CSDTIMER,
	.rating		= 300,
	.read		= xlp_cs_read,
	.mask		= CLOCKSOURCE_MASK(64),		/* clock is 64 bit */
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.mult		= 30,
	.shift		= 2,	/* for 100 cycles, (100 * 30) >> 2 == 750(ns) */
	.enable		= xlp_cs_enable,
	.disable	= xlp_cs_disable,
	}
};

#if defined ARCH_HAS_READ_CURRENT_TIMER
int read_current_timer(unsigned long *tv)
{
	*tv = (unsigned long)(xlp_cs_read(&xlp_pit_cs[0]));
	return 0;
}
#endif

/* Run time initialization of the clock source structure */
static void xlp_pit_cs_init(u8 node)
{
	u8 timer;
	u8 ref_clk_mhz;
	char *ptr = kmalloc(XLP_SYSTIMER_OFFSET + 3, GFP_KERNEL);

	if(ptr == NULL) {
		panic("No memory to allocate clocksource\n");
	}
	memcpy(ptr, XLP_CSDTIMER, strlen(XLP_CSDTIMER));
	ptr[XLP_SYSTIMER_OFFSET + 1] = '0' + node;
	ptr[XLP_SYSTIMER_OFFSET + 2] = 0;
	xlp_pit_cs[node].name = ptr;
	timer = xlp_str2timer(xlp_pit_cs[node].name);
	xlp_timer_mvwrite(node, timer, XLP_PIT_TIMER_MAX);
	xlp_timer_enable(node, xlp_str2timer(xlp_pit_cs[node].name));
	ref_clk_mhz = nlm_hal_get_ref_clk_freq() / 1000 / 1000;
	if (ref_clk_mhz <= 	67)
		xlp_pit_cs[node].mult = 60;
	else if (ref_clk_mhz <= 100)
		xlp_pit_cs[node].mult = 40;
	else if (ref_clk_mhz <= 125)
		xlp_pit_cs[node].mult = 32;
	/* Default struct init value for .mult of 30 is correct for 133 MHz. */
	if (clocksource_register(&xlp_pit_cs[node])) {
		panic("Cannot register clocksource on node %d\n", node);
	}
}

/* END of clocksource functions END_CS*/

/* Clock event device functions start START_CED
 * Clock event device is the interrupt source. We have to configure some
 * features on this, mainly oneshot timer and periodic timer. Periodic feature
 * is used during boot up and later moved to oneshot. The function .set_mode
 * is called with these paramters and we do set the PIT systimer accordingly
 * in that function.
 */

int xlp_pit_ced_next_event(unsigned long evt, struct clock_event_device *ced)
{
	u8 timer = xlp_str2timer(ced->name);
	u8 nid = xlp_str2node(ced->name);

	/* We have delta (cycles, I assume, in future) for an event
	 * Program a one shot timer with the CED with that many cycles for
	 * interrupt */
	xlp_timer_mvwrite(nid, timer, (u64)evt);
	return 0;
}

void xlp_pit_ced_mode(enum clock_event_mode mode, struct clock_event_device *ced)
{
	u8 timer = xlp_str2timer(ced->name);
	u8 node = xlp_str2node(ced->name);
	u64 xlp_pit_tick_rate = nlm_hal_get_xlp_pit_tick_rate();

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk(KERN_DEBUG "[%d] Timer[%d]: Periodic\n", node, timer);
		__xlp_timer_ctrl(node, timer, 1); /* Enable timer */
		xlp_timer_mvwrite(node, timer, (u64)(xlp_pit_tick_rate/HZ));
		__xlp_timer_ctrl(node, timer, 1); /* Enable timer */
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		__xlp_timer_ctrl(node, timer, 0); /* Disable timer */
		break;
	case CLOCK_EVT_MODE_UNUSED:
		if (ced->mode == CLOCK_EVT_MODE_PERIODIC ||
		    ced->mode == CLOCK_EVT_MODE_ONESHOT)
			__xlp_timer_ctrl(node, timer, 0); /* Disable timer */
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		printk(KERN_DEBUG "Set timer %d to Oneshot\n", timer);
		__xlp_timer_ctrl(node, timer, 1); /* Enable timer */
		xlp_timer_mvwrite(node, timer, (u64)(xlp_pit_tick_rate/HZ));
		break;

	case CLOCK_EVT_MODE_RESUME:
		/* Nothing to do here */
		break;
	}
}

static struct clock_event_device xlp_pit_ced[NLM_MAX_CPU_NODE] = {
	[0 ... NLM_MAX_CPU_NODE-1] = {
	.features       = CLOCK_EVT_FEAT_PERIODIC,
	.max_delta_ns	= 0,	/* Change in init func */
	.min_delta_ns	= 0,	/* Change in init func */
	.shift		= 2,
	.mult		= 30,
	.rating		= 300,
	.set_next_event	= xlp_pit_ced_next_event,
	.set_mode	= xlp_pit_ced_mode,
	}
};

void constrict_mask_to_node(u8, struct cpumask *, const struct cpumask *);
struct cpumask xlp_ced_mask[NLM_MAX_CPU_NODE];
int __init xlp_pit_ced_init(u8 nid)
{
	char *ptr = kmalloc(XLP_SYSTIMER_OFFSET + 3, GFP_KERNEL);

	if(ptr == NULL) {
		panic("No memory to allocate CED on node %d\n", nid);
	}
	memcpy(ptr, XLP_CEDTIMER, strlen(XLP_CEDTIMER));
	ptr[XLP_SYSTIMER_OFFSET + 1] = '0' + nid;
	ptr[XLP_SYSTIMER_OFFSET + 2] = 0;
	xlp_pit_ced[nid].name = ptr;

	xlp_pit_ced[nid].cpumask = cpumask_of(nid * NLM_MAX_CPU_PER_NODE);
	xlp_pit_ced[nid].max_delta_ns = clockevent_delta2ns((u64)~0,
			&xlp_pit_ced[nid]);
	xlp_pit_ced[nid].min_delta_ns = clockevent_delta2ns(1ULL,
			&xlp_pit_ced[nid]);
	xlp_pit_ced[nid].irq = XLP_TIMER_IRQ(nid, xlp_str2timer(XLP_CEDTIMER));
	clockevents_register_device(&xlp_pit_ced[nid]);
	return 0;
}

static irqreturn_t xlp_timer_handler(int irq, void *dev_id)
{
	u8 nid;

	preempt_disable();
	nid = hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE;
	preempt_enable();

	xlp_ack_pic(nid, xlp_irq_to_irt(irq));
	/* We don't have to use dev_id; this avoids some type casting */
	if (xlp_pit_ced[nid].event_handler) {
		xlp_pit_ced[nid].event_handler(&xlp_pit_ced[nid]);
	}
	//do_IRQ(irq);
	return IRQ_HANDLED;
}


/*
 * Restricts timer mask to cpu
 *
 * @cpu : cpu number
 * @timer : timer
 */
void program_timer_mask(u8 node, u8 timer, struct cpumask *m)
{
	struct irq_desc *desc;
	unsigned long flags;

	/* The following is a nasty hack, but there is no other way to do it
	 * as far as I know. We make sure that desc has affinity (and status
	 * flag already set before calling setup_irq */
	desc = irq_to_desc(XLP_TIMER_IRQ(node, xlp_str2timer(XLP_CEDTIMER)));
	spin_lock_irqsave(&desc->lock, flags);
	desc->status = IRQ_AFFINITY_SET;
	cpumask_copy(desc->affinity, m);
	spin_unlock_irqrestore(&desc->lock, flags);
}

/* Called from netlogic/xlp/smp.c:nlm_cpus_done()
 *
 * This function reassigns cpumask for timer interrupts in the system
 */
extern int irq_select_affinity_usr(unsigned int);
void xlp_reprogram_timer_masks(void)
{
	struct cpumask m, m2;
	u8 nid;
	u8 timer = xlp_str2timer(XLP_CEDTIMER);

	for_each_online_node(nid) {
		cpumask_setall(&m);
		constrict_mask_to_node(nid, &m2, &m);
		program_timer_mask(nid, timer, &m2);
		irq_select_affinity_usr(XLP_TIMER_IRQ(nid, xlp_str2timer(XLP_CEDTIMER)));
	}
}

static struct irqaction timer_irqa = {
	.handler = xlp_timer_handler,
	.flags = IRQF_DISABLED | IRQF_TIMER,
	.name = "xlp_pit_timer",
	.dev_id = (void *)xlp_pit_ced,
};

void xlp_disable_count_compare(u32 cpu)
{
	u32 cause;
	u64 val;

	printk(KERN_DEBUG "Disabling count_compare register on cpu %d\n", cpu);
	/* Set bit 27 so that count is disabled for this cpu
	 * Then clear rvec 7, and write non zero val to the compare reg
	 * so that count and compare do not match */

	val = read_64bit_cp0_eirr();
	val &= ~(1 << XLP_IRQ_TIMER_RVEC);
	write_64bit_cp0_eirr(val);

	cause = read_c0_cause();
	cause |= (1 << 27);	/* MIPS_CAUSE_DISABLE_COUNT_COMPARE */
	write_c0_cause(cause);

	/* Now write a non zero value to compare register */
	write_c0_compare(0xffff);
}

void busy_delay(u32 milli)
{
	volatile idx = 0;
	u64 max = milli << 20;
	while (idx < max) idx++;
}
/*
 * This function is called from include/asm/time.h:mips_clockevent_init()
 * This is executed in every cpu during bootup
 */
int __init xlp_pic_ced_timer_init(int ced)
{
	u8 node;
	u32 cpu;
	int ret = 0;
	struct cpumask m;
	u8 timer = xlp_str2timer(XLP_CEDTIMER);

	cpu = hard_smp_processor_id();
	node = (u8)(cpu / NLM_MAX_CPU_PER_NODE);
	xlp_disable_count_compare(cpu);
	/* This code is valid only for CPU0 of the node (0, 32, 64 and 96) */
	if ((cpu % NLM_MAX_CPU_PER_NODE) != 0) {
		return 0;
	}
	ret = xlp_pit_ced_init(node);
	if (ret != 0) {
		panic("Clock Event Device initialization failure\n");
	}
	cpumask_clear(&m);
	busy_delay(10);
	cpumask_set_cpu(cpu, &m);
	program_timer_mask(node, timer, &m);
	if (setup_irq(XLP_TIMER_IRQ(node, timer), &timer_irqa) < 0) {
		printk(KERN_WARNING "Node (%d) timer irq(%d) setup failed\n",
				node, XLP_TIMER_IRQ(node, timer));
	}
	return 0;
}
/*
 * This function is called from include/asm/time.h:init_mips_clocksource()
 * This is executed in every cpu during bootup
 */
void __init xlp_pic_cs_timer_init(int cs)
{
	u8 node;

	node = (u8)(hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE);
	xlp_pit_cs_init(node);
	return;
}
