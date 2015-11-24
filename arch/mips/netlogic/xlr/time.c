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


#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include <asm/irq.h>
#include <asm/ptrace.h>
#include <asm/addrspace.h>
#include <asm/time.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/perfctr.h>
#include <linux/oprofile.h>

#include <linux/proc_fs.h>

extern spinlock_t phnx_pic_lock;

#if defined(CONFIG_PERFCTR) && defined(CONFIG_OPROFILE)
#error "Cannot enable both VPERF and OProfile at the same time"
#endif

#ifndef CONFIG_NLMCOMMON_MAC
void nlm_common_user_mac_update_time(void)
{
}
void nlm_common_user_mac_update_ktime(void)
{
}
#else
extern void nlm_common_user_mac_update_time(void);
extern void nlm_common_user_mac_update_ktime(void);
#endif
 
extern struct irq_chip phnx_rsvd_pic;
extern struct irqaction phnx_rsvd_action;

void save_epc(unsigned long *epc)
{
	__asm__ __volatile__(".set push\n"
			     ".set noreorder\n"
			     "mfc0 %0, $14\n" ".set pop\n":"=r"(*epc));
}

#ifdef CONFIG_OPROFILE
extern void xlr_oprofile_int_handler(int irq, void *dev_id,
					 struct pt_regs *regs);
#endif
void xlr_timer_interrupt(struct pt_regs *regs, int irq)
{
	int cpu = smp_processor_id();

#ifdef CONFIG_NLM_WATCHDOG
        nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);

	/* ack the watchdog */
	netlogic_write_reg(mmio, 0x0c, 1 << cpu_logical_map(cpu));
#endif

#if defined (CONFIG_OPROFILE) || defined (CONFIG_PERFCTR_INTERRUPT_SUPPORT)
    int cntr0, cntr1;
    uint32_t ctrl0, ctrl1;
	int    perfctr_overflow = 0;
#endif

	if (irq != IRQ_TIMER) {
		printk("[%s]:cpu_%d: bad timer irq = %x\n", __FUNCTION__, cpu, irq);
		BUG();
	}

#if defined (CONFIG_PERFCTR_INTERRUPT_SUPPORT) || defined (CONFIG_OPROFILE)
    ctrl0 = __read_32bit_c0_register($25, 0);
    ctrl1 = __read_32bit_c0_register($25, 2);
    cntr0 = __read_32bit_c0_register($25, 1);
    cntr1 = __read_32bit_c0_register($25, 3);

    /* if interrupts are enabled for perf events, check if any counter has
       overflowed. Then we know for sure that this is a perf event
       */
    if((ctrl0 & 0x10) || (ctrl1 & 0x10))
            if((cntr0 < 0) || (cntr1 < 0))
                perfctr_overflow = 1;
    if(perfctr_overflow == 0)
#endif
    {
        do_IRQ(irq);

        if (cpu == 0) {
            nlm_common_user_mac_update_time();
	    nlm_common_user_mac_update_ktime();
        }
    }

#if defined (CONFIG_PERFCTR_INTERRUPT_SUPPORT) || defined (CONFIG_OPROFILE)
	if (perfctr_overflow) {
#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT
		(*perfctr_ihandler) (instruction_pointer(regs));
#endif
    }
#ifdef CONFIG_OPROFILE
	if (perfctr_overflow) {
		if(netlogic_thr_id() == 0) {
			nlm_common_oprofile_int_handler (irq, NULL, regs);
		}
    }
#endif
#endif

}

/* PIC clock at 66Mhz takes more than 60 secs to come to 0 from max. So 32bit 
   counter is sufficient
   */
#define PIC_FREE_RUNNING_TIMER_MAX_VAL 0xffffffff
cycle_t xlr_hpt_read(void)
{
	uint32_t counter = 0;
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
	counter = netlogic_read_reg(mmio, PIC_TIMER_6_COUNTER_0);
	return (cycle_t)(PIC_FREE_RUNNING_TIMER_MAX_VAL - counter);
}
EXPORT_SYMBOL(xlr_hpt_read);

int read_current_timer(unsigned long *timer_val)
{
	*timer_val = xlr_hpt_read();
	return 0;
}

void nlm_common_timer_setup(void)
{
        nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
        unsigned long flags = 0;

        spin_lock_irqsave(&phnx_pic_lock, flags);

        /* Use PIC Timer 6 as a free running counter */
        netlogic_write_reg(mmio, PIC_TIMER_6_MAXVAL_0, 0xffffffff);
        netlogic_write_reg(mmio, PIC_TIMER_6_MAXVAL_1, 0xffffffff);
        /* we Don't need interrupts */
        netlogic_write_reg(mmio, PIC_IRT_0_TIMER_6, 0);
        netlogic_write_reg(mmio, PIC_IRT_1_TIMER_6,
                          (1 << 31) | (0 << 30) | (1 << 6) | (PIC_TIMER_6_IRQ));
        pic_update_control(1 << (8 + 6));

        spin_unlock_irqrestore(&phnx_pic_lock, flags);

        //do_gettimeoffset = xlr_gettimeoffset;

}

static int nlm_timer_proc_read(char *page, char **start, off_t off, int count,
			       int *eof, void *data)
{
	int len = 0;

	preempt_disable();
	len += sprintf(page + len, "cpu = %d, eimr = 0x%016llx, status = 0x%x\n", 
				   smp_processor_id(), 
                   (unsigned long long)read_64bit_cp0_eimr(), read_c0_status());
	preempt_enable();
	*eof = 1;

	return len;
}

extern struct proc_dir_entry *nlm_root_proc;
struct proc_dir_entry *main_entry;
struct proc_dir_entry *sub_entry;

static int init_pic_timer_procfs(void)
{
	main_entry = proc_mkdir("nlm_timer", nlm_root_proc);
	if (!main_entry) {
		printk(KERN_ERR "unable to create /proc/nlm_timer\n");
		return -ENOMEM;
	}

	sub_entry = create_proc_entry("debug", 0644, main_entry);

	if (!sub_entry) {
		remove_proc_entry("nlm_timer", nlm_root_proc);
		return -ENOMEM;
	}

	sub_entry->read_proc = nlm_timer_proc_read;

	printk("created NLM_timer proc fs entry\n");

	return 0;
}

static void exit_pic_timer_procfs(void)
{
	remove_proc_entry("debug", main_entry);
	remove_proc_entry("nlm_timer", nlm_root_proc);
}

module_init(init_pic_timer_procfs);
module_exit(exit_pic_timer_procfs);
