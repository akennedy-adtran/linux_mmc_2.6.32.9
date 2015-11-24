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
#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <asm/errno.h>
#include <asm/signal.h>
#include <asm/system.h>
#include <asm/ptrace.h>
#include <asm/kgdb.h>
#include <asm/mipsregs.h>

#include <asm/netlogic/sim.h>
#include <asm/netlogic/nlm_srio.h>
#include <asm/netlogic/msidef.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/debug.h>
#include <asm/thread_info.h>
#include <linux/irq.h>
#ifdef CONFIG_NLM_XLP
#include <asm/netlogic/hal/nlm_hal_pic.h>
#endif
/*
 * These are the routines that handle all the low level interrupt stuff. 
 * Actions handled here are: initialization of the interrupt map, requesting of
 * interrupt lines by handlers, dispatching if interrupts to handlers, probing
 * for interrupt lines 
 */

/* Externs */
extern void xlr_timer_interrupt(struct pt_regs *regs, int irq);
extern void nlm_xlr_msgring_int_handler(unsigned int irq, struct pt_regs *regs);
#ifdef XLP_MERGE_TODO
extern void xlr_smp_time_init(void);
#endif

extern void *ht_config_base;
extern int link0, link1;
struct pic_tmask pic_tmask[PIC_NUM_IRTS];

__u64 phnx_irq_mask;
spinlock_t phnx_pic_lock = SPIN_LOCK_UNLOCKED;

static unsigned int pic_startup(unsigned int irq)
{
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
	unsigned long flags;
	nlm_reg_t reg;
/* 	uint32_t thread_mask = (1 << cpu_logical_map(0)); */

/* 	printk("[%s]: IN irq=%d\n", __func__, irq); */

	if (!PIC_IRQ_IS_IRT(irq))
		return EINVAL;

	spin_lock_irqsave(&phnx_pic_lock, flags);

	/* What happens if this irq was previously not ack'ed? 
	 * Assume, that doesn't happen?
	 */
	reg = netlogic_read_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE);
	/* netlogic_write_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE, reg | (1<<31)); */
	/* By default all the interrupts are initialized as level senstive - fix for the PCMCIA flash */
	netlogic_write_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE,
			  reg | (1 << 6) | (1 << 30) | (1 << 31));
	printk("%s: Writing IRT reg %d with IRQ %d\n", __FUNCTION__, PIC_IRT_1_BASE+irq-PIC_IRQ_BASE, irq);

	spin_unlock_irqrestore(&phnx_pic_lock, flags);

	return 0;
}

static void pic_unmask(unsigned int irq)
{
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
	unsigned long flags;
	nlm_reg_t reg;
/* 	uint32_t thread_mask = (1 << cpu_logical_map(0)); */

/* 	printk("%s.%d: IN irq=%d\n", __func__, __LINE__, irq); */

	if (!PIC_IRQ_IS_IRT(irq))
		return;

	spin_lock_irqsave(&phnx_pic_lock, flags);

	/* What happens if this irq was previously not ack'ed? 
	 * Assume, that doesn't happen?
	 */
	reg = netlogic_read_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE);
	/* netlogic_write_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE, reg | (1<<31)); */
	/* By default all the interrupts are initialized as level senstive - fix for the PCMCIA flash */
	netlogic_write_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE,
			  reg | (1 << 6) | (1 << 30) | (1 << 31));
/* 	printk("%s: Writing IRT reg %d with IRQ %d\n", __FUNCTION__, PIC_IRT_1_BASE+irq-PIC_IRQ_BASE, irq); */

	spin_unlock_irqrestore(&phnx_pic_lock, flags);

	return;
}

static void pic_ack(unsigned int irq)
{
	unsigned long flags;
	nlm_reg_t *pci_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);
	nlm_reg_t *ht_mmio = netlogic_io_mmio(NETLOGIC_IO_HT_OFFSET);
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);

	/* XLS PCIE : the Little Endian region */
	nlm_reg_t *pcie_mmio_le = NULL;

	unsigned long i;
	nlm_reg_t reg;

	//dbg_msg("IN irq=%d\n", irq);

	if (is_xls()) {
		pcie_mmio_le = netlogic_io_mmio(NETLOGIC_IO_PCIE_1_OFFSET);
	}

	if (!PIC_IRQ_IS_IRT(irq))
		return;

	/* Interrupt (level sensitive ) acknowledge method for the PCMCIA flash */

	if (irq == 21) {
		reg = *(unsigned char *)(unsigned long)(0xffffffffBD0001f7ULL);
		reg = *(unsigned int *)(unsigned long)(0xffffffffBEF19180ULL);
		for (i = 0; i < 0x100; i++) ;
		*(unsigned int *)(unsigned long)(0xffffffffBEF19180ULL) = reg;
		for (i = 0; i < 0x1000; i++) ;
		reg = *(unsigned int *)(unsigned long)(0xffffffffBEF19180ULL);
	}

	/* Deal with All PCI-Interrupts.. Brigde ACK */
	if ((irq == 24) && (!is_xls()))
		netlogic_read_reg(pci_mmio, (0x140 >> 2));

	if (irq == 23) {

		/* HyperTransport: Clear INT Status */
		netlogic_read_reg(ht_mmio, (0x700 >> 2));

		/* 
		 *  ---------------------------------------------------------
		 *  Generating EOI.
		 *  Clear Interrupts by directly writing to PLX's CFG Space. 
		 *  1. setup the off value in register 0xB8
		 *     (Interrupt Discovery Configuration, bits 23-16). 
		 *  2. clear the interrupt by setting the IRR bit 
		 *     (bit 63) in reg 0xBC (IRDR). 
		 *  ---------------------------------------------------------
		 *  If more devices are added to HT, we have to use the EOI 
		 *  broadcast.
		 *  ---------------------------------------------------------
		 *  NOTE: Send EOI for all interrupts (INT A, B, C and D).
		 *  Bridge Cards, if plugged into the slot, may re-route 
		 *  interrupts. E.g: Intel Bridge 31154 eval board re-routes 
		 *  INTA of the endpoint to INTC of PLX.
		 *  ---------------------------------------------------------
		 */

		/* Generate EOI for INTA */
		*(volatile uint32_t *)(ht_config_base + 0x008b8) = 0x08c01180;
		*(volatile uint32_t *)(ht_config_base + 0x008bc) = 0x00000080;

		/* Generate EOI for INTB */
		*(volatile uint32_t *)(ht_config_base + 0x008b8) = 0x08c01380;
		*(volatile uint32_t *)(ht_config_base + 0x008bc) = 0x00000080;

		/* Generate EOI for INTC */
		*(volatile uint32_t *)(ht_config_base + 0x008b8) = 0x08c01580;
		*(volatile uint32_t *)(ht_config_base + 0x008bc) = 0x00000080;

		/* Generate EOI for INTD */
		*(volatile uint32_t *)(ht_config_base + 0x008b8) = 0x08c01780;
		*(volatile uint32_t *)(ht_config_base + 0x008bc) = 0x00000080;
	}

	/* Ack the PCIE Block MSI Status Register(s) */
	if (is_xls() && !is_xlsb0_srio()) {
		if (irq == 34) {
			/*Link0 */
			netlogic_write_reg(pcie_mmio_le, (0x90 >> 2),
					  0xffffffff);
		}
		if (irq == 35) {
			/*Link1 */
			netlogic_write_reg(pcie_mmio_le, (0x94 >> 2),
					  0xffffffff);
		}
		if ((is_xls2xx() && irq == 31) || (is_xls_b0() && irq == 36)) {
			/*Link2 */
			netlogic_write_reg(pcie_mmio_le, (0x190 >> 2),
					  0xffffffff);
		}
		if ((is_xls2xx() && irq == 32) || (is_xls_b0() && irq == 37)) {
			/*Link3 */
			netlogic_write_reg(pcie_mmio_le, (0x194 >> 2),
					  0xffffffff);
		}
	}

	spin_lock_irqsave(&phnx_pic_lock, flags);
	netlogic_write_reg(mmio, PIC_INT_ACK,
			(1 << (irq - PIC_IRQ_BASE)));
	spin_unlock_irqrestore(&phnx_pic_lock, flags);
}

static void pic_end(unsigned int irq)
{
	unsigned long flags;
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);

	//dbg_msg("IN irq=%d\n", irq);

	if (!PIC_IRQ_IS_IRT(irq))
		return;

	/* If level triggered, ack it after the device condition is cleared */
	if (!PIC_IRQ_IS_EDGE_TRIGGERED(irq)) {

		spin_lock_irqsave(&phnx_pic_lock, flags);

		netlogic_write_reg(mmio, PIC_INT_ACK,
				  (1 << (irq - PIC_IRQ_BASE)));
		spin_unlock_irqrestore(&phnx_pic_lock, flags);
	}
}

static void pic_shutdown(unsigned int irq)
{
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
	unsigned long flags;
	nlm_reg_t reg;

	//dbg_msg("IN irq=%d\n", irq);

	if (!PIC_IRQ_IS_IRT(irq))
		return;

	spin_lock_irqsave(&phnx_pic_lock, flags);

	/* What happens if this irq is currently pending an ack? 
	 * Assume, that doesn't happen?
	 */
	reg = netlogic_read_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE);
	netlogic_write_reg(mmio, PIC_IRT_1_BASE + irq - PIC_IRQ_BASE,
			  (reg & ~(1 << 31)));

	spin_unlock_irqrestore(&phnx_pic_lock, flags);
}

static int pic_set_affinity(unsigned int irq, const struct cpumask *mask)
{
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
	unsigned long flags;

	//dbg_msg("IN irq=%d, mask=%lx\n", irq, mask);

	if (!PIC_IRQ_IS_IRT(irq))
		return -1;

	spin_lock_irqsave(&phnx_pic_lock, flags);


	netlogic_write_reg(mmio, PIC_IRT_0_BASE + irq - PIC_IRQ_BASE,
			  (uint32_t) (mask->bits[0]));
	spin_unlock_irqrestore(&phnx_pic_lock, flags);

	return 0;
}

static struct irq_chip xlr_pic = {
	.name = "Phoenix-PIC",
	.unmask = pic_unmask,
	.mask = pic_shutdown,
	.ack = pic_ack,
	.end = pic_end,
	.set_affinity = pic_set_affinity
};

static void rsvd_pic_handler_1_1(unsigned int irq)
{
	if(irq < PIC_IRQ_BASE)
		return;
  dbg_msg("Requesting a reserved irq (%d)??", irq);
  return;
}

static void rsvd_pic_handler_1(unsigned int irq)
{
	if(irq < PIC_IRQ_BASE)
		return;
  dbg_msg("handler called for a reserved irq (%d)\n", irq);
}

static int rsvd_pic_handler_2(unsigned int irq, const struct cpumask *mask)
{
	if(irq < PIC_IRQ_BASE)
		return -1;
  dbg_msg("handler called for a reserved irq (%d)\n", irq);
  return 0;
}

struct irq_chip phnx_rsvd_pic_irq_timer = {
  .name     =          "Count-Compare",
  .unmask	=          rsvd_pic_handler_1_1,
  .mask		=          rsvd_pic_handler_1,
  .ack          =          rsvd_pic_handler_1,
  .end          =          rsvd_pic_handler_1,
  .set_affinity =          rsvd_pic_handler_2
};

struct irq_chip phnx_rsvd_pic = {
	.name = "Phoenix-RSVD-PIC",
	.unmask = rsvd_pic_handler_1_1,
	.mask = rsvd_pic_handler_1,
	.ack = rsvd_pic_handler_1,
	.end = rsvd_pic_handler_1,
	.set_affinity = rsvd_pic_handler_2
};

static irqreturn_t phnx_rsvd_irq_handler(int irq, void *dev_id)
{
	if(irq == IRQ_TIMER) 
		return IRQ_HANDLED;
  dbg_msg("handler for reserved irq %d\n", irq);
  return IRQ_NONE;
}

struct irqaction xlr_rsvd_action = {
	.handler = phnx_rsvd_irq_handler,
	.flags = 0,
	//.mask = 0,
	.name = "xlr_rsvd_action",
	.dev_id = 0,
	.next = 0
};

#ifdef CONFIG_KGDB
extern irqreturn_t xlr_kgdb_ipi_handler(int irq, struct pt_regs *regs);
void nlm_kgdb_handler(int irq, struct irq_desc *desc)
{
	irqreturn_t ret;
	struct pt_regs *pt_regs = current_thread_info()->regs;

	ret = xlr_kgdb_ipi_handler(irq, pt_regs);
}
#endif

#ifdef CONFIG_OPROFILE
extern void nlm_common_oprofile_int_handler(int irq, void *dev_id,
					 struct pt_regs *regs);

void nlm_oprofile_ipi_handler(int irq, struct irq_desc *desc)
{
	struct pt_regs *pt_regs = current_thread_info()->regs;

	nlm_common_oprofile_int_handler(irq, NULL, pt_regs);
}
#endif

extern void nlm_smp_function_ipi_handler(unsigned int irq, 
					struct irq_desc *desc);
extern void nlm_smp_resched_ipi_handler(unsigned int irq, 
					struct irq_desc *desc);
extern void nlm_ip_flow_ipi_handler(unsigned int irq, 
						struct irq_desc *desc);
void nlm_msgring_int_handler(unsigned int irq, struct irq_desc *desc);

void __init init_xlr_irqs(void)
{
	int i;

	/* Make all IRQs as level triggered by default */
	for (i = 0; i < NR_IRQS; i++) {
		set_irq_chip(i, &xlr_pic);
		set_irq_handler(i, handle_level_irq);
	}


#ifdef CONFIG_SMP
	irq_desc[IRQ_IPI_SMP_FUNCTION].chip = &phnx_rsvd_pic;
	irq_desc[IRQ_IPI_SMP_FUNCTION].action = &xlr_rsvd_action;
	set_irq_handler(IRQ_IPI_SMP_FUNCTION, nlm_smp_function_ipi_handler);

	irq_desc[IRQ_IPI_SMP_RESCHEDULE].chip = &phnx_rsvd_pic;
	irq_desc[IRQ_IPI_SMP_RESCHEDULE].action = &xlr_rsvd_action;
	set_irq_handler(IRQ_IPI_SMP_RESCHEDULE, nlm_smp_resched_ipi_handler);

	phnx_irq_mask |=
	    ((1ULL << IRQ_IPI_SMP_FUNCTION) | (1ULL << IRQ_IPI_SMP_RESCHEDULE));

#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
	/* PR: New IPI added here for netrx balancing */
	irq_desc[IRQ_IPI_NETRX].chip = &phnx_rsvd_pic;
	irq_desc[IRQ_IPI_NETRX].action = &xlr_rsvd_action;
	phnx_irq_mask |= (1ULL << IRQ_IPI_NETRX);
	set_irq_handler(IRQ_IPI_NETRX, nlm_ip_flow_ipi_handler);
#endif				/* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */

#endif

	/* msgring interrupt */
	irq_desc[IRQ_MSGRING].chip = &phnx_rsvd_pic;
	irq_desc[IRQ_MSGRING].action = &xlr_rsvd_action;
	phnx_irq_mask |= (1ULL << IRQ_MSGRING);
	set_irq_handler(IRQ_MSGRING, nlm_msgring_int_handler);

	/* unmask all PIC related interrupts. If no handler is installed by the 
	 * drivers, it'll just ack the interrupt and return 
	 */
	for (i = PIC_IRT_FIRST_IRQ; i <= PIC_IRT_LAST_IRQ(); i++)
		phnx_irq_mask |= (1ULL << i);

#ifdef CONFIG_OPROFILE
	phnx_irq_mask |= (1ULL << IRQ_IPI_OPROFILE);
	set_irq_handler(IRQ_IPI_OPROFILE, nlm_oprofile_ipi_handler);
#endif

#ifdef CONFIG_KGDB
	phnx_irq_mask |= (1ULL << IRQ_IPI_SMP_KGDB);
	set_irq_handler(IRQ_IPI_SMP_KGDB, nlm_kgdb_handler);
#endif

	phnx_irq_mask |= (1ULL << IRQ_TIMER);
}

void __cpuinit nlm_smp_irq_init(void)
{
	/* Set up kseg0 to be cachable coherent */
//hsxlr	change_c0_config(CONF_CM_CMASK, CONF_CM_DEFAULT);

	/* set interrupt mask for non-zero cpus */
	write_64bit_cp0_eimr(phnx_irq_mask | (1 << IRQ_TIMER));
#ifdef XLP_MERGE_TODO
	xlr_smp_time_init();
#endif
}

/* 
 * MSI hook-up routines for NLM Boards;
 * Arch-dependent implementation called
 * from generic msi.c routines.
 */

struct irq_chip phnx_pic_msi = {
	.name = "Phoenix-PIC-MSI",
	.startup = pic_startup,
	.shutdown = pic_shutdown,
	.ack = pic_ack,
	.end = pic_end,
	.set_affinity = pic_set_affinity
};

void destroy_irq(unsigned int irq)
{
    /* no-op */
}

#ifdef CONFIG_PCI_MSI_XLR

static int get_irq_vector(struct pci_dev *dev)
{

    int irq = 0;

	if (is_xls() && !is_xls2xx() && !is_xls_b0()) {
		/* Currently, PCIE bridges not supported */
		if (link0) {
			if (dev->bus->number == 1)
				irq = PIC_PCIE_LINK0_IRQ;
			else
				irq = PIC_PCIE_LINK1_IRQ;
		} else if (link1) {
			if (dev->bus->number == 1)
				irq = PIC_PCIE_LINK1_IRQ;
		}
	} else if (is_xls2xx() || is_xls_b0()) {
		switch (dev->bus->self->devfn) {
		case 0x0:
			irq = PIC_PCIE_LINK0_IRQ;
			break;
		case 0x8:
			irq = PIC_PCIE_LINK1_IRQ;
			break;
		case 0x10:
			if (is_xls_b0())
				irq = PIC_PCIE_XLSB0_LINK2_IRQ;
			else
				irq = PIC_PCIE_LINK2_IRQ;
			break;
		case 0x18:
			if (is_xls_b0())
				irq = PIC_PCIE_XLSB0_LINK3_IRQ;
			else
				irq = PIC_PCIE_LINK3_IRQ;
			break;
		default:
			break;
		}
	} else {
		irq = PIC_HYPER_IRQ;
	}

	return irq;
}

static int msi_compose_msg(struct pci_dev *pdev, unsigned int irq,
			   struct msi_msg *msg)
{

	unsigned dest;

	if (irq >= 0) {
		dest = 0x00;
		msg->address_hi = MSI_ADDR_BASE_HI;
		msg->address_lo = MSI_ADDR_BASE_LO |
		    MSI_ADDR_DEST_MODE_PHYSICAL |
		    MSI_ADDR_REDIRECTION_CPU | MSI_ADDR_DEST_ID(dest);
		msg->data = MSI_DATA_TRIGGER_EDGE |
		    MSI_DATA_LEVEL_ASSERT |
		    MSI_DATA_DELIVERY_FIXED | MSI_DATA_VECTOR(irq);
	}
	return irq;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	destroy_irq(irq);
}

int arch_setup_msi_irq(struct pci_dev *dev, struct msi_desc *desc)
{
	struct msi_msg msg;
	int irq, ret;

	irq = get_irq_vector(dev);
	if (irq < 0)
		return irq;
	set_irq_msi(irq, desc);
	ret = msi_compose_msg(dev, irq, &msg);
	if (ret < 0) {
		destroy_irq(irq);
		return ret;
	}
	write_msi_msg(irq, &msg);
	irq_desc[irq].chip = &phnx_pic_msi;
	phnx_irq_mask |= (1ULL << irq);
	return irq;
}
#endif


void __init arch_init_irq(void)
{

#ifdef CONFIG_KGDB
	if (kgdb_early_setup)
		return;
#endif

	/* TODO:
	 * Initialize the irq registers on the PIC to route
	 * interrupts to appropriate pins
	 */

	/* Initialize the irq descriptors */
	init_xlr_irqs();

	write_64bit_cp0_eimr(phnx_irq_mask);

}

asmlinkage void plat_irq_dispatch(void)
{
	uint64_t eirr;
	struct pt_regs *pt_regs = current_thread_info()->regs;
	int i = 0;
	eirr = read_64bit_cp0_eirr() & read_64bit_cp0_eimr();

	if (!eirr)
		return;

	if (eirr & (1 << IRQ_TIMER)) {
		xlr_timer_interrupt(pt_regs, IRQ_TIMER);
		return;
	}
	/*TODO use dcltz: optimize below code */
	for (i = 63; i != -1; i--) {
		if (eirr & (1ULL << i))
			break;
	}
	if (i == -1) {
		printk("no interrupt !!\n");
		return;
	}
	/*ack eirr */
	write_64bit_cp0_eirr(1ULL << i);

	do_IRQ(i);
	return;
}


void pic_setup_threadmask(unsigned int irt, uint32_t mask)
{
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
	pic_tmask[irt].mask = mask;
	pic_tmask[irt].set = 1;
	pic_tmask[irt].valid = 1;
 	netlogic_write_reg(mmio, PIC_IRT_0_BASE + irt, mask);
	return;
}

void nlm_msgring_int_handler(unsigned int irq, struct irq_desc *desc)
{
        struct pt_regs *pt_regs = current_thread_info()->regs;

        nlm_xlr_msgring_int_handler(irq, pt_regs);
}
