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

#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/msidef.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/debug.h>
#include <asm/thread_info.h>
#include <asm/netlogic/nlm_common_wdt.h>

/* About this file: irq.c
 * This file contains routines that handle all the low level interrupt stuff.
 * Some of the platform specific portions are moved to arch/mips/pci/pci-xlp.c
 * Actions handled here are: initialization of the interrupt map, requesting of
 * interrupt lines by handlers, dispatching interrupts to handlers, probing
 * for interrupt lines..etc.
 */

/* Externs */
extern int xlp_span_multiple_nodes(const struct cpumask *);
extern void constrict_mask_to_node(u8, struct cpumask *, const struct cpumask*);
extern void nlm_common_timer_interrupt(struct pt_regs *, int);
extern void nlm_oprofile_interrupt(struct pt_regs *, int);
extern void xlp_msgring_int_handler(int, struct pt_regs *);
extern int xlp_ctrl_fn_from_dev(const struct pci_dev *, struct xlp_nodefn_struct *);
int xlp_intx_enable(u8, int);
int xlp_intx_disable(u8, int);
extern void xlp_set_cpumask_on_nodes(const struct cpumask *m, int irt);
extern int xlp_get_ctrl_intmode(u8, int);
extern int xlp_set_ctrl_intmode(u8, int fn, int mode);
#if defined CONFIG_PCI_MSI
int xlp_msi_enable(u8, int, u32);
extern int is_msi_set(u8, int);
extern int calc_msi_vector_offset(u8, int);
int xlp_msi_disable(u8, int, int);
extern u32 xlp_msi_set_mask(u8, int, int, int);
volatile const void *xlp_msix_addr_start(u8, int);
volatile const void *xlp_msi_addr_start(u8, int);
extern u32 xlp_msix_status_clear(u8, int);
int xlp_msix_enable(u8,  int);
int xlp_msix_disable(u8, int);
void mask_msi_irq(unsigned int);
void unmask_msi_irq(unsigned int);
#endif

#ifdef CONFIG_NUMA
extern void xlp_numa_ack_pic(int);
#endif

/* own variables */

/* xlp_irq_mask is retained for legacy. It can be removed at a later point of
 * time. Initially it was meant to keep a copy of present interrupt mask; with
 * multi cpus each having its own mask register, we might not need this variable
 */
static volatile uint64_t xlp_irq_mask;

/* spin lock for all interrupt related data structures
 * This variable is used in timer init, so we export it
 */
spinlock_t xlp_pic_lock = SPIN_LOCK_UNLOCKED;
EXPORT_SYMBOL(xlp_pic_lock);
#if defined CONFIG_PCI_MSI
/*
 * This bitmap keeps track of the MSI vectors allocated from
 * XLP_MSIX_IRQ_START(x)
 */
struct msi_alloc_bitmap {
	u64 bitmap;	/* Can be any data structure to keep bits */
	u32 count;	/* #of bits set at any point of time */
};
static struct msi_alloc_bitmap msix_vec[NLM_MAX_NODES][XLP_MAX_SLOTS];
static struct msi_alloc_bitmap msi_vec[NLM_MAX_NODES][XLP_MAX_SLOTS];
#endif

/*
 * There are two data structures pivotal for interrupt delivery mechanism
 * irq_map[] and rvec_map[]
 * irq_map[] : An array of struct irq_map_elem.
 * Number of elements in this array is equal to NR_IRQ => there should be an
 * entry corresponding to each interrupt in the system.
 * Initial 8 elements (0-XLP_IRQ_RESERVED_MAX) are unpopulated. They are
 * reserved for system internal interrupts which are not explicitly handled
 * by plat_irq_dispatch; the handlers for these interrupts are called
 * differently.
 *
 * All other entries are handled by plat_irq_dispatch()
 * An entry would contain the rvec entry for this interrupt. The offset of this
 * entry would be presented as the interrupt number for any requests
 * E.g, for UART1, index is 141. This is the value of uart1 interrupt.
 * asm/netlogic/xlp_irq.h defines this value as XLP_UART_IRQ(1)
 *
 * Each irq_map_elem has two members : rvec -> the rvec entry for this entry,
 * usage : the number of successful irq_request() called on this IRQ.
 *
 * rvec_map is meant to map rvec numbers back to Interrupts.
 * RVEC is just a number for s/w; which is the bit offset that is set in EIRR
 * Refer PRM : 9.3 for details.
 *
 * irq_map : {<IERR RVEC>, <#of s/w vector multiplexed on this RVEC> }
 * Some RVECs are reserved : so use only 9 through 63
 * Any irt index can be derived from irq using the macros
 * xlp_irt_to_irq() or xlp_irq_to_irt()
 * These macros are required because of the imposed 64 bits (if RVEC)
 * to 160 entries (size of IRT table)
 *
 * It is further complicated by the fact that PCI LINK interrupts are
 * multiplexed with MSI interrupts. In that mode, each PCI Link interrupts
 * can be caused by 32 MSI interrupts. That means, we need a meachinsm to map
 * 32 msi interrupts * 4 pci slots (128 interrupts) to 4 possible RVECs.
 * the irq_map table serves this purpose as well as follows
 *
 * We limit the per pci slot interrupt (for the time being) to XLP_MSI_PER_SLOT
 * (currently 8). This is done to keep total number of interrupts to NR_IRQ;
 *
 */
struct irq_map_elem {
	int rvec;
	int usage[NLM_MAX_NODES];	/* This is the usage count of an rvec, not the number
			   of times a vector is used in s/w */
};

static struct irq_map_elem irq_map[NR_IRQS] = {
	{0, {0,0,0,0}},	/* Dummy			:	0 */
	{0, {0,0,0,0}},	/* Dummy			:	1 */
	{0, {0,0,0,0}},	/* Dummy			:	2 */
	{0, {0,0,0,0}},	/* Dummy			:	3 */
	{0, {0,0,0,0}},	/* Dummy			:	4 */
	{0, {0,0,0,0}},	/* Dummy			:	5 */
	{0, {0,0,0,0}},	/* Dummy			:	6 */
	{0, {0,0,0,0}},	/* Dummy			:	7 */
        {9, {0,0,0,0}}, /*XLP_WD_IDX(0)			:	8 */
        {9, {0,0,0,0}}, /*XLP_WD_IDX(1)			:	9 */
        {19, {0,0,0,0}}, /*XLP_WD_NMI_IDX(0)		:	10 */
        {19, {0,0,0,0}}, /*XLP_WD_NMI_IDX(1)		:	11 */
        {26, {0,0,0,0}}, /*XLP_TIMER_IDX(0): Dedicated systimer	:12 */
        {10, {0,0,0,0}}, /*XLP_TIMER_IDX(1): Dedicated clocksource:13 */
        {10, {0,0,0,0}}, /*XLP_TIMER_IDX(2)		:	14 */
        {10, {0,0,0,0}}, /*XLP_TIMER_IDX(3)		:	15 */
        {10, {0,0,0,0}}, /*XLP_TIMER_IDX(4)		:	16 */
        {10, {0,0,0,0}}, /*XLP_TIMER_IDX(5)		:	17 */
        {10, {0,0,0,0}}, /*XLP_TIMER_IDX(6)		:	18 */
        {10, {0,0,0,0}}, /*XLP_TIMER_IDX(7)		:	19 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(0)		:	20 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(1)		:	21 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(2)		:	22 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(3)		:	23 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(4)		:	24 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(5)		:	25 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(6)		:	26 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(7)		:	27 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(8)		:	28 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(9)		:	29 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(10)		:	30 */
        {30, {0,0,0,0}}, /*XLP_MSGQ_IDX(11)		:	31 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(12)		:	32 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(13)		:	33 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(14)		:	34 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(15)		:	35 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(16)		:	36 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(17)		:	37 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(18)		:	38 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(19)		:	39 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(20)		:	40 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(21)		:	41 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(22)		:	42 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(23)		:	43 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(24)		:	44 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(25)		:	45 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(26)		:	46 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(27)		:	47 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(28)		:	48 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(29)		:	49 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(30)		:	50 */
        {59, {0,0,0,0}}, /*XLP_MSGQ_IDX(31)		:	51 */
        {49, {0,0,0,0}}, /*XLP_MSG_IDX(0)		:	52 */
        {48, {0,0,0,0}}, /*XLP_MSG_IDX(1)		:	53 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(0)		:	54 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(1)		:	55 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(2)		:	56 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(3)		:	57 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(4)		:	58 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(5)		:	59 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(6)		:	60 */
        {32, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(7)		:	61 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(8)		:	62 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(9)		:	63 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(10)	:	64 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(11)	:	65 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(12)	:	66 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(13)	:	67 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(14)	:	68 */
        {33, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(15)	:	69 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(16)	:	70 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(17)	:	71 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(18)	:	72 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(19)	:	73 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(20)	:	74 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(21)	:	75 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(22)	:	76 */
        {34, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(23)	:	77 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(24)	:	78 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(25)	:	79 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(26)	:	80 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(27)	:	81 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(28)	:	82 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(29)	:	83 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(30)	:	84 */
        {35, {0,0,0,0}}, /*XLP_PCIE_MSIX_IDX(31)	:	85 */
        {41, {0,0,0,0}}, /*XLP_PCIE_LINK_IRQ(0)		:	86 */
        {42, {0,0,0,0}}, /*XLP_PCIE_LINK_IRQ(1)		:	87 */
        {43, {0,0,0,0}}, /*XLP_PCIE_LINK_IRQ(2)		:	88 */
        {44, {0,0,0,0}}, /*XLP_PCIE_LINK_IRQ(3)		:	89 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(0)		:	90 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(1)		:	91 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(2)		:	92 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(3)		:	93 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(4)		:	94 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(5)		:	95 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(6)		:	96 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(7)		:	97 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(8)		:	98 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(9)		:	99 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(10)		:	100 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(11)		:	101 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(12)		:	102 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(13)		:	103 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(14)		:	104 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(15)		:	105 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(16)		:	106 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(17)		:	107 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(18)		:	108 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(19)		:	109 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(20)		:	110 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(21)		:	111 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(22)		:	112 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(23)		:	113 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(24)		:	114 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(25)		:	115 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(26)		:	116 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(27)		:	117 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(28)		:	118 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(29)		:	119 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(30)		:	120 */
        {58, {0,0,0,0}}, /*XLP_NAE_IDX(31)		:	121 */
        {60, {0,0,0,0}}, /*XLP_POE_IDX			:	122 */
        {24, {0,0,0,0}}, /*XLP_USB_IDX(0)		:	123 */
        {24, {0,0,0,0}}, /*XLP_USB_IDX(1)		:	124 */
        {24, {0,0,0,0}}, /*XLP_USB_IDX(2)		:	125 */
        {25, {0,0,0,0}}, /*XLP_USB_IDX(3)		:	126 */
        {25, {0,0,0,0}}, /*XLP_USB_IDX(4)		:	127 */
        {25, {0,0,0,0}}, /*XLP_USB_IDX(5)		:	128 */
        {61, {0,0,0,0}}, /*XLP_GDX_IDX			:	129 */
        {63, {0,0,0,0}}, /*XLP_SEC_IDX			:	130 */
        {62, {0,0,0,0}}, /*XLP_RSA_IDX			:	131 */
        {39, {0,0,0,0}}, /*XLP_COMP_IDX(0)		:	132 */
        {39, {0,0,0,0}}, /*XLP_COMP_IDX(1)		:	133 */
        {39, {0,0,0,0}}, /*XLP_COMP_IDX(2)		:	134 */
        {39, {0,0,0,0}}, /*XLP_COMP_IDX(3)		:	135 */
        {0, {0,0,0,0}}, /*RESERVED_IDX			:	136 */
        {37, {0,0,0,0}}, /*XLP_ICC_IDX(0)		:	137  ICC - Inter Chip Coherency*/
        {37, {0,0,0,0}}, /*XLP_ICC_IDX(1)		:	138 */
        {37, {0,0,0,0}}, /*XLP_ICC_IDX(2)		:	139 */
        {36, {0,0,0,0}}, /*XLP_CAM_IDX			:	140 */
        {17, {0,0,0,0}}, /*XLP_UART_IDX(0)		:	141 */
        {18, {0,0,0,0}}, /*XLP_UART_IDX(0)		:	142 */
        {11, {0,0,0,0}}, /*XLP_I2C_IDX(0)		:	143 */
        {11, {0,0,0,0}}, /*XLP_I2C_IDX(0)		:	144 */
        {12, {0,0,0,0}}, /*XLP_SYS_IDX(0)		:	145 */
        {12, {0,0,0,0}}, /*XLP_SYS_IDX(1)		:	146 */
        {55, {0,0,0,0}}, /*XLP_JTAG_IDX			:	147 */
        {50, {0,0,0,0}}, /*XLP_PIC_IDX			:	148 */
        {54, {0,0,0,0}}, /*XLP_NBU_IDX			:	149 */
        {53, {0,0,0,0}}, /*XLP_TCU_IDX			:	150 */
        {31, {0,0,0,0}}, /*XLP_SATA			:	151 */
        {38, {0,0,0,0}}, /*XLP_DMC_IDX			:	152 */	/* collision */
        {38, {0,0,0,0}}, /*XLP_DMC_IDX			:	153 */
        {13, {0,0,0,0}}, /*XLP_GPIO_IDX(0)		:	154 */
        {14, {0,0,0,0}}, /*XLP_GPIO_IDX(1)		:	155 */
        {15, {0,0,0,0}}, /*XLP_GPIO_IDX(2)		:	156 */
        {16, {0,0,0,0}}, /*XLP_GPIO_IDX(3)		:	157 */
        {20, {0,0,0,0}}, /*XLP_NOR_IDX			:	158 */
        {21, {0,0,0,0}}, /*XLP_NAND_IDX			:	159 */
        {22, {0,0,0,0}}, /*XLP_SPI_IDX			:	160 */
        {23, {0,0,0,0}}, /*XLP_MMC_IDX			:	161 */
        {0, {0,0,0,0}}, /*			    162 */
        {0, {0,0,0,0}}, /*                          163 */
        {0, {0,0,0,0}}, /*                          164 */
        {0, {0,0,0,0}}, /*                          165 */
        {0, {0,0,0,0}}, /*                          166 */
        {0, {0,0,0,0}}, /*                          167 */
};

/*
 * When startup function is called on an IRQ, that IRT's rvec map's bitmap
 * would be set. This serves as a quick reverse lookup at the time of dispatch
 */
struct rvec_map_elem {
	/* irt = elem.irt + ffs(bitmap), where bitmap != 0 */
	int irt;	/* The first IRT corresponding to this rvec */
	volatile unsigned long bitmap[NLM_MAX_NODES];	/* bit set is the offset from irt */
};
static struct rvec_map_elem rvec_map[XLP_EIRR_SIZE] = {
	{-1, {0,0,0,0}},			/* 0 */
	{-1, {0,0,0,0}},			/* 1 */
	{-1, {0,0,0,0}},			/* 2 */
	{-1, {0,0,0,0}},			/* 3 */
	{-1, {0,0,0,0}},			/* 4 */
	{-1, {0,0,0,0}},			/* 5 */
	{-1, {0,0,0,0}},			/* 6 */
	{-1, {0,0,0,0}},			/* 7 */
	{-1, {0,0,0,0}},			/* 8 */
	{0, {0,0,0,0}},				/* 9  Watchdog timer */
	{5, {0,0,0,0}},				/* 10 PIC Timers 2 through 7, 0 and 1 reserved*/
	{135, {0,0,0,0}},			/* 11 */
	{137, {0,0,0,0}},			/* 12 */
	{146, {0,0,0,0}},			/* 13 */
	{147, {0,0,0,0}},			/* 14 */
	{148, {0,0,0,0}},			/* 15 */
	{149, {0,0,0,0}},			/* 16 */
	{133, {0,0,0,0}},			/* 17 */
	{134, {0,0,0,0}},			/* 18 */
	{2, {0,0,0,0}},				/* 19 , Watchdog NMI */
	{150, {0,0,0,0}},			/* 20 */
	{151, {0,0,0,0}},			/* 21 */
	{152, {0,0,0,0}},			/* 22 */
	{153, {0,0,0,0}},			/* 23 */
	{115, {0,0,0,0}},			/* 24 */
	{118, {0,0,0,0}},			/* 25 */
	{4, {0,0,0,0}},				/* 26  Dedicated systimer(0) */
	{-1, {0,0,0,0}},			/* 27 */
	{-1, {0,0,0,0}},			/* 28 */
	{-1, {0,0,0,0}},			/* 29 */
	{12, {0,0,0,0}},			/* 30 */
	{143, {0,0,0,0}},			/* 31 */
	{46, {0,0,0,0}},			/* 32  MSIX - FN(0)*/
	{54, {0,0,0,0}},			/* 33  MSIX - FN(1)*/
	{62, {0,0,0,0}},			/* 34  MSIX - FN(2)*/
	{70, {0,0,0,0}},			/* 35  MSIX - FN(3)*/
	{132, {0,0,0,0}},			/* 36 */
	{129, {0,0,0,0}},			/* 37 */
	{144, {0,0,0,0}},			/* 38 */
	{124, {0,0,0,0}},			/* 39 */
	{-1, {0,0,0,0}},			/* 40 */
	{78, {0,0,0,0}},			/* 41 */
	{79, {0,0,0,0}},			/* 42 */
	{80, {0,0,0,0}},			/* 43 */
	{81, {0,0,0,0}},			/* 44 */
	{-1, {0,0,0,0}},			/* 45 */
	{-1, {0,0,0,0}},			/* 46 */
	{-1, {0,0,0,0}},			/* 47 */
	{45, {0,0,0,0}},			/* 48 */
	{44, {0,0,0,0}},			/* 49 XLP_MSG_IDX*/
	{140, {0,0,0,0}},			/* 50 */
	{-1, {0,0,0,0}},			/* 51 */
	{143, {0,0,0,0}},			/* 52 */
	{142, {0,0,0,0}},			/* 53 */
	{141, {0,0,0,0}},			/* 54 */
	{139, {0,0,0,0}},			/* 55 */
	{-1, {0,0,0,0}},			/* 56 */
	{-1, {0,0,0,0}},			/* 57 */
	{82, {0,0,0,0}},			/* 58 */
	{24, {0,0,0,0}},			/* 59 , MSGQ*/
	{114, {0,0,0,0}},			/* 60 */
	{121, {0,0,0,0}},			/* 61 */
	{123, {0,0,0,0}},			/* 62 */
	{122, {0,0,0,0}},			/* 63 */
};

/*
 * Set some eimr bits on each cpu
 * This function will be called on each cpu by on_each_cpu()
 * @bitmask	: bitmask to set in EIMR
 */
void xlp_set_eimr(void *param)
{
	u64 bitmask = (u64) param;
	u64 eimr;

	eimr = read_64bit_cp0_eimr();
	eimr |= bitmask;
	write_64bit_cp0_eimr(eimr);
	return;
}

/*
 * Clear some eimr bits on each cpu
 * This function will be called on each cpu by on_each_cpu()
 * @bitmask	: bitmask to clear in EIMR
 */
void xlp_clear_eimr(void *param)
{
	u64 bitmask = (u64) param;
	u64 eimr = read_64bit_cp0_eimr();
	eimr &= ~bitmask;
	write_64bit_cp0_eimr(eimr);
	return;
}

/*
 * Returns the base IRQ (index of irq_map) from an rvec
 */
static inline int __irqbase_from_rvec(int rvec)
{
	int irt;

	irt = rvec_map[rvec].irt;
	if (irt < 0) {
		return -EINVAL;
	}
	return(irt + XLP_IRQ_RESERVED_MAX);
}


/*
 * Returns the base IRQ from an rvec
 */
static inline int irqbase_from_rvec(int rvec)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&xlp_pic_lock, flags);
	ret = __irqbase_from_rvec(rvec);
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return ret;
}

/*
 * returns rvec from an IRQ entry
 * An IRQ entry is (irt table index + reserved max)
 *
 * @irq : irq number
 */
int xlp_rvec_from_irq(int irq)
{
	irq %= XLP_IRQS_PER_NODE;
	if (irq < XLP_IRQ_RESERVED_MAX){
		return -EINVAL;
	}
	return(irq_map[irq].rvec);
}
EXPORT_SYMBOL(xlp_rvec_from_irq);

/*
 * Masks out one IRQ in the EIMR register
 * Must NOT be called with xlp_pic_lock held
 * @irq : IRQ number
 */
static void __nlm_irq_mask(u8 node, unsigned int irq)
{
	int rvec;

	rvec = xlp_rvec_from_irq(irq);
	if (rvec < 0) {
		return;
	}
	if (read_64bit_cp0_eimr() & (1ULL << rvec)) {
		/* We do not clear eimr, this is a TODO for later time */
		//on_each_cpu(xlp_clear_eimr, (void *) (1ULL << rvec), 1);
	}
	return;
}

/*
 * This function checks if the irq has a valid range for INTX
 * @irq : irq number to check
 */
int check_intx_range(u32 irq)
{
	u32 lirq = irq % XLP_IRQS_PER_NODE;
	/* local intx has a range XLP_IRQ_RESERVED_MAX to XLP_IRQ_MAX */
	if((lirq < XLP_IRQ_RESERVED_MAX) && (lirq >= 0)) {
		return -EINVAL;
	} else if(lirq >= XLP_IRQ_MAX) {
		return -EINVAL;
	}
	return 0;
}

/*
 * This function checks if the irq has a valid range for IRQ
 * @irq : irq number to check
 */
int check_irq_range(u32 oirq)
{
	u32 irq = oirq % XLP_IRQS_PER_NODE;
	if ((irq < XLP_IRQ_RESERVED_MAX) && (irq >= 0)) {
		return -EINVAL;
	} else if(irq >= XLP_IRQ_MAX) {
		pr_err("irq = %d. Invalid irq requested\n", oirq);
		return -EINVAL;
	}
	return 0;
}

/*
 * Interface function (unlocked version) to mask an IRQ
 * Calls helper function after input tests and spin_lock holds
 *
 * @irq : IRQ number
 */
static void nlm_intx_mask(unsigned int irq)
{
	u8 node = XLP_IRQ_TO_NODE(irq);
	//unsigned long flags;
	if(check_intx_range(irq) < 0) {
		return;
	}
	// Once enabled, we don't mask it out
	//spin_lock_irqsave(&xlp_pic_lock, flags);	// Remove XXX
	__nlm_irq_mask(node, irq);				// XXX
	//spin_unlock_irqrestore(&xlp_pic_lock, flags);	// XXX remove
	return;
}

/*
 * Changes eimr bit value corresponding to IRT
 * @irq : IRQ number
 * TODO : Need to find a method to send messages to a subset of cpus as
 * target for this vector is a node and only the cpus in that node should
 * change eimr
 */
static void __nlm_irq_unmask(u8 node, int irq)
{
	int rvec = xlp_rvec_from_irq(irq);

	if (rvec < 0) {
		return;
	} else if (((1ULL << rvec) & read_64bit_cp0_eimr()) == 0) {
		/* This is only for those interrupts which are not statically
		 * set in EIMR. Could dump stack if spin lock held */
		 on_each_cpu(xlp_set_eimr, (void *) (1ULL << rvec), 1);
	}
	return;
}

/*
 * Interface function (unlocked version) to mask an IRQ
 * Calls helper function after input tests and spin_lock holds
 *
 * @irq : IRQ number
 */
static void nlm_intx_unmask(unsigned int irq)
{
	u8 node = XLP_IRQ_TO_NODE(irq);
	//unsigned long flags;
	if(check_intx_range(irq) < 0) {
		return;
	}
	//spin_lock_irqsave(&xlp_pic_lock, flags);
	__nlm_irq_unmask(node, irq);
	//spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return;
}

static void nlm_irq_ack(u8 node, unsigned int irt)
{
	unsigned long flags;
	spin_lock_irqsave(&xlp_pic_lock, flags);
	xlp_ack_pic(node, irt);
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
}

static void nlm_intx_ack(unsigned int irq)
{
	u32 irt = xlp_irq_to_irt(irq);
	if (check_intx_range(irq) < 0) {
		/* No need to ack. Not by PIC */
		return;
	}
#if !defined CONFIG_XLP_REPLACE_R4K_TIMER/* all are level triggered in XLP */
	if (PIC_IRQ_IS_EDGE_TRIGGERED(irt)) {
		nlm_irq_ack(XLP_IRQ_TO_NODE(irq), irt);
	}
#endif
}

static void nlm_irq_end(u8 node, unsigned int irq)
{
	if (check_irq_range(irq) < 0) {
		/* No need to end(ack). Not by PIC */
		return;
	}
	/* If level triggered, ack it after the device condition is cleared */
#if defined CONFIG_XLP_REPLACE_R4K_TIMER	/* all are level triggered in XLP */
	nlm_irq_ack(node, xlp_irq_to_irt(irq));
#else
	if (!PIC_IRQ_IS_EDGE_TRIGGERED(xlp_irq_to_irt(irq))) {
		nlm_irq_ack(node, xlp_irq_to_irt(irq));
	}
#endif
	return;
}

static void nlm_intx_end(unsigned int irq)
{
	u8 node = XLP_IRQ_TO_NODE(irq);
	if (check_intx_range(irq) < 0) {
		/* No need to end(ack). Not by PIC */
		return;
	}
	/* If level triggered, ack it after the device condition is cleared */
#if defined CONFIG_XLP_REPLACE_R4K_TIMER	/* all are level triggered in XLP */
	nlm_irq_ack(node, xlp_irq_to_irt(irq));
#else
	if (!PIC_IRQ_IS_EDGE_TRIGGERED(xlp_irq_to_irt(irq))) {
		nlm_irq_ack(node, xlp_irq_to_irt(irq));
	}
#endif
	return;
}

/*
 * Startup function for any IRQ
 * @irq: irq number
 *
 * This function is called as chip->startup() for all IRQs.
 * In case of XLP, all normal interrupts must fall below XLP_MSI_IRQ_OFFSET
 * When an interrupt is started, we force it to be enabled only in cpu0, it can
 * be changed later by calling nlm_irq_set_affinity()
 */
extern void xlp_set_cpumask_on_node(u8, const struct cpumask *, int);
static unsigned int nlm_irq_startup(u8 node, unsigned int irq)
{
	__label__ __failure;
	int ret = 0;
	int idx, rvec;
	struct cpumask m;

	if (check_irq_range(irq) < 0) {
		return -EFAULT;
	}
	cpumask_clear(&m);
	cpumask_set_cpu(NLM_MAX_CPU_PER_NODE * node, &m);
	spin_lock(&xlp_pic_lock);		// Don't disable interrupts here - this will be called with interrupts disabled if needed
	rvec = xlp_rvec_from_irq(irq);
	if (irq_map[irq].usage[node] == 0) {
		/* Currently unused => not enabled. So, setup and enable */
		xlp_set_cpumask_on_node(node, &m, irq);
		ret = __nlm_hal_request_irq(node, xlp_irq_to_irt(irq), rvec);
		if (ret != 0) {
			printk(KERN_WARNING "Failed to setup IRQ %d\n", irq);
			goto __failure;
		}
		idx = irq - __irqbase_from_rvec(rvec);
		set_bit(idx, &(rvec_map[rvec].bitmap[node]));
		irq_map[irq].usage[node]++;
		/* At this point, make sure that each CPU has eimr bit
		 * corresponding to this IRQ set. Later the driver can set
		 * the cpu affinity of this interrupt. The rationale for
		 * setting up EIMR here is that it can be moved to any CPUs
		 * (well, a subset of any CPUs) later
		 */
		__nlm_irq_unmask(node, irq);
	} else if (irq_map[irq].usage[node] > 0) {
		/* already being used. No need to check mask
		 * if masked, will be unmasked later
		 */
		irq_map[irq].usage[node]++;
		ret = 0;
	} else {
		pr_err("Error irq = %d, rvec = %d, usage count %d\n", irq,
				irq_map[irq].rvec, irq_map[irq].usage[node]);
		ret = -EFAULT;
	}
__failure:
	spin_unlock(&xlp_pic_lock);
	return ret;
}

/*
 * Startup functionf or regular interrupts
 */
static unsigned int nlm_intx_startup(unsigned int irq)
{
	int fn, ret;
	u8 node = XLP_IRQ_TO_NODE(irq);

	if (check_intx_range(irq) < 0) {
#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
		if (irq !=  XLP_IRQ_TIMER_RVEC)
#endif
		fdebug("Invalid irq %#x\n", irq);
		return -EINVAL;
	}
	/* if this irq correspond to any of the pci slots in any node,
	 * enable intx on the controller of node */
	if ((irq >= XLP_PCIE_LINK_IRQ(node, 0))
			&& (irq <= XLP_PCIE_LINK_IRQ(node, 3))) {
		fn = XLP_INTX_TO_CTRL_FN(irq);
		if ((ret = xlp_intx_enable(node, fn)) < 0) {
			return (unsigned int)ret;
		}
	}
	return nlm_irq_startup(node, irq % XLP_IRQS_PER_NODE);
}

/*
 * IRQ shut down function
 * Disables one IRQ
 *
 * @irq : irq to shut down
 *
 * This function is called whenever release_irq() is called by means of
 * chip->shutdown(). In this function, the rvec bit in every EIMR is cleared if
 * usage falls to zero (in case of shared interrupts)
 */
static void nlm_irq_shutdown(u8 node, unsigned int irq)
{
	unsigned long flags;
	int idx, rvec;

	if (check_irq_range(irq) < 0) {
		return;
	}
	spin_lock_irqsave(&xlp_pic_lock, flags);
	if (irq_map[irq].usage[node] == 0) {
		spin_unlock_irqrestore(&xlp_pic_lock, flags);
		return;
	} else if (irq_map[irq].usage[node] > 0) {
		irq_map[irq].usage[node]--;
	}
	if ((rvec = xlp_rvec_from_irq(irq)) < 0) {
		return;
	}
	/* If the usage reaches zero as a result of above subtraction,
	 * free up the rvec */
	if (irq_map[irq].usage[node] == 0) {
		idx = irq - __irqbase_from_rvec(rvec);
		clear_bit(idx, &(rvec_map[rvec].bitmap[node]));
		spin_unlock_irqrestore(&xlp_pic_lock, flags);
		__nlm_irq_mask(node, irq); /* masks this IRQ */
	} else {
		spin_unlock_irqrestore(&xlp_pic_lock, flags);
	}
	return;
}

/*
 * Shutdown function for intx
 */
static void nlm_intx_shutdown(unsigned int irq)
{
	int fn, ret;
	u8 node = XLP_IRQ_TO_NODE(irq);

	if (check_intx_range(irq) < 0) {
		fdebug("Invalid irq %#x\n", irq);
		return;
	}
	/* if this irq correspond to any of the pci slots, disable intx on
	 * the controller  before shutting it down */
	if ((irq >= XLP_PCIE_LINK_IRQ(node, 0))
			&& (irq <= XLP_PCIE_LINK_IRQ(node, 3))) {
		fn = XLP_INTX_TO_CTRL_FN(irq);
		if ((ret = xlp_intx_disable(node, fn)) < 0) {
			return;
		}
	}
	return nlm_irq_shutdown(node, irq % XLP_IRQS_PER_NODE);
}

static int nlm_irq_set_affinity(u8 node, unsigned int irq, const struct cpumask *mask)
{
	unsigned long flags;

	if (check_irq_range(irq) < 0) {
		return -EINVAL;
	}
	spin_lock_irqsave(&xlp_pic_lock, flags);
	xlp_set_cpumask_on_node(node, mask, irq);
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return 0;
}

/*
 * Set affinity for the intx for chips
 *
 * When an interrupt is setup, its EIMR bit is set in all online cpus. That is,
 * any cpu _can_ receive that interrupt. But it is the IRT entry that decides
 * whether to send that interrupt (i.e, whether to set EIRR bit or not) to any
 * particular CPU.
 *
 * IRT has two modes to decide the target CPUs for one interrupt.
 * Method 1 : Using IRT table entry bits DT and DTE
 * If DT==1, this interrupt can be routed to a max of 16 CPUs (well, hw threads)
 * If DT==1, there is one more level of indirection called DTE. Each DTE entry
 * has 128 bits and there are a total of 8 DTE entries. Each DTE entry contains
 * the bitmask of target CPU for an interrupt. One of them is chosen based
 * on the specified cpumask.
 *
 * The actual bitmask can be different from the specified bitmask based
 * on the logic of xlp_closest_match_cpumask()
 */
static int nlm_intx_set_affinity(unsigned int irq, const struct cpumask *mask)
{
	struct cpumask m;
	u8 node = XLP_IRQ_TO_NODE(irq);

	if (check_intx_range(irq) < 0) {
		return -EINVAL;
	}
	if (xlp_span_multiple_nodes(mask) != 0) {
		/* this is the policy for MSI. Change later TODO */
		constrict_mask_to_node(node, &m, mask);
	} else {
		cpumask_copy(&m, mask);
	}
	return nlm_irq_set_affinity(node, irq % XLP_IRQS_PER_NODE, &m);
}

static struct irq_chip nlm_intx_pic = {
	.name = "XLP-INTX",
	.mask = nlm_intx_mask,
	.unmask = nlm_intx_unmask,
	.startup = nlm_intx_startup,
	.shutdown = nlm_intx_shutdown,
	.ack = nlm_intx_ack,
	.end = nlm_intx_end,
	.set_affinity = nlm_intx_set_affinity
};

static void rsvd_pic_handler_1(unsigned int irq)
{
	if (check_irq_range(irq) < 0) {
		pr_err("Operation on a reserved irq (%d)??", irq);
	}
	return;
}

static int rsvd_pic_handler_2(unsigned int irq, const struct cpumask *mask)
{
	rsvd_pic_handler_1(irq);
	return 0;
}

struct irq_chip nlm_common_rsvd_pic = {
	.name = "Netlogic-RSVD-PIC",
	.unmask = rsvd_pic_handler_1,
	.mask = rsvd_pic_handler_1,
	.ack = rsvd_pic_handler_1,
	.end = rsvd_pic_handler_1,
	.set_affinity = rsvd_pic_handler_2
};

static irqreturn_t nlm_common_rsvd_irq_handler(int irq, void *dev_id)
{
#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
	if ((irq % XLP_IRQS_PER_NODE) == XLP_IRQ_TIMER_RVEC) {
		return IRQ_HANDLED;
	}
#else
	switch (irq) {
		case XLP_TIMER_IRQ(0, 0) ... XLP_TIMER_IRQ(0, 7):
		case XLP_TIMER_IRQ(1, 0) ... XLP_TIMER_IRQ(1, 7):
		case XLP_TIMER_IRQ(2, 0) ... XLP_TIMER_IRQ(2, 7):
		case XLP_TIMER_IRQ(3, 0) ... XLP_TIMER_IRQ(3, 7):
		return IRQ_HANDLED;
		default:
		break;
	}
#endif
	return IRQ_NONE;
}

struct irqaction nlm_common_rsvd_action = {
	.handler = nlm_common_rsvd_irq_handler,
	.flags = 0,
	.name = "nlm_common_rsvd_action",
	.dev_id = 0,
	.next = 0
};

void do_nlm_common_IRQ(unsigned int irq, struct pt_regs *regs)
{
	int lirq = irq % XLP_IRQS_PER_NODE;

	if (lirq == XLP_IRQ_IPI_SMP_FUNCTION_RVEC || lirq == XLP_IRQ_IPI_SMP_RESCHEDULE_RVEC) {
		nlm_common_ipi_handler(lirq, regs);
		return;
	}
	if (lirq == XLP_IRQ_MSGRING_RVEC) {
		xlp_msgring_int_handler(lirq, regs);
		return;
	}
	else if (lirq == XLP_IRQ_IPI_SMP_KGDB_RVEC) {
		/* ignore now */
	} else {
		do_IRQ(irq);	/* Pass IRQ; not lirq */
	}
}

/* Unused function? Remove later */
void __cpuinit nlm_smp_irq_init(void)
{
#ifdef XLP_MERGE_TODO
	/* Set up kseg0 to be cachable coherent */
	change_c0_config(CONF_CM_CMASK, CONF_CM_DEFAULT);
#endif
	write_64bit_cp0_eimr(xlp_irq_mask);
}

void destroy_irq(unsigned int irq)
{
    /* no-op */
}

#ifdef CONFIG_PCI_MSI

static int check_msi_range(unsigned int msi)
{
	int lirq = msi % XLP_IRQS_PER_NODE;
	if((lirq < XLP_IRQ_RESERVED_MAX) && (lirq >= 0)) {
		return -EINVAL;
	} else if(lirq < XLP_MSI_INDEX_START || lirq > XLP_MSI_INDEX_END) {
		return -EINVAL;
	}
	return 0;
}
/*
 * The MSI and MSI-X functionality is supported only by the PCIe Controller.
 * Whenever there is a request for MSI/MSI-X, we need to find out the
 * controller on which that request is made. But in the PCI structure, I could
 * not find a place where that information can be kept. Moreover, a hack job
 * is not justified for the reason that only a small subset of PCI devices
 * (no onboard ones) require this. So, we have the file arch/mips/pci/pci-xlp.c
 * for XLP specific functionality.
 *
 * In case of MSI, we have to share one interrupt (XLP_PCIE_LINK_IRQ(x)) for
 * for 32 possible MSI on a slot.
 * We work around this problem by limiting the #of MSI per slot to
 * XLP_MSI_PER_SLOT. MSI IRQ vectors start from XLP_MSI_IRQ_OFFSET.
 * plat_irq_dispatch checks the vector number and dispatch it correctly.
 *
 * These bunch of functions would find out the controller function using the
 * passed parameter and use nlm_irq_* function to operate on that IRT
 */
static unsigned int nlm_msi_startup(unsigned int msi)
{
	int bit, irq, fn, base_msi, ret;
	u8 node;

	if (check_msi_range(msi) < 0) {
		return 0;
	}
	fn = XLP_MSI_TO_CTRL_FN(msi);
	node = XLP_MSI_TO_NODE(msi);
	irq = XLP_PCIE_LINK_IRQ(0, XLP_MSI_TO_CTRL_FN(msi)); /*Note:NODE == 0*/
	base_msi = XLP_MSI_IRQ_START(0, fn); /*Note:NODE == 0*/
	bit = msi - base_msi;
	if ((ret = xlp_msi_enable(node, fn, bit)) < 0) {
		return ret;
	}
	/* unmask MSI in the device */
	unmask_msi_irq(msi);
	return nlm_irq_startup(node, irq);
}

static int nlm_msi_set_affinity(unsigned int msi, const struct cpumask *mask)
{
	struct cpumask m;
	u8 node = XLP_MSI_TO_NODE(msi);

	if (check_msi_range(msi) < 0) {
		return -EINVAL;
	}
	if (xlp_span_multiple_nodes(mask) != 0) {
		/* this is the policy for MSI. Change later TODO */
		constrict_mask_to_node(node, &m, mask);
	} else {
		cpumask_copy(&m, mask);
	}
	return nlm_irq_set_affinity(node,
		XLP_PCIE_LINK_IRQ(0, XLP_MSI_TO_CTRL_FN(msi)), &m); /* Note : node == 0 passed for link_irq*/
}

static void nlm_msi_shutdown(unsigned int msi)
{
	int bit, irq, fn, base_msi;
	u8 node = XLP_MSI_TO_NODE(msi);

	if (check_msi_range(msi) < 0) {
		return;
	}
	/* mask MSI in the device */
	mask_msi_irq(msi);
	fn = XLP_MSI_TO_CTRL_FN(msi);
	irq = XLP_PCIE_LINK_IRQ(0, fn);	/* actual irq line (irt + max reserved) */
	base_msi = XLP_MSI_IRQ_START(node, fn);
	bit = msi - base_msi;
	if (xlp_msi_disable(node, fn, bit) < 0) {
		return;
	}
	return nlm_irq_shutdown(node, irq);
}

static void nlm_msi_end(unsigned int msi)
{
	if (check_msi_range(msi) < 0) {
		return;
	}
	nlm_irq_end(XLP_MSI_TO_NODE(msi), /* Note 0 as node below */
			XLP_PCIE_LINK_IRQ(0, XLP_MSI_TO_CTRL_FN(msi)));
	return;
}

static void nlm_msi_ack(unsigned int msi)
{
	if (check_msi_range(msi) < 0) {
		return;
	}
	return nlm_irq_ack(XLP_MSI_TO_NODE(msi),
		/* Note 0 as node below */
		xlp_irq_to_irt(XLP_PCIE_LINK_IRQ(0, XLP_MSI_TO_CTRL_FN(msi))));
}

/*
 * Masks just one MSI
 * @msi : the MSI to mask
 */

static u32 nlm_msi_change_mask(unsigned int msi, int val)
{
	unsigned long flags;
	int bit, fn;
	u32 mask;
	u8 node = XLP_MSI_TO_NODE(msi);

	fn = XLP_MSI_TO_CTRL_FN(msi);
	bit = msi - XLP_MSI_IRQ_START(node, fn);
	spin_lock_irqsave(&xlp_pic_lock, flags);
	mask = xlp_msi_set_mask(node, fn, bit, val);
	if (val == 0) {
		if (mask == 0) { /* This was the last bit to clear */
			__nlm_irq_mask(node,
				XLP_PCIE_LINK_IRQ(0, XLP_MSI_TO_CTRL_FN(msi)));
		}
		if ((mask & (mask - 1)) == 0) {	/* Just set the only bit*/
			__nlm_irq_unmask(node,
				XLP_PCIE_LINK_IRQ(0, XLP_MSI_TO_CTRL_FN(msi)));
		}
	}
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return mask;
}

/*
 * Mask just one MSI
 * The corresponding IRT will also be masked
 */
static void nlm_msi_mask(unsigned int msi)
{
	if (check_msi_range(msi) < 0) {
		return ;
	}
	/* mask MSI in the device */
	mask_msi_irq(msi);
	/* This is the h/w specific per vector masking function
	 * We can't use the standard function because we don't support
	 * it in the capability structure */
	nlm_msi_change_mask(msi, 0);
	return;
}

/*
 * Unmask just one MSI
 * The corresponding IRT will also be unmasked
 */
static void nlm_msi_unmask(unsigned int msi)
{
	if (check_msi_range(msi) < 0) {
		return ;
	}
	/* unmask MSI in the device */
	unmask_msi_irq(msi);
	nlm_msi_change_mask(msi, 1);
	return;
}

/*
 * MSI hook-up routines for Netlogic Boards;
 * Arch-dependent implementation called
 * from generic msi.c routines.
 */

struct irq_chip nlm_msi_pic = {
	.name = "XLP-PIC-MSI",
	.startup = nlm_msi_startup,
	.shutdown = nlm_msi_shutdown,
	.ack = nlm_msi_ack,
	.end = nlm_msi_end,
	.mask = nlm_msi_mask,
	.unmask = nlm_msi_unmask,
	.set_affinity = nlm_msi_set_affinity
};

/*
 * These functions would find out the controller function using the
 * passed parameter and use nlm_irq_* function to operate on that IRT
 */
static int check_msix_range(unsigned int msix)
{
	int lirq = msix % XLP_IRQS_PER_NODE;
	if((lirq < XLP_IRQ_RESERVED_MAX) && (lirq >= 0)) {
		return -EINVAL;
	} else if(lirq < XLP_MSIX_INDEX_START || lirq > XLP_MSIX_INDEX_END) {
		return -EINVAL;
	}
	return 0;
}

static int nlm_msix_set_affinity(unsigned int msix, const struct cpumask *mask)
{
	struct cpumask m;
	u8 node = XLP_MSIX_TO_NODE(msix);
	u32 lmsix;

	if (check_msix_range(msix) < 0) {
		return -EINVAL;
	}
	if (xlp_span_multiple_nodes(mask) != 0) {
		constrict_mask_to_node(node, &m, mask);
	} else {
		cpumask_copy(&m, mask);
	}
	lmsix = msix % XLP_IRQS_PER_NODE;
	return nlm_irq_set_affinity(node, XLP_PCIE_MSIX_IRQ(0, lmsix - XLP_MSIX_INDEX_START), &m);
}

static void nlm_msix_end(unsigned int msix)
{
	u32 lmsix;
	u8 node = XLP_MSIX_TO_NODE(msix);

	if (check_msix_range(msix) < 0) {
		return;
	}
	lmsix = msix % XLP_IRQS_PER_NODE;
	nlm_irq_end(node, XLP_PCIE_MSIX_IRQ(0, lmsix - XLP_MSIX_INDEX_START));
	return;
}

static void nlm_msix_ack(unsigned int msix)
{
	u32 lmsix;
	u8 node = XLP_MSIX_TO_NODE(msix);

	if (check_msix_range(msix) < 0) {
		return;
	}
	lmsix = msix % XLP_IRQS_PER_NODE;
	nlm_irq_ack(node, xlp_irq_to_irt(XLP_PCIE_MSIX_IRQ(0, lmsix - XLP_MSIX_INDEX_START)));
	return;
}

/*
 * Masks just one MSIX
 * @msix : the MSIX to mask
 *
 * MSI-X masking is different from MSI masking (msi->temporarily disable
 * in the h/w. That is an XLP oddity.
 * MSI-X has masking as a standard feature
 */
static void nlm_msix_mask(unsigned int msix)
{
	unsigned long flags;
	if (check_msix_range(msix) < 0) {
		return;
	}
	spin_lock_irqsave(&xlp_pic_lock, flags);
	mask_msi_irq(msix); /* This function masks MSI-X -- please note */
	__nlm_irq_mask(XLP_MSIX_TO_NODE(msix), XLP_PCIE_MSIX_IRQ(0,
			(msix % XLP_IRQS_PER_NODE) - XLP_MSIX_INDEX_START));
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return;
}

/*
 * Unmask just one MSIX
 * If required, unmask the corresponding IRT as well
 */
static void nlm_msix_unmask(unsigned int msix)
{
	unsigned long flags;
	if (check_msix_range(msix) < 0) {
		return;
	}
	spin_lock_irqsave(&xlp_pic_lock, flags);
	__nlm_irq_unmask(XLP_MSIX_TO_NODE(msix), XLP_PCIE_MSIX_IRQ(0,
		(msix % XLP_IRQS_PER_NODE) - XLP_MSIX_INDEX_START));
	unmask_msi_irq(msix); /* Enable MSI-X -- please note */
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return;
}

static unsigned int nlm_msix_startup(unsigned int msix)
{
	u8 node = XLP_MSIX_TO_NODE(msix);
	int fn = XLP_MSIX_TO_CTRL_FN(msix), ret;

	if (check_msix_range(msix) < 0) {
		return -EINVAL;
	}
	if ((ret = xlp_msix_enable(node, fn)) < 0) {
		return ret;
	}
	nlm_msix_unmask(msix);
	return nlm_irq_startup(node, XLP_PCIE_MSIX_IRQ(0,
			(msix % XLP_IRQS_PER_NODE) - XLP_MSIX_INDEX_START));
}

static void nlm_msix_shutdown(unsigned int msix)
{
	int fn = XLP_MSIX_TO_CTRL_FN(msix), ret;
	u8 node = XLP_MSIX_TO_NODE(msix);

	if (check_msix_range(msix) < 0) {
		return;
	}
	nlm_msix_mask(msix);
	if ((ret = xlp_msix_disable(node, fn)) < 0) {
		return;
	}
	nlm_irq_shutdown(node, XLP_PCIE_MSIX_IRQ(0,
			(msix % XLP_IRQS_PER_NODE) - XLP_MSIX_INDEX_START));
	return;
}

/*
 * MSI-X hook-up routines for Netlogic Boards;
 * Arch-dependent implementation called
 * from generic msi.c routines.
 */

struct irq_chip nlm_msix_pic = {
	.name = "XLP-PIC-MSIX",
	.startup = nlm_msix_startup,
	.shutdown = nlm_msix_shutdown,
	.ack = nlm_msix_ack,
	.end = nlm_msix_end,
	.mask = nlm_msix_mask,
	.unmask = nlm_msix_unmask,
	.set_affinity = nlm_msix_set_affinity
};


/*
 * Composes MSI/MSI-X messages
 */
static int xlp_msi_compose_msg(struct pci_dev *pdev, struct msi_desc *desc,
		unsigned int irq, struct msi_msg *msg)
{
	struct xlp_nodefn_struct nfn;
	u8 offset;

	if( xlp_ctrl_fn_from_dev(pdev, &nfn) < 0) {
		return -EINVAL;
	}
	if (desc->msi_attrib.is_msix) {
		if (check_msix_range(irq)) {	/* enforce minimum */
			dev_err(&pdev->dev, "Invalid irq %d", irq);
			return -EINVAL;
		}
		offset = (irq % XLP_IRQS_PER_NODE) - XLP_MSIX_INDEX_START;
		msg->address_hi = (virt_to_phys(xlp_msix_addr_start(nfn.node, nfn.fn)) >> 32);
		msg->address_lo = (virt_to_phys(xlp_msix_addr_start(nfn.node, nfn.fn)) & 0xffffffff);
		dev_err(&pdev->dev, "MSI-X hi = %#x, lo = %#x, data = %#x\n", msg->address_hi, msg->address_lo, offset);
	} else {
		if (check_msi_range(irq)) {	/* enforce minimum */
			return -EINVAL;
		}
		offset = (irq % XLP_IRQS_PER_NODE) -
					(XLP_MSI_IRQ_START(0, nfn.fn));
		msg->address_hi = (virt_to_phys(xlp_msi_addr_start(nfn.node, nfn.fn)) >> 32) & 0xffffffff;
		msg->address_lo = (virt_to_phys(xlp_msi_addr_start(nfn.node, nfn.fn)) & 0xffffffff);
		dev_err(&pdev->dev, "MSI hi = %#x, lo = %#x, data = %#x\n", msg->address_hi, msg->address_lo, offset);
	}
	msg->data = offset;
	return 0;
}

/*
 * Returns the bitmask of currently used MSI-X on controller fn
 *
 * Must call with lock held
 * @fn : controller number
 */
# if 0
u32 __xlp_msix_bitmask(u8 node, int fn)
{
	int idx = 0, ret = 0;

	while (idx < XLP_MSIX_PER_SLOT) {
		if (irq_map[XLP_MSIX_IRQ_START(fn) + idx].usage[node] > 0) {
			ret |= (1ULL << idx);
		}
		idx++;
	}
	return ret;
}
#endif

/*
 * Back end of disable_msi()/ disable_msix()
 */
void arch_teardown_msi_irq(unsigned int msi)
{
	unsigned long flags;
	int bit, fn;
	u8 node = XLP_MSIX_TO_NODE(msi);
	unsigned int lmsi = msi % XLP_IRQS_PER_NODE;

	switch (msi) {
	case XLP_MSI_INDEX_START ... XLP_MSI_INDEX_END:
		spin_lock_irqsave(&xlp_pic_lock, flags);
		fn = XLP_MSI_TO_CTRL_FN(msi);
		bit = lmsi - XLP_MSI_IRQ_START(0, fn);
		msi_vec[node][fn].count--;
		msi_vec[node][fn].bitmap &= ~(1ULL << bit);
		if (xlp_get_ctrl_intmode(node, fn) == XLP_INTMODE_MSI) {
			if (xlp_set_ctrl_intmode(node, fn, XLP_INTMODE_NONE) < 0){
			}
		}
		spin_unlock_irqrestore(&xlp_pic_lock, flags);
		return;
	case XLP_MSIX_INDEX_START ... XLP_MSIX_INDEX_END:
		fn = XLP_MSIX_TO_CTRL_FN(msi);
		bit = (msi % XLP_IRQS_PER_NODE) - XLP_MSIX_IRQ_START(0, fn);
		spin_lock_irqsave(&xlp_pic_lock, flags);
		msix_vec[node][fn].count--;
		msix_vec[node][fn].bitmap &= ~(1ULL << bit);
		if (xlp_get_ctrl_intmode(node, fn) == XLP_INTMODE_MSIX) {
			if (xlp_set_ctrl_intmode(node, fn, XLP_INTMODE_NONE) < 0){
			}
		}
		spin_unlock_irqrestore(&xlp_pic_lock, flags);
		return;
	default:
		return;	/* Do not proceed if !(msi || msix) */
	}
}

#endif		// CONFIG_PCI_MSI

static int xlp_perf_irq(void)
{
	return IRQ_HANDLED;
}

/*
 * Entry function for interrupts
 */
asmlinkage void plat_irq_dispatch(void)
{
	volatile u64 eirr;
	volatile u64 eimr;
	volatile u64 bitmap;
	struct pt_regs *pt_regs = current_thread_info()->regs;
	int rvec = 0, idx = 0, base_irq, irq, fn;
	unsigned long flags;
	u8 node = hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE;

	eirr = read_64bit_cp0_eirr();
	eimr = read_64bit_cp0_eimr();
	eirr &= eimr;
#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
	if (eirr & (1ULL << XLP_IRQ_TIMER_RVEC)) {
		nlm_common_timer_interrupt(pt_regs, XLP_IRQ_TIMER_RVEC);
		return;
	}
	if ( eirr & ( 1ULL << XLP_IRQ_OPROFILE)) {
		nlm_oprofile_interrupt(pt_regs,XLP_IRQ_OPROFILE);
		return;
	}
#else
	/* Dedicated processing only for timer interrupt (RVEC 26, IRT 12) */
	if (eirr & (1ULL << XLP_PIC_SYSTIMER_RVEC)) {
		rvec = __ilog2_u64(eirr);
		write_64bit_cp0_eirr(1ULL << rvec);
		do_IRQ(XLP_TIMER_IRQ(node, 0));
		eirr &= ~(1ULL << XLP_PIC_SYSTIMER_RVEC);
	}
#endif
	/* Loop till all bits of eirr is cleared */
	while (eirr) {

	rvec = __ilog2_u64(eirr);
	if (rvec == -1) {
		return;
	}
	eirr &= ~(1ULL << rvec);
	write_64bit_cp0_eirr(1ULL << rvec);
	if (rvec < XLP_IRQ_RESERVED_MAX) {
		irq = rvec;
		do_nlm_common_IRQ(irq, pt_regs);
		return;
	} else {
		/* We need to loop through all possible irqs for an rvec */
		base_irq = irqbase_from_rvec(rvec);
		if(base_irq < 0) {
			return;
		}
		spin_lock_irqsave(&xlp_pic_lock, flags);
		bitmap = rvec_map[rvec].bitmap[node];
		spin_unlock_irqrestore(&xlp_pic_lock, flags);
		switch(base_irq) {
		/* For INTX, bitmap and base irq already set */
#if defined CONFIG_PCI_MSI
		/* These are not MSI vector numbers, but IRT #s */
		case XLP_PCIE_LINK_IRQ(0, 0) ... XLP_PCIE_LINK_IRQ(0, 3):
			/* Here fn # of controller is easily calculated
			 * Check the IRT table : 0 -> 78, 1-> 79 ..etc */
			fn = base_irq - XLP_PCIE_LINK_IRQ(0, 0);
			if (is_msi_set(node, fn) != 0) { /* this is an MSI */
				/* find vectors set, overwrite bitmap */
				bitmap = calc_msi_vector_offset(node, fn);
				/* recalculate base_irq for MSI */
				base_irq = XLP_MSI_IRQ_START(node, fn);
				/* now handle it as any other interrupt */
			} else { /* If MSI is not set, must be INTX */
				base_irq += (node * XLP_IRQS_PER_NODE);
			}
			break;
		case XLP_PCIE_MSIX_IRQ(0, 0) ... XLP_PCIE_MSIX_IRQ(0, 31):
			fn = XLP_MSIX_TO_CTRL_FN(base_irq -
				XLP_PCIE_MSIX_IRQ(0, 0)); /* This _is_ correct because of((x >>3) &3) */
			/* this is a MSI-X. Find vectors set */
			bitmap = xlp_msix_status_clear(node, fn);
			/* recalculate base_irq for MSI */
			base_irq = XLP_MSIX_IRQ_START(node, fn);
			/* now handle it as any other interrupt */
			break;
#endif
		default:
			/* Except MSIX and MSI, all are treated INTX */
			base_irq += (node * XLP_IRQS_PER_NODE);
			break;
		}
		while (bitmap) {
			/* now that we have bitmap, serve all of them */
			idx = ffs(bitmap) - 1;	/* man ffs */
			bitmap &= ~(1 << idx);
			irq = base_irq + idx;
			do_nlm_common_IRQ(irq, pt_regs);
		}
	}

	}	// End of while (eirr)...
	return;
}

#if defined CONFIG_PCI_MSI
#ifdef arch_setup_msi_irqs
/*
 * Arch specific setup functions and helpers
 */
/*
 * Backend function that sets up MSI.
 * Called from arch_setup_msi_irqs()
 *
 * On XLP, we can have more than one MSI per device. But no device requests it
 * because of possible x86 influence (one MSI per device limitation).
 * We enforce this limitation as well.
 */
int xlp_setup_msi_irq(struct pci_dev *dev, struct msi_desc *desc, int nvec)
{
	__label__ setup_end;
	__label__ setup_fail;
	struct msi_msg msg;
	int ret, bit, base_msi;
	unsigned long flags;
	struct xlp_nodefn_struct nfn;

	if (xlp_ctrl_fn_from_dev(dev, &nfn) < 0) {
		return -EFAULT;
	}
	base_msi = XLP_MSI_IRQ_START(nfn.node, nfn.fn);
	if (nvec != 1) {
		return -EINVAL;
	}
	spin_lock_irqsave(&xlp_pic_lock, flags);
	ret = xlp_get_ctrl_intmode(nfn.node, nfn.fn);
	if ((ret == XLP_INTMODE_MSIX ) || (ret == XLP_INTMODE_INTX)) {
		ret = -EBUSY;
		goto setup_end;
	}
	/* search for the first unused bit in the bitmap.
	 * Please note that the usage is different from that of MSIX allocation
	 * where we have the luxury of 1 irt entry per MSIX. Here we have to
	 * multiplex in software */
	if (msi_vec[nfn.node][nfn.fn].bitmap == 0) {
		bit = 0;
	} else {
		bit = ffz(msi_vec[nfn.node][nfn.fn].bitmap);
	}
	if (bit > (XLP_MSI_PER_SLOT - 1)) {
		ret = -ENOSPC;
		goto setup_end;
	}
	msi_vec[nfn.node][nfn.fn].bitmap |= (1ULL << bit);
	msi_vec[nfn.node][nfn.fn].count++;
	base_msi += bit;
	set_irq_msi(base_msi, desc);
	ret = xlp_msi_compose_msg(dev, desc, base_msi, &msg);
	if (ret < 0) {
		goto setup_fail;
	}
	write_msi_msg(base_msi, &msg);
	ret = xlp_set_ctrl_intmode(nfn.node, nfn.fn, XLP_INTMODE_MSI);
	if (ret == 0) {	/* success */
		goto setup_end;	/* All done */
	}
setup_fail:
	msi_vec[nfn.node][nfn.fn].bitmap &= ~(1ULL << bit);
	msi_vec[nfn.node][nfn.fn].count--;
setup_end:
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return ret;
}

/*
 * MSI-X setup functions
 */

/*
 * Allocates a single MSI-X vector for msi_desc entry
 * @dev		: pci device
 * @desc	: msi_descriptor for this msi entry
 * @nvec	: outstanding number of interrupts required for this device
 */
int xlp_setup_msix_irq(struct pci_dev *dev, int nvec)
{
	__label__ fail_loop;
	__label__ setup_end;
	struct msi_msg msg;
	int old_mode, ret, idx, base_msix, bit, old_count;
	u32 old_bitmap;
	unsigned long flags;
	struct msi_desc *desc;
	struct xlp_nodefn_struct nfn;

	if (xlp_ctrl_fn_from_dev(dev, &nfn) < 0) {
		return -EFAULT;
	}
	base_msix = XLP_MSIX_IRQ_START(nfn.node, nfn.fn);
	spin_lock_irqsave(&xlp_pic_lock, flags);
	old_bitmap = msix_vec[nfn.node][nfn.fn].bitmap;
	old_count = msix_vec[nfn.node][nfn.fn].count;
	old_mode = xlp_get_ctrl_intmode(nfn.node, nfn.fn);
	if ((old_mode == XLP_INTMODE_MSI ) || (old_mode == XLP_INTMODE_INTX)) {
		ret = -EBUSY;
		goto setup_end;
	}
	ret = xlp_set_ctrl_intmode(nfn.node, nfn.fn, XLP_INTMODE_MSIX);
	if (ret < 0) {
		goto setup_end;
	}
	bit = 0, idx = 0;
	list_for_each_entry(desc, &dev->msi_list, list) {
		/* this loops exactly nvec times */
		if (msix_vec[nfn.node][nfn.fn].bitmap == 0) {
			bit = 0;
		} else {
			bit = ffz(msix_vec[nfn.node][nfn.fn].bitmap);
		}
		if (bit > (XLP_MSIX_PER_SLOT - 1)) {
			dev_err(&dev->dev, "No more vectors to allocate\n");
			/* if we allocated at least one vector, we need to
			 * return a positve value. Else return negative */
			if (idx > 0) {
				ret = idx;
			} else {
				ret = -ENOSPC;
			}
			goto setup_end;	/* could be a partial success */
		}
		/* We have allocated one bit, now get a vector for it */
		msix_vec[nfn.node][nfn.fn].bitmap |= (1ULL << bit);
		msix_vec[nfn.node][nfn.fn].count++;
		set_irq_msi(base_msix + bit, desc);
		ret = xlp_msi_compose_msg(dev, desc, base_msix + bit, &msg);
		if (ret < 0) {
			goto fail_loop;
		}
		write_msi_msg(base_msix + bit, &msg);
		idx++;
	}
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return 0;
fail_loop:
	msi_vec[nfn.node][nfn.fn].bitmap = old_bitmap;
	msi_vec[nfn.node][nfn.fn].count = old_count;
	xlp_set_ctrl_intmode(nfn.node, nfn.fn, old_mode);
setup_end:
	spin_unlock_irqrestore(&xlp_pic_lock, flags);
	return ret;
}

int arch_setup_msi_irqs(struct pci_dev *dev, int nvec, int type)
{
	struct msi_desc *entry;
	int ret;

	if (nvec == 0) {
		return -EINVAL;
	}
	/*
	 * For MSI, nvec = 1 is not an architectural limitation
	 * But at least in Linux 2.6.32, there is only one entry allocated
	 * in dev->msi_list. That means, we cannot setup more than one
	 * MSI, even if architecture allows it.  */
	if (type == PCI_CAP_ID_MSI) {
		if (nvec > 1) {
			return -EINVAL;
		}
		entry = list_first_entry(&dev->msi_list, struct msi_desc, list);
		ret = xlp_setup_msi_irq(dev, entry, nvec);
	} else if (type == PCI_CAP_ID_MSIX) {
	/* MSI-X has nvec entries allocated
	 * if nvec is greater than max number vectors that can be allocated,
	 * ret will be a positive value. For any failures, ret will be -ve */
		ret = xlp_setup_msix_irq(dev, nvec);
	} else {
		ret = -EINVAL;	/* To suppress compiler "ret unused" warning */
	}
	return ret;
}
EXPORT_SYMBOL(arch_setup_msi_irqs);
#endif
#endif

void __init init_nlm_common_irqs(void)
{
	int i;
	u64	mask = 0;
	u8 node;

	for_each_online_node(node) {
		for (i = 0; i < XLP_IRQ_MAX; i++) {	// IRQ : 0 - 167
			set_irq_chip((node * XLP_IRQS_PER_NODE) + i, &nlm_intx_pic);
		}
#ifdef CONFIG_PCI_MSI
		for (i = XLP_MSI_INDEX_START; i <= XLP_MSI_INDEX_END; i++) {
			set_irq_chip((node * XLP_IRQS_PER_NODE) + i, &nlm_msi_pic);
		}
		for (i = XLP_MSIX_INDEX_START; i <= XLP_MSIX_INDEX_END; i++) {
			set_irq_chip((node * XLP_IRQS_PER_NODE) + i, &nlm_msix_pic);
		}
#endif
	}

	for_each_online_node(node) {
#ifdef CONFIG_REMOTE_DEBUG	/* REMOVE on XLP TODO */
	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_REMOTE_DEBUG].chip = &nlm_common_rsvd_pic;
	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_REMOTE_DEBUG].action = nlm_common_rsvd_action;
	// xlp_irq_mask |= (1ULL << XLP_IRQ_REMOTE_DEBUG);
#endif
#ifdef CONFIG_SMP
	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_IPI_SMP_FUNCTION_RVEC].chip = &nlm_common_rsvd_pic;
	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_IPI_SMP_FUNCTION_RVEC].action = &nlm_common_rsvd_action;

	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_IPI_SMP_RESCHEDULE_RVEC].chip = &nlm_common_rsvd_pic;
	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_IPI_SMP_RESCHEDULE_RVEC].action = &nlm_common_rsvd_action;

#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY	/* REMOVE on XLP TODO */
	/* PR: New IPI added here for netrx balancing */
	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_IPI_NETRX].chip = &nlm_common_rsvd_pic;
	irq_desc[(node * XLP_IRQS_PER_NODE) + XLP_IRQ_IPI_NETRX].action = &nlm_common_rsvd_action;
	//xlp_irq_mask |= (1ULL << XLP_IRQ_IPI_NETRX);
#endif				/* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */

#endif
	}

	/* msgring interrupt */
	irq_desc[XLP_IRQ_MSGRING_RVEC].chip = &nlm_common_rsvd_pic;
	irq_desc[XLP_IRQ_MSGRING_RVEC].action = &nlm_common_rsvd_action;

	mask = (
#if defined CONFIG_XLP_REPLACE_R4K_TIMER
			(1ULL << XLP_PIC_SYSTIMER_RVEC) | /* PIC Systimer (0)*/
#else
			(1ULL << XLP_IRQ_TIMER_RVEC) |
#endif
			(1ULL << XLP_PIC_TIMERS_RVEC) |	/* Other PIC timers */
			(1ULL << 49) |	/* msg_idx */
			(0x3ULL << 48) |	/* msg_idx */
			(0xfULL << 32) |	/* pci msix */
			(0xfULL << 41) |	/* pci and msi */
			(0x1ULL << 58) |	/* nae */
			(0x3ULL << 24) |	/* usb */
			(0x3ULL	<< 17) |	/* uart */
			(0xfULL << 13) |	/* gpio */
			(0x1ULL << 31) |	/* SATA on xlp3xx */
			(0x1ULL << 30) |	/* SMSC  - xlp3xx */
			(0x1ULL << 26) |	/* MMC  */
#ifdef CONFIG_SMP
			(1ULL << XLP_IRQ_IPI_SMP_FUNCTION_RVEC) |
			(1ULL << XLP_IRQ_IPI_SMP_RESCHEDULE_RVEC) |
#endif
#ifdef CONFIG_OPROFILE
			(1ULL << XLP_IRQ_OPROFILE ) |
#endif
#ifdef CONFIG_KGDB
			(1ULL << XLP_IRQ_IPI_SMP_KGDB_RVEC) |
#endif
			(1ULL << XLP_IRQ_MSGRING_RVEC) |
#ifdef CONFIG_NLM_WATCHDOG
			(1ULL << XLP_PIC_WATCHDOG_TIMERS_RVEC) |
#endif
			(0xfULL << 20)		/* nor, nand, spi and mmc */
	       );

	/* set interrupt mask for non-zero cpus */
	mask |= read_64bit_cp0_eimr();
	xlp_irq_mask = mask;

	/* Moved watchdog enable to watchdog driver */
}

void __init arch_init_irq(void)
{
	extern int (*perf_irq)(void);

	perf_irq = xlp_perf_irq;

#ifdef CONFIG_KGDB
	if (kgdb_early_setup)
		return;
#endif

	/* Initialize the irq descriptors */
	init_nlm_common_irqs();

	write_64bit_cp0_eimr(xlp_irq_mask);

}
