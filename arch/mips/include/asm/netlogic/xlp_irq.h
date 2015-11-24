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


#ifndef _ASM_NLM_XLP_IRQ_H
#define _ASM_NLM_XLP_IRQ_H

/* This is the de-facto rvec bit associated with count compare timer */
#define XLP_IRQ_TIMER_RVEC		7
#define XLP_PIC_SYSTIMER_RVEC		26
#define XLP_PIC_TIMERS_RVEC		10
#define XLP_PIC_WATCHDOG_TIMERS_RVEC    9

/* The following interrupt assignments (0-7) are special.
 * I need to find out what governs these assignments
 * XXX
 */

#define XLP_IRQ_DUMMY_UART_RVEC           2
#define XLP_IRQ_IPI_SMP_FUNCTION_RVEC     3
#define XLP_IRQ_IPI_SMP_RESCHEDULE_RVEC   4
 // #define XLP_IRQ_REMOTE_DEBUG_RVEC         5
#define XLP_IRQ_MSGRING_RVEC              5

#define XLP_IRQ_IPI_SMP_KGDB_RVEC	     50
#define XLP_IRQ_OPROFILE    	  6 
#define PIC_IRQ_IS_EDGE_TRIGGERED(x)	0

#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
#define XLP_IRQ_IPI_NETRX_RVEC		49
#define SMP_NETRX_IPI_RVEC			32
#endif /* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */

/* if you want some common #defines, please do it here */
#define NLM_IRQ_DUMMY_UART		XLP_IRQ_DUMMY_UART_RVEC
#define NLM_IRQ_IPI_SMP_FUNCTION	XLP_IRQ_IPI_SMP_FUNCTION_RVEC
#define NLM_IRQ_IPI_SMP_RESCHEDULE	XLP_IRQ_IPI_SMP_RESCHEDULE_RVEC
#define NLM_IRQ_REMOTE_DEBUG		XLP_IRQ_REMOTE_DEBUG_RVEC
#define NLM_IRQ_MSGRING			XLP_IRQ_MSGRING_RVEC
// #define NLM_IRQ_TIMER			XLP_IRQ_TIMER_RVEC
#define NLM_IRQ_IPI_SMP_KGDB		XLP_IRQ_IPI_SMP_KGDB_RVEC
// #define NLM_IRQ_IPI_OPROFILE		XLP_IRQ_IPI_OPROFILE_RVEC

/* These are flags required for SMP
 * Not XLP specifc -- possibly mips specific.
 * Need to move out XXX
 */
#define SMP_CALL_KGDB_HOOK_RVEC	8
#define SMP_OPROFILE_IPI_RVEC        16

#if defined __ASSEMBLY__
#define ASM_XLP_IO_PIC_OFFSET        0xffffffffb8004100 /* TODO: This will change in to function */
#define C_XLP_IO_PIC_OFFSET        0xffffffffb8004100ULL /* TODO: This will change in to function */
#define XLP_IO_PIC_OFFSET        C_XLP_IO_PIC_OFFSET

#else
#include <asm/netlogic/pic.h>
#include <asm/netlogic/xlp.h>
/*
 *     Register Offsets
 */
#define XLP_PIC_CTRL             0x40
#define XLP_PIC_BYTESWAP         0x42
#define XLP_PIC_STATUS           0x44
#define XLP_PIC_INT_TIMEOUT      0x46
#define XLP_PIC_ICI0_INT_TIMEOUT 0x48
#define XLP_PIC_ICI1_INT_TIMEOUT 0x4A
#define XLP_PIC_ICI2_INT_TIMEOUT 0x4C
#define XLP_PIC_IPI_CTL          0x4E
#define XLP_PIC_INT_ACK          0x50
#define XLP_PIC_INT_PENDING0     0x52
#define XLP_PIC_INT_PENDING1     0x54
#define XLP_PIC_INT_PENDING2     0x56

#define XLP_PIC_WD_MAXVAL(x)		(0x58 + (0xE) *(x))
#define XLP_PIC_WD_COUNT(x)		(0x5A + (0xE) *(x))
#define XLP_PIC_WD_THREN0(x)		(0x5C + (0xE) *(x))
#define XLP_PIC_WD_THREN1(x)		(0x5E + (0xE) *(x))
#define XLP_PIC_WD_BEATCMD(x)		(0x60 + (0xE) *(x))
#define XLP_PIC_WD_HB1(x)		(0x62 + (0xE) *(x))
#define XLP_PIC_WD_HB2(x)		(0x64 + (0xE) *(x))

#define XLP_PIC_SYSTIMER_MAXVAL(x)	(0x74 + ((x) << 1))
#define XLP_PIC_SYSTIMER_COUNT(x)	(0x84 + ((x) << 1))
#define XLP_PIC_INT_THREADEN01(x)	(0x94 + ((x) << 2))
#define XLP_PIC_INT_THREADEN23(x)	(0x96 + ((x) << 2))
#define XLP_PIC_IRT_ENTRY(x)		(0xB4 + ((x) << 1))
#define XLP_IRQ_RESERVED_MAX		8
//#define TIMER_CYCLES_MAXVAL        0xffffffffffffffffULL

// #define XLP_PIC_IRTREG_START 0xB4
#define XLP_ITE_ENTRIES		8
#define XLP_8XX_MAX_CPUS	(NLM_MAX_CPU_NODE * NLM_MAX_CPU_PER_NODE)

#define XLP_IRTENT_ENABLE	(1ULL << 31)
#define XLP_IRTENT_NMI		(1ULL << 29)
#define XLP_IRTENT_SCH_LCL	(1ULL << 28)
#define XLP_IRTENT_RVEC(x)	(((x) & 0x3fULL) << 20)
#define XLP_IRTENT_DT		(1ULL << 19)
#define XLP_IRTENT_DB(x)	((x & 7) << 16)
#define XLP_IRTENT_DTE(x)	((x) & 0xffff)

#define fdebug(fmt,arg...)\
	printk(KERN_DEBUG "%s:%d " fmt, __FILE__, __LINE__, ##arg)
#define __nlm_hal_set_irq_to_cpu	__nlm_hal_set_irt_to_cpu
void __nlm_hal_set_irt_to_cpu(int, int);
void __nlm_hal_release_irq(u8, int);
int __nlm_hal_request_irq(u8, int, int);
struct xlp_nodefn_struct {
	/* Structure to distinguish the controller function */
	u8 node;
	u8 fn;
};

/* #define XLP_PIT_TICK_RATE and extern nlm_hal_is_ref_clk_133MHz moved to HAL */

#define XLP_PIT_TIMER_MAX	(u64)(~0ULL)

extern u64 __nlh_pic_r64o(u8, u64);
extern void __nlh_pic_w64o(u8, u64, u64);
extern u64 __nlh_pci_r64o(u8, u64);
extern void __nlh_pci_w64o(u8, u64, u64);
extern void xlp_ack_pic(u8, int);
void nlm_hal_pic_send_ipi(int nmi, int vec, int node, int cpu);

#if !defined CONFIG_XLP_REPLACE_R4K_TIMER
void nlm_hal_pic_update_control(u64);
#endif

#define nlh_pic_r64r(nid, reg)	__nlh_pic_r64o(nid, (reg << 2))
#define nlh_pic_w64r(nid, reg, val) __nlh_pic_w64o(nid, (reg << 2), val)
/* Note PCI should take function into account, PIC has only one function */
#define nlh_pci_r32r(nid, fn, reg)	__nlh_pci_r32o(nid, fn, (reg << 2))
#define nlh_pci_w32r(nid, fn, reg, val) __nlh_pci_w32o(nid, fn, (reg << 2), val)

void xlp_ite_cpu_op(u8 node, u8 cpu, u8 ite, u8 bitval);
#define xlp_ite_cpu_set(node,cpu,ite) xlp_ite_cpu_op(node,cpu,ite,1)
#define xlp_ite_cpu_clear(node,cpu,ite) xlp_ite_cpu_op(node,cpu,ite,0)

int xlp_rvec_from_irq(int irq);

/* We define NR_IRQs to be 254, but IRT entries are 160 in size
 * Effectively, we cannot use anything more than 159 */
#define XLP_IRQS_PER_NODE	384
#define NR_IRQS			(XLP_IRQS_PER_NODE * NLM_MAX_CPU_NODE)
/* Maximum IRQ vector numbers supported by MIPS */
#define XLP_EIRR_SIZE		64
#define XLP_IRT_NUM	160
#define XLP_IRQ_MAX	168	/* 0-7 are reserved + 160 IRT entries */

/*
 *    IRT Map
 */
#define arch_setup_msi_irqs	arch_setup_msi_irqs /* defines arch. specific msi setup function */
#define xlp_irq_to_irt(x) (((x) % XLP_IRQS_PER_NODE) - XLP_IRQ_RESERVED_MAX)
#define xlp_irt_to_irq(n,x)\
	(((n) * XLP_IRQS_PER_NODE + XLP_IRQ_RESERVED_MAX + (x)))
#define XLP_WD_BASE(n)		(XLP_IRQ_RESERVED_MAX + (n * XLP_IRQS_PER_NODE))

#define XLP_WD_IRT_OFFSET(n)            (XLP_WD_BASE(n))
#define XLP_WD_IRQ(n,x)                 (XLP_WD_IRT_OFFSET(n) + (x))

#define XLP_WD_NMI_IRT_OFFSET(n)	(2 + XLP_WD_BASE(n))
#define XLP_WD_NMI_IRQ(n,x)		(XLP_WD_NMI_IRT_OFFSET(n) + (x))

#define XLP_TIMER_IRT_OFFSET(n)		(4 + XLP_WD_BASE(n))
#define XLP_TIMER_IRQ(n,x)		(XLP_TIMER_IRT_OFFSET(n) + (x))

#define XLP_MSGQ_IRT_OFFSET(n)		(12 + XLP_WD_BASE(n))
#define XLP_MSGQ_IRQ(n,x)		(XLP_MSGQ_IRT_OFFSET(n) + (x))

#define XLP_MSG_IRT_OFFSET(n)		(44 + XLP_WD_BASE(n))
#define XLP_MSG_IRQ(n,x)		(XLP_MSG_IRT_OFFSET(n) + (x))

#define XLP_PCIE_MSIX_IRT_OFFSET(n)	(46 + XLP_WD_BASE(n))
#define XLP_PCIE_MSIX_IRQ(n,x)		(XLP_PCIE_MSIX_IRT_OFFSET(n) + (x))

#define XLP_PCIE_LINK_IRT_OFFSET(n)	(78 + XLP_WD_BASE(n))
#define XLP_PCIE_LINK_IRQ(n,x)		(XLP_PCIE_LINK_IRT_OFFSET(n) + (x))

#define XLP_NAE_IRT_OFFSET(n)		(82 + XLP_WD_BASE(n))
#define XLP_XLP_NAE_IRQ(n,x)		(XLP_XLP_NAE_IRT_OFFSET(n) + (x))

#define XLP_POE_IRT_OFFSET(n)		(114 + XLP_WD_BASE(n))
#define XLP_POE_IRQ(n,x)		(XLP_POE_IRT_OFFSET(n) + (x))

#define XLP_USB_IRT_OFFSET(n)		(115 + XLP_WD_BASE(n))
#define XLP_USB_IRQ(n,x)		(XLP_USB_IRT_OFFSET(n) + (x))

#define XLP_DTR_IRT_OFFSET(n)		(121 + XLP_WD_BASE(n))
#define XLP_DTR_IRQ(n,x)		(XLP_DTR_IRT_OFFSET(n) + (x))

#define XLP_SAE_IRT_OFFSET(n)		(122 + XLP_WD_BASE(n))
#define XLP_SAE_IRQ(n,x)		(XLP_SAE_IRT_OFFSET(n) + (x))

#define XLP_RSA_IRT_OFFSET(n)		(123 + XLP_WD_BASE(n))
#define XLP_RSA_IRQ(n,x)		(XLP_RSA_IRT_OFFSET(n) + (x))

#define XLP_COMP_IRT_OFFSET(n)		(124 + XLP_WD_BASE(n))
#define XLP_COMP_IRQ(n,x)		(XLP_COMP_IRT_OFFSET(n) + (x))

#define XLP_FLASH_IRT_OFFSET(n)		(128 + XLP_WD_BASE(n))
#define XLP_FLASH_IRQ(n,x)		(XLP_FLASH_IRT_OFFSET(n) + (x))

#define	XLP_ICI_IRT_OFFSET(n)		(131 + XLP_WD_BASE(n))
#define XLP_ICI_IRQ(n,x)		(XLP_ICI_IRT_OFFSET(n) + (x))

#define	XLP_KBP_IRT_OFFSET(n)		(132 + XLP_WD_BASE(n))
#define XLP_KBP_IRQ(n,x)		(XLP_KBP_IRT_OFFSET(n) + (x))

#define XLP_UART_IRT_OFFSET(n)		(133 + XLP_WD_BASE(n))
#define XLP_UART_IRQ(n,x)		(XLP_UART_IRT_OFFSET(n) + (x))

#define XLP_I2C_IRT_OFFSET(n)		(135 + XLP_WD_BASE(n))
#define XLP_I2C_IRQ(n,x)		(XLP_I2C_IRT_OFFSET(n) + (x))

#define XLP_SM_IRT_OFFSET(n)		(137 + XLP_WD_BASE(n))
#define XLP_SM_IRQ(n,x)			(XLP_SM_IRT_OFFSET(n) + (x))

#define	XLP_JTAG_IRT_OFFSET(n)		(139 + XLP_WD_BASE(n))
#define XLP_JTAG_IRQ(n,x)		(XLP_JTAG_IRT_OFFSET(n) + (x))

#define XLP_PIC_IRT_OFFSET(n)		(140 + XLP_WD_BASE(n))
#define XLP_PIC_IRQ(n,x)		(XLP_PIC_IRT_OFFSET(n) + (x))

#define XLP_MIOCB_IRT_OFFSET(n)		(141 + XLP_WD_BASE(n))
#define XLP_MIOCB_IRQ(n,x)		(XLP_MIOCB_IRT_OFFSET(n) + (x))

#define XLP_TCU_IRT_OFFSET(n)		(142 + XLP_WD_BASE(n))
#define XLP_TCU_IRQ(n,x)		(XLP_TCU_IRT_OFFSET(n) + (x))

#define XLP_GCU_IRT_OFFSET(n)		(143 + XLP_WD_BASE(n))
#define XLP_GCU_IRQ(n,x)		(XLP_GCU_IRT_OFFSET(n) + (x))

#define XLP_DRAM_IRT_OFFSET(n)		(144 + XLP_WD_BASE(n))
#define XLP_DRAM_IRQ(n,x)		(XLP_DRAM_IRT_OFFSET(n) + (x))

#define XLP_GPIO_IRT_OFFSET(n)		(146 + XLP_WD_BASE(n))
#define XLP_GPIO_IRQ(n,x)		(XLP_GPIO_IRT_OFFSET(n) + (x))

#define XLP_NOR_IRT_OFFSET(n)		(150 + XLP_WD_BASE(n))
#define XLP_NOR_IRQ(n,x)		(XLP_NOR_IRT_OFFSET(n) + (x))

#define XLP_NAND_IRT_OFFSET(n)		(151 + XLP_WD_BASE(n))
#define XLP_NAND_IRQ(n,x)		(XLP_NAND_IRT_OFFSET(n) + (x))

#define XLP_SPI_IRT_OFFSET(n)		(152 + XLP_WD_BASE(n))
#define XLP_SPI_IRQ(n,x)		(XLP_SPI_IRT_OFFSET(n) + (x))

#define XLP_MMC_IRT_OFFSET(n)		(153 + XLP_WD_BASE(n))
#define XLP_MMC_IRQ(n,x)		(XLP_MMC_IRT_OFFSET(n) + (x))

/* The following are the values supported per slot. A slot can have a device or
 * a bridge, but only this much MSI/MSI-X can be alloted on that slot
 * */

#define XLP_INTX_TO_CTRL_FN(irq)\
({\
 u32 lirq = irq % XLP_IRQS_PER_NODE;\
 ((lirq - XLP_PCIE_LINK_IRQ(0,0)) & 0x3);\
})
#define XLP_IRQ_TO_NODE(x)		((u8)((x) / XLP_IRQS_PER_NODE))

#define XLP_BDF_BASE(b,d,f)	(0x18000000 + ((b) << 20) + ((d) << 15) + ((f) << 12))
#define XLP_MAX_SLOTS		4	/* Only 4 slots now */

#ifdef CONFIG_XLP_MSI_ADDRESSES
#define XLP_MSI_ADDR		0xFEE00000
#endif
#define XLP_MSI_ADDR_SIZE	0x00002000
#define XLP_MSIX_ADDR_SIZE	0x00004000
#ifdef CONFIG_PCI_MSI
 /* We are using IRQ 256 - 383 for MSI */
#define XLP_MSI_MM_CAP		5	/* Multiple message capability */
#define XLP_MSI_PER_SLOT	(1 << XLP_MSI_MM_CAP)
#define XLP_MSI_IRQ_OFFSET	256	/* Note IRQ not IRT */
#define XLP_MSI_IRQ_START(n,fn)	((XLP_IRQS_PER_NODE * (n)) + \
			XLP_MSI_IRQ_OFFSET + (fn) * XLP_MSI_PER_SLOT)
#define XLP_MSI_INDEX_START	XLP_MSI_IRQ_START(0, 0)
#define XLP_MSI_INDEX_END	(XLP_MSI_IRQ_START(0, XLP_MAX_SLOTS) - 1) /* 128 Vectors */
#define XLP_MSI_TO_CTRL_FN(msi) (((msi % XLP_IRQS_PER_NODE) >> (XLP_MSI_MM_CAP)) & 3)
#define XLP_MSI_TO_NODE(x)	XLP_IRQ_TO_NODE(x)
 /* We are using IRQ 192 - 223 for MSI-X */
#define XLP_MSIX_PER_SLOT	8
#define XLP_MSIX_IRQ_OFFSET	192
#define XLP_MSIX_TO_CTRL_FN(msix) (((msix % XLP_IRQS_PER_NODE) >> 3) & 3)
#define XLP_MSIX_TO_NODE(x)	XLP_IRQ_TO_NODE(x)
#define XLP_MSIX_IRQ_START(n, fn)	((XLP_IRQS_PER_NODE * n) + XLP_MSIX_IRQ_OFFSET + (fn) * XLP_MSIX_PER_SLOT)
#define XLP_MSIX_INDEX_START	XLP_MSIX_IRQ_START(0, 0)
#define XLP_MSIX_INDEX_END	(XLP_MSIX_IRQ_START(0, XLP_MAX_SLOTS) - 1)// 31 vectors

#endif
enum xlp_intmode {
	XLP_INTMODE_NONE = 0,
	XLP_INTMODE_INTX = 1,
	XLP_INTMODE_MSI = 2,
	XLP_INTMODE_MSIX = 4,
};

#define xlp_incr_ctrl_intmode(n, fn, mode) xlp_ctrl_intmode_add(n, fn, mode, 1)
#define xlp_decr_ctrl_intmode(n, fn, mode) xlp_ctrl_intmode_add(n, fn, mode, -1)
#define xlp_soc_pcidev_to_node(dev) ((u8)(PCI_SLOT(dev->devfn)/8))
#define nlm_xlp_request_irq(node, irt)	((irt) + XLP_WD_BASE(node))

#if defined CONFIG_PCIEPORTBUS
#define PORT_TYPE_MASK			0xf
#define PCIE_CAPABILITIES_REG		0x2
#endif

#endif		/* __ASSEMBLY__ */

#endif
