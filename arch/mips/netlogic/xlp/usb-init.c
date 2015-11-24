/*
 * Copyright (c) 2003-2014 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the Broadcom
 * license below:
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
 */

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/xlp_usb.h>

int xlp_usb_dev;

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_EHCI_HCD)
static void __init nlm_usb_intr_en(int node, int ctrl_no)
{
	uint32_t val;
#ifdef CONFIG_USB_OHCI_HCD
	val = USB_CTRL_INTERRUPT_EN;
	if((ctrl_no != 0) && (ctrl_no != 3)) {	// OHCI controllers (functions 1,2,4,5)
		val |= USB_OHCI_INTERRUPT_EN;
//		val |= USB_OHCI_INTERRUPT1_EN | USB_OHCI_INTERRUPT12_EN;	// Keyboard interrupts
	}
#else
	if((ctrl_no == 0) || (ctrl_no == 3))	// EHCI controllers (functions 0, 3)
		val = USB_CTRL_INTERRUPT_EN;
	else
		val = 0;
#endif
	usb_reg_write(node, ctrl_no, USB_INT_EN, val);
}

static void __init nlm_usb_hw_reset(int node, int ctrl_no)
{
	uint32_t val;

	/* Reset USB phys */
	val = usb_reg_read(node, ctrl_no, USB_PHY_0);
	if (ctrl_no == 0)
		val &= ~USB_PHY_RESET;
	val &= ~(USB_PHY_PORT_RESET_0 | USB_PHY_PORT_RESET_1);
	usb_reg_write(node, ctrl_no, USB_PHY_0, val);

	mdelay(100);
	/* Release controller reset and start clock */
	val = usb_reg_read(node, ctrl_no, USB_CTL_0);
	val &= ~(USB_CONTROLLER_RESET);
	val |= USB_OHCI_START_CLK;
	usb_reg_write(node, ctrl_no, USB_CTL_0, val);
}
#endif

#ifdef CONFIG_USB_XHCI_HCD
static void __init xlp_usb3_hw_start_controller(int node, int ctrl_no)
{
	uint32_t val;
	uint64_t corebase = usb_reg_read(node, ctrl_no, XLP_USB_PCIE_MBAR) & ~0xf;

	/* Set frequency This does nothing - default value has 6'h27 << 20 set already */
	val = usb_reg_read(node, ctrl_no, XLP_USB3_PHY_LOS_LEV);
	val &= ~(0x3f << 20);
	val |= (0x27 << 20);
	usb_reg_write(node, ctrl_no, XLP_USB3_PHY_LOS_LEV, val);

	/* Set VBus external valid */
	val = usb_reg_read(node, ctrl_no, XLP_USB3_REF_CLK);
	val |= (1<<30);
	usb_reg_write(node, ctrl_no, XLP_USB3_REF_CLK, val);

	/* Clear phy reset */
	val = usb_reg_read(node, ctrl_no, XLP_USB3_PHY_TEST);
	val &= 0x3e;
	usb_reg_write(node, ctrl_no, XLP_USB3_PHY_TEST, val);

	/* Release reset to controller IP, set number of ports = 1 + 1 */
	val = 0x2e02203;
	usb_reg_write(node, ctrl_no, XLP_USB3_CTL, val);

	/* Enable interrupts */
	val = 0x1;
	usb_reg_write(node, ctrl_no, XLP_USB3_INT_MASK, val);

	/* clear all interrupts */
	usb_reg_write(node, ctrl_no, XLP_USB3_INT, 0xffffffff);

	udelay(2000);

	if(ctrl_no == 1) {

		/* USB3 phy delay P0 to P1/P2/P3 request */
		val = 0x240002;
		nlh_write_cfg_reg32(corebase + XLP_USB3_PIPE3CTL, val);

		/* Set host mode */
		val = nlh_read_cfg_reg32(corebase + XLP_USB3_GCTL);
		val &= ~(0x3<<12);
		val |= (1<<12);
		nlh_write_cfg_reg32(corebase + XLP_USB3_GCTL, val);
		udelay(1000);

		/* Disable USB2 phy suspend */
		val = nlh_read_cfg_reg32(corebase + XLP_USB3_PHY_CFG);
		val &= ~(1<<6);
		nlh_write_cfg_reg32(corebase + XLP_USB3_PHY_CFG, val);
		udelay(1000);

		/* Disable USB3 phy suspend - this does nothing */
		val = nlh_read_cfg_reg32(corebase + XLP_USB3_PIPE3CTL);
		val &= ~(1<<17);
		nlh_write_cfg_reg32(corebase + XLP_USB3_PIPE3CTL, val);
	}
}
#endif /* CONFIG_USB_XHCI_HCD */

static int __init nlm_platform_usb_init(void)
{
	int i, n;

	if (is_nlm_xlp2xx()) {
#ifdef CONFIG_USB_XHCI_HCD
		pr_info("Initializing XLP1xx/2xx USB Interfaces\n");
		xlp_usb_dev = XLP_PCIE_USB3_DEV;
		for(i=1; i < 4; i++)
			xlp_usb3_hw_start_controller(NODE_0, i);
#endif
	} else {
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_EHCI_HCD)
		pr_info("Initializing XLP USB Interfaces\n");
		xlp_usb_dev = XLP_PCIE_USB_DEV;
		for(n = 0; n < num_possible_nodes(); n++) {
			if(!node_online(n))
				continue;

			/* Controller reset */
			nlm_usb_hw_reset(n, 0);
			nlm_usb_hw_reset(n, 3);

			/* Interrupts */
			for(i=0; i < 6; i++)
				nlm_usb_intr_en(n, i);
		}
#endif
	}

	return 0;
}
arch_initcall(nlm_platform_usb_init);

static u64 xlp_usb_dmamask = ~(u32)0;

/* Fixup the IRQ for USB devices which is exist on XLP SOC PCIE bus. Kernel config must
 * include CONFIG_PCI_QUIRKS for this to be applied.
 */
static void __devinit quirk_nlm_usb_irq(struct pci_dev *dev)
{
	dev->dev.dma_mask = &xlp_usb_dmamask;
	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	switch (dev->devfn) {
#ifdef CONFIG_USB_OHCI_HCD
	case 0x11:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_OHCI_0);
		break;
	case 0x12:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_OHCI_1);
		break;
	case 0x14:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_OHCI_2);
		break;
	case 0x15:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_OHCI_3);
		break;
#endif
#ifdef CONFIG_USB_EHCI_HCD
	case 0x10:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_EHCI_0);
		break;
	case 0x13:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_EHCI_1);
		break;
#endif
#ifdef CONFIG_USB_XHCI_HCD
	case 0x21:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_XHCI_0);
		break;
	case 0x22:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_XHCI_1);
		break;
	case 0x23:
		dev->irq = xlp_irt_to_irq(0, PIC_IRT_XHCI_2);
		break;
#endif
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_NETL_VENDOR, XLP_DEVID_EHCI, quirk_nlm_usb_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_NETL_VENDOR, XLP_DEVID_OHCI, quirk_nlm_usb_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_NETL_VENDOR, XLP2XX_DEVID_XHCI, quirk_nlm_usb_irq);
