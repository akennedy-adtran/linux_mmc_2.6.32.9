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

#ifndef __XLP_USB_H
#define __XLP_USB_H

#define USB_CTL_0                      0x41
#define USB_PHY_0                      0x4A
#define USB_PHY_RESET                  0x01
#define USB_PHY_PORT_RESET_0           0x10
#define USB_PHY_PORT_RESET_1           0x20
#define USB_CONTROLLER_RESET           0x01
#define USB_OHCI_START_CLK             0x04
#define USB_INT_STATUS                 0x4E
#define USB_INT_EN                     0x4F
#define USB_PHY_INTERRUPT_EN           0x01
#define USB_OHCI_INTERRUPT_EN          0x02
#define USB_OHCI_INTERRUPT1_EN         0x04
#define USB_OHCI_INTERRUPT12_EN        0x08
#define USB_CTRL_INTERRUPT_EN          0x10

#define PIC_IRT_EHCI_0                  115
#define PIC_IRT_OHCI_0                  116
#define PIC_IRT_OHCI_1                  117
#define PIC_IRT_EHCI_1                  118
#define PIC_IRT_OHCI_2                  119
#define PIC_IRT_OHCI_3                  120

#define PIC_IRT_XHCI_0                  115
#define PIC_IRT_XHCI_1                  116
#define PIC_IRT_XHCI_2                  117

extern int xlp_usb_dev;

static __inline__ uint32_t usb_reg_read(int node, int func, int regidx)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, xlp_usb_dev, func);
	return nlm_hal_read_32bit_reg(mmio, regidx);
}

static __inline__ void usb_reg_write(int node, int func, int regidx, uint32_t val)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, xlp_usb_dev, func);
	nlm_hal_write_32bit_reg(mmio, regidx, val);
}

#endif //__XLP_USB_H
