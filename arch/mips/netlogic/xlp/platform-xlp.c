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


#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/pci.h>
#include <linux/serial_reg.h>

#include <asm/time.h>
#include <asm/netlogic/hal/nlm_hal_macros.h>
#include <asm/netlogic/hal/nlm_hal_pic.h>

#define XLP_SOC_PCI_DRIVER "XLP SoC Driver"

#define PCI_NETL_VENDOR             0x184E

#define PCI_DEVID_DEFAULT           0x1000
#define PCI_DEVID_BASE              (PCI_DEVID_DEFAULT + 0x00)

#define PCI_DEVID_OFF_SBU           0x01
#define PCI_DEVID_OFF_ICI           0x02
#define PCI_DEVID_OFF_PIC           0x03
#define PCI_DEVID_OFF_PCIE          0x04
#define PCI_DEVID_OFF_CAM           0x05
#define PCI_DEVID_OFF_USB           0x06
#define PCI_DEVID_OFF_EHCI          0x07
#define PCI_DEVID_OFF_OHCI          0x08
#define PCI_DEVID_OFF_NET           0x09

#define PCI_DEVID_OFF_POE           0x0A
#define PCI_DEVID_OFF_MSG           0x0B
#define PCI_DEVID_OFF_GDX           0x0C
#define PCI_DEVID_OFF_SEC           0x0D
#define PCI_DEVID_OFF_RSA           0x0E
#define PCI_DEVID_OFF_CMP           0x0F

#define PCI_DEVID_OFF_UART          0x10
#define PCI_DEVID_OFF_I2C           0x11
#define PCI_DEVID_OFF_GPIO          0x12
#define PCI_DEVID_OFF_SYS           0x13
#define PCI_DEVID_OFF_JTAG          0x14
#define PCI_DEVID_OFF_NOR           0x15
#define PCI_DEVID_OFF_NAND          0x16
#define PCI_DEVID_OFF_SPI           0x17
#define PCI_DEVID_OFF_MMC           0x18
#define PCI_DEVID_OFF_LAST          0x19

#define PCI_MAX_DEVICES             (PCI_DEVID_OFF_LAST + 1)

#define UART_CLK 133333333

#define MAX_NUM_UARTS 3

struct soc_dev {
	const char *name;
	void (*probe)(struct pci_dev *pdev, unsigned long pci_cfg_dev_base);
};

const char *pci_cfg_dev_regs[16] = {
	[0] = "Dev Info 0",
	[1] = "Dev Info 1",
	[2] = "Dev Info 2",
	[3] = "Dev Info 3",
	[4] = "Dev Info 4",
	[5] = "Dev Info 5",
	[6] = "Dev Info 6",
	[7] = "Dev Info 7",
	[8] = "Dev Scratch 0",
	[9] = "Dev Scratch 1",
	[10] = "Dev Scratch 2",
	[11] = "Dev Scratch 3",
	[12] = "Dev Msg Stn Info",
	[13] = "Dev IRT Info",
	[14] = "uCode Engine Info",
	[15] = "SBB BW Weight Entry Info"
};

static struct pci_device_id soc_pci_table[PCI_MAX_DEVICES] __devinitdata;

static struct plat_serial8250_port uart_ports[MAX_NUM_UARTS] = {
	[MAX_NUM_UARTS - 1] = { .flags = 0 } /* tenetlogicnating condition */
};

static void nlmc_hal_pci_cfg(int devfn, unsigned long pci_cfg_dev_base)
{
	int i = 0;
	unsigned int *cfg_base = (unsigned int *)pci_cfg_dev_base;

	cfg_base += (0xC0 >> 2);
	for (i = 0; i < 16; i++) {
		unsigned int value = cfg_base[i];

		if (!value) continue;
/* 		printk("[%s]: reg[%s] = %08x\n", __FUNCTION__, pci_cfg_dev_regs[i], value); */
	}
}

static void xlp_sbu_probe(struct pci_dev *pdev, unsigned long pci_cfg_dev_base)
{
}

static void xlp_pic_probe(struct pci_dev *pdev, unsigned long pci_cfg_dev_base)
{
}

static void xlp_cms_probe(struct pci_dev *pdev, unsigned long pci_cfg_dev_base)
{
}

static void xlp_sys_probe(struct pci_dev *pdev, unsigned long pci_cfg_dev_base)
{
}

static void xlp_default_probe(struct pci_dev *pdev, unsigned long pci_cfg_dev_base)
{
}

unsigned int xlp_uart_in(struct uart_port *p, int offset)
{
	nlm_reg_t *mmio;
	unsigned int value;

	/* XLP uart does not need any mapping of regs */
	offset = offset << p->regshift;
	mmio = (nlm_reg_t *)(p->membase + offset);
	value = netlogic_read_reg(mmio, 0);

	return value;

}

void xlp_uart_out(struct uart_port *p, int offset, int value)
{
	nlm_reg_t *mmio;

	/* XLP uart does not need any mapping of regs */
	offset = offset << p->regshift;
	mmio = (nlm_reg_t *)(p->membase + offset);
	netlogic_write_reg(mmio, 0, value);
}

static void xlp_uart_probe(struct pci_dev *pdev, unsigned long pci_cfg_dev_base)
{
	static atomic_t num_uarts = ATOMIC_INIT(0);
	int instance = atomic_inc_return(&num_uarts) - 1;

	if (instance < 0 || instance >= (MAX_NUM_UARTS - 1)) {
		printk("Request for Invalid uart port_%d\n", instance);
		return ;
	}

 	if (!instance) {
		struct platform_device *uart = 0;

		/* If this is the first instance of registration, create the platform device */

		uart = platform_device_alloc("serial8250", PLAT8250_DEV_PLATFORM);
		if (!uart) {
			printk("Unable to allocate memory for UART platform device!\n");
			return;
		}

		uart->dev.platform_data = &uart_ports[0];

		if (platform_device_add(uart)) {
			printk("Unable to register uart plaform device!\n");
			return;
		}
		printk("Platform registered UART device\n");
	}

	uart_ports[instance].mapbase       = pci_cfg_dev_base + 0x100; /* skip PCI CFG header */
	uart_ports[instance].membase       = (void __iomem *)uart_ports[instance].mapbase;
	uart_ports[instance].irq           = PIC_UART_0_IRQ + instance;

	uart_ports[instance].uartclk       = UART_CLK;
	uart_ports[instance].iotype        = UPIO_NLM;
	uart_ports[instance].flags         = UPF_SKIP_TEST|UPF_FIXED_TYPE|UPF_BOOT_AUTOCONF;
	uart_ports[instance].type          = PORT_16550A;
	uart_ports[instance].regshift      = 2;
	uart_ports[instance].serial_in      = xlp_uart_in;
	uart_ports[instance].serial_out      = xlp_uart_out;

	printk("Platform added UART port_%d (irq=%d, @%lx)\n", instance,
	       uart_ports[instance].irq, (unsigned long)uart_ports[instance].mapbase);

	nlmc_hal_pci_cfg(pdev->devfn, pci_cfg_dev_base);
}

static struct soc_dev soc_devices[PCI_MAX_DEVICES] = {
	[PCI_DEVID_OFF_SBU]  = {.name = "South Bridge",                      .probe = xlp_sbu_probe},
	[PCI_DEVID_OFF_PIC]  = {.name = "Programmable Interrupt Controller", .probe = xlp_pic_probe},
	[PCI_DEVID_OFF_MSG]  = {.name = "Central Messaging Station",         .probe = xlp_cms_probe},
	[PCI_DEVID_OFF_UART] = {.name = "UART",                              .probe = xlp_uart_probe},
	[PCI_DEVID_OFF_SYS]  = {.name = "Sys",                               .probe = xlp_sys_probe},
};

static int __devinit soc_device_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	unsigned long mmio_paddr = 0, mmio_size = 0;
	void *mmio_vaddr = 0;
	int irq;
	int result;
	int base_class, sub_class, prog_int;
	int dev = (pdev->devfn >> 3) & 0x1f;

	int dev_off = pdev->device - PCI_DEVID_BASE;
	void (*probe)(struct pci_dev *, unsigned long);

	int fn = (pdev->devfn & 0x7);
	const int num_fns = 8;
	const int pci_cfg_size = 0x1000;
	unsigned long pci_cfg_dev_base = (unsigned long)(xlp_io_base + (dev * num_fns * pci_cfg_size)
							 + (fn * pci_cfg_size));

	base_class = (pdev->class >> 16) & 0xff;
	sub_class = (pdev->class >> 8) & 0xff;
	prog_int = (pdev->class >> 0) & 0xff;

	result = pci_enable_device(pdev);
	if (result) return result;

	mmio_paddr = pci_resource_start(pdev, 0);
	mmio_size = pci_resource_len(pdev, 0);
	irq = pdev->irq;

	if (mmio_paddr) {
		result = pci_request_regions(pdev, XLP_SOC_PCI_DRIVER);
		if (result) {
			printk("[%s]: unable to pci_request_regions\n", __FUNCTION__);
			return result;
		}

		mmio_vaddr = ioremap(mmio_paddr, mmio_size);
		if (!mmio_vaddr) {
			printk("[%s]: unable to ioremap\n", __FUNCTION__);
			pci_release_regions(pdev);
			return 1;
		}
	}

	probe = soc_devices[dev_off].probe;
	if (!probe) {
		printk("No probe handler found for XLP device \"%s\"\n", soc_devices[dev_off].name);
	}
	else {
		printk("Invoking probe handler for XLP device \"%s\"\n", soc_devices[dev_off].name);
		probe(pdev, pci_cfg_dev_base);
	}

	return 0;
}

static struct pci_driver soc_driver = {
	.name             = XLP_SOC_PCI_DRIVER,
	.id_table         = soc_pci_table,
	.probe            = soc_device_probe,
};

static int __init soc_device_init(void)
{
	int i = 0;

	for (i = 0; i < PCI_MAX_DEVICES; i++) {
		soc_pci_table[i].vendor        = PCI_NETL_VENDOR;
		soc_pci_table[i].device        = PCI_DEVID_BASE + i;
		soc_pci_table[i].subvendor     = PCI_ANY_ID;
		soc_pci_table[i].subdevice        = PCI_ANY_ID;
		soc_pci_table[i].driver_data   = 0;

		if (soc_devices[i].name) continue;
		soc_devices[i].name = "unrecognized";
		soc_devices[i].probe = xlp_default_probe;
	}
	soc_pci_table[PCI_DEVID_OFF_LAST].vendor        = 0;
	soc_pci_table[PCI_DEVID_OFF_LAST].device        = 0;
	soc_pci_table[PCI_DEVID_OFF_LAST].subvendor     = 0;
	soc_pci_table[PCI_DEVID_OFF_LAST].subdevice     = 0;
	soc_pci_table[PCI_DEVID_OFF_LAST].driver_data   = 0;

	return pci_register_driver(&soc_driver);
}

static void __init soc_device_exit(void)
{
	pci_unregister_driver(&soc_driver);
}

module_init(soc_device_init);
module_exit(soc_device_exit);

MODULE_DEVICE_TABLE(pci, soc_pci_table);
