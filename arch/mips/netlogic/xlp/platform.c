/*-
 * Copyright (c) 2003-2015 Broadcom Corporation
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
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/pci.h>
#include <linux/serial_reg.h>
#include <linux/spinlock.h>

#include <asm/time.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/xlp.h>

#define XLP_SOC_PCI_DRIVER 	"XLP SoC Driver"

#define XLP_MAX_DEVICE			8
#define XLP_MAX_FUNC			8
#define MAX_NUM_UARTS			4
#define XLP_UART_PORTIO_OFFSET	0x1000

static struct plat_serial8250_port xlp_uart_port[MAX_NUM_UARTS];

static u64 xlp_dev_dmamask = DMA_BIT_MASK(32);

struct dev2drv {
	uint32_t 	devid;
	uint8_t 	drvname[16];
	uint8_t 	len;
	uint8_t 	id;
};

#ifdef CONFIG_SERIAL_8250
unsigned int xlp_uart_in(struct uart_port *p, int offset) {

	nlm_reg_t *mmio;
	unsigned int value;

	/* XLP uart does not need any mapping of regs 
	 */
	offset = offset << p->regshift;
	mmio = (nlm_reg_t *)(p->membase + offset);
	value = netlogic_read_reg(mmio, 0);

	return value;
}

void xlp_uart_out(struct uart_port *p, int offset, int value)
{
	nlm_reg_t *mmio;

	offset = offset << p->regshift;
	mmio = (nlm_reg_t *)(p->membase + offset);
	netlogic_write_reg(mmio, 0, value);
}

static void __init xlp_init_uart(int port_id)
{
        xlp_uart_port[port_id].mapbase       = DEFAULT_NETLOGIC_IO_BASE 
						+ NETLOGIC_IO_UART_0_OFFSET + port_id * XLP_UART_PORTIO_OFFSET;
        xlp_uart_port[port_id].membase       = (void __iomem *)xlp_uart_port[port_id].mapbase;
        xlp_uart_port[port_id].irq           = XLP_UART_IRQ(0, port_id);
		xlp_uart_port[port_id].uartclk       = nlm_hal_get_ref_clk_freq();
        xlp_uart_port[port_id].iotype        = UPIO_NLM;
        xlp_uart_port[port_id].flags         = UPF_SKIP_TEST|UPF_FIXED_TYPE|UPF_BOOT_AUTOCONF;
        xlp_uart_port[port_id].type          = PORT_16550A;
        xlp_uart_port[port_id].regshift      = 2;
        xlp_uart_port[port_id].serial_in     = xlp_uart_in;
        xlp_uart_port[port_id].serial_out    = xlp_uart_out;
}
#endif /* CONFIG_SERIAL_8250 */

/* The ID column in the table below is incremented every
 * time a device matches the PCI-e devid. struct fields:
 *  PCI-e devid,		name,	  name_len, id */
static struct dev2drv dev2drv_table[] __initdata = {
	{XLP_DEVID_UART,	"serial8250",	11,  0},
#ifdef CONFIG_SPI_XLP							// TODO convert to PCI driver
	{XLP_DEVID_SPI,		"spi-xlp",		8,   0},
#endif
	{XLP_DEVID_NOR,		"nor-xlp",		8,   0},
	{XLP_DEVID_NAND,	"nand-xlp",		9,   0}
};

/* Find index of entry in the table that matches the PCI-e device ID */
static int __init get_dev2drv(uint32_t devid)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(dev2drv_table); i++) {	
		if(devid == dev2drv_table[i].devid)
			return i;
	}
	return -1;
}

/* TODO - Read fdt to get this info, add BCM65500 platform driver detection */
static int __init xlp_find_pci_dev(void)
{
	uint16_t i, j, id, maxdevice=0;
	int idx;
	uint64_t mmio;
	uint32_t val, devid, irt, irq;
	struct platform_device* pplatdev;
	struct resource pres[2] = { {0} };
	int total=num_possible_nodes();

	for(i=0; i<total; i++) {
		j=node_online(i);
		if(!j)     continue; 
		maxdevice += XLP_MAX_DEVICE;
	}	

	printk(KERN_DEBUG "XLP platform devices:\n");

	for (i=0; i<maxdevice; i++) {

		for (j=0; j<XLP_MAX_FUNC; j++) {

			mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, i, j);
			val  = nlm_hal_read_32bit_reg(mmio, 0);

			if(val == 0xFFFFFFFF)
				continue;		// No PCI device

			devid = (val & 0xFFFF0000) >> 16;
//			printk("PCI-e Device ID 0x%04X found at bus 0, device %d, function %d\n", devid, i, j);

			idx = get_dev2drv(devid);
			if(idx < 0)
				continue;		// Not found in table

			/* Register NAND only for other nodes.
			 * Remove if condition when other devices are supported on other nodes as well.
			 * */
			if(!((i>8 && (devid == XLP_DEVID_NAND) ) || (i<8)))
				continue;

			id = dev2drv_table[idx].id++;

			/* Funny UART exception */
			if (devid == XLP_DEVID_UART)
				id += PLAT8250_DEV_PLATFORM;

			pplatdev = platform_device_alloc((const char*)dev2drv_table[idx].drvname, id);
			if (!pplatdev) {
				printk(KERN_WARNING "platform_device_alloc failed\n");
				continue;
			}

			if(devid == XLP_DEVID_UART) {
				pplatdev->dev.platform_data = &xlp_uart_port[id];
				xlp_init_uart(id);
			}

			/* Set-up I/O Memory and IRQ resources */
			irt = (nlm_hal_read_32bit_reg(mmio, (XLP_PCIE_DEV_IRT_INFO >> 2)) & 0xFFFF);
			irq = xlp_irt_to_irq(NODE_0, irt);
			pres[0].start = irq;
			pres[0].end   = irq;
			pres[0].flags = IORESOURCE_IRQ;

			pres[1].start = mmio;
			pres[1].end   = mmio + 0xFFF;
			pres[1].flags = IORESOURCE_MEM;

			platform_device_add_resources(pplatdev, pres, 2);
			pplatdev->dev.dma_mask = &xlp_dev_dmamask;
			pplatdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
			platform_device_add(pplatdev);

			printk(KERN_DEBUG "%12s.%d (PCIe 00:%02u.%u, DevID = 0x%04X), IRQ = %3d, "
					"Mem = 0x%llX-0x%llX,\n",
					dev2drv_table[idx].drvname, id, i, j, devid, irq,
					mmio, mmio + 0xFFF);
		}
	}

	return 0;
}

static int __init platform_devinit(void)
{
	xlp_find_pci_dev();
	return 0;
}

#ifdef CONFIG_RAPIDIO
void __init platform_rio_init(void)
{
	extern int bcm_rio_module_init();
	bcm_rio_module_init();
}
#endif

static void __exit platform_devexit(void)
{
	return;
}

module_init(platform_devinit);
module_exit(platform_devexit);
