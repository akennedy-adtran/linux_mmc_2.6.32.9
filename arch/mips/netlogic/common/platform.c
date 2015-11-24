/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * Copyright 2004, Matt Porter <mporter@kernel.crashing.org>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <asm/netlogic/nlm_srio.h>

static u64 xls_usb_dmamask = ~(u32) 0;

static struct platform_device xls_usb_ehci_device = {
	.name = "ehci-xls",
	.id = 0,
	.num_resources = 2,
	.dev = {
		.dma_mask = &xls_usb_dmamask,
		.coherent_dma_mask = 0xffffffff,
		},
	.resource = (struct resource[]){
					{
					 .start = 0x1EF24000,
					 .end = (0x1EF24000 + 0x400 - 0x01),
					 .flags = IORESOURCE_MEM,
					 },
					{
					 .start = 39,
					 .end = 39,
					 .flags = IORESOURCE_IRQ,
					 },
					},
};

static struct platform_device xls_usb_ohci_device_0 = {
	.name = "ohci-xls-0",
	.id = 1,
	.num_resources = 2,
	.dev = {
		.dma_mask = &xls_usb_dmamask,
		.coherent_dma_mask = 0xffffffff,
		},
	.resource = (struct resource[]){
					{
					 .start = 0x1EF24400,
					 .end = (0x1EF24400 + 0x400 - 0x01),
					 .flags = IORESOURCE_MEM,
					 },
					{
					 .start = 39,
					 .end = 39,
					 .flags = IORESOURCE_IRQ,
					 },
					},
};

static struct platform_device xls_usb_ohci_device_1 = {
	.name = "ohci-xls-1",
	.id = 2,
	.num_resources = 2,
	.dev = {
		.dma_mask = &xls_usb_dmamask,
		.coherent_dma_mask = 0xffffffff,
		},
	.resource = (struct resource[]){
					{
					 .start = 0x1EF24800,
					 .end = (0x1EF24800 + 0x400 - 0x01),
					 .flags = IORESOURCE_MEM,
					 },
					{
					 .start = 39,
					 .end = 39,
					 .flags = IORESOURCE_IRQ,
					 },
					},
};

static struct platform_device *xls_platform_devices[] __initdata = {
	&xls_usb_ehci_device,
	&xls_usb_ohci_device_0,
	&xls_usb_ohci_device_1,
};

int xls_platform_init(void)
{
    return platform_add_devices(xls_platform_devices, ARRAY_SIZE(xls_platform_devices));
}

arch_initcall(xls_platform_init);

#ifdef CONFIG_RAPIDIO
void platform_rio_init(void)
{
	nlm_rio_setup();
}
#endif				/* CONFIG_RAPIDIO */
