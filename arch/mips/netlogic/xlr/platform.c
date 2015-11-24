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
#include <linux/i2c.h>
#include <asm/netlogic/nlm_srio.h>
#include <asm/netlogic/i2c.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>

int __initdata boot_noi2c;
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

static struct i2c_board_info nlm_i2c_info1[] __initdata = {
	/* All XLR boards have this RTC Chip */
	{
		I2C_BOARD_INFO("ds1374", 0x68),
	},
};

static struct platform_device nlm_xlr_i2c_1 = {
	.name           = NLM_XLR_I2C_BUS,
	.id             = 1,
};


static int __init nlm_i2c_init(void)
{
	int err=0;

	if(boot_noi2c == 0) {
		platform_device_register(&nlm_xlr_i2c_1);

		err = i2c_register_board_info(1, nlm_i2c_info1,
				ARRAY_SIZE(nlm_i2c_info1));
		if (err < 0)
			printk(KERN_ERR
			"nlm-i2c: cannot register board I2C devices\n");
	} 
	return err;
}


static int __init xlr_noi2c_setup(char *str)
{
	boot_noi2c = 1;
	return 1;
}

unsigned int nlm_xlr_uart_in(struct uart_port *p, int offset)
{
	nlm_reg_t *mmio;
	unsigned int value;

	/* XLR uart does not need any mapping of regs */
	mmio = (nlm_reg_t *)(p->membase + (offset << p->regshift));
	value = netlogic_read_reg(mmio, 0);

	/* See XLR/XLS errata */
	if (offset == UART_MSR)
		value ^= 0xF0;
	else if (offset == UART_MCR)
		value ^= 0x3;

	return value;

}

void nlm_xlr_uart_out(struct uart_port *p, int offset, int value)
{
	nlm_reg_t *mmio;
	/* XLR uart does not need any mapping of regs */
	mmio = (nlm_reg_t *)(p->membase + (offset << p->regshift));

	/* See XLR/XLS errata */
	if (offset == UART_MSR)
		value ^= 0xF0;
	else if (offset == UART_MCR)
		value ^= 0x3;

	netlogic_write_reg(mmio, 0, value);
}


#define PORT(_base, _irq)                               \
	{                                               \
		.iobase		= _base,                \
		.membase	= (void __iomem *)_base,\
		.mapbase	= _base,                \
		.irq		= _irq,                 \
		.regshift	= 2,                    \
		.iotype		= UPIO_NLM,         	\
		.flags		= (UPF_SKIP_TEST|UPF_FIXED_TYPE|UPF_BOOT_AUTOCONF),\
		.uartclk        = 66666666,		\
		.type		= PORT_16550A,		\
		.serial_in	= nlm_xlr_uart_in,	\
		.serial_out	= nlm_xlr_uart_out,	\
	}

static struct plat_serial8250_port xlr_uart_data[] = {
        PORT(DEFAULT_NETLOGIC_IO_BASE+NETLOGIC_IO_UART_0_OFFSET, PIC_UART_0_IRQ),
        PORT(DEFAULT_NETLOGIC_IO_BASE+NETLOGIC_IO_UART_1_OFFSET, PIC_UART_1_IRQ),
	{},
};

static struct platform_device uart_device = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_NETL_XLR,
	.dev = {
		.platform_data = xlr_uart_data,
	},
};

static int __init nlm_uart_init(void)
{
	return platform_device_register(&uart_device);
}

__setup("xlr_noi2c", xlr_noi2c_setup);
arch_initcall(nlm_i2c_init);
arch_initcall(nlm_uart_init);



