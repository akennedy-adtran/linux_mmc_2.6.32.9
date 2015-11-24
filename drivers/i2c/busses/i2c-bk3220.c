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

/*
 *  i2c-palm-bk3220.c driver for the BK-3220 Host Adapter on the
 *  RMI Phoenix System.
 */

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-palm.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/i2c.h>

#undef 	DEBUG

#define ARIZONA_RTC_BUS 1
#define NETLOGIC_CPLD_PHYS_ADDR	0xbd850000

static wait_queue_head_t palm_wait;
__u32 * iobase_i2c_regs = 0;

__u32 * get_i2c_base(unsigned short bus)
{
	nlm_reg_t *mmio = 0;

	if (bus == 0)
		mmio = netlogic_io_mmio(NETLOGIC_IO_I2C_0_OFFSET);
	else
		mmio = netlogic_io_mmio(NETLOGIC_IO_I2C_1_OFFSET);

	return (__u32 *)mmio;
}

static void	
palm_bk3220_write(int reg, int val)
{
	/* Code to access the low-level
	 * I2C Block on the RMI Phoenix 
	 */		
	netlogic_write_reg(iobase_i2c_regs, reg, val);
}

static int
palm_bk3220_read(int reg)
{
  /* Code to access the low-level
   * I2C Block on the RMI Phoenix 
   */		
  __u32 retVal = netlogic_read_reg(iobase_i2c_regs, reg);
  return (int)retVal;
}

static struct i2c_algo_palm_data palm_bk3220_data = {
	.write		= palm_bk3220_write,
	.read		= palm_bk3220_read,
};

/* This is our i2c_adapter structure */
static struct i2c_adapter palm_bk3220_ops = {
	.owner          = THIS_MODULE,
	.id		= I2C_HW_PALM_BK3220,			
	.algo_data	= &palm_bk3220_data,
	.name		= "Palm Chip BK3220 Adapter",
};

static int __devinit palm_bk3220_probe(struct platform_device *pd)
{
	printk("Registering I2C Bus %d\n", pd->id);

    	iobase_i2c_regs = get_i2c_base(ARIZONA_RTC_BUS);

	init_waitqueue_head(&palm_wait);

	if (i2c_palm_add_bus(&palm_bk3220_ops) < 0) {
		printk(KERN_ERR "i2c-palm-bk3220: Failed to add i2c bus\n");
		goto out;
	}
	else {
		printk("i2c-palm-bk3220: Added I2C Bus.\n");
	}

	return 0;
out:
	return -ENODEV;
}

static int __devexit palm_bk3220_remove(struct platform_device *pd)
{
	return i2c_palm_del_bus(&palm_bk3220_ops);
}

static struct platform_driver nlm_bk3220_i2c_driver = {
	.probe  = palm_bk3220_probe,
	.remove = __devexit_p(palm_bk3220_remove),
	.driver = {
		.owner  = THIS_MODULE,
		.name   = NLM_XLR_I2C_BUS,
	},
};


static int __init palm_bk3220_init(void)
{
	return platform_driver_register(&nlm_bk3220_i2c_driver);
}

static void __exit palm_bk3220_exit(void)
{
	platform_driver_unregister(&nlm_bk3220_i2c_driver);
}



MODULE_AUTHOR("Netlogic Semiconductors.");
MODULE_DESCRIPTION("BK3220 I2C Host adapter driver");
MODULE_LICENSE("GPL");

module_init(palm_bk3220_init);
module_exit(palm_bk3220_exit);
