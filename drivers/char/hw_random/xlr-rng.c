/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
 * RNG driver for Netlogic XLR CPU
 *
 * derived from
 *
 * Hardware driver for the AMD 768 Random Number Generator (RNG)
 * (c) Copyright 2001 Red Hat Inc <alan@redhat.com>
 *
 * derived from
 *
 * Hardware driver for Intel i810 Random Number Generator (RNG)
 * Copyright 2000,2001 Jeff Garzik <jgarzik@pobox.com>
 * Copyright 2000,2001 Philipp Rumpf <prumpf@mandrakesoft.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hw_random.h>
#include <asm/io.h>
#include <asm/netlogic/gpio.h>


#define PFX	KBUILD_MODNAME ": "


static nlm_reg_t   *mmio;
static int xlr_data_present(struct hwrng *rng, int wait)
{
	return 1;
}

static int xlr_data_read(struct hwrng *rng, u32 *data)
{
	*data = netlogic_read_reg(mmio, NETLOGIC_GPIO_RNG_REG);
	return 4;
}

static int xlr_init(struct hwrng *rng)
{
	mmio = netlogic_io_mmio(NETLOGIC_IO_GPIO_OFFSET);
	return 0;

}

static void xlr_cleanup(struct hwrng *rng)
{
}


static struct hwrng xlr_rng = {
	.name		= "xlr",
	.init		= xlr_init,
	.cleanup	= xlr_cleanup,
	.data_present	= xlr_data_present,
	.data_read	= xlr_data_read,
};


static int __init mod_init(void)
{
	int err = -ENODEV;
	err = hwrng_register(&xlr_rng);
	if (err) {
		printk(KERN_ERR PFX "RNG registering failed (%d)\n",
		       err);
	}
	printk(KERN_INFO "Netlogic RNG registered\n");
	return err;
}

static void __exit mod_exit(void)
{
	hwrng_unregister(&xlr_rng);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR("The Linux Kernel team");
MODULE_DESCRIPTION("H/W RNG driver for Netlogic XLR chipsets");
MODULE_LICENSE("GPL");
