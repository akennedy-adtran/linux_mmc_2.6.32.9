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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/gpio.h>

#define XLP_GPIO1_BASE		32

spinlock_t xlp_gpio_lock  = SPIN_LOCK_UNLOCKED;

static uint32_t gpio_reg_read(int regidx)
{
	uint64_t mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_GPIO_FUNC);
	return nlm_hal_read_32bit_reg(mmio, regidx);
}

static void gpio_reg_write(int regidx, uint32_t val)
{
	uint64_t mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_GPIO_FUNC);
	nlm_hal_write_32bit_reg(mmio, regidx, val);
}

void gpio_set_value(unsigned gpio, int v)
{
	uint32_t val;
	unsigned long flags;

	spin_lock_irqsave(&xlp_gpio_lock, flags);

	if(gpio < XLP_GPIO1_BASE) {
		val = gpio_reg_read(XLP_GPIO_OUTPUT0);
		if(v)
			val |= (1 << gpio);
		else
			val &= ~(1 << gpio);
		gpio_reg_write(XLP_GPIO_OUTPUT0, val);
	} else {
		val = gpio_reg_read(XLP_GPIO_OUTPUT1);
		if(v)
			val |= (1 << (gpio - XLP_GPIO1_BASE));
		else
			val &= ~(1 << (gpio - XLP_GPIO1_BASE));
		gpio_reg_write(XLP_GPIO_OUTPUT1, val);
	}

	spin_unlock_irqrestore(&xlp_gpio_lock, flags);

	return;
}
EXPORT_SYMBOL(gpio_set_value);

int gpio_get_value(unsigned gpio)
{
	uint32_t mask;
	if(gpio < XLP_GPIO1_BASE) {
		mask = 1 << gpio;
		return ((gpio_reg_read(XLP_GPIO_INPUT0) & mask) >> gpio);
	} else {
		mask = 1 << (gpio - XLP_GPIO1_BASE);
		return ((gpio_reg_read(XLP_GPIO_INPUT1) & mask) >> gpio);
	}
}
EXPORT_SYMBOL(gpio_get_value);

static int xlp_gpio_direction_input(unsigned gpio)
{
	uint32_t val;
	unsigned long flags;

	if(gpio < XLP_GPIO1_BASE)
		val = gpio_reg_read(XLP_GPIO_OUTPUT_EN0);
	else
		val = gpio_reg_read(XLP_GPIO_OUTPUT_EN1);

	spin_lock_irqsave(&xlp_gpio_lock, flags);
	if(gpio < XLP_GPIO1_BASE)
		val &= ~(1 << gpio);
	else
		val &= ~(1 << (gpio-XLP_GPIO1_BASE));

	if(gpio < XLP_GPIO1_BASE)
		gpio_reg_write(XLP_GPIO_OUTPUT_EN0, val);
	else
		gpio_reg_write(XLP_GPIO_OUTPUT_EN1, val);
	spin_unlock_irqrestore(&xlp_gpio_lock, flags);
	return 0;
}

static int xlp_gpio_direction_output(unsigned gpio, int v)
{
	uint32_t val;
	unsigned long flags;

	spin_lock_irqsave(&xlp_gpio_lock, flags);

	if(gpio < XLP_GPIO1_BASE) {

		val = gpio_reg_read(XLP_GPIO_OUTPUT0);
		if(v)
			val |= (1 << gpio);
		else
			val &= ~(1 << gpio);
		gpio_reg_write(XLP_GPIO_OUTPUT0, val);

		val = gpio_reg_read(XLP_GPIO_OUTPUT_EN0);
		val |= (1 << gpio);
		gpio_reg_write(XLP_GPIO_OUTPUT_EN0, val);

	} else {

		val = gpio_reg_read(XLP_GPIO_OUTPUT1);
		if(v)
			val |= (1 << (gpio - XLP_GPIO1_BASE));
		else
			val &= ~(1 << (gpio - XLP_GPIO1_BASE));
		gpio_reg_write(XLP_GPIO_OUTPUT1, val);

		val = gpio_reg_read(XLP_GPIO_OUTPUT_EN1);
		val |= (1 << (gpio - XLP_GPIO1_BASE));
		gpio_reg_write(XLP_GPIO_OUTPUT_EN1, val);
	}

	spin_unlock_irqrestore(&xlp_gpio_lock, flags);

	return 0;
}
static int nlm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	return gpio_get_value(offset);
}

static void nlm_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	gpio_set_value(offset, value);
}

static int nlm_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return xlp_gpio_direction_input(offset);
}

static int nlm_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	return xlp_gpio_direction_output(offset, value);
}

#ifdef XLP_GPIO_DEBUG
static void gpio_dump_reg(void)
{
	int i;
	for(i = 0; i < 8; i++) {
		printk("0x%0x = 0x%8x\n", i, gpio_reg_read(i) );
	}
	for(i = 0x40; i < 0x54; i++) {
		printk("0x%0x = 0x%8x\n", i, gpio_reg_read(i) );
	}
}

void gpio_test(void)
{
	int i;

	printk("read gpio value\n");
	for(i = 0; i < 41; i++) {
		printk("gpio%d = 0x%8x\n", i, gpio_get_value(i) );
	}
	printk("set gpio value 1\n");

	for(i = 0; i < 41; i++) {
		gpio_direction_output(i, 1);
		printk("0x%0x = 0x%8x\n",  i < 32 ? 40 : 41, gpio_reg_read(i < 32 ? 40 : 41) );
		gpio_direction_input(i);
		printk("0x%0x = 0x%8x\n",  i < 32 ? 40 : 41, gpio_reg_read(i < 32 ? 40 : 41) );
		printk("gpio%d = 0x%8x\n", i, gpio_get_value(i));
	}

	printk("set gpio value 0\n");
	for(i = 0; i < 41; i++) {
		gpio_direction_output(i, 0);
		printk("0x%0x = 0x%8x\n",  i < 32 ? 40 : 41, gpio_reg_read(i < 32 ? 40 : 41) );
		gpio_direction_input(i);
		printk("0x%0x = 0x%8x\n",  i < 32 ? 40 : 41, gpio_reg_read(i < 32 ? 40 : 41) );
		printk("gpio%d = 0x%8x\n", i, gpio_get_value(i));
	}

}
#endif /* XLP_GPIO_DEBUG */

struct gpio_chip xlp_gpio_chip = {
	.label				= "xlp-gpio",
	.direction_input	= nlm_gpio_direction_input,
	.direction_output	= nlm_gpio_direction_output,
	.get				= nlm_gpio_get,
	.set				= nlm_gpio_set,
	.base				= 0,
	.ngpio				= XLP_GPIO_MAX,
};

static int __init xlp_gpiolib_init(void)
{
	gpiochip_add(&xlp_gpio_chip);
	return 0;
}

arch_initcall(xlp_gpiolib_init);
