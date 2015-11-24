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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/i2c/pcf857x.h>

static struct pcf857x_platform_data platform_data_pcf857x = {
	.gpio_base = 128,
	.n_latch = 0,
	.setup = NULL,
	.teardown = NULL,
	.context = NULL,
};

#if defined(CONFIG_I2C_NLM_XLP) && defined(CONFIG_I2C_BOARDINFO)
/* Moved nlm_eeprom stuff to new device driver */

/* TODO - Get I2C platform data from DTB */
/* Bus 0 on XMC, bus 1 on Eval boards */
static struct i2c_board_info xlp_i2c_device_info[] __initdata =
{
#ifdef CONFIG_RTC_DRV_DS1374
	{"ds1374",      0,  0x68,  NULL,  NULL,  0},
#endif

#if 0
	{"max6657",	    0,  0x4c,  NULL,  NULL,  0},
#else
	{"lm90",	0,  0x4c,  NULL,  NULL,  0},
	{"tmp421",      0,  0x4e,  NULL,  NULL,  0},
	{"tmp421",      0,  0x4f,  NULL,  NULL,  0},
	{"pcf8574a",     0,  0x3a,  &platform_data_pcf857x,  NULL,  0},
    {"pcf8574a",     0,  0x3e,  &platform_data_pcf857x,  NULL,  0},
#endif

#ifdef CONFIG_NLM_XLP_EEPROM
	{"nlm_eeprom",  0,  0x57,  NULL,  NULL,  0}
#endif
};

static int __init xlp_i2c_device_init(void)
{
#ifdef CONFIG_NLM_XMC_SUPPORT
	// XMC uses bus 0 for common devices
	return i2c_register_board_info(0, xlp_i2c_device_info, ARRAY_SIZE(xlp_i2c_device_info));
#else
	return i2c_register_board_info(1, xlp_i2c_device_info, ARRAY_SIZE(xlp_i2c_device_info));
#endif
}
arch_initcall(xlp_i2c_device_init);
#endif /* CONFIG_I2C_NLM_XLP  && CONFIG_I2C_BOARDINFO */

#ifdef CONFIG_SPI_XLP
#ifdef CONFIG_NLM_XMC_SUPPORT
	// XMC SPI bus goes to backplane and has no dedicated devices
static int __init xlp_spi_device_init(void) {;}
#else
static struct spi_board_info spsn_spi_board_info[] __initdata = {
	{
		.modalias = "m25p80",
		.max_speed_hz = 40000000,
		.bus_num = 0,
		.chip_select = 1
	},
	{
		.modalias = "mt29f",
		.max_speed_hz = 50000000,
		.bus_num = 0,
		.chip_select = 2
	},
};

static int __init xlp_spi_device_init(void)
{
	return spi_register_board_info(spsn_spi_board_info, ARRAY_SIZE(spsn_spi_board_info));
}
arch_initcall(xlp_spi_device_init);
#endif
#endif /* CONFIG_SPI_XLP */
