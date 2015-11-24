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
 * Flash memory access on XLR evaluation boards
 *
 * (C) 2008, 2009  RMI Corp <sandip@rmicorp.com>
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/netlogic/sim.h>

#ifdef 	DEBUG_RW
#define	DBG(x...)	printk(x)
#else
#define	DBG(x...)
#endif

#define BOARD_MAP_NAME "SPS Flash"
#define BOARD_FLASH_SIZE 0x01000000 /* 16MB */
#define BOARD_FLASH_BASE 0x1c000000 
#define BOARD_FLASH_WIDTH 2 /* 16-bits */

static struct map_info sps_map = {
	.name =	BOARD_MAP_NAME,
};

static struct mtd_partition sps_partitions[] = {
        {
                .name = "User FS",
                .offset = 0x800000, // Upto 8 MB is used by bootloader.
		.size = MTDPART_SIZ_FULL ,
        }
};

static struct mtd_info *mymtd;

int __init sps_mtd_init(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	unsigned long window_addr;
	unsigned long window_size;

	if (xlr_board_atx_viii()){
		return -ENODEV;
	}
	/* Default flash buswidth */
	sps_map.bankwidth = BOARD_FLASH_WIDTH;

	window_addr = BOARD_FLASH_BASE;
	window_size = BOARD_FLASH_SIZE;

	/*
	 * Static partition definition selection
	 */
	parts = sps_partitions;
	nb_parts = ARRAY_SIZE(sps_partitions);
	sps_map.size = window_size;

	/*
	 * Now let's probe for the actual flash.  Do it here since
	 * specific machine settings might have been set above.
	 */
	printk(KERN_NOTICE BOARD_MAP_NAME ": probing %d-bit flash bus\n",
			sps_map.bankwidth*8);
	sps_map.virt = ioremap(window_addr, window_size);
	mymtd = do_map_probe("cfi_probe", &sps_map);
	if (!mymtd) {
		iounmap(sps_map.virt);
		return -ENXIO;
	}
	mymtd->owner = THIS_MODULE;

	add_mtd_partitions(mymtd, parts, nb_parts);
	return 0;
}

static void __exit sps_mtd_cleanup(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
		iounmap(sps_map.virt);
	}
}

module_init(sps_mtd_init);
module_exit(sps_mtd_cleanup);

MODULE_AUTHOR("Sandip Matte, RMI Corporation");
MODULE_DESCRIPTION(BOARD_MAP_NAME " MTD driver");
MODULE_LICENSE("GPL");
