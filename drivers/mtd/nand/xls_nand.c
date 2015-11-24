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

#include <linux/genhd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/netlogic/sim.h>

#include "xls_nand.h"

static u8 hwctl;
static void __iomem *xls_io_base;
static int xls_nand_phys_base = 0xBD800000;

/* register offset */
#define FLASHIO	 	0xBD800000		/* Flash I/O */

/*
 * MTD structure for xls
 */
static struct mtd_info *xls_mtd= NULL;

/*
 * Define partitions for flash device
 */
#define DEFAULT_NUM_PARTITIONS 2

#define CLE_REG (0xbef19240 + (long)(((struct nand_chip *)(mtd->priv))->priv) * 4)
#define ALE_REG (0xbef19280 + (long)(((struct nand_chip *)(mtd->priv))->priv) * 4)
#define RW_OFFSET  0xbd800000 

#define WRITE_NAND_CLE(command)(nand_write_cmd((long)(int)CLE_REG, command))
#define WRITE_NAND_ALE(address)(nand_write_addr((long)(int)ALE_REG, address))
#define WRITE_NAND_ARRAY(data,n) (nand_write_multi((long)(int)RW_OFFSET, data, n))
#define READ_NAND_BYTE(data)(nand_read_byte((long)(int)RW_OFFSET, &(data)))
#define READ_NAND_ARRAY(data,n) (nand_read_multi((long)(int)RW_OFFSET, data, n))


static int nr_partitions;
static struct mtd_partition xls_nand_default_partition_info[] = {
	{
	.name = "Root Filesystem",
	.offset = 64 * 64 * 2048, /* 54M@8M */
	.size = 432 * 64 * 2048,
	},
	{
	.name = "Home Filesystem",
	.offset = MTDPART_OFS_APPEND , /* Rest@62M */
	.size = MTDPART_SIZ_FULL ,
	},
};

/*
 *	hardware specific access to control-lines
 *	In case of xls, we remember the access, and accordingly do read/write of
 *	ale/cle or IO.
 */
static void xls_nand_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	unsigned char bits = ctrl & 0x07;

		if (bits & NAND_CLE)
			WRITE_NAND_CLE(cmd);
		else if (bits & NAND_ALE)
			WRITE_NAND_ALE(cmd);
}

#define STATUS_BIT_0 0x01
#define STATUS_BIT_6 0x40
#define MAX_READ_STATUS_COUNT 100000
#define NAND_CMD_READ_STATUS 0x70

int nand_read_status(struct mtd_info *mtd)
{
	int status_count;
	unsigned char status;

	WRITE_NAND_CLE(NAND_CMD_READ_STATUS);		
	status_count = 0;

	while(status_count < MAX_READ_STATUS_COUNT)
	{
		/* Read status byte */
		READ_NAND_BYTE(status);
		/* Check status */
		if((status& STATUS_BIT_6) == STATUS_BIT_6)  /* if 1, device is ready */
		{
			if((status& STATUS_BIT_0) == 0)	/* if 0, the last operation was succesful */
				return 1;
			else
				return 0;
		}			
		status_count++;
	}

	return 0;
}

static unsigned char xls_nand_read_byte(struct mtd_info *mtd)
{
	unsigned char x;
	READ_NAND_BYTE(x);
	return x;
}

static void xls_nand_write_buf(struct mtd_info *mtd, const unsigned char *buf, int len)
{
	int i;
    unsigned char *tmp = (unsigned char *)buf;

	for (i = 0; i < len; i++) {
		WRITE_NAND_ARRAY((void *)&tmp[i],1);
	}
}

static void xls_nand_read_buf(struct mtd_info *mtd, unsigned char *buf, int len)
{
	READ_NAND_ARRAY(buf,len);
}

static int xls_nand_verify_buf(struct mtd_info *mtd, const unsigned char *buf, int len)
{
	int i;
	unsigned char temp_byte;

	for (i = 0; i < len; i++) {
		READ_NAND_BYTE(temp_byte);
		if (buf[i] != temp_byte)
			return i;
	}

	return 0;
}

static int xls_nand_dev_ready(struct mtd_info *mtd)
{
	return nand_read_status(mtd);
}

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

/*
 * Main initialization routine
 */
int __init
xls_nand_init(void)
{
	struct nand_chip *this;
	struct mtd_partition* xls_partition_info;
	int err = 0;
	uint32_t nand_gpio_check;
	uint32_t read_cs_base;
	int nand_chip_select;
	
	hwctl = 0;

	if (!is_xls())
		return -ENODEV;


	/* nand is present */
	nand_gpio_check = *(uint32_t *)((long)(int)(0xbef00000 + 0x18000 + 84));
	if ((nand_gpio_check >> 16) & 0x1)
		nand_chip_select = 0;
	else
		nand_chip_select = 2;
	
	read_cs_base = *(uint32_t *)((long)(int)(0xbef19000 + nand_chip_select * 4));

	read_cs_base = read_cs_base & 0xffff;

	if (read_cs_base  != 384) { /* This checks if flash bar is mapped at
				       0xbd800000 */
		printk ("Burn bootloader version > 1.4.2 to use nand flash\n");
		return -ENODEV;
	}
	/* Allocate memory for MTD device structure and private data */
	xls_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip),
				GFP_KERNEL);
	if (!xls_mtd) {
		printk ("Unable to allocate xls NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* map physical adress */
	xls_io_base = ioremap((long)xls_nand_phys_base, 0x1000);
	if(!xls_io_base){
		printk("ioremap to access XLS NAND chip failed\n");
		kfree(xls_mtd);
		return -EIO;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&xls_mtd[1]);

	/* Initialize structures */
	memset((char *) xls_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	xls_mtd->priv = this;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = (char *)(long)(int)FLASHIO;
	this->IO_ADDR_W = (char *)(long)(int)FLASHIO;
	/* Set address of hardware control function */
	this->cmd_ctrl = xls_nand_hwcontrol;
	this->dev_ready = xls_nand_dev_ready;
	this->read_byte  = xls_nand_read_byte;
	this->write_buf  = xls_nand_write_buf;
	this->read_buf   = xls_nand_read_buf;
	this->verify_buf = xls_nand_verify_buf;
	
	/* 15 us command delay time */
	this->chip_delay = 15;
	this->ecc.mode = NAND_ECC_SOFT;
	this->priv = (void *)(unsigned long)nand_chip_select;

	/* Scan to find existence of the device */
	err=nand_scan(xls_mtd,1);
	if (err) {
		iounmap(xls_io_base);
		kfree(xls_mtd);
		return err;
	}

	xls_mtd->name = "xls-nand";
	/* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
	nr_partitions = parse_mtd_partitions(xls_mtd, part_probes,
						&xls_partition_info, 0);
#endif 

	if (nr_partitions <= 0) {
		nr_partitions = DEFAULT_NUM_PARTITIONS;
		xls_partition_info = xls_nand_default_partition_info;
	}

	add_mtd_partitions(xls_mtd, xls_partition_info, nr_partitions);

	return 0;
}
module_init(xls_nand_init);

#ifdef MODULE
static void __exit xls_nand_cleanup(void)
{
	struct nand_chip *this = (struct nand_chip *) &xls_mtd[1];

	/* Release resources, unregister device */
	nand_release(xls_mtd);

	iounmap(xls_io_base);

	/* Free the MTD device structure */
	kfree(xls_mtd);
}
module_exit(xls_nand_cleanup);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Raza Micronelectronics <sandip@razamicro.com>");
MODULE_DESCRIPTION("Device specific logic for NAND flash on XLS Series");
