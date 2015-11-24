/*-
 * Copyright 2013 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
spinand_lld.c

Copyright (c) 2009-2010 Micron Technology, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/delay.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/spinand.h>

#include <linux/spi/spi.h>

#define mu_spi_nand_driver_version "Beagle-MTD_01.00_Linux2.6.33_20100507"
#define SPI_NAND_MICRON_DRIVER_KEY 0x1233567

/****************************************************************************/

/**
   OOB area specification layout:  Total 32 available free bytes.
*/

#ifdef CONFIG_MTD_SPINAND_ONDIEECC 
int enable_hw_ecc = 0;
int enable_read_hw_ecc = 0;
#endif
static struct nand_ecclayout spinand_oob_64 = {
	.eccbytes = 24,
	.eccpos = {
		   1, 2, 3, 4, 5, 6,
		   17, 18, 19, 20, 21, 22,
		   33, 34, 35, 36, 37, 38,
		   49, 50, 51, 52, 53, 54, },
	.oobavail = 32,
	.oobfree = {
		{.offset = 8,
		 .length = 8},
		{.offset = 24,
		 .length = 8},
		{.offset = 40,
		 .length = 8},
		{.offset = 56,
		 .length = 8}, }
};

/**
 * spinand_cmd - to process a command to send to the SPI Nand
 * 
 * Description:
 *    Set up the command buffer to send to the SPI controller.
 *    The command buffer has to initized to 0
 */

int spinand_cmd(struct spi_device *spi, struct spinand_cmd *cmd)
{
	int	ret;
	struct spi_message	message;
	struct spi_transfer	x[4];
	u8 dummy = 0xff;
	char cmd_buff[16];


	spi_message_init(&message);
	memset(x, 0, sizeof x);
	
	x[0].len = 1;
	x[0].tx_buf = &cmd->cmd;
	
	memcpy(cmd_buff,  &cmd->cmd, 1);
	
	if (cmd->n_addr)
	{
		x[1].len = cmd->n_addr;
		x[1].tx_buf = cmd->addr;
		x[0].len = 1 +  x[1].len;
		memcpy(&cmd_buff[1],  cmd->addr, x[1].len);
	}

	x[0].tx_buf = &cmd_buff[0];
	if(cmd->n_tx || cmd->n_rx)
		x[0].spi_cont_cmd = 1;
	spi_message_add_tail(&x[0], &message);
	

	if (cmd->n_dummy)
	{
		x[2].len = cmd->n_dummy;
		x[2].tx_buf = &dummy;
		x[2].spi_cont_cmd = 1;
		spi_message_add_tail(&x[2], &message);		
	}

	if (cmd->n_tx)
	{
		x[3].len = cmd->n_tx;
		x[3].tx_buf = cmd->tx_buf;
		spi_message_add_tail(&x[3], &message);		
	}

	if (cmd->n_rx)
	{
		x[3].len = cmd->n_rx;
		x[3].rx_buf = cmd->rx_buf;
		spi_message_add_tail(&x[3], &message);	
	}
	
	ret = spi_sync(spi, &message);

	return ret; 
}

/**
 * spinand_read_id- Read SPI Nand ID
 * 
 * Description:
 *    Read ID: read two ID bytes from the SPI Nand device
 */
static int spinand_read_id(struct spi_device *spi_nand, u8 *id)
{
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	u8 nand_id[3];

	
	cmd.cmd = CMD_READ_ID;
	cmd.n_rx = 3;
	cmd.rx_buf = &nand_id[0];
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d reading id\n",
				(int) retval);
		return retval;
	}
	id[0] = nand_id[1];
        id[1] = nand_id[2];	

	return 0;	
}

/**
 * spinand_lock_block- send write register 0x1f command to the Nand device
 * 
 * Description:
 *    After power up, all the Nand blocks are locked.  This function allows
 *    one to unlock the blocks, and so it can be wriiten or erased.
 */
static int spinand_lock_block(struct spi_device *spi_nand, struct spinand_info *info, u8 lock)
{
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_WRITE_REG;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_BLOCK_LOCK;
	cmd.n_tx = 1;
	cmd.tx_buf = &lock;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d lock block\n",
				(int) retval);
		return retval;
	}
	
	return 0;	
}

/**
 * spinand_read_status- send command 0xf to the SPI Nand status register
 * 
 * Description:
 *    After read, write, or erase, the Nand device is expected to set the busy status.
 *    This function is to allow reading the status of the command: read, write, and erase.
 *    Once the status turns to be ready, the other status bits also are valid status bits.
 */
static int spinand_read_status(struct spi_device *spi_nand, struct spinand_info *info, uint8_t *status)
{
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_READ_REG;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_STATUS;
	cmd.n_rx = 1;
	cmd.rx_buf = status;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d reading status register\n",
				(int) retval);
		return retval;
	}

	return 0;
}

/**
 * spinand_get_otp- send command 0xf to read the SPI Nand OTP register
 * 
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spinand_get_otp(struct spi_device *spi_nand, struct spinand_info *info, u8* otp)
{
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_READ_REG;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_OTP;
	cmd.n_rx = 1;
	cmd.rx_buf = otp;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d get otp\n",
				(int) retval);
		return retval;
	}
	
	return 0;	
}


/**
 * spinand_set_otp- send command 0x1f to write the SPI Nand OTP register
 * 
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spinand_set_otp(struct spi_device *spi_nand, struct spinand_info *info, u8* otp)
{
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_WRITE_REG;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_OTP;
	cmd.n_tx = 1;
	cmd.tx_buf = otp;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d set otp\n",
				(int) retval);
		return retval;
	}
	
	return 0;
}

#ifdef CONFIG_MTD_SPINAND_ONDIEECC
/**
 * spinand_enable_ecc- send command 0x1f to write the SPI Nand OTP register
 * 
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spinand_enable_ecc(struct spi_device *spi_nand, struct spinand_info *info)
{
	ssize_t retval;
	u8 otp = 0;
	
	retval = spinand_get_otp(spi_nand, info, &otp);

	if ((otp & OTP_ECC_MASK) == OTP_ECC_MASK)
	{
		return 0;
	}
	else
	{
		otp |= OTP_ECC_MASK;
		retval = spinand_set_otp(spi_nand, info, &otp);
		retval = spinand_get_otp(spi_nand, info, &otp);
		return retval;
	}
}
#endif

static int spinand_disable_ecc(struct spi_device *spi_nand, struct spinand_info *info)
{
	ssize_t retval;
	u8 otp = 0;
	
	retval = spinand_get_otp(spi_nand, info, &otp);


	if ((otp & OTP_ECC_MASK) == OTP_ECC_MASK)
	{
		otp &= ~OTP_ECC_MASK;
		retval = spinand_set_otp(spi_nand, info, &otp);
		retval = spinand_get_otp(spi_nand, info, &otp);
		return retval;
	}
	else
	{
		return 0;
	}
}

/**
 * spinand_write_enable- send command 0x06 to enable write or erase the Nand cells
 * 
 * Description:
 *   Before write and erase the Nand cells, the write enable has to be set.
 *   After the write or erase, the write enable bit is automatically cleared( status register bit 2 )
 *   Set the bit 2 of the status register has the same effect
 */
static int spinand_write_enable(struct spi_device *spi_nand, struct spinand_info *info)
{
	struct spinand_cmd cmd = {0};

	cmd.cmd = CMD_WR_ENABLE;

	return spinand_cmd(spi_nand, &cmd);
}

static int spinand_read_page_to_cache(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id)
{
	struct spinand_cmd cmd = {0};
	u16 row;

	row = page_id;

	cmd.cmd = CMD_READ;
	cmd.n_addr = 3;
	cmd.addr[1] = (u8)((row&0xff00)>>8);
	cmd.addr[2] = (u8)(row&0x00ff);

	return spinand_cmd(spi_nand, &cmd);
}

/**
 * spinand_read_from_cache- send command 0x03 to read out the data from the cache register( 2112 bytes max )
 * 
 * Description:
 *   The read can specify 1 to 2112 bytes of data read at the coresponded locations.
 *   No tRd delay.
 */
static int spinand_read_from_cache(struct spi_device *spi_nand, struct spinand_info *info, u16 byte_id, u16 len, u8* rbuf)
{
	struct spinand_cmd cmd = {0};
	u16 column;

	column = byte_id;

	cmd.cmd = CMD_READ_RDM;
	cmd.n_addr = 3;
	cmd.addr[0] = (u8)((column&0xff00)>>8);
	cmd.addr[1] = (u8)(column&0x00ff);
	cmd.addr[2] = (u8)(0xff);
	cmd.n_dummy = 0;
	cmd.n_rx = len;
	cmd.rx_buf = rbuf;
	
	return spinand_cmd(spi_nand, &cmd);	
}

/**
 * spinand_read_page-to read a page with:
 * @page_id: the physical page number
 * @offset:  the location from 0 to 2111
 * @len:     number of bytes to read
 * @rbuf:    read buffer to hold @len bytes
 *
 * Description:
 *   The read icludes two commands to the Nand: 0x13 and 0x03 commands
 *   Poll to read status to wait for tRD time.
 */
static int spinand_read_page(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id, u16 offset, u16 len, u8* rbuf)
{
	ssize_t retval;
	u8 status = 0;
	
#ifdef CONFIG_MTD_SPINAND_ONDIEECC 
	if(enable_read_hw_ecc)	
		retval = spinand_enable_ecc(spi_nand, info);
#endif	
	retval = spinand_read_page_to_cache(spi_nand, info, page_id);

	while (1)
	{
		retval = spinand_read_status(spi_nand, info, &status);
		if (retval<0) {
			dev_err(&spi_nand->dev, "error %d reading status register\n",
					(int) retval);
			return retval;
		}

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{
			if ((status & STATUS_ECC_MASK) == STATUS_ECC_ERROR)
			{
				dev_err(&spi_nand->dev, "ecc error, page=%d\n", page_id);
				return 0;
			}
			break;
		}
	}

	retval = spinand_read_from_cache(spi_nand, info, offset, len, rbuf);

#ifdef CONFIG_MTD_SPINAND_ONDIEECC 
	if(enable_read_hw_ecc)	{	
		retval = spinand_disable_ecc(spi_nand, info);
		enable_read_hw_ecc = 0;
	}
#endif	
	return 0;
		
}

/**
 * spinand_program_data_to_cache--to write a page to cache with:
 * @byte_id: the location to write to the cache
 * @len:     number of bytes to write
 * @rbuf:    read buffer to hold @len bytes
 *
 * Description:
 *   The write command used here is 0x84--indicating that the cache is not cleared first.
 *   Since it is writing the data to cache, there is no tPROG time.
 */
static int spinand_program_data_to_cache(struct spi_device *spi_nand, struct spinand_info *info, u16 byte_id, u16 len, u8* wbuf)
{
	struct spinand_cmd cmd = {0};
	u16 column;

	column = byte_id;

	cmd.cmd = CMD_PROG_PAGE_CLRCACHE;
	cmd.n_addr = 2;
	cmd.addr[0] = (u8)((column&0xff00)>>8);
	cmd.addr[1] = (u8)(column&0x00ff);
	cmd.n_tx = len;
	cmd.tx_buf = wbuf;

	return spinand_cmd(spi_nand, &cmd);
}

/**
 * spinand_program_execute--to write a page from cache to the Nand array with:
 * @page_id: the physical page location to write the page.
 *
 * Description:
 *   The write command used here is 0x10--indicating the cache is writing to the Nand array.
 *   Need to wait for tPROG time to finish the transaction.
 */
static int spinand_program_execute(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id)
{
	struct spinand_cmd cmd = {0};
	u16 row;

	row = page_id;

	cmd.cmd = CMD_PROG_PAGE_EXC;
	cmd.n_addr = 3;
	cmd.addr[1] = (u8)((row&0xff00)>>8);
	cmd.addr[2] = (u8)(row&0x00ff);

	return spinand_cmd(spi_nand, &cmd);
}

/**
 * spinand_program_page--to write a page with:
 * @page_id: the physical page location to write the page.
 * @offset:  the location from the cache starting from 0 to 2111
 * @len:     the number of bytes to write 
 * @wbuf:    the buffer to hold the number of bytes
 *
 * Description:
 *   The commands used here are 0x06, 0x84, and 0x10--indicating that the write enable is first
 *   sent, the write cache command, and the write execute command
 *   Poll to wait for the tPROG time to finish the transaction.
 */
static int spinand_program_page(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id, u16 offset, u16 len, u8* buf)
{
	ssize_t retval;
	u8 status = 0;
	uint8_t *wbuf;
#ifdef CONFIG_MTD_SPINAND_ONDIEECC 
	unsigned int i, j;

	enable_read_hw_ecc = 0;
	wbuf = kzalloc(2112,GFP_KERNEL);
	spinand_read_page(spi_nand, info, page_id, 0, 2112, wbuf);
	for(i=offset, j=0; i<len; i++,j++)
	{
		wbuf[i] &= buf[j];
	}
	if(enable_hw_ecc)
		retval = spinand_enable_ecc(spi_nand, info);

#else
	wbuf = buf;
#endif

	retval = spinand_write_enable(spi_nand, info);
	
	retval = spinand_program_data_to_cache(spi_nand, info, offset, len, wbuf);

	retval = spinand_program_execute(spi_nand, info, page_id);

	while (1)
	{
		retval = spinand_read_status(spi_nand, info, &status);
		if (retval<0) {
			dev_err(&spi_nand->dev, "error %d reading status register\n",
					(int) retval);
			return retval;
		}

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{

			if ((status & STATUS_P_FAIL_MASK) == STATUS_P_FAIL)
			{
				dev_err(&spi_nand->dev, "program error, page=%d\n", page_id);
				return -1;
			}
			else
				break;
		}
	}
#ifdef CONFIG_MTD_SPINAND_ONDIEECC 
	if(enable_hw_ecc)	{
		retval = spinand_disable_ecc(spi_nand, info);
		enable_hw_ecc = 0;
	}
	kfree(wbuf);
#endif

	return 0;
}

/**
 * spinand_erase_block_erase--to erase a page with:
 * @block_id: the physical block location to erase.
 *
 * Description:
 *   The command used here is 0xd8--indicating an erase command to erase one block--64 pages
 *   Need to wait for tERS.
 */
static int spinand_erase_block_erase(struct spi_device *spi_nand, struct spinand_info *info, u16 block_id)
{
	struct spinand_cmd cmd = {0};
	u16 row;

	//row = block_id << 6;
	row = block_id;
	cmd.cmd = CMD_ERASE_BLK;
	cmd.n_addr = 3;
	cmd.addr[1] = (u8)((row&0xff00)>>8);
	cmd.addr[2] = (u8)(row&0x00ff);

	return spinand_cmd(spi_nand, &cmd);	
}

/**
 * spinand_erase_block--to erase a page with:
 * @block_id: the physical block location to erase.
 *
 * Description:
 *   The commands used here are 0x06 and 0xd8--indicating an erase command to erase one block--64 pages
 *   It will first to enable the write enable bit ( 0x06 command ), and then send the 0xd8 erase command
 *   Poll to wait for the tERS time to complete the tranaction.
 */
static int spinand_erase_block(struct spi_device *spi_nand, struct spinand_info *info, u16 block_id)
{
	ssize_t retval;
	u8 status= 0;
	
	retval = spinand_write_enable(spi_nand, info);
	
	retval = spinand_erase_block_erase(spi_nand, info, block_id);

	while (1)
	{
		retval = spinand_read_status(spi_nand, info, &status);
		if (retval<0) {
			dev_err(&spi_nand->dev, "error %d reading status register\n",
					(int) retval);
			return retval;
		}

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{
			if ((status & STATUS_E_FAIL_MASK) == STATUS_E_FAIL)
			{
				dev_err(&spi_nand->dev, "erase error, block=%d\n", block_id);
				return -1;
			}
			else
				break;
		}
	}

	return 0;
}

/*
* spinand_get_info: get NAND info, from read id or const value 

 * Description:
 *   To set up the device parameters.
 */
static int spinand_get_info(struct spi_device *spi_nand, struct spinand_info *info, u8* id)
{
	if (id[0]==0x2C && (id[1]==0x11 || id[1]==0x12 || id[1]==0x13))
	{
		info->mid = id[0];
		info->did = id[1];
		info->name = "MT29F1G01ZAC";
		info->nand_size = (1024 * 64 * 2112);
		info->usable_size = (1024 * 64 * 2048);
		info->block_size = (2112*64);
		info->block_main_size = (2048*64);
		info->block_num_per_chip = 1024;
		info->page_size = 2112;
		info->page_main_size = 2048;
		info->page_spare_size = 64;
		info->page_num_per_block = 64;

		info->block_shift = 17;
		info->block_mask = 0x1ffff;

		info->page_shift = 11;
		info->page_mask = 0x7ff;
	}	
	
	return 0;
}

#ifdef CONFIG_MTD_SPINAND_ONDIEECC 
static void spinand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
	const uint8_t *p = buf;
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;

	enable_hw_ecc = 1;
	chip->write_buf(mtd, p, eccsize * eccsteps);
	return;
}

static int spinand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buf, int page)
{
	u8 retval, status;
	uint8_t *p = buf;
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;
        struct spinand_info *info = (struct spinand_info *)chip->priv;

	enable_read_hw_ecc = 1;	

	chip->read_buf(mtd, p, eccsize*eccsteps);
	while(1)
	{	
		retval = spinand_read_status(info->spi, info, &status);
		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{
			if ((status & STATUS_ECC_MASK) == STATUS_ECC_ERROR)
			{
				printk("spinand: ECC error \n");
				mtd->ecc_stats.failed++;
			}
			else if((status & STATUS_ECC_MASK) == STATUS_ECC_1BIT_CORRECTED )	{
				mtd->ecc_stats.corrected ++;
			}
			break;
		}

	
	}
	return 0;

}
#endif

static void spinand_select_chip(struct mtd_info *mtd, int dev)
{
        struct nand_chip *chip = (struct nand_chip *)mtd->priv;
        struct spinand_info *info = (struct spinand_info *)chip->priv;
        struct nand_state *state = (struct nand_state *)info->priv;

                state->cs = 1;
}


static uint8_t spinand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	struct spinand_info *info = (struct spinand_info *)chip->priv;
	struct nand_state *state = (struct nand_state *)info->priv;
	u8 data;

	data = state->buf[state->buf_ptr];
	state->buf_ptr++;
	return data;
}

static int spinand_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct spinand_info *info = (struct spinand_info *)chip->priv;

	unsigned long timeo = jiffies;
	int retval, state = chip->state;
	u8 status;

	if (state == FL_ERASING)
		timeo += (HZ * 400) / 1000;
	else
		timeo += (HZ * 20) / 1000;
	

	while (time_before(jiffies, timeo)) {
		retval = spinand_read_status(info->spi, info, &status);
		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{
			return 0;
		}
		cond_resched();
	}
	return 0;
}

static void spinand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	struct spinand_info *info = (struct spinand_info *)chip->priv;
	struct nand_state *state = (struct nand_state *)info->priv;

	memcpy(state->buf+state->buf_ptr, buf, len);
	state->buf_ptr += len;
	return;
}

static void spinand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	struct spinand_info *info = (struct spinand_info *)chip->priv;
	struct nand_state *state = (struct nand_state *)info->priv;

	memcpy(buf, state->buf+state->buf_ptr, len);
	state->buf_ptr += len;
	return;
}

static void cmdfunc(struct mtd_info *mtd,
                    unsigned int command,
                    int column,
                    int page)
{
        struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	struct spinand_info *info = (struct spinand_info *)chip->priv;
	struct nand_state *state = (struct nand_state *)info->priv;
        switch (command) {
	/*
	 * READ0 - read in first  0x800 bytes
	 */
        case NAND_CMD_READ1:
        case NAND_CMD_READ0:
                state->buf_ptr = 0;
		spinand_read_page(info->spi, info, page, 0x0, 0x840, state->buf);
		break;
        /* READOOB reads only the OOB because no ECC is performed. */
        case NAND_CMD_READOOB:
		state->buf_ptr = 0;
		spinand_read_page(info->spi, info, page, 0x800, 0x40, state->buf);
		break;
	case NAND_CMD_RNDOUT:
		state->buf_ptr = column;
		break;
        case NAND_CMD_READID:
                state->buf_ptr = 0;
		spinand_read_id(info->spi, (u8*)state->buf);
		break;
        case NAND_CMD_PARAM:
                state->buf_ptr = 0;
		break;
        /* ERASE1 stores the block and page address */
        case NAND_CMD_ERASE1:
		spinand_erase_block(info->spi, info, page);
		break;
        /* ERASE2 uses the block and page address from ERASE1 */
        case NAND_CMD_ERASE2:
		break;
        /* SEQIN sets up the addr buffer and all registers except the length */
        case NAND_CMD_SEQIN:
		state->col	= column;
		state->row	= page;
		state->buf_ptr = 0;
		break;
        /* PAGEPROG reuses all of the setup from SEQIN and adds the length */
        case NAND_CMD_PAGEPROG:
		spinand_program_page(info->spi, info, state->row, state->col, state->buf_ptr, state->buf);
		break;

        case NAND_CMD_STATUS:
		spinand_get_otp(info->spi, info, state->buf);
		
		if(!(state->buf[0] & 0x80))
		state->buf[0] = 0x80;
		state->buf_ptr = 0;

		break;
        /* RESET command */
        case NAND_CMD_RESET:
		break;
	default:
		printk("command: 0x%x \n", command);
		break;
        }
}



/**
* spinand_probe - [spinand Interface] 
* @spi_nand: registered device driver.
 *
 * Description:
 *   To set up the device driver parameters to make the device available.
 */
static int __devinit spinand_probe(struct spi_device *spi_nand)
{
	ssize_t retval;
	struct mtd_info *mtd;
	struct nand_chip *chip; 
	struct spinand_info *info;
	struct nand_state *state;
	u8 id[2]= {2};
	
	
	retval = spinand_read_id(spi_nand, (u8*)&id);
	if (id[0]==0 && id[1]==0)
	{
		printk(KERN_INFO "SPINAND: read id error! 0x%02x, 0x%02x!\n", id[0], id[1]); 
		return 0;
	}

	info  = kzalloc(sizeof(struct spinand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->spi = spi_nand;
	
	retval = spinand_get_info(spi_nand, info, (u8*)&id);
	printk(KERN_INFO "SPINAND: 0x%02x, 0x%02x, %s\n", id[0], id[1], info->name); 
	printk(KERN_INFO "%s\n", mu_spi_nand_driver_version);
	retval = spinand_lock_block(spi_nand, info, BL_ALL_UNLOCKED);

	state = kzalloc(sizeof(struct nand_state), GFP_KERNEL);
	if(!state)
		return -ENOMEM; 
	
	info->priv = state;
	state->last_cmd  = 0;
	state->cs        = 0;
	state->buf_ptr   = 0 ;
	state->buf =  kzalloc(10 * 64 * 2048, GFP_KERNEL);
	if(!state->buf)
		return -ENOMEM;
	
	chip  = kzalloc(sizeof(struct nand_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	
	chip->ecc.mode      = NAND_ECC_NONE;

#ifdef CONFIG_MTD_SPINAND_SWECC
	chip->ecc.mode      = NAND_ECC_SOFT;
	chip->ecc.size	    = 0x200;
	chip->ecc.steps     = 0x4;
	chip->ecc.total     = chip->ecc.steps * chip->ecc.bytes;
	chip->ecc.layout    = &spinand_oob_64;
        chip->ops.mode 	    = MTD_OOB_AUTO;
#endif

#ifdef CONFIG_MTD_SPINAND_ONDIEECC
	chip->ecc.mode      = NAND_ECC_HW;
	chip->ecc.size	    = 0x200;
	chip->ecc.bytes     = 0x6;
	chip->ecc.steps     = 0x4;
	chip->ecc.total     = chip->ecc.steps * chip->ecc.bytes;
	chip->ecc.layout    = &spinand_oob_64;
        chip->ecc.write_page= spinand_write_page_hwecc;
	chip->ecc.read_page = spinand_read_page_hwecc;	
        chip->ops.mode 	    = MTD_OOB_AUTO;
#else
	retval = spinand_disable_ecc(spi_nand, info);
#endif

	chip->priv = info;
	chip->options |= NAND_CACHEPRG | NAND_SKIP_BBTSCAN;
	chip->read_buf   = spinand_read_buf;
	chip->write_buf  = spinand_write_buf;
	chip->read_byte  = spinand_read_byte;
	chip->select_chip = spinand_select_chip;
	chip->cmdfunc      = cmdfunc;
	chip->waitfunc      = spinand_wait;

	mtd = kzalloc(sizeof(struct mtd_info), GFP_KERNEL);
	if (!mtd)
		return -ENOMEM;

	dev_set_drvdata(&spi_nand->dev, mtd);
	
	mtd->priv = chip;
	mtd->oobsize = 64;

	if(nand_scan(mtd, 1))	{
		return -1;
	}
	return add_mtd_device(mtd) == 1 ? -ENODEV : 0;
}

/**
 * __devexit spinand_remove--Remove the device driver
 * @spi: the spi device.
 *
 * Description:
 *   To remove the device driver parameters and free up allocated memories.
 */
static int __devexit spinand_remove(struct spi_device *spi)
{
	struct mtd_info *mtd;
	struct nand_chip *chip; 
	struct spinand_info *info;
	struct nand_state *state;

	DEBUG(MTD_DEBUG_LEVEL1, "%s: remove\n", dev_name(&spi->dev));

	mtd = dev_get_drvdata(&spi->dev);
	

	chip = (struct nand_chip *)mtd->priv;
	info = (struct spinand_info *)chip->priv;
	state = (struct nand_state *)info->priv;

	del_mtd_device(mtd);	
	kfree(state);
	kfree(info);
	kfree(chip);
	kfree(mtd);
	
	return 0;
}

/**
 * Device name structure description
*/
static struct spi_driver spinand_driver = {
	.driver = {
		.name		= "mt29f",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},

	.probe		= spinand_probe,
	.remove		= __devexit_p(spinand_remove),
};

/**
 * Device driver registration
*/
static int __init spinand_init(void)
{
	int val;
	val = spi_register_driver(&spinand_driver);
	return val;
}

/**
 * unregister Device driver.
*/
static void __exit spinand_exit(void)
{
	spi_unregister_driver(&spinand_driver);
}

module_init(spinand_init);
module_exit(spinand_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Netlogicmicro System Inc.");
MODULE_DESCRIPTION("MTD SPI driver for Micron MT29f nand chips");
MODULE_ALIAS("platform:spinand");
