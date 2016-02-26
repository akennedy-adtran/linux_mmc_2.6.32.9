/*
 * Debugfs support for hosts and cards
 *
 * Copyright (C) 2008 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/fault-inject.h>

#include <linux/scatterlist.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include "core.h"
#include "mmc_ops.h"

#ifdef CONFIG_FAIL_MMC_REQUEST

static DECLARE_FAULT_ATTR(fail_default_attr);
static char *fail_request;
module_param(fail_request, charp, 0);

#endif /* CONFIG_FAIL_MMC_REQUEST */

/* The debugfs functions are optimized away when CONFIG_DEBUG_FS isn't set. */
static int mmc_ios_show(struct seq_file *s, void *data)
{
	static const char *vdd_str[] = {
		[8]	= "2.0",
		[9]	= "2.1",
		[10]	= "2.2",
		[11]	= "2.3",
		[12]	= "2.4",
		[13]	= "2.5",
		[14]	= "2.6",
		[15]	= "2.7",
		[16]	= "2.8",
		[17]	= "2.9",
		[18]	= "3.0",
		[19]	= "3.1",
		[20]	= "3.2",
		[21]	= "3.3",
		[22]	= "3.4",
		[23]	= "3.5",
		[24]	= "3.6",
	};
	struct mmc_host	*host = s->private;
	struct mmc_ios	*ios = &host->ios;
	const char *str;

	seq_printf(s, "clock:\t\t%u Hz\n", ios->clock);
	if (host->actual_clock)
		seq_printf(s, "actual clock:\t%u Hz\n", host->actual_clock);
	seq_printf(s, "vdd:\t\t%u ", ios->vdd);
	if ((1 << ios->vdd) & MMC_VDD_165_195)
		seq_printf(s, "(1.65 - 1.95 V)\n");
	else if (ios->vdd < (ARRAY_SIZE(vdd_str) - 1)
			&& vdd_str[ios->vdd] && vdd_str[ios->vdd + 1])
		seq_printf(s, "(%s ~ %s V)\n", vdd_str[ios->vdd],
				vdd_str[ios->vdd + 1]);
	else
		seq_printf(s, "(invalid)\n");

	switch (ios->bus_mode) {
	case MMC_BUSMODE_OPENDRAIN:
		str = "open drain";
		break;
	case MMC_BUSMODE_PUSHPULL:
		str = "push-pull";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "bus mode:\t%u (%s)\n", ios->bus_mode, str);

	switch (ios->chip_select) {
	case MMC_CS_DONTCARE:
		str = "don't care";
		break;
	case MMC_CS_HIGH:
		str = "active high";
		break;
	case MMC_CS_LOW:
		str = "active low";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "chip select:\t%u (%s)\n", ios->chip_select, str);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		str = "off";
		break;
	case MMC_POWER_UP:
		str = "up";
		break;
	case MMC_POWER_ON:
		str = "on";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "power mode:\t%u (%s)\n", ios->power_mode, str);
	seq_printf(s, "bus width:\t%u (%u bits)\n",
			ios->bus_width, 1 << ios->bus_width);

	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
		str = "legacy";
		break;
	case MMC_TIMING_MMC_HS:
		str = "mmc high-speed";
		break;
	case MMC_TIMING_SD_HS:
		str = "sd high-speed";
		break;
	case MMC_TIMING_UHS_SDR50:
		str = "sd uhs SDR50";
		break;
	case MMC_TIMING_UHS_SDR104:
		str = "sd uhs SDR104";
		break;
	case MMC_TIMING_UHS_DDR50:
		str = "sd uhs DDR50";
		break;
	case MMC_TIMING_MMC_DDR52:
		str = "mmc DDR52";
		break;
	case MMC_TIMING_MMC_HS200:
		str = "mmc HS200";
		break;
	case MMC_TIMING_MMC_HS400:
		str = "mmc HS400";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "timing spec:\t%u (%s)\n", ios->timing, str);

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		str = "3.30 V";
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		str = "1.80 V";
		break;
	case MMC_SIGNAL_VOLTAGE_120:
		str = "1.20 V";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "signal voltage:\t%u (%s)\n", ios->chip_select, str);

	return 0;
}

static int mmc_ios_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_ios_show, inode->i_private);
}

static const struct file_operations mmc_ios_fops = {
	.open		= mmc_ios_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mmc_clock_opt_get(void *data, u64 *val)
{
	struct mmc_host *host = data;

	*val = host->ios.clock;

	return 0;
}

static int mmc_clock_opt_set(void *data, u64 val)
{
	struct mmc_host *host = data;

	/* We need this check due to input value is u64 */
	if (val > host->f_max)
		return -EINVAL;

	mmc_claim_host(host);
	mmc_set_clock(host, (unsigned int) val);
	mmc_release_host(host);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mmc_clock_fops, mmc_clock_opt_get, mmc_clock_opt_set,
	"%llu\n");

void mmc_add_host_debugfs(struct mmc_host *host)
{
	struct dentry *root;

	root = debugfs_create_dir(mmc_hostname(host), NULL);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		goto err_root;

	host->debugfs_root = root;

	if (!debugfs_create_file("ios", S_IRUSR, root, host, &mmc_ios_fops))
		goto err_node;

	if (!debugfs_create_file("clock", S_IRUSR | S_IWUSR, root, host,
			&mmc_clock_fops))
		goto err_node;

#ifdef CONFIG_MMC_CLKGATE
	if (!debugfs_create_u32("clk_delay", (S_IRUSR | S_IWUSR),
				root, &host->clk_delay))
		goto err_node;
#endif
#ifdef CONFIG_FAIL_MMC_REQUEST
	if (fail_request)
		setup_fault_attr(&fail_default_attr, fail_request);
	host->fail_mmc_request = fail_default_attr;
	if (IS_ERR(fault_create_debugfs_attr("fail_mmc_request",
					     root,
					     &host->fail_mmc_request)))
		goto err_node;
#endif
	return;

err_node:
	debugfs_remove_recursive(root);
	host->debugfs_root = NULL;
err_root:
	dev_err(&host->class_dev, "failed to initialize debugfs\n");
}

void mmc_remove_host_debugfs(struct mmc_host *host)
{
	debugfs_remove_recursive(host->debugfs_root);
}

static int mmc_dbg_card_status_get(void *data, u64 *val)
{
	struct mmc_card	*card = data;
	u32		status;
	int		ret;

	mmc_get_card(card);

	ret = mmc_send_status(data, &status);
	if (!ret)
		*val = status;

	mmc_put_card(card);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_card_status_fops, mmc_dbg_card_status_get,
		NULL, "%08llx\n");

#define EXT_CSD_STR_LEN 4096
typedef struct mmc_buf_t
{
	unsigned int max_len;
	unsigned int actual_len;
	char buf[EXT_CSD_STR_LEN];
} mmc_buf_t;

static int mmc_ext_csd_open(struct inode *inode, struct file *filp)
{
	struct mmc_card *card = inode->i_private;
	mmc_buf_t  * mmcbuf = 0;
	char *buf;
	ssize_t n = 0;
	u8 *ext_csd;
	int err, i;

	mmcbuf = (mmc_buf_t  *) kmalloc(sizeof(*mmcbuf), GFP_KERNEL);
	if (!mmcbuf)
		return -ENOMEM;
	mmcbuf->max_len = sizeof(*mmcbuf);
	buf = mmcbuf->buf;

	mmc_get_card(card);
	err = mmc_get_ext_csd(card, &ext_csd);
	mmc_put_card(card);
	if (err)
		goto out_free;

	for (i = 0; i < 512; i++) {
		if( !(i & 0xf )) {
			n += sprintf(buf + n, "\n%03d: ",i);
		}
		n += sprintf(buf + n, "%02x ", ext_csd[i]);
	}
	n += sprintf(buf + n, "\n");
	buf[n++] = 0;
	mmcbuf->actual_len = n;

	BUG_ON(mmcbuf->actual_len >= mmcbuf->max_len);
	filp->private_data = mmcbuf;
	kfree(ext_csd);
	return 0;

out_free:
	kfree(mmcbuf);
	return err;
}

static ssize_t mmc_ext_csd_read(struct file *filp, char __user *ubuf,
				size_t cnt, loff_t *ppos)
{
	mmc_buf_t  *mmcbuf = filp->private_data;
	char *buf = mmcbuf->buf;

	return simple_read_from_buffer(ubuf, cnt, ppos,
				       buf, mmcbuf->actual_len);
}

static int mmc_ext_csd_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static const struct file_operations mmc_dbg_ext_csd_fops = {
	.open		= mmc_ext_csd_open,
	.read		= mmc_ext_csd_read,
	.release	= mmc_ext_csd_release,
	.llseek		= default_llseek,
};

static int mmc_status_ext_open(struct inode *inode, struct file *filp)
{
	struct mmc_card *card = inode->i_private;
	mmc_buf_t  * mmcbuf = 0;
	char *buf;
	ssize_t n = 0;
	u32 mmc_status = 0;
	u32 mmc_state = 0;
	int err;

	mmcbuf = kmalloc(sizeof(*mmcbuf), GFP_KERNEL);
	if (!mmcbuf)
		return -ENOMEM;
	mmcbuf->max_len = sizeof(*mmcbuf);
	buf = mmcbuf->buf;

	mmc_get_card(card);
	err = mmc_send_status(card, &mmc_status);
	mmc_put_card(card);
	if (err)
		goto out_free;

	mmc_state = R1_CURRENT_STATE(mmc_status);
	n += sprintf(buf + n, "State        : ");
	switch(mmc_state){
	case R1_STATE_PRG:
		n += sprintf(buf + n, "Prg\n");
		break;
	case R1_STATE_IDLE:
		n += sprintf(buf + n, "Idle\n");
		break;
	case R1_STATE_READY:
		n += sprintf(buf + n, "Ready\n");
		break;
	case R1_STATE_IDENT:
		n += sprintf(buf + n, "Ident\n");
		break;
	case R1_STATE_STBY:
		n += sprintf(buf + n, "Stby\n");
		break;
	case R1_STATE_TRAN:
		n += sprintf(buf + n, "Tran\n");
		break;
	case R1_STATE_DATA:
		n += sprintf(buf + n, "Data\n");
		break;
	case R1_STATE_RCV:
		n += sprintf(buf + n, "Rcv\n");
		break;
	case R1_STATE_DIS:
		n += sprintf(buf + n, "Dis\n");
		break;
	default:
		n += sprintf(buf + n, "Illegal State\n");
		break;
	}

	if (mmc_status & R1_OUT_OF_RANGE) {
		n += sprintf(buf + n, "error        : out of range\n");
	}
	if (mmc_status & R1_ADDRESS_ERROR) {
		n += sprintf(buf + n, "error        : Address Error\n");
	}
	if (mmc_status & R1_BLOCK_LEN_ERROR) {
		n += sprintf(buf + n, "error        : Block Len\n");
	}
	if (mmc_status & R1_ERASE_SEQ_ERROR) {
		n += sprintf(buf + n, "error        : Erase Seq\n");
	}
	if (mmc_status & R1_ERASE_PARAM) {
		n += sprintf(buf + n, "error        : Erase Param\n");
	}
	if (mmc_status & R1_WP_VIOLATION) {
		n += sprintf(buf + n, "error        : Write Protect\n");
	}
	if (mmc_status & R1_LOCK_UNLOCK_FAILED) {
		n += sprintf(buf + n, "error        : Unlock Failed\n");
	}
	if (mmc_status & R1_COM_CRC_ERROR) {
		n += sprintf(buf + n, "error        : Com CRC\n");
	}
	if (mmc_status & R1_ILLEGAL_COMMAND) {
		n += sprintf(buf + n, "error        : Illegal Command\n");
	}
	if (mmc_status & R1_CARD_ECC_FAILED) {
		n += sprintf(buf + n, "error        : Card CRC\n");
	}
	if (mmc_status & R1_CC_ERROR) {
		n += sprintf(buf + n, "error        : CC\n");
	}
	if (mmc_status & R1_ERROR) {
		n += sprintf(buf + n, "error        : Error\n");
	}
	if (mmc_status & R1_UNDERRUN) {
		n += sprintf(buf + n, "error        : Underrun\n");
	}
	if (mmc_status & R1_OVERRUN) {
		n += sprintf(buf + n, "error        : Overrun\n");
	}
	if (mmc_status & R1_CID_CSD_OVERWRITE) {
		n += sprintf(buf + n, "error        : CID CSD Overwrite\n");
	}
	if (mmc_status & R1_WP_ERASE_SKIP) {
		n += sprintf(buf + n, "error        : Erase Skip\n");
	}
	if (mmc_status & R1_ERASE_RESET) {
		n += sprintf(buf + n, "error        : Erase Reset\n");
	}
	if (mmc_status & R1_SWITCH_ERROR) {
		n += sprintf(buf + n, "error        : Switch\n");
	}
	if (mmc_status & R1_EXCEPTION_EVENT) {
		n += sprintf(buf + n, "error        : Exception Event\n");
	}
	if (mmc_status & R1_APP_CMD) {
		n += sprintf(buf + n, "App Command  : Enabled\n");
	}
	else {
		n += sprintf(buf + n, "App Command  : Disabled\n");
	}
	if (mmc_status & R1_READY_FOR_DATA) {
		n += sprintf(buf + n, "Buffer Status: Ready For Data\n");
	}
	else {
		n += sprintf(buf + n, "Buffer Status: Not Ready for Data\n");
	}

	if (mmc_status & R1_ERASE_RESET) {
		n += sprintf(buf + n, "Erase Reset  : Command Abort\n");
	}
	if (mmc_status & R1_CARD_ECC_DISABLED) {
		n += sprintf(buf + n, "ECC          : Internal Failed to Correct Data\n");
	}
	if (mmc_status & R1_CARD_IS_LOCKED) {
		n += sprintf(buf + n, "Locked\n");
	}

	n += sprintf(buf + n, "\n");
	buf[n++] = 0;
	mmcbuf->actual_len = n;

	BUG_ON(mmcbuf->actual_len >= mmcbuf->max_len);
	filp->private_data = mmcbuf;
	return 0;

out_free:
	kfree(mmcbuf);
	return err;
}

#define EMMC_MFG_ID_SPANSION		0x01
#define EMMC_MFG_ID_MICRON			0xFE

static const struct file_operations mmc_status_ext_fops = {
	.open		= mmc_status_ext_open,
	.read		= mmc_ext_csd_read,
	.release	= mmc_ext_csd_release,
	.llseek		= default_llseek,
};

static int mmc_cid_open(struct inode *inode, struct file *filp)
{
	struct mmc_card *card = inode->i_private;
	mmc_buf_t  *mmcbuf = 0;
	char *buf;
	ssize_t n = 0;
	int err;

	mmcbuf = kmalloc(sizeof(*mmcbuf), GFP_KERNEL);
	if (!mmcbuf)
		return -ENOMEM;

	mmcbuf->max_len = sizeof(*mmcbuf);
	buf = mmcbuf->buf;

	if (! mmc_card_mmc(card) ) {
		err = -ENOMEM;
		goto out_free;
	}

	n += sprintf(buf + n, "Manufacturer : ");
	if (card->cid.manfid == EMMC_MFG_ID_SPANSION)
		n += sprintf(buf + n, "Spansion\n");
	else if (card->cid.manfid == EMMC_MFG_ID_MICRON)
		n += sprintf(buf + n, "Micron\n");
	else
		n += sprintf(buf + n, "Unknown (%i)\n", card->cid.manfid);

	n += sprintf(buf + n, "Product Name : %s\n", card->cid.prod_name);
	n += sprintf(buf + n, "Revision     : %02x\n", (unsigned) card->cid.prv);
	n += sprintf(buf + n, "Serial Number: %08x\n", (unsigned) card->cid.serial);
	n += sprintf(buf + n, "OEM ID       : %04x\n", (unsigned) card->cid.oemid);
	n += sprintf(buf + n, "Hardware Rev : %02x\n", (unsigned) card->cid.hwrev);
	n += sprintf(buf + n, "Firmware Rev : %02x\n", (unsigned)card->cid.fwrev);
	n += sprintf(buf + n, "Year         : %04i\n", (unsigned) card->cid.year);
	n += sprintf(buf + n, "Month        : %02i\n", (unsigned) card->cid.month);
	n += sprintf(buf + n, "\n");
	buf[n++] = 0;
	mmcbuf->actual_len = n;

	BUG_ON(mmcbuf->actual_len >= mmcbuf->max_len);
	filp->private_data = mmcbuf;
	return 0;

out_free:
	kfree(mmcbuf);
	return err;
}

static const struct file_operations mmc_cid_fops = {
	.open		= mmc_cid_open,
	.read		= mmc_ext_csd_read,
	.release	= mmc_ext_csd_release,
	.llseek		= default_llseek,
};

/* Additions for MMC Health Request Monitoring */
#define HEALTH_REQ_DATA_SZ			512
#define HEALTH_REQ(r,idx,a1,a2)		((r) | ((idx) << 1) | ((a1) << 8) | ((a2) << 16))
#define HEALTH_REQ_ENABLE			0x4B534BFB
#define HEALTH_REQ_BAD_BLK_INFO		HEALTH_REQ(1, 0x08, 0, 0)
#define HEALTH_REQ_ERASE_COUNT		HEALTH_REQ(1, 0x10, 0, 0)
#define HEALTH_REQ_MLC_ERASE_COUNT	HEALTH_REQ(1, 0x11, 0, 0)
#define HEALTH_REQ_SLC_ERASE_COUNT	HEALTH_REQ(1, 0x12, 0, 0)
/* Health monitoring functions can take a lot of time (125 - 500 ms). With the 4.2.4
 * MMC core code, if a data timeout occurs, we can not complete the data transfer.
 * Thus we can't poll the card for the end of the busy state like with earlier kernels.
 * So set a big enough timeout. Note need to set the cmd.flag MMC_RSP_BUSY in order
 * for sdhci_prepare_data to actually set the timeout.
 */
#define HEALTH_REQ_TIMEOUT_NS		500000000
static int mmc_health_request(struct mmc_card *card, u32 arg, u8 *buf)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	int err;

	mmc_claim_host(card->host);

	err = mmc_set_blocklen(card, HEALTH_REQ_DATA_SZ);
	if (err) {
		mmc_release_host(card->host);
		printk("%s: Set block length failed, err = %d\n", __func__, err);
		return err;
	}

	mrq.cmd = &cmd;
	mrq.data = &data;
	cmd.opcode = MMC_GEN_CMD; // CMD56
	cmd.arg = arg;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC | MMC_RSP_BUSY;
	data.blksz = HEALTH_REQ_DATA_SZ;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;
	data.timeout_ns = HEALTH_REQ_TIMEOUT_NS;
	sg_init_one(&sg, buf, HEALTH_REQ_DATA_SZ);

	mmc_wait_for_req(card->host, &mrq);
	if (cmd.error) {
		mmc_release_host(card->host);
		printk("\n%s: MMC command error %d\n", __func__, cmd.error);
		return cmd.error;
	}
	if (data.error) {
		mmc_release_host(card->host);
		printk("\n%s: MMC data error %d\n", __func__, data.error);
		return data.error;
	}

	mmc_release_host(card->host);
	if (data.bytes_xfered != data.blocks * data.blksz) return -EIO;

	return 0;
}

static int spansion_mmc_health_enable(struct mmc_card *card)
{
	static int isenabled = 0;
	int i, err;
	u8 *buf;

	if (isenabled) return 0;

/* Required for XLP-I SDHCI controller - can't use DMA address > 32b and no IOMMU */
#if defined(CONFIG_64BIT) && defined(CONFIG_CPU_XLP)
	buf = kzalloc(HEALTH_REQ_DATA_SZ, GFP_KERNEL | GFP_DMA);
#else
	buf = kzalloc(HEALTH_REQ_DATA_SZ, GFP_KERNEL);
#endif
	if (buf == NULL) return -ENOMEM;

	/* Spansion command to enable health monitoring. See - 
	 * per eMMC_Enabling_Health_Monitoring_Command_AN_01.pdf 
	 */
	err = mmc_health_request(card, HEALTH_REQ_ENABLE, buf);
	if (err) {
		printk("%s: Health monitor enable cmd failed, err = %d\n", __func__, err);
		kfree(buf);
		return -EIO;
	}

	for (i = 0; i < HEALTH_REQ_DATA_SZ; ++i) {
		if (buf[i] != 0xff) {
			printk("%s: Health monitor enable failed, byte %3d = %02X\n",
					__func__, i, buf[i]);
			kfree(buf);
			return -EIO;
		}
	}

	isenabled = 1;
	kfree(buf);
	printk("Spansion eMMC Health Monitoring enabled\n");
	return 0;
}

static int mmc_health_show(struct seq_file *s, void *d)
{
	struct mmc_card *card = s->private;
	u8 *buf;
	int err, i;

	// EXT_CSD health data:
	mmc_claim_host(card->host);
	err = mmc_get_ext_csd(card, &buf);
	mmc_release_host(card->host);
	if (err) {
		printk("%s: mmc_get_ext_csd failed, err = %d\n", __func__, err);
		kfree(buf);
		return err;
	}

	seq_printf(s, "Device version: %02x %02x\n", buf[262], buf[263]);
	seq_printf(s, "Firmware version:");
	for (i = 0; i < 8; i++) seq_printf(s, " %02x", buf[254 + i]);
	seq_printf(s, "\n");

	seq_printf(s, "Bad block lifetime: %d ", buf[267]);
	switch (buf[267]) {
		case 0x01: seq_printf(s, "(normal)\n"); break;
		case 0x02: seq_printf(s, "(Warning - consumed 80%% of spare blocks)\n"); break;
		case 0x03: seq_printf(s, "(Urgent - 8 or less remaining spare blocks)\n"); break;
		default:   seq_printf(s, "(unknown)\n");
	}

	seq_printf(s, "MLC device lifetime: %d ", buf[268]);
	switch (buf[268]) {
		case 0x01: seq_printf(s, "(0-10%% device lifetime used)\n"); break;
		case 0x02: seq_printf(s, "(10-20%% device lifetime used)\n"); break;
		case 0x03: seq_printf(s, "(20-30%% device lifetime used)\n"); break;
		case 0x04: seq_printf(s, "(30-40%% device lifetime used)\n"); break;
		case 0x05: seq_printf(s, "(40-50%% device lifetime used)\n"); break;
		case 0x06: seq_printf(s, "(50-60%% device lifetime used)\n"); break;
		case 0x07: seq_printf(s, "(60-70%% device lifetime used)\n"); break;
		case 0x08: seq_printf(s, "(70-80%% device lifetime used)\n"); break;
		case 0x09: seq_printf(s, "(80-90%% device lifetime used)\n"); break;
		case 0x0a: seq_printf(s, "(90-100%% device lifetime used)\n"); break;
		case 0x0b: seq_printf(s, "(exceeded its max. est. device lifetime)\n"); break;
		default:   seq_printf(s, "(unknown)\n");
	}

	seq_printf(s, "SLC device lifetime: %d ", buf[269]);
	switch (buf[269]) {
		case 0x01: seq_printf(s, "(0-10%% device lifetime used)\n"); break;
		case 0x02: seq_printf(s, "(10-20%% device lifetime used)\n"); break;
		case 0x03: seq_printf(s, "(20-30%% device lifetime used)\n"); break;
		case 0x04: seq_printf(s, "(30-40%% device lifetime used)\n"); break;
		case 0x05: seq_printf(s, "(40-50%% device lifetime used)\n"); break;
		case 0x06: seq_printf(s, "(50-60%% device lifetime used)\n"); break;
		case 0x07: seq_printf(s, "(60-70%% device lifetime used)\n"); break;
		case 0x08: seq_printf(s, "(70-80%% device lifetime used)\n"); break;
		case 0x09: seq_printf(s, "(80-90%% device lifetime used)\n"); break;
		case 0x0a: seq_printf(s, "(90-100%% device lifetime used)\n"); break;
		case 0x0b: seq_printf(s, "(exceeded its max. est. device lifetime)\n"); break;
		default:   seq_printf(s, "(unknown)\n");
	}

	err = mmc_health_request(card, HEALTH_REQ_BAD_BLK_INFO, buf);
	if (err) {
		printk("%s: Request bad block info failed\n", __func__);
		kfree(buf);
		return -EIO;
	}
	seq_printf(s, "Initial bad block count: %d\n", buf[0] * 256 + buf[1]);
	seq_printf(s, "Runtime bad block count: %d\n", buf[2] * 256 + buf[3]);
	seq_printf(s, "Spare block count: %d\n", buf[4] * 256 + buf[5]);

	err = mmc_health_request(card, HEALTH_REQ_ERASE_COUNT, buf);
	if (err) {
		printk("%s: Request erase count failed\n", __func__);
		kfree(buf);
		return -EIO;
	}
	seq_printf(s, "Min. block erase count: %d\n", buf[0] * 256 + buf[1]);
	seq_printf(s, "Max. block erase count: %d\n", buf[2] * 256 + buf[3]);
	seq_printf(s, "Avg. block erase count: %d\n", buf[4] * 256 + buf[5]);

	err = mmc_health_request(card, HEALTH_REQ_MLC_ERASE_COUNT, buf);
	if (err) {
		printk("%s: Request MLC erase count failed\n", __func__);
		kfree(buf);
		return -EIO;
	}
	seq_printf(s, "Min. block erase count (MLC area): %d\n", buf[0] * 256 + buf[1]);
	seq_printf(s, "Max. block erase count (MLC area): %d\n", buf[2] * 256 + buf[3]);
	seq_printf(s, "Avg. block erase count (MLC area): %d\n", buf[4] * 256 + buf[5]);

	err = mmc_health_request(card, HEALTH_REQ_SLC_ERASE_COUNT, buf);
	if (err) {
		printk("%s: Request SLC erase count failed\n", __func__);
		kfree(buf);
		return -EIO;
	}
	seq_printf(s, "Min. block erase count (SLC area): %d\n", buf[0] * 256 + buf[1]);
	seq_printf(s, "Max. block erase count (SLC area): %d\n", buf[2] * 256 + buf[3]);
	seq_printf(s, "Avg. block erase count (SLC area): %d\n", buf[4] * 256 + buf[5]);

	kfree(buf);
	return 0;
}

static int mmc_health_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_health_show, inode->i_private);
}

static const struct file_operations mmc_dbg_health_fops = {
	.open    = mmc_health_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

void mmc_add_card_debugfs(struct mmc_card *card)
{
	struct mmc_host	*host = card->host;
	struct dentry	*root;
	int err;

	if (!host->debugfs_root)
		return;

	root = debugfs_create_dir(mmc_card_id(card), host->debugfs_root);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		goto err;

	card->debugfs_root = root;

	if (!debugfs_create_x32("state", S_IRUSR, root, &card->state))
		goto err;

	if (!debugfs_create_file("status_ext", S_IRUSR, root, card,
		&mmc_status_ext_fops))
		goto err;
	
	if (!debugfs_create_file("cid", S_IRUSR, root, card,
		&mmc_cid_fops))
		goto err;

	if (mmc_card_mmc(card) || mmc_card_sd(card))
		if (!debugfs_create_file("status", S_IRUSR, root, card,
					&mmc_dbg_card_status_fops))
			goto err;

	if (mmc_card_mmc(card))
		if (!debugfs_create_file("ext_csd", S_IRUSR, root, card,
					&mmc_dbg_ext_csd_fops))
			goto err;

	/* Enable health output for Micron and Spansion eMMC devices. Note
	 * Spansion device requires an extra step to enable the health commands.
	 */
	err = -1;
	if (mmc_card_mmc(card) && (host->caps & MMC_CAP_NONREMOVABLE)) {
		if (card->cid.manfid == EMMC_MFG_ID_SPANSION)
			err = spansion_mmc_health_enable(card);
		else if (card->cid.manfid == EMMC_MFG_ID_MICRON)
			err = 0;
	}
	if (!err)
		debugfs_create_file("health", S_IRUSR, root, card, &mmc_dbg_health_fops);

	return;

err:
	debugfs_remove_recursive(root);
	card->debugfs_root = NULL;
	dev_err(&card->dev, "failed to initialize debugfs\n");
}

void mmc_remove_card_debugfs(struct mmc_card *card)
{
	debugfs_remove_recursive(card->debugfs_root);
}

