/*
 * Copyright (c) 2003-2012 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the Broadcom
 * license below:
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
 */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/platform_device.h>

#include <asm/netlogic/common.h>
#include <asm/netlogic/haldefs.h>
#include <asm/netlogic/xlp-hal/iomap.h>
#include <asm/netlogic/xlp-hal/xlp.h>
#include <linux/andy.h>

#include "sdhci.h"

#define XLP_SLOT_SIZE		0x100
#define XLP_NUM_SD_SLOT		1

#define nlm_get_mmc_pcibase(node)	\
	nlm_pcicfg_base(cpu_is_xlp9xx() ?  XLP9XX_IO_MMC_OFFSET(node) : \
						XLP_IO_MMC_OFFSET(node))
#define nlm_get_mmc_regbase(node)	\
			(nlm_get_mmc_pcibase(node) + XLP_IO_PCI_HDRSZ)

#define nlm_get_mmcsd_regbase(node, slot) \
		(nlm_get_mmc_regbase(node) + slot * XLP_SLOT_SIZE)

struct sdhci_xlp_chip {
	struct pci_dev		*pdev;
	struct sdhci_host	*host[XLP_NUM_SD_SLOT];
};

/* Setting XLP device and Vendor IDs */
static const struct pci_device_id xlp_pci_ids[] __devinitdata = {
	{
		.vendor         = PCI_VENDOR_NETLOGIC,
		.device         = PCI_DEVICE_ID_NLM_MMC,
		.subvendor      = PCI_ANY_ID,
		.subdevice      = PCI_ANY_ID,
	},
	{
		.vendor         = PCI_VENDOR_ID_BROADCOM,
		.device         = PCI_DEVICE_ID_XLP9XX_MMC,
		.subvendor      = PCI_ANY_ID,
		.subdevice      = PCI_ANY_ID,
	},
};

static unsigned int xlp_get_max_clock(struct sdhci_host *sd_host)
{
	/*
	 * SDHCI_CAPABILITIES[15:8] register gives 0x00 instead of 0x85(133)
	 * on xlp2xx B0 and xlp9xx chips. Forcibly returning 133MHz.
	 */
	return 133000000;
}

static inline void xlp_writel(struct sdhci_host *host, u32 val, int reg)
{
	__raw_writel(val, host->ioaddr + reg);
}

static inline void xlp_writew(struct sdhci_host *host, u16 val, int reg)
{
	__raw_writew(val, host->ioaddr + reg);
}

static inline void xlp_writeb(struct sdhci_host *host, u8 val, int reg)
{
	__raw_writeb(val, host->ioaddr + reg);
}

static inline u32 xlp_readl(struct sdhci_host *host, int reg)
{
	return __raw_readl(host->ioaddr + reg);
}

static inline u16 xlp_readw(struct sdhci_host *host, int reg)
{
	return __raw_readw(host->ioaddr + reg);
}

static inline u8 xlp_readb(struct sdhci_host *host, int reg)
{
	return __raw_readb(host->ioaddr + reg);
}

#define HC_NORMAL_INT_STS_EN            0x0034
#define HC_NORMAL_INT_SIGNAL_EN         0x0038
#define HC_PC_HC                        0x0028
#define HC_NORMAL_INT_STS               0x0030
#define HC_ERROR_INT_STS                0x0032
#define XLP_INT_RESET			0x37ff7fff

static void xlpmmc_reset_controller(struct sdhci_host *host, u8 mask)
{
	/* FIXME:  get rid of the two volatiles above for the preferred xlp_write* */
	volatile u32 *mmc_base32 = (u32 *)(host->ioaddr);
	volatile u16 *mmc_base16 = (u16 *)(host->ioaddr);

	/* Enable Interrupts */
	xlp_writel(host, XLP_INT_RESET, SDHCI_INT_ENABLE);
	//hc_wr32(host->ioaddr, HC_NORMAL_INT_STS_EN, 0x37ff7fff, slot);

	/* Enable Interrupt Signals */
	xlp_writel(host, XLP_INT_RESET, SDHCI_SIGNAL_ENABLE);
	//hc_wr32(host->base, HC_NORMAL_INT_SIGNAL_EN, 0x37ff7fff, slot);

	/* Send HW Reset to eMMC-4.4 Card */
	mmc_base16[HC_PC_HC>>1] = 0x1e00;
	//hc_wr16(host->base, HC_PC_HC, 0x1e00, slot);
	mmc_base16[HC_PC_HC>>1] = 0x1f00;
	//hc_wr16(host->base, HC_PC_HC, 0x1f00, slot);

	/* Remove HW Reset to eMMC-4.4 Card */
	mmc_base16[HC_PC_HC>>1] = 0x0f00;
	//hc_wr16(host->base, HC_PC_HC, 0x0f00, slot);

	/* Flush any pending interrupts */
	mmc_base32[HC_ERROR_INT_STS>>2] = mmc_base32[HC_ERROR_INT_STS>>2];
	//hc_wr32(base, HC_ERROR_INT_STS, hc_rd32(base, HC_ERROR_INT_STS, slot), slot);
	mmc_base32[HC_NORMAL_INT_STS>>2] = mmc_base32[HC_NORMAL_INT_STS>>2];
	//hc_wr32(base, HC_NORMAL_INT_STS, hc_rd32(base, HC_NORMAL_INT_STS, slot), slot);

	return;
}


static struct sdhci_ops xlp_sdhci_ops = {
	.read_l = xlp_readl,
	.read_w = xlp_readw,
	.read_b = xlp_readb,
	.write_l = xlp_writel,
	.write_w = xlp_writew,
	.write_b = xlp_writeb,
	.get_max_clock = xlp_get_max_clock,
	.set_clock = sdhci_set_clock,
	.get_timeout_clock = xlp_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.set_bus_width = sdhci_set_bus_width,
	.reset = xlpmmc_reset_controller,
};

/* VDD volt 2.70 ~ 3.60 */
#define MMC_VDD_27_36	(MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 | \
			 MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 | \
			 MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36)
/* Reference clock to MMC host controller - assume 133 MHz.  TODO
   Add code to actually check the reference in case it was changed */
#define XLP_REF_CLK133MHZ		133333333

#define XLPMMC_DESCRIPTOR_SIZE	(64<<10)
#define XLPMMC_DESCRIPTOR_COUNT	1
#define GPIO_MMC_DETECT		29

static uint32_t gpio_regread(int regidx) 
{
	uint64_t mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_GPIO_FUNC);
	return nlm_hal_read_32bit_reg(mmio, regidx);
}

static void gpio_regwrite(int regidx, uint32_t val)
{
	uint64_t mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_GPIO_FUNC);
	nlm_hal_write_32bit_reg(mmio, regidx, val);
}

/* Platform driver pointer from arch/mips/netlogic/xlp/platform.c */
extern struct platform_device *mmc_pplat_dev;

static int sdhci_xlp_probe_slot(struct pci_dev *pdev,
					struct sdhci_xlp_chip *chip, int sltno)
{
	struct sdhci_xlp_chip *chip_sd;
	struct sdhci_host *sd_host;
	struct mmc_host *mmc;
	int ret;

printm();
	sd_host = sdhci_alloc_host(&pdev->dev, sizeof(struct sdhci_xlp_chip));
printm();
	if (IS_ERR(sd_host)) {
		dev_err(&pdev->dev, "sdhci_alloc_host() failed!\n");
printb(-ENODEV);
		return -ENODEV;
	}

printm();
	chip_sd = sdhci_priv(sd_host);
	chip->host[sltno] = sd_host;
	chip_sd->pdev = chip->pdev;

printm();
	sd_host->hw_name = "xlp-mmc";
	sd_host->ops = &xlp_sdhci_ops;
	sd_host->irq = pdev->irq;

printm();
	/*
	 * The capabilities register reports block size as 3 for 4096,
	 * force it to 2048.
	 * The cmd and data lines are not getting cleared by hardware on writing
	 * 0x2 and 0x4 to MMC_SWRESET [2,1] register, if the card is not present
	 * in the slot.
	 * Avoid reset if the card is not present.
	 */
	sd_host->quirks = SDHCI_QUIRK_FORCE_BLK_SZ_2048 |
				SDHCI_QUIRK_NO_CARD_NO_RESET;

printm();
	ret = pci_request_region(pdev, sltno, mmc_hostname(sd_host->mmc));
printm();
	if (ret) {
printm();
		dev_err(&pdev->dev, "can't request region\n");
		goto err1;
	}

printm();
	sd_host->ioaddr = devm_ioremap(&pdev->dev,
					pci_resource_start(pdev, sltno),
					pci_resource_len(pdev, sltno));
printm();
	if (!sd_host->ioaddr) {
printm();
		dev_err(&pdev->dev, "failed to remap registers\n");
		ret = -ENOMEM;
		goto err0;
	}

printm();

	/* *********************************************************************** */
	/* *********************************************************************** */
	/* *********************************************************************** */
	/* *********************************************************************** */
	/* I don't know if anything in this block is needed. . . I do, however, know
	 * that the further I got in the code, some of this stuff was required to
	 * make things on the eMMC work */

	/* These GPIO calls seemed to have no effect there or not. */
printv(gpio_regread(XLP_GPIO_INTEN00));
	gpio_regwrite(XLP_GPIO_INTEN00, gpio_regread(XLP_GPIO_INTEN00) | 
			0x1<<(GPIO_MMC_DETECT+sltno));
printv(gpio_regread(XLP_GPIO_INTEN00));
printv(gpio_regread(XLP_8XX_GPIO_INT_POLAR0));
	gpio_regwrite(XLP_8XX_GPIO_INT_POLAR0, gpio_regread(XLP_8XX_GPIO_INT_POLAR0) | 
			0x1<<(GPIO_MMC_DETECT+sltno));
printv(gpio_regread(XLP_8XX_GPIO_INT_POLAR0));
printv(gpio_regread(XLP_8XX_GPIO_INT_TYPE0));
	gpio_regwrite(XLP_8XX_GPIO_INT_TYPE0, gpio_regread(XLP_8XX_GPIO_INT_TYPE0) | 
			0x1<<(GPIO_MMC_DETECT+sltno));
printv(gpio_regread(XLP_8XX_GPIO_INT_TYPE0));
	mmc = sd_host->mmc ;

	/* Pretty sure that the following is needed -- through the xlpmmc_reset below */

	/* Inform kernel of min and max clock divider values we support - assuming
	 * 10-bit divider and all divider values supported.  Note in 10b mode, divider
	 * value does not appear to be 2x the value in the register (PRM says it IS 2x) */
	mmc->f_min = XLP_REF_CLK133MHZ / 1023;
	mmc->f_max = XLP_REF_CLK133MHZ / 3;     // 44.33 MHz

	mmc->max_blk_size = 512;
	mmc->max_blk_count = 2048;

	mmc->max_seg_size = XLPMMC_DESCRIPTOR_SIZE;
	mmc->max_segs = XLPMMC_DESCRIPTOR_COUNT;

	mmc->ocr_avail = MMC_VDD_27_36; /* volt 2.70 ~ 3.60 */
	mmc->caps = MMC_CAP_NONREMOVABLE;

	/* High speed mode at single data rate can be supported with 3.3V signaling */
	sd_host->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED |
			MMC_CAP_MMC_HIGHSPEED;

	xlpmmc_reset_controller(sd_host, 0);
	msleep(5);
	/* *********************************************************************** */
	/* *********************************************************************** */
	/* *********************************************************************** */
	/* *********************************************************************** */

	/* Registering host */
printm();
	ret = sdhci_add_host(sd_host);
/* Don't know what I've done, but here we never return from sdhci_add_host anymore. . . */
#warning I just wanted you to see this note -- We don't return from sdhci_add_host after today. . . 
printm();
	if (ret) {
printm();
		dev_err(&pdev->dev, "sdhci_add_host() failed\n");
		goto err0;
	}

#if 0 /* This was added to try to trick the MMC layer into thinking that the
	 card was inserted.  It didn't seem to have an effect. */
#define XLP_ENABLE_SIMULATION		(1<<7)	
#define XLP_SIMULATE_CARD_INSERT	(1<<6)
	xlp_writel(sd_host, XLP_ENABLE_SIMULATION, SDHCI_HOST_CONTROL);
	msleep(1);
	xlp_writel(sd_host, XLP_SIMULATE_CARD_INSERT, SDHCI_HOST_CONTROL);
#endif


printv(gpio_regread(XLP_GPIO_INTEN00));
printv(gpio_regread(XLP_8XX_GPIO_INT_POLAR0));
printv(gpio_regread(XLP_8XX_GPIO_INT_TYPE0));
printg();
	return 0;

err0:
printm();
	pci_release_region(pdev, sltno);
printm();
err1:
printm();
	sdhci_free_host(sd_host);
printb(ret);
	return ret;
}

/* there are other places where I key off of if (andy) in the code so that
 * I can check to see values as they are being set ONLY during init phase. */
int andy = 0;
static int __devinit sdhci_xlp_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	struct sdhci_xlp_chip *chip;
	struct resource res;
	void __iomem *sys_addr;
	int ret, slotno, node;

printm();
andy=1;
	if (cpu_is_xlp9xx())
		node = PCI_FUNC(pdev->bus->self->devfn);
	else
		node = PCI_SLOT(pdev->devfn) / 8;

printm();
	chip = kzalloc(sizeof(struct sdhci_xlp_chip), GFP_KERNEL);
printm();
	if (!chip) {
printb(-ENOMEM);
		return -ENOMEM;
	}

printm();
	ret = pci_enable_device(pdev);
printm();
	if (ret) {
printm();
		dev_err(&pdev->dev, "pci_enable_device() failed!\n");
		kfree(chip);
printb(ret);
		return ret;
	}

printm();
#if 0
	pdev->irq = nlm_irq_to_xirq(node, PIC_MMC_IRQ);
#else
{
#define DEV_IRT_INFO		0x3d
#define BUS_0			0
#define XLP_MMC_PCIE_DEV	7
#define XLP_MMC_PCIE_FN		3
	uint64_t iobase = nlm_hal_get_dev_base(NODE_0, BUS_0,
				XLP_MMC_PCIE_DEV, XLP_MMC_PCIE_FN);
	uint16_t irt = nlm_hal_read_32bit_reg(iobase, DEV_IRT_INFO) & 0xff;

	pdev->irq = xlp_irt_to_irq(NODE_0, irt);
printv(pdev->irq);
prints("pdev->irq = %i", pdev->irq);
}
#endif
printm();
	chip->pdev = pdev;
	pci_set_drvdata(pdev, chip);
printm();

	/*
	 * Enable slots.
	 * Get system control base, node set to zero.
	 * TODO: node need to be taken care on multinode case.
	 */
	memcpy(&pdev->resource[0], &mmc_pplat_dev->resource[1],
		sizeof(struct resource));
printm();
#if 0
	res.start = CPHYSADDR(nlm_get_mmcsd_regbase(node, 2));
#else
	res.start = mmc_pplat_dev->resource[1].start;
#endif
printm();
	sys_addr = devm_ioremap(&pdev->dev, res.start, 0x28);
printm();
	if (!sys_addr) {
printm();
		ret = -ENOMEM;
		goto err;
	}
printm();
#if XLP_NUM_SD_SLOT == 1
	__raw_writel(0x14, sys_addr);
#else
	__raw_writel(0x1c, sys_addr);
#endif
	msleep(5);

printm();
	/*
	 * The XLP MMC/SD controller has two slots. The registers for the
	 * slots are at fixed location in the PCIe ECFG space, and not
	 * in any PCI BARs.
	 */
	for (slotno = 0; slotno < XLP_NUM_SD_SLOT; slotno++) {
printm();
		pdev->resource[slotno].flags = IORESOURCE_MEM;
#if 0
		pdev->resource[slotno].start =
			CPHYSADDR(nlm_get_mmcsd_regbase(node, slotno));
		pdev->resource[slotno].end = pdev->resource[slotno].start +
						XLP_SLOT_SIZE - 1;
#else
		pdev->resource[slotno].start = res.start + XLP_SLOT_SIZE * slotno + XLP_SLOT_SIZE;
		pdev->resource[slotno].end = res.start + XLP_SLOT_SIZE * slotno + XLP_SLOT_SIZE * 2 - 1;
#endif

printm();
		ret = sdhci_xlp_probe_slot(pdev, chip, slotno);
printm();
		if (ret)
			dev_err(&pdev->dev, "failed to probe slot%d\n", slotno);
printm();
	}

printm();
andy=0;
printg();
	return 0;

err:
printm();
	pci_disable_device(pdev);
printm();
	kfree(chip);
andy=0;
printb(ret);
	return ret;
}

static void __devexit sdhci_xlp_remove(struct pci_dev *pdev)
{
	struct sdhci_xlp_chip *chip;
	int slotno, dead;
	u32 scratch;

	chip = pci_get_drvdata(pdev);
	if (chip) {
		for (slotno = 0; slotno < XLP_NUM_SD_SLOT; slotno++) {
			scratch = readl(chip->host[slotno]->ioaddr +
					SDHCI_INT_STATUS);
			if (scratch == (u32)-1)
				dead = 1;

			sdhci_remove_host(chip->host[slotno], dead);
			pci_release_region(pdev, slotno);
			sdhci_free_host(chip->host[slotno]);
		}
		pci_set_drvdata(pdev, NULL);
	}
	pci_disable_device(pdev);
}

static struct pci_driver sdhci_xlp_driver = {
	.name		= "sdhci-xlp",
	.id_table	= xlp_pci_ids,
	.probe		= sdhci_xlp_probe,
	.remove		= sdhci_xlp_remove,
};

module_pci_driver(sdhci_xlp_driver);

MODULE_AUTHOR("Kamlakant Patel <kamlakant.patel@broadcom.com>");
MODULE_DESCRIPTION("SDHCI Driver for Netlogic XLP");
MODULE_LICENSE("GPL v2");
