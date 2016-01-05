/*
 * Copyright (c) 2003-2015 Broadcom Corporation
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

/* SDHCI PCI-e Driver for XLP SD Host Controller */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/interrupt.h>
#include <asm/netlogic/xlp.h>
#include <asm/io.h>
#include "sdhci.h"

/* Debug options */
//#define XLP_SDHCI_DEBUG				// Enable for debug messages
//#define XLP_SDHCI_DEBUG_VERBOSE		// Above and then some
//#define XLP_SDHCI_DEBUG_REGS			// Show all register read/writes (very verbose!)

/* Module parameters */
static int slot0_is_emmc = 1;
module_param(slot0_is_emmc, int, 0444);
MODULE_PARM_DESC(slot0_is_emmc, "   Set to 1 if slot 0 is connected to an eMMC device");

static int slot0_read_only;
module_param(slot0_read_only, int, 0444);
MODULE_PARM_DESC(slot0_read_only, " Set to 1 to force slot 0 write protect state to true");

static int slot1_enable;
module_param(slot1_enable, int, 0444);
MODULE_PARM_DESC(slot1_enable, "    Set to 1 to enable the second host controller slot");

static int slot1_is_emmc;
module_param(slot1_is_emmc, int, 0444);
MODULE_PARM_DESC(slot1_is_emmc, "   Set to 1 if slot 1 is connected to an eMMC device");

static int slot1_read_only;
module_param(slot1_read_only, int, 0444);
MODULE_PARM_DESC(slot1_read_only, " Set to 1 to force slot 1 write protect state to true");

/* XLP-specific host controller wrapper registers.
 * Note address is byte offset from start of PCI-e header!
 */
#define HC_SYSCTRL					(0xC0 << 2)
	#define HC_RESET				(1 << 0)
	#define HC_CLK_DISABLE			(1 << 1)
	#define HC_ENABLE_SLOT_0		(1 << 2)
	#define HC_ENABLE_SLOT_1		(1 << 3)
	#define HC_ENABLE_L3_ALLOCATE	(1 << 4)
	#define HC_WRITE_PROT_S0		(1 << 6)
	#define HC_WRITE_PROT_S1		(1 << 7)
#define HC_MMC_CAPCFG0_S0			(0xC1 << 2)	// Override for
#define HC_MMC_CAPCFG1_S0			(0xC2 << 2)	// capability registers
#define HC_MMC_INIT_PRESET_CFG_S0	(0xC3 << 2)	// Sets MMC_PRESET0 contents
#define HC_MMC_DEF_PRESET_CFG_S0	(0xC4 << 2)	// Sets MMC_PRESET1 contents
#define HC_MMC_CAPCFG0_S1			(0xC6 << 2)
#define HC_MMC_CAPCFG1_S1			(0xC7 << 2)
#define HC_MMC_INIT_PRESET_CFG_S1	(0xC8 << 2)
#define HC_MMC_DEF_PRESET_CFG_S1	(0xC9 << 2)

#define SDHCI_POWER_RESET			(1 << 4)

#define XLP_SLOT_SIZE				0x100
#define XLP_NUM_SD_SLOT				2

#ifdef XLP_SDHCI_DEBUG_VERBOSE
#define SDHCI_DEBUG_VERBOSE			printk
#define XLP_SDHCI_DEBUG
#else
#define SDHCI_DEBUG_VERBOSE(...)
#endif

#ifdef XLP_SDHCI_DEBUG
#define SDHCI_DEBUG					printk
#define KDBG						KERN_INFO
#else
#define SDHCI_DEBUG(...)
#define KDBG						KERN_DEBUG
#endif

#ifdef XLP_SDHCI_DEBUG_REGS
#define SDHCI_DEBUG_REGS			printk
#else
#define SDHCI_DEBUG_REGS(...)
#endif

/* Insure the capabilities registers reflect the correct capabilities.
 * Note this doesn't match the PRM for good reason.
 */
#define XLP_SDHCI_CAPABILITIES											\
		(  (1 << 22)	/* 3.3V signaling supported */					\
		 | (1 << 21)	/* Suspend/Resume supported */					\
		 | (1 << 20)	/* SDMA supported */							\
		 | (1 << 19)	/* High-speed mode supported (but need quirk) */\
		 | (1 << 18)	/* ADMA2 supported */							\
		 | (2 << 15)	/* Max Block Length (2 = 2048 bytes) */			\
		 | (1 << 6)		/* Time-out clock frequency units (1 = MHz) */	\
		 | 0x30			/* Time-out clock frequency */					\
		)

/* Quirks for 4.2.4 MMC core:
 * 1) The cmd and data lines are not getting cleared by hardware on writing
 *    0x2 and 0x4 to MMC_SWRESET [2,1] register, if the card is not present
 *    in the slot. Avoid reset if the card is not present. Note for an eMMC
 *    device, the card detect signal never gets asserted so this quirk will
 *    prevent reset from being applied. Thus do not apply the quirk if the
 *    slot type is set for eMMC.
 * 2) Experiment - Controller doesn't support high-speed bit in host control
 *    register but does support high-speed modes.
 */
#define XLP_SDHCI_QUIRKS					\
		(  SDHCI_QUIRK_NO_CARD_NO_RESET		\
		 | SDHCI_QUIRK_NO_HISPD_BIT			\
		)
#define XLP_SDHCI_QUIRKS_EMMC	SDHCI_QUIRK_NO_HISPD_BIT
#define XLP_SDHCI_QUIRKS2		0

static struct pci_driver sdhci_xlp_driver;

struct xlp_sdhci_chip {
	struct sdhci_host	*host[XLP_NUM_SD_SLOT];
	void __iomem		*ioaddr;
	struct pci_dev		*pdev;
	uint16_t			 num_slots;
};

/* Use our I/O accessors since the kernel accessors expect little endian.
 * When reading XLP PCI-e config space registers the endianness is always
 * the same as the CPU.
 *
 * For all of these we are assuming reg is given as a byte offset from the
 * start of the device-specific register space (i.e. after the PCIe header
 * offset - therefore the slot probe code needs to set host->ioaddr correctly.
 */
#define REG(x)		(unsigned int)(((unsigned long)(x) & 0x3FFUL) >> 2)
static void xlp_writel(struct sdhci_host *host, u32 data, int reg) {
	volatile u32 *mmio = (volatile u32 *)(host->ioaddr + reg);
	SDHCI_DEBUG_REGS("  Write 4 bytes to 0x%p (reg 0x%02x) <- 0x%08x\n",
			mmio, REG(mmio), data);
	*mmio = data;
}

static void xlp_writew(struct sdhci_host *host, u16 data, int reg) {
	volatile u16 *mmio = (volatile u16 *)(host->ioaddr + reg);
	SDHCI_DEBUG_REGS("  Write 2 bytes to 0x%p (reg 0x%02x) <- 0x    %04x\n",
			mmio, REG(mmio), data);
	*mmio = data;
}

static void xlp_writeb(struct sdhci_host *host, u8 data, int reg) {
	volatile u8 *mmio = (volatile u8 *)(host->ioaddr + reg);
	SDHCI_DEBUG_REGS("  Write 1 byte  to 0x%p (reg 0x%02x) <- 0x      %02x\n",
			mmio, REG(mmio), data);
	*mmio = data;
}

static u32 xlp_readl(struct sdhci_host *host, int reg) {
	volatile u32 *mmio = (volatile u32 *)(host->ioaddr + reg);
	u32 data;
	SDHCI_DEBUG_REGS("  Read  4 bytes at 0x%p (reg 0x%02x)....",
			mmio, REG(mmio));
	data = *mmio;
	SDHCI_DEBUG_REGS(KERN_CONT "0x%08x\n", data);
	return data;
}

static u16 xlp_readw(struct sdhci_host *host, int reg) {
	volatile u16 *mmio = (volatile u16 *)(host->ioaddr + reg);
	u16 data;
	SDHCI_DEBUG_REGS("  Read  2 bytes at 0x%p (reg 0x%02x)....",
			mmio, REG(mmio));
	data = *mmio;
	SDHCI_DEBUG_REGS(KERN_CONT "0x    %04x\n", data);
	return data;
}

static u8 xlp_readb(struct sdhci_host *host, int reg) {
	volatile u8 *mmio = (volatile u8 *)(host->ioaddr + reg);
	u8 data;
	SDHCI_DEBUG_REGS("  Read  1 byte  at 0x%p (reg 0x%02x)....",
			mmio, REG(mmio));
	data = *mmio;
	SDHCI_DEBUG_REGS(KERN_CONT "0x      %02x\n", data);
	return data;
}

static int xlp_enable_dma(struct sdhci_host *host)
{
	unsigned long *priv = (unsigned long *)sdhci_priv(host);
	struct pci_dev *pdev = (struct pci_dev *)priv[0];
	int rv = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));

	if(rv) {
		dev_err(&pdev->dev, "%s Failed to set 32b DMA mask\n",
				mmc_hostname(host->mmc));
		host->flags &= ~(SDHCI_USE_SDMA | SDHCI_USE_ADMA);
		return rv;
	}

	pci_set_master(pdev);

	SDHCI_DEBUG("DMA Enabled for %s\n", mmc_hostname(host->mmc));
	return 0;
}

static void xlp_hw_reset(struct sdhci_host *host)
{
	u8 data = xlp_readb(host, SDHCI_POWER_CONTROL);

	if(!(data & SDHCI_POWER_ON)) return;

	SDHCI_DEBUG("Issuing hardware reset to %s\n", mmc_hostname(host->mmc));
	xlp_writeb(host, data | SDHCI_POWER_RESET, SDHCI_POWER_CONTROL);
	udelay(10);		// eMMC minimum 1us
	xlp_writeb(host, data, SDHCI_POWER_CONTROL);

	usleep_range(300, 1000);	// eMMC minimum 200us
}

/* Host clock control functions - see above quirk
 * For this to get called, the SDClkBF field in the capabilities
 * register must be 0 (the default), which tells the host to get
 * the max clock via some other means.
 */
static unsigned int xlp_get_max_clock(struct sdhci_host *host)
{
	unsigned int max_clk;

	/* Retrieve the SD/MMC controller SOC reference clock */
	if(is_nlm_xlp2xx())
		max_clk = nlm_hal_get_soc_freq(NODE_0, XLP2XX_CLKDEVICE_MMC);
	else
		max_clk = nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_MMC);
		
	/* 4.2.4 controller assumes the clock is multiplied by the value + 1
	 * specified in the capabilities register ClkMul field (we set this
	 * to 1 since 0 means programmable clock is not supported). Thus the
	 * core assumes the clock is multiplied by two. However this is not
	 * the case with XLP so instead give the core 1/2 the actual clock.
	 */
	max_clk /= 2;
	SDHCI_DEBUG("SDHCI Clock Control: Returning max clock = %d MHz\n",
			max_clk / 1000 / 1000);
	return max_clk;
}

static void xlp_setup_hc(struct pci_dev *pdev)
{
	u16 sysctrl;
	u32 capcfg0, capcfg1;

	capcfg0 = XLP_SDHCI_CAPABILITIES;
	/* Cap 1: Bit 1 = SDR50 support, bit 13 = Programmable Clock Multiplier support */
	capcfg1 = (1 << 0) | (1 << 13);

	pci_write_config_dword(pdev, HC_MMC_CAPCFG0_S0, capcfg0 | (slot0_is_emmc << 27));
	pci_write_config_dword(pdev, HC_MMC_CAPCFG1_S0, capcfg1);

	/* Set-up capabilities registers for slot 1 (even if not used) */
	pci_write_config_dword(pdev, HC_MMC_CAPCFG0_S1, capcfg0 | (slot1_is_emmc << 27));
	pci_write_config_dword(pdev, HC_MMC_CAPCFG1_S1, capcfg1);

	/* Enable host controller */
	sysctrl = HC_ENABLE_SLOT_0 | HC_ENABLE_L3_ALLOCATE;
	if(slot0_read_only) sysctrl |= HC_WRITE_PROT_S0;
	if(slot1_enable)    sysctrl |= HC_ENABLE_SLOT_1;
	if(slot1_read_only) sysctrl |= HC_WRITE_PROT_S1;
	pci_write_config_dword(pdev, HC_SYSCTRL, sysctrl);
	msleep(5);
}

/* Callbacks for the 4.2.4 MMC core. Have to populate the
 * following call-backs even if not providing them in this
 * driver (sdhci.c version is listed on the right):
 *	reset				sdhci_reset
 *	set_bus_width		sdhci_set_bus_width
 *	set_clock			sdhci_set_clock
 *	set_uhs_signaling	sdhci_set_uhs_signaling
 *
 * The remaining callbacks in the ops struct are optional.
 */
static struct sdhci_ops xlp_sdhci_ops = {
	.reset             = sdhci_reset,
	.set_clock         = sdhci_set_clock,
	.set_bus_width     = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.hw_reset          = xlp_hw_reset,
	.enable_dma        = xlp_enable_dma,
	.get_max_clock     = xlp_get_max_clock,
	.write_l           = xlp_writel,
	.write_w           = xlp_writew,
	.write_b           = xlp_writeb,
	.read_l            = xlp_readl,
	.read_w            = xlp_readw,
	.read_b            = xlp_readb
};

/* Per-slot host set-up */
static int probe_slot(struct xlp_sdhci_chip *chip,
				struct pci_dev *pdev, int sltno)
{
	struct sdhci_host *sd_host = NULL;
	int ret;

	SDHCI_DEBUG("%s for slot number %d\n", __func__, sltno);

	/* We don't need sdhci_alloc_host to allocate space for our private
	 * data (struct xlp_sdhci_chip) since that is done elsewhere. However
	 * we DO want to save a pointer to the PCI device struct so we need to
	 * allocate at least enough bytes for a pointer. Since the private
	 * data area is cacheline aligned in the struct definition, might as
	 * well have sdhci_alloc_host allocate an entire cache line.
	 */
	sd_host = sdhci_alloc_host(&pdev->dev, XLP_CACHELINE_SIZE);
	if (IS_ERR(sd_host)) {
		dev_err(&pdev->dev, "sdhci_alloc_host failed for slot %d\n", sltno);
		return PTR_ERR(sd_host);
	}
	SDHCI_DEBUG_VERBOSE("  Allocated host struct at 0x%p\n", sd_host);

	sd_host->private[0] = (unsigned long)pdev;
	sd_host->ioaddr = chip->ioaddr + XLP_PCIE_HDR_OFFSET + (sltno * XLP_SLOT_SIZE);
	sd_host->hw_name = "xlp-sdhci";
	sd_host->ops = &xlp_sdhci_ops;
	sd_host->irq = pdev->irq;
	sd_host->dma_mask = DMA_BIT_MASK(32);

	/* Slight difference in quirks and capabilities if the slot is
	 * configured for an eMMC device vs. removable. XLP supports
	 * CMD14/CMD19 and hardware reset pin to SD/MMC card.
	 */
	if(((sltno == 0) && slot0_is_emmc) || ((sltno == 1) && slot1_is_emmc)) {
		sd_host->mmc->caps |= MMC_CAP_NONREMOVABLE;
		sd_host->quirks = XLP_SDHCI_QUIRKS_EMMC;
	} else {
		sd_host->quirks  = XLP_SDHCI_QUIRKS;
	}
	sd_host->quirks2 = XLP_SDHCI_QUIRKS2;
	sd_host->mmc->caps |= MMC_CAP_HW_RESET | MMC_CAP_BUS_WIDTH_TEST;

	SDHCI_DEBUG_VERBOSE("  Slot Virt Base = 0x%p\n", sd_host->ioaddr);
	SDHCI_DEBUG("  Capabilities register 0 val = 0x%08X\n",
			xlp_readl(sd_host, SDHCI_CAPABILITIES));
	SDHCI_DEBUG("  Capabilities register 1 val = 0x%08X\n",
			xlp_readl(sd_host, SDHCI_CAPABILITIES + 4));
	SDHCI_DEBUG("  Present state register val  = 0x%08X\n",
			xlp_readl(sd_host, SDHCI_PRESENT_STATE));

	/* Registering host */
	SDHCI_DEBUG("  Calling sdhci_add_host\n");
	ret = sdhci_add_host(sd_host);
	if (ret) {
		sdhci_free_host(sd_host);
		dev_err(&pdev->dev, "sdhci_add_host() failed for slot %d\n", sltno);
		return ret;
	}
	chip->host[sltno] = sd_host;

	printk("Initialized host driver for %s\n", mmc_hostname(sd_host->mmc));

	return 0;
}

/* Probe routine called when the PCI driver is instantiated */
static int __devinit sdhci_xlp_probe(struct pci_dev *pdev,
									const struct pci_device_id *id)
{
	struct xlp_sdhci_chip *chip = NULL;
	u32 irt;
	unsigned int irq;
	int ret = 0, slotno;
	void __iomem *base;
	unsigned long long physbase;
	unsigned short pcie_devn = PCI_SLOT(pdev->devfn);
	unsigned short pcie_fn   = PCI_FUNC(pdev->devfn);

	printk("Broadcom XLP SDHCI Driver\n");
	printk(KDBG "  Number of slots: %d\n", slot1_enable ? 2 : 1);
	printk(KDBG "  PCI-e Vendor 0x%04X, Device ID = 0x%04X\n",
			pdev->vendor, pdev->device);
	printk(KDBG "  PCI-e Bus 0, Device %d, Function %d\n", pcie_devn, pcie_fn);
	if(slot0_is_emmc)	printk(KDBG "  Slot 0 type set to non-removable / eMMC\n");
	if(slot0_read_only)	printk(KDBG "  Slot 0 write protected\n");
	if(slot1_enable || slot1_read_only)
						printk(KDBG "  Slot 1 write protected\n");

	/* Enable the PCI device */
	SDHCI_DEBUG("  Calling pci_enable_device\n");
	ret = pci_enable_device(pdev);
	if(ret) {
		dev_err(&pdev->dev, "pci_enable_device() failed!\n");
		return ret;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(struct xlp_sdhci_chip), GFP_KERNEL);
	if(chip == NULL) {
		dev_err(&pdev->dev, "can't allocate memory\n");
		ret = -ENOMEM;
		goto perr;
	}
	SDHCI_DEBUG_VERBOSE("  Allocated chip struct at 0x%p\n", chip);
	chip->num_slots = slot1_enable ? 2 : 1;

	/* Get I/O memory region for this device by bus, device, function */
	physbase = nlm_hal_get_dev_base(NODE_0, BUS_0, pcie_devn, pcie_fn);
	SDHCI_DEBUG("  PCI-E config phys base = 0x%llx\n", physbase);
	base = xlp_pci_config_base + (physbase - XLP_PCI_ECONFIG_BASE);
	SDHCI_DEBUG("    Virtual base = 0x%p\n", base);
	chip->ioaddr = base;

	/* Get PIC IRT entry from PCI-e header, convert to IRQ */
	pci_read_config_dword(pdev, XLP_PCIE_DEV_IRT_INFO, &irt);
	irt &= 0xFFFF;
	irq = xlp_irt_to_irq(NODE_0, irt);
 	SDHCI_DEBUG("  IRT = %u ==> IRQ %u\n", irt, irq);
	pdev->irq = irq;

	/* Save the I/O memory region in the pdev
	 * struct so we can free it later.
	 */
	pdev->resource[0].flags = IORESOURCE_MEM;
	pdev->resource[0].start = physbase;
	pdev->resource[0].end   = physbase + 0xFFF;

	SDHCI_DEBUG("  Requesting PCI I/O memory region\n");
	ret = pci_request_region(pdev, 0, sdhci_xlp_driver.name);
	if (ret) dev_warn(&pdev->dev, "can't request region\n");

	pci_set_drvdata(pdev, chip);

	/* Enable host controller slot(s) and set write protect */
	xlp_setup_hc(pdev);
	for (slotno = 0; slotno < chip->num_slots; slotno++) {
		ret = probe_slot(chip, pdev, slotno);
		if(ret) break;
	}

	if(!ret) return 0;

	/* Failed to enable one or both slots - clean-up the driver */
	/* Disable both slots and place the host controller in reset */
	pci_write_config_dword(pdev, HC_SYSCTRL, HC_RESET | HC_CLK_DISABLE);
	pci_set_drvdata(pdev, NULL);
	SDHCI_DEBUG("  Calling pci_release_region\n");
	pci_release_region(pdev, 0);
	SDHCI_DEBUG_VERBOSE("  Calling devm_kfree for chip struct at 0x%p\n", chip);
	devm_kfree(&pdev->dev, chip);
perr:
	SDHCI_DEBUG("  Calling pci_disable_device\n");
	pci_disable_device(pdev);

	return ret;
}

static void __devexit sdhci_xlp_remove(struct pci_dev *pdev)
{
	struct xlp_sdhci_chip *chip;
	struct sdhci_host *host;
	int slotno, dead;
	u32 scratch;

	SDHCI_DEBUG("%s enter\n", __func__);

	chip = pci_get_drvdata(pdev);
	SDHCI_DEBUG_VERBOSE("  chip struct at 0x%p\n", chip);
	if (chip) {
		for (slotno = 0; slotno < chip->num_slots; slotno++) {
			host = chip->host[slotno];
			SDHCI_DEBUG("  Removing slot %d host \n", slotno);
			SDHCI_DEBUG_VERBOSE("    Host struct at 0x%p\n", host);
			if (host == NULL) continue;

			SDHCI_DEBUG("    Checking if host has died...");
			scratch = xlp_readl(host, SDHCI_INT_STATUS);
			dead = (scratch == (u32)-1) ? 1 : 0;
			SDHCI_DEBUG("%s\n", dead ? "yes" : "no");
			SDHCI_DEBUG("    Calling sdhci_remove_host\n");
			sdhci_remove_host(host, dead);
			SDHCI_DEBUG("    Calling sdhci_free_host\n");
			sdhci_free_host(host);
		}

		/* Disable both slots and place the host controller in reset */
		pci_write_config_dword(pdev, HC_SYSCTRL, HC_RESET | HC_CLK_DISABLE);

		/* Clean-up */
		SDHCI_DEBUG_VERBOSE("  Calling devm_kfree for chip struct at 0x%p\n", chip);
		devm_kfree(&pdev->dev, chip);
	}
	pci_set_drvdata(pdev, NULL);
	SDHCI_DEBUG("  Calling pci_release_region\n");
	pci_release_region(pdev, 0);
	SDHCI_DEBUG("  Calling pci_disable_device\n");
	pci_disable_device(pdev);
}

/* PCI-e enumeration stuff */
static const struct pci_device_id xlp_sdhci_pci_ids[] __devinitdata = {
	{
		.vendor         = PCI_NETL_VENDOR,
		.device         = XLP_DEVID_MMC,
		.subvendor      = PCI_ANY_ID,
		.subdevice      = PCI_ANY_ID,
	},
	{ /* End - all zeros */ }
};

static struct pci_driver sdhci_xlp_driver = {
	.name		= "sdhci-xlp",
	.id_table	= xlp_sdhci_pci_ids,
	.probe		= sdhci_xlp_probe,
	.remove		= __devexit_p(sdhci_xlp_remove)
};

static int __devinit sdhci_xlp_init(void)
{
	return pci_register_driver(&sdhci_xlp_driver);
}

static void __devexit sdhci_xlp_exit(void)
{
	pci_unregister_driver(&sdhci_xlp_driver);
}

module_init(sdhci_xlp_init);
module_exit(sdhci_xlp_exit);
MODULE_AUTHOR("Lewis Carroll <lcarroll@broadcom.com>");
MODULE_DESCRIPTION("SDHCI PCI Driver for Broadcom XLP SoC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("xlp-sdhci");
