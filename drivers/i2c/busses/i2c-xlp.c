/*-
 * Copyright (c) 2003-2015 Broadcom Corporation
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

/* I2C PCI device driver for Broadcom XLP SOCs */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/i2c-xlp.h>

/* Debug options */
//#define XLP_I2C_DEBUG			// Enable PCI driver-level debug output
//#define XLP_I2C_DBG_REGS		// Show low-level register reads/writes

/* Module parameters */
static int i2c_speed = 45; /* KHz */
module_param(i2c_speed, int, 0);
MODULE_PARM_DESC(i2c_speed, " Specify I2C bus speed in KHz (default is 45 KHz)");


#define I2C_CLKFREQ_HZ			133333333 /* 133.333 MHz */
#define I2C_TIMEOUT				500000
#define XLP2XX_MAX_BUSSES		4
#define XLP1XX_MAX_BUSSES		2
#define XLP2XX_I2C_BUS_OFFSET	0x020

#ifdef XLP_I2C_DEBUG
#define KDBG					KERN_INFO
#define I2C_XLP_DBG				printk
#else
#define KDBG					KERN_DEBUG
#define I2C_XLP_DBG(...)
#endif

#ifdef XLP_I2C_DBG_REGS
#define I2C_XLP_DBG_REG			printk
#else
#define I2C_XLP_DBG_REG(...)
#endif

static struct pci_driver xlp_smbus_driver;

struct i2c_xlp_chip {
	struct i2c_xlp_data    *priv[XLP2XX_MAX_BUSSES];
	void __iomem           *iobase;
	uint16_t                num_busses;
};

static inline uint32_t i2c_reg_read(struct i2c_xlp_data *priv, int regidx)
{
	volatile uint32_t *mmio = (volatile uint32_t *)priv->iobase;
	uint32_t data;

	I2C_XLP_DBG_REG("  Reading from 0x%p (reg %d)....", &mmio[regidx], regidx);
	data = mmio[regidx];
	I2C_XLP_DBG_REG(KERN_CONT "0x%08x\n", data);
	return data;
}

static inline void i2c_reg_write(struct i2c_xlp_data *priv, int regidx, uint32_t val)
{
	volatile uint32_t *mmio = (volatile uint32_t *)priv->iobase;

	I2C_XLP_DBG_REG("  Writing  to  0x%p (reg %d) <- 0x%08x\n",
			&mmio[regidx], regidx, val);
	*mmio = val;
}

/* These are called with a register index from 0-5.
 * Registers are 32b and we want the 8 LSb.
 */
static inline uint8_t i2c_read_reg8(struct i2c_xlp_data *priv, int regidx)
{
	volatile uint8_t *mmio = (volatile uint8_t *)(priv->iobase + (regidx << 2));
	uint8_t data;

	I2C_XLP_DBG_REG("  Reading from 0x%p (reg %d)....", &mmio[3], regidx);
	data = mmio[3];
	I2C_XLP_DBG_REG(KERN_CONT "0x%02x\n", data);
	return data;
}

static void i2c_write_reg8(struct i2c_xlp_data *priv, int regidx, uint8_t value)
{
	volatile uint8_t *mmio = (volatile uint8_t *)(priv->iobase + (regidx << 2));

	I2C_XLP_DBG_REG("  Writing  to  0x%p (reg %d) <- 0x%02x\n",
			&mmio[3], regidx, value);
	mmio[3] = value;
}

static int wait_xfer_done(struct i2c_xlp_data *priv)
{
	int timeout = I2C_TIMEOUT;

	while ((i2c_read_reg8(priv, XLP_I2C_STATUS) & XLP_I2C_STATUS_TIP) && timeout) {
		timeout--;
	}
	if (timeout == 0) {
		printk(KERN_NOTICE "%s.%d timed out Waiting for TIP to clear\n",
				priv->adap.name, priv->adap.nr);
		return -1;
	}
	return 0;
}

static int bus_idle(struct i2c_xlp_data *priv)
{
	int timeout = I2C_TIMEOUT;

	while ((i2c_read_reg8(priv, XLP_I2C_STATUS) & XLP_I2C_STATUS_BUSY) && timeout) {
		timeout--;
	}
	if (timeout == 0) {
		printk(KERN_NOTICE "%s.%d timed out waiting for bus busy to clear\n",
				priv->adap.name, priv->adap.nr);
		return -1;
	}
	return 0;
}

static int wait_ack(struct i2c_xlp_data *priv)
{
	if (i2c_read_reg8(priv, XLP_I2C_STATUS) & XLP_I2C_STATUS_NACK) {
		return -1;
	}
	return 0;
}

int xlp_i2c_read(struct i2c_xlp_data *priv, uint8_t slave_addr, uint32_t slave_offset,
		int alen, int len, uint8_t *data)
{
	int i;

	/* Verify the driver was initialized */
	if(!priv) {
		printk(KERN_WARNING "%s: driver not initialized - aborting\n", __func__);
		return -1;
	}

	/* Verify the bus is idle */
	if (i2c_read_reg8(priv, XLP_I2C_STATUS) & XLP_I2C_STATUS_BUSY) {
		printk(KERN_NOTICE "%s.%d busy - aborting\n",
				priv->adap.name, priv->adap.nr);
		goto i2c_rx_error;
	}

	i2c_write_reg8(priv, XLP_I2C_DATA, (slave_addr << 1) | XLP_WRITE_BIT);
	i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_START);
	if (wait_xfer_done(priv) < 0) {
		goto i2c_rx_error;
	}
	if (wait_ack(priv) < 0) {
		goto i2c_rx_error;
	}

	/* Verify Arbitration is not Lost */
	if (i2c_read_reg8(priv, XLP_I2C_STATUS) & XLP_I2C_STATUS_AL) {
		printk(KERN_NOTICE "%s.%d arbitration lost - aborting\n",
				priv->adap.name, priv->adap.nr);
		goto i2c_rx_error;
	}

	for (i = 0; i<=alen; i++) {
		i2c_write_reg8(priv, XLP_I2C_DATA, ( (slave_offset >> (i*8) ) & 0xff) | XLP_WRITE_BIT);
		i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_WRITE);
		if (wait_xfer_done(priv) < 0) {
			goto i2c_rx_error;
		}
		if (wait_ack(priv) < 0) {
			goto i2c_rx_error;
		}
	}

	/* Address Phase Done, Data Phase begins */
	i2c_write_reg8(priv, XLP_I2C_DATA, (slave_addr << 1) | XLP_READ_BIT);
	i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_START);
	if (wait_xfer_done(priv) < 0) {
		goto i2c_rx_error;
	}
	if (wait_ack(priv) < 0) {
		goto i2c_rx_error;
	}
	if (len > 1) {
		int bytenr = 0;

		for (bytenr = 0; bytenr < len-1; bytenr++) {
			i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_READ);
			if (wait_xfer_done(priv) < 0) {
				goto i2c_rx_error;
			}
			if (data != NULL) {
				*data = i2c_read_reg8(priv, XLP_I2C_DATA);
				data++;
			}
		}
	}

	/* Last (or only) Byte: Set RD, NACK, STOP Bits */
	i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_STOP | XLP_I2C_CMD_RDNACK);
	if (wait_xfer_done(priv) < 0) {
		goto i2c_rx_error;
	}
	if(data != NULL)
		*data = i2c_read_reg8(priv, XLP_I2C_DATA);
	return bus_idle(priv);

i2c_rx_error:
	/* Release Bus */
	i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_STOP);
	bus_idle(priv);
	return -1;
}
EXPORT_SYMBOL(xlp_i2c_read);

int xlp_i2c_write(struct i2c_xlp_data *priv, uint8_t slave_addr, uint32_t slave_offset, int alen,
		int len, uint8_t *data)
{
	int i;

	/* Verify the driver was initialized */
	if(!priv) {
		printk(KERN_WARNING "%s: driver not initialized - aborting\n", __func__);
		return -1;
	}

	/* Verify the bus is idle */
	if (i2c_read_reg8(priv, XLP_I2C_STATUS) & XLP_I2C_STATUS_BUSY) {
		printk(KERN_NOTICE "%s.%d busy - aborting\n",
				priv->adap.name, priv->adap.nr);
		goto i2c_tx_error;
	}

	i2c_write_reg8(priv, XLP_I2C_DATA, (slave_addr << 1) | XLP_WRITE_BIT);
	i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_START);
	if (wait_xfer_done(priv) < 0) {
		goto i2c_tx_error;
	}
	if (wait_ack(priv) < 0) {
		goto i2c_tx_error;
	}

	/* Verify Arbitration is not Lost */
	if (i2c_read_reg8(priv, XLP_I2C_STATUS) & XLP_I2C_STATUS_AL) {
		printk(KERN_NOTICE "%s.%d arbitration lost - aborting\n",
				priv->adap.name, priv->adap.nr);
		goto i2c_tx_error;
	}

	for (i = 0; i<=alen; i++) {
		i2c_write_reg8(priv, XLP_I2C_DATA, ( (slave_offset >> (i*8) ) & 0xff) | XLP_WRITE_BIT);
		i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_WRITE);
		if (wait_xfer_done(priv) < 0) {
			goto i2c_tx_error;
		}
		if (wait_ack(priv) < 0) {
			goto i2c_tx_error;
		}
	}

	if (len > 1) {
		int bytenr = 0;

		for (bytenr = 0; bytenr < len-1; bytenr++) {
			i2c_write_reg8(priv, XLP_I2C_DATA, *data);
			i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_WRITE);
			if (wait_xfer_done(priv) < 0) {
				goto i2c_tx_error;
			}
			data++;
		}
	}

	i2c_write_reg8(priv, XLP_I2C_DATA, *data);
	i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_STOP | XLP_I2C_CMD_WRITE);
	if (wait_xfer_done(priv) < 0) {
		goto i2c_tx_error;
	}
	if (wait_ack(priv) < 0) {
		goto i2c_tx_error;
	}

	return bus_idle(priv);

i2c_tx_error:
	/* Release Bus */
	i2c_write_reg8(priv, XLP_I2C_COMMAND, XLP_I2C_CMD_STOP);
	bus_idle(priv);
	return -1;
}
EXPORT_SYMBOL(xlp_i2c_write);

static int xlp_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct i2c_xlp_data *priv = adap->algo_data;
	struct i2c_msg *p;
	int err = 0, command, len;

	/* Verify the driver was initialized */
	if(!priv) {
		printk(KERN_WARNING "%s: driver not initialized - aborting\n", __func__);
		return -1;
	}

	command = msgs[0].buf[0];
	if(num == 1) {
		p = &msgs[0];
		if(p && p->len == 0){
			len = 1;
		}
		else if (p && (p->flags & I2C_M_RD)){
			len = p->len;
		}
		else{
			len = p->len - 1;
		}
	}
	else if(num == 2) {
		p = &msgs[1];
		len = p->len;
	}
	else {
		printk("ERR: msg num =%d large than 2\n", num);
		return -1;
	}
	if (p->flags & I2C_M_RD)
		err = xlp_i2c_read(priv, p->addr, command, 0, len, &p->buf[0]);
	else
		err = xlp_i2c_write(priv, p->addr, command, 0, len, &p->buf[1]);

	/* Return the number of messages processed, or the error code.
	*/
	if (err == 0)
		err = num;


	return err;
}

static int xlp_i2c_smbus_xfer(struct i2c_adapter *adap, u16 addr, unsigned short flags,
		char read_write, u8 command, int protocol, union i2c_smbus_data *data )
{
	struct i2c_xlp_data *priv = adap->algo_data;
	int err;
	int len;

	/* Verify the driver was initialized */
	if(!priv) {
		printk(KERN_WARNING "%s: driver not initialized - aborting\n", __func__);
		return -1;
	}

	switch(protocol)
	{
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ)
			err = xlp_i2c_read(priv, addr, command, 0, 1, &data->byte);
		else
			err = xlp_i2c_write(priv, addr, command, 0, 1, &command);

		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ)
			err = xlp_i2c_read(priv, addr, command, 0, 1, &data->byte);
		else
			err = xlp_i2c_write(priv, addr, command, 0, 1, &data->byte);
		break;

	case I2C_SMBUS_WORD_DATA:
	case I2C_SMBUS_PROC_CALL:
		if (read_write == I2C_SMBUS_READ)
			err = xlp_i2c_read(priv, addr, command, 0, 2, (u8 *)&data->word);
		else
			err = xlp_i2c_write(priv, addr, command, 0, 2, (u8 *)&data->word);

		break;
	case I2C_FUNC_SMBUS_BLOCK_DATA:
	case I2C_SMBUS_I2C_BLOCK_DATA:
		len = (data->block[0] > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX: data->block[0];
		if (read_write == I2C_SMBUS_READ)
			err = xlp_i2c_read(priv, addr, command, 0, len, &data->block[1]);
		else
			err = xlp_i2c_write(priv, addr, command, 0, len, &data->block[1]);

		break;
	default:
		err = -1;

	}
	return err;
}

static uint32_t xlp_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm xlp_i2c_algo = {
	.master_xfer	= xlp_i2c_xfer,
	.smbus_xfer		= xlp_i2c_smbus_xfer,
	.functionality	= xlp_i2c_func,
};

static void i2c_xlp_setup(struct i2c_xlp_data *priv)
{
	uint32_t prescaler = (I2C_CLKFREQ_HZ / (5 * (i2c_speed * 1000))) - 1;

	/* disable I2C before setting the prescaler values */
	i2c_reg_write(priv, XLP_I2C_CONTROL, 0);
	
	/* set prescaler values*/	
	i2c_reg_write(priv, XLP_PRESCALE0, prescaler & 0xff);
	i2c_reg_write(priv, XLP_PRESCALE1, prescaler >> 8);
	
	/* re-enable I2C */
	i2c_reg_write(priv, XLP_I2C_CONTROL, XLP_I2C_CTRL_EN);	
}

static void i2c_xlp_disable(struct i2c_xlp_data *priv)
{
	i2c_reg_write(priv, XLP_I2C_CONTROL, 0);	
}

/* XLP1xx/2xx use a common PCI-e function (and thus common I/O area) with different
 * register addresses for each bus. XLP3xx/XLP8xx use different PCI-e functions for
 * the two busses. In 3xx/8xx, I2C lives in device 6 and functions 2 and 3
 * correspond to interfaces 0 and 1. In XLP2XX, I2C is device 6 function 7, and
 * register addresses are offset by 8 x interface(bus).
 *
 * XLP EVBs use bus 1 for a dedicated EEPROM for MAC address, and bus 0 is unused.
 */
static int __devinit i2c_xlp_probe(struct pci_dev *pdev,
									const struct pci_device_id *id)
{
	struct i2c_xlp_chip *chip;
	void __iomem *iobase;
	unsigned long long physbase;
	int ret, num_busses, bus;
	static int bus_id = 0;
	unsigned short pcie_devn = PCI_SLOT(pdev->devfn);
	unsigned short pcie_fn   = PCI_FUNC(pdev->devfn);

	printk("Broadcom XLP I2C Driver\n");
	printk(KDBG "  PCI-e Vendor 0x%04X, Device ID = 0x%04X\n",
			pdev->vendor, pdev->device);
	printk(KDBG "  PCI-e Bus 0, Device %d, Function %d\n", pcie_devn, pcie_fn);

	if(is_nlm_xlp1xx())      num_busses = XLP1XX_MAX_BUSSES;
	else if(is_nlm_xlp2xx()) num_busses = XLP2XX_MAX_BUSSES;
	else                     num_busses = 1;

	I2C_XLP_DBG("  Calling pci_enable_device\n");
	ret = pci_enable_device(pdev);
	if(ret) {
		dev_err(&pdev->dev, "pci_enable_device() failed!\n");
		return ret;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(struct i2c_xlp_chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&pdev->dev, "Can't allocate memory\n");
		ret = -ENOMEM;
		goto perr;
	}
	I2C_XLP_DBG("  Allocated chip struct at 0x%p\n", chip);
	chip->num_busses = num_busses;

	/* Get I/O memory region for this device by bus, device, function */
	physbase = nlm_hal_get_dev_base(NODE_0, BUS_0, pcie_devn, pcie_fn);
	I2C_XLP_DBG("  PCI-E cibfug ogys base = 0x%llx\n", physbase);
	iobase = xlp_pci_config_base + (physbase - XLP_PCI_ECONFIG_BASE);
	I2C_XLP_DBG("    Virtual base = 0x%p\n", iobase);
	chip->iobase = iobase;

	/* Save the I/O memory region in the pdev
	 * struct so we can free it later.
	 */
	pdev->resource[0].flags = IORESOURCE_MEM;
	pdev->resource[0].start = physbase;
	pdev->resource[0].end   = physbase + XLP_PCIE_REGION_SIZE - 1;

	I2C_XLP_DBG("  Requesting PCI I/O memory region\n");
	ret = pci_request_region(pdev, 0, xlp_smbus_driver.name);
	if (ret) dev_warn(&pdev->dev, "can't request region\n");

	pci_set_drvdata(pdev, chip);

	for(bus = 0; bus < num_busses; bus++) {
		struct i2c_xlp_data *priv = NULL;
		I2C_XLP_DBG("Adding adapter for bus %d\n", is_nlm_xlp2xx() ? bus : bus_id);
		priv = devm_kzalloc(&pdev->dev, sizeof(struct i2c_xlp_data), GFP_KERNEL);
		if (!priv) {
			dev_err(&pdev->dev, "Can't allocate memory\n");
			ret = -ENOMEM;
			break;
		}
		I2C_XLP_DBG("  Allocated adapter private data at 0x%p\n", priv);

		/* Calculate the register I/O base */
		priv->iobase = iobase + XLP_PCIE_HDR_OFFSET;

		/* Set-up the I2C adapter struct. Note adap.nr is the bus number. No idea
		 * what adap.id is used for.
		 */
		priv->adap.owner = THIS_MODULE;
		priv->adap.dev.parent = &pdev->dev;
		priv->adap.algo = &xlp_i2c_algo;
		priv->adap.algo_data = priv;
		priv->adap.timeout = 200;	// In jiffies
		strlcpy(priv->adap.name, xlp_smbus_driver.name, sizeof(priv->adap.name));
		if(is_nlm_xlp2xx()) {
			priv->adap.nr = bus;
			priv->iobase += (bus * XLP2XX_I2C_BUS_OFFSET);
		} else {
			priv->adap.nr = bus_id;
			bus_id++;
		}

		I2C_XLP_DBG("   Bus Virt Base = 0x%p\n", priv->iobase);

		/* Now, set up the PSC for SMBus PIO mode */
		i2c_xlp_setup(priv);

		I2C_XLP_DBG("  Calling i2c_add_numbered_adapter\n");
		ret = i2c_add_numbered_adapter(&priv->adap);
		if (ret) {
			dev_err(&pdev->dev, "i2c_add_numbered_adapter failed for adapter %d (err = %d)\n",
					bus, ret);
			i2c_xlp_disable(priv);
			devm_kfree(&pdev->dev, priv);
			break;
		}
		chip->priv[bus] = priv;

		printk("Initialized host driver for %s.%d\n",
				xlp_smbus_driver.name, priv->adap.nr);
	}

	if(!ret) return 0;

	/* Something failed - clean-up */
	for(bus = 0; bus < num_busses; bus++) {
		struct i2c_xlp_data *priv = chip->priv[bus];
		if(!priv) continue;
		i2c_del_adapter(&priv->adap);
		i2c_xlp_disable(priv);
		devm_kfree(&pdev->dev, priv);
	}

	pci_set_drvdata(pdev, NULL);
	I2C_XLP_DBG("  Calling pci_release_region\n");
	pci_release_region(pdev, 0);
	I2C_XLP_DBG("  Calling kfree for chip struct at 0x%p\n", chip);
	devm_kfree(&pdev->dev, chip);
perr:
	I2C_XLP_DBG("  Calling pci_disable_device\n");
	pci_disable_device(pdev);

	return ret;
}

static void __devexit i2c_xlp_remove(struct pci_dev *pdev)
{
	struct i2c_xlp_chip *chip;
	int bus;

	chip = pci_get_drvdata(pdev);
	if(chip) {
		for(bus = 0; bus < chip->num_busses; bus++) {
			struct i2c_xlp_data *priv = chip->priv[bus];
			if(!priv) continue;

			i2c_del_adapter(&priv->adap);
			i2c_xlp_disable(priv);
			devm_kfree(&pdev->dev, priv);
		}

		I2C_XLP_DBG("  Calling iounmap for virtual address 0x%p\n", chip->iobase);
		iounmap(chip->iobase);
		I2C_XLP_DBG("  Calling kfree for chip struct at 0x%p\n", chip);
		devm_kfree(&pdev->dev, chip);
	}
	pci_set_drvdata(pdev, NULL);
	I2C_XLP_DBG("  Calling pci_release_region\n");
	pci_release_region(pdev, 0);
	I2C_XLP_DBG("  Calling pci_disable_device\n");
	pci_disable_device(pdev);
}

/* PCI-e enumeration stuff */
static const struct pci_device_id xlp_i2c_pci_ids[] __devinitconst = {
	{
		.vendor         = PCI_NETL_VENDOR,
		.device         = XLP_DEVID_I2C,
		.subvendor      = PCI_ANY_ID,
		.subdevice      = PCI_ANY_ID,
	},
	{
		.vendor         = PCI_NETL_VENDOR,
		.device         = XLP2XX_DEVID_I2C,
		.subvendor      = PCI_ANY_ID,
		.subdevice      = PCI_ANY_ID,
	},
	{ /* End - all zeros */ }
};

static struct pci_driver xlp_smbus_driver = {
	.name		= "i2c-xlp",
	.id_table	= xlp_i2c_pci_ids,
	.probe		= i2c_xlp_probe,
	.remove		= __devexit_p(i2c_xlp_remove)
};

static int __init i2c_xlp_init(void)
{
	return pci_register_driver(&xlp_smbus_driver);
}

static void __exit i2c_xlp_exit(void)
{
	pci_unregister_driver(&xlp_smbus_driver);
}

module_init (i2c_xlp_init);
module_exit (i2c_xlp_exit);
MODULE_ALIAS("i2c-xlp");
MODULE_AUTHOR("Lewis Carroll <lcarroll@broadcom.com>");
MODULE_DESCRIPTION("I2C PCI Driver for Broadcom XLP SoC");
MODULE_LICENSE("GPL");
