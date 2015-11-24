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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/i2c-xlp.h>

//#define XLP_I2C_DEBUG

#define I2C_CLKFREQ_HZ			133333333 /* 133.333 MHz */
#define I2C_TIMEOUT				500000
#define BUS_0					0

static struct resource xlp2xx_iomem;
static int i2c_speed = 45; /* KHz */
module_param(i2c_speed, int, 0);

/* Fix for XLP2XX.  In 8xx/4xx, I2C lives in device 6 and functions 2 and 3
 * correspond to interfaces 0 and 1 (i/f 1 / func 3 used on EVP).  In XLP2XX
 * I2C is device 6 function 7, and register addresses are offset by
 * 8 x interface (i/f 1 still used on EVP).  EVBs use bus 0 for a dedicated
 * EEPROM for MAC address, and bus 1 is used normally.  Define mmio as the
 * base address of the PCI-e function.  Offset register index for XLP2xx.
 */
static int32_t i2c_reg_read(int bus, int regidx)
{
	uint64_t mmio;
	if(is_nlm_xlp2xx()) {
		mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP2_PCIE_I2C_FUNC_7);
		return nlm_hal_read_32bit_reg(mmio, regidx + (bus << 3));
	} else {
		mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_I2C0_FUNC + bus);
		return nlm_hal_read_32bit_reg(mmio, regidx);
	}
}

static void i2c_reg_write(int bus, int regidx, int32_t val)
{
	uint64_t mmio;
	if (is_nlm_xlp2xx()) {
		mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP2_PCIE_I2C_FUNC_7);
		nlm_hal_write_32bit_reg(mmio, regidx + (bus << 3), val);
	} else {
		mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_I2C0_FUNC + bus);
		nlm_hal_write_32bit_reg(mmio, regidx, val);
	}
}

#ifdef XLP_I2C_DEBUG
static void i2c_dump_reg()
{
	int i, bus = 0;
	for(bus = 0; bus < 1; bus++) {
		printk("dump i2c_%d register\n", bus);
		for(i = 0; i < 6; i++) {
			printk("0x%0x = 0x%8x\n", i, i2c_reg_read(bus, i));
		}
                for(i = 0x3C; i < 0x42; i++) {
			printk("0x%0x = 0x%8x\n", i, i2c_reg_read(bus, i));
		}
	}
}
#endif

static uint8_t i2c_read_reg8(struct i2c_xlp_data *priv, int offset)
{
	volatile uint32_t *i2c_mmio = (u32*)priv->iobase;
	return ((uint8_t)i2c_mmio[offset]);
}

static void i2c_write_reg8(struct i2c_xlp_data *priv, int offset, uint8_t value)
{
	volatile uint32_t *i2c_mmio = (u32*)priv->iobase;
	i2c_mmio[offset] = value;
}

static int wait_xfer_done(struct i2c_xlp_data *priv)
{
	volatile int timeout = I2C_TIMEOUT;

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
	volatile int timeout = I2C_TIMEOUT;

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
	uint32_t val, prescaler;
	int bus = priv->bus;
	
	prescaler = (I2C_CLKFREQ_HZ/(5 * (priv->speed * 1000))) - 1;  /* (speed * 1000) - convert KHz to Hz */

	/* disable I2C before setting the prescaler values */
	val = i2c_reg_read(bus, XLP_I2C_CONTROL);
	val &= ~(XLP_I2C_CTRL_EN | XLP_I2C_CTRL_IEN);
	i2c_reg_write(bus, XLP_I2C_CONTROL, val);
	
	/* set prescaler values*/	
	i2c_reg_write(bus, XLP_PRESCALE0, prescaler & 0xff);
	i2c_reg_write(bus, XLP_PRESCALE1, prescaler >> 8);
	
	/* re-enable I2C */
	val &= ~0xFFFF;
	val |= XLP_I2C_CTRL_EN; 

	i2c_reg_write(bus, XLP_I2C_CONTROL, val);	
}

static void i2c_xlp_disable(struct i2c_xlp_data *priv)
{
	int32_t val;
	int bus = priv->bus;
	
	val = i2c_reg_read(bus, XLP_I2C_CONTROL);
	val &= ~XLP_I2C_CTRL_EN;
	i2c_reg_write(bus, XLP_I2C_CONTROL, val);	
}

/* XLP2XX uses a common PCI-e function (and thus common I/O area) with different
 * register addresses for each bus.  XLP3xx/XLP8xx use different PCI-e functions
 * for the two busses. */
static int __devinit i2c_xlp_probe(struct platform_device *pdev)
{
	struct i2c_xlp_data *priv;
	struct resource *ioarea;
	unsigned long iobase = 0;
	int ret;

#ifdef XLP_I2C_DEBUG
	i2c_dump_reg();	
#endif

	priv = kzalloc(sizeof(struct i2c_xlp_data), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	if(is_nlm_xlp2xx())
		ioarea = &xlp2xx_iomem;
	else {
		ioarea = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!ioarea) {
			dev_err(&pdev->dev, "No memory resource defined\n");
			kfree(priv);
			return -EINVAL;
		}
	}

	iobase = (unsigned long)ioremap_nocache(ioarea->start, PAGE_SIZE);
	if(!iobase) {
		dev_err(&pdev->dev, "ioremap failed\n");
		if(!is_nlm_xlp2xx())
			release_resource(ioarea);
		kfree(priv);
		return -ENOMEM;
	}

	if(is_nlm_xlp2xx())
		priv->iobase = iobase + 0x100 + (pdev->id << 5);
	else
		priv->iobase = iobase + 0x100;

	priv->ioarea = ioarea;
	priv->xfer_timeout = 200;
	priv->ack_timeout = 200;
	priv->bus = pdev->id;
	priv->adap.nr = pdev->id;
	priv->adap.algo = &xlp_i2c_algo;
	priv->adap.algo_data = priv;
	priv->adap.dev.parent = &pdev->dev;
	strlcpy(priv->adap.name, "i2c-xlp", sizeof(priv->adap.name));

	/* Now, set up the PSC for SMBus PIO mode.
	*/
	i2c_xlp_setup(priv);

	ret = i2c_add_numbered_adapter(&priv->adap);
	if (ret == 0) {
		platform_set_drvdata(pdev, priv);
		printk("Initializing XLP host driver for i2c-xlp.%d\n", pdev->id);
		return 0;
	}

	dev_err(&pdev->dev, "i2c_add_numbered_adapter failed for adapter %d (err = %d)\n",
			pdev->id, ret);
	i2c_xlp_disable(priv);
	iounmap((void *)(priv->iobase & ~0xFFF));
	if(!is_nlm_xlp2xx())
		release_resource(ioarea);
	kfree(priv);

	return ret;
}

static int __devexit i2c_xlp_remove(struct platform_device *pdev)
{
	struct i2c_xlp_data *priv = platform_get_drvdata(pdev);

	i2c_xlp_disable(priv);
	i2c_del_adapter(&priv->adap);
	iounmap((void *)(priv->iobase & ~0xFFF));
	if(!is_nlm_xlp2xx())
		release_resource(priv->ioarea);
	kfree(priv);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int i2c_xlp_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct i2c_xlp_data *priv = platform_get_drvdata(pdev);

	i2c_xlp_disable(priv);

	return 0;
}

static int i2c_xlp_resume(struct platform_device *pdev)
{
	struct i2c_xlp_data *priv = platform_get_drvdata(pdev);

	i2c_xlp_setup(priv);

	return 0;
}
#else
#define i2c_xlp_suspend	NULL
#define i2c_xlp_resume	NULL
#endif

static struct platform_driver xlp_smbus_driver = {
	.driver = {
		.name	= "i2c-xlp",
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_xlp_probe,
	.remove		= __devexit_p(i2c_xlp_remove),
	.suspend	= i2c_xlp_suspend,
	.resume		= i2c_xlp_resume,
};

static int __init i2c_xlp_init(void)
{
	/* XLP2xx I2C devices share I/O memory - so let the platform driver manage
	 * it instead of each platform device (I2C bus).
	 */
	if(is_nlm_xlp2xx()) {
		xlp2xx_iomem.start = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP2_PCIE_I2C_FUNC_7);
		xlp2xx_iomem.end   = xlp2xx_iomem.start + 0xFFF;
		xlp2xx_iomem.name  = xlp_smbus_driver.driver.name;
		xlp2xx_iomem.flags = IORESOURCE_MEM | IORESOURCE_BUSY;
		insert_resource(&iomem_resource, &xlp2xx_iomem);
	}
	return platform_driver_register(&xlp_smbus_driver);
}

static void __exit i2c_xlp_exit(void)
{
	if(is_nlm_xlp2xx())
		release_resource(&xlp2xx_iomem);
	platform_driver_unregister(&xlp_smbus_driver);
}

module_init (i2c_xlp_init);
module_exit (i2c_xlp_exit);
MODULE_ALIAS("platform:i2c-xlp");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("I2C adapter for XLP SoC");
MODULE_LICENSE("GPL");
