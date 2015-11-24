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

#include <asm/netlogic/hal/nlm_eeprom.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>


#define I2C_CLKFREQ_HZ                 133333333 /* 133.333 MHz */
#define I2C_TIMEOUT                    500000

/* Low-level routines
 */
int my_read (u8 devaddr, uint addr, int alen, u8 * buf, int len);
int my_write (u8 devaddr, uint addr, int alen, u8 * buf, int len);
static inline u8 xlp_i2c_read_reg(int offset)
{
	volatile u32 *i2c_mmio;
#ifdef CONFIG_NLM_XMC_SUPPORT
	i2c_mmio = ioremap_nocache((nlm_hal_get_dev_base(0, 0, XLP_PCIE_GIO_DEV, XLP_GIO_I2C0_FUNC) +0x100),1000);
#else
	
	if(is_nlm_xlp2xx())
		i2c_mmio = ioremap_nocache((nlm_hal_get_dev_base(0, 0, XLP_PCIE_GIO_DEV,0x7/*func for xlp2xx*/) +0x100)+0x20,1000);
	else
		i2c_mmio = ioremap_nocache((nlm_hal_get_dev_base(0, 0, XLP_PCIE_GIO_DEV, XLP_GIO_I2C1_FUNC) +0x100),1000);
#endif
	return ((u8)i2c_mmio[offset]);

}

static inline void xlp_i2c_write_reg(int offset, u8 value) {

	volatile u32 *i2c_mmio;
#ifdef CONFIG_NLM_XMC_SUPPORT
	i2c_mmio =  ioremap_nocache((nlm_hal_get_dev_base(0, 0, XLP_PCIE_GIO_DEV, XLP_GIO_I2C0_FUNC) +0x100),1000);
#else
	if(is_nlm_xlp2xx())
		i2c_mmio =  ioremap_nocache((nlm_hal_get_dev_base(0, 0, XLP_PCIE_GIO_DEV,0x7/*func for xlp2xx*/) +0x100)+0x20,1000);
	else
                i2c_mmio = ioremap_nocache((nlm_hal_get_dev_base(0, 0, XLP_PCIE_GIO_DEV, XLP_GIO_I2C1_FUNC) +0x100),1000);
#endif
	i2c_mmio[offset] = value;
}

static inline int xaction_complete(void) {

	volatile int timeout = I2C_TIMEOUT;
	int retval = 0;

	while ((xlp_i2c_read_reg(XLP_I2C_STATUS) & XLP_I2C_STATUS_TIP) && timeout) {
		timeout--;
	}
	if (timeout == 0) {
		printk("Timed Out Waiting for TIP to Clear.\n");
		retval = -1;
	}
	return retval;
}

static inline int bus_idle(void) {

	volatile int timeout = I2C_TIMEOUT;
	int retval = 0;

	while ((xlp_i2c_read_reg(XLP_I2C_STATUS) & XLP_I2C_STATUS_BUSY) && timeout) {
		timeout--;
	}
	if (timeout == 0) {
		printk("Timed Out Waiting for Bus Busy to Clear.\n");
		retval = -1;
	}
	return retval;
}

static inline int check_for_ack(void) {

	if (xlp_i2c_read_reg(XLP_I2C_STATUS) & XLP_I2C_STATUS_NACK) {
		//printk("ACK not received from Slave Device.\n");
		return -1;
	}
	return 0;
}

/* ------------------------------------------
 * Sequence to Read byte(s) from an
 * offset within the Slave Device: -
 * ------------------------------------------
 * -- Set up 'DATA' with 'Slave Address + W'
 * -- Trigger the START Condition
 * 	-> Wait for Xfer complete & Slave Ack
 * -- Set up 'DATA' with 'Slave Offset + W'
 * -- Set the 'WRITE' bit in COMMAND Reg
 * 	-> Wait for Xfer complete & Slave Ack
 * -- Set up 'DATA' with 'Slave Address + R'
 * -- Trigger the (Re-)START Condition
 * 	-> Wait for Xfer complete & Slave Ack
 * -- For 'n-1' out of 'n' bytes,
 * 	-> Set 'READ' bit in COMMAND
 *	-> Keep Reading 'DATA' Register
 * -- For the last (or only) byte
 * 	-> Set 'READ/STOP/NACK' bits in COMMAND
 *	-> Read 'DATA' Register
 * ------------------------------------------
 */

int xlp_i2c_rx(u8 slave_addr, u32 slave_offset,
	      u8 alen, int len, u8 *data) {
	int i ;

//	printk("slave_addr=0x%x offset=0x%x alen=%d buff=0x%x len=%d\n", slave_addr, slave_offset, alen, data, len);

	/* Verify the bus is idle */
	if (xlp_i2c_read_reg(XLP_I2C_STATUS) & XLP_I2C_STATUS_BUSY) {
		printk("I2C Bus BUSY (Not Available), Aborting.\n");
		goto i2c_rx_error;
	}

	xlp_i2c_write_reg(XLP_I2C_DATA, (slave_addr << 1) | XLP_WRITE_BIT);
	xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_START);
	if (xaction_complete() < 0) {
		goto i2c_rx_error;
	}
	if (check_for_ack() < 0) {
		goto i2c_rx_error;
	}

	/* Verify Arbitration is not Lost */
	if (xlp_i2c_read_reg(XLP_I2C_STATUS) & XLP_I2C_STATUS_AL) {
		printk("I2C Bus Arbitration Lost, Aborting.\n");
		goto i2c_rx_error;
	}
	for (i = 0; i<=alen; i++) {
		xlp_i2c_write_reg(XLP_I2C_DATA, ( (slave_offset >> (i*8) ) & 0xff) | XLP_WRITE_BIT);
		xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_WRITE);
		if (xaction_complete() < 0) {
			goto i2c_rx_error;
		}
		if (check_for_ack() < 0) {
			goto i2c_rx_error;
		}
	}

	/* Address Phase Done, Data Phase begins
	 */
	xlp_i2c_write_reg(XLP_I2C_DATA, (slave_addr << 1) | XLP_READ_BIT);
	xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_START);
	if (xaction_complete() < 0) {
		goto i2c_rx_error;
	}
	if (check_for_ack() < 0) {
		goto i2c_rx_error;
	}

	if (len > 1) {

		int bytenr = 0;

		for (bytenr = 0; bytenr < len; bytenr++) {
			xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_READ);
			if (xaction_complete() < 0) {
				goto i2c_rx_error;
			}
			if (data != NULL) {
				*data = xlp_i2c_read_reg(XLP_I2C_DATA);
				//printk("data=0x%x\n", *data);
				data++;
			}
		}
	}

	/* Last (or only) Byte: -
	 * 	Set RD, NACK, STOP Bits
	 */
	xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_STOP | XLP_I2C_CMD_RDNACK);
	if (xaction_complete() < 0) {
		goto i2c_rx_error;
	}

	if (data != NULL) {
		*data = xlp_i2c_read_reg(XLP_I2C_DATA);
	}
	return bus_idle();

i2c_rx_error:
	/* Release Bus */
	xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_STOP);
	bus_idle();
	return -1;
}

/* Write byte(s) to the I2C Slave Device
 */
int xlp_i2c_tx(u8 slave_addr, u16 slave_offset, u8 alen,
		int len, u8 *data) {
	int i ;

	// printk("slave_addr=0x%x offset=0x%x alen=%d buff=0x%x len=%d\n", slave_addr, slave_offset, alen, data, len);

	/* Verify the bus is idle */
	if (xlp_i2c_read_reg(XLP_I2C_STATUS) & XLP_I2C_STATUS_BUSY) {
		printk("I2C Bus BUSY (Not Available), Aborting.\n");
		goto i2c_tx_error;
	}

	xlp_i2c_write_reg(XLP_I2C_DATA, (slave_addr << 1) | XLP_WRITE_BIT);
	xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_START);
	if (xaction_complete() < 0) {
		goto i2c_tx_error;
	}
	if (check_for_ack() < 0) {
		goto i2c_tx_error;
	}

	/* Verify Arbitration is not Lost */
	if (xlp_i2c_read_reg(XLP_I2C_STATUS) & XLP_I2C_STATUS_AL) {
		printk("I2C Bus Arbitration Lost, Aborting.\n");
		goto i2c_tx_error;
	}

	for (i = 0; i<=alen; i++) {
		xlp_i2c_write_reg(XLP_I2C_DATA, ( (slave_offset >> (i*8) ) & 0xff) | XLP_WRITE_BIT);
		xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_WRITE);
		if (xaction_complete() < 0) {
			goto i2c_tx_error;
		}
		if (check_for_ack() < 0) {
			goto i2c_tx_error;
		}
	}

	if (len > 1) {

		int bytenr = 0;

		for (bytenr = 0; bytenr < len; bytenr++) {
			xlp_i2c_write_reg(XLP_I2C_DATA, *data);
			xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_WRITE);
			if (xaction_complete() < 0) {
				goto i2c_tx_error;
			}
			data++;
		}
	}
	/* Last (or only) Byte: -
	 * 	Set WR, STOP Bits
	 */
	xlp_i2c_write_reg(XLP_I2C_DATA, *data);
	xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_STOP | XLP_I2C_CMD_WRITE);
	if (xaction_complete() < 0) {
		goto i2c_tx_error;
	}
	if (check_for_ack() < 0) {
		goto i2c_tx_error;
	}

	return bus_idle();

i2c_tx_error:
	/* Release Bus */
	xlp_i2c_write_reg(XLP_I2C_COMMAND, XLP_I2C_CMD_STOP);
	bus_idle();
	return -1;
}

/* Initialization for the Controller: -
 * -- Disable Controller/Interrupts
 * -- Program PreScaler
 * -- Enable Controller (Keep Intrs disabled)
 */
int eeprom_init (int speed)
{
	int i, prescaler;

	for (i=0; i<1; i++) {

		u8 ctrl_reg;

		prescaler = (I2C_CLKFREQ_HZ/(5 * speed)) - 1;
		ctrl_reg = xlp_i2c_read_reg(XLP_I2C_CONTROL);

		xlp_i2c_write_reg(XLP_I2C_CONTROL,
			     ctrl_reg & ~(XLP_I2C_CTRL_EN | XLP_I2C_CTRL_IEN));

		xlp_i2c_write_reg(XLP_PRESCALE0, prescaler & 0xff);
		xlp_i2c_write_reg(XLP_PRESCALE1, prescaler >> 8);

		xlp_i2c_write_reg(XLP_I2C_CONTROL,
			     xlp_i2c_read_reg(XLP_I2C_CONTROL) | XLP_I2C_CTRL_EN);

		if (xlp_i2c_read_reg(XLP_I2C_CONTROL) & XLP_I2C_CTRL_EN) {
			printk("Initialized I2C%d Controller.\n", i);
		} else {
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(eeprom_init);

int eeprom_read (u8 devaddr, uint addr, int alen, u8 * buf, int len)
{
	return xlp_i2c_rx(devaddr,
			 addr,
			 alen,
			 len,
			 buf);
}
EXPORT_SYMBOL(eeprom_read);

int eeprom_write (u8 devaddr, uint addr, int alen, u8 * buf, int len)
{
	return xlp_i2c_tx(devaddr,
			 addr,
			 alen,
			 len,
			 buf) ;
}
EXPORT_SYMBOL(eeprom_write);

int xlp_i2c_probe (u8 devaddr)
{
	return eeprom_read(devaddr,0,0,NULL,0) ;
}

