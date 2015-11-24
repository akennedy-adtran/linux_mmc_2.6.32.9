/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 *  i2c-algo-palm.c i2c driver algorithms for the BK3220 I2C Host 
 *  adapter on the RMI Phoenix System.
 *  Derived from the PCA-ISA I2C-Algo/Bus files.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-palm.h>
#include <asm/netlogic/i2c-algo-palm.h>

#define DRIVER "i2c-algo-palm"

#define DEB1(fmt, args...) do { if (i2c_debug>=1) printk(fmt, ## args); } while(0)
#define DEB2(fmt, args...) do { if (i2c_debug>=2) printk(fmt, ## args); } while(0)
#define DEB3(fmt, args...) do { if (i2c_debug>=3) printk(fmt, ## args); } while(0)

static int i2c_debug=0;
spinlock_t palm_lock;
					
#define palm_write(algo_data, reg, val) 	algo_data->write(reg, val)
#define palm_read(algo_data, reg) 		algo_data->read(reg)

#define palm_clock(adap) 		adap->get_clock(adap)
#define palm_status(adap) 		palm_inw(adap, I2C_PCA_STA)
#define palm_set_con(adap, val) 	palm_outw(adap, I2C_PCA_CON, val)
#define palm_get_con(adap) 		palm_inw(adap, I2C_PCA_CON)

/*
 * Check if the I2C Bus is idle or busy
 */
static int wait_for_idle(struct i2c_algo_palm_data *algo_data)
{
	int timeOut=0x1000;
	volatile __u32 regVal=0x00;
	regVal = palm_read(algo_data, I2C_PALM_STATUS) & 0x0001;
	while (regVal && timeOut--) {
		regVal = palm_read(algo_data, I2C_PALM_STATUS) & 0x0001;
	}
	if (timeOut == 0x00)
		return -1;	/* Timed Out */
	else
		return 0;
}


static int palm_rx(struct i2c_algo_palm_data *algo_data, __u8 *buf,
		__u16 addr, __u16 length)
{
	volatile __u32 tmp, regVal;
	int len = length, i;

	if(length == 0) /* special case of I2C_SMBUS_QUICK */ {
		/* issue a address only transaction */
		palm_write(algo_data, I2C_PALM_DEVADDR, addr);
		palm_write(algo_data, I2C_PALM_CFG,     0xfa);
		palm_write(algo_data, I2C_PALM_STARTXFR,0x3);

		regVal = palm_read(algo_data, I2C_PALM_STATUS);
		if (regVal & 0x0008) {
			printk("start quick: ACKERR. Aborting...\n");
			return -1;
		}
		return 0;
	}


	palm_write(algo_data, I2C_PALM_DEVADDR, addr);
	palm_write(algo_data, I2C_PALM_CFG,     0xfa);
	palm_write(algo_data, I2C_PALM_BYTECNT, len-1);
	palm_write(algo_data, I2C_PALM_STARTXFR,0x1);


	regVal = palm_read(algo_data, I2C_PALM_STATUS);
	if (regVal & 0x0008) {
		printk("start read: ACKERR. Aborting...\n");
		return -1;
	}

	for(tmp=0; tmp < len; tmp++) {
		i = 0;
		while(1) {
			regVal = palm_read(algo_data, I2C_PALM_STATUS);
			if (regVal & 0x4) {
				buf[tmp] = (__u8)palm_read(algo_data, 
							I2C_PALM_DATAIN);
				break;

			}
			mdelay(1);
			i++;
			if (i >= 1000) {
				printk("* read Timed OUT byte %d.\n", tmp);
				return -1;
			}
		}
	}
	return 0;
}




static int palm_tx(struct i2c_algo_palm_data *algo_data,  __u16 len, 
		__u8 *buf, __u16 addr)
{
	volatile __u32 tmp, regVal;
	int i;

	if (wait_for_idle(algo_data) < 0) {
		printk("TimedOut on Waiting for I2C Bus Idle.\n");
		return -1;
	}

	if(len == 0) { /* special case of I2C_SMBUS_QUICK */
		/* issue a address only transaction */
		palm_write(algo_data, I2C_PALM_DEVADDR, addr);
		palm_write(algo_data, I2C_PALM_CFG,     0xfa);
		palm_write(algo_data, I2C_PALM_STARTXFR,0x2);

		regVal = palm_read(algo_data, I2C_PALM_STATUS);
		if (regVal & 0x0008) {
			printk("start quick: ACKERR. Aborting...\n");
			return -1;
		}
		return 0;
	}

	palm_write(algo_data, I2C_PALM_DEVADDR, addr);
	palm_write(algo_data, I2C_PALM_DATAOUT, buf[0]);
	palm_write(algo_data, I2C_PALM_CFG,     0xfa);
	palm_write(algo_data, I2C_PALM_BYTECNT, len-1);
	palm_write(algo_data, I2C_PALM_STARTXFR, 0x0);

	regVal = palm_read(algo_data, I2C_PALM_STATUS);
	if (regVal & 0x0008) {
		printk("write: ACKERR. Aborting...\n");
		return -1;
	}


	i= 0x1000;
	regVal = palm_read(algo_data, I2C_PALM_STATUS);
	while (!(regVal & 0x0002) && i) {
		regVal = palm_read(algo_data, I2C_PALM_STATUS);
		i--;
	}
	if (i==0x00) {
		printk(" Write %d Test failed.[TimeOut]SDOEMPTY Not Set\n", tmp);
		printk(" status: 0x%x.\n", regVal);
		return -1;
	}

	for(tmp=1; tmp < len; tmp++) {
		// palm_write(algo_data, I2C_PALM_CFG,     0xfa);
		// palm_write(algo_data, I2C_PALM_BYTECNT, 0);
		palm_write(algo_data, I2C_PALM_DATAOUT, buf[tmp]);
		palm_write(algo_data, I2C_PALM_STARTXFR,0x0);

		regVal = palm_read(algo_data, I2C_PALM_STATUS);
		if (regVal & 0x0008) {
			printk("start write: ACKERR. Aborting...\n");
			return -1;
		}

		i= 0x1000;
		regVal = palm_read(algo_data, I2C_PALM_STATUS);
		while (!(regVal & 0x0002) && i) {
			regVal = palm_read(algo_data, I2C_PALM_STATUS);
			i--;
		}
		if (i==0x00) {
			printk(" Write %d Test failed.[TimeOut]SDOEMPTY Not Set\n", tmp);
			return -1;
		}
	}
	return 0;
}


static int palm_xfer(struct i2c_adapter *i2c_adap,
		struct i2c_msg msgs[],
		int num)
{
	struct 	i2c_algo_palm_data *algo_data = i2c_adap->algo_data;
	struct 	i2c_msg *msg = NULL;
	int 	curmsg;


	for (curmsg = 0; curmsg < num; curmsg++) {

		int addr;
		msg = &msgs[curmsg];

		addr = (0x7f & msg->addr);

		/*
		 * Check if I2C State Machine is idle
		 * 'wait_for_idle' returns 0 => timedOut
		 * 'BUSY' bit cleared => BUS is IDLE
		 */
		if (wait_for_idle(algo_data) < 0) {
			printk("TimedOut on Waiting for I2C Bus Idle.\n");
			return -EIO;
		}
		if (msg->flags & I2C_M_RD ) {
			if ((palm_rx(algo_data, &msg->buf[0], addr, 
							msg->len)) == -1) {
				printk("I2C Read Fail.\n");
				return -EIO;
			}
			if(msg->flags & I2C_M_RECV_LEN)
				msg->len += msg->buf[0];
		}
		else {
			if ((palm_tx(algo_data, msg->len, &msg->buf[0], 
						addr)) == -1) {
				printk("I2C Write Fail.\n");
				return -EIO;
			}
		}
	}
	return num;
}

static u32 palm_func(struct i2c_adapter *adap)
{
	/* We emulate SMBUS over I2C */
	return I2C_FUNC_SMBUS_EMUL;
}

static int palm_init(struct i2c_algo_palm_data *algo_data)
{
	printk("Intializing BK-3220 I2C Host Adapter...");
	spin_lock_init(&palm_lock);
#if 0
	/* RMI Phoenix has a hardcoded value for CLKDIV now... */
	palm_write(algo_data, I2C_PALM_CLKDIV, I2C_PALM_CLKDIV_DEF);
	/* Needed only for Multi-master environments */
	palm_write(algo_data, I2C_PALM_HDSTATIM, I2C_PALM_HDSTATIM_DEF);
#endif
	printk("done.\n");
	return 0;
}

static struct i2c_algorithm palm_algo = {
/* 	.name		= "PalmChips I2C algorithm", */
/* 	.id		= I2C_ALGO_PALM, */
	.master_xfer	= palm_xfer,
	.functionality	= palm_func,
};

/* 
 * registering functions to load algorithms at runtime 
 */
int i2c_palm_add_bus(struct i2c_adapter *adap)
{
	struct i2c_algo_palm_data *palm_adap = adap->algo_data;
	int rval;

/* 	adap->id |= palm_algo.id; */
	/* HLDS */
	adap->nr = 1;
	adap->algo = &palm_algo;

	adap->timeout = 100;		
	adap->retries = 3;		

	rval = palm_init(palm_adap);

	/* register new adapter to i2c module... */
	if (!rval)
		i2c_add_numbered_adapter(adap);
		//i2c_add_adapter(adap);

	return rval;
}

int i2c_palm_del_bus(struct i2c_adapter *adap)
{
	return i2c_del_adapter(adap);
}

EXPORT_SYMBOL(i2c_palm_add_bus);
EXPORT_SYMBOL(i2c_palm_del_bus);

MODULE_AUTHOR("RMI");
MODULE_DESCRIPTION("I2C-Bus PalmChip's Host Adapter algorithm");
MODULE_LICENSE("GPL");

module_param(i2c_debug, int, 0);
