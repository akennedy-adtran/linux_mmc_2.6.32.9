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
 * Board dependent code for Netlogic's XLR-based boards
 */

#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/pm.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>

#include <asm/netlogic/sim.h>
#include <asm/mipsregs.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/xlr_user_mac.h>
#include <asm/netlogic/msgring.h>

#include <asm/netlogic/nlm_pcix_gen_dev.h>
#include <asm/netlogic/bootinfo.h>
#include <asm/netlogic/memory-exclusion.h>

#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/mach-netlogic/mmu.h>
#include <asm/netlogic/i2c-algo-palm.h>
#include <asm/netlogic/xlr_board.h>


#define EEPROM_MAJOR_OFFSET 0x18
#define EEPROM_MINOR_OFFSET 0x19
#define EEPROM_ETH_MAC_OFFSET 0x20
#define GPIO_RESET_CFG 			21

int wait_i2c_idle(nlm_reg_t *mmio)
{
	int i;
	nlm_reg_t regVal;

	i=0x1000;
	regVal = netlogic_read_reg(mmio, I2C_PALM_STATUS) & 0x0001;
	while (regVal && i--) {
		regVal = netlogic_read_reg(mmio, I2C_PALM_STATUS) & 0x0001;
	}
	if(i == 0) {
		printk("Bus not idle\n");
		return -1;
	}
	return 0;
}


static uint32_t xls_get_cpu_clk(uint32_t cfg)
{
	int adivq, adivf;
	int ref = 6667;
	int res;

	/* REF/2 * DIVF / DIVQ = PLLOUT */
	adivf = (cfg & 0xff) + 1;
	adivq = (cfg >> 8) & 0x7 ;
	adivq = (1 << adivq); /* adivq = 2 ^ divq */

	/* multiply adivf by 1000 so that we get
	 * better result with integer division.
	 * Also divide the final result with 100 (from ref
	 * earlier) and the 1000 we multiplied with adivf
	 */
	res = ((ref / 2) * (adivf*1000/adivq))/(100*1000);
	printk("Determined CPU frequency = %dMhz\n", res);
	return (uint32_t)res;
}

void get_cpu_clock(xlr_board_info_t *board)
{
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_GPIO_OFFSET);
	uint32_t freq=0, cfg;

	cfg = netlogic_read_reg(mmio, GPIO_RESET_CFG);

	if(is_xls()) {
		freq = xls_get_cpu_clk(cfg);
	} else {

		freq = (((((cfg >> 2) & 0x7f) + 1) * 16667)/1000);

		if (cfg & 0x200)
			freq = freq >> 1;
	}
	if(!freq) {
		printk("Unable to determine CPU frequency from CFGREG [0x%x]\n", cfg);
		board->cpu_freq = 1000 * 1000 * 1000;
	} else {
		board->cpu_freq = freq * 1000 * 1000;
	}
	
}

int  read_board_info(xlr_board_info_t *board) 
{
	uint8_t ebuf[48];
	int len;
	volatile uint32_t regVal, i, tmp;
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_I2C_1_OFFSET);

	/* if this is not read from FDT */
	if(board->cpu_freq == 0)
		get_cpu_clock(board);

	/* Read other board dependent stuff */
	/* EEPROM on Netlogic XLR/XLS boards are on BUS 1 */
	/* read first 40 bytes of the EEPROM */
	
	len = 40;

	for(tmp=0; tmp < len; tmp++) {

		if(wait_i2c_idle(mmio) != 0)
			return -1;

		/* EEPROM is at address 0x50 on bus 1 */
		netlogic_write_reg(mmio, I2C_PALM_DEVADDR, 0x50);
		netlogic_write_reg(mmio, I2C_PALM_ADDR, tmp);
		netlogic_write_reg(mmio, I2C_PALM_CFG,     0xf8);
		netlogic_write_reg(mmio, I2C_PALM_BYTECNT, 0);
		netlogic_write_reg(mmio, I2C_PALM_STARTXFR,0x2);


		if(wait_i2c_idle(mmio) != 0)
			return -1;

		regVal = netlogic_read_reg(mmio, I2C_PALM_STATUS);
		if (regVal & 0x0008) {
			printk("start read: ACKERR. Aborting...\n");
			return -1;
		}


		netlogic_write_reg(mmio, I2C_PALM_DEVADDR, 0x50);
		netlogic_write_reg(mmio, I2C_PALM_CFG,     0xfa);
		netlogic_write_reg(mmio, I2C_PALM_BYTECNT, 0);
		netlogic_write_reg(mmio, I2C_PALM_STARTXFR,0x1);


		if(wait_i2c_idle(mmio) != 0)
			return -1;

		regVal = netlogic_read_reg(mmio, I2C_PALM_STATUS);
		if (regVal & 0x0008) {
			printk("start read: ACKERR. Aborting...\n");
			return -1;
		}
		i = 0;
		while(1) {
			regVal = netlogic_read_reg(mmio, I2C_PALM_STATUS);
			if (regVal & 0x4) {
				ebuf[tmp] = (__u8)netlogic_read_reg(mmio,
						I2C_PALM_DATAIN);
				break;

			}
			i++;
			if (i >= 1000000) {
				printk("* read Timed OUT byte %d.\n", tmp);
				return -1;
			}
		}
	}
	
	if(board->major == 0) 
		board->major = ebuf[EEPROM_MAJOR_OFFSET] - '0';
	if(board->minor == 0) 
	board->minor = ebuf[EEPROM_MINOR_OFFSET] - '0';

	if(*((uint32_t *)board->mac_addr) == 0) {
		for(i=0; i < 6; i++) 
			board->mac_addr[i] = ebuf[EEPROM_ETH_MAC_OFFSET + i];
	}
	return 0;

}
