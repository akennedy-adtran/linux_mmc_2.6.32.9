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


#include <asm/netlogic/iomap.h>
#include "nlm_spi4_config.h"
#include "nlm_vits_wrapper.h"
#include "nlm_vits_driver.h"
#include "vitesse_common.h"
#include "vitesse_highlevel.h"
#include "vitesse_io.h"
#include "vitesse_phy_ctrl.h"
#include <asm/netlogic/debug.h>
#include "meigsii_reg.h"
#include <linux/kernel.h>

vtss_mapped_port_t eds_ports_table[VTSS_PORT_ARRAY_SIZE] = {

	/* chip_port, miim_controller, phy_addr */
	/* logical port 0 doesn't exsist */
	{         -1,              -1,     -1  }, 
	{          0,               0,     0x3 },
	{          1,               0,     0x2 },
	{          2,               0,     0x1 },
	{          3,               0,     0x0 },
	{          4,               0,     0x7 },
	{          5,               0,     0x6 },
	{          6,               0,     0x5 },
	{          7,               0,     0x4 },
	{          8,               0,     0xb },
	{          9,               0,     0xa },
	{         10,               1,     0x1 }  /* This is 10g port */
};


void vtss_eds_init( vtss_mac_major_mode_t mmode, vtss_port_interface_t pmode, BOOL fc)
{
	vtss_rc phy_val;

	/* Note that the (R/)GMII ports must be mapped before calling this function */

	int ports_in_use = XLR_TOTAL_CHANNELS; /* all 10 GMII ports are used */
	int portnum = 0;       
	vtss_system_setup_t sys_setup;
	vtss_fifo_setup_t   egress_fifo_setup;
	vtss_fifo_setup_t   ingress_fifo_setup;
	vtss_port_setup_t   port_1G_setup;
	vtss_spi4_setup_t   spi4_setup;
	vtss_fifo_fc_watermarks_t fc_watermarks;
	vtss_mac_t smac = {{0,0,0,0,0,0}};

	/* Reset I/O-level software (will also reset the chip) */
	vtss_chip_reset();

	/* Prepare major mode for runtime calls */
	vtss_major_mode_set(mmode);

	/* Mapping logical ports with physical ports */
	vtss_port_map_set(eds_ports_table);

	/* Setup of basic system parameters -- clock, endianess, etc.. */
	vtss_system_setup_get_default_values( &sys_setup, mmode);
	vtss_system_setup( &sys_setup);

	/* Setup Host interface */
	vtss_spi4_setup_get_default_values( &spi4_setup, mmode);
	vtss_spi4_setup( &spi4_setup);

	/* Setup FIFO */
	vtss_fifo_setup_get_default_values( &ingress_fifo_setup, 
				&egress_fifo_setup, mmode);
	vtss_fifo_setup( &ingress_fifo_setup, &egress_fifo_setup);


	/* Setup logical ports */
	for (portnum = 1; portnum <= ports_in_use; portnum++) {

		phy_val = vtss_phy_reset(portnum) ;
		if(phy_val != VTSS_OK)
			printk("Not able to reset phy=%d\n",portnum);

		vtss_port_setup_get_default_values( &port_1G_setup, mmode);
		/* Change default gmii to pmode */
		port_1G_setup.interface_mode.interface_type = pmode;

		if (fc) {
			/* Change flowcontrol settings i MAC */
			smac.addr[5] = portnum;
			port_1G_setup.flowcontrol.smac = smac;
			port_1G_setup.flowcontrol.obey = 1;
			port_1G_setup.flowcontrol.generate = 1;

			/* Change watermarks settings in FIFO buffer 
			 * to be used for flowcontrol Ingress for port 
			 * flowcontrol. Egress for SPI4 flowcontrol */
			fc_watermarks.low_watermark = 0xc6;
			fc_watermarks.high_watermark = 0xca;
			vtss_fifo_watermarks_set( portnum, &fc_watermarks,
						 &fc_watermarks);
		}
		if(vtss_port_setup( portnum, &port_1G_setup))
			printk("Not able to set the port=%d\n",portnum);
		if(vtss_port_set_mode(portnum, VTSS_SPEED_100M, 1)!= VTSS_OK)
			printk("not able to set 100mb\n");	

		/* Port enable */
		vtss_port_set_enable( portnum, 1, 1);
	}
}



void vtss_nlm_init(int device_number) 
{
	long egr_control, crc_cfg, crc_add;
	int loop;
	int  portnum  ;
	unsigned long	tx_rx_status;

	ulong value;
	BOOL  flow_ena;
	vtss_mac_major_mode_t eds_mmode; /* major mode */
	vtss_port_interface_t eds_pmode; /* port  mode */ 

	megis_nlm_common_init(device_number);

	/* Reset I/O layer, configure operating system driver, and reset chip */
	vtss_io_reset();

	/* Get Chip ID */
	value = vtss_chip_id_get();


	/* Initialize MAC to major mode and port mode */
	eds_mmode = VTSS_MAC_MAJOR_MODE_SPI4_1G; /* default SPI4<->1G*/
	eds_pmode = VTSS_PORT_INTERFACE_RGMII; /* default RGMII */
	flow_ena = 0; 

	vtss_eds_init( eds_mmode, eds_pmode, flow_ena);

	// reset training period 
	vtss_io_write(M2_BLK_SPI4,0,M2_SPI4_ING_SETUP1, 0x1000f);

	// setup debug counter to count ingress fifo events
	vtss_io_write(M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_TEST, 0x00010000);
	vtss_io_write(M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_DEBUG_BUF_CNT, 0x00010000);

	// Setting IFG to small value
	for(loop=0; loop<=10; loop++){
		vtss_io_write(M2_BLK_MACS, loop, M2E_TX_IFG, 0x6);
	}

	// not to drop frame with crc error in egress
	crc_cfg = vtss_io_read(M2_BLK_SYSTEM,M2_SUBBLK_CTRL,0xb);
	crc_cfg |= 1<<5;
	vtss_io_write(M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 0xb, crc_cfg);

	egr_control = vtss_io_read(M2_BLK_FIFO,M2_SUBBLK_EGRESS,M2_EGR_CONTROL);
	egr_control |= 1<<18;
	vtss_io_write(M2_BLK_FIFO,M2_SUBBLK_EGRESS,M2_EGR_CONTROL, egr_control);

	for(portnum=0; portnum < XLR_TOTAL_CHANNELS; portnum++){
			crc_add = vtss_io_read(M2_BLK_MACS, portnum, M2_TRI_DENORM);
			crc_add &= ~(1<<5); // clear 5th bit crc_upd
			crc_add |= 1<<4;  // set 4th bit crc_add
			vtss_io_write(M2_BLK_MACS, portnum, 
						M2_TRI_DENORM,crc_add);
			crc_add = vtss_io_read(M2_BLK_MACS, 
						portnum, M2_TRI_DENORM);

				crc_add = vtss_io_read(M2_BLK_MACS, 
						portnum, M2_DEV_SETUP);
			tx_rx_status = vtss_io_read(M2_BLK_MACS, portnum, M2_MODE_CFG);
			tx_rx_status &= ~(0X3) ; // DISABLE TX RX
	  	vtss_io_write(M2_BLK_MACS, portnum, M2_MODE_CFG, tx_rx_status);
		}
	

	// setup debug counter to count egress fifo events
	vtss_io_write(M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TEST, 0x00010000);
	vtss_io_write(M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_DEBUG_BUF_CNT, 0);


	// setup debug counter to count spi4 egress traffic
	vtss_io_write(M2_BLK_SPI4, 0, M2_SPI4_DBG_CNT, 0);
	vtss_io_write(M2_BLK_SPI4, 0, M2_SPI4_DBG_SETUP, 0x00000002);

	return;
}


int vtss_nlm_monitor_phy_status(int port, int *speed, int *duplexity)
{
	vtss_rc ret, dup;
	vtss_phy_status_t   status;

	ret = vtss_phy_status_get(port, &status);
	if(ret != VTSS_OK){
		return 0;
	}
	dup = ret = vtss_phy_read(port, 0x1c);
	if(ret<0) return 0;

	ret >>=3;
	ret &= 0x3;
	switch(ret){
		case 2:
			(*speed) =(int) SPEED_1000M;
			break;
		case 1:
			(*speed )= (int)SPEED_100M;
			break;
		case 0:
			(*speed) = (int)SPEED_10M;
			break;
		default:
			(*speed) = (int)UNDEFINED_SPEED;
	}
	dup >>= 5;
	dup &= 0x01;
	if(dup){
		(*duplexity) = 1;
	}
	else{
		(*duplexity) = 0;
	}

	return 1;
}

int vtss_nlm_change_port_status(int port, int speed, int duplexity)
{
	int i;

	switch(speed){
	case SPEED_1000M:
		speed = VTSS_SPEED_1G;
		break;
	case SPEED_100M:
		speed = VTSS_SPEED_100M;
		break;
	case SPEED_10M:
		speed = VTSS_SPEED_10M;
		break;
	default :
		speed = VTSS_SPEED_1G;
	}
	i = vtss_port_set_mode(port, speed, duplexity);
	if(i == VTSS_OK ){
		return 1;
	}
	else{
		return 0;
	}
}

