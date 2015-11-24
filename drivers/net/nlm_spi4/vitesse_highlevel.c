/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/************************************************************-*- mode: C -*-*/
/*                                                                          */
/*           Copyright (C) 2003 Vitesse Semiconductor Corporation           */
/*                           All Rights Reserved.                           */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                            Copyright Notice:                             */
/*                                                                          */
/*  Unpublished rights reserved under the copyright laws of the United      */
/*  States of America, other countries and international treaties.          */
/*                                                                          */
/*      The software is provided without fee.                               */
/*                                                                          */
/*  Permission to use,  copy, store, modify, disclose, transmit or          */
/*  distribute the software is granted, provided that this copyright notice */
/*  must appear in any copy, modification, disclosure, transmission or      */
/*  distribution of the software.                                           */
/*                                                                          */
/*  Vitesse Semiconductor Corporation retains all ownership, copyright,     */
/*  trade secret and proprietary rights in the software.                    */
/*                                                                          */
/*  THIS SOFTWARE HAS BEEN PROVIDED "AS IS," WITHOUT EXPRESS OR IMPLIED     */
/*  WARRANTY INCLUDING, WITHOUT LIMITATION, IMPLIED WARRANTIES OF           */
/*  MERCHANTABILITY, FITNESS FOR A PARTICULAR USE AND NON-INFRINGEMENT.     */
/*                                                                          */
/*    $Id: vitesse_highlevel.c,v 1.1.2.6 2008-05-22 02:59:56 kmurthy Exp $         */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*  File content:                                                           */
/*                                                                          */
/*  vitesse_highlevel.c -- a set of functions for configuration of various  */
/*                  system features in a MeigsII/MeigsIIe/Campbell chip     */
/*                                                                          */
/****************************************************************************/
#define __MEIGSII_LIBRARY__



#include "vitesse_highlevel.h"
#include "meigsii_reg.h"
#include "vitesse_io.h"
#include <asm/netlogic/debug.h>
#include <linux/kernel.h>

#define     SET_BIT( A,SHIFT)    A |=  ((ulong)1<<SHIFT)
#define     CLR_BIT( A,SHIFT)    A &= ~((ulong)1<<SHIFT)

#define     SET_BITS_MASKED( result, value, mask)  \
	result = (result & ~(mask))|(value & mask)


/****************************************************************************/
/*                                                                          */
/*   Type definitions                                                       */
/*                                                                          */
/****************************************************************************/
typedef struct _vtss_device_setup_t {

	vtss_mac_major_mode_t    mmode;
	ulong                    chip_id;

} vtss_device_setup_t;


/************     Use of the normalised and preamble headers   *************/
/*
   The related NLE bit selects the way data is transmitted internally in the 
   device and is always used when the normalized header is used.
 */
typedef struct _m2_header_t {

	BOOL use_norm_hdr;
	BOOL use_prm_hdr;
	/*
	   BOOL mpls_normalize;
	   BOOL vlan_normalize;
	 */
} m2_header_t;


/****************************************************************************/
/*                                                                          */
/*   Local function prototypes                                              */
/*                                                                          */
/****************************************************************************/
/*static vtss_rc vtss_aggr_mode_set( vtss_aggr_mode_t *p_aggrmode);*/

/* control of the use of header in ingress direction */
static vtss_rc vtss_hdr_insert( BOOL enable, m2_header_t hdr);
#if 0
/* control of the use of header in egress direction */
static vtss_rc vtss_hdr_expect( BOOL enable, m2_header_t hdr);
#endif


/* Enabling/disabling use of the header in different blocks */
static void vtss_fifo_use_hdr( int fifo, BOOL enable, m2_header_t hdr);

static void vtss_aggr_use_header( BOOL enable, m2_header_t hdr);


static void vtss_port10G_header_insert( BOOL enable, m2_header_t hdr);

static void vtss_port1G_header_insert( int ppn, BOOL enable, 
		const m2_header_t hdr);

/* vtss_rc vtss_device_major_mode_setup( vtss_mac_major_mode_t mm); */


/****************************************************************************/
/*                                                                          */
/*   Global data                                                            */
/*                                                                          */
/****************************************************************************/

static vtss_mapped_port_t vtss_logical_ports[VTSS_PORT_ARRAY_SIZE] = {
	{ -1, -1, -1},
	{ -1, -1, -1},
};

static vtss_device_setup_t vtss_device_setup = {
	VTSS_MAC_MAJOR_MODE_UNDEFINED,
	0,  /* chip id */
};


/* Current state of 1G ports */
static vtss_port_setup_t vtss_1G_ports_setup[VTSS_PORT_ARRAY_SIZE];/* = {} */


/* Pointer to the global device setup structure */
static vtss_device_setup_t *pdevice = &vtss_device_setup;


/*******************************************************************************

  Function vtss_port_map_set()
  =================================================================================

Description:
The function initializes internally kept port mapping structure based on 
the information received from the user 


Syntax
vtss_rc  vtss_port_map_set( const vtss_mapped_port_t 
mapped_ports[VTSS_PORT_ARRAY_SIZE]);

Arguments:
mapped_ports    an array mapping logical port numbers to physical 
(on-chip) port numbers, MIIM controller channel and
PHY addresses.

Return code:
VTSS_OK   if operation completed successfully

 *******************************************************************************/
vtss_rc vtss_port_map_set( const vtss_mapped_port_t 
		mapped_ports[VTSS_PORT_ARRAY_SIZE])
{
	int i;

	for (i=0;i<VTSS_PORT_ARRAY_SIZE; i++) {
		vtss_logical_ports[i] = mapped_ports[i];
	}


	return VTSS_OK;
}


/*******************************************************************************

  Function vtss_port_mapped()
  =================================================================================

Description:
Checks whether the logical port is mapped to any physical port or not.

Syntax
BOOL vtss_port_mapped( const vtss_port_no_t port_no )

Arguments:
port_no     logical port number


Return code:
VTSS_OK   if operation completed successfully

 *******************************************************************************/
BOOL vtss_port_mapped( const vtss_port_no_t port_no )
{

	return ( -1 != vtss_logical_ports[port_no].chip_port);
}


/*******************************************************************************

  Function vtss_chip_reset()
================================================================================

Description:
Performs a software reset of the chip


Syntax
void vtss_chip_reset(void);

Arguments:
None

Return code:
None

*******************************************************************************/
void vtss_chip_reset(void)
{

	/* Reset the chip */
	vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
			M2_SW_RESET, M2_SW_GLOBAL_RESET);
	VTSS_NSLEEP(VTSS_T_RESET);

}


/*******************************************************************************

  Function vtss_get_chip_id()
================================================================================

Description:
Returns the content of the chip identification register

NOTE: -----------------------------    
|  device         chip id   |
-----------------------------
|  VSC7321       0x0F407321 |
|  VSC7323       0x073230E9 |
|  VSC7331       0x073310E9 |
-----------------------------

Syntax
ulong vtss_chip_id_get(void);


Arguments:
None


Return code:
The content of the chip identification register.

 *******************************************************************************/
ulong vtss_chip_id_get(void)
{
	vtss_device_setup_t *pvds = pdevice;
	pvds->chip_id = vtss_io_read( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_CHIPID);
	return pvds->chip_id;
}


/*******************************************************************************

  Function vtss_major_mode_set( )
  ================================================================================

Description:
Prepares the system to run in one of the major modes.
This function must be called only once after reset. If the function is 
called when the system is up and running it will return an error.
It is not allowed to call this function with VTSS_MAC_MAJOR_MODE_UNDEFINED
as an argument.

NOTE:


Syntax
vtss_rc vtss_major_mode_set( vtss_mac_major_mode_t mm);

Arguments:
mm     select the major mode for the session


Return code:
VTSS_OK                  successful completion
VTSS_WRONG_PARAMETER     if called after the major mode has been set

 *******************************************************************************/
vtss_rc vtss_major_mode_set( vtss_mac_major_mode_t mm)
{
	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;

	if (mm == VTSS_MAC_MAJOR_MODE_UNDEFINED) {
		return VTSS_WRONG_PARAMETER;
	}

	if ( VTSS_MAC_MAJOR_MODE_UNDEFINED != pvds->mmode) { 
		return VTSS_WRONG_PARAMETER;
	} else {
		pvds->mmode = mm;
	}

	return VTSS_OK;
}


/******************************************************************************
 *
 * Description: Performs several low-level initialization operations on the chip.
 *              The function must be called after start-up or chip reset.
 *
 * NOTE:        This function does not perform COMPLETE initialization of the chip.
 *
 * \param:      psystem   pointer to a structure with configuration data
 *
 * \return:     VTSS_OK   if operation completed successfully or
 *              VTSS_WRONG_PARAMETER
 *
 **************************************************************************kbp*/
vtss_rc vtss_system_setup( vtss_system_setup_t *ps)
{
	ulong reg  = 0;

	/* Setup Serial and Parallel CPU interfaces */
	/* Assuming order of bits in a byte big-endian, 
		however it can be changed */
	if ( ps->big_endian == FALSE) {
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
			M2_SI_TRANSFER_SEL, M2_LTL_END_BYTE_BIG_END_BIT);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
			M2_CPU_TRANSFER_SEL, M2_LTL_END_BYTE_BIG_END_BIT);
	} else {
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
			M2_SI_TRANSFER_SEL, M2_BIG_END_BYTE_BIG_END_BIT);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
			M2_CPU_TRANSFER_SEL, M2_BIG_END_BYTE_BIG_END_BIT);
	}


#if !defined MEIGS2 && !defined VSC7321
	/* Set-up wait states on serial read when frequency > 0.5MHz 
	   to match the chip response time on reads of 1 us. */
	if ( ps->si_dummy_bytes < 8 ) {
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
				M2E_SI_INSERT_BYTES, ps->si_dummy_bytes);
	} else {
		return VTSS_WRONG_PARAMETER;
	}
#endif


	reg = 0;
	/* Take SerDes PLL out of reset when not in Single Channel mode */
	if ( ps->rx_chain_mode_10G == FALSE) { SET_BIT( reg, 7); }

	/* Setup SerDes reference clock sources */
	if ( ps->serdes_ref_clk_external == FALSE) {
		/* NB: serdes_ref_clk_external must be 
			TRUE when using VSC7323 or VSC7331 */
#if !defined MEIGS2 && !defined VSC7321
		return VTSS_WRONG_PARAMETER;
#endif
	} else {
		SET_BIT( reg, 1);
	}
	vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
			M2_PLL_CLK_SPEED, reg, 0x82);


	/* Setup System reference clock sources */
	if ( ps->system_ref_clk_extern == FALSE) {
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_SYS_CLK_SELECT, 0);
	} else {
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_SYS_CLK_SELECT, 1);
	}


	reg = 0;
	/* Setup interface mode */
	if( ps->single_chip_mode != FALSE) { SET_BIT( reg, 3); SET_BIT( reg, 2); }
	if( ps->rx_chain_mode_10G != FALSE) { SET_BIT( reg, 1); SET_BIT( reg, 0); }
	vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_IFACE_MODE, reg);


	return VTSS_OK;
}


/******************************************************************************
 *
 * Description: This is a supplementary function.
 *              It configures a structure referenced by the pointer to the
 *              default values for the chosen major mode.
 *
 * \param:      ps        pointer to a structure, which will be filled out
 *              mjm       mode the device will run
 *
 * \return:     VTSS_OK   if operation completed successfully or
 *
 **************************************************************************kbp*/
vtss_rc vtss_system_setup_get_default_values( vtss_system_setup_t *ps,
		vtss_mac_major_mode_t mjm)
{

	/* TRUE (default):  CPU interface runs as big-endian */
	ps->big_endian = TRUE;


	/* Number of dummy bytes that will be inserted in the 
	   reply before valid data when serial interface is used
	   at high frequency. Set to 0 if not used */
	ps->si_dummy_bytes = 0;  


	/* Select the source of the serdes reference clock */
	/* TRUE (default):  external source of the reference clock */
	/* FALSE         :  internal source of the reference clock */
	/* NOTE!!   MUST be TRUE when using VSC7323 or VSC7331 */
	ps->serdes_ref_clk_external = TRUE;


	/* Select the source of the system clock */
	/* FALSE (default): system clock uses internal source */
	/* TRUE             system clock uses GPIO15 as system clock input */
	ps->system_ref_clk_extern = FALSE; 


	/* Single Chip Aggregation or trunking mode */
	if ( mjm == VTSS_MAC_MAJOR_MODE_10G_1G_AGGR || 
			mjm == VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK) {
		ps->single_chip_mode = TRUE; 
	} else {
		ps->single_chip_mode = FALSE;
	}


	/* Single Channel mode */
	if ( mjm == VTSS_MAC_MAJOR_MODE_SPI4_10G) {
		ps->rx_chain_mode_10G = TRUE;  /* SPI4.2<->10G mode */
	} else {
		ps->rx_chain_mode_10G = FALSE; /* all other modes */
	}

	return VTSS_OK;
}


/****************************************************************************/
/*                                                                          */
/*   1G(triple speed) port setup                                            */
/*                                                                          */
/****************************************************************************/



/*******************************************************************************

  Function 
  ================================================================================

Description:
Configures 10M/100M/1G port according to the structure provided.


Syntax
vtss_rc vtss_port_setup( vtss_port_no_t portnum, vtss_port_setup_t* ps);

Arguments:
portnum      logical port to be initialized
ps           pointer to the setup structure

Return code:
VTSS_OK   if operation completed successfully
VTSS_PORT_NOT_MAPPED
VTSS_MAJOR_MODE_NOT_SET
VTSS_WRONG_MAJOR_MODE
VTSS_WRONG_PARAMETER

 *******************************************************************************/
vtss_rc vtss_port_setup( vtss_port_no_t portnum, vtss_port_setup_t* ps)
{

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;

	ulong mode_cfg_reg = 0;
	ulong dev_setup_reg_value = 0x80; /* Bit 7 shall be set to 1 */
	ulong pcs_enable = 0;
	ulong pause_reg_value = 0;
	ulong reg = 0;

	/* Some hardcoded values */
	BOOL use_back_seed = FALSE;
	/*  ulong backseed = 0; */

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	//  VTSS_N(("Port setup. Logic port #%d, phys prt #%d", portnum, ppn));
	//printk("Port setup. Logic port #%d, phys prt #%d\n", portnum, ppn);

	/* Check the physical port number */
	if (ppn < 0) { 
		printk("[%s]port=%d not mapped\n",__FUNCTION__,portnum);
		return VTSS_PORT_NOT_MAPPED; 
	}


	/* Check major mode */
	switch ( pvds->mmode) {
	/* valid major modes */
	case VTSS_MAC_MAJOR_MODE_SPI4_1G:      /* SPI4 <-> 10x1G */
	case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:  /* Single Chip aggr */
	case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* Single Chip trunking */
		break;

	/* Error - major mode undefined */
	case VTSS_MAC_MAJOR_MODE_UNDEFINED:
		return VTSS_MAJOR_MODE_NOT_SET;

	/* Error - wrong major mode */
	case VTSS_MAC_MAJOR_MODE_SPI4_10G:     /* SPI4 <-> 1x10G */
	case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:   /* */
	case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
	default:
		printk("[%s]wrong major mode=%d\n",__FUNCTION__,pvds->mmode);
		return VTSS_WRONG_MAJOR_MODE;
	}

#if defined MEIGS2 || defined VSC7321
	/* VSC7321 (Meigs-II) requires BOTH fields being either set or cleared */
	if (ps->flowcontrol.obey != ps->flowcontrol.generate) { return VTSS_WRONG_PARAMETER; }
#endif

	/* -------------------- Device Setup Register ------------------------ */

	/* While changing speed, the port must be in reset state */
	SET_BIT( dev_setup_reg_value, 0);

	/* Select speed and interface for the device */
	switch (ps->interface_mode.speed) {
	case VTSS_SPEED_10M:
		SET_BITS_MASKED( dev_setup_reg_value, M2_1G_PORT_CLOCK_MODE_10M<<1, 
				M2_1G_PORT_CLOCK_MODE_MASK<<1);
		break;
	case VTSS_SPEED_100M:
		SET_BITS_MASKED( dev_setup_reg_value, M2_1G_PORT_CLOCK_MODE_100M<<1, 
				M2_1G_PORT_CLOCK_MODE_MASK<<1);
		break;
	case VTSS_SPEED_1G:
	case VTSS_SPEED_ETH_GFPT:
	case VTSS_SPEED_FC2_GFPT: /* 6    ½Gbit/s Fibre Channel */
	case VTSS_SPEED_FC4_GFPT: /* 7    ¼Gbit/s Fibre Channel */
	case VTSS_SPEED_1FC_GFPT: /* 8    1Gbit/s Fibre Channel or FICON*/
	case VTSS_SPEED_2FC_GFPT: /* 9    2Gbit/s Fibre Channel */
	case VTSS_SPEED_ESC_GFPT: /* 10 200Mbit/s ESCON or SBCON */
	case VTSS_SPEED_DVB_GFPT: /* 11 270Mbit/s DVB ASI */

	if (ps->interface_mode.interface_type == VTSS_PORT_INTERFACE_SERDES) {
		SET_BITS_MASKED( dev_setup_reg_value, 
			M2_1G_PORT_CLOCK_MODE_1G_SERDES<<1, 
			M2_1G_PORT_CLOCK_MODE_MASK<<1);

	} else if (ps->interface_mode.interface_type == VTSS_PORT_INTERFACE_TBI) {
		SET_BITS_MASKED( dev_setup_reg_value, 
			M2E_1G_PORT_CLOCK_MODE_TBI<<1, 
			M2_1G_PORT_CLOCK_MODE_MASK<<1);

	} else {
		SET_BITS_MASKED( dev_setup_reg_value, 
			M2_1G_PORT_CLOCK_MODE_1G_GMII<<1, 
			M2_1G_PORT_CLOCK_MODE_MASK<<1);
	}

	/* Overrule setting by disable Ingress CRC updating when in GFP-T mode */
	if (ps->interface_mode.speed != VTSS_SPEED_1G) { ps->crc_update = FALSE; }
		break;

	default:
		//VTSS_E(("1G port #%d. Wrong speed: %d", ppn, ps->interface_mode.speed));
		printk("1G port #%d. Wrong speed: %d\n", ppn, ps->interface_mode.speed);
		return VTSS_WRONG_PARAMETER;
	}

	/* Set-up selected interface */
	switch ( ps->interface_mode.interface_type) {
	case VTSS_PORT_INTERFACE_GMII:
		/* Invert (R)GMII_TX_clock when GMII */
		SET_BIT( dev_setup_reg_value, 6); 
			break;
#if !defined MEIGS2 && !defined VSC7321
	case VTSS_PORT_INTERFACE_RGMII:
		SET_BIT( dev_setup_reg_value, 4);   /* Enable RGMII */
		break;
#endif
	case VTSS_PORT_INTERFACE_SERDES:
		SET_BIT( pcs_enable, 0);            /* Enable Ethernet PCS */
		break;
	case VTSS_PORT_INTERFACE_TBI:
		/* Disable Signal detect */
		vtss_io_writemasked( M2_BLK_MACS, ppn, M2_PCS_CTRL, 0, (ulong)3<<22);
		SET_BIT( dev_setup_reg_value, 23);  /* Enable TBI */
		SET_BIT( dev_setup_reg_value, 22);  /* Invert TBI_TX_clock when TBI */
		SET_BIT( pcs_enable, 0);            /* Enable Ethernet PCS */
		break;
	default:
		printk("1G port #%d. Wrong interface type: %d\n", 
			ppn, ps->interface_mode.interface_type);
		break;
	}


	/* -------------------- Mode config Register ------------------------ */

	/* interframe gaps */  
	SET_BITS_MASKED( mode_cfg_reg, ps->frame_gaps.ifg1 << 6, VTSS_IFG_MASK << 6);
	SET_BITS_MASKED( mode_cfg_reg, ps->frame_gaps.ifg2 << 10, VTSS_IFG_MASK << 10);

	/* duplex mode */
	if ( ps->fdx != FALSE) {
		SET_BIT( mode_cfg_reg, 2);
		if (ps->interface_mode.speed == VTSS_SPEED_1G) { SET_BIT( mode_cfg_reg, 3); }
	}

	/* vlan_awr */
	if ( ps->vlan_aware != FALSE) { SET_BIT( mode_cfg_reg, 4); }

	/* back_seed */
	/*
	   if (use_back_seed) {
	   SET_BITS_MASKED( mode_cfg_reg, (ulong)back_seed << 16); SET_BIT( mode_cfg_reg, 14);
	   }
	 */

	/* drop_on_length_error */
	if ( ps->drop_on_length_error != FALSE) { SET_BIT( mode_cfg_reg, 15); }

	/* Old Backoff Enable */
	/*
	//VTSS_ASSERT((old_backoff >= 0) && (old_backoff <= 2));
	switch (old_backoff) {
	case 0:
	CLR_BITS_MASKED( mode_cfg_reg, (ulong)0x3 << 24);
	break;
	case 1:
	SET_BITS_MASKED( mode_cfg_reg, (ulong)0x1 << 24);
	break;
	case 2:
	SET_BITS_MASKED( mode_cfg_reg, (ulong)0x2 << 24);
	break;
	}
	 */

	/* Write to device Mode configureation register */
	vtss_io_write( M2_BLK_MACS, ppn, M2_MODE_CFG, mode_cfg_reg);

	/* Clear load_seed bit */
	if (use_back_seed) { 
		vtss_io_writemasked( M2_BLK_MACS, ppn, 
				M2_MODE_CFG, 0, (ulong)1<<14); 
	}

	/* -------------------- Update other Register ------------------------ */

	/* Set flow control */
	pause_reg_value = (ps->tx_pause_value & VTSS_PORT_1G_TX_PAUSE_MASK);
	if( ps->enable_tx_pause_xon_xoff != FALSE){ 
		SET_BIT( pause_reg_value, 17); 
	}
	vtss_io_write( M2_BLK_MACS, ppn, M2_PAUSE_CFG, pause_reg_value);

	/* VSC7321 (Meigs-II) requires both bits to be set */
	vtss_port_flow_control_mode( portnum, ps->flowcontrol.obey, ps->flowcontrol.generate);

	/* Set-up MAC address. Will be written if flowcontrol setup is requested. */
	if( ps->flowcontrol.obey != FALSE || ps->flowcontrol.generate != FALSE ) {
		ulong mac_addr_low_reg = 
			(ps->flowcontrol.smac.addr[2] << 16)| 
			(ps->flowcontrol.smac.addr[1] <<  8)|
			ps->flowcontrol.smac.addr[0];

		ulong mac_addr_high_reg = 
			(ps->flowcontrol.smac.addr[5] << 16)| 
			(ps->flowcontrol.smac.addr[4] <<  8)|
			ps->flowcontrol.smac.addr[3];

		vtss_io_write( M2_BLK_MACS, ppn, M2_MAC_ADDR_HIGH_CFG, mac_addr_high_reg);
		vtss_io_write( M2_BLK_MACS, ppn, M2_MAC_ADDR_LOW_CFG, mac_addr_low_reg);
	}

	/* Write to device max length register */
	vtss_io_write( M2_BLK_MACS, ppn, M2_MAXLEN_CFG, 
			ps->maxframelength & VTSS_MAX_FRAME_LENGTH_MASK);

	/* Change Egress CT_THRSHLD to maxframelength when set to store&forward */
	if (( pvds->mmode == VTSS_MAC_MAJOR_MODE_SPI4_1G )|| 
		(pvds->mmode == VTSS_MAC_MAJOR_MODE_SPI4_10G )){
		if ( vtss_io_read ( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
					M2_CT_THRHLD + ppn) == 0 ) { 
			vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
					M2_CT_THRHLD + ppn, ps->maxframelength>>5); 
		}
	}

	/* HOLD device clock in reset while changing the clock-mode */
	/* vtss_io_writemasked( M2_BLK_MACS, ppn, M2_DEV_SETUP, 0x1, 0x1); */

	/* Write to device PCS config register */
	vtss_io_write( M2_BLK_MACS, ppn, M2_PCS_CONFIG, pcs_enable);

	/* Write to device setup register */
	vtss_io_write( M2_BLK_MACS, ppn, M2_DEV_SETUP, dev_setup_reg_value);

	/* Configure 1G SERDES */
	if (ps->interface_mode.interface_type == VTSS_PORT_INTERFACE_SERDES) {
		/* Apply RESET to SERDES. This will keep it reset */
		vtss_io_write( 	M2_BLK_MACS, ppn, M2_SERDES_TEST, 
				VTSS_SERDES_RESET_AND_ENABLE);
		/* Write the serdes configuration word */
		vtss_io_write( M2_BLK_MACS, ppn, M2_SERDES_CONF, 
				VTSS_SERDES_CONFIG_WORD);
		/* Take SERDES out of reset */
		vtss_io_write( M2_BLK_MACS, ppn, M2_SERDES_TEST, 
				VTSS_SERDES_ENABLE);
	}

	/* Take the MAC block out of RESET */
	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_DEV_SETUP, 0, 0x1);

	/* --------------- Update Normalizer Register ------------- */

	/* Set-up port number in fake preamble header */
	reg = ((ps->prm_hdr_value & 0xF)<<12);

	/* Insert fake preamble header */
	if( ps->prm_hdr_insert != FALSE) { SET_BIT( reg, 0); }

	/* Insert Normalized header */
	if( ps->norm_hdr_insert != FALSE) {
		SET_BIT( reg, 1);
	} else {
		SET_BIT( reg, 2); /* NLE */
	}

	/* Disable Ingress CRC updating */
	if( ps->crc_update == FALSE) { SET_BIT( reg, 9); }

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_TRI_NORMALIZER, reg, (ulong)0xF207);

	/* ----------- Update De-Normalizer Register --------- */

	reg = 0;
	if ( ps->norm_hdr_expect != FALSE) { SET_BIT( reg, 1); }
	if ( ps->prm_hdr_expect != FALSE) { SET_BIT( reg, 0); }

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_TRI_DENORM, reg, (ulong)0x3);

	/* Egress CRC-32 Checker after SPI4 must be disabled to 
		let bad frames pass through */
	if( VTSS_OK != vtss_port_fcs_modify( portnum, ps->fcs_modify)){
		/* Issue a warning */
		printk("[%s] not able to modify fcs\n",__FUNCTION__);
	}

	/* Save user data to the global structure */
	vtss_1G_ports_setup[ppn] = *ps;

	return VTSS_OK;
}


/*******************************************************************************

  Function void vtss_port_setup_get_default_values( vtss_port_setup_t* ps,
  vtss_mac_major_mode_t mmode)

================================================================================

Description:
Supplementary function: fills the structure referenced by the pointer 
with the default values for a given major_mode.
By default the port is setup to run in 1G GMII mode.

NOTE: 
Requires global port mapping table to be initialized

Syntax
void vtss_port_setup_get_default_values( vtss_port_setup_t* ps,
vtss_mac_major_mode_t mmode);

Arguments:
ps           pointer to a structure that will be filled with default data
mmode        one of the major modes

Return code:
VTSS_OK

 *******************************************************************************/
vtss_rc vtss_port_setup_get_default_values( vtss_port_setup_t* ps,
		vtss_mac_major_mode_t mmode)
{

	vtss_mac_t mac_addr = {{0,0,0,0,0,0}};


	switch ( mmode) {
		/* valid major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* By default 1G GMII interface is assumed */
	ps->interface_mode.interface_type = VTSS_PORT_INTERFACE_GMII;
	ps->interface_mode.speed = VTSS_SPEED_1G;


	ps->fdx = TRUE; /* Full duplex: TRUE, half duplex: FALSE */

	/* Flow control setup */
	ps->flowcontrol.obey = FALSE;
	ps->flowcontrol.generate = FALSE;
	ps->flowcontrol.smac = mac_addr;


	ps->enable_tx_pause_xon_xoff = TRUE;
	ps->tx_pause_value = VTSS_PORT_1G_TX_PAUSE_VALUE;


	/* interframe gaps for 1G speed */
	ps->frame_gaps.ifg1 = VTSS_IFG1_1G;
	ps->frame_gaps.ifg2 = VTSS_IFG2_1G;

	ps->maxframelength = VTSS_MAX_FRAME_LENGTH; /* Max frame length. */

	ps->vlan_aware = FALSE;

	ps->drop_on_length_error = FALSE;

	ps->crc_update = FALSE;
	ps->fcs_modify = VTSS_FCS_DO_NOTHING;



	/* The following two fields are hw related */
	ps->invert_gtx_tx_clock = TRUE;  /* HW related */
	ps->invert_rx_clock     = FALSE; /* HW related */


	switch ( mmode) {
		/* valid major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
			ps->norm_hdr_insert = FALSE;
			ps->prm_hdr_insert = FALSE;
			ps->prm_hdr_value = 0;

			ps->norm_hdr_expect = FALSE;
			ps->prm_hdr_expect = FALSE;
			break;
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
			ps->norm_hdr_insert = TRUE;
			ps->prm_hdr_insert = FALSE;
			ps->prm_hdr_value = 0;

			ps->norm_hdr_expect = FALSE;
			ps->prm_hdr_expect = FALSE;
			break;
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			/* by default configured to preamble trunking */
			ps->norm_hdr_insert = TRUE;
			ps->prm_hdr_insert = TRUE;
			ps->prm_hdr_value = 0;

			ps->norm_hdr_expect = FALSE;
			ps->prm_hdr_expect = TRUE;
			break;

		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:
			/* these cases are needed only to make the compiler happy */
			return VTSS_WRONG_MAJOR_MODE;
	}


	return VTSS_OK;
}


/*----------------  Run-time funcitons  ---------------------------------------*/
/*******************************************************************************

  Function  vtss_port_set_mode( )
================================================================================

Description:
Configure speed and duplex mode of a 10M/100M/1G port

NOTE: 


Registers affected:
DEV_SETUP     0xB
MODE_CFG      0x0


Syntax
vtss_rc vtss_port_set_mode( vtss_port_no_t portnum, vtss_speed_t speed, 
BOOL fdx);

Arguments:
portnum   logical port number
speed     10M, 100M, 1G
fdx       TRUE - full duplex, FALSE -- half duplex

Return code:
VTSS_OK                 if operation completed successfully
VTSS_PORT_NOT_MAPPED    if the logical port is not mapped
VTSS_WRONG_PARAMETER    wrong value or combination of input parameters

*******************************************************************************/
vtss_rc vtss_port_set_mode( 	vtss_port_no_t 	portnum, 
				vtss_speed_t 	speed, 
				BOOL 		fdx)
{

	ulong dev_setup_reg = 0;
	ulong mode_cfg_reg = 0, mode_cfg_reg_mask = 0;
	ulong old_value = 0;
	uchar ifg1 = 0, ifg2 = 0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	/* 1G requre full duplex mode */
	if ((speed == VTSS_SPEED_1G) && (!fdx)) {
		return VTSS_WRONG_PARAMETER;
	}


	switch (speed) {
		case VTSS_SPEED_10M:
			SET_BIT( dev_setup_reg, 1);

			ifg1 = (fdx) ? VTSS_IFG1_10M_FDX : VTSS_IFG1_10M_HDX;
			ifg2 = (fdx) ? VTSS_IFG2_10M_FDX : VTSS_IFG2_10M_HDX;

			break;
		case VTSS_SPEED_100M:
			SET_BIT( dev_setup_reg, 2);

			ifg1 = (fdx) ? VTSS_IFG1_10M_FDX : VTSS_IFG1_10M_HDX;
			ifg2 = (fdx) ? VTSS_IFG2_10M_FDX : VTSS_IFG2_10M_HDX;

			break;
		case VTSS_SPEED_1G:
			if ( vtss_1G_ports_setup[ppn].interface_mode.interface_type == 
					VTSS_PORT_INTERFACE_SERDES) {
				SET_BIT( dev_setup_reg, 3);
			} else {
				SET_BIT( dev_setup_reg, 2);
				SET_BIT( dev_setup_reg, 1);
			}

			ifg1 = VTSS_IFG1_1G;
			ifg2 = VTSS_IFG2_1G;

			SET_BIT( mode_cfg_reg, 3); /*GIGA mode*/

			break;
		default:
			//    VTSS_E(("1G port #%d. Wrong speed: %d", ppn, speed));
			printk("1G port #%d. Wrong speed: %d\n", ppn, speed);
			return VTSS_WRONG_PARAMETER;
	}

	SET_BIT( mode_cfg_reg_mask, 3);

	/* Prepare port for reset */
	SET_BIT( dev_setup_reg, 0);

	if(fdx) {
		SET_BIT( mode_cfg_reg, 2);
	}
	SET_BIT( mode_cfg_reg_mask, 2);

	SET_BITS_MASKED( mode_cfg_reg, (ulong)ifg1<<6, (ulong)VTSS_IFG_MASK<<6);
	SET_BITS_MASKED( mode_cfg_reg, (ulong)ifg2<<10, (ulong)VTSS_IFG_MASK<<10);
	/* set the mode_cfg_reg mask */
	SET_BITS_MASKED( mode_cfg_reg_mask, (ulong)VTSS_IFG_MASK<<6, 
			(ulong)VTSS_IFG_MASK<<6);
	SET_BITS_MASKED( mode_cfg_reg_mask, (ulong)VTSS_IFG_MASK<<10, 
			(ulong)VTSS_IFG_MASK<<10);

	/* Read the current value */
	old_value = 0x3 & vtss_io_read( M2_BLK_MACS, ppn, M2_MODE_CFG); 
	/* Disable port */
	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_MODE_CFG, 0, 0x3);

	/* Change the mode */
	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_MODE_CFG,
			mode_cfg_reg, mode_cfg_reg_mask);

	/* Write device setup register. It will also reset the port */
	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_DEV_SETUP,
			dev_setup_reg, 0xF);

	/* leave reset state */
	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_DEV_SETUP,
			0, 0x1);

	/* Restore old RX and TX values */
	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_MODE_CFG, old_value, 0x3);


	/* Update global data */
	vtss_1G_ports_setup[ppn].interface_mode.speed = speed;
	vtss_1G_ports_setup[ppn].fdx = fdx;
	vtss_1G_ports_setup[ppn].frame_gaps.ifg1 = ifg1;
	vtss_1G_ports_setup[ppn].frame_gaps.ifg2 = ifg2;

	return VTSS_OK;
}


/*******************************************************************************

  Function vtss_port_set_enable()
================================================================================

Description:
Enable/disable rx, tx or both in a 10M/100M/1G port 

NOTE: 


Syntax
vtss_rc vtss_port_set_enable( vtss_port_no_t portnum, 
BOOL rx_en, BOOL tx_en);

Arguments:
portnum      logical port number
rx_en        if TRUE, enables reception of Ethernet frames
if FALSE, disables reception of Ethernet frames
tx_en        if TRUE, enables transmission of Ethernet frames
if FALSE, disables transmission of Ethernet frames

Return code:
VTSS_OK                if operation completed successfully
VTSS_PORT_NOT_MAPPED   if the logical port is not mapped

 *******************************************************************************/
vtss_rc vtss_port_set_enable( vtss_port_no_t portnum, BOOL rx_en, BOOL tx_en)
{
	ulong mode_cfg_reg = 0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;


	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	if(rx_en) {
		SET_BIT(mode_cfg_reg, 1);
	}
	if(tx_en) {
		SET_BIT(mode_cfg_reg, 0);
	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_MODE_CFG, mode_cfg_reg, 0x3);


	return VTSS_OK;
}


/*******************************************************************************

  Function vtss_port_fcs_modify()
================================================================================

Description:
Set CRC adding/updating mode (done in the denormalizer).

NOTE:
Only in egress direction.
SPI4 block must not discard frames with incorrect FCS.


Syntax
vtss_rc vtss_port_fcs_modify( vtss_port_no_t portnum, vtss_fcs_modify_t mc);

Arguments:
portnum     logical port number
mc          select the way FCS is modified

Return code:
VTSS_OK                 if operation completed successfully
VTSS_PORT_NOT_MAPPED    logical port not mapped

 *******************************************************************************/
vtss_rc vtss_port_fcs_modify( vtss_port_no_t portnum, vtss_fcs_modify_t mc)
{
	ulong denorm_reg = 0;
	ulong mask = (1<<5)|(1<<4);

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;


	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	/* Meigs2 has two independent bits: UPDATE and ADD, which should always have
	   opposite values */
	switch ( mc) {
		case  VTSS_FCS_DO_NOTHING:
			/* denorm_reg is already set to 0 */
			break;
		case VTSS_FCS_UPDATE:
			SET_BIT( denorm_reg, 5);
			/* CLR_BIT( denorm_reg, 4); denorm_reg is already set to 0 */
			break;
		case VTSS_FCS_ADD:
			SET_BIT( denorm_reg, 4);
			/* CLR_BIT( denorm_reg, 5); denorm_reg is already set to 0 */
			break;
	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_TRI_DENORM, denorm_reg, mask);


	return VTSS_OK;
}


/*******************************************************************************

  Function 
  ================================================================================

Description:
Discard/do_not_discard  Ethernet frames with wrong FCS in ingress direciton

NOTE: 
Not only frames with incorrect FCS will be allowed upstream, but also 
all wrong frames including fragments.

Syntax
vtss_rc vtss_port_check_fcs( vtss_port_no_t portnum, BOOL check);

Arguments:
portnum   logical port number
check     TRUE   -- frames with wrong FCS are discarded (default)
FALSE  -- frames with wrong FCS are forwarded upstream

Return code:
VTSS_OK   if operation completed successfully
VTSS_FEATURE_NOT_SUPPORTED
VTSS_PORT_NOT_MAPPED   logical port not mapped

 *******************************************************************************/
vtss_rc vtss_port_check_fcs( vtss_port_no_t portnum, BOOL check)
{

#if defined MEIGS2 || defined VSC7321

	/* MEIGS2 always discards ingress frames with bad FCS */
	/* it is not possible to disable this feature */
	return VTSS_FEATURE_NOT_SUPPORTED;

#else

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;

	ulong norm_reg_value = 0;
	ulong mdbg_reg_value = 0;
	ulong gress_test_reg = 0;
	ulong gress = M2_SUBBLK_INGRESS;

	vtss_device_setup_t *pvds = pdevice;

	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	/* Only one global device per API is currently supported */
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
			break;

		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			gress = M2_SUBBLK_EGRESS;
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}

	if (check) {
		SET_BIT( norm_reg_value, 9); /* NO_CRC update */
		SET_BITS_MASKED( mdbg_reg_value, 0x3, 0x3); /* SET_FAIL & KEEP_BAD */
		SET_BIT( gress_test_reg, 4); /* NO_DROP_IN_FRM */
	} else {
		/* bits are already cleared, just write them down */
	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_TRI_NORMALIZER, 
			norm_reg_value, 1<<9);

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2E_TRI_MULTI_DBG, 
			mdbg_reg_value, 0x3);

	vtss_io_writemasked(M2_BLK_FIFO, gress, ppn,
			gress_test_reg, 1<<4);

	return VTSS_OK;

#endif
}


/*******************************************************************************

  Function   vtss_port_forward_pause_frames( )
  ================================================================================

Description: 
Allow pause frames pass through the device. By default 10M/100M/1G block
prevents propagation of pause frames through the device.

NOTE: 

Syntax
vtss_rc vtss_port_forward_pause_frames( vtss_port_no_t portnum, BOOL allow);

Arguments:
portnum   number of the logical port
allow     if TRUE, allows pause frames pass through the device
if FALSE, pause frames will not be allowed to pass

Return code:
VTSS_OK               if operation completed successfully
VTSS_PORT_NOT_MAPPED  logical port is not mapped to any physical port

 *******************************************************************************/
vtss_rc vtss_port_forward_pause_frames( vtss_port_no_t portnum, BOOL allow)
{
	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_TRI_NORMALIZER, (allow)?0:(1<<6), 1<<6);
	return VTSS_OK;
}



/*******************************************************************************

  Function vtss_port_flow_control_mode( )

  ================================================================================

Description:
Select different modes of flow control on Ethernet interface.

Terminology:
obey        obeys pause frames received from Ethernet 
generate    generate pause frames/backpressure upon request from the 
destination (FIFO or the host interface)

NOTE:
If fc_obey AND fc_generate are set to FALSE, then flow control is disabled

IMPORTANT
VSC7321 supports only symmetrical flow control and requires both
arguments to be set to TRUE.

Syntax
vtss_rc vtss_port_flow_control_mode( vtss_port_no_t port_num, 
BOOL fc_obey, 
BOOL fc_generate)

Arguments:
obey        if TRUE, obeys pause frames from Ethernet
if FALSE, disregard pause frames from Ethernet
generate    if TRUE, generate pause frames/backpressure upon request 
from the destination block (FIFO or the host interface)


Return code:
VTSS_OK               if operation completed successfully
VTSS_PORT_NOT_MAPPED  logical port is not mapped to any physical port

 *******************************************************************************/
vtss_rc vtss_port_flow_control_mode( vtss_port_no_t port_num, 
		BOOL fc_obey, 
		BOOL fc_generate)
{
	ulong pause_reg_value = 0;
	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}



	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}


#if defined MEIGS2 || defined VSC7321

	if(fc_obey && fc_generate) {

		/* The only mode supported in Meigs-II*/
		SET_BIT( pause_reg_value, 16);
		/* IMPORTANT:                                                */
		/* tx_pause_value and mac_address must be already initialized */

	} else {
		/* disable flow control */
	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_PAUSE_CFG, pause_reg_value, 1<<16);

#else /* VSC7323 && VSC7331*/

	if(fc_obey && fc_generate) {
		SET_BIT( pause_reg_value, 16);
	} else if (fc_obey) {
		SET_BIT( pause_reg_value, 18);
		CLR_BIT( pause_reg_value, 16);
	} else if (fc_generate) {
		SET_BIT( pause_reg_value, 19);
		/* SET_BIT( pause_reg_value, 17); */
		CLR_BIT( pause_reg_value, 16);
	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_PAUSE_CFG, 
			pause_reg_value, 0xD<<16);

#endif

	return VTSS_OK;
}


/*******************************************************************************

  Function vtss_port_counters_clear(  )

  ================================================================================

Description:
Clears counter for the logical port 

NOTE:
In its current form this funciton clears counters in all 1G ports.
This behaviour will be corrected in the next release.


Syntax
vtss_rc vtss_port_counters_clear( const vtss_port_no_t portnum );

Arguments:


Return code:

 *******************************************************************************/
vtss_rc vtss_port_counters_clear( const vtss_port_no_t portnum )
{
	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	//  VTSS_N(("Port setup. Logic port #%d, phys prt #%d", portnum, ppn));
	//printk("Port setup. Logic port #%d, phys prt #%d\n", portnum, ppn);


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}


	/* Writing to this register clears counters in all 1G ports */
	vtss_io_write( M2_BLK_STAT, ppn, M2_STAT_INIT, 0x0);


	return VTSS_OK;
}



/*******************************************************************************

  Function  vtss_port_counters_get()

  ================================================================================

Description:
Clears counter for the logical port 

Syntax
vtss_rc vtss_port_counters_get( const vtss_port_no_t  portnum,
vtss_port_counters_t *pcounters)

Arguments:
portnum   number of the logical port
pc 

Return code:

 *******************************************************************************/
vtss_rc vtss_port_counters_get( const vtss_port_no_t  portnum,
		vtss_port_counters_t *pc)
{
	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	//  VTSS_N(("Port setup. Logic port #%d, phys prt #%d", portnum, ppn));
	//	printk("Port setup. Logic port #%d, phys prt #%d\n", portnum, ppn);
	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}


	pc->rx_in_bytes = vtss_io_read(M2_BLK_STAT, ppn, M2_RX_IN_BYTES);
	pc->rx_symbol_carrier = vtss_io_read( M2_BLK_STAT, ppn, 
			M2_RX_SYMBOL_CARRIER);
	pc->rx_pause = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_PAUSE);
	pc->rx_unsup_opcode = vtss_io_read( M2_BLK_STAT, ppn, 
					M2_RX_UNSUP_OPCODE);

	pc->rx_ok_bytes = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_OK_BYTES);
	pc->rx_bad_bytes = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_BAD_BYTES);

	pc->rx_unicast = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_UNICAST);
	pc->rx_multicast = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_MULTICAST);
	pc->rx_broadcast = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_BROADCAST);
	pc->rx_crc = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_CRC);
	pc->rx_alignment = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_ALIGNMENT); 
	pc->rx_undersize = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_UNDERSIZE);

	pc->rx_fragments = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_FRAGMENTS);
	pc->rx_in_range_error = vtss_io_read( M2_BLK_STAT, ppn, 
			M2_RX_IN_RANGE_LENGTH_ERROR);
	pc->rx_out_of_range_error = vtss_io_read( M2_BLK_STAT, ppn, 
			M2_RX_OUT_OF_RANGE_ERROR);
	pc->rx_oversize = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_OVERSIZE);
	pc->rx_jabbers = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_JABBERS);
	pc->rx_size_64 = vtss_io_read( M2_BLK_STAT, ppn, M2_RX_SIZE_64);

	pc->rx_size_65_to_127 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_RX_SIZE_65);
	pc->rx_size_128_to_255 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_RX_SIZE_128);
	pc->rx_size_256_to_511 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_RX_SIZE_256);
	pc->rx_size_512_to_1023 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_RX_SIZE_512);
	pc->rx_size_1024_to_1518 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_RX_SIZE_1024);
	pc->rx_size_1519_to_max = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_RX_SIZE_1519);

	pc->rx_xgmii_prot_err = 0; /* Only on 10G MAC port */

	pc->rx_ipg_shrink = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_RX_IPG_SHRINK);


	pc->tx_out_bytes = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_OUT_BYTES);
	pc->tx_pause = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_PAUSE);
	pc->tx_ok_bytes = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_OK_BYTES);

	pc->tx_unicast = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_UNICAST);
	pc->tx_multicast = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_MULTICAST);
	pc->tx_broadcast = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BROADCAST);

	pc->tx_multiple_coll = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_MULTIPLE_COLL);
	pc->tx_late_coll = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_LATE_COL);
	pc->tx_xcoll = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_XCOLL);
	pc->tx_defer = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_DEFER);
	pc->tx_xdefer = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_XDEFER);
	pc->tx_carrier_sense = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_CSENSE);


	pc->tx_size_64 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_SIZE_64);
	pc->tx_size_65_to_127 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_SIZE_65);
	pc->tx_size_128_to_255 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_SIZE_128);
	pc->tx_size_256_to_511 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_SIZE_256);
	pc->tx_size_512_to_1023 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_SIZE_512);
	pc->tx_size_1024_to_1518 = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_SIZE_1024);
	pc->tx_size_1519_to_max = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_SIZE_1519);


	pc->tx_single_coll = vtss_io_read( M2_BLK_STAT, ppn, 
						M2_TX_SINGLE_COL);
	pc->tx_backoff2 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF2);
	pc->tx_backoff3 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF3);
	pc->tx_backoff4 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF4);
	pc->tx_backoff5 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF5);
	pc->tx_backoff6 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF6);
	pc->tx_backoff7 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF7);
	pc->tx_backoff8 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF8);
	pc->tx_backoff9 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF9);
	pc->tx_backoff10 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF10);
	pc->tx_backoff11 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF11);
	pc->tx_backoff12 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF12);
	pc->tx_backoff13 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF13);
	pc->tx_backoff14 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF14);
	pc->tx_backoff15 = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_BACKOFF15);

	pc->tx_underrun = vtss_io_read( M2_BLK_STAT, ppn, M2_TX_UNDERRUN);


	/* 10G is not properly being handled for the drop counters */
	/* It must reflect the major mode: 10G as single or multi channel, kbp */

	pc->ingress_overflow_drop = 
		vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_FIFO_DROP_CNT+ppn);
	pc->egress_overflow_drop = 
		vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_FIFO_DROP_CNT+ppn);


	return VTSS_OK;
}


/*******************************************************************************

  Function   

  ================================================================================

Description:
Configure 1G MAC block to insert normalisation of incoming ethernet frames.
Works in the direction from 1G MAC to the ingress FIFO.

NOTE. If an element in the 'hdr' structure is set to FALSE, that type of 
header is left unmodified. 

Syntax



Arguments:
ppn        physical port (port-on-chip)
enable     action to be taken -- enable or disable header insertion
hdr        normalized header/preamble header structure


Return code:

 *******************************************************************************/
static void vtss_port1G_header_insert( int ppn, BOOL enable, 
		const m2_header_t hdr)
{
	ulong reg = 0, reg_mask = 0;

	if ( hdr.use_norm_hdr) {

		if (enable == TRUE) {
			reg |= M2_NORMALIZER_BIT_NH;
			/* Norm hdr requires NLE bit to be set */
			reg |= M2_NORMALIZER_BIT_NLE;
		}
		/* else -- do nothing, the bits are already cleared */

		reg_mask |= M2_NORMALIZER_BIT_NH;
		reg_mask |= M2_NORMALIZER_BIT_NLE;
	}

	if ( hdr.use_prm_hdr) {
		if (enable == TRUE)
			reg |= M2_NORMALIZER_BIT_PH;

		reg_mask |= M2_NORMALIZER_BIT_PH;
	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_NORMALIZER, reg, reg_mask);

	return;
}


/*******************************************************************************
 *
 *      Autonegotiation functions. Used when TBI or Serdes is enabled.
 *
 ******************************************************************************/

/*
   Returns the autonegotiation advertisment word and the status (enabled/disabled) 
   of the autonegotiation.
 */
vtss_rc vtss_pcs_autoneg_control_get( const vtss_port_no_t  port_num,
		vtss_pcs_autoneg_control_t * const        control )
{

	ulong reg_value = 0;


	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}



	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	reg_value = vtss_io_read( M2_BLK_MACS, ppn, M2_PCS_CTRL);


	control->advertisement.fdx = ( (reg_value >> 5) & 0x1) ? TRUE : FALSE;
	control->advertisement.hdx = ( (reg_value >> 6) & 0x1) ? TRUE : FALSE;

	control->advertisement.symmetric_pause =   /* a.k.a. PAUSE (PS1) */
		((reg_value >> 7) & 0x1) ? TRUE : FALSE;

	control->advertisement.asymmetric_pause = /* a.k.a. ASM_DIR (PS2) */
		((reg_value >> 8) & 0x1) ? TRUE : FALSE;

	switch ( ( reg_value >> 12) & 0x3) {

	case VTSS_1000BASEX_LINK_OK:
		control->advertisement.remote_fault = VTSS_1000BASEX_LINK_OK;
		break;
	case VTSS_1000BASEX_OFFLINE:
		control->advertisement.remote_fault = VTSS_1000BASEX_OFFLINE;
		break;
	case VTSS_1000BASEX_LINK_FAILURE:
		control->advertisement.remote_fault = VTSS_1000BASEX_LINK_FAILURE;
		break;
	case VTSS_1000BASEX_AUTONEG_ERROR:
		control->advertisement.remote_fault = VTSS_1000BASEX_AUTONEG_ERROR;
		break;
	}


	control->advertisement.acknowledge = ((reg_value>>14) & 0x1)?TRUE:FALSE;

	control->advertisement.next_page = ( ( reg_value>>15) & 0x1)?TRUE:FALSE;


	control->enable = ( ( reg_value>>17) & 0x1)?TRUE:FALSE;

	return VTSS_OK;
}

/*
   Enables/disables autonegotiation in the PCS module and sets advertisment word
 */
vtss_rc vtss_pcs_autoneg_control_set( const vtss_port_no_t                      port_num,
		const vtss_pcs_autoneg_control_t * const  control )
{

	ulong reg_value = 0, reg_mask = 0;


	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}

	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	if ( control->enable == TRUE) {
		/* Autonegotiation must be enabled and the advertisment word must be 
		   written down */

		if ( control->advertisement.fdx == TRUE) {
			SET_BIT( reg_value, 5);
		} 
		SET_BIT(reg_mask, 5);

		if ( control->advertisement.hdx) {
			SET_BIT( reg_value, 6);
		}
		SET_BIT( reg_mask, 6);

		if ( control->advertisement.symmetric_pause) { /* a.k.a. PAUSE (PS1) */
			SET_BIT( reg_value, 7);
		}
		SET_BIT(reg_mask, 7);

		if ( control->advertisement.asymmetric_pause) { /* a.k.a. ASM_DIR (PS2) */
			SET_BIT( reg_value, 8);
		}
		SET_BIT( reg_mask, 8);

		switch ( control->advertisement.remote_fault) {
			case VTSS_1000BASEX_LINK_OK:
				CLR_BIT( reg_value, 12);
				CLR_BIT( reg_value, 13);
				break;
			case VTSS_1000BASEX_OFFLINE:
				CLR_BIT( reg_value, 12);
				SET_BIT( reg_value, 13);
				break;
			case VTSS_1000BASEX_LINK_FAILURE:
				SET_BIT( reg_value, 12);
				CLR_BIT( reg_value, 13);
				break;
			case VTSS_1000BASEX_AUTONEG_ERROR:
				SET_BIT( reg_value, 12);
				SET_BIT( reg_value, 13);
				break;
		}
		SET_BIT( reg_mask, 12);
		SET_BIT( reg_mask, 13);


		if( control->advertisement.acknowledge == TRUE) {
			SET_BIT( reg_value, 14);
		}
		SET_BIT( reg_mask, 14);

		if ( control->advertisement.next_page == TRUE) {
			SET_BIT( reg_value, 15);
		}
		SET_BIT( reg_mask, 15);


		/* enable and restart autoneg bits */
		SET_BIT( reg_value, 16);
		SET_BIT( reg_value, 17);

		SET_BIT( reg_mask, 16);
		SET_BIT( reg_mask, 17);

		/* and the write_strobe: used in Meigs2, harmless in VSC7323/7331 */
		SET_BIT( reg_value, 31);
		SET_BIT( reg_mask, 31);

	} else {
		/* restart autoneg bits must be set when disabling autoneg */
		SET_BIT( reg_value, 16);
		/* disable autoneg */
		CLR_BIT( reg_value, 17);

		SET_BIT( reg_mask, 16);
		SET_BIT( reg_mask, 17);

		/* and the write_strobe: used in Meigs2, harmless in VSC7323/7331 */
		SET_BIT( reg_value, 31);
		SET_BIT( reg_mask, 31);

	}

	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_PCS_CTRL, reg_value, reg_mask);

	return VTSS_OK;
}


/*
   Restarts autonegotiation, does not check whether autonegotiation is enabled
 */
vtss_rc vtss_pcs_autoneg_restart( const vtss_port_no_t  port_num )
{

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}



	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}


	/* Autoneg restart bit#16
	   Write strobe bit#31 -- used in VSC7321, harmless in VSC7323/7331
	 */
	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_PCS_CTRL, 
			0x8001<<16, 0x8001<<16);

	return VTSS_OK;
}


/*  
    Return the status of the current status of the link partner
 */
vtss_rc vtss_pcs_autoneg_status_get( const vtss_port_no_t port_num,
		vtss_pcs_autoneg_status_t *paneg)
{

	ulong reg_value = 0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	reg_value = vtss_io_read( M2_BLK_MACS, ppn, M2_PCS_STATUS);


	/*  Read XMIT_MODE: bits #18-19  */
	switch ( ( reg_value >> 18) & 0x3) {

		case VTSS_PCS_ANEG_STATE_IDLE:      /* Idle */
			paneg->aneg_state = VTSS_PCS_ANEG_STATE_IDLE;
			break;
		case VTSS_PCS_ANEG_STATE_CONFIG:    /* Config (i.e. ANEG in progress) */
			paneg->aneg_state = VTSS_PCS_ANEG_STATE_CONFIG;
			break;
		case VTSS_PCS_ANEG_STATE_DATA:      /* Data */
			paneg->aneg_state = VTSS_PCS_ANEG_STATE_DATA;
			break;
		default:
			paneg->aneg_state = VTSS_PCS_ANEG_STATE_NOTVALID;
	}

	/* ANC:  autoneg complete bit #16 */
	paneg->aneg_complete = ( (reg_value >> 16) & 0x1) ? TRUE : FALSE;

	paneg->partner_advertisement.fdx = ( (reg_value >> 5) & 0x1) ? TRUE : FALSE;
	paneg->partner_advertisement.hdx = ( (reg_value >> 6) & 0x1) ? TRUE : FALSE;

	paneg->partner_advertisement.symmetric_pause =   /* a.k.a. PAUSE (PS1) */
		((reg_value >> 7) & 0x1) ? TRUE : FALSE;

	paneg->partner_advertisement.asymmetric_pause = /* a.k.a. ASM_DIR (PS2) */
		((reg_value >> 8) & 0x1) ? TRUE : FALSE;



	switch ( ( reg_value >> 12) & 0x3) {

		case VTSS_1000BASEX_LINK_OK:
			paneg->partner_advertisement.remote_fault = VTSS_1000BASEX_LINK_OK;
			break;
		case VTSS_1000BASEX_OFFLINE:
			paneg->partner_advertisement.remote_fault = VTSS_1000BASEX_OFFLINE;
			break;
		case VTSS_1000BASEX_LINK_FAILURE:
			paneg->partner_advertisement.remote_fault = VTSS_1000BASEX_LINK_FAILURE;
			break;
		case VTSS_1000BASEX_AUTONEG_ERROR:
			paneg->partner_advertisement.remote_fault = VTSS_1000BASEX_AUTONEG_ERROR;
			break;
	}


	paneg->partner_advertisement.acknowledge = ((reg_value>>14) & 0x1)?TRUE:FALSE;

	paneg->partner_advertisement.next_page = ( ( reg_value>>15) & 0x1)?TRUE:FALSE;

	return VTSS_OK;
}


/* Returns more detailed status than vtss_pcs_autoneg_status_get */
vtss_rc vtss_pcs_status_get( const vtss_port_no_t       port_num,
		vtss_pcs_status_t * const  pstatus )
{

	ulong pcs_status = 0;
	ulong pcs_ctrl = 0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	pcs_status = vtss_io_read( M2_BLK_MACS, ppn, M2_PCS_STATUS);
	pcs_ctrl = vtss_io_read( M2_BLK_MACS, ppn, M2_PCS_CTRL);


	/*  Read XMIT_MODE: bits #18-19  */
	switch ( ( pcs_status >> 18) & 0x3) {

		case VTSS_PCS_ANEG_STATE_IDLE:  /* Idle */
			pstatus->autoneg.aneg_state = VTSS_PCS_ANEG_STATE_IDLE;
			break;
		case VTSS_PCS_ANEG_STATE_CONFIG:    /* Config (i.e. ANEG in progress) */
			pstatus->autoneg.aneg_state = VTSS_PCS_ANEG_STATE_CONFIG;
			break;
		case VTSS_PCS_ANEG_STATE_DATA:       /* Data */
			pstatus->autoneg.aneg_state = VTSS_PCS_ANEG_STATE_DATA;
			break;
		default:
			pstatus->autoneg.aneg_state = VTSS_PCS_ANEG_STATE_NOTVALID;
	}

	/* ANC:  autoneg complete bit #16 */
	pstatus->autoneg.aneg_complete = ( (pcs_status >> 16) & 0x1) ? TRUE : FALSE;

	/* SHOW_LDC_TOP bit of the PCS_CTRL register (bit #25) selects how to 
	   interpret bits 24..31 of the PCS_STATUS register */
	if ( ( pcs_ctrl >> 25) & 0x1) { /* the link down counter is 8 bit long */

		pstatus->show_ldc_top = TRUE;

		pstatus->losync = FALSE;

		pstatus->pcs_in_sync = FALSE;

		/* link down counter is 8bit long */
		pstatus->link_down_counter = (pcs_status >> 24) & 0xFF;

	} else { /* the link down counter is 6 bit long */

		pstatus->show_ldc_top = FALSE;;

		pstatus->losync = 
			((pcs_status >> 31) & 0x1) ? TRUE : FALSE;

		pstatus->pcs_in_sync = 
			((pcs_status >> 30) & 0x1) ? TRUE : FALSE;


		/* link down counter is only 6 bits long */
		pstatus->link_down_counter = (pcs_status >> 24) & 0x3F;

	}


	/* signal_detected indicates that there is light in the fiber; 
	   require SD_EN in the PCS_CTRL register to be set */
	pstatus->signal_detected = 
		((pcs_status >> 23) & 0x1) ? TRUE : FALSE;


	pstatus->jtp_lock = 
		((pcs_status >> 22) & 0x1) ? TRUE : FALSE;

	pstatus->jtp_error = 
		((pcs_status >> 21) & 0x1) ? TRUE : FALSE;


	/* link_status:  FALSE if link has been down since last status read */
	pstatus->link_status_ok =
		((pcs_status >> 20) & 0x1) ? TRUE : FALSE;




	pstatus->autoneg.partner_advertisement.fdx = ( (pcs_status >> 5) & 0x1) ? TRUE : FALSE;
	pstatus->autoneg.partner_advertisement.hdx = ( (pcs_status >> 6) & 0x1) ? TRUE : FALSE;

	pstatus->autoneg.partner_advertisement.symmetric_pause =   /* a.k.a. PAUSE (PS1) */
		((pcs_status >> 7) & 0x1) ? TRUE : FALSE;

	pstatus->autoneg.partner_advertisement.asymmetric_pause = /* a.k.a. ASM_DIR (PS2) */
		((pcs_status >> 8) & 0x1) ? TRUE : FALSE;



	switch ( ( pcs_status >> 12) & 0x3) {

		case VTSS_1000BASEX_LINK_OK:
			pstatus->autoneg.partner_advertisement.remote_fault = VTSS_1000BASEX_LINK_OK;
			break;
		case VTSS_1000BASEX_OFFLINE:
			pstatus->autoneg.partner_advertisement.remote_fault = VTSS_1000BASEX_OFFLINE;
			break;
		case VTSS_1000BASEX_LINK_FAILURE:
			pstatus->autoneg.partner_advertisement.remote_fault = VTSS_1000BASEX_LINK_FAILURE;
			break;
		case VTSS_1000BASEX_AUTONEG_ERROR:
			pstatus->autoneg.partner_advertisement.remote_fault = VTSS_1000BASEX_AUTONEG_ERROR;
			break;
	}


	pstatus->autoneg.partner_advertisement.acknowledge = ((pcs_status>>14) & 0x1)?TRUE:FALSE;

	pstatus->autoneg.partner_advertisement.next_page = ( ( pcs_status>>15) & 0x1)?TRUE:FALSE;

	return VTSS_OK;
}


vtss_rc vtss_serdes_signal_detect_setup( const vtss_port_no_t port_num,
		BOOL enable,
		BOOL sd_polarity_high,
		BOOL sd_source_extern)
{

	ulong reg = 0, mask = 0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}



	if ( enable == TRUE) {

		/* set Signal Detect source */
		if ( sd_source_extern == TRUE)
			SET_BIT( reg, 8);
		SET_BIT( mask, 8);
		vtss_io_writemasked( M2_BLK_MACS, ppn, M2_DEV_SETUP, reg, mask);
		reg = mask = 0;

		/* and now Polarity and Enable */
		SET_BIT( reg, 22); /* Signal Detect Enable */

		if ( sd_polarity_high == TRUE)
			SET_BIT( reg, 23); /* Signal Detect Polarity */

		SET_BIT( mask, 22);
		SET_BIT( mask, 23);

	} else {
		/* Disable serdes Signal Detect */
		SET_BIT( mask, 22);
	}

	/* write_strobe used in Meigs2, otherwise harmless */
	SET_BIT( reg, 31);
	SET_BIT( mask, 31);


	vtss_io_writemasked( M2_BLK_MACS, ppn, M2_PCS_CTRL, reg, mask);



	return VTSS_OK;

}



vtss_rc vtss_serdes_extern_signal_detect_status_get(const vtss_port_no_t port_num, 
		BOOL *pstatus)
{
	ulong reg = 0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_num].chip_port;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* right major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR: /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* single chip trunking */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}


	reg = vtss_io_read( M2_BLK_MACS, ppn, M2_PCS_STATUS);

	if ( (reg >> 23) & 0x1 ) { /* bit 23 is set */
		*pstatus = TRUE;
	} else {
		*pstatus = FALSE;
	}

	return VTSS_OK;
}


/*****************************************************************************/
/*                                                                           */
/*        10G port setup                                                     */
/*                                                                           */
/*****************************************************************************/


/*--       Setup  Funtions                            -----------------------*/


/* Setup 10G port */
vtss_rc vtss_10Gport_setup( vtss_port_no_t port_num, vtss_10Gport_setup_t* ps)
{


	ulong misc_reg_value = 0;
	ulong reg_mask = 0;
	ulong reg_value = 0;

	ulong max_len_reg_value = 0;

	ulong pause_reg_value = 0;

	ulong denorm_reg = 0;

	m2_header_t hdr;


	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;


	//  VTSS_N(("10G port setup."));
	printk("10G port setup.\n");

	/* check for appropriate major mode */
	switch ( pvds->mmode) {
		/* Suppported modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:      /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:
			break;

			/* Undefined major mode */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;
			/* Wrong mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:    /* SPI4 <-> 10x1G */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}



	if (VTSS_SPEED_10G != ps->interface_mode.speed) {
		printk("10G port. Wrong speed: %d\n", ps->interface_mode.speed);
		return VTSS_WRONG_PARAMETER;
	}

	if (VTSS_PORT_INTERFACE_XAUI != ps->interface_mode.interface_type) {
		printk("10G port. Wrong interface type: %d\n", 
					ps->interface_mode.interface_type);
		return VTSS_WRONG_PARAMETER;
	}

	/* -------------------  MISC  ------------------------- */

	/* Enable the 10G block */
	SET_BIT( misc_reg_value, 1);
	SET_BIT( misc_reg_value, 0);

	if ( ps->vlan_aware)           { SET_BIT( misc_reg_value, 2); }
	if ( ps->prm_hdr_insert)       { SET_BIT( misc_reg_value, 3); }
	if ( ps->pace_mode)            { SET_BIT( misc_reg_value, 5); }
	if ( ps->drop_in_range_error)  { SET_BIT( misc_reg_value, 6); }
	if ( ps->drop_on_length_error) { SET_BIT( misc_reg_value, 7); }
	if ( ps->sfd_check)            { SET_BIT( misc_reg_value, 8); }

	/* It is recommended to set 3-bit LFS_MODE bitfield to 000b */
	SET_BITS_MASKED( misc_reg_value, 0, (ulong)7<<9);
	SET_BITS_MASKED( reg_mask, (ulong)7<<9, (ulong)7<<9);

	if ( ps->ext_sop_check_enable) { SET_BIT( misc_reg_value, 13); }
	if ( ps->ext_eop_check_enable) { SET_BIT( misc_reg_value, 14); }

	/* Before writing the value down to the register, take the 10g block 
	   out of reset */
	CLR_BIT( misc_reg_value, 31);

	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_MISC_10G, 
			misc_reg_value, 0XFFFFFFFF);


	/* -------------------  DENORM  ------------------------- */

	/* For this to work SPI4(when used) must be set to let bad frames 
	   pass through */
	switch ( ps->fcs_modify) {
		case  VTSS_FCS_DO_NOTHING:
			/* bits are cleared, do nothing */
			break;
		case VTSS_FCS_UPDATE:
			SET_BIT( denorm_reg, 5);
			/* bit #4 is cleared */
			break;
		case VTSS_FCS_ADD:
			SET_BIT( denorm_reg, 4);
			/* bit #5 is cleared */
			break;
	}

	if( ps->norm_hdr_expect == TRUE) { SET_BIT(denorm_reg, 1); }
	if( ps->prm_hdr_expect  == TRUE) { SET_BIT(denorm_reg, 0); }

	/* Denormalizer register */
	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_DENORM, denorm_reg, 0x3f);


	/* -------------------  Flowcontrol  ----------------------- */

	if(ps->flowcontrol.obey || ps->flowcontrol.generate) {

		/* Mac address will be written if flowcontrol setup is requested */
		ulong mac_addr_low_reg = 
			(ps->flowcontrol.smac.addr[3] << 16)| 
			(ps->flowcontrol.smac.addr[4] <<  8)|
			ps->flowcontrol.smac.addr[5];

		ulong mac_addr_high_reg = 
			(ps->flowcontrol.smac.addr[0] << 16)| 
			(ps->flowcontrol.smac.addr[1] <<  8)|
			ps->flowcontrol.smac.addr[2];

		vtss_io_write( M2_BLK_MACS, M2_SUBBLK_MAC_10G, 
				M2_MAC_HIGH_ADDR, mac_addr_high_reg);
		vtss_io_write( M2_BLK_MACS, M2_SUBBLK_MAC_10G, 
				M2_MAC_LOW_ADDR, mac_addr_low_reg);

		if (ps->flowcontrol.obey) {
			SET_BIT( pause_reg_value, 17);
		}
		if (ps->flowcontrol.generate) {
			SET_BIT( pause_reg_value, 16);
		}


	}


	if( ps->enable_tx_pause_xon_xoff == TRUE) {
		SET_BIT( pause_reg_value, 18);
	}

	/*  TX pause value */
	SET_BITS_MASKED( pause_reg_value, ps->tx_pause_value,
			VTSS_PORT_10G_TX_PAUSE_MASK);
	vtss_io_write( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_PAUSE, pause_reg_value);


	/* maxframelength */
	SET_BITS_MASKED( max_len_reg_value, ps->maxframelength, 
			VTSS_MAX_FRAME_LENGTH_MASK);

	vtss_io_write( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_MAX_LEN, max_len_reg_value);



	/* -------------------  NORMALIZER  ------------------------- */
	/* RxChainMode must be set in the system block */

	/* 10G port always reads data from the FIFO (i.e. in egress direction) on 
	   frame-interleaved basis, thus normalised header is needed */
	/* Chip major mode must be defined at this point */
	hdr.use_norm_hdr = TRUE;
	hdr.use_prm_hdr = FALSE;
	//VTSS_ASSERT( VTSS_OK == vtss_hdr_expect( TRUE, hdr));

	reg_value = reg_mask =  0;
	if ( ps->norm_hdr_insert == TRUE ) {
		SET_BIT(reg_value, 1); /* NH bit */
		/* NLE must be cleared */
	} else {
		/* witout normalized header NLE must be set */
		SET_BIT(reg_value, 2);
	}
	SET_BIT(reg_mask, 1); /* NH bit */
	SET_BIT(reg_mask, 2); /* NLE bit */

	if ( ps->prm_hdr_insert == TRUE) {
		SET_BIT( reg_value, 0);
	}
	SET_BIT( reg_mask, 0);

	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_NORMALIZER,
			reg_value, reg_mask);


	/* -------------------  XAUI  ------------------------- */


	/* Power-up control */
	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_XAUI_CONF_B,
			0x4, 0x7);


	/* Reset the block */
	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_XAUI_CONF_A,
			(ulong)1<<31, (ulong)1<<31);
	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_XAUI_CONF_A,
			0, (ulong)1<<31);
	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_XAUI_CONF_A,
			(ulong)1<<31, (ulong)1<<31);

	return VTSS_OK;
}


/* 
   Supplementary function: fills the structure referenced by the pointer 
   with the default values for a given major_mode.
 */
vtss_rc vtss_10Gport_setup_get_default_values( vtss_10Gport_setup_t* psetup,
		vtss_mac_major_mode_t major_mode)
{
	vtss_mac_t smac_addr = {{0,0,0,0,0,0}};

	switch ( major_mode) {
		/* Suppported modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:      /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:
			break;

			/* Wrong mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:    /* SPI4 <-> 10x1G */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}

	psetup->interface_mode.interface_type = VTSS_PORT_INTERFACE_XAUI;
	psetup->interface_mode.speed          = VTSS_SPEED_10G;


	psetup->flowcontrol.obey     = TRUE;
	psetup->flowcontrol.generate = TRUE;
	psetup->flowcontrol.smac     = smac_addr;


	psetup->enable_tx_pause_xon_xoff = TRUE;
	psetup->tx_pause_value           = VTSS_PORT_10G_TX_PAUSE_VALUE;


	psetup->maxframelength = VTSS_MAX_FRAME_LENGTH; /* Max frame length. */

	psetup->vlan_aware = FALSE;
	psetup->pace_mode = FALSE;
	psetup->drop_on_length_error = FALSE;
	psetup->drop_in_range_error = FALSE;

	psetup->fcs_modify = VTSS_FCS_DO_NOTHING;

	psetup->ext_eop_check_enable = FALSE;
	psetup->ext_sop_check_enable = FALSE;



	switch ( major_mode) {
		/* Suppported modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:      /* SPI4 <-> 1x10G */
			psetup->norm_hdr_insert = FALSE;
			psetup->prm_hdr_insert = FALSE;
			psetup->norm_hdr_expect = FALSE;
			psetup->prm_hdr_expect = FALSE;
			psetup->sfd_check = TRUE;
			break;
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:
			psetup->norm_hdr_insert = FALSE;
			psetup->prm_hdr_insert = FALSE;
			psetup->norm_hdr_expect = TRUE;
			psetup->prm_hdr_expect = FALSE;
			psetup->sfd_check = TRUE;
			break;
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:
			psetup->norm_hdr_insert = FALSE;
			psetup->prm_hdr_insert = TRUE;
			psetup->norm_hdr_expect = TRUE;
			psetup->prm_hdr_expect = TRUE;
			psetup->sfd_check = FALSE;
			break;
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:
			psetup->norm_hdr_insert = TRUE;
			psetup->prm_hdr_insert = FALSE;
			psetup->norm_hdr_expect = TRUE;
			psetup->prm_hdr_expect = FALSE;
			psetup->sfd_check = TRUE;
			break;
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:
			psetup->norm_hdr_insert = TRUE;
			psetup->prm_hdr_insert = TRUE;
			psetup->norm_hdr_expect = TRUE;
			psetup->prm_hdr_expect = TRUE;
			psetup->sfd_check = FALSE;
			break;

			/* Wrong mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:    /* SPI4 <-> 10x1G */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}

	return VTSS_OK;

}


/*----------------  Run-time funcitons  ---------------------------------------*/
/* Enable/disable rx, tx or both in a 10G port */
vtss_rc vtss_10Gport_set_enable( vtss_port_no_t port_num, 
		BOOL rx_en, BOOL tx_en)
{

	ulong mode_cfg_reg = 0;
	ulong mode_cfg_reg_mask = 0x3; /* two bits are affected */


	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;

	/* check for appropriate major mode */
	switch ( pvds->mmode) {
		/* Suppported modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:      /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:
			break;

			/* Undefined major mode */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;
			/* Wrong mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:    /* SPI4 <-> 10x1G */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	if(rx_en) {
		SET_BIT(mode_cfg_reg, 1);
	}
	if(tx_en) {
		SET_BIT(mode_cfg_reg, 0);
	}


	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_MISC_10G, 
			mode_cfg_reg, mode_cfg_reg_mask);


	return VTSS_OK;
}


/* Set CRC adding/updating mode */
vtss_rc vtss_10Gport_fcs_modify( vtss_port_no_t port_num, vtss_fcs_modify_t mc)
{

	ulong denorm_reg = 0;
	ulong denorm_reg_mask = (1<<5)|(1<<4);

	/* Check for the right major mode */
	/* TBD */

	/* Meigs2 has two independent bits: UPDATE and ADD, which should always have
	   opposite values */
	switch ( mc) {
		case  VTSS_FCS_DO_NOTHING:
			break;
		case VTSS_FCS_UPDATE:
			SET_BIT( denorm_reg, 5);
			CLR_BIT( denorm_reg, 4);
			break;
		case VTSS_FCS_ADD:
			SET_BIT( denorm_reg, 4);
			CLR_BIT( denorm_reg, 5);
			break;
	}

	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_DENORM, 
			denorm_reg, denorm_reg_mask);


	return VTSS_OK;

}


/* Allow pause frames pass through the device. By default 10G block
   prevents propagation of pause frames through the device.
 */
vtss_rc vtss_10Gport_forward_pause_frames( vtss_port_no_t port_num, BOOL allow)
{

	/*!!! DROP_PAUSE bit (bit #5) is different from its counterpart in 1G ports*/
	ulong norm_reg = (allow)?0:(1<<5);
	ulong norm_reg_mask = 1<<5;

	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_NORMALIZER, 
			norm_reg, norm_reg_mask);
	return VTSS_OK;
}

/*
   obey -- obeys pause frames from external Eth client 
   generate -- generate pause frames/backpressure upon request from the 
   destination (FIFO or the host interface)

   If fc_obey AND fc_generate are set to FALSE, then flow control is disabled


 */
vtss_rc vtss_10Gport_flow_control_mode( vtss_port_no_t port_num, 
		BOOL fc_obey, 
		BOOL fc_generate)
{
	ulong pause_reg_value = 0;
	ulong pause_reg_mask = ((ulong)1<<16)|(1<<17);



	if (fc_obey) {
		SET_BIT( pause_reg_value, 17);
	} 

	if (fc_generate) {
		SET_BIT( pause_reg_value, 16);
	}

	vtss_io_writemasked( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_PAUSE, 
			pause_reg_value, pause_reg_mask);


	return VTSS_OK;
}


/*******************************************************************************

  Function   vtss_10Gport_signal_detect

  ================================================================================

Description:
Once a 10G Base-X optical module is connected to the XAUI port,
this function can use to drive an GPIO connected LED to visualize 
signal detection/link status. The CPU must poll this function.

Syntax

Arguments: None

Return code: TRUE/FALSE

 *******************************************************************************/
BOOL vtss_10Gport_signal_detect(void)
{
	ulong mac_tx_sticky_value = 0;

	/* Clear sticky-bits in MAC_TX_STICKY */
	vtss_io_write( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_MAC_TX_STICKY, 0x1f);
	mac_tx_sticky_value = vtss_io_read( M2_BLK_MACS, M2_SUBBLK_MAC_10G, M2_MAC_TX_STICKY);

	return (mac_tx_sticky_value & ((ulong)1 << 4))?(FALSE):(TRUE);
}


/*******************************************************************************

  Function   

  ================================================================================

Description:
Works in the direction from MAC to FIFO.

Syntax

Arguments:
hdr   normalized header/preamble header structure

Return code:

 *******************************************************************************/
static void vtss_port10G_header_insert( BOOL enable, const m2_header_t hdr)
{
	ulong reg = 0, reg_mask = 0;

	if ( hdr.use_norm_hdr) {

		if( enable == TRUE) {
			SET_BIT( reg, 1);
			/* Normalized header requires the NLE bit set */
			SET_BIT( reg, 2);
		}

		SET_BIT( reg_mask,1);
		SET_BIT( reg_mask,2);
	}


	if ( hdr.use_prm_hdr) {
		if ( enable == TRUE)
			SET_BIT( reg, 0);

		SET_BIT( reg_mask, 0);
	}


	/* If any changes, write to the register */
	if( reg_mask) {
		vtss_io_writemasked( M2_BLK_MACS, M2_PHYS_PORT_10G, M2_NORMALIZER, 
				reg, reg_mask);
	}

	return;
}

/*******************************************************************************

  Function   

  ================================================================================

Description:
FIFO -> MAC

Syntax



Arguments:
ppn   physical port (port-on-chip)
hdr   normalized header/preamble header structure


Return code:

 *******************************************************************************/
#if 0 
static void vtss_port10G_header_expect( BOOL enable, const m2_header_t hdr)
{
	ulong reg = 0, reg_mask = 0;

	if ( hdr.use_norm_hdr) {
		if (enable == TRUE) {
			SET_BIT( reg,1);
		}

		SET_BIT( reg_mask,1);
	}

	if ( hdr.use_prm_hdr) {
		if (hdr.use_prm_hdr == TRUE)
			SET_BIT( reg, 0);

		SET_BIT( reg_mask, 0);
	}

	if( reg_mask != 0) {
		vtss_io_writemasked( M2_BLK_MACS, M2_PHYS_PORT_10G, M2_DENORM, 
				reg, reg_mask);
	}

	return;
}
#endif

/******************************************************************************
 *                                                                            *
 *       SPI4.2 Setup                                                         *
 *                                                                            *
 ******************************************************************************/


/******************************************************************************
 *
 * Description: Set up the SPI4.2 block 
 *
 * \param:      ps       Pointer to a structure which determines configuration
 *
 * \return:     VTSS_OK  Operation completed successfully.
 *              VTSS_WRONG_MAJOR_MODE
 *              VTSS_MAJOR_MODE_NOT_SET
 *              VTSS_WRONG_PARAMETER
 *
 **************************************************************************kbp*/
vtss_rc vtss_spi4_setup( vtss_spi4_setup_t* ps)
{
	ulong reg  = 0;
	ulong mask = 0;
	ulong active_ports = 0;
	ulong pll_clk_speed = 0;
	int i;
	m2_header_t hdr;
	BOOL hdr_present;


	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	switch ( pvds->mmode) {
		/* valid major modes */
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:  /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK: /* */
			break;

			/* Error - major mode undefined */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			return VTSS_MAJOR_MODE_NOT_SET;

			/* Error - wrong major mode */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:  /* Single Chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK: /* Single Chip trunking */
		default:
			return VTSS_WRONG_MAJOR_MODE;
	}


	/* Burst size from ingress FIFO -> SPI4 */
	reg = ps->burst_size & 0xF;

	/* Selection between burst or frame interleaved scheduling
	   is done in the ingress FIFO: CM bit in ing_ctrl_reg */
	hdr.use_norm_hdr = TRUE; hdr.use_prm_hdr = FALSE;
	switch ( ps->sch_mode) {
#if !defined MEIGS2 && !defined VSC7321
		case VTSS_SPI4_BURST_MODE_WITH_HEADER:
			vtss_hdr_insert( TRUE, hdr);
			SET_BIT( reg, 4); /* set CM bit */
			break;
#endif
		case VTSS_SPI4_BURST_MODE:
			SET_BIT( reg, 4); /* set CM bit */
			break;
		case VTSS_SPI4_FRAME_MODE:
			CLR_BIT( reg, 4); /* clear CM bit */
			vtss_hdr_insert( TRUE, hdr);
			break;
		default:
			return VTSS_WRONG_PARAMETER;
	}
	/* Write to ingress fifo control register */
	vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_ING_CONTROL, reg, 0x1F);


	/* Set-up Header Stripper located before Ingress SPI4.2 */
	if( ps->norm_hdr_strip == FALSE) {
		vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_IFACE_MODE, 0, (ulong)1<<4);
	} else {
		vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_IFACE_MODE, (ulong)1<<4, (ulong)1<<4);
	}


	/* Set-up FCS/CRC-32 Checker located after Egress SPI4.2 */
	if ( ps->norm_hdr_expect != FALSE) {
		hdr_present = TRUE;
		if ( ps->prm_hdr_expect != FALSE) {
			reg = 0x0; /* hdr_size = 16 bytes */
		} else {
			reg = 0x9; /* hdr_size = 9 bytes */
		}
	} else {
		if ( ps->prm_hdr_expect != FALSE) {
			hdr_present = TRUE;
			reg = 0x7; /* hdr_size = 7 bytes */
		} else {
			hdr_present = FALSE; /* no header */
			reg = 0x0; /* hdr_size = 0 bytes */
		}
	}
	vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_CRC_CFG,
			((hdr_present)?((ulong)1<<4):0) | reg, 0x1F);


	/******************    SPI4  MISC register  setup    ************************/

	/* SPI4 output frequency 390(default), 312, 195, 156 MHz */
	reg = 0x1; mask = 0x11;
	switch ( ps->spi4_output_freq) {
		case VTSS_SPI4_OUTPUT_FREQ_390MHZ:
			SET_BIT( pll_clk_speed, 0);
			break;
#if defined MEIGS2 || defined VSC7321
		case VTSS_SPI4_OUTPUT_FREQ_312MHZ:
			break;
		case VTSS_SPI4_OUTPUT_FREQ_156MHZ:
			SET_BIT( reg, 4);
			break;
#endif
		case VTSS_SPI4_OUTPUT_FREQ_195MHZ:
			SET_BIT( pll_clk_speed, 0);
			SET_BIT( reg, 4);
			break;
		default:
			return VTSS_WRONG_PARAMETER;
	}
	/* Select SPI4 clock source from either SerDes or XAUI PLL */
	vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_PLL_CLK_SPEED, 
			pll_clk_speed, 0x1);


	/* 1. Power-up CML logic
	   2. Release reset on CML logic and set-up RQC

	   3. Set-up rest
	   Always assert bit 18, SD; spi4_swap_bytes in order to be spi4 compliant
	   Always de-assert bits 29-31: SPI clock reset bits */

	/* Power-up CML logic */
	vtss_io_writemasked( M2_BLK_SPI4, M2_SUBBLK_NONE, M2_SPI4_MISC, reg, mask);

	/* Set-up RQC. Board dependent. */
	if ( ps->spi4_data_clock_skew != FALSE)  { SET_BIT( reg, 2); }
	SET_BIT( mask, 2);

	/* Release reset on CML logic */
	SET_BIT( reg, 3); SET_BIT( mask, 3);

	/* Release reset on CML logic and set-up RQC */
	vtss_io_writemasked( M2_BLK_SPI4, M2_SUBBLK_NONE, M2_SPI4_MISC, reg, mask);

	/* Bit SD. Swap bytes in order to be spi4 compliant */
	SET_BIT( reg, 18); SET_BIT( mask, 18);

	/* Enable all clocks */
	SET_BITS_MASKED( mask, (ulong)7<<29, (ulong)7<<29);

	/* Bit WI. Board dependent. */
	if ( ps->spi4_swap_ingress_data != FALSE) { SET_BIT( reg, 9); }
	SET_BIT( mask, 9);

	/* Bit WE. Board dependent. */
	if ( ps->spi4_swap_egress_data != FALSE) { SET_BIT( reg, 8); }
	SET_BIT( mask, 8);

	vtss_io_writemasked( M2_BLK_SPI4, M2_SUBBLK_NONE, M2_SPI4_MISC, reg, mask);


	/******************    SPI4_ING_SETUP0 register setup   *********************/

	/* Calendar length is defined by number of ports. Calculate active ports */
	for(i=0; i<VTSS_PORT_ARRAY_SIZE; i++) {
		ulong ppn = vtss_logical_ports[i].chip_port;
		if ( ppn != -1) {
			SET_BIT(active_ports, ppn);
		}
	}
	/* Setup active ports, kbp_test: active_ports = 0x3FF;*/
	reg = active_ports & 0x3FF; mask = 0x3FF;

	/* SPI4_calendar_order */
	if ( ps->spi4_calendar_order_ascending == FALSE) { SET_BIT( reg, 10); }
	SET_BIT( mask, 10);

	/* SPI4_calendar_m */
	SET_BITS_MASKED( reg, (ulong)(ps->spi4_calendar_m)<<12, 0xF<<12);
	SET_BITS_MASKED( mask, 0xF<<12, 0xF<<12);

	/* write to spi4_ingress_setup0_reg register */
	vtss_io_writemasked( M2_BLK_SPI4, M2_SUBBLK_NONE, M2_SPI4_ING_SETUP0, reg, mask);


	/******************    SPI4_ING_SETUP1 register setup   *********************/

	/* kbp: Where do we set-up this ?
#if defined MEIGS2 || defined VSC7321
ps->tm = VTSS_SPI4_TRAINING_OFF;
#else
ps->tm = VTSS_SPI4_TRAINING_AUTO;
#endif
	 */

	/* Parameters of the training mode (only in ingress direction) */
	vtss_io_write( M2_BLK_SPI4, M2_SUBBLK_NONE, M2_SPI4_ING_SETUP1,
			(ps->alpha<<16) | (ps->tsperiod));


	/******************    SPI4_ING_SETUP2 register setup   *********************/

	vtss_io_writemasked( M2_BLK_SPI4, M2_SUBBLK_NONE, M2_SPI4_ING_SETUP2,
			(ps->maxburst2<<24) | (ps->maxburst1<<16) | ps->burst_size, 
			0xFFFF000F);


	/******************    SPI4_EGR_SETUP0 register setup   *********************/

	/* Setup active ports */
	reg = active_ports & 0x3FF; mask = 0x3FF;

	/* Shift status clock output. Board dependent. */
	if ( ps->spi4_status_clock_skew != FALSE) { SET_BIT( reg, 18); }
	SET_BIT( mask, 18);

	/* SPI4_calendar_order */
	if ( ps->spi4_calendar_order_ascending == FALSE) { SET_BIT( reg, 10); }
	SET_BIT( mask, 10);

	/* SPI4_calendar_m */
	SET_BITS_MASKED( reg, (ulong)(ps->spi4_calendar_m)<<12, 0xF<<12);
	SET_BITS_MASKED( mask, 0xF<<12, 0xF<<12);

	/* write to spi4_egress_setup0_reg register. */
	vtss_io_writemasked( M2_BLK_SPI4, M2_SUBBLK_NONE, M2_SPI4_EGR_SETUP0, reg, mask);


	return VTSS_OK;
}


/******************************************************************************
 *
 * Description: Configures a user provided structure with default values for
 *              the selected major mode.
 *
 * \param:      ps       Pointer to a structure which determines configuration
 *              mode     Major mode
 *
 * \return:     VTSS_OK  Operation completed successfully.
 *              VTSS_MAJOR_MODE_NOT_SET
 *
 **************************************************************************kbp*/
vtss_rc vtss_spi4_setup_get_default_values( vtss_spi4_setup_t* ps, 
		vtss_mac_major_mode_t mode)
{

	/* By default in the ingress direction SPI4.2 runs in burst interleaved mode*/
	ps->sch_mode   = VTSS_SPI4_BURST_MODE;
	ps->burst_size = VTSS_SPI4_BURST_SIZE;
	ps->maxburst1  = VTSS_SPI4_MAX_BURST_1;
	ps->maxburst2  = VTSS_SPI4_MAX_BURST_2;


	/* Parameters of the training mode (only in ingress direction) */
#if defined MEIGS2 || defined VSC7321
	ps->tm = VTSS_SPI4_TRAINING_OFF;
#else
	ps->tm = VTSS_SPI4_TRAINING_AUTO;
#endif
	ps->alpha    = 1;
	ps->tsperiod = 0;

	/* spi4 output frequency: 390(default), 312, 195, 156 MHz */
	ps->spi4_output_freq = VTSS_SPI4_OUTPUT_FREQ_390MHZ;

	/* For the description of the following four elements refer to the desription 
	   of the SPI4 misc register in the datasheet */
	ps->spi4_swap_ingress_data = FALSE;  /* Board dependent */ /* Bit WI, SPI4 MISC reg*/
	ps->spi4_swap_egress_data  = FALSE;  /* Board dependent */ /* Bit WE, SPI4 MISC reg*/
	ps->spi4_data_clock_skew   = FALSE;  /* Board dependent */
	ps->spi4_status_clock_skew = FALSE;  /* Board dependent */

	/* calendar length is defined by number of ports */
	ps->spi4_calendar_order_ascending = TRUE;
	ps->spi4_calendar_m = 1;

	/* normalized and preamble headers */
	/* ingress direction: strip the norm header before sending it over SPI4 */
	/* if normalization is not used (i.e there is no header to strip) this 
	   parameter must be set to FALSE */
	ps->norm_hdr_strip = FALSE;


	/* egress direction: expect/do not expect headers  */
	switch (mode) {
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:  /* SPI4 <-> 10x1G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G: /* SPI4 <-> 1x10G */
			ps->prm_hdr_expect = FALSE;
			ps->norm_hdr_expect = FALSE;
			break;
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
			ps->prm_hdr_expect = FALSE;
			ps->norm_hdr_expect = TRUE;
			break;
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK: /* */
			ps->prm_hdr_expect = TRUE;
			ps->norm_hdr_expect = TRUE;
			break;
		default:
			/* VTSS_MAC_MAJOR_MODE_10G_1G_AGGR  - Single Chip aggr */
			/* VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK - Single Chip trunking */
			return VTSS_WRONG_MAJOR_MODE;
	}

	return VTSS_OK;
}


/*******************************************************************************

  Function  vtss_spi4_fcs_check_enable()
  ================================================================================

Description:
The function turns on/off FCS check of Ethernet frames arriving from the 
host interface. This is done for all channels simultaneously.


NOTE. FCS check in the SPI4 block must be OFF, if FCS will be modified
(added/updated) in Ethernet port(s) in the egress direction

Syntax
vtss_rc vtss_spi4_fcs_check_enable( BOOL check_fcs);

Arguments:
check_fcs    TRUE  -- turns FCS check ON
FALSE -- turns FCS check OFF

Return code:
VTSS_OK   if operation completed successfully

 *******************************************************************************/
vtss_rc vtss_spi4_fcs_check_enable( BOOL check_fcs)
{
	ulong fcs = 0;

	if (check_fcs) {
		/* CLR_BIT(fcs,5); Already cleared */
	} else {
		/*ignore fcs check*/
		SET_BIT(fcs,5);
	}

	vtss_io_writemasked(M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_CRC_CFG,
			fcs, 1<<5);

	return VTSS_OK;
}




/*******************************************************************************

  Function  vtss_spi4_keep_norm_header()
  ================================================================================

Description:
The function controls whether to strip the normalised header or to keep it.
Usually the normalised header is stripped before the frame is sent over 
SPI4. However, if the remove receiver can use information in the header, 
keeping it may be an advantage.


NOTE. Works in ingress direction

Syntax
vtss_rc vtss_spi4_keep_norm_header( BOOL keep);

Arguments:
keep     TRUE  -- the noramlised header is not stripped
FALSE -- the noramlised header is stripped (Default)

Return code:
VTSS_OK   if operation completed successfully

 *******************************************************************************/
vtss_rc vtss_spi4_keep_norm_header( BOOL keep)
{
	ulong reg_value = 0, reg_mask = 0;


	if ( keep == TRUE)
		SET_BIT( reg_value, 4);

	SET_BIT( reg_mask, 4);

	vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_IFACE_MODE, 
			reg_value, reg_mask);

	return VTSS_OK;
}


/******************************************************************************/
/***        Flow Control Configuration        *********************************/
/******************************************************************************/

/* 

   transparent off -- filling of the FIFO causes generation of pause frames
   transparent on  -- destination (host i/f), not FIFO, requests pause frames

Note:

 */
/*******************************************************************************

  Function 
  ================================================================================

Description:
Function vtss_device_transparent_flow_control_mode() selects FIFO flow 
control mode 

NOTE: 
This mode is enabled for all 10/100/1000 Eth ports simultaneously.
This functions does not set RX_PAUSE or TX_PAUSE bits in 1G port blocks,
the user need to do it after calling this function.


Syntax
vtss_rc vtss_device_transparent_flow_control_mode( BOOL ingress_enable,
BOOL egress_enable);


Arguments:
ingress_enable      when set to TRUE, SPI4.2 flow control signals will 
cause generation of Ethernet flow control frames
egress_enable       when set to TRUE, Ethernet flow control frames will
translate to SPI4.2 flow control signals

Return code:
VTSS_OK   if operation completed successfully

 *******************************************************************************/
vtss_rc vtss_device_transparent_flow_control_mode( BOOL ingress_enable,
		BOOL egress_enable)
{
	ulong gress_ctrl_reg = 0;
	ulong gress = M2_SUBBLK_INGRESS;


	if (ingress_enable) {
		SET_BIT( gress_ctrl_reg, 11); /* IPT bit */
		SET_BIT( gress_ctrl_reg, 12); /* IGI bit */
	} 
	vtss_io_writemasked(M2_BLK_FIFO, gress, M2_ING_CONTROL,
			gress_ctrl_reg, (1<<11)|(1<<12));


	gress_ctrl_reg = 0;
	gress = M2_SUBBLK_EGRESS;

	if (egress_enable) {
		/* In egress direction use the MFE bit, not IPT/IGI */
		SET_BIT( gress_ctrl_reg, 15);
	}
	vtss_io_writemasked(M2_BLK_FIFO, gress, M2_EGR_CONTROL,
			gress_ctrl_reg, 1<<15);

	/* Remember to set RX_PAUSE_EN, TX_PAUSE_EN or PAUSE_EN bits in the PAUSE_CFG 
	   register of 1G ports */


	return VTSS_OK;
}




/******************************************************************************/
/***        FIFO Configuration                       **************************/
/******************************************************************************/


/*-----------------  FIFO setup funcitons   ----------------------------------*/


/*******************************************************************************

  Function vtss_fifo_setup()
  ================================================================================

Description:
Initializes the FIFO block based on the information in the provided 
structures. If the pointer to the structure is set to NULL, the corresponding
FIFO will not be initialized (it will be left in its current state).

The function must be called after start-up and may be called at run-
time to reconfigure the FIFO. 

NOTE: port mapping table must be initialized.


Syntax
vtss_rc vtss_fifo_setup( vtss_fifo_setup_t *ps_ingress, 
vtss_fifo_setup_t *ps_egress)

Arguments:
ps_ingress   pointer to a structure for the ingress fifo
ps_egress   pointer to a structure for the egress fifo

Return code:
VTSS_OK   if operation completed successfully
VTSS_MAJOR_MODE_NOT_SET

 *******************************************************************************/

vtss_rc vtss_fifo_setup( vtss_fifo_setup_t *ps_ingress, 
		vtss_fifo_setup_t *ps_egress)
{
	int i, ppn;
	ulong reg, mask;

	/* Only one global device per API is currently supported */
	vtss_device_setup_t *pvds = pdevice;
	if ( pvds->mmode == VTSS_MAC_MAJOR_MODE_UNDEFINED) { return VTSS_MAJOR_MODE_NOT_SET; }

	for ( i=1; i<=VTSS_PORTS_1G; i++) {

		ppn = vtss_logical_ports[i].chip_port;
		//    VTSS_N(("FIFO setup: log port#%d, phys port #%d", i, ppn));
		//printk("FIFO setup: log port#%d, phys port #%d\n", i, ppn);
		if( ppn < 0 ) { continue; }

		/* Clear pointer mode when changing top/bottom */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
				M2_TEST + ppn, M2_TEST_FIFO_CLR, 0xF);
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
				M2_TEST + ppn, M2_TEST_FIFO_CLR, 0xF);

		/* TOP and BOTTOM values */
		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_TOP_BOTTOM + ppn, 
				(ulong)((ps_ingress->fifo_port_area[i].top<<16) | 
					ps_ingress->fifo_port_area[i].bottom));

		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TOP_BOTTOM + ppn, 
				(ulong)((ps_egress->fifo_port_area[i].top<<16) | 
					ps_egress->fifo_port_area[i].bottom));

		/* Watermarks */
		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_HIGH_LOW_WM + ppn, 
			(ulong)((ps_ingress->fifo_port_wm[i].low_watermark<<16) | 
					ps_ingress->fifo_port_wm[i].high_watermark));

		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_HIGH_LOW_WM + ppn, 
			(ulong)((ps_egress->fifo_port_wm[i].low_watermark<<16) | 
				ps_egress->fifo_port_wm[i].high_watermark));

		/* Threshold, if cut-through mode enabled */
		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_CT_THRHLD + ppn, 
				(ulong)((ps_ingress->thrhld[i].cut_through_enable)?
					ps_ingress->thrhld[i].threshold_value:0));

		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_CT_THRHLD + ppn, 
				(ulong)((ps_egress->thrhld[i].cut_through_enable)?
					ps_egress->thrhld[i].threshold_value:0));

		/* Enable FIFO again */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
				M2_TEST + ppn, M2_TEST_FIFO_NORMAL, 0xF);
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
				M2_TEST + ppn, M2_TEST_FIFO_NORMAL, 0xF);
	}

	/* Ageing */
	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_AGE_INC,
		 (ps_ingress->ageing.enable)? ps_ingress->ageing.interval:0);
	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_AGE_INC, 
			(ps_egress->ageing.enable)? ps_egress->ageing.interval:0);

	/* --- Ingress FIFO control register setup (major mode dependent) --- */
	reg = 0;
	if( ps_ingress->prm_hdr_used != FALSE) { SET_BIT( reg, 9); }

	if( ps_ingress->norm_hdr_used != FALSE) {
		SET_BIT( reg, 8);
		/* Ingress FIFO gets data only from MACs thus bit 
		   LE must always be set when the normalized header is used */
		SET_BIT( reg, 13);
	}

	/* OUT_PORT_OFFSET */
	reg  |= (ps_ingress->output_port_offset<<28) & 0xF0000000;

	/* IN_PORT_OFFSET */
	reg  |= (ps_ingress->input_port_offset<<24) & 0x0F000000;

	/* Single Channel, M10G mode */
	if (pvds->mmode == VTSS_MAC_MAJOR_MODE_SPI4_10G){ SET_BIT( reg, 5); }

	/* Ingress FIFO by default is always set to run in burst-interleaved mode */
	SET_BIT( reg, 4);

	/* MUX bits, SS bit and burst size */
	if ( (pvds->mmode == VTSS_MAC_MAJOR_MODE_10G_1G_AGGR) ||
			(pvds->mmode == VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK)) {
		/* if Single Chip aggr or single chip trunking */
		SET_BIT( reg, 16);
		SET_BIT( reg, 7);
		/* burst size towards 1G ports */
		SET_BITS_MASKED( reg, 
			VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_1G_PORT, 0xF);
	} else {
		/* burst size towards SPI4.2 */
		SET_BITS_MASKED( reg, 
			VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_SPI4_2, 0xF); 
	}

	/* Enable the FIFO */
	CLR_BIT( reg, 6);

	mask = 0xFF0323FF;
	vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
		M2_ING_CONTROL, reg, mask);


	/* --- Egress FIFO control register setup (major mode dependent) --- */
	reg = 0;
	if( ps_egress->prm_hdr_used != FALSE) { SET_BIT( reg, 9); }

	if( ps_egress->norm_hdr_used != FALSE) {
		SET_BIT( reg, 8);
		/* Single Chip aggr o trunking */
		if( ( pvds->mmode == VTSS_MAC_MAJOR_MODE_10G_1G_AGGR) || 
			( pvds->mmode == VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK)) {
			/* Receiving data from 1G MACs in SMES mode */
			SET_BIT( reg, 13);
		} else {
			/* Receiving data from SPI4.2 always SME mode */
			CLR_BIT( reg, 13);
		}
	}

	/* OUT_PORT_OFFSET */
	reg  |= (ps_egress->output_port_offset<<28) & 0xF0000000;

	/* IN_PORT_OFFSET */
	reg  |= (ps_egress->input_port_offset<<24) & 0x0F000000;

	switch (pvds->mmode) {

		case VTSS_MAC_MAJOR_MODE_SPI4_10G:       /* SPI4 <-> 1x10G */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK: /* */
			CLR_BIT( reg, 17);
			CLR_BIT( reg, 16); /* MUX bits */
			CLR_BIT( reg, 7); /* SS bit */
			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_1G:        /* SPI4 <-> 10x1G */
			CLR_BIT( reg, 17);
			SET_BIT( reg, 16); /* MUX bits */
			SET_BIT( reg, 7); /* SS bit */
			break;

		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:    /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:   /* single chip trunking */
			SET_BIT( reg, 17);
			CLR_BIT( reg, 16); /* MUX bits */
			CLR_BIT( reg, 7); /* SS bit */
			break;

		default:
			break;
	}

	/* Activate the FIFO */
	CLR_BIT( reg, 6);

	/* Frame-, burst- interleaving and burst size */
	switch (pvds->mmode) {

		case VTSS_MAC_MAJOR_MODE_SPI4_10G:  /* Single Channel, M10G mode */
			/* Egress FIFO sends data to 10G MAC from onei
			 channel burst-interleaved is fine */
			SET_BIT( reg, 4);
			SET_BIT( reg, 5);
			SET_BITS_MASKED( reg, 
			VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_10G_PORT, 0xF);
			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:   /* */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */
		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:     /* Single Chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:    /* Single Chip trunking */
			/* Egress FIFO sends data to 10G MAC 
			from upto 10 channels always frame-interleaved */
			CLR_BIT( reg, 4);
			/* It's recommended to write a value into the 
			   burst size bitfield even  when running in frame-interleaved
			     mode */
			SET_BITS_MASKED( reg, 
			VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_10G_PORT, 0xF);
			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_1G: /* SPI4 <-> 10x1G */
			/* Egress FIFO sends data to 1G MACs from upto
			 10 channels always burst-interleaved */
			SET_BIT( reg, 4);
			SET_BITS_MASKED( reg,
			VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_1G_PORT, 0xF);
			break;

		default:
			break;
	}

	mask = 0xFF0323FF;
	vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
				M2_EGR_CONTROL, reg, mask);

	return VTSS_OK;
}


/******************************************************************************
 * Description:
 *     This is a supplementary function. It configures a structure referenced 
 *     by the pointer to the default values for the chosen major mode.
 *     After that the user may modify the fields of the structure for 
 *     fine-tuning of the system and then call the vtss_fifo_setup() function.
 *
 * Arguments:
 *     ps_ingress   pointer to a structure for the ingress fifo
 *     ps_egress    pointer to a structure for the egress fifo
 *     major_mode   select the major mode  for the session
 *
 * Return code:
 *     VTSS_OK      if operation completed successfully
 *     VTSS_WRONG_MAJOR_MODE
 *     VTSS_PORT_NOT_MAPPED
 *
 **************************************************************************kbp*/
vtss_rc vtss_fifo_setup_get_default_values( vtss_fifo_setup_t *ps_ingress,
		vtss_fifo_setup_t *ps_egress,
		vtss_mac_major_mode_t major_mode)
{
	uint  i;
	uint  ports = 0;
	uint  ingress_tb = 0;
	uint  egress_tb  = 0;
	ulong fifo_ingress_block_size = 0x0; 
	ulong fifo_egress_block_size  = 0x0; 

	switch (major_mode) {
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:   /* SPI4 <-> 1x10G */
			ps_ingress->prm_hdr_used = FALSE;
			ps_ingress->norm_hdr_used = FALSE;
			ps_egress->prm_hdr_used = FALSE;
			ps_egress->norm_hdr_used = FALSE;

			ps_ingress->output_port_offset = 0;
			ps_ingress->input_port_offset = 0;
			ps_egress->output_port_offset = 0;
			ps_egress->input_port_offset = 0;
			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_1G:    /* SPI4 <-> 10x1G */
			ps_ingress->prm_hdr_used = FALSE;
			ps_ingress->norm_hdr_used = FALSE;
			ps_egress->prm_hdr_used = FALSE;
			ps_egress->norm_hdr_used = FALSE;

			ps_ingress->output_port_offset = 0;
			ps_ingress->input_port_offset = 0xA;
			ps_egress->output_port_offset = 0xA;
			ps_egress->input_port_offset = 0;
			break;

		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:
			ps_ingress->prm_hdr_used = FALSE;
			ps_ingress->norm_hdr_used = FALSE;
			ps_egress->prm_hdr_used = FALSE;
			ps_egress->norm_hdr_used = TRUE;

			ps_ingress->output_port_offset = 0xA;
			ps_ingress->input_port_offset = 0;
			ps_egress->output_port_offset = 0;
			ps_egress->input_port_offset = 0xA;
			break;

		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:
			ps_ingress->prm_hdr_used = TRUE;
			ps_ingress->norm_hdr_used = FALSE;
			ps_egress->prm_hdr_used = TRUE;
			ps_egress->norm_hdr_used = TRUE;

			ps_ingress->output_port_offset = 0xA;
			ps_ingress->input_port_offset = 0;
			ps_egress->output_port_offset = 0;
			ps_egress->input_port_offset = 0xA;
			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:
			ps_ingress->prm_hdr_used = FALSE;
			ps_ingress->norm_hdr_used = TRUE;
			ps_egress->prm_hdr_used = FALSE;
			ps_egress->norm_hdr_used = TRUE;

			ps_ingress->output_port_offset = 0;
			ps_ingress->input_port_offset = 0;
			ps_egress->output_port_offset = 0;
			ps_egress->input_port_offset = 0;
			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:
			ps_ingress->prm_hdr_used = TRUE;
			ps_ingress->norm_hdr_used = TRUE;
			ps_egress->prm_hdr_used = TRUE;
			ps_egress->norm_hdr_used = TRUE;

			ps_ingress->output_port_offset = 0;
			ps_ingress->input_port_offset = 0;
			ps_egress->output_port_offset = 0;
			ps_egress->input_port_offset = 0;
			break;

		default:
			/* VTSS_MAC_MAJOR_MODE_UNDEFINED */
			return VTSS_WRONG_MAJOR_MODE;
	}

	fifo_ingress_block_size = VTSS_MAX_INGRESS_FIFO_SIZE >> 11;
	fifo_egress_block_size  = VTSS_MAX_EGRESS_FIFO_SIZE  >> 11;

	/* Single Channel mode, 10G is logical port number 1 */
	if (major_mode == VTSS_MAC_MAJOR_MODE_SPI4_10G) {   /* SPI4 <-> 1x10G */

		/* vtss_fifo_block_t  */
		ps_ingress->fifo_port_area[1].bottom = 0;
		ps_ingress->fifo_port_area[1].top = fifo_ingress_block_size;
		ps_egress->fifo_port_area[1].bottom = 0;
		ps_egress->fifo_port_area[1].top = fifo_egress_block_size;

		/* vtss_fifo_fc_watermarks_t */
		ps_ingress->fifo_port_wm[1].low_watermark = 0xFFFFFFFF;
		ps_ingress->fifo_port_wm[1].high_watermark = 0xFFFFFFFF;
		ps_egress->fifo_port_wm[1].low_watermark = 0xFFFFFFFF;
		ps_egress->fifo_port_wm[1].high_watermark = 0xFFFFFFFF;

		/* cut-through mode/threshold. If not enabled then store-forward */
		/* vtss_fifo_cut_through_mode_t */
		ps_ingress->thrhld[1].cut_through_enable = FALSE;
		ps_ingress->thrhld[1].threshold_value = 0;
		ps_egress->thrhld[1].cut_through_enable = FALSE;
		ps_egress->thrhld[1].threshold_value = 0;

	} else {

		/* Multi Channel mode */
		for( i=1; i<=VTSS_PORTS_1G; i++ ) {
			if( vtss_logical_ports[i].chip_port >= 0 ) { ports++; }
		}

		if ( ports == 0 ) { return VTSS_PORT_NOT_MAPPED; } 
		fifo_ingress_block_size = fifo_ingress_block_size / ports;
		fifo_egress_block_size  = fifo_egress_block_size  / ports;


		for(i=1; i<=VTSS_PORTS_1G; i++) {
			if( vtss_logical_ports[i].chip_port < 0 ) { continue; }

			/* TOP and BOTTOM values */
			ps_ingress->fifo_port_area[i].bottom = ingress_tb;
			ingress_tb = ingress_tb + fifo_ingress_block_size;
			ps_ingress->fifo_port_area[i].top = ingress_tb;
			ps_egress->fifo_port_area[i].bottom = egress_tb;
			egress_tb = egress_tb + fifo_egress_block_size;
			ps_egress->fifo_port_area[i].top = egress_tb;
			/*VTSS_D(("tb: 0x%08lX 0x%08lX", ingress_tb, egress_tb));*/

			/* Watermarks */
			ps_ingress->fifo_port_wm[i].low_watermark = 0xFFFFFFFF;
			ps_ingress->fifo_port_wm[i].high_watermark = 0xFFFFFFFF;
			ps_egress->fifo_port_wm[i].low_watermark = 0x40; 
			ps_egress->fifo_port_wm[i].high_watermark = 256; 

			/* cut-through mode/threshold. If not enabled then store-forward */
			ps_ingress->thrhld[i].cut_through_enable = FALSE;
			ps_ingress->thrhld[i].threshold_value = 0;
			ps_egress->thrhld[i].cut_through_enable = FALSE;
			ps_egress->thrhld[i].threshold_value = 0;
		}

	}

	/* vtss_fifo_ageing_t */
	ps_ingress->ageing.enable = FALSE;
	ps_ingress->ageing.interval = 0;
	ps_egress->ageing.enable = FALSE;
	ps_egress->ageing.interval = 0;

	return VTSS_OK;
}

/*-----------------  Run-time funcitons   ------------------------------------*/



/*******************************************************************************
  Function 
  Syntax 
  Parameters 

  Returns 
  VTSS_OK        No errors detected

  Description

 ********************************************************************************/
vtss_rc vtss_fifo_thrhld_set( vtss_port_no_t portnum, 
		vtss_fifo_cut_through_mode_t *pctt_ingress,
		vtss_fifo_cut_through_mode_t *pctt_egress)
{
	/* Find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;

	/* Check the physical port number */
	if (ppn == -1) { return VTSS_PORT_NOT_MAPPED; }

	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
			M2_CT_THRHLD + ppn, 
			(pctt_ingress->cut_through_enable)?
			(pctt_ingress->threshold_value):0);

	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
			M2_CT_THRHLD + ppn, 
			(pctt_egress->cut_through_enable)?
				(pctt_egress->threshold_value):0);

	return VTSS_OK;
}


/*******************************************************************************
  Function 
  vtss_fifo_watermarks_set()

  Syntax 
  vtss_fifo_watermarks_set( vtss_port_no_t portnum,
  vtss_fifo_fc_watermarks_t *pwm_ingress,
  vtss_fifo_fc_watermarks_t *pwm_egress)
  Parameters 
  portnum        Logical port number
  pwm_ingress    Pointer to the 'watermarks' structure in ingress direction. 
  If set to NULL no initialization is performed.
  pwm_egress     Pointer to the 'watermarks' structure in egress directon.
  If set to NULL no initalization is performed

  Returns 
  VTSS_OK        No errors detected

  Description
  This function sets watermarks for a given logical port

 ********************************************************************************/
vtss_rc vtss_fifo_watermarks_set( vtss_port_no_t portnum,
		vtss_fifo_fc_watermarks_t *pwm_ingress,
		vtss_fifo_fc_watermarks_t *pwm_egress)
{
	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_HIGH_LOW_WM + ppn, 
			(ulong)((pwm_ingress->low_watermark<<16) |
			 pwm_ingress->high_watermark));

	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_HIGH_LOW_WM + ppn, 
			(ulong)((pwm_egress->low_watermark<<16) |  
			pwm_egress->high_watermark));

	return VTSS_OK;
}


/*

   Set a fifo to expect incoming data with/without the header (normalised or 
   preamble)

   The function does not check how the fifo is configured. (Note: running in 
   cut-through mode with the header makes no sense)


   Parameters
   fifo      which fifo to use: engress or ingress
   enable    enable or disable the feature
   if TRUE,  incoming frame have normalized and/or preamble header
   if FALSE, incoming frame co not have normalized and/or preamble 
   header
   hdr       Describes what header to expect

Note: this function will set or clear only that bit in the register which 
corresponding element in the 'hdr' structure is set, i.e. it will not 
affect preamble header settings, if the use_prm_hdr element is not set.

 */
static void vtss_fifo_use_hdr( int fifo, BOOL enable, m2_header_t hdr)
{
	ulong value = 0;
	ulong mask = 0;
	ulong reg_offset = 0;

	switch( fifo) {
		case M2_SUBBLK_EGRESS:
			reg_offset = M2_EGR_CONTROL;
			break;
		case M2_SUBBLK_INGRESS:
			reg_offset = M2_ING_CONTROL;
			break;
		default:
			return;
	}

	if ( hdr.use_norm_hdr != FALSE) {
		if ( enable != FALSE)
			SET_BIT( value, 8);
		SET_BIT( mask, 8);
	}

	if ( hdr.use_prm_hdr != FALSE){
		if (enable != FALSE)
			SET_BIT( value, 9);
		SET_BIT( mask, 9);
	}



	vtss_io_writemasked( M2_BLK_FIFO, fifo, reg_offset, value, mask);

	return;
}


#define VTSS_MIIM_READ_OPR      0x2
#define VTSS_MIIM_WRITE_OPR     0x1

#define VTSS_MIIM_OPR_MODE      0x1
#define VTSS_MDIO_OPR_MODE      0x0


/* Direct miim read/write operation using miim_controller address as a 
   parameter */
long vtss_miim_subblock_read( const uint miim_chnl, const uint phy_addr, 
		const uint phy_reg)
{
	uint opcode = VTSS_MIIM_READ_OPR; /* read operation */
	uint mode = VTSS_MIIM_OPR_MODE; /* MIIM operation, not MDIO */
	ulong i = 0;
	ulong data;


	/* read status of the channel */
	while ( 0 != (0x1C & vtss_io_read( M2_BLK_MIIM, 
				miim_chnl, M2_MIIM_STATUS))) {
		i++;
		if(i > VTSS_MIIM_READ_ATTEMPT) {
//			printk("API: channel busy1\n");
			return VTSS_MIIM_CHANNEL_BUSY;
		}
	}


	vtss_io_write( M2_BLK_MIIM, miim_chnl, M2_MIIM_CMD, 
			(phy_addr<<9)|(phy_reg<<4)|(opcode <<2)|mode);


	/* read status of the channel */
	while ( 0 != (0x1C & vtss_io_read( M2_BLK_MIIM, 
			miim_chnl, M2_MIIM_STATUS))) {
		VTSS_NSLEEP( 2000000); /* sleep 2 uS */
		i++;
		if(i > VTSS_MIIM_READ_ATTEMPT) {
			printk("API: channel busy2\n");
			return VTSS_MIIM_CHANNEL_BUSY;
		}
	}

	data = vtss_io_read( M2_BLK_MIIM, miim_chnl, M2_MIIM_DATA);

	return (data & ((ulong)1 << 16))?(-1):(data&0xFFFF);

}


void vtss_miim_subblock_write( const uint miim_chnl, const uint phy_addr, 
		const uint phy_reg, const ushort value)
{
	uint opcode = VTSS_MIIM_WRITE_OPR; /* write operation */
	uint mode = VTSS_MIIM_OPR_MODE; /* MIIM operation, not MDIO */


	/* read status of the channel */
	while( 0 != (0x1C & vtss_io_read( M2_BLK_MIIM, miim_chnl, M2_MIIM_STATUS))) 
		; /* wait until the channel is ready */


	vtss_io_write( M2_BLK_MIIM, miim_chnl, M2_MIIM_CMD, 
	(ulong)((value<<16)|(phy_addr<<9)|(phy_reg<<4)|(opcode <<2)|mode));

	return;
}


/* Indirect miim read/write operation. Accesses the PHY corresponding to the 
   logical port number (PHY address is found via the mapping table)
 */
long vtss_miim_port_reg_read( const vtss_port_no_t portnum, 
		const uint phy_reg){

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	int miimchnl =  vtss_logical_ports[portnum].miim_controller;
	int phy_addr =  vtss_logical_ports[portnum].phy_addr;
	long data;

	/* check the physical port number */
	if (ppn == -1) {
		printk("API: port not mapped\n");
		return VTSS_PORT_NOT_MAPPED;
	}

	/* long */ 
	data = vtss_miim_subblock_read( miimchnl, phy_addr,  phy_reg);
	return data;
}


void  vtss_miim_port_reg_write( const vtss_port_no_t portnum, 
		const uint phy_reg, const ushort value)
{
	/* find the physical port number in the global map table */
	/* int ppn = vtss_logical_ports[portnum].chip_port; */
	int miimchnl =  vtss_logical_ports[portnum].miim_controller;
	int phy_addr =  vtss_logical_ports[portnum].phy_addr;




	/* long */ 
	vtss_miim_subblock_write( miimchnl, phy_addr,  phy_reg, value);
}


BOOL vtss_phy_mapped( const vtss_port_no_t port_no )
{
	return  -1 != vtss_logical_ports[port_no].phy_addr;

}

/*******************************************************************************
  Policing and shaping traffic.

Terminology:
Policer limits traffic in ingress direction
Shaper does the same in engress direction

In functions below only the term 'shaper' is used.
 ********************************************************************************/
int get_shaper_register( int port_on_chip)
{
	if ( port_on_chip >= 0 && port_on_chip <= 7)
		return 0xA | (port_on_chip << 4);
	else if ( port_on_chip >= 8 && port_on_chip <= 9)
		return 0xB | ((port_on_chip & 0x7) << 4);
	else 
		return -1;

}

#if 0
/*******************************************************************************
  Function 
  vtss_egress_shaper_set()

  Description
  This function configures bitrate and leaky buvket level in the egress 
  direction for a given logical port.

  Syntax 
  vtss_rc vtss_egress_shaper_set( vtss_port_no_t portnum, vtss_bitrate_t br);

  Parameters 
  portnum        Logical port number
  br             Bitrate (in kilobits/second)
  lblvl          Leaky bucket level in bytes. Range (0...0xFFFF)*128 bytes

  Note
  The bitrate is in kbits/second, so to get a throughput in bits/second 
  the value must be multiplied by 1000.

  Returns 
  VTSS_OK        No errors detected


 ********************************************************************************/
vtss_rc vtss_egress_shaper_set( vtss_port_no_t portnum, vtss_kbitrate_t br, 
		ulong lblvl)
{
	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	//  VTSS_N(("Shaper setup (egress dir). Logic port #%d, phys prt #%d", portnum, ppn));
	//	printk("Shaper setup (egress dir). Logic port #%d, phys prt #%d\n", portnum, ppn);
	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	if( br == VTSS_DISABLE_SHAPER) {
		/* disable the shaper */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TRAFFIC_SHAPER_CTRL,
				(ulong)1<<22, (ulong)1<<22);

	} else {

		ulonglong reg_value = (ulonglong)br*1000/VTSS_SHAPER_BITRATE_UNIT_PER_PORT;
		ulong lvl = lblvl/VTSS_SHAPER_LEAKY_BUCKET_UNIT;

		if (reg_value > 0xFFFF)
			reg_value = 0xFFFF;

		if ( lvl > 0xFFFF)
			lvl = 0xFFFF;

		reg_value |= (lvl << 16);

		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, get_shaper_register( ppn),
				(ulong)reg_value);

		/* Enable the shaper */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TRAFFIC_SHAPER_CTRL,
				0, (ulong)1<<22);

	}
	return VTSS_OK;
}


/*******************************************************************************
  Function 
  vtss_ingress_shaper_set()

  Description
  This function configures bitrate in the ingress direction for a given 
  logical port.

  Syntax 
  vtss_rc vtss_ingress_shaper_set( vtss_port_no_t portnum, vtss_bitrate_t br);

  Parameters 
  portnum        Logical port number
  br             Bitrate (in kilobits/second)

  Note
  The bitrate is in kbits/second, so to get a throughput in bits/second 
  the value must be multiplied by 1000.

  Returns 
  VTSS_OK        No errors detected


 ********************************************************************************/
vtss_rc vtss_ingress_shaper_set( vtss_port_no_t portnum, vtss_kbitrate_t br,
		ulong lblvl)
{


	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	//  VTSS_N(("Shaper setup (ingress dir). Logic port #%d, phys prt #%d", portnum, ppn));
	printk("Shaper setup (ingress dir). Logic port #%d, phys prt #%d\n", portnum, ppn);
	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}

	if( br == VTSS_DISABLE_SHAPER) {
		/* disable the shaper */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_TRAFFIC_SHAPER_CTRL,
				(ulong)1<<22, (ulong)1<<22);

	} else {

		ulonglong reg_value = (ulonglong)br*1000/VTSS_SHAPER_BITRATE_UNIT_PER_PORT;
		ulong lvl = lblvl/VTSS_SHAPER_LEAKY_BUCKET_UNIT;

		if (reg_value > 0xFFFF)
			reg_value = 0xFFFF;

		if ( lvl > 0xFFFF)
			lvl = 0xFFFF;

		reg_value |= (lvl << 16);

		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, get_shaper_register( ppn), 
				reg_value);

		/* Enable the shaper */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TRAFFIC_SHAPER_CTRL,
				0, (ulong)1<<22 | (ulong)1<<ppn);
	}

	return VTSS_OK;
}


/*******************************************************************************
  Function 
  vtss_egress_common_shaper_set()

  Description
  This function configures bitrate and leaky bucket level  of the common 
  bucket in the egress direction.

  Syntax 
  vtss_rc vtss_ingress_common_shaper_set( vtss_bitrate_t br);

  Parameters 
  br             Bitrate (in kilobits/second)
  lblvl          Leaky bucket level in bytes. Range (0..0xFFFF)*128 bytes

  Note
  The bitrate is in kbits/second, so to get a throughput in bits/second 
  the value must be multiplied by 1000.


  Returns 
  VTSS_OK        No errors detected


 ********************************************************************************/
vtss_rc vtss_egress_common_shaper_set( vtss_kbitrate_t br, ulong lblvl)
{
	if( br == VTSS_DISABLE_SHAPER) {
		/* disable the shaper */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TRAFFIC_SHAPER_CTRL,
				(ulong)1<<22, (ulong)1<<22);

	} else {
		/* Check that shaping is enabled */

		ulonglong reg_value = (ulonglong)br*1000/VTSS_SHAPER_BITRATE_UNIT_PER_LINK;
		ulong lvl = lblvl/VTSS_SHAPER_LEAKY_BUCKET_UNIT;

		if (reg_value > 0xFFFF)
			reg_value = 0xFFFF;

		if ( lvl > 0xFFFF)
			lvl = 0xFFFF;

		reg_value |= (lvl << 16);

		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TRAFFIC_SHAPER_BUCKET10,
				reg_value);

		/* Enable the shaper */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_EGRESS, M2_TRAFFIC_SHAPER_CTRL,
				0, (ulong)1<<22 | (ulong)1<<10);
	}

	return VTSS_OK;
}


/*******************************************************************************
  Function 
  vtss_ingress_common_shaper_set()

  Description
  This function configures bitrate and leaky bucket level in the ingress 
  direction for the common shaper

  Syntax 
  vtss_rc vtss_ingress_shaper_set( vtss_port_no_t portnum, vtss_bitrate_t br);

  Parameters 
  portnum        Logical port number
  br             Bitrate (in kilobits/second)

  Note
  The bitrate is in kbits/second, so to get a throughput in bits/second 
  the value must be multiplied by 1000.

  Returns 
  VTSS_OK        No errors detected


 ********************************************************************************/
vtss_rc vtss_ingress_common_shaper_set( vtss_kbitrate_t br, ulong lblvl)
{
	if( br == VTSS_DISABLE_SHAPER) {
		/* disable the shaper -- set DB (disable buckets) bit in the */
		/* TRAFFIC_SHAPER_CTRL register */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_TRAFFIC_SHAPER_CTRL,
				(ulong)1<<22, (ulong)1<<22);

	} else {
		/* Check that shaping is enabled */

		ulonglong reg_value = (ulonglong)br*1000/VTSS_SHAPER_BITRATE_UNIT_PER_LINK;
		ulong lvl = lblvl/VTSS_SHAPER_LEAKY_BUCKET_UNIT;

		if (reg_value > 0xFFFF)
			reg_value = 0xFFFF;

		if ( lvl > 0xFFFF)
			lvl = 0xFFFF;

		reg_value |= (lvl << 16);

		vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_TRAFFIC_SHAPER_BUCKET10,
				reg_value);

		/* Enable the shaper */
		vtss_io_writemasked( M2_BLK_FIFO, M2_SUBBLK_INGRESS, M2_TRAFFIC_SHAPER_CTRL,
				0, (ulong)1<<22 | (ulong)1<<10);
	}

	return VTSS_OK;
}


vtss_rc vtss_egress_shaper_get( vtss_port_no_t port_no, vtss_kbitrate_t *pbr, 
		ulong *leaky_bucket_level)
{
	ulong ctrl_reg_value=0;
	ulong bitrate_reg_value=0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_no].chip_port;
	//  VTSS_N(("Shaper setup (ingress dir). Logic port #%d, phys prt #%d", port_no, ppn));
	printk("Shaper setup (ingress dir). Logic port #%d, phys prt #%d\n", port_no, ppn);
	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}


	/* Check that shaping is enabled */
	ctrl_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
			M2_TRAFFIC_SHAPER_CTRL);
	bitrate_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
			get_shaper_register( ppn));


	if( 0x1 == (( ctrl_reg_value >> 22) & 0x1)) {
		/* shaping disabled, return VTSS_DISABLE_SHAPER */
		*pbr = VTSS_DISABLE_SHAPER;
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value >> 16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	} else {
		/* if the shaper is enabled, return the value from the device */
		*pbr = (ulong)( (ulonglong)VTSS_SHAPER_BITRATE_UNIT_PER_PORT * 
				(bitrate_reg_value & 0xFFFF) / 1000);
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value>>16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	}

	return VTSS_OK;
}


vtss_rc vtss_ingress_shaper_get( vtss_port_no_t port_no, vtss_kbitrate_t *pbr, 
		ulong *leaky_bucket_level)
{

	ulong ctrl_reg_value=0;
	ulong bitrate_reg_value=0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[port_no].chip_port;
	//  VTSS_N(("Shaper setup (ingress dir). Logic port #%d, phys prt #%d", port_no, ppn));
	printk("Shaper setup (ingress dir). Logic port #%d, phys prt #%d\n", port_no, ppn);
	/* check the physical port number */
	if (ppn == -1) {
		return VTSS_PORT_NOT_MAPPED;
	}


	/* Check that shaping is enabled */
	ctrl_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
			M2_TRAFFIC_SHAPER_CTRL);
	bitrate_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
			get_shaper_register( ppn));


	if( 0x1 == (( ctrl_reg_value >> 22) & 0x1)) {
		/* shaping disabled, return VTSS_DISABLE_SHAPER */
		*pbr = VTSS_DISABLE_SHAPER;
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value >> 16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	} else {
		/* if the shaper is enabled, return the value from the device */
		*pbr = (ulong)( (ulonglong)VTSS_SHAPER_BITRATE_UNIT_PER_PORT * 
				(bitrate_reg_value & 0xFFFF) / 1000);
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value>>16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	}

	return VTSS_OK;
}



vtss_rc vtss_egress_common_shaper_get( vtss_kbitrate_t *pbr, 
		ulong *leaky_bucket_level)
{

	ulong ctrl_reg_value=0;
	ulong bitrate_reg_value=0;

	//  VTSS_N(("Egress common shaper get"));
	printk("Egress common shaper get\n");

	/* Check that shaping is enabled */
	ctrl_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
			M2_TRAFFIC_SHAPER_CTRL);
	bitrate_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
			M2_TRAFFIC_SHAPER_BUCKET10);


	if( 0x1 == (( ctrl_reg_value >> 22) & 0x1)) {
		/* shaping disabled, return VTSS_DISABLE_SHAPER */
		*pbr = VTSS_DISABLE_SHAPER;
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value >> 16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	} else {
		/* if the shaper is enabled, return the value from the device */
		*pbr = (ulong)( (ulonglong)VTSS_SHAPER_BITRATE_UNIT_PER_LINK * 
				(bitrate_reg_value & 0xFFFF) / 1000);
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value>>16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	}

	return VTSS_OK;
}


vtss_rc vtss_ingress_common_shaper_get( vtss_kbitrate_t *pbr, 
		ulong *leaky_bucket_level)
{

	ulong ctrl_reg_value=0;
	ulong bitrate_reg_value=0;

	//  VTSS_N(("Ingress common shaper get"));
	printk("Ingress common shaper get\n");

	/* Check that shaping is enabled */
	ctrl_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
			M2_TRAFFIC_SHAPER_CTRL);
	bitrate_reg_value = vtss_io_read( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
			M2_TRAFFIC_SHAPER_BUCKET10);


	if( 0x1 == (( ctrl_reg_value >> 22) & 0x1)) {
		/* shaping disabled, return VTSS_DISABLE_SHAPER */
		*pbr = VTSS_DISABLE_SHAPER;
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value >> 16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	} else {
		/* if the shaper is enabled, return the value from the device */
		*pbr = (ulong)( (ulonglong)VTSS_SHAPER_BITRATE_UNIT_PER_LINK * 
				(bitrate_reg_value & 0xFFFF) / 1000);
		*leaky_bucket_level = 
			(0xFFFF & (bitrate_reg_value>>16)) * VTSS_SHAPER_LEAKY_BUCKET_UNIT;
	}

	return VTSS_OK;
}

#endif

/*******************************************************************************

  Aggregation/trunking 


 ********************************************************************************/

/*
   Initialises port map table of the aggregator.
   The function must be called after virtual ports have been mapped to 
   physical ports.
   The function also tests whether the port is enabled or disabled.
   Disabled ports are removed from the aggregation table.
 */
vtss_rc vtss_aggr_pmap_table_initialise( void)
{
	int i;

	for ( i=0; i < M2_AGGR_PMAP_TABLE_SIZE; i++) {
		vtss_aggr_pmap_table_set( i, i % VTSS_PORTS_1G);
	}

	return VTSS_OK;
}



/*
   Writes one entry into the port map table of the aggregator
 */
vtss_rc vtss_aggr_pmap_table_set( int index, int portnum)
{
	vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR,
		 M2_PMAP_TABLE, (portnum<<8)|(index & 0xFF));

	return VTSS_OK;
}



vtss_rc vtss_aggr_setup( vtss_aggr_mode_t *pamode)
{
	ulong reg_value = 0;
	ulong reg_mask = 0x1BE; /* will not alter NLE,HDR,P_PAR and reserved bits */



	if (pamode->preamble_trunking) {

		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_PRE_BIT0POS,
				pamode->u.bit.pos0);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_PRE_BIT1POS,
				pamode->u.bit.pos1);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_PRE_BIT2POS,
				pamode->u.bit.pos2);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_PRE_BIT3POS,
				pamode->u.bit.pos3);
		reg_value = 1 << 5;

	} else if (pamode->mpls_trunking) {

		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_MPLS_BIT0,
				pamode->u.bit.pos0);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_MPLS_BIT1,
				pamode->u.bit.pos1);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_MPLS_BIT2,
				pamode->u.bit.pos2);
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_MPLS_BIT3,
				pamode->u.bit.pos3);
		reg_value = (1<<8) |( 1<<7);

	} else if (pamode->mpls_aggregation) {

		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_MPLS_BITMASK,
				pamode->u.mpls_bitmask);
		reg_value = 1<<7;

	} else if (pamode->l2_aggr || pamode->l3_aggr || pamode->l4_aggr) {

		if ( pamode->l2_aggr)
			SET_BIT( reg_value, 2);
		if ( pamode->l3_aggr)
			SET_BIT( reg_value, 3);
		if ( pamode->l4_aggr)
			SET_BIT( reg_value, 4);

		SET_BIT(reg_value, 1);

	}

	if( pamode->norm_hdr_used == TRUE) {
		SET_BIT( reg_value, 0);/* Norm header is present */
		CLR_BIT( reg_value, 9);/* NLE bit must be cleared when the norm header
					  is present */
	} else {
		CLR_BIT( reg_value, 0);
		SET_BIT( reg_value, 9);
	}

	vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_AGGR, M2_AGGR_SETUP,
			reg_value, reg_mask);

	return VTSS_OK;
}



vtss_rc vtss_aggr_setup_get_default_values( vtss_aggr_mode_t* pam, 
		vtss_mac_major_mode_t mmode)
{

	pam->preamble_trunking = FALSE;
	pam->mpls_trunking = FALSE;
	pam->mpls_aggregation = FALSE;
	pam->l2_aggr = FALSE;
	pam->l3_aggr = FALSE;
	pam->l4_aggr = FALSE;

	pam->u.bit.pos0 = 0;
	pam->u.bit.pos1 = 0;
	pam->u.bit.pos2 = 0;
	pam->u.bit.pos3 = 0;

	pam->u.mpls_bitmask = 0;

	pam->norm_hdr_used = FALSE;


	switch (mmode) {
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:   /* SPI4 <-> 1x10G */

			return VTSS_WRONG_MAJOR_MODE;

		case VTSS_MAC_MAJOR_MODE_SPI4_1G:    /* SPI4 <-> 10x1G */

			return VTSS_WRONG_MAJOR_MODE;

		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:
			/* Default: layer 2 aggregation */
			pam->l2_aggr = TRUE;
			break;

		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:

			/* Default: preamble trunking */
			pam->preamble_trunking = TRUE;

			pam->u.bit.pos0 = M2_PREAMBLE_BIT0_POS;
			pam->u.bit.pos1 = M2_PREAMBLE_BIT1_POS;
			pam->u.bit.pos2 = M2_PREAMBLE_BIT2_POS;
			pam->u.bit.pos3 = M2_PREAMBLE_BIT3_POS;

			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:
			/* Default: layer 2 aggregation */
			pam->l2_aggr = TRUE;
			break;

		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:
			/* Default: preamble trunking */
			pam->preamble_trunking = TRUE;

			pam->u.bit.pos0 = M2_PREAMBLE_BIT0_POS;
			pam->u.bit.pos1 = M2_PREAMBLE_BIT1_POS;
			pam->u.bit.pos2 = M2_PREAMBLE_BIT2_POS;
			pam->u.bit.pos3 = M2_PREAMBLE_BIT3_POS;

			break;

		default:
			/* VTSS_MAC_MAJOR_MODE_UNDEFINED */
			return VTSS_WRONG_MAJOR_MODE;

	}

	return VTSS_OK;
}


/*
   Inform the aggregator that frames are with/without a normalized and/or 
   preamble header
 */
static void vtss_aggr_use_header( BOOL enable, m2_header_t hdr)
{
	ulong cfg_reg = 0;
	BOOL prm_hdr_enabled = FALSE;
	BOOL norm_hdr_enabled = FALSE;

	if( hdr.use_norm_hdr == FALSE && hdr.use_prm_hdr == FALSE)
		return;

	cfg_reg = vtss_io_read( M2_BLK_FIFO, M2_BLK_SYSTEM, M2_CRC_CFG);

	if ( cfg_reg & (1<<4)) {
		switch (cfg_reg & 0xf) {
			case 0:
				prm_hdr_enabled = TRUE;
				norm_hdr_enabled = TRUE;
				break;
			case 7:
				prm_hdr_enabled = TRUE;
				break;
			case 9:
				norm_hdr_enabled = TRUE;
				break;
			default:
				break;
		}
	}

	if( hdr.use_norm_hdr == TRUE) {

		if( enable == TRUE) {
			SET_BIT( cfg_reg, 4);
			if( prm_hdr_enabled == TRUE)
				cfg_reg &= ~(ulong)0xF;
			else
				cfg_reg = (cfg_reg & ~(ulong)0xF) + 9;

		} else {
			/* disable norm hdr */
			if (prm_hdr_enabled == TRUE) {
				SET_BIT( cfg_reg, 4);
				cfg_reg = (cfg_reg & ~(ulong)0xF) + 7;
			}

		}

	}

	if (hdr.use_prm_hdr == TRUE) {

		if ( enable == TRUE) {
			SET_BIT( cfg_reg, 4);
			if( norm_hdr_enabled == TRUE)
				cfg_reg &= ~(ulong)0xF;
			else
				cfg_reg = (cfg_reg & ~(ulong)0xF) + 7;

		} else {
			/* disable preamble header */
			if ( norm_hdr_enabled == TRUE) {
				SET_BIT( cfg_reg, 4);
				cfg_reg = (cfg_reg & ~(ulong)0xF) + 9;
			}

		}

	}

	vtss_io_writemasked( M2_BLK_FIFO, M2_BLK_SYSTEM, 
				M2_CRC_CFG, cfg_reg, 0x1F);

}


/*

   Works only in INGRESS direction. To use normalized header in egress direction
   use the vtss_norm_hdr_expect_enable() function.

   Configures the whole device to use normalized header in ingress direction.
   In 1G<->SPI4 mode normalization (insertion of the normalized header)
   is done by 1G MAC devices.
   In other modes normalized header is inserted by 10G MAC.

   Normalized header is needed when the ingress FIFO runs in frame-interleaved
   mode. 

   To enable normalized header (that also requires setting the LE/NLE bit):
   - write to the 10M/100M/1G/10G MAC block
   - write to the ingress FIFO
   - 
 */
static vtss_rc vtss_hdr_insert( BOOL able, m2_header_t hdr)
{

	/* Global pointer to a structure that can tell in which mode we are */
	vtss_device_setup_t *pdv = pdevice;
	int i;

	switch( pdv->mmode) {
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:        /* SPI4 <-> 10x1G */
			/* configure all 1G devices to normalize frames */
			for (i = M2_PHYS_PORT_1G_MIN; i <= M2_PHYS_PORT_1G_MAX; i++) {

				vtss_port1G_header_insert( i, able, hdr);
			}

			/* Aggregator is not involved in this major mode */

			/* Configure the ingress FIFO */
			vtss_fifo_use_hdr( M2_SUBBLK_INGRESS, able, hdr);
			break;


		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */
			/* Aggregator must be configured */
			vtss_aggr_use_header( able, hdr);

		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */

			/* normalizer in 10G device, only one 10G port in M2 */
			vtss_port10G_header_insert( able, hdr);

			/* Aggregator must be configured */

			/* Ingress FIFO must be configured to run with the norm header */
			vtss_fifo_use_hdr( M2_SUBBLK_INGRESS, able, hdr);

			break;


			/* Undefined major mode */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			/* undefined major mode */
			return VTSS_MAJOR_MODE_NOT_SET;


			/* In the following two modes the ingress channel 
			(1x10G->10x1G) always runs in burst-interleaved mode,
			 i.e. without the normalised  header. Currently, 
			there does not seem to be a reason to use 
		   	normalisation of frames in ingress direction.

			In the egress direction (10x1G->1x10G) the normalized header
			is always enabled, because the egress fifo runs in 
			frame-interleaved mode. Currently there is no reason to
			disable it   */

		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:    /* single chip aggr */

		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:   /* single chip trunking */

		/* In the next major mode ingress fifo always runs in the 
		burst-interleaved  mode */
		case VTSS_MAC_MAJOR_MODE_SPI4_10G:       /* SPI4 <-> 1x10G */

		default:
			return VTSS_WRONG_MAJOR_MODE;

	}
	return VTSS_OK;
}

#if 0
/*
   Works only in EGRESS direction

 */
static vtss_rc vtss_hdr_expect( BOOL able, m2_header_t hdr)
{

	/* Global pointer to a structure that can tell in which mode we are */
	vtss_device_setup_t *pdv = pdevice;
	int i;

	switch( pdv->mmode) {
		case VTSS_MAC_MAJOR_MODE_SPI4_1G:        /* SPI4 <-> 10x1G */
			/*
			   Blocks affected:
			   - System (CRC_CFG)
			   - SPI4
			   - Egress FIFO
			   - 1G MAC (denormalizer register)
			 */

			/* configure all 1G devices to denormalize frames */
			for (i = M2_PHYS_PORT_1G_MIN; i <= M2_PHYS_PORT_1G_MAX; i++) {

				vtss_port1G_header_insert( i, able, hdr);
			}

			/* Aggregator is not involved in this major mode */

			/* Configure the egress FIFO */
			vtss_fifo_use_hdr( M2_SUBBLK_EGRESS, able, hdr);
			break;


		case VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR:  /* */

		case VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK:  /* */

			/* normalizer in 10G device, only one 10G port in M2 */
			vtss_port10G_header_insert( able, hdr);

			/* Aggregator must be configured */

			/* Ingress FIFO must be configured to run with the norm header */

			break;


			/* In the following two modes the ingress channel 
			(1x10G->10x1G) always runs in burst-interleaved mode, 
			i.e. without the normalised header. Thus just repeat 
			initalization.

			In the egress direction (10x1G->1x10G) the normalized 
			header  is always enabled, because the egress fifo 
			runs in frame-interleaved mode */

		case VTSS_MAC_MAJOR_MODE_10G_1G_AGGR:    /* single chip aggr */
		case VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK:   /* single chip trunking */

		/* 1G devices are configured in vtss_norm_hdr_expect_enable() 
		function */
		/* configure all 1G devices to normalize frames */
		/*
			   hdr.use_norm_hdr = TRUE;
			   hdr.use_prm_hdr = FALSE;
			   for (i = M2_PHYS_PORT_1G_MIN; 
				i <= M2_PHYS_PORT_1G_MAX; i++) {

			   vtss_port1G_header_insert_enable( i, hdr);
			   }
			 */

			/* Aggregator is not involved in this major mode, 
			skipping it */

			/* Configure the egress FIFO */
			vtss_fifo_use_hdr( M2_SUBBLK_EGRESS, able, hdr);


			/* normalizer in 10G device, only one 10G port in M2 */
			vtss_port10G_header_insert( able, hdr);

			break;

			/* */
		case VTSS_MAC_MAJOR_MODE_UNDEFINED:
			/* undefined major mode */
			return VTSS_MAJOR_MODE_NOT_SET;

		case VTSS_MAC_MAJOR_MODE_SPI4_10G:       /* SPI4 <-> 1x10G */
			/* In this major mode ingress and egress fifo's 
			always runs in burst-interleaved mode */
			break;

		default:
			return VTSS_WRONG_MAJOR_MODE;

	}

	return VTSS_OK;
}
#endif


/******************************************************************************
 * Description: Set GPIO direction to input or output.
 *
 * \param gpio_no (input): GPIO pin number.
 * \param output (input) : TRUE if output, FALSE if input.
 *
 * \return : Return code OK.
 ******************************************************************************/
vtss_rc vtss_gpio_direction_set(const vtss_gpio_no_t gpio_no, const BOOL output)
{
	//  VTSS_D(("gpio_no: %d, direction: %d",gpio_no,output));
//	printk("gpio_no: %d, direction: %d\n",gpio_no,output);  
	vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_GPIO_CTRL, 
			output ? (ulong)1<<(gpio_no & 0xF) : 0, 
			(ulong)1<<(gpio_no & 0xF));

	return VTSS_OK;
}


/******************************************************************************
 * Description: Read from GPIO input pin.
 *
 * \param gpio_no (input): GPIO pin number.
 *
 * \return : TRUE if pin is high, FALSE if it is low.
 ******************************************************************************/
BOOL vtss_gpio_input_read(const vtss_gpio_no_t gpio_no)
{
	uint value;

	value = vtss_io_read( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, M2_GPIO_IN);

	//  VTSS_D(("gpio_no: %d, input: %i", gpio_no, value));
//	printk("gpio_no: %d, input: %i\n", gpio_no, value);  
	return ( value & ((ulong)1<<(gpio_no & 0xF))) ? 1 : 0;
}


/******************************************************************************
 * Description: Write to GPIO output pin.
 *
 * \param gpio_no (input): GPIO pin number.
 * \param value (input)  : TRUE to set pin high, FALSE to set pin low.
 *
 * \return : TRUE if pin is high, FALSE if it is low.
 ******************************************************************************/
vtss_rc vtss_gpio_output_write(const vtss_gpio_no_t gpio_no, const BOOL value)
{
	//  VTSS_D(("gpio_no: %d, output: %d", gpio_no, value));
//	printk("gpio_no: %d, output: %d\n",gpio_no, value);
	vtss_io_writemasked( M2_BLK_SYSTEM, M2_SUBBLK_CTRL, 
		M2_GPIO_OUT, value ?
		(ulong)1<<(gpio_no & 0xF) : 0, (ulong)1<<(gpio_no & 0xF));

	return VTSS_OK;
}


/******************************************************************************/
/***        BIST test                                **************************/
/******************************************************************************/

/******************************************************************************
 * Description: Start a specific bist test
 *
 * \param bist_no (input): bist test number range <0..VTSS_BIST_NAME_SIZE-1>.
 *
 * \return : VTSS_OK if write was started,
 *           VTSS_BIST_CMD_FAILED if last write was still in progress.
 **************************************************************************kbp*/
vtss_rc vtss_chip_bist_start(const uint bist_no)
{
	ulong value = 0;

	/*Last write completed?*/
	value = vtss_io_read( M2_BLK_SYSTEM, M2_SUBBLK_BIST, M2_RAM_BIST_CMD);
	if ( (value & (1<<27)) != 0 ) { return  VTSS_BIST_CMD_FAILED; }

	/*Enable bist_no test by writing 0x1 to the indirect BIST_COMMAND register*/
	vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_BIST, M2_RAM_BIST_CMD,  
			(1<<24)|(M2_BIST_COMMAND<<16)|(0x1<<8)|bist_no);

	return VTSS_OK;
}


/******************************************************************************
 * Description: Get result for a specific bist test
 *
 * \param bist_no (input): bist test number range <0..VTSS_BIST_NAME_SIZE-1>.
 *                         >= VTSS_BIST_NAME_SIZE returns the write status.
 *
 * \return : VTSS_OK               when bist test went OK
 *           VTSS_BIST_CMD_FAILED  when write accesses failed.
 *           VTSS_BIST_TEST_FAILED when bist test failed.
 **************************************************************************kbp*/
vtss_rc vtss_chip_bist_result(const uint bist_no)
{
	ulong value = 0;
	uint i = 0;

	/* Test for any previous write errors */
	if ( bist_no >= VTSS_BIST_NAME_SIZE ) {
		value = vtss_io_read( M2_BLK_SYSTEM, M2_SUBBLK_BIST, M2_RAM_BIST_CMD);
		if ( (value & (1<<25)) != 0 ) { return  VTSS_BIST_CMD_FAILED; }
		return VTSS_OK;
	}

	/*Wait for the BIST test to be done RD_DATA and when valid.*/
	while ( (value & 0x3FD) == 0 && i++ < 0xF ) {

		/*Transfer bist_no BIST_STATUS register to the 
		RAM_BIST_RESULT register*/
		vtss_io_write( M2_BLK_SYSTEM, M2_SUBBLK_BIST, 
			M2_RAM_BIST_CMD,  
			(0<<24)|(M2_BIST_STATUS<<16)|(bist_no & 0x3f));

		/*Wait for read to complete*/
		do {
			value = vtss_io_read( M2_BLK_SYSTEM, 
			M2_SUBBLK_BIST, M2_RAM_BIST_RESULT);
		} while ( (value & (1<<9) ) != 0 ) ;
	}
	return ( value & (ulong)1<<1 ? VTSS_OK : VTSS_BIST_TEST_FAILED );
}


/******************************************************************************
 * Description: Setup for GFP-T handling in Campbell-I
 *
 * \param portnum
 *          vtss_gfpt_setup_t* gs
 *       
 *
 * \return : VTSS_OK               
 *           VTSS_PORT_NOT_MAPPED
 *           VTSS_WRONG_PARAMETER
 **************************************************************************kbp*/
vtss_rc vtss_gfpt_setup( vtss_port_no_t portnum, vtss_gfpt_setup_t* gs)
{
	ulong gfpt_rx_pcs_reg = 0;

	/* find the physical port number in the global map table */
	int ppn = vtss_logical_ports[portnum].chip_port;
	/* Check the physical port number */
	if (ppn < 0) { return VTSS_PORT_NOT_MAPPED; }

	/* Check selected interface */
	switch ( gs->interface_mode.interface_type) {
		/* right interface */
		case VTSS_PORT_INTERFACE_SERDES:
			if (gs->interface_mode.speed != VTSS_SPEED_ETH_GFPT) {
				return VTSS_WRONG_PARAMETER; 
			}
			break;
		case VTSS_PORT_INTERFACE_TBI:
			/* Select Rx path as TBI interface */
			SET_BIT(gfpt_rx_pcs_reg, 8); 
			break;
			/* Error - wrong interface */
		default:
			return VTSS_WRONG_PARAMETER;
	}

	/* Setup client clock when in clock source mode. */
	/* Note: Per default the GPIos are set as input. */
	/* GBE: use Internal SerDes, FC/2,FC/4,FC: Use GPIO1,
	 (2FC or GBE: Use GPIO4), ESC: Use GPIO7, DVB: Use GPIO10 */  
	if (gs->source_mode_en) {
		vtss_io_writemasked( M2_BLK_MACS, ppn, 
		M2_DEV_SETUP, (ulong)((1<<20)|
			(gs->interface_mode.speed<<16)), (ulong)17<<16);
	}

	/* Setup client signal for the Egress and Ingress direction */
	switch (gs->interface_mode.speed) {
	case VTSS_SPEED_ETH_GFPT:
		/* Setup client signal */
		vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_CLIENT1, 
			(C1_GFPT_CLIENT_GBE<<16)|(C1_GFPT_CLIENT_GBE));
		/* Setup PCS TX */
		vtss_io_write( M2_BLK_MACS, ppn, 
		C1_GFPT_PCS_TX, (ulong)(
			(1<<20)|(0xF1<<4)|((gs->gfpt_en)? 1:0)));
		break;
	case VTSS_SPEED_FC2_GFPT: /* 6    ½Gbit/s Fibre Channel */
	case VTSS_SPEED_FC4_GFPT: /* 7    ¼Gbit/s Fibre Channel */
	case VTSS_SPEED_1FC_GFPT: /* 8    1Gbit/s Fibre Channel or FICON*/
	case VTSS_SPEED_2FC_GFPT: /* 9    2Gbit/s Fibre Channel */
		/* Setup client signal */
		vtss_io_write( M2_BLK_MACS, ppn,
		C1_GFPT_CLIENT1, 
		(C1_GFPT_CLIENT_FC<<16)|(C1_GFPT_CLIENT_FC));
		/* Setup PCS TX */
		vtss_io_write( M2_BLK_MACS, ppn, 
		C1_GFPT_PCS_TX, (ulong)(
			(3<<20)|(0xF1<<4)|((gs->gfpt_en)? 1:0)));
		/* Rx Synchronization mode, FC/FICON */    
		SET_BIT(gfpt_rx_pcs_reg, 1); 
		break;
	case VTSS_SPEED_ESC_GFPT: /* 10 200Mbit/s ESCON or SBCON */
		/* Setup client signal */
		vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_CLIENT1, 
			(C1_GFPT_CLIENT_ESC<<16)|(C1_GFPT_CLIENT_ESC));
		/* Setup PCS TX */
		vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_PCS_TX, 
			(ulong)((0xF1<<4)|((gs->gfpt_en)? 1:0)));
		/* Rx Synchronization mode, ESCON/SBCON */    
		SET_BIT(gfpt_rx_pcs_reg, 2); 
		break;
	case VTSS_SPEED_DVB_GFPT: /* 11 270Mbit/s DVB ASI */
		/* Setup client signal */
		vtss_io_write( M2_BLK_MACS, ppn, 
		C1_GFPT_CLIENT1, (C1_GFPT_CLIENT_DVB<<16)|(C1_GFPT_CLIENT_DVB));
		/*Setup PCS TX */
		vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_PCS_TX, 
		(ulong)((0xF1<<4)|((gs->gfpt_en)? 1:0)));
		/* Rx Synchronization mode, DVB ASI */    
		SET_BIT(gfpt_rx_pcs_reg, 2); 
		SET_BIT(gfpt_rx_pcs_reg, 1); 
		break;

		/* Error - wrong client */
		default:
			return VTSS_WRONG_PARAMETER;
	}

	/* Enable Rx symbol alignment, explicit bit 6=0 */
	SET_BIT(gfpt_rx_pcs_reg, 7); 
	/* Enable GFP-T processing */
	if (gs->gfpt_en != FALSE) { SET_BIT(gfpt_rx_pcs_reg, 0); }
	/* Setup PCS RX */
	vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_PCS_RX, gfpt_rx_pcs_reg);

	/* Change FIFO Threshold levels for GFP-T */
	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_INGRESS, 
		M2_CT_THRHLD + ppn, (ulong)1); 
	vtss_io_write( M2_BLK_FIFO, M2_SUBBLK_EGRESS, 
		M2_CT_THRHLD + ppn, (ulong)((gs->threshold_level<<16)|1)); 

	/* Setup rate adaptation rules */
	if ( gs->rate_period >= 16 ) {
		return VTSS_WRONG_PARAMETER;
	} else if ( gs->rate_max_delta >= 8 ) {
		return VTSS_WRONG_PARAMETER;
	} else if ( gs->rate_min_idles >= 8 ) {
		return VTSS_WRONG_PARAMETER;
	} else {
		vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_CLIENT2, 
		/* (ulong)1<<26 */
		(gs->rate_period<<16) | (gs->rate_max_delta<<8) | 
						gs->rate_min_idles);
	}

	/* Setup Ingress Pading rate */
	if ( gs->rate_padding >= 1024) {
		return VTSS_WRONG_PARAMETER;
	} else {
		vtss_io_write( M2_BLK_MACS, ppn, 
		C1_GFPT_BLOCK_CODE, gs->rate_padding);
	}

	/* GFPT frame length */
	if ( gs->frame_length <= 0 || gs->frame_length >= 256) {
		return VTSS_WRONG_PARAMETER;
	} else {
		vtss_io_write( M2_BLK_MACS, ppn, 
		C1_GFPT_FRM_LEN, gs->frame_length);
	}

	/* GFPT header */
	vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_HEAD_ENA,
			(ulong)(((gs->egress_header_expect)? 1<<1:0) |
			((gs->ingress_header_insert)? 1:0)));

	/* Error correction */
	vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_BIT_ERR_CORR,
		(ulong)((gs->egress_err_corr == VTSS_GFPT_DISABLE_ERR)? 0 : 
		(1 | (gs->egress_err_corr<<1)) & 0x7));

	/* GFP enable */
	vtss_io_write( M2_BLK_MACS, ppn, C1_GFPT_CONFIG, 
		(ulong)((gs->gfpt_en)? 1:0));

	return VTSS_OK;
}


/******************************************************************************
 * Description: Get default setting for GFP-T handling in Campbell-I
 *
 * \param     : vtss_gfpt_setup_t* gs
 *              vtss_port_interface_mode_t  mode
 *
 * \return    : VTSS_OK               
 *              VTSS_WRONG_PARAMETER
 **************************************************************************kbp*/
vtss_rc vtss_gfpt_setup_get_default_values( vtss_gfpt_setup_t* gs,
		vtss_port_interface_mode_t mode)
{

	gs->gfpt_en = TRUE;
	gs->interface_mode = mode;
	gs->rate_padding = 0;
	gs->egress_err_corr = VTSS_GFPT_FORWARD_ERR; /* Let 
			uncorretable error come through */
	gs->source_mode_en = FALSE; /* Let Campbell-I 
		deliver the clock source for TBI */

	/* Setup parameters specific for the client signal  */
	switch (gs->interface_mode.speed) {
		case VTSS_SPEED_ETH_GFPT:
			gs->threshold_level = 14;
			gs->rate_period = 12;
			gs->rate_max_delta = 1;
			gs->rate_min_idles = 3;
			gs->frame_length = 95;
			gs->ingress_header_insert = TRUE;
			gs->egress_header_expect = TRUE;
			break;
		case VTSS_SPEED_2FC_GFPT: /* 6    2Gbit/s Fibre Channel */
		case VTSS_SPEED_1FC_GFPT: /* 7    1Gbit/s Fibre 
					Channel or FICON*/
		case VTSS_SPEED_FC2_GFPT: /* 8    ½Gbit/s Fibre Channel */
		case VTSS_SPEED_FC4_GFPT: /* 9    ¼Gbit/s Fibre Channel */
			gs->threshold_level = 12;
			gs->rate_period = 12;
			gs->rate_max_delta = 1;
			gs->rate_min_idles = 2;
			gs->frame_length = 13;
			gs->ingress_header_insert = FALSE;
			gs->egress_header_expect = FALSE;
			break;
		case VTSS_SPEED_ESC_GFPT: /* 10 200Mbit/s ESCON or SBCON */
			gs->threshold_level = 8;
			gs->rate_period = 10;
			gs->rate_max_delta = 2;
			gs->rate_min_idles = 2;
			gs->frame_length = 1;
			gs->ingress_header_insert = TRUE;
			gs->egress_header_expect = TRUE;
			break;
		case VTSS_SPEED_DVB_GFPT: /* 11 270Mbit/s DVB ASI */
			gs->threshold_level = 10;
			gs->rate_period = 10;
			gs->rate_max_delta = 2;
			gs->rate_min_idles = 2;
			gs->frame_length = 2;
			gs->ingress_header_insert = TRUE;
			gs->egress_header_expect = TRUE;
			break;
			/* Error - wrong client */
		default:
			return VTSS_WRONG_PARAMETER;
	}

	return VTSS_OK;
}


/****************************************************************************/
/*                                                                          */
/*  End of file.                                                            */
/*                                                                          */
/****************************************************************************/



