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
/*    $Id: vitesse_phy_ctrl.c,v 1.1.2.3 2006-09-28 01:24:16 nphilips Exp $           */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*  File content:                                                           */
/*                                                                          */
/*  vitesse_phy_ctrl.c -- C-library for controlling PHY devices             */
/*                        Copied/compiled from Heathrow PHY ctrl functions  */
/*                                                                          */
/****************************************************************************/
#include "vitesse_highlevel.h"
#include "meigsii_reg.h"
#include "vitesse_io.h"
#include "vitesse_phy_ctrl.h"
#include <asm/netlogic/debug.h>


/* ================================================================= *
 *  PHY
 * ================================================================= */

/* - Read/Write PHY registers via MII-Management ------------------- */
/* returns >=0: (ushort)result or <0: (long)error */
long vtss_phy_read(             const vtss_port_no_t    port_no,
		const uint              phy_reg )
{
	long rc;

	if (!vtss_phy_mapped(port_no)) return VTSS_PHY_NOT_MAPPED;
	rc = vtss_miim_port_reg_read( port_no, phy_reg );
	if (rc<0) return VTSS_PHY_READ_ERROR;
	return rc;
}

vtss_rc vtss_phy_write(         const vtss_port_no_t    port_no,
		const uint              phy_reg,
		const ushort            value )
{
	if (!vtss_phy_mapped(port_no)) return VTSS_PHY_NOT_MAPPED;
	vtss_miim_port_reg_write( port_no, phy_reg, value );
	return VTSS_OK;
}

vtss_rc vtss_phy_writemasked(   const vtss_port_no_t    port_no,
		const uint              phy_reg,
		const ushort            value,
		const ushort            mask )
{
	long rc;

	if (!vtss_phy_mapped(port_no)) return VTSS_PHY_NOT_MAPPED;
	rc = vtss_miim_port_reg_read( port_no, phy_reg );
	if (rc<0) return VTSS_PHY_READ_ERROR;
	rc = (rc & (0xFFFF^mask)) | (value & mask);
	vtss_miim_port_reg_write( port_no, phy_reg, rc );
	return VTSS_OK;
}

/* - PHY Registers, see IEEE 802.3 clause 22.2.4 ------------------- */

vtss_rc vtss_phy_reset( const vtss_port_no_t port_no )
{
	vtss_rc rc;
	vtss_phy_control_t  control;

	control.reset = 1;
	if ((rc=vtss_phy_control_set( port_no, &control ))<0) return rc;
	do {
		uint read_attempt=0;
		do {
			rc = vtss_phy_control_get( port_no, &control );
		} while ((rc==VTSS_PHY_READ_ERROR) 
				&& 
			(read_attempt++<VTSS_PHY_RESET_READ_MAXRETRIES));
		if (rc<0) return rc;
	} while (control.reset);
	return VTSS_OK;
}

vtss_rc vtss_phy_control_get(   const vtss_port_no_t                port_no,
		vtss_phy_control_t * const          control )
{
	long reg0;

	if ((reg0 = vtss_phy_read( port_no, 0 ))<0) return (vtss_rc)reg0;
	control->reset          = MAKEBOOL01(reg0 & (1<<15));
	control->loopback       = MAKEBOOL01(reg0 & (1<<14));
	switch ( reg0 & ((1<<6)|(1<<13)) ) {
	case (1<<6)|(1<<13): control->speed = VTSS_SPEED_UNDEFINED; 
		break;
	case (1<<6)|(0<<13): control->speed = VTSS_SPEED_1G; 
		break;
	case (0<<6)|(1<<13): control->speed = VTSS_SPEED_100M; 
		break;
	case (0<<6)|(0<<13): control->speed = VTSS_SPEED_10M; 
		break;
	}
	control->autoneg_enable = MAKEBOOL01(reg0 & (1<<12));
	control->powerdown      = MAKEBOOL01(reg0 & (1<<11));
	control->isolate        = MAKEBOOL01(reg0 & (1<<10));
	control->autoneg_restart = MAKEBOOL01(reg0 & (1<< 9));
	control->fdx            = MAKEBOOL01(reg0 & (1<< 8));
	control->collision_test = MAKEBOOL01(reg0 & (1<< 7));
	return VTSS_OK;
}

vtss_rc vtss_phy_control_set(   const vtss_port_no_t                port_no,
				const vtss_phy_control_t * const    control )
{
	vtss_rc rc;
	ushort reg0 = 0;

	reg0 |= control->reset          ? (1<<15) : 0;
	reg0 |= control->loopback       ? (1<<14) : 0;
	switch (control->speed) {
	case VTSS_SPEED_UNDEFINED:  
		reg0 |= (1<<6)|(1<<13); 
		break;
	case VTSS_SPEED_1G:         
		reg0 |= (1<<6)|(0<<13); 
		break;
	case VTSS_SPEED_100M:       
		reg0 |= (0<<6)|(1<<13); 
		break;
	case VTSS_SPEED_10M:        
		reg0 |= (0<<6)|(0<<13); 
		break;
	default:  
		reg0 |= (1<<6)|(1<<13); 
		break;
	}
	reg0 |= control->autoneg_enable ? (1<<12) : 0;
	reg0 |= control->powerdown      ? (1<<11) : 0;
	reg0 |= control->isolate        ? (1<<10) : 0;
	reg0 |= (control->autoneg_enable && control->autoneg_restart) ? (1<< 9) : 0;
	reg0 |= control->fdx            ? (1<< 8) : 0;
	reg0 |= control->collision_test ? (1<< 7) : 0;

	rc=vtss_phy_write( port_no, 0, reg0 );
#if VTSS_PHY_RESET_PAUSE
	if (control->reset) {
		/* Wait after issuing a soft reset to the PHY. */
		VTSS_NSLEEP(VTSS_PHY_RESET_PAUSE);
	}
#endif
	return rc;
}

vtss_rc vtss_phy_status_get(    const vtss_port_no_t        port_no,
		vtss_phy_status_t * const   status )
{
	const long reg1 = vtss_phy_read( port_no, 1 );
	if (reg1<0) return (vtss_rc)reg1;
	status->ability_100base_t4              = MAKEBOOL01(reg1 & (1<<15));
	status->ability_100base_x_fdx           = MAKEBOOL01(reg1 & (1<<14));
	status->ability_100base_x_hdx           = MAKEBOOL01(reg1 & (1<<13));
	status->ability_10mbps_fdx              = MAKEBOOL01(reg1 & (1<<12));
	status->ability_10mbps_hdx              = MAKEBOOL01(reg1 & (1<<11));
	status->ability_100base_t2_fdx          = MAKEBOOL01(reg1 & (1<<10));
	status->ability_100base_t2_hdx          = MAKEBOOL01(reg1 & (1<< 9));
	status->accepts_mf_preamble_suppression = MAKEBOOL01(reg1 & (1<< 6));
	status->autoneg_complete                = MAKEBOOL01(reg1 & (1<< 5));
	status->remote_fault                    = MAKEBOOL01(reg1 & (1<< 4));
	status->ability_autoneg                 = MAKEBOOL01(reg1 & (1<< 3));
	status->link_status                     = MAKEBOOL01(reg1 & (1<< 2));
	status->jabber_detected                 = MAKEBOOL01(reg1 & (1<< 1));
	status->extended_capability             = MAKEBOOL01(reg1 & (1<< 0));
	status->extended_status                 = MAKEBOOL01(reg1 & (1<< 8));
	if (status->extended_status) {
		const long reg15 = vtss_phy_read( port_no, 15 );
		if (reg15<0) return (vtss_rc)reg15;
		status->extended.ability_1000base_x_fdx = 
					MAKEBOOL01(reg15 & (1<<15));
		status->extended.ability_1000base_x_hdx = 
					MAKEBOOL01(reg15 & (1<<14));
		status->extended.ability_1000base_t_fdx = 
					MAKEBOOL01(reg15 & (1<<13));
		status->extended.ability_1000base_t_hdx = 
					MAKEBOOL01(reg15 & (1<<12));
	} else {
		status->extended.ability_1000base_x_fdx = 0;
		status->extended.ability_1000base_x_hdx = 0;
		status->extended.ability_1000base_t_fdx = 0;
		status->extended.ability_1000base_t_hdx = 0;
	}
	return VTSS_OK;
}

/* Note: This function can be used to see if a PHY is present by calling it with id=NULL. */
vtss_rc vtss_phy_id_get(const vtss_port_no_t    port_no,
			vtss_phy_id_t * const   id )
{
	long reg2, reg3;

	if ((reg2 = vtss_phy_read( port_no, 2 ))<0) 
		return (vtss_rc)reg2;
	if ((reg3 = vtss_phy_read( port_no, 3 ))<0) 
		return (vtss_rc)reg3;
	if (id) {
		/* 16 bits from reg2 and 6 bits from reg3 */
		id->manufacturer = (reg2 <<6) | ((reg3 >> 10) & 0x3F); 
		id->model        = (reg3 >> 4) & 0x3F; /* 6 bits */
		id->revision     = reg3 & 0xF;         /* 4 bits */
	}
	return VTSS_OK;
}

vtss_rc vtss_phy_autoneg_advertisement_get( const 	uint   	port_no,
		vtss_phy_autoneg_advertisement_t * const        advertisement )
{
	long reg4;

	if ((reg4 = vtss_phy_read( port_no, 4 ))<0) return (vtss_rc)reg4;
	advertisement->ability_100base_t4   = MAKEBOOL01(reg4 & (1<< 9));
	advertisement->ability_100base_x_fdx= MAKEBOOL01(reg4 & (1<< 8));
	advertisement->ability_100base_x_hdx= MAKEBOOL01(reg4 & (1<< 7));
	advertisement->ability_10base_t_fdx = MAKEBOOL01(reg4 & (1<< 6));
	advertisement->ability_10base_t_hdx = MAKEBOOL01(reg4 & (1<< 5));
	advertisement->symmetric_pause      = MAKEBOOL01(reg4 & (1<<10));
	advertisement->asymmetric_pause     = MAKEBOOL01(reg4 & (1<<11));
	advertisement->remote_fault         = MAKEBOOL01(reg4 & (1<<13));
	advertisement->acknowledge          = 0;    /* Unused in Register 4 */
	advertisement->next_page_present    = MAKEBOOL01(reg4 & (1<<15));
	return VTSS_OK;
}

vtss_rc vtss_phy_autoneg_advertisement_set( const uint   port_no,
		const vtss_phy_autoneg_advertisement_t * const  advertisement )
{
	ushort reg4 = 0;

	reg4 |= advertisement->ability_100base_t4   ? (1<< 9) : 0;
	reg4 |= advertisement->ability_100base_x_fdx? (1<< 8) : 0;
	reg4 |= advertisement->ability_100base_x_hdx? (1<< 7) : 0;
	reg4 |= advertisement->ability_10base_t_fdx ? (1<< 6) : 0;
	reg4 |= advertisement->ability_10base_t_hdx ? (1<< 5) : 0;
	reg4 |= advertisement->symmetric_pause      ? (1<<10) : 0;
	reg4 |= advertisement->asymmetric_pause     ? (1<<11) : 0;
	reg4 |= advertisement->remote_fault         ? (1<<13) : 0;
	/* advertisement->acknowledge is unused in Register 4 */
	reg4 |= advertisement->next_page_present    ? (1<<15) : 0;

	/* Selector field must be set to 1 for IEEE802.3 twisted pair. 
	(Refer to IEEE 802.3 Clause 28.2.1.2.1 and Annex 28A) */
	reg4 |= ((1 & 0x1F)<<0);

	return vtss_phy_write( port_no, 4, reg4 );
}

vtss_rc vtss_phy_autoneg_linkpartner_ability_get(   const uint    port_no,
		vtss_phy_autoneg_advertisement_t * const    linkpartner_advertisement )
{
	long reg5;

	if ((reg5 = vtss_phy_read( port_no, 5 ))<0) return (vtss_rc)reg5;
	linkpartner_advertisement->ability_100base_t4   = 
					MAKEBOOL01(reg5 & (1<< 9));
	linkpartner_advertisement->ability_100base_x_fdx= 
					MAKEBOOL01(reg5 & (1<< 8));
	linkpartner_advertisement->ability_100base_x_hdx= 
					MAKEBOOL01(reg5 & (1<< 7));
	linkpartner_advertisement->ability_10base_t_fdx = 
					MAKEBOOL01(reg5 & (1<< 6));
	linkpartner_advertisement->ability_10base_t_hdx = 
					MAKEBOOL01(reg5 & (1<< 5));
	linkpartner_advertisement->symmetric_pause      = 
					MAKEBOOL01(reg5 & (1<<10));
	linkpartner_advertisement->asymmetric_pause     = 
					MAKEBOOL01(reg5 & (1<<11));
	linkpartner_advertisement->remote_fault         = 
					MAKEBOOL01(reg5 & (1<<13));
	linkpartner_advertisement->acknowledge          = 
					MAKEBOOL01(reg5 & (1<<14));
	linkpartner_advertisement->next_page_present    = 
					MAKEBOOL01(reg5 & (1<<15));
	return VTSS_OK;
}

vtss_rc vtss_phy_masterslave_control_get(   const vtss_port_no_t    port_no,
		vtss_phy_masterslave_control_t * const  masterslave_control )
{
	long reg9;

	if ((reg9 = vtss_phy_read( port_no, 9 ))<0) 
		return (vtss_rc)reg9;
	masterslave_control->test_mode              = 
					(reg9 & (7<<13)) >> 13;
	masterslave_control->masterslave_force      = 
					MAKEBOOL01(reg9 & (1<<12));
	masterslave_control->masterslave_master     = 
					MAKEBOOL01(reg9 & (1<<11));
	masterslave_control->port_type              = 
					MAKEBOOL01(reg9 & (1<<10));
	masterslave_control->ability_1000base_t_fdx = 
					MAKEBOOL01(reg9 & (1<< 9));
	masterslave_control->ability_1000base_t_fdx = 
					MAKEBOOL01(reg9 & (1<< 8));
	return VTSS_OK;
}

vtss_rc vtss_phy_masterslave_control_set(   const vtss_port_no_t  port_no,
	const vtss_phy_masterslave_control_t * const  masterslave_control )
{
	ushort reg9 = 0;

	reg9 |= (masterslave_control->test_mode & 7)            <<13;
	reg9 |= masterslave_control->masterslave_force      ? (1<<12) : 0;
	reg9 |= masterslave_control->masterslave_master     ? (1<<11) : 0;
	reg9 |= masterslave_control->port_type              ? (1<<10) : 0;
	reg9 |= masterslave_control->ability_1000base_t_fdx ? (1<< 9) : 0;
	reg9 |= masterslave_control->ability_1000base_t_fdx ? (1<< 8) : 0;

	return vtss_phy_write( port_no, 9, reg9 );
}

vtss_rc vtss_phy_masterslave_status_get(    const vtss_port_no_t     port_no,
	vtss_phy_masterslave_status_t * const   masterslave_status )
{
	long reg10;

	if ((reg10 = vtss_phy_read( port_no, 10 ))<0) return (vtss_rc)reg10;
	masterslave_status->fault                               = 
					MAKEBOOL01(reg10 & (1<<15));
	masterslave_status->master                              = 
					MAKEBOOL01(reg10 & (1<<14));
	masterslave_status->local_receiver_status_ok            = 
					MAKEBOOL01(reg10 & (1<<13));
	masterslave_status->remote_receiver_status_ok           = 
					MAKEBOOL01(reg10 & (1<<12));
	masterslave_status->linkpartner_ability_1000base_t_fdx  = 
					MAKEBOOL01(reg10 & (1<<11));
	masterslave_status->linkpartner_ability_1000base_t_hdx  = 
					MAKEBOOL01(reg10 & (1<<10));
	masterslave_status->idle_error_count                    = 
							reg10 & 0xFF;
	return VTSS_OK;
}

/* - PHY Auto-Negotiation and Forced Speed ------------------------- */

/* Note: Some PHYs (DP83865) requires a reset to run with forced speed. This function does not reset the PHY. */
vtss_rc vtss_phy_force_speed(   const vtss_port_no_t    port_no,
		const vtss_speed_t      speed,
		const BOOL              fdx /* Full duplex: TRUE, Half duplex: FALSE */ )
{
	vtss_rc rc;
	vtss_phy_status_t   status;
	vtss_phy_control_t  control;

	/* Verify PHY ability. */
	if ((rc=vtss_phy_status_get( port_no, &status ))<0) 
		return rc;
	switch (speed) {
	case VTSS_SPEED_10M:
		if (fdx) {
			if (status.ability_10mbps_fdx) break;
		} else {
			if (status.ability_10mbps_hdx) break;
		}
		return VTSS_PHY_ABILITY;
	case VTSS_SPEED_100M:
		if (fdx) {
			if (status.ability_100base_x_fdx) break;
			if (status.ability_100base_t2_fdx) break;
		} else {
			/* Note: 100Base-T4 is always half duplex. */
			if (status.ability_100base_t4) break;
			if (status.ability_100base_x_hdx) break;
			if (status.ability_100base_t2_hdx) break;
		}
		return VTSS_PHY_ABILITY;
	case VTSS_SPEED_1G:
		if (!status.extended_status) return VTSS_PHY_ABILITY;
		if (fdx) {
			if (status.extended.ability_1000base_x_fdx) break;
			if (status.extended.ability_1000base_t_fdx) break;
		} else {
			if (status.extended.ability_1000base_x_hdx) break;
			if (status.extended.ability_1000base_t_hdx) break;
		}
		return VTSS_PHY_ABILITY;
	default:
		return VTSS_PHY_ABILITY;
	}

	/* Set PHY mode. */
	control.reset           = 0;
	control.loopback        = 0;
	control.speed           = speed;
	control.autoneg_enable  = 0;
	control.powerdown       = 0;
	control.isolate         = 0;
	control.autoneg_restart = 0;
	control.fdx             = fdx;
	control.collision_test  = 0;
	return vtss_phy_control_set( port_no, &control );
}




/****************************************************************************/
/*                                                                          */
/*  End of file.                                                            */
/*                                                                          */
/****************************************************************************/
