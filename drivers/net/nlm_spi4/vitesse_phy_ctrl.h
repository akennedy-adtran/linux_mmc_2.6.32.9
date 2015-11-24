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
/*    $Id: vitesse_phy_ctrl.h,v 1.1.2.3 2006-09-28 01:24:16 nphilips Exp $           */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*  File content:                                                           */
/*                                                                          */
/*  vitesse_phy_ctrl.h -- C-library for controlling PHY devices             */
/*                                                                          */
/*                                                                          */
/****************************************************************************/



/* ================================================================= *
 *  PHY
 * ================================================================= */

/* - Read/Write PHY registers via MII-Management ------------------- */

/* returns >=0: (ushort)result or <0: (long)error */
long vtss_phy_read(             const vtss_port_no_t    port_no,
                                const uint              phy_reg );

vtss_rc vtss_phy_write(         const vtss_port_no_t    port_no,
                                const uint              phy_reg,
                                const ushort            value );

vtss_rc vtss_phy_writemasked(   const vtss_port_no_t    port_no,
                                const uint              phy_reg,
                                const ushort            value,
                                const ushort            mask );

/* - PHY Registers, see IEEE 802.3 clause 22.2.4 ------------------- */

/* Some PHYs do not respond to MII-M READ during Reset, although they
   must (according to IEEE 802.3 clause 22.2.4.1.1), so we try reading
   a number of times before we give up when there is no read response
   from the PHY during reset. Set to 0 to disable retrying. */
#define VTSS_PHY_RESET_READ_MAXRETRIES 15625

/* Some PHYs require silence on the MII-M bus a short period after
   soft reset, so we pause this many nanoseconds.
   Set to 0 to disable. */
#define VTSS_PHY_RESET_PAUSE 1000

vtss_rc vtss_phy_reset( const vtss_port_no_t port_no );

/* PHY Basic Capability registers */

/* PHY Control Register (Register 0) */
typedef struct _vtss_phy_control_t {
    BOOL                reset;              /* Self Clearing */
    BOOL                loopback;
    vtss_speed_t        speed;
    BOOL                autoneg_enable;
    BOOL                powerdown;
    BOOL                isolate;
    BOOL                autoneg_restart;    /* Self Clearing */
    BOOL                fdx;                /* Full duplex: TRUE, Half duplex: FALSE */
    BOOL                collision_test;
} vtss_phy_control_t;

vtss_rc vtss_phy_control_get(   const vtss_port_no_t                port_no,
                                vtss_phy_control_t * const          control );

vtss_rc vtss_phy_control_set(   const vtss_port_no_t                port_no,
                                const vtss_phy_control_t * const    control );

/* PHY Status Registers (Registers 1 and 15) */
/* Note: The link_status entry is "shared" between vtss_port_status and 
	vtss_phy_status_get functions. 
*/

typedef struct _vtss_phy_status_t {
    BOOL            ability_100base_t4; /* Note: 100Base-T4 is always hdx. */
    BOOL            ability_100base_x_fdx; /* Covers 100Base-TX or 100Base-FX */
    BOOL            ability_100base_x_hdx; /* Covers 100Base-TX or 100Base-FX */
    BOOL            ability_10mbps_fdx;
    BOOL            ability_10mbps_hdx;
    BOOL            ability_100base_t2_fdx;
    BOOL            ability_100base_t2_hdx;
    BOOL            accepts_mf_preamble_suppression;
    BOOL            autoneg_complete;
    BOOL            remote_fault;
    BOOL            ability_autoneg;
    BOOL            link_status;
    BOOL            jabber_detected;
    BOOL            extended_capability;    /* PHY supports Registers 2-14 and 16-31 */
    BOOL            extended_status;
    struct {
    BOOL            ability_1000base_x_fdx;
    BOOL            ability_1000base_x_hdx;
    BOOL            ability_1000base_t_fdx;
    BOOL            ability_1000base_t_hdx;
    }               extended;    /* Only available when extended_status==TRUE. */
} vtss_phy_status_t;

vtss_rc vtss_phy_status_get(    const vtss_port_no_t        port_no,
                                vtss_phy_status_t * const   status );

/* PHY Extended Capability registers */

/* PHY Identifier (Registers 2 and 3) */
typedef struct _vtss_phy_id_t {
    uint            manufacturer;   /* 22 bits (bits 3-24 of the manufacturer's OUI) */
    uint            model;          /* 6 bits */
    uint            revision;       /* 4 bits */
} vtss_phy_id_t;

/* Note: This function can be used to see if a PHY is present by calling it with id=NULL. */
vtss_rc vtss_phy_id_get(    const vtss_port_no_t    port_no,
                            vtss_phy_id_t * const   id );

/* PHY Auto-Negotiation Advertisement (Register 4) and Link Partner Ability (Register 5) */
typedef struct _vtss_phy_autoneg_advertisement_t {
    BOOL            ability_100base_t4; /* Note: 100Base-T4 is always half duplex. */
    BOOL            ability_100base_x_fdx; /* Covers 100Base-TX or 100Base-FX */
    BOOL            ability_100base_x_hdx; /* Covers 100Base-TX or 100Base-FX */
    BOOL            ability_10base_t_fdx;
    BOOL            ability_10base_t_hdx;
    BOOL            symmetric_pause;    /* a.k.a. PAUSE (PS1) */
    BOOL            asymmetric_pause;   /* a.k.a. ASM_DIR (PS2) */
    BOOL            remote_fault;
    BOOL            acknowledge;    /* Unused in Register 4 */
    BOOL            next_page_present;
} vtss_phy_autoneg_advertisement_t;

/* PHY Auto-Negotiation Advertisement (Register 4) */
vtss_rc vtss_phy_autoneg_advertisement_get( const uint   port_no,
      vtss_phy_autoneg_advertisement_t * const        advertisement );

/* PHY Auto-Negotiation Advertisement (Register 4) */
vtss_rc vtss_phy_autoneg_advertisement_set( const uint   port_no,
        const vtss_phy_autoneg_advertisement_t * const  advertisement );

/* PHY Auto-Negotiation Link Partner Ability (Register 5) */
vtss_rc vtss_phy_autoneg_linkpartner_ability_get(   const uint    port_no,
      vtss_phy_autoneg_advertisement_t * const    linkpartner_advertisement );

/* PHY Master-Slave Control Register (Register 9), 1000Base-T only. 
Set this before PHY Control Register */
typedef struct _vtss_phy_masterslave_control_t {
    uint                test_mode;          /* (3 bits wide), 0: Normal operation */
    BOOL                masterslave_force;  /* Manually set Master/Slave mode */
    BOOL                masterslave_master; /* Only used when masterslave_force==TRUE */
    BOOL                port_type;          /* TRUE: multi-port device, FALSE: single-port device */
    BOOL                ability_1000base_t_fdx; /* Used for Auto-Negotiation */
    BOOL                ability_1000base_t_hdx; /* Used for Auto-Negotiation */
} vtss_phy_masterslave_control_t;

vtss_rc vtss_phy_masterslave_control_get(   const vtss_port_no_t  port_no,
     vtss_phy_masterslave_control_t * const          masterslave_control );

vtss_rc vtss_phy_masterslave_control_set(   const vtss_port_no_t   port_no,
      const vtss_phy_masterslave_control_t * const    masterslave_control );

/* PHY Master-Slave Status Register (Register 10), 1000Base-T only */
typedef struct _vtss_phy_masterslave_status_t {
    BOOL                fault;
    BOOL                master;
    BOOL                local_receiver_status_ok;
    BOOL                remote_receiver_status_ok;
    BOOL                linkpartner_ability_1000base_t_fdx; /* Used for Auto-Negotiation */
    BOOL                linkpartner_ability_1000base_t_hdx; /* Used for Auto-Negotiation */
    uchar               idle_error_count;   /* Saturates when reaching 0xFF */
} vtss_phy_masterslave_status_t;

vtss_rc vtss_phy_masterslave_status_get(    const vtss_port_no_t    port_no,
         vtss_phy_masterslave_status_t * const   masterslave_status );

/* - PHY Auto-Negotiation and Forced Speed ------------------------- */

vtss_rc vtss_phy_force_speed(   const vtss_port_no_t    port_no,
                                const vtss_speed_t      speed,
                                const BOOL              fdx /* Full duplex: TRUE, 
				Half duplex: FALSE */ );


#if defined(HEATHROW2) || defined(STAPLEFORD)
/* ================================================================= *
 *  TBI Auto-Negotiation and Status
 * ================================================================= */

BOOL vtss_tbi_enabled(  const vtss_port_no_t    port_no );

/* Advertisement Word (Refer to IEEE 802.3 Clause 37):
 *  MSB                                                                         LSB
 *  D15  D14  D13  D12  D11  D10   D9   D8   D7   D6   D5   D4   D3   D2   D1   D0 
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 * | NP | Ack| RF2| RF1|rsvd|rsvd|rsvd| PS2| PS1| HD | FD |rsvd|rsvd|rsvd|rsvd|rsvd|
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 */

/* enum used in vtss_autoneg_1000base_x_config_t */
typedef enum _vtss_autoneg_1000base_x_remote_fault_t {
                                     /* RF2 */  /* RF1 */
    VTSS_1000BASEX_LINK_OK          = (( 0 <<1) | ( 0 <<0)),
    VTSS_1000BASEX_OFFLINE          = (( 1 <<1) | ( 0 <<0)),
    VTSS_1000BASEX_LINK_FAILURE     = (( 0 <<1) | ( 1 <<0)),
    VTSS_1000BASEX_AUTONEG_ERROR    = (( 1 <<1) | ( 1 <<0))
} vtss_autoneg_1000base_x_remote_fault_t;

typedef struct _vtss_autoneg_1000base_x_advertisement_t {
    BOOL                                    fdx;
    BOOL                                    hdx;
    BOOL                                    symmetric_pause;    /* a.k.a. PAUSE (PS1) */
    BOOL                                    asymmetric_pause;   /* a.k.a. ASM_DIR (PS2) */
    vtss_autoneg_1000base_x_remote_fault_t  remote_fault;
    BOOL                                    acknowledge;
    BOOL                                    next_page;
} vtss_autoneg_1000base_x_advertisement_t;

typedef struct _vtss_tbi_autoneg_control_t {
    BOOL                                    enable;
    vtss_autoneg_1000base_x_advertisement_t advertisement;
} vtss_tbi_autoneg_control_t;

vtss_rc vtss_tbi_autoneg_control_get( const vtss_port_no_t    port_no,
                   vtss_tbi_autoneg_control_t * const        control );

vtss_rc vtss_tbi_autoneg_control_set( const vtss_port_no_t   port_no,
	        const vtss_tbi_autoneg_control_t * const  control );

vtss_rc vtss_tbi_autoneg_restart( const vtss_port_no_t  port_no );

/* TBI Status, Current state of the PCS */
typedef enum _vtss_tbi_pcs_state_t {
    VTSS_TBI_PCS_STATE_IDLE,      /* Idle */
    VTSS_TBI_PCS_STATE_CONFIG,    /* Config (i.e. ANEG in progress) */
    VTSS_TBI_PCS_STATE_DATA       /* Data */
} vtss_tbi_pcs_state_t;

/* Note: The link_status entry is "shared" between vtss_port_status 
and vtss_tbi_status_get functions. */
typedef struct _vtss_tbi_status_t {
    BOOL                                    link_status;        /* FALSE if link has been down since last status read */
    uint                                    link_down_counter;  /* Note: Saturates when reaching VTSS_TBI_LINK_DOWN_COUNTER_SATURATED. */
    struct {
    vtss_tbi_pcs_state_t                    pcs_state;
    BOOL                                    priority_resolution;
    BOOL                                    complete;
    vtss_autoneg_1000base_x_advertisement_t partner_advertisement;
    }                                       autoneg;
} vtss_tbi_status_t;

#define VTSS_TBI_LINK_DOWN_COUNTER_SATURATED 255

vtss_rc vtss_tbi_status_get( const vtss_port_no_t       port_no,
                             vtss_tbi_status_t * const  status );

#endif /* HEATHROW2/STAPLEFORD */


/****************************************************************************/
/*                                                                          */
/*  End of file.                                                            */
/*                                                                          */
/****************************************************************************/
