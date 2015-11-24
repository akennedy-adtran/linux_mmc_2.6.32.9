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
/*  Unpublished rights reserved under the copyright laws of the             */
/*  United States of America, other countries and international treaties.   */
/*  The software is provided without fee.                                   */  
/*  Permission to use,  copy, store, modify, disclose, transmit or          */
/*  distribute the software is granted, provided that this copyright        */
/*  notice must appear in any copy, modification, disclosure,               */
/*  transmission or distribution of the software.                           */
/*  Vitesse Semiconductor Corporation retains all ownership, copyright,     */
/*  trade secret and proprietary rights in the software.                    */
/*  THIS SOFTWARE HAS BEEN PROVIDED "AS IS," WITHOUT EXPRESS OR IMPLIED     */
/*  WARRANTY INCLUDING, WITHOUT LIMITATION, IMPLIED WARRANTIES OF           */
/*  MERCHANTABILITY, FITNESS FOR A PARTICULAR USE AND NON-INFRINGEMENT.     */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*                                                                          */
/*  vitesse_highlevel.h  -- API functions and data structures for MAC       */
/*                          devices.                                        */
/*                                                                          */
/*                                                                          */
/*  $Id: vitesse_highlevel.h,v 1.1.2.4 2006-09-28 01:24:16 nphilips Exp $           */
/*                                                                          */
/****************************************************************************/
#ifndef _VITESSE_HIGHLEVEL_H
#define _VITESSE_HIGHLEVEL_H 1



/******************************************************************************/
/***        Common Type Definitions                  **************************/
/******************************************************************************/

#include <linux/types.h>
#include "vitesse_common.h"



/******     Return codes   ****************************************************/
typedef int vtss_rc;

#define VTSS_OK                        0     /* rc>=0 means OK */
/* General warnings */
#define VTSS_WARNING                (-0x01)  /* Error, but fixed by API. */
/* General errors */
#define VTSS_UNSPECIFIED_ERROR      (-0x02)
#define VTSS_NOT_IMPLEMENTED        (-0x03)
#define VTSS_INVALID_PARAMETER      (-0x04)
#define VTSS_DATA_NOT_READY         (-0x05)
#define VTSS_PORT_NOT_MAPPED        (-0x06)
#define VTSS_FEATURE_NOT_SUPPORTED  (-0x07)
#define VTSS_WRONG_PARAMETER        (-0x08)
#define VTSS_MAJOR_MODE_NOT_SET     (-0x09)
#define VTSS_WRONG_MAJOR_MODE       (-0x0A)
#define VTSS_MIIM_CHANNEL_BUSY      (-0x0C)
#define VTSS_PHY_READ_ERROR         (-0x0D)
#define VTSS_PHY_NOT_MAPPED         (-0x0E)
#define VTSS_PHY_ABILITY            (-0x0F)
#define VTSS_BIST_CMD_FAILED        (-0x10)
#define VTSS_BIST_TEST_FAILED       (-0x11)


/******************************************************************************/
/***        Type definitions common for 1G and 10G MAC ports   ****************/
/******************************************************************************/
typedef struct _vtss_mac_t {
  uchar addr[6]; /* Network byte order */
} vtss_mac_t;


typedef enum _vtss_speed_t {
  VTSS_SPEED_UNDEFINED,
  VTSS_SPEED_10M,      /* 1   10Mbit/s Ethernet */
  VTSS_SPEED_100M,     /* 2  100Mbit/s Ethernet */
  VTSS_SPEED_1G,       /* 3    1Gbit/s Ethernet */
  VTSS_SPEED_10G,      /* 4   10Gbit/s Ethernet */
  VTSS_SPEED_ETH_GFPT, /* 5    1Gbit/s Ethernet */
  VTSS_SPEED_FC2_GFPT, /* 6    ½Gbit/s Fibre Channel */
  VTSS_SPEED_FC4_GFPT, /* 7    ¼Gbit/s Fibre Channel */
  VTSS_SPEED_1FC_GFPT, /* 8    1Gbit/s Fibre Channel or FICON*/
  VTSS_SPEED_2FC_GFPT, /* 9    2Gbit/s Fibre Channel */
  VTSS_SPEED_ESC_GFPT, /* 10 200Mbit/s ESCON or SBCON */
  VTSS_SPEED_DVB_GFPT  /* 11 270Mbit/s DVB ASI */
} vtss_speed_t;


/* In egress direction it is possible to alter FCS in those frames that 
   were inserted in the stream without FCS, for ex. by the host 
   
   For this feature to work FCS check done in the SPI4 block must be 
   disabled.
*/
typedef enum _vtss_fcs_modify_t {
  VTSS_FCS_DO_NOTHING,
  VTSS_FCS_UPDATE,
  VTSS_FCS_ADD
} vtss_fcs_modify_t;


/******************************************************************************/
/***        Major mode type definitions              **************************/
/******************************************************************************/
/* ----- TBD: mode detailed description of major modes  ------- */
typedef enum _vtss_mac_major_mode_t {
  VTSS_MAC_MAJOR_MODE_UNDEFINED,      /* After the reset */
  VTSS_MAC_MAJOR_MODE_SPI4_1G,        /* SPI4 <-> 10x1G */
  VTSS_MAC_MAJOR_MODE_SPI4_10G,       /* SPI4 <-> 1x10G */
  VTSS_MAC_MAJOR_MODE_10G_1G_AGGR,    /* single chip aggr */
  VTSS_MAC_MAJOR_MODE_10G_1G_TRUNK,   /* single chip trunking */
  VTSS_MAC_MAJOR_MODE_SPI4_10G_AGGR,  /* */
  VTSS_MAC_MAJOR_MODE_SPI4_10G_TRUNK  /* */
} vtss_mac_major_mode_t;


/******************************************************************************/
/***        Device Specific Configuration                 *********************/
/***        may be moved to a device specific .h file     *********************/
/******************************************************************************/

/* Time to wait after a reset in nanoseconds */
#define VTSS_T_RESET                     2000000L

#if defined MEIGS2 || defined VSC7321
#define VTSS_MAX_INGRESS_FIFO_SIZE       (128*1024)
#define VTSS_MAX_EGRESS_FIFO_SIZE        (128*1024)
#elif defined CAMPBELL || defined VSC7331 || defined MEIGS2E || defined VSC7323
/* Meigs2e and Campbell have 3Mbit ingress fifo, egress fifo is the same */
#define VTSS_MAX_INGRESS_FIFO_SIZE       (3*128*1024)
#define VTSS_MAX_EGRESS_FIFO_SIZE        (128*1024)
#endif


#define M2_TEST_FIFO_NORMAL              0x0
#define M2_TEST_FIFO_RX_STOP             0x1
#define M2_TEST_FIFO_CLR                 0x2
#define M2_TEST_FIFO_TX_STOP             0x3
#define M2_TEST_FIFO_REPLAY_TO_TOP       0x4
#define M2_TEST_FIFO_REPLAY_TO_TAIL      0xC


#define VTSS_MAX_FRAME_LENGTH            1518  /* 16-bit value */
#define VTSS_MAX_FRAME_LENGTH_MASK       0xFFFF
#define VTSS_MAX_IFG_VALUE               0xF   /*  4-bit value */
#define VTSS_IFG_MASK                    0xF

/* Recommended values for interframe gaps */
#define VTSS_IFG1_1G                     5
#define VTSS_IFG1_100M_HDX               6
#define VTSS_IFG1_100M_FDX               7
#define VTSS_IFG1_10M_HDX                6
#define VTSS_IFG1_10M_FDX                7


#define VTSS_IFG2_1G                     1
#define VTSS_IFG2_100M_HDX               8
#define VTSS_IFG2_100M_FDX               11
#define VTSS_IFG2_10M_HDX                8
#define VTSS_IFG2_10M_FDX                11


#define M2_1G_PORT_CLOCK_MODE_10M        0x1
#define M2_1G_PORT_CLOCK_MODE_100M       0x2
#define M2_1G_PORT_CLOCK_MODE_1G_GMII    0x3
#define M2_1G_PORT_CLOCK_MODE_1G_SERDES  0x4
#ifndef VSC7321
#define M2E_1G_PORT_CLOCK_MODE_TBI       0x5
#endif

#define M2_1G_PORT_CLOCK_MODE_MASK       0x7


/* Order of bytes in a word and bits in a byte */
#define M2_BIG_END_BYTE_BIG_END_BIT      0x99999999
#define M2_LTL_END_BYTE_BIG_END_BIT      0x81818181

#define M2_BIG_END_BYTE_LTL_END_BIT      0x18181818
#define M2_LTL_END_BYTE_LTL_END_BIT      0x00000000


/**/
#define M2_SW_GLOBAL_RESET               0x80000001

/* Some default values */
#define VTSS_PORT_1G_TX_PAUSE_VALUE      ((ushort)0xFF)
#define VTSS_PORT_1G_TX_PAUSE_MASK       0xFFFF   /* 16bit field */
/* #define VTSS_PORT_1G_TX_PAUSE_XON_XOFF   FALSE */

#define VTSS_PORT_10G_TX_PAUSE_VALUE      ((ushort)0xFF)
#define VTSS_PORT_10G_TX_PAUSE_MASK       0xFFFF   /* 16bit field */
/* #define VTSS_PORT_10G_TX_PAUSE_XON_XOFF   FALSE */


#define VTSS_SPI4_BURST_SIZE     4
#define VTSS_SPI4_MAX_BURST_1    8
#define VTSS_SPI4_MAX_BURST_2    8

#define VTSS_SPI4_OUTPUT_FREQ_390MHZ   390
#define VTSS_SPI4_OUTPUT_FREQ_312MHZ   312
#define VTSS_SPI4_OUTPUT_FREQ_195MHZ   195
#define VTSS_SPI4_OUTPUT_FREQ_156MHZ   156


/* Meigs2 physical port numbering */
#define M2_PHYS_PORT_1G_MIN   0
#define M2_PHYS_PORT_1G_MAX   9

#define M2_PHYS_PORT_10G     10


/* Default values for preamble bit position */
/*     same values as in the datasheet      */
#define M2_PREAMBLE_BIT0_POS   0x37
#define M2_PREAMBLE_BIT1_POS   0x36
#define M2_PREAMBLE_BIT2_POS   0x35
#define M2_PREAMBLE_BIT3_POS   0x34


/* Default values for burst size from FIFO to the ouput module */
#define VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_1G_PORT   1
#define VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_10G_PORT  1
#define VTSS_FIFO_DEFAULT_BURST_SIZE_TOWARDS_SPI4_2    4


#define VTSS_SHAPER_BITRATE_UNIT_PER_PORT   152588
#define VTSS_SHAPER_BITRATE_UNIT_PER_LINK   305176 
#define VTSS_SHAPER_LEAKY_BUCKET_UNIT          128


/* GPIO number: VTSS_GPIO_NO_START..(VTSS_GPIO_NO_END-1) */
typedef uint vtss_gpio_no_t;

#define VTSS_GPIOS (16)
#define VTSS_GPIO_NO_START ((vtss_gpio_no_t)0)
#define VTSS_GPIO_NO_END   (VTSS_GPIO_NO_START+VTSS_GPIOS)



/******************************************************************************/
/***        Default Device Configuration             **************************/
/******************************************************************************/


/*-------        Mapping of logical into physical ports           ------------*/
/* Port Number: 1..VTSS_PORTS */
typedef uint vtss_port_no_t;    /* VTSS_PORT_NO_START..(VTSS_PORT_NO_END-1) */

#define VTSS_PORTS              (10+1)
#define VTSS_PORTS_1G           10
#define VTSS_PORTS_10G          1 /* How many of the VTSS_PORTS are 10G ports */
#define VTSS_CHIP_PORTS         11 /* Number of ports inside chip */
#define VTSS_CHIP_PORT_IS_10G(chip_port) (chip_port == 10)

#define VTSS_PORT_NO_START      ((vtss_port_no_t)1) /* The first logical port 
						       number is 1 */
#define VTSS_PORT_NO_END        (VTSS_PORT_NO_START+VTSS_PORTS)
#define VTSS_PORT_ARRAY_SIZE    VTSS_PORT_NO_END

#define VTSS_PORT_IS_PORT(x)    (x>=VTSS_PORT_NO_START && x<VTSS_PORT_NO_END)

/* MeigsII/MeigsIIe/Campbell port mapping: 0..9 -- 10/100/1000M, 10 -- 10G */




/******************************************************************************/
/***        General Device Configuration             **************************/
/******************************************************************************/

/* 1G ports of a MAC device are known to the user under the numbers on the 
   front panel of the final system. This numeration is not necessarily the same 
   as the internal numeration from 0..MAX_PORT_NUMBER. Thus the need for 
   mapping. Terminology: logical port number -- the number on the front panel,
   physical "chip_port" port number -- internal number from 0 to 
   MAX_1G_PORT_NUMBER. Some additional information such as the address of the 
   corresponding PHY device and the number of a MIIM controller to use to 
   communicate with the PHY device is also in the mapping table */

typedef struct _vtss_mapped_port_t {
    int     chip_port;          /* Physical port. Set to -1 if not in use. */
    int     miim_controller;    /* Set to 0 or 1. Set to -1 if not in use. */
    int     phy_addr;           /* Ignored if miim_controller==-1. */
} vtss_mapped_port_t;


/* 
   Set physical addresses of logical ports. Index is logical port_no. 
   The mapping table must be initialized early in the initialization phase,
   as most funcitons use logical port number as an argument and get the
   matching physical port from the table.
*/
vtss_rc vtss_port_map_set( const vtss_mapped_port_t mapped_ports[VTSS_PORT_ARRAY_SIZE] );

BOOL vtss_port_mapped( const vtss_port_no_t port_no );


/*******************************************************************************

  Function vtss_major_mode_set( ).

  Prepares the system to run in one of the major modes.
  This function must be called only once after reset. If the function is 
  called when the system is up and running it will return an error.
  It is not allowed to call this function with VTSS_MAC_MAJOR_MODE_UNDEFINED
  as an argument.

*******************************************************************************/
vtss_rc vtss_major_mode_set( vtss_mac_major_mode_t mm);



/******************************************************************************/
/***        System Configuration                     **************************/
/******************************************************************************/
typedef struct vtss_system_setup_t {

  BOOL big_endian;      /* TRUE -- CPU interface runs as big-endian */
  uint si_dummy_bytes;  /* number of dummy bytes that will be inserted in the 
			   reply before valid data when serial interface is used
			   at high frequency. Set to 0 if not used */

  /* serdes_ref_clk_external must be TRUE when using VSC7323 or VSC7331 */
  BOOL serdes_ref_clk_external; /* TRUE serdes ref clk is external, 
				   FALSE -- internal */

  BOOL system_ref_clk_extern;  /* TRUE  -- one of the GPIO pins is used as 
				  system clock input 
				  FALSE -- internal source is used for the 
				  system clock (default)
			       */

  BOOL single_chip_mode;  /* TRUE -- 1x10G<->10x1G mode is used 
			     FALSE -- all other modes */

  BOOL rx_chain_mode_10G; /* TRUE -- when in 10G<->SPI4.2 mode 
		         FALSE -- all other modes */

} vtss_system_setup_t;


/*
  Function vtss_system_setup()

  Performs most basic initialization of the chip.
  The function must be called after start-up or chip reset.


  Arguments:
       psystem   pointer to a structure with configuration data

  Return code:
       VTSS_OK   if operation completed successfully
       
 */
vtss_rc vtss_system_setup( vtss_system_setup_t *psystem);

/* 
   This is a supplementary function. It configures a structure referenced 
   by the pointer to the default values for the chosen major mode.
   After that the user may modify the fields of the structure for 
   fine-tuning of the system and than call the vtss_system_setup() function.
*/
vtss_rc vtss_system_setup_get_default_values( vtss_system_setup_t *psystem,
					   vtss_mac_major_mode_t major_mode);


/* vtss_chip_reset() performs a software reset of the chip */
void vtss_chip_reset(void);


/* vtss_get_chip_id() returns the content of the chip identification register */
ulong vtss_chip_id_get(void);



/******************************************************************************/
/***        FIFO Configuration                       **************************/
/******************************************************************************/
typedef struct _vtss_fifo_block_t {
  uint top;       /* 2Kbytes granularity */
  uint bottom;
} vtss_fifo_block_t;


/* Flow control watermarks */
typedef struct _vtss_fifo_fc_watermarks_t {
  uint low_watermark; /* 32 bytes granularity */
  uint high_watermark;
} vtss_fifo_fc_watermarks_t;


/* Cut-through threshold */
typedef struct _vtss_fifo_cut_through_mode_t {
  BOOL cut_through_enable;  /* If FALSE -- FIFO runs in store-forward mode */
  uint threshold_value;
} vtss_fifo_cut_through_mode_t;


/* Ageing mode */
typedef struct _vtss_fifo_ageing_t {
  BOOL enable;    /* enables ageing timer */
  uint interval;  /* ageing interval */
} vtss_fifo_ageing_t;


/* 
   Used for one-time complete setup of the fifo. Usually used after reset,
   Using this structure the user can alter the default setup of the block.
*/
/* May be used to initialize both ingress and egress fifo */
typedef struct vtss_fifo_setup_t {

  vtss_fifo_block_t       fifo_port_area[VTSS_PORT_NO_START+VTSS_PORTS_1G];
  vtss_fifo_fc_watermarks_t fifo_port_wm[VTSS_PORT_NO_START+VTSS_PORTS_1G];

  /* cut-through mode/threshold. If not enabled then store-forward */
  vtss_fifo_cut_through_mode_t    thrhld[VTSS_PORT_NO_START+VTSS_PORTS_1G];

  vtss_fifo_ageing_t ageing;

  /* */
  BOOL norm_hdr_used;
  BOOL prm_hdr_used;

  /**/
  ulong output_port_offset;
  ulong input_port_offset;

} vtss_fifo_setup_t;


/*-----------------  FIFO setup funcitons   ----------------------------------*/

/*
  Function vtss_fifo_setup()

  Initializes the FIFO block based on the information in the provided 
  structures. If the pointer to the structure is set to NULL, the corresponding
  FIFO will not be initialized (it will be left in its current state).

  The function must be called after start-up and may be called at run-
  time to reconfigure the FIFO. 

  Note: port mapping table must be initialized.

  Arguments:
       ps_ingress   pointer to a structure for the ingress fifo
       ps_egress   pointer to a structure for the egress fifo

  Return code:
       VTSS_OK   if operation completed successfully
       
 */
vtss_rc vtss_fifo_setup( vtss_fifo_setup_t *ps_ingress, 
			 vtss_fifo_setup_t *ps_egress);

/* 
   This is a supplementary function. It configures a structure referenced 
   by the pointer to the default values for the chosen major mode.
   After that the user may modify the fields of the structure for 
   fine-tuning of the system and then call the vtss_fifo_setup() function.
*/
vtss_rc vtss_fifo_setup_get_default_values( vtss_fifo_setup_t *ps_ingress,
					    vtss_fifo_setup_t *ps_engress,
					    vtss_mac_major_mode_t major_mode);

/*-----------------  Run-time funcitons   ------------------------------------*/
/* 
   Select store-forward or cut-through mode of fifo for a specified logical 
   port 
*/
vtss_rc vtss_fifo_thrhld_set( vtss_port_no_t portnum, 
			      vtss_fifo_cut_through_mode_t *pctt_ingress,
			      vtss_fifo_cut_through_mode_t *pctt_egress);

/*
  Configure watermarks for a given logical port
*/
vtss_rc vtss_fifo_watermarks_set( vtss_port_no_t portnum,
				  vtss_fifo_fc_watermarks_t *pwm_ingress,
				  vtss_fifo_fc_watermarks_t *pwm_egress);



/******************************************************************************/
/***      10M/100M/1G  Ethernet Port Configuration   **************************/
/******************************************************************************/
/* ================================================================= *
 *  Port
 * ================================================================= */

/* - Port Setup (Operational) -------------------------------------- */

/* The different interfaces for connecting MAC and PHY. */
typedef enum _vtss_port_interface_t {
    VTSS_PORT_INTERFACE_NO_CONNECTION,  /* No connection */
    VTSS_PORT_INTERFACE_MII,            /* MII */
    VTSS_PORT_INTERFACE_GMII,           /* GMII */
    VTSS_PORT_INTERFACE_RGMII,          /* RGMII */
    VTSS_PORT_INTERFACE_TBI,            /* TBI */
    VTSS_PORT_INTERFACE_SERDES,         /* SERDES */
    VTSS_PORT_INTERFACE_XAUI            /* XAUI */
} vtss_port_interface_t;


typedef struct _vtss_port_interface_mode_t {
    vtss_port_interface_t   interface_type; /* not just "interface" because it 
					       is a reserved word in C++ */
    vtss_speed_t            speed;
} vtss_port_interface_mode_t;


typedef struct _vtss_flowcontrol_setup_t {
    BOOL                obey;       /* Should PAUSE frames be obeyed. */
    BOOL                generate;   /* Show flow control:
				       fdx: PAUSE frames, 
				       hdx: backpressure  be generated. */
    vtss_mac_t          smac;
} vtss_flowcontrol_setup_t;


/* Interframe gaps */
typedef struct _vtss_frame_gaps_t {

    uint    ifg1;
    uint    ifg2;

} vtss_frame_gaps_t;



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


typedef struct _vtss_pcs_autoneg_control_t {
    BOOL                                    enable;
    vtss_autoneg_1000base_x_advertisement_t advertisement;
} vtss_pcs_autoneg_control_t;



/* Used for the initial setup of the PCS module */
typedef struct _vtss_pcs_control_t {

  BOOL   disable_lss_on_gpio14;

  BOOL   disable_link_timer;
  BOOL   debug_link_timer_select;

  BOOL   signal_detect_polarity_low;
  BOOL   signal_detect_enable;

  BOOL   show_complete_link_down_counter;

  BOOL   pcs_aneg_sw_resolve;

  BOOL   pcs_autoneg_enable;
  BOOL   pcs_autoneg_restart;

  vtss_autoneg_1000base_x_advertisement_t advertisement;

} vtss_pcs_control_t;


/* Used for the setup of the Serdes module */
#define VTSS_SERDES_CONFIG_WORD        0x0000003E
/* the following two values to be written to the SERDES_TEST register */
#define VTSS_SERDES_RESET_AND_ENABLE   0x00000003
#define VTSS_SERDES_ENABLE             0x00000023


/* Port setup, which may change dynamically, e.g. after auto-negotiation */
typedef struct _vtss_port_setup_t {
  vtss_port_interface_mode_t  interface_mode;
  BOOL                        fdx;                /* Full duplex: TRUE, 
						     Half duplex: FALSE */
  vtss_flowcontrol_setup_t    flowcontrol;
  vtss_frame_gaps_t           frame_gaps;
  BOOL                        enable_tx_pause_xon_xoff;
  uint                        tx_pause_value;
  uint                        maxframelength; /* Max frame length. */
  BOOL                        vlan_aware;
  BOOL                        drop_on_length_error;
  BOOL                        crc_update;
  vtss_fcs_modify_t           fcs_modify;
  BOOL                        invert_gtx_tx_clock;  /* HW related */
  BOOL                        invert_rx_clock;      /* HW related */
  BOOL norm_hdr_insert;
  BOOL prm_hdr_insert;
  uint prm_hdr_value;
  BOOL norm_hdr_expect;
  BOOL prm_hdr_expect;
} vtss_port_setup_t;


/* Setup 10M/100M/1G port */
vtss_rc vtss_port_setup( vtss_port_no_t portnum, vtss_port_setup_t* psetup);

/* 
   Supplementary function: fills the structure referenced by the pointer 
   with the default values for a given major_mode.

   Requires global port mapping table to be initialized
*/
vtss_rc vtss_port_setup_get_default_values( vtss_port_setup_t* psetup,
					 vtss_mac_major_mode_t major_mode);

/*----------------  Run-time funcitons  ---------------------------------------*/
/* Configure speed and duplex mode of a 10M/100M/1G port */
/*
  Arguments:
      portnum   logical port number
      speed     10M, 100M, 1G
      fdx       TRUE - full duplex, FALSE -- half duplex
 */
vtss_rc vtss_port_set_mode( vtss_port_no_t portnum, vtss_speed_t speed, 
			    BOOL fdx);

/* Enable/disable rx, tx or both in a 10M/100M/1G port */
vtss_rc vtss_port_set_enable( vtss_port_no_t portnum, BOOL rx_en, BOOL tx_en);


/* Set CRC adding/updating mode */
/* Only 10/100/1000 ports have this feature -- done in the denormalizer */
vtss_rc vtss_port_fcs_modify( vtss_port_no_t portnum, vtss_fcs_modify_t mc);

/* 
   Discard/do_not_discard  Ethernet frames with wrong FCS in ingress direciton

   Arguments:
        portnum   logical port number
	check     TRUE   -- frames with wrong FCS are discarded (default)
	          FALSE  -- frames with wrong FCS are forwarded upstream
*/
vtss_rc vtss_port_check_fcs( vtss_port_no_t portnum, BOOL check);


/* Allow pause frames pass through the device. By default 10M/100M/1G block
   prevents propagation of pause frames through the device.
 */
vtss_rc vtss_port_forward_pause_frames( vtss_port_no_t portnum, BOOL allow);



/*
  obey -- obeys pause frames from external Eth client 
  generate -- generate pause frames/backpressure upon request from the 
              destination (FIFO or the host interface)

  If fc_obey AND fc_generate are set to FALSE, then flow control is disabled


 */
vtss_rc vtss_port_flow_control_mode( vtss_port_no_t port_num, 
				     BOOL fc_obey, 
				     BOOL fc_generate);



/******************************************************************************/
/***      10G  Ethernet Port Configuration           **************************/
/******************************************************************************/

/* 10G Port setup */
typedef struct _vtss_10Gport_setup_t {
  vtss_port_interface_mode_t  interface_mode;

  vtss_flowcontrol_setup_t    flowcontrol;

  uint                        maxframelength; /* Max frame length. */

  BOOL                        enable_tx_pause_xon_xoff;
  uint                        tx_pause_value;

  BOOL                        vlan_aware;
  BOOL                        pace_mode;
  BOOL                        drop_on_length_error;
  BOOL                        drop_in_range_error;

  /* Extended End-of-packet and Start-of-packet checks */
  BOOL                        ext_eop_check_enable;
  BOOL                        ext_sop_check_enable;

  vtss_fcs_modify_t           fcs_modify;


  /* Start-of-frame delimiter check */
  /* Must be disabled, ie set to FALSE, when using preamble trunking */
  BOOL sfd_check;

  /* Use of the headers */
  BOOL norm_hdr_insert;
  BOOL prm_hdr_insert;

  BOOL norm_hdr_expect;
  BOOL prm_hdr_expect;


} vtss_10Gport_setup_t;


/* Setup 10G port */
vtss_rc vtss_10Gport_setup( vtss_port_no_t port_num, vtss_10Gport_setup_t* psetup);

/* 
   Supplementary function: fills the structure referenced by the pointer 
   with the default values for a given major_mode.

*/
vtss_rc vtss_10Gport_setup_get_default_values( vtss_10Gport_setup_t* psetup,
					    vtss_mac_major_mode_t major_mode);

/*----------------  Run-time funcitons  ---------------------------------------*/
/* Enable/disable rx, tx or both in a 10G port */
vtss_rc vtss_10Gport_set_enable( vtss_port_no_t port_num, 
				 BOOL rx_en, BOOL tx_en);


/* Set CRC adding/updating mode */
vtss_rc vtss_10Gport_fcs_modify( vtss_port_no_t port_num, vtss_fcs_modify_t mc);



/* Allow pause frames pass through the device. By default 10G block
   prevents propagation of pause frames through the device.
 */
vtss_rc vtss_10Gport_forward_pause_frames( vtss_port_no_t port_num, BOOL allow);



/*
  obey -- obeys pause frames from external Eth client 
  generate -- generate pause frames/backpressure upon request from the 
              destination (FIFO or the host interface)

  If fc_obey AND fc_generate are set to FALSE, then flow control is disabled


 */
vtss_rc vtss_10Gport_flow_control_mode( vtss_port_no_t port_num, 
					BOOL fc_obey, 
					BOOL fc_generate);

/*
  Once a 10G Base-X optical module is connected to the XAUI port,
  this function can use to drive an GPIO connected LED to visualize 
  signal detection/link status. The CPU must poll this function.
 */
BOOL vtss_10Gport_signal_detect(void);


/******************************************************************************/
/***        Host Interface (SPI-4.2) Configuration     ************************/
/******************************************************************************/

/*--        type definitions                           -----------------------*/

typedef enum _vtss_spi4_sch_mode_t {
  /* Controled by Ingress(Egress)_Fifo_Ctrl register */
  VTSS_SPI4_BURST_MODE_WITH_HEADER, /* when user wants header on SPI4 */
  VTSS_SPI4_BURST_MODE,  /* Burst interleaved output */
  VTSS_SPI4_FRAME_MODE   /* Frame interleaved output */
} vtss_spi4_sch_mode_t;


typedef enum _vtss_spi4_training_mode_t {
  /* VTSS_SPI4_TRAINING_AUTO not used in Meigs-II */
  VTSS_SPI4_TRAINING_AUTO, /* issue training sequences in response to all '11's
			     in the status channel */
  VTSS_SPI4_TRAINING_OFF,  /* don't issue training sequences */
  VTSS_SPI4_TRAINING_FORCE /* send training sequences continiously */

} vtss_spi4_training_mode_t;


typedef struct _vtss_spi4_setup_t {

  /* 
     This stucture gives a possibility to alter the default configuration of
     the spi4 block after reset according to end-user needs.
  */

  vtss_spi4_sch_mode_t sch_mode;
  uint burst_size;
  uint maxburst1;
  uint maxburst2;

  /* Parameters of the training mode (only in ingress direction) */
  vtss_spi4_training_mode_t tm;
  uint alpha;
  uint tsperiod;

  /* spi4 output frequency*/
  uint spi4_output_freq; /* 390, 312, 195, 156 MHz */

  /* For the description of the following four elements refer to the desription 
     of the SPI4 misc register in the datasheet */
  BOOL spi4_swap_ingress_data;    /* Board dependent */ /* Bit WI, SPI4 MISC reg*/
  BOOL spi4_swap_egress_data;     /* Board dependent */ /* Bit WE, SPI4 MISC reg*/
  BOOL spi4_data_clock_skew;      /* Board dependent */
  BOOL spi4_status_clock_skew;    /* Board dependent */

  /* calendar length is defined by number of ports */
  BOOL	spi4_calendar_order_ascending;  /* Ascending or descending order */
  uint	spi4_calendar_m;

  /* In ingress direction */
  BOOL norm_hdr_strip;

  /* In egress direction */
  BOOL prm_hdr_expect;
  BOOL norm_hdr_expect;


} vtss_spi4_setup_t;


/*--        Setup  Funtions                            -----------------------*/
vtss_rc vtss_spi4_setup( vtss_spi4_setup_t* ps);

/* Configures a user provided structure with values default for the selected 
   major mode.
*/
vtss_rc vtss_spi4_setup_get_default_values( vtss_spi4_setup_t* ps, 
					    vtss_mac_major_mode_t mode);

/*--        Run-time Functions                         -----------------------*/


/*
  Function vtss_spi4_fcs_check_enable()

  The function turns on/off FCS check of Ehternet frames arriving from the 
  host interface. This is done for all channels.


  NOTE. FCS check in the SPI4 block must be OFF if the FCS modify feature is
        used in the egress direction

*/
vtss_rc vtss_spi4_fcs_check_enable( BOOL check);


/*
  Function vtss_spi4_keep_norm_header()

  The funciton controls whether to strip the normalised header or to keep it.
  Usually the normalised header is stripped before the frame is sent over 
  SPI4. However, if the remove receiver can use information in the header, 
  keeping it may be an advantage.


  NOTE. Works in ingress direction

*/
vtss_rc vtss_spi4_keep_norm_header( BOOL keep);



/******************************************************************************/
/***        Flow Control Configuration        *********************************/
/******************************************************************************/

/* 
   Function device_flow_control() selects FIFO flow control mode 
      transparent off -- filling of the FIFO causes generation of pause frames
      transparent on  -- destination (host i/f), not FIFO, requests pause frames

   Note:
   This mode is enabled for all 10/100/1000 Eth ports simultaneously 
*/
vtss_rc vtss_device_transparent_flow_control_mode( BOOL ingress_enable,
						   BOOL egress_enable);





/******************************************************************************/
/***        Statistics                                         ****************/
/******************************************************************************/
typedef struct _vtss_port_counters_t {

  /* if a counter is not implemented in 10G or 1G module the software will 
     return 0 */
  ulong rx_in_bytes;
  ulong rx_symbol_carrier;
  ulong rx_pause;
  ulong rx_unsup_opcode;

  ulong rx_ok_bytes;
  ulong rx_bad_bytes;

  ulong rx_unicast;
  ulong rx_multicast;
  ulong rx_broadcast;
  ulong rx_crc;
  ulong rx_alignment; /* 10/100/1G mac only */
  ulong rx_undersize;

  ulong rx_fragments;
  ulong rx_in_range_error;
  ulong rx_out_of_range_error;
  ulong rx_oversize;
  ulong rx_jabbers;
  ulong rx_size_64;

  ulong rx_size_65_to_127;
  ulong rx_size_128_to_255;
  ulong rx_size_256_to_511;
  ulong rx_size_512_to_1023;
  ulong rx_size_1024_to_1518;
  ulong rx_size_1519_to_max;

  ulong rx_xgmii_prot_err; /* Only on 10G MAC port */

  ulong rx_ipg_shrink;


  ulong tx_out_bytes;
  ulong tx_pause;
  ulong tx_ok_bytes;

  ulong tx_unicast;
  ulong tx_multicast;
  ulong tx_broadcast;

  ulong tx_multiple_coll; /* 10/100/1G MAC only */
  ulong tx_late_coll;     /* 10/100/1G MAC only */
  ulong tx_xcoll;         /* 10/100/1G MAC only */
  ulong tx_defer;         /* 10/100/1G MAC only */
  ulong tx_xdefer;        /* 10/100/1G MAC only */
  ulong tx_carrier_sense; /* 10/100/1G MAC only */


  ulong tx_size_64;
  ulong tx_size_65_to_127;
  ulong tx_size_128_to_255;
  ulong tx_size_256_to_511;
  ulong tx_size_512_to_1023;
  ulong tx_size_1024_to_1518;
  ulong tx_size_1519_to_max;


  ulong tx_single_coll; /* 10/100/1G MAC only */
  ulong tx_backoff2;    /* 10/100/1G MAC only */
  ulong tx_backoff3;    /* 10/100/1G MAC only */
  ulong tx_backoff4;    /* 10/100/1G MAC only */
  ulong tx_backoff5;    /* 10/100/1G MAC only */
  ulong tx_backoff6;    /* 10/100/1G MAC only */
  ulong tx_backoff7;    /* 10/100/1G MAC only */
  ulong tx_backoff8;    /* 10/100/1G MAC only */
  ulong tx_backoff9;    /* 10/100/1G MAC only */
  ulong tx_backoff10;   /* 10/100/1G MAC only */
  ulong tx_backoff11;   /* 10/100/1G MAC only */
  ulong tx_backoff12;   /* 10/100/1G MAC only */
  ulong tx_backoff13;   /* 10/100/1G MAC only */
  ulong tx_backoff14;   /* 10/100/1G MAC only */
  ulong tx_backoff15;   /* 10/100/1G MAC only */

  ulong tx_underrun;

  ulong ingress_overflow_drop;
  ulong egress_overflow_drop;


} vtss_port_counters_t;


/*------------   TBD:  Error monitoring    ---------------------------------*/
/* DIP4, sync errors, fifo ageing drop, etc.. */




/* Clears counter for the logical port */
vtss_rc vtss_port_counters_clear( const vtss_port_no_t port_no );

/* */
vtss_rc vtss_port_counters_get( const vtss_port_no_t  port_no,
				vtss_port_counters_t *pcounters);


/******************************************************************************/
/***        MIIM(MDIO) Interface Functions           **************************/
/******************************************************************************/

#define VTSS_MIIM_READ_ATTEMPT    0xF000

/* Direct miim read/write operation using miim_controller address as a 
parameter */
long vtss_miim_subblock_read( const uint miim_chnl, const uint phy_addr, 
			       const uint phy_reg);
void vtss_miim_subblock_write( const uint miim_chnl, const uint phy_addr, 
			       const uint phy_reg, const ushort value);


/* Indirect miim read/write operation. Accesses the PHY corresponding to the 
   logical port number (PHY address is found via the mapping table)
*/
long vtss_miim_port_reg_read( const vtss_port_no_t port_no, 
			       const uint phy_reg);
void  vtss_miim_port_reg_write( const vtss_port_no_t port_no, 
				const uint phy_reg, const ushort value);


/******************************************************************************/
/***        Policer/shaper                           **************************/
/******************************************************************************/
typedef ulong   vtss_kbitrate_t;

#define VTSS_DISABLE_SHAPER  ((vtss_kbitrate_t)-1)

vtss_rc vtss_egress_shaper_set( vtss_port_no_t port_no, vtss_kbitrate_t br,
				ulong lb_lvl);
vtss_rc vtss_ingress_shaper_set( vtss_port_no_t port_no, vtss_kbitrate_t br,
				 ulong lb_lvl);
vtss_rc vtss_egress_common_shaper_set( vtss_kbitrate_t br, ulong lb_lvl);
vtss_rc vtss_ingress_common_shaper_set( vtss_kbitrate_t br, ulong lb_lvl);

vtss_rc vtss_egress_shaper_get( vtss_port_no_t port_no, vtss_kbitrate_t *pbr, 
				ulong *lb_lvl);
vtss_rc vtss_ingress_shaper_get( vtss_port_no_t port_no, vtss_kbitrate_t *pbr, 
				ulong *lb_lvl);
vtss_rc vtss_egress_common_shaper_get( vtss_kbitrate_t *pbr, ulong *lb_lvl);
vtss_rc vtss_ingress_common_shaper_get( vtss_kbitrate_t *pbr, ulong *lb_lvl);



/******************************************************************************/
/***        Aggregator Configuration                 **************************/
/******************************************************************************/

#define M2_AGGR_PMAP_TABLE_SIZE    256

typedef struct _vtss_aggr_mode_t {
  BOOL preamble_trunking;
  BOOL mpls_trunking;
  BOOL mpls_aggregation;
  BOOL l2_aggr;
  BOOL l3_aggr;
  BOOL l4_aggr;
  union {
    struct {
      ulong pos0;
      ulong pos1;
      ulong pos2;
      ulong pos3;
    } bit;
    ulong mpls_bitmask;
  } u;

  BOOL norm_hdr_used;

} vtss_aggr_mode_t;


vtss_rc vtss_aggr_pmap_table_set( int index, int portnum);

/* Reinitialises aggregation map table, if the port is mot mapped or disabled */
/* it will be removed from the table, if the port has been enabled it will be */
/* added to the table */
vtss_rc vtss_aggr_pmap_table_initialise( void);


/*--        Setup  Funtions                            -----------------------*/
vtss_rc vtss_aggr_setup( vtss_aggr_mode_t* pam);

/* Configures a user provided structure with values default for the selected 
   major mode.
*/
vtss_rc vtss_aggr_setup_get_default_values( vtss_aggr_mode_t* pam, 
					    vtss_mac_major_mode_t mmode);


/******************************************************************************/
/***        PCS Autonegotiation (Serdes and TBI)     **************************/
/******************************************************************************/

vtss_rc vtss_pcs_autoneg_control_get( const vtss_port_no_t                      port_no,
                                      vtss_pcs_autoneg_control_t * const        control );

vtss_rc vtss_pcs_autoneg_control_set( const vtss_port_no_t                      port_no,
                                      const vtss_pcs_autoneg_control_t * const  control );

vtss_rc vtss_pcs_autoneg_restart( const vtss_port_no_t  port_no );


/* Current state of the PCS autonegotiation state machine */
typedef enum _vtss_pcs_aneg_state_t {
  VTSS_PCS_ANEG_STATE_IDLE,      /* Idle */
  VTSS_PCS_ANEG_STATE_CONFIG,    /* Config (i.e. ANEG in progress) */
  VTSS_PCS_ANEG_STATE_NOTVALID,  /*  */
  VTSS_PCS_ANEG_STATE_DATA       /* Data */
} vtss_pcs_aneg_state_t;

typedef struct _vtss_pcs_autoneg_status_t {
  vtss_pcs_aneg_state_t                   aneg_state;
  BOOL                                    aneg_complete;
  vtss_autoneg_1000base_x_advertisement_t partner_advertisement;
} vtss_pcs_autoneg_status_t;



vtss_rc vtss_pcs_autoneg_status_get( const vtss_port_no_t port_no,
				     vtss_pcs_autoneg_status_t *paneg);


typedef struct _vtss_pcs_status_t {

  BOOL  losync;   /* loss of sync: sticky self-cleared bit. PCS sync state 
		     machine has lost sync at least once since last read 
		     of the register */
  BOOL  pcs_in_sync; 
  BOOL  signal_detected; /* signal_detected indicates that there is light 
			    in fiber; require SD_EN in the PCS_CTRL register 
			    to be set */
  BOOL  jtp_lock;
  BOOL  jtp_error;

  BOOL  link_status_ok;  /* link_status:  FALSE if link has been down since 
			    last status read */

  uint  link_down_counter;  /* link down counter: 8bit counter (only 6 bits 
			       visible), saturates when reaching 256 */

  BOOL  show_ldc_top; /* if TRUE indicates that the link down counter 
			 is 8 bit long, not 6 bit as usually */

  vtss_pcs_autoneg_status_t autoneg;

} vtss_pcs_status_t;

#define VTSS_PCS_LINK_DOWN_COUNTER_SATURATED 255

/* Returns more detailed status than vtss_pcs_autoneg_status_get */
vtss_rc vtss_pcs_status_get( const vtss_port_no_t       port_no,
                             vtss_pcs_status_t * const  status );



/******************************************************************************/
/***        Serdes Signal Detect Control             **************************/
/******************************************************************************/
vtss_rc vtss_serdes_signal_detect_setup( const vtss_port_no_t port_num,
					 BOOL enable,
					 BOOL sd_polarity_high,
					 BOOL sd_source_extern);


/*
  Reads serdes signal detect status.

  Return values 
    VTSS_OK                    successful completion

    or an error:
    VTSS_WRONG_MAJOR_MODE
    VTSS_MAJOR_MODE_NOT_SET
    VTSS_PORT_NOT_MAPPED

  If VTSS_OK, then if the content of psignal_detected is 
    TRUE    signal is detected
    FALSE   no signal detected


*/
vtss_rc vtss_serdes_extern_signal_detect_status_get( const vtss_port_no_t port_num,
						     BOOL *psignal_detected);




/******************************************************************************
 * Description: Set GPIO direction to input or output.
 *
 * \param chip_no (input): Chip number (for multi chip targets only).
 * \param gpio_no (input): GPIO pin number.
 * \param output (input) : TRUE if output, FALSE if input.
 *
 * \return : Return code.
 ******************************************************************************/
#if defined(VTSS_CHIPS)
vtss_rc vtss_gpio_direction_set(const vtss_chip_no_t chip_no,
                                const vtss_gpio_no_t gpio_no,
                                const BOOL output);
#else
vtss_rc vtss_gpio_direction_set(const vtss_gpio_no_t gpio_no,
                                const BOOL output);
#endif /* VTSS_CHIPS */


/******************************************************************************
 * Description: Read from GPIO input pin.
 *
 * \param chip_no (input): Chip number (for multi chip targets only).
 * \param gpio_no (input): GPIO pin number.
 *
 * \return : TRUE if pin is high, FALSE if it is low.
 ******************************************************************************/
#if defined(VTSS_CHIPS)
BOOL vtss_gpio_input_read(const vtss_chip_no_t chip_no,
                          const vtss_gpio_no_t gpio_no);
#else
BOOL vtss_gpio_input_read(const vtss_gpio_no_t gpio_no);
#endif /* VTSS_CHIPS */


/******************************************************************************
 * Description: Write to GPIO output pin.
 *
 * \param chip_no (input): Chip number (for multi chip targets only).
 * \param gpio_no (input): GPIO pin number.
 * \param value (input)  : TRUE to set pin high, FALSE to set pin low.
 *
 * \return : TRUE if pin is high, FALSE if it is low.
 ******************************************************************************/
#if defined(VTSS_CHIPS)
/* Write to GPIO output pin */
vtss_rc vtss_gpio_output_write(const vtss_chip_no_t chip_no,
                            const vtss_gpio_no_t gpio_no, 
                            const BOOL value);
#else
vtss_rc vtss_gpio_output_write(const vtss_gpio_no_t gpio_no, 
                            const BOOL value);
#endif /* VTSS_CHIPS */



/******************************************************************************/
/***        BIST test                                **************************/
/******************************************************************************/

#if defined MEIGS2 || defined VSC7321
#define VTSS_BIST_NAME_SIZE 29
#else
#define VTSS_BIST_NAME_SIZE 33
#endif


/******************************************************************************
 * Description: Start a specific bist test
 *
 * \param bist_no (input): bist test number range <0..VTSS_BIST_NAME_SIZE-1>.
 *
 * \return : VTSS_OK if write was started,
 *           VTSS_BIST_CMD_FAILED if last write was still in progress.
 **************************************************************************kbp*/
vtss_rc vtss_chip_bist_start(const uint bist_no);


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
vtss_rc vtss_chip_bist_result(const uint bist_no);


/******************************************************************************/
/***        GFP-T Configuration                      **************************/
/******************************************************************************/

/* Used to define the client signal type for GFP-T */
#define C1_GFPT_CLIENT_GBE    0x0
#define C1_GFPT_CLIENT_FC     0x1
#define C1_GFPT_CLIENT_ESC    0x2
#define C1_GFPT_CLIENT_DVB    0x3

/* GFP-T Egress Frame Disassembler, Single bit Error Correction mode */
typedef enum _vtss_gfpt_error_corr_t {
  VTSS_GFPT_REPLACE_10B_ERR,
  VTSS_GFPT_FORWARD_ERR,
  VTSS_GFPT_DISCARD_ERR,
  VTSS_GFPT_DISABLE_ERR
} vtss_gfpt_error_corr_t;

/* GFP-T port setup, which may change dynamically */
typedef struct _vtss_gfpt_setup_t {
  BOOL                        gfpt_en;
  BOOL                        source_mode_en;
  vtss_port_interface_mode_t  interface_mode;
  uint                        threshold_level;
  uint                        rate_period;
  uint                        rate_max_delta;
  uint                        rate_min_idles;
  uint                        rate_padding;
  uint                        frame_length;
  BOOL                        ingress_header_insert;
  BOOL                        egress_header_expect;
  vtss_gfpt_error_corr_t      egress_err_corr;
} vtss_gfpt_setup_t;


/******************************************************************************
 * Description: Setup for GFP-T handling in Campbell-I
 *
 * \param     : portnum
 *              vtss_gfpt_setup_t* gs
 *       
 *
 * \return    : VTSS_OK               
 *              VTSS_PORT_NOT_MAPPED
 *              VTSS_WRONG_PARAMETER
 **************************************************************************kbp*/
vtss_rc vtss_gfpt_setup( vtss_port_no_t portnum, vtss_gfpt_setup_t* gs);


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
					    vtss_port_interface_mode_t mode);


/******************************************************************************/
/***        Debug Section                            **************************/
/******************************************************************************/

/*-------------   Loopback  funtions will be added here  ---------------------*/

/*-------------   Error counters will be added here      ---------------------*/


BOOL vtss_port_mapped( const vtss_port_no_t port_no );
BOOL vtss_phy_mapped( const vtss_port_no_t port_no );

#endif /* _VITESSE_HIGHLEVEL_H */
/****************************************************************************/
/*                                                                          */
/*  End of file.                                                            */
/*                                                                          */
/****************************************************************************/
