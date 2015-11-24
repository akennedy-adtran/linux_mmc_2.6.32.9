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
/*  meigsii_reg.h  -- Definitions for Meigs-II chip internal registers.     */
/*                                                                          */
/*                                                                          */
/* The general rule used for naming the registers is to prefix M2_ to the   */
/* register name as described in the datasheet, using all capital letters.  */
/* However, there are a few deviations, which are kept to a minimum and     */
/* pointed out in the code using comments.                                  */
/*                                                                          */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
#ifndef _MEIGSII_REG_H
#define _MEIGSII_REG_H 1


/*
 * Register Addressing:
 *
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |Block ID| Sub Block |    Register Address   |WS|
 * +--------+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *
 * The WS (Word Select) bit selects between the most or least
 * significant 16 bits of the register. Which one it selects
 * depends on the endian mode set in the Parallel Interface
 * setup register.
 */

/*
#define M2_REG_ADDR(block,subblock,register) (((block)<<13)|((subblock)<<9)|((register)<<1))

*/

/* Meigs-II Block IDs */

#define M2_BLK_MACS     1       /* Port - Sub Block: 0-10)                */
#define M2_BLK_FIFO     2       /* FIFOs - Sub Block: 0 Ingress           */
                                /*                    1 Egress            */
#define M2_BLK_MIIM     3       /* MII Management - Sub Block: 0-1)       */
#define M2_BLK_STAT     4       /* Statistics - Sub Block: 0-10)          */
#define M2_BLK_SPI4     5       /* SPI-4.2 I/F - Sub Block: 0)            */
#define M2_BLK_SYSTEM   7       /* System Regs - Sub Block: 1 Aggregator  */
                                /*                          2 Ram BIST    */
                                /*                          F Regs/CPU IF */


/* Meigs-II Sub Block IDs */

#define M2_SUBBLK_MIIM_0 0      /* MII Management Sub Block 0 */
#define M2_SUBBLK_MIIM_1 1      /* MII Management Sub Block 1 */



/* System Block Registers (Block ID: SYSTEM) */
/* Subblock Control 0x0F */
#define M2_SUBBLK_CTRL      0x0F
/* Subblock Aggregator 0x01 */
#define M2_SUBBLK_AGGR      0x01
/* Subblock BIST 0x02 */
#define M2_SUBBLK_BIST      0x02


/* Block without subblocks -- currently only the value of '0' is used */
#define M2_SUBBLK_NONE      0x00

/* Two fifo's subblocks */
#define M2_SUBBLK_INGRESS   0x00
#define M2_SUBBLK_EGRESS    0x01

/* 10x1G mac subblocks + 1x10G mac */
#define M2_SUBBLK_MAC_10G   0x0A



#define M2_CHIPID           0x00    /* Chip Identification */
#define M2_BLADE_ID         0x01    /* Blade Identification */
#define M2_SW_RESET         0x02    /* Global Software Reset */
#define M2_IFACE_MODE       0x07    /* Interface Mode */
#define M2_CRC_CNT          0x0A    /* CRC Error Count */
#define M2_CRC_CFG          0x0B    /* CRC Configuration */
#define M2E_SI_INSERT_BYTES 0x0F    /* SI insert bytes on read */
#define M2_SI_TRANSFER_SEL  0x18    /* SI Transfer Select */
#define M2_PLL_CLK_SPEED    0x19    /* Clock Speed Selection */
#define M2_SYS_CLK_SELECT   0x1C    /* System Clock Select */
#define M2_GPIO_CTRL        0x1D    /* GPIO Control */
#define M2_GPIO_OUT         0x1E    /* GPIO Out */
#define M2_GPIO_IN          0x1F    /* GPIO In */
#define M2_CPU_TRANSFER_SEL 0x20    /* CPU Transfer Select */
#define M2_LOCAL_DATA       0xFE    /* Local CPU Data */
#define M2_LOCAL_STATUS     0xFF    /* Local CPU Status */

/* System Block Registers (Block ID: SYSTEM) */
/* Subblock Aggregator 0x01 */
#define M2_AGGR_SETUP       0x00    /* Aggregator Setup */
#define M2_PMAP_TABLE       0x01    /* Port Map Table */
#define M2_MPLS_BIT0        0x08    /* MPLS Bit0 Position */
#define M2_MPLS_BIT1        0x09    /* MPLS Bit1 Position */
#define M2_MPLS_BIT2        0x0A    /* MPLS Bit2 Position */
#define M2_MPLS_BIT3        0x0B    /* MPLS Bit3 Position */
#define M2_MPLS_BITMASK     0x0C    /* MPLS Bit Mask */
#define M2_PRE_BIT0POS      0x10    /* Preamble Bit0 Position */
#define M2_PRE_BIT1POS      0x11    /* Preamble Bit1 Position */
#define M2_PRE_BIT2POS      0x12    /* Preamble Bit2 Position */
#define M2_PRE_BIT3POS      0x13    /* Preamble Bit3 Position */
#define M2_PRE_ERR_CNT      0x14    /* Preamble Parity Error Counter */

/* System Block Registers (Block ID: SYSTEM) */
/* Subblock BIST 0x02 */

#define M2_RAM_BIST_CMD     0x00    /* RAM BIST Command */
#define M2_RAM_BIST_RESULT  0x01    /* RAM BIST Read Status & Read Result */

/* System Block Registers (Block ID: SYSTEM) */
/* Subblock BIST 0x02, Indirect BIST Register */

#define M2_BIST_PORT_SELECT 0x00    /* BIST Port Select */
#define M2_BIST_COMMAND     0x01    /* BIST Command */
#define M2_BIST_STATUS      0x02    /* BIST Status */
#define M2_BIST_ERR_CNT_LSB 0x03    /* BIST Error Count LSB */
#define M2_BIST_ERR_CNT_MSB 0x04    /* BIST Error Count MSB */
#define M2_BIST_ERR_SEL_LSB 0x05    /* BIST Error Select LSB */
#define M2_BIST_ERR_SEL_MSB 0x06    /* BIST Error Select MSB */
#define M2_BIST_ERROR_STATE 0x07    /* BIST Error State */
#define M2_BIST_ERR_ADR0    0x08    /* BIST Error Address 0 */
#define M2_BIST_ERR_ADR1    0x09    /* BIST Error Address 1 */
#define M2_BIST_ERR_ADR2    0x0A    /* BIST Error Address 2 */
#define M2_BIST_ERR_ADR3    0x0B    /* BIST Error Address 3 */

/* FIFO Block Registers (Block ID: FIFO) */
/* Subblock Ingress 0x00 */
/* Subblock Egress 0x01 */
/* These values are actually base addresses where the actual port
 * address is obtained by adding the port # (0-9) to the address
 */

#define M2_TEST             0x00    /* Mode & Test */
#define M2_TOP_BOTTOM       0x10    /* FIFO Buffer Top & Bottom */
#define M2_TAIL             0x20    /* Write Pointer */
#define M2_HEAD             0x30    /* Read Pointer */
#define M2_HIGH_LOW_WM      0x40    /* Flow Control Water Marks */
#define M2_CT_THRHLD        0x50    /* Cut Through Threshold */
#define M2_FIFO_DROP_CNT    0x60    /* Drop & CRC Error Counter */
#define M2_DEBUG_BUF_CNT    0x70    /* Input Side Debug Counter */

/* FIFO Block Registers (Block ID: FIFO) */
/* Subblock Ingress 0x00 */
/* Subblock Egress 0x01 */

#define M2_TRAFFIC_SHAPER_BUCKET0  0x0A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET1  0x1A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET2  0x2A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET3  0x3A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET4  0x4A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET5  0x5A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET6  0x6A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET7  0x7A    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET8  0x0B    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET9  0x1B    /* Traffic Shaper Bucket Setting */
#define M2_TRAFFIC_SHAPER_BUCKET10 0x2B    /* Traffic Shaper Bucket Setting */

#define M2_TRAFFIC_SHAPER_CTRL     0x3B    /* Traffic Shaper Control Register */


#define M2_SRAM_ADR         0x0E    /* SRAM Address in FIFO Buffer */
#define M2_SRAM_WR_STRB     0x1E    /* SRAM Write Storage Block */
#define M2_SRAM_RD_STRB     0x2E    /* SRAM Read Storage Block */
#define M2_SRAM_DATA_0      0x3E    /* Bytes  3-0 of Data in Block */
#define M2_SRAM_DATA_1      0x4E    /* Bytes  7-4 of Data in Block */
#define M2_SRAM_DATA_2      0x5E    /* Bytes 11-8 of Data in Block */
#define M2_SRAM_DATA_3      0x6E    /* Bytes 15-12 of Data in Block */
#define M2_DATA_BLK_TYPE    0x7E    /* Data Block Type */


/* FIFO Block Registers (Block ID: FIFO) */
/* Subblock Ingress 0x00 */

#define M2_ING_CONTROL      0x0F    /* Ingress Control */

/* FIFO Block Registers (Block ID: FIFO) */
/* Subblock Egress 0x01 */

#define M2_EGR_CONTROL      0x0F    /* Egress Control */

/* FIFO Block Registers (Block ID: FIFO) */
/* Subblock Ingress 0x00 */
/* Subblock Egress 0x01 */

#define M2_AGE_TIMER        0x1F    /* Age Timer */
#define M2_AGE_INC          0x2F    /* Age Increment */
#define M2_DEBUG_OUT        0x3F    /* Output Side Debug Counter Control */
#define M2_DEBUG_CNT        0x4F    /* Output Side Debug Counter */


/* SPI-4.2 Host Interface Block Registers (Block ID: SPI4) */
/* Subblock SPI-4.2 Interface 0x00 */

#define M2_SPI4_MISC        0x00    /* Miscellaneous */
#define M2_SPI4_STATUS      0x01    /* CML Status */
#define M2_SPI4_ING_SETUP0  0x02    /* Ingress Status Channel Setup */
#define M2_SPI4_ING_SETUP1  0x03    /* Ingress Data Training Setup */
#define M2_SPI4_ING_SETUP2  0x04    /* Ingress Data Burst Sizes Setup */
#define M2_SPI4_EGR_SETUP0  0x05    /* Egress Status Channel Setup */
#define M2_SPI4_DBG_CNT     0x10    /* Debug Counters (Add Port # 0 - 9) */
#define M2_SPI4_DBG_SETUP   0x1A    /* Counters Setup */

#define M2_SPI4_TEST        0x20    /* Test Setup */
#define M2_TPGEN_UP0        0x21    /* Test Pattern Generator User Pattern0 */
#define M2_TPGEN_UP1        0x22    /* Test Pattern Generator User Pattern1 */
#define M2_TPCHK_UP0        0x23    /* Test Pattern Checker User Pattern0 */
#define M2_TPCHK_UP1        0x24    /* Test Pattern Checker User Pattern1 */
#define M2_TPSAM_P0         0x25    /* Sampled Pattern 0 */
#define M2_TPSAM_P1         0x26    /* Sampled Pattern 1 */
#define M2_TPERR_CNT        0x27    /* Pattern Checker Error Counter */
#define M2_SPI4_STICKY      0x30    /* Sticky Bits */

#define M2_SPI4_DBG_INH     0x31    /* MeigsII Core Egress & Ingress Inhibit */
#define M2_SPI4_DBG_STATUS  0x32    /* Sampled Ingress Status Channel */
#define M2_SPI4_DBG_GRANT   0x33    /* Ingress Status Channel Granted
                                       Credit Value */

/* MAC Block Registers (Block ID: MAC) */
/* Subblock 10 GbE 0x0A */

#define M2_MISC_10G          0x00    /* Miscellaneous 10GbE Setup */
#define M2_PAUSE             0x01    /* Pause */
#define M2_MAX_LEN           0x02    /* Max Length */
#define M2_MAC_HIGH_ADDR     0x03    /* MAC High Address */
#define M2_MAC_LOW_ADDR      0x04    /* MAC Low Address */
#define M2_NORMALIZER        0x05    /* Normalizer Control */
#define M2_STICKY_RX         0x06    /* RX Debug */
#define M2_DENORM            0x07    /* Denormalizer Control */
#define M2_STICKY_TX         0x08    /* TX Debug */
#define M2_MAC_RXHIGH        0x0A    /* XGMII Lane 0-3 Debug */
#define M2_MAC_RXLOW         0x0B    /* XGMII Lane 4-7 Debug */
#define M2_MAC_TX_STICKY     0x0C    /* MAC TX State Sticky Debug */
#define M2_MAC_TX_RUNNING    0x0D    /* MAC TX State Running Debug */
#define M2_TX_ABORT_AGE      0x14    /* Aged TX Frames Discards Counter */
#define M2_TX_ABORT_SHORT    0x15    /* Short TX Frames Discards Counter */
#define M2_TX_ABORT_TAXI     0x16    /* Taxi Error TX Frames Discards 
                                        Counter */
#define M2_TX_ABORT_UNDERRUN 0x17    /* TX Underrun Abort Discards Counter */
#define M2_TX_DENORM_DISCARD 0x18    /* TX Denormalizer Discards Counter */

#define M2_XAUI_STAT_A       0x20    /* XAUI Status Register A */
#define M2_XAUI_STAT_B       0x21    /* XAUI Status Register B */
#define M2_XAUI_STAT_C       0x22    /* XAUI Status Register C */
#define M2_XAUI_CONF_A       0x23    /* XAUI Configuration Register A */
#define M2_XAUI_CONF_B       0x24    /* XAUI Configuration Register B */
#define M2_XAUI_CODE_GRP_CNT 0x25    /* XAUI Code Group Error Counter */
#define M2_XAUI_CONF_TEST_A  0x26    /* XAUI Test Register A */
#define M2_PDERRCNT          0x27    /* XAUI Test Register B */

/* MAC Block Registers (Block ID: MAC) */
/* Subblock Tri-Speed MACs 0x00-0x09 */

#define M2_MODE_CFG          0x00    /* Mode Configuration */
#define M2_PAUSE_CFG         0x01    /* Pause Configuration */
#define M2_MAXLEN_CFG        0x02    /* Max Length Configuration */
#define M2_MAC_ADDR_HIGH_CFG 0x03    /* MAC Address Configuration - High */
#define M2_MAC_ADDR_LOW_CFG  0x04    /* MAC Address Configuration - Low */
/* Following #define is duplicate of 10 GbE version, so redefine the name
 * to avoid namespace collision, although, the value is equivalent, so
 * it's benign to use either #define
 */
#define M2_TRI_NORMALIZER    0x05    /* Tri-Speed MAC Normalizer */
#define M2_PCS_STATUS        0x06    /* PCS Status */
#define M2_PCS_STATUS_DBG    0x07    /* PCS Status Debug */
#define M2_PCS_CTRL          0x08    /* PCS Control */
#define M2_PCS_CONFIG        0x09    /* PCS Configuration */
#define M2_STICK_BIT         0x0A    /* Sticky Bits */
#define M2_DEV_SETUP         0x0B    /* Tri-Speed MAC Clock/Reset Setup */
#define M2_DROP_CNT          0x0C    /* Drop Counter */
#define M2_PORT_POS          0x0D    /* Preamble Port Position */
#define M2_SERDES_CONF       0x0F    /* SerDes Configuration */
#define M2_SERDES_TEST       0x10    /* SerDes Test */
#define M2_SERDES_STAT       0x11    /* SerDes Status */
#define M2_SERDES_COM_CNT    0x12    /* SerDes Comma Detect Counter */


#define M2E_TRI_MULTI_DBG    0x0E    /* Multidebug register */


/* M2_DENORM is already defined as 0x07 for 10 GbE, but it's 0x15 for
 * tri-speed macs.  This is an unfortunate name collision in the manual,
 * so define it as M2_TRI_DENORM for tri-speed macs. Unfortunately, it is
 * not benign to mix up the tri-speed and 10G #defines!!!
 */

#define M2_TRI_DENORM        0x15    /* Frame Denormalization */

#define M2E_TX_IFG           0x18    /* Tx Inter Frame Gap configuration */
#define M2E_ADV_HDX_CFG      0x19    /* Advance Half Duplex configuration */


/* Only Campbell-I have the following GFP-T registers */
#define C1_GFPT_CONFIG       0x1A    /* GFP-T block configuration */
#define C1_GFPT_CLIENT1      0x1B    /* Client signal configuration */
#define C1_GFPT_CLIENT2      0x1C    /* Rate adaptation configuration */
#define C1_GFPT_BLOCK_CODE   0x1D    /* GFP-T block encode */
#define C1_GFPT_FRM_LEN      0x1E    /* GFP-T Frame length */
#define C1_GFPT_HEAD_ENA     0x1F    /* GFP-T header expectation/generation */
#define C1_GFPT_BIT_ERR_CORR  0x20   /* Bit error correction configuration */
#define C1_GFPT_BIT_ERR_POS1  0x21   /* Bit error insertion position 1 */
#define C1_GFPT_BIT_ERR_POS2  0x22   /* Bit error insertion position 2 */
#define C1_GFPT_BIT_ERR_ONCE  0x23   /* Force bit error insertion once or continuously */
#define C1_GFPT_BIT_ERR_FORCE 0x24   /* Force bit error insert at pos1 and/or pos2 */
#define C1_GFPT_STATUS       0x25    /* GFPT Status */
#define C1_GFPT_PCS_RX       0x26    /* PCS RX Setup */
#define C1_GFPT_PCS_TX       0x27    /* PCS TX Setup */


/* Statistics Block Registers (Block ID: STATISTICS) */
/* Subblock Tri-Speed MACs 0x00-0x09 */

#define M2_RX_IN_BYTES       0x00    /* RX # of Nibbles or Bytes */
#define M2_RX_SYMBOL_CARRIER 0x01    /* RX # of Symbol Errors, No Collisions */
#define M2_RX_PAUSE          0x02    /* RX # of Pause Control Frames */
#define M2_RX_UNSUP_OPCODE   0x03    /* RX # of Unsupported Opcode Frames */
#define M2_RX_OK_BYTES       0x04    /* RX # of Bytes in Valid Frames */
#define M2_RX_BAD_BYTES      0x05    /* RX # of Bytes in Error Frames */
#define M2_RX_UNICAST        0x06    /* RX # of Valid Unicast Frames */
#define M2_RX_MULTICAST      0x07    /* RX # of Valid Multicast Frames */
#define M2_RX_BROADCAST      0x08    /* RX # of Valid Broadcast Frames */
#define M2_RX_CRC            0x09    /* RX # of CRC Error Only Frames */
#define M2_RX_ALIGNMENT      0x0A    /* RX # of Alignment Error Frames */
#define M2_RX_UNDERSIZE      0x0B    /* RX # of Undersize Well Formed Frames */
#define M2_RX_FRAGMENTS      0x0C    /* RX # of Undersize with CRC Error */
#define M2_RX_IN_RANGE_LENGTH_ERROR 0x0D    /* RX # of Frames with Legal
                                               Lengths but Mismatch */
#define M2_RX_OUT_OF_RANGE_ERROR    0x0E    /* RX # of Frames with Illegal
                                               Length Field */

/* NOTE: Names for RX_SIZE_ and TX_SIZE_ registers deviate from manual names
 * for typing convenience
 */

#define M2_RX_OVERSIZE       0x0F    /* RX # of Oversize Well Formed Frames */
#define M2_RX_JABBERS        0x10    /* RX # of Oversize with CRC Error */
#define M2_RX_SIZE_64        0x11    /* RX # of 64 Byte Frames */
#define M2_RX_SIZE_65        0x12    /* RX # of 65-127 Byte Frames */
#define M2_RX_SIZE_128       0x13    /* RX # of 128-255 Byte Frames */
#define M2_RX_SIZE_256       0x14    /* RX # of 256-511 Byte Frames */
#define M2_RX_SIZE_512       0x15    /* RX # of 512-1023 Byte Frames */
#define M2_RX_SIZE_1024      0x16    /* RX # of 1024-1518 Byte Frames */
#define M2_RX_SIZE_1519      0x17    /* RX # of > 1518 Byte Allowed Frames */


#define M2_TX_OUT_BYTES      0x18    /* TX # of Bytes (Good, Bad, Framing) */
#define M2_TX_PAUSE          0x19    /* TX # of Pause Control Frames */
#define M2_TX_OK_BYTES       0x1A    /* TX # of Bytes Successful */
#define M2_TX_UNICAST        0x1B    /* TX # of Unicast Frames */
#define M2_TX_MULTICAST      0x1C    /* TX # of Multicast Frames */
#define M2_TX_BROADCAST      0x1D    /* TX # of Broadcast Frames */
#define M2_TX_MULTIPLE_COLL  0x1E    /* TX # of Frames Successful After
                                        Multiple Collisions */
#define M2_TX_LATE_COL       0x1F    /* TX # of Late Collisions Detected */
#define M2_TX_XCOLL          0x20    /* TX # of Frames Lost Due to 
                                        Excessive Collisions */
#define M2_TX_DEFER          0x21    /* TX # of Frames Deferred on First Try */
#define M2_TX_XDEFER         0x22    /* TX # of Frames Sent with Excessive
                                        Deferral*/
#define M2_TX_CSENSE         0x23    /* TX # of Carrier Sense Error at End
                                        of Frame Transmission*/
#define M2_TX_SIZE_64        0x24    /* TX # of 64 Byte Frames */
#define M2_TX_SIZE_65        0x25    /* TX # of 65-127 Byte Frames */
#define M2_TX_SIZE_128       0x26    /* TX # of 128-255 Byte Frames */
#define M2_TX_SIZE_256       0x27    /* TX # of 256-511 Byte Frames */
#define M2_TX_SIZE_512       0x28    /* TX # of 512-1023 Byte Frames */
#define M2_TX_SIZE_1024      0x29    /* TX # of 1024-1518 Byte Frames */
#define M2_TX_SIZE_1519      0x2A    /* TX # of > 1518 Byte Allowed Frames */
#define M2_TX_SINGLE_COL     0x2B    /* TX # of Single Collision Transmits */
#define M2_TX_BACKOFF2       0x2C    /* TX # of Frames 2 backoff/collision */
#define M2_TX_BACKOFF3       0x2D    /* TX # of Frames 3 backoff/collision */
#define M2_TX_BACKOFF4       0x2E    /* TX # of Frames 4 backoff/collision */
#define M2_TX_BACKOFF5       0x2F    /* TX # of Frames 5 backoff/collision */
#define M2_TX_BACKOFF6       0x30    /* TX # of Frames 6 backoff/collision */
#define M2_TX_BACKOFF7       0x31    /* TX # of Frames 7 backoff/collision */
#define M2_TX_BACKOFF8       0x32    /* TX # of Frames 8 backoff/collision */
#define M2_TX_BACKOFF9       0x33    /* TX # of Frames 9 backoff/collision */
#define M2_TX_BACKOFF10      0x34    /* TX # of Frames 10 backoff/collision */
#define M2_TX_BACKOFF11      0x35    /* TX # of Frames 11 backoff/collision */
#define M2_TX_BACKOFF12      0x36    /* TX # of Frames 12 backoff/collision */
#define M2_TX_BACKOFF13      0x37    /* TX # of Frames 13 backoff/collision */
#define M2_TX_BACKOFF14      0x38    /* TX # of Frames 14 backoff/collision */
#define M2_TX_BACKOFF15      0x39    /* TX # of Frames 15 backoff/collision */
#define M2_TX_UNDERRUN       0x3A    /* TX # of FIFO Underrun Frame Drops */


#define M2_RX_XGMII_PROT_ERR 0x3B    /* XGMII_RX Interface Protocol Errors */
                                     /* NOTE: Only Available on 10 GbE MAC */

#define M2_RX_IPG_SHRINK     0x3C    /* IPG Shrink Detected Counter */

#define M2_STAT_STICKY1G     0x3E    /* Tri-Speed MAC Sticky Bits */
#define M2_STAT_STICKY10G    0x3E    /* 10 GbE MAC Sticky Bits */

#define M2_STAT_INIT         0x3F    /* Clears Statistics in all Tri-Speed
                                        or 10 GbE depending on subblock */

/* MII Management Block Registers (Block ID: MIIM) */
/* Subblock Management #0-1 0x00-0x01 */

#define M2_MIIM_STATUS       0x00    /* MII-M Status */
#define M2_MIIM_CMD          0x01    /* MII-M Command */
#define M2_MIIM_DATA         0x02    /* MII-M Data */
#define M2_MIIM_PRESCALE     0x03    /* MII-M MDC Pre-Scale */


/*************   Most oftern used bits and bitmasks       ******************** */
#define M2_NORMALIZER_BIT_NLE    0x00000004
#define M2_NORMALIZER_BIT_NH     0x00000002
#define M2_NORMALIZER_BIT_PH     0x00000001


#define M2_DENORMAL_BIT_EXP_NH   0x00000002
#define M2_DENORMAL_BIT_ECP_PH   0x00000001


#endif /* _MEIGSII_REG_H */
/****************************************************************************/
/*                                                                          */
/*  End of file.                                                            */
/*                                                                          */
/****************************************************************************/
