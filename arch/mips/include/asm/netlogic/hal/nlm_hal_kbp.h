
/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * http://www.gnu.org/licenses/gpl-2.0.txt  
 * or the Broadcom license below:

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
 * #BRCM_4# */


#ifndef _NLM_HAL_KBP_H_
#define _NLM_HAL_KBP_H_
#include "nlm_hal_nae.h"
#include "nlm_hal_xlp_dev.h"

#define KBP_CTL                   0x40
#define KBP_INT                   0x41
#define KBP_ERR_CNT               0x42
#define LA_MEM_USAGE              0x43
#define KBP_INT_MASK              0x44
#define KBP_REQ_ID                0x45
#define KBP_REQ_INDEX             0x46
#define TO_LEV2_LOW               0x47
#define TO_LEV2_HIGH              0x48
#define TO_HDR                    0x49
#define TO_DR0_LOW                0x4A
#define TO_DR0_HIGH               0x4B
#define TO_DR1_LOW                0x4C
#define TO_DR1_HIGH               0x4D
#define IODI_TIMEOUT              0x4E
#define KBP_PKT_CNT_CTL           0x4F
#define KBP_PKT_TX_CNT            0x50
#define KBP_PKT_RX_CNT            0x51
#define KBP_RSP_ERROR_PAR         0x52
#define KBP_REQ_TEST_HEAD         0x53
#define KBP_RSP_TEST_HEAD         0x54
#define LA_INT                    0x55
#define LA_INT_MASK               0x56
#define LA_RX_DIAGWORD            0x57
#define LA_RX_SYNC                0x58
#define LA_ERROR1                 0x59
#define LA_ERROR2                 0x5A
#define LA_TX_ENABLE              0x5B
#define LA_TX_EN                  0x01
#define LA_BAD_LANE               0x5C
#define LA_TX_STAT                0x5D
#define LA_TX_RLIM                0x5E
#define LA_SETUP                  0x5F
#define LA_LEN                    0x60
#define SER_CTL                   0x61
#define SER_REG0                  0x62
#define SER_REGALL                0x63
#define IO_CONFIG_SWAP_DIS        0x64
#define STATE_RST                 0x65
#define PKT_ERR_LO                0x66
#define PKT_ERR_HI                0x67
#define RD_EX_BASE                0x68
#define RD_EX_LIMIT               0x69
#define TX_SERR_MSG_0             0x6A
#define TX_SERR_MSG_1             0x6B
#define TX_SERR_MSG_2             0x6C
#define TX_SERR_MSG_3             0x6D
#define TX_MERR_MSG_0             0x6E
#define TX_MERR_MSG_1             0x6F
#define TX_MERR_MSG_2             0x70
#define TX_MERR_MSG_3             0x71
#define RX_ERR_MSG_0              0x72
#define RX_ERR_MSG_1              0x73
#define RX_ERR_MSG_2              0x74
#define RX_ERR_MSG_3              0x75
#define IODI_SLAVE_RD_ERR_MSG_0   0x76
#define IODI_SLAVE_RD_ERR_MSG_1   0x77
#define IODI_SLAVE_RD_ERR_MSG_2   0x78
#define IODI_SLAVE_RD_ERR_MSG_3   0x79
#define IODI_SLAVE_RD_ERR_MSG_4   0x7A
#define IODI_SLAVE_RD_ERR_MSG_5   0x7B
#define IODI_SLAVE_RD_ERR_MSG_6   0x7C
#define IODI_SLAVE_RD_ERR_MSG_7   0x7D

#define XLP_PCIE_KBP_DEV   1 /* pcie device number */
#define KBP_START_LANE     2 /* func number     */

#define DEFAULT_CPLD_MEM_BAR   0x170000

/*KBP PCS common config (CSM) register */

#define KBP_RX_LANE_0_15_EN             0x8100
#define KBP_RX_LANE_16_23_EN            0x8101
#define KBP_TX_LANE_0_11_EN             0x8102
#define KBP_CRX_LANE_0_11_EN            0x8103
#define KBP_CTX_LANE_0_15_EN            0x8104
#define KBP_CTX_LANE_16_23_EN           0x8105
#define KBP_RX_FRM_LEN                  0x8106
#define KBP_TX_FRM_LEN                  0x8107
#define KBP_LANE_SWAP                   0x8108
#define KBP_RX_FIFO_THRD                0x8109
#define KBP_TX_FIFO_THRD                0x810A
#define KBP_TX_BUR_SHORT                0x810B
#define KBP_RX_TX_EN                    0x810C
#define KBP_TX_SER_0_11_SQCH            0x8117
#define KBP_CTX_SER_0_15_SQCH           0x8118
#define KBP_CTX_SER_16_23_SQCH          0x8119
#define KBP_RXTX_CLK_SEL                0x811A
#define KBP_SER_SW_RST                  0x811B
#define KBP_GL_SW_RST                   0x811C
#define KBP_SPEED_SEL                   0x811D
#define KBP_COM_STATUS                  0x8180
#define KBP_GEN_STATUS0                 0x8181
#define KBP_GEN_STATUS1                 0x8182
#define KBP_GEN_STATUS2                 0x8183
#define KBP_GEN_STATUS3                 0x8184

#define KBP_PCS_RX_STAT                 0x8300
#define KBP_PCS_RX_WD_ALGN_STAT         0x8301
#define KBP_PCS_RX_WD_ERR               0x8302
#define KBP_PCS_RX_BLK_ERR              0x8303
#define KBP_PCS_RX_FRM_ERR              0x8304
#define KBP_PCS_RX_DSC_SYNC_ERR         0x8305
#define KBP_PCS_RX_DSC_SGL_ERR          0x8306
#define KBP_PCS_RX_FIFO_ERR             0x8307
#define KBP_PCS_RX_CRC32_ERR            0x8308

#ifndef __ASSEMBLY_
#define MDIO_STATUS_RETRIES             20000

#define EXT_XG_MDIO_CTRL_RST_POS        30
#define EXT_XG_MDIO_CTRL_SMP_POS        20

#define MAKE_MDIO_CTRL(x1, x2, x3, x4, x5, x6, x7, x8, x9)     (\
          (x1 << EXT_XG_MDIO_CTRL_OP_POS)        \
        | (x2 << EXT_XG_MDIO_CTRL_PHYADDR_POS)   \
        | (x3 << EXT_XG_MDIO_CTRL_DEVTYPE_POS)   \
        | (x4 << EXT_XG_MDIO_CTRL_TA_POS)        \
        | (x5 << EXT_XG_MDIO_CTRL_MIIM_POS)      \
        | (x6 << EXT_XG_MDIO_CTRL_LOAD_POS)      \
        | (x7 << EXT_XG_MDIO_CTRL_SMP_POS)       \
        | (x8 << EXT_XG_MDIO_CTRL_XDIV_POS)      \
        | (x9 << EXT_XG_MDIO_CTRL_RST_POS)       )

enum KBP_MDIO_DEV {
    MDIO_DEV_CSM = 1,
    MDIO_DEV_RX_0_3 = 2,
    MDIO_DEV_RX_4_7 = 3,
    MDIO_DEV_RX_8_11 = 4,
    MDIO_DEV_RX_12_15 = 5,
    MDIO_DEV_RX_16_19 = 6,
    MDIO_DEV_RX_20_23 = 7,
    MDIO_DEV_CRX_0_3 = 8,
    MDIO_DEV_CRX_4_7 = 9,
    MDIO_DEV_CRX_8_11 = 10,
    MDIO_DEV_TX_0_3 = 11,
    MDIO_DEV_TX_4_7 = 12,
    MDIO_DEV_TX_8_11 = 13,
    MDIO_DEV_CTX_0_3 = 14,
    MDIO_DEV_CTX_4_7 = 15,
    MDIO_DEV_CTX_8_11 = 16,
    MDIO_DEV_CTX_12_15 = 17,
    MDIO_DEV_CTX_16_19 = 18,
    MDIO_DEV_CTX_20_23 = 19,
    MDIO_DEV_RX_BCAST = 30,
    MDIO_DEV_TX_BCAST = 31,
};

#define XLP_NA_REG_BLOCK_SIZE       0x2000 /* 8KB */
#define PCI_MEM_BAR_0               0x0004
#define MAX_NUMBER_OF_SERDES_LANE   0x8

/**waste cpu clock cycle to achieve delay in time
 @param x - number of 64-bit add operation to perform
 @return None
 */
__inline__ void KBP_DELAY( uint64_t x ){
    volatile uint64_t i = 0;
    volatile uint64_t j = 0;
    for( i = 0; i < x ; i++ ){
        j++;
    } /* end for */
} /* end KBP_DELAY */

__inline__ void WAIT_MDIO_BSY_CLEAR( uint32_t node ){
    volatile uint64_t i = 0;
    for( i = 0; i < MDIO_STATUS_RETRIES ; i++ ){
        if( ( nlm_hal_read_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_RD_STAT ) & EXT_XG_MDIO_STAT_MBSY ) == 0 )
            break;
        /* end if */
    } /* end for */
} /* end WAIT_MDIO_BSY_CLEAR; */

/**read from ILA register on XLP
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param func - function number of register in PCIe space. usually 0x2
 @param reg_addr - register address
 @return 32-bit register data
 */
__inline__ uint32_t nlm_hal_ila_reg_read( uint32_t node, uint32_t func, uint32_t reg_addr ){
    volatile uint64_t mmio = nlm_hal_get_dev_base( node, 0, XLP_PCIE_KBP_DEV, func );
    return nlm_hal_read_32bit_reg( mmio, reg_addr );
} /* end nlm_hal_ila_reg_read */

/**write to ILA register on XLP
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param func - function number of register in PCIe space. usually 0x2
 @param reg_addr - register address
 @param reg_data - register data
 @return None
 */
__inline__ void nlm_hal_ila_reg_write( uint32_t node, uint32_t func, uint32_t reg_addr, uint32_t reg_data ){
    volatile uint64_t mmio = nlm_hal_get_dev_base( node, 0, XLP_PCIE_KBP_DEV, func );
    nlm_hal_write_32bit_reg( mmio, reg_addr, reg_data );
} /* end nlm_hal_ila_reg_write */

/**read from 8-bit SerDes register for ILA
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param lane - SerDes lane index
 @param reg_addr - register address
 @return 8-bit register data
 */
static __inline__ uint8_t nlm_hal_ila_serdes_reg_read( uint32_t node, uint32_t lane, uint8_t reg_addr ){
    uint32_t serdes_select = 0x1 << lane;
    uint32_t serdes_reg_data_wr = 0x00;

    serdes_reg_data_wr = ( serdes_select << 16 ) | ( (uint32_t)reg_addr << 8 ) | 0x00;
    nlm_hal_ila_reg_write( node, KBP_START_LANE, SER_REG0, serdes_reg_data_wr );
    return nlm_hal_ila_reg_read( node, KBP_START_LANE, SER_REGALL );

} /* end nlm_hal_ila_serdes_reg_read */

/**write to 8-bit SerDes register for ILA
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param lane - SerDes lane index
 @param reg_addr - register address
 @param reg_data - register data
 @return None
 */
static __inline__ void nlm_hal_ila_serdes_reg_write( uint32_t node, uint32_t lane, uint8_t reg_addr, uint8_t reg_data ){
    uint32_t serdes_select = 0x1 << lane;
    uint32_t serdes_reg_data_wr = 0x00;

    serdes_reg_data_wr = ( serdes_select << 16 ) | ( (uint32_t)reg_addr << 8 ) | (uint32_t)reg_data;
    nlm_hal_ila_reg_write( node, KBP_START_LANE, SER_REG0, serdes_reg_data_wr );
    nlm_hal_ila_reg_write( node, KBP_START_LANE, SER_REGALL, serdes_reg_data_wr );

} /* end nlm_hal_ila_serdes_reg_write */

/**perform read on list of address-data pair of 8-bit SerDes registers for given number of SerDes lanes starting from lane 0 for ILA
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param number_of_lanes - number of lanes to write register
 @param number_of_addr - number of register address per SerDes lane
 @param reg_addr - array of list of register address
 @param reg_data - array to store read data
 @return None
 * e.x. nlm_hal_ila_serdes_lanes_reg_read( 0, 4, 2, [0x64, 0x65], [0, 0, 0, 0, 0, 0, 0, 0] );
 *      read SerDes register 0x64 and 0x65 for SerDes lane 0, 1, 2 and 3
 *      reg_data = [ 0x64 data of lane 0, 0x64 data of lane 1, 0x64 data of lane 2, 0x64 data of lane 3,
 *                   0x65 data of lane 0, 0x65 data of lane 1, 0x65 data of lane 2, 0x65 data of lane 3 ]
 */
static __inline__ void nlm_hal_ila_serdes_lanes_reg_read( uint32_t node, uint32_t number_of_lanes, uint32_t number_of_addr, uint8_t* reg_addr, uint8_t* reg_data ){
    uint32_t serdes_counter = 0x00;
    uint32_t reg_counter = 0x00;

    for( reg_counter = 0x0; reg_counter < number_of_addr ; reg_counter++ ){
        for( serdes_counter = 0; serdes_counter < number_of_lanes ; serdes_counter++ ){
            reg_data[reg_counter * number_of_lanes + serdes_counter] = nlm_hal_ila_serdes_reg_read( node, serdes_counter, reg_addr[reg_counter] );
        } /* end for */
    } /* end for */

} /* end nlm_hal_ila_serdes_lanes_reg_read */

/**perform write on list of address-data pair of 8-bit SerDes registers for given number of SerDes lanes starting from lane 0 for ILA
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param number_of_lanes - number of lanes to write register
 @param number_of_addr - number of register address per SerDes lane
 @param reg_addr - array of list of register address
 @param reg_data - array of list of register data
 @return None
 * e.x. nlm_hal_ila_serdes_lanes_reg_write( 0, 4, 2, [0x23,0x64], [0xBE,0xEF] )
 *      SerDes lane 0 register 0x23 data becomes 0xBE
 *      SerDes lane 0 register 0x64 data becomes 0xEF
 *      SerDes lane 1 register 0x23 data becomes 0xBE
 *      SerDes lane 1 register 0x64 data becomes 0xEF
 *      SerDes lane 2 register 0x23 data becomes 0xBE
 *      SerDes lane 2 register 0x64 data becomes 0xEF
 *      SerDes lane 3 register 0x23 data becomes 0xBE
 *      SerDes lane 3 register 0x64 data becomes 0xEF
 */
static __inline__ void nlm_hal_ila_serdes_lanes_reg_write( uint32_t node, uint32_t number_of_lanes, uint32_t number_of_addr, uint8_t* reg_addr, uint8_t* reg_data ){
    uint32_t serdes_counter = 0x00;
    uint32_t reg_counter = 0x00;

    for( reg_counter = 0x0; reg_counter < number_of_addr ; reg_counter++ ){
        for( serdes_counter = 0; serdes_counter < number_of_lanes ; serdes_counter++ ){
            nlm_hal_ila_serdes_reg_write( node, serdes_counter, reg_addr[reg_counter], reg_data[reg_counter * number_of_lanes + serdes_counter] );
        } /* end for */
    } /* end for */

} /* end nlm_hal_ila_serdes_lanes_reg_write */

/**read from 16-bit cpld chip-select 0x2 registers on EVP board for KBP
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param reg_addr - register address
 @return 16-bit register data
 */
uint16_t nlm_hal_kbp_cpld_reg_read( uint32_t node, uint16_t addr ){
    volatile uint64_t mmio = nlm_hal_get_dev_base( node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_NOR );
    volatile uint64_t cpld_base = nlm_hal_read_32bit_reg( mmio, 0x42 );
    volatile uint16_t data = 0;

    if( cpld_base != DEFAULT_CPLD_MEM_BAR )
        nlm_hal_write_32bit_reg( mmio, 0x42, DEFAULT_CPLD_MEM_BAR );
    /* end if */

    data = nlm_hal_read_16bit_reg( ( cpld_base << 8 ), ( addr >> 1 ) );

    /* NetOS and HELinux needs byte swap */
#if defined(NLM_HAL_NETOS) | defined(NLM_HAL_LINUX_USER)
    data = ( data << 8 ) | ( data >> 8 );
#endif

    return data;

} /* end nlm_hal_kbp_cpld_reg_read */

/**write to 16-bit cpld chip-select 0x2 registers on EVP board for KBP
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param reg_addr - register address
 @param data - register data
 @return None
 */
void nlm_hal_kbp_cpld_reg_write( uint32_t node, uint16_t addr, uint16_t data ){
    volatile uint64_t mmio = nlm_hal_get_dev_base( node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_NOR );
    volatile uint64_t cpld_base = nlm_hal_read_32bit_reg( mmio, 0x42 );

    if( cpld_base != DEFAULT_CPLD_MEM_BAR )
        nlm_hal_write_32bit_reg( mmio, 0x42, DEFAULT_CPLD_MEM_BAR );
    /* end if */

    /* NetOS and HELinux needs byte swap */
#if defined(NLM_HAL_NETOS) | defined(NLM_HAL_LINUX_USER)
    data = ( data << 8 ) | ( data >> 8 );
#endif

    nlm_hal_write_16bit_reg( ( cpld_base << 8 ), ( addr >> 1 ), data );
} /* end nlm_hal_kbp_cpld_reg_write */

/**reset KBP MDIO interface
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @return None
 */
static __inline__ void nlm_hal_kbp_mdio_reset( uint32_t node ){
    uint32_t reg_data;

    reg_data = nlm_hal_read_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL ); /*0x25*/
    reg_data = 0x04000000;
    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, reg_data );
    reg_data = nlm_hal_read_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL );

} /* end nlm_hal_kbp_mdio_reset */

/**read from KBP MDIO register
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param regaddr - register address
 @param dev_id - MDIO device id
 @return register data
 */
static __inline__ uint32_t nlm_hal_kbp_mdio_read( uint32_t node, uint32_t regaddr, uint32_t dev_id ){

    uint32_t data = 0;
    volatile uint32_t status;

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL_DATA, regaddr );

    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 1, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    /* now the address has been written in indirectly*/
    /* Now get the load bit to be 0 ( bit 19) */
    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    data = MAKE_MDIO_CTRL(3, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    data = MAKE_MDIO_CTRL(3, 0x08, dev_id, 2, 5, 1, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    /* get the load bit to be 0 ( bit 19) */
    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 1, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    status = nlm_hal_read_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_RD_STAT ); /* 0x27*/
    return ( status & 0xFFFF );
} /* end nlm_hal_kbp_mdio_read */

/**write to KBP MDIO register
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param regaddr - register address
 @param indirect_data - register data
 @param dev_id - MDIO device id
 @return None
 */
static __inline__ void nlm_hal_kbp_mdio_write( uint32_t node, uint32_t regaddr, uint32_t indirect_data, uint32_t dev_id ){
    uint32_t data = 0;

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL_DATA, regaddr );
    /*0x26*/

    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );
    /*0x25*/

    WAIT_MDIO_BSY_CLEAR( node );

    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 1, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    /* now the address has been written in indirectly */
    /* next write in the data , but before that get the load bit to be 0 ( bit 19)*/
    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);
    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL_DATA, indirect_data );

    /* for write bits[3:2] is 01 */
    data = MAKE_MDIO_CTRL(1, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    data = MAKE_MDIO_CTRL(1, 0x08, dev_id, 2, 5, 1, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

    /* next write in the data , but before that get the load bit to be 0 ( bit 19) */
    data = MAKE_MDIO_CTRL(0, 0x08, dev_id, 2, 5, 0, 0, 0x7F, 0);

    nlm_hal_write_mac_reg( node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_CTRL, data );

    WAIT_MDIO_BSY_CLEAR( node );

} /* end nlm_hal_kbp_mdio_write */

/**perform KBP System-ON Reset Sequence
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param number_of_lane - number of SerDes lane to use
 @param serdes_speed_in_gbps - SerDes lane speed
 @param tx_metaframe_length - tx meta frame
 @param rx_metaframe_length - rx meta frame
 @return None
 * meta frame is for synchronization
 */
static __inline__ void nlm_hal_kbp_system_on_reset( uint32_t node, uint32_t number_of_lane, uint32_t serdes_speed_in_gbps, uint16_t tx_metaframe_length, uint16_t rx_metaframe_length ){
    volatile uint16_t cpld_data = 0;

    nlm_print( "Perform KBP System-ON Reset Sequence...\n" );

    /* 1. Ensure SRST_L & CRST_L are de-asserted (held high) before assert (pull low) SRST_L & CRST_L */
    cpld_data = nlm_hal_kbp_cpld_reg_read( node, 0x02 );
    /* de-assert SRST and CRST */
    cpld_data |= 0x0104;
    nlm_hal_kbp_cpld_reg_write( node, 0x02, cpld_data );
    /* wait for a while */
    KBP_DELAY( 50000 );

    cpld_data = nlm_hal_kbp_cpld_reg_read( node, 0x02 );
    /* assert low SRST and CRST */
    cpld_data &= 0xFEFB;
    nlm_hal_kbp_cpld_reg_write( node, 0x02, cpld_data );

    /* 2. Wait for 3.0us and de-assert SRST_L */
    /* wait for some long time to ensure sufficient wait time even when CPU runs at 1.6 GHz */
    KBP_DELAY( 50000 );

    cpld_data = nlm_hal_kbp_cpld_reg_read( node, 0x02 );
    /* de-assert SRST */
    cpld_data |= 0x0004;
    nlm_hal_kbp_cpld_reg_write( node, 0x02, cpld_data );

    /* 3. If MDIO-based device configuration is required, follow the steps as outlined in Section 6.7 */
    uint16_t reg_data = 0;

    /*    Device Configuration using MDIO */
    nlm_hal_kbp_mdio_reset( node );

    /*    1. Squelch the Transmit SerDes using "TX_SerDes_11_0_squelch", "CTX_SerDes_15_0_squelch", and "CTX_SerDes_23_16_squelch" registers. See Table 18 on page 39 */
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_TX_SER_0_11_SQCH, 0x0FFF, MDIO_DEV_CSM );

    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_CTX_SER_0_15_SQCH, 0xFFFF, MDIO_DEV_CSM );

    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_CTX_SER_16_23_SQCH, 0x00FF, MDIO_DEV_CSM );

    /*    2. Disable RX and TX PCS using "Global RX / TX Enable" register. See Table 18 on page 39 */
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_RX_TX_EN, 0x0000, MDIO_DEV_CSM );

    /*    3. Reset SerDes-Initialization state machine, Core PLL and Core Logic by writing 16'h7 to "Global SW Reset" register. See Table 18 on page 39 */
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_GL_SW_RST, 0x0007, MDIO_DEV_CSM );

    /*    4. Configure MDIO registers */
    KBP_DELAY( 100 );

    if( serdes_speed_in_gbps == 6 ){
        reg_data = nlm_hal_kbp_mdio_read( node, KBP_SPEED_SEL, MDIO_DEV_CSM );
        reg_data = ( reg_data & 0xFF88 ) | 0x44;
        nlm_hal_kbp_mdio_write( node, KBP_SPEED_SEL, reg_data, MDIO_DEV_CSM );
    }else{
        reg_data = nlm_hal_kbp_mdio_read( node, KBP_SPEED_SEL, MDIO_DEV_CSM );
        reg_data = ( reg_data & 0xFF88 ) | 0x00;
        nlm_hal_kbp_mdio_write( node, KBP_SPEED_SEL, reg_data, MDIO_DEV_CSM );
        /* workaround for NL11000 RA01 KBP running 3.125 Gbps */
        uint16_t counter = 0;
        for( counter = 0; counter < 4 ; counter++ ){
            nlm_hal_kbp_mdio_write( node, 0x120, 0x0043, MDIO_DEV_RX_BCAST );
            nlm_hal_kbp_mdio_write( node, 0x130, 0x0043, MDIO_DEV_RX_BCAST );
            nlm_hal_kbp_mdio_write( node, 0x140, 0x0043, MDIO_DEV_RX_BCAST );
            nlm_hal_kbp_mdio_write( node, 0x150, 0x0043, MDIO_DEV_RX_BCAST );
            nlm_hal_kbp_mdio_write( node, 0x120, 0x0041, MDIO_DEV_RX_BCAST );
            nlm_hal_kbp_mdio_write( node, 0x130, 0x0041, MDIO_DEV_RX_BCAST );
            nlm_hal_kbp_mdio_write( node, 0x140, 0x0041, MDIO_DEV_RX_BCAST );
            nlm_hal_kbp_mdio_write( node, 0x150, 0x0041, MDIO_DEV_RX_BCAST );
        } /* end for */
/*        // workaround for NL11000 RA12 KBP running 3.125 Gbps */
/*        nlm_hal_kbp_mdio_write( node, 0x0100, 0x0100, MDIO_DEV_RX_BCAST ); */
/*        nlm_hal_kbp_mdio_write( node, 0x0101, 0x0100, MDIO_DEV_RX_BCAST ); */
/*        nlm_hal_kbp_mdio_write( node, 0x0102, 0x0100, MDIO_DEV_RX_BCAST ); */
/*        nlm_hal_kbp_mdio_write( node, 0x0103, 0x0100, MDIO_DEV_RX_BCAST ); */
/*        nlm_hal_kbp_mdio_write( node, 0x0160, 0x0080, MDIO_DEV_RX_BCAST ); */
/*        nlm_hal_kbp_mdio_write( node, 0x0160, 0x0000, MDIO_DEV_RX_BCAST ); */
    } /* end if & else */

    /* workaround for NL11000 RA01 KBP START */
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, 0x0502, 0x0F68, MDIO_DEV_RX_BCAST );
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, 0x0502, 0x0F68, MDIO_DEV_TX_BCAST );
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, 0x8121, 0x7C53, MDIO_DEV_CSM );
    /* workaround for NL11000 RA01 KBP END */

    KBP_DELAY( 100 );
    /* TX Metaframe length */
    nlm_hal_kbp_mdio_write( node, KBP_TX_FRM_LEN, tx_metaframe_length, MDIO_DEV_CSM );

    KBP_DELAY( 100 );
    /* RX Metaframe length */
    nlm_hal_kbp_mdio_write( node, KBP_RX_FRM_LEN, rx_metaframe_length, MDIO_DEV_CSM );

    /*    5. Deassert Core PLL and Core Logic reset, and trigger the SerDes-Initialization state machine by writing 16'h0 to "Global SW Reset" register. See Table 18 on page 39 */
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_GL_SW_RST, 0x0000, MDIO_DEV_CSM );

    /*    6. Poll for assertion of bit[3], SerDes Reset Sequence Done, in the "General Purpose Status Register 3". See Table 19 on page 45 */nlm_print( "    Polling for SerDes Reset Sequence Done...\n" );
    KBP_DELAY( 100 );
    do{
        reg_data = nlm_hal_kbp_mdio_read( node, KBP_GEN_STATUS2, MDIO_DEV_CSM );
        reg_data = ( reg_data >> 3 ) & 0b1;
    }while( reg_data == 0 ); /* end do & while */
    nlm_print( "    SerDes Reset Sequence is Done\n" );

    /*    7. Enable RX and TX PCS using Global RX/TX Enable register. See Table 18 on page 39.*/
    if( number_of_lane <= 16 ){
        KBP_DELAY( 100 );
        nlm_hal_kbp_mdio_write( node, KBP_RX_LANE_0_15_EN, ( 0x1 << number_of_lane ) - 1, MDIO_DEV_CSM );
        KBP_DELAY( 100 );
        nlm_hal_kbp_mdio_write( node, KBP_RX_LANE_16_23_EN, 0x0000, MDIO_DEV_CSM );
    }else if( number_of_lane > 16 && number_of_lane <= 24 ){
        KBP_DELAY( 100 );
        nlm_hal_kbp_mdio_write( node, KBP_RX_LANE_0_15_EN, 0xFFFF, MDIO_DEV_CSM );
        KBP_DELAY( 100 );
        nlm_hal_kbp_mdio_write( node, KBP_RX_LANE_16_23_EN, ( 0x1 << ( number_of_lane - 16 ) ) - 1, MDIO_DEV_CSM );
    }else{
        nlm_print( "    ERROR!!!!!!!!!!!!! KBP Doesn't support %u SerDes lanes for requests!!!!!!!!!!!!!\n", number_of_lane );
    } /* end if & else */

    KBP_DELAY( 100 );
    if( number_of_lane <= 12 ){
        nlm_hal_kbp_mdio_write( node, KBP_TX_LANE_0_11_EN, ( 0x1 << number_of_lane ) - 1, MDIO_DEV_CSM );
    }else{
        nlm_print( "    ERROR!!!!!!!!!!!!! KBP Doesn't support %u SerDes lanes for responses!!!!!!!!!!!!!\n", number_of_lane );
    } /* end if & else */

    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_RX_TX_EN, 0x0003, MDIO_DEV_CSM ); /* Global Enable for Tx & Rx PCS*/

    /*    8. Disable squelch for the Transmit SerDes using TX_SerDes_11_0_squelch CTX_SerDes_15_0_squelch and CTX_SerDes_23_16_squelch registers. See Table 18 on page 39.*/
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_TX_SER_0_11_SQCH, 0x0FFF - ( ( 0x1 << number_of_lane ) - 1 ), MDIO_DEV_CSM );

    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_CTX_SER_0_15_SQCH, 0xFFFF, MDIO_DEV_CSM );

    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_CTX_SER_16_23_SQCH, 0x00FF, MDIO_DEV_CSM );

    /*    9. Read the General Purpose Status Register 3, 0x8184 to release assertion of GIO_L that may happen due to errors on the lanes during initialization.*/
    nlm_hal_kbp_mdio_read( node, KBP_GEN_STATUS3, MDIO_DEV_CSM );

    /* 4. Wait for 2.5ms and de-assert CRST_L */
    KBP_DELAY( 5000000 );

    cpld_data = nlm_hal_kbp_cpld_reg_read( node, 0x02 );
    /* de-assert CRST */
    cpld_data |= 0x0104;
    nlm_hal_kbp_cpld_reg_write( node, 0x02, cpld_data );

    /* 5. Interlaken Look-Aside framing and synchronization.
     *       Idle control words will be sent on the response port with XOFF until synchronization is complete.
     *       It is expected that only idle control words are sent on the RX port until XON is provided on the TX port.*/
    KBP_DELAY( 5000000 );

    /* 6. Assert then de-assert reset Rx satellete sticky register */
    KBP_DELAY( 100 );
    nlm_hal_kbp_mdio_write( node, KBP_SER_SW_RST, 0x0040, MDIO_DEV_CSM );

    KBP_DELAY( 2500000 );

    nlm_hal_kbp_mdio_write( node, KBP_SER_SW_RST, 0x0000, MDIO_DEV_CSM );

    nlm_print( "KBP System-ON Reset Sequence is Done\n" );

} /* end nlm_hal_kbp_system_on_reset */

/**perform ILA Reset Sequence
 @param node - ICI node. single node system should use 0. node other than 0 is not supported yet
 @param number_of_lane - number of SerDes lane to use
 @param serdes_speed_in_gbps - SerDes lane speed
 @param tx_metaframe_length - tx meta frame
 @param rx_metaframe_length - rx meta frame
 @return None
 * meta frame is for synchronization
 */
static __inline__ void nlm_hal_ila_reset( uint32_t node, uint32_t number_of_lane, uint32_t serdes_speed_in_gbps, uint16_t tx_metaframe_length, uint16_t rx_metaframe_length ){
    uint32_t reg_data = 0;

    nlm_print( "Perform ILA Reset Sequence...\n" );

    if( serdes_speed_in_gbps == 6 ){
        /* do nothing */
    }else{
        /* Configure SerDes to run at 3.125 Gbps */
        uint8_t addr = 0x04;
        uint8_t data[MAX_NUMBER_OF_SERDES_LANE] = { 0x0 };

        memset( data, 0x30, MAX_NUMBER_OF_SERDES_LANE );
        nlm_hal_ila_serdes_lanes_reg_write( node, number_of_lane, 1, &addr, data );

        /* Writing to reg05 to change M value to 2 */
        addr = 0x05;
        memset( data, 0x33, MAX_NUMBER_OF_SERDES_LANE );
        nlm_hal_ila_serdes_lanes_reg_write( node, number_of_lane, 1, &addr, data );
    } /* end if */

    /* Assert active low PLL reset */
    reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, KBP_CTL );
    reg_data &= 0xFFFFF7FF;
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_CTL, reg_data );

    KBP_DELAY( 1000 );

    /* De-assert active low PLL reset */
    reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, KBP_CTL );
    reg_data |= 0x00000800;
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_CTL, reg_data );

    /* Mask all interrupts */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_INT_MASK, 0x3FF );

    /* Configure number of active lanes by writing index of last active lane */
    reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, LA_BAD_LANE );
    reg_data &= 0xFFFFFF0F;
    reg_data &= 0xFF0FFFFF;
    if( number_of_lane == 2 ){
        /* TX lane */
        reg_data &= 0xFFFFFFF7;
        reg_data |= ( 0x1 ) << 7;
        reg_data |= ( 0x2 ) << 4;
        reg_data |= ( 0x2 ) << 0;
        /* RX lane */
        reg_data &= 0xFFF7FFFF;
        reg_data |= ( 0x1 ) << 23;
        reg_data |= ( 0x2 ) << 20;
        reg_data |= ( 0x2 ) << 16;
    }else if( number_of_lane >= 3 && number_of_lane <= 8 ){
        /* TX lane */
        reg_data |= ( 0x0 ) << 7;
        reg_data |= ( number_of_lane - 1 ) << 4;
        /* RX lane */
        reg_data |= ( 0x0 ) << 24;
        reg_data |= ( number_of_lane - 1 ) << 20;
    }else{
        nlm_print( "    ERROR!!!!!!!!!!!!! ILA Doesn't support %u SerDes lanes!!!!!!!!!!!!!\n", number_of_lane );
    } /* end if & else if & else */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, LA_BAD_LANE, reg_data );

    /* set timeout time tick */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_REQ_ID, 0x00000400 );

    /* set metaframe */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, LA_LEN, ( ( rx_metaframe_length - 1 ) << 16 ) | ( tx_metaframe_length - 1 ) );

    /* set LSB position of Context Buffer ID to [17], so the valid Context Buffer ID length is 12-bit in an ILA request at descriptor0 bit [28:17] */
    /* set Zero Pad Length of Context Buffer ID to [0] */
    /* these 12 bits correspond to context buffer address [13:2] at KBP because ILA automatically masks out LSBs to zeros when sending request to KBP */
    reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, KBP_CTL );
    reg_data = ( reg_data & 0xFFFFFFF0 ) | 0x1;
    reg_data = ( reg_data & 0xFFFE0FFF ) | ( 0x11 << 12 );
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_CTL, reg_data );

    /* Enable packet counter and counter interrupts */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_PKT_CNT_CTL, 0xC000C000 );

    /* wait until PCS lanes are ready */
    nlm_print( "    Polling for PCS Lanes ready...\n" );
    do{
        KBP_DELAY( 10000 );
        reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, SER_CTL );
        reg_data &= 0xFF;
    }while( reg_data != 0xFF ); /* end do & while */
    nlm_print( "    All PCS Lanes are ready\n" );

    /* Reset RX SerDes clocks, TX SerDes, RX SerDes and TX */
    KBP_DELAY( 10000 );
    reg_data = ( 0xFF << 24 ) | ( 0x1 << 22 ) | ( 0x1 << 21 ) | ( 0x1 << 20 );
    nlm_hal_ila_reg_write( node, KBP_START_LANE, LA_RX_SYNC, reg_data );

    /* Wait for all RX lanes to be aligned/de-skewed */
    nlm_print( "    Polling for Word Boundary and Scrambler Lock...\n" );
    nlm_print( "        If Word Boundary and Scrambler Lock does not come up, check Meta Frame, Enabled Number of Lanes and SerDes Speed setting\n" );
    do{
        reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, LA_RX_SYNC );
        reg_data = ( reg_data >> 18 ) & 0b1;
    }while( reg_data != 1 ); /* end do & while */
    nlm_print( "    Word Boundary and Scrambler are Locked\n" );

    /* Enable TX */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, LA_TX_ENABLE, 0x1 );

    /* Enable capture of 3'bxxx except 3'b000 in pick1 counter and 3'bx1x in pick0 counter on error status bits [31:29] of KBP responses */
    reg_data = ( 0xFE << 16 ) | ( 0xCC << 8 );
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_RSP_ERROR_PAR, reg_data );

    /* Set TX short burst length to 16 bytes */
    reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, LA_SETUP );
    reg_data &= 0xFFFFF8FF;
    reg_data |= ( 0x1 << 8 );
    nlm_hal_ila_reg_write( node, KBP_START_LANE, LA_SETUP, reg_data );

    /* Change LA MEM Usage Register Threshold */
    reg_data = nlm_hal_ila_reg_read( node, KBP_START_LANE, LA_MEM_USAGE );
    reg_data &= 0xFFFFFF00;
    if( is_nlm_xlp8xx_ax() ){
        reg_data |= 0x3A;
    }else if( is_nlm_xlp8xx_bx() ){
        reg_data |= 0xFF;
    } /* end if & else if */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, LA_MEM_USAGE, reg_data );

    /* Set up counters to capture errors flagged at bit [31:29] in KBP response packets control word */
    /* 0xFE : increase counter when [31:29] = one of {3'b111, 3'b110, 3'b101, 3'b100, 3'b011, 3'b010, 3'b001} */
    /* 0xCC : increase counter when [31:29] = one of {3'b111, 3'b110, 3'b011, 3'b010} */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_RSP_ERROR_PAR, 0x00FECC00 );

    /* Set Tick Scale for timeout when waiting for response from KBP */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_REQ_ID, 0x00000400 );

    /* check for KBP link status */
    nlm_print( "    Checking for KBP status on [RX PCS Ready], [Core Init Done], [SerDes Reset Sequence Done] after ILA Reset\n" );
    reg_data = nlm_hal_kbp_mdio_read( node, KBP_GEN_STATUS2, MDIO_DEV_CSM );
    if( ( reg_data & 0xE ) == 0xE ){
        nlm_print( "    KBP is ready after ILA Reset\n" );
    }else{
        nlm_print( "    ERROR!!!!!!!!!!!!! KBP is not ready after ILA Reset!!!!!!!!!!!!!\n" );
        nlm_print( "ILA Reset Sequence is NOT Done\n" );
        return;
    } /* end if & else */

    /* clear interrupts after initialization */
    nlm_hal_ila_reg_write( node, KBP_START_LANE, KBP_INT, 0xFFFFFFFF );
    nlm_hal_ila_reg_write( node, KBP_START_LANE, LA_INT, 0xFFFFFFFF );

    nlm_print( "ILA Reset Sequence is Done\n" );

} /* end nlm_hal_ila_reset */

#endif /*__ASSEMBLY_*/
#endif /* _NLM_HAL_KBP_H_ */
