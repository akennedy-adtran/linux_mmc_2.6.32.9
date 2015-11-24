/*-
 * Copyright (c) 2003-2013 Broadcom Corporation
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

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/netdevice.h>
#endif

#include "nlm_hal_fmn.h"
#include "nlm_hal_nae.h"
#include "nlm_hal_sys.h"
#include "libfdt.h"
#include "fdt_helper.h"
#include "nlm_evp_cpld.h"

void nae_ext_mdio_wait(int n)
{
	volatile int s=0, i,j;
	unsigned long long freq = nlm_hal_cpu_freq();
	freq >>= 17;
	if ( is_nlm_xlp3xx_ax() || is_nlm_xlp8xx_ax() || is_nlm_xlp8xx_b0() )
	   return;
	for(j=0; j<n; j++)
	for(i=0; i<freq; i++) s++;
	return;
}

/*
 * MDIO CLK = NAE freq (not 2x freq) divided by both of the dividers below:
 *----------------------
 * EXT_G0_MDIO_CTRL[4:2]
 *----------------------
 *  0	 4
 *  1	 4
 *  2	 6
 *  3	 8
 *  4	10
 *  5	14
 *  6	20
 *  7	54
 *----------------------
 * EXT_G0_MDIO_CTRL[1:0] - old chips / mid-life chips / new chips (Firefly B0)
 *----------------------
 * 0	 1		 64		  8
 * 1	 2		128		 32
 * 2	 4		256		128
 * 3	 8		512		512
 */
static uint32_t nae_get_EXT_G_MDIO_DIV(void)
{
	/* Older chips without a /64 post divider - EXT_G_MDIO_DIV = 0x1E */
	if (is_nlm_xlp3xx_ax() || is_nlm_xlp8xx_ax() || is_nlm_xlp8xx_b0() )
		return EXT_G_MDIO_DIV;

#ifndef NLM_HAL_XLP1
	/* Mid-life chips - EXT_G_MDIO_DIV_WITH_HW_DIV64_11 = 0x11 */
	if (is_nlm_xlp3xx() || is_nlm_xlp8xx() || is_nlm_xlp2xx_ax() )
		return EXT_G_MDIO_DIV_WITH_HW_DIV64_11;

	/* Should leave only Firefly B0.  The first divider has changed from 64/128/256/512 to
	 * 8/32/128/512 and default frequencies have doubled on some speed grades. Thus
	 * test NAE clock and decide.  Target is 195KHz from 250MHz / 500 MHz NAE clock. */
	if (nlm_hal_xlp2xx_get_clkdev_frq(XLP2XX_CLKDEVICE_NAE) <= 375000000)
		return EXT_G_MDIO_DIV_2XX_SLOW;
	else
		return EXT_G_MDIO_DIV_2XX_FAST;
#else
	return EXT_G_MDIO_DIV_WITH_HW_DIV64_11;
#endif
}

/* INT_MDIO_CTRL, block7, 0x799
 * EXT_XG_MDIO_CTRL, block7, 0x7A5,0x7A9
 * 29:28: MCDiv Master Clock Divider
 * 0 1
 * 1 2
 * 2 4
 * 3 8
 * 27:21: XDiv Clock Divisor
 *   M(mdc) = F(mstclk)/(2*(XDiv+1))
 *    500MHz/4 = 125MHz
 *    125MHz/(2*(0x7F+1)) = 0.5MHz
 *
 *    250MHz/4 = 62.5MHz
 *    62.5MHz/(2*(0x7F+1)) = 0.24MHz
 */

static uint32_t nae_get_EXT_XG_MDIO_DIV(void)
{
	return ((0x7F << EXT_XG_MDIO_CTRL_XDIV_POS) | (2 << EXT_XG_MDIO_CTRL_MCDIV_POS));
}

/* INT_MDIO_CTRL, block7, 0x799
 * EXT_XG_MDIO_CTRL, block7, 0x7A5,0x7A9
 * 29:28: MCDiv Master Clock Divider
 * 0 1
 * 1 2
 * 2 4
 * 3 8
 * 27:21: XDiv Clock Divisor
 *   M(mdc) = F(mstclk)/(2*(XDiv+1))
 *    500MHz/4 = 125MHz
 *    125MHz/(2*(0x7F+1)) = 0.5MHz
 *
 *    250MHz/4 = 62.5MHz
 *    62.5MHz/(2*(0x7F+1)) = 0.24MHz
 */
static uint32_t nae_get_INT_MDIO_DIV(void)
{
	return ((0x7F << INT_MDIO_CTRL_XDIV_POS) | (2 << INT_MDIO_CTRL_MCDIV_POS));
}

#define PHY_STATUS_RETRIES 20000

#define NUM_EGRESS_PORTS 18
#define TX_IF_BURST_MAX  2048
#define DRR_QUANTA       2048
#define SP_EN            0
#define SP_NUM           0

#define WAIT_XGMAC_MDIO_BSY_CLEAR(node)  \
	for (i = 0; i < PHY_STATUS_RETRIES; i++) {	\
		if((nlm_hal_read_mac_reg(node, BLOCK_7, LANE_CFG,	\
			EXT_XG0_MDIO_RD_STAT + bus * 4 ) & EXT_XG_MDIO_STAT_MBSY) == 0)	\
                        break;							\
        }

#define WAIT_XGMAC_IMDIO_BSY_CLEAR(node)   \
	for (i = 0; i < PHY_STATUS_RETRIES; i++) { \
		if((nlm_hal_read_mac_reg(node, BLOCK_7, LANE_CFG, \
			INT_MDIO_RD_STAT) & INT_MDIO_STAT_MBSY) == 0)        \
                        break;                                                  \
        }

/*
 *      MDIO Support
 */
/* Internal MDIO READ/WRITE Routines
 */
/**
* @brief nae_int_gmac_mdio_read function is used to read an SGMII PCS register.
* 
* @param [in] node Node number
* @param [in] bus Internal MDIO bus number
* @param [in] phyaddr Internal PHY's address
* @param [in] regidx MDIO register index
*
* @return
* 	- value of MDIO register
* 
* @ingroup hal_nae
*
*/
static int nae_int_gmac_mdio_read(int node, int bus, int phyaddr, int regidx)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
	uint32_t mdio_ld_cmd = nlm_hal_read_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4);

	if (mdio_ld_cmd & INT_MDIO_CTRL_CMD_LOAD) {
		nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
				       (mdio_ld_cmd & ~INT_MDIO_CTRL_CMD_LOAD));
	}

	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
			       INT_MDIO_CTRL_SMP
			       | (phyaddr << INT_MDIO_CTRL_PHYADDR_POS)
			       | (regidx << INT_MDIO_CTRL_DEVTYPE_POS)
			       | (2 << INT_MDIO_CTRL_OP_POS)
			       | (1 << INT_MDIO_CTRL_ST_POS)
			       | (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS)
			       | (2 << INT_MDIO_CTRL_TA_POS)
			       | (2 << INT_MDIO_CTRL_MIIM_POS)
			       | (0 << INT_MDIO_CTRL_LOAD_POS)
			       | (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));

	/* Toggle Load Cmd Bit */
	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
			       INT_MDIO_CTRL_SMP
			       | (phyaddr << INT_MDIO_CTRL_PHYADDR_POS)
			       | (regidx << INT_MDIO_CTRL_DEVTYPE_POS)
			       | (2 << INT_MDIO_CTRL_OP_POS)
			       | (1 << INT_MDIO_CTRL_ST_POS)
			       | (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS)
			       | (2 << INT_MDIO_CTRL_TA_POS)
			       | (2 << INT_MDIO_CTRL_MIIM_POS)
			       | (1 << INT_MDIO_CTRL_LOAD_POS) /* */
			       | (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));

	/* poll master busy bit until it is not busy
	 */
	while(nlm_hal_read_mac_reg(node, block, intf_type,
				    INT_MDIO_RD_STAT + bus * 4) & INT_MDIO_STAT_MBSY);

	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
			       INT_MDIO_CTRL_SMP
			       | (phyaddr << INT_MDIO_CTRL_PHYADDR_POS)
			       | (regidx << INT_MDIO_CTRL_DEVTYPE_POS)
			       | (2 << INT_MDIO_CTRL_OP_POS)
			       | (1 << INT_MDIO_CTRL_ST_POS)
			       | (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS)
			       | (2 << INT_MDIO_CTRL_TA_POS)
			       | (2 << INT_MDIO_CTRL_MIIM_POS)
			       | (0 << INT_MDIO_CTRL_LOAD_POS)
			       | (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));

	/* Read the data back
	 */
	return (nlm_hal_read_mac_reg(node, block, intf_type, INT_MDIO_RD_STAT + bus * 4) & 0xffff);
}

/* Internal MDIO WRITE Routines
 */
/**
* @brief nae_int_gmac_mdio_write function is used to write an SGMII PCS register.
*
* @param [in] node Node number
* @param [in] bus Internal MDIO bus number
* @param [in] phyaddr Internal PHY's address
* @param [in] regidx MDIO register index
* @param [in] val Value to write
*
* @return
* 	- 0 on success
* 
* @ingroup hal_nae
*
*/
static int nae_int_gmac_mdio_write(int node, int bus, int phyaddr, int regidx, uint16_t val)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
	uint32_t mdio_ld_cmd = nlm_hal_read_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4);

	if (mdio_ld_cmd & INT_MDIO_CTRL_CMD_LOAD) {
		nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
				       (mdio_ld_cmd & ~INT_MDIO_CTRL_CMD_LOAD));
	}

	/* load data into ctrl data reg
	 */
	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL_DATA + bus * 4, val);

	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
			       INT_MDIO_CTRL_SMP
			       | (phyaddr << INT_MDIO_CTRL_PHYADDR_POS)
			       | (regidx << INT_MDIO_CTRL_DEVTYPE_POS)
			       | (1 << INT_MDIO_CTRL_OP_POS)
			       | (1 << INT_MDIO_CTRL_ST_POS)
			       | (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS)
			       | (2 << INT_MDIO_CTRL_TA_POS)
			       | (1 << INT_MDIO_CTRL_MIIM_POS)
			       | (0 << INT_MDIO_CTRL_LOAD_POS)
			       | (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));

	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
			       INT_MDIO_CTRL_SMP
			       | (phyaddr << INT_MDIO_CTRL_PHYADDR_POS)
			       | (regidx << INT_MDIO_CTRL_DEVTYPE_POS)
			       | (1 << INT_MDIO_CTRL_OP_POS)
			       | (1 << INT_MDIO_CTRL_ST_POS)
			       | (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS)
			       | (2 << INT_MDIO_CTRL_TA_POS)
			       | (1 << INT_MDIO_CTRL_MIIM_POS)
			       | (1 << INT_MDIO_CTRL_LOAD_POS)
			       | (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));

	/* poll master busy bit until it is not busy
	 */
	while(nlm_hal_read_mac_reg(node, block, intf_type,
				    INT_MDIO_RD_STAT + bus * 4) & INT_MDIO_STAT_MBSY);

	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
			       INT_MDIO_CTRL_SMP
			       | (phyaddr << INT_MDIO_CTRL_PHYADDR_POS)
			       | (regidx << INT_MDIO_CTRL_DEVTYPE_POS)
			       | (1 << INT_MDIO_CTRL_OP_POS)
			       | (1 << INT_MDIO_CTRL_ST_POS)
			       | (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS)
			       | (2 << INT_MDIO_CTRL_TA_POS)
			       | (1 << INT_MDIO_CTRL_MIIM_POS)
			       | (0 << INT_MDIO_CTRL_LOAD_POS)
			       | (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));

	return 0;
}

/**
* @brief internal_nae_gmac_mdio_reset function is used to reset an internal MDIO controller.
*
* @param [in] node Node number
* @param [in] bus Internal MDIO bus number
*
* @return
* 	- 0 on success
* 
* @ingroup hal_nae
*
*/
static int internal_nae_gmac_mdio_reset(int node, int bus)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
			       INT_MDIO_CTRL_RST | INT_MDIO_CTRL_SMP |
			       (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS) 	|
			       (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));

	nlm_hal_write_mac_reg(node, block, intf_type, INT_MDIO_CTRL + bus * 4,
				INT_MDIO_CTRL_SMP | /* supress pre-amble */
			       (INT_MDIO_CTRL_XDIV << INT_MDIO_CTRL_XDIV_POS) 	|
			       (INT_MDIO_CTRL_MCDIV << INT_MDIO_CTRL_MCDIV_POS));
	return 0;
}

/**********************************************************************
 *  nae_gmac_mdio_read - Read sgmii phy register
 *
 *  Input parameters:
 *         bus          - bus number, nae has two external gmac bus: 0 and 1
 *         phyaddr      - PHY's address
 *         regidx       - index of register to read
 *
 *  Return value:
 *         value read (16 bits), or 0xffffffff if an error occurred.
 ********************************************************************* */
/**
* @brief nae_gmac_mdio_read function is used to read an SGMII PHY register.
*
* @param [in] node Node number
* @param [in] bus External MDIO bus number
* @param [in] phyaddr External PHY's address
* @param [in] regidx PHY register index to read
*
* @return
*	- value read (16 bits), or 0xffffffff if an error occurred.
* 
* @ingroup hal_nae
*
*/
static int nae_gmac_mdio_read(int node, int bus, int phyaddr, int regidx)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
	uint32_t clk_div   = nae_get_EXT_G_MDIO_DIV();
	uint32_t mdio_ld_cmd = nlm_hal_read_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4);

	if (mdio_ld_cmd & EXT_G_MDIO_CMD_LCD) {
		nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
				       (mdio_ld_cmd & ~EXT_G_MDIO_CMD_LCD));
		while(nlm_hal_read_mac_reg(node, block, intf_type,
					    EXT_G0_MDIO_RD_STAT + bus * 4) & EXT_G_MDIO_STAT_MBSY);
	}

	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
			       EXT_G_MDIO_CMD_SP
			       | (phyaddr << EXT_G_MDIO_PHYADDR_POS)
			       | (regidx << EXT_G_MDIO_REGADDR_POS)
			       | (0<<18) | clk_div);

	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
			       EXT_G_MDIO_CMD_SP
			       | (phyaddr << EXT_G_MDIO_PHYADDR_POS)
			       | (regidx << EXT_G_MDIO_REGADDR_POS)
			       | (1<<18) | clk_div);

	nae_ext_mdio_wait(1);

	while(nlm_hal_read_mac_reg(node, block, intf_type,
				    EXT_G0_MDIO_RD_STAT + bus * 4) & EXT_G_MDIO_STAT_MBSY);

	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
			       EXT_G_MDIO_CMD_SP
			       | (phyaddr << EXT_G_MDIO_PHYADDR_POS)
			       | (regidx << EXT_G_MDIO_REGADDR_POS)
			       | (0<<18) | clk_div);

	/* Read the data back */
	return (nlm_hal_read_mac_reg(node, block, intf_type, EXT_G0_MDIO_RD_STAT + bus * 4) & 0xffff);
}

/**********************************************************************
 *  nae_gmac_mdio_write -Write sgmac mii PHY register.
 *
 *  Input parameters:
 *         bus          - bus number, nae has two external gmac bus: 0 and 1
 *         phyaddr      - PHY to use
 *         regidx       - register within the PHY
 *         val          - data to write to register
 *
 *  Return value:
 *         0 - success
 ********************************************************************* */
/**
* @brief nae_gmac_mdio_write function is used to write an SGMII PHY register.
*
* @param [in] node Node number
* @param [in] bus External MDIO bus number
* @param [in] phyaddr External PHY's address
* @param [in] regidx PHY register index to read
* @param [in] val Value to write
*
* @return
* 	- 0 on success
* 
* @ingroup hal_nae
*
*/
static int nae_gmac_mdio_write(int node, int bus, int phyaddr, int regidx, uint16_t val)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
	uint32_t clk_div   = nae_get_EXT_G_MDIO_DIV();
	uint32_t mdio_ld_cmd = nlm_hal_read_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL+ bus * 4);

	if (mdio_ld_cmd & EXT_G_MDIO_CMD_LCD) {
		nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
				       (mdio_ld_cmd & ~EXT_G_MDIO_CMD_LCD));
		while(nlm_hal_read_mac_reg(node, block, intf_type,
					    EXT_G0_MDIO_RD_STAT + bus * 4) & EXT_G_MDIO_STAT_MBSY);
	}

	/* load data into ctrl data reg
	 */
	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL_DATA + bus * 4, val);

	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
			       EXT_G_MDIO_CMD_SP 	|
			       (phyaddr << EXT_G_MDIO_PHYADDR_POS) 	|
			       (regidx << EXT_G_MDIO_REGADDR_POS)	|
			       (0<<18) | clk_div);

	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
			       EXT_G_MDIO_CMD_LCD | EXT_G_MDIO_CMD_SP 	|
			       (phyaddr << EXT_G_MDIO_PHYADDR_POS) 	|
			       (regidx << EXT_G_MDIO_REGADDR_POS)	|
			       (0<<18) | clk_div);

	nae_ext_mdio_wait(1);

	while(nlm_hal_read_mac_reg(node, block, intf_type,
				    EXT_G0_MDIO_RD_STAT + bus * 4) & EXT_G_MDIO_STAT_MBSY);

	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
			       EXT_G_MDIO_CMD_SP 	|
			       (phyaddr << EXT_G_MDIO_PHYADDR_POS) 	|
			       (regidx << EXT_G_MDIO_REGADDR_POS)	|
			       (0<<18) | clk_div);

	return 0;
}

/**********************************************************************
 *  external_nae_gmac_mdio_reset -Reset sgmii mdio module.
 *
 *  Input parameters:
 *         bus - bus number, nae has two external gmac bus: 0 and 1
 *
 *  Return value:
 *        0 - success
 ********************************************************************* */
/**
* @brief external_nae_gmac_mdio_reset function is used to reset an external MDIO controller.
*
* @param [in] node Node number
* @param [in] bus External MDIO bus number
*
* @return
* 	- 0 on success
* 
* @ingroup hal_nae
*
*/
static int external_nae_gmac_mdio_reset(int node, int bus)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
	uint32_t clk_div   = nae_get_EXT_G_MDIO_DIV();

//	nlm_print("%s: Bus %d, clock divider = 0x%X\n", __func__, bus, clk_div);

	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4,
			EXT_G_MDIO_MMRST | clk_div);
	nlm_hal_write_mac_reg(node, block, intf_type, EXT_G0_MDIO_CTRL + bus * 4, clk_div);
	return 0;
}

/**********************************************************************
 *  external_nae_xgmac_mdio_reset -Reset sgmii mdio module.
 *
 *  Input parameters:
 *         bus - bus number, nae has two external gmac bus: 0 and 1
 *
 *  Return value:
 *        0 - success
 ********************************************************************* */
/**
* @brief external_nae_xgmac_mdio_reset function is used to reset an external MDIO controller.
*
* @param [in] node Node number
* @param [in] bus External MDIO bus number
*
* @return
* 	- 0 on success
*
* @ingroup hal_nae
*
*/
static int external_nae_xgmac_mdio_reset(int node, int bus)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
        nlm_hal_write_mac_reg( node, block, intf_type, EXT_XG0_MDIO_CTRL + (bus * 4),
				 nae_get_EXT_XG_MDIO_DIV()
                                | EXT_XG_MDIO_CTRL_RST );

        nlm_hal_write_mac_reg( node, block, intf_type, EXT_XG0_MDIO_CTRL + (bus * 4),
				 nae_get_EXT_XG_MDIO_DIV()
                                );
	return 0;
}

static int nlm_hal_xgmac_imdio_addr(int node, int phyaddr, int regidx)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
        int32_t i;
        /* internal xgmac pcs is always 5 */
        /* phyadd 0x13,0x14, 0x15, 0x16 */
        int dev_addr = 5;

        /* load  XGMC_MDIO_CTRL_DATA register with addr */
        nlm_hal_write_mac_reg( node, block, intf_type,
                                INT_MDIO_CTRL_DATA, regidx);

        nlm_hal_write_mac_reg( node, block, intf_type, INT_MDIO_CTRL,
                                 phyaddr << INT_MDIO_CTRL_PHYADDR_POS
                                | dev_addr << INT_MDIO_CTRL_DEVTYPE_POS
				| nae_get_INT_MDIO_DIV()
                                | INT_MDIO_CTRL_CMD_LOAD
                                | MDIO_MIIM_CMD_10G_MMD << INT_MDIO_CTRL_MIIM_POS
                                | INT_MDIO_CTRL_TA << INT_MDIO_CTRL_TA_POS
                                | MDIO_CTRL_OP_INDIRECT_ADDR << INT_MDIO_CTRL_OP_POS
                                | INT_MDIO_CTRL_ST);


        /* poll master busy bit until it is not busy */
        WAIT_XGMAC_IMDIO_BSY_CLEAR(node);

        nlm_hal_write_mac_reg( node, block, intf_type, INT_MDIO_CTRL,
                                 phyaddr << INT_MDIO_CTRL_PHYADDR_POS
                                | dev_addr << INT_MDIO_CTRL_DEVTYPE_POS
				| nae_get_INT_MDIO_DIV()
                                | MDIO_MIIM_CMD_IDLE << INT_MDIO_CTRL_MIIM_POS
                                | 0x0 << INT_MDIO_CTRL_TA_POS
                                | MDIO_CTRL_OP_INDIRECT_ADDR << INT_MDIO_CTRL_OP_POS
                                | INT_MDIO_CTRL_ST);


        /* poll master busy bit until it is not busy */
        WAIT_XGMAC_IMDIO_BSY_CLEAR(node)
        return 0;
}

static int nlm_hal_xgmac_imdio_write(int node, int phyaddr, int regidx, uint16_t val)
{

	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
        int32_t  i;
        /* internal xgmac pcs is always 5 */
        /* phyadd 0x13,0x14, 0x15, 0x16 */
        int dev_addr = 5;

         /* first is indirect address cycle */
        nlm_hal_xgmac_imdio_addr(node, phyaddr, regidx);

        nlm_hal_write_mac_reg( node, block , intf_type,
                                INT_MDIO_CTRL_DATA, val);

        nlm_hal_write_mac_reg( node, block, intf_type, INT_MDIO_CTRL,
                                 phyaddr << INT_MDIO_CTRL_PHYADDR_POS
                                | dev_addr << INT_MDIO_CTRL_DEVTYPE_POS
				| nae_get_INT_MDIO_DIV()
                                | INT_MDIO_CTRL_CMD_LOAD
                                | MDIO_MIIM_CMD_10G_MMD << INT_MDIO_CTRL_MIIM_POS
                                | INT_MDIO_CTRL_TA << INT_MDIO_CTRL_TA_POS
                                | MDIO_CTRL_OP_WRITE_10G_MMD << INT_MDIO_CTRL_OP_POS
                                | INT_MDIO_CTRL_ST);

        /* poll master busy bit until it is not busy */
        WAIT_XGMAC_IMDIO_BSY_CLEAR(node);

        nlm_hal_write_mac_reg( node, block, intf_type, INT_MDIO_CTRL,
                                 phyaddr << INT_MDIO_CTRL_PHYADDR_POS
                                | dev_addr << INT_MDIO_CTRL_DEVTYPE_POS
				| nae_get_INT_MDIO_DIV()
                                | MDIO_MIIM_CMD_IDLE << INT_MDIO_CTRL_MIIM_POS
                                | 0x0 << INT_MDIO_CTRL_TA_POS
                                | 0 << INT_MDIO_CTRL_OP_POS
                                | INT_MDIO_CTRL_ST);


        /* poll master busy bit until it is not busy */
        WAIT_XGMAC_IMDIO_BSY_CLEAR(node);
        return 0;

}
static int nlm_hal_xgmac_imdio_read(int node, int phyaddr, int regidx)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
        int32_t  i;
        int rdval;
        /* internal xgmac pcs is always 5 */
        /* phyadd 0x13,0x14, 0x15, 0x16 */
        int dev_addr = 5;

         /* first is indirect address cycle */
        nlm_hal_xgmac_imdio_addr(node, phyaddr, regidx);

        nlm_hal_write_mac_reg( node, block, intf_type, INT_MDIO_CTRL,
                                 phyaddr << INT_MDIO_CTRL_PHYADDR_POS
                                | dev_addr << INT_MDIO_CTRL_DEVTYPE_POS
				| nae_get_INT_MDIO_DIV()
                                | INT_MDIO_CTRL_CMD_LOAD
                                | MDIO_MIIM_CMD_10G_MMD << INT_MDIO_CTRL_MIIM_POS
                                | INT_MDIO_CTRL_TA << INT_MDIO_CTRL_TA_POS
                                | MDIO_CTRL_OP_READ_10G_MMD << INT_MDIO_CTRL_OP_POS
                                | INT_MDIO_CTRL_ST);

        /* poll master busy bit until it is not busy */
        WAIT_XGMAC_IMDIO_BSY_CLEAR(node);

        rdval =  nlm_hal_read_mac_reg(node, block, intf_type, INT_MDIO_RD_STAT) & 0xFFFF;

        nlm_hal_write_mac_reg( node, block, intf_type, INT_MDIO_CTRL,
                                 phyaddr << INT_MDIO_CTRL_PHYADDR_POS
                                | dev_addr << INT_MDIO_CTRL_DEVTYPE_POS
				| nae_get_INT_MDIO_DIV()
                                | MDIO_MIIM_CMD_IDLE << INT_MDIO_CTRL_MIIM_POS
                                | 0x0 << INT_MDIO_CTRL_TA_POS
                                | 0 << INT_MDIO_CTRL_OP_POS
                                | INT_MDIO_CTRL_ST);


        /* poll master busy bit until it is not busy */
        WAIT_XGMAC_IMDIO_BSY_CLEAR(node);

        return rdval;

}

/**
* @brief nlm_hal_mdio_read function is used to read a register through MDIO.
*
* @param [in] node Node number
* @param [in] type NLM_HAL_INT_MDIO or NLM_HAL_EXT_MDIO
* @param [in] bus MDIO bus number
* @param [in] [To be deleted] block NAE Register Memory Map Block
* @param [in] [To be deleted] LANE_CFG (only valid for block 7)
* @param [in] phyaddr PHY's address: devType=Bit[12:8]; phyAddr:=Bit[4:0]
* @param [in] regidx PHY register index to read
*
* @return
*	- value read (16 bits), or 0xffffffff if an error occurred.
*	- -1: Invalid type
* 
* @ingroup hal_nae
*
*/
int nlm_hal_mdio_wr(int node, int type, int bus, int phyaddr, int regidx, uint16_t val);
int nlm_hal_mdio_rd(int node, int type, int bus, int phyaddr, int regidx);

int nlm_hal_mdio_rd(int node, int type, int bus, int phyaddr, int regidx)
{
	if (type == NLM_HAL_INT_MDIO) {
		/* INT_MDIO_CTRL: 0x799 */
		return nae_int_gmac_mdio_read(node, bus, phyaddr&0x1F, regidx);
	} else if (type == NLM_HAL_INT_MDIO_C45) {
		/* INT_MDIO_CTRL: 0x799 */
		return nlm_hal_xgmac_imdio_read(node, phyaddr&0x1F, regidx);
	} else if (type == NLM_HAL_EXT_MDIO) {
	        /* 1GE MDIO EXT_G<0,1> : bus0:0x79D, bus1:0x7A1*/
		return nae_gmac_mdio_read(node, bus, phyaddr&0x1F, regidx);
	} else if (type == NLM_HAL_EXT_MDIO_C45) {
	        /* 10GE MDIO EXT_XG<0,1> : bus0:0x7A5, bus1:0x7A9*/
		return nlm_hal_xgmac_mdio_read(node, bus,
						(phyaddr&0x1F), (phyaddr>>8)&0x1F, regidx);
	} else {
		nlm_print("NAE_ERROR: Invalid type for MDIO read !!\n");
		return -1;
	}
}
int nlm_hal_mdio_read(int node, int type, int bus, int block, int intf_type,
		      int phyaddr, int regidx)
{
	return nlm_hal_mdio_rd(node, type, bus, phyaddr, regidx);
}

/**
* @brief nlm_hal_mdio_write function is used to write a register through MDIO.
*
* @param [in] node Node number
* @param [in] type NLM_HAL_INT_MDIO or NLM_HAL_EXT_MDIO
* @param [in] bus MDIO bus number
* @param [in] [To be deleted] block NAE Register Memory Map Block
* @param [in] [To be deleted] LANE_CFG (only valid for block 7)
* @param [in] phyaddr PHY's address: devType=Bit[12:8]; phyAddr:=Bit[4:0]
* @param [in] regidx PHY register index to read
* @param [in] val Value to write
*
* @return
*	- 0 on success
*	- -1: Invalid type
* 
* @ingroup hal_nae
*
*/
int nlm_hal_mdio_wr(int node, int type, int bus, int phyaddr, int regidx, uint16_t val)
{
	if (type == NLM_HAL_INT_MDIO) {
		/* INT_MDIO_CTRL: 0x799 */
		return nae_int_gmac_mdio_write(node, bus, phyaddr&0x1F, regidx, val);
	} else if (type == NLM_HAL_INT_MDIO_C45) {
		/* INT_MDIO_CTRL: 0x799 */
		return nlm_hal_xgmac_imdio_write(node, phyaddr&0x1F, regidx, val);
	} else if (type == NLM_HAL_EXT_MDIO) {
	        /* 1GE MDIO EXT_G<0,1> : bus0:0x79D, bus1:0x7A1*/
		return nae_gmac_mdio_write(node, bus, phyaddr&0x1F, regidx, val);
	} else if (type == NLM_HAL_EXT_MDIO_C45) {
	        /* 10GE MDIO EXT_XG<0,1> : bus0:0x7A5, bus1:0x7A9*/
		return nlm_hal_xgmac_mdio_write(node, bus,
						(phyaddr&0x1F), (phyaddr>>8)&0x1F, regidx, val);
	} else {
		nlm_print("NAE_ERROR: Invalid type for MDIO write !!\n");
		return -1;
	}
}
int nlm_hal_mdio_write(int node, int type, int bus, int block, int intf_type,
		       int phyaddr, int regidx, uint16_t val)
{
	return nlm_hal_mdio_wr(node, type, bus, phyaddr, regidx, val);
}

/**
* @brief nlm_hal_mdio_reset function is used to reset an MDIO controller.
*
* @param [in] node Node number
* @param [in] type NLM_HAL_INT_MDIO or NLM_HAL_EXT_MDIO
* @param [in] bus MDIO bus number
*
* @return
*	- 0 on success
*	- -1: Invalid type
* 
* @ingroup hal_nae
*
*/
int nlm_hal_mdio_reset(int node, int type, int bus)
{
	if ((type == NLM_HAL_INT_MDIO) || (type == NLM_HAL_INT_MDIO_C45)) {
		return internal_nae_gmac_mdio_reset(node, bus);
	} else if (type == NLM_HAL_EXT_MDIO) {
		return external_nae_gmac_mdio_reset(node, bus);
	} else if (type == NLM_HAL_EXT_MDIO_C45) {
		return external_nae_xgmac_mdio_reset(node, bus);
	} else {
		nlm_print("NAE_ERROR: Invalid type for MDIO reset !!\n");
		return -1;
	}
}

/* Moved nlm_hal_init_poe_ext_storage to mod/nlm_hal_nae.c where it belongs */

/*
 *  Interface support
 */

#define PHY_STATUS_RETRIES 20000

#define WAIT_XGMAC_MDIO_BSY_CLEAR(node)  \
	for (i = 0; i < PHY_STATUS_RETRIES; i++) {	\
		if((nlm_hal_read_mac_reg(node, BLOCK_7, LANE_CFG,	\
			EXT_XG0_MDIO_RD_STAT + bus * 4 ) & EXT_XG_MDIO_STAT_MBSY) == 0)	\
                        break;							\
        }

#define WAIT_XGMAC_IMDIO_BSY_CLEAR(node)   \
	for (i = 0; i < PHY_STATUS_RETRIES; i++) { \
		if((nlm_hal_read_mac_reg(node, BLOCK_7, LANE_CFG, \
			INT_MDIO_RD_STAT) & INT_MDIO_STAT_MBSY) == 0)        \
                        break;                                                  \
        }

/*
 *                   XAUI Support
 *
 */
int nlm_hal_xgmac_mdio_write(int node, int bus,
	int phyaddr, int dev_addr, int regidx, uint16_t val)
{
	uint32_t block     = BLOCK_7;
	uint32_t intf_type = LANE_CFG;
        int32_t  i;
	uint32_t ctrl =   (phyaddr << EXT_XG_MDIO_CTRL_PHYADDR_POS)
			| (dev_addr << EXT_XG_MDIO_CTRL_REG_POS)
			| (regidx << EXT_XG_MDIO_CTRL_OP_POS)
			/* load = 0 */
			| nae_get_EXT_XG_MDIO_DIV()
			| (EXT_XG_MDIO_CTRL_TA << EXT_XG_MDIO_CTRL_TA_POS)
			| (MDIO_MIIM_CMD_10G_MMD << EXT_XG_MDIO_CTRL_MIIM_POS);

	/*nlm_print("xw: bus:%d phy:%d d:%d %x.%x=%x\n", bus, phyaddr, dev_addr, ctrl, regidx, val); */
        nlm_hal_write_mac_reg( node, block, intf_type, EXT_XG0_MDIO_CTRL_DATA + (bus * 4), val);

        nlm_hal_write_mac_reg( node, block, intf_type, EXT_XG0_MDIO_CTRL+ bus * 4, ctrl);
        WAIT_XGMAC_MDIO_BSY_CLEAR(node)

        nlm_hal_write_mac_reg( node, block, intf_type, EXT_XG0_MDIO_CTRL+ bus * 4, ctrl|EXT_XG_MDIO_CTRL_CMD_LOAD);
        WAIT_XGMAC_MDIO_BSY_CLEAR(node)

        nlm_hal_write_mac_reg( node, block, intf_type, EXT_XG0_MDIO_CTRL+ bus * 4, ctrl);
        WAIT_XGMAC_MDIO_BSY_CLEAR(node)

        return 0;
}

int nlm_hal_xgmac_mdio_read(int node, int bus, int phyaddr, int dev_addr, int regidx)
{
	int rdval;
	nlm_hal_xgmac_mdio_write(node, bus, phyaddr, dev_addr, MDIO_CTRL_OP_INDIRECT_ADDR, regidx);
	nlm_hal_xgmac_mdio_write(node, bus, phyaddr, dev_addr, MDIO_CTRL_OP_READ_10G_MMD, regidx);
        rdval =  nlm_hal_read_mac_reg(node, BLOCK_7, LANE_CFG, EXT_XG0_MDIO_RD_STAT + (bus * 4)) & 0xFFFF;
        return rdval;
}

int nlm_hal_c45_mdio_indirect_write_external(int node, int bus,
	int phyaddr, int dev_addr, uint32_t reg_addr, uint32_t write_data)
{
	write_data &= 0xFFFF;
	nlm_hal_xgmac_mdio_write(node, bus, phyaddr, dev_addr, MDIO_CTRL_OP_INDIRECT_ADDR, reg_addr);
	nlm_hal_xgmac_mdio_write(node, bus, phyaddr, dev_addr, MDIO_CTRL_OP_WRITE_10G_MMD, write_data);
	/*nlm_print("C45 MDIO w phy:%d dev:%d reg[%04X]=[%04X]\n", phyaddr, dev_addr, reg_addr, write_data); */
	return 0;
}

int nlm_hal_c45_mdio_indirect_read_external (int node, int bus,
	int phyaddr, int dev_addr, uint32_t reg_addr)
{
	int rdval = nlm_hal_xgmac_mdio_read(node, bus, phyaddr, dev_addr, reg_addr);
	/*nlm_print("C45 MDIO r phy:%d dev:%d reg[%04X]=[%04X]\n", phyaddr, dev_addr, reg_addr, rdval); */
	return rdval;
}

/* Removed N511 support */

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/module.h>
EXPORT_SYMBOL(nlm_hal_mdio_reset);
EXPORT_SYMBOL(nlm_hal_mdio_read);
EXPORT_SYMBOL(nlm_hal_mdio_write);
EXPORT_SYMBOL(nlm_hal_mdio_rd);
EXPORT_SYMBOL(nlm_hal_mdio_wr);
EXPORT_SYMBOL(nlm_hal_c45_mdio_indirect_read_external);
EXPORT_SYMBOL(nlm_hal_c45_mdio_indirect_write_external);
#endif

