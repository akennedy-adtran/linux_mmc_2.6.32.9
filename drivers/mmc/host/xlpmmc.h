/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#ifndef _XLPMMC_H_
#define _XLPMMC_H_

#ifdef CONFIG_MMC_XLP_ENABLE_SLOT_1
#define XLPMMC_MAX_SLOTS		2
#else
#define XLPMMC_MAX_SLOTS		1
#endif

#define XLP_SLOT_SIZE			0x100

#define PCIE_HDR_OFFSET			0x100
#define GPIO_MMC_DETECT			29

#define XLPMMC_DESCRIPTOR_SIZE	(64<<10)
#define XLPMMC_DESCRIPTOR_COUNT	1

/* Reference clock to MMC host controller - assume 133 MHz.  TODO
   Add code to actually check the reference in case it was changed */
#define XLP_REF_CLK133MHZ		133333333

/* Status flags used by the host structure */
#define HOST_F_XMIT				0x0001
#define HOST_F_RECV				0x0002
#define HOST_F_STOP				0x1000

#define HOST_S_IDLE				0x0001
#define HOST_S_CMD				0x0002
#define HOST_S_DATA				0x0003

/*Host controller RESPONSE defines */
#define RSP_TYPE_MASK			(0x3 << 16)
#define RSP_TYPE_NORSP			(0x0 << 16)
#define RSP_TYPE_LGHT136		(0x1 << 16)
#define RSP_TYPE_LGHT48			(0x2 << 16)
#define RSP_TYPE_LGHT48B		(0x3 << 16)
#define CCCE_NOCHECK			(0x0 << 19)
#define CCCE_CHECK				(0x1 << 19)
#define CICE_NOCHECK			(0x0 << 20)
#define CICE_CHECK				(0x1 << 20)
#define DP_NO_DATA				(0x0 << 21)
#define DP_DATA					(0x1 << 21)
#define DDIR_WRITE				(0x0 << 4)
#define DDIR_READ				(0x1 << 4)
#define MBS_SINGLE				(0x0 << 5)
#define MBS_MTPLE				(0x1 << 5)
#define DMA_EN					(0x1 << 0)

/* Response types */
#define RSP_TYPE_NONE	(RSP_TYPE_NORSP   | CCCE_NOCHECK | CICE_NOCHECK)
#define RSP_TYPE_R1		(RSP_TYPE_LGHT48  | CCCE_CHECK   | CICE_CHECK)
#define RSP_TYPE_R1B	(RSP_TYPE_LGHT48B | CCCE_CHECK   | CICE_CHECK)
#define RSP_TYPE_R2		(RSP_TYPE_LGHT136 | CCCE_CHECK   | CICE_NOCHECK)
#define RSP_TYPE_R3		(RSP_TYPE_LGHT48  | CCCE_NOCHECK | CICE_NOCHECK)
#define RSP_TYPE_R4		(RSP_TYPE_LGHT48  | CCCE_NOCHECK | CICE_NOCHECK)
#define RSP_TYPE_R5		(RSP_TYPE_LGHT48  | CCCE_CHECK   | CICE_CHECK)
#define RSP_TYPE_R6		(RSP_TYPE_LGHT48  | CCCE_CHECK   | CICE_CHECK)
#define RSP_TYPE_R7		(RSP_TYPE_LGHT48  | CCCE_CHECK   | CICE_CHECK)

/* Host buf size*/
#define HOST_BUF_SZ_64				(1<<14)

/*Command Support Registers*/	
#define HC_SDMA_SA_OR_ARG2_LO		0x0000
#define HC_SDMA_SA_OR_ARG2_HI		0x0002
#define HC_BLOCK_SIZE				0x0004
#define HC_BLOCK_COUNT				0x0006
	#define BLK_CNT_SHT				16
	#define BLK_SZ_LOW_BITS			(1<<12)
	#define BLK_SZ_HGH_BIT			(1<<15)

#define HC_ARG1_LO					0x0008
	#define AUTO_CMD12_EN			(1<<2)
	
#define HC_ARG1_HI					0x000A
#define HC_TX_MODE_COMMAND			0x000C
	#define CMD_IDX_SHT				24

/*Response registers*/
#define HC_RESPONSE0				0x0010
#define HC_RESPONSE1				0x0014
#define HC_RESPONSE2				0x0018
#define HC_RESPONSE3				0x001C

/*Data Port Registers */
#define HC_BUFF_DATA_PORT0			0x0020
#define HC_BUFF_DATA_PORT1			0x0022

/*Controller status and control registers*/
#define HC_PRESENT_STATE_LO			0x0024
#define HC_PRESENT_STATE_HI			0x0026
#define HC_PC_HC					0x0028
#define HC_WC_BGC					0x002A
#define HC_CLOCK_CTRL				0x002C
	#define HCC_INT_CLK_EN			(1<<0)
	#define HCC_INT_CLK_STABLE		(1<<1)
	#define HCC_SD_CLK_EN			(1<<2)
	#define HCC_10BIT				(1<<5)
	#define HCC_VALUE(x)			((((x-1) & 0x0ff) << 8) | (((x-1) & 0x300) >> 2))
#define HC_SWRST_TIMEOUT_CTRL		0x002E
	#define TIMEOUT_VAL				0xE
	#define SW_RST_CMD				((1<<9) | TIMEOUT_VAL)
#define HC_SYSCTRL					0x0200 

/*Interrupt Registers*/
#define HC_NORMAL_INT_STS			0x0030
	#define HNIS_CMD_CMPL			(1<<0)  /* command completion intr. */
	#define HNIS_TC_CMPL			(1<<1)  /* transaction completion intr. */
	#define HNIS_DMA				(1<<3)  /* DMA interrupt */
	#define HNIS_BUFF_WR_RDY		(1<<4)  /* write buffer ready intr. */
	#define HNIS_BUFF_RD_RDY		(1<<5)  /* ready to read buffer intr. */
	#define HNIS_CINS				(1<<6)  /* Card inserted Interrupt */
	#define HNIS_CREM				(1<<7)  /* Card removed Interrupt */
	#define HNIS_ERR 				(1<<15) /* An error Interrupt */
#define HC_ERROR_INT_STS			0x0032
	#define	CMD_TIMEOUT_ERR			(1<<0)
	#define CMD_CRC_ERR				(1<<1)
	#define DATA_TIMEOUT_ERR		(1<<4)
#define HC_NORMAL_INT_STS_EN 		0x0034
#define HC_ERROR_INT_STS_EN 		0x0036
#define HC_NORMAL_INT_SIGNAL_EN 	0x0038
#define HC_ERROR_INT_SIGNAL_EN 		0x003A
#define HC_AUTO_CMD_ERROR_STS 		0x003C

/*Capabilities registers*/
#define HC_MMC_CAP0					0x40
#define HC_MMC_CAP1					0x44
#define HC_MAX_CURRENT_CAP_LO		0x48
#define HC_MAX_CURRENT_CAP_HI		0x4A

/*Sector size*/
#define MMCSD_SECTOR_SIZE			512
#define HC_HCR_4BIT_MODE			0x2

/* OR of OCR voltages we want. sector mode support */
#define OCR_VDD_27_36					0x00FF8000	/* VDD volt 2.70 ~ 3.60 */

#endif/*_XLPMMC_H_*/
