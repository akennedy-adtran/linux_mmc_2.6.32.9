
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

#ifndef __NLM_SRIO_XLP_H__
#define __NLM_SRIO_XLP_H__

/* SRIO Block configuration */
#define SRIO_MODE_4x1
#define SRIO_BUF_SIZE		4096

#define SRIO_BAUD_1250M		1	/*Mode4 1.25G */
#define SRIO_BAUD_2500M		2	/*Mode3 25.G */
#define SRIO_BAUD_3125M		3	/*Mode2 3.125G */
#define SRIO_BAUD_5000M		4	/*Mode1 5G */
#define SRIO_BAUD_6250M		5	/*Mode0 6.25G */
#define SRIO_PORT_BAUD_RATE	SRIO_BAUD_6250M	

#define MAX_DOORBELL_XLP3XX_AX	16
#define MAX_MSGDESTINATION	256
#define MAX_SWITCH_ENTRY	32
#define MAX_MBOX		4

#define NLM_SRIO_TT		0	/* 8 bit devid by default */
#define NLM_SRIODEV_MASK        0xFFFF 

#define PATH_MODE_SEL_VAL_0	0
#define PATH_MODE_SEL_VAL_4	4

#ifdef SRIO_MODE_1x4
#define PATH_MODE_SEL_VAL	0
#define NLM_MAX_SRIO_PORTS	1
#elif defined(SRIO_MODE_4x1)
#define PATH_MODE_SEL_VAL       4
#define NLM_MAX_SRIO_PORTS      4
#endif

/*SRIO MODE */
#define SRIO_MODE_x1		1
#define SRIO_MODE_x4		2

/* FTYPE */
#define FTYPE_REQUEST		2  /* NREAD, ATMOIC		 */
   #define TRANS_NREAD	  	4
   #define TRANS_ATOMIC_INC  	12
   #define TRANS_ATOMIC_DEC	13
   #define TRANS_ATOMIC_SET  	14
   #define TRANS_ATOMIC_CLR	15		

#define FTYPE_WRITE		5  /* NWRITE, NWRITE_R, ATOMIC */
   #define TRANS_NWRITE		4
   #define TRANS_NWRITE_R	5
   #define TRANS_ATOMIC_SWP	12
   #define TRANS_ATOMIC_CMPSWP	13
   #define TRANS_ATOMIC_TSTSWP  14	

#define FTYPE_SWRITE		6  /* SWRITE */

#define FTYPE_MAINTENANCE	8  /* MAINTENANCE */
   #define TRANS_MREAD		0
   #define TRANS_MWRITE		1
   #define TRANS_MREAD_RESP	2
   #define TRANS_MWRITE_RESP	3
   #define TRANS_PWRITE		4

#define FTYPE_DOORBELL		10 /* Doorbell */

#define FTYPE_MESSAGE		11 /* Data message */

#define FTYPE_RESPONSE		13 /* Response */

/* SRIO VC */
#define XLP_SRIO_TXVC_BASE	280
#define XLP_SRIO_ADDRQ_VC	288

#define NLM_SRIO_BUS_NUM 	0
#define NLM_SRIO_DEV_NUM 	5
#define NLM_SRIO_FUN_NUM 	4 

#define NLM_SRIO_CFG_BASE	 ( 0x18000000 | (NLM_SRIO_DEV_NUM << 15) | (NLM_SRIO_FUN_NUM << 12))   
/* SriComplex Configuration Register */

#define SRICOMCONFIG    	0x40
   #define WR_BLOCK_DISABLE   	0x00000001
   #define DEV_ECC_DISABLE	0x00000002
   #define BIU_TOUT_ENABLE      0x00000004
   #define RESP_TOUT_ENABLE     0x00000008
   #define SRICOMCONFIG_RSVD1   4
   #define DBELL_INDX_START_POS 13  /* 16:13   */
   #define IPCLK_GATING_DIS0    0x00020000 
   #define IPCLK_GATING_DIS1	0x00040000
   #define IPCLK_GATING_DIS2	0x00080000
   #define IPCLK_GATING_DIS3	0x00100000
   #define PAYLOAD_INDESC	0x00200000
   #define SRICOMCONFIG_RSVD2   22  /* 25:22  */
   #define SRI_EACTRL           0x04000000   
   #define IP_RESET	        0x08000000 
   #define REG_RESET	        0x10000000
   #define PLL_RESET	        0x20000000 
   #define SRICOMCONFIG_RSVD3   30  /* 31:30 */

   #define DBELLINDEX_MASK	0xFFFE1FFF
   #define MAX_DBELL_OFFSET_XLP3XX_AX	12
	

/* SRIOIP Control Register */
#define SRIOIPCTRL              0x41
   #define PORT0_RSTREQ		0x00000001
   #define PORT1_RSTREQ		0x00000002
   #define PORT2_RSTREQ         0x00000004        
   #define PORT3_RSTREQ         0x00000008        
   #define PORT0_READY		0x00000010
   #define PORT1_READY          0x00000020                
   #define PORT2_READY          0x00000040                
   #define PORT3_READY          0x00000080                
   #define PORT0_LOOPBACK	0x00000100
   #define PORT1_LOOPBACK       0x00000200                
   #define PORT2_LOOPBACK       0x00000400                
   #define PORT3_LOOPBACK       0x00000800                
   #define PORT0_ENUM		0x00001000
   #define PORT1_ENUM           0x00002000
   #define PORT2_ENUM           0x00004000
   #define PORT3_ENUM           0x00008000
   #define LANE0_TXMODE		0x00010000
   #define LANE1_TXMODE         0x00020000
   #define LANE2_TXMODE         0x00040000
   #define LANE3_TXMODE         0x00080000
   #define BOOT_COMPLETE	0x00100000
   #define SRIO_SWPORT_POS	21 /*28:21 */
   #define PORT_WR_REQ		0x20000000
   #define SRIOIPCTRL_RSVD	30 /*31:30 */
		
/* SRIOIP Configuration Register */
#define SRIOIPCONFIG            0x42
   #define PORT0_PWRDWN		0x00000001
   #define PORT1_PWRDWN		0x00000002
   #define PORT2_PWRDWN		0x00000004
   #define PORT3_PWRDWN		0x00000008
   #define SRIO_PATHID_POS	4  /*8:4 */
   #define PATH_MODE_SEL_POS	9  /*11:9 */
   #define PATH_BAUD_RATE_POS	12 /*16:12 */
   #define PORT0_BAUD_RATE_POS	17 /*19:17 */
   #define PORT1_BAUD_RATE_POS	20 /*22:20 */
   #define PORT2_BAUD_RATE_POS	23 /*25:23 */
   #define PORT3_BAUD_RATE_POS	26 /*28:26 */
   #define PORT_GEN_CTRL_POS	29 /*31:29  */

/* SRIOIP Configuration Status Register */
#define SRIOIPSTATUS		0x43
   #define PORT0_PWRDWN_STS	0x00000001
   #define PORT1_PWRDWN_STS	0x00000002
   #define PORT2_PWRDWN_STS	0x00000004
   #define PORT3_PWRDWN_STS	0x00000008
   #define PATH_MODE_SEL_STS_POS	4  /*6:4 */
   #define LANE0_BAUD_RATE_STS_POS	7  /*10:7 */
   #define LANE1_BAUD_RATE_STS_POS	11 /*14:11 */
   #define LANE2_BAUD_RATE_STS_POS 	15 /*18:15 */
   #define LANE3_BAUD_RATE_STS_POS	19 /*22:19 */
   #define SRIO_HOST_END_STS	0x00800000
   #define SRIO_HOST_EN_STS	0x01000000
   #define SRIO_MASTER_EN_STS	0x02000000
   #define SRIO_DEV_DISC_STS	0x04000000
   #define SRIOIPSTATUS_RSVD	27 /*31:27 */
 	
/* SRIOIP Base Device ID Register */
#define SRIOBASEDEVID           0x44
   #define SRIO_BASE_DEVID_POS		0 /*23:0 */
   #define SRIOBASEDEVID_RSVD		24 /*31:24 */
   #define SRIO_8BIT_DEVID_POS		16	

/* SRIOIP Base Device ID Status Register */
#define  SRIOBASEDEVID_STATUS	0x45
   #define SRIOBASEDEVID_STAT_POS	0 /*23:0 */
   #define SRIOBASEDEVIDS_RSVD 		24 /*31:24  */

/* SRIOIP Device ID Register */
#define SRIODEVICEID            0x46

/* SRIOIP Device INFO Register */
#define SRIODEVICEINFO          0x47

/* SRIOIP Port Write Data Register */
#define SRIOPORTWDATA           0x48

/*SRIO PHY Reg Read/Write Control Register (Indirect Access) */
#define PHYRDWRCTRL             0x49
   #define PHY_REGLANE		0
   #define PHY_REGADDR		4

/* SRIO PHY Reg Read/Write Access Register (Indirect Access) */
#define PHYRDWRACCS            	0x4A

/* Biu Timeout Value Register */
#define BIU_TIMEOUT_VAL		0x4B

/*Response Packet Timeout Value Register */
#define RESP_TIMEOUT_VAL	0x4C

/* Message Destination ID for Port */
/* Total of 4 ports */
#define MSG_DESTID		0x4D
   #define MSG_DESTID_PORT_POS		0 /*11:0 */
   #define MSG_DESTID_CODE_POS  	12 /* 19:12 */

/* Message Destination ID for Doorbell Distribution Table  */
/* Total of 16 entries */
#define DOORBELL_DESTID_XLP3XX_AX	0x51

/* Destination ID for Data Message Lookup Byte Offset */
#define DATAMSGBYTE     	0x61 /*Lookup Byte Offset; 0-255 */

/* Destination ID for Data Message Lookup Enable */
/* Total of 4 entries - 32x4=128 bits */
#define DATAMSGEN0               0x62
#define DATAMSGEN1               0x63
#define DATAMSGEN2               0x64
#define DATAMSGEN3               0x65

/* Destination ID for Data Message Distribution Table */
/* Total of 256 entries */
#define DATAMSG_DESTID		0x66

/* Switch CAM Table */
/* Total of 32 entries */
#define SWITCH_ENTRY		0x166
   #define SWITCH_ENTRY_KEY	0
   #define SWITCH_RES_PORT	16
   #define SWITCH_RES_CPU	18
   #define SWITCH_RES_DROP	19

/* Switch Default */
#define SWITCHDEFAULT           0x186
   #define SWITCH_DEF_PORT	0
   #define SWITCH_DEF_CPU	2
   #define SWITCH_DEF_DROP	3

/* Direct IO Operation Table */
/* Totll of 32 entries */
#define DIRECTIO_CMD		0x187
   #define DIRIO_CMD_DESTID	0
   #define DIRIO_CMD_PORT	16
   #define DIRIO_CMD_TT		18
   #define DIRIO_CMD_CRC	19
   #define DIRIO_CMD_CRF	20
   #define DIRIO_CMD_PRIO	21
   #define DIRIO_CMD_HOP	23

/* Diret IO Response Table */
/* Total of 32 entires */
#define DIRECTIO_RESP   	0x1a7

/* Diret IO Ctrl */
#define DIRECTIO_CTRL		0x1c7

/* Byte Swap Register */
#define BYTE_SWAP 		0x1ca

/* Alloc Base Register */
#define ALLOC_BASE		0x1cb

/* Alloc Limit Register */
#define ALLOC_LIMIT		0x1cc

/* Read Exclusive Base Register */
#define RDEXC_BASE		0x1cd

/* Read Exclusive Limit Register */
#define RDEXC_LIMIT		0x1ce

/* Sri Error Source 0 Register */
#define SRIERR_SRC0		0x1cf
   #define BIU_TIMEOUT		0
   #define DMA_RDERR		8
   #define INMSG_ERR		16
   #define OUTMSG_S_ERR		25
   #define OUTMSG_M_ERR		26
   #define SRIGEN_EXCP		27

/* Sri Error Source 1 Register */
#define SRIERR_SRC1 		0x1d0

/* Sri Error Interrupt Enable 0 Register */
#define SRIERRINT_EN0    	0x1d1

/* Sri Error Interrupt Enable 1 Register */
#define SRIERRINT_EN1 		0x1d2

/* FMB Egress SECC Info Register */
#define SRI_SECCINF        	0x1d3

/* FMB Egress MECC Info Register */
#define SRI_MECCINF		0x1d4

/* Sri Response Timeout Info 0 Register */
#define SRI_RSPTMINF0		0x1d5

#define SRI_RSPTMINF1           0x1d6
#define SRI_RSPTMINF2           0x1d7

#define SRI_RSPTMINF3           0x1d8


/*RIO Specific registers */
#define RIO_SRC_OP		0x18
#define RIO_PORTMAINT_HDR	0x100
#define RIO_LNKTOUT_CSR		0x120
#define RIO_RESPTOUT_CSR	0x124
#define RIO_PORTGEN_CSR 	0x13c

#define RIO_PORT0CTRL2_CSR      0x154
#define RIO_PORT1CTRL2_CSR      0x174
#define RIO_PORT2CTRL2_CSR      0x194
#define RIO_PORT3CTRL2_CSR      0x1b4

#define RIO_PORT0CTRL_CSR	0x15c	
#define RIO_PORT1CTRL_CSR       0x17c               
#define RIO_PORT2CTRL_CSR       0x19c               
#define RIO_PORT3CTRL_CSR       0x1bc               

/*/ SBB */

#define DIRIO_RESP_MASK		0xF
/* dirio 40 bit phys addr */
#define DIRIO_PHYS_ADDR		0
#define DIRIO_PHYS_THRD		32
#define DIRIO_PHYS_OP		37
#define DIRIO_PHYS_SRIO		39

/* dirio opcode */
#define DIRIO_OP_NREAD		0
#define DIRIO_OP_MREAD		3

#define DIRIO_OP_NWRITE		0
#define DIRIO_OP_NWRITE_R	1
#define DIRIO_OP_MWRITE		2
#define DIRIO_OP_PWRITE		3


#define SRIO_DIRIO_CMD(dst, port, tt, crc, crf, prio, hop) \
	((hop << DIRIO_CMD_HOP)  | (prio << DIRIO_CMD_PRIO) | \
	 (crf << DIRIO_CMD_CRF) | (crc << DIRIO_CMD_CRC) | \
	 (tt << DIRIO_CMD_TT) | (port << DIRIO_CMD_PORT) | dst)

	
#define SRIO_DIRIO_40BITPHYS(addr, cpu, opcode) \
	(uint64_t)((1ULL << DIRIO_PHYS_SRIO) | ((uint64_t)opcode << DIRIO_PHYS_OP) | ((uint64_t)cpu << DIRIO_PHYS_THRD) | addr)

#define DIRECTIO_CMD_OFF	(DIRECTIO_CMD + getpid())
#define DIRECTIO_RESP_OFF	(DIRECTIO_RESP + getpid())

#define nlm_read_srio_devreg(index) nlm_hal_read_32bit_reg((uint64_t)NLM_SRIO_CFG_BASE, index)
#define nlm_write_srio_devreg(index, val) nlm_hal_write_32bit_reg((uint64_t)NLM_SRIO_CFG_BASE, index, val)

#if 0
#define nlm_read_srio_reg8(offset)	lb_40bit_phys_uncached(((uint64_t)xlp_srio_base) + (offset))	
#define nlm_read_srio_reg16(offset)  	lh_40bit_phys_uncached(((uint64_t)xlp_srio_base) + (offset))
#define nlm_read_srio_reg64(offset)     ld_40bit_phys_uncached(((uint64_t)xlp_srio_base) + (offset))
#endif

#define nlm_read_srio_reg32(offset) 	lw_40bit_phys_uncached((uint64_t)xlp_srio_base + offset)
#define nlm_read_srio_reg(index)	nlm_read_srio_reg32((index<<2))


#if 0
#define nlm_write_srio_reg8(offset, val)	sb_40bit_phys_uncached(((uint64_t)xlp_srio_base) + (offset), val)	
#define nlm_write_srio_reg16(offset, val)	sh_40bit_phys_uncached(((uint64_t)xlp_srio_base) + (offset), val)	
#define nlm_write_srio_reg64(offset, val)       sd_40bit_phys_uncached(((uint64_t)xlp_srio_base) + (offset), val)  
#endif

#define nlm_write_srio_reg32(offset, val)	sw_40bit_phys_uncached((uint64_t)xlp_srio_base + offset, val)	
#define nlm_write_srio_reg(index, val)	nlm_write_srio_reg32((index<<2), val)


/* FMN */

/* xlp3xx-Ax */
#define IS_FREEBACK(desc)		((desc >> 63) & 0x01ULL)
#define IS_DMA_ERR(desc)		((desc >> 62) & 0x01ULL)	
#define SRIO_PORT(desc)			((desc >> 59) & 0x03ULL)	
#define SRIO_FTYPE(desc) 		((desc >> 48) & 0x0FULL)
#define SRIO_DEST_ID(desc)		((desc >> 32) & 0xFFFFULL)
#define SRIO_SRC_ID(desc)		((desc >> 16) & 0xFFFFULL)
#define SRIO_TRANS_TYPE(desc)		((desc >> 12) & 0x0FULL)
#define SRIO_RESP_STATUS(desc)		((desc >> 8) & 0x0FULL)	
#define SRIO_PAYLOAD_LENGTH_XLP3XX_AX(desc)	((desc >> 48) & 0xFFFFULL)
#define SRIO_PAYLOAD_ADDR(desc)		(desc & 0xFFFFFFFFFFULL)   
#define SRIO_DBELL_INFO(desc)		((desc >> 48) & 0xFFFFULL)
#define SRIO_MBOX(desc)			((desc >> 4) & 0x3)

#define SRIO_FREEBACK			(1ULL << 58)
#define SRIO_FMB_NO_DMA			(1ULL << 63)
#define SRIO_MIN_SEGMENT_SIZE		8
#define SRIO_MAX_SEGMENT_SIZE		256

#define XLP_SRIO_FMBRX_DESC0(port, ftype, dest_dev, msg_dest_id, pri)  \
	(uint64_t)(((uint64_t)port<<59) | ((uint64_t)pri<<54) | ((uint64_t)NLM_SRIO_TT << 52) | ((uint64_t)ftype << 48) | \
	((uint64_t)(dest_dev & NLM_SRIODEV_MASK) << 32) | (msg_dest_id << 16))


/* 
 * Support for XLP3xx-B0, XLP1xx , XLP2xx
 * Defined the register and descriptor format which is different from XLP3XX_AX
 */

#define SRIO_MAX_P2DS		128

#define IS_LINK_RESP_TOUT(desc) ((desc >> 61) & 0x01ULL)
#define SRIO_COS(desc)		((desc >> 8) & 0xffULL)
#define SRIO_STREAM_ID(desc)	(desc & 0xffffULL)
#define SRIO_LENGTH_ERROR(desc) ((desc >> 16) & 0x01ULL)

#define MAX_DOORBELL		32
#define MAX_DBELL_OFFSET	11

/* Register offsets */
    /*comconfig register changes  */
#define SRICOMCONFIG_RXDMSG_TIMER_EN	0x00000010
#define SRICOMCONFIG_TXDMSG_TIMER_EN	0x00000040

#define DOORBELL_DESTID		0x1F0

#define SRIO_DATAMSG_SIZE	0x1D9

#define SRIO_SUPPRESS_CTRL      0x1E8
#define TYPE2_SUPRS_EN          0x0001
#define TYPE5_NORESP_SUPRS_EN   0x0002
#define TYPE5_RESP_SUPRS_EN     0x0004
#define TYPE6_SUPRS_EN          0x0008
#define TYPE9_SUPRS_EN          0x0010
#define TYPE10_SUPRS_EN         0x0020
#define TYPE11_SUPRS_EN         0x0040

#define RXEDMESG_TIMEOUT	0x210		
#define TXEDMESG_TIMEOUT	0x212

#define FLUSHADDRQ             0x278

#define MAX_PACKET_LEN			4096
#define MAX_P2PDMSG_PACKET_LEN		1024
#define SRIO_PAYLOAD_LENGTH(desc)       ((desc >> 47) & 0x1FFFFULL)
#define ADDR_MASK_40BIT			0xffffffffffULL
/*------------------------------------------------------------------- 
 * Message 1
 *
 * ---------------------------------------------------------------------*/

/* NREAD, NWRITE, NWRITE_R, SWRITE */
#define SRIO_P2P		(0x8000000000000000ULL)
#define SRIO_P2D_LAST		(0x4000000000000000ULL)
#define SRIO_INJECT_CRC		(0x2000000000000000ULL)
#define SRIO_SEND_FREEBACK	(0x0400000000000000ULL)
#define SRIO_SEND_RESP		(0x0200000000000000ULL)
#define SRIO_NODMA_RESP		(0x0000000080000000ULL)

#define	SRIO_P2P_POS		63	/* P2P or P2D desc */
#define SRIO_P2D_LAST_POS	62	/* Last P2D desc  */
#define SRIO_TT_POS		52	/* Transport type */
#define SRIO_FTYPE_POS		48	/* Format type */
#define SRIO_DESTID_POS		32	/* Destination device ID */
#define SRIO_MSG_DESTID_POS	16	/* Message destination ID */
#define SRIO_TRANS_POS		12	/* Transaction type */
#define SRIO_SIZE_POS		0	/* Transfer byte size */

#define SRIO_SWRITE_SIZE_POS	3	/* SWRITE transfer byte size */

#define SRIO_MAINT_SIZE_POS	0

#define SRIO_STREAM_COS_POS	8

#define SRIO_DBELL_INFO_POS	0

#define SRIO_DMSG_LTR_POS	6
#define SRIO_DMSG_MBOX_POS	4
#define SRIO_DMSG_XMBOX_POS	0


#define SRIO_TT_MASK		(0x03ULL)
#define SRIO_FTYPE_MASK		(0x0fULL)
#define SRIO_DESTID_MASK	(0xffffULL)
#define SRIO_MSG_DESTID_MASK	(0x0fffULL)
#define SRIO_TRANS_MASK		(0x0fULL)
#define SRIO_SIZE_MASK		(0x0fffULL)

#define SRIO_SWRITE_SIZE_MASK	(0x1ffULL)

#define SRIO_MAINT_SIZE_MASK	(0xfULL)

#define SRIO_DBELL_INFO_MASK	(0xffffULL)

#define SRIO_P2P_MESSAGE	0x8000000000000000ULL
#define SRIO_DATA_MESSAGE_P2P	0x2000000000000000ULL
#define SRIO_DATA_MESSAGE_POS	61
#define SRIO_SINGLE_SEGMENT_POS	60
		
#define SRIO_DONT_SEND_RESP	0ULL
#define SRIO_DONT_SEND_FREEBACK	0ULL
#define SRIO_P2D_NOT_LAST	0ULL

/*-------------------------------------------------------------------
 * Message 2: NREAD/NWRITE/NWRITE_R
 *  	63..40 Rsvd
 *	39..0 Address
 * Message 2: Maintenance
 * 	63..32 Rsvd
 *	31..24	hop count
 *	23..2	configuration offset
 *----------------------------------------------------------------*/
#define SRIO_PACKET_ADDR_MASK	ADDR_MASK_40BIT

#define SRIO_HOP_COUNT_POS	24
#define SRIO_CONFIG_OFF_POS	2

#define SRIO_STREAM_SIZE_POS	48
#define SRIO_STREAM_ID_POS	0
#define SRIO_STREAM_SIZE_MASK	0xffffULL
#define SRIO_STREAM_ID_MASK	0xffffULL


#define SRIO_DMSG_SIZE_POS	52
#define SRIO_DMSG_LEN_POS	48

#define SRIO_DATAMSG_TIMEOUT	0x20000ULL

#define SRIO_P2P_NP2DS_POS	42
#define SRIO_P2P_P2DADDR_POS	6
#define SRIO_P2D_ADDR_MASK	0xFFFFFFFFC0ULL

/*-------------------------------------------------------------------
 * Message 3 : NREAD
 *  	63..41	Rsvd
 *	40	cache alloc
 *	39..0	Base address to store response
 * Message 3 : NWRITE/NWRITE_R
 *  	63..42	Rsvd / dword[63..42]
 *	41	Read exclusive / dword[41]
 *	40	Cache alloc  / dword[40]
 *	39..0	payload base address / dword[39..0]
 *--------------------------------------------------------------------*/

#define SRIO_READ_EXCL		41
#define SRIO_CACHE_ALLOC_POS	40
#define SRIO_PLOAD_ADDR_MASK	ADDR_MASK_40BIT


static __inline__ uint64_t xlp_nread_fmbrx_desc0(uint64_t p2dlast, uint64_t resp, uint32_t dest_dev, uint32_t  msg_dest_id, uint32_t size) 
{
	return ((uint64_t)( ((uint64_t) p2dlast) | ((uint64_t) resp) | ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) |
			    ((uint64_t)	FTYPE_REQUEST << SRIO_FTYPE_POS) | ((uint64_t) dest_dev << SRIO_DESTID_POS) |
			    ((uint64_t) msg_dest_id << SRIO_MSG_DESTID_POS) |
			    ((uint64_t) TRANS_NREAD << SRIO_TRANS_POS) | ((uint64_t) (size & SRIO_SIZE_MASK))	
        		  ));
}

static __inline__ uint64_t xlp_nwrite_fmbrx_desc0(uint64_t p2dlast, uint64_t freeback, uint32_t dest_dev, uint32_t  msg_dest_id, uint32_t size) 
{
	return ((uint64_t)( ((uint64_t) p2dlast) | ((uint64_t) freeback) | ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) |
			    ((uint64_t)	FTYPE_WRITE << SRIO_FTYPE_POS) | ((uint64_t) dest_dev << SRIO_DESTID_POS) |
			    ((uint64_t) msg_dest_id << SRIO_MSG_DESTID_POS) |
			    ((uint64_t) TRANS_NWRITE << SRIO_TRANS_POS) | ((uint64_t) (size & SRIO_SIZE_MASK))
        		  ));
}


static __inline__ uint64_t xlp_nwriter_fmbrx_desc0(uint64_t p2dlast, uint64_t resp, uint32_t dest_dev, uint32_t  msg_dest_id, uint32_t size) 
{
	return ((uint64_t)( ((uint64_t) p2dlast) | ((uint64_t) resp) | ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) |
			    ((uint64_t)	FTYPE_WRITE << SRIO_FTYPE_POS) | ((uint64_t) dest_dev << SRIO_DESTID_POS) |
			    ((uint64_t) msg_dest_id << SRIO_MSG_DESTID_POS) |
			    ((uint64_t) TRANS_NWRITE_R << SRIO_TRANS_POS) | ((uint64_t)(size & SRIO_SIZE_MASK))	
        		  ));
}

static __inline__ uint64_t xlp_swrite_fmbrx_desc0(uint64_t p2dlast, uint64_t freeback, uint32_t dest_dev, uint32_t  msg_dest_id, uint32_t size) 
{
	return ((uint64_t)( ((uint64_t) p2dlast) | ((uint64_t) freeback) | ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) |
			    ((uint64_t)	FTYPE_SWRITE << SRIO_FTYPE_POS) | ((uint64_t) dest_dev << SRIO_DESTID_POS) |
			    ((uint64_t) msg_dest_id << SRIO_MSG_DESTID_POS) |
			    ((uint64_t) (size & SRIO_SIZE_MASK))	
        		  ));
}

static __inline__ uint64_t xlp_dbell_fmbrx_desc0(uint64_t p2dlast, uint64_t resp, uint32_t dest_dev, uint32_t  msg_dest_id, uint16_t info) 
{
	return ((uint64_t)( ((uint64_t) p2dlast) | ((uint64_t) resp) | ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) |
			    ((uint64_t)	FTYPE_DOORBELL << SRIO_FTYPE_POS) | ((uint64_t) dest_dev << SRIO_DESTID_POS) |
			    ((uint64_t) msg_dest_id << SRIO_MSG_DESTID_POS) |
			    ((uint64_t) (info & SRIO_DBELL_INFO_MASK))	
        		  ));
}

static __inline__ uint64_t xlp_dmesg_fmbrx_desc0(uint64_t p2dlast, uint64_t resp, uint32_t dest_dev, uint32_t  msg_dest_id, uint32_t mbox) 
{
	return ((uint64_t)( ((uint64_t) p2dlast) | ((uint64_t) resp) | ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) |
			    ((uint64_t)	FTYPE_MESSAGE << SRIO_FTYPE_POS) | ((uint64_t) dest_dev << SRIO_DESTID_POS) |
			    ((uint64_t) msg_dest_id << SRIO_MSG_DESTID_POS) |
			    ((uint64_t) (mbox << SRIO_DMSG_MBOX_POS))	
        		  ));
}

static __inline__ uint64_t xlp_dmesg_fmbrx_desc1(uint32_t size, uint32_t msglen, uint64_t paddr)
{
	return ((uint64_t)( ((uint64_t) (size & 0xFFF) << SRIO_DMSG_SIZE_POS) |
		     	    ((uint64_t) ((msglen - 1) / SRIO_MAX_SEGMENT_SIZE) << SRIO_DMSG_LEN_POS) |
			    ((uint64_t) (paddr & SRIO_PACKET_ADDR_MASK))
			  ));
}

static __inline__ uint64_t xlp_p2p_dmesg_fmbrx_desc0(uint32_t ssegment, uint32_t dest_id, uint32_t mbox)
{
	return ((uint64_t)( ((uint64_t) SRIO_P2P_MESSAGE) | ((uint64_t) SRIO_DATA_MESSAGE_P2P) |
			    ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) | ((uint64_t) ssegment << SRIO_SINGLE_SEGMENT_POS) |
			    ((uint64_t) dest_id << SRIO_DESTID_POS) | ((uint64_t) mbox << SRIO_DMSG_MBOX_POS)
			  ));
}

static __inline__ uint64_t xlp_p2p_fmbrx_desc0(void)
{
	return ((uint64_t)( ((uint64_t) SRIO_P2P_MESSAGE) | ((uint64_t) NLM_SRIO_TT << SRIO_TT_POS) ));
}

static __inline__ uint64_t xlp_p2p_fmbrx_desc1(uint32_t np2ds, uint64_t p2dlist)
{
	return ((uint64_t)( ((uint64_t) (np2ds & 0x7fULL) << SRIO_P2P_NP2DS_POS) |
			    ((uint64_t) (p2dlist & SRIO_P2D_ADDR_MASK))
			  ));	
}

#endif
