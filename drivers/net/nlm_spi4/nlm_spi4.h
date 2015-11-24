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


#ifndef _ASM_NLM_SPI4_H
#define _ASM_NLM_SPI4_H

#include <linux/types.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/xlr_mac.h>
#include <asm/netlogic/sim.h>
#include "nlm_spi4_config.h"


#define NLM_SPI4_MAX_THREADS 32
#define NLM_SPI4_PORTS_PER_CARD XLR_TOTAL_CHANNELS


#define MAX_SPILL_SIZE          (MAX_NUM_DESC + 128)
#define MAX_FRIN_SPILL          (MAX_SPILL_SIZE * NLM_SPI4_PORTS_PER_CARD)
#define MAX_FROUT_SPILL         (MAX_SPILL_SIZE * NLM_SPI4_PORTS_PER_CARD)
#define MAX_CLASS_0_SPILL       (MAX_SPILL_SIZE * NLM_SPI4_PORTS_PER_CARD)
#define MAX_CLASS_1_SPILL       (MAX_SPILL_SIZE * NLM_SPI4_PORTS_PER_CARD)
#define MAX_CLASS_2_SPILL       (MAX_SPILL_SIZE * NLM_SPI4_PORTS_PER_CARD)
#define MAX_CLASS_3_SPILL       (MAX_SPILL_SIZE * NLM_SPI4_PORTS_PER_CARD)



#define SPI4_0_BASE_ADDR 0x10000
#define SPI4_1_BASE_ADDR 0x12000
#define XLR_MAX_SPI4_CHANNEL 0x10
#define XLR_MAX_SPI4_BYTE_OFFSET 7
#define XLR_MAX_TX_BURST 8
#define XLR_MAX_RX_BURST 32
#define TX_CAL_SEQ      0x01
#define RX_CAL_SEQ      0x01


#define TOTAL_SPI4	2
#define SPI4_0	0
#define SPI4_1	1


/* SPI4 registers */
#define SPI4_TX_CAL_LEN 	0x0
#define SPI4_TX_CAL_MAX	0x04
#define SPI4_TX_CAL_X 0x08
#define SPI4_RX_CAL_LEN	0x18
#define SPI4_RX_CAL_X 0x20
#define SPI4_RX_CAL_MAX	0x1c
#define SPI4_TX_MAXBURST1_I 0x30
#define	SPI4_TX_MAXBURST2_I 0x34
#define SPI4_RX_MAXBURST1_I 0x38
#define	SPI4_RX_MAXBURST2_I 0x3c

/* spi4 control register and its bit fields */
#define SPI4_CNTRL_REG	0x6c
#define	TX_RESET		0x1
#define	TX_ENABLE		0x2	
#define	RX_RESET		0x04
#define	RX_ENABLE		0x08
#define	RX_CAL_Y		0x10
#define	USR_SHAL_LPB	0x20
#define	USR_DEEP_LPB	0x40
#define	SPI_SHAL_LPB	0x80
#define	SPI_DEP_LPB		0x100
#define	DDL_ENABLE		0x200
#define	FIFO_SW_RESET	0x400
#define	RX_TRAIN_RESET 	0x800
#define	RX_TRAIN_LOS		0x1000
#define	TX_FRM_ERR_EN		0x2000
#define	MORE_TRN_EN			0x4000
#define	ERL_TRN_EN			0x8000
#define	SYNC_PAT_EN			0x10000


#define SPI4_INTR_REG		0x70
#define	SPI4_TX_STATUS		0x7c
#define	SPI4_TX_STATUS_TX_SYNC 0x4
#define	SPI4_RX_STATUS		0x80
#define SPI4_RX_STATUS_RX_SYNC 0x02
#define	SPI4_RX_FIFO_DEPTH_I 0x88
#define	SPI4_RX_FIFO_BASE_I 0x90
#define	SPI4_TX_FIFO_DEPTH_I 0x8c
#define	SPI4_TX_FIFO_BASE_I	0x94

/* glue logic registers */

#define DESC_PKT_CTRL_1 0xa9
#define DESC_PKT_CTRL_2 0xaa
#define PAD_CALIB_0	0x231
#define RX_P_PRESET 0x1
#define RX_N_PRESET 0x02
#define	TX_P_PRESET 0x4
#define TX_N_PRESET	0x08
#define	RX_EN_COUNTER	 0x10
#define	TX_EN_COUNTER  0x20
#define	CAL_PRESET		0
#define	HSTL_TERMINATION 0x1000
#define	LVDS_TERMINATION 0x2000

/********** CPLD register     **************/
#define  CPLD_MISC_STATUS_REG  0x0e
#define SPI4_MASK_BIT1 1


typedef struct _spi4_driver_data{

	uint32  tx_calendar; // number of ports
	uint32  tx_cal_sequence ;

	uint32  rx_calendar; // number of ports
	uint32  rx_cal_sequence ;

	uint32  tx_maxburst1;
	uint32  tx_maxburst2;

	uint32  rx_maxburst1;
	uint32  rx_maxburst2;

	uint32 		*mmio;
	uint    spi4_slot ; 
	uint32_t cfg_flag;

	spi4_callback_func calbk_func;

	void*	frin_spill;
	void*	frout_spill;
	void*	class_0_spill;
	void*	class_1_spill;
	void*	class_2_spill;
	void*	class_3_spill;
	
}spi4_driver_data;

/* spi4 descriptors */
#define PORT_MASK 0xf
#define DESK_CTRL_MASK 0x7
#define DESC_CTRL_OFFSET  61
#define DESC_LEN_MASK 0x3fff
#define DESC_LEN_OFFSET 40
#define DESC_ERROR_OFFSET 60 
#define DESC_ERROR_MASK 0x01

#define get_error(a) ((((a)>> DESC_ERROR_OFFSET) & DESC_ERROR_MASK ))
#define get_port(a) ((a)&(PORT_MASK))
#define get_ctrl(a) (((a)>>(DESC_CTRL_OFFSET))& (DESK_CTRL_MASK))
#define get_length(a) (((a) >> (DESC_LEN_OFFSET)) & (DESC_LEN_MASK))
#define get_address(a) (((a)&(0xffffffffe0ULL)))



#define SPI4_TX_DESC_ALIGNMENT (SMP_CACHE_BYTES - 1)
extern int cpu_to_frstid[];

static inline int spi4_make_desc_tx(unsigned int thr_id,
		struct  msgrng_msg *msg,
    int   id,
    int   port,
    int   type,
    unsigned long addr,
    unsigned long skb,
    int   len)
{
  int tx_stid = 0;
  int fr_stid = 0;

  tx_stid = msgrng_xgmac_stid_tx(id);
  tx_stid += port;
  fr_stid = cpu_to_frstid[thr_id];

  msg->msg0 = ( ((uint64_t)1 << 63) |
              ( ((uint64_t)127) << 54) |
                ((uint64_t)len << 40) |
                ((uint64_t)addr & 0xffffffffffULL)
              );

  msg->msg1 = ( ((uint64_t)1 << 63) |
              ( ((uint64_t)fr_stid) << 54) |
                ((uint64_t)0 << 40) |
                ((uint64_t)virt_to_phys((void*)skb) & 0xffffffffffULL)
              );

  msg->msg2 = msg->msg3 = 0;

  return tx_stid;

}


static inline int spi4_make_desc_rfr(struct msgrng_msg *msg, int id, int type,
                                           unsigned long addr)
{
  int stid = 0;

  stid = msgrng_xgmac_stid_rfr(id);

  msg->msg0 = (uint64_t)addr & 0xffffffffe0ULL;
  msg->msg1 = msg->msg2 = msg->msg3 = 0;

  return stid;
}


#endif
