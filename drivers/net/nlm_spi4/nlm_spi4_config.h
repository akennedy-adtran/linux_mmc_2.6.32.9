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


#ifndef _ASM_NLM_SPI4_CONFIG_H
#define _ASM_NLM_SPI4_CONFIG_H

#include <linux/types.h>

typedef unsigned int uint32;



/* configuration parameters*/

/* indicates how many spi4 channels currently need to be used
 * by the upper driver */
#define XLR_TOTAL_CHANNELS 0xa

/* indicates number of 16 byte blocks that fifo can accept
 * during fifo starving for the transmit path*/
#define XLR_SPI4_TX_MAXBURST1    0x8

/* indicates number of 16 byte blocks that fifo can accept
 *  * during fifo hungry for the transmit path*/
#define XLR_SPI4_TX_MAXBURST2    0x8

/* indicates number of 16 byte blocks that fifo can accept
 * during fifo starving for the receive path*/
#define XLR_SPI4_RX_MAXBURST1    0x8

/* indicates number of 16 byte blocks that fifo can accept
 * during fifo hungry for the receive path*/
#define XLR_SPI4_RX_MAXBURST2    0x8



/* folowing are the configuration parameters for the ethrnet mac 
 * driver */

/* threshold for tx starts*/
#define TX_THRESHOLD_SIZE      	512

/* max buffer size for the tx and rx*/
#define REG_FRAME_SIZE 		1536
#define MAX_FRAME_SIZE          REG_FRAME_SIZE

/* indicates byte offset for rx data start 
 * from the rx buffer  */
#define SPI4_BYTE_OFFSET        2

#define MAC_CRC_LEN             4
#define MAC_SKB_BACK_PTR_SIZE   SMP_CACHE_BYTES

#ifdef CONFIG_NLMCOMMON_PTP_SUPPORT
#define MAC_PREPAD             32
#else 
#define MAC_PREPAD              0
#endif

/* number of descriptors for each spi4 channel given for the 
 * rx data*/
#define MAX_NUM_DESC            512




typedef void (*spi4_callback_func)(uint32,uint32, uint32, char*, 
		uint32, uint32 error);

enum spi4_callback_cmd{
	SPI4_TX_DONE ,
	SPI4_RX_IND
};

/* exported API*/

extern unsigned int spi4_init(uint32   slot, spi4_callback_func);
extern void spi4_program_rx_desc(uint32  slot,  char*);
extern int spi4_open(uint32 slot);
extern void spi4_close(uint32 slot);
extern int spi4_tx(unsigned int thr_id, uint32 slot,  uint32 spi4_port,
                char* data, unsigned char* skb,uint32 len);
extern int spi4_read_reg(uint32 slot, uint32 addr);


enum spi4_returns{
        SPI4_PASS = 0x0,
        SPI4_TX_FAIL,
        SPI4_SLOT_ERROR,
        SPI4_MALLOC_FAIL,
        SPI4_TX_SYNC_FAIL,
	SPI4_RX_SYNC_FAIL,
        SPI4_INIT_SUCCESS,
	SPI4_CALENDER_LEN_ERROR,
	SPI4_TX_MAXBURST1_ERROR,
	SPI4_TX_MAXBURST2_ERROR,
	SPI4_RX_MAXBURST1_ERROR,
	SPI4_RX_MAXBURST2_ERROR,
	SPI4_TX_MAX_BURST_ERROR,
	SPI4_RX_MAX_BURST_ERROR,
	SPI4_BYTE_OFFSET_ERROR,
	SPI4_PARAMS_VALID,
	SPI4_CONFIG_SPILL_FAIL,
        SPI4_CONFIG_SPILL_SUCCESS,
        SPI4_CONFIG_PDE_SUCCESS,
	SPI4_REGISTER_MSGRING_FAIL,
        SPI4_REGISTER_MSGRING_SUCESS,
        SPI4_MMIO_ERROR,
        SPI4_OPEN_SUCCESS,

};

#endif

