/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#ifndef _NLH_FMN_H
#define _NLH_FMN_H

#include "nlm_hal.h"
#include "nlm_hal_xlp_dev.h"

#ifndef __ASSEMBLY__

#define FMN_INT_BIT_MASK 0x800000000000000ULL /* Bit 59 */

#define xlp_read_tx_status() _read_32bit_cp2_register(XLP_MSG_TXSTATUS_REG)
#define xlp_read_rx_status() _read_32bit_cp2_register(XLP_MSG_RXSTATUS_REG)

#define xlp_read_status1()   _read_32bit_cp2_register(XLP_MSG_STATUS1_REG)
#define xlp_write_status1(value)   _write_32bit_cp2_register(XLP_MSG_STATUS1_REG, value)

#define xlp_read_config()       _read_32bit_cp2_register(XLP_MSG_CONFIG_REG)
#define xlp_write_config(value) _write_32bit_cp2_register(XLP_MSG_CONFIG_REG, value)

#define xlp_read_msg_int()	_read_32bit_cp2_register(XLP_MSG_INT_REG)
#define xlp_write_msg_int(value)	_write_32bit_cp2_register(XLP_MSG_INT_REG, value)

#define xlp_load_rx_msg0() _read_64bit_cp2_register_sel(XLP_RX_BUF_REG, 0)
#define xlp_load_rx_msg1() _read_64bit_cp2_register_sel(XLP_RX_BUF_REG, 1)
#define xlp_load_rx_msg2() _read_64bit_cp2_register_sel(XLP_RX_BUF_REG, 2)
#define xlp_load_rx_msg3() _read_64bit_cp2_register_sel(XLP_RX_BUF_REG, 3)

#define xlp_load_tx_msg0(value) _write_64bit_cp2_register_sel(XLP_TX_BUF_REG, value, 0)
#define xlp_load_tx_msg1(value) _write_64bit_cp2_register_sel(XLP_TX_BUF_REG, value, 1)
#define xlp_load_tx_msg2(value) _write_64bit_cp2_register_sel(XLP_TX_BUF_REG, value, 2)
#define xlp_load_tx_msg3(value) _write_64bit_cp2_register_sel(XLP_TX_BUF_REG, value, 3)

/* Default number of on-chip 32-message pages per output queue */
#define XLP_FMN_DEFAULT_ONCHIP_PAGES	1
#define XLP_FMN_DEFAULT_QUEUE_SIZE		8192 /* 256K max, must be multiple of 4K */
#define XLP_FMN_DEFAULT_CREDITS			46
#define XLP_FMN_MAX_Q_SIZE				(256ULL * 1024)
#define XLP_FMN_Q_PAGE_SIZE				(4ULL * 1024)
#define XLP_FMN_MAX_ONCHIP_SIZE			1024
#define XLP_FMN_ONCHIP_PAGE_SIZE		32

enum FMN_MSG_BLKS {
	XLP_MSG_BLK_CPU = 0, 
	XLP_MSG_BLK_POPQ, 
	XLP_MSG_BLK_PCIE0,
	XLP_MSG_BLK_PCIE1,
	XLP_MSG_BLK_PCIE2,
	XLP_MSG_BLK_PCIE3,
	XLP_MSG_BLK_GDX,
	XLP_MSG_BLK_REGX,
	XLP_MSG_BLK_RSA_ECC,
	XLP_MSG_BLK_CRYPTO,
	XLP_MSG_BLK_SRIO,
	XLP_MSG_BLK_CMP,
	XLP_MSG_BLK_POE,
	XLP_MSG_BLK_NAE,
	XLP_MSG_BLK_FREEIN,
	XLP_MSG_BLK_MAX
};

/* Added for more options when setting VC interrupt */
enum TMR_INT_TYPES {
	TMR_INT_DISABLE  = 0,
	TMR_INT_CONSUMER = 1,
	TMR_INT_PRODUCER = 2
};

enum TMR_INT_VALUES {
	TMR_2E8_CYCLES  = 0,
	TMR_2E10_CYCLES = 1,
	TMR_2E12_CYCLES = 2,
	TMR_2E14_CYCLES = 3,
	TMR_2E16_CYCLES = 4,
	TMR_2E18_CYCLES = 5,
	TMR_2E20_CYCLES = 6,
	TMR_2E22_CYCLES = 7
};

enum LVL_INT_TYPES {
	LVL_INT_DISABLE = 0,
	LVL_INT_LOW_WM  = 1,
	LVL_INT_HIGH_WM = 2
};

enum LWM_INT_VALUES {
	LWM_EMPTY    = 0,
	LWM_1_4_FULL = 1,
	LWM_1_2_FULL = 2,
	LWM_3_4_FULL = 3,
	LWM_NON_FULL = 4
};

enum HWM_INT_VALUES {
	HWM_NON_EMPTY = 0,
	HWM_1_4_FULL  = 1,
	HWM_1_2_FULL  = 2,
	HWM_3_4_FULL  = 3,
	HWM_FULL      = 4
};

/*
 *  FMN Reg access macros
 */
enum XLP_REGS{
  XLP_OUTQ_CONFIG_REG,
  XLP_CREDIT_CONFIG_REG,
  XLP_INTERCHIP_LINK_CONFIG_REG,
  XLP_ERROR_REG
};

struct fmn_sender
{
	char q_name[8];											// Pretty station name
	unsigned int base_vc;									// Source ID for messages from this station
	unsigned int valid;										// Valid on this silicon
	unsigned int credits[NLM_MAX_NODES][XLP_MSG_BLK_MAX];	// Credits to system-wide receivers
};

struct fmn_receiver
{
	char q_name[8];			// Pretty station group name
	unsigned int b_stid;	// Beginning destination ID
	unsigned int e_stid;	// End destination ID
	unsigned int valid;		// Valid on this silicon
	unsigned int q_size;	// Configured spill queue size in bytes
	unsigned int pages;		// Configured on-chip storage in units of 32 messages
};

struct fmn_cfg {
	uint64_t fmn_spill_base;
	uint64_t fmn_spill_size;
	unsigned int fmn_default_qsize;
	unsigned int fmn_default_credits;
	unsigned int fmn_default_onchip_pages;
	struct fmn_sender senders[XLP_MSG_HANDLE_MAX];
	struct fmn_receiver receivers[XLP_MSG_BLK_MAX];
	/* onchip mem */
	uint32_t q_ram_base_cur;
	/* spill mem */
	uint32_t spill_base_cur;
};

typedef volatile unsigned long long msg_reg_t;

static inline unsigned long long nlh_qid_to_virt_addr(int node, int reg, int sel)
{
  unsigned long long base = xlp_fmn_base[node] & 0xffffffc000ULL;

#if 0 /*defined(NLM_HAL_LINUX_USER) */
  base |= NLH_XKPHYS_UNCACHED;
#endif

  if (reg == XLP_OUTQ_CONFIG_REG) {
    return ((sel * 8) | base); /* 'sel' is the qid */

  } else if (reg == XLP_CREDIT_CONFIG_REG) {
    return (0x2000 | base);

  } else if (reg == XLP_ERROR_REG) {
    return (0x2020 | base);

  } else {
    /*    nlm_print("FMN Error: Unknown register ID %d\n", reg); */
  }
  return 0;
}
/* */

#define nlm_hal_read_outq_config(node, qid) \
	nlh_read_cfg_reg64(nlh_qid_to_virt_addr(node, XLP_OUTQ_CONFIG_REG, qid))

#define nlm_hal_write_outq_config(node, qid, val) \
	nlh_write_cfg_reg64(nlh_qid_to_virt_addr(node, XLP_OUTQ_CONFIG_REG, qid), val)

/*
 *  Messaging Operations 
 */
/**
* @brief xlp_send function is used to send any configured message to a destination, used by the HAL send message API's for different number of messages. Performs a sync before sending.
*
* @param [in]  dest 		:Destination Message Queue number
*
* @return
*  - "1" on  send success, "0" on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_send(unsigned int dest)
{
        unsigned int success = 0;

        __asm__ volatile (".set push\n"
                          ".set noreorder\n"
                          ".set arch=xlp\n"
                          "sync\n"
                          "msgsnds %0, %1\n"
                          ".set pop\n"
                          : "=&r" (success)
                          : "r" (dest));

        return success;
}
/* */
#if 0
static inline void xlp_receive(unsigned int pri)
{
	__asm__ volatile (".set push\n"
			  ".set noreorder\n"
                          ".set arch=xlp\n"
			  "msglds $0, %0\n" ".set pop\n"::"r" (pri)
	    );
}
#endif
/* */
/**
* @brief xlp_message_wait function is a non-blocking API used to wait for the first message to come to a mailbox. 
*
* @param [in]  mask 		:bitmask of the 4 VC's of the CPU, for which queues to monitor for a message
*
* @return
*  - none
* 
* @ingroup hal_fmn
*
*/
static inline void xlp_message_wait(unsigned int mask)
{
    __asm__ volatile(".set push\n"
            ".set noreorder\n"
            " msgwait %0\n"
            ".set pop\n"::"r" (mask)
            );
}
/* */
#define xlp_enable(flags)                        \
do {                                                \
  __asm__ volatile (                                \
		    ".set push\n\t"                 \
		    ".set reorder\n\t"              \
		    ".set noat\n\t"                 \
		    "mfc0 %0, $12\n\t"              \
		    "li  $8, 0x40000001\n\t"        \
		    "or  $1, %0, $8\n\t"            \
		    "xori $1, 1\n\t"                \
		    ".set noreorder\n\t"            \
		    "mtc0 $1, $12\n\t"              \
		    ".set\tpop\n\t"                 \
		    : "=r" (flags)                  \
		    :                               \
		    : "$8"                          \
		    );                              \
} while (0)
/* */
#define xlp_disable(flags) __asm__ volatile (    \
                 "mtc0 %0, $12" : : "r" (flags))

/* */
static __inline__ unsigned long long xlp_cpu_to_bucket_mask(unsigned int
							       cpumask)
{
  return 0;
}
/* */
static __inline__ unsigned int xlp_cpu_to_bucket(int pid)
{
  return 0;
}

/*
   XLP API
   RT[63 : 32] - Reserved
   RT[31 : 24] - Software Code
   RT[23 : 21] - Reserved
   RT[20 : 19] - Pop Message Source VC no.
   RT[18 : 18] - Reserved
   RT[17 : 16] - Message Size-1
   RT[15 : 12] - Reserved
   RT[11 : 0]  - Message Destination ID
 */
/* message send API NON blocking for single entry message*/
/**
* @brief xlp_message_send_1 function is a non-blocking API used to send a single entry message to a mailbox. Will retry the message send 4 times. Performs a sync before sending.
*
* @param [in]  dst		:Destination Message Queue number
* @param [in]  code		:8b SW code to send with the message
* @param [in]  data 	:64b data value for the single message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_send_1(uint32_t dst, uint32_t code,
		uint64_t data)
{
	unsigned int dest;
	int retry;

	xlp_load_tx_msg0(data);
	dest = ((code << 24) | dst);

#ifdef MSGRING_DUMP_MESSAGES
	nlm_hal_dbg_msg("Sending msg<%llx> to dest = %x\n",
			(unsigned long long)data, dest);
#endif

	if(xlp_send(dest))
		return 0;

	retry = 4;
	while(retry--) {
		if(xlp_send(dest))
			return 0;
	}

	return xlp_read_tx_status();
}

/* message send API NON blocking for double entry message*/
/**
* @brief xlp_message_send_2 function is a non-blocking API used to send a two entry message to a mailbox. Will retry the message send 4 times. Performs a sync before sending.
*
* @param [in]  dst		:Destination Message Queue number
* @param [in]  code		:8b SW code to send with the message
* @param [in]  data0 	:64b data value for the first message
* @param [in]  data1 	:64b data value for the second message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_send_2(uint32_t dst, uint32_t code,
		uint64_t data0, uint64_t data1)
{
	unsigned int dest;
	int retry;

	xlp_load_tx_msg0(data0);
	xlp_load_tx_msg1(data1);

	dest = ((code << 24) | (1 << 16) | dst);

#ifdef MSGRING_DUMP_MESSAGES
	nlm_hal_dbg_msg("Sending msg<%llx, %llx> to dest = %x\n", 
			(unsigned long long)data0, (unsigned long long)data1, dest);
#endif

	if(xlp_send(dest))
		return 0;

	retry = 4;
	while(retry--) {
		if(xlp_send(dest))
			return 0;
	}

	return xlp_read_tx_status();
}

/* message send API NON blocking for double entry message*/
/**
* @brief xlp_message_send_3 function is a non-blocking API used to send a three entry message to a mailbox. Does not retry the send message. Performs a sync before sending.
*
* @param [in]  dst		:Destination Message Queue number
* @param [in]  code		:8b SW code to send with the message
* @param [in]  data0 	:64b data value for the first message
* @param [in]  data1 	:64b data value for the second message
* @param [in]  data2 	:64b data value for the third message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_send_3(uint32_t dst, uint32_t code,
		uint64_t data0, uint64_t data1, uint64_t data2)
{
	unsigned int dest = 0;


	xlp_load_tx_msg0(data0);
	xlp_load_tx_msg1(data1);
	xlp_load_tx_msg2(data2);

	dest = ((code << 24) | (2 << 16) | dst);

#ifdef MSGRING_DUMP_MESSAGES
	nlm_hal_dbg_msg("Sending msg<%llx, %llx, %llx> to dest = %x\n", 
			(unsigned long long)data0, (unsigned long long)data1, (unsigned long long)data2, dest);
#endif
	
	if (!xlp_send(dest) ) {
		/* Check the status */
		return xlp_read_tx_status();
	}

	return 0;
}


/* message send API NON blocking for double entry message*/
/**
* @brief xlp_message_send_4 function is a non-blocking API used to send a four entry message to a mailbox. Does not retry the send message. Performs a sync before sending.
*
* @param [in]  dst		:Destination Message Queue number
* @param [in]  code		:8b SW code to send with the message
* @param [in]  data0 	:64b data value for the first message
* @param [in]  data1 	:64b data value for the second message
* @param [in]  data2 	:64b data value for the third message
* @param [in]  data3 	:64b data value for the fourth message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_send_4(uint32_t dst, uint32_t code,
		uint64_t data0, uint64_t data1, uint64_t data2, uint64_t data3)
{
	unsigned int dest = 0;

	xlp_load_tx_msg0(data0);
	xlp_load_tx_msg1(data1);
	xlp_load_tx_msg2(data2);
	xlp_load_tx_msg3(data3);

	dest = ((code << 24) | (2 << 16) | dst);

#ifdef MSGRING_DUMP_MESSAGES
	nlm_hal_dbg_msg("Sending msg<%llx, %llx, %llx, %llx> to dest = %x\n", 
		(unsigned long long)data0, (unsigned long long)data1,
		(unsigned long long)data2, (unsigned long long)data3, dest);
#endif
	
	if (!xlp_send(dest) ) {
		/* Check the status */
		return xlp_read_tx_status();
	}

	return 0;
}

/* Generic message send API NON blocking */
/**
* @brief xlp_message_send function is a non-blocking API for sending a one to four entry message to a mailbox.  Does not retry the send message. Performs a sync before sending.
*
* @param [in]  dst		:Destination Message Queue number
* @param [in]  size		:Number of messages to transmit (1 to 4)
* @param [in]  code		:8b SW code to send with the message
* @param [in]  *data 	:uint64_t array of data[0] to data[3] representing each 64b message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_send(uint32_t dst, uint32_t size, uint32_t code,
		uint64_t *data)
{
	unsigned int dest = 0;

	switch (size) {
	case 4:
		xlp_load_tx_msg3(data[3]);

	case 3:
		xlp_load_tx_msg2(data[2]);

	case 2:
		xlp_load_tx_msg1(data[1]);
          
	default:
		xlp_load_tx_msg0(data[0]);
	}

	dest = ((code << 24) | ((size - 1) << 16) | dst);
 
	if (!xlp_send(dest) ) {
		/* Check the status */
		return xlp_read_tx_status();
	}

	return 0;
}

#if (_MIPS_SIM == _MIPS_SIM_ABI32)
static inline void xlp_message_send_block_fast_1(unsigned int code,
						 unsigned int dest_vc,
						 unsigned long long msg0)
{
	unsigned int high = msg0>>32;
	unsigned int low = msg0 & 0xffffffff;

  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dsll32 $9, %3, 0\n"
			"dsll32 $10, %2, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2  $9, "STR(XLP_TX_BUF_REG)", 0\n"
			/* "dmtc2 %1, "STR(XLP_TX_BUF_REG)", 0\n" */
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"
			:
			: "r"((code << 24) |  dest_vc), /* %0 */
			  "r"(msg0),
			  "r" (high), "r" (low) /* %2, %3 */
			: "$8", "$9", "$10"
			);
}

static inline void xlp_message_send_block_fast_2(unsigned int code,
						 unsigned int dest_vc,
						 unsigned long long msg0,
						 unsigned long long msg1)
{
	unsigned int high0 = msg0>>32;
	unsigned int low0 = msg0 & 0xffffffff;
	unsigned int high1 = msg1>>32;
	unsigned int low1 = msg1 & 0xffffffff;

  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dsll32 $9, %4, 0\n"
			"dsll32 $10, %3, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2  $9, "STR(XLP_TX_BUF_REG)", 0\n"
			"sync\n"
			/* "dmtc2 %1, "STR(XLP_TX_BUF_REG)", 0\n" */
			"dsll32 $9, %6, 0\n"
			"dsll32 $10, %5, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2  $9, "STR(XLP_TX_BUF_REG)", 1\n"
			"sync\n"
			/* "dmtc2 %2, "STR(XLP_TX_BUF_REG)", 1\n" */
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"
			:
			: "r"((code << 24) | (1 << 16) | dest_vc), /* %0 */
			  "r"(msg0), "r" (msg1),
			  "r"(high0), "r"(low0), /* %3, %4 */
			  "r"(high1), "r"(low1)  /* %5, %6 */
			: "$8", "$9", "$10"
			);
}

static inline void xlp_message_send_block_fast_3(unsigned int code,
						 unsigned int dest_vc,
						 unsigned long long msg0,
						 unsigned long long msg1,
						 unsigned long long msg2)
{
	unsigned int high0 = msg0>>32;
	unsigned int low0 = msg0 & 0xffffffff;
	unsigned int high1 = msg1>>32;
	unsigned int low1 = msg1 & 0xffffffff;
	unsigned int high2 = msg2>>32;
	unsigned int low2 = msg2 & 0xffffffff;

  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dsll32 $9, %5, 0\n"
			"dsll32 $10, %4, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2  $9, "STR(XLP_TX_BUF_REG)", 0\n"
			"sync\n"
			/* "dmtc2 %1, "STR(XLP_TX_BUF_REG)", 0\n" */
			"dsll32 $9, %7, 0\n"
			"dsll32 $10,%6, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2  $9, "STR(XLP_TX_BUF_REG)", 1\n"
			"sync\n"
			/* "dmtc2 %2, "STR(XLP_TX_BUF_REG)", 1\n" */
			"dsll32 $9, %9, 0\n"
			"dsll32 $10,%8, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2  $9, "STR(XLP_TX_BUF_REG)", 2\n"
			"sync\n"
			/* "dmtc2 %3, "STR(XLP_TX_BUF_REG)", 2\n" */
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"
			:
			: "r"((code << 24) | (2 << 16) | dest_vc), /* %0 */
			  "r"(msg0), "r" (msg1), "r" (msg2),
			  "r"(high0), "r"(low0), /* %4, %5 */
			  "r"(high1), "r"(low1), /* %6, %7 */
			  "r"(high2), "r"(low2)  /* %8, %9 */
			: "$8", "$9", "$10"
			);
}

static inline void xlp_message_send_block_fast(int size, unsigned int code,
					       unsigned int dest_vc,
					       unsigned long long msg0,
					       unsigned long long msg1,
					       unsigned long long msg2,
					       unsigned long long msg3)
{
	unsigned int high0 = msg0>>32;
	unsigned int low0 = msg0 & 0xffffffff;
	unsigned int high1 = msg1>>32;
	unsigned int low1 = msg1 & 0xffffffff;
	unsigned int high2 = msg2>>32;
	unsigned int low2 = msg2 & 0xffffffff;
	unsigned int high3 = msg3>>32;
	unsigned int low3 = msg3 & 0xffffffff;

  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dsll32 $9, %6, 0\n"
			"dsll32 $10, %5, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2 $9, " STR(XLP_TX_BUF_REG) ", 0\n"
			"sync\n"
			/* "dmtc2 %1, " STR(XLP_TX_BUF_REG) ", 0\n" */
			"dsll32 $9, %8, 0\n"
			"dsll32 $10, %7, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2 $9, " STR(XLP_TX_BUF_REG) ", 1\n"
			"sync\n"
			/* "dmtc2 %2, " STR(XLP_TX_BUF_REG) ", 1\n" */
			"dsll32 $9, %10, 0\n"
			"dsll32 $10, %9, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2 $9, " STR(XLP_TX_BUF_REG) ", 2\n"
			"sync\n"
			/* "dmtc2 %3, " STR(XLP_TX_BUF_REG) ", 2\n" */
			"dsll32 $9, %12, 0\n"
			"dsll32 $10, %11, 0\n"
			"dsrl32 $9, $9, 0\n"
			"or     $9, $9, $10\n"
			"dmtc2 $9, " STR(XLP_TX_BUF_REG) ", 3\n"
			"sync\n"
			/* "dmtc2 %4, " STR(XLP_TX_BUF_REG) ", 3\n" */
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"::
			"r"((code << 24) | ((size-1) << 16) | dest_vc), /* %0 */
			"r"(msg0), "r"(msg1),
			"r"(msg2), "r"(msg3),
			"r"(high0), "r"(low0), /* %5, %6 */
			"r"(high1), "r"(low1), /* %7, %8 */
			"r"(high2), "r"(low2), /* %9, %10 */
			"r"(high3), "r"(low3)  /* %11, %12 */
			:"$8", "$9", "$10");
}

#else

/* API to send a 1 entry message to "stid" with given "code" */
/**
* @brief xlp_message_send_block_fast_1 function is a blocking API for sending a one entry message to a mailbox.  It will continuously retry the send message until successful. Performs a sync before sending.
*
* @param [in]  code		:8b SW code to send with the message
* @param [in]  dest_vc	:Destination Message Queue number
* @param [in]  msg0 	:64b data value for the first message
*
* @return
*  - none
* 
* @ingroup hal_fmn
*
*/

static inline void xlp_message_send_block_fast_1(unsigned int code, 
						 unsigned int dest_vc,
						 unsigned long long msg0)
{
  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dmtc2 %1, "STR(XLP_TX_BUF_REG)", 0\n"
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"
			:
			: "r"((code << 24) |  dest_vc), /* %0 */
			  "r"(msg0) 
			: "$8"
			);
}

/* */
/* API to send a 2 entry message to "stid" with given "code" */
/**
* @brief xlp_message_send_block_fast_2 function is a blocking API for sending a two entry message to a mailbox.  It will continuously retry the send message until successful. Performs a sync before sending.
*
* @param [in]  code		:8b SW code to send with the message
* @param [in]  dest_vc	:Destination Message Queue number
* @param [in]  msg0 	:64b data value for the first message
* @param [in]  msg1 	:64b data value for the second message
*
* @return
*  - none
* 
* @ingroup hal_fmn
*
*/
static inline void xlp_message_send_block_fast_2(unsigned int code, 
						 unsigned int dest_vc,
						 unsigned long long msg0,
						 unsigned long long msg1)
{
  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dmtc2 %1, "STR(XLP_TX_BUF_REG)", 0\n"
			"dmtc2 %2, "STR(XLP_TX_BUF_REG)", 1\n"
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"
			:
			: "r"((code << 24) | (1 << 16) | dest_vc), /* %0 */
			  "r"(msg0), "r" (msg1) 
			: "$8"
			);
}
/* API to send a 3 entry message to "stid" with given "code" */
/**
* @brief xlp_message_send_block_fast_3 function is a blocking API for sending a three entry message to a mailbox.  It will continuously retry the send message until successful. Performs a sync before sending.
*
* @param [in]  code		:8b SW code to send with the message
* @param [in]  dest_vc	:Destination Message Queue number
* @param [in]  msg0 	:64b data value for the first message
* @param [in]  msg1 	:64b data value for the second message
* @param [in]  msg2 	:64b data value for the third message
*
* @return
*  - none
* 
* @ingroup hal_fmn
*
*/
static inline void xlp_message_send_block_fast_3(unsigned int code, 
						 unsigned int dest_vc,
						 unsigned long long msg0,
						 unsigned long long msg1,
						 unsigned long long msg2)
{
  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dmtc2 %1, "STR(XLP_TX_BUF_REG)", 0\n"
			"dmtc2 %2, "STR(XLP_TX_BUF_REG)", 1\n"
			"dmtc2 %3, "STR(XLP_TX_BUF_REG)", 2\n"
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"
			:
			: "r"((code << 24) | (2 << 16) | dest_vc), /* %0 */
			  "r"(msg0), "r" (msg1), "r" (msg2)
			: "$8"
			);
}

/* */
/**
* @brief xlp_message_send_block_fast function is a blocking API for sending a four entry message to a mailbox.  It will continuously retry the send message until successful. Performs a sync before sending.
*
* @param [in]  code		:8b SW code to send with the message
* @param [in]  dest_vc	:Destination Message Queue number
* @param [in]  msg0 	:64b data value for the first message
* @param [in]  msg1 	:64b data value for the second message
* @param [in]  msg2 	:64b data value for the third message
* @param [in]  msg3 	:64b data value for the fourth message
*
* @return
*  - none
* 
* @ingroup hal_fmn
*
*/
static inline void xlp_message_send_block_fast(int size, unsigned int code,
					       unsigned int dest_vc,
					       unsigned long long msg0,
					       unsigned long long msg1,
					       unsigned long long msg2,
					       unsigned long long msg3)
{
  __asm__ __volatile__ (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"dmtc2 %1, " STR(XLP_TX_BUF_REG) ", 0\n"
			"dmtc2 %2, " STR(XLP_TX_BUF_REG) ", 1\n"
			"dmtc2 %3, " STR(XLP_TX_BUF_REG) ", 2\n"
			"dmtc2 %4, " STR(XLP_TX_BUF_REG) ", 3\n"
			"sync\n"
			"1: \n"
			"msgsnds  $8, %0\n"    /* msgsnds rD, rt */
			"andi $8, $8, 0x1\n"
			"beqz $8, 1b\n"
			"move $8, %0\n"
			".set mips64\n"
			".set pop\n"::
			"r"((code << 24) | ((size-1) << 16) | dest_vc), /* %0 */
			"r"(msg0), "r"(msg1),
			"r"(msg2), "r"(msg3)
			:"$8");
}

#endif

/* */
/**
* @brief xlp_receive function is used to receive message from a mailbox vc, used by the HAL receive message API's for different number of messages
*
* @param [in]  vc 		:VC mailbox of the CPU (1 to 4)
*
* @return
*  - "1" on load success, "0" on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_receive(unsigned int vc)
{
	unsigned int success = 0;

	__asm__ volatile (".set push\n"
	                  ".set noreorder\n"
	                  ".set arch=xlp\n"
	                  "msglds %0, %1\n"
	                  ".set pop\n"
	                  : "=&r" (success)
	                  : "r" (vc));

	return success;
}

/* */
/**
* @brief xlp_message_receive_1 function is used to receive a single entry message from a VC of the CPU. Size should be used to determine how other 64b messages were available with data.
*
* @param [in]  vc 		:VC mailbox of the CPU (0 to 3)
* @param [out]  src_id	:Source Message Queue Number
* @param [out]  size	:# of messages returned (1 to 4)
* @param [out]  code	:8b SW code of the received message
* @param [out]  msg0 	:64b data value for the received message
*
* @return
*  - "0" on receive success, "-1" on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_receive_1(uint32_t vc, uint32_t *src_id,
	uint32_t *size, uint32_t *code,	uint64_t *msg0)
{
	unsigned int status;

	if (!xlp_receive(vc))
		return -1;

	status = xlp_read_rx_status();
	*size = ((status >> 26) & 0x3) + 1;
	*code = (status >> 18) & 0xff;
	*src_id = (status >> 4) & 0xfff;
	*msg0 = xlp_load_rx_msg0();
	return 0;
}
/**
* @brief xlp_message_receive_2 function is used to receive a single entry message from a VC of the CPU. Size should be used to determine how many of msg0-msg1 have valid data and if there were more messages available.
*
* @param [in]  vc 		:VC mailbox of the CPU (0 to 3)
* @param [out]  src_id	:Source Message Queue Number
* @param [out]  size	:# of messages that were in this received message (1 to 4)
* @param [out]  code	:8b SW code of the received message
* @param [out]  msg0 	:64b data value for the first received message
* @param [out]  msg1 	:64b data value for the second received message
*
* @return
*  - "0" on receive success, "-1" on failure, "1" on load failure, "2" on pop failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_receive_2(uint32_t vc, uint32_t *src_id,
	uint32_t *size, uint32_t *code, uint64_t *msg0, uint64_t *msg1)
{
	unsigned int status;

	if (!xlp_receive(vc))
		return -1;

	status = xlp_read_rx_status();
	*size = ((status >> 26) & 0x3) + 1;
	*code = (status >> 18) & 0xff;
	*src_id = (status >> 4) & 0xfff;
	*msg0 = xlp_load_rx_msg0();
	*msg1 = xlp_load_rx_msg1();
	return (status & 0x3);
}
/* */
/**
* @brief xlp_message_receive function is used to receive a four entry message from a VC of the CPU.  Size should be used to determine how many of msg0-msg3 have valid data.
*
* @param [in]  vc 		:VC mailbox of the CPU (0 to 3)
* @param [out]  src_id	:Source Message Queue Number
* @param [out]  size	:# of messages that were in this received message (1 to 4)
* @param [out]  code	:8b SW code of the received message
* @param [out]  msg0 	:64b data value for the first received message
* @param [out]  msg1 	:64b data value for the second received message
* @param [out]  msg2 	:64b data value for the third received message
* @param [out]  msg3 	:64b data value for the fourth received message
*
* @return
*  - "0" on receive success, "-1" on failure
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_receive(uint32_t vc, uint32_t *src_id,
	uint32_t *size, uint32_t *code, uint64_t *msg0, uint64_t *msg1,
	uint64_t *msg2, uint64_t *msg3)
{
	unsigned int status;

	if (!xlp_receive(vc))
		return -1;

	status = xlp_read_rx_status();
	*size = ((status >> 26) & 0x3) + 1;
	*code = (status >> 18) & 0xff;
	*src_id = (status >> 4) & 0xfff;
	*msg0 = xlp_load_rx_msg0();
	*msg1 = xlp_load_rx_msg1();
	*msg2 = xlp_load_rx_msg2();
	*msg3 = xlp_load_rx_msg3();
	return 0;
}

/* 
   *
   * This API can be used for both enabling and disabling the POP operation 
   * in the msgconfig register.  
   * 
   * In vc_mask:
   * '0' means disable  and 
   * '1' means enable.
   *
   * bit0 = vc0, bit1 = vc1, bit2 = vc2, bit3 = vc3
   *
   */
static inline int nlm_hal_pop_cfg_update (uint32_t vc_mask)
{
	uint32_t vc, config; 

	/* mask out the other bits in vc_mask, just to be safe */
	vc_mask &= 0x0f;
	vc = (vc_mask << 1);

	config = xlp_read_config();
	config &= ~(0x1e);
	config |= vc;
	xlp_write_config(config);

	return 0;
}

static inline int nlm_hal_pop_send (uint32_t popq)
{
	int rc = 0;

	if (!xlp_send(popq) )
	{
		/* Check the status */
		rc = xlp_read_tx_status();
	}

	return rc;
}

/* */
/* Generic Messaging API */
/* */
/**
* @brief xlp_message_send_block function is a blocking API for sending a one to four entry message to a mailbox.  It will continuously retry the send message until successful. Performs a sync before sending.
*
* @param [in]  size		:# of 64b messages to be sent (1 to 4)
* @param [in]  code		:8b SW code to send with the message
* @param [in]  stid		:Destination Message Queue number
* @param [in]  data0 	:64b data value for the first message
* @param [in]  data1 	:64b data value for the second message, if size < 2 can be any value
* @param [in]  data2 	:64b data value for the third message, if size < 3 can be any value
* @param [in]  data3 	:64b data value for the fourth message, if size < 4 can be any value
*
* @return
*  - none
* 
* @ingroup hal_fmn
*
*/
static inline int xlp_message_send_block(unsigned int size, unsigned int code,
					 unsigned int stid, uint64_t data0, uint64_t data1,
					 uint64_t data2, uint64_t data3)
{
    xlp_message_send_block_fast(size, code, stid, data0, data1, data2, data3);
    return 0;
}
/* */

#endif				/* __ASSEMBLY__ */

/* Returns the TxStatus reg, if unsuccessful, 0 if success */
extern uint32_t nlm_hal_send_msg4(uint32_t dst, uint32_t code, uint64_t data0, uint64_t data1, uint64_t data2, uint64_t data3);
extern uint32_t nlm_hal_send_msg3(uint32_t dst, uint32_t code, uint64_t data0, uint64_t data1, uint64_t data2);
extern uint32_t nlm_hal_send_msg2(uint32_t dst, uint32_t code, uint64_t data0, uint64_t data1);
extern uint32_t nlm_hal_send_msg1(uint32_t dst, uint32_t code, uint64_t data0);
/* Returns the RxStatus reg */
extern uint32_t nlm_hal_recv_msg2(uint32_t dst, uint32_t *src, uint32_t *size, uint32_t *code, uint64_t *data0, uint64_t *data1);
extern uint32_t nlm_hal_recv_msg1(uint32_t dst, uint32_t *src, uint32_t *size, uint32_t *code, uint64_t *data0);

static __inline__ int fmn_level_int_type(uint64_t outq_config)
{
	return xlp_get_field_dw(outq_config, 54, 2);
}

static __inline__ int fmn_level_int_val(uint64_t outq_config)
{
	return xlp_get_field_dw(outq_config, 56, 3);
}

extern int get_dom_fmn_node_ownership(void *fdt, int dom_id);
extern void nlm_hal_fmn_init(void *fdt, int node);
extern void nlm_hal_set_fmn_interrupt(int irq);

extern void nlm_hal_disable_vc_intr(int node, int vc);
extern void nlm_hal_enable_vc_intr(int node, int vc,
		int level_type, int thresh_val, int tmr_type, int tmr_val);

#endif /* #ifndef _NLH_FMN_H */
