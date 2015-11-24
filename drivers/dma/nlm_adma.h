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


#ifndef __NLM_ADMA_H__

#define __NLM_ADMA_H__

#include <linux/interrupt.h>
#include <linux/dmaengine.h>

#define DTRE_MAX_TX_DESC_PER_CHAN 	4096
#define DTRE_MAX_TX_Q_LEN		(DTRE_MAX_TX_DESC_PER_CHAN *2)
#define DTRE_MAX_TX_Q_MASK		(DTRE_MAX_TX_Q_LEN - 1)
#define DTRE_MAX_LIST_LENGTH 		24
#define DTRE_RAID_LIST_MSG_SIZE 	3
#define DTRE_TRANSFER_MSG_SIZE		4
#define DTRE_RAID_MESSAGE_TYPE		6
#define DTRE_RAID_ENTRY_SIZE		8
#define DTRE_RAID_MAX_ENTRIES		384
#define DTRE_RAID_MAX_DEVICES		24
#define DTRE_RAID_MAX_SRC		16
#define DTRE_NLM_Q_POLY			0x1d

#define DTRE_MAX_MEMCPY_SIZE		((1*1024*1024) - 1)  /* 1 MB */
#define DTRE_MAX_MEMCPY_DESC_LIST	4
#define DTRE_MAX_MEMCPY_DESC_SIZE	(DTRE_MAX_MEMCPY_DESC_LIST*DTRE_MAX_MEMCPY_SIZE)

#define DTRE_NUM_VC		4
#define DTRE_MIN_VC		264
#define DTRE_MAX_VC		267
#define DTRE_NUM_CHANNELS	4



struct nlm_adma_device {
	struct platform_device *pdev;
	struct dma_device common;
	spinlock_t lock;
};

struct nlm_hw_desc {
	uint64_t entries [DTRE_RAID_MAX_ENTRIES] __attribute__((__aligned__(64)));
	uint64_t src;
	uint64_t dst;
	uint32_t src_cnt;
};

struct nlm_tx_desc {
	/* DTRE_MAX_MEMCPY_DESC_LIST(4) descs, each of 4 entry */
	uint64_t list_desc[DTRE_MAX_MEMCPY_DESC_LIST*4] ____cacheline_aligned; 

	struct nlm_tx_desc * next;
	struct dma_async_tx_descriptor async_tx;
	enum dma_transaction_type optype;
	unsigned long len;
	int     int_en;
	struct nlm_hw_desc hw_desc;
	unsigned int entries_count;
	int done_flag;
	u32 *xor_check_result;
	void    *chan; /* backpointer to channel */
	struct page * pg;
	int memset_val;
};

struct nlm_adma_chan {
	int chan_num; /* channel number */
	dma_cookie_t completed_cookie;
	spinlock_t lock; /* protects the descriptor slot pool */
	struct nlm_adma_device *device;
	struct dma_chan common;
	struct nlm_tx_desc desc_pool_head; /* Free TX descriptors are linked here */
	int num_tx_desc;
	uint64_t alloc_desc;
	struct tasklet_struct irq_tasklet;

	/* transactions are queued here */
	struct nlm_tx_desc * tx_queue[DTRE_MAX_TX_Q_LEN];
	int pending_idx, process_idx;
};

enum nlm_raid_type {
	raid5 = 0x1,
	raid6 = 0x2
};

enum nlm_write_syndrome {
	readop = 0,
	writeop = 1
};

enum nlm_message_type {
	p2p = 3,
	p2d = 4
};

#define to_nlm_adma_chan(chan) container_of(chan, struct nlm_adma_chan, common)
#define to_nlm_adma_device(dev) \
	container_of(dev, struct nlm_adma_device, common)
#define tx_to_nlm_adma_tx(tx) container_of(tx, struct nlm_tx_desc, async_tx)

#endif
