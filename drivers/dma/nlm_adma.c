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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/async_tx.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/timer.h>

#include <asm/netlogic/mips-exts.h>

#include "nlm_adma.h"

#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/netlogic/hal/nlm_hal_fmn.h>

uint64_t nlm_dtre_debug = 0;

#define shift_lower_bits(x, bitshift, numofbits) \
	(((unsigned long long)(x) & ((1ULL << (numofbits)) - 1)) << (bitshift))

extern void nlm_hal_dtr_init(void);

struct nlm_adma_device nlm_adma_raid_device;
struct page * nlm_dtre_null_page;

#define CACHELINE_ALIGNED_ADDR(addr) (((unsigned long)(addr)) & ~(SMP_CACHE_BYTES-1))

static __inline__ void *cacheline_aligned_kzalloc(int size, int gfp_mask)
{
	void *buf = kzalloc(size + SMP_CACHE_BYTES, gfp_mask);
	if (buf)
		buf =
			(void
			 *)(CACHELINE_ALIGNED_ADDR((unsigned long)buf +
					 SMP_CACHE_BYTES));
	return buf;
}


static inline int nlm_adma_get_max_xor(void)
{
	/*
	   * The final device in a RAID-5 list may be read to check the 
	   * syndrome, or written with the computed syndrome, according to the 
	   * WriteSyndrome bit
	   */
	return (DTRE_MAX_LIST_LENGTH-1);
}

static inline int nlm_adma_get_max_pq(void)
{
	/*
	   * A RAID-6 list behaves similarly, except that the P syndrome 
	   * is read from or written to the second to final device, and the 
	   * Q syndrome is read from or written to the final device
	   */
	return (DTRE_MAX_LIST_LENGTH-2);
}

static __inline__
uint64_t gen_dtr_raid_msg_format_0 (const uint32_t src_cnt,
		const uint64_t* src_addr)
{
	return 1ULL << 63
		| shift_lower_bits (DTRE_RAID_MESSAGE_TYPE, 60, 3)
		| shift_lower_bits ((DTRE_RAID_ENTRY_SIZE * src_cnt), 40, 12)
		| shift_lower_bits (virt_to_phys ((volatile void *)src_addr), 0, 40);
}

static __inline__
uint64_t gen_dtr_raid_msg_format_1 (const uint32_t raid_type,
		const uint32_t operation,
		const uint32_t disks,
		const uint32_t freeback_msg_dest_id)
{
	return 0ULL << 63
		| shift_lower_bits (1, 56, 1) /* Inform Source */
		| shift_lower_bits (freeback_msg_dest_id, 44, 12)
		| shift_lower_bits (DTRE_NLM_Q_POLY, 12, 8) /* Q polynomial*/
		| shift_lower_bits (raid_type, 10, 2)
		| shift_lower_bits (operation, 9, 1)
		| shift_lower_bits (disks, 0, 5);
}

static __inline__
uint64_t gen_dtr_raid_msg_format_2 (const void * ret_entry)
{
	return 0ULL << 63
		| shift_lower_bits ((unsigned long)ret_entry, 0, 63);
}


static __inline__
uint64_t gen_dtr_xfer_msg_format_0 (const unsigned long len, const uint64_t src)
{
	return (1ULL << 63)
		| shift_lower_bits(0, 60, 3)  /* msgtype 0 for transfer */
		| shift_lower_bits(len, 40, 20)
		| shift_lower_bits ((volatile void *)src, 0, 40);
}

static __inline__
uint64_t gen_dtr_xfer_msg_format_1 (const uint32_t dest_id)
{
	return (0ULL << 63)
		| shift_lower_bits (1, 59, 1) /* write control */
		| shift_lower_bits (1, 56, 1) /* Inform Source */
		| shift_lower_bits (dest_id, 44, 12);
}

static __inline__
uint64_t gen_dtr_xfer_msg_format_2 (const uint64_t dst)
{
	return (0ULL << 63)
		| shift_lower_bits (1, 40, 1) /* perform transfer */
		| shift_lower_bits ((volatile void *)dst, 0, 40);
}

static __inline__
uint64_t gen_dtr_xfer_msg_format_3 (const void * ret_entry)
{
	return 0ULL << 63
		| shift_lower_bits ((unsigned long)ret_entry, 0, 63);
}

static __inline__
uint64_t gen_dtr_xfer_p2pmsg_format_0 (const uint32_t len, const uint64_t src)
{
	return (1ULL << 63)
		| shift_lower_bits (7, 60, 3) /* msgtype 7 for p2p */
		| shift_lower_bits (len, 40, 20)
		| shift_lower_bits ((volatile void *)src, 0, 40);
}

static __inline__
uint64_t gen_dtr_xfer_p2pmsg_format_1 (const uint32_t dest_id)
{
	return (0ULL << 63)
		| shift_lower_bits (0, 62, 1)  /* i/o data interconnect */
		| shift_lower_bits (1, 56, 1) /* Inform Source */
		| shift_lower_bits (dest_id, 44, 12);
}

static __inline__
uint64_t gen_dtr_xfer_p2pmsg_format_2 (const void * ret_entry)
{
	return 0ULL << 63
		| shift_lower_bits ((unsigned long)ret_entry, 0, 63);
}



static void nlm_dtre_msgring_handler(uint32_t vc, uint32_t src_id,
		uint32_t size, uint32_t code,
		uint64_t msg0, uint64_t msg1,
		uint64_t msg2, uint64_t msg3, 
		void* data)
{       

	struct nlm_tx_desc *desc;
	struct nlm_adma_chan *nlm_chan ;
	uint64_t addr = (unsigned long) (msg1<<1);


	desc = (struct nlm_tx_desc *)addr;

	if (desc == NULL)
	{
		printk("NULL msg in nlm_dtre_msgring_handler..\n");
		return;
	}

	if (nlm_dtre_debug) 
	{
		printk("DTRE recv msg: vc %d sender 0x%x, size 0x%x, data0 0x%llx data1 0x%llx optype %d.\n",
				vc, src_id, size, msg0, msg1, desc->optype);
	}

	if ((desc->xor_check_result) != NULL)
		*(desc->xor_check_result) = 0;

	/* save the DMA_XOR_VAL and DMA_PQ_VAL operation result */
	/* update if check failed */
	if ((desc->optype == DMA_XOR_VAL) | (desc->optype == DMA_PQ_VAL))
	{
		if ( ((msg0>>42) & 0x3) == 0x1)
			*(desc->xor_check_result) = (1<<SUM_CHECK_P);
	}

	if (desc->optype == DMA_PQ_VAL)
	{
		if ( ((msg0>>44) & 0x3) == 0x1) 
			(*(desc->xor_check_result)) |= (1<<SUM_CHECK_Q);
	}

	nlm_chan = (struct nlm_adma_chan *)(desc->chan);

	desc->done_flag = 1;
	wmb();

	tasklet_schedule(&nlm_chan->irq_tasklet);
}               

static dma_cookie_t
nlm_desc_assign_cookie(struct nlm_adma_chan *chan,
		struct nlm_tx_desc *desc)
{
	dma_cookie_t cookie = chan->common.cookie;
	cookie++;
	if (cookie < 0)
		cookie = 1;
	chan->common.cookie = desc->async_tx.cookie = cookie;
	return cookie;
}

/* Called with chan->lock held */
static void free_tx_desc(struct nlm_tx_desc *desc)
{
	struct nlm_adma_chan *chan = (struct nlm_adma_chan *)desc->chan;

	/* return the descriptor back to the channel's pool */
	desc->next = chan->desc_pool_head.next;
	chan->desc_pool_head.next = desc;
}


/* Called with chan->lock held */
static void __process_completed_tx(struct nlm_adma_chan * chan)
{
	struct nlm_tx_desc *ptr;
	dma_cookie_t cookie = 0;
	int i;

	if (chan->process_idx == chan->pending_idx)
	{
		/* nothing to process */

		for (i=0; i<DTRE_MAX_TX_Q_LEN; i++)
		{
			if (chan->tx_queue[i] != NULL)
			{
				printk("DTRE index %d found, expected to be empty\n", i);
			}
		}

		return;
	}

	while (1)
	{
		ptr = chan->tx_queue[chan->process_idx];

		if ((ptr->async_tx.cookie > 0) && (ptr->done_flag == 1))
		{
			if (nlm_dtre_debug)
				printk("* process_idx: %d.\n", chan->process_idx);
			cookie = ptr->async_tx.cookie;
			ptr->async_tx.cookie = 0;
			ptr->done_flag = 2;
			wmb();

			if (ptr->async_tx.callback)
			{
				ptr->async_tx.callback(ptr->async_tx.callback_param);
			}

			chan->completed_cookie = cookie;

			dma_run_dependencies(&ptr->async_tx);
		}

		if (!async_tx_test_ack(&ptr->async_tx))
		{
			tasklet_schedule(&chan->irq_tasklet);
			break;
		}

		if (ptr->done_flag == 2)
		{
			if (nlm_dtre_debug)
				printk("pro_idx: %d.\n", chan->process_idx);

			ptr->done_flag = 0;
			wmb();
			chan->tx_queue[chan->process_idx] = NULL;
			chan->process_idx = (chan->process_idx + 1) & DTRE_MAX_TX_Q_MASK;
			if (ptr->pg)
				put_page(ptr->pg);

			free_tx_desc(ptr);
		}

		if (chan->process_idx == chan->pending_idx)
		{
			for (i=0; i<DTRE_MAX_TX_Q_LEN; i++)
			{
				if (chan->tx_queue[i] != NULL)
				{
					printk("DTRE idx %d found, expected to be empty\n", i);
				}
			}
			break;
		}

		/* exit clause */
		if ((chan->process_idx != chan->pending_idx) &&
				(ptr != NULL) &&
				(ptr->done_flag == 0))
		{
			tasklet_schedule(&chan->irq_tasklet);
			break;
		}
	}
}

static void dtre_send_message(int vc_id, int msgtype, uint64_t * msg)
{
	uint64_t pop_data[4];
	uint32_t pop_vc, pop_src, pop_size, pop_code;
	uint32_t freeback_msg_dest_id, msgstatus1;
	int rc, i;

	freeback_msg_dest_id = (netlogic_cpu_id()*16)+(netlogic_thr_id()*4);
	rc = 0;
	i = 0;


	while(1)
	{
		rc = xlp_message_send(vc_id, msgtype, 0, msg);
		if (rc == 0)
			break;

		/* pop out messages from vc, if any */
		pop_vc = freeback_msg_dest_id;

		if ((nlm_hal_recv_msg2(pop_vc, &pop_src, &pop_size, &pop_code, &pop_data[0], &pop_data[1])) == 0)
		{
			if (nlm_dtre_debug)
				printk("POP msg found in vc.\n");
			nlm_dtre_msgring_handler(pop_vc, pop_src, pop_size, pop_code, pop_data[0], pop_data[1], pop_data[2], pop_data[3], NULL);
		}

		if ((i%100) == 0) {
			msgstatus1 = xlp_read_status1();
			// printk("DTRE:continuing retry, rc:0x%08x, msgstatus1: 0x%08x.\n", rc, msgstatus1);
		}
	}

	if (rc != 0) {
		msgstatus1 = xlp_read_status1();
		printk("Error: unable to send DTRE Xfer msg: rc:%d, msgstatus1: 0x%08x.\n", rc, msgstatus1);
		return;
	}
	else {
		if (nlm_dtre_debug) 
		{
			printk("sent msg 0x%llx 0x%llx 0x%llx to %d.\n",
					msg[0], msg[1], msg[2], vc_id);
		}
	}
}


static dma_cookie_t nlm_tx_submit (struct dma_async_tx_descriptor *tx)
{
	enum dma_transaction_type optype;
	int vc_id;
	uint32_t disks, freeback_msg_dest_id;
	dma_cookie_t cookie;
	int i, index, loopout, j=0;

	enum nlm_raid_type raid_type;
	enum nlm_write_syndrome operation;
	enum nlm_message_type msgtype;

	struct nlm_tx_desc *nlm_tx ;
	struct nlm_adma_chan *chan ;

	uint64_t raid_list_msg[DTRE_RAID_LIST_MSG_SIZE] = { 0ULL };
	uint64_t transfer_msg[DTRE_TRANSFER_MSG_SIZE] = { 0ULL };

	unsigned long msg_len, desc_len;

	disks = 0;
	nlm_tx = tx_to_nlm_adma_tx(tx);
	chan = (struct nlm_adma_chan *)(nlm_tx->chan);

	/* update dest vc_id from channel number */
	vc_id = DTRE_MIN_VC + chan->chan_num;

	spin_lock_bh(&chan->lock);
	cookie = nlm_desc_assign_cookie(chan, nlm_tx);

	optype = nlm_tx->optype;
	freeback_msg_dest_id = (netlogic_cpu_id()*16)+(netlogic_thr_id()*4);

	if ((chan->tx_queue[chan->pending_idx]) != NULL)
	{
		/* no space in tx array, try to cleanup by processing */
		__process_completed_tx(chan);
	}

	chan->tx_queue[chan->pending_idx] = nlm_tx;

	if (nlm_dtre_debug)
		printk("pending_idx: %d.\n", chan->pending_idx);

	chan->pending_idx = (chan->pending_idx + 1) & DTRE_MAX_TX_Q_MASK;

	spin_unlock_bh(&chan->lock);

	/* handle size restrictions first, in case of 
	   DMA_MEMSET and DMA_MEMCPY */

	if (optype == DMA_MEMSET)
	{
		if ((nlm_tx->len > PAGE_SIZE) || (nlm_tx->memset_val != 0))
		{
			memset(page_address(pfn_to_page((nlm_tx->hw_desc.dst)>>PAGE_SHIFT)), nlm_tx->memset_val, nlm_tx->len);
			nlm_tx->done_flag = 1;
			wmb();
			tasklet_schedule(&chan->irq_tasklet);
			return cookie;
		}
	}

	if (optype == DMA_MEMCPY)
	{
		/* handle in CPU */
		if (nlm_tx->len >= DTRE_MAX_MEMCPY_DESC_SIZE)
		{
			memcpy(page_address(pfn_to_page((nlm_tx->hw_desc.dst)>>PAGE_SHIFT)), page_address(pfn_to_page((nlm_tx->hw_desc.src)>>PAGE_SHIFT)), nlm_tx->len);
			nlm_tx->done_flag = 1;
			wmb();
			tasklet_schedule(&chan->irq_tasklet);

			return cookie;
		}

		/* need to send list of descriptors in this case */
		if ((nlm_tx->len > DTRE_MAX_MEMCPY_SIZE) &&
				(nlm_tx->len <= DTRE_MAX_MEMCPY_DESC_SIZE))
		{
			msg_len = nlm_tx->len;

			/* 8 bytes per entry * 4 entry msg */
			memset(nlm_tx->list_desc, 0, (8*4*DTRE_MAX_MEMCPY_DESC_LIST));

			/* construct a list of msgtype 0 (msg transfer) */
			loopout = 0;
			for (i=0; i<DTRE_MAX_MEMCPY_DESC_LIST; i++)
			{
				index = i*4;  /* 4 entry msg */

				if (msg_len > DTRE_MAX_MEMCPY_SIZE) {
					desc_len = DTRE_MAX_MEMCPY_SIZE;
					msg_len -= DTRE_MAX_MEMCPY_SIZE;
				}
				else {
					desc_len = msg_len;
					loopout = 1;
				}


				/* This is a 4-entry msg */
				nlm_tx->list_desc[index+0] = gen_dtr_xfer_msg_format_0(desc_len, nlm_tx->hw_desc.src);
				nlm_tx->list_desc[index+1] = gen_dtr_xfer_msg_format_1(freeback_msg_dest_id);
				nlm_tx->list_desc[index+2] = gen_dtr_xfer_msg_format_2(nlm_tx->hw_desc.dst);
				nlm_tx->list_desc[index+3] = gen_dtr_xfer_msg_format_3((void *)((unsigned long)nlm_tx>>1));

				if (nlm_dtre_debug)
				{
					printk("i %d, len 0x%lx, msg_len 0x%lx.\n", i, desc_len, msg_len);
					printk("0x%llx, 0x%llx, 0x%llx, 0x%llx.\n",
							nlm_tx->list_desc[index+0],
							nlm_tx->list_desc[index+1],
							nlm_tx->list_desc[index+2],
							nlm_tx->list_desc[index+3]);
				}

				if (loopout == 1)
					break;

				/* increment src and dest */
				nlm_tx->hw_desc.src += desc_len;
				nlm_tx->hw_desc.dst += desc_len;

			}

#ifdef CONFIG_CPU_LITTLE_ENDIAN
			for (j=0; j<(DTRE_MAX_MEMCPY_DESC_LIST*4); j++)
				nlm_tx->list_desc[j] = __swab64(nlm_tx->list_desc[j]);

#endif

			/* construct msgtype 7 (p2p) */
			/* 8 bytes per entry * 4 entry msg */
			raid_list_msg[0] = gen_dtr_xfer_p2pmsg_format_0((i+1)*8*4, (unsigned long)(virt_to_phys((volatile void*)nlm_tx->list_desc)));
			raid_list_msg[1] = gen_dtr_xfer_p2pmsg_format_1(freeback_msg_dest_id);
			raid_list_msg[2] = gen_dtr_xfer_p2pmsg_format_2((void *)((unsigned long)nlm_tx>>1));

			if (nlm_dtre_debug)
			{
				printk("msg7: 0x%llx, 0x%llx, 0x%llx.\n",
						raid_list_msg[0],
						raid_list_msg[1],
						raid_list_msg[2]);
			}

			/* call the send msg function */
			msgtype = p2p;
			dtre_send_message(vc_id, msgtype, raid_list_msg);

			return cookie;
		}
	}


	/* Handle the DMA_MEMSET and DMA_MEMCPY cases first and return.
	   All RAID messages are handled later 
	   */

	if ((optype == DMA_MEMCPY) || (optype == DMA_MEMSET))
	{
		transfer_msg[0] = gen_dtr_xfer_msg_format_0(nlm_tx->len, nlm_tx->hw_desc.src); 
		transfer_msg[1] = gen_dtr_xfer_msg_format_1(freeback_msg_dest_id); 
		transfer_msg[2] = gen_dtr_xfer_msg_format_2(nlm_tx->hw_desc.dst);
		transfer_msg[3] = gen_dtr_xfer_msg_format_3((void *)((unsigned long)nlm_tx>>1));

		msgtype = p2d;
		dtre_send_message(vc_id, msgtype, transfer_msg);

		return cookie;
	}

	if (optype == DMA_XOR)
	{
		raid_type = raid5;  	// raid5
		operation = writeop;  	// WRITE operation
		msgtype = p2p;    	// P2P msg type
		disks = (nlm_tx->hw_desc.src_cnt) + 1;
	}

	if (optype == DMA_XOR_VAL)
	{
		raid_type = raid5;  	// raid5
		operation = readop;  	// READ operation
		msgtype = p2p;    	// P2P msg type
		disks = (nlm_tx->hw_desc.src_cnt);
	}

	if (optype == DMA_PQ)
	{
		raid_type = raid6;  	// raid6
		operation = writeop;  	// WRITE operation
		msgtype = p2p;    	// P2P msg type
		disks = (nlm_tx->hw_desc.src_cnt) + 2;
	}

	if (optype == DMA_PQ_VAL)
	{
		raid_type = raid6;  	// raid6
		operation = readop;  	// READ operation
		msgtype = p2p;    	// P2P msg type
		disks = (nlm_tx->hw_desc.src_cnt) + 2;
	}

#ifdef CONFIG_CPU_LITTLE_ENDIAN
	for (j=0; j<nlm_tx->entries_count; j++)
		nlm_tx->hw_desc.entries[j] = __swab64(nlm_tx->hw_desc.entries[j]);
#endif

	raid_list_msg [0] = gen_dtr_raid_msg_format_0((nlm_tx->entries_count), nlm_tx->hw_desc.entries);
	raid_list_msg [1] = gen_dtr_raid_msg_format_1(raid_type, operation, disks, freeback_msg_dest_id);
	raid_list_msg [2] = gen_dtr_raid_msg_format_2((void *)((unsigned long)nlm_tx>>1));

	dtre_send_message(vc_id, msgtype, raid_list_msg);

	return cookie;

}


/* returns the actual number of allocated descriptors */
static int nlm_adma_alloc_chan_resources(struct dma_chan *chan)
{
	struct nlm_adma_chan * nlm_chan = to_nlm_adma_chan(chan);

	return (nlm_chan->num_tx_desc);
}



static struct nlm_tx_desc *alloc_tx_desc(struct nlm_adma_chan *chan)
{
	struct nlm_tx_desc *ptr;

	spin_lock_bh(&chan->lock);
	ptr = chan->desc_pool_head.next;
	if(ptr != NULL) {
		chan->desc_pool_head.next = ptr->next;
		ptr->next = NULL;
		chan->alloc_desc++;
		spin_unlock_bh(&chan->lock);
		return ptr;
	} else {
		/* dynamic allocation */
		ptr = cacheline_aligned_kzalloc(sizeof(struct nlm_tx_desc), GFP_KERNEL);
		if(!ptr) {
			spin_unlock_bh(&chan->lock);
			panic("OUT OF TX descriptors");
		}
		dma_async_tx_descriptor_init(&ptr->async_tx, &chan->common);
		ptr->async_tx.tx_submit = nlm_tx_submit;
		ptr->chan = (void *)chan;
		chan->alloc_desc++;
		spin_unlock_bh(&chan->lock);
		return ptr;
	}

}


static void process_completed_tx(struct nlm_adma_chan * nlm_chan)
{
	spin_lock_bh(&nlm_chan->lock);
	__process_completed_tx(nlm_chan);
	spin_unlock_bh(&nlm_chan->lock);
}

static void nlm_adma_tasklet(unsigned long data)
{
	struct nlm_adma_chan *nlm_chan = (struct nlm_adma_chan *) data;

	spin_lock(&nlm_chan->lock);
	__process_completed_tx(nlm_chan);
	spin_unlock(&nlm_chan->lock);
}

static void nlm_adma_free_chan_resources(struct dma_chan *chan)
{
	struct nlm_adma_chan * nlm_chan = to_nlm_adma_chan(chan);
	process_completed_tx(nlm_chan);
}


/**
 * nlm_adma_is_complete - poll the status of an ADMA transaction
 * @chan: ADMA channel handle
 * @cookie: ADMA transaction identifier
 */
static enum dma_status nlm_adma_is_complete(struct dma_chan *chan,
		dma_cookie_t cookie, 
		dma_cookie_t *done, 
		dma_cookie_t *used)
{
	struct nlm_adma_chan *nlm_chan = to_nlm_adma_chan(chan);
	dma_cookie_t last_used;
	dma_cookie_t last_complete;
	enum dma_status ret;

	rmb();
	last_used = chan->cookie;
	last_complete = nlm_chan->completed_cookie;

	if (done)
		*done = last_complete;

	if (used)
		*used = last_used;

	ret = dma_async_is_complete(cookie, last_complete, last_used);
	if (ret == DMA_SUCCESS)
		return ret;

	process_completed_tx(nlm_chan);

	rmb();
	last_used = chan->cookie;
	last_complete = nlm_chan->completed_cookie;

	if (done)
		*done = last_complete;

	if (used)
		*used = last_used;

	ret = dma_async_is_complete(cookie, last_complete, last_used);

	return ret;

}

static void nlm_adma_issue_pending(struct dma_chan *chan)
{
	struct nlm_adma_chan * nlm_chan = to_nlm_adma_chan(chan);

	process_completed_tx(nlm_chan);
}


static struct dma_async_tx_descriptor *
nlm_adma_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dma_dest,
		dma_addr_t dma_src, size_t len, unsigned long flags)
{
	int i;
	struct nlm_tx_desc *desc;
	uint64_t * ent;
	struct nlm_adma_chan *nlm_chan = to_nlm_adma_chan(chan);

	if (nlm_dtre_debug) {
		printk("prep_dma_memcpy len 0x%lx.\n", len);
	}

	desc = alloc_tx_desc(nlm_chan);
	if (!desc)
	{
		printk("DTRE Error: cannot allocate desc\n");
		return (NULL);
	}

	desc->optype = DMA_MEMCPY;
	desc->len = (unsigned long)len;
	desc->int_en = flags & DMA_PREP_INTERRUPT;
	desc->async_tx.flags = flags;
	desc->async_tx.cookie = -EBUSY;
	desc->done_flag = 0;
	desc->xor_check_result = 0;
	desc->entries_count = 0;
	desc->hw_desc.src_cnt = 0;
	desc->hw_desc.src = dma_src;
	desc->hw_desc.dst = dma_dest;
	desc->pg = NULL;
	desc->memset_val = 0;

	ent = desc->hw_desc.entries;

	for (i=0; i<DTRE_RAID_MAX_ENTRIES; i++)
		desc->hw_desc.entries[i] = 0ULL;

	return &desc->async_tx;
}


static struct dma_async_tx_descriptor *
nlm_adma_prep_dma_memset(struct dma_chan *chan, dma_addr_t dma_dest,
		int value, size_t len, unsigned long flags)
{
	int i;
	struct nlm_tx_desc *desc;
	uint64_t * ent;
	struct nlm_adma_chan *nlm_chan = to_nlm_adma_chan(chan);

	if (nlm_dtre_debug) {
		printk("prep_dma_memset len 0x%lx.\n", len);
	}

	desc = alloc_tx_desc(nlm_chan);
	if (!desc)
	{
		printk("DTRE Error: cannot allocate desc\n");
		return (NULL);
	}

	desc->optype = DMA_MEMSET;
	desc->len = (unsigned long)len;
	desc->int_en = flags & DMA_PREP_INTERRUPT;
	desc->async_tx.flags = flags;
	desc->async_tx.cookie = -EBUSY;
	desc->done_flag = 0;
	desc->xor_check_result = 0;
	desc->entries_count = 0;
	desc->hw_desc.src_cnt = 0;
	desc->hw_desc.src = (unsigned long)page_to_phys(nlm_dtre_null_page);
	desc->hw_desc.dst = dma_dest;
	desc->pg = NULL;
	desc->memset_val = value;

	ent = desc->hw_desc.entries;

	for (i=0; i<DTRE_RAID_MAX_ENTRIES; i++)
		desc->hw_desc.entries[i] = 0ULL;

	return &desc->async_tx;
}


static struct dma_async_tx_descriptor *
nlm_adma_prep_dma_xor(struct dma_chan *chan, dma_addr_t dma_dest,
		dma_addr_t *dma_src, unsigned int src_cnt, size_t len,
		unsigned long flags)
{
	int i, p_device_id, disks;
	struct nlm_tx_desc *desc;
	struct nlm_adma_chan *nlm_chan = to_nlm_adma_chan(chan);
	uint64_t * ent;

	if (nlm_dtre_debug) {
		printk("dma_xor src_cnt %d len 0x%lx.\n", src_cnt, len);
	}

	disks = src_cnt + 1;
	p_device_id = src_cnt + 1;

	if (disks > DTRE_RAID_MAX_DEVICES)
	{
		printk("DTRE Error: device count %d exceeds max %d\n",
				disks, DTRE_RAID_MAX_DEVICES);
		return (NULL);
	}

	desc = alloc_tx_desc(nlm_chan);

	if (!desc)
	{
		printk("DTRE Error: cannot allocate desc\n");
		return (NULL);
	}

	ent = desc->hw_desc.entries;

	desc->optype = DMA_XOR;
	desc->len = (unsigned long)len;
	desc->int_en = flags & DMA_PREP_INTERRUPT;
	desc->async_tx.flags = flags;
	desc->async_tx.cookie = -EBUSY;
	desc->done_flag = 0;
	desc->xor_check_result = 0;
	desc->entries_count = 0;
	desc->hw_desc.src_cnt = src_cnt;
	desc->hw_desc.src = 0;
	desc->hw_desc.dst = 0;
	desc->pg = NULL;
	desc->memset_val = 0;

	for (i=0; i<DTRE_RAID_MAX_ENTRIES; i++)
		desc->hw_desc.entries[i] = 0ULL;

	/* fill the coefficients entry */
	for (i=0; i<disks; i++)
	{
		/* set the Coefficient bit */
		ent[i] = (1ULL << 63);

		/* PCoefficient */
		ent[i] |= shift_lower_bits (1, 0, 8);

		/* QCoefficient */
		ent[i] |= shift_lower_bits ((1 << i), 8, 8);

		if ((i+1) == p_device_id)
		{
			/* fill for dest also , this is the p device id*/
			ent[i] |= shift_lower_bits (1, 40, 2);

			/* PCoefficient */
			ent[i] |= shift_lower_bits (1, 0, 8);
			ent[i] |= shift_lower_bits (0, 8, 8);

		}
		desc->entries_count++;
	}
	// printk("* check 1, entries %d.\n", desc->entries_count);

	/* fill the segments entry */
	for (i=0; i<disks; i++)
	{
		/* segment entry bit */
		ent[disks + i] = (0ULL << 63);

		/* SOD */
		ent[disks + i] |= shift_lower_bits (1, 62, 1);

		/* EOD */
		ent[disks + i] |= shift_lower_bits (1, 61, 1);

		/* segment length */
		ent[disks + i] |= shift_lower_bits(desc->len, 40, 20);

		/* segment address */
		if (i < src_cnt)
			ent[disks + i] |= shift_lower_bits(virt_to_phys ((volatile void *) dma_src[i]), 0, 40);

		if ((i+1) == p_device_id)
			ent[disks + i] |= shift_lower_bits(virt_to_phys ((volatile void *) dma_dest), 0, 40);

		desc->entries_count++;
	}

	// printk("* check 2, entries %d.\n", desc->entries_count);
	if (nlm_dtre_debug) {
		for (i=0; i<desc->entries_count; i++)
			printk(" entry %d, value: 0x%llx.\n", i, ent[i]);
	}

	return &desc->async_tx;
}


static struct dma_async_tx_descriptor *
nlm_adma_prep_dma_xor_val(struct dma_chan *chan, dma_addr_t *dma_src,
		unsigned int src_cnt, size_t len, 
		enum sum_check_flags *result, unsigned long flags)
{
	int i, p_device_id, disks;
	struct nlm_tx_desc *desc;
	struct nlm_adma_chan *nlm_chan = to_nlm_adma_chan(chan);
	uint64_t * ent;

	if (nlm_dtre_debug) {
		printk("* fn dma_xor_val src_cnt %d len 0x%lx.\n", src_cnt, len);
	}

	/* assume last dev is p_device_id */
	disks = src_cnt;
	p_device_id = src_cnt;

	if (disks > DTRE_RAID_MAX_DEVICES)
	{
		printk("DTRE Error: device count %d exceeds max %d\n",
				disks, DTRE_RAID_MAX_DEVICES);
		return (NULL);
	}

	desc = alloc_tx_desc(nlm_chan);

	if (!desc)
	{
		printk("DTRE Error: cannot allocate desc\n");
		return (NULL);
	}

	ent = desc->hw_desc.entries;

	desc->optype = DMA_XOR_VAL;
	desc->len = (unsigned long)len;
	desc->int_en = flags & DMA_PREP_INTERRUPT;
	desc->async_tx.flags = flags;
	desc->async_tx.cookie = -EBUSY;
	desc->done_flag = 0;
	desc->xor_check_result = result;
	desc->entries_count = 0;
	desc->hw_desc.src_cnt = src_cnt;
	desc->hw_desc.src = 0;
	desc->hw_desc.dst = 0;
	desc->pg = NULL;
	desc->memset_val = 0;

	for (i=0; i<DTRE_RAID_MAX_ENTRIES; i++)
		desc->hw_desc.entries[i] = 0ULL;

	/* fill the coefficients entry */
	for (i=0; i<disks; i++)
	{
		/* set the Coefficient bit */
		ent[i] = (1ULL << 63);

		/* PCoefficient */
		ent[i] |= shift_lower_bits (1, 0, 8);

		/* QCoefficient */
		ent[i] |= shift_lower_bits ((1 << i), 8, 8);

		if ((i+1) == p_device_id)
		{
			/* fill for dest also , this is the p device id*/
			ent[i] |= shift_lower_bits (1, 40, 2);

			/* PCoefficient */
			ent[i] |= shift_lower_bits (1, 0, 8);
			ent[i] |= shift_lower_bits (0, 8, 8);

		}
		desc->entries_count++;
	}
	// printk("* xor_val check 1, entries %d.\n", desc->entries_count);

	/* fill the segments entry */
	for (i=0; i<disks; i++)
	{
		/* segment entry bit */
		ent[disks + i] = (0ULL << 63);

		/* SOD */
		ent[disks + i] |= shift_lower_bits (1, 62, 1);

		/* EOD */
		ent[disks + i] |= shift_lower_bits (1, 61, 1);

		/* segment length */
		ent[disks + i] |= shift_lower_bits(desc->len, 40, 20);

		/* segment address */
		if (i < src_cnt)
			ent[disks + i] |= shift_lower_bits(virt_to_phys ((volatile void *) dma_src[i]), 0, 40);

		desc->entries_count++;
	}

	// printk("* check 2, entries %d.\n", desc->entries_count);
	if (nlm_dtre_debug) {
		for (i=0; i<desc->entries_count; i++)
			printk(" entry %d, value: 0x%llx.\n", i, ent[i]);
	}

	return &desc->async_tx;
}



static struct dma_async_tx_descriptor *
nlm_adma_prep_dma_pq(struct dma_chan *chan, dma_addr_t *dst, dma_addr_t *src,
		unsigned int src_cnt, const unsigned char *scf, size_t len,
		unsigned long flags)
{
	int i, p_device_id, q_device_id, disks;
	struct nlm_tx_desc *desc;
	struct nlm_adma_chan *nlm_chan = to_nlm_adma_chan(chan);
	uint64_t * ent;

	dma_addr_t *local_src;
	const unsigned char *local_scf;
	unsigned int local_src_cnt;
	void * pgaddr = NULL;

	local_src = src;
	local_scf = scf;

	if (dmaf_p_disabled_continue(flags))
		local_src_cnt = src_cnt + 1;
	else if (dmaf_continue(flags))
		local_src_cnt = src_cnt + 3;
	else
		local_src_cnt = src_cnt;

	if (nlm_dtre_debug) {
		printk(" fn dma_pq src_cnt %d new_src_cnt %d len 0x%lx flags 0x%lx.\n", 
				src_cnt, local_src_cnt, len, flags);
	}

	if (nlm_dtre_debug) {
		for (i=0; i<src_cnt; i++) 
			printk("* dma_pq COEFF for %i is 0x%x.\n", i, (uint8_t)local_scf[i]);
	}

	disks = local_src_cnt + 2;
	p_device_id = local_src_cnt + 1;
	q_device_id = local_src_cnt + 2;

	if (disks > DTRE_RAID_MAX_DEVICES)
	{
		printk("DTRE Error: device count %d exceeds max %d\n",
				disks, DTRE_RAID_MAX_DEVICES);
		return (NULL);
	}

	desc = alloc_tx_desc(nlm_chan);

	if (!desc)
	{
		printk("DTRE Error: cannot allocate desc\n");
		return (NULL);
	}

	ent = desc->hw_desc.entries;

	desc->optype = DMA_PQ;
	desc->len = (unsigned long)len;
	desc->int_en = flags & DMA_PREP_INTERRUPT;
	desc->async_tx.flags = flags;
	desc->async_tx.cookie = -EBUSY;
	desc->done_flag = 0;
	desc->xor_check_result = 0;
	desc->entries_count = 0;
	desc->hw_desc.src_cnt = local_src_cnt;
	desc->hw_desc.src = 0;
	desc->hw_desc.dst = 0;
	desc->pg = NULL;
	desc->memset_val = 0;

	if ((flags & DMA_PREP_PQ_DISABLE_P) || (flags & DMA_PREP_PQ_DISABLE_Q))
	{
		desc->pg = alloc_page(GFP_KERNEL);
		if (desc->pg == NULL) {
			printk("DTRE error: page allocation failed.\n");
			return NULL;
		}
		pgaddr = (void *)page_to_phys(desc->pg);
	}

	/* specify valid address for disabled result */
	if (flags & DMA_PREP_PQ_DISABLE_P)
		dst[0] = (unsigned long) pgaddr;
	if (flags & DMA_PREP_PQ_DISABLE_Q)
		dst[1] = (unsigned long) pgaddr;

	if (nlm_dtre_debug) {
		for (i=0; i<src_cnt; i++)
			printk("*src index %d, addr 0x%lx.\n", i, (unsigned long)src[i]);
		printk("dst0 : 0x%lx.\n", (unsigned long)dst[0]);
		printk("dst1 : 0x%lx.\n", (unsigned long)dst[1]);
	}

	for (i=0; i<DTRE_RAID_MAX_ENTRIES; i++)
		desc->hw_desc.entries[i] = 0ULL;

	/* fill the coefficients entry */
	for (i=0; i<disks; i++)
	{
		/* set the Coefficient bit */
		ent[i] = (1ULL << 63);

		/* cache write control */
		ent[i] |= shift_lower_bits (1, 59, 1);

		/* PCoefficient */
		if (i < src_cnt)
			ent[i] |= shift_lower_bits (1, 0, 8);

		/* QCoefficient */
		if (i < src_cnt)
			ent[i] |= shift_lower_bits((uint8_t)local_scf[i], 8, 8);

		if ((i+1) == p_device_id)
		{
			/* fill for dest also , this is the p device id*/
			ent[i] |= shift_lower_bits (1, 40, 2);

			/* PCoefficient */
			ent[i] |= shift_lower_bits (1, 0, 8);
			ent[i] |= shift_lower_bits (0, 8, 8);
		}

		if ((i+1) == q_device_id)
		{
			ent[i] |= shift_lower_bits (2, 40, 2);

			/* QCoefficient */
			ent[i] |= shift_lower_bits (0, 0, 8);
			ent[i] |= shift_lower_bits (1, 8, 8);
		}
		desc->entries_count++;
	}

	if (local_src_cnt == (src_cnt + 1)) {
		ent[src_cnt] |= shift_lower_bits (0, 0, 8); /* P coef */
		ent[src_cnt] |= shift_lower_bits (1, 8, 8); /* Q coef */
	}

	if (local_src_cnt == (src_cnt + 3)) {
		/* P coef */
		ent[src_cnt]   |= shift_lower_bits (0, 0, 8);
		ent[src_cnt+1] |= shift_lower_bits (0, 0, 8);
		ent[src_cnt+2] |= shift_lower_bits (0, 0, 8);

		/* Q coef */
		ent[src_cnt]   |= shift_lower_bits (0, 8, 8);
		ent[src_cnt+1] |= shift_lower_bits (1, 8, 8);
		ent[src_cnt+2] |= shift_lower_bits (0, 8, 8);
	}
	// printk("* check 1, entries %d.\n", desc->entries_count);

	/* fill the segments entry */
	for (i=0; i<disks; i++)
	{
		/* segment entry bit */
		ent[disks + i] = (0ULL << 63);

		/* SOD */
		ent[disks + i] |= shift_lower_bits (1, 62, 1);

		/* EOD */
		ent[disks + i] |= shift_lower_bits (1, 61, 1);

		/* segment length */
		ent[disks + i] |= shift_lower_bits(desc->len, 40, 20);

		/* segment address */
		if (i < src_cnt)
			ent[disks + i] |= shift_lower_bits(local_src[i], 0, 40);

		if ((i+1) == p_device_id)
			ent[disks + i] |= shift_lower_bits(dst[0], 0, 40);

		if ((i+1) == q_device_id)
			ent[disks + i] |= shift_lower_bits(dst[1], 0, 40);

		desc->entries_count++;
	}

	if (local_src_cnt == (src_cnt + 1)) {
		ent[disks + src_cnt] |= shift_lower_bits(dst[1], 0, 40);
	}

	if (local_src_cnt == (src_cnt + 3)) {
		ent[disks + src_cnt]   |= shift_lower_bits(dst[0], 0, 40);
		ent[disks + src_cnt+1] |= shift_lower_bits(dst[1], 0, 40);
		ent[disks + src_cnt+2] |= shift_lower_bits(dst[1], 0, 40);
	}
	// printk("* check 2, entries %d.\n", desc->entries_count);

	if (nlm_dtre_debug) {
		for (i=0; i<desc->entries_count; i++)
			printk(" entry %d, value: 0x%llx.\n", i, ent[i]);
	}

	return &desc->async_tx;
}


static struct dma_async_tx_descriptor *
nlm_adma_prep_dma_pq_val(struct dma_chan *chan, dma_addr_t *pq, dma_addr_t *src,
		unsigned int src_cnt, const unsigned char *scf,
		size_t len, enum sum_check_flags *pqres,
		unsigned long flags)
{
	int i, p_device_id, q_device_id, disks;
	struct nlm_tx_desc *desc;
	struct nlm_adma_chan *nlm_chan = to_nlm_adma_chan(chan);
	uint64_t * ent;
	void * pgaddr = NULL;

	if (nlm_dtre_debug) {
		printk("* fn dma_pq_val src_cnt %d len 0x%lx flags 0x%lx.\n", 
				src_cnt, len, flags);
	}

	if (nlm_dtre_debug) {
		for (i=0; i<src_cnt; i++)
			printk("* dma_pq_val COEFF for %i is 0x%x \n", i, (uint8_t)scf[i]);
	}

	disks = src_cnt + 2;
	p_device_id = src_cnt + 1;
	q_device_id = src_cnt + 2;

	if (disks > DTRE_RAID_MAX_DEVICES)
	{
		printk("DTRE Error: device count %d exceeds max %d\n",
				disks, DTRE_RAID_MAX_DEVICES);
		return (NULL);
	}

	desc = alloc_tx_desc(nlm_chan);

	if (!desc)
	{
		printk("DTRE Error: cannot allocate desc\n");
		return (NULL);
	}

	ent = desc->hw_desc.entries;

	desc->optype = DMA_PQ_VAL;
	desc->len = (unsigned long)len;
	desc->int_en = flags & DMA_PREP_INTERRUPT;
	desc->async_tx.flags = flags;
	desc->async_tx.cookie = -EBUSY;
	desc->done_flag = 0;
	desc->xor_check_result = pqres;
	desc->entries_count = 0;
	desc->hw_desc.src_cnt = src_cnt;
	desc->hw_desc.src = 0;
	desc->hw_desc.dst = 0;
	desc->pg = NULL;
	desc->memset_val = 0;

	if ((flags & DMA_PREP_PQ_DISABLE_P) || (flags & DMA_PREP_PQ_DISABLE_Q))
	{
		desc->pg = alloc_page(GFP_KERNEL);
		if (desc->pg == NULL) {
			printk("DTRE error: page allocation failed.\n");
			return NULL;
		}
		pgaddr = (void *) page_to_phys(desc->pg);
	}

	for (i=0; i<DTRE_RAID_MAX_ENTRIES; i++)
		desc->hw_desc.entries[i] = 0ULL;

	/* specify valid address for disabled result */
	if (flags & DMA_PREP_PQ_DISABLE_P)
		pq[0] = (unsigned long) pgaddr;
	if (flags & DMA_PREP_PQ_DISABLE_Q)
		pq[1] = (unsigned long) pgaddr;

	/* fill the coefficients entry */
	for (i=0; i<disks; i++)
	{
		/* set the Coefficient bit */
		ent[i] = (1ULL << 63);

		/* cache write control */
		ent[i] |= shift_lower_bits (1, 59, 1);

		/* PCoefficient */
		if (i < src_cnt)
			ent[i] |= shift_lower_bits (1, 0, 8);

		/* QCoefficient */
		if (i < src_cnt)
			ent[i] |= shift_lower_bits ((uint8_t)(scf[i]), 8, 8);

		if ((i+1) == p_device_id)
		{
			/* fill for dest also , this is the p device id*/
			ent[i] |= shift_lower_bits (1, 40, 2);

			/* PCoefficient */
			ent[i] |= shift_lower_bits (1, 0, 8);
			ent[i] |= shift_lower_bits (0, 8, 8);

		}

		if ((i+1) == q_device_id)
		{
			ent[i] |= shift_lower_bits (2, 40, 2);

			/* QCoefficient */
			ent[i] |= shift_lower_bits (0, 0, 8);
			ent[i] |= shift_lower_bits (1, 8, 8);
		}
		desc->entries_count++;
	}
	// printk("* pq_val check 1, entries %d.\n", desc->entries_count);

	/* fill the segments entry */
	for (i=0; i<disks; i++)
	{
		/* segment entry bit */
		ent[disks + i] = (0ULL << 63);

		/* SOD */
		ent[disks + i] |= shift_lower_bits (1, 62, 1);

		/* EOD */
		ent[disks + i] |= shift_lower_bits (1, 61, 1);

		/* segment length */
		ent[disks + i] |= shift_lower_bits(desc->len, 40, 20);

		/* segment address */
		if (i < src_cnt)
			ent[disks + i] |= shift_lower_bits(src[i], 0, 40);

		if ((i+1) == p_device_id)
			ent[disks + i] |= shift_lower_bits(pq[0], 0, 40);

		if ((i+1) == q_device_id)
			ent[disks + i] |= shift_lower_bits(pq[1], 0, 40);

		desc->entries_count++;
	}
	// printk("* check 2, entries %d.\n", desc->entries_count);

	if (nlm_dtre_debug) {
		for (i=0; i<desc->entries_count; i++)
			printk(" entry %d, value: 0x%llx.\n", i, ent[i]);
	}

	return &desc->async_tx;
}



static void free_initial_tx_desc_pool(struct nlm_adma_chan *nlm_chan)
{
	struct nlm_tx_desc *desc, *next;

	desc = nlm_chan->desc_pool_head.next;
	while(desc) {
		next = desc->next;
		kfree(desc);
		desc = next;
	}
	nlm_chan->desc_pool_head.next = NULL;
}

static int alloc_initial_tx_desc_pool (struct nlm_adma_chan *nlm_chan, int count)
{
	int i;
	struct nlm_tx_desc *ptr, *head;

	head = &nlm_chan->desc_pool_head;
	head->next = NULL;

	for(i=0; i < count; i++) {
		ptr = cacheline_aligned_kzalloc(sizeof(struct nlm_tx_desc), GFP_KERNEL);
		if(!ptr) {
			break;
		}

		dma_async_tx_descriptor_init(&ptr->async_tx, &nlm_chan->common);
		ptr->async_tx.tx_submit = nlm_tx_submit;

		/* insert at the head */
		ptr->next = head->next;
		head->next = ptr;
		ptr->chan = (void *)nlm_chan;
	}

	printk("Channel %d: Allocated %d tx descriptors\n", nlm_chan->chan_num, count);
	nlm_chan->num_tx_desc = count;

	return 0;

}

static int __devinit nlm_adma_probe(struct platform_device *pdev)
{
	struct nlm_adma_device *adev;
	struct dma_device *dma_dev;
	struct nlm_adma_chan *nlm_chan;
	int i, err, loop;
	uint64_t base;
	uint32_t value;


	if (register_xlp_msgring_handler(XLP_MSG_HANDLE_GDX, nlm_dtre_msgring_handler, NULL)){
		printk("Error: NLM-ADMA unable to register for msgring handler\n");
		return -1;
	}

	for (loop = 0; loop < DTRE_NUM_CHANNELS; loop++)
	{

		adev = kzalloc(sizeof(*adev), GFP_KERNEL);
		if (!adev)
			return -ENOMEM;
		dma_dev = &(adev->common);

		/*
		   adev = &nlm_adma_raid_device;
		   dma_dev = &nlm_adma_raid_device.common;
		 */

		INIT_LIST_HEAD(&dma_dev->channels);

		/* channel init and allocation */
		nlm_chan = kzalloc(sizeof(*nlm_chan), GFP_KERNEL);
		if (!nlm_chan) {
			kfree(adev);
			return -ENOMEM;
		}

		nlm_chan->chan_num = loop;
		nlm_chan->device = adev;
		spin_lock_init(&nlm_chan->lock);
		nlm_chan->common.device = dma_dev;
		list_add_tail(&nlm_chan->common.device_node, &dma_dev->channels);

		err = alloc_initial_tx_desc_pool(nlm_chan, DTRE_MAX_TX_DESC_PER_CHAN);

		if(err != 0) {
			free_initial_tx_desc_pool(nlm_chan);
			kfree(adev);
			kfree(nlm_chan);
			return -ENOMEM;
		}

		for (i=0; i<DTRE_MAX_TX_Q_LEN; i++)
			nlm_chan->tx_queue[i] = NULL;

		tasklet_init(&nlm_chan->irq_tasklet, nlm_adma_tasklet, 
				(unsigned long) nlm_chan);

		/* set base routines */

		/* set RAID capabilities for chan 0 and 
		   MEM capabilities for chan 1-3 */
		if (loop == 0)
		{
			dma_cap_set(DMA_XOR, dma_dev->cap_mask);
			dma_cap_set(DMA_XOR_VAL, dma_dev->cap_mask);
			dma_cap_set(DMA_PQ, dma_dev->cap_mask);
			dma_cap_set(DMA_PQ_VAL, dma_dev->cap_mask);

			dma_dev->max_xor = nlm_adma_get_max_xor();
			dma_dev->device_prep_dma_xor = nlm_adma_prep_dma_xor;
			dma_dev->device_prep_dma_xor_val = nlm_adma_prep_dma_xor_val;
			dma_set_maxpq(dma_dev, nlm_adma_get_max_pq(), 0);
			dma_dev->device_prep_dma_pq = nlm_adma_prep_dma_pq;
			dma_dev->device_prep_dma_pq_val = nlm_adma_prep_dma_pq_val;
		}
		else 
		{
			dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
			dma_cap_set(DMA_MEMSET, dma_dev->cap_mask);
			dma_dev->device_prep_dma_memcpy = nlm_adma_prep_dma_memcpy;
			dma_dev->device_prep_dma_memset = nlm_adma_prep_dma_memset;
		}


		dma_dev->device_alloc_chan_resources = nlm_adma_alloc_chan_resources;
		dma_dev->device_free_chan_resources = nlm_adma_free_chan_resources;
		dma_dev->device_is_tx_complete = nlm_adma_is_complete;
		dma_dev->device_issue_pending = nlm_adma_issue_pending;

		// dma_cap_set(DMA_INTERRUPT, dma_dev->cap_mask);
		// dma_dev->device_prep_dma_interrupt = nlm_adma_prep_dma_interrupt;

		dma_dev->dev = &pdev->dev;
		dma_async_device_register(dma_dev);

	} /* end of loop DTRE_NUM_CHANNELS */

	nlm_dtre_null_page = alloc_page(GFP_KERNEL);
	if (nlm_dtre_null_page == NULL) {
		printk("DTRE error: page allocation failed.\n");
		return 0;
	}
	memset((void *)(page_address(nlm_dtre_null_page)), 0, PAGE_SIZE);

	printk("NLM ASYNC Device Registered\n");

	/* hal init stuff */
	nlm_hal_dtr_init();

	base = nlm_hal_get_dev_base (0 /*node*/, 0 /*B*/, 5 /*D*/, 0 /*F*/);
	value = nlm_hal_read_32bit_reg (base, 0x40);
	nlm_hal_write_32bit_reg (base, 0x40, value | (0x10));

	return 0;
}


static int __devexit nlm_adma_remove(struct platform_device *dev)
{
	return 0;
}


MODULE_ALIAS("platform:nlm-adma");

static struct platform_driver nlm_adma_driver = {
	.probe		= nlm_adma_probe,
	.remove		= nlm_adma_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "NLM-ADMA",
	},
};

static int __init nlm_adma_init (void)
{
	return platform_driver_register(&nlm_adma_driver);
}

static void __exit nlm_adma_exit (void)
{
	platform_driver_unregister(&nlm_adma_driver);
	return;
}

module_init(nlm_adma_init);
module_exit(nlm_adma_exit);


static __init int nlm_add_dma_dev(void)
{
	struct platform_device *pd;
	int ret;

	pd = platform_device_alloc("NLM-ADMA", -1);
	if (!pd)
		return -ENOMEM;

	ret = platform_device_add(pd);
	if (ret)
		platform_device_put(pd);

	return ret;
}
device_initcall(nlm_add_dma_dev);


MODULE_AUTHOR("Netlogic");
MODULE_DESCRIPTION("Netlogic ADMA Engine Driver");
MODULE_LICENSE("GPL");
