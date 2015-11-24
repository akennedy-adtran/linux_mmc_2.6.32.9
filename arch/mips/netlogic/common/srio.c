/*-
 * Copyright 2005-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * MPC85xx RapidIO support
 *
 * Copyright 2005 MontaVista Software, Inc.
 * Matt Porter <mporter@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/delay.h>
#include <asm/netlogic/nlm_srio.h>
#include <asm/netlogic/gpio.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/netlogic/sim.h>

#define MYDBG(a,b...)
#define Message(a,b...)		//printk("\nFunction[%s]-Line[%d]\n"a"\n",__FUNCTION__,__LINE__,##b)

struct transaction_queue {
	uint32_t *orig;
	uint32_t start;
	uint32_t end;
	uint32_t head;
	uint32_t tail;
	spinlock_t lock;
	uint32_t max_entries;
	int curr_free_tq_entry;
	int active;
};

struct status_queue {
	uint32_t *orig;
	uint32_t start;
	uint32_t end;
	uint32_t head;
	uint32_t tail;
	int active;
	spinlock_t lock;
	void *dev_id;
	uint32_t max_entries;
	int curr_sq_index;
};

struct mailbox_queue {
	uint32_t *orig;
	uint32_t start;
	uint32_t end;
	uint32_t head;
	uint32_t tail;
	int active;
	spinlock_t lock;
	void *dev_id;
	uint32_t max_entries;
};

struct freel_queue {
	uint32_t *orig;
	uint32_t start;
	uint32_t end;
	uint32_t head;
	uint32_t tail;
	int active;
	spinlock_t lock;
	uint32_t max_entries;
};

struct nlm_rio_controller {
	struct rio_mport *port;
	int host_deviceid;
	int index;
	unsigned long regs_phy_start;
	unsigned long regs_virt_start;
	struct transaction_queue tq[MAX_TRANSACTION_Q];
	struct status_queue sq[MAX_STATUS_Q];
	struct mailbox_queue mq[MAX_MAILBOX_Q];
	struct freel_queue fq[MAX_FREEL_Q];
	int irq;
	spinlock_t maintenance_lock;	/*lock for maintenance transactions. */
	spinlock_t mailbox_lock;	/*lock for mailbox transactions. */
	spinlock_t freel_lock;	/*lock for freeq. */
};

/*All function declaration goes here.*/
int nlm_local_config_read(struct nlm_rio_controller *rio_ctrl,
			  int pindex, u32 offset, int len, u32 * data);
int nlm_local_config_write(struct nlm_rio_controller *rio_ctrl,
			   int pindex, u32 offset, int len, u32 data);
int nlm_rio_config_read(struct nlm_rio_controller *rio_ctrl,
			int pindex, u16 destid, u8 hopcount,
			u32 offset, int len, u32 * val);
int nlm_rio_config_write(struct nlm_rio_controller *rio_ctrl,
			 int pindex, u16 destid, u8 hopcount, u32 offset,
			 int len, u32 val);
int nlm_rio_doorbell_send(struct nlm_rio_controller *rio_ctrl,
			  int pindex, u16 destid, u16 data);

int nlm_setup_transaction_queue(struct nlm_rio_controller *rio_ctrl,
				struct transaction_queue *tq, int tq_index,
				int tq_entries);

int nlm_setup_status_queue(struct nlm_rio_controller *rio_ctrl,
			   struct status_queue *sq, int sq_index,
			   int sq_entries);

int nlm_setup_mailbox_queue(struct nlm_rio_controller *rio_ctrl,
			    struct mailbox_queue *mq, int mq_index,
			    int mq_entries);

int nlm_setup_freel_queue(struct nlm_rio_controller *rio_ctrl,
			  struct freel_queue *fq, int fq_index, int fq_entries);

void free_unregistered_resrources(void);

int nlm_clear_transaction_queue(struct nlm_rio_controller *rio_ctrl,
				struct transaction_queue *tq, int tq_index);

int nlm_clear_status_queue(struct nlm_rio_controller *rio_ctrl,
			   struct status_queue *sq, int sq_index);

int nlm_clear_mailbox_queue(struct nlm_rio_controller *rio_ctrl,
			    struct mailbox_queue *mq, int mq_index);

int nlm_clear_freel_queue(struct nlm_rio_controller *rio_ctrl,
			  struct freel_queue *fq, int fq_index);

/*Global data structures*/
int srio_ports;
int srio_mode;
extern unsigned long netlogic_io_base;

int nlm_rio_host_id[MAX_SRIO_PORTS] = { -1, -1, -1, -1 };

unsigned long rio_reg_phy_start[MAX_SRIO_PORTS] =
    { NLM_SRIO_MEM_0, NLM_SRIO_MEM_1, NLM_SRIO_MEM_2, NLM_SRIO_MEM_3 };

struct nlm_rio_controller *rio_controller[MAX_SRIO_PORTS] = { NULL };
extern int fdt_get_srio_port_map(uint32_t *srio_port_map);
extern uint32_t dev_tree_en;

/*controller-0 rio ops*/
RIO_OPS_LOCAL_CONFIG_READ(0)
    RIO_OPS_LOCAL_CONFIG_WRITE(0)
    RIO_OPS_CONFIG_READ(0)
    RIO_OPS_CONFIG_WRITE(0)
    RIO_OPS_DOORBELL_SEND(0)

/*controller-1 rio ops*/
    RIO_OPS_LOCAL_CONFIG_READ(1)
    RIO_OPS_LOCAL_CONFIG_WRITE(1)
    RIO_OPS_CONFIG_READ(1)
    RIO_OPS_CONFIG_WRITE(1)
    RIO_OPS_DOORBELL_SEND(1)

/*controller-2 rio ops*/
    RIO_OPS_LOCAL_CONFIG_READ(2)
    RIO_OPS_LOCAL_CONFIG_WRITE(2)
    RIO_OPS_CONFIG_READ(2)
    RIO_OPS_CONFIG_WRITE(2)
    RIO_OPS_DOORBELL_SEND(2)

/*controller-3 rio ops*/
    RIO_OPS_LOCAL_CONFIG_READ(3)
    RIO_OPS_LOCAL_CONFIG_WRITE(3)
    RIO_OPS_CONFIG_READ(3)
    RIO_OPS_CONFIG_WRITE(3)
    RIO_OPS_DOORBELL_SEND(3)

int nlm_rio_get_cmdline_0(char *s)
{
	int id;
	if (!s)
		return 0;
	id = simple_strtoul(s, NULL, 10);
	nlm_rio_host_id[0] = id;
	return 1;
}

__setup("riohdid0=", nlm_rio_get_cmdline_0);

int nlm_rio_get_cmdline_1(char *s)
{
	int id;
	if (!s)
		return 0;
	id = simple_strtoul(s, NULL, 10);
	nlm_rio_host_id[1] = id;
	return 1;
}

__setup("riohdid1=", nlm_rio_get_cmdline_1);

int nlm_rio_get_cmdline_2(char *s)
{
	int id;
	if (!s)
		return 0;
	id = simple_strtoul(s, NULL, 10);
	nlm_rio_host_id[2] = id;
	return 1;
}

__setup("riohdid2=", nlm_rio_get_cmdline_2);

int nlm_rio_get_cmdline_3(char *s)
{
	int id;
	if (!s)
		return 0;
	id = simple_strtoul(s, NULL, 10);
	nlm_rio_host_id[3] = id;
	return 1;
}

__setup("riohdid3=", nlm_rio_get_cmdline_3);

void *nlm_rio_ops[MAX_SRIO_PORTS][5] = {

	{nlm_local_config_read_0,
	 nlm_local_config_write_0,
	 nlm_rio_config_read_0,
	 nlm_rio_config_write_0,
	 nlm_rio_doorbell_send_0},
	{nlm_local_config_read_1,
	 nlm_local_config_write_1,
	 nlm_rio_config_read_1,
	 nlm_rio_config_write_1,
	 nlm_rio_doorbell_send_1},
	{nlm_local_config_read_2,
	 nlm_local_config_write_2,
	 nlm_rio_config_read_2,
	 nlm_rio_config_write_2,
	 nlm_rio_doorbell_send_2},
	{nlm_local_config_read_3,
	 nlm_local_config_write_3,
	 nlm_rio_config_read_3,
	 nlm_rio_config_write_3,
	 nlm_rio_doorbell_send_3}
};

#ifdef NLM_SRIO_DEBUG

void dump_mq_regs(struct nlm_rio_controller *rio_ctrl,
		  struct mailbox_queue *mq, int mq_index)
{
	uint32_t data;
	printk("\nmq->orig %#lx\n", mq->orig);
	nlm_local_config_read(rio_ctrl, 0, MAILBOX_QUEUE_START(mq_index), 4,
			      &data);
	printk("\nMQ QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, MAILBOX_QUEUE_END(mq_index), 4,
			      &data);
	printk("\nMQ QUEUE END = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, MAILBOX_QUEUE_HEAD(mq_index), 4,
			      &data);
	printk("\nMQ QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, MAILBOX_QUEUE_TAIL(mq_index), 4,
			      &data);
	printk("\nMQ QUEUE TAIL = %#x\n", data);

}

void dump_fq_regs(struct nlm_rio_controller *rio_ctrl,
		  struct freel_queue *fq, int fq_index)
{
	uint32_t data;
	printk("\nfq->orig %#lx\n", fq->orig);
	nlm_local_config_read(rio_ctrl, 0, FREEL_QUEUE_START(fq_index), 4,
			      &data);
	printk("\nFREEL QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, FREEL_QUEUE_END(fq_index), 4, &data);
	printk("\nFREEL QUEUE END = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, FREEL_QUEUE_HEAD(fq_index), 4,
			      &data);
	printk("\nFREEL QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, FREEL_QUEUE_TAIL(fq_index), 4,
			      &data);
	nlm_local_config_write(rio_ctrl, 0, FREEL_QUEUE_TAIL(fq_index), 4,
			       data | 0x1);
	printk("\nFREEL QUEUE TAIL = %#x\n", data);

}

void dump_tq_regs(struct nlm_rio_controller *rio_ctrl,
		  struct transaction_queue *tq, int tq_index)
{
	uint32_t data;
	printk("\ntq->orig %#lx\n", tq->orig);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_START(tq_index), 4,
			      &data);
	printk("\nTRANSACTION QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_END(tq_index), 4,
			      &data);
	printk("\nTRANSACTION QUEUE END = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_HEAD(tq_index), 4,
			      &data);
	printk("\nTRANSACTION QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_TAIL(tq_index), 4,
			      &data);
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_TAIL(tq_index), 4,
			       data | 0x1);
	printk("\nTRANSACTION QUEUE TAIL = %#x\n", data);

}

void dump_sq_regs(struct nlm_rio_controller *rio_ctrl,
		  struct status_queue *sq, int sq_index)
{
	uint32_t data;
	printk("\nsq->orig %#lx\n", sq->orig);
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_START(sq_index), 4,
			      &data);
	printk("\nSTATUS QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_END(sq_index), 4,
			      &data);
	printk("\nSTATUS QUEUE END = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_HEAD(sq_index), 4,
			      &data);
	printk("\nSTATUS QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_TAIL(sq_index), 4,
			      &data);
	printk("\nSTATUS QUEUE TAIL = %#x\n", data);

}

#endif

struct nlm_rio_controller *to_nlm_rio_ctrl(struct rio_mport *mport)
{
	int i;
	for (i = 0; i < MAX_SRIO_PORTS; i++) {
		if (rio_controller[i]) {
			if (rio_controller[i]->port == mport)
				return rio_controller[i];
		}
	}
	return NULL;
}

/**
 * rio_open_outb_mbox - Initialize outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 *
 * Initializes buffer ring, request the outbound message interrupt,
 * and enables the outbound message unit. Returns %0 on success and
 * %-EINVAL or %-ENOMEM on failure.
 */
int rio_open_outb_mbox(struct rio_mport *mport, void *dev_id, int mbox,
		       int entries)
{
	struct nlm_rio_controller *rio_ctrl = to_nlm_rio_ctrl(mport);
	int ret = 0;
	uint32_t data;

	MYDBG("Called");

	if (entries < 4) {
		printk("Minimum entries have to be 4\n");
		return -1;
	}

	if (mbox != 0) {
		printk("Invalid Resource for outb mbox - %d\n", mbox);
		return -1;
	}

	/*setup transaction queue 0 */
	ret = nlm_setup_transaction_queue(rio_ctrl, &rio_ctrl->tq[mbox], 0,
					  entries);
	if (ret)
		return ret;

	ret = nlm_setup_status_queue(rio_ctrl, &rio_ctrl->sq[mbox], 0, entries);

	if (ret) {
		nlm_clear_transaction_queue(rio_ctrl, &rio_ctrl->tq[mbox], 0);
		return ret;
	}

	rio_ctrl->tq[mbox].curr_free_tq_entry = 0;
	rio_ctrl->sq[mbox].curr_sq_index = 0;

	/*Store max entries for this tq & sq */
	rio_ctrl->tq[mbox].max_entries = entries;
	rio_ctrl->sq[mbox].max_entries = entries;

	/*Store dev_id to pass during callback */
	rio_ctrl->sq[mbox].dev_id = dev_id;

	/* Enable interrupts for status queue - TX OK */
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_IER, 4, &data);
	data = data | (1 << NNE_INTR(mbox));
	nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_IER, 4, data);

	return 0;
}

/**
 * rio_close_outb_mbox - Shut down outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, free all buffers, and
 * frees the outbound message interrupt.
 */
void rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct nlm_rio_controller *rio_ctrl = to_nlm_rio_ctrl(mport);
	struct transaction_queue *tq = &rio_ctrl->tq[mbox];
	struct status_queue *sq = &rio_ctrl->sq[mbox];
	unsigned long flags;

	Message("");

	spin_lock_irqsave(&tq->lock, flags);	/*lock against xmit */
	spin_lock(&sq->lock);	/*lock against rx */

	Message("");
	/*clear transaction queue */
	nlm_clear_transaction_queue(rio_ctrl, tq, mbox);

	Message("");
	/*clear status queue */
	nlm_clear_status_queue(rio_ctrl, sq, mbox);

	Message("");
	spin_unlock(&sq->lock);
	spin_unlock_irqrestore(&tq->lock, flags);
	return;
}

/**
 * rio_hw_add_outb_message - Add message to the outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Destination mailbox number
 * @buffer: MYDBG to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the outbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int
rio_hw_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev, int mbox,
			void *buffer, size_t len)
{
	struct nlm_rio_controller *rio_ctrl = to_nlm_rio_ctrl(mport);
	int tq_index = 0;
	unsigned long flags;
	uint32_t tail_value = 0;
	uint32_t *tail_ptr = NULL;
	struct transaction_queue *tq = &rio_ctrl->tq[tq_index];
	uint32_t word[6];
	uint64_t phys64 = 0;
	uint32_t phys = 0;
#ifdef CONFIG_32BIT
	uint32_t max_phys = 256 << 20;
#else
	uint32_t max_phys = 0xffffffff;
#endif
	uint32_t tran_size;
	int retry_count = 0;
	int destid = rdev->destid;

	MYDBG("");
	if (!tq->active)
		return -1;

	MYDBG("");
	if (!len || (len & 0x7)) {
		printk("\nXmit len has to be 8 byte aligned");
		printk("\nInvalid buffer len\n");
		return -1;
	}

	MYDBG("");
	phys64 = virt_to_phys(buffer);

	if ((phys64 > max_phys) || ((phys64 + len) > max_phys)) {
		printk("\nInvalid xmit buffer address\n");
#ifdef CONFIG_32BIT
		printk("\nBuffer address has to be below 256M\n");
#else
		printk("\nBuffer address has to be below 4G\n");
#endif
		return -1;
	}
	phys = (uint32_t) phys64;
	tran_size = (len / 8) - 1;

      retry:

	MYDBG("");
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_TAIL(tq_index), 4,
			      &tail_value);
	while ((tail_value & (1 << TQ_LOCK))) {
		printk("\nTailQ - Locked- %#x\n", tail_value);
		nlm_local_config_read(rio_ctrl, 0,
				      TRANSACTION_QUEUE_TAIL(tq_index), 4,
				      &tail_value);
	}

	if (tail_value & (1 << TQ_FULL)) {
		tail_value |= (1 << TQ_LOCK);
		printk("\nTailQ -Full - %#x\n", tail_value);
		nlm_local_config_write(rio_ctrl, 0,
				       TRANSACTION_QUEUE_TAIL(tq_index), 4,
				       tail_value);
		if (retry_count++ < 20)
			goto retry;
		else
			return -1;
	}

	spin_lock_irqsave(&tq->lock, flags);

	MYDBG("");
	if (!tq->active) {
		spin_unlock_irqrestore(&tq->lock, flags);
		return -1;
	}

	MYDBG("");
	tail_value = tail_value & 0xfffffff8;
	tail_ptr = (uint32_t *) (unsigned long)CKSEG0ADDR(tail_value);

#ifdef CONFIG_RAPIDIO_8_BIT_TRANSPORT
	destid = destid & 0xff;
	word[0] = __swab32((destid << MSG_DEST_ID) |
			   (0 << MSG_DID_SIZE) | TYPE_MESSAGE);
#else
	destid = destid & 0xffff;
	word[0] = __swab32((destid << MSG_DEST_ID) |
			   (1 << MSG_DID_SIZE) | TYPE_MESSAGE);
#endif
	word[1] = __swab32(0x7 /*lower 3 bits are always set to 1 */  |
			   tran_size << MSG_TRAN_SIZE | 0xe << MSG_SEG_SIZE |	/*256 bytes per segment */
			   mbox << MSG_MAILBOX_NUMBER);
	word[2] = __swab32((uint32_t) (phys));
	word[3] = word[4] = word[5] = 0x0;

	MYDBG("");
	tail_ptr[0] = word[0];
	tail_ptr[1] = word[1];
	tail_ptr[2] = word[2];
	tail_ptr[3] = word[3];
	tail_ptr[4] = word[4];
	tail_ptr[5] = word[5];
	mb();

	tq->curr_free_tq_entry++;
	if (tq->curr_free_tq_entry >= tq->max_entries)
		tq->curr_free_tq_entry = 0;

	/*Kick the transcation by incrementing tail ptr */
	tail_value += SIZE_OF_TQ_ENTRY;
	if (tail_value > tq->end) {
		MYDBG("Wraparound of transaction Queue %d", tq_index);
		tail_value = tq->start;
	}
	tail_value = tail_value & ~0x7;
	mb();
	/* `sync` - make sure all data is written */
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_TAIL(tq_index), 4,
			       tail_value);
	/*`sync` - make sure transaction is initiated */
	mb();
	MYDBG("");
	spin_unlock_irqrestore(&tq->lock, flags);
	return 0;
}

/**
 * rio_hw_get_inb_message - Fetch inbound message from the message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
void *rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct nlm_rio_controller *rio_ctrl = to_nlm_rio_ctrl(mport);
	uint32_t head_value;
	volatile uint32_t word[4];
	uint32_t orig_head_value;
	volatile uint32_t *head_ptr = NULL;
	struct mailbox_queue *mq = &rio_ctrl->mq[mbox];

	nlm_local_config_read(rio_ctrl, 0, MAILBOX_QUEUE_HEAD(mbox), 4,
			      &head_value);

	if (!(head_value & (1 << MQ_EMPTY))) {

		orig_head_value = head_value;
		head_value = head_value & 0xfffffff8;

		/*Left shift by 1 to get the actual physical address. */
		head_value = head_value << 1;

		/*Give sufficient delay so that glue logic copies data */
//      udelay(1);

		head_ptr =
		    (volatile uint32_t *)(unsigned long)CKSEG0ADDR(head_value);

	      retry:
		word[0] = __swab32(head_ptr[0]);
		if ((word[0] >> 2 & 0xf) == 0xf) {
			goto retry;
		}
		mb();
		if (((word[0] >> 2) & 0x8)) {
			printk("\nword[0] = %#x\n", word[0]);
			panic("\nSRIO: dma read from freel failed!!\n");
		}

		word[1] = __swab32(head_ptr[1]);
		word[2] = __swab32(head_ptr[2]);
		word[3] = __swab32(head_ptr[3]);

#ifdef NLM_SRIO_DEBUG
		if (((word[0] >> 2) & 0xf) != 0) {
			printk
			    ("\nMailbox Transaction turned in to an error !! Error %#x",
			     word[0] >> 2 & 0xf);
			panic("\npanic!!");
		}
#endif
		head_ptr[0] = 0xffffffff;
		mb();
		/*Move head pointer ahead */
		head_value += SIZE_OF_MQ_ENTRY;

		/*Glue logic needs address to be right shifted by 1 */
		head_value = head_value >> 1;

		if (head_value > mq->end) {
			head_value = mq->start;
			MYDBG("Wraparound of Status Queue %d", sq_index);
		}
		nlm_local_config_write(rio_ctrl, 0, MAILBOX_QUEUE_HEAD(mbox), 4,
				       head_value);

		return phys_to_virt(word[2]);
	}
	return NULL;
}

/**
 * rio_hw_add_inb_buffer - Add buffer to the inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Assumes buffer size 4KB
 * Adds the @buf buffer to the inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int rio_hw_add_inb_buffer(struct rio_mport *mport, int mbox, void *buf)
{
	struct nlm_rio_controller *rio_ctrl = to_nlm_rio_ctrl(mport);
	unsigned long flags;
	uint32_t tail_value = 0;
	uint32_t *tail_ptr = NULL;
	struct freel_queue *fq = &rio_ctrl->fq[mbox];
	uint32_t word[2];
	uint64_t phys64 = 0;
	uint32_t phys = 0;
#ifdef CONFIG_32BIT
	uint32_t max_phys = 256 << 20;
#else
	uint32_t max_phys = 0xffffffff;
#endif
	int retry_count = 0;
	int len = 4096;

	if (!fq->active)
		return -1;

	phys64 = virt_to_phys(buf);

	if ((unsigned long long)phys64 & 0x7ULL) {
		printk("Invalid inb buffer address - %#llx\n",
		       (unsigned long long)phys64);
		return -1;
	}

	if ((phys64 > max_phys) || ((phys64 + len) > max_phys)) {
		printk("\nInvalid xmit buffer address\n");
#ifdef CONFIG_32BIT
		printk("\nBuffer address has to be below 256M\n");
#else
		printk("\nBuffer address has to be below 4G\n");
#endif
		return -1;
	}

	phys = (uint32_t) phys64;

      retry:

	nlm_local_config_read(rio_ctrl, 0, FREEL_QUEUE_TAIL(mbox), 4,
			      &tail_value);
	while ((tail_value & (1 << FQ_LOCK))) {
		printk("\nTailQ - Locked- %#x\n", tail_value);
		nlm_local_config_read(rio_ctrl, 0, FREEL_QUEUE_TAIL(mbox),
				      4, &tail_value);
	}

	if (tail_value & (1 << FQ_FULL)) {
		tail_value |= (1 << FQ_LOCK);
		printk("\nTailQ -Full - %#x\n", tail_value);
		nlm_local_config_write(rio_ctrl, 0,
				       FREEL_QUEUE_TAIL(mbox), 4, tail_value);
		if (retry_count++ < 20) {
			goto retry;
		} else {
			printk("\nFQ is full!! can't enqueue message.\n");
			return -1;
		}
	}

	spin_lock_irqsave(&fq->lock, flags);

	if (!fq->active) {
		spin_unlock_irqrestore(&fq->lock, flags);
		return -1;
	}

	tail_value = tail_value & 0xfffffff8;
	tail_ptr = (uint32_t *) (unsigned long)CKSEG0ADDR(tail_value);

	word[0] = __swab32(phys);
	word[1] = 0x0;

	tail_ptr[0] = word[0];
	tail_ptr[1] = word[1];
	mb();

	/*Kick the transcation by incrementing tail ptr */
	tail_value += SIZE_OF_FQ_ENTRY;
	if (tail_value > fq->end) {
		MYDBG("Wraparound of transaction Queue %d", mbox);
		tail_value = fq->start;
	}
	tail_value = tail_value & ~0x7;
	mb();
	/* `sync` - make sure all data is written */
	nlm_local_config_write(rio_ctrl, 0, FREEL_QUEUE_TAIL(mbox), 4,
			       tail_value);
	/*`sync` - make sure transaction is initiated */
	mb();
	spin_unlock_irqrestore(&fq->lock, flags);

	return 0;
}

/**
 * rio_open_inb_mbox - Initialize inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring, request the inbound message interrupt,
 * and enables the inbound message unit. Returns %0 on success
 * and %-EINVAL or %-ENOMEM on failure.
 */
int rio_open_inb_mbox(struct rio_mport *mport, void *dev_id, int mbox,
		      int entries)
{
	struct nlm_rio_controller *rio_ctrl = to_nlm_rio_ctrl(mport);
	struct mailbox_queue *mq = &rio_ctrl->mq[mbox];
	struct freel_queue *fq = &rio_ctrl->fq[mbox];
	int ret = 0;

	MYDBG("Called");

	if (entries < 4) {
		printk("Minimum entries have to be 4\n");
		return -1;
	}

	if (mbox < 0 || mbox > 3) {
		printk("Invalid Resource for outb mbox - %d\n", mbox);
		return -1;
	}

	ret = nlm_setup_mailbox_queue(rio_ctrl, mq, mbox, entries);
	if (ret)
		return -1;

	ret = nlm_setup_freel_queue(rio_ctrl, fq, mbox, entries);
	if (ret) {
		nlm_clear_mailbox_queue(rio_ctrl, mq, mbox);
		return -1;
	}

	/*Store max entries for this tq & sq */
	rio_ctrl->mq[mbox].max_entries = entries;
	rio_ctrl->fq[mbox].max_entries = entries;

	/*Store dev_id to pass during callback */
	rio_ctrl->mq[mbox].dev_id = dev_id;

	return 0;
}

int nlm_clear_mailbox_queue(struct nlm_rio_controller *rio_ctrl,
			    struct mailbox_queue *mq, int mq_index)
{
	uint32_t data;
	unsigned long flags;

	spin_lock_irqsave(&rio_ctrl->mailbox_lock, flags);

	nlm_local_config_read(rio_ctrl, 0, MAILBOX_CONTROL_1, 4, &data);
	data = data & ~(1 << CONFIGURE_MQUEUE(mq_index));
	nlm_local_config_write(rio_ctrl, 0, MAILBOX_CONTROL_1, 4, data);

	mq->active = 0;

	/*Disable NNE interrupt */
	nlm_local_config_read(rio_ctrl, 0, MAILBOX_INT_EN_3, 4, &data);
	data = data & ~(1 << mq_index);
	nlm_local_config_write(rio_ctrl, 0, MAILBOX_INT_EN_3, 4, data);

	kfree(mq->orig);

	spin_unlock_irqrestore(&rio_ctrl->mailbox_lock, flags);

	return 0;
}

int nlm_clear_freel_queue(struct nlm_rio_controller *rio_ctrl,
			  struct freel_queue *fq, int fq_index)
{
	uint32_t data;
	unsigned long flags;
	fq->active = 0;

	spin_lock_irqsave(&rio_ctrl->freel_lock, flags);

	nlm_local_config_read(rio_ctrl, 0, FREEL_CONTROL_REG, 4, &data);
	data = data & ~(1 << CONFIGURE_FLQUEUE(fq_index));
	nlm_local_config_write(rio_ctrl, 0, FREEL_CONTROL_REG, 4, data);

	kfree(fq->orig);

	spin_unlock_irqrestore(&rio_ctrl->freel_lock, flags);
	return 0;
}

/**
 * rio_close_inb_mbox - Shut down inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the inbound message unit, free all buffers, and
 * frees the inbound message interrupt.
 */
void rio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct nlm_rio_controller *rio_ctrl = to_nlm_rio_ctrl(mport);
	struct mailbox_queue *mq = &rio_ctrl->mq[mbox];
	struct freel_queue *fq = &rio_ctrl->fq[mbox];
	unsigned long flags;

	MYDBG("Called");

	spin_lock_irqsave(&mq->lock, flags);	/*lock against xmit */
	spin_lock(&fq->lock);	/*lock against rx */

	nlm_clear_mailbox_queue(rio_ctrl, mq, mbox);
	nlm_clear_freel_queue(rio_ctrl, fq, mbox);

	spin_unlock(&fq->lock);	/*lock against rx */
	spin_unlock_irqrestore(&mq->lock, flags);	/*lock against xmit */
	return;
}

int nlm_setup_mailbox_queue(struct nlm_rio_controller *rio_ctrl,
			    struct mailbox_queue *mq, int mq_index,
			    int mq_entries)
{
	uint32_t phys = 0x0;
	uint32_t size = SIZE_OF_MQ_ENTRY * mq_entries;
	uint32_t data;
	static int max_mailbox_number = 0;
	unsigned long flags;

	mq->orig = kmalloc(size + SMP_CACHE_BYTES, GFP_KERNEL | GFP_DMA);

	if (!mq->orig)
		return -ENOMEM;

	memset(mq->orig, 0xff, size + SMP_CACHE_BYTES);

	phys = virt_to_phys(mq->orig);

	if (phys & (SMP_CACHE_BYTES - 1))
		phys = (phys + SMP_CACHE_BYTES) & ~0x1f;

	/*Glue logic needs address to be right shifted by 1 */
	mq->start = phys >> 1;
	mq->end = (phys + size - SIZE_OF_SQ_ENTRY) >> 1;

	/*Configure MQ Start and End pointers */
	nlm_local_config_write(rio_ctrl, 0, MAILBOX_QUEUE_START(mq_index), 4,
			       mq->start);
	nlm_local_config_write(rio_ctrl, 0, MAILBOX_QUEUE_END(mq_index), 4,
			       mq->end);

	/*Set MQ Upper pointer to 0 */
	nlm_local_config_write(rio_ctrl, 0, MAILBOX_QUEUE_UPTR(mq_index), 4, 0);

	if (max_mailbox_number == 0) {
		/*Configure MAX Mailbox number */
		max_mailbox_number = MAX_MAILBOX_Q - 1;
		nlm_local_config_write(rio_ctrl, 0, MAILBOX_CONTROL_3, 4,
				       max_mailbox_number << HIGH_MAILB_NO);
	}

	spin_lock_irqsave(&rio_ctrl->mailbox_lock, flags);

	nlm_local_config_read(rio_ctrl, 0, MAILBOX_CONTROL_1, 4, &data);
	data = data | (1 << CONFIGURE_MQUEUE(mq_index));
	nlm_local_config_write(rio_ctrl, 0, MAILBOX_CONTROL_1, 4, data);

	mq->active = 1;

	/*Enable NNE interrupt */
	nlm_local_config_read(rio_ctrl, 0, MAILBOX_INT_EN_3, 4, &data);
	data = data | (1 << mq_index);
	nlm_local_config_write(rio_ctrl, 0, MAILBOX_INT_EN_3, 4, data);

#ifdef NLM_SRIO_DEBUG
	dump_mq_regs(rio_ctrl, mq, mq_index);
#endif
	spin_unlock_irqrestore(&rio_ctrl->mailbox_lock, flags);

	spin_lock_init(&mq->lock);
	return 0;
}

int nlm_setup_freel_queue(struct nlm_rio_controller *rio_ctrl,
			  struct freel_queue *fq, int fq_index, int fq_entries)
{
	uint32_t phys = 0x0;
	uint32_t size = SIZE_OF_FQ_ENTRY * fq_entries;
	uint32_t data;
	unsigned long flags;

	fq->orig = kmalloc(size + SMP_CACHE_BYTES, GFP_KERNEL | GFP_DMA);

	if (!fq->orig)
		return -ENOMEM;

	phys = virt_to_phys(fq->orig);

	if (phys & (SMP_CACHE_BYTES - 1))
		phys = (phys + SMP_CACHE_BYTES) & ~0x1f;

	fq->start = phys;
	fq->end = phys + size - SIZE_OF_FQ_ENTRY;

	phys = (uint32_t) virt_to_phys((void *)(unsigned long)fq->start);

	/*Configure FQ Start and End pointers */
	nlm_local_config_write(rio_ctrl, 0, FREEL_QUEUE_START(fq_index), 4,
			       fq->start);
	nlm_local_config_write(rio_ctrl, 0, FREEL_QUEUE_END(fq_index), 4,
			       fq->end);

	/*Set FQ Upper pointer to 0 */
	nlm_local_config_write(rio_ctrl, 0, FREEL_QUEUE_UPTR(fq_index), 4, 0);

	/*Freelist buff size */
	nlm_local_config_write(rio_ctrl, 0, FREEL_BUF_SIZE(fq_index), 4,
			       511 << FL_BUF_SIZE);

	fq->active = 1;
	/*Configure freelist q */

	spin_lock_irqsave(&rio_ctrl->freel_lock, flags);

	nlm_local_config_read(rio_ctrl, 0, FREEL_CONTROL_REG, 4, &data);
	data = data | (1 << CONFIGURE_FLQUEUE(fq_index));
	nlm_local_config_write(rio_ctrl, 0, FREEL_CONTROL_REG, 4, data);

#ifdef NLM_SRIO_DEBUG
	dump_fq_regs(rio_ctrl, fq, fq_index);
#endif
	spin_unlock_irqrestore(&rio_ctrl->freel_lock, flags);

	spin_lock_init(&fq->lock);

	return 0;
}

int nlm_clear_transaction_queue(struct nlm_rio_controller *rio_ctrl,
				struct transaction_queue *tq, int tq_index)
{
	uint32_t data;
	tq->active = 0;

	/*disable transaction queue */
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_CTRL_1, 4, &data);
	data = data & ~(1 << CONFIGURE_TQUEUE(tq_index));
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_CTRL_1, 4, data);

#ifdef NLM_SRIO_DEBUG
	dump_tq_regs(rio_ctrl, tq, tq_index);
#endif

	kfree(tq->orig);
	return 0;
}

int nlm_clear_status_queue(struct nlm_rio_controller *rio_ctrl,
			   struct status_queue *sq, int sq_index)
{
	uint32_t data;
	sq->active = 0;

	/* Disable status q interrupt */
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_IER, 4, &data);
	data = data & ~(1 << NNE_INTR(sq_index));
	nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_IER, 4, data);

#ifdef NLM_SRIO_DEBUG
	dump_sq_regs(rio_ctrl, sq, sq_index);
#endif

	/*disable status queue */
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_CTRL, 4, &data);
	data = data & ~(1 << CONFIGURE_SQUEUE(sq_index));
	nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_CTRL, 4, data);
	kfree(sq->orig);

	return 0;
}

int nlm_setup_transaction_queue(struct nlm_rio_controller *rio_ctrl,
				struct transaction_queue *tq, int tq_index,
				int tq_entries)
{
	uint32_t phys = 0x0;
	uint32_t size = SIZE_OF_TQ_ENTRY * tq_entries;
	uint32_t data;

	tq->orig = kmalloc(size + SMP_CACHE_BYTES, GFP_KERNEL | GFP_DMA);

	if (!tq->orig)
		return -ENOMEM;

	phys = virt_to_phys(tq->orig);

	if (phys & (SMP_CACHE_BYTES - 1))
		phys = (phys + SMP_CACHE_BYTES) & ~0x1f;

	tq->start = phys;
	tq->end = phys + size - SIZE_OF_TQ_ENTRY;

#if 0
	/*Below are internal pointers. */
	tq->head = phys;
	tq->tail = phys + size - SIZE_OF_TQ_ENTRY;
#endif

	phys = (uint32_t) virt_to_phys((void *)(unsigned long)tq->start);

	/*Configure TQ Start and End pointers */
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_START(tq_index),
			       4, tq->start);
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_END(tq_index), 4,
			       tq->end);

	/*Set TQ Upper pointer to 0 */
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_UPTR(tq_index), 4,
			       0);
	/*Configure and Enable TQ */
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_CTRL_1, 4, &data);
	data = data | (1 << CONFIGURE_TQUEUE(tq_index)) | (1 << ENABLE_QUEUE);
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_CTRL_1, 4, data);

	tq->active = 1;

#ifdef NLM_SRIO_DEBUG
	dump_tq_regs(rio_ctrl, tq, tq_index);
#endif

	spin_lock_init(&tq->lock);

	return 0;
}

int nlm_setup_status_queue(struct nlm_rio_controller *rio_ctrl,
			   struct status_queue *sq, int sq_index,
			   int sq_entries)
{
	uint32_t phys = 0x0;
	uint32_t size = SIZE_OF_SQ_ENTRY * sq_entries;
	uint32_t data;

	sq->orig = kmalloc(size + SMP_CACHE_BYTES, GFP_KERNEL | GFP_DMA);

	if (!sq->orig)
		return -ENOMEM;

	memset(sq->orig, 0xff, size + SMP_CACHE_BYTES);

	phys = virt_to_phys(sq->orig);

	if (phys & (SMP_CACHE_BYTES - 1))
		phys = (phys + SMP_CACHE_BYTES) & ~0x1f;

	/*Glue logic needs address to be right shifted by 1 */
	sq->start = phys >> 1;
	sq->end = (phys + size - SIZE_OF_SQ_ENTRY) >> 1;

	/*Configure SQ Start and End pointers */
	nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_START(sq_index), 4,
			       sq->start);
	nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_END(sq_index), 4,
			       sq->end);

	/*Set SQ Upper pointer to 0 */
	nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_UPTR(sq_index), 4, 0);

	/*Configure and Enable SQ */
	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_CTRL, 4, &data);
	data = data | (1 << CONFIGURE_SQUEUE(sq_index));
	nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_CTRL, 4, data);

	sq->active = 1;

#ifdef NLM_SRIO_DEBUG
	dump_sq_regs(rio_ctrl, sq, sq_index);
#endif

	spin_lock_init(&sq->lock);

	return 0;
}

#if 0
int nlm_setup_maint_trans_rsrc_test(struct nlm_rio_controller *rio_ctrl)
{
	int ret = 0;
	uint32_t data;

	/*Init maintainence transaction lock. */
	spin_lock_init(&rio_ctrl->maintenance_lock);

	ret = nlm_setup_transaction_queue(rio_ctrl, &rio_ctrl->tq[1], 1, 16);
	if (ret)
		return ret;

	printk("\nAllocated buffer for tq = %#x\n", rio_ctrl->tq[1].start);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_HEAD(1), 4, &data);
	printk("\nTRANSACTION QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_TAIL(1), 4, &data);
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_TAIL(1), 4,
			       data | 0x1);
	printk("\nTRANSACTION QUEUE TAIL = %#x\n", data);

	printk("\nDisableing Transaction Q Config bit for testing...\n");

	/*Disable config bit */
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_CTRL_1, 4, &data);
	data = data & ~(1 << CONFIGURE_TQUEUE(1));
	nlm_local_config_write(rio_ctrl, 0, TRANSACTION_QUEUE_CTRL_1, 4, data);

	printk("\nReconfiguring Transaction Queue for testing...\n");
	ret = nlm_setup_transaction_queue(rio_ctrl, &rio_ctrl->tq[1], 1, 16);
	if (ret)
		return ret;

	printk("\nNew Allocated buffer for tq = %#x\n", rio_ctrl->tq[1].start);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_HEAD(1), 4, &data);
	printk("\nNew TRANSACTION QUEUE HEAD = %#x\n", data);
	nlm_local_config_read(rio_ctrl, 0, TRANSACTION_QUEUE_TAIL(1), 4, &data);
	printk("\nNew TRANSACTION QUEUE TAIL = %#x\n", data);
//    ret = nlm_setup_status_queue(rio_ctrl,&rio_ctrl->sq[1], 1, 16);
//    if(ret)
//        return ret;

	return ret;
}
#endif

int nlm_setup_maint_trans_rsrc(struct nlm_rio_controller *rio_ctrl)
{
	int ret = 0;

	/*Init maintainence transaction lock. */
	spin_lock_init(&rio_ctrl->maintenance_lock);

	ret = nlm_setup_transaction_queue(rio_ctrl, &rio_ctrl->tq[1], 1, 16);
	if (ret)
		return ret;

	ret = nlm_setup_status_queue(rio_ctrl, &rio_ctrl->sq[1], 1, 16);
	if (ret) {
		nlm_clear_transaction_queue(rio_ctrl, &rio_ctrl->tq[1], 1);
		return ret;
	}

	return ret;
}

/**
 * nlm_local_rio_config_read - Generate a XLS local config space read
 * @pindex: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a XLS local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
int nlm_local_config_read(struct nlm_rio_controller *rio_ctrl,
			  int pindex, u32 offset, int len, u32 * data)
{
    unsigned long virt = rio_ctrl->regs_virt_start;
	MYDBG("nlm_local_config_read: index %d offset %8.8x\n", pindex,
		 offset);
#ifdef CONFIG_64BIT
	*data = *(u32 *)(unsigned long)(virt + offset);
#else
	*data = *(u32 *)(u32)(virt + offset);
#endif
	return 0;
}

/**
 * nlm_local_rio_config_write - Generate a XLS local config space write
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a XLS local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
int nlm_local_config_write(struct nlm_rio_controller *rio_ctrl,
			   int pindex, u32 offset, int len, u32 data)
{
	unsigned long virt = rio_ctrl->regs_virt_start;
	MYDBG("nlm_local_config_write: index %d offset %8.8x\n", pindex,
		 offset);
#ifdef CONFIG_64BIT
    *(u32 *)(unsigned long)(virt + offset) = data;
#else
    *(u32 *)(u32)(virt + offset) = data;
#endif
    return 0;
}

int setup_maint_tq_entry(struct nlm_rio_controller *rio_ctrl,
			 int pindex, int tq_index, int type, u16 destid,
			 u8 hopcount, u32 offset, int len, u32 phys)
{
	int words = 0;
	uint32_t tail_value = 0;
	uint32_t *tail_ptr = NULL;
	struct transaction_queue *tq = &rio_ctrl->tq[tq_index];
	uint32_t word[6];

      retry:
	nlm_local_config_read(rio_ctrl, pindex,
			      TRANSACTION_QUEUE_TAIL(tq_index), 4, &tail_value);
	while ((tail_value & (1 << TQ_LOCK))) {
		printk("\nTailQ - Locked- %#x\n", tail_value);
		nlm_local_config_read(rio_ctrl, pindex,
				      TRANSACTION_QUEUE_TAIL(tq_index), 4,
				      &tail_value);
	}

	if (tail_value & (1 << TQ_FULL)) {
		tail_value |= (1 << TQ_LOCK);
		printk("\nTailQ -Full - %#x\n", tail_value);
		nlm_local_config_write(rio_ctrl, pindex,
				       TRANSACTION_QUEUE_TAIL(tq_index), 4,
				       tail_value);
		goto retry;
	}

	tail_value = tail_value & 0xfffffff8;
	tail_ptr = (uint32_t *) (unsigned long)CKSEG0ADDR(tail_value);

#ifdef CONFIG_RAPIDIO_8_BIT_TRANSPORT
	destid = destid & 0xff;
	word[0] = __swab32((destid << MAINT_DEST_ID) |
			   (0 << MAINT_DID_SIZE) | type);
#else
	destid = destid & 0xffff;
	word[0] = __swab32((destid << MAINT_DEST_ID) |
			   (1 << MAINT_DID_SIZE) | type);
#endif
	word[1] = __swab32((words << MAINT_TRANS_SIZE) |
			   (0x3 << MAINT_TRANS_DEFAULT_BITS));
	word[2] = __swab32((uint32_t) (phys));
	word[3] = __swab32((hopcount << MAINT_HOP_COUNT) | (offset >> 2));
	word[4] = word[5] = 0x0;

	tail_ptr[0] = word[0];
	tail_ptr[1] = word[1];
	tail_ptr[2] = word[2];
	tail_ptr[3] = word[3];
	tail_ptr[4] = word[4];
	tail_ptr[5] = word[5];
	mb();
	/*Kick the transcation by incrementing tail ptr */
	tail_value += SIZE_OF_TQ_ENTRY;
	if (tail_value > tq->end) {
		MYDBG("Wraparound of transaction Queue %d", tq_index);
		tail_value = tq->start;
	}
	tail_value = tail_value & ~0x7;
	mb();
	/*`sync` - make sure all data is written */
	nlm_local_config_write(rio_ctrl, pindex,
			       TRANSACTION_QUEUE_TAIL(tq_index), 4, tail_value);
	/*`sync` - make sure transaction is initiated */
	mb();
	return 0;
}

int check_maint_sq_entry(struct nlm_rio_controller *rio_ctrl,
			 int pindex, int sq_index, int type,
			 u16 destid, u32 phys)
{
	uint32_t head_value = 0;
	uint32_t *head_ptr = NULL;
	struct status_queue *sq = &rio_ctrl->sq[sq_index];
	uint32_t word[4];
	uint32_t orig_head_value;

	nlm_local_config_read(rio_ctrl, pindex, STATUS_QUEUE_HEAD(sq_index), 4,
			      &head_value);
	while (head_value & (1 << SQ_EMPTY)) {
		mdelay(5);
//      printk("\nStatusQ Is empty yet.. %#x\n",head_value);
		nlm_local_config_read(rio_ctrl, pindex,
				      STATUS_QUEUE_HEAD(sq_index), 4,
				      &head_value);
	}
	orig_head_value = head_value;
	head_value = head_value & 0xfffffff8;
	/*Left shift by 1 to get the actual physical address. */
	head_value = head_value << 1;

	/*Give sufficient delay so that glue logic copies data */
	udelay(1);

	head_ptr = (uint32_t *) (unsigned long)CKSEG0ADDR(head_value);
	word[0] = __swab32(head_ptr[0]);
	word[1] = __swab32(head_ptr[1]);
	word[2] = __swab32(head_ptr[2]);
	word[3] = __swab32(head_ptr[3]);
	mb();

#ifdef NLM_SRIO_DEBUG
	/*Check the transaction type is expected or not. */
	if ((word[0] & 0x1f) != type) {
		printk("\nhead_value = %#x\n", orig_head_value);
		printk("word0=%#x, word1=%#x, word2=%#x, word3=%#x\n",
		       word[0], word[1], word[2], word[3]);
		printk("\nUnexpected Transaction %#x instead of %#x \n",
		       word[0] & 0x1f, type);
		panic("\npanic!!");
	}

	if (((word[0] >> 12) & 0xf) != 0) {
		printk("\nTransaction turned in to an error !! Error %#x",
		       word[0] >> 12 & 0xf);
		panic("\npanic!!");
	}
	/*Check the phy address in response */
	if (word[2] != phys) {
		printk
		    ("\n!!!Got the result for phys address %#x instead of %#x\n",
		     word[2], phys);
		panic("\npanic!!");
	}
#endif

	/*Move head pointer ahead */
	head_value += SIZE_OF_SQ_ENTRY;

	/*Glue logic needs address to be right shifted by 1 */
	head_value = head_value >> 1;

	if (head_value > sq->end) {
		head_value = sq->start;
		MYDBG("Wraparound of Status Queue %d", sq_index);
	}
	nlm_local_config_write(rio_ctrl, pindex, STATUS_QUEUE_HEAD(sq_index), 4,
			       head_value);
	return 0;
}

/**
 * nlm_rio_config_read - Generate a XLS read maintenance transaction
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a XLS read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
int
nlm_rio_config_read(struct nlm_rio_controller *rio_ctrl,
		    int pindex, u16 destid, u8 hopcount, u32 offset, int len,
		    u32 * val)
{
	u8 *data;
	int tq_index = 1;	/*Use TQ-1 for all maint transactions. */
	uint32_t result = 0x0;
	uint32_t result_phys = 0x0;
	int sq_index = 1;
	unsigned long flags;
	MYDBG
	    ("index %d destid %d hopcount %d offset %8.8x len %d\n",
	     pindex, destid, hopcount, offset, len);
	result_phys = virt_to_phys(&result);

	spin_lock_irqsave(&rio_ctrl->maintenance_lock, flags);

	setup_maint_tq_entry(rio_ctrl, pindex, tq_index, TYPE_MAINTAIN_READ,
			     destid, hopcount, offset, len, result_phys);

	/*Check on the status Q for transaction completion. */
	check_maint_sq_entry(rio_ctrl, pindex, sq_index, TYPE_MAINTAIN_READ,
			     destid, result_phys);

	spin_unlock_irqrestore(&rio_ctrl->maintenance_lock, flags);

	/*Store the result back.. */
	data = (u8 *) (unsigned long)((unsigned long)&result + (offset & 0x3));

	switch (len) {
	case 1:
		*val = (u32) * (u8 *) data;
		break;
	case 2:
		*val = (u32) * (u16 *) data;
		break;
	default:
		*val = (u32) * (u32 *) data;
		break;
	}
	return 0;
}

/**
 * nlm_rio_config_write - Generate a XLS write maintenance transaction
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an XLS write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
int
nlm_rio_config_write(struct nlm_rio_controller *rio_ctrl, int pindex,
		     u16 destid, u8 hopcount, u32 offset, int len, u32 val)
{
	u8 *data;
	int tq_index = 1;	/*Use TQ-1 for all maint transactions. */
	uint32_t r_data = 0x0;
	uint32_t r_data_phys = 0x0;
	int sq_index = 1;
	unsigned long flags;
	uint32_t w_data = 0;
	uint32_t w_data_phys = 0;

	MYDBG
	    ("index %d destid %d hopcount %d offset %8.8x len %d\n",
	     pindex, destid, hopcount, offset, len);
	w_data_phys = virt_to_phys(&w_data);

	/*Check if we have to do read modify write */
	if (len == 4) {
		w_data = val;
		goto skip;
	}

	/*Do Read Modify Write */

	printk("\nWriting unaligned data\n");

	r_data_phys = virt_to_phys(&r_data);

	nlm_rio_config_read(rio_ctrl, pindex, destid, hopcount, offset & ~(0x3),
			    4, &r_data);

	data = (u8 *) (unsigned long)((unsigned long)&r_data + (offset & 0x3));

	switch (len) {
	case 1:
		*data = (u8) val;
		w_data = r_data;
		break;
	case 2:
		*(u16 *) data = (u16) val;
		w_data = r_data;
		break;
	default:
		break;
	}

      skip:
	spin_lock_irqsave(&rio_ctrl->maintenance_lock, flags);

	setup_maint_tq_entry(rio_ctrl, pindex, tq_index, TYPE_MAINTAIN_WRITE,
			     destid, hopcount, offset, len, w_data_phys);

	/*Check on the status Q for transaction completion. */
	check_maint_sq_entry(rio_ctrl, pindex, sq_index, TYPE_MAINTAIN_WRITE,
			     destid, w_data_phys);

	spin_unlock_irqrestore(&rio_ctrl->maintenance_lock, flags);

	return 0;
}

int setup_doorbell_tq_entry(struct nlm_rio_controller *rio_ctrl,
			    int pindex, int tq_index, u16 destid, u16 data)
{
	uint32_t tail_value = 0;
	uint32_t *tail_ptr = NULL;
	struct transaction_queue *tq = &rio_ctrl->tq[tq_index];
	uint32_t word[6];
	int type = TYPE_DOORBELL;

      retry:

	nlm_local_config_read(rio_ctrl, pindex,
			      TRANSACTION_QUEUE_TAIL(tq_index), 4, &tail_value);

	while ((tail_value & (1 << TQ_LOCK))) {
		printk("\nTailQ - Locked- %#x\n", tail_value);
		nlm_local_config_read(rio_ctrl, pindex,
				      TRANSACTION_QUEUE_TAIL(tq_index), 4,
				      &tail_value);
	}

	if (tail_value & (1 << TQ_FULL)) {
		tail_value |= (1 << TQ_LOCK);
		printk("\nTailQ -Full - %#x\n", tail_value);
		nlm_local_config_write(rio_ctrl, pindex,
				       TRANSACTION_QUEUE_TAIL(tq_index), 4,
				       tail_value);
		goto retry;
	}

	tail_value = tail_value & 0xfffffff8;
	tail_ptr = (uint32_t *) (unsigned long)CKSEG0ADDR(tail_value);

#ifdef CONFIG_RAPIDIO_8_BIT_TRANSPORT
	destid = destid & 0xff;
	word[0] = __swab32((destid << MAINT_DEST_ID) |
			   (0 << MAINT_DID_SIZE) | type);
#else
	destid = destid & 0xffff;
	word[0] = __swab32((destid << MAINT_DEST_ID) |
			   (1 << MAINT_DID_SIZE) | type);
#endif
	word[1] = __swab32(data << DBELL_INFO);
	word[2] = word[3] = word[4] = word[5] = 0x0;

	tail_ptr[0] = word[0];
	tail_ptr[1] = word[1];
	tail_ptr[2] = word[2];
	tail_ptr[3] = word[3];
	tail_ptr[4] = word[4];
	tail_ptr[5] = word[5];
	mb();
	/*Kick the transcation by incrementing tail ptr */
	tail_value += SIZE_OF_TQ_ENTRY;
	if (tail_value > tq->end) {
		MYDBG("Wraparound of transaction Queue %d", tq_index);
		tail_value = tq->start;
	}
	tail_value = tail_value & ~0x7;
	mb();
	/*`sync` - make sure all data is written */
	nlm_local_config_write(rio_ctrl, pindex,
			       TRANSACTION_QUEUE_TAIL(tq_index), 4, tail_value);
	/*`sync` - make sure transaction is initiated */
	mb();

	return 0;
}

int check_doorbell_sq_entry(struct nlm_rio_controller *rio_ctrl,
			    int pindex, int sq_index, u16 destid)
{
	uint32_t head_value = 0;
	uint32_t *head_ptr = NULL;
	struct status_queue *sq = &rio_ctrl->sq[sq_index];
	uint32_t word[4];
	uint32_t orig_head_value;

#ifdef NLM_SRIO_DEBUG
	int type = TYPE_DOORBELL;
#endif

	nlm_local_config_read(rio_ctrl, pindex, STATUS_QUEUE_HEAD(sq_index), 4,
			      &head_value);
	while (head_value & (1 << SQ_EMPTY)) {
		mdelay(5);
//      printk("\nStatusQ Is empty yet.. %#x\n",head_value);
		nlm_local_config_read(rio_ctrl, pindex,
				      STATUS_QUEUE_HEAD(sq_index), 4,
				      &head_value);
	}
	orig_head_value = head_value;
	head_value = head_value & 0xfffffff8;
	/*Left shift by 1 to get the actual physical address. */
	head_value = head_value << 1;

	/*Give sufficient delay so that glue logic copies data */
	udelay(1);

	head_ptr = (uint32_t *) (unsigned long)CKSEG0ADDR(head_value);
	word[0] = __swab32(head_ptr[0]);
	word[1] = __swab32(head_ptr[1]);
	word[2] = __swab32(head_ptr[2]);
	word[3] = __swab32(head_ptr[3]);
	mb();
#ifdef NLM_SRIO_DEBUG
	/*Check the transaction type is expected or not. */
	if ((word[0] & 0x1f) != type) {
		printk("\nhead_value = %#x\n", orig_head_value);
		printk("word0=%#x, word1=%#x, word2=%#x, word3=%#x\n",
		       word[0], word[1], word[2], word[3]);
		printk("\nUnexpected Transaction %#x instead of %#x \n",
		       word[0] & 0x1f, type);
		panic("\npanic!!");
	}

	if (((word[0] >> 12) & 0xf) != 0) {
		printk("\nTransaction turned in to an error !! Error %#x",
		       word[0] >> 12 & 0xf);
//      panic("\npanic!!");
	}
#endif

	/*Move head pointer ahead */
	head_value += SIZE_OF_SQ_ENTRY;

	/*Glue logic needs address to be right shifted by 1 */
	head_value = head_value >> 1;

	if (head_value > sq->end) {
		head_value = sq->start;
		MYDBG("Wraparound of Status Queue %d", sq_index);
	}
	nlm_local_config_write(rio_ctrl, pindex, STATUS_QUEUE_HEAD(sq_index), 4,
			       head_value);
	return 0;
}

/**
 * nlm_rio_doorbell_send - Send a XLS doorbell message
 * @index: ID of RapidIO interface
 * @destid: Destination ID of target device
 * @data: 16-bit info field of RapidIO doorbell message
 *
 * Sends a XLS doorbell message. Returns %0 on success or
 * %-EINVAL on failure.
 */
int nlm_rio_doorbell_send(struct nlm_rio_controller *rio_ctrl,
			  int pindex, u16 destid, u16 data)
{
	int tq_index = 1;
	int sq_index = 1;
	unsigned long flags;

	MYDBG("index %d destid %4.4x data %4.4x\n", pindex, destid, data);

	spin_lock_irqsave(&rio_ctrl->maintenance_lock, flags);

	MYDBG("");
	setup_doorbell_tq_entry(rio_ctrl, pindex, tq_index, destid, data);
	/*Check on the status Q for transaction completion. */
	MYDBG("");
	check_doorbell_sq_entry(rio_ctrl, pindex, sq_index, destid);
	MYDBG("");
	spin_unlock_irqrestore(&rio_ctrl->maintenance_lock, flags);

	return 0;
}

void doorbell_intr(struct nlm_rio_controller *rio_ctrl, struct rio_mport *port)
{
	uint32_t no_of_msgs = 0;
	uint32_t data;
	struct rio_dbell *dbell;
	u16 info, src;
	int found = 0;

	nlm_local_config_read(rio_ctrl, 0, DOORBELL_INFO, 4, &data);
	no_of_msgs = (data >> DFIFO_NUM) & 0xf;
	while (no_of_msgs) {
		nlm_local_config_read(rio_ctrl, 0, DOORBELL_READ, 4, &data);

		info = data & 0xffff;

#ifdef CONFIG_RAPIDIO_8_BIT_TRANSPORT
		src = (data >> DB_SRCID) & 0xff;
#else
		src = (data >> DB_SRCID) & 0xffff;
#endif
		list_for_each_entry(dbell, &port->dbells, node) {
			if ((dbell->res->start <= info) &&
			    (dbell->res->end >= info)) {
				found = 1;
				break;
			}
		}
		if (found) {
			dbell->dinb(port, dbell->dev_id, src, -1, info);
		} else {
			printk("RIO: spurious doorbell, sid %2.2x info %4.4x\n",
			       src, info);
		}
		no_of_msgs--;
	}
	return;
}

void mailbox_intr(struct nlm_rio_controller *rio_ctrl, struct rio_mport *port)
{
	struct mailbox_queue *mq;
	uint32_t status;
	int i = 0;

	nlm_local_config_read(rio_ctrl, 0, MAILBOX_STATUS_3, 4, &status);

	for (i = 0; i < MAX_MAILBOX_Q; i++) {
		if (!(status & (1 << i)))
			continue;
		mq = &rio_ctrl->mq[i];

		spin_lock(&mq->lock);
		if (!mq->active) {
			spin_unlock(&mq->lock);
			continue;
		}
		if (port->inb_msg[i].mcback)
			port->inb_msg[i].mcback(port, mq->dev_id, -1, -1);
		else
			printk("No call back for this message !! Screwed ??\n");
		spin_unlock(&mq->lock);
	}
	return;
}

void statusq_intr(struct nlm_rio_controller *rio_ctrl, struct rio_mport *port)
{
	int sq_index = 0;
	struct status_queue *sq = &rio_ctrl->sq[sq_index];
	uint32_t head_value = 0;
	uint32_t orig_head_value = 0;
	volatile uint32_t *head_ptr = NULL;
	volatile uint32_t word[4];
#ifdef NLM_SRIO_DEBUG
	uint32_t ttype;
#endif

	MYDBG("Got status q intr");
	/*Hold a lock, so that close does not free resources */
	spin_lock(&sq->lock);

	if (!sq->active) {
		spin_unlock(&sq->lock);
		return;
	}

	nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_HEAD(sq_index), 4,
			      &head_value);

	while (!(head_value & (1 << SQ_EMPTY))) {

		orig_head_value = head_value;
		head_value = head_value & 0xfffffff8;
		/*Left shift by 1 to get the actual physical address. */
		head_value = head_value << 1;

		/*Give sufficient delay so that glue logic copies data */
//      udelay(1);

		head_ptr =
		    (volatile uint32_t *)(unsigned long)CKSEG0ADDR(head_value);
	      retry:
		word[0] = __swab32(head_ptr[0]);
		if (((word[0] >> 12) & 0xf) == 0xf) {
			goto retry;
		}

		word[1] = __swab32(head_ptr[1]);
		word[2] = __swab32(head_ptr[2]);
		word[3] = __swab32(head_ptr[3]);
		mb();
#ifdef NLM_SRIO_DEBUG
		ttype = word[0] & 0x1f;
		/*Check the transaction type is expected or not. */
		if (ttype != TYPE_MESSAGE) {
			printk("\nhead_value = %#x\n", orig_head_value);
			printk("word0=%#x, word1=%#x, word2=%#x, word3=%#x\n",
			       word[0], word[1], word[2], word[3]);
			printk("\nUnexpected Transaction %#x \n",
			       word[0] & 0x1f);
			panic("\npanic!!");
		}

		if (((word[0] >> 12) & 0xf) != 0) {
			printk
			    ("\nTransaction turned in to an error !! Error %#x",
			     word[0] >> 12 & 0xf);
			panic("\npanic!!");
		}
#endif
		head_ptr[0] = 0xffffffff;
		mb();

		/*Move head pointer ahead */
		head_value += SIZE_OF_SQ_ENTRY;

		/*Glue logic needs address to be right shifted by 1 */
		head_value = head_value >> 1;

		if (head_value > sq->end) {
			head_value = sq->start;
			MYDBG("Wraparound of Status Queue %d", sq_index);
		}
		nlm_local_config_write(rio_ctrl, 0, STATUS_QUEUE_HEAD(sq_index),
				       4, head_value);

		sq->curr_sq_index++;
		if (sq->curr_sq_index >= sq->max_entries)
			sq->curr_sq_index = 0;
		/*Call the callback for msg transactions. */
		port->outb_msg[sq_index].mcback(port, sq->dev_id, -1,
						sq->curr_sq_index);

		nlm_local_config_read(rio_ctrl, 0, STATUS_QUEUE_HEAD(sq_index),
				      4, &head_value);
	}

	spin_unlock(&sq->lock);

	return;
}

void general_intr(struct nlm_rio_controller *rio_ctrl, struct rio_mport *port)
{
	uint32_t gisr = 0;
	uint32_t dma_high, dma_low, dma_info;
	uint32_t perr_csr = 0;

	nlm_local_config_read(rio_ctrl, port->index, GENERAL_INTR_STATUS_REG, 4,
			      &gisr);
	if (gisr & GISR_DEC) {
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_HIGH,
				      4, &dma_high);
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_LOW, 4,
				      &dma_low);
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_INFO,
				      4, &dma_info);
		printk("\nSRIO DMA ERROR:\n dma_high: [%#x], dma_low: [%#x],\
                dma_info[%#x]\n", dma_high, dma_low, dma_info);
		panic("SRIO: DMA Error");
	}
	if (gisr & GISR_MQWE) {
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_HIGH,
				      4, &dma_high);
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_LOW, 4,
				      &dma_low);
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_INFO,
				      4, &dma_info);
		printk("\nMQ DMA ERROR:\n dma_high: [%#x], dma_low: [%#x], \
                dma_info[%#x]\n", dma_high, dma_low, dma_info);
		panic("SRIO: MQ Error");
	}
	if (gisr & GISR_SQWE) {
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_HIGH,
				      4, &dma_high);
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_LOW, 4,
				      &dma_low);
		nlm_local_config_read(rio_ctrl, port->index, DMA_ERR_CAP_INFO,
				      4, &dma_info);
		printk("\nSQ DMA ERROR:\n dma_high: [%#x], dma_low: [%#x], \
                dma_info[%#x]\n", dma_high, dma_low, dma_info);
		panic("SRIO: SQ Error");
	}
	if (gisr & GISR_PERR) {
		printk("\nPORT ERROR!!\n");
		nlm_local_config_read(rio_ctrl, port->index, P0_EAS_CSR, 4,
				      &perr_csr);
		printk("port error status register [%#x]\n", perr_csr);
		panic("SRIO: PORT ERROR - Software Recovery not available\n");
	}
}

irqreturn_t nlm_srio_irq_handler(int irq, void *dev_id)
{
	struct nlm_rio_controller *rio_ctrl =
	    (struct nlm_rio_controller *)dev_id;
	struct rio_mport *port = rio_ctrl->port;
	uint32_t misr = 0;

	MYDBG("");
	/*Read MISR */
	nlm_local_config_read(rio_ctrl, port->index, MASTER_INTR_STATUS_REG, 4,
			      &misr);
	if (misr & MISR_GEN) {
		general_intr(rio_ctrl, port);
	}

	if (misr & MISR_DF) {
		MYDBG("DF INTR");
		doorbell_intr(rio_ctrl, port);
	}

	if (misr & MISR_MQ3) {
		MYDBG("MQ INTR");
		mailbox_intr(rio_ctrl, port);
	}

	if (misr & MISR_SQ) {
		MYDBG("SQ INTR");
		statusq_intr(rio_ctrl, port);
	}

	MYDBG("");
	return IRQ_HANDLED;
}

void setup_controller(struct nlm_rio_controller *rio_ctrl,
		      struct rio_mport *port, int index)
{
	rio_ctrl->index = index;
	rio_ctrl->port = port;
	rio_ctrl->irq = SRIO_IRQ(index);
	rio_ctrl->regs_phy_start = rio_reg_phy_start[index];
	rio_ctrl->host_deviceid = nlm_rio_host_id[index];
	rio_ctrl->regs_virt_start =
	    ((unsigned long)CKSEG1ADDR(rio_reg_phy_start[index]));
	MYDBG("\nController[%d] regs starts @ [%#lx]\n", index,
	      rio_ctrl->regs_virt_start);
	MYDBG("\nHost device id %d\n", rio_ctrl->host_deviceid);
}

int is_rio_link_up(int index)
{
	unsigned long addr = CKSEG1ADDR(rio_reg_phy_start[index]);
	uint32_t result;
	result = *(uint32_t *) (addr + P0_EAS_CSR);
	if (result & PORT_N_ERR_STS_PORT_OK)
		return 1;
	return 0;
}

int setup_rio_ops(struct rio_mport *mport, int index)
{
	struct rio_ops *ops = NULL;

	ops = kmalloc(sizeof(struct rio_ops), GFP_KERNEL | GFP_DMA);

	if (!ops) {
		printk("Allocation failed for rio_ops!\n");
		return -1;
	}
	ops->lcread = nlm_rio_ops[index][0];
	ops->lcwrite = nlm_rio_ops[index][1];
	ops->cread = nlm_rio_ops[index][2];
	ops->cwrite = nlm_rio_ops[index][3];
	ops->dsend = nlm_rio_ops[index][4];
	mport->ops = ops;
	return 0;
}

static int nlm_srio_glue_init_done(void)
{
    nlm_reg_t *srio_be_mmio = 
                (nlm_reg_t *)(netlogic_io_base + NETLOGIC_IO_SRIO_1_OFFSET);
    uint32_t srio_ctrl = 0x0;
    
    srio_ctrl = netlogic_read_reg(srio_be_mmio, SRIO_CTRL);
    if(((srio_ctrl>>28) & 0xf) == 0x3)
        /*Bootloader has done required glue logic init*/
        return 1;
    return 0;
}

void nlm_rio_setup(void)
{
	nlm_reg_t *gpio_mmio =
	    (nlm_reg_t *) (netlogic_io_base + NETLOGIC_IO_GPIO_OFFSET);
	uint32_t gpio_reset_cfg;
	struct rio_mport *port;
    int i = 0;
    extern int avail_mem_above_4G;
    struct nlm_rio_controller *rio_ctrl;
	uint32_t srio_port_map;
    
    if(!is_xlsb0_srio())
        return;

    if (avail_mem_above_4G) {
        printk("------------------------------------------------\n");
        printk("[SRIO]: Detected DRAM above 4G\n");
        printk("      : HW Support for DMA >32-bit Un-available. \n");
        printk("------------------------------------------------\n");
        return;
    }

    if(!nlm_srio_glue_init_done()){
        printk("------------------------------------------------\n");
        printk("SRIO Init is not done from bootloader\n");
        printk("Skipping SRIO Initialization\n");
        printk("------------------------------------------------\n");
        return;
    }

    /*Check srio mode - x1 or x4*/
    gpio_reset_cfg = 
        netlogic_read_reg(gpio_mmio,NETLOGIC_GPIO_PWRON_RESET_CFG_REG);
    
    if(gpio_reset_cfg & (1<<SRIO_CFG_BIT)){
        srio_mode = SRIO_X1_MODE;
        srio_ports = MAX_SRIO_PORTS;
        printk("Detected SRIO x1 mode\n");
        printk("Detected %d SRIO ports\n",srio_ports);
    }else{
        srio_mode = SRIO_X4_MODE;
        srio_ports = MIN_SRIO_PORTS;
        printk("Detected SRIO x4 mode\n");
        printk("Detected %d SRIO port\n",srio_ports);
    }

	if(dev_tree_en) 
		fdt_get_srio_port_map(&srio_port_map);
	 else 
		srio_port_map = (1 << srio_ports) - 1;

	//printk("\nsrio_port_map = %#x\n",srio_port_map);
    for(i=0; i<srio_ports; i++){

	if(!(srio_port_map & ( 1 << i)))
		continue;

        /*Check if link is up*/
        if(!is_rio_link_up(i)){
//          printk("\nLink %d is down!!!\n",i);
            continue;
        }


        printk("SRIO link %d up\n",i);

	if (gpio_reset_cfg & (1 << SRIO_CFG_BIT)) {
		srio_mode = SRIO_X1_MODE;
		srio_ports = MAX_SRIO_PORTS;
		printk("Detected SRIO x1 mode\n");
		printk("Detected %d SRIO ports\n", srio_ports);
	} else {
		srio_mode = SRIO_X4_MODE;
		srio_ports = MIN_SRIO_PORTS;
		printk("Detected SRIO x4 mode\n");
		printk("Detected %d SRIO port\n", srio_ports);
	}

	for (i = 0; i < srio_ports; i++) {
		/*Check if link is up */
		if (!is_rio_link_up(i)) {
			//printk("\nLink %d is down!!!\n",i);
			continue;
		}

		printk("SRIO link %d up\n", i);

		rio_ctrl = rio_controller[i] = kmalloc
		    (sizeof(struct nlm_rio_controller), GFP_KERNEL | GFP_DMA);

		if (!rio_ctrl)
			goto fail;

		port = kmalloc(sizeof(struct rio_mport), GFP_KERNEL | GFP_DMA);

		if (!port) {
			printk("Allocation failed for rio_mport!\n");
			printk("Registered %d rapidio controller\n", i);
			goto fail;
		}

		sprintf(port->name, "RIO%d mport", i);

		port->id = 0;
		port->index = 0;
		INIT_LIST_HEAD(&port->dbells);

		port->phy_type = RIO_PHY_SERIAL;
		port->sys_size = 1;	/*Controller supports 16bit transport addressing*/
		port->host_deviceid = rio_ctrl->host_deviceid;

		setup_controller(rio_ctrl, port, i);

		port->iores.start = rio_ctrl->regs_phy_start;
		port->iores.end = rio_ctrl->regs_phy_start + NLM_SRIO_MEM_SIZE;
		port->iores.flags = IORESOURCE_MEM;

		rio_init_dbell_res(&port->riores[RIO_DOORBELL_RESOURCE], 0,
				   0xffff);
		rio_init_mbox_res(&port->riores[RIO_INB_MBOX_RESOURCE], 0, 3);
		rio_init_mbox_res(&port->riores[RIO_OUTB_MBOX_RESOURCE], 0, 0);

		port->host_deviceid = rio_ctrl->host_deviceid;

		/*Enable Output, Input and Multicast event partcipant */
		nlm_local_config_write(rio_ctrl, port->index, P0_CTRL_CSR, 4,
				       (1 << OUTPUT_PORT_EN) | (1 <<
								INPUT_PORT_EN) |
				       (1 << MULTI_EVENT_EN));

		/*
		   Setup resources for maintenance transactions.
		   -setup Transaction Queue 1
		   -setup Status Queue 1
		 */
		if (nlm_setup_maint_trans_rsrc(rio_ctrl))
			goto fail;

		/*Register irq for controller. */
		if (request_irq
		    (rio_ctrl->irq, nlm_srio_irq_handler, IRQF_DISABLED,
		     rio_ctrl->port->name, rio_ctrl))
			panic("Can't reserve srio irq!!");

		/*Enable GENERL ERROR INTERRUPTS */
		nlm_local_config_write(rio_ctrl, rio_ctrl->port->index,
				       GENERAL_INTR_ENABLE_REG, 4,
				       (1 << GIER_PERR) | (1 << GIER_DEC) |
				       (1 << GIER_MQWE) | (1 << GIER_SQWE));
		/*Enable MIER for SQ, MQ3 and DF */
		nlm_local_config_write(rio_ctrl, rio_ctrl->port->index,
				       MASTER_INTR_ENABLE_REG, 4,
				       (1 << MIER_DF) | (1 << MIER_MQ3) | (1 <<
									   MIER_SQ)
				       | (1 << MIER_GEN));
#if 0
		nlm_setup_maint_trans_rsrc_test(rio_ctrl);
		continue;
#endif
		/*Enable doorbell interrupts. Kernel doesn't give us any hook!! */
		nlm_local_config_write(rio_ctrl, rio_ctrl->port->index,
				       DOORBELL_INTR, 4, (1 << DB_NNE));

		spin_lock_init(&rio_ctrl->mailbox_lock);
		spin_lock_init(&rio_ctrl->freel_lock);
		rio_register_mport(port);
	}

      fail:
	free_unregistered_resrources();
	return;
}

void free_unregistered_resrources(void)
{
	struct rio_mport *port = NULL;
	struct rio_ops *ops = NULL;
	int i = 0;

	for (i = 0; i < srio_ports; i++) {
		/*Free all partially initalized controller data struct */
		if (!rio_controller[i])
			continue;
		port = rio_controller[i]->port;
		if (!port) {
			kfree(rio_controller[i]);
			continue;
		}
		ops = port->ops;
		if (!ops) {
			kfree(port);
			kfree(rio_controller[i]);
			continue;
		}
	}
}
