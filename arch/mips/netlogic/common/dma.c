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


#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/sim.h>

#define CH0_CONTROL 8
#define MSGRNG_CODE_DMA 8
#define XLR_DMA_RESP_TIMEOUT 500
#define MAX_DMA_QUEUE_LEN 256
#define PCIX_ACK_TIMER_VAL 0x18

#define MAX_DMA_TRANS_PER_CPU 256
#define XLR_MAX_DMA_LEN_PER_DESC ((1 << 20) - 1)	/* 1 MB - 1 */

#define NEXT_SEQ_NUM(x) ((x->sequence_number + 1) & (MAX_DMA_TRANS_PER_CPU - 1))
#define INC_SEQ_NUM(x) x->sequence_number = \
		((x->sequence_number + 1) & (MAX_DMA_TRANS_PER_CPU - 1))

#define DMA_SLOT_BUSY(x) (x->trans[x->sequence_number].pending)

#define DMA_RESP_PENDING(x, seq) (x->trans[seq].pending)

#define DMA_SLOT_GET(x) (x->trans[x->sequence_number].pending = 1); \
				INC_SEQ_NUM(ctrl);

#define DMA_SLOT_PUT(x, seq) (x->trans[seq].pending = 0)

#define DMA_GET_RESP(x, seq) (x->trans[seq].dma_resp)

#define DMA_PUT_RESP(x, seq, msg) x->trans[seq].dma_resp = msg; \
				x->trans[seq].pending = 0;

#define DMA_DONE(x, seq) (x->trans[seq].pending == 0)

#define Message(a,b...)		//printk("\n[%s] - "a"\n",__FUNCTION__,##b)
#define ErrorMsg(a,b...) printk("\nError in [%s] - "a"\n",__FUNCTION__,##b)
enum dma_msgring_bucket_config {

	DMA_MSG_BUCKET0_SIZE = 0x320,
	DMA_MSG_BUCKET1_SIZE,
	DMA_MSG_BUCKET2_SIZE,
	DMA_MSG_BUCKET3_SIZE,
};

enum dma_msgring_credit_config {

	DMA_CC_CPU0_0 = 0x380,
	DMA_CC_CPU1_0 = 0x388,
	DMA_CC_CPU2_0 = 0x390,
	DMA_CC_CPU3_0 = 0x398,
	DMA_CC_CPU4_0 = 0x3a0,
	DMA_CC_CPU5_0 = 0x3a8,
	DMA_CC_CPU6_0 = 0x3b0,
	DMA_CC_CPU7_0 = 0x3b8
};

/* We use 10 bit transaction id in the DMA message to uniquely identify a DMA
   response.
   0-7 indicate a sequence number (0 to 255)
   8-9 bits encode the CPU thread id (0 to 3)
   */
typedef struct dma_trans {
	volatile int pending;
	uint64_t dma_resp;
	void (*func) (void *, uint64_t);
	void *data;
} dma_trans_t;

typedef struct xlr_dma_ctrl {
	spinlock_t q_lock;
	int sequence_number;
	dma_trans_t trans[MAX_DMA_TRANS_PER_CPU];
} xlr_dma_ctrl_t;

volatile static int xlr_dma_producer = 0;
volatile static int xlr_dma_consumer = 0;
struct msgrng_msg xlr_dma_queue[MAX_DMA_QUEUE_LEN];
static int xlr_dma_init_done = 0;
xlr_dma_ctrl_t xlr_dma_ctrl[NR_CPUS];

spinlock_t xlr_dma_lock = SPIN_LOCK_UNLOCKED;
spinlock_t xlr_enqueue_dma_spin = SPIN_LOCK_UNLOCKED;
uint32_t xlr_total_dma_reqs, xlr_total_dma_bytes;
uint32_t xlr_dma_req_failed, xlr_dma_timeout_errors, xlr_dma_errors,
    xlr_dma_stale_resp;
uint32_t xlr_dma_msg_send_failed;

void xlr_async_dma_task(unsigned long data);
extern unsigned int nlm_common_get_shared_mem_base(void);
#define CONFIG_PROC_FS 1
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>

extern int xlr_loader_own_dma;
extern struct proc_dir_entry *nlm_root_proc;

static int xlr_dma_proc_read(char *page, char **start, off_t off,
			     int count, int *eof, void *data)
{
	int len, total_len;
	char *ptr = page;

	if (count < 512)	/* Need minimum of this space */
		return -EINVAL;

	total_len = 0;
	len = sprintf(ptr, "Total DMA Requests = %d\n", xlr_total_dma_reqs);
	ptr += len;
	total_len += len;

	len = sprintf(ptr, "Total DMA Bytes = %d\n", xlr_total_dma_bytes);
	ptr += len;
	total_len += len;

	len = sprintf(ptr, "DMA Requests failed = %d\n", xlr_dma_req_failed);
	ptr += len;
	total_len += len;

	len = sprintf(ptr, "DMA Timeout errors = %d\n", xlr_dma_timeout_errors);
	ptr += len;
	total_len += len;

	len = sprintf(ptr, "DMA errors = %d\n", xlr_dma_errors);
	ptr += len;
	total_len += len;

	len = sprintf(ptr, "DMA Stale responses = %d\n", xlr_dma_stale_resp);
	total_len += len;

	len = sprintf(ptr, "DMA Message Send Failed = %d\n",
		      xlr_dma_msg_send_failed);
	total_len += len;

	return total_len;
}
void xlr_init_dma_proc(void)
{
	struct proc_dir_entry *entry;

	if (!(entry = create_proc_entry("xlr_dma_stats", 0444, nlm_root_proc))) {
		printk("%s: create_proc_entry failed\n", __FUNCTION__);
		return;
	}
	entry->read_proc = xlr_dma_proc_read;

}
void xlr_uninit_dma_proc(void)
{
	remove_proc_entry("xlr_dma_stat", nlm_root_proc);
}
#endif

/* DMA message handler - Called from interrupt context */
static void xlr_dma_msgring_handler(int bucket, int size, int code,
				    int stid, struct msgrng_msg *msg,
				    void *data /* ignored */ )
{
	int cpu, thr_id, tx_id, seq;
	xlr_dma_ctrl_t *ctrl;

	tx_id = (msg->msg0 >> 48) & 0x3ff;
	thr_id = (tx_id >> 8) & 0x3;
	seq = (tx_id & 0xff);

	cpu = (netlogic_cpu_id() * 4) + thr_id;
	ctrl = &xlr_dma_ctrl[cpu];

	spin_lock(&ctrl->q_lock);
	/* Check if there was a pending request. This can happen if the
	   requestor times out and gives up the request. So in that case
	   do not update the response
	   NOTE: One corner case that is not handled here is that when seq no
	   wraps around and request was pending for the new one and this response
	   was for the old request. This ideally must not happen.
	 */
	if (DMA_RESP_PENDING(ctrl, seq)) {

		DMA_PUT_RESP(ctrl, seq, msg->msg0);
		if (ctrl->trans[seq].func) {
			ctrl->trans[seq].func(ctrl->trans[seq].data, msg->msg0);
		}
		spin_unlock(&ctrl->q_lock);
		return;
	}
	spin_unlock(&ctrl->q_lock);
	printk("ERROR: Stale response from DMA engine for transaction id %d\n",
	       seq);
	spin_lock(&xlr_dma_lock);
	xlr_dma_stale_resp++;
	spin_unlock(&xlr_dma_lock);
	return;
}

inline void xlr_build_xfer_msg(struct msgrng_msg *msg, uint64_t src,
			       uint64_t dest, uint32_t len, int tx_id,
			       int resp_bkt)
{
	msg->msg0 = (1ULL << 63) | ((uint64_t) len << 40) |
	    (src & 0xffffffffffULL);
	msg->msg1 = (1ULL << 58) | ((uint64_t) tx_id << 48) |
	    ((uint64_t) resp_bkt << 40) | (dest & 0xffffffffffull);

}

int xlr_enqueue_dma_msg(struct msgrng_msg *msg)
{
	unsigned long mflags;
	spin_lock_irqsave(&xlr_enqueue_dma_spin, mflags);
	if (((xlr_dma_producer + 1) % (MAX_DMA_QUEUE_LEN)) == xlr_dma_consumer) {
		//ErrorMsg("DMA Async Queue is full");
		spin_unlock_irqrestore(&xlr_enqueue_dma_spin, mflags);
		return -ENOMEM;
	}
	//Enqueue Msg 
	xlr_dma_queue[xlr_dma_producer].msg0 = msg->msg0;
	xlr_dma_queue[xlr_dma_producer].msg1 = msg->msg1;
	xlr_dma_queue[xlr_dma_producer].msg2 = 0ULL;
	xlr_dma_queue[xlr_dma_producer].msg3 = 0ULL;
	xlr_dma_producer = (xlr_dma_producer + 1) % (MAX_DMA_QUEUE_LEN);
	spin_unlock_irqrestore(&xlr_enqueue_dma_spin, mflags);
	return 0;
}

DECLARE_TASKLET(xlr_dma_task, xlr_async_dma_task, 0);
void xlr_async_dma_task(unsigned long data)
{
	unsigned long flags = 0;
	int data_len;
	uint64_t phys1, phys2;
	struct msgrng_msg *msg;
	static int last_msg_send_success = 1;
	int msg_send_success = 0;

	while ((xlr_dma_consumer != xlr_dma_producer)) {

		msg = &xlr_dma_queue[xlr_dma_consumer];
		phys1 = msg->msg0 & 0xffffffffffULL;	//DEST
		phys2 = msg->msg1 & 0xffffffffffULL;	//SRC
		data_len = (msg->msg0 >> 40) & 0xfffff;
		msgrng_access_enable(flags);
		if (xlr_dma_consumer == xlr_dma_producer) {
			ErrorMsg("Shdnt Happen.");
		}
		if (message_send_retry(2, MSGRNG_CODE_DMA, MSGRNG_STNID_DMA_0,
				       &xlr_dma_queue[xlr_dma_consumer])) {
			xlr_dma_msg_send_failed++;
			msgrng_access_disable(flags);
			last_msg_send_success = 1;
			msg_send_success = 0;
			tasklet_schedule(&xlr_dma_task);
			break;
		}

		msg_send_success = 1;
		last_msg_send_success = 1;
		xlr_dma_consumer = (xlr_dma_consumer + 1) % (MAX_DMA_QUEUE_LEN);
		msgrng_access_disable(flags);
	}

}

/* Returns 0 on success, -1 otherwise */
int xlr_async_request_dma(uint64_t src, uint64_t dest, uint32_t len,
			  void (*func) (void *, uint64_t), void *data)
{
	int thr_id, cpu;
	xlr_dma_ctrl_t *ctrl;
	int tx_id, resp_bkt, seq;
	struct msgrng_msg msg;
	unsigned long flags;

	/* Driver does not support multiple descriptor DMA yet */
	if (len > XLR_MAX_DMA_LEN_PER_DESC) {
		ErrorMsg("%s: Cannot do DMA for more than %d bytes\n",
			 __FUNCTION__, XLR_MAX_DMA_LEN_PER_DESC);
		return -1;
	}
	if (xlr_dma_init_done == 0) {
		ErrorMsg("%s: XLR DMA engine is not initialized\n",
			 __FUNCTION__);
		return -1;
	}
	Message("\nSrc Addr %#llx Dst Addr %#llx\n", src, dest);

	preempt_disable();
	thr_id = netlogic_thr_id();
	cpu = (netlogic_cpu_id() * 4) + thr_id;
	ctrl = &xlr_dma_ctrl[cpu];

	spin_lock_irqsave(&ctrl->q_lock, flags);
	preempt_enable();

	if (DMA_SLOT_BUSY(ctrl)) {
		//ErrorMsg("%s: No space to enqueue this request\n", __FUNCTION__);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);
		return -1;
	}
	tx_id = (thr_id << 8) | ctrl->sequence_number;
	seq = ctrl->sequence_number;
	DMA_SLOT_GET(ctrl);

	/* use bucket 0 of each core as the bucket where response will be 
	   received
	 */
	resp_bkt = netlogic_cpu_id() * 8;

	spin_unlock_irqrestore(&ctrl->q_lock, flags);

	/*CallBack For Async Call. */
	ctrl->trans[seq].func = func;
	ctrl->trans[seq].data = data;

	/* Form the DMA simple xfer request and send to Channel 0 */

	xlr_build_xfer_msg(&msg, src, dest, len, tx_id, resp_bkt);

	if (xlr_enqueue_dma_msg(&msg)) {
		//ErrorMsg("Cant Enqueue Msg.");
		spin_lock_irqsave(&ctrl->q_lock, flags);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);
		tasklet_schedule(&xlr_dma_task);
		return -1;
	}
	tasklet_schedule(&xlr_dma_task);
	return 0;
}

/* Returns 0 on success, -1 otherwise */
int xlr_request_dma(uint64_t src, uint64_t dest, uint32_t len)
{
	int thr_id, cpu, i;
	xlr_dma_ctrl_t *ctrl;
	int tx_id, resp_bkt, seq, ret, err;
	struct msgrng_msg  msg = {0}, r_msg = {0};
	unsigned long flags;

	/* Driver does not support multiple descriptor DMA yet */
	if (len > XLR_MAX_DMA_LEN_PER_DESC) {
		printk("%s: Cannot do DMA for more than %d bytes\n",
		       __FUNCTION__, XLR_MAX_DMA_LEN_PER_DESC);
		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_req_failed++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);
		return -1;
	}
	if (xlr_dma_init_done == 0) {
		printk("%s: XLR DMA engine is not initialized\n", __FUNCTION__);
		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_req_failed++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);
		return -1;
	}

	preempt_disable();
	thr_id = netlogic_thr_id();
	cpu = (netlogic_cpu_id() * 4) + thr_id;
	ctrl = &xlr_dma_ctrl[cpu];

	spin_lock_irqsave(&ctrl->q_lock, flags);
	preempt_enable();	/* will not enable actually */
	if (DMA_SLOT_BUSY(ctrl)) {
		printk("%s: No space to enqueue this request\n", __FUNCTION__);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);

		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_req_failed++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);
		return -1;
	}
	tx_id = (thr_id << 8) | ctrl->sequence_number;
	seq = ctrl->sequence_number;
	DMA_SLOT_GET(ctrl);

	/* use bucket 0 of each core as the bucket where response will be 
	   received
	 */
	resp_bkt = netlogic_cpu_id() * 8;
	spin_unlock_irqrestore(&ctrl->q_lock, flags);

	/*Reset Callback - As this is not async call */
	ctrl->trans[seq].func = NULL;
	ctrl->trans[seq].data = NULL;

	/* Form the DMA simple xfer request and send to Channel 0 */
	xlr_build_xfer_msg(&msg, src, dest, len, tx_id, resp_bkt);
	msgrng_access_enable(flags);
	if (message_send_retry(2, MSGRNG_CODE_DMA, MSGRNG_STNID_DMA_0, &msg)) {
		printk
		    ("Message_send failed: Cannot submit DMA request to engine\n");
		msgrng_access_disable(flags);
		spin_lock_irqsave(&ctrl->q_lock, flags);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);

		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_req_failed++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);
		return -1;
	}
	msgrng_access_disable(flags);
	/* wait for the response here */
	for (i = 0; i < XLR_DMA_RESP_TIMEOUT; i++) {
		if (DMA_DONE(ctrl, seq))
			break;
		udelay(50);
	}
	if (i == XLR_DMA_RESP_TIMEOUT) {
		printk("%s:Did not get response from DMA engine\n",
		       __FUNCTION__);
		spin_lock_irqsave(&ctrl->q_lock, flags);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);

		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_timeout_errors++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);
		return -1;
	}
	/* Do some error checks */

	r_msg.msg0 = DMA_GET_RESP(ctrl, seq);
	ret = (r_msg.msg0 >> 62) & 0x3;
	err = (r_msg.msg0 >> 60) & 0x3;
	if (ret != 0x3) {
		printk("%s: Bad return code %d from DMA engine\n", __FUNCTION__,
		       ret);
		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_errors++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);
		return -1;
	}
	if (err & 0x2) {
		printk("%s:DMA engine reported Message format error\n",
		       __FUNCTION__);
		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_errors++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);
		return -1;
	}
	if (err & 0x1) {
		printk("%s:DMA engine reported Bus error\n", __FUNCTION__);
		spin_lock_irqsave(&xlr_dma_lock, flags);
		xlr_dma_errors++;
		spin_unlock_irqrestore(&xlr_dma_lock, flags);

		return -1;
	}
	spin_lock_irqsave(&xlr_dma_lock, flags);
	xlr_total_dma_reqs++;
	xlr_total_dma_bytes += len;
	spin_unlock_irqrestore(&xlr_dma_lock, flags);
	return 0;
}

int xlr_init_dma(void)
{
	int i;
	nlm_reg_t *mmio = netlogic_io_mmio(0x1A000);
	xlr_dma_ctrl_t *ctrl;
#ifdef NLM_BRIDGE_WKAROUND
	unsigned int flags=0;
#endif

	for (i = 0; i < NR_CPUS; i++) {
		ctrl = &xlr_dma_ctrl[i];
		spin_lock_init(&ctrl->q_lock);
		ctrl->sequence_number = 0;
	}
	/* Register for the Message ring handler */
	if (register_msgring_handler(TX_STN_DMA, xlr_dma_msgring_handler, NULL)) {
		printk("Couldn't register DMA msgring handler\n");
		return -1;
	}
#ifdef NLM_BRIDGE_WKAROUND
	if (nlm_enable_br_wrkaround) {
		flags = nlm_write_lock_irq_save(nlm_bridge_lock);
	}
#endif
	/* Use channel 0 for all DMA */
	/* Pci Stream Hint En = 1, Report Error = 1, Section Size = 4, 
	 * RMaxCr=4, WMaxCr =4, En=1; */
	mmio[CH0_CONTROL] = (1 << 28) | (1 << 24) | (4 << 12) | (4 << 8) |
	    (4 << 5) | (1 << 4);
	mmio[PCIX_ACK_TIMER_VAL] = 100;
	/* Configure the bucket sizes */
	if (xlr_loader_own_dma) {
		mmio[DMA_MSG_BUCKET0_SIZE] =
		    shared_bucket_sizes.bucket[MSGRNG_STNID_DMA_0];
		mmio[DMA_MSG_BUCKET1_SIZE] =
		    shared_bucket_sizes.bucket[MSGRNG_STNID_DMA_1];
		mmio[DMA_MSG_BUCKET2_SIZE] =
		    shared_bucket_sizes.bucket[MSGRNG_STNID_DMA_2];
		mmio[DMA_MSG_BUCKET3_SIZE] =
		    shared_bucket_sizes.bucket[MSGRNG_STNID_DMA_3];

		/* Configure the DMA credits */
		for (i = 0; i < 128; i++) {
			mmio[DMA_CC_CPU0_0 + i] =
			    shared_cc_table_dma.counters[i >> 3][i & 0x07];
		}
	} else {
		mmio[DMA_MSG_BUCKET0_SIZE] =
		    bucket_sizes.bucket[MSGRNG_STNID_DMA_0];
		mmio[DMA_MSG_BUCKET1_SIZE] =
		    bucket_sizes.bucket[MSGRNG_STNID_DMA_1];
		mmio[DMA_MSG_BUCKET2_SIZE] =
		    bucket_sizes.bucket[MSGRNG_STNID_DMA_2];
		mmio[DMA_MSG_BUCKET3_SIZE] =
		    bucket_sizes.bucket[MSGRNG_STNID_DMA_3];

		/* Configure the DMA credits */
		for (i = 0; i < 128; i++) {
			mmio[DMA_CC_CPU0_0 + i] =
			    cc_table_dma.counters[i >> 3][i & 0x07];
		}
	}
#ifdef NLM_BRIDGE_WKAROUND
	if (nlm_enable_br_wrkaround) {
		nlm_write_unlock_irq_restore(nlm_bridge_lock, flags);
	}
#endif
#ifdef CONFIG_PROC_FS
	xlr_init_dma_proc();
#endif
	xlr_dma_init_done = 1;
	printk("Initialized XLR DMA Controller, Channel 0 \n");
	tasklet_schedule(&xlr_dma_task);
	return 0;
}

#ifdef TEST_DMA
void xlr_dma_test(void)
{
	int i;
	uint8_t *ptr1, *ptr2;
	unsigned long s_jiffy, e_jiffy;

	ptr1 = (uint8_t *) kmalloc(0x1000, GFP_KERNEL);
	ptr2 = (uint8_t *) kmalloc(0x1000, GFP_KERNEL);
	if (!ptr1 || !ptr2) {
		if (ptr1)
			kfree(ptr1);
		if (ptr2)
			kfree(ptr2);

		printk("DMA test buffer alloc failed\n");
		return;
	}
	memset(ptr1, 0xa5, 0x1000);
	s_jiffy = read_c0_count();
	for (i = 0; i < 512; i++) {
		xlr_request_dma((uint64_t) virt_to_phys(ptr1),
				(uint64_t) virt_to_phys(ptr2), 0x1000);
	}
	e_jiffy = read_c0_count();
	if (memcmp(ptr1, ptr2, 0x1000)) {
		printk("DMA Data does not match. Test failed\n");
	} else
		printk("DMA Data Matches. Test Successful\n");

	printk("Start time = %lx end time = %lx\n", s_jiffy, e_jiffy);
	kfree(ptr1);
	kfree(ptr2);
}
#endif
EXPORT_SYMBOL(xlr_request_dma);
