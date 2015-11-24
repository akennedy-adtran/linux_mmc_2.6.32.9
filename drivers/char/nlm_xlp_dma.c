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

/* TODO - xlp_poll_vc0_messages has been removed from arch/mips/netlogic/xlp/on_chip.c.
 * Need to update this driver to work with the new on_chip.c - register a vc interrupt
 * handler for the DMA VC and enable the VC interrupt - see updated NAE driver for an
 * example.
 */
#error "nlm_xlp_dma.c has not been updated to work with the new on_chip.c"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#ifdef CONFIG_NLM_XLP
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/cpumask.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/nlm_dma.h>
#include <hal/nlm_hal.h>
#include <hal/nlm_hal_macros.h>
#include <hal/nlm_hal_fmn.h>
#include <hal/nlm_hal_xlp_dev.h>
#endif


#define CH0_CONTROL 8
#define MSGRNG_CODE_DMA 8
#define XLP_DMA_RESP_TIMEOUT 500
#define MAX_DMA_QUEUE_LEN 256 
#define PCIX_ACK_TIMER_VAL 0x18

#define MAX_DMA_TRANS_PER_CPU 256 
#define XLP_MAX_DMA_LEN_PER_DESC ((1 << 20) - 1) /* 1 MB - 1 */

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

#define XLP_DMA_DONE(x, seq) (x->trans[seq].pending == 0)

/* XLP DMA Global structures */
static u64 pcie_shared_membase;
static struct workqueue_struct *dma_wq;
/* We use 10 bit transaction id in the DMA message to uniquely identify a DMA
   response.
   0-7 indicate a sequence number (0 to 255)
   8-9 bits encode the CPU thread id (0 to 3)
   */
typedef struct dma_trans {
	volatile int pending;
	uint64_t dma_resp;
	void (*func) (void *,uint64_t);
	void *data;
}dma_trans_t;

typedef struct xlp_dma_ctrl {
	spinlock_t q_lock;
	int sequence_number;
	dma_trans_t trans[MAX_DMA_TRANS_PER_CPU];
}xlp_dma_ctrl_t;

volatile static int xlp_dma_producer=0;
volatile static int xlp_dma_consumer=0;
struct msgrng_msg xlp_dma_queue[MAX_DMA_QUEUE_LEN];
static int xlp_dma_init_done = 0;
xlp_dma_ctrl_t xlp_dma_ctrl[32];	// XXX change to percpu

spinlock_t xlp_dma_lock = SPIN_LOCK_UNLOCKED;
spinlock_t xlp_enqueue_dma_spin=SPIN_LOCK_UNLOCKED;
uint32_t xlp_total_dma_reqs, xlp_total_dma_bytes;
uint32_t xlp_dma_req_failed, xlp_dma_timeout_errors, xlp_dma_errors, 
	 xlp_dma_stale_resp;
uint32_t xlp_dma_msg_send_failed;

void xlp_async_dma_task(unsigned long data);
#define CONFIG_PROC_FS 1
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>

extern int xlp_loader_own_dma;
extern struct proc_dir_entry proc_root;
extern struct proc_dir_entry *nlm_root_proc;

static int xlp_dma_proc_read(char *page, char **start, off_t off,
		                        int count, int *eof, void *data)
{
	int len, total_len;
	char *ptr = page;

	if(count < 512) /* Need minimum of this space */
		return -EINVAL;
        memset(ptr, 0, count);

	total_len = 0;
	len  = sprintf(ptr, "Total DMA Requests = %d\n", xlp_total_dma_reqs);
	ptr += len;
	total_len += len;

	len  = sprintf(ptr, "Total DMA Bytes = %d\n", xlp_total_dma_bytes);
	ptr += len;
	total_len += len;

	len  = sprintf(ptr, "DMA Requests failed = %d\n", xlp_dma_req_failed);
	ptr += len;
	total_len += len;

	len  = sprintf(ptr, "DMA Timeout errors = %d\n", xlp_dma_timeout_errors);
	ptr += len;
	total_len += len;

	len  = sprintf(ptr, "DMA errors = %d\n", xlp_dma_errors);
	ptr += len;
	total_len += len;

	len  = sprintf(ptr, "DMA Stale responses = %d\n", xlp_dma_stale_resp);
	total_len += len;

	len = sprintf(ptr,"DMA Message Send Failed = %d\n",
			xlp_dma_msg_send_failed);
	total_len += len;

	return total_len;
}

struct proc_dir_entry *dma_procfs_dir;
EXPORT_SYMBOL(dma_procfs_dir);
static void xlp_init_dma_proc(void)
{
	struct proc_dir_entry *entry;

        dma_procfs_dir = proc_mkdir("dma_debug", NULL);
        if (dma_procfs_dir == NULL) {
                printk(" **********  cannot create proc dir ******* \n");
                return;
        }
        printk("CREATED dma_procfs_dir\n");

#if 0
	if (!(entry = create_proc_entry("xlp_dma_stats", 0444, dma_procfs_dir))) {
		printk("%s: create_proc_entry failed\n", __FUNCTION__);
		return;
	}
	entry->read_proc = xlp_dma_proc_read;
#else
        entry = create_proc_read_entry("xlp_dma_stats", 0444, dma_procfs_dir,
                                        xlp_dma_proc_read, NULL);
        if (!entry) {
                printk("%s: create_proc_entry failed\n", __FUNCTION__);
                return;
        }
#endif
        printk("xlp_dma_stats create_proc_entry passed\n");

}
void xlp_uninit_dma_proc(void)
{
	remove_proc_entry("xlp_dma_stat", dma_procfs_dir);
}
#endif

/* DMA message handler - Called from interrupt context */
static void xlp_dma_msgring_handler(uint32_t bucket, uint32_t stid,
		uint32_t size, uint32_t code, uint64_t msg0, uint64_t msg1,
		uint64_t msg2, uint64_t msg3, void *data/* ignored */)
{
	int cpu, tx_id, seq;
	xlp_dma_ctrl_t *ctrl;
	unsigned long flags;

	tx_id = msg1;
	cpu = (tx_id >> 8) & 0x1f;
	seq = (tx_id & 0xff);

	ctrl = &xlp_dma_ctrl[cpu];

	spin_lock_irqsave(&ctrl->q_lock, flags);
	/* Check if there was a pending request. This can happen if the
	   requestor times out and gives up the request. So in that case
	   do not update the response
	NOTE: One corner case that is not handled here is that when seq no
	wraps around and request was pending for the new one and this response
	was for the old request. This ideally must not happen.
	*/
	if(DMA_RESP_PENDING(ctrl, seq)){

		DMA_PUT_RESP(ctrl, seq, msg1);
		if(ctrl->trans[seq].func){
			ctrl->trans[seq].func(ctrl->trans[seq].data,msg1);
		}
		spin_unlock_irqrestore(&ctrl->q_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&ctrl->q_lock, flags);
	printk("ERROR: Stale response from DMA engine for transaction id %d\n",
			seq);
	spin_lock_irqsave(&ctrl->q_lock, flags);
	xlp_dma_stale_resp++;
	spin_unlock_irqrestore(&ctrl->q_lock, flags);
	return;
}

#define NLM_HAL_PCI_TXRXMSG0(src, reten, ep, attr, l2alloc, rdx)\
	((0xFFFFFFFFFFULL & (u64)src) | (((u64)(reten & 1)) << 45) |\
	(((u64) (ep & 1))<< 44) | (((u64)(attr & 3)) << 42) |\
	(((u64)(l2alloc & 0x1)) << 41) | (((u64)(rdx & 1)) << 40))

#define NLM_HAL_PCI_TXRXMSG1(dst, len)\
(\
	((u64)dst & 0xFFFFFFFFFFULL) | (((u64)(len) & 0xFFFFF) << 40)\
)

#define NLM_HAL_PCI_TXRXMSG2(id)\
(\
	(u64)id\
)

inline void xlp_build_xfer_msg(struct msgrng_msg *msg, uint64_t src, 
				uint64_t dest, uint32_t len, int tx_id,
				unsigned int station)
{
//	msg->msg0 = (1ULL << 63) | ((uint64_t)len << 40) | 
//				(src & 0xffffffffffULL);
//	msg->msg1 = (1ULL << 58) | ((uint64_t)tx_id << 48) |
//			((uint64_t)resp_bkt << 40) | (dest & 0xffffffffffull);

	msg->msg0 = NLM_HAL_PCI_TXRXMSG0(src, 1, 0, 0, 0, 0);
	msg->msg1 = NLM_HAL_PCI_TXRXMSG1(dest, (len-1));
	msg->msg2 = NLM_HAL_PCI_TXRXMSG2(tx_id);
	msg->msg3 = (uint64_t)station;
}

int xlp_enqueue_dma_msg(struct msgrng_msg  *msg)
{
	unsigned long mflags;
	spin_lock_irqsave(&xlp_enqueue_dma_spin,mflags);
	if(((xlp_dma_producer+1)%(MAX_DMA_QUEUE_LEN)) == xlp_dma_consumer){
		spin_unlock_irqrestore(&xlp_enqueue_dma_spin,mflags);
		return -ENOMEM;
	}
	//Enqueue Msg 
	xlp_dma_queue[xlp_dma_producer].msg0 = msg->msg0;
	xlp_dma_queue[xlp_dma_producer].msg1 = msg->msg1;
	xlp_dma_queue[xlp_dma_producer].msg2 = msg->msg2;
	xlp_dma_queue[xlp_dma_producer].msg3 = msg->msg3;
	xlp_dma_producer = (xlp_dma_producer + 1)%(MAX_DMA_QUEUE_LEN);
	spin_unlock_irqrestore(&xlp_enqueue_dma_spin,mflags);
	return 0;
}

DECLARE_TASKLET(xlp_dma_task,xlp_async_dma_task , 0);
void xlp_async_dma_task(unsigned long data)
{
	struct msgrng_msg *msg;
	static int last_msg_send_success=1;
	int msg_send_success=0;

	while((xlp_dma_consumer != xlp_dma_producer)){
		msg = &xlp_dma_queue[xlp_dma_consumer];
		msgrng_access_enable(flags);
		if(xlp_dma_consumer == xlp_dma_producer){
		}
		if(xlp_message_send((uint32_t)msg->msg3, 3, MSGRNG_CODE_DMA, 
				    (uint64_t *)msg)) {
			xlp_dma_msg_send_failed++;
			msgrng_access_disable(flags);
			last_msg_send_success = 1;
			msg_send_success = 0;
			tasklet_schedule(&xlp_dma_task);
			break;
		}
		msg_send_success = 1;
		last_msg_send_success = 1;
		xlp_dma_consumer = (xlp_dma_consumer + 1) % (MAX_DMA_QUEUE_LEN);
		msgrng_access_disable(flags);
	}
}

/* Returns 0 on success, -1 otherwise */
int xlp_async_request_dma(uint64_t src, uint64_t dest, uint32_t len,
			void (*func)(void *,uint64_t),void *data,
			enum dma_data_direction dir)
{
	int cpu;
	volatile xlp_dma_ctrl_t *ctrl;
	int tx_id, seq ;
	struct msgrng_msg  msg;
	unsigned long flags;
	unsigned int station;

	/* Driver does not support multiple descriptor DMA yet */
	if(len > XLP_MAX_DMA_LEN_PER_DESC) {
		printk(KERN_WARNING "%s: Cannot do DMA for more than %d bytes\n",
				__FUNCTION__, XLP_MAX_DMA_LEN_PER_DESC);
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	if(xlp_dma_init_done == 0) {
		printk(KERN_WARNING "%s: XLR DMA engine is not initialized\n", __FUNCTION__);
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}

	if (dir == DMA_TO_DEVICE)
		station = XLP_PCIE0_RX_BUCKET;
	else if (dir == DMA_FROM_DEVICE)
		station = XLP_PCIE0_TX_BUCKET;
	else {
		printk("Invalid DMA direction specified\n");
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}

	preempt_disable();
	cpu = hard_smp_processor_id();
	ctrl = (volatile xlp_dma_ctrl_t *)&xlp_dma_ctrl[cpu];
	preempt_enable();
	spin_lock_irqsave( (spinlock_t *)&(ctrl->q_lock),flags);

	if(DMA_SLOT_BUSY(ctrl)) {
		spin_unlock_irqrestore((spinlock_t *)&ctrl->q_lock,flags);
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	tx_id = (cpu << 8) | ctrl->sequence_number;
	seq = ctrl->sequence_number;
	DMA_SLOT_GET(ctrl);

	/*CallBack For Async Call.*/
	ctrl->trans[seq].func = func;
	ctrl->trans[seq].data = data;

	spin_unlock_irqrestore((spinlock_t *)&ctrl->q_lock,flags);
	/* Form the DMA simple xfer request and send to Channel 0 */

	xlp_build_xfer_msg(&msg, src, dest, len, tx_id, station);

	if(xlp_enqueue_dma_msg(&msg)){
		spin_lock_irqsave((spinlock_t *)&ctrl->q_lock, flags);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irqrestore((spinlock_t *)&ctrl->q_lock, flags);
		tasklet_schedule(&xlp_dma_task);
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	spin_lock_irqsave(&xlp_dma_lock, flags);
	xlp_total_dma_reqs++;
	xlp_total_dma_bytes += len;
	spin_unlock_irqrestore(&xlp_dma_lock, flags);
	tasklet_schedule(&xlp_dma_task);
	return 0;
}
EXPORT_SYMBOL(xlp_async_request_dma);

/* Returns 0 on success, -1 otherwise */
int xlp_request_dma(uint64_t src, uint64_t dest, uint32_t len, unsigned char dir)
{
	int cpu, i;
	xlp_dma_ctrl_t *ctrl;
	int tx_id, seq;
	struct msgrng_msg  msg = {0}, r_msg = {0};
	unsigned long flags;
	unsigned int station;


	/* Driver does not support multiple descriptor DMA yet */
	if(len > XLP_MAX_DMA_LEN_PER_DESC) {
		printk("%s: Cannot do DMA for more than %d bytes\n",
				__FUNCTION__, XLP_MAX_DMA_LEN_PER_DESC);
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	if(xlp_dma_init_done == 0) {
		printk("%s: XLR DMA engine is not initialized\n", __FUNCTION__);
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	if (dir == DMA_TO_DEVICE)
		station = XLP_PCIE0_RX_BUCKET;
	else if (dir == DMA_FROM_DEVICE)
		station = XLP_PCIE0_TX_BUCKET;
	else {
		printk("Invalid DMA direction specified\n");
		return -1;
	}

	preempt_disable();
	cpu = hard_smp_processor_id();
	ctrl = &xlp_dma_ctrl[cpu];
	preempt_enable();

	spin_lock_irqsave(&ctrl->q_lock, flags);
	if(DMA_SLOT_BUSY(ctrl)) {
		printk("%s: No space to enqueue this request\n", __FUNCTION__);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);

		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	tx_id = (cpu << 8) | ctrl->sequence_number;
	seq = ctrl->sequence_number;
	DMA_SLOT_GET(ctrl);

	spin_unlock_irqrestore(&ctrl->q_lock, flags);

	/*Reset Callback - As this is not async call*/	
	ctrl->trans[seq].func = NULL;
	ctrl->trans[seq].data= NULL;

	/* Form the DMA simple xfer request and send to Channel 0 */
	xlp_build_xfer_msg(&msg, src, dest, len, tx_id, station);
	msgrng_access_enable(flags);
//	if(message_send_retry(2, MSGRNG_CODE_DMA, MSGRNG_STNID_DMA_0, &msg)) {
	if (xlp_message_send(station, XLP_DMA_MSGSIZE, MSGRNG_CODE_DMA, (uint64_t *)&msg) != 0){
		printk("Message_send failed: Cannot submit DMA request to engine\n");
		msgrng_access_disable(flags);
		spin_lock_irqsave(&ctrl->q_lock, flags);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);

		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_req_failed++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	msgrng_access_disable(flags);
	/* wait for the response here */
	for(i=0; i < XLP_DMA_RESP_TIMEOUT; i++) {
		if(XLP_DMA_DONE(ctrl, seq)) break;
		udelay(50);
	}
	if(i == XLP_DMA_RESP_TIMEOUT) {
		printk("%s:Did not get response from DMA engine\n", 
						__FUNCTION__);
		spin_lock_irqsave(&ctrl->q_lock, flags);
		DMA_SLOT_PUT(ctrl, seq);
		spin_unlock_irqrestore(&ctrl->q_lock, flags);

		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_timeout_errors++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	/* Do some error checks */

	r_msg.msg0 = DMA_GET_RESP(ctrl, seq);
	if(r_msg.msg0 & 0x2) {
		printk("%s:DMA engine reported Message load error\n", 
				__FUNCTION__);
		spin_lock_irqsave(&xlp_dma_lock, flags);
		xlp_dma_errors++;
		spin_unlock_irqrestore(&xlp_dma_lock, flags);
		return -1;
	}
	spin_lock_irqsave(&xlp_dma_lock, flags);
	xlp_total_dma_reqs++;
	xlp_total_dma_bytes += len;
	spin_unlock_irqrestore(&xlp_dma_lock, flags);
	return 0;
}

extern void xlp_poll_vc0_messages (void);
struct delayed_work msg_poll_work[NR_CPUS];
#define NLM_DMA_MSG_POLL_DELAY 50
static void msg_poll_func(struct work_struct *work)
{
        int cpu;

        preempt_disable();
        xlp_poll_vc0_messages();
        cpu = smp_processor_id();
        preempt_enable();
        schedule_delayed_work_on(cpu,  &msg_poll_work[cpu], NLM_DMA_MSG_POLL_DELAY);
}

/*
 * This function is taken from nae driver
 */
static void config_fmn(void)
{
	struct cpumask cpumask;

	/* bind cpu to n0c0t0 and drain all leftover firmware messages */
	sched_bindto_save_affinity(0, &cpumask);
	/* Configure FMN again but only cpu credits */
	msgrng_access_enable(mflags);
	// xlp_msgring_int_handler(IRQ_MSGRING, NULL);
	/* Configure credits to non-n0c0 cores */
	/* Following two lines are commented out because
	 * nlm_hal_fmn_init() signature is changed. No longer can we do
	 * this call. FMN should be initialized before dma engine init */
	//printk(KERN_ERR "Potential collission. Change params of nlm_hal_fmn_init\n");
	//nlm_hal_fmn_init(0x10000000, 0x02000000, 50);
	msgrng_access_disable(mflags);
	sched_bindto_restore_affinity(&cpumask);
}

static int is_xlp_rc(int node, int fn)
{
	u32 mode = xlp_get_power_on_reset_cfg(node);

	mode &= (0xF << 19);
	mode >>= 19;
	return (mode & (1 << fn));
}

static int nlm_init_dma(void)
{
	int i;
	xlp_dma_ctrl_t *ctrl;

	if(is_xlp_rc(0, 0)){ /* Only node 0, fn 0 is configured as EP */
		printk(KERN_ERR "ERROR: Controller 0 is RC\n");
		return -ENODEV;
	}
	for_each_online_cpu(i){
		ctrl = &xlp_dma_ctrl[i];
		spin_lock_init(&ctrl->q_lock);
		ctrl->sequence_number = 0;
	}
	dma_wq = create_workqueue("xlp_dma");
	if (dma_wq == NULL) {
		return -EFAULT;
	}
	config_fmn();
	if (register_xlp_msgring_handler(XLP_MSG_HANDLE_PCIE0,
			xlp_dma_msgring_handler, NULL)) {
		printk("Couldn't register DMA msgring handler\n");
		return -1;
	}
	xlp_init_dma_proc();
	xlp_dma_init_done = 1;
	printk("Initialized XLR DMA Controller, Channel 0 \n");
	for_each_online_cpu(i) {
		INIT_DELAYED_WORK(&msg_poll_work[i], msg_poll_func);
		schedule_delayed_work_on(i, &msg_poll_work[i], NLM_DMA_MSG_POLL_DELAY);
	}
	tasklet_schedule(&xlp_dma_task);
	return 0;
}

/*
 * This function sets up the memory that is exposed as PCI memory to the host
 * PRM 22.8.7
 */
u64 setup_pcie_shared_memspace(u64 * len)
{
	u64 phys;
	u64 xlp_pci_base = XLP_BDF_BASE(0,1,0); /* fn = 0 for EP */
	u32 r;

	if (pcie_shared_membase == 0) {
		pcie_shared_membase = (u64) __get_free_pages(GFP_KERNEL, get_order(NLM_XLP_PCIE_SHARED_MEMSIZE));/* Reserve 32 MB */
		if (pcie_shared_membase == 0) {
			printk(KERN_WARNING "Alloc failed\n");
			return -ENOMEM;
		}
		phys = virt_to_phys((const volatile void *)pcie_shared_membase);
		if (phys == 0) {
			free_pages(pcie_shared_membase, get_order(NLM_XLP_PCIE_SHARED_MEMSIZE));
			printk(KERN_WARNING "virt_to_phys failed\n");
			return -ENOMEM;
		}
		/* program the PCIe address map registers : 22.8.7 PRM */
		nlm_hal_write_32bit_reg(xlp_pci_base, 0x251,
				((phys + (0 * 8 * 1024 * 1024)) >> 23));
		nlm_hal_write_32bit_reg(xlp_pci_base, 0x252,
				((phys + (1 * 8 * 1024 * 1024)) >> 23));
		nlm_hal_write_32bit_reg(xlp_pci_base, 0x253,
				((phys + (2 * 8 * 1024 * 1024)) >> 23));
		nlm_hal_write_32bit_reg(xlp_pci_base, 0x254,
				((phys + (3 * 8 * 1024 * 1024)) >> 23));
		*len = NLM_XLP_PCIE_SHARED_MEMSIZE;
		r = nlm_hal_read_32bit_reg(xlp_pci_base, 0x240);
		r |= 0x02000000;
		nlm_hal_write_32bit_reg(xlp_pci_base, 0x240, r);
	}
	return pcie_shared_membase;
}

EXPORT_SYMBOL(setup_pcie_shared_memspace);

#ifdef TEST_DMA
void xlp_dma_test(void)
{
	int i;
	uint8_t *ptr1, *ptr2;
	unsigned long s_jiffy, e_jiffy;

	ptr1 = (uint8_t *)kmalloc(0x1000, GFP_KERNEL);
	ptr2 = (uint8_t *)kmalloc(0x1000, GFP_KERNEL);
	if(!ptr1 || !ptr2){
		if(ptr1)
			kfree(ptr1);
		if(ptr2)
			kfree(ptr2);

		printk("DMA test buffer alloc failed\n");
		return;
	}
	memset(ptr1, 0xa5, 0x1000);
	s_jiffy = read_c0_count();
	for(i=0; i < 512; i++) {
		xlp_request_dma((uint64_t)virt_to_phys(ptr1),
				(uint64_t)virt_to_phys(ptr2), 0x1000);
	}
	e_jiffy = read_c0_count();
	if(memcmp(ptr1, ptr2, 0x1000)) {
		printk("DMA Data does not match. Test failed\n");
	}else
		printk("DMA Data Matches. Test Successful\n");

	printk("Start time = %lx end time = %lx\n", s_jiffy, e_jiffy);
	kfree(ptr1);
	kfree(ptr2);
}
#endif
EXPORT_SYMBOL(xlp_request_dma);

static void nlm_exit_dma(void)
{
	free_pages(pcie_shared_membase, get_order(NLM_XLP_PCIE_SHARED_MEMSIZE));
	return;
}

static inline void raise_intx(int ignore)
{
	/* Currently the device could be configured in INTx mode,
	 * MSI or MSI-X mode. Find out the mode before raising int.*/
	u64 xlp_pci_base = XLP_BDF_BASE(0,1,0); /* fn = 0 for EP */
	u32 r;

	/* We dont' have to read reg1 to check if INTx are disabled.
	 * PCIe core would take care of this (I hope) */
	r = nlm_hal_read_32bit_reg(xlp_pci_base, 0x240);
	r |= (1 << 22);
	nlm_hal_write_32bit_reg(xlp_pci_base, 0x240, r);
}

static inline void raise_msix(int vec)
{
	u64 xlp_pci_base = XLP_BDF_BASE(0,1,0); /* fn = 0 for EP */
	u32 r;

	r = ((1 << 5) | vec);
	nlm_hal_write_32bit_reg(xlp_pci_base, 0x25c, r);
}

static inline void raise_msi(int vec)
{
	u64 xlp_pci_base = XLP_BDF_BASE(0,1,0); /* fn = 0 for EP */
	u32 r;

	r = ((1 << 5) | vec);
	nlm_hal_write_32bit_reg(xlp_pci_base, 0x258, r);
	while (nlm_hal_read_32bit_reg(xlp_pci_base, 0x259) != 1){
		if (printk_ratelimit()){
			printk(KERN_WARNING "Still looping\n");
		}
	}
	nlm_hal_write_32bit_reg(xlp_pci_base, 0x259, 1);
}

void raise_host_interrupt(int vec)
{
	int msi, msix;
	u64 xlp_pci_base = XLP_BDF_BASE(0,1,0); /* fn = 0 for EP */

	msix = nlm_hal_read_32bit_reg(xlp_pci_base, 0x2C);
	msix = (msix >> 31) & 1;
	/* We don't care if fmask == 1 or not */
	if (msix != 0) {
		raise_msix(vec);
		return;
	}

	msi = (nlm_hal_read_32bit_reg(xlp_pci_base, 0x14) >> 16) & 1;
	if (msi != 0) {
		raise_msi(vec);
		return;
	}

	/* if MSI and MSI-X are not enabled, raise INTx, even if it is disabled
	 * in reg1, bit 10. We cannot distinguish between intX not enabled and
	 * intX disabled on purpose (e.g NAPI) */
	raise_intx(vec);
	return;
}
EXPORT_SYMBOL(raise_host_interrupt);

module_init(nlm_init_dma);
module_exit(nlm_exit_dma);
MODULE_LICENSE("GPL");
