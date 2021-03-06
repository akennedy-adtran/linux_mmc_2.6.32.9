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


#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/hardirq.h>
#include <linux/netdevice.h>
#include <asm/current.h>
#include <asm/atomic.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/utils.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/linux_crf.h>

#ifdef CONFIG_OCF_OCF_MODULE
#include <linux/crypto.h>
#include <cryptodev.h>
#endif /* CONFIG_OCF_OCF_MODULE */
#include "nlmsec_internal.h"
#include "ecc_ucode_data.h"

#define	ECC_UC_LOAD 0x70
MODULE_AUTHOR("Netlogic Microsystem Inc.");
MODULE_DESCRIPTION("XLS/XLR SAE Driver");
MODULE_LICENSE("Dual BSD/GPL");

typedef struct device_info *device_info_pt;

enum progress_type {
	NOT_IN_PROGRESS = 0,
	IN_PROGRESS,
	IN_WAIT_QUEUE
};

#define IS_IN_PROGRESS(m) ((m)->in_progress == IN_PROGRESS)
#define IS_IN_WAIT_QUEUE(m) ((m)->in_progress == IN_WAIT_QUEUE)

#define NR_CRYPTO_BITS 9
#define NR_PK_BITS 3
/* we use only the last 8 bits, so we offet result of
 * clz with this offset */
#define PK_BITMAP_OFFSET 24

#define NR_CRYPTO_OPS (1 << NR_CRYPTO_BITS)
#define NR_PK_OPS     (1 << NR_PK_BITS)

#define MAX_CRYPTO_OPS_INDEX   (NR_CRYPTO_OPS >> 5)

#define CRYPTO_BITMAP_INDEX(x) (x >> 5)
#define BITMAP_OFF_MASK(x)     ~(0x80000000 >> ((x) & 0x1f))
#define BITMAP_ON_MASK(x)       (0x80000000 >> ((x) & 0x1f))

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI

DECLARE_PER_CPU(struct napi_struct, xlr_napi_poll_struct);
extern struct net_device xlr_napi_dummy_dev;
void xlr_napi_poll_upper(struct net_device *dummy_dev, int *budget);
extern int xlr_napi_ready, nlm_on_chip_napi;
#define upper_buckets_nonempty() ((~msgrng_read_status() >> 28) & 0xf)
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */

extern __u32 cpu_to_frstid[];
static char driver_name[] = DRIVER_NAME;
static char debug_name[] __attribute__((unused)) = "debug";
static char stats_name[] = "stats";
static char NLMSAE_BUILD_VERSION[] = "1.7";
static const char *versionstr = NLMSAE_BUILD_VERSION;
extern int xlr_loader_support;
extern int xlr_loader_sharedcore;

static atomic_t vma_count __cacheline_aligned;
static uint64_t msgs_sent[NR_CPUS] __cacheline_aligned;
static uint64_t resp_recieved[NR_CPUS] __cacheline_aligned;
static uint64_t wait_count[NR_CPUS] __cacheline_aligned;
static uint64_t mmap_cnt[NR_CPUS] __cacheline_aligned __attribute__((unused));
static uint64_t mmapfree[NR_CPUS] __cacheline_aligned __attribute__((unused));
static uint64_t cipher_cnt[NR_CPUS][MAX_CIPHER_ALGO][MAX_CIPHER_MODE] __cacheline_aligned;
static uint64_t cipher_data_cnt[NR_CPUS][MAX_CIPHER_ALGO][MAX_CIPHER_MODE] __cacheline_aligned;
static uint64_t hash_cnt[NR_CPUS][MAX_HASH_ALGO] __cacheline_aligned;
static uint64_t hash_data_cnt[NR_CPUS][MAX_HASH_ALGO] __cacheline_aligned;
static uint64_t mod_exp_cnt[NR_CPUS][2] __cacheline_aligned;
static uint64_t ecc_prime_cnt[NR_CPUS][NLM_ECC_PRIME_CURVE_MAX][NLM_ECC_PRIME_OP_MAX] __cacheline_aligned __attribute__((unused));
static uint64_t ecc_binary_cnt[NR_CPUS][NLM_ECC_BINARY_CURVE_MAX][NLM_ECC_BINARY_OP_MAX] __cacheline_aligned __attribute__((unused));

static struct device_info {
	int version;
	dev_t device;
	struct proc_dir_entry *pdir;
	struct proc_dir_entry *pdebug;
	struct proc_dir_entry *pstats;
	struct cdev nlmsec_cdev;
#ifdef CONFIG_OCF_OCF_MODULE
	op_callback_t ocf_cb;
	softc_device_decl ocf_dev;
	int32_t ocf_id;
#endif /* CONFIG_OCF_OCF_MODULE */
	spinlock_t mem_lock;
	struct list_head pmem_list;

	spinlock_t crypto_lock;
	atomic_t crypto_used_slots;
	unsigned int max_crypto_used_slots;
	meminfo_pt crypto_ops_slot[NR_CRYPTO_OPS];
	unsigned int pk_used_slots;
	unsigned int max_pk_used_slots;
	uint32_t crypto_ops_bitmap[MAX_CRYPTO_OPS_INDEX];
	unsigned int bitmap_start;

	spinlock_t pkops_lock;
	volatile uint32_t pk_ops_bitmap;
	meminfo_pt pk_ops_slot[NR_PK_OPS];
	wait_queue_head_t crypto_queue;
	wait_queue_head_t pkop_queue;
	int exit_driver;
} dev_info;

extern void nlm_common_msgring_int_handler(unsigned int irq, struct pt_regs *regs);
extern int msgring_int_type;

static inline void remote_napi_poll_upper(void)
{
	unsigned long flags;
#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
	if(xlr_napi_ready && nlm_on_chip_napi &&
	   upper_buckets_nonempty()) {
		xlr_napi_poll_upper(&xlr_napi_dummy_dev, 0);
	} else if (in_softirq() && (!msgring_int_type)){
		local_irq_save(flags);
		nlm_common_msgring_int_handler(-1,NULL);
		local_irq_restore(flags);
	}
#else
	if (in_softirq() && (!msgring_int_type)) {
		local_irq_save(flags);
		nlm_common_msgring_int_handler(-1,NULL);
		local_irq_restore(flags);
	}
	return;
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */
}


#ifdef CONFIG_OCF_OCF_MODULE
int nlmsae_ocf_newsession(device_t dev, u_int64_t *sidp, struct cryptoini *cri);
int nlmsae_ocf_freesession(device_t dev, u_int64_t sid);
int nlmsae_ocf_process(device_t dev, struct cryptop *crp, int hint);

static device_method_t nlmsae_ocf_methods = {
	DEVMETHOD(cryptodev_newsession, nlmsae_ocf_newsession),
	DEVMETHOD(cryptodev_freesession, nlmsae_ocf_freesession),
	DEVMETHOD(cryptodev_process, nlmsae_ocf_process)
};

static struct {
	int ocf_algo_num;
	CipherAlgo_t c;
	CipherMode_t m;
	HashAlgo_t h;
	int hmac;
} algo_mapper[] = {
	{CRYPTO_DES_CBC, NLM_DES, NLM_CBC, HASH_BYPASS, 0},
	{CRYPTO_3DES_CBC, NLM_DES3, NLM_CBC, HASH_BYPASS, 0},
	{-1},/* CRYPTO_BLF_CBC */
	{-1},/* CRYPTO_CAST_CBC */
	{-1},/* CRYPTO_SKIPJACK_CBC */
	{CRYPTO_MD5_HMAC, CIPHER_BYPASS, NLM_ECB, NLM_MD5, 1},
	{CRYPTO_SHA1_HMAC, CIPHER_BYPASS, NLM_ECB, NLM_SHA1, 1},
	{-1},/* CRYPTO_RIPEMD160_HMAC */
	{-1},/* CRYPTO_MD5_KPDK */
	{-1},/* CRYPTO_SHA1_KPDK */
	{CRYPTO_AES_CBC, NLM_AES128, NLM_CBC, HASH_BYPASS, 0},
	{-1},/* ARC4 */
	{CRYPTO_MD5, CIPHER_BYPASS, NLM_ECB, NLM_MD5, 0},
	{CRYPTO_SHA1, CIPHER_BYPASS, NLM_ECB, NLM_SHA1, 0},
	{-1},/* CRYPTO_NULL_HMAC */
	{-1},/* CRYPTO_NULL_CBC */
	{-1},/* CRYPTO_DEFLATE_COMP */
	{CRYPTO_SHA2_256_HMAC, CIPHER_BYPASS, NLM_ECB, NLM_SHA256, 1},
	{CRYPTO_SHA2_384_HMAC, CIPHER_BYPASS, NLM_ECB, NLM_SHA384, 1},
	{CRYPTO_SHA2_512_HMAC, CIPHER_BYPASS, NLM_ECB, NLM_SHA512, 1},
	{-1},/* CRYPTO_CAMELLIA_CBC */
	{CRYPTO_SHA2_256, CIPHER_BYPASS, NLM_ECB, NLM_SHA256, 0},
	{CRYPTO_SHA2_384, CIPHER_BYPASS, NLM_ECB, NLM_SHA384, 0},
	{CRYPTO_SHA2_512, CIPHER_BYPASS, NLM_ECB, NLM_SHA512, 0},
	{-1},/* CRYPTO_RIPEMD160 */
};

int nlmsae_ocf_newsession(device_t dev, u_int64_t *sidp, struct cryptoini *cri)
{
	Crypto_Operation_pt cop;
	if(in_atomic() || in_interrupt()) {
		cop = (Crypto_Operation_pt)kmalloc(sizeof(Crypto_Operation_t),
						  GFP_ATOMIC);
	} else {
		cop = (Crypto_Operation_pt)kmalloc(sizeof(Crypto_Operation_t),
						  GFP_KERNEL);
	}
	if(cop == NULL) {
		return -ENOMEM;
	}

	memset(cop, 0, sizeof(Crypto_Operation_t));

	// populate all crypto fields here
	cop->c = algo_mapper[cri->cri_alg].c;
	cop->m = algo_mapper[cri->cri_alg].m;
	cop->key = cri->key;
	cop->key_len = cri->klen;
	cop->iv = cri->cri_iv;
	if(crp->flags & CRYPTO_OP_ENCRYPT) {
		cop->encrypt = CIPHER_ENCRYPT;
	}

	/* authentication parameters */
	if(cri->cri_next != NULL) {
		cri = cri->next;
		cop->h = algo_mapper[cri->cri_alg].h;
		if(algo_mapper[cri->cri_alg].hmac) {
			cop->hmac = (unsigned char *)cri->cri_key;
			cop->hmac_len = cri->cri_klen;
		}
	}


	*sidp = (u_int64_t)op;
	return 0;

	return nlmsec_op_init(sidp);
}

int nlmsae_ocf_freesession(device_t dev, u_int64_t sid)
{
	op_handle_t op = (op_handle_t)sid;
	return nlmsec_op_cleanup(&op);
}

int nlmsae_ocf_process(device_t dev, struct cryptop *crp, int hint)
{
	Crypto_Operation_pt *cop = (Crypto_Operation_pt)crp->crp_sid;
	op_handle_t op;
	int ret = NLMSAE_SUCCESS;

	ret = nlmsec_op_init(&op);
	if(ret) {
		goto bail;
	}

	if((ret = nlmsec_op_callback_setup(op, &dev_info.ocf_cb,
					  (unsigned long)op, 0))) {
		printk(KERN_ALERT "%s:%d Error while submitting operation to SAE.",
		       __FUNCTION__, __LINE__);
		nlmsec_op_cleanup(&op);
		goto bail;
	}

	ret = nlmsec_cipher_and_hash(op, cop->c, cop->m, cop->key, cop->iv,
				     0, (crp->crp_desc.crd_flags & CRD_F_ENCRYPT),
				     cop->h, cop->hmac,
				     0, /* hash_src */
				     0, /* hash_bytes_to_skip */
				     crp->crp_buf, crp->crp_ilen,

);
	crypto_copyback(crp->crp_flags, crp->crp_buf,
			crd->crd_inject, sw->u.hmac.sw_mlen, result);

	if(!IS_ASYNC_SUCCESS_SAEOP(ret)) {
		nlmsec_op_cleanup(&op);
	}
 bail:
	return ret;
}

void nlmsae_ocf_callback(int result, unsigned long arg)
{
}

#endif /* CONFIG_OCF_OCF_MODULE */

#ifdef CONFIG_64BIT
// doesn't read the old value
static __inline__ void ldadd_d_noread(long long value, uint64_t *addr)
{
  __asm__ __volatile__(
                       ".set push\n"
                       ".set noreorder\n"
                       "move $8, %2\n"
                       "move $9, %3\n"
//                       "ldaddd $8, $9\n"
		       ".word 0x71280012\n"
                       // "move %0, $8\n"
                       ".set pop\n"
                       : "=&r"(value), "+m"(*addr)
                       : "0" (value), "r"((unsigned long)addr)
                       : "$8", "$9"
                       );
}
#endif /* CONFIG_64BIT */

static inline int
message_send_fast_2(unsigned int code,
                    unsigned int stid,
                    unsigned long long msg0,
                    unsigned long long msg1)
{
        int ret;


        msgrng_load_tx_msg0(msg0);
        msgrng_load_tx_msg1(msg1);

        __asm__ __volatile__ (".set push\n"
                              ".set noreorder\n"
                              ".set mips64\n"
                              "sync\n"
                              "1:move $8, %1\n"
                              "c2 0x80001\n"
			      "nop\n"
			      "nop\n"
			      "nop\n"
			      "nop\n"
			      "nop\n"
                              "mfc2 $8, "STR(MSGRNG_MSG_STATUS_REG)"\n"
                              "andi $8, $8, 0x6\n"
                              "beqz $8, 2f\n"
                              "andi $8, 2\n"
                              "bnez $8, 1b\n"
                              "2:move %0, $8\n"
                              ".set pop\n"
                              :"=r"(ret)
                              : "r"((1<<16)|(code<<8)|stid)
                              : "$8"
                             );
        return ret;
}

static inline int add_meminfo_to_queue(secop_queue_pt q, meminfo_pt mem)
{
	unsigned long flags = 0;
	if(q->response_type == SECOP_Q) {
		spin_lock_irqsave(&q->q.lock, flags);

		if(q->q.max_length != -1 && q->q.length >= q->q.max_length) {
			spin_unlock_irqrestore(&q->q.lock, flags);
			return -EAGAIN;
		}

		q->q.length++;
#ifdef SAE_STATS
		if(q->q.length > q->q.stats_max_length) {
			q->q.stats_max_length = q->q.length;
		}
#endif /* SAE_STATS */
		mem->async_next = &q->q.head;
		q->q.tail->async_next = mem;
		q->q.tail = mem;

#ifdef DEBUG_QUEUE
		printk("%s:%d q=%p q->head=%p q->length=%d q->tail=%p "
		       "h->next=%p t->next=%p\n", __FILE__, __LINE__, q,
		       &q->q.head, q->q.length, q->q.tail, q->q.head.async_next,
		       q->q.tail->async_next);
#endif /* DEBUG_QUEUE */

		spin_unlock_irqrestore(&q->q.lock, flags);
	} else {
		q->cb.cbfunc(mem->result, mem->return_value);
	}

	return 0;
}

static inline meminfo_pt remove_meminfo_from_queue(secop_queue_pt q)
{
	meminfo_pt ret = NULL;
	unsigned long flags;

	if(q->response_type == SECOP_Q) {
		spin_lock_irqsave(&q->q.lock, flags);

		if(q->q.head.async_next != &q->q.head) {
			ret = q->q.head.async_next;
			q->q.length--;
			q->q.head.async_next = ret->async_next;

			if(ret->async_next == &q->q.head) {
				q->q.tail = &q->q.head;
			}
			ret->async_next = NULL;
		}

		spin_unlock_irqrestore(&q->q.lock, flags);
	}
	return ret;
}

#define ALL_SEC_STN 120
#define PK_STN 124
struct {
	wait_queue_head_t *wq;
	int stid;
} op_data[OP_ALL] = {
	{0, 0},
	{&dev_info.crypto_queue, ALL_SEC_STN},
	{&dev_info.pkop_queue, PK_STN},
	{&dev_info.pkop_queue, PK_STN}
};

struct page *nlmsec_vma_nopage(struct vm_area_struct *vma,
			       unsigned long address, int *type);

void nlmsec_vma_open(struct vm_area_struct *vma);
void nlmsec_vma_close(struct vm_area_struct * vma);
void write_magic(meminfo_pt mem, char __user *buf);

struct vm_operations_struct vmops = {
	.open = nlmsec_vma_open,
	.close = nlmsec_vma_close,
//	.nopage = nlmsec_vma_nopage,
};

#define MEMINFO_MATCHED_PTR(m, ptr) ((m)->ptr == ptr)
#define MEMINFO_MATCHED_VMA(m, vma) ((m)->vma == vma)

/*
 * Ops slot functions
 */
static int inline
find_free_crypto_slot(device_info_pt dinfo)
{
	unsigned int start = dinfo->bitmap_start % MAX_CRYPTO_OPS_INDEX, i, j;
 	uint32_t *ptr = &dinfo->crypto_ops_bitmap[0];

	i = start;
	do {
		j = find_32bit_1st_zero_bit(ptr[i]);
		if(j < 32) {
			ptr[i] |= BITMAP_ON_MASK(j);
			dinfo->bitmap_start = i;
			atomic_inc(&dinfo->crypto_used_slots);
			return ((i << 5) + j);
		}
		i = ((i + 1) % MAX_CRYPTO_OPS_INDEX);
	} while(i != start);

	return -1;
}

static int inline
alloc_crypto_op_slot(device_info_pt dinfo, meminfo_pt mem)
{
	unsigned long flags;
	int i;
	spin_lock_irqsave(&dinfo->crypto_lock, flags);
	i = find_free_crypto_slot(dinfo);
	spin_unlock_irqrestore(&dinfo->crypto_lock, flags);
	if(i > -1) {
		dinfo->crypto_ops_slot[i] = mem;
#ifdef SAE_STATS
		if(dinfo->max_crypto_used_slots < dinfo->crypto_used_slots.counter) {
			dinfo->max_crypto_used_slots = dinfo->crypto_used_slots.counter;
		}
#endif /* SAE_STATS */
	}
	return i;
}

static int inline
alloc_pk_ops_slot(device_info_pt dinfo, meminfo_pt mem)
{
	int j;
	unsigned long flags;
	spin_lock_irqsave(&dinfo->pkops_lock, flags);
	j = find_32bit_1st_zero_bit(dinfo->pk_ops_bitmap);
	if(j < 32) {
		dinfo->pk_ops_bitmap |= BITMAP_ON_MASK(j);
	}
	spin_unlock_irqrestore(&dinfo->pkops_lock, flags);
	if(j < 32) {
		j = j - PK_BITMAP_OFFSET;
		dinfo->pk_ops_slot[j] = mem;
		/* for PK Ops bit 3 needs to be set to 1 */
		j |= 0x8;
		ldadd_wu_no_read(1, &dinfo->pk_used_slots);
#ifdef SAE_STATS
		if(dinfo->max_pk_used_slots < dinfo->pk_used_slots) {
			dinfo->max_pk_used_slots = dinfo->pk_used_slots;
		}
#endif /* SAE_STATS */
	} else {
		j = -1;
	}
	return j;
}

static int inline
alloc_op_slot(device_info_pt dinfo, meminfo_pt mem)
{
	if(mem->op_type == NLM_CRYPTO_OP) {
		return alloc_crypto_op_slot(dinfo, mem);
	} else if(mem->op_type > NLM_CRYPTO_OP && mem->op_type < OP_ALL) {
		return alloc_pk_ops_slot(dinfo, mem);
	} else {
		printk("%s:%d Unknown op_type=%d\n", __FUNCTION__, __LINE__, mem->op_type);
	}
	return -1;
}

static void inline
clear_op_slot(device_info_pt dinfo, OpType_t op_type, int cs)
{
	unsigned long flags;

	if(op_type == NLM_CRYPTO_OP) {
		spin_lock_irqsave(&dinfo->crypto_lock, flags);
		dinfo->crypto_ops_bitmap[CRYPTO_BITMAP_INDEX(cs)] &= BITMAP_OFF_MASK(cs);
		atomic_dec(&dinfo->crypto_used_slots);
		spin_unlock_irqrestore(&dinfo->crypto_lock, flags);
	} else if(op_type > NLM_CRYPTO_OP && op_type < OP_ALL) {
		spin_lock_irqsave(&dinfo->pkops_lock, flags);
		dinfo->pk_ops_bitmap &= BITMAP_OFF_MASK(cs + PK_BITMAP_OFFSET);
		ldadd_w_no_read(-1, &dinfo->pk_used_slots);
		spin_unlock_irqrestore(&dinfo->pkops_lock, flags);
	} else {
		printk("%s:%d Unknown op_type=%d crypto=%d cs=%d\n", __FUNCTION__, __LINE__,
		       op_type, NLM_CRYPTO_OP, cs);
	}
	return;
}

/*
 * Proc memlist functions
 */
static void inline
free_proc_memlist(device_info_pt dinfo, proc_memlist_pt proc_info)
{
	meminfo_pt mem;
	memlist_pt tmp, m;
	int i = 0;

	if(proc_info == NULL) {
		printk(KERN_ALERT "%s:%d Invalid proc_memlist passed for "
		       "freeing.\n", __FUNCTION__, __LINE__);
		return;
	}

	list_for_each_safe(m, tmp, &proc_info->mem) {
		mem = (meminfo_pt) m;
		if(mem->owner != current->tgid) {
			printk(KERN_ALERT "%s:%d Memory attempted to be freed by "
			       "non-owner. owner=%d tgid=%d\n", __FUNCTION__, __LINE__,
			       mem->owner, current->tgid);
			return;
		}
		/*
		 * Waiting for existing operation to complete.
		 */
		while(IS_IN_PROGRESS(mem)) {
			i = 0;
			while(IS_IN_PROGRESS(mem) && i < 100000) ++i;
			if(IS_IN_PROGRESS(mem)) {
				printk(KERN_NOTICE "%s:%d Waiting for meminfo "
				       "in_progress state to change mem=%p\n",
				       __FUNCTION__, __LINE__, mem);
			}
		}
		free_meminfo(mem);
	}

	INIT_LIST_HEAD(&proc_info->mem);

	kfree(proc_info);
	return;
}

static proc_memlist_pt inline
find_proc_memlist(device_info_pt dev)
{
	proc_memlist_pt proc_info = (proc_memlist_pt)dev->pmem_list.next;
	while(proc_info != (proc_memlist_pt)&dev->pmem_list) {
		if(proc_info->tgid == current->tgid) {
			return proc_info;
		}
		proc_info = (proc_memlist_pt)proc_info->elem.next;
	}
	return NULL;
}

/*
 * Meminfo functions
 */
static inline meminfo_pt
find_meminfo_for_virt_addr(proc_memlist_pt proc_info, unsigned long start_addr)
{
	memlist_pt tmp;
	meminfo_pt mem;
	list_for_each(tmp, &proc_info->mem) {
		mem = (meminfo_pt)tmp;
		if(mem->owner == current->tgid) {
			if(mem->vma->vm_start == start_addr) {
				return mem;
			}
		} else {
			printk(KERN_ALERT "%s:%d Memory attempted to be freed by "
			       "non-owner. owner=%d tgid=%d\n", __FUNCTION__, __LINE__,
			       mem->owner, current->tgid);
		}
	}
	return 0;
}

void
free_meminfo(meminfo_pt mem)
{
	if(!mem) return;
	mem->magic = 0xBADBADBADBADBADAULL;
#ifdef SAE_STATS
#ifdef SYSTEM_CALL_STATS
	dev_info.mmapfree[hard_smp_processor_id()]++;
#endif /* SYSTEM_CALL_STATS */
#endif /* SAE_STATS */
	if(mem->ptr && mem->order > -1) {
		if(mem->ctx == PROCESS_CTX) {
			__free_pages((struct page *)mem->ptr, mem->order);
		} else {
			free_pages((unsigned long)mem->ptr, mem->order);
		}
	}
	mem->memlist.next = mem->memlist.prev = NULL;
	mem->ptr = NULL;
	mem->proc = NULL;
	mem->async_next = NULL;
	mem->owner = 0;
	mem->order = -1;
	kfree(mem);
	return;
}

static void inline
insert_meminfo(proc_memlist_pt proc_info, meminfo_pt mem)
{
	list_add_tail((memlist_pt)mem, &proc_info->mem);
	return;
}

static void inline
free_meminfo_for_proc(proc_memlist_pt proc_info, meminfo_pt mem)
{
	if(mem->owner == current->tgid) {
		list_del((memlist_pt)mem);
		free_meminfo(mem);
	} else {
		printk(KERN_ALERT "%s:%d Memory attempted to be freed by "
		       "non-owner. owner=%d tgid=%d\n", __FUNCTION__, __LINE__,
		       mem->owner, current->tgid);
	}
	return;
}

void nlmsec_vma_open(struct vm_area_struct *vma)
{
#ifdef DEBUG_2
	printk(KERN_NOTICE "%s:%d\n", __FUNCTION__, __LINE__);
#endif /* DEBUG */
	atomic_inc(&vma_count);
	return;
}

static meminfo_pt inline
alloc_meminfo(proc_memlist_pt proc_info, CALL_CTX c_ctx,
	      struct vm_area_struct *vma, int korder)
{
	unsigned long size;
	unsigned int nrpages, order;
	void *page_or_addr;
	gfp_t gfp_flags = GFP_KERNEL;
	int in_asi = (in_atomic() || in_softirq() || in_interrupt());

	meminfo_pt ret;

	if(c_ctx == PROCESS_CTX) {
		/* find required size */
		size = vma->vm_end - vma->vm_start;

		/* find next higher order of 2 */
		nrpages = CEIL(size, PAGE_SHIFT);

		order = (31 - find_32bit_1st_one_bit(nrpages));

		order += HAS_REMINDER(nrpages, order);
		if(order) {
			page_or_addr = alloc_pages(GFP_HIGHUSER|__GFP_COMP, order);
		} else {
			page_or_addr = alloc_pages(GFP_HIGHUSER, order);
		}
		if(!page_or_addr) {
			printk("%s:%d No memory allocated to kernel %p\n", __FUNCTION__,
			       __LINE__, page_or_addr);
		}
	} else {
		if(in_asi) {
		    gfp_flags = GFP_ATOMIC;
		}
		order = korder;
		if(order) {
			page_or_addr = (void *)__get_free_pages(gfp_flags|__GFP_COMP,
								order);
		} else {
			page_or_addr = (void *)__get_free_page(gfp_flags);
		}
	}

	if(page_or_addr != NULL) {
		ret = (meminfo_pt)kmalloc(sizeof(meminfo_t), gfp_flags);
		if(ret == NULL) {
			if(c_ctx == PROCESS_CTX) {
				__free_pages((struct page *)page_or_addr, order);
			} else {
				free_pages((unsigned long)page_or_addr, order);
			}
			printk(KERN_ALERT "%s:%d Error allocating meminfo "
			       "structure.\n", __FUNCTION__, __LINE__);
			return NULL;
		}
	} else {
		printk(KERN_ALERT "%s:%d Error allocating pages "
		       "order=%d\n", __FUNCTION__, __LINE__, order);
		return NULL;
	}

	memset(ret, 0, sizeof(meminfo_t));
	ret->magic = DRIVER_MAGIC;
	ret->order = order;
	ret->ptr = page_or_addr;
	ret->ctx = c_ctx;
	ret->vma = vma;
	ret->memlist.next = ret->memlist.prev = NULL;
	ret->async_next = NULL;
	ret->in_progress = NOT_IN_PROGRESS;
	init_waitqueue_head(&ret->wq);

	if(c_ctx == PROCESS_CTX) {
		ret->return_queue = &proc_info->async_queue;
		ret->return_value = vma->vm_start;
		ret->owner = current->tgid;
		ret->proc = proc_info;
		vma->vm_ops = &vmops;
		vma->vm_private_data = ret;
//		vma->vm_flags &= ~(VM_IO | VM_RESERVED);
		vma->vm_flags |= VM_DONTCOPY;
		nlmsec_vma_open(vma);
	}

#ifdef DEBUG_MEMINFO
	printk(KERN_NOTICE "%s:%d mem=%p order=%d page=%p\n",
	       __FUNCTION__, __LINE__, ret, ret->order, ret->ptr);
#endif /* DEBUG_MEMINFO */

	return ret;
}

meminfo_pt
alloc_meminfo_kernel_ctx(int order)
{
	meminfo_pt ret= alloc_meminfo(NULL, KERNEL_CTX, NULL, order);
	if(ret) {
		write_magic(ret, NULL);
	}
	return ret;
}

static meminfo_pt inline
alloc_meminfo_process_ctx(proc_memlist_pt proc_info,
			  struct vm_area_struct *vma)
{
	return alloc_meminfo(proc_info, PROCESS_CTX, vma, 0);
}

void nlmsec_vma_close(struct vm_area_struct *vma)
{
	meminfo_pt mem = (meminfo_pt)vma->vm_private_data;
	proc_memlist_pt proc_info;
	int i;

#ifdef DEBUG_2
	printk(KERN_NOTICE "%s:%d\n", __FUNCTION__, __LINE__);
#endif /* DEBUG */

	if(mem != NULL && mem->magic == DRIVER_MAGIC &&
	   mem->owner == current->tgid && mem->ptr != NULL && mem->order > -1) {
		proc_info = mem->proc;
		mem->proc = NULL;
		if(proc_info && proc_info->tgid == current->tgid) {
			do {
				i = 0;
				while(IS_IN_PROGRESS(mem) && i < 100000) ++i;
				if(IS_IN_PROGRESS(mem)) {
					printk(KERN_NOTICE "%s:%d Waiting for meminfo "
					       "in_progress state to change mem=%p\n",
					       __FUNCTION__, __LINE__, mem);
				}
			} while(IS_IN_PROGRESS(mem));
			spin_lock_bh(&dev_info.mem_lock);
			free_meminfo_for_proc(proc_info, mem);
			spin_unlock_bh(&dev_info.mem_lock);

			vma->vm_ops = NULL;
			vma->vm_private_data = NULL;
			atomic_dec(&vma_count);
		}
#if 0
	} else {
		if(mem) {
			printk(KERN_ALERT "%s:%d Invalid meminfo %p %"LLX_FMT,
			       __FUNCTION__, __LINE__, mem, mem->magic);
		} else {
			printk(KERN_ALERT "%s:%d meminfo is null",
			       __FUNCTION__, __LINE__);
		}
#endif
	}
	return;
}

#if 0
struct page *
nlmsec_vma_nopage(struct vm_area_struct *vma, unsigned long address, int *type)
{
	meminfo_pt m = (meminfo_pt)vma->vm_private_data;
	struct page *page = NOPAGE_SIGBUS;
	unsigned long offset = address - vma->vm_start;
	int pg_idx = offset >> PAGE_SHIFT;

	if(m && m->magic == DRIVER_MAGIC) {
		if(pg_idx > ((1 << m->order) - 1)) {
			printk(KERN_ALERT "%s:%d page=%p idx=%d count=%d map_count=%d "
			       "flags=%08lx\n", __FUNCTION__, __LINE__, page, pg_idx,
			       page->_count.counter, page->_mapcount.counter,
			       page->flags);
			return page;
		}

		page = ((struct page *)m->ptr) + pg_idx;
#ifdef DEBUG
		printk(KERN_NOTICE "%s:%d pid=%d tid=%d idx=%d pfn=0x%lx page=%p "
		       "address=0x%lx vma->vm_start=0x%lx\n",
		       __FUNCTION__, __LINE__, current->pid, current->tgid, pg_idx,
		       page_to_pfn(page), page, address, vma->vm_start);
#endif /* DEBUG */

		get_page(page);

		if(type)
			*type = VM_FAULT_MINOR;
	} else {
		printk(KERN_ALERT "%s:%d no page mapping found at addr=0x%lx\n",
		       __FILE__, __LINE__, address);
	}

	return page;
}
#endif /* 0 */

void insert_proc_memlist(device_info_pt dinfo, proc_memlist_pt proc_info)
{
	proc_info->elem.prev = dinfo->pmem_list.prev;
	proc_info->elem.next = &dinfo->pmem_list;

	dinfo->pmem_list.prev->next = (struct list_head *)proc_info;
	dinfo->pmem_list.prev = (struct list_head *)proc_info;
}

int nlmsec_open(struct inode *iptr, struct file *fptr)
{
#ifdef DEBUG_SYSTEM_CALL
	printk(KERN_NOTICE "%s:%d pid=%d tgid=%d ftpr=%p\n",
	       __FUNCTION__, __LINE__, current->pid, current->tgid, fptr);
#endif /* DEBUG_SYSTEM_CALL */
	fptr->private_data = (void *)&dev_info;
	return 0;
}

int nlmsec_flush(struct file *fptr, fl_owner_t id)
{
#ifdef DEBUG_SYSTEM_CALL
	printk(KERN_NOTICE "%s:%d pid=%d tgid=%d\n", __FUNCTION__, __LINE__,
	       current->pid, current->tgid);
#endif /* DEBUG_SYSTEM_CALL */
	return 0;
}

int nlmsec_release(struct inode *iptr, struct file *fptr)
{
	proc_memlist_pt proc_info;
	device_info_pt dinfo = (device_info_pt)fptr->private_data;
	if(current->pid != current->tgid) {
		printk(KERN_DEBUG "%s:%d thread exit pid=%d tgid=%d\n",
		       __FUNCTION__, __LINE__, current->pid, current->tgid);
		return 0;
	}

	if(dinfo) {
		fptr->private_data = NULL;
		spin_lock_bh(&dinfo->mem_lock);
		proc_info = find_proc_memlist(dinfo);
		if(proc_info) {
			/* dequeue proc_info */
			proc_info->elem.prev->next = proc_info->elem.next;
			proc_info->elem.next->prev = proc_info->elem.prev;
			proc_info->elem.prev = proc_info->elem.next = NULL;
		}
		spin_unlock_bh(&dinfo->mem_lock);
		if (proc_info) free_proc_memlist(dinfo, proc_info);
	}

#ifdef DEBUG_SYSTEM_CALL
	printk(KERN_NOTICE "%s:%d pid=%d tgid=%d\n", __FUNCTION__, __LINE__,
	       current->pid, current->tgid);
#endif /* DEBUG_SYSTEM_CALL */
	return 0;
}

void write_magic(meminfo_pt mem, char __user *buf)
{
	uint64_t ptr2 = 0;
	control_struct_t ctrl;
	control_struct_t *ptr;

	if(mem->ctx == PROCESS_CTX) {
		ptr = &ctrl;
	} else {
		ptr = mem->ptr;
	}

	memset(ptr, 0, sizeof(control_struct_t));
	if(ptr != NULL) {
		ptr->version = dev_info.version;
		ptr->owner = current->tgid;
		if(mem->ctx == PROCESS_CTX && ptr->owner == 0) {
			printk("%s:%d tgid=0x%x\n", __FUNCTION__, __LINE__,
			       current->tgid);
/* 		} else { */
/* 			printk("%s:%d tgid=0x%x\n", __FUNCTION__, __LINE__, */
/* 			       current->tgid); */
		}
		ptr->magic = DRIVER_MAGIC;
		if(mem->ctx == PROCESS_CTX) {
			ptr2 = page_to_pfn((struct page *)(mem->ptr));
			ptr2 <<= PAGE_SHIFT;
		} else {
			ptr2 = virt_to_phys(mem->ptr);
		}
		ptr->phy_addr = ptr2;
		ptr->mem_addr = (uint64_t)((unsigned long)mem);
		ptr->msg0 = ptr->msg1 = 0ULL;
	} else {
		printk(KERN_ALERT "%s:%d mem=%p ctx=%d ptr=NULL\n",
		       __FUNCTION__, __LINE__, mem, mem->ctx);
		return;
	}

	ptr = NULL;
	if(mem->ctx == PROCESS_CTX) {
		copy_to_user(buf, &ctrl, sizeof(control_struct_t));
	}

#ifdef DEBUG_MEMINFO
	printk(KERN_NOTICE "%s:%d version=%x mem=%p ctx=%d buf=%p mptr=%p "
	       "ptr2=0x%" LLX_FMT "\n",
	       __FUNCTION__, __LINE__, dev_info.version, mem, mem?mem->ctx:-1,
	       buf, mem->ptr, ptr2);
#endif /* DEBUG_MEMINFO */
	return;
}


int nlmsec_mmap(struct file *fptr, struct vm_area_struct *vma)
{
	proc_memlist_pt proc_info;
	meminfo_pt mem;
	device_info_pt dinfo = (device_info_pt)fptr->private_data;
	int ret = 0;
	unsigned long pfn;
//	unsigned long flags;

	if(!dinfo) {
		printk(KERN_NOTICE "%s:%d pid=%d tgid=%d ftpr=%p\n",
		       __FUNCTION__, __LINE__, current->pid, current->tgid, fptr);
		return -EINVAL;
	}
#ifdef DEBUG_SYSTEM_CALL
	printk(KERN_NOTICE "%s:%d pid=%d tgid=%d ftpr=%p\n",
	       __FUNCTION__, __LINE__, current->pid, current->tgid, fptr);
#endif /* DEBUG_SYSTEM_CALL */

	spin_lock_bh(&dinfo->mem_lock);

	proc_info = find_proc_memlist(dinfo);
#ifdef SAE_STATS
#ifdef SYSTEM_CALL_STATS
	dinfo->mmapcnt[hard_smp_processor_id()]++;
#endif /* SYSTEM_CALL */
#endif /* SAE_STATS */
	spin_unlock_bh(&dev_info.mem_lock);

	if(proc_info == NULL) {
		proc_info = (proc_memlist_pt)kmalloc(sizeof(proc_memlist_t),
						     GFP_KERNEL);
		if(proc_info != NULL) {
			INIT_SECOP_QUEUE(&proc_info->async_queue, -1);
			proc_info->tgid = current->tgid;
			INIT_LIST_HEAD(&proc_info->mem);
			insert_proc_memlist(dinfo, proc_info);
		} else {
			printk("%s:%d Error allocating proc_memlist.\n",
			       __FUNCTION__, __LINE__);
			ret = -ENOMEM;
			goto bail;
		}
	}

	mem = alloc_meminfo_process_ctx(proc_info, vma);
	if(mem == NULL) {
		printk("%s:%d Error allocating meminfo.\n", __FUNCTION__,
		       __LINE__);
		ret = -ENOMEM;
		goto bail;
	}

	spin_lock_bh(&dinfo->mem_lock);
	insert_meminfo(proc_info, mem);
#if 1
	pfn = page_to_pfn((struct page *)mem->ptr);
	if((ret = remap_pfn_range(vma, vma->vm_start,
				  pfn, vma->vm_end - vma->vm_start,
				  vma->vm_page_prot)) < 0) {
		printk("%s:%d ret=%d\n", __FUNCTION__, __LINE__, ret);
	}
//	printk("%s:%d pfn=0x%lx page=%p\n", __FUNCTION__, __LINE__, pfn, mem->ptr);
#endif
	spin_unlock_bh(&dinfo->mem_lock);


 bail:
	return ret;
}

/*
 * Send the operation pointed by meminfo to the crypto engine.
 * Params:
 * mem - Memory structure where the operation exists
 * allow_sync - If asynchronous dispatch of this operation is allowed
 * Returns:
 * EAGAIN         : if the operation could not be sent to security engine
 *                  due to credit failure
 * NLMSAE_SUCCESS : If the operation succeded
 */

int send_to_sae(meminfo_pt mem, int allow_async)
{
	device_info_pt dinfo = &dev_info;
	int ret = NLMSAE_SUCCESS;
	int cs, stid;
	unsigned long flags;
	OpType_t op_type = mem->op_type;
	struct msgrng_msg msg = {0ULL};
	unsigned int t, hcpuid;
	int in_asi = (in_atomic() || in_softirq() || in_interrupt());

	if ((op_type != NLM_CRYPTO_OP) && (op_type != NLM_RSA_OP) &&
	    (op_type != NLM_ECC_OP)) {
		printk(KERN_ALERT "%s:%d Invalid op_type=%d mem=%p page=%p\n",
		       __FUNCTION__, __LINE__, op_type, mem, mem->ptr);
		return -EINVAL;
	}

	do {
		cs = alloc_op_slot(dinfo, mem);
		if(cs < 0) {
			if(!in_asi &&
			   waitqueue_active(op_data[op_type].wq)) {
				init_waitqueue_entry(&mem->wqe, current);
				add_wait_queue_exclusive(&mem->wq, &mem->wqe);
				schedule_timeout(1);
				remove_wait_queue(op_data[op_type].wq,
						  &mem->wqe);
			}
		}
	} while(cs < 0);

	/* set response bucket and callslot.  That's the only
	 * information userspace driver can't set.
	 */
	msg.msg0 = mem->msg0;
	msg.msg1 = mem->msg1;

	if(op_type == NLM_CRYPTO_OP) {
		SET_CTRL_SCRATCH_VALUE(msg.msg0, cs);
		SET_DATA_SCRATCH_VALUE(msg.msg1, cs);
	} else if(op_type == NLM_RSA_OP || op_type == NLM_ECC_OP) {
		/* bit 3 needs to be set to 1 for public key ops */
		SET_CTRL_PK_SCRATCH_VALUE(msg.msg0, cs);
		SET_DATA_PK_SCRATCH_VALUE(msg.msg1, cs);
	}

	stid = op_data[op_type].stid;

	if(is_xls() && op_type != NLM_CRYPTO_OP) {
		stid = MSGRNG_STNID_XLS_PK0;
	}


#ifdef DEBUG_MSGRNG
	printk(KERN_NOTICE "%s:%d stid=%d ctrl_msg=0x%016" LLX_FMT
	       " data_msg=0x%016" LLX_FMT " mem=%p cs=0x%x op_type=%d\n",
	       __FUNCTION__, __LINE__, stid, msg.msg0, msg.msg1, mem,
	       cs, op_type);
#endif /* DEBUG_MSGRNG */

	mem->resp0 = mem->resp1 = 0ULL;
	mem->in_progress = IN_PROGRESS;

	if(!in_asi) {
		init_waitqueue_entry(&mem->wqe, current);
		add_wait_queue(&mem->wq, &mem->wqe);
	}

	do {
		t = 0;
		msgrng_access_enable(flags);
		hcpuid = hard_smp_processor_id();
		remote_napi_poll_upper();
		SET_FREEBACK_STN(msg.msg1, (cpu_to_frstid[hcpuid]));

		ret = message_send_fast_2(0, stid, msg.msg0, msg.msg1);
		if(ret != 0) {
#ifdef DEBUG
			printk(KERN_WARNING "%s:%d Error returned=%d cs=%d op_type=%d\n",
			       __FUNCTION__, __LINE__, ret, cs, op_type);
#endif /* DEBUG */
#ifdef SAE_STATS
			wait_count[hcpuid]++;
#endif /* SAE_STATS */
			if(allow_async && IS_ASYNC_OP(mem)) {
				msgrng_access_disable(flags);
				clear_op_slot(dinfo, op_type, cs);
				ret = -EAGAIN;
				mem->in_progress = NOT_IN_PROGRESS;
				goto out;
			} else {
				if(++t == 100000) {
					printk(KERN_WARNING
					       "%s:%d Waiting to send message to SAE "
					       "op_type=%d stid=%x ret=0x%x\n",
					       __FUNCTION__, __LINE__, op_type, stid, ret);
					t = 0;
				}
			}
		}
		msgrng_access_disable(flags);
	} while(ret);
#ifdef SAE_STATS
	msgs_sent[hcpuid]++;
#endif
	t = 0;

	// wait only if the ASYNC flag is not set
	if(!IS_ASYNC_OP(mem)) {
		if(op_type == NLM_CRYPTO_OP) {
			while(IS_IN_PROGRESS(mem)) {
				if(dinfo->crypto_used_slots.counter > 60 && !in_asi) {
					schedule();
				}
				msgrng_access_enable(flags);
				remote_napi_poll_upper();
				msgrng_access_disable(flags);
				if(++t == 10000000) {
					printk("Stuck here\n");
					t = 0;
				}
			}
		} else {
			while(IS_IN_PROGRESS(mem)) {
				if(dinfo->pk_used_slots > 4 && !in_asi) {
					schedule();
				}
				if(++t == 10000000) {
					printk("Stuck here\n");
					t = 0;
				}
				msgrng_access_enable(flags);
				remote_napi_poll_upper();
				msgrng_access_disable(flags);
			}

#if 0
			printk("%s:%d th=%p qh=%p wq_active=%d\n",
			       __FUNCTION__, __LINE__, mem->wqe.task_list.next,
			       op_data[op_type].wq,
			       waitqueue_active(op_data[op_type].wq));
			printk("%s:%d qh=%p\n", __FUNCTION__, __LINE__,
			       op_data[op_type].wq);
#endif
		}
		ret = mem->result;
	}

 out:
	if(!in_asi) {
		remove_wait_queue(&mem->wq, &mem->wqe);
	}
	return ret;
}

#if 0
static void print_meminfo_proclist(device_info_pt dinfo)
{
	proc_memlist_pt proc_info;
	memlist_pt tmp;
	meminfo_pt mem;

	spin_lock_bh(&dinfo->mem_lock);
	proc_info = find_proc_memlist(dinfo);
	if (proc_info) {
		list_for_each(tmp, &proc_info->mem) {
			mem = (meminfo_pt)tmp;
			printk(KERN_ALERT "[%s:%d] magic=%llx op_type=%x mem=%p page=%p, "
			       "msg0=%" LLX_FMT ", msg1=%" LLX_FMT "\n",
			       __FUNCTION__, __LINE__, mem->magic, mem->op_type,
			       mem, mem->ptr, mem->msg0, mem->msg1);
		}
	}
	spin_unlock_bh(&dinfo->mem_lock);
}
#endif

static long
nlmsec_ioctl(struct file *fptr, unsigned int type, unsigned long val)
{
	char __user *buf = (char __user *)val;
	device_info_pt dinfo = (device_info_pt)fptr->private_data;
	proc_memlist_pt proc_info;
	meminfo_pt mem = NULL;
	int ret = 0;

	if(type == NLMSEC_IOCTL_GET_MEMINFO) {
		spin_lock_bh(&dinfo->mem_lock);
		proc_info = find_proc_memlist(dinfo);

		mem = find_meminfo_for_virt_addr(proc_info, val);
		if(mem) {
			write_magic(mem, buf);
		} else {
			ret = -2;
		}
		spin_unlock_bh(&dinfo->mem_lock);
	} else {
		ret = -1;
	}

#ifdef DEBUG
	printk("%s:%d type=%d val=%lx ret=%d\n", __FUNCTION__, __LINE__, type, val, ret);
#endif /* DEBUG */
	return ret;
}

// no range checking... Take care while calling this function
void xlr_inc_enc_stat(int cipher_algo, int cipher_mode, int data_size)
{
	int cpu = hard_smp_processor_id();

	if(cipher_algo != CIPHER_BYPASS) {
		cipher_cnt[cpu][cipher_algo][cipher_mode]++;
		cipher_data_cnt[cpu][cipher_algo][cipher_mode] += data_size;
	}
}

// no range checking... Take care while calling this function
void xlr_inc_auth_stat(int hash_algo, int data_size)
{
	int cpu = hard_smp_processor_id();

	if(hash_algo != HASH_BYPASS) {
		hash_cnt[cpu][hash_algo]++;
		hash_data_cnt[cpu][hash_algo] += data_size;
	}
}

void
add_stats_count(control_struct_pt ctrl, int cpu)
{
	if(ctrl->op_type == NLM_CRYPTO_OP) {
		if(ctrl->cipher_algo != CIPHER_BYPASS) {
			cipher_cnt[cpu][ctrl->cipher_algo][ctrl->cipher_mode]++;
			cipher_data_cnt[cpu][ctrl->cipher_algo][ctrl->cipher_mode] += ctrl->data_size;
		}

		if(ctrl->hash_algo != HASH_BYPASS) {
			hash_cnt[cpu][ctrl->hash_algo]++;
			hash_data_cnt[cpu][ctrl->hash_algo] += ctrl->data_size;
		}
	}

	if(ctrl->op_type == NLM_RSA_OP) {
		if(ctrl->rsa_algo == BIT_512 ||
		   ctrl->rsa_algo == BIT_1024) {
			mod_exp_cnt[cpu][ctrl->rsa_algo - 1]++;
		}
	}

	if(ctrl->op_type == NLM_ECC_OP) {
		if(ctrl->ecc_degree < NLM_ECC_PRIME_CURVE_MAX) {
			ecc_prime_cnt[cpu][ctrl->ecc_degree][ctrl->ecc_algo]++;
		} else {
			ecc_binary_cnt[cpu][ctrl->ecc_degree -
					    NLM_ECC_BINARY_163][ctrl->ecc_algo]++;
		}
	}

	return;
}

static ssize_t
nlmsec_read(struct file *fptr, char __user *cptr, size_t size,
	     loff_t *off)
{
	uint64_t magic;
	pid_t owner_pid;
	int ret = 0;
	meminfo_pt mem = NULL;
	device_info_pt dinfo = (device_info_pt)fptr->private_data;
	control_struct_t ctrl;
	int async = 0;

	if(size != sizeof(control_struct_t)) {
		printk(KERN_ALERT "%s:%d Incorrect operation attempted.\n",
		       __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	copy_from_user(&ctrl, cptr, sizeof(control_struct_t));

	if(ctrl.magic != DRIVER_MAGIC) {
		printk(KERN_ALERT "%s:%d Invalid call to write system call id=%"
		       LLX_FMT "\n",
		       __FUNCTION__, __LINE__, ctrl.magic);
		return -EINVAL;
	}


	mem = (meminfo_pt)((unsigned long)ctrl.mem_addr);

#ifdef DEBUG
	printk(KERN_NOTICE "%s:%d tgid=0x%x pid=0x%x ptr=%p size=%u magic=%llx "
	       "owner=0x%x mem=%p\n",
	       __FUNCTION__, __LINE__, current->tgid, current->pid, cptr,
	       (unsigned int)size, ctrl.magic, ctrl.owner, mem);
#endif /* DEBUG */

	spin_lock_bh(&dinfo->mem_lock);

	if(mem == NULL || mem->ptr == NULL || mem->magic != DRIVER_MAGIC) {
		printk(KERN_ALERT "%s:%d Unknown operation structure mem=%p magic=%llx\n",
		       __FUNCTION__, __LINE__, mem, mem?mem->magic:0ULL);

//		print_meminfo_proclist(dinfo);
		ret = -EINVAL;
		goto bail;
	}

	if(mem->owner != current->tgid) {
		printk(KERN_ALERT "%s:%d The memory region was allocated by "
		       "another process pid=%d and now being accessed by process %d",
		       __FUNCTION__, __LINE__, mem->owner, current->tgid);
		ret = -EFAULT;
		goto bail;
	}

	magic = ctrl.magic;
	mem->msg0 = ctrl.msg0;
	mem->msg1 = ctrl.msg1;
	if(async) {
		mem->op_flags = OP_NO_WAIT;
	}
	mem->result = NLMSAE_SUCCESS;
//	mem->return_queue = proc_info->async_queue;
//	mem->return_value = proc_info;
	mem->op_type = ctrl.op_type;
	mem->in_progress = IN_PROGRESS;
	owner_pid = ctrl.owner;
 bail:
	spin_unlock_bh(&dinfo->mem_lock);
	if(ret) {
		printk("%s:%d Aborting operation due to error=%d\n",
		       __FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = -1;
	/* Check magic of the memory passed to kernel */
	if ((magic != DRIVER_MAGIC) || (current->tgid != owner_pid) ||
	     ((mem->op_type != NLM_CRYPTO_OP) &&
	      (mem->op_type != NLM_RSA_OP) && (mem->op_type != NLM_ECC_OP))) {

		printk(KERN_ALERT "%s:%d magic=%" LLX_FMT
		       " pid=0x%x owner=0x%x op_type=%x mem=%p page=%p, msg0=%" LLX_FMT
		       ", msg1=%" LLX_FMT "\n", __FUNCTION__, __LINE__, magic,
		       current->tgid, owner_pid, mem->op_type, mem, mem->ptr,
		       mem->msg0, mem->msg1);
		mem->in_progress = NOT_IN_PROGRESS;
		return -EFAULT;
	}

	ret = send_to_sae(mem, async);

#ifdef DEBUG_3
	copy_from_user(&junk_buf[0], cptr, 4096);
	print_4k(&junk_buf[0], 0);
#endif /* DEBUG_2 */
	if(IS_SUCCESS_SAEOP(ret)) {
		add_stats_count(&ctrl, hard_smp_processor_id());
		/* copying response back */
		ctrl.msg0 = mem->resp0;
		ctrl.msg1 = mem->resp1;
		copy_to_user(cptr, &ctrl, sizeof(control_struct_t));
		ret = mem->result;
	}

	return ret;
}

struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = nlmsec_open,
    .mmap = nlmsec_mmap,
    .read = nlmsec_read,
//    .write = nlmsec_write,

    .unlocked_ioctl = nlmsec_ioctl,
//    .flush = nlmsec_flush,
    .release = nlmsec_release,
};

static int
print_stats_info(device_info_pt dinfo, char *buf, int size)
{
	int i, x, t = size, p, q;
	uint64_t m_sent = 0, r_received = 0, w_count = 0;
	uint64_t c_cnt[MAX_CIPHER_ALGO][MAX_CIPHER_MODE] = {{0}};
	uint64_t c_data_cnt[MAX_CIPHER_ALGO][MAX_CIPHER_MODE] = {{0}};
	uint64_t h_cnt[MAX_HASH_ALGO] = {0};
	uint64_t h_data_cnt[MAX_HASH_ALGO] = {0};
	uint64_t m_exp_cnt[2] = {0};
	uint64_t ec_p_cnt[NLM_ECC_PRIME_CURVE_MAX][NLM_ECC_PRIME_OP_MAX] = {{0}};
	uint64_t ec_b_cnt[NLM_ECC_BINARY_CURVE_MAX][NLM_ECC_BINARY_OP_MAX] = {{0}};

#ifdef SYSTEM_CALL_STATS
	uint64_t map_cnt = 0, map_free = 0;
#endif /* SYSTEM_CALL_STATS */
	x = snprintf(buf, t, "CPU\t\tSent\t\tReceived\n");
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	for(i = 0; i < NR_CPUS; ++i) {
		x = snprintf(buf, t, "%d\t\t%" LLU_FMT "\t\t%" LLU_FMT "\n", i,
			     msgs_sent[i], resp_recieved[i]);
		t -= x;
		buf += x;
		if(t <= 0) {
			goto end;
		}

		m_sent += msgs_sent[i];
		r_received += resp_recieved[i];
		w_count += wait_count[i];
		for(p = 1; p < MAX_CIPHER_ALGO; ++p) {
			for(q = 0; q < MAX_CIPHER_MODE; ++q) {
				if(valid_cipher_algo_mode_matrix[p][q]) {
					c_cnt[p][q] += cipher_cnt[i][p][q];
					c_data_cnt[p][q] += cipher_data_cnt[i][p][q];
//					cipher_cnt[i][p][q] = cipher_data_cnt[i][p][q] = 0;
				}
			}
		}

		for(p = 1; p < MAX_HASH_ALGO-2; ++p) {
			h_cnt[p] += hash_cnt[i][p];
			h_data_cnt[p] += hash_data_cnt[i][p];
//			hash_cnt[i][p] = hash_data_cnt[i][p] = 0;
		}

		if(is_xls()) {
			for(p = NLM_ECC_PRIME_160;
			    p < NLM_ECC_PRIME_CURVE_MAX; ++p) {
				for(q = NLM_ECC_PRIME_P_MUL; q < NLM_ECC_PRIME_OP_MAX;
				    ++q) {
					ec_p_cnt[p][q] += ecc_prime_cnt[i][p][q];
//					ecc_prime_cnt[i][p][q] = 0;
				}
			}

			for(p = 0; p < NLM_ECC_BINARY_CURVE_MAX; ++p) {
				for(q = NLM_ECC_BINARY_P_MUL;
				    q < NLM_ECC_BINARY_OP_MAX; ++q) {
					ec_b_cnt[p][q] += ecc_binary_cnt[i][p][q];
//					ecc_binary_cnt[i][p][q] = 0;
				}
			}
		}
#ifdef SYSTEM_CALL_STATS
		map_cnt += mmapcnt[i];
		map_free += mmapfree[i];
		mmapcnt[i] = mmapfree[i] =
#endif /* SYSTEM_CALL_STATS */
//			wait_count[i] =
//			msgs_sent[i] =
//			resp_recieved[i] = 0;
	}

	x = snprintf(buf, t, "Total msgs\t\t%"LLU_FMT "\t\t%" LLU_FMT "\n",
		     m_sent, r_received);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	x = snprintf(buf, t, "Cipher Algorithms:\n");
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	for(p = 1; p < MAX_CIPHER_ALGO; ++p) {
		for(q = 0; q < MAX_CIPHER_MODE; ++q) {
			if(valid_cipher_algo_mode_matrix[p][q] && c_cnt[p][q]) {
				x = snprintf(buf, t,
					     "%11s-%-4s Count: %16" LLU_FMT
					     " Size: %16" LLU_FMT "\n",
					     cipher_str[p], mode_str[q], c_cnt[p][q],
					     c_data_cnt[p][q]);
				t -= x;
				buf += x;
				if(t <= 0) {
					goto end;
				}
			}
		}
	}

	x = snprintf(buf, t, "Hash Algorithms:\n");
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	for(p = 1; p < MAX_HASH_ALGO - 2; ++p) {
		if(h_cnt[p]) {
			x = snprintf(buf, t, "%11s Count: %16" LLU_FMT " Size: %16"
				     LLU_FMT "\n", hash_str[p],
				     h_cnt[p], h_data_cnt[p]);
			t -= x;
			buf += x;
			if(t <= 0) {
				goto end;
			}
		}
	}

	x = snprintf(buf, t, "RSA Mod Exp Algorithms:\n");
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	for(p = BIT_512; p < BIT_SIZE_MAX; ++p) {
		for(i = 0; i < NR_CPUS; ++i) {
			m_exp_cnt[p-1] += mod_exp_cnt[i][p-1];
//			mod_exp_cnt[i][p-1] = 0;
		}
		if(m_exp_cnt[p-1]) {
			x = snprintf(buf, t, "%11s: %11" LLU_FMT "\n",
				     rsa_mod_str[p-1], m_exp_cnt[p-1]);
			t -= x;
			buf += x;
			if(t <= 0) {
				goto end;
			}
		}
	}

	if(is_xls()) {
		x = snprintf(buf, t, "ECC Algorithms:\n");
		t -= x;
		buf += x;
		if(t <= 0) {
			goto end;
		}


		for(p = NLM_ECC_PRIME_160;
		    p < NLM_ECC_PRIME_CURVE_MAX; ++p) {
			for(q = NLM_ECC_PRIME_P_MUL; q < NLM_ECC_PRIME_OP_MAX;
			    ++q) {
				if(ec_p_cnt[p][q]) {
					x = snprintf(buf, t,
						     "%11s-%-11s: %11" LLU_FMT "\n",
						     ecc_prime_curve_str[p],
						     ecc_prime_func_str[q],
						     ec_p_cnt[p][q]);
					t -= x;
					buf += x;
					if(t <= 0) {
						goto end;
					}
				}
			}
		}

		for(p = 0; p < NLM_ECC_BINARY_CURVE_MAX; ++p) {
			for(q = NLM_ECC_BINARY_P_MUL;
			    q < NLM_ECC_BINARY_OP_MAX; ++q) {
				if(ec_b_cnt[p][q]) {
					x = snprintf(buf, t,
						     "%11s-%-11s: %11" LLU_FMT "\n",
						     ecc_binary_curve_str[p],
						     ecc_binary_func_str[q],
						     ec_b_cnt[p][q]);
					t -= x;
					buf += x;
					if(t <= 0) {
						goto end;
					}
				}
			}
		}
	}

	x = snprintf(buf, t, "Current Crypto outstanding msg=%u\n", dinfo->crypto_used_slots.counter);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	x = snprintf(buf, t, "Max Crypto outstanding msg=%u\n", dinfo->max_crypto_used_slots);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}
//	dinfo->max_crypto_used_slots = 0;

	x = snprintf(buf, t, "Current Pk outstanding msg=%u\n", dinfo->pk_used_slots);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	x = snprintf(buf, t, "Max Pk outstanding msg=%u\n", dinfo->max_pk_used_slots);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}
//	dinfo->max_pk_used_slots = 0;

	x = snprintf(buf, t, "Wait count =%"LLU_FMT "\n", w_count);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	x = snprintf(buf, t, "VMA count =%u\n", vma_count.counter);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}
//	vma_count.counter = 0;

#ifdef SYSTEM_CALL_STATS
	x = snprintf(buf, t, "MMap count =%"LLU_FMT "\n", map_cnt);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}

	x = snprintf(buf, t, "MMap free =%"LLU_FMT "\n", map_free);
	t -= x;
	buf += x;
	if(t <= 0) {
		goto end;
	}
#endif /* SYSTEM_CALL_STATS */
 end:
	return size - t;
}

#ifdef ENABLE_DEBUG_PROC
static int
print_page_info(device_info_pt dinfo, char *buf, int size)
{
	proc_memlist_pt proc_info;
	memlist_pt m;
	meminfo_pt mem;
	int t = size, x;

	if(!spin_trylock_bh(&dinfo->mem_lock)) {
		return snprintf(buf, t, "Lock not acquired.  Please try later.\n");
	}

	x = snprintf(buf, t, "Build version: %s\n", versionstr);
	t -= x;
	buf += x;

	x = snprintf(buf, t, "pid\torder\tpage addr\t\tpfn\n");
	t -= x;
	buf += x;

	proc_info = (proc_memlist_pt)dinfo->pmem_list.next;
	while(proc_info != (proc_memlist_pt)&dinfo->pmem_list) {
		list_for_each(m, &proc_info->mem) {
			mem = (meminfo_pt)m;
			x = snprintf(buf, t, "%d\t%d\t%p\t\t%lx\n", proc_info->tgid,
				     mem->order, mem->ptr,
				     page_to_pfn((struct page *)(mem->ptr)));
			t -= x;
			buf += x;
			if(t <= 0) {
				break;
			}
		}
		proc_info = (proc_memlist_pt)proc_info->elem.next;
	}

	spin_unlock_bh(&dinfo->mem_lock);
	return size - t;
}
#endif /* ENABLE_DEBUG_PROC */

static inline
meminfo_pt get_meminfo(device_info_pt dinfo, uint64_t msg0, uint64_t msg1)
{
#define CRYPTO_SCRATCH_VALUE(x,y)     ((((x) & 0xfULL) << 5) | \
				       ((y) & 0x1fULL))

#define PK_SCRATCH_VALUE(x,y)         ((((x) & 0x8ULL) >> 1) | \
				       (((y) >> 3) & 0x3ULL))
	int cs;
	meminfo_pt mem = NULL;

	if(msg0 & 0x10) {  /* PK OP Condition */
		cs = PK_SCRATCH_VALUE(msg0, msg1);
		mem = dinfo->pk_ops_slot[cs];
		dinfo->pk_ops_slot[cs] = NULL;
		clear_op_slot(dinfo, NLM_RSA_OP, cs);
	} else { /* Crypto Condition */
		cs = CRYPTO_SCRATCH_VALUE(msg0, msg1);
		mem = dinfo->crypto_ops_slot[cs];
		dinfo->crypto_ops_slot[cs] = NULL;
		clear_op_slot(dinfo, NLM_CRYPTO_OP, cs);
	}

	/* Added this to catch a problem observed a few times when
	 * response comes back, but mem ptr is NULL. */
	if(mem == NULL) {
		int i;
		printk("%s:%d MEMPTR is NULL. OpType=%" LLX_FMT " msg0=%" LLX_FMT " msg1=%"
		       LLX_FMT " cs=%d mem=%p\n",
		       __FUNCTION__, __LINE__, (msg0 & 0x10), msg0, msg1, cs, mem);
		if(msg0 & 0x10) {
			for(i = 0; i < 8; ++i) {
				printk("%d=[%p] ", i, dinfo->pk_ops_slot[i]);
			}
		} else {
			for(i = 0; i < 8; ++i) {
				printk(" %d=[%p] ", i,
				       dinfo->crypto_ops_slot[i]);
			}
		}
		printk("\n");
	}

	return mem;
}

void nlmsec_msgring_handler(int bucket, int size, int code, int stid,
                            struct msgrng_msg *msg, void *data)
{
	device_info_pt dinfo = (device_info_pt)data;
	volatile meminfo_pt mem;
	int ret = NLMSAE_SUCCESS;
	control_struct_pt ctrl;
#ifdef SAE_STATS
	resp_recieved[hard_smp_processor_id()]++;
#endif /* SAE_STATS */

#ifdef DEBUG_MSGRNG
	printk("%s:%d msg0=%016" LLX_FMT " msg1=%016" LLX_FMT "\n", __FUNCTION__,
	       __LINE__, msg->msg0, msg->msg1);
#endif

	if(CTRL_HEAD(msg->msg0) != 2ULL ||
	   CTRL_DEST_CTRL(msg->msg0) != 6ULL ||
	   DATA_HEAD(msg->msg1) != 2ULL ||
	   DATA_DEST_CTRL(msg->msg1) != 5ULL ||
	   CTRL_ERROR(msg->msg0) != 0ULL ||
	   DATA_ERROR(msg->msg1) != 0ULL) {
		printk(KERN_ALERT "%s:%d Invalid/Error response from SAE "
		       "msg0=%016" LLX_FMT " msg1=%016" LLX_FMT " ctrl_head=%"
		       LLX_FMT " ctrl_error=%" LLX_FMT " ctrl_ctrl=%" LLX_FMT
		       " data_head=%" LLX_FMT " data_error=%" LLX_FMT
		       " dest_ctrl=%" LLX_FMT "\n",
		       __FUNCTION__, __LINE__,
		       msg->msg0, msg->msg1, CTRL_HEAD(msg->msg0),
		       CTRL_ERROR(msg->msg0),
		       CTRL_DEST_CTRL(msg->msg0), DATA_HEAD(msg->msg1),
		       DATA_ERROR(msg->msg1), DATA_DEST_CTRL(msg->msg1));
		ret = -CTRL_ERROR(msg->msg0);
		if(!ret)
			ret = -DATA_ERROR(msg->msg1);
	}

#if 0
	if(dinfo != &dev_info) {
		printk(KERN_ALERT
		       "%s:%d Error returning handler data=%p dev_info=%p\n",
		       __FUNCTION__, __LINE__, data, &dev_info);
		dinfo = &dev_info;
	}
#endif

	mem = get_meminfo(dinfo, msg->msg0, msg->msg1);
#ifdef DEBUG_MSGRNG
	printk(KERN_NOTICE "%s:%d stid=%d code=%d meminfo=%p\n",
	       __FUNCTION__, __LINE__, stid, code, mem);
#endif /* DEBUG_MSGRNG */

	if(mem != NULL && mem->magic == DRIVER_MAGIC) {
		mem->resp0 = msg->msg0;
		mem->resp1 = msg->msg1;

		if(mem->ctx == PROCESS_CTX) {
			mem->resp0 = msg->msg0;
			mem->resp1 = msg->msg1;
		} else if(mem->ctx == KERNEL_CTX) {
			ctrl = (control_struct_pt)(mem->ptr);
//			magic = ctrl->magic;
			ctrl->msg0 = msg->msg0;
			ctrl->msg1 = msg->msg1;
		}

		ctrl = NULL;

		mem->result = ret;

		if(IS_ASYNC_OP(mem)) {
			add_meminfo_to_queue(mem->return_queue, mem);
		} else {
			if(waitqueue_active(&mem->wq)) {
				wake_up(&mem->wq);
			}

			if(waitqueue_active(op_data[mem->op_type].wq)) {
				wake_up_interruptible(op_data[mem->op_type].wq);
			}
		}

		mem->in_progress = NOT_IN_PROGRESS;
		wmb();

#if 0
		if(unlikely(magic != DRIVER_MAGIC)) {
			printk(KERN_ALERT "%s:%d Invalid magic=%" LLX_FMT " mem=%p\n",
			       __FUNCTION__, __LINE__, magic, mem);
		}
#endif

#ifdef DEBUG
	} else {
		printk(KERN_NOTICE "%s:%d Operation completed but, "
		       "no meminfo found.\n", __FUNCTION__, __LINE__);
#endif /* DEBUG */
	}

	return;
}

static int
nlmsec_read_stats_proc(char *page, char **start, off_t offset, int count,
		       int *eof, void *data)
{
	device_info_pt dinfo = (device_info_pt)data;
	int len = 0;
	if(offset == 0) {
		len = print_stats_info(dinfo, page, count);
	}
	*eof = 1;
	return len;
}

#ifdef ENABLE_DEBUG_PROC
static int
nlmsec_read_proc(char *page, char **start, off_t offset, int count,
		 int *eof, void *data)
{
	device_info_pt dinfo = (device_info_pt)data;
	int len = 0;
	if(offset == 0) {
		len = print_page_info(dinfo, page, count);
	}
	*eof = 1;
	return len;
}
#endif /* ENABLE_DEBUG_PROC */

static void nlmsec_driver_exit(void)
{
	unsigned long flags;

	/* device cleanup */
	// unregister the character device
	dev_info.exit_driver = 1;
	unregister_chrdev_region(dev_info.device, 1);

	cdev_del(&dev_info.nlmsec_cdev);

	// remove proc entries
	spin_lock_irqsave(&dev_info.mem_lock, flags);
	remove_proc_entry(stats_name, dev_info.pdir);
#ifdef ENABLE_DEBUG_PROC
	remove_proc_entry(debug_name, dev_info.pdir);
#endif /* ENABLE_DEBUG_PROC */
	remove_proc_entry(driver_name, (struct proc_dir_entry *)NULL);
	spin_unlock_irqrestore(&dev_info.mem_lock, flags);


	printk(KERN_NOTICE "Unloading driver\n");

	return;
}


static int ecc_init(void)
{
	meminfo_t mem;
	struct page *page;
	unsigned char *ecc;
	unsigned long flags, phy;

	memset(&mem, 0, sizeof(meminfo_t));

	mem.msg0 = ecc_msg0;
	mem.msg1 = ecc_msg1;
	mem.magic = DRIVER_MAGIC;
	mem.ctx = INIT_CTX;
	mem.op_type = NLM_ECC_OP;
	mem.in_progress = IN_PROGRESS;
	mem.result = NLMSAE_SUCCESS;
	mem.return_value = (unsigned long)&mem;
	init_waitqueue_head(&mem.wq);

	page = alloc_pages(GFP_ATOMIC | __GFP_ZERO, 0);

	local_irq_save(flags);
	ecc = (unsigned char *)kmap_atomic(page, KM_USER0);
	mem.ptr = ecc;
	memcpy(ecc, ecc_uc_data, sizeof(ecc_uc_data));
	kunmap_atomic(page, KM_USER0);
	local_irq_restore(flags);
	ecc = NULL;

	phy = page_to_pfn(page);
	phy <<= PAGE_SHIFT;
	SET_SEGMENT_ADDR(mem.msg0, phy);
	send_to_sae(&mem, 0);

	__free_page(page);

	if(mem.result != NLMSAE_SUCCESS) {
		printk(KERN_ALERT "%s:%d ECC init failed with error=%x\n", __FUNCTION__,
		       __LINE__, mem.result);
		return -1;
	}
	return 0;
}

static int msgring_config_init(void)
{
	int i;
	uint8_t sae_bkt[8];
	uint8_t sae_crdt[16][8];
	
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_SECURITY_OFFSET);

	netlogic_write_reg(mmio, SEC_DMA_CREDIT, SEC_DMA_CREDIT_CONFIG);

	netlogic_write_reg(mmio, SEC_CONFIG2, SEC_CFG2_ROUND_ROBIN_ON);


	if (xlr_hybrid_rmios_ipsec() || xlr_hybrid_rmios_tcpip_stack())
		return;

	if(dev_tree_en) {
		if(fdt_get_sae_bucket_conf(sae_bkt, 8, sae_crdt, 128) != 0) {
			printk("Bucket and credit config failed for sae\n");
			return -1;
		}
		
		for(i = 0; i < 5; i++)
			netlogic_write_reg (mmio, SEC_MSG_BUCKET0_SIZE + i, sae_bkt[i]);

		for(i = 0; i < 128; i++)
			netlogic_write_reg (mmio,  SEC_CC_CPU0_0 + i, sae_crdt[i/8][i%8]);

	} else if (!is_xls()) {
		for(i = 0; i < 8; i++)
			netlogic_write_reg (mmio, SEC_MSG_BUCKET0_SIZE + i,
					   bucket_sizes.bucket[MSGRNG_STNID_SEC + i]);
		for(i = 0; i < 128; i++)
			netlogic_write_reg (mmio,  SEC_CC_CPU0_0 + i,
						   cc_table_sec.counters[i>>3][i&0x07]);
	} else {
		for(i = 0; i < 8; i++)
			netlogic_write_reg (mmio, SEC_MSG_BUCKET0_SIZE + i,
					   xls_bucket_sizes.bucket[MSGRNG_STNID_SEC + i]);
		for(i = 0; i < 128; i++)
			netlogic_write_reg (mmio,  SEC_CC_CPU0_0 + i,
					   xls_cc_table_sec.counters[i>>3][i&0x07]);
	}
	return 0;
}

extern int xlr_loader_support;
extern int xlr_loader_sharedcore;
static int __init nlmsec_driver_init(void)
{
    int ret;
    int major, minor;

    /* In case of RMI CRF, check for ownership */
    if (dev_tree_en && fdt_get_sae_enabled() == 0)
	return -ENODEV;

    if ((xlr_loader_support && xlr_loader_sharedcore))
        return -ENODEV;

    /* init device_info strucuture */
    memset(&dev_info, 0, sizeof(struct device_info));
    dev_info.version = MKDEV(2,0);

    /* the pk ops bitmap uses only 8 bits, rest are marked used */
    dev_info.pk_ops_bitmap = 0xffffff00;

#if 0 /* set this to serialize crypto operations */
    for(ret = 0; ret < 15; ++ret) {
	    dev_info.crypto_ops_bitmap[ret] = 0xffffffff;
    }
    dev_info.crypto_ops_bitmap[ret] = 0xfffffffe;
#endif

    init_waitqueue_head(&dev_info.crypto_queue);
    init_waitqueue_head(&dev_info.pkop_queue);
    spin_lock_init(&dev_info.pkops_lock);
    spin_lock_init(&dev_info.crypto_lock);

    spin_lock_init(&dev_info.mem_lock);
    dev_info.pmem_list.next = &dev_info.pmem_list;
    dev_info.pmem_list.prev = &dev_info.pmem_list;

    /* registering device */
    dev_info.device = MKDEV(0, 0);
    ret = alloc_chrdev_region(&dev_info.device, 0, 1, driver_name);
    if(ret < 0) {
	printk(KERN_ALERT "%s:%d device could not get major number\n",
	       __FILE__, __LINE__);
	goto bail;
    }

    major = MAJOR(dev_info.device);
    minor = MINOR(dev_info.device);

    cdev_init(&dev_info.nlmsec_cdev, &fops);
    dev_info.nlmsec_cdev.owner = THIS_MODULE;

#ifdef CONFIG_OCF_OCF_MODULE
    /* create a asyncrhonous queue to process OCF operations */
    if(nlmsec_create_operation_callback(&dev_info.ocf_cb, nlmsae_ocf_callback)) {
	    unregister_chrdev_region(dev_info.device, 1);
	    printk(KERN_ALERT "%s:%d OCF async queue initialization failed.",
		   __FUNCTION__, __LINE__);
	    goto bail;
    }

    softc_device_init(&dev_info.ocf_dev, "nlmsae", 0, nlmsae_methods);
    dev_info.ocf_id = crypto_get_driverid(softc_get_device(&dev_info.ocf_dev),
					  CRYPTOCAP_F_HARDWARE);
    if(dev_info.ocf_id < 0) {
	    unregister_chrdev_region(dev_info.device, 1);
	    printk(KERN_ALERT "%s:%d NLMSAE cannot initialize into OCF.",
		   __FUNCTION__, __LINE__);
	    goto bail;
    } else {
#define	REGISTER(alg) \
	crypto_register(dev_info.ocf_id,alg,0,0)
	REGISTER(CRYPTO_DES_CBC);
	REGISTER(CRYPTO_3DES_CBC);
	REGISTER(CRYPTO_RIJNDAEL128_CBC);
	REGISTER(CRYPTO_MD5);
	REGISTER(CRYPTO_SHA1);
	REGISTER(CRYPTO_MD5_HMAC);
	REGISTER(CRYPTO_SHA1_HMAC);
    }
#endif /* CONFIG_OCF_OCF_MODULE */

    // create character device
    ret = cdev_add(&dev_info.nlmsec_cdev, dev_info.device, 1);
    if(ret < 0) {
	    unregister_chrdev_region(dev_info.device, 1);
#ifdef CONFIG_OCF_OCF_MODULE
	    crypto_unregister_all(dev_info.ocf_id);
#endif /* CONFIG_OCF_OCF_MODULE */
	    printk(KERN_ALERT "%s:%d Error adding char driver err=%d.\n",
		   __FUNCTION__, __LINE__, ret);
	    goto bail;
    }

    // initialize the sae configuration
    msgring_config_init();

    // create proc fs entries
    dev_info.pdir = proc_mkdir("netlogic/" DRIVER_NAME, NULL);
    if(dev_info.pdir == NULL)
    {
	    printk("%s:%d failed creating proc directory: %s\n",
		   __FUNCTION__, __LINE__, driver_name);
	    ret = -ENOMEM;
	    goto bail;
    } else {
#ifdef ENABLE_DEBUG_PROC
	    dev_info.pdebug = create_proc_read_entry(debug_name, 0, dev_info.pdir,
						     nlmsec_read_proc,
						     &dev_info);
	    if(dev_info.pdebug == NULL) {
		    remove_proc_entry(driver_name, NULL);
		    printk("%s:%d failed creating debug entry: %s/%s\n",
			   __FUNCTION__, __LINE__, driver_name, debug_name);
		    ret = -EINVAL;
		    goto bail;
	    }
#endif /* ENABLE_DEBUG_PROC */

	    dev_info.pstats = create_proc_read_entry(stats_name, 0, dev_info.pdir,
						     nlmsec_read_stats_proc,
						     &dev_info);

	    if(dev_info.pstats == NULL) {
#ifdef ENABLE_DEBUG_PROC
		    remove_proc_entry(debug_name, dev_info.pdir);
#endif /* ENABLE_DEBUG_PROC */
		    remove_proc_entry("netlogic/" DRIVER_NAME, NULL);
		    printk("%s:%d failed creating stats entry: %s/%s\n",
			   __FUNCTION__, __LINE__, driver_name, stats_name);
		    ret = -EINVAL;
		    goto bail;
	    }

    }

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
    if(nlm_on_chip_napi) {
	    printk(KERN_NOTICE "SAE NAPI-compatible Subsystem.\n");
    }
#endif

    // register handler
    if ((ret = register_msgring_handler(TX_STN_SEC,
					nlmsec_msgring_handler,
					&dev_info))) {
	    nlmsec_driver_exit();
	    ret = -EINVAL;
	    goto bail;
    }

    if(is_xls()) {
	    ecc_init();
    }

#ifdef DEBUG
    printk(KERN_NOTICE "%s:%d SEC handler=%p\n", __FUNCTION__, __LINE__,
	   nlmsec_msgring_handler);
#endif /* DEBUG */
    printk(KERN_NOTICE "Loaded SAE driver version=[%s] major=%d minor=%d\n",
	   versionstr, major, minor);

    return 0;
 bail:
    nlmsec_driver_exit();
    return ret;
}

/* Queuing functions */
int nlmsec_op_setup(op_handle_t handle, op_queue_t async_queue,
		    unsigned long arg, int op_flags)
{
	operation_pt op = (operation_pt)handle;

	if(op == NULL)
		return -1;

	op->return_queue = (secop_queue_pt)async_queue;
	op->op_flags = op_flags;

	if(op->return_queue != NULL) {
		op->op_flags |= OP_NO_WAIT;
	} else {
		op->op_flags &= ~(OP_NO_WAIT);
	}

	op->arg = arg;
	return 0;
}
EXPORT_SYMBOL(nlmsec_op_setup);

int
nlmsec_op_callback_setup(op_handle_t handle, op_callback_t async_callback,
			     unsigned long arg, int op_flags)
{
	return nlmsec_op_setup(handle, (op_queue_t)async_callback, arg, op_flags);
}
EXPORT_SYMBOL(nlmsec_op_callback_setup);

int nlmsec_create_operation_queue(op_queue_pt handle)
{
	secop_queue_pt ret;
	gfp_t flg;
	if(in_atomic() || in_softirq() || in_interrupt()) {
		flg = GFP_ATOMIC;
	} else {
		flg = GFP_KERNEL;
	}

	ret = (secop_queue_pt)kmalloc(sizeof(secop_queue_t), flg);
	if(ret == NULL) {
		return -ENOMEM;
	}
	INIT_SECOP_QUEUE(ret, -1);
	*handle = (op_queue_t)ret;

	return 0;
}
EXPORT_SYMBOL(nlmsec_create_operation_queue);

int nlmsec_destroy_operation_queue(op_queue_pt handle)
{
	secop_queue_pt ret = (secop_queue_pt)(*handle);
	if(ret && ret->response_type == SECOP_Q) {
		if(ret->q.length != 0) {
			printk(KERN_INFO "%s:%d Error queue not empty: %d\n",
			       __FUNCTION__, __LINE__, ret->q.length);
			return -1;
		}

		kfree(ret);
		*handle = 0;
	}
	return 0;
}
EXPORT_SYMBOL(nlmsec_destroy_operation_queue);

int nlmsec_create_operation_callback(op_callback_pt handle,
				     op_callback_func_t func)
{
	secop_queue_pt ret;
	gfp_t flg;
	if(in_atomic() || in_softirq() || in_interrupt()) {
		flg = GFP_ATOMIC;
	} else {
		flg = GFP_KERNEL;
	}

	ret = (secop_queue_pt)kmalloc(sizeof(secop_queue_t), flg);
	if(ret == NULL) {
		return -ENOMEM;
	}
	INIT_SECOP_CB(ret, func);
	*handle = (op_queue_t)ret;

	return 0;
}
EXPORT_SYMBOL(nlmsec_create_operation_callback);

int nlmsec_destroy_operation_callback(op_callback_pt handle)
{
	secop_queue_pt queue = (secop_queue_pt)handle;
	if(queue && queue->response_type == SECOP_CB) {
		kfree(queue);
		*handle = (op_callback_t)NULL;
	}
	return 0;
}
EXPORT_SYMBOL(nlmsec_destroy_operation_callback);

int nlmsec_op_queue_dequeue(op_queue_t qhandle, int *result, op_handle_pt phandle)
{
	secop_queue_pt queue = (secop_queue_pt)qhandle;
	operation_pt op;
	meminfo_pt mem = remove_meminfo_from_queue(queue);

	if(mem != NULL) {
		if(mem->magic == DRIVER_MAGIC) {
			*result = mem->result;
			op = (operation_pt)mem->return_value;
			if(mem->ctx == KERNEL_CTX ) {
				*result = post_process_op(*result, op);
			} else {
				printk(KERN_WARNING "%s:%d Meminfo Not kernel context\n",
				       __FUNCTION__, __LINE__);
			}
			*phandle = (op_handle_t)op;
			return 0;
		} else {
			printk(KERN_WARNING "%s:%d Invalid mem=%p magic=%llx\n",
			       __FUNCTION__, __LINE__, mem, mem->magic);
		}
	}
	return -1;
}
EXPORT_SYMBOL(nlmsec_op_queue_dequeue);

unsigned int nlmsec_op_queue_size(op_queue_t qhandle)
{
	int ret = 0;
	secop_queue_pt queue = (secop_queue_pt)qhandle;
	if(queue && queue->response_type == SECOP_Q) {
		ret = queue->q.length;
	}
	return ret;
}
EXPORT_SYMBOL(nlmsec_op_queue_size);

unsigned long nlmsec_op_get_arg(op_handle_t handle)
{
	operation_pt op;
	if(handle != 0UL) {
		op = (operation_pt)handle;
		return op->arg;
	}
	return 0;
}
EXPORT_SYMBOL(nlmsec_op_get_arg);

module_init(nlmsec_driver_init);
module_exit(nlmsec_driver_exit);
