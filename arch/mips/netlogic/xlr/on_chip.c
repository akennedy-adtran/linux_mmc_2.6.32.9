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


#include <linux/init.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/timer.h>

#include <asm/netlogic/msgring.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/xlr_user_mac.h>
#include <asm/netlogic/sim.h>

#ifdef CONFIG_NLM_XLP
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/hal/nlm_hal_pic.h>
#endif

unsigned long netlogic_io_base = (unsigned long)(DEFAULT_NETLOGIC_IO_BASE);
EXPORT_SYMBOL(netlogic_io_base);
int msgring_timer_irq;

#define MSGRNG_CC_INIT_CPU_DEST(conf, dest,cpu) \
do { \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][0], 0 ); \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][1], 1 ); \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][2], 2 ); \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][3], 3 ); \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][4], 4 ); \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][5], 5 ); \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][6], 6 ); \
     msgrng_write_cc(MSGRNG_CC_##dest##_REG, conf##cc_table_cpu_##cpu.counters[dest][7], 7 ); \
} while(0)

/* Initialized CC for cpu 0 to send to all buckets at 0-7 cpus */
#define MSGRNG_CC_INIT_CPU(conf, cpu) \
do { \
  MSGRNG_CC_INIT_CPU_DEST(conf,0,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,1,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,2,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,3,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,4,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,5,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,6,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,7,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,8,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,9,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,10,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,11,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,12,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,13,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,14,cpu); \
  MSGRNG_CC_INIT_CPU_DEST(conf,15,cpu); \
} while (0)

#define MSGRNG_BUCKETSIZE_INIT_CPU(conf, base) \
do { \
  msgrng_write_bucksize(0, conf##bucket_sizes.bucket[base+0]);         \
  msgrng_write_bucksize(1, conf##bucket_sizes.bucket[base+1]);         \
  msgrng_write_bucksize(2, conf##bucket_sizes.bucket[base+2]);  \
  msgrng_write_bucksize(3, conf##bucket_sizes.bucket[base+3]);  \
  msgrng_write_bucksize(4, conf##bucket_sizes.bucket[base+4]);  \
  msgrng_write_bucksize(5, conf##bucket_sizes.bucket[base+5]);  \
  msgrng_write_bucksize(6, conf##bucket_sizes.bucket[base+6]);  \
  msgrng_write_bucksize(7, conf##bucket_sizes.bucket[base+7]);  \
} while(0)

#define XLR_MSG_TBL
#define XLS_MSG_TBL  xls_
#define SHARED_XLR_MSG_TBL shared_

#define X_MSGRNG_BUCKETSIZE_INIT_CPU(x,y) MSGRNG_BUCKETSIZE_INIT_CPU(x,y)

__u32  pop_bucket_mask[NR_CORES];
__u32  pop_bucket_start[NR_CORES];
__u32  pop_bucket_end[NR_CORES];
__u32 cpu_to_bktmask[NR_CPUS];
__u32 cpu_to_frstid[NR_CPUS];

uint32_t hard_cpu_online_map = 0;
uint32_t msgring_global_thread_mask = 0;

/* make this a read/write spinlock */
spinlock_t msgrng_lock;
static nlm_common_atomic_t msgring_registered;

int msgring_int_type;
int msgring_int_en;
int msgring_watermark_count;
static __u32 msgring_thread_mask;

extern int nlm_dev_own_bucket_list_get(int *start, int *end, int *mask);
extern struct irq_chip nlm_common_rsvd_pic;
extern struct irqaction nlm_common_rsvd_action;

#ifdef CONFIG_PHOENIX_MSGRING_NAPI
extern int nlm_msgring_napi;
extern int xlr_napi_ready;
extern void xlr_napi_rx_schedule(void);
#endif				/* CONFIG_PHOENIX_MSGRING_NAPI */

struct tx_stn tx_stns[MAX_TX_STNS];

int rxstn_to_txstn_map[128] = {
	[0 ... 7] = TX_STN_CPU_0,
	[8 ... 15] = TX_STN_CPU_1,
	[16 ... 23] = TX_STN_CPU_2,
	[24 ... 31] = TX_STN_CPU_3,
	[32 ... 39] = TX_STN_CPU_4,
	[40 ... 47] = TX_STN_CPU_5,
	[48 ... 55] = TX_STN_CPU_6,
	[56 ... 63] = TX_STN_CPU_7,
	[64 ... 95] = TX_STN_INVALID,
	[96 ... 103] = TX_STN_GMAC,
	[104 ... 107] = TX_STN_DMA,
	[108 ... 111] = TX_STN_INVALID,
	[112 ... 113] = TX_STN_XGS_0,
	[114 ... 115] = TX_STN_XGS_1,
	[116 ... 119] = TX_STN_INVALID,
	[120 ... 127] = TX_STN_SEC
};

int xls_rxstn_to_txstn_map[128] = {
        [0 ... 7] = TX_STN_CPU_0,
        [8 ... 15] = TX_STN_CPU_1,
	[16 ... 23] = TX_STN_CPU_2,
	[24 ... 31] = TX_STN_CPU_3,
        [32 ... 80] = TX_STN_INVALID,
	[80 ... 87] = TX_STN_GMAC1,
	[96 ... 103] = TX_STN_GMAC0,
	[104 ... 107] = TX_STN_DMA,
	[108 ... 111] = TX_STN_CMP,
	[112 ... 115] = TX_STN_INVALID,
	[116 ... 119] = TX_STN_PCIE,
	[120 ... 121] = TX_STN_SEC,
	[122 ... 127] = TX_STN_INVALID,
};

void dummy_handler(int bucket, int size, int code, int tx_stid,
		   struct msgrng_msg *msg, void *dev_id)
{
	printk("[%s]: No Handler for message from stn_id=%d, bucket=%d, "
	       "size=%d, msg0=%llx, dropping message\n",
	       __FUNCTION__, tx_stid, bucket, size,
	       (unsigned long long)msg->msg0);
}

struct tx_stn_handler tx_stn_handler_map[128] = {
	[0 ... 127] = {dummy_handler, NULL},
};

void nlm_common_msgring_cpu_init(void)
{
	int id;
	unsigned long flags;
	int shared_msgring = 0;

	id = cpu_logical_map(get_cpu());

	msgring_int_en = 1;

	if (xlr_hybrid_user_mac() || xlr_hybrid_user_mac_xgmac()) {
		/* msgring interrupt should be disabled */
		msgring_int_type = 0x0;

		pop_bucket_start[id >> 2] = 0;
		pop_bucket_end[id >> 2] = 4;
		pop_bucket_mask[id >> 2] = 0xf;
	} else {
		/* all the stations are owned by linux */
		pop_bucket_start[id >> 2] = 0;
		pop_bucket_end[id >> 2] = 8;
		pop_bucket_mask[id >> 2] = 0xff;
	}

	/* if not thead 0 */
	if ((id & 0x03) != 0) {
		put_cpu();
		return;
	}

	prom_dbg_msg("Initializing message ring for cpu_%d\n", id);

	msgrng_flags_save(flags);

	/* Message Stations are shared among all threads in a cpu core
	 * Assume, thread 0 on all cores are always active when more than
	 * 1 thread is active in a core
	 */
	if (is_xls()) {
		if (id == 0) {
			X_MSGRNG_BUCKETSIZE_INIT_CPU(XLS_MSG_TBL, 0);
			MSGRNG_CC_INIT_CPU(XLS_MSG_TBL, 0);
		} else if (id == 4) {
			X_MSGRNG_BUCKETSIZE_INIT_CPU(XLS_MSG_TBL, 8);
			MSGRNG_CC_INIT_CPU(XLS_MSG_TBL, 1);
		} else if (id == 8) {
			X_MSGRNG_BUCKETSIZE_INIT_CPU(XLS_MSG_TBL, 16);
			MSGRNG_CC_INIT_CPU(XLS_MSG_TBL, 2);
		} else if (id == 12) {
			X_MSGRNG_BUCKETSIZE_INIT_CPU(XLS_MSG_TBL, 24);
			MSGRNG_CC_INIT_CPU(XLS_MSG_TBL, 3);
		}
	} else {
		if (shared_msgring) {
			if (id == 0) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     0);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 0);
			} else if (id == 4) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     8);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 1);
			} else if (id == 8) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     16);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 2);
			} else if (id == 12) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     24);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 3);
			} else if (id == 16) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     32);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 4);
			} else if (id == 20) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     40);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 5);
			} else if (id == 24) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     48);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 6);
			} else if (id == 28) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(SHARED_XLR_MSG_TBL,
							     56);
				MSGRNG_CC_INIT_CPU(SHARED_XLR_MSG_TBL, 7);
			}
		} else {
			if (id == 0) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 0);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 0);
			} else if (id == 4) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 8);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 1);
			} else if (id == 8) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 16);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 2);
			} else if (id == 12) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 24);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 3);
			} else if (id == 16) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 32);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 4);
			} else if (id == 20) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 40);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 5);
			} else if (id == 24) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 48);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 6);
			} else if (id == 28) {
				X_MSGRNG_BUCKETSIZE_INIT_CPU(XLR_MSG_TBL, 56);
				MSGRNG_CC_INIT_CPU(XLR_MSG_TBL, 7);
			}
		}
	}
	msgrng_flags_restore(flags);
	put_cpu();
}

void nlm_common_msgring_config(void)
{

#ifdef CONFIG_PHOENIX_MSGRING_NAPI
	/* If we use NAPI then we enable queue non-empty interrupt */
	msgring_int_type = nlm_msgring_napi ? 0x01 : 0x02;
#else
	msgring_int_type = 0x02;
#endif				/* CONFIG_PHOENIX_MSGRING_NAPI */

	msgring_watermark_count = 1;
	msgring_thread_mask = 0x0f;

/* 	printk("[%s]: int_type = 0x%x, pop_num_buckets=%d, pop_bucket_mask=%x" */
/* 	       "watermark_count=%d, thread_mask=%x\n", __FUNCTION__, */
/* 	       msgring_int_type, msgring_pop_num_buckets, msgring_pop_bucket_mask, */
/* 	       msgring_watermark_count, msgring_thread_mask); */
}

void nlm_common_derive_cpu_to_bkt_map(void)
{
	int cpus_per_core[NR_CORES];
	int stns_per_core[NR_CORES];
	int num_cpus, cpus, cpu_off, from, i;
	int bucket_mask[NR_CPUS_PER_CORE];
	int fr_bucket[NR_CPUS_PER_CORE];
	int core, bkt_idx, bkt_mask;

#define GET_NEXT_SET_BIT_U8(val, rv) { \
    if(val < ( 1 << rv)) \
        rv = 0; \
    for(i = rv; val != 0 && i <= 7; i++) { \
        if(val & (1 << i)) { \
            rv = i; \
            break; \
        } \
    }  \
    if( i >= 8) \
        rv = 0; \
}

	memset(cpus_per_core, 0, sizeof(cpus_per_core));
	memset(stns_per_core, 0, sizeof(stns_per_core));

	for (i = 0; i < NR_CPUS; i++) {
		if (!(hard_cpu_online_map & (1 << i)))
			continue;
		core = i / NR_CPUS_PER_CORE;
		cpus_per_core[core]++;
	}
	for (core = 0; core < NR_CORES; core++) {
		for (i = 0; i < NR_STNS_PER_CORE; i++) {
			if (!(pop_bucket_mask[core] & (1 << i)))
				continue;
			stns_per_core[core]++;
		}
	}

	for (core = 0; core < NR_CORES; core++) {
		int filled_all = 0;
		int rv = 0;
		uint8_t fr_bucket_map = 0;
		memset(bucket_mask, 0, sizeof(bucket_mask));
		memset(fr_bucket, 0xff, sizeof(fr_bucket));

        num_cpus = cpus_per_core[core];
        if(num_cpus == 0)
            continue;
        for(cpus = 0, bkt_idx = 0, bkt_mask = pop_bucket_mask[core];
                bkt_mask; bkt_mask = bkt_mask >> 1, bkt_idx++) {
            if(!(bkt_mask & 0x01))
                continue;
            bucket_mask[cpus] |=  (1 << bkt_idx);
			
			if(((int)fr_bucket[cpus] != -1) && (fr_bucket[cpus] < NR_CPUS_PER_CORE))
                fr_bucket_map &= (~(1 << fr_bucket[cpus]));
            fr_bucket_map |= (1 << bkt_idx);
			fr_bucket[cpus] = bkt_idx;

			if((cpus + 1) == num_cpus)
            	filled_all = 1;

            cpus = (cpus + 1) % num_cpus;
        }

        /* fill the non filled cpus */
		if(filled_all == 0) {
	        for(from = 0; cpus < num_cpus; cpus++, from++) {
    	        bucket_mask[cpus] = bucket_mask[from];
       	 	}
		}
        cpu_off = core * NR_CPUS_PER_CORE;
        for(from = 0, cpus = cpu_off;
                cpus < cpu_off + NR_CPUS_PER_CORE; cpus++) {
            if(!(hard_cpu_online_map & (1 << cpus)))
                continue;
            cpu_to_bktmask[cpus] = bucket_mask[from];
			GET_NEXT_SET_BIT_U8(fr_bucket_map, rv);
            cpu_to_frstid[cpus] = rv + (core * NR_STNS_PER_CORE);
            from++;
			rv++;
        }
    }
#if 0
	for (i = 0; i < NR_CPUS; i++)
		printk("%d: bktmask=0x%x frstid=%d\n",
		       i, cpu_to_bktmask[i], cpu_to_frstid[i]);
#endif

	return;
}

static int __init xlr_msgring_watermark_setup(char *str)
{
	if (*str == '=')
		str++;

	msgring_watermark_count = (int)simple_strtoul(str, NULL, 10);

	return 1;
}

static int __init xlr_msgring_thread_mask_setup(char *str)
{
	if (*str == '=')
		str++;

	msgring_thread_mask = simple_strtoul(str, NULL, 16);
	msgring_thread_mask &= 0x0f;

	return 1;
}

static int __init xlr_complete_msgring_thread_mask_setup(char *str)
{
	if (*str == '=')
		str++;
	msgring_global_thread_mask = simple_strtoul(str, NULL, 16);
	msgring_global_thread_mask &= 0xffffffff;
	return 1;
}

__setup("xlr_msgring_watermark=", xlr_msgring_watermark_setup);
__setup("xlr_msgring_thread_mask=", xlr_msgring_thread_mask_setup);
__setup("xlr_complete_msgring_thread_mask=",
	xlr_complete_msgring_thread_mask_setup);

extern void nlm_cpu_stat_update_msgring_int(void);
extern void nlm_cpu_stat_update_msgring_cycles(__u32 cycles);
extern void nlm_cpu_stat_update_msgring_pic_int(void);

void msgring_process_rx_msgs(int start_bucket, int end_bucket,
			     __u32 pop_bucket_mask)
{
	unsigned int bucket_empty_bm = 0;
	int bucket = 0;
	int size = 0, code = 0, rx_stid = 0;
	struct msgrng_msg msg;
	struct tx_stn_handler *handler = 0;
	unsigned int status = 0;

#ifdef CONFIG_PHOENIX_MSGRING_NAPI
	if (xlr_napi_ready && in_irq()) {
		xlr_napi_rx_schedule();
		return;
	}
#endif				/* CONFIG_PHOENIX_MSGRING_NAPI */

	/* First Drain all the high priority messages */
	for (;;) {

		bucket_empty_bm =
		    (msgrng_read_status() >> 24) & pop_bucket_mask;

		/* all buckets empty, break */
		if (bucket_empty_bm == pop_bucket_mask)
			break;

		for (bucket = start_bucket; bucket < end_bucket; bucket++) {

			if ((bucket_empty_bm & (1 << bucket)) ||	/* empty */
			    !((1 << bucket) & pop_bucket_mask))	/* not in mask */
				continue;

			status =
			    message_receive(bucket, &size, &code, &rx_stid,
					    &msg);
			if (status)
				continue;

			handler = &tx_stn_handler_map[rx_stid];
			/* Handler is always present. If not actual, atleast 
			 * dummy_handler
			 */
			(handler->action) (bucket, size, code, rx_stid, &msg,
					   handler->dev_id);
		}
	}
}

#if !defined(CONFIG_NLMCOMMON_MAC) && !defined(CONFIG_NLM_XLP)
//__u64 xlr_cp2_exceptions[32];
//struct user_mac_kernal_data user_mac_krnl_data;
//struct xlr_user_mac_config xlr_user_mac;
void nlm_cpu_stat_update_msgring_int(void) { }
void nlm_cpu_stat_update_msgring_cycles(__u32 cycles) { }
void nlm_cpu_stat_update_msgring_pic_int(void) { }
#endif /* CONFIG_PHOENIX_MAC */

__u32 msgrng_msg_cycles = 0;
void nlm_xlr_msgring_int_handler(unsigned int irq, struct pt_regs *regs)
{
	unsigned long mflags;
	int core;
	__u32 cycles = 0;

	if (irq == IRQ_MSGRING) {
		/* normal message ring interrupt */
		xlr_inc_counter(MSGRNG_INT);
		nlm_cpu_stat_update_msgring_int();
	} else {
		nlm_cpu_stat_update_msgring_pic_int();
	}


	//dbg_msg("IN irq=%d\n", irq);

	/* TODO: not necessary to disable preemption */
	msgrng_flags_save(mflags);

	cycles = read_c0_count();

	core = cpu_logical_map(smp_processor_id()) >> 2;
	msgring_process_rx_msgs(pop_bucket_start[core], pop_bucket_end[core], pop_bucket_mask[core]);

	nlm_cpu_stat_update_msgring_cycles(read_c0_count() - cycles);

	msgrng_flags_restore(mflags);

	//dbg_msg("OUT irq=%d\n", irq);

}

static void enable_msgring_int(void *info)
{
	unsigned long flags = 0, mflags = 0;
	unsigned int th_mask;
	unsigned int core;
	msgrng_access_save(&msgrng_lock, flags, mflags);

	core = hard_smp_processor_id() & ~(0x3);
	th_mask = (msgring_global_thread_mask >> core) & 0x0f;

#if 0
	printk
	    ("[%s:%d] cpu_%d cpu_online_map=0x%04x msgring_global_mask=0x%08x "
	     "th_mask=0x%02x intype=%d wm=%d\n", __FUNCTION__, __LINE__,
	     hard_smp_processor_id(), hard_cpu_online_map,
	     msgring_global_thread_mask, th_mask, msgring_int_type,
	     msgring_watermark_count);
#endif

	/* enable the message ring interrupts */
	msgrng_write_config((msgring_watermark_count << 24) |
			    (IRQ_MSGRING << 16)
			    | (th_mask << 8) | msgring_int_type);
	msgrng_access_restore(&msgrng_lock, flags, mflags);
}

static void msgring_bkp_timer(unsigned long data)
{
	unsigned long flags;
	struct timer_list *timer = (struct timer_list *)data;
	local_irq_save(flags);
	irq_enter();
	nlm_xlr_msgring_int_handler(-1,NULL);
	irq_exit();
	local_irq_restore(flags);
	mod_timer(timer, timer->expires+2);
}

static void enable_msgring_timer(void *data)
{
	struct timer_list *timer;
	timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	setup_timer(timer, msgring_bkp_timer, (unsigned long)timer);
	timer->expires = jiffies + 2;
	add_timer(timer);
}

extern spinlock_t nlm_common_pic_lock;
int register_msgring_handler(int major,
			     void (*action) (int, int, int, int,
					     struct msgrng_msg *, void *),
			     void *dev_id)
{
	struct tx_stn_handler *handler = 0;
	int ret = 1;
	int i,j,tx_stid;
	unsigned long flags = 0;
	cpumask_t timer_cpu_mask;

	if (major >= MAX_TX_STNS || action == NULL) {
		printk(KERN_ALERT "%s:%d  Invalid parameter: major=%d, "
		       "MAX_TX_STN=%d action=%p",
		       __FUNCTION__, __LINE__, major, MAX_TX_STNS, action);
		return ret;
	}

	/* Check if the message station is valid, if not return error */
	spin_lock_irqsave(&msgrng_lock, flags);

	for (i = 0; i < 128; i++) {
		if (is_xls())
			tx_stid = xls_rxstn_to_txstn_map[i];
		else
			tx_stid = rxstn_to_txstn_map[i];
		if (tx_stid == major) {
			tx_stn_handler_map[i].action = action;
			tx_stn_handler_map[i].dev_id = dev_id;
		}
	}

	handler = &tx_stns[major].handler;

	// dbg_msg("major=%d, action=%p, dev_id=%p\n", major, action, dev_id);
	handler->action = action;
	handler->dev_id = dev_id;

	ret = 0;
	spin_unlock_irqrestore(&msgrng_lock, flags);

	if (!ret && nlm_common_test_and_set(&msgring_registered)) {
		int i=0;

		hard_cpu_online_map = 0;
		for (i = 0; i < NR_CPUS; i++) {
			if (cpu_isset(i, cpu_online_map))
				hard_cpu_online_map |=
				    (1 << cpu_logical_map(i));
		}

		/* derive the cpu to bucket map */
		nlm_common_derive_cpu_to_bkt_map();


		/* Configure PIC to deliver msgring interrupt for timeouts */
		if (msgring_global_thread_mask == 0) {
			for (i = 0; i < NR_CORES; i++) {
				msgring_global_thread_mask |=
				    (msgring_thread_mask << (i << 2));
			}
		}

		msgring_global_thread_mask &= hard_cpu_online_map;

		/* configure the msgring interrupt on all cpus */
		if (msgring_int_en)
			on_each_cpu(enable_msgring_int, 0, 1);

/* 		printk("[%s]: cpu_online_map = %lx, hard_cpu_online_map=%x, " */
/* 		       "msgring_global_thread_mask=%x\n", */
/* 		       __FUNCTION__,  */
/* 		       (unsigned long)cpu_online_map,  */
/* 		       hard_cpu_online_map,  */
/* 		       msgring_global_thread_mask); */

		/* Schedule a messagering backup timer at every 2 jiffies on one 
		   therad per core 
		 */

		cpus_clear(timer_cpu_mask);
		for(i = 0; i < NR_CORES; i++) {
			int core_mask;			
			int phys_id, logical_id;
			if(hard_cpu_online_map & (0xf<<(i*NR_CPUS_PER_CORE))){
				core_mask = (hard_cpu_online_map>>(i*NR_CPUS_PER_CORE)) & 0xf;
				for(j=0; j<NR_CPUS_PER_CORE; j++){
					if(core_mask & (1<<j))
						break;
				}
				phys_id = (i*NR_CPUS_PER_CORE) + j;
				logical_id = cpu_number_map(phys_id);
				cpu_set(logical_id, timer_cpu_mask);
			}
		}
		preempt_disable();
		smp_call_function_many(&timer_cpu_mask, enable_msgring_timer, NULL, 1);
		preempt_enable();
		if(cpu_isset(cpu_number_map(hard_smp_processor_id()),timer_cpu_mask))
			enable_msgring_timer(NULL);
	}

	return ret;
}

EXPORT_SYMBOL(register_msgring_handler);

static void pic_init(void)
{
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_PIC_OFFSET);
	int i = 0;
	int level;
	uint32_t thread_mask = (1 << hard_smp_processor_id());

	for (i = 0; i < PIC_NUM_IRTS; i++) {

		level = PIC_IRQ_IS_EDGE_TRIGGERED(i);

		/* Bind all PIC irqs to boot cpu */
		netlogic_write_reg(mmio, PIC_IRT_0_BASE + i, thread_mask);

		/* Use local scheduling and high polarity for all IRTs
		 * Invalidate all IRTs, by default
		 */
		netlogic_write_reg(mmio, PIC_IRT_1_BASE + i,
				  (level << 30) | (1 << 6) | (PIC_IRQ_BASE +
							      i));
	}
}

atomic_t nlm_common_counters[NR_CPUS][NLM_MAX_COUNTERS] __cacheline_aligned;

static void nlm_usb_init (void)
{
	nlm_reg_t * gpio_mmio = netlogic_io_mmio(NETLOGIC_IO_GPIO_OFFSET);
	nlm_reg_t * usb_mmio  = netlogic_io_mmio(NETLOGIC_IO_USB_1_OFFSET);

   /* The NLM-Specific USB Block */
   netlogic_write_reg(usb_mmio, 49, 0x10000000); //Clear Rogue Phy INTs
   netlogic_write_reg(usb_mmio, 50, 0x1f000000);

	if (is_xls1xx()) {
		/* Enabling only 1 USB Port */
		if (xlr_board_atx_viii()) {
			/* LTE board has usb port #1 */
			netlogic_write_reg(usb_mmio,  1, 0x05000500);
		}
		else {
			/* enable usb port #0 */
			netlogic_write_reg(usb_mmio,  1, 0x03000500);
		}
	}
	else {
   	netlogic_write_reg(usb_mmio,  1, 0x07000500);
	}

   {
      volatile unsigned int value = gpio_mmio[21];
      if ((value >> 22) & 0x01) {
         printk("Detected USB Host mode..\n");
         netlogic_write_reg(usb_mmio,  0, 0x02000000);
      }
      else {
         printk("Detected USB Device mode..\n");
         netlogic_write_reg(usb_mmio,  0, 0x01000000);
      }
   }
}

void on_chip_init(void)
{
	int i = 0, j = 0;

	cpu_logical_map(0)  = hard_smp_processor_id();

	/* Set netlogic_io_base to the run time value */
	spin_lock_init(&msgrng_lock);

	msgring_registered.value = 0;

#if defined(CONFIG_NLM_XLP)
	nlm_hal_init();
#endif

	nlm_common_msgring_config();

	pic_init(); 

	nlm_common_msgring_cpu_init();



	for (i = 0; i < NR_CPUS; i++)
		for (j = 0; j < NLM_MAX_COUNTERS; j++)
			atomic_set(&nlm_common_counters[i][j], 0);

	if (is_xls())
		nlm_usb_init();
}
