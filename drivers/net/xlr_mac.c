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


#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/if_ether.h>	/* For the statistics structure. */
#include <linux/if_arp.h>	/* For ARPHRD_ETHER */
#include <linux/autoconf.h>
#include <linux/proc_fs.h>
#include <linux/mii.h>
#include <linux/delay.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cache.h>

#include <asm/netlogic/debug.h>
#include <asm/netlogic/pci.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/xlr_mac.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/xlr_user_mac.h>
#include <asm/netlogic/atx_cpld.h>
#include <asm/netlogic/xgmac_mdio.h>
#include <asm/netlogic/proc.h>
#include <asm/smp.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/gpio.h>
#include <user/netlogic/xlr_user_mac.h>
#include <asm/netlogic/debug.h>
/*
#include <asm/netlogic/xlr_rmik.h>
*/
#define DRV_NAME	"nlm_xlr_mac"
#define DRV_VERSION	"0.1"
/* #define DEBUG */

#ifdef DEBUG
#undef dbg_msg
int mac_debug = 1;
#define dbg_msg(fmt, args...) \
        do {\
            if (mac_debug) {\
                printk("[%s@%d|%s]: cpu_%d: " fmt, \
                __FILE__, __LINE__, __FUNCTION__,  smp_processor_id(), ##args);\
            }\
        } while(0);

#define DUMP_PACKETS
#else
#undef dbg_msg
#define dbg_msg(fmt, args...)
int mac_debug = 0;
#endif
extern int xlr_loader_support;
extern int xlr_loader_sharedcore;
extern int xlr_loader_own_gmac;


extern struct proc_dir_entry *nlm_root_proc;
extern int xlsb0_in_xaui(int block);

static struct net_device_ops nlm_mac_net_ops;

#ifdef CONFIG_NLMCOMMON_PTP_SUPPORT
void dump_all_interface(unsigned int reg);
void ( *p_ptp_set_ts) (u32 , u32, ktime_t *, u32);
#endif
/* 
 *  Packet dump tools
*/
#define dump_packet(skb) \
do \
{ \
	int i = 0; \
	printk("%s: Packet: length=%d\n", __FUNCTION__, skb->len); \
	for (i = 0; i < 64; i++) { \
		printk("%02x ", skb->data[i]); \
		if (i && (i % 16) == 0) \
			printk("\n"); \
	} \
	printk("\n"); \
} while (0)



/*
 * This macro returns non-zero if any of the upper buckets (4-7) is non-empty
*/
#define upper_buckets_nonempty() ((~msgrng_read_status() >> 28) & 0xf)


extern __u32 cpu_to_frstid[];
extern __u32 cpu_to_bktmask[];
extern uint32_t hard_cpu_online_map;
uint64_t free_back_stid_map = 0ULL;

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI

/* XLR_NAPI global data strucutre */
struct net_device xlr_napi_dummy_dev;  
DEFINE_PER_CPU(struct napi_struct, xlr_napi_poll_struct);  
EXPORT_SYMBOL(xlr_napi_dummy_dev);
EXPORT_PER_CPU_SYMBOL(xlr_napi_poll_struct);

/* XLR NAPI per CPU packet counter */
DEFINE_PER_CPU(unsigned long long, xlr_napi_rx_count); 

/* NAPI is disabled by default */
int nlm_msgring_napi = 0; 
int nlm_on_chip_napi = 0; 
EXPORT_SYMBOL(nlm_on_chip_napi);
int xlr_napi_ready = 0; 
EXPORT_SYMBOL(xlr_napi_ready);
static inline void nlm_xlr_free_skb(struct msgrng_msg *msg);

struct napi_control_s 
{
  /* core-wide lock */
  spinlock_t xlr_napi_msgrng_lock __attribute__((aligned(SMP_CACHE_BYTES)));

  /* Mask tracking if NetRx SoftIRQ is scheduled on core's threads
   *
   * Only bits 0-3 are used:
   *
   *   i-th bit 0:  iff netrx SoftIRQ has been scheduled for thread i
   *   i-th bit 1:  iff netrx SoftIRQ has NOT been scheduled for thread i
  */
  unsigned long netrx_mask;
};

__aligned(SMP_CACHE_BYTES) struct napi_control_s napi_control[NR_CPUS / 4];


/* We need this little hack to improve handler speed */
static int *rxstn_to_txstn_ptr;
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */


#define MAC_B2B_IPG             88

/* 
 * Weight for GMAC/XGMAC NAPI polls. We override standard value of 300 
 * to improve packet forwarding rate 
*/
#define XLR_NAPI_WEIGHT		1200
static int napi_weight = XLR_NAPI_WEIGHT;


/* 
 * TCP stack termination in NAPI mode requires spill area for FreeOut's of 
 * considerable size. We always set it to 15K descriptors which traslates into
 * 15K * 8 bytes kernel memory alloc.
*/
#define XLR_FROUT_JUMBO_SPILL	(15 * 1024)


/* NOTE:
   Don't change this threshold to > 15 if MAX_NUM_DESC is 512. 
   When msgring_thread_mask is 0xf, each cpu could receive 16 packets 
   and replenishment may never happen.
   THRESHOLD should be less than 
   max_num_desc / (number of threads processing msgring * number of cores)
   */
#define MAC_FRIN_TO_BE_SENT_THRESHOLD max_frin_threshold
#define MAC_FRIN_WORK_NUM 32

/* Computed as described above */
static int max_frin_threshold;

/* Total Nr of Free Descriptors to GMACs > 2816 for Usermac 
 * If configuring max_num_desc use at least 2816/4.
 */

static struct net_device *dev_mac[NETLOGIC_MAX_MACS];
struct net_device *dev_mac_type[MAX_NET_TYPES][NETLOGIC_MAX_MACS];
#define mac_addr_to_ptr(x) ((void *)(long)x)

#define NLM_NUM_REG_DUMP 9 /* Register 0xa0 to 0xa8 */
#define NLM_ETHTOOL_REG_LEN (NLM_NUM_REG_DUMP * 4) 

static void xlr_get_mac_stats(struct net_device *dev, 
					struct net_device_stats *stats);

/*
 * New message assembly toolbox: newer, faster versions of messge send functions!
 *
 * NB: Please be advised that they require interrupts be off
 *
 * TODO: move them into msgring.h, if all goes well here
*/
static inline int 
message_send_fast_1(unsigned int code, 
                    unsigned int stid, 
                    unsigned long long msg0)
{
	int ret, retry = 16;


  	msgrng_load_tx_msg0(msg0);

	__asm__ __volatile__ (".set push\n"
	                      ".set noreorder\n"
		              ".set mips64\n"
		              "move $8, %1\n"
		              "1: c2 0x80001\n"
		              "mfc2 $8, "STR(MSGRNG_MSG_STATUS_REG)"\n"
		              "andi $8, $8, 0x6\n"
		              "beqz $8, 2f\n"
			      "addi %2, -1\n"
			      "bnez %2, 1b\n"
		              "move $8, %1\n"
			      "addiu $8, $0, 4\n"
			      "2: move %0, $8\n"
		              ".set pop\n"
		              :"=r"(ret)
		              : "r"((code<<8)|stid), "r"(retry)
		              : "$8"
		             );
	return ret;
}


static inline int 
message_send_fast_2(unsigned int code, 
                    unsigned int stid, 
                    unsigned long long msg0, 
                    unsigned long long msg1)
{
	int ret, retry = 16;


  	msgrng_load_tx_msg0(msg0);
  	msgrng_load_tx_msg1(msg1);

	__asm__ __volatile__ (".set push\n"
	                      ".set noreorder\n"
		              ".set mips64\n"
		              "move $8, %1\n"
		              "1: c2 0x80001\n"
		              "mfc2 $8, "STR(MSGRNG_MSG_STATUS_REG)"\n"
		              "andi $8, $8, 0x6\n"
		              "beqz $8, 2f\n"
			      "addi %2, -1\n"
			      "bnez %2, 1b\n"
		              "move $8, %1\n"
			      "addiu $8, $0, 4\n"
			      "2: move %0, $8\n"
		              ".set pop\n"
		              :"=r"(ret)
		              : "r"((1<<16)|(code<<8)|stid), "r"(retry)
		              : "$8"
		             );
	return ret;
}


#define message_receive_fast_1(bucket, size, code, stid, msg0)   \
        ( { unsigned int _status=0, _tmp=0;                     \
           msgrng_receive(bucket);                              \
           while ( (_status=msgrng_read_status()) & 0x08) ;     \
           _tmp = _status & 0x30;                               \
           if (likely(!_tmp)) {                                 \
                 (size)=((_status & 0xc0)>>6)+1;                \
                 (code)=(_status & 0xff00)>>8;                  \
                 (stid)=(_status & 0x7f0000)>>16;               \
                 (msg0)=msgrng_load_rx_msg0();                  \
                 _tmp=0;                                        \
                }                                               \
           _tmp;                                                \
        } )


static inline void prefetch_local(const void *addr)
{
	__asm__ __volatile__(
	"	.set	mips4		\n"
	"	pref	%0, (%1)	\n"
	"	.set	mips0		\n"
	:
	: "i" (Pref_StoreStreamed), "r" (addr));
}



#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
/* skb transfer statistics */
unsigned long long skb_transfer_stat[NR_CPUS][NR_CPUS];
void skb_transfer_finish(void);
static void skb_transfer(int bucket, struct sk_buff *skb);


/* skb transfer queues, one per CPU */
static struct sk_buff_head cpu_skb_tqueue[NR_CPUS];

static void
cpu_tx_queue_init(void)
{
  int i;

  for (i = 0; i < NR_CPUS; i++)
  {
    skb_queue_head_init(&(cpu_skb_tqueue[i]));
  }
}

#endif /* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */


/* This message ring interrupt type, can be adjusted by NAPI setup callback */
extern int msgring_int_type;
extern struct user_mac_data *user_mac;
extern struct user_mac_kernal_data user_mac_krnl_data;

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
static int nlm_xlr_napi_setup(void);
#endif

#ifdef CONFIG_NLMCOMMON_HW_BUFFER_MGMT
static int setup_auto_free(struct sk_buff *skb, int type, struct msgrng_msg *msg);
#endif /* CONFIG_NLMCOMMON_HW_BUFFER_MGMT */

/* global flag for automatic hardware buffer management, disabled by default */
int nlm_auto_buffer_mgmt = 0;


#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
void xlr_napi_rx_schedule(void);
static int mac_frin_replenish_one_msg(struct net_device *dev);
static int nlm_xlr_napi_mac_xmit(struct sk_buff *skb, struct net_device *dev);

/*
 * get_adjusted_bucket_index: returns index of the highest bit set incremented by 4
 * 
 * E.g.: get_adjusted_bucket_index(1) = 4
 *       get_adjusted_bucket_index(2) = 5
 *       get_adjusted_bucket_index(3) = 5
 *
*/
static inline int
get_adjusted_bucket_index(int word)
{
	__asm__ __volatile__ (".set push\n"
	                      ".set noreorder\n"
		              ".set mips64\n"
			      "clz %0, %1\n"
			      ".set pop\n"
			      : "=r" (word) : "r" (word));
	return 35 - word;
}


/*
 * The following function checks if bucket/freeback allocation scheme is NAPI-compatible
 *
 * Returns 0 if napi compatibility fails and != 0 otherwise
*/
static int
nlm_napi_compatibility_check(void)
{
	__u32 freeback, i;


	printk("MSGRING_NAPI: HARD_CPU_ONLINE_MAP: 0x%08x\n", hard_cpu_online_map);

/*
	if (rmik_en) {
		for (i = 0; i < NR_CPUS; i++) {
			if ((hard_cpu_online_map & (1 << i)) == 0)
				continue;
			free_back_stid_map |= (1ULL << cpu_to_frstid[i]);
		}

		printk("MSGRING_NAPI: Incompatibility with CRF mode detected\n");
		return 0;
	}
*/
	for (i = 0; i < NR_CPUS; i++) {

		if ((hard_cpu_online_map & (1 << i)) == 0) {
			continue;
		}

		freeback = i / 4 * 8 + i % 4 + 4;

		if (cpu_to_frstid[i] != freeback) {

			printk("MSGRING_NAPI: Bucket allocation is not compatible with NAPI mode\n");
			printk("MSGRING_NAPI: Conflict detected: thread %d, freeback %d\n", i, cpu_to_frstid[i]);
			printk("MSGRING_NAPI: Expected freeback %d\n", freeback);

			return 0;
		}
	}
	return 1;
}

#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */

/*****************************************************************
 * Phoenix Generic Mac driver
 *****************************************************************/

typedef enum { xlr_mac_speed_10, xlr_mac_speed_100,
	       xlr_mac_speed_1000, xlr_mac_speed_rsvd
} xlr_mac_speed_t;

typedef enum { xlr_mac_duplex_auto, xlr_mac_duplex_half,
	       xlr_mac_duplex_full
} xlr_mac_duplex_t;

typedef enum { xlr_mac_fc_auto, xlr_mac_fc_disabled, xlr_mac_fc_frame,
	       xlr_mac_fc_collision, xlr_mac_fc_carrier
} xlr_mac_fc_t;

/* These 2 structures are always indexed by "hard_smp_processor_id()" */
static struct work_struct mac_frin_replenish_work[MAC_FRIN_WORK_NUM];
static struct tasklet_struct mac_frin_replenish_task[MAC_FRIN_WORK_NUM];

struct cpu_stat {
	unsigned long tx_packets;
	unsigned long txc_packets;
	unsigned long rx_packets;
	unsigned long interrupts;
};

struct phy_info {
	int addr;
	int mode;
	nlm_reg_t *mii_addr;
	nlm_reg_t *pcs_addr;
	nlm_reg_t *serdes_addr;
};

struct driver_data {

	/* Let these be the first fields in this structure 
	 * the structure is cacheline aligned when allocated in 
	 * init_etherdev
	 */
	struct fr_desc *frin_spill;
	struct fr_desc *frout_spill;
	union rx_tx_desc *class_0_spill;
	union rx_tx_desc *class_1_spill;
	union rx_tx_desc *class_2_spill;
	union rx_tx_desc *class_3_spill;

	struct net_device *dev;	/* pointer to linux device */
	struct timer_list link_timer;	/* for monitoring MII */
	struct net_device_stats stats;
	spinlock_t lock;

	nlm_reg_t *mmio;

	__u8 hwaddr[6];
	int phy_oldbmsr;
	int phy_oldanlpar;
	int phy_oldk1stsr;
	int phy_oldlinkstat;
	unsigned char phys_addr[2];

	xlr_mac_speed_t speed;	/* current speed */
	xlr_mac_duplex_t duplex;	/* current duplex */
	xlr_mac_fc_t flow_ctrl;	/* current flow control setting */
	int				advertising;

	int id;
	int type;
	int instance;
	uint32_t cfg_flag;

	int spill_init;
	int config_pde;
	int num_desc;

	struct phy_info phy;
	
	atomic_t frin_to_be_sent[MAC_FRIN_WORK_NUM];
	int init_frin_desc;

	struct cpu_stat cpu_stats[NR_CPUS];

	int fr_stid;
	int tx_stid;
	int frstid_rsvd;
};

enum {
	PORT_TX,
	PORT_TX_COMPLETE,
	PORT_STARTQ,
	PORT_STOPQ,
	PORT_START_DEV_STATE,
	PORT_STOP_DEV_STATE,
};

#define port_inc_counter(port, counter) 	atomic_inc(&port_counters[port][(counter)])
#define port_set_counter(port, counter, value) 	atomic_set(&port_counters[port][(counter)], (value))
static atomic_t port_counters[8][8] __cacheline_aligned;
static spinlock_t pending_tx_lock[NETLOGIC_MAX_MACS] __cacheline_aligned;
static volatile int pending_tx[NETLOGIC_MAX_MACS] __cacheline_aligned;

int mac_xmit(struct sk_buff *skb, struct net_device *dev,
		    struct driver_data *priv, int txq);

static __inline__ unsigned int nlm_ldadd_wu(unsigned int value, unsigned long *addr)
{
	__asm__ __volatile__(".set push\n"
			     ".set noreorder\n"
			     ".set mips64\n"
			     "move $8, %2\n" "move $9, %3\n"
#ifdef CONFIG_64BIT
			     //"ldadd $8, $9\n"
			     ".dword 0x71280012\n"
#else
			     //"ldaddwu $8, $9\n"
			     ".word 0x71280011\n"
#endif
			     "move %0, $8\n"
			     ".set pop\n":"=&r"(value), "+m"(*addr)
			     :"0"(value), "r"((unsigned long)addr)
			     :"$8", "$9");
	return value;
}
#define mac_stats_add(x, val) nlm_ldadd_wu(val, &x)

void mac_stats_update(int pkts, struct sk_buff *skb)
{
	struct driver_data *priv;
	priv = netdev_priv(skb->dev);
	mac_stats_add(priv->stats.rx_packets, pkts);
	mac_stats_add(priv->stats.rx_bytes, skb->len);
}

void nlm_xlr_mac_set_enable(struct driver_data *priv, int flag);
static void xlr_mac_set_rx_mode(struct net_device *dev);
void nlm_xlr_mac_msgring_handler(int bucket, int size, int code,
				  int stid, struct msgrng_msg *msg,
				  void *data);
static irqreturn_t nlm_xlr_mac_int_handler(int irq, void *dev_id);
static int nlm_xlr_mac_open(struct net_device *dev);
static int nlm_xlr_mac_xmit(struct sk_buff *skb, struct net_device *dev);
static int nlm_xlr_mac_close(struct net_device *dev);
static void nlm_xlr_mac_timer(unsigned long data);
static struct net_device_stats *nlm_xlr_mac_get_stats(struct net_device *dev);
static void nlm_xlr_mac_set_multicast_list(struct net_device *dev);
static int nlm_xlr_mac_do_ioctl(struct net_device *dev,
				 struct ifreq *rq, int cmd);
static void nlm_xlr_mac_tx_timeout(struct net_device *dev);
static int nlm_xlr_mac_change_mtu(struct net_device *dev, int new_mtu);
static int nlm_xlr_mac_fill_rxfr(struct net_device *dev);
static void nlm_xlr_config_spill_area(struct driver_data *priv);
static int mac_frin_replenish_one_msg(struct net_device *dev);



#define MSGRING_PROCESS_FROUT_START_BUCKET 4
#define MSGRING_PROCESS_FROUT_END_BUCKET 8
#define MSGRING_PROCESS_FROUT_POP_BUCKET_MASK 0xf0
void msgring_process_rx_msgs(int start_bucket, int end_bucket, __u32 pop_bucket_mask);



/*****************************************************************
 * Driver Helper Functions
 *****************************************************************/

static __inline__ struct sk_buff *mac_get_skb_back_ptr(unsigned long addr)
{
	unsigned long *back_ptr =
		(unsigned long *)(addr - MAC_SKB_BACK_PTR_SIZE);
	dbg_msg("addr = %lx,  skb = %lx\n", addr, *back_ptr);
	/* this function should be used only for newly allocated packets. It assumes
	 * the first cacheline is for the back pointer related book keeping info
	 */
	return (struct sk_buff *)(*back_ptr);
}

static __inline__ void mac_put_skb_back_ptr(struct sk_buff *skb)
{
	unsigned long *back_ptr = (unsigned long *)skb->data;

	/* this function should be used only for newly allocated packets. It assumes
	 * the first cacheline is for the back pointer related book keeping info
	 */
	skb_reserve(skb, MAC_SKB_BACK_PTR_SIZE);
	*back_ptr = (unsigned long)skb;
	dbg_msg("p=%p, skb=%p\n", back_ptr, skb);
}

#define CACHELINE_ALIGNED_ADDR(addr) (((unsigned long)(addr)) & ~(SMP_CACHE_BYTES-1))

static __inline__ void *cacheline_aligned_kmalloc(int size, int gfp_mask)
{
	void *buf = kmalloc(size + SMP_CACHE_BYTES, gfp_mask);
	if (buf)
		buf =
			(void
			 *)(CACHELINE_ALIGNED_ADDR((unsigned long)buf +
						   SMP_CACHE_BYTES));
	return buf;
}

static __inline__ struct sk_buff *nlm_xlr_alloc_skb(void)
{
	int offset = 0;
	struct sk_buff *skb = __dev_alloc_skb(NLM_RX_BUF_SIZE, GFP_KERNEL);

	if (!skb) {
		return NULL;
	}

	/* align the data to the next cache line */
	offset = ((unsigned long)skb->data + SMP_CACHE_BYTES) &
		~(SMP_CACHE_BYTES - 1);
	skb_reserve(skb, (offset - (unsigned long)skb->data));

	return skb;
}

/**********************************************************************
 **********************************************************************/
void nlm_xlr_mac_set_enable(struct driver_data *priv, int flag)
{
	uint32_t regval;
	int tx_threshold = 512;

	if(!(PORT_EN(priv->cfg_flag)))
		return;


	if (flag) {
		regval = netlogic_read_reg(priv->mmio, R_TX_CONTROL);
		regval &= ~(0x3fff);
		regval |= (1 << O_TX_CONTROL__TxEnable) |
			(tx_threshold << O_TX_CONTROL__TxThreshold);

		netlogic_write_reg(priv->mmio, R_TX_CONTROL, regval);

		regval = netlogic_read_reg(priv->mmio, R_RX_CONTROL);
		regval |= 1 << O_RX_CONTROL__RxEnable;
		if (priv->phy.serdes_addr != 0 && (priv->phy.mode & PHY_MODE_RGMII))
			regval |= 1 << O_RX_CONTROL__RGMII;
		netlogic_write_reg(priv->mmio, R_RX_CONTROL, regval);
	} else {
		regval = netlogic_read_reg(priv->mmio, R_TX_CONTROL);
		regval &= ~((1 << O_TX_CONTROL__TxEnable) |
			    (tx_threshold << O_TX_CONTROL__TxThreshold));

		netlogic_write_reg(priv->mmio, R_TX_CONTROL, regval);

		regval = netlogic_read_reg(priv->mmio, R_RX_CONTROL);
		regval &= ~(1 << O_RX_CONTROL__RxEnable);
		netlogic_write_reg(priv->mmio, R_RX_CONTROL, regval);
	}
}



/**********************************************************************
 **********************************************************************/
static __inline__ int xlr_mac_send_fr(struct driver_data *priv,
				       unsigned long addr, int len)
{
	int stid = 0;
	struct msgrng_msg msg;

 	stid = priv->fr_stid;
	msg.msg0 = (uint64_t)addr & 0xffffffffe0ULL;
    	msg.msg1 = msg.msg2 = msg.msg3 = 0;

	/* Send the packet to MAC */
	dbg_msg("mac_%d: Sending free packet to stid %d\n",
		                                  priv->instance, stid);
	__sync();
	if (priv->type == TYPE_XGMAC) {
		while (message_send_fast_1(MSGRNG_CODE_XGMAC, stid, msg.msg0));
	} else {
		while (message_send_fast_1(MSGRNG_CODE_MAC, stid, msg.msg0));
	}

	/* Let the mac keep the free descriptor */
	return 0;
}





/*
 * Configure and send SKB to device free-in ring
*/
static int
mac_frin_send_skb(struct net_device *dev, struct sk_buff *skb)
{
	int offset = 0;
	unsigned long msgrng_flags = 0;
	struct driver_data *priv;


	priv = netdev_priv(dev);

	/* align the data to the next cache line */
	offset = (((unsigned long)skb->data + SMP_CACHE_BYTES) & ~(SMP_CACHE_BYTES - 1));
	skb_reserve(skb, (offset - (unsigned long)skb->data));

#ifdef CONFIG_NLMCOMMON_HW_BUFFER_MGMT
	if (nlm_auto_buffer_mgmt) {
		/* Put skb under automatic buffer management */
		skb_shinfo(skb)->nlm_flags = 1;
		skb_shinfo(skb)->nlm_owner = dev;
		skb_shinfo(skb)->nlm_refill = mac_frin_replenish_one_msg;
	} else {
		skb_shinfo(skb)->nlm_flags = 0;
		skb_shinfo(skb)->nlm_owner = NULL;
		skb_shinfo(skb)->nlm_refill = NULL;
	}
#endif /* CONFIG_NLMCOMMON_HW_BUFFER_MGMT */

	mac_put_skb_back_ptr(skb);
	msgrng_access_enable(msgrng_flags);
	if (xlr_mac_send_fr(priv, virt_to_bus(skb->data), skb->len)) {
		dev_kfree_skb(skb);
		printk("[%s]: rx free message_send failed!\n", __FUNCTION__);
	}
	msgrng_access_disable(msgrng_flags);

	return 0; 
}


/*
 * Allocates new SKB for a particular device and queues it
 * up to the device Rx ring
*/
static int
mac_frin_replenish_one_msg(struct net_device *dev)
{
	struct sk_buff *skb = 0;


	if (!dev) {
		return 0;
	}

	skb = __dev_alloc_skb(NLM_RX_BUF_SIZE, GFP_ATOMIC);
	if (!skb) {
		printk(KERN_ALERT "%s: can't alloc skb\n", __FUNCTION__);
		return 0;
	}
	xlr_inc_counter(REPLENISH_FRIN);
  
	return mac_frin_send_skb(dev, skb); 
}




#ifdef CONFIG_NLMCOMMON_HW_BUFFER_MGMT
/*
 * This helper macro resets SKB data pointers for reuse
 * as free-in buffer
*/
#define skb_reset_ptrs(skb) \
do { \
	struct skb_shared_info *shinfo; \
	\
	shinfo = skb_shinfo(skb); \
	\
	\
	/* Now reinitialize old skb, cut & paste from dev_alloc_skb */ \
	memset(skb, 0, offsetof(struct sk_buff, tail)); \
	skb->data = skb->head;  \
	skb_reset_tail_pointer(skb);\
	\
	atomic_set(&shinfo->dataref, 1); \
	shinfo->nr_frags  = 0; \
	shinfo->gso_size = 0; \
	shinfo->gso_segs = 0; \
	shinfo->gso_type = 0; \
	shinfo->ip6_frag_id = 0; \
	shinfo->frag_list = NULL; \
} while (0)


/*
 * If we are in the HW buffer management case we handler frames with rx errors
 * via this function
*/
static void 
discard_rx_frame(struct net_device *dev, struct sk_buff *skb, int cpu)
{
	/* Reset all fields to 0, reset data pointers */
	skb_reset_ptrs(skb);

	mac_frin_send_skb(dev, skb); 
}


/*
 *  Prepare SKB for automatic memory management operation (buffer recycling)
 *
 *  Return: 0 -- recycling is not possible
 *          1 -- SKB set up for recycling successfully
*/
static int
setup_auto_free(struct sk_buff *skb, int type, struct msgrng_msg *msg)
{
	struct driver_data *priv;
	struct skb_shared_info *shinfo;
	int fr_stid, offset;

	shinfo = skb_shinfo(skb);
	if (!shinfo->nlm_flags)
		return 0;

	if (atomic_read(&skb->users) != 1) {
		printk(KERN_ALERT "%s: Can't recycle because of users count\n", __FUNCTION__);
		return 0;
	}

	if (skb->cloned || atomic_read(&(skb_shinfo(skb)->dataref)) != 1) {
		printk(KERN_EMERG "%s: Can't recycle because of cloned or dataref\n", __FUNCTION__);
		return 0;
	}

	/* Leak no dsk entries! */
	skb_dst_drop(skb);

	/* Reset all fields to 0, reset data pointers */
	skb_reset_ptrs(skb);

	offset = (((unsigned long)skb->data + SMP_CACHE_BYTES) & ~(SMP_CACHE_BYTES - 1));
	skb_reserve(skb, (offset - (unsigned long)skb->data));

	priv = netdev_priv(skb_shinfo(skb)->nlm_owner);
	fr_stid = priv->fr_stid;

	mac_put_skb_back_ptr(skb);

	msg->msg1 = ( ((uint64_t) 1 << 63) |
		      ((uint64_t) fr_stid << 54) |
		      ((uint64_t) 0 << 40) |
		      ((uint64_t)virt_to_phys(skb->data) & 0xffffffffffULL)
		    );
	return 1;
}

#endif /* CONFIG_NLMCOMMON_HW_BUFFER_MGMT */



/**************************************************************/

static void xgmac_mdio_setup(volatile unsigned int *_mmio)
{
	int i;
	uint32_t rd_data;

	for (i = 0; i < 4; i++) {
		rd_data = xmdio_read(_mmio, 1, 0x8000 + i);
		rd_data = rd_data & 0xffffdfff;	// clear isolate bit
		xmdio_write(_mmio, 1, 0x8000 + i, rd_data);
	}
}

/**********************************************************************
 *  Init MII interface
 *  
 *  Input parameters: 
 *  	   s - priv structure
 ********************************************************************* */
#define PHY_STATUS_RETRIES 25000

static void nlm_xlr_mac_mii_init(struct driver_data *priv)
{
    /* use the lowest clock divisor - divisor 28 */
    netlogic_write_reg(priv->phy.mii_addr, R_MII_MGMT_CONFIG, 0x07);
}

/**********************************************************************
 *  Read a PHY register.
 *  
 *  Input parameters: 
 *  	   phyaddr - PHY's address
 *  	   regidx = index of register to read
 *  	   
 *  Return value:
 *  	   value read (16 bits), or 0xffffffff if an error occurred.
 ********************************************************************* */
static unsigned int nlm_xlr_mac_mii_read(nlm_reg_t *mmio, int phyaddr, int regidx)
{
	int i;
	/* setup the phy reg to be used */
	netlogic_write_reg(mmio, R_MII_MGMT_ADDRESS,
			  (phyaddr << 8) | (regidx << 0));

	/* Issue the read command */
	netlogic_write_reg(mmio, R_MII_MGMT_COMMAND,
			  (1 << O_MII_MGMT_COMMAND__rstat));

	/* poll for the read cycle to complete */
	for (i = 0; i < PHY_STATUS_RETRIES; i++) {
		if (netlogic_read_reg(mmio, R_MII_MGMT_INDICATORS) == 0)
			break;
	}

	/* clear the read cycle */
	netlogic_write_reg(mmio, R_MII_MGMT_COMMAND, 0);

	if (i == PHY_STATUS_RETRIES) {
		return 0xffffffff;
	}

	/* Read the data back */
	return netlogic_read_reg(mmio, R_MII_MGMT_STATUS);
}

/**********************************************************************
 *  Write a value to a PHY register.
 *  
 *  Input parameters: 
 *  	   s - priv structure
 *  	   phyaddr - PHY to use
 *  	   regidx - register within the PHY
 *  	   regval - data to write to register
 *  	   
 *  Return value:
 *  	   nothing
 ********************************************************************* */
static void nlm_xlr_mac_mii_write(nlm_reg_t *mmio, int phyaddr, int regidx, unsigned int regval)
{
	int i = 0;

	netlogic_write_reg(mmio, R_MII_MGMT_ADDRESS,
			  (phyaddr << 8) | (regidx << 0));

	/* Write the data which starts the write cycle */
	netlogic_write_reg(mmio, R_MII_MGMT_WRITE_DATA, regval);

	/* poll for the write cycle to complete */
	for (i = 0; i < PHY_STATUS_RETRIES; i++) {
		if (netlogic_read_reg(mmio, R_MII_MGMT_INDICATORS) == 0)
			break;
	}

	return;
}


/*****************************************************************
 * Initialize GMAC
 *****************************************************************/
static void nlm_xlr_config_pde(struct driver_data *priv)
{
	int i = 0, cpu = 0, bucket = 0;
	__u64 bucket_map = 0;

	for (i = 0; i < 32; i++) {
		if (cpu_isset(i, cpu_online_map)) {
			cpu = cpu_logical_map(i);
				bucket = ((cpu >> 2) << 3) | (cpu & 0x03);
			bucket_map |= (1ULL << bucket);
			dbg_msg("i=%d, cpu=%d, bucket = %d, bucket_map=%llx\n",
				i, cpu, bucket, bucket_map);
		}
	}
	netlogic_write_reg(priv->mmio, R_PDE_CLASS_0, (bucket_map & 0xffffffff));
	netlogic_write_reg(priv->mmio, R_PDE_CLASS_0 + 1,
			  ((bucket_map >> 32) & 0xffffffff));

	netlogic_write_reg(priv->mmio, R_PDE_CLASS_1, (bucket_map & 0xffffffff));
	netlogic_write_reg(priv->mmio, R_PDE_CLASS_1 + 1,
			  ((bucket_map >> 32) & 0xffffffff));

	netlogic_write_reg(priv->mmio, R_PDE_CLASS_2, (bucket_map & 0xffffffff));
	netlogic_write_reg(priv->mmio, R_PDE_CLASS_2 + 1,
			  ((bucket_map >> 32) & 0xffffffff));

	netlogic_write_reg(priv->mmio, R_PDE_CLASS_3, (bucket_map & 0xffffffff));
	netlogic_write_reg(priv->mmio, R_PDE_CLASS_3 + 1,
			  ((bucket_map >> 32) & 0xffffffff));
}

static void nlm_xlr_config_parser(struct driver_data *priv)
{
	/* Mark it as no classification 
	 * The parser extract is gauranteed to be zero with no classfication
	 */
	netlogic_write_reg(priv->mmio, R_L2TYPE_0, 0x00);
	
}

static void nlm_xlr_config_classifier(struct driver_data *priv)
{
	int i = 0;

	if (priv->type == TYPE_XGMAC) {
		/* xgmac translation table doesn't have sane values on reset */
		for(i=0;i<64;i++)
			netlogic_write_reg(priv->mmio, R_TRANSLATETABLE + i, 0x0);		

		/* use upper 7 bits of the parser extract to index the translate
		 * table
		 */
		netlogic_write_reg(priv->mmio, R_PARSERCONFIGREG, 0x0);
	}
}

static void nlm_xlr_gmac_clr_pending_intr(struct driver_data *phy_priv)
{
	nlm_reg_t *mmio = NULL;

    if(!xlr_board_atx_vii())
        return;

    if(phy_priv->instance == 0){
        /*All MDIO interrupts goes to mdio 0 - ack mac 0*/
        mmio = phy_priv->mmio;
	    netlogic_write_reg(mmio, R_INTREG, 0xffffffff);
    }
}

static void nlm_xlr_gmac_config_speed(struct driver_data *priv)
{
	nlm_reg_t *mmio = priv->mmio;
	int id = priv->instance;

	priv->speed = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, 28);
	priv->speed = (priv->speed >> 3) & 0x03;

	if (priv->speed == xlr_mac_speed_10) {
		if (priv->phy.serdes_addr)
			netlogic_write_reg(mmio, R_INTERFACE_CONTROL, SGMII_SPEED_10);
		netlogic_write_reg(mmio, R_MAC_CONFIG_2, 0x7117);
		netlogic_write_reg(mmio, R_CORECONTROL, 0x02);
		printk("configuring gmac_%d in 10Mbps mode\n", id);
	} else if (priv->speed == xlr_mac_speed_100) {
		if (priv->phy.serdes_addr)
			netlogic_write_reg(mmio, R_INTERFACE_CONTROL, SGMII_SPEED_100);
		netlogic_write_reg(mmio, R_MAC_CONFIG_2, 0x7117);
		netlogic_write_reg(mmio, R_CORECONTROL, 0x01);
		printk("configuring gmac_%d in 100Mbps mode\n", id);
	} else {
		if (priv->phy.serdes_addr)
			netlogic_write_reg(mmio, R_INTERFACE_CONTROL, SGMII_SPEED_1000);
		if (priv->speed != xlr_mac_speed_1000) {
			printk("gmac_%d phy reported unknown mac speed," 
				" defaulting to 100Mbps mode\n",id);
			netlogic_write_reg(mmio, R_MAC_CONFIG_2, 0x7117);
			netlogic_write_reg(mmio, R_CORECONTROL, 0x01);
		}else{
			netlogic_write_reg(mmio, R_MAC_CONFIG_2, 0x7217);
			netlogic_write_reg(mmio, R_CORECONTROL, 0x00);
			printk("configuring gmac_%d in 1000Mbps mode\n", id);
		}
	}
}

/*****************************************************************
 * Initialize XGMAC
 *****************************************************************/
static void nlm_xlr_xgmac_init(struct driver_data *priv, struct port_cfg *pcfg)
{
	int i = 0;
	nlm_reg_t *mmio = priv->mmio;
	int id = priv->instance;
	volatile unsigned short *cpld;
	uint32_t rx_control;
	bucket_t *bucket;
	struct stn_cc *credit;

	cpld = (volatile unsigned short *)(unsigned long)0xffffffffBD840000ULL;
	netlogic_write_reg(priv->mmio, R_DESC_PACK_CTRL,
			  (MAC_MAX_FRAME_SIZE << O_DESC_PACK_CTRL__RegularSize) | (4 << 20));
	netlogic_write_reg(priv->mmio, R_BYTEOFFSET0, BYTE_OFFSET);

	if(priv->config_pde) {
	nlm_xlr_config_pde(priv);
	nlm_xlr_config_parser(priv);
	nlm_xlr_config_classifier(priv);
	}

	netlogic_write_reg(priv->mmio, R_MSG_TX_THRESHOLD, 1);

	/* configure the XGMAC Registers */
	netlogic_write_reg(mmio, R_XGMAC_CONFIG_1, 0x50000026);

	/* configure the XGMAC_GLUE Registers */
	netlogic_write_reg(mmio, R_DMACR0, 0xffffffff);
	netlogic_write_reg(mmio, R_DMACR1, 0xffffffff);
	netlogic_write_reg(mmio, R_DMACR2, 0xffffffff);
	netlogic_write_reg(mmio, R_DMACR3, 0xffffffff);
	netlogic_write_reg(mmio, R_STATCTRL, 0x04);
	netlogic_write_reg(mmio, R_L2ALLOCCTRL, 0xffffffff);

	netlogic_write_reg(mmio, R_XGMACPADCALIBRATION, 0x030);
	netlogic_write_reg(mmio, R_EGRESSFIFOCARVINGSLOTS, 0x0f);
	netlogic_write_reg(mmio, R_L2ALLOCCTRL, 0xffffffff);
	netlogic_write_reg(mmio, R_XGMAC_MIIM_CONFIG, 0x3e);

	/* take XGMII phy out of reset 
	 */
	/* we are pulling everything out of reset because writing a 0 would
	 * reset other devices on the chip
	 */
	cpld[ATX_CPLD_RESET_1] = 0xffff;
	cpld[ATX_CPLD_MISC_CTRL] = 0xffff;
	cpld[ATX_CPLD_RESET_2] = 0xffff;

	xgmac_mdio_setup(mmio);

	nlm_xlr_config_spill_area(priv);

	bucket = pcfg->bucket;
	credit = pcfg->credit;

	if (id == 0) {
		for (i = 0; i < 16; i++) {
			netlogic_write_reg(mmio, R_XGS_TX0_BUCKET_SIZE + i,
					  bucket[MSGRNG_STNID_XGS0_TX + i]);
		}

		netlogic_write_reg(mmio, R_XGS_RFR_BUCKET_SIZE,
				  bucket[MSGRNG_STNID_XMAC0RFR]);

	} else if (id == 1) {
		for (i = 0; i < 16; i++) {
			netlogic_write_reg(mmio, R_XGS_TX0_BUCKET_SIZE + i,
					  bucket[MSGRNG_STNID_XGS1_TX + i]);
		}

		netlogic_write_reg(mmio, R_XGS_RFR_BUCKET_SIZE,
				  bucket[MSGRNG_STNID_XMAC1RFR]);

	}
		for (i = 0; i < MAX_NUM_MSGRNG_STN_CC; i++) {
			netlogic_write_reg(mmio, R_CC_CPU0_0 + i,
				credit->counters[i >> 3][i & 0x07]);
	}

	priv->init_frin_desc = 1;

    /* Clear the flagging of rx length check errors */
    rx_control = netlogic_read_reg(mmio, R_RX_CONTROL);
    rx_control &= ~(1 << 9);
    netlogic_write_reg(mmio, R_RX_CONTROL, rx_control);
}


void sgmii_serdes_reset(void) 
{
	int i;
	volatile unsigned int *mmio_gpio;
	mmio_gpio = (unsigned int *)(netlogic_io_base + NETLOGIC_IO_GPIO_OFFSET);

	for (i=0;i<1000000;i++);

	// use 125 Mhz instead of 156.25Mhz ref clock
	if (!xlsb0_in_xaui(0)) {
		mmio_gpio[0x10] = 0x7103;
	}

	if (!xlsb0_in_xaui(1)) {
		mmio_gpio[0x21] = 0x7103;
	}

	for (i=0;i<1000000;i++);
}



static void serdes_regs_init(struct driver_data *priv) 
{
    int i;
    volatile unsigned int *mmio_gpio;
    mmio_gpio = (unsigned int *)(netlogic_io_base + NETLOGIC_IO_GPIO_OFFSET);
    /*
       P Reg   Val
       -------------
       26 0     6DB0
       26 1     0FFF
       26 2     B6D0
       26 3     00FF
       26 4     0000
       26 5     0000
       26 6     0005
       26 7     0001
       26 8     0000
       26 9     0000
       26 10    0000
     */
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 0, 0x6DB0);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 1, 0xFFFF);  
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 2, 0xB6D0);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 3, 0x00FF);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 4, 0x0000);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 5, 0x0000);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 6, 0x0005);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 7, 0x0001);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 8, 0x0000);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26, 9, 0x0000);
    nlm_xlr_mac_mii_write (priv->phy.serdes_addr, 26,10, 0x0000);

    for(i=0;i<10000000;i++){}

    /* program  GPIO values for serdes init parameters */
    mmio_gpio[0x20] = 0x7e6802;
    mmio_gpio[0x10] = 0x7104;
    for(i=0;i<100000000;i++){}

    if (xlr_board_atx_xaui_rework())
	    sgmii_serdes_reset();
}

static void serdes_autoconfig(struct driver_data *priv)
{
    int delay = 100;

    /* Enable Auto negotiation in the PCS Layer*/
    if ( (priv->instance == 0) || (priv->instance == 4)) {
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 27, 0, 0x1000);
	mdelay(delay);
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 27, 0, 0x0200);
	mdelay(delay);
    }

    if ( (priv->instance == 1) || (priv->instance == 5)) {
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 28, 0, 0x1000);
	mdelay(delay);
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 28, 0, 0x0200);
	mdelay(delay);
    }

    if ( (priv->instance == 2) || (priv->instance == 6)) {
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 29, 0, 0x1000);
	mdelay(delay);
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 29, 0, 0x0200);
	mdelay(delay);
    }

    if ( (priv->instance == 3) || (priv->instance == 7)) {
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 30, 0, 0x1000);
	mdelay(delay);
	nlm_xlr_mac_mii_write (priv->phy.pcs_addr, 30, 0, 0x0200);
	mdelay(delay);
    }

    return;
}

void xaui_serdes_reset( void ) 
{
    int i;
    volatile unsigned int *mmio_gpio;
    mmio_gpio = (unsigned int *)(netlogic_io_base +
            NETLOGIC_IO_GPIO_OFFSET);

    for (i=0;i<1000000;i++);

    // disable serdes pll for both serdes
    mmio_gpio[0x20] = 0x007e6804;
    mmio_gpio[0x22] = 0x007e6804;
    for (i=0;i<1000000;i++);

    // use 156.25Mhz ref clock instead of 125Mhz
    // ref clk
    mmio_gpio[0x10] = 0x7104;
    mmio_gpio[0x21] = 0x7104;
    for (i=0;i<1000000;i++);

    // re-enable serdes pll
    mmio_gpio[0x20] = 0x007e6801;
    mmio_gpio[0x22] =
        0x007e6801;

    for (i=0;i<1000000;i++);
}


static void nlm_xlr_xaui_init(struct driver_data *priv, struct port_cfg *pcfg)
{
	int i = 0;
	nlm_reg_t *mmio = priv->mmio;
	__u32 value = 0;
	bucket_t *bucket;
	struct stn_cc *credit;

    value = netlogic_read_reg(mmio, R_XGMAC_CONFIG_1);
    netlogic_write_reg(mmio, R_XGMAC_CONFIG_1, (value | 0x50000020));
    netlogic_write_reg(mmio, R_XGMAC_MAX_FRAME_LEN, 0x0A000A00);

	nlm_xlr_config_spill_area(priv);

	netlogic_write_reg(priv->mmio, R_DESC_PACK_CTRL,
			  (BYTE_OFFSET << O_DESC_PACK_CTRL__ByteOffset) |
			  (1 << O_DESC_PACK_CTRL__MaxEntry)| 
			  (MAC_MAX_FRAME_SIZE << O_DESC_PACK_CTRL__RegularSize));
   #ifdef CONFIG_NLMCOMMON_PTP_SUPPORT 
   	netlogic_write_reg(priv->mmio, R_DESC_PACK_CTRL,
                      netlogic_read_reg(priv->mmio, R_DESC_PACK_CTRL)|
                      (1<< O_DESC_PACK_CTRL__PrePadEnable)); 
   #endif

	if(priv->config_pde) {
	nlm_xlr_config_pde(priv);
	nlm_xlr_config_parser(priv);
	nlm_xlr_config_classifier(priv);
	}

    netlogic_write_reg(priv->mmio, R_MSG_TX_THRESHOLD, 3);

    netlogic_write_reg(mmio,R_RX_CONTROL,(0x7<<6));

	netlogic_write_reg(mmio, R_DMACR0, (7<<28)|(7<<24));
	netlogic_write_reg(mmio, R_DMACR3,
            (2<<21)|(2<<18)|(2<<15)|(2<<12)|(2<<9)|(2<<6)|(2<<3)|(2<<0));
	netlogic_write_reg(mmio, R_L2ALLOCCTRL, 0xffffffff);
	netlogic_write_reg(mmio, 0x221, (224 << 16));
	netlogic_write_reg(mmio, R_STATCTRL, 0x04);
	netlogic_write_reg(mmio, R_INTMASK, 0);
	netlogic_write_reg(mmio, R_FREEQCARVE, 0);

    priv->init_frin_desc = 1;
    bucket = pcfg->bucket;
	credit = pcfg->credit;

	if(bucket != NULL) {
			netlogic_write_reg(mmio, R_GMAC_RFR0_BUCKET_SIZE, bucket[1]);
			netlogic_write_reg(mmio, R_GMAC_TX0_BUCKET_SIZE,  bucket[2]);
			netlogic_write_reg(mmio, R_GMAC_TX1_BUCKET_SIZE,  bucket[3]);
			netlogic_write_reg(mmio, R_GMAC_TX2_BUCKET_SIZE,  bucket[4]);
			netlogic_write_reg(mmio, R_GMAC_TX3_BUCKET_SIZE,  bucket[5]);
			netlogic_write_reg(mmio, R_GMAC_RFR1_BUCKET_SIZE, bucket[7]);
	}

	if(credit != NULL) {
		for (i = 0; i < MAX_NUM_MSGRNG_STN_CC; i++) {
			netlogic_write_reg(mmio, R_CC_CPU0_0 + i,
					credit->counters[i >> 3][i & 0x07]);
		}
	}
	return;
}
/*******************************************************
 * Initialization gmac
 *******************************************************/
static void nlm_xlr_gmac_init(struct driver_data *priv, struct port_cfg *pcfg)
{
	int i = 0;
	nlm_reg_t *mmio = priv->mmio;
	__u32 value = 0;
	bucket_t *bucket;
	struct stn_cc *credit;

	nlm_xlr_config_spill_area(priv);

	netlogic_write_reg(priv->mmio, R_DESC_PACK_CTRL,
			  (BYTE_OFFSET << O_DESC_PACK_CTRL__ByteOffset) |
			  (1 << O_DESC_PACK_CTRL__MaxEntry)| 
			  (MAC_MAX_FRAME_SIZE << O_DESC_PACK_CTRL__RegularSize));
   #ifdef CONFIG_NLMCOMMON_PTP_SUPPORT 
   	netlogic_write_reg(priv->mmio, R_DESC_PACK_CTRL,
                      netlogic_read_reg(priv->mmio, R_DESC_PACK_CTRL)|
                      (1<< O_DESC_PACK_CTRL__PrePadEnable)); 
   #endif

	if(priv->config_pde) {
	nlm_xlr_config_pde(priv);
	nlm_xlr_config_parser(priv);
	nlm_xlr_config_classifier(priv);
	}

	netlogic_write_reg(priv->mmio, R_MSG_TX_THRESHOLD, 3);

	netlogic_write_reg(mmio, R_MAC_CONFIG_1, 0x35);

          netlogic_write_reg(mmio,R_RX_CONTROL,(0x7<<6));

	if (priv->phy.serdes_addr != 0 && (priv->phy.mode & PHY_MODE_RGMII)) {
		value = netlogic_read_reg(priv->mmio, R_RX_CONTROL);
		value |= 1 << O_RX_CONTROL__RGMII;
		netlogic_write_reg(priv->mmio, R_RX_CONTROL, value);
	}

	nlm_xlr_mac_mii_init(priv);

	priv->advertising = ADVERTISED_10baseT_Full | ADVERTISED_10baseT_Half | 
			ADVERTISED_100baseT_Full | ADVERTISED_100baseT_Half |
			ADVERTISED_1000baseT_Full | ADVERTISED_Autoneg |
			ADVERTISED_MII;
    
	/*Clear pending mdio interrupt*/
	nlm_xlr_gmac_clr_pending_intr(priv);
    

	/* Enable all MDIO interrupts in the phy 
	 * RX_ER bit seems to be get set about every 1 sec in GigE mode,
	 * ignore it for now...
	 */
	nlm_xlr_mac_mii_write(priv->phy.mii_addr, priv->phy.addr, 25, 0xfffffffe);
	if(priv->phy.serdes_addr) {
        serdes_regs_init(priv);
        mdelay(10);
        serdes_autoconfig(priv);
    }
	nlm_xlr_gmac_config_speed(priv);

	value = netlogic_read_reg(mmio, R_IPG_IFG);
	netlogic_write_reg(mmio, R_IPG_IFG, ((value & ~0x7f) | MAC_B2B_IPG));
	netlogic_write_reg(mmio, R_DMACR0, 0xffffffff);
	netlogic_write_reg(mmio, R_DMACR1, 0xffffffff);
	netlogic_write_reg(mmio, R_DMACR2, 0xffffffff);
	netlogic_write_reg(mmio, R_DMACR3, 0xffffffff);
	netlogic_write_reg(mmio, R_STATCTRL, 0x04);
	netlogic_write_reg(mmio, R_L2ALLOCCTRL, 0xffffffff);
	netlogic_write_reg(mmio, R_INTMASK, 0);
	netlogic_write_reg(mmio, R_FREEQCARVE, 0);

		priv->init_frin_desc = 1;
	bucket = pcfg->bucket;
	credit = pcfg->credit;

	if(bucket != NULL) {
			netlogic_write_reg(mmio, R_GMAC_RFR0_BUCKET_SIZE, bucket[1]);
			netlogic_write_reg(mmio, R_GMAC_TX0_BUCKET_SIZE,  bucket[2]);
			netlogic_write_reg(mmio, R_GMAC_TX1_BUCKET_SIZE,  bucket[3]);
			netlogic_write_reg(mmio, R_GMAC_TX2_BUCKET_SIZE,  bucket[4]);
			netlogic_write_reg(mmio, R_GMAC_TX3_BUCKET_SIZE,  bucket[5]);
			netlogic_write_reg(mmio, R_GMAC_RFR1_BUCKET_SIZE, bucket[7]);
	}

	if(credit != NULL) {
		for (i = 0; i < MAX_NUM_MSGRNG_STN_CC; i++) {
			netlogic_write_reg(mmio, R_CC_CPU0_0 + i,
					credit->counters[i >> 3][i & 0x07]);
		}
	}
	return;
}

/**********************************************************************
 * Set promiscuous mode
 **********************************************************************/
static void xlr_mac_set_rx_mode(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	uint32_t regval;

	regval = netlogic_read_reg(priv->mmio, R_MAC_FILTER_CONFIG);

	if (dev->flags & IFF_PROMISC) {
		regval |= (1 << O_MAC_FILTER_CONFIG__BROADCAST_EN) |
			(1 << O_MAC_FILTER_CONFIG__PAUSE_FRAME_EN) |
			(1 << O_MAC_FILTER_CONFIG__ALL_MCAST_EN) |
			(1 << O_MAC_FILTER_CONFIG__ALL_UCAST_EN);
	} else {
		regval &= ~((1 << O_MAC_FILTER_CONFIG__PAUSE_FRAME_EN) |
			    (1 << O_MAC_FILTER_CONFIG__ALL_UCAST_EN));
#ifdef PA10401_1_GMAC_PKT_DISCARD
	  if (!is_xls()){
                regval |= (1 << O_MAC_FILTER_CONFIG__ALL_MCAST_EN) |
		          (1 << O_MAC_FILTER_CONFIG__ALL_UCAST_EN);
          } 
#endif
	}

	netlogic_write_reg(priv->mmio, R_MAC_FILTER_CONFIG, regval);
}

/**********************************************************************
 *  Configure LAN speed for the specified MAC.
 ********************************************************************* */
static int nlm_xlr_mac_set_speed(struct driver_data *s, xlr_mac_speed_t speed)
{
	return 0;
}

/**********************************************************************
 *  Set Ethernet duplex and flow control options for this MAC
 ********************************************************************* */
static int nlm_xlr_mac_set_duplex(struct driver_data *s,
				   xlr_mac_duplex_t duplex, xlr_mac_fc_t fc)
{
	return 0;
}

/*****************************************************************
 * Kernel Net Stack <-> MAC Driver Interface
 *****************************************************************/
/**********************************************************************
 **********************************************************************/
#define MAC_TX_PASS NETDEV_TX_OK
#define MAC_TX_FAIL NETDEV_TX_BUSY

static inline int xlr_netif_queue_tx(struct net_device *dev, 
		struct sk_buff *skb, int txq)
{
	unsigned long flags, mflags;
	int port = ((struct driver_data *)netdev_priv(dev))->id;
	struct driver_data *priv = netdev_priv(dev);
	int ret;

	spin_lock_irqsave(&pending_tx_lock[port], flags);
	/* try xmit once again. This should take care of the race b/w stopq 
	   here and wakeup in tx complete 
	   */

	msgrng_access_enable(mflags);
	ret = mac_xmit(skb, dev, priv, txq);
	msgrng_access_disable(mflags);

	if (ret == MAC_TX_PASS) {
		mac_stats_add(priv->cpu_stats[txq].tx_packets, 1);
		spin_unlock_irqrestore(&pending_tx_lock[port], flags);
		return ret;
	}
	pending_tx[port]++;
	netif_tx_stop_queue(netdev_get_tx_queue(dev, smp_processor_id()));
	priv->stats.tx_dropped++;
	ret = MAC_TX_FAIL;
	spin_unlock_irqrestore(&pending_tx_lock[port], flags);
	return ret;
}

static inline void xlr_netif_queue_tx_complete(struct net_device *dev)
{
	int port = ((struct driver_data *)netdev_priv(dev))->id;
	struct driver_data *priv = netdev_priv(dev);
	int end_port = 0;
	if(port < NETLOGIC_MAX_GMACS){
		port = (port/4) * 4;
		end_port = port + 4;
		for(; port<end_port; port++){
			if(pending_tx[port]) {
				priv = netdev_priv(dev_mac[port]);
				spin_lock(&pending_tx_lock[port]);
				if(pending_tx[port]){
					pending_tx[port] = 0;
					/* is there a easy way to wake up only stopped queues ? */
					netif_tx_wake_all_queues(dev);
				}
				spin_unlock(&pending_tx_lock[port]);
			}
		}
	}else{
		if(pending_tx[port]) {
			spin_lock(&pending_tx_lock[port]);
			if(pending_tx[port]){
				pending_tx[port] = 0;
				/* is there a easy way to wake up only stopped queues ? */
				netif_tx_wake_all_queues(dev);
			}
			spin_unlock(&pending_tx_lock[port]);
		}
	}
}



static int mac_fill_tx_stid(int id, int type)
{
	int tx_stid;
	if (type == TYPE_XGMAC) {
		tx_stid = msgrng_xgmac_stid_tx(id);
	} 
	else {
		tx_stid = msgrng_gmac_stid_tx(id);
	}
	return tx_stid;
}


static inline int mac_make_desc_b0_tx(struct msgrng_msg *msg, struct driver_data *priv,
				      unsigned long addr, struct sk_buff *skb, unsigned long desc_id)
{
	int tx_stid = 0;
	int fr_stid = 0;
	int cpu = (netlogic_cpu_id() << 2) | netlogic_thr_id();
	int len = skb->len;

	fr_stid = cpu_to_frstid[cpu];
	tx_stid = priv->tx_stid;


	msg->msg0 = ( ((uint64_t)1 << 63) | 
		      (((uint64_t)desc_id) << 54) | 
		      ((uint64_t)len << 40) | 
		      ((uint64_t)addr)
		    );

#ifdef CONFIG_NLMCOMMON_HW_BUFFER_MGMT
        if (nlm_auto_buffer_mgmt && setup_auto_free(skb, priv->type, msg))
		return tx_stid;
	else 
#endif /* CONFIG_NLMCOMMON_HW_BUFFER_MGMT */
	{
		msg->msg1 = ( ((uint64_t)1 << 63) |
			      ((uint64_t)fr_stid << 54) |
		      	      ((uint64_t)0 << 40) |
#ifdef CONFIG_64BIT
		              ((uint64_t)virt_to_phys(skb))
#else
		              ((unsigned long)(skb)&0xffffffffUL)
#endif
		            );
	}
		
	msg->msg2 = msg->msg3 = 0;

	return tx_stid;
}

int
mac_xmit(struct sk_buff *skb, struct net_device *dev,
		    struct driver_data *priv, int txq)
{
	struct msgrng_msg msg;
	int stid = 0;
	int cpu = (netlogic_cpu_id() << 2) | netlogic_thr_id();

	if(cpu_to_bktmask[cpu] == 0) {
		printk("Tx fail : No buckets are allocated for this cpu\n");
		return MAC_TX_FAIL;
	}
#ifdef  CONFIG_NLMCOMMON_PTP_SUPPORT
    if(skb->sk && sock_flag(skb->sk, SOCK_TIMESTAMP)) {
           dbg_msg("transmit timestamp packet \n"); 
       	   stid = mac_make_desc_b0_tx(&msg, priv, virt_to_phys(skb->data), skb, 126);
        } else 
#endif
        {
           	stid = mac_make_desc_b0_tx(&msg, priv, virt_to_phys(skb->data), skb, 127);
        }
       
	__sync();

	if (message_send_fast_2(MSGRNG_CODE_MAC, stid, msg.msg0, msg.msg1))
		return MAC_TX_FAIL;

	port_inc_counter(priv->instance, PORT_TX);

	/* Send the packet to MAC */
	dbg_msg("Sent tx packet to stid %d, msg0=%llx, msg1=%llx \n", stid, msg.msg0, msg.msg1);
#ifdef DUMP_PACKETS
	dump_packet(skb);
#endif

	xlr_inc_counter(NETIF_TX);

	dev->trans_start = jiffies;

	return MAC_TX_PASS;
}



#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI

/* 
 * NAPI poll function on upper four buckets 
*/
void
xlr_napi_poll_upper(struct net_device *dummy_dev, int budget)
{
	struct msgrng_msg msg_body, *msg = &msg_body;
	int bucket, stid = 0, length;
	unsigned long mflags = 0;
	unsigned int status;
	int data_rx_bucket;
	int size = 0, code = 0;
	struct tx_stn_handler *handler;
	int tx_stid, rcv_mask;

	data_rx_bucket = netlogic_thr_id() + 4;

	msg_body.msg0 = 0; // Keep compiler happy
	while (1) {

		msgrng_access_enable(mflags);
		if ((rcv_mask = (~msgrng_read_status() >> 28) & 0xf))
			bucket = get_adjusted_bucket_index(rcv_mask);
		else {
			msgrng_access_disable(mflags);
			break;
		}

		if (nlm_on_chip_napi) {
			status = message_receive(bucket, &size, &code, &stid, msg);
		}
		else {
			status = message_receive_fast_1(bucket, size, code, stid, msg_body.msg0);
		}

		msgrng_access_disable(mflags);

		if (status) {
			continue;
		}

		if (nlm_on_chip_napi) {
			/* this block is a quick check for messages arriving from non GMAC/XGMAC stations */
			tx_stid = rxstn_to_txstn_ptr[stid];
			if (tx_stns[tx_stid].handler.action != nlm_xlr_mac_msgring_handler) {
				handler = &tx_stns[tx_stid].handler;
				if (handler->action) {
					(handler->action)(bucket, size, code, stid, msg, handler->dev_id);
				}
				continue;
			}
		}

		length = (msg->msg0 >> 40) & 0x3fff;

		if (length) {
			printk("%s: message with non-zero length from buckets 4-7\n", __FUNCTION__);
			continue;
		}

		nlm_xlr_free_skb(msg);
	} /* closing while (1) */
}
EXPORT_SYMBOL(xlr_napi_poll_upper);

/* 
 * NAPI poll function on lower four buckets 
*/
int
napi_poll_lower(struct net_device *dummy_dev, int budget)
{
	struct msgrng_msg msg_body, *msg = &msg_body;
	int stid = 0, length;
	int port;
	struct sk_buff *skb;
	int received = 0;
	unsigned int data_rx_bucket; 
	unsigned long addr;
	unsigned long mflags = 0;
	unsigned int status;
	int size = 0, code = 0;
	int cpu = hard_smp_processor_id();
	struct driver_data *priv;
	unsigned int rxStatus;
	struct tx_stn_handler *handler;
	int tx_stid;
#ifdef CONFIG_NLMCOMMON_PTP_SUPPORT
	unsigned char *prepad = NULL;
#endif /* CONFIG_NLMCOMMON_PTP_SUPPORT */

	data_rx_bucket = netlogic_thr_id();
	msg_body.msg0 = 0;

	while (1) {
		msgrng_access_enable(mflags);

		if  ((msgrng_read_status() >> (data_rx_bucket + 24)) & 0x1) {
			msgrng_access_disable(mflags);
			break;
		}

		if (nlm_on_chip_napi) {
			status = message_receive(data_rx_bucket, &size, &code, &stid, msg);
		}
		else {
			status = message_receive_fast_1(data_rx_bucket, size, code, stid, msg_body.msg0);
		}

		msgrng_access_disable(mflags);
   
		if (status) {
			continue;
		}

		if (nlm_on_chip_napi) {
			/* quick check for messages arriving from stations different from GMAC/XGMAC */
			tx_stid = rxstn_to_txstn_ptr[stid];
			if (tx_stns[tx_stid].handler.action != nlm_xlr_mac_msgring_handler) {
				handler = &tx_stns[tx_stid].handler;
				if (handler->action) {
					(handler->action)(data_rx_bucket, size, code, stid, msg, handler->dev_id);
				}
				continue;
			}
		}

		length = (msg->msg0 >> 40) & 0x3fff;
		if (length == 0) {
			printk("%s: message from data buckets with zero length...\n", __FUNCTION__);
			continue;
		}

		/* we got a rx buffer with data from the MAC */
		addr = (unsigned long) bus_to_virt(msg->msg0 & 0xffffffffe0ULL);
		length = length  - BYTE_OFFSET -MAC_CRC_LEN - MAC_PREPAD;
		port = msg->msg0 & 0x0f;
		skb = mac_get_skb_back_ptr(addr);
    
		prefetch_local(skb->data);
	    
		if (is_xls()) {
			if (stid == MSGRNG_STNID_GMAC0)
				skb->dev = dev_mac_type[TYPE_GMAC][port];
			else if (stid == MSGRNG_STNID_GMAC1)
				skb->dev = dev_mac_type[TYPE_GMAC][4 + port];
			else {
				printk("[%s]: desc (0x%lx) for unknown station %d? dropping\n",
									__FUNCTION__, addr, stid);
				continue;
			}
		}
		else {
			if (stid == MSGRNG_STNID_XGS0FR)
				skb->dev = dev_mac_type[TYPE_XGMAC][0];
			else if (stid == MSGRNG_STNID_XGS1FR)
				skb->dev = dev_mac_type[TYPE_XGMAC][1];
			else
				skb->dev = dev_mac_type[TYPE_GMAC][port];
		}

		priv = netdev_priv(skb->dev);
   
		if (msg->msg0 & (0x40ULL << 56))
		{
			rxStatus = (msg->msg0 >> 56 ) & 0x7f;
			dbg_msg("Rx err 0x%x\n",rxStatus);
			mac_stats_add(priv->stats.rx_errors,1);
			if (rxStatus & 0x02)
				mac_stats_add(priv->stats.rx_crc_errors,1);
			if (rxStatus & 0x01)
				mac_stats_add(priv->stats.rx_length_errors,1);

			if (!nlm_auto_buffer_mgmt) {
				mac_frin_replenish_one_msg(skb->dev);
			}
			dev_kfree_skb(skb);

			continue;
		}

#ifdef PA10401_1_GMAC_PKT_DISCARD
		if ((!is_xls()) && (!(skb->dev->flags & IFF_PROMISC))) {
			if (!(msg->msg0 & (0x20ULL << 56))) {
				if ((*(uint64_t *)(skb->data + MAC_PREPAD + BYTE_OFFSET)>>16) !=
							   ((*(uint64_t *)skb->dev->dev_addr)>>16))
				{
					if (!nlm_auto_buffer_mgmt) {
						mac_frin_replenish_one_msg(skb->dev);
					}
					dev_kfree_skb(skb);
					continue;
				}
			}
		}
#endif

       	skb_reserve(skb, MAC_PREPAD+BYTE_OFFSET );
        
		skb_put(skb, length);
		skb->protocol = eth_type_trans(skb, skb->dev);
// 1588
    #ifdef CONFIG_NLMCOMMON_PTP_SUPPORT
        prepad =(unsigned char *) addr; 
        if(p_ptp_set_ts)
            p_ptp_set_ts(*((unsigned int *) prepad), 
                         *((unsigned int *) prepad + 1), 
                         &skb->tstamp, 1); 
    #endif
        
		mac_stats_add(priv->stats.rx_packets, 1);
		mac_stats_add(priv->stats.rx_bytes, skb->len);
		mac_stats_add(priv->cpu_stats[cpu].rx_packets, 1);

		xlr_inc_counter(NETIF_RX);
		xlr_set_counter(NETIF_RX_CYCLES, (read_c0_count() - msgrng_msg_cycles));

		if (!nlm_auto_buffer_mgmt) {
			mac_frin_replenish_one_msg(skb->dev);
		}

		skb->dev->last_rx = jiffies;
		netif_receive_skb(skb);

		__get_cpu_var(xlr_napi_rx_count)++; 

		/* If number of received packets is exceeding poll weight we exit */
		if (++received >= budget) {
			break;
		}
	} /* end of while loop */

	return received;
}



/*
 *  Version of transmit used in conjunction with NAPI mode
*/
static int
nlm_xlr_napi_mac_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	unsigned long mflags = 0;
	int txq = hard_smp_processor_id();
	int count = 0, ret;


	xlr_inc_counter(NETIF_STACK_TX);

	do {
		msgrng_access_enable(mflags);

		if (upper_buckets_nonempty()) {
			msgrng_access_disable(mflags);
			xlr_napi_poll_upper(dev, 0);
			msgrng_access_enable(mflags);
		}

		ret = mac_xmit(skb, dev, priv, txq);
		msgrng_access_disable(mflags);

		if (ret == MAC_TX_PASS) {
			mac_stats_add(priv->cpu_stats[txq].tx_packets, 1);
			return ret;
		}

		count++;

		if(count < 16)
			continue;

		ret = xlr_netif_queue_tx(dev, skb, txq);
		break;

	} while (1);
	
	if (ret == MAC_TX_FAIL) {
		/* FULL */
		dbg_msg("Msg Ring Full. Stopping upper layer Q\n");
		port_inc_counter(priv->instance, PORT_STOPQ);
	}

	return ret;
}

#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */


/*
 * Version of transmit used in regular interrupt-driven mode
*/
static int
nlm_xlr_mac_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int ret = -ENOSPC;
	unsigned long mflags = 0;
	int txq = hard_smp_processor_id();
	int count = 0;


	xlr_inc_counter(NETIF_STACK_TX);

	do {
		msgrng_access_enable(mflags);
		
		if (priv->frstid_rsvd == 1 && upper_buckets_nonempty()) {

			irq_enter();
			msgring_process_rx_msgs(MSGRING_PROCESS_FROUT_START_BUCKET,
						MSGRING_PROCESS_FROUT_END_BUCKET,
						MSGRING_PROCESS_FROUT_POP_BUCKET_MASK);
			irq_exit();
		}
		ret = mac_xmit(skb, dev, priv, txq);
		msgrng_access_disable(mflags);

		if (ret == MAC_TX_PASS) {
			mac_stats_add(priv->cpu_stats[txq].tx_packets, 1);
			break;
		}

		count++;

		if(count < 16)
			continue;

		ret = xlr_netif_queue_tx(dev, skb, txq);
		break;
	} while (1);

	return ret;
}


/* If allocation fails in the replenish tasklet, this function will replenish
   the RX buffers from the workqueue context. Replenish tasklet is disabled
   until this function is done with replenishment.
   */
static void mac_frin_replenish_wq(struct work_struct *args /* ignored */)
{
	int cpu = hard_smp_processor_id();
	int done = 0;
	int i = 0;

	xlr_inc_counter(REPLENISH_ENTER);
	//xlr_set_counter(REPLENISH_ENTER_COUNT, atomic_read(frin_to_be_sent));
	xlr_set_counter(REPLENISH_CPU, hard_smp_processor_id());

	for (;;) {

		done = 0;

		for (i = 0; i < NETLOGIC_MAX_MACS; i++) {
			int offset = 0;
			unsigned long msgrng_flags;
			struct sk_buff *skb = 0;
			__u32 cycles;
			struct net_device *dev;
			struct driver_data *priv;
			atomic_t *frin_to_be_sent;

			dev = dev_mac[i];
			if (dev == 0)
				goto skip;

			priv = netdev_priv(dev);
			frin_to_be_sent = &priv->frin_to_be_sent[cpu];

			if(!(MSGRNG_OWN(priv->cfg_flag)))
				goto skip;

			if (atomic_read(frin_to_be_sent) < 0) {
				panic
					("BUG?: [%s]: gmac_%d illegal value for frin_to_be_sent=%d\n",
					 __FUNCTION__, i,
					 atomic_read(frin_to_be_sent));
			}

			if (!atomic_read(frin_to_be_sent))
				goto skip;

			cycles = read_c0_count();
			{
				skb = __dev_alloc_skb(NLM_RX_BUF_SIZE,
					GFP_ATOMIC | __GFP_REPEAT |
				       	__GFP_NOWARN);

				if (!skb) {
					skb =
					__dev_alloc_skb(NLM_RX_BUF_SIZE,
								GFP_KERNEL);
					if (!skb)
						panic
						("[%s]:Unable to allocate skb!\n",
							 __FUNCTION__);
				}
			}
			xlr_inc_counter(REPLENISH_FRIN);

			/* align the data to the next cache line */
			offset = (((unsigned long)skb->data + SMP_CACHE_BYTES) &
				  ~(SMP_CACHE_BYTES - 1));
			skb_reserve(skb, (offset - (unsigned long)skb->data));

			//skb->dev = dev;

			msgrng_access_enable(msgrng_flags);
			mac_put_skb_back_ptr(skb);

			if (xlr_mac_send_fr(priv, virt_to_bus(skb->data), skb->len)) {
				dev_kfree_skb(skb);
				printk("[%s]: rx free message_send failed!\n",
				       __FUNCTION__);
				break;
			}
			msgrng_access_disable(msgrng_flags);

			xlr_set_counter(REPLENISH_CYCLES,
					 (read_c0_count() - cycles));

			atomic_dec(frin_to_be_sent);

			continue;
		skip:
			done++;
		}
		if (done == NETLOGIC_MAX_MACS)
			break;
	}
	tasklet_enable(&mac_frin_replenish_task[cpu]);
}


static void mac_frin_replenish(unsigned long arg /* ignored */ )
{
	int cpu = hard_smp_processor_id();
	int done = 0;
	int i = 0;

	xlr_inc_counter(REPLENISH_ENTER);
	//xlr_set_counter(REPLENISH_ENTER_COUNT, atomic_read(frin_to_be_sent));
	xlr_set_counter(REPLENISH_CPU, hard_smp_processor_id());

	for (;;) {

		done = 0;

		for (i = 0; i < NETLOGIC_MAX_MACS; i++) {
			int offset = 0;
			unsigned long msgrng_flags;
			struct sk_buff *skb = 0;
			__u32 cycles;
			struct net_device *dev;
			struct driver_data *priv;
			atomic_t *frin_to_be_sent;

			dev = dev_mac[i];
			if (dev == 0)
				goto skip;

			priv = netdev_priv(dev);
			frin_to_be_sent = &priv->frin_to_be_sent[cpu];

			if(!(MSGRNG_OWN(priv->cfg_flag)))
				goto skip;

			if (atomic_read(frin_to_be_sent) < 0) {
				panic
					("BUG?: [%s]: gmac_%d illegal value for frin_to_be_sent=%d\n",
					 __FUNCTION__, i,
					 atomic_read(frin_to_be_sent));
			}

			if (!atomic_read(frin_to_be_sent))
				goto skip;

			cycles = read_c0_count();
			{
				skb =
					__dev_alloc_skb(NLM_RX_BUF_SIZE,
							GFP_ATOMIC | __GFP_REPEAT |
							__GFP_NOWARN);
				if (!skb) {
					tasklet_disable_nosync
					(&mac_frin_replenish_task[cpu]);
					schedule_work
					(&mac_frin_replenish_work[cpu]);
					return;
				}
			}
			xlr_inc_counter(REPLENISH_FRIN);

			/* align the data to the next cache line */
			offset = (((unsigned long)skb->data + SMP_CACHE_BYTES) &
				  ~(SMP_CACHE_BYTES - 1));
			skb_reserve(skb, (offset - (unsigned long)skb->data));

			//skb->dev = dev;

			msgrng_access_enable(msgrng_flags);
			mac_put_skb_back_ptr(skb);

			if (xlr_mac_send_fr(priv, virt_to_bus(skb->data), skb->len)) {
				dev_kfree_skb(skb);
				printk("[%s]: rx free message_send failed!\n",
				       __FUNCTION__);
				break;
			}
			msgrng_access_disable(msgrng_flags);

			xlr_set_counter(REPLENISH_CYCLES,
					 (read_c0_count() - cycles));

			atomic_dec(frin_to_be_sent);

			continue;
		skip:
			done++;
		}
		if (done == NETLOGIC_MAX_MACS)
			break;
	}
}

/*
 * Send a packet back to the station
 */
void nlm_xlr_drop_message_unowned(int fbid, uint64_t physaddr, int cop_en)
{
	struct msgrng_msg msg;
	unsigned long msgrng_flags = 0;

	/*printk(" nlm_xlr_drop_message_unowned fbid = %d physaddr=%llx\n",
			fbid, physaddr); */

	if(cop_en)
		msgrng_access_enable(msgrng_flags);

	msg.msg0 =
		((u64) CTRL_REG_FREE << 61) | ((u64) fbid << 52) | (u64) physaddr;
	msg.msg1 = msg.msg2 = msg.msg3 = 0;
	while (message_send(1, MSGRNG_CODE_MAC, fbid, &msg)) ;

	if(cop_en)
		msgrng_access_disable(msgrng_flags);
}




static inline void nlm_xlr_free_skb(struct msgrng_msg *msg)
{
	struct sk_buff *skb;
	struct driver_data *priv;
	int cpu = hard_smp_processor_id();
	#ifdef CONFIG_64BIT
	unsigned long tmp;
	tmp = (unsigned long)(msg->msg0 & 0xffffffffffULL);
	skb = (struct sk_buff *)phys_to_virt(tmp);
	#else
	skb = (struct sk_buff *)(unsigned long)msg->msg0;
	#endif
	/* Tx Complete */
	xlr_inc_counter(NETIF_TX_COMPLETE);

	dbg_msg("skb = %p\n", skb);
	/* release the skb and update statistics outside the spinlock */
	priv = netdev_priv(skb->dev);
	mac_stats_add(priv->stats.tx_packets, 1);
	mac_stats_add(priv->stats.tx_bytes, skb->len);
	mac_stats_add(priv->cpu_stats[cpu].txc_packets, 1);
#ifdef CONFIG_NLMCOMMON_PTP_SUPPORT
        if(skb->sk) {
        	if (sock_flag(skb->sk, SOCK_TIMESTAMP)) {
//               dump_all_interface(0x75); 
              if(p_ptp_set_ts) {
                   
                    p_ptp_set_ts(netlogic_read_reg(priv->mmio, 0x76), 
                                   netlogic_read_reg(priv->mmio, 0x75), NULL, 1); 
                }
            }
        }
#endif


	port_inc_counter(priv->instance, PORT_TX_COMPLETE);
	xlr_netif_queue_tx_complete(skb->dev);

	xlr_set_counter(NETIF_TX_COMPLETE_CYCLES,
		(read_c0_count() - msgrng_msg_cycles));
	dev_kfree_skb_any(skb);
}


/*
 * Send a packet back to ipsec rmios
 */
static void ipsec_drop_packet(IPSEC_PACKET * pbuf)
{
	int stid=0;
	u32 addr;
	struct msgrng_msg msg;
        if (is_xls()) {
            if (pbuf->src_id == MSGRNG_STNID_GMAC0)
		stid = MSGRNG_STNID_GMAC0_FR;
            else if (pbuf->src_id == MSGRNG_STNID_GMAC1)
		stid = MSGRNG_STNID_GMAC1_FR;
            else {
                printk("[%s]: rx packet (0x%p) for unknown station %d? dropping packet\n",
                       __FUNCTION__, pbuf, stid);
                return;
	    }
        }
        else {
	    if (pbuf->src_id == MSGRNG_STNID_XGS0FR)
		stid = MSGRNG_STNID_XMAC0RFR;
	    else if (pbuf->src_id == MSGRNG_STNID_XGS1FR)
		stid = MSGRNG_STNID_XMAC1RFR;
	    else
		stid = MSGRNG_STNID_GMACRFR_0;
        }
	addr = virt_to_phys(pbuf->packet_data + SKBUF_HEAD);
	msg.msg0 =
		((u64) CTRL_REG_FREE << 61) | ((u64) stid << 52) | (u64) addr;
	msg.msg1 = msg.msg2 = msg.msg3 = 0;
	while (message_send(1, MSGRNG_CODE_MAC, stid, &msg)) ;
}

/*
 * Receive a packet from rmios ipsec. This function is called by the message
 * ring driver when the message source is a CPU and the message code indicates
 * a packet from rmios. The message ring driver can also receive a fifo message
 * from a CPU sending an event or response to a user space process.
 */
void nlm_xlr_rmios_msgring_handler(int bucket, int size, int code,
				    int stid, struct msgrng_msg *msg,
				    void *data /* ignored */ )
{
	unsigned long addr;
	__u32 length;
	int port;
	struct sk_buff *skb;
	struct driver_data *priv;
	IPSEC_PACKET *ipsec_packet;
	/*
	 * Find the ipsec packet
	 */
	addr = (unsigned long)bus_to_virt(msg->msg0 & 0xffffffffe0ULL);
	ipsec_packet = (IPSEC_PACKET *) (addr - SKBUF_HEAD -
					 offsetof(IPSEC_PACKET, packet_data));
	/*
	 * Do nothing during the boot.
	 */
	if (system_state != SYSTEM_RUNNING) {
		ipsec_drop_packet(ipsec_packet);
		return;
	}
	/*
	 * Allocate an skbuff, initialize it, and copy the data to it.
	 */
	length =
		((msg->msg0 >> 40) & 0x3fff) - BYTE_OFFSET - MAC_CRC_LEN -MAC_PREPAD;
	skb = __dev_alloc_skb(NLM_RX_BUF_SIZE, GFP_ATOMIC);
	if (!skb) {
		printk("[%s] - no skbuff\n", __FUNCTION__);
		ipsec_drop_packet(ipsec_packet);
		return;
	}
	port = code >> 4;
	skb->dev = dev_mac_type[TYPE_GMAC][port];
	skb_put(skb, length);
	memcpy(skb->data, (char *)addr + 2, length);
	ipsec_drop_packet(ipsec_packet);
	skb->protocol = eth_type_trans(skb, skb->dev);
	/*
	 * Increment the driver stats counters.
	 */
	priv = netdev_priv(skb->dev);
	mac_stats_add(priv->stats.rx_packets, 1);
	mac_stats_add(priv->stats.rx_bytes, skb->len);
	/*
	 * Queue the packet to the upper layer.
	 */
	skb->dev->last_rx = jiffies;
	netif_rx(skb);
}



/* This function is called from an interrupt handler */
void nlm_xlr_mac_msgring_handler(int bucket, int size, int code,
				  int stid, struct msgrng_msg *msg,
				  void *data /* ignored */ )
{
#ifndef CONFIG_NLMCOMMON_HW_BUFFER_MGMT
/* 
 * Special helper macro to handle Rx errors within this interrupt handler
 * macro is used locally in this function only
 *
 * NB: the alternative case when HW buffer management is on is handled by a 
 * larger function defined above
*/
#define discard_rx_frame(dev, skb, cpu) \
do { \
	if (atomic_inc_return(&priv->frin_to_be_sent[cpu]) > \
				MAC_FRIN_TO_BE_SENT_THRESHOLD) { \
		tasklet_schedule(&mac_frin_replenish_task[cpu]); \
	} \
	dev_kfree_skb_irq(skb); \
} while (0)

#endif /* !CONFIG_NLMCOMMON_HW_BUFFER_MGMT */

	unsigned long addr = 0;
	__u32 length = 0;
	int ctrl = 0, port = 0;
	struct sk_buff *skb = 0;
	int cpu = hard_smp_processor_id();

#ifdef CONFIG_NLMCOMMON_PTP_SUPPORT
	unsigned char *prepad = NULL;
#endif /* CONFIG_NLMCOMMON_PTP_SUPPORT */

	dbg_msg("mac: bucket=%d, size=%d, code=%d, stid=%d, msg0=%llx msg1=%llx\n",
		 bucket, size, code, stid, msg->msg0, msg->msg1);

	addr = (unsigned long)bus_to_virt(msg->msg0 & 0xffffffffe0ULL);
	length = (msg->msg0 >> 40) & 0x3fff;
	if (length == 0) {
		ctrl = CTRL_REG_FREE;
		port = (msg->msg0 >> 54) & 0x0f;
	}
	else {
		ctrl = CTRL_SNGL;
		length = length - BYTE_OFFSET - MAC_CRC_LEN - MAC_PREPAD;
		port = msg->msg0 & 0x0f;
	}

	dbg_msg("msg0 = %llx, msg1 = %llx, stid = %d, port = %d, addr=%lx, length=%d, ctrl=%d\n", 
		msg->msg0, msg->msg1, stid, port, addr, length, ctrl);

	if (ctrl == CTRL_REG_FREE) {
		/* free the message , freeback should be the 
			packets send by linux */
		nlm_xlr_free_skb(msg);

	} else if (ctrl == CTRL_SNGL || ctrl == CTRL_START) {
		/* Rx Packet */

		struct driver_data *priv = 0;
                unsigned int rxStatus=0;

		dbg_msg("Received packet, port = %d\n", port);

		skb = mac_get_skb_back_ptr(addr);
		if (!skb) {
			printk("[%s]: rx desc (0x%lx) for unknown skb? dropping packet\n", 
                                                                                __FUNCTION__, addr);
			return;
		}
		
		if (is_xls()) {
			if (stid == MSGRNG_STNID_GMAC0)
				skb->dev = dev_mac_type[TYPE_GMAC][port];
			else if (stid == MSGRNG_STNID_GMAC1)
				skb->dev = dev_mac_type[TYPE_GMAC][4 + port];
			else {
				printk("[%s]: rx desc (0x%lx) for unknown station %d? dropping packet\n",
					__FUNCTION__, addr, stid);
				return;
			}
		}
		else {
			if (stid == MSGRNG_STNID_XGS0FR)
				skb->dev = dev_mac_type[TYPE_XGMAC][0];
			else if (stid == MSGRNG_STNID_XGS1FR)
				skb->dev = dev_mac_type[TYPE_XGMAC][1];
			else
				skb->dev = dev_mac_type[TYPE_GMAC][port];
		}
		
		priv = netdev_priv(skb->dev);
                
                rxStatus = (msg->msg0 >> 56 ) & 0x7f;
                if (rxStatus & 0x40)
                {
                  dbg_msg("Rx err 0x%x\n",rxStatus);
                  mac_stats_add(priv->stats.rx_errors,1);
                  if (rxStatus & 0x02)
                    mac_stats_add(priv->stats.rx_crc_errors,1);
                  if (rxStatus & 0x01)
                    mac_stats_add(priv->stats.rx_length_errors,1);

                  discard_rx_frame(skb->dev, skb, cpu);
                  return;
                }

#ifdef PA10401_1_GMAC_PKT_DISCARD
               if ((!is_xls()) && (!(skb->dev->flags & IFF_PROMISC)))
                {
                  if (!(rxStatus & 0x20))
                  {
                    if ((*(uint64_t *)(skb->data+MAC_PREPAD + BYTE_OFFSET)>>16) !=
                     ((*(uint64_t *)skb->dev->dev_addr)>>16))
                     {
		      discard_rx_frame(skb->dev, skb, cpu);
                      return;
                     }
                  }
               }
#endif

		/* if num frins to be sent exceeds threshold, wake up the helper thread */
		if (!nlm_auto_buffer_mgmt && 
			     atomic_inc_return(&priv->frin_to_be_sent[cpu]) > MAC_FRIN_TO_BE_SENT_THRESHOLD) {
			tasklet_schedule(&mac_frin_replenish_task[cpu]);
		}

#ifdef DUMP_PACKETS
		dump_packet(skb);
#endif /* DUMP_PACKETS */

		/* compensate for the prepend data, byte offset */
		skb_reserve(skb, MAC_PREPAD + BYTE_OFFSET);

		skb_put(skb, length);
		skb->protocol = eth_type_trans(skb, skb->dev);
                
		dbg_msg("gmac_%d: rx packet: addr = %lx, length = %x, protocol=%d\n",
			 priv->instance, addr, length, skb->protocol);

		mac_stats_add(priv->stats.rx_packets, 1);
		mac_stats_add(priv->stats.rx_bytes, skb->len);
		mac_stats_add(priv->cpu_stats[cpu].rx_packets, 1);

    #ifdef CONFIG_NLMCOMMON_PTP_SUPPORT
        prepad =(unsigned char *) addr; 
        if(p_ptp_set_ts)
            p_ptp_set_ts(*((unsigned int *) prepad), 
                         *((unsigned int *) prepad + 1), 
                         &skb->tstamp, 1); 
    #endif
 
		xlr_inc_counter(NETIF_RX);
		xlr_set_counter(NETIF_RX_CYCLES, (read_c0_count() - msgrng_msg_cycles));

#ifdef CONFIG_NLMCOMMON_IP_QUEUE_AFFINITY
		/* 
                 * We pass bucket number in the last field of skb->cb[] structure 
                 * it might be later picked up by multiprocess ip_queue
                */
                skb->cb[sizeof(skb->cb) - 1] = bucket;
#endif /* CONFIG_NLMCOMMON_IP_QUEUE_AFFINITY */


#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
		skb_transfer(bucket, skb);
#else
		skb->dev->last_rx = jiffies;
		netif_rx(skb);
#endif /* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */

	} else {
		printk("[%s]: unrecognized ctrl=%d!\n", __FUNCTION__, ctrl);
	}
}

/* message ring handler where mac is owned by apps not linux */
void nlm_xlr_station_unowned_msgring_handler(int bucket, int size, int code,
				    int stid, struct msgrng_msg *msg,
				    void *data /* ignored */ )
{
	unsigned long addr;
	__u32 length;
	int port;
	struct sk_buff *skb = NULL;
	struct driver_data *priv;
	int fbstid = 0x0;

	addr = (unsigned long)bus_to_virt(msg->msg0 & 0xffffffffe0ULL);
	port = ((msg->msg0)  & 0x0f);
	length = ((msg->msg0 >> 40) & 0x3fff);

	/* printk("[%s] : port=%d length=%d\n", __FUNCTION__, port, length); */

	/* free back should be the packets send by linux */
	if(length == 0x0)  {
		/* free the message , freeback should be the 
			packets send by linux */
		nlm_xlr_free_skb(msg);
		return;
	}


	/*
	 * Allocate an skbuff, initialize it, and copy the data to it.
	 */
	skb = __dev_alloc_skb(NLM_RX_BUF_SIZE, GFP_ATOMIC);
	if (!skb) {
		printk("[%s] - no skbuff\n", __FUNCTION__);
		goto err_exit;
	}

	if (is_xls()) {
		if (stid == MSGRNG_STNID_GMAC0) {
			skb->dev = dev_mac_type[TYPE_GMAC][port];
			fbstid = MSGRNG_STNID_GMAC0_FR;
		} else if (stid == MSGRNG_STNID_GMAC1) {
			skb->dev = dev_mac_type[TYPE_GMAC][4 + port];
			fbstid = MSGRNG_STNID_GMAC1_FR;
		} else {
			printk("[%s]: rx desc (0x%lx) for unknown station %d? dropping packet\n",
				__FUNCTION__, addr, stid);
			goto err_exit;
		}
	} else {
		if (stid == MSGRNG_STNID_XGS0FR) {
			skb->dev = dev_mac_type[TYPE_XGMAC][0];
			fbstid = MSGRNG_STNID_XMAC0RFR;
		} else if (stid == MSGRNG_STNID_XGS1FR) {
			skb->dev = dev_mac_type[TYPE_XGMAC][1];
			fbstid = MSGRNG_STNID_XMAC1RFR;
		} else {
			skb->dev = dev_mac_type[TYPE_GMAC][port];
			fbstid = MSGRNG_STNID_GMACRFR_0;
		}
	}

#if 0
	printk("nlm_xlr_station_unowned_msgring_handler ingress port=%d stid=%d\n", port, stid);
#endif
	if(skb->dev == 0) {
		printk("[%s] - no dev\n", __FUNCTION__);
		goto err_exit;
	}
	
	length = length - (BYTE_OFFSET + MAC_CRC_LEN);
	skb_put(skb, length);
	memcpy(skb->data, (char *)addr + 2, length);
/*	
	if(rmik_queue_pkt_mem(fbstid, msg->msg0 & 0xffffffffe0ULL) < 0)
		nlm_xlr_drop_message_unowned(fbstid, msg->msg0 & 0xffffffffe0ULL, 1);
*/
#if 0
		{
			int i = 0;
			printk("Rx Packet: length=%d\n", length);
			for (i = 0; i < 64; i++) {
				printk("%02x ", skb->data[i]);
				if (i && (i % 16) == 0)
					printk("\n");
			}
			printk("\n");
		}
#endif

	skb->protocol = eth_type_trans(skb, skb->dev);
	/*
	 * Increment the driver stats counters.
	 */
	priv = netdev_priv(skb->dev);
	mac_stats_add(priv->stats.rx_packets, 1);
	mac_stats_add(priv->stats.rx_bytes, skb->len);
	/*
	 * Queue the packet to the upper layer.
	 */
	skb->dev->last_rx = jiffies;
	netif_rx(skb);
	return;

	err_exit:
/*
		nlm_xlr_drop_message_unowned(fbstid, msg->msg0 & 0xffffffffe0ULL, 1);
*/
		if(skb)
			kfree_skb(skb);
		return;

}


#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
extern void core_send_ipi(int cpu, unsigned int action);

static void
skb_transfer(int bucket, struct sk_buff *skb)
{
  u_long my_cpu_no, my_thread_no, my_core_no, target_cpu_no, target_thread_no;


  target_thread_no = bucket & 0x3;
  my_cpu_no = smp_processor_id();
  my_thread_no = netlogic_thr_id();
  my_core_no = netlogic_cpu_id();
  target_cpu_no = cpu_number_map((my_core_no << 2) | target_thread_no);

  /*
   * Version with NETRX IPI aggregation
  */
  if (target_thread_no != my_thread_no && cpu_isset(target_cpu_no, cpu_online_map))
  {
    unsigned long flags;
    struct sk_buff_head *ptqueue = &cpu_skb_tqueue[target_cpu_no];

    spin_lock_irqsave(&ptqueue->lock, flags);
    if (ptqueue->qlen)
    {
       __skb_queue_tail(ptqueue, skb);
    }
    else
    {
       __skb_queue_tail(ptqueue, skb);
       core_send_ipi(target_cpu_no, SMP_NETRX_IPI);
    }
    spin_unlock_irqrestore(&ptqueue->lock, flags);

    skb_transfer_stat[my_cpu_no][target_cpu_no]++;
  }
  else
  {
    skb_transfer_stat[my_cpu_no][my_cpu_no]++;

    skb_queue_tail(&cpu_skb_tqueue[my_cpu_no], skb);
    skb_transfer_finish();
  }

}


/* second part of SKB transfer logic, called from IRQ_IPI_NETRX handler */
void
skb_transfer_finish(void)
{
  struct sk_buff *skb;
  u_long cpu = smp_processor_id();

  while ((skb = skb_dequeue(&cpu_skb_tqueue[cpu])) != NULL)
  {
	  skb->dev->last_rx = jiffies;
	  netif_rx(skb);
  }
}

#endif /* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */


/**********************************************************************
 **********************************************************************/
static irqreturn_t nlm_xlr_mac_int_handler(int irq, void *dev_id)
{
    struct net_device *dev = (struct net_device *)dev_id;
    struct driver_data *priv = netdev_priv(dev);
    nlm_reg_t *mmio = priv->mmio;
    __u32 intreg = netlogic_read_reg(mmio, R_INTREG);
    int cpu = hard_smp_processor_id();

    mac_stats_add(priv->cpu_stats[cpu].interrupts, 1);

    if (intreg & (1 << O_INTREG__MDInt)) {
        __u32 phy_int_status = 0;
        int i=0;

        for(i=0; i<NETLOGIC_MAX_MACS; i++) {
            struct net_device *phy_dev = 0;
            struct driver_data *phy_priv = 0;
            uint32_t config_val =0;

            phy_dev = dev_mac[i];
			if(phy_dev == 0)
				continue;

            phy_priv = netdev_priv(phy_dev);

            if (phy_priv->type == TYPE_XGMAC) continue;
            if (phy_priv->phy.mode == PHY_MODE_XAUI) continue;

           		phy_int_status = nlm_xlr_mac_mii_read(phy_priv->phy.mii_addr, phy_priv->phy.addr, 26);
			
            /*printk(KERN_DEBUG"[%s]: Received MDIO interrupt from mac_%d (type=%d), "
              "phy_int_status = 0x%08x reconfiguring gmac speed \n",
              __FUNCTION__, phy_priv->instance, phy_priv->type,
              phy_int_status);*/
            if (!phy_priv->instance && phy_priv->phy.serdes_addr != 0x0 
						&& phy_priv->phy.mode & PHY_MODE_SELECTABLE) {
				int phyaddr;
				unsigned long mii_addr, temp;
				int mode = PHY_MODE_RGMII;
				if(phy_priv->phy.mode & PHY_MODE_RGMII)
					mode = PHY_MODE_SGMII;

				phyaddr = xlr_get_phy_info(phy_priv->instance, mode, &mii_addr, &temp, &temp);
		        phy_int_status = nlm_xlr_mac_mii_read((nlm_reg_t *)mii_addr, phyaddr, 26);
				/*ack rgmii/sgmii mac*/
    			netlogic_write_reg((nlm_reg_t *)mii_addr, R_INTREG, 
										0xffffffff);

                /*	printk(KERN_DEBUG"[%s]: Received MDIO interrupt from mac_%d (type=%d), "
                    "phy_int_status = 0x%08x reconfiguring gmac speed \n",
                    __FUNCTION__, phy_priv->instance, phy_priv->type,
                    phy_int_status);*/
            } 
            config_val = netlogic_read_reg(phy_priv->mmio, R_MAC_CONFIG_1);
            netlogic_write_reg(phy_priv->mmio,R_MAC_CONFIG_1,(config_val & ~(0x35)));

            if(phy_priv->phy.serdes_addr) {
                serdes_autoconfig(phy_priv);
            }
            nlm_xlr_gmac_config_speed(phy_priv);
            netlogic_write_reg(phy_priv->mmio,R_MAC_CONFIG_1,config_val);
        }
    } else {
        printk("[%s]: mac type = %d, instance %d error "
                "interrupt: INTREG = 0x%08x\n", 
                __FUNCTION__, priv->type, priv->instance, intreg);
    }

    /* clear all interrupts and hope to make progress */
    netlogic_write_reg(mmio, R_INTREG, 0xffffffff);
    //xlr_set_counter(NETIF_INT_REG, intreg);

    /* on A0 and B0, xgmac interrupts are routed only to xgs_1 irq */
    if ( (xlr_revision_b0()) && (priv->type == TYPE_XGMAC) ) {
        struct net_device *xgs0_dev = dev_mac_type[TYPE_XGMAC][0];
        struct driver_data *xgs0_priv = netdev_priv(xgs0_dev);
        nlm_reg_t *xgs0_mmio = xgs0_priv->mmio;			
        __u32 xgs0_intreg = netlogic_read_reg(xgs0_mmio, R_INTREG);

        if (xgs0_intreg) {
            printk("[%s]: mac type = %d, instance %d error "
                    "interrupt: INTREG = 0x%08x\n", 
                    __FUNCTION__, xgs0_priv->type, xgs0_priv->instance, xgs0_intreg);

            netlogic_write_reg(xgs0_mmio, R_INTREG, 0xffffffff);
        }
    }

    return IRQ_HANDLED;
}

/**********************************************************************
 **********************************************************************/
static int nlm_xlr_mac_open(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);

	dbg_msg("IN\n");

	if (nlm_xlr_mac_fill_rxfr(dev)) {
		return -1;
	}

	spin_lock_bh(&priv->lock);
	if(PORT_INIT(priv->cfg_flag))
		xlr_mac_set_rx_mode(dev);


	if(PORT_INT_ATTACH(priv->cfg_flag)) {
		netlogic_write_reg(priv->mmio, R_INTMASK, 
				(1<<O_INTMASK__TxIllegal)       |
				(((priv->instance&0x3)==0)<<O_INTMASK__MDInt)|
				(1<<O_INTMASK__TxFetchError)    |
				(1<<O_INTMASK__P2PSpillEcc)     |
				(1<<O_INTMASK__TagFull)         |
				(1<<O_INTMASK__Underrun)        |
				(1<<O_INTMASK__Abort)
				);
	}

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
	if (nlm_msgring_napi) {
		xlr_napi_ready = 1; 
	}
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */

	if(PORT_INIT(priv->cfg_flag)) {
	/*
	 * Configure the speed, duplex, and flow control
	 */
	nlm_xlr_mac_set_speed(priv, priv->speed);
	nlm_xlr_mac_set_duplex(priv, priv->duplex, priv->flow_ctrl);
	nlm_xlr_mac_set_enable(priv, 1);
	}

	spin_unlock_bh(&priv->lock);
	netif_tx_start_all_queues(dev);

	/* Set the timer to check for link beat. */
	init_timer(&priv->link_timer);
	priv->link_timer.expires = jiffies + 2 * HZ / 100;
	priv->link_timer.data = (unsigned long)dev;
	priv->link_timer.function = &nlm_xlr_mac_timer;
	priv->phy_oldlinkstat = -1; /* set link state to undefined */
	add_timer(&priv->link_timer);

	return 0;
}

/**********************************************************************
 **********************************************************************/
static int nlm_xlr_mac_close(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	spin_lock_irq(&priv->lock);

	/* There may have left over skbs in the ring as well as in free in 
	 * they will be reused next time open is called 
	 */

	nlm_xlr_mac_set_enable(priv, 0);

	del_timer_sync(&priv->link_timer);
	netif_tx_stop_all_queues(dev);
	xlr_inc_counter(NETIF_STOP_Q);
	port_inc_counter(priv->instance, PORT_STOPQ);

	spin_unlock_irq(&priv->lock);

	return 0;
}

/**********************************************************************
 **********************************************************************/
static void nlm_xlr_mac_timer(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct driver_data *priv = netdev_priv(dev);
	int next_tick = HZ;
	int mii_status;

	spin_lock_irq(&priv->lock);

	if((priv->type == TYPE_GMAC) && (priv->phy.mode != PHY_MODE_XAUI))
	/* read flag "Link established" (0x04) of MII status register (1) */
	mii_status = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, 1) & 0x04;
	else
		mii_status = 1;

	if (mii_status != priv->phy_oldlinkstat) {
		priv->phy_oldlinkstat = mii_status;
		if (mii_status) {
			netif_carrier_on(dev);
			netif_tx_start_all_queues( dev);
		} else {
			netif_carrier_off(dev);
		}
	}

	spin_unlock_irq(&priv->lock);
	priv->link_timer.expires = jiffies + next_tick;
	add_timer(&priv->link_timer);
}

/**********************************************************************
 **********************************************************************/
static struct net_device_stats *nlm_xlr_mac_get_stats(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	xlr_get_mac_stats(dev, &priv->stats);

	/* XXX update other stats here */
	spin_unlock_irqrestore(&priv->lock, flags);

	return &priv->stats;
}

/**********************************************************************
 **********************************************************************/
static void nlm_xlr_mac_set_multicast_list(struct net_device *dev)
{
	/* 
	 * Clear out entire multicast table.  We do this by nuking
	 * the entire hash table and all the direct matches except
	 * the first one, which is used for our station address 
	 */

	/*
	 * Clear the filter to say we don't want any multicasts.
	 */

	if (dev->flags & IFF_ALLMULTI) {
		/* 
		 * Enable ALL multicasts.  Do this by inverting the 
		 * multicast enable bit. 
		 */
		return;
	}

	/* 
	 * Progam new multicast entries.  For now, only use the
	 * perfect filter.  In the future we'll need to use the
	 * hash filter if the perfect filter overflows
	 */
}



/**********************************************************************
 **********************************************************************/
static int
nlm_xlr_mac_do_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	int rc = 0;
	switch (cmd) {
	default:
		rc = -EOPNOTSUPP;
		break;
	}

	return rc;
}

/**********************************************************************
 **********************************************************************/
static void nlm_xlr_mac_tx_timeout(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);

	spin_lock_irq(&priv->lock);

	dev->trans_start = jiffies;
	mac_stats_add(priv->stats.tx_errors, 1);

	spin_unlock_irq(&priv->lock);

	netif_tx_wake_all_queues(dev);
	xlr_inc_counter(NETIF_START_Q);
	port_inc_counter(priv->instance, PORT_STARTQ);

	printk(KERN_WARNING "%s: Transmit timed out\n", dev->name);
}

/**********************************************************************
 **********************************************************************/
static int nlm_xlr_mac_change_mtu(struct net_device *dev, int new_mtu)
{
	struct driver_data *priv = netdev_priv(dev);
	unsigned long flags;

	if ((new_mtu > 1500) || (new_mtu < 64)) {
		return -EINVAL;
	}

	spin_lock_irqsave(&priv->lock, flags);

	dev->mtu = new_mtu;

	if (netif_running(dev)) {
		/* Disable MAC TX/RX */
		nlm_xlr_mac_set_enable(priv, 0);

		/* Flush RX FR IN */
		/* Flush TX IN */
		nlm_xlr_mac_set_enable(priv, 1);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

/**********************************************************************
 **********************************************************************/
static int nlm_xlr_mac_fill_rxfr(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	struct sk_buff *skb = 0;
	unsigned long msgrng_flags;
	int i;
	int ret = 0;

	dbg_msg("\n");
	if (!priv->init_frin_desc) return ret;
	priv->init_frin_desc = 0;	

	if(!(MSGRNG_OWN(priv->cfg_flag)))
		return ret; 
	
	for (i = 0; i < priv->num_desc; i++) {
		skb = nlm_xlr_alloc_skb();
		if (!skb) {
			ret = -ENOMEM;
			break;
		}

		skb->dev = dev;

#ifdef CONFIG_NLMCOMMON_HW_BUFFER_MGMT
		if (nlm_auto_buffer_mgmt) {
			skb_shinfo(skb)->nlm_flags = 1;
			skb_shinfo(skb)->nlm_owner = dev;
			skb_shinfo(skb)->nlm_refill = mac_frin_replenish_one_msg;
		}
#endif /* CONFIG_NLMCOMMON_HW_BUFFER_MGMT */

		/* Send the free Rx desc to the MAC */
		msgrng_access_enable(msgrng_flags);
		mac_put_skb_back_ptr(skb);
		if (xlr_mac_send_fr(priv, virt_to_bus(skb->data), skb->len)) {
			dev_kfree_skb(skb);
			printk
				("message_send failed!, unable to send free desc to mac\n");
			ret = -EIO;
			break;
		}
		msgrng_access_disable(msgrng_flags);
	}

	for (i = 0; i < MAC_FRIN_WORK_NUM; i++)
		atomic_set(&priv->frin_to_be_sent[i], 0);
	return ret;
}


/**********************************************************************
 **********************************************************************/
static __inline__ void *nlm_xlr_config_spill(nlm_reg_t * mmio,
					      int reg_start_0, int reg_start_1,
					      int reg_size, int size)
{
	__u32 spill_size = CACHELINE_ALIGNED_ADDR(size);
	void *spill = cacheline_aligned_kmalloc(spill_size, GFP_KERNEL);
	__u64 phys_addr = 0;

	if (!spill) {
		panic("Unable to allocate memory for spill area!\n");
	}
	phys_addr = virt_to_phys(spill);
	netlogic_write_reg(mmio, reg_start_0, (phys_addr >> 5) & 0xffffffff);
	netlogic_write_reg(mmio, reg_start_1, (phys_addr >> 37) & 0x07);
	netlogic_write_reg(mmio, reg_size, spill_size);

	return spill;
}

static void nlm_xlr_config_spill_area(struct driver_data *priv)
{
	int max_frin_spill    = 0;
	int max_frout_spill   = 0;
	int max_class_0_spill = 0;
	int max_class_1_spill = 0;
	int max_class_2_spill = 0;
	int max_class_3_spill = 0;

	if(!priv->num_desc || !priv->spill_init)
		return;

	if(!(MSGRNG_OWN(priv->cfg_flag)))
		return;

	max_frin_threshold = (priv->num_desc/NR_CPUS);
	if(max_frin_threshold)
			max_frin_threshold -= 1;


	/* 
	 * This is new approach to set up spill sizes. TCP stack termination in
	 * the NAPI mode requires spill area for FreeOut's of considerable size.
	 * We set frout spill here always to 15K descriptors which traslates into
	 * 15K * 8 bytes kernel memory alloc.
	*/
	max_frin_spill = priv->num_desc << 2;
	max_frout_spill = XLR_FROUT_JUMBO_SPILL; /* 15K  */
	max_class_0_spill = priv->num_desc;
	max_class_1_spill = priv->num_desc;
	max_class_2_spill = priv->num_desc;
	max_class_3_spill = priv->num_desc;

	priv->frin_spill =
		nlm_xlr_config_spill(priv->mmio,
				      R_REG_FRIN_SPILL_MEM_START_0,
				      R_REG_FRIN_SPILL_MEM_START_1,
				      R_REG_FRIN_SPILL_MEM_SIZE,
				      max_frin_spill *
				      sizeof(struct fr_desc));

	priv->class_0_spill =
		nlm_xlr_config_spill(priv->mmio,
				      R_CLASS0_SPILL_MEM_START_0,
				      R_CLASS0_SPILL_MEM_START_1,
				      R_CLASS0_SPILL_MEM_SIZE,
				      max_class_0_spill *
				      sizeof(union rx_tx_desc));
	priv->class_1_spill =
		nlm_xlr_config_spill(priv->mmio,
				      R_CLASS1_SPILL_MEM_START_0,
				      R_CLASS1_SPILL_MEM_START_1,
				      R_CLASS1_SPILL_MEM_SIZE,
				      max_class_1_spill *
				      sizeof(union rx_tx_desc));

	priv->frout_spill =
		nlm_xlr_config_spill(priv->mmio, R_FROUT_SPILL_MEM_START_0,
				      R_FROUT_SPILL_MEM_START_1,
				      R_FROUT_SPILL_MEM_SIZE,
				      max_frout_spill *
				      sizeof(struct fr_desc));

	priv->class_2_spill =
		nlm_xlr_config_spill(priv->mmio,
				      R_CLASS2_SPILL_MEM_START_0,
				      R_CLASS2_SPILL_MEM_START_1,
				      R_CLASS2_SPILL_MEM_SIZE,
				      max_class_2_spill *
				      sizeof(union rx_tx_desc));
	priv->class_3_spill =
		nlm_xlr_config_spill(priv->mmio,
				      R_CLASS3_SPILL_MEM_START_0,
				      R_CLASS3_SPILL_MEM_START_1,
				      R_CLASS3_SPILL_MEM_SIZE,
				      max_class_3_spill *
				      sizeof(union rx_tx_desc));
}

/*****************************************************************
 * Write the MAC address to the PHNX registers
 * All 4 addresses are the same for now
 *****************************************************************/
static void xlr_mac_setup_hwaddr(struct driver_data *priv)
{
	struct net_device *dev = priv->dev;

	netlogic_write_reg(priv->mmio, R_MAC_ADDR0,
			  ((dev->dev_addr[5] << 24) | (dev->dev_addr[4] << 16)
			   | (dev->dev_addr[3] << 8) | (dev->dev_addr[2]))
		);

	netlogic_write_reg(priv->mmio, R_MAC_ADDR0 + 1,
			  ((dev->dev_addr[1] << 24) | (dev->
						       dev_addr[0] << 16)));

	netlogic_write_reg(priv->mmio, R_MAC_ADDR_MASK2, 0xffffffff);

	netlogic_write_reg(priv->mmio, R_MAC_ADDR_MASK2 + 1, 0xffffffff);

	netlogic_write_reg(priv->mmio, R_MAC_ADDR_MASK3, 0xffffffff);

	netlogic_write_reg(priv->mmio, R_MAC_ADDR_MASK3 + 1, 0xffffffff);

	netlogic_write_reg(priv->mmio, R_MAC_FILTER_CONFIG,
				  (1 << O_MAC_FILTER_CONFIG__BROADCAST_EN) |
				  (1 << O_MAC_FILTER_CONFIG__ALL_MCAST_EN) |
				  (1 << O_MAC_FILTER_CONFIG__MAC_ADDR0_VALID)
			);

}

/*****************************************************************
 * Read the MAC address from the PHNX registers
 * All 4 addresses are the same for now
 *****************************************************************/
static void xlr_mac_get_hwaddr(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);

	dev->dev_addr[0] = xlr_base_mac_addr[0];
	dev->dev_addr[1] = xlr_base_mac_addr[1];
	dev->dev_addr[2] = xlr_base_mac_addr[2];
	dev->dev_addr[3] = xlr_base_mac_addr[3];
	dev->dev_addr[4] = xlr_base_mac_addr[4];
	dev->dev_addr[5] = xlr_base_mac_addr[5] + priv->id;
}

/**********************************************************************
 * Set a new Ethernet address for the interface.
 **********************************************************************/
static int nlm_xlr_set_mac_address(struct net_device *dev, void *addr) {
    struct driver_data *priv = netdev_priv(dev);
    struct sockaddr *p_sockaddr = (struct sockaddr *) addr;

    memcpy(dev->dev_addr, p_sockaddr->sa_data, 6);
    xlr_mac_setup_hwaddr(priv);
    return 0;
}

/*****************************************************************
 * Mac Module Initialization
 *****************************************************************/
static void mac_common_init(struct driver_data *priv)
{
	int i = 0, stid;
	void (*handler)(int, int,int,int,struct msgrng_msg *, void *);


	for (i = 0; i < MAC_FRIN_WORK_NUM; i++) {
		if(mac_frin_replenish_work[i].func == 0) 
			INIT_WORK(&mac_frin_replenish_work[i],
				  mac_frin_replenish_wq);
		if(mac_frin_replenish_task[i].func == 0)
			tasklet_init(&mac_frin_replenish_task[i],
				  mac_frin_replenish, 0UL);
	}

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
	/* Cached pointer to station ID translation table, needed for NAPI */ 
	if(is_xls())
		rxstn_to_txstn_ptr = &xls_rxstn_to_txstn_map[0];
	else
		rxstn_to_txstn_ptr = &rxstn_to_txstn_map[0];
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */


	if(priv->type == TYPE_GMAC) {
		if (is_xls()) {
			if(priv->instance <  NETLOGIC_GMAC_PORTS_PER_CTRL)
				stid = TX_STN_GMAC0;
			else
				stid = TX_STN_GMAC1;
		} else
			stid = TX_STN_GMAC;
	} else if(priv->type == TYPE_XGMAC) {
		if(priv->instance == 0)
			stid = TX_STN_XGS_0;
		else
			stid = TX_STN_XGS_1;
	} else {
		printk("Invalid type %d\n", priv->type);
		return;
	}

	if((MSGRNG_OWN(priv->cfg_flag))) 
		handler = nlm_xlr_mac_msgring_handler;
	else 
		handler = nlm_xlr_station_unowned_msgring_handler;

	if (register_msgring_handler(stid, handler, NULL))
			panic("Couldn't register msgring handler for TX_STN_GMAC0\n");

	return;
}




#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
/*
 * Function covering only gmac/xgmac NAPI statistics
*/
static int
xlr_napi_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	off_t begin = 0;
	int cpu = 0;


	if (nlm_msgring_napi) 
        {
		len += sprintf(page + len, "NAPI Poll Weight=%u\n",
			       napi_weight);
		len += sprintf(page + len, "         CPU          RX_COUNT\n");

		for (cpu = 0; cpu < NR_CPUS; cpu++) {
			if (!cpu_isset(cpu, cpu_online_map)) {
				continue;
			}
			len += sprintf(page + len, "napi: cpu=%02d: %16lld\n", cpu, 
								per_cpu(xlr_napi_rx_count, cpu));

			if (!proc_pos_check(&begin, &len, off, count))
				goto out;			 
		}      

		/* Clear on read */
		for (cpu = 0; cpu < NR_CPUS; cpu++) {
			per_cpu(xlr_napi_rx_count, cpu) = 0; 
		}
	}
	*eof = 1;

out:
	*start = page + (off - begin);
	len -= (off - begin);
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;	
}
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */

static int __init
nlm_napi_poll_weight(char *str)
{
	unsigned int wt = simple_strtoul(str, 0, 10);
	if(wt < 1200) {
		napi_weight = wt;
	}
	return 0;
}

early_param("nlm_napi_poll_weight", nlm_napi_poll_weight);

static int
xlr_mac_proc_read(char *page, char **start, off_t off,
			     int count, int *eof, void *data)
{
	int len = 0;
	off_t begin = 0;
	int i = 0, cpu = 0;
	struct net_device *dev = 0;
	struct driver_data *priv = 0;


	for(i=0; i<NETLOGIC_MAX_MACS; i++) {
		dev = dev_mac[i];
		if(dev == 0)
			continue;

		priv = netdev_priv(dev);
		
		for(cpu=0;cpu<32;cpu++) {
		
                        if (!cpu_isset(cpu, cpu_online_map))
                          continue;
		
			len += sprintf(page + len, "per_cpu: %d %d %d %d %lx %lx %lx %lx\n", 
				       i, cpu,
				       user_mac ? user_mac->time.hi : user_mac_krnl_data.time.hi,
				       user_mac ? user_mac->time.lo : user_mac_krnl_data.time.lo,
				       priv->cpu_stats[cpu].tx_packets,
				       priv->cpu_stats[cpu].txc_packets,
				       priv->cpu_stats[cpu].rx_packets,
				       priv->cpu_stats[cpu].interrupts);
			if (!proc_pos_check(&begin, &len, off, count))
				goto out;			       
		}

		len += sprintf(page + len,
			       "per_port: %d %d %d %lx %lx %lx %lx\n",
			       i,
			       user_mac ? user_mac->time.hi : user_mac_krnl_data.time.hi,
			       user_mac ? user_mac->time.lo : user_mac_krnl_data.time.lo,
			       priv->stats.rx_packets, priv->stats.rx_bytes,
			       priv->stats.tx_packets, priv->stats.tx_bytes);
		if (!proc_pos_check(&begin, &len, off, count))
			goto out;	     
	}

	*eof = 1;

      out:
	*start = page + (off - begin);
	len -= (off - begin);
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;	
}



#ifdef CONFIG_NLMCOMMON_HW_BUFFER_MGMT
/*
 * Setup for XLR/XLS automatic hardware buffer management. 
*/
static int __init
nlm_auto_buffer_mgmt_setup(char *str)
{
	if (str == NULL || !strcmp(str, "yes") || !strcmp(str, "y")) {
		nlm_auto_buffer_mgmt = 1;
		printk(KERN_ALERT "Enabling automatic hardware buffer management\n");
	}
	else if (!strcmp(str, "no") || !strcmp(str, "n")) {
		nlm_auto_buffer_mgmt = 0;
		printk(KERN_ALERT "Disabling automatic hardware buffer management\n");
	}

	return 0;
}

/* for compatibility we use "xlr_" prefix for the option */
early_param("xlr_auto_buffer_mgmt", nlm_auto_buffer_mgmt_setup);
#endif /* CONFIG_NLMCOMMON_HW_BUFFER_MGMT */




#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
/*
 * This function is used upon the exit from NAPI poll to re-enable interrupts
*/
static void
xlr_napi_enable_ints(void)
{
	unsigned int msgring_config;
	unsigned long flags = 0, mflags = 0; 
	struct napi_control_s *p = &napi_control[netlogic_cpu_id()];
	unsigned long rcv_bmask; 
	unsigned long this_thread_bmask = (1 << netlogic_thr_id());


	msgrng_access_save(&p->xlr_napi_msgrng_lock, flags, mflags);
	p->netrx_mask |= this_thread_bmask;


	/* Read message ring status */
	rcv_bmask = (~(msgrng_read_status() >> 24)) & 0xff;
	rcv_bmask = ((rcv_bmask & 0xf) | (rcv_bmask >> 4));

	rcv_bmask &= p->netrx_mask; 

	if (rcv_bmask) {
		/* rewrite the interrupt mask */
		msgring_config = msgrng_read_config();
		msgring_config |= (rcv_bmask << 8);
		msgrng_write_config(msgring_config); 
	}
	else {

		/* rewrite the interrupt mask */
		msgring_config = msgrng_read_config();
		msgring_config |= (this_thread_bmask << 8);
		msgrng_write_config(msgring_config); 
	}
	msgrng_access_restore(&p->xlr_napi_msgrng_lock, flags, mflags);
}


/* called from msgring_process_rx_msgs() from on_chip.c  */
void
xlr_napi_rx_schedule(void)
{
	unsigned int msgring_config;
	unsigned long flags = 0, mflags = 0; 
	struct napi_struct *napi;
	unsigned long rcv_bmask; // bit array: bmask[i] is 1 iff bucket[i] or bucket[i + 4] non-empty 
	unsigned long this_thread_bmask; // non-zero if THIS thread has packets in its buckets
	struct napi_control_s *p = &napi_control[netlogic_cpu_id()];

	if (!xlr_napi_ready)
		return;

	/* rewrite the interrupt mask */
	msgrng_access_save(&p->xlr_napi_msgrng_lock, flags, mflags);

	/* Read message ring status */
	rcv_bmask = (~(msgrng_read_status() >> 24)) & 0xff;
	rcv_bmask = ((rcv_bmask & 0xf) | (rcv_bmask >> 4));

	if (rcv_bmask == 0) {
		msgrng_access_restore(&p->xlr_napi_msgrng_lock, flags, mflags);
		write_64bit_cp0_eirr(1ULL << IRQ_MSGRING);
		return;
	}

	this_thread_bmask = rcv_bmask & (1 << netlogic_thr_id());
	p->netrx_mask &= ~this_thread_bmask;
	rcv_bmask = rcv_bmask & p->netrx_mask; // & ~(1 << netlogic_thr_id());

	msgring_config = msgrng_read_config();
	msgring_config = (msgring_config & 0xfffff0ff) | (rcv_bmask << 8) ;

	msgrng_write_config(msgring_config);

	msgrng_access_restore(&p->xlr_napi_msgrng_lock, flags, mflags);
  
	/* Acknowledge interrupt in eirr */
	write_64bit_cp0_eirr(1ULL << IRQ_MSGRING);

	if (this_thread_bmask) {
		/* schedule polling for this cpu using dummy_dev */
		napi = &__get_cpu_var(xlr_napi_poll_struct);
		napi_schedule(napi);
	}
}


/*
 * Main NAPI poll loop
*/
int
xlr_napi_poll(struct napi_struct *napi, int budget)
{
	int rx_pkts = 0;
	xlr_napi_poll_upper(&xlr_napi_dummy_dev, budget);
	rx_pkts = napi_poll_lower(&xlr_napi_dummy_dev, budget);

	if(rx_pkts < budget)
	{
		napi_complete(napi);
		/* enable message ring interrupts */
		xlr_napi_enable_ints();
	}
	return rx_pkts;
}


/*
 * Setup for XLR/XLS msgring NAPI parameter. 
*/
static int __init
nlm_msgring_napi_setup(char *str)
{
	if (str == NULL || !strcmp(str, "yes") || !strcmp(str, "y")) {
		nlm_msgring_napi = 1;
		nlm_on_chip_napi = 1;
	}
	else if (!strcmp(str, "no") || !strcmp(str, "n")) {
		nlm_msgring_napi = 0;
		nlm_on_chip_napi = 0;
	}

	return 0;
}

/* for compatibility we use "xlr_" prefix for the option */
early_param("xlr_msgring_napi", nlm_msgring_napi_setup);


/*
 * Setup for XLR/XLS msgring NAPI parameter. 
*/
static int __init
nlm_deprecated_napi_setup(char *str)
{
	if (str == NULL || !strcmp(str, "yes") || !strcmp(str, "y")) {
		nlm_msgring_napi = 1;
		nlm_on_chip_napi = 1;
	}
	return 0;
}

/* Deprecated setup option for NAPI */
early_param("xlr_napi", nlm_deprecated_napi_setup);



/*
 * NAPI setup for non-networking on-chip devices. 
*/
static int __init
nlm_on_chip_napi_setup(char *str)
{
	if (str == NULL || !strcmp(str, "yes") || !strcmp(str, "y")) {
		if (nlm_msgring_napi == 0) {
			printk("MSGRING_NAPI:*****************************************************************\n");
			printk("MSGRING_NAPI:  Can't enable on_chip NAPI: enable xlr_msgring_napi first      *\n");
			printk("MSGRING_NAPI:*****************************************************************\n");
			nlm_on_chip_napi = 0;
		} else {
			nlm_on_chip_napi = 1;
		}
	}
	else if (!strcmp(str, "no") || !strcmp(str, "n")) {
		nlm_on_chip_napi = 0;
	}

	return 0;
}

/* for compatibility we use "xlr_" prefix for the option */
early_param("xlr_on_chip_napi", nlm_on_chip_napi_setup);



/*
 * Setup XLR/XLS NAPI subsystem 
*/
static int
nlm_xlr_napi_setup(void)
{
	int i, cpu_count; 
	struct napi_struct *napi; 
	int weight_p = napi_weight; 

	/* napi required msgring interrupt to be enabled, 
	 * but it can be enabled only if both gmac and xgmac/spi4
	 * are owned by linux 
         */
	if (xlr_hybrid_user_mac_xgmac())
		return 0;

	printk("MSGRING_NAPI: Initializing RMI GMAC/XGMAC NAPI subsystem\n");

	msgring_int_type = 0x01;

	atomic_set(&(xlr_napi_dummy_dev.refcnt), 1);
	set_bit(__LINK_STATE_START, &xlr_napi_dummy_dev.state);

	for (cpu_count = 0; cpu_count < NR_CPUS; cpu_count++) 
	{
		napi = &per_cpu(xlr_napi_poll_struct, cpu_count);
		memset(napi, 0, sizeof(napi));
		netif_napi_add(&xlr_napi_dummy_dev, napi, xlr_napi_poll, weight_p);
		napi_enable(napi);
	}

	for (i = 0; i < NR_CPUS; i++) {
		per_cpu(xlr_napi_rx_count, i) = 0; 
	}

	return 0;
}

#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */




static int xlr_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;

	if ((priv->type == TYPE_XGMAC) || (priv->phy.mode == PHY_MODE_XAUI)){
		cmd->supported = SUPPORTED_FIBRE|SUPPORTED_10000baseT_Full;
		cmd->advertising = SUPPORTED_FIBRE|SUPPORTED_10000baseT_Full;
		cmd->speed = SPEED_10000;
		cmd->port = PORT_FIBRE;
		cmd->duplex = DUPLEX_FULL;
		cmd->phy_address = priv->instance;
		cmd->autoneg = AUTONEG_DISABLE;
		cmd->maxtxpkt = 0;
		cmd->maxrxpkt = 0;

	}else{

		cmd->supported = SUPPORTED_10baseT_Full | 
			SUPPORTED_10baseT_Half | 
			SUPPORTED_100baseT_Full | SUPPORTED_100baseT_Half |
			SUPPORTED_1000baseT_Full | SUPPORTED_MII |
			SUPPORTED_Autoneg | SUPPORTED_TP;

		cmd->advertising = priv->advertising;

		mii_status = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, MII_NCONFIG);
		priv->speed = (mii_status >> 3) & 0x03;

		cmd->speed = (priv->speed == xlr_mac_speed_1000) ? SPEED_1000 :
		(priv->speed == xlr_mac_speed_100) ? SPEED_100: SPEED_10;

		cmd->duplex = (mii_status >> 5) & 0x1;
		cmd->port = PORT_TP;
		cmd->phy_address = priv->instance;
		cmd->transceiver = XCVR_INTERNAL;
		cmd->autoneg = (~(mii_status >> 14)) & 0x1;
		cmd->maxtxpkt = 0;
		cmd->maxrxpkt = 0;
	}

	return 0;
}
static int xlr_enable_autoneg(struct net_device *dev, u32 adv)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;
	u32 adv1, adv2;
    unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	nlm_xlr_mac_set_enable(priv, 0);
	/* advertising for 10/100 Mbps */
	adv1 = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, MII_ADVERTISE);
	adv1 &= ~(ADVERTISE_ALL | ADVERTISE_100BASE4);
	/* advertising for 1000 Mbps */
	adv2 = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, 0x9);
	adv2 &= ~(0x300);

	if(adv & ADVERTISED_10baseT_Half)
		adv1 |= ADVERTISE_10HALF;
	if(adv & ADVERTISED_10baseT_Full)
		adv1 |= ADVERTISE_10FULL;
	if(adv & ADVERTISED_100baseT_Full)
		adv1 |= ADVERTISE_100FULL;
	if(adv & ADVERTISED_100baseT_Half)
		adv1 |= ADVERTISE_100HALF;

	if(adv & ADVERTISED_1000baseT_Full)
		adv2 |= 0x200;
	if(adv & ADVERTISED_1000baseT_Half)
		adv2 |= 0x100;

	/* Set the advertising parameters */
	nlm_xlr_mac_mii_write(priv->phy.mii_addr, priv->phy.addr, MII_ADVERTISE, adv1);
	nlm_xlr_mac_mii_write(priv->phy.mii_addr, priv->phy.addr, 0x9, adv2);

	priv->advertising = adv1 | adv2;

	mii_status = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, MII_BMCR);
	/* enable autoneg and force restart autoneg */
	mii_status |= (BMCR_ANENABLE | BMCR_ANRESTART);
	nlm_xlr_mac_mii_write(priv->phy.mii_addr, priv->phy.addr, MII_BMCR, mii_status);

	nlm_xlr_mac_set_enable(priv, 1);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int xlr_set_link_speed(struct net_device *dev, int speed, int duplex)
{
	u32 adv;
	int ret =0;

	switch(speed) {
		case SPEED_10:
			if ( duplex == DUPLEX_FULL )
				adv = ADVERTISED_10baseT_Full;
			else
				adv = ADVERTISED_10baseT_Half;
			break;
		case SPEED_100:
			if ( duplex == DUPLEX_FULL )
				adv = ADVERTISED_100baseT_Full;
			else
				adv = ADVERTISED_100baseT_Half;
			break;
		case SPEED_1000:
			if ( duplex == DUPLEX_FULL )
				adv = ADVERTISED_1000baseT_Full;
			else
				adv = ADVERTISED_1000baseT_Half;
			break;
		default:
			ret = -EINVAL;
			return ret;
	}
	ret = xlr_enable_autoneg( dev,adv);
	return ret;

}

static int xlr_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	int ret;
	struct driver_data *priv = netdev_priv(dev);

	if ((priv->type == TYPE_XGMAC) || (priv->phy.mode == PHY_MODE_XAUI)){
		return -EIO;
	}
	if (cmd->autoneg == AUTONEG_ENABLE) {
		ret = xlr_enable_autoneg(dev, cmd->advertising);
	}else {
		ret = xlr_set_link_speed(dev, cmd->speed, cmd->duplex);
	}
	return ret;
}

static void xlr_get_drvinfo(struct net_device *dev, 
				struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
}

static int xlr_get_regs_len(struct net_device *dev) 
{
	return NLM_ETHTOOL_REG_LEN;
}
static void xlr_get_regs(struct net_device *dev,
				struct ethtool_regs *regs, void *p)
{
	u32 *data = (u32 *)p;
	int i;
	struct driver_data *priv = netdev_priv(dev);
	unsigned long flags;

	memset((void *)data, 0, NLM_ETHTOOL_REG_LEN);

	spin_lock_irqsave(&priv->lock, flags);
	for(i=0; i <= NLM_NUM_REG_DUMP; i++)
		*(data + i) = netlogic_read_reg(priv->mmio,  R_TX_CONTROL + i);
	spin_unlock_irqrestore(&priv->lock, flags);
}
static u32 xlr_get_msglevel(struct net_device *dev)
{
	return mac_debug;
}
static void xlr_set_msglevel(struct net_device *dev, u32 value)
{
	mac_debug = value;
}

static int xlr_nway_reset(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;
	unsigned long flags;
	int ret = -EINVAL;

   if ((priv->type == TYPE_XGMAC) || (priv->phy.mode == PHY_MODE_XAUI))
    return -EIO;

	spin_lock_irqsave(&priv->lock, flags);
	mii_status = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, MII_BMCR);
	if(mii_status & BMCR_ANENABLE)
	{
		nlm_xlr_mac_mii_write(priv->phy.mii_addr, priv->phy.addr, 
				MII_BMCR, BMCR_ANRESTART | mii_status);
		ret = 0;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
static u32 xlr_get_link(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;
	unsigned long flags;

   if ((priv->type == TYPE_XGMAC) || (priv->phy.mode == PHY_MODE_XAUI))
    return -EIO;

	spin_lock_irqsave(&priv->lock, flags);
	mii_status = nlm_xlr_mac_mii_read(priv->phy.mii_addr, priv->phy.addr, MII_BMSR);

	spin_unlock_irqrestore(&priv->lock, flags);

	if(mii_status & BMSR_LSTATUS)
		return 1;
	return 0;
}
#define NLM_STATS_KEY_LEN  \
		(sizeof(struct net_device_stats) / sizeof(unsigned long))
static struct {
	        const char string[ETH_GSTRING_LEN];
} xlr_ethtool_stats_keys[NLM_STATS_KEY_LEN] = {
	{ "rx_packets" },
	{ "tx_packets" },
	{ "rx_bytes" },
	{ "tx_bytes" },
	{ "rx_errors" },
	{ "tx_errors" },
	{ "rx_dropped" },
	{ "tx_dropped" },
	{ "multicast" },
	{ "collisions" },
	{ "rx_length_errors" },
	{ "rx_over_errors" },
	{ "rx_crc_errors" },
	{ "rx_frame_errors" },
	{ "rx_fifo_errors" },
	{ "rx_missed_errors" },
	{ "tx_aborted_errors" },
	{ "tx_carrier_errors" },
	{ "tx_fifo_errors" },
	{ "tx_heartbeat_errors" },
	{ "tx_window_errors" },
	{ "rx_compressed" },
	{ "tx_compressed" }
};
static int xlr_get_stats_count (struct net_device *dev)
{
	return NLM_STATS_KEY_LEN;
}

static void xlr_get_strings (struct net_device *dev, u32 stringset, u8 *buf)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(buf, &xlr_ethtool_stats_keys, 
				sizeof(xlr_ethtool_stats_keys));
		break;
	default:
		printk(KERN_WARNING "%s: Invalid stringset %d\n", 
				__FUNCTION__, stringset);
		break;
	}
}

static void xlr_get_mac_stats(struct net_device *dev, 
					struct net_device_stats *stats)
{
	struct driver_data *priv = netdev_priv(dev);

	stats->tx_errors = netlogic_read_reg(priv->mmio, TX_FCS_ERROR_COUNTER);
	stats->rx_dropped = netlogic_read_reg(priv->mmio, 
						RX_DROP_PACKET_COUNTER);
	stats->tx_dropped = netlogic_read_reg(priv->mmio, TX_DROP_FRAME_COUNTER);

	stats->multicast = netlogic_read_reg(priv->mmio, 
						RX_MULTICAST_PACKET_COUNTER);
	stats->collisions = netlogic_read_reg(priv->mmio, 
						TX_TOTAL_COLLISION_COUNTER);

	stats->rx_length_errors = netlogic_read_reg(priv->mmio, 
						RX_FRAME_LENGTH_ERROR_COUNTER);
	stats->rx_over_errors = netlogic_read_reg(priv->mmio, 
						RX_DROP_PACKET_COUNTER);
	stats->rx_crc_errors = netlogic_read_reg(priv->mmio, 
						RX_FCS_ERROR_COUNTER);
	stats->rx_frame_errors = netlogic_read_reg(priv->mmio, 
						RX_ALIGNMENT_ERROR_COUNTER);

	stats->rx_fifo_errors = netlogic_read_reg(priv->mmio,
					    	RX_DROP_PACKET_COUNTER);
	stats->rx_missed_errors = netlogic_read_reg(priv->mmio,
					    	RX_CARRIER_SENSE_ERROR_COUNTER);

	stats->rx_errors = (stats->rx_over_errors + stats->rx_crc_errors +
			     stats->rx_frame_errors + stats->rx_fifo_errors +
			     stats->rx_missed_errors);

	stats->tx_aborted_errors = netlogic_read_reg(priv->mmio, 
			TX_EXCESSIVE_COLLISION_PACKET_COUNTER);
	stats->tx_carrier_errors = netlogic_read_reg(priv->mmio, 
					TX_DROP_FRAME_COUNTER);
	stats->tx_fifo_errors = netlogic_read_reg(priv->mmio, 
					TX_DROP_FRAME_COUNTER);

}

static void xlr_get_ethtool_stats (struct net_device *dev,
			struct ethtool_stats *estats, u64 *stats)
{
	int i;
	struct driver_data *priv = netdev_priv(dev);
	unsigned long flags;
	unsigned long *tmp_stats;
	
	spin_lock_irqsave(&priv->lock, flags);
	
	xlr_get_mac_stats(dev, &priv->stats);
	
	
	spin_unlock_irqrestore(&priv->lock, flags);

	tmp_stats = (unsigned long *)&priv->stats;
	for(i=0; i < NLM_STATS_KEY_LEN; i++) {
		*stats = (u64)*tmp_stats;
		stats++;
		tmp_stats++;
	}
}

static struct ethtool_ops xlr_ethtool_ops= {
        .get_settings           = xlr_get_settings,
        .set_settings           = xlr_set_settings,
        .get_drvinfo            = xlr_get_drvinfo,
        .get_regs_len           = xlr_get_regs_len,
        .get_regs               = xlr_get_regs,
        .get_msglevel           = xlr_get_msglevel,
        .set_msglevel           = xlr_set_msglevel,
        .nway_reset             = xlr_nway_reset,
        .get_link               = xlr_get_link,
        .get_strings            = xlr_get_strings,
        .get_stats_count        = xlr_get_stats_count,
        .get_ethtool_stats      = xlr_get_ethtool_stats,
};

void nlm_reset_gmac(nlm_reg_t *mmio)
{
    volatile uint32_t val;

        /* Disable MAC RX */
        val = netlogic_read_reg(mmio, R_MAC_CONFIG_1);
        val &= ~0x4;
        netlogic_write_reg(mmio, R_MAC_CONFIG_1, val);

        /* Disable Core RX */
        val = netlogic_read_reg(mmio, R_RX_CONTROL);
        val &= ~0x1;
        netlogic_write_reg(mmio, R_RX_CONTROL, val);

        /* wait for rx to halt */
        while(1) {
            val = netlogic_read_reg(mmio, R_RX_CONTROL);
            if(val & 0x2)
                break;
            mdelay(1);
        }

        /* Issue a soft reset */
        val = netlogic_read_reg(mmio, R_RX_CONTROL);
        val |= 0x4;
        netlogic_write_reg(mmio, R_RX_CONTROL, val);
           
        /* wait for reset to complete */
        while(1) {
            val = netlogic_read_reg(mmio, R_RX_CONTROL);
            if(val & 0x8)
                break;
            mdelay(1);
        }

        /* Clear the soft reset bit */
        val = netlogic_read_reg(mmio, R_RX_CONTROL);
        val &= ~0x4;
        netlogic_write_reg(mmio, R_RX_CONTROL, val);
}

void nlm_reset_xaui(nlm_reg_t *mmio)
{
    volatile uint32_t val;

    /* Disable Core RX */
    val = netlogic_read_reg(mmio, R_RX_CONTROL);
    val &= ~0x1;
    netlogic_write_reg(mmio, R_RX_CONTROL, val);

    /* wait for rx to halt */
    while(1) {
        val = netlogic_read_reg(mmio, R_RX_CONTROL);
        if(val & 0x2)
            break;
        mdelay(1);
    }

    /* Issue a soft reset */
    val = netlogic_read_reg(mmio, R_RX_CONTROL);
    val |= 0x4;
    netlogic_write_reg(mmio, R_RX_CONTROL, val);

    /* wait for reset to complete */
    while(1) {
        val = netlogic_read_reg(mmio, R_RX_CONTROL);
        if(val & 0x8)
            break;
        mdelay(1);
    }

    /* Clear the soft reset bit */
    val = netlogic_read_reg(mmio, R_RX_CONTROL);
    val &= ~0x4;
    netlogic_write_reg(mmio, R_RX_CONTROL, val);
}

u16  nlm_select_tx_queue(struct net_device *dev, struct sk_buff *skb)
{
	return (u16)smp_processor_id();
}


static void setup_net_ops(struct net_device_ops *mac_ops)
{
	mac_ops->ndo_open = nlm_xlr_mac_open;
	mac_ops->ndo_stop = nlm_xlr_mac_close;
	mac_ops->ndo_get_stats = nlm_xlr_mac_get_stats;
	mac_ops->ndo_set_multicast_list = nlm_xlr_mac_set_multicast_list;
	mac_ops->ndo_set_mac_address = nlm_xlr_set_mac_address;
	mac_ops->ndo_do_ioctl = nlm_xlr_mac_do_ioctl;
	mac_ops->ndo_tx_timeout = nlm_xlr_mac_tx_timeout;
	mac_ops->ndo_change_mtu = nlm_xlr_mac_change_mtu;
	mac_ops->ndo_select_queue = nlm_select_tx_queue;
#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
	if (nlm_msgring_napi) {
		mac_ops->ndo_start_xmit = nlm_xlr_napi_mac_xmit;
	}
	else {
		mac_ops->ndo_start_xmit = nlm_xlr_mac_xmit;
	}
#else
	mac_ops->ndo_start_xmit = nlm_xlr_mac_xmit;

#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */
}

int nlm_xlr_mac_init_module(void)
{
	struct net_device *dev = 0;
	struct driver_data *priv = 0;
	unsigned long mmio_start = 0;
	int i = 0, num_desc = 0, num_desc_total = 0;
	int ret = 0, port_type;
	struct proc_dir_entry *entry;
	struct port_cfg *port_cfg;
	extern struct net_device_cfg xlr_net_dev_cfg;
	struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;
    int port_xaui = 0;

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI

	/* Run NAPI compatibility test */
	if (!nlm_napi_compatibility_check()) {
		nlm_msgring_napi = 0;
		nlm_on_chip_napi = 0;
	}

	if (nlm_msgring_napi) {
		nlm_xlr_napi_setup();
	}
	else {
		printk("MSGRING_NAPI: NAPI is not enabled!\n");
	}

        /* Initialize spinlock protecting NAPI msgring_config access */
        for (i = 0; i < NR_CPUS / 4; i++) {
		spin_lock_init(&napi_control[i].xlr_napi_msgrng_lock);
		napi_control[i].netrx_mask = 0xf;
        }
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */
        for (i = 0; i < NETLOGIC_MAX_MACS; i++) {
		spin_lock_init(&pending_tx_lock[i]);
	}

	chip_is_xls = is_xls();

	entry = create_proc_read_entry("nlm_mac_stats", 0 /* def mode */ ,
				       nlm_root_proc /* parent */ ,
				       xlr_mac_proc_read
				       /* proc read function */ ,
				       0	/* no client data */
		);
	if (!entry) {
		printk("[%s]: Unable to create proc read entry for xlr_mac!\n",
		       __FUNCTION__);
	}	

#ifdef CONFIG_NLMCOMMON_MSGRING_NAPI
	if (nlm_msgring_napi) {
		entry = create_proc_read_entry("nlm_napi_stats", 0 /* def mode */ ,
					       nlm_root_proc /* parent */ ,
					       xlr_napi_proc_read
					       /* proc read function */ ,
					       0 /* no client data */
		);
		if (!entry) {
			printk("[%s]: Unable to create proc read entry for xlr_napi!\n",
				__FUNCTION__);
		}	
	}
#endif /* CONFIG_NLMCOMMON_MSGRING_NAPI */
	setup_net_ops(&nlm_mac_net_ops);

	for (i = 0; i < NETLOGIC_MAX_MACS; i++) {

		if(i < NETLOGIC_MAX_GMACS) {
			port_cfg = &net_cfg->gmac_port[i];
			port_type = TYPE_GMAC;
            port_xaui = (port_cfg->phy_mode == PHY_MODE_XAUI);
		} else if(net_cfg->xgs_type[i - NETLOGIC_MAX_GMACS] == TYPE_XGMAC) {
			port_cfg = &net_cfg->xgs_port[i - NETLOGIC_MAX_GMACS];
			port_type = TYPE_XGMAC;
		} else
			continue;

		if(port_cfg->cfg_flag == 0)
			continue;

		dbg_msg("Registering xlr_mac[%d]\n", i);

		dev = alloc_etherdev_mq(sizeof(struct driver_data), 32);
		if (!dev) {
			ret = -ENOMEM;
			goto out;
		}

		priv = netdev_priv(dev);
		priv->dev = dev;
		priv->cfg_flag = port_cfg->cfg_flag;

		priv->mmio = (nlm_reg_t *) port_cfg->mmio_addr;
		if (!priv->mmio) {
			dbg_panic
				("Unable to ioremap MMIO region of size %x @ %lx\n",
				 NETLOGIC_IO_SIZE, mmio_start);
		}

		dbg_msg(" priv->mmio=%p\n",	priv->mmio);

		if(port_type == TYPE_GMAC && PORT_INIT(priv->cfg_flag)) {
            if(port_xaui)
                nlm_reset_xaui(priv->mmio);
            else
                nlm_reset_gmac(priv->mmio);
        }

		/* Initialize the net_device */
		if (PORT_INT_ATTACH(priv->cfg_flag)) {
			dev->irq = port_cfg->irqno;
			if (request_irq(dev->irq, nlm_xlr_mac_int_handler,
						IRQF_DISABLED, dev->name, dev)) {
				ret = -EBUSY;
				panic("Couldn't get mac interrupt line (%d)", dev->irq);
			}
		}

		ether_setup(dev);

		dev->base_addr = (long)priv->mmio;
		dev->mem_end = (long)priv->mmio + NETLOGIC_IO_SIZE - 1;
		dev->netdev_ops = &nlm_mac_net_ops;

		dev->watchdog_timeo = (1000 * HZ);

		dev->features |= NETIF_F_LLTX;

		SET_ETHTOOL_OPS(dev,&xlr_ethtool_ops);
		/* Initialize the device specific driver data */
		spin_lock_init(&priv->lock);

		priv->id = i;
		priv->instance = port_cfg->instance;
		priv->type = port_type;
		if(port_cfg->num_desc) {
			num_desc = port_cfg->num_desc;
			priv->spill_init = 1;
		} else
			priv->spill_init = 0;

		dev->tx_queue_len = (num_desc/32) - 1;
		priv->num_desc = num_desc;
		priv->config_pde = port_cfg->config_pde;

		num_desc_total += num_desc;

		/* Caching FRF and TX station IDs */
  		priv->fr_stid = msgrng_stid_rfr(priv->instance, priv->type);
  		priv->tx_stid = mac_fill_tx_stid(priv->instance, priv->type);

		/* fill the phy info*/
		priv->phy.addr 	   = port_cfg->phy_addr;
		priv->phy.mode = port_cfg->phy_mode;
		priv->phy.mii_addr = mac_addr_to_ptr(port_cfg->mii_addr);
		priv->phy.pcs_addr = mac_addr_to_ptr(port_cfg->pcs_addr);
		priv->phy.serdes_addr = mac_addr_to_ptr(port_cfg->serdes_addr);


		xlr_mac_get_hwaddr(dev);

		if(PORT_INIT(priv->cfg_flag)) {
			if (priv->type == TYPE_GMAC) {
                if(port_xaui)
                    nlm_xlr_xaui_init(priv, port_cfg);
                else
                    nlm_xlr_gmac_init(priv, port_cfg);
            }
			else if(priv->type == TYPE_XGMAC) 
				nlm_xlr_xgmac_init(priv, port_cfg);
			
            xlr_mac_setup_hwaddr(priv);
		}

		if(PORT_ATTACH(priv->cfg_flag))  {
			ret = register_netdev(dev);
			if (ret) {
				dbg_panic("Unable to register net device\n");
			}
			else {
				if (priv->type == TYPE_GMAC)
					printk("GMAC_%d initialized as %s\n", priv->instance, priv->dev->name);
				else if (priv->type == TYPE_XGMAC)
					printk("XGMAC_%d initialized as %s\n", priv->instance, priv->dev->name);
			}
		}

//		if(PORT_INIT(priv->cfg_flag)) 
//			rmik_config_pde(priv->type, priv->instance, priv->mmio);

/*		if(rmik_en) {
			uint64_t pde_bkt_map = 0ULL;
			pde_bkt_map = rmik_get_pde_bktmap(priv->type, priv->instance);
			priv->frstid_rsvd = ((pde_bkt_map & free_back_stid_map) == 0ULL) ? 1 : 0;
		} else
*/
		priv->frstid_rsvd = 1;


		mac_common_init(priv);
		nlm_xlr_mac_set_enable(priv, 0);

		dbg_msg("%s: Phoenix Mac at 0x%p (mtu=%d)\n",
			dev->name, priv->mmio, dev->mtu);

		dev_mac_type[priv->type][priv->instance] = dev;
		dev_mac[i] = dev;
	}
/*	
	rmik_init_replenish_work(num_desc);
	rmik_register_net_events();
*/
	dbg_msg("port_counters = %p\n", port_counters);
	dbg_msg("pending_tx_lock = %p, pending_tx = %p\n", port_counters,
		pending_tx);


#ifdef CONFIG_NLMCOMMON_IP_FLOW_AFFINITY
        /* initialize cpu skb queues */
        cpu_tx_queue_init();
#endif /* CONFIG_NLMCOMMON_IP_FLOW_AFFINITY */

 out:
        if ( (xlr_board_atx_v() || xlr_board_atx_iv_b())) {
            /* on atx-v and atx-iv-b read rgmii interrupt at least once */
            dev = dev_mac_type[TYPE_GMAC][0];
            if(dev != 0) {
                priv = netdev_priv(dev);
                nlm_xlr_mac_mii_read(priv->phy.mii_addr, 3, 26);
            }
        }
 
	for (i = 0; i < NETLOGIC_MAX_GMACS; i++) {
		if(dev_mac[i] == 0)
                       continue;

		priv = netdev_priv(dev_mac[i]);
		if (PORT_INIT(priv->cfg_flag)) 
			xlr_mac_set_rx_mode(dev_mac[i]);

        if (PORT_INT_ATTACH(priv->cfg_flag)) {
            netlogic_write_reg(priv->mmio, R_INTMASK,
                    (1<<O_INTMASK__TxIllegal)       |
                    (((priv->instance&0x3)==0)<<O_INTMASK__MDInt)           |
                    (1<<O_INTMASK__TxFetchError)    |
                    (1<<O_INTMASK__P2PSpillEcc)     |
                    (1<<O_INTMASK__TagFull)         |
                    (1<<O_INTMASK__Underrun)        |
                    (1<<O_INTMASK__Abort));
        }
	}
	if (ret < 0) {
		dbg_panic("Error, ret = %d\n", ret);
	}
	return ret;
}

/**********************************************************************
 **********************************************************************/
void nlm_xlr_mac_exit_module(void)
{
	struct net_device *dev;
	int idx;

	for (idx = 0; idx < NETLOGIC_MAX_MACS; idx++) {
		dev = dev_mac[idx];
		if (dev == 0)
			continue;

		unregister_netdev(dev);
		free_netdev(dev);
	}
}
#ifdef CONFIG_NLMCOMMON_PTP_SUPPORT

int nlm_macreg_set(int inf, unsigned int reg, unsigned int val)
{
struct driver_data *priv = NULL;
struct net_device *dev = NULL;
 
    dev = dev_mac[inf];

    if (!dev) 
         return -1;
 
   	priv = netdev_priv(dev);
    netlogic_write_reg(priv->mmio, reg, netlogic_read_reg(priv->mmio, reg)|val);
    return 0;
}

u32 nlm_macreg_get(int inf, unsigned int reg)
{
struct driver_data *priv = NULL;
struct net_device *dev = NULL;
 
    dev = dev_mac[inf];

    if (!dev) 
         return -1;
 
    priv = netdev_priv(dev);
    return netlogic_read_reg(priv->mmio, reg);
}

int nlm_mac_get_inf_idx(char *infname)
{
int i = 0;
struct net_device *dev = NULL;
u32  rc = -1; 
  for( i= 0 ; i < NETLOGIC_MAX_MACS ; i++) {
        if(!strncmp( infname, dev[i].name, sizeof(infname))) {
            rc = i;
           break;
        }
    }
   return rc; 
}

int nlm_macreg_clr_set(int inf, u32 reg, u32 val, u32 mask)
{
struct driver_data *priv = NULL;
struct net_device *dev = NULL;
u32 curr_val = 0;
 
    dev = dev_mac[inf];

    if (!dev) 
         return -1;
 
   	priv = netdev_priv(dev);
    curr_val = netlogic_read_reg(priv->mmio, reg) & (~mask);
//    printk("reg %x curval %x reg %x\n", reg, curr_val, netlogic_read_reg(priv->mmio,reg));
//    printk("wr reg %x curval %x val %x mask %x\n", reg, curr_val| (val& mask), val , mask);
    netlogic_write_reg(priv->mmio, reg, curr_val|(val & mask));
    
    //printk("reg %x rdval %x\n", reg, netlogic_read_reg(priv->mmio,reg));
    return 0;
}
void nlm_macreg_set_all(u32 reg, u32 val,u32 mask)
{
int i = 0;

    for( i = 0; i < NETLOGIC_MAX_MACS; i++) {
        nlm_macreg_clr_set(i, reg, val, mask);        
        //nlm_macreg_set(i, reg, val);        
    }
}
void nlm_macreg_clr_set_all(u32 reg, u32 val,u32 mask)
{
int i = 0;

    for( i = 0; i < NETLOGIC_MAX_MACS; i++) {
        nlm_macreg_clr_set(i, reg, val, mask);        
//        nlm_macreg_set(i, reg, val);        
    }
}
void dump_all_interface(unsigned int reg)
{
int i = 0;
struct net_device *dev;
struct driver_data *priv;
u32 curr_val = 0; 
       
    for( i = 0; i < NETLOGIC_MAX_MACS; i++) {

        dev = dev_mac[i];
           if (!dev) 
                continue; 
     	priv = netdev_priv(dev);
        curr_val = netlogic_read_reg(priv->mmio,reg);
//        printk("interface %x    val %x name %s\n", priv->mmio,  curr_val, dev->name); 
    }

}
void nlm_register_ptp_ts_fp(void(*fp)(u32, u32,ktime_t *, u32))
{
    printk("register ptp\n");
    p_ptp_set_ts = fp;
}
void nlm_clr_ptp_ts_fp(void){
    p_ptp_set_ts = NULL;
}
EXPORT_SYMBOL(nlm_macreg_set_all);
EXPORT_SYMBOL(dump_all_interface);
EXPORT_SYMBOL(nlm_register_ptp_ts_fp);
#endif
module_init(nlm_xlr_mac_init_module);
module_exit(nlm_xlr_mac_exit_module);

/*************************************************************************
 * TODO:
 *     o Currently, if Tx completes do not come back, Tx hangs for ever. Though it is good
 *       for debugging, there should be a timeout mechanism.
 *     o Right now, all cpu-threads across cpus are serialized for transmitting
 *       packets. However, message_send is "atomic", hence all of them should
 *       transmit without contending for the lock. some like per-cpu, per device lock 
 *       and handling tx complete on the cpu that did the transmit
 *     o use fetchadd for stat variable. currently, it not even atomic
 *************************************************************************/
