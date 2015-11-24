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
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/cpumask.h>
#include <linux/hugetlb.h>
#include <linux/bootmem.h>

#include <asm/uaccess.h>
#include <asm/mman.h>
#include <asm/atomic.h>
#include <asm/smp.h>
#include <asm/page.h>

#include <asm/netlogic/pic.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/devices.h>
#include <asm/netlogic/xlr_mac.h>
#include <asm/netlogic/xlr_user_mac.h>
#include <asm/netlogic/gpio.h>
#include <asm/netlogic/proc.h>
#include <user/netlogic/xlr_user_mac.h>

#include <asm/mach-netlogic/mmu.h>

#define MB(x)	(x<<20)
#define	KB(x)	(x<<10)
#ifdef dbg_msg
#undef dbg_msg
#endif
#define dbg_msg(fmt, args...) //printk(fmt, ##args)
#define Message(a, b...) //printk("\nFunc[%s], Line[%d], "a"\n",__FUNCTION__,__LINE__,##b)

/* this flag will be set by netlogic spi4 driver, it indicates
   whether spi4 daughter cards are present on the board or not
*/
#ifdef CONFIG_NLMCOMMON_SPI4
extern unsigned int g_spi4_card_flag;
extern void spi4_enable_tx_rx(unsigned int  *mmio);
extern void spi4_disable_tx_rx(unsigned int  *mmio);
#else /* CONFIG_NLMCOMMON_SPI4 */
static unsigned int g_spi4_card_flag;
#endif /* CONFIG_NLMCOMMON_SPI4 */

//#define USER_MAC_LOOPBACK

extern void *nlm_common_psb_shm;
extern unsigned long nlm_common_psb_shm_size;

static unsigned long long nlm_common_psb_shm_paddr;
static int user_mac_major;
struct user_mac_data *user_mac;
struct user_mac_kernal_data user_mac_krnl_data;
/* use per cpu data structures for the Queues below */
extern int xlr_hybrid;
extern struct proc_dir_entry *nlm_root_proc;

#define USER_MAC_TX_THRESHOLD 1

#define MAC_CRC_LEN 4
#define BYTE_OFFSET 2
#define MAC_PREPAD_LEN 32

#define MAC_DEV_NULL_BUCKET 127

struct usermac_priv
{
	int num_desc;
	int type;
	nlm_reg_t *mmio;	
	void*	frin_spill;
	void*	frout_spill;
	void*	class_0_spill;
	void*	class_1_spill;
	void*	class_2_spill;
	void*	class_3_spill;
};

struct usermac_dev {
	uint32_t gmac_list;
	int xgmac_present;
	int spi4_present;
	struct usermac_priv priv[32];
};

static struct usermac_dev usermac_dev;
static int hybrid_mem_init = 1;

/*Hugetlb usermac data structures*/
static struct page **page_array;
static void *htlb_kvaddr = NULL;
static phys_t htlb_kpaddr= 0ULL;
static void user_mac_mem_init(void);


static unsigned long long user_mac_vaddr_to_phys(void *addr)
{
	if(htlb_kvaddr){
		return ((unsigned long)addr - (unsigned long)htlb_kvaddr + htlb_kpaddr);
	}else{
		return __pa(addr);
	}
}

static __inline__ int user_mac_ptr2index(unsigned long long addr)
{
	int index = (addr - user_mac_vaddr_to_phys(user_mac->pkt_data)) / USER_MAC_PKT_BUF_SIZE;

	if (index < 0 || index >= MAX_USER_MAC_PKTS) {
		printk("[%s]: bad index=%d, addr=%llx, pkt_data=%p\n",
		       __FUNCTION__, index, addr, &user_mac->pkt_data);
		return -1;
	}
	return index;
}

#ifdef MAC_TX_DESC_ALIGNMENT
#undef MAC_TX_DESC_ALIGNMENT
#endif
#define MAC_TX_DESC_ALIGNMENT (SMP_CACHE_BYTES + MAC_PREPAD_LEN - 1)

#define CTRL_RES0           0
#define CTRL_RES1           1
#define CTRL_REG_FREE       2
#define CTRL_CONT           4
#define CTRL_EOP            5
#define CTRL_START          6
#define CTRL_SNGL           7

static __inline__ int user_mac_send_frin_num_xgs_pkts(void)
{
	int num_xgmac_pkts = MAX_USER_MAC_FRIN_PKTS / 2.5;

	return num_xgmac_pkts;
}

static __inline__ int user_mac_send_frin_num_gmac_pkts(void)
{
	return user_mac_send_frin_num_xgs_pkts() / 2;
}

static __inline__ int user_mac_send_frin_is_desc_gmac(int index)
{
	int start_index = 0, end_index = 0;

	start_index = 0;
	end_index = start_index + user_mac_send_frin_num_gmac_pkts();
	return ( (index >= start_index) && (index < end_index) ) ? 1 : 0;
}

static __inline__ int user_mac_send_frin_is_desc_xgs0(int index)
{
	int start_index = 0, end_index = 0;

	start_index = user_mac_send_frin_num_gmac_pkts();
	end_index = start_index + user_mac_send_frin_num_xgs_pkts();
	return ( (index >= start_index) && (index < end_index) ) ? 1 : 0;
}

static __inline__ int user_mac_send_frin_is_desc_xgs1(int index)
{
	int start_index = 0, end_index = 0;

	start_index = user_mac_send_frin_num_gmac_pkts() + user_mac_send_frin_num_xgs_pkts();
	end_index = start_index + user_mac_send_frin_num_xgs_pkts();
	return ( (index >= start_index) && (index < end_index) ) ? 1 : 0;
}

static __inline__ int user_mac_send_frin_stid(int index)
{
	if (is_xls()) {
		/* If only gmac-block 0 is active , Or only gmac-block1 is active */
		if((usermac_dev.gmac_list & 0x0f) && (!(usermac_dev.gmac_list & 0xf0)))
			return MSGRNG_STNID_GMAC0_FR;
		else if((usermac_dev.gmac_list & 0xf0) && (!(usermac_dev.gmac_list & 0x0f)))
			return MSGRNG_STNID_GMAC1_FR;
		if((index >= 0) && (index < (MAX_USER_MAC_FRIN_PKTS / 2)))
			return MSGRNG_STNID_GMAC0_FR;
		return MSGRNG_STNID_GMAC1_FR;
	}
	/* xlr case */
	if(usermac_dev.gmac_list & 0x0f) {
		switch(usermac_dev.xgmac_present | usermac_dev.spi4_present) {
			case 0x03:
				if (user_mac_send_frin_is_desc_gmac(index)) return MSGRNG_STNID_GMACRFR_0;
				if (user_mac_send_frin_is_desc_xgs0(index)) return MSGRNG_STNID_XMAC0RFR;
				if (user_mac_send_frin_is_desc_xgs1(index)) return MSGRNG_STNID_XMAC1RFR;
				return MSGRNG_STNID_GMACRFR_0;
			case 0x01:
				if((index >= 0) && (index < (MAX_USER_MAC_FRIN_PKTS / 4)))
					return MSGRNG_STNID_GMACRFR_0;
				return MSGRNG_STNID_XMAC0RFR;
			case 0x02:
				if((index >= 0) && (index < (MAX_USER_MAC_FRIN_PKTS / 4)))
					return MSGRNG_STNID_GMACRFR_0;
				return MSGRNG_STNID_XMAC1RFR;
			default:
				return MSGRNG_STNID_GMACRFR_0;
		}
	} else {
		/* xgmac should be present, this function should not be called when
                   all the gmac & xgmac stations are not owned by linux */
		switch(usermac_dev.xgmac_present | usermac_dev.spi4_present) {
			case 0x03:
				if((index >= 0) && (index < (MAX_USER_MAC_FRIN_PKTS / 2)))
					return MSGRNG_STNID_XMAC0RFR;
				return MSGRNG_STNID_XMAC1RFR;
			case 0x01:
				return MSGRNG_STNID_XMAC0RFR;
			case 0x02:
				return MSGRNG_STNID_XMAC1RFR;
			default:
				return -1;


		}
	}
}

static void user_mac_send_frin(void)
{
	struct msgrng_msg msg;
	struct packet_data *packet_data = user_mac->pkt_data;
	int i = 0;
	int cnt[4] = { 0, 0, 0, 0 };
	uint64_t addr = 0;
	unsigned long mflags = 0;
	int stid = 0;
	int host_gen_num_pkts = (MAX_USER_MAC_PKTS - MAX_USER_MAC_FRIN_PKTS) / 32;

	if(usermac_dev.gmac_list == 0 && usermac_dev.xgmac_present == 0 &&
			usermac_dev.spi4_present == 0)
		return;

	
	msgrng_flags_save(mflags);

	for (i = 0; i < MAX_USER_MAC_FRIN_PKTS; i++) {
		addr = user_mac_vaddr_to_phys(&packet_data[i].data);
		user_mac->pkt_desc[i].free = 0;

		msg.msg0 = (uint64_t) addr & ~(SMP_CACHE_BYTES - 1);
		msg.msg1 = msg.msg2 = msg.msg3 = 0;

		stid = user_mac_send_frin_stid(i);

		if (usermac_dev.spi4_present){
            		if((stid == MSGRNG_STNID_XMAC0RFR) && (!(g_spi4_card_flag & 0x01))){
                		/*spi4-0 card is not present*/
						Message("SPI4-0 Card not present - %d index not send",i);
                		continue;
            		}
            		else if((stid == MSGRNG_STNID_XMAC1RFR) && (!(g_spi4_card_flag & 0x02))){
                		/*spi4-1 card is not present*/
						Message("SPI4-1 Card not present - %d index not send",i);
                		continue;
            		}
       	}

        	do {
            		if (!message_send_retry(1, MSGRNG_CODE_MAC, stid, &msg)){
							if(stid == MSGRNG_STNID_XMAC0RFR){
								Message("Index %d, Phys Addr [%#llx] sent to Station XGMAC0",i, (unsigned long long)(addr & ~(SMP_CACHE_BYTES - 1)));
								cnt[2]++;
							}else if(stid == MSGRNG_STNID_XMAC1RFR){
								Message("Index %d, Phys Addr [%#llx] sent to Station XGMAC1",i,(unsigned long long)(addr & ~(SMP_CACHE_BYTES - 1)));
								cnt[3]++;
							}else if(stid == MSGRNG_STNID_GMACRFR_0){
								Message("Index %d Phys Addr [%#llx] sent to Station GMAC0",i, (unsigned long long)(addr & ~(SMP_CACHE_BYTES - 1)));
								cnt[0]++;
							}else if(stid == MSGRNG_STNID_GMAC1_FR){
								Message("Index %d Phys Addr [%#llx] sent to Station GMAC1",i, (unsigned long long)(addr & ~(SMP_CACHE_BYTES - 1)));
								cnt[1]++;

							}else{
								Message("Index %d sent to unknown station!!! %d ",i,stid);
							}
						   	break;
					}
            		printk("[%s:%d]: retrying free_desc[%d] message send to stid=%d, [status gmac0=%d xmac0=%d xmac1=%d\n", 
                    		__FUNCTION__,
                    		__LINE__, i, stid, cnt[0], cnt[2], cnt[3]);
        	} while(1);
	
        	xlr_inc_counter(USER_MAC_FRIN);
    	}
	msgrng_flags_restore(mflags);
	printk("[%s]:...done[Free descriptors gmac0=%d gmac1=%d xgmac0=%d xgmac1=%d\n", __FUNCTION__,
				cnt[0], cnt[1], cnt[2], cnt[3]);

	for (i=0; i<32; i++) {
		user_mac->host_pkt_next_free[i] =
			MAX_USER_MAC_FRIN_PKTS + (i * host_gen_num_pkts);
	}

	for(i=MAX_USER_MAC_FRIN_PKTS; i<MAX_USER_MAC_PKTS; i++)
		user_mac->pkt_desc[i].free = 1;

	printk
	    ("[%s]: packet_data[first].data=%llx, packet_data[last].data=%llx\n",
	     __FUNCTION__, user_mac_vaddr_to_phys(&packet_data[0].data),
	     user_mac_vaddr_to_phys(&packet_data[MAX_USER_MAC_PKTS - 1].data));
}

static void user_mac_send_frin_xgmac(void)
{
	struct msgrng_msg msg;
	struct packet_data *packet_data = user_mac->pkt_data;
	int i = 0;
	unsigned long addr = 0, mflags = 0;
	int stid = 0;
	int cnt[2] = {0, 0 };
	int host_gen_num_pkts = (MAX_USER_MAC_PKTS - MAX_USER_MAC_FRIN_PKTS) / 32;
		
	if(usermac_dev.xgmac_present == 0 && usermac_dev.spi4_present == 0)
		return;

	msgrng_flags_save(mflags);

	for (i = 0; i < MAX_USER_MAC_FRIN_PKTS; i++) {
		
		stid = user_mac_send_frin_stid(i);
	
		if(stid == MSGRNG_STNID_XMAC0RFR)
			cnt[0]++;
		else if( stid == MSGRNG_STNID_XMAC1RFR)
			cnt[1]++;
		else
			continue;
		
		addr = user_mac_vaddr_to_phys(&packet_data[i].data);
		user_mac->pkt_desc[i].free = 0;

		msg.msg0 = (uint64_t) addr & ~(SMP_CACHE_BYTES - 1);
		msg.msg1 = msg.msg2 = msg.msg3 = 0;

        	do {
            		if (!message_send_retry(1, MSGRNG_CODE_MAC, stid, &msg)) break;
            		printk("[%s:%d]: retrying free_desc[%d] message send to stid=%d\n", 
                    		__FUNCTION__,
                    		__LINE__, i, stid);
        	} while(1);
	
        	xlr_inc_counter(USER_MAC_FRIN);
    	}
	msgrng_flags_restore(mflags);
	printk("[%s]:...done, Free descriptors xgmac0=%d xgmac1=%d\n", __FUNCTION__, cnt[0], cnt[1]);

	for (i=0; i<32; i++) {
		user_mac->host_pkt_next_free[i] =
			MAX_USER_MAC_FRIN_PKTS + (i * host_gen_num_pkts);
	}

	for(i=MAX_USER_MAC_FRIN_PKTS; i<MAX_USER_MAC_PKTS; i++)
		user_mac->pkt_desc[i].free = 1;

	printk
	    ("[%s]: packet_data[first].data=%llx, packet_data[last].data=%llx\n",
	     __FUNCTION__, user_mac_vaddr_to_phys(&packet_data[0].data),
	     user_mac_vaddr_to_phys(&packet_data[MAX_USER_MAC_PKTS - 1].data));
}

void nlm_common_user_mac_update_time(void)
{
	if (user_mac) {
		user_mac->time.lo++;
		if (!user_mac->time.lo)
			user_mac->time.hi++;
	}else {
		user_mac_krnl_data.time.lo++;
		if (!user_mac_krnl_data.time.lo)
			user_mac_krnl_data.time.hi++;
	}
		
	
}
void nlm_common_user_mac_update_ktime(void)
{
    if(user_mac) {
       user_mac->ktime = current_kernel_time(); 
     } else {
	user_mac_krnl_data.ktime = current_kernel_time();
	}

}
static int user_mac_open(struct inode *inode, struct file *filp)
{
	//printk("user_mac_open() invoked\n");

	filp->private_data = NULL;

	return 0;
}

static int user_mac_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long long shm_addr = nlm_common_psb_shm_paddr;
	unsigned long shm_size = nlm_common_psb_shm_size;
	unsigned long size = 0;
	unsigned long vm_size = vma->vm_end - vma->vm_start;

	dbg_msg
	    ("[%s]: shm_addr=%lx, shm_size=%lx, offset = %lx, vm_start=%lx, vm_size=%lx, vm_flags=%lx, "
	     "vm_page_prot=%lx\n", __FUNCTION__, shm_addr, shm_size, offset,
	     vma->vm_start, vm_size, vma->vm_flags,
	     pgprot_val(vma->vm_page_prot));

	if (vma->vm_start != (unsigned long)NLM_USER_MAC_MMAP_VIRT_START)
		return -EINVAL;

	if (!shm_addr)
		return -ENXIO;

	if (offset >= shm_size)
		return -ESPIPE;

	if (vma->vm_flags & VM_LOCKED)
		return -EPERM;

	size = shm_size - offset;
	if (vm_size > size)
		return -ENOSPC;

	vma->vm_flags |= (VM_RESERVED | VM_IO);

	if (remap_pfn_range
	    (vma, vma->vm_start, (shm_addr >> PAGE_SHIFT), size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

/*****************************************************************
 * Initialize GMAC
 *****************************************************************/
static void nlm_usermac_config_pde(struct usermac_priv *priv)
{
	int i = 0, cpu = 0, bucket = 0;
	__u64 bucket_map = 0;

	for (i = 0; i < 32; i++) {
		if (cpu_isset(i, cpu_online_map)) {
			cpu = cpu_logical_map(i);
			bucket = 4 + (((cpu >> 2) << 3) | (cpu & 0x03));
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

static void nlm_usermac_config_parser(struct usermac_priv *priv)
{
	/* Mark it as no classification 
	 * The parser extract is gauranteed to be zero with no classfication
	 */
	
	netlogic_write_reg(priv->mmio, R_L2TYPE_0, 0x01);
	
	/* configure the parser : L2 Type is configured in the bootloader */
	/* extract IP: src, dest protocol */
	netlogic_write_reg(priv->mmio, R_L3CTABLE,
			  (9 << 20) | (1 << 19) | (1 << 18) | (0x01 << 16) |
			  (0x0800 << 0));
	netlogic_write_reg(priv->mmio, R_L3CTABLE + 1,
			  (12 << 25) | (4 << 21) | (16 << 14) | (4 << 10));

	if (xlr_user_mac_l4_extract()) {
		/* extract TCP: src port, dest port */
		netlogic_write_reg(priv->mmio, R_L4CTABLE, (6 << 0));
		netlogic_write_reg(priv->mmio, R_L4CTABLE + 1,
				  (0 << 21) | (2 << 17) | (2 << 11) | (2 << 7));
		/* extract UDP: src port, dest port */
		netlogic_write_reg(priv->mmio, R_L4CTABLE + 2, (17 << 0));
		netlogic_write_reg(priv->mmio, R_L4CTABLE + 3,
				  (0 << 21) | (2 << 17) | (2 << 11) | (2 << 7));
	}
}

static void nlm_usermac_port_enable(struct usermac_priv *priv, int flag)
{
	uint32_t regval;
	int tx_threshold = 1518;


	netlogic_write_reg(priv->mmio, R_MAC_FILTER_CONFIG,
		(1 << O_MAC_FILTER_CONFIG__BROADCAST_EN) |
		(1 << O_MAC_FILTER_CONFIG__ALL_MCAST_EN) |
		(1 << O_MAC_FILTER_CONFIG__ALL_UCAST_EN) |
		(1 << O_MAC_FILTER_CONFIG__MAC_ADDR0_VALID));


	if (flag) {

		regval = netlogic_read_reg(priv->mmio, R_TX_CONTROL);
		regval |= (1 << O_TX_CONTROL__TxEnable) |
			(tx_threshold << O_TX_CONTROL__TxThreshold);

		netlogic_write_reg(priv->mmio, R_TX_CONTROL, regval);

		regval = netlogic_read_reg(priv->mmio, R_RX_CONTROL);
		regval |= 1 << O_RX_CONTROL__RxEnable;
		netlogic_write_reg(priv->mmio, R_RX_CONTROL, regval);

#ifdef CONFIG_NLMCOMMON_SPI4
		if(priv->type == TYPE_SPI4)
			spi4_enable_tx_rx((uint32_t *)priv->mmio);
#endif

	
	} else {

		if(priv->type == TYPE_SPI4)
			spi4_disable_tx_rx((uint32_t *)priv->mmio);

		regval = netlogic_read_reg(priv->mmio, R_TX_CONTROL);
		regval &= ~((1 << O_TX_CONTROL__TxEnable) |
			    (tx_threshold << O_TX_CONTROL__TxThreshold));

		netlogic_write_reg(priv->mmio, R_TX_CONTROL, regval);

		regval = netlogic_read_reg(priv->mmio, R_RX_CONTROL);
		regval &= ~(1 << O_RX_CONTROL__RxEnable);
		netlogic_write_reg(priv->mmio, R_RX_CONTROL, regval);

	}
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

static __inline__ void *nlm_usermac_config_spill(nlm_reg_t * mmio,
					      int reg_start_0, int reg_start_1,
					      int reg_size, int size)
{
	__u32 spill_size = CACHELINE_ALIGNED_ADDR(size);
	void *spill = cacheline_aligned_kmalloc(spill_size, GFP_KERNEL);
	__u64 phys_addr = 0;

	if (!spill) {
		panic("Unable to allocate memory for spill area!\n");
	}
	phys_addr = user_mac_vaddr_to_phys(spill);
	netlogic_write_reg(mmio, reg_start_0, (phys_addr >> 5) & 0xffffffff);
	netlogic_write_reg(mmio, reg_start_1, (phys_addr >> 37) & 0x07);
	netlogic_write_reg(mmio, reg_size, spill_size);

	return spill;
}

static void nlm_usermac_config_spill_area(struct usermac_priv *priv)
{
	/* */
	int max_frin_spill    = 0;
	int max_frout_spill   = 0;
	int max_class_0_spill = 0;
	int max_class_1_spill = 0;
	int max_class_2_spill = 0;
	int max_class_3_spill = 0;

	if(!priv->num_desc)
		return;

	max_frin_spill = priv->num_desc << 2;
	max_frout_spill = priv->num_desc << 2;

	max_class_0_spill = priv->num_desc;
	max_class_1_spill = priv->num_desc;
	max_class_2_spill = priv->num_desc;
	max_class_3_spill = priv->num_desc;


	priv->frin_spill =
		nlm_usermac_config_spill(priv->mmio,
				      R_REG_FRIN_SPILL_MEM_START_0,
				      R_REG_FRIN_SPILL_MEM_START_1,
				      R_REG_FRIN_SPILL_MEM_SIZE,
				      max_frin_spill *
				      sizeof(struct fr_desc));

	priv->class_0_spill =
		nlm_usermac_config_spill(priv->mmio,
				      R_CLASS0_SPILL_MEM_START_0,
				      R_CLASS0_SPILL_MEM_START_1,
				      R_CLASS0_SPILL_MEM_SIZE,
				      max_class_0_spill *
				      sizeof(union rx_tx_desc));
	priv->class_1_spill =
		nlm_usermac_config_spill(priv->mmio,
				      R_CLASS1_SPILL_MEM_START_0,
				      R_CLASS1_SPILL_MEM_START_1,
				      R_CLASS1_SPILL_MEM_SIZE,
				      max_class_1_spill *
				      sizeof(union rx_tx_desc));

	priv->frout_spill =
		nlm_usermac_config_spill(priv->mmio, R_FROUT_SPILL_MEM_START_0,
				      R_FROUT_SPILL_MEM_START_1,
				      R_FROUT_SPILL_MEM_SIZE,
				      max_frout_spill *
				      sizeof(struct fr_desc));

	priv->class_2_spill =
		nlm_usermac_config_spill(priv->mmio,
				      R_CLASS2_SPILL_MEM_START_0,
				      R_CLASS2_SPILL_MEM_START_1,
				      R_CLASS2_SPILL_MEM_SIZE,
				      max_class_2_spill *
				      sizeof(union rx_tx_desc));
	priv->class_3_spill =
		nlm_usermac_config_spill(priv->mmio,
				      R_CLASS3_SPILL_MEM_START_0,
				      R_CLASS3_SPILL_MEM_START_1,
				      R_CLASS3_SPILL_MEM_SIZE,
				      max_class_3_spill *
				      sizeof(union rx_tx_desc));
}


/*Translate user space vaddr to page*/
static struct page *user_vaddr_to_page(unsigned long addr, unsigned long size)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte = NULL;
	pte_t pteval;
	unsigned long pfn = 0;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma = NULL;

	/*Check whether address falls in user space or not*/
	if(addr >= PAGE_OFFSET){
		printk("\nInvalid Userspace address\n");
		return NULL;
	}
	
	/*Make sure addr doesn't overlap*/
	if(addr > (addr+size)){
		printk("\nAddress overlaps!!!\n");
		return NULL;
	}
	
	/*Make sure address is already present in VMA*/
	vma = find_vma(mm, addr+size-1);
	if(!vma){
		printk("\nNo VMA found!!!\n");
		return NULL;
	}
	
	if(vma->vm_start > addr){
		/*`addr` doesn't fall under vma*/
		printk("\nAddress doesn't fall under vma!!\n");
		printk("\nvma_start = %#lx, vma_end = %#lx\n",
						vma->vm_start, vma->vm_end);
		return NULL;
	}

#ifdef CONFIG_HUGETLBFS
	if(!is_vm_hugetlb_page(vma))
		return NULL;
#else
	return NULL;
#endif
	/*Acquire pagetable sem*/
	down_read(&mm->mmap_sem);
	pgd = pgd_offset(mm, addr);
	if (!pgd_none(*pgd)){
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)){
				pte = pte_offset_map(pmd, addr);
			}
		}
	}
	/*Release pagetable sem*/
	up_read(&mm->mmap_sem);

	if(!pte){
		printk("\nHugepage is not allocated yet\n");
		return NULL;
	}
	pteval = *pte;
	if(pte_none(pteval)){
		printk("\nPTE not allocated yet\n");
		return NULL;
	}
	pfn = (pte_val(pteval) & 0xffffffffffULL) >> PAGE_SHIFT;
	Message("User virt Addr [%#lx], PTE VAL [%#llx], pfn [%#lx]",
					(unsigned long)addr, (unsigned long long)pte_val(pteval),
					(unsigned long)pfn);
	return pfn_to_page(pfn);
}

int user_mac_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		   unsigned long arg)
{
    extern struct net_device_cfg xlr_net_dev_cfg;
    struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;
    extern cpumask_t phys_cpu_present_map;

	switch (cmd) {
	case USER_MAC_IOC_HUGETLB_SHM_VIRT_ADDR:{
			__u64 *ptr = (__u64 *) arg;
			__u64	user_vaddr;
			__u64	user_vaddr_len;
			int no_of_pages = 0;
			int i = 0;

			if(htlb_kvaddr)
				/*Usermac memory already initialized*/
				return 0;	
			
			copy_from_user(&user_vaddr, ptr, sizeof(*ptr));
			copy_from_user(&user_vaddr_len, ptr+1, sizeof(*ptr));

			if(user_vaddr_len < MB(8ULL)){
				printk("\nCan't init usermac buffers with < 8MB buffer\n");
				return -ENOMEM; 
			}else{
				no_of_pages = MB(8ULL)/KB(4);
				page_array = (struct page **)kmalloc
						(sizeof(struct page *)*(no_of_pages),GFP_KERNEL);
				if(!page_array){
					printk("\nCan't allocate memory for page_array\n");
					return -ENOMEM;
				}
				Message("User Space Virtual ADdress [%#lx]",(unsigned long)user_vaddr);
				for(i=0; i<no_of_pages; i++, user_vaddr+=KB(4)){
					page_array[i] = 
						user_vaddr_to_page((unsigned long)user_vaddr, KB(4));
					if(!page_array[i]){
						kfree(page_array);
						printk("\nuser_vaddr_to_page returned NULL!!\n");
						return -ENOMEM;
					}
				}
				/*vmap this page range in kernel address space*/
				htlb_kvaddr = vmap(page_array, no_of_pages, VM_MAP, PAGE_KERNEL);
				if(!htlb_kvaddr){
					printk("\nNot enough virtual address space available to map the hugepage in kernel\n");
					kfree(page_array);
					return -ENOMEM;
				}
				htlb_kpaddr = ((unsigned long long)vmalloc_to_pfn((const void *)htlb_kvaddr))<<PAGE_SHIFT;
				Message("HugeTlb Physical Address [%#llx]",(unsigned long long)htlb_kpaddr);
			}
			nlm_common_psb_shm = htlb_kvaddr;
			/*Init user mac descriptors now!!*/
			user_mac_mem_init();
		}
		break;
	case USER_MAC_IOC_EARLY_MEM_INIT:{
			put_user((unsigned int)hybrid_mem_init, (unsigned int*)arg);
		}
		break;
	case USER_MAC_IOC_GSHMPHYS:{
			put_user((unsigned long long)nlm_common_psb_shm_paddr,(unsigned long long*)arg);
		}
		break;

	case USER_MAC_IOC_GSHMVIRT:{
			put_user((unsigned long long)(unsigned long)nlm_common_psb_shm,(unsigned long long *)arg);
		}
		break;

	case USER_MAC_IOC_GSHMSIZE:{
			put_user((unsigned int)nlm_common_psb_shm_size,(unsigned int*)arg);
		}
		break;

	case USER_MAC_IOC_GMMAP_START:{
			put_user((unsigned int)NLM_USER_MAC_MMAP_VIRT_START,(unsigned int*)arg);
		}
		break;

	case USER_MAC_IOC_GREAD_REG:{
			__u32 *ptr = (__u32 *) arg;
			__u32 dev = 0, reg = 0, value = 0;
			nlm_reg_t *mmio = 0;

			get_user(dev, ptr + 0);
			get_user(reg, ptr + 1);

			if(dev < 31)
				mmio = usermac_dev.priv[dev].mmio;
			
			if(mmio == NULL || (reg>(0x1000>>2))){
				printk("[%s]: bad args, dev=0x%x, reg=0x%x\n",
				       __FUNCTION__, dev, reg);
				value = 0xdeadbeef;
			}else {
				printk("\nMMIO %#lx, REG %#lx\n",(unsigned long)mmio,(unsigned long)reg);
				printk("\nReading @ Address %#lx-->%#x\n",
						(unsigned long)&mmio[reg],
						mmio[reg]);
				value = netlogic_read_reg(mmio, reg);
				dbg_msg	
				    ("[%s]: dev=0x%x, reg=0x%x, value=0x%x\n",
				     __FUNCTION__, dev, reg, value);
			}
			put_user(value, ptr + 2);
		}
		break;

	case USER_MAC_IOC_SWRITE_REG:{
			__u32 *ptr = (__u32 *) arg;
			__u32 dev = 0, reg = 0, value = 0;
			nlm_reg_t *mmio = 0;

			get_user(dev, ptr + 0);
			get_user(reg, ptr + 1);
			get_user(value, ptr + 2);
			if(dev < 31)
				mmio = usermac_dev.priv[dev].mmio;
			if(mmio == NULL || (reg>(0x1000>>2))){
				printk("[%s]: bad args, dev=0x%x, reg=0x%x\n",
				       __FUNCTION__, dev, reg);
			} else{
				dbg_msg
				    ("[%s]: dev=0x%x, reg=0x%x, value=0x%x\n",
				     __FUNCTION__, dev, reg, value);

				netlogic_write_reg(mmio, reg, value);
			}

		}
		break;

	case USER_MAC_IOC_GPHYS_CPU_PRESENT_MAP:{
			put_user((unsigned int)phys_cpu_present_map.bits[0],(unsigned int*)arg);
		}
		break;

	case USER_MAC_IOC_GCPU_ONLINE_MAP:{
			put_user((unsigned int)cpu_online_map.bits[0],(unsigned int*)arg);
		}
		break;
	case USER_MAC_IOC_HYBRID_MODE_SETUP:{
			if(xlr_hybrid_user_mac()){
				if(net_cfg->xgs_type[0] == TYPE_XGMAC || net_cfg->xgs_type[1] == TYPE_XGMAC){
					/*ATX-II*/
					put_user((unsigned int)XLR_HYBRID_USER_MAC_GMAC_XGMAC,
								   	(unsigned int*)arg);
				}
				else if(net_cfg->xgs_type[0] == TYPE_SPI4 || net_cfg->xgs_type[1] == TYPE_SPI4){
					/*ATX-I*/
					put_user((unsigned int)XLR_HYBRID_USER_MAC_GMAC_SPI4,
								   	(unsigned int*)arg);
				}
				else{
					/*All remaining XLR and XLS boards.*/
					put_user((unsigned int)XLR_HYBRID_USER_MAC_GMAC,
								   	(unsigned int*)arg);
				}
			}
			else{
				put_user((unsigned int)xlr_hybrid, (unsigned int*)arg);
			}
		}
		break;
	default:{
			printk("ioctl(): invalid command=0x%x\n", cmd);
			//return -EINVAL;
			return 0;
		}

	}

	return 0;
}

long user_mac_compat_ioctl(struct file *filp, unsigned int cmd,
	       			unsigned long arg)
{
	unsigned long ret = -1;
	lock_kernel();	
	ret = user_mac_ioctl(NULL,filp,cmd,arg);
	unlock_kernel();
	if(ret){
		printk("user_mac_ioctl returned with an error.\n");
		return -ENOIOCTLCMD;
	}
	return ret;
}

  // called only when the reference count (maintained in inode) is zero
static int user_mac_release(struct inode *inode, struct file *filp)
{

	return 0;
}

struct file_operations user_mac_fops = {
	.mmap = user_mac_mmap,
	.open = user_mac_open,
	.ioctl = user_mac_ioctl,
	.compat_ioctl = user_mac_compat_ioctl,
	.release = user_mac_release,
};

static int proc_read_count;

static int user_mac_proc_read(char *page, char **start, off_t off,
			      int count, int *eof, void *data)
{
	int len = 0;
	off_t begin = 0;
	struct user_mac_time *time;
	if (user_mac)
		time = &user_mac->time;
	else
		time = &user_mac_krnl_data.time;

	proc_read_count++;

	len += sprintf(page + len,
		       "\n*************** USER MAC STATISTICS ****************\n"
		       "cpu_%d: proc_read_count = %d\n",
		       smp_processor_id(), proc_read_count);
	if (!proc_pos_check(&begin, &len, off, count))
		goto out;

	len +=
	    sprintf(page + len,
		    "\nshm_paddr=%llx, shm_size=%lx, mmap_virt_start=%x\n"
		    "sizeof(user_mac_data)=0x%x\n", nlm_common_psb_shm_paddr,
		    nlm_common_psb_shm_size, NLM_USER_MAC_MMAP_VIRT_START,
		    (unsigned int)sizeof(struct user_mac_data));
	if (!proc_pos_check(&begin, &len, off, count))
		goto out;

	len +=
	    sprintf(page + len,
		    "\noffsetof(time)=0x%x, time.hi=%u, time.lo=%u\n",
		    (unsigned int)offsetof(struct user_mac_data, time), 
			(unsigned int)time->hi,
		    (unsigned int)time->lo);
	if (!proc_pos_check(&begin, &len, off, count))
		goto out;

	len += sprintf(page + len, "\n");
	if (!proc_pos_check(&begin, &len, off, count))
		goto out;

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

extern struct xlr_user_mac_config xlr_user_mac;

static int __init xlr_user_mac_setup(char *str)
{
	if ( (strcmp(str, "=fast_syscall") == 0) || (strcmp(str, "fast_syscall") == 0)) {
		xlr_user_mac.fast_syscall = 1;
		printk("XLR: user_mac configured with fast syscalls\n");
	} else {
		printk("XLR: user_mac configured with unknown args \"%s\"\n", str);
	}

	return 1;
}

early_param("xlr_user_mac", xlr_user_mac_setup);

static void user_mac_mem_init(void)
{
	int i = 0;
	static int init_mem = 0;

	if(init_mem){
		printk("\nUser Mac Memory is already initialized\n");	
		return;
	}
	
	user_mac = (struct user_mac_data *)nlm_common_psb_shm;
	if (!user_mac) {
		printk("[%s]: Null Shared Memory Pointer?\n", __FUNCTION__);
		printk("\nInvalid user mac shared memory !!\n");
		return;
	}
	printk("\nUserMac Data structures Starts @ %#lx\n",(unsigned long)nlm_common_psb_shm);
	if (sizeof(struct user_mac_data) > nlm_common_psb_shm_size) {
		printk("[%s]: psb shared memory is too small: user_mac_data=0x%x, psb_shm_size=0x%lx\n", __FUNCTION__, (unsigned int)sizeof(struct user_mac_data),
		     (unsigned long)nlm_common_psb_shm_size);
		printk("User Mac Memory initialization failed!!\n");
		return;
	}

	for(i=0;i<MAX_USER_MAC_PKTS;i++)
		user_mac->pkt_desc[i].free = 1;

	if(htlb_kpaddr){
		nlm_common_psb_shm_paddr = htlb_kpaddr;
	}else{
		nlm_common_psb_shm_paddr = user_mac_vaddr_to_phys(nlm_common_psb_shm);
	}

	if (xlr_hybrid_user_mac()){
		Message("Calling User Mac Send FRIN ");
		user_mac_send_frin();
	}
	else if(xlr_hybrid_user_mac_xgmac())
		user_mac_send_frin_xgmac();

	for(i = 0; i < NETLOGIC_MAX_MACS; i++) {
		if(usermac_dev.priv[i].mmio != 0)
			nlm_usermac_port_enable(&usermac_dev.priv[i], 1);	
	}
	init_mem = 1;
	return;
}


#ifdef CONFIG_HUGETLBFS
static int __init xlr_hybrid_early_init(char *str)
{
	if(HPAGE_SIZE < MB(8)){
		printk("Hugetlb user mac is not supported with < 8MB huge page size\n");
		return 0;
	}
	if (strcmp(str,"no")==0){
		hybrid_mem_init = 0;
	}
	return 0;
}
early_param("xlr_hybrid_early_init", xlr_hybrid_early_init);
#endif

static int user_mac_init(void)
{
	int i = 0, next = 0;
	struct proc_dir_entry *entry;
	extern struct net_device_cfg xlr_net_dev_cfg;
	struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;

	usermac_dev.gmac_list = 0;
	usermac_dev.xgmac_present = 0;
	usermac_dev.spi4_present = 0;

	if (xlr_hybrid_user_mac())	{
		for(i = 0; i < NETLOGIC_MAX_GMACS; i++) {
			if(net_cfg->gmac_port[i].mmio_addr == 0 || net_cfg->gmac_port[i].cfg_flag == 0)
				continue;
			usermac_dev.gmac_list |= (1 << i);
			usermac_dev.priv[i].mmio = (void *)net_cfg->gmac_port[i].mmio_addr;
			usermac_dev.priv[i].type = TYPE_GMAC;

			/* Need to call only once, num_desc will be nonzero for only the first
                           port of every gmac block */
			if(net_cfg->gmac_port[i].num_desc != 0) { 
				usermac_dev.priv[i].num_desc = MAX_USER_MAC_FRIN_PKTS;
				nlm_usermac_config_spill_area(&usermac_dev.priv[i]);
				nlm_usermac_config_parser(&usermac_dev.priv[i]);
				nlm_usermac_config_pde(&usermac_dev.priv[i]);
			}		
			next = i + 1;
		}
		if(usermac_dev.gmac_list == 0) 
			printk("Skipping usermac configuration on gmac ports..\n");
	}

	if (xlr_hybrid_user_mac() || xlr_hybrid_user_mac_xgmac())	{
		for(i = 0; i < NETLOGIC_MAX_XGMACS; i++) {
			if(net_cfg->xgs_port[i].mmio_addr == 0 || net_cfg->xgs_port[i].cfg_flag == 0)
				continue;
			if(net_cfg->xgs_type[i] == TYPE_XGMAC)
				usermac_dev.xgmac_present |= (1  << i);
			else
				usermac_dev.spi4_present |= (1 << i);
			usermac_dev.priv[next + i].mmio = (void *)net_cfg->xgs_port[i].mmio_addr;
			usermac_dev.priv[next + i].type = net_cfg->xgs_type[i];
			usermac_dev.priv[next + i].num_desc = MAX_USER_MAC_FRIN_PKTS;
			nlm_usermac_config_spill_area(&usermac_dev.priv[next + i]);
			nlm_usermac_config_parser(&usermac_dev.priv[next + i]);
			nlm_usermac_config_pde(&usermac_dev.priv[next + i]);
		}
		if(usermac_dev.xgmac_present == 0 && usermac_dev.spi4_present == 0) 
			printk("Skipping usermac configuration on xgmac ports..\n");
	}

	if(usermac_dev.gmac_list != 0 || usermac_dev.xgmac_present != 0 ||
			usermac_dev.spi4_present != 0) {
		user_mac_major =
			register_chrdev(XLR_USER_MAC_MAJOR, NLM_USER_MAC_CHRDEV_NAME, &user_mac_fops);
		if (user_mac_major < 0) {
			printk("user_mac_init() register_chrdev() failed\n");
			return user_mac_major;
		}
		printk("Registered user_mac driver: major=%d\n", XLR_USER_MAC_MAJOR);
	}

	entry = create_proc_read_entry(NLM_USER_MAC_CHRDEV_NAME, 0 /* def mode */ ,
				       nlm_root_proc /* parent directory*/ ,
                                       user_mac_proc_read
				       /* proc read function */ ,
				       0	/* no client data */
		);

	if (!entry) {
		printk("[%s]: Unable to create proc read entry for %s!\n",
		       __FUNCTION__, NLM_USER_MAC_CHRDEV_NAME);
	}

	if(hybrid_mem_init){
		user_mac_mem_init();
		return 0;
	}
	printk("Memory init for hybrid mode is not done.\n");
	return 0;
}

static void user_mac_exit(void)
{
	unregister_chrdev(user_mac_major, NLM_USER_MAC_CHRDEV_NAME);
}
static int user_mac_mem(char *str)
{

	if ( !(xlr_hybrid_user_mac()) && !(xlr_hybrid_user_mac_xgmac()))
		return 0;

	nlm_common_psb_shm = alloc_bootmem_low( NLM_USER_MAC_SIZE );
	nlm_common_psb_shm_size = NLM_USER_MAC_SIZE;
	return 0;
}

module_init(user_mac_init);
__setup("xlr_hybrid",user_mac_mem);
module_exit(user_mac_exit);
