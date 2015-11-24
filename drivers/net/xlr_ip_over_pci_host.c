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

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cache.h>
#include <linux/delay.h>
#include <linux/timer.h>


#ifdef CONFIG_NLM_COMMON
#include <asm/netlogic/nlm_pcix_gen_host.h>
#else
#include "nlm_pcix_gen_host.h"
#endif

#define DRV_NAME	"nlm_ip_over_pci"
#define DRV_VERSION	"0.1"
#define DRV_RELDATE	"10Feb2004"

#define XLR_TX_DESC 512 
#define XLR_RX_DESC 512  
#define RX_DESC_SIZE (sizeof(struct xlr_rcv_desc) / sizeof(unsigned int)) 
#define TX_DESC_SIZE (sizeof(struct xlr_xmit_desc) / sizeof(unsigned int)) 
#define NLM_SMP_CACHE_BYTES 32

#define Message(a,b...) //printk("\n[%s] - "a"\n",__FUNCTION__,##b)
#define ErrorMsg(a,b...) printk("\nError in [%s] - "a"\n",__FUNCTION__,##b)
#define XLR_MAX_RX_LEN 1536

static struct net_device *xlr_netdev= NULL;
static unsigned int *xlr_ip_over_pci_base = NULL;
typedef volatile unsigned int xlr_reg_t;
static struct sk_buff *tx_skb_ptr[XLR_TX_DESC];
static void setup_shared_mem(void);
static struct sk_buff * xlr_alloc_skb(void);
static int xlr_replenish_buffer(void);
static int replenish_buffer_at_index(int index);
static int process_rx_desc(void);
static void process_tx_ok_desc(void);
static void free_rx_desc(void);
static void free_rx_desc_index(int i);
static void free_tx_desc(void);

extern unsigned volatile long nlm_common_get_shared_mem_base_host(void);
extern int xlr_init_dma(void);
extern int xlr_request_dma(uint64_t src, uint64_t dest, uint32_t len);
extern void nlm_nlm_common_interrupt_device(void);


#define XLR_GET_OWN(x) (((x)->info) & 0x80000000)
#define XLR_RESET_OWN(x) ((x)->info = ((x)->info) & 0x7fffffff)
#define XLR_SET_OWN(x) ((x)->info = (x)->info | 0x80000000)
#define XLR_GET_HOST_PHYS(x) ((x)->addr)
#define XLR_GET_LEN(x) ((x)->info & 0x3fff)
#define XLR_SET_LEN(x,len) (x)->info = ((x)->info & ~(0x3fff));\
		   	    (x)->info |= len
#define XLR_SET_HOST_PHYS(x,y) ((x)->addr = (y))

#define XLR_INTERFACE_IS_UP 1
#define XLR_INTERFACE_IS_DOWN 2
#define XLR_MAGIC_NO 0xdeadbeef
struct xlr_xmit_desc{
	uint32_t addr;
	uint32_t info;/*  Bit 0 to 14 LEN
    		 	  Bit 15 to 30 Reserved 
			  Bit 31 OWN						                       */

};
struct xlr_rcv_desc{
	uint32_t addr;
	uint32_t info;/*  Bit 0 to 15 LEN
    		 	  Bit 16 to 30 RESERVED
			  Bit 31 OWN						                       */

};
struct tx_free_info
{
        struct sk_buff *tx_skb_ptr;
        dma_addr_t phys_addr;
        int len;
}xlr_tx_free_info[XLR_TX_DESC];

struct rx_free_info
{
        struct sk_buff *skb;
        dma_addr_t phys_addr;
        int len;
}xlr_rx_info[XLR_RX_DESC];
extern struct pci_dev *nlm_pdev;

static struct net_device_ops nlm_host_net_ops;
/*DEBUG CNTR START*/
static int xlr_tx_pkt;
static int xlr_rx_pkt;
static int xlr_tx_enqueue_failed;
/*DEBUG CNTR END*/

static spinlock_t xlr_tx_ok_sync = SPIN_LOCK_UNLOCKED;
static volatile int xlr_queue_is_stop=0;

// xlr_xmit_desc - Xmit Pkt towards host from xlr. Host has to give its memory address where xlr can xmit.
static volatile struct xlr_xmit_desc *xlr_xmit_desc_base;

static volatile struct xlr_rcv_desc *xlr_rcv_desc_base;

static unsigned int xmit_producer;
static unsigned int xmit_consumer;
static unsigned int rx_consumer;
#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
static int msi_index;
#endif
static volatile uint32_t *xlr_magic_no;
static volatile uint32_t *xlr_host_status;
static volatile uint32_t *xlr_dev_status;
static volatile uint32_t xlr_host_status_local;

extern unsigned int nlm_common_pci_readl(unsigned int  *base);
extern void nlm_common_pci_writel(unsigned int  data,unsigned int *addr);
static struct driver_data{
	struct net_device *dev;	
	struct xlr_xmit_desc *curr_xmit;
	struct xlr_xmit_desc *xmit_base;
	struct xlr_rcv_desc *curr_rx;
	struct xlr_rcv_desc *rx_base;
}*ip_over_pci_priv;

volatile static int rmmod_is_set=0;
static struct timer_list xlr_link_status_timer;

static void nlm_pci_read(u32 *src, int words, u32 *dest)
{
  int i;

  for(i=0; i<words; i++){
      *(dest + i) = nlm_common_pci_readl(src+i);
  }
  return;
}

static void nlm_pci_write(u32 *src, int words, u32 *dest)
{
  int i;
  for(i=0; i<words; i++){
	nlm_common_pci_writel(*(src+i), dest + i);
  }
  return;
}

static int process_rx_desc(void)
{
	volatile struct xlr_rcv_desc *curr_rcv_desc;
	struct xlr_rcv_desc tmp_rx;
	uint32_t phys_addr=0;
	struct sk_buff *skb;
	int data_len;
	unsigned int status;
  int count = 0x0;

	curr_rcv_desc = xlr_rcv_desc_base + rx_consumer;
	nlm_pci_read((uint32_t *)curr_rcv_desc,RX_DESC_SIZE,
			(uint32_t *)&tmp_rx);
	while(!XLR_GET_OWN(&tmp_rx)){
                pci_unmap_single(nlm_pdev,xlr_rx_info[rx_consumer].phys_addr,
                                xlr_rx_info[rx_consumer].len,DMA_FROM_DEVICE);

		count++;
		data_len = XLR_GET_LEN(&tmp_rx);
		if(data_len > 1514){
			ErrorMsg("Rcvd Wrong Len Pkt %d",data_len);
		}
		Message("\nRcvd A Pkt Frm XLR With LEN %d, Addr %#x Len %#x\n",
					data_len,tmp_rx.addr,tmp_rx.info);
                skb = xlr_rx_info[rx_consumer].skb;
		if(!skb){
			panic("xlr_ip_over_pci.c - Rcvd skb is null\n");
		}	
		if(replenish_buffer_at_index(rx_consumer)){
			Message("Droppin Packet As replenishment is failed");
			phys_addr = pci_map_single(nlm_pdev,skb->data,
						XLR_MAX_RX_LEN,DMA_FROM_DEVICE);
			xlr_rx_info[rx_consumer].phys_addr = phys_addr;
        		xlr_rx_info[rx_consumer].skb = skb;
		        xlr_rx_info[rx_consumer].len = XLR_MAX_RX_LEN;

			XLR_SET_HOST_PHYS(&tmp_rx,phys_addr);
			XLR_SET_OWN(&tmp_rx);
			nlm_pci_write((uint32_t *)&tmp_rx,RX_DESC_SIZE,
				(uint32_t *)curr_rcv_desc);	
			rx_consumer = (rx_consumer + 1)% XLR_RX_DESC;
			break;
		}
		skb_put(skb,data_len);
		skb->dev = xlr_netdev;
		skb->protocol = eth_type_trans(skb,skb->dev);

		if((status = netif_rx(skb)) == NET_RX_DROP){
			ErrorMsg("netif rx failed with status %d",status);	
		}
		xlr_rx_pkt++;
		rx_consumer = (rx_consumer + 1)% XLR_RX_DESC;
		curr_rcv_desc = xlr_rcv_desc_base + rx_consumer;
		nlm_pci_read((uint32_t *)curr_rcv_desc,RX_DESC_SIZE,
				(uint32_t *)&tmp_rx);
	}
	return count;
}

static void process_tx_ok_desc()
{
	volatile struct xlr_xmit_desc *curr_xmit = NULL; 
	struct xlr_xmit_desc tmp_xmit;

	if(xmit_consumer == xmit_producer)
		return;
	curr_xmit = xlr_xmit_desc_base + xmit_consumer;
	nlm_pci_read((uint32_t *)curr_xmit+1,TX_DESC_SIZE-1,
			(uint32_t *)&tmp_xmit.info);
	
	while((!XLR_GET_OWN(&tmp_xmit)) && (xmit_consumer != xmit_producer)){
		Message("Rcvd Tx Ok");
                pci_unmap_single(nlm_pdev,
                        xlr_tx_free_info[xmit_consumer].phys_addr,
                        xlr_tx_free_info[xmit_consumer].len,
                        DMA_TO_DEVICE);
#if defined(CONFIG_NLM_COMMON) || !defined(XLR_MSI_IS_SUPPORTED)
                dev_kfree_skb(xlr_tx_free_info[xmit_consumer].tx_skb_ptr);
#else
                dev_kfree_skb_irq(xlr_tx_free_info[xmit_consumer].tx_skb_ptr);
#endif
		xmit_consumer = (xmit_consumer + 1) % XLR_TX_DESC;
		curr_xmit = xlr_xmit_desc_base + xmit_consumer;
		xlr_tx_pkt++;
		nlm_pci_read((uint32_t *)curr_xmit+1,TX_DESC_SIZE-1,
			(uint32_t *)&tmp_xmit.info);
		spin_lock(&xlr_tx_ok_sync);
		if(xlr_queue_is_stop){
			netif_start_queue(xlr_netdev);
			xlr_queue_is_stop = 0;
		}
		spin_unlock(&xlr_tx_ok_sync);
	}
}

#if defined(CONFIG_NLM_COMMON) || !defined(XLR_MSI_IS_SUPPORTED)
static void ip_over_pci_rx(struct work_struct *data);
static DECLARE_DELAYED_WORK(ip_over_pci_task_host,ip_over_pci_rx);
static void ip_over_pci_rx(struct work_struct *data)
#else
static irqreturn_t ip_over_pci_rx (void *data,struct pt_regs *regs)
#endif
{
	uint32_t dev_status , count;
	nlm_pci_read((uint32_t *)xlr_dev_status,1,&dev_status);

	if(dev_status == XLR_INTERFACE_IS_UP){
		process_tx_ok_desc();	
		count = process_rx_desc();
	}
#if defined(CONFIG_NLM_COMMON) || !defined(XLR_MSI_IS_SUPPORTED)
	if(!rmmod_is_set) {
		schedule_delayed_work(&ip_over_pci_task_host, 1);
	}
	return;
#else
	return IRQ_HANDLED;
#endif
}

static void put_dummy_mac_address(struct net_device *dev)
{
	dev->dev_addr[0] = 0x0;
	dev->dev_addr[1] = 0xb;
	dev->dev_addr[2] = 0x0;
	dev->dev_addr[3] = 0xb;
	dev->dev_addr[4] = 0x0;
	dev->dev_addr[5] = 0xb;
}

static int xlr_ip_over_pci_open(struct net_device *dev)
{
	uint32_t tmp = XLR_INTERFACE_IS_UP;
	xlr_host_status_local = XLR_INTERFACE_IS_UP;
	nlm_pci_write((uint32_t *)&tmp,1,(uint32_t *)xlr_host_status);
	return 0;
}

static int xlr_ip_over_pci_close(struct net_device *dev)
{
	uint32_t tmp = XLR_INTERFACE_IS_DOWN;
	if(!netif_queue_stopped(dev))
		netif_stop_queue(dev);
	xlr_host_status_local = XLR_INTERFACE_IS_DOWN;
	nlm_pci_write((uint32_t *)&tmp,1,(uint32_t *)xlr_host_status);
	return 0;
}

static int xlr_ip_over_pci_xmit (struct sk_buff *skb,struct net_device *dev)
{
	volatile struct xlr_xmit_desc *curr_xmit_desc;
	struct xlr_xmit_desc tmp_xmit;
	uint64_t phys_addr=0;
	uint32_t dev_status;
	unsigned long mflags;

	nlm_pci_read((uint32_t *)xlr_dev_status,1,&dev_status);

	if(dev_status != XLR_INTERFACE_IS_UP){
		netif_stop_queue(xlr_netdev);
		return -EIO;
	}
	
	if(skb->len > 1514){
		ErrorMsg("Pkt Iz Greater Than Max Size %d",skb->len);
		return -EIO;
	}
	if(((xmit_producer + 1)%(XLR_TX_DESC)) == xmit_consumer){
		Message("No TX Desc Available.");
		spin_lock_irqsave(&xlr_tx_ok_sync,mflags);
	        if(((xmit_producer + 1)%(XLR_TX_DESC)) != xmit_consumer){
			spin_unlock_irqrestore(&xlr_tx_ok_sync,mflags);
			goto try_again;
		}
		xlr_queue_is_stop=1;
		netif_stop_queue(xlr_netdev);
		spin_unlock_irqrestore(&xlr_tx_ok_sync,mflags);
		xlr_tx_enqueue_failed++;
		return -ENOMEM;
	}
try_again:
	curr_xmit_desc = xlr_xmit_desc_base + xmit_producer;
	Message("\nCurrXmitDescBase %#x, xmit_prod %d, xlr_xmit_desc_base %#x\n",curr_xmit_desc,xmit_producer,xlr_xmit_desc_base);
	nlm_pci_read((uint32_t *)curr_xmit_desc, TX_DESC_SIZE,
				 (uint32_t *)&tmp_xmit);
	if(XLR_GET_OWN(&tmp_xmit)){
		Message("All Xmit Desc are full...");
		return -ENOMEM;
	}
	tx_skb_ptr[xmit_producer] = skb;
        phys_addr = xlr_tx_free_info[xmit_producer].phys_addr =
                pci_map_single(nlm_pdev,skb->data,skb->len,DMA_TO_DEVICE);
        xlr_tx_free_info[xmit_producer].tx_skb_ptr = skb;
	memset(&tmp_xmit,0,sizeof(tmp_xmit));
	XLR_SET_HOST_PHYS(&tmp_xmit,phys_addr);
	XLR_SET_LEN(&tmp_xmit,skb->len);
	XLR_SET_OWN(&tmp_xmit);
	Message("\nXmittin Pkt With Len %d\n",XLR_GET_LEN(&tmp_xmit));
	nlm_pci_write((uint32_t *)&tmp_xmit,TX_DESC_SIZE,
			(uint32_t *)curr_xmit_desc);
	xmit_producer = (xmit_producer + 1) % XLR_TX_DESC;
	nlm_nlm_common_interrupt_device();
	return 0;
}

static void setup_shared_mem()
{
	uint32_t tmp;
	struct xlr_rcv_desc tmp_desc;
	int i;
	xlr_ip_over_pci_base = (uint32_t *)(nlm_common_get_shared_mem_base_host() +
			NLM_IP_OVER_PCI_MEM_BASE);
	xlr_rcv_desc_base = (struct xlr_rcv_desc *)xlr_ip_over_pci_base;
	xlr_xmit_desc_base = (struct xlr_xmit_desc *)
				((unsigned char *)xlr_rcv_desc_base
				 + XLR_RX_DESC*sizeof(struct xlr_rcv_desc));

        xlr_magic_no = (unsigned int *)((xlr_xmit_desc_base + XLR_TX_DESC));
	Message("\nMajic no. Addr %#x\n",virt_to_phys(xlr_magic_no));
	xlr_host_status = xlr_magic_no + 1;
	xlr_dev_status = xlr_host_status + 1;
	Message("\nxlr_rcv_desc_base %#x",xlr_rcv_desc_base);
	Message("\nxlr_xmit_desc_base %#x",xlr_xmit_desc_base);
	Message("\nxlr_magic_no %#x",xlr_magic_no);
	Message("\nxlr_host_status %#x",xlr_host_status);
	Message("\nxlr_dev_status %#x",xlr_dev_status);
	
	tmp = 0;
	nlm_pci_write((uint32_t *)&tmp,1,(uint32_t *)xlr_magic_no);
	nlm_pci_write((uint32_t *)&tmp,1,(uint32_t *)xlr_host_status);
	nlm_pci_write((uint32_t *)&tmp,1,(uint32_t *)xlr_dev_status);

	memset(&tmp_desc,0,sizeof(struct xlr_rcv_desc));
	for(i=0;i<XLR_RX_DESC;i++){
		nlm_pci_write((uint32_t *)&tmp_desc,2,
				(uint32_t *)xlr_rcv_desc_base+i);	
		nlm_pci_write((uint32_t *)&tmp_desc,2,
				(uint32_t *)xlr_xmit_desc_base+i);	
	}

	Message("\nXlrRxBase %#x to %#x\n",
		(uint32_t)xlr_rcv_desc_base,(uint32_t)xlr_xmit_desc_base);
	Message("\nXlrXmitBase %#x to %#x\n",
			(uint32_t)xlr_xmit_desc_base,
	(uint32_t)(xlr_xmit_desc_base+XLR_TX_DESC*sizeof(struct xlr_rcv_desc)));
}


static struct sk_buff * xlr_alloc_skb()
{
	struct sk_buff *skb;
	unsigned long *tmp;
	unsigned long offset;
	unsigned char *tmp_addr;
	skb = dev_alloc_skb(1536+32+32);
	if(!skb){
		Message("SKB Allocation Failed");
		return NULL;
	}
	tmp_addr = skb->data;
	offset = ((unsigned long)skb->data + NLM_SMP_CACHE_BYTES) &  
			~(NLM_SMP_CACHE_BYTES - 1);
	skb_reserve(skb, (offset - (unsigned long)skb->data));
	tmp = (unsigned long*)(skb->data);
	*tmp = (unsigned long)skb;
	skb_reserve(skb,NLM_SMP_CACHE_BYTES+2);
	Message("\nSKB->DATA %#x, orig_addr %#x, diff %#x\n",
			(unsigned int)virt_to_phys((void *)skb->data),
			(uint32_t)virt_to_phys((void *)tmp_addr),
			(uint32_t)virt_to_phys
		((void *)((unsigned int)tmp_addr - (unsigned int)skb->data)));
	return skb;
}

static int replenish_buffer_at_index(int index)
{
	struct sk_buff *skb;
	struct xlr_rcv_desc *curr_desc =
			 (struct xlr_rcv_desc *)xlr_rcv_desc_base+index;
	struct xlr_rcv_desc tmp_desc;
        unsigned int phys_addr;

	memset(&tmp_desc,0,sizeof(struct xlr_rcv_desc));

	skb = xlr_alloc_skb();

	if(!skb){
		ErrorMsg("\nCouldnt Replenish Buffer\n");
		return -ENOMEM;
	}
        phys_addr = pci_map_single(nlm_pdev,skb->data,XLR_MAX_RX_LEN,
                                DMA_FROM_DEVICE);
        xlr_rx_info[index].phys_addr = phys_addr;
        xlr_rx_info[index].skb = skb;
        xlr_rx_info[index].len = XLR_MAX_RX_LEN;

	XLR_SET_HOST_PHYS(&tmp_desc,phys_addr);
	XLR_SET_OWN(&tmp_desc);
	nlm_pci_write((uint32_t *)&tmp_desc,RX_DESC_SIZE,
			(uint32_t *)curr_desc);	
	return 0;
}

static int xlr_replenish_buffer()
{
	int i, j;

	for(i=0;i<XLR_RX_DESC;i++){
		if(replenish_buffer_at_index(i)){
			ErrorMsg("replenish failed for buffer %d",i);
			break;
		}
	}
	if(i != XLR_RX_DESC){
		for(j=0;j<i;j++)
			free_rx_desc_index(j);
		return -ENOMEM;
	}
	return 0;
}

static struct net_device_stats xlr_net_stats;
static struct net_device_stats* xlr_get_stats(struct net_device *dev)
{
	memset(&xlr_net_stats,0,sizeof(struct net_device_stats));
	xlr_net_stats.rx_packets = xlr_rx_pkt;
	xlr_net_stats.tx_packets = xlr_tx_pkt;
	return &xlr_net_stats;
}

static void xlr_link_status(unsigned long data)
{
	uint32_t host_status, dev_status;

	/*Read Host n Dev Status*/
	/*1 = open, 2 = close*/
	host_status = xlr_host_status_local;
	nlm_pci_read((uint32_t *)xlr_dev_status,1,&dev_status);

	if(host_status == XLR_INTERFACE_IS_UP && 
			dev_status == XLR_INTERFACE_IS_UP){
		if(netif_queue_stopped(xlr_netdev) && !xlr_queue_is_stop){
			netif_start_queue(xlr_netdev);
		}
	}else if(!netif_queue_stopped(xlr_netdev)){
		Message("\nStoppin Queue.\n");
		netif_stop_queue(xlr_netdev);
	}
	xlr_link_status_timer.expires = jiffies + 10;
	add_timer(&xlr_link_status_timer);
}

static void setup_net_ops(struct net_device *dev)
{
	nlm_host_net_ops.ndo_open = xlr_ip_over_pci_open;
	nlm_host_net_ops.ndo_stop = xlr_ip_over_pci_close;
	nlm_host_net_ops.ndo_get_stats = xlr_get_stats;
	nlm_host_net_ops.ndo_start_xmit = xlr_ip_over_pci_xmit;
	dev->netdev_ops = &nlm_host_net_ops;
}
static int xlr_ip_over_pci_init(void)
{
	struct net_device *dev = 0;
	struct driver_data *priv = 0;
	int ret = 0;
	uint32_t status;

#ifdef CONFIG_NLM_COMMON
  if(nlm_get_pci_mode() == XLR_PCI_DEV_MODE){
    Message("Xlr Is configured in Dev Mode - unloading xlr_ip_over_pci_host");
    return -ENODEV;
  }
#endif
  if(nlm_common_get_shared_mem_base_host() == 0){
    printk("\nLooks like device is not connected.\n");
    return -ENODEV; 
  }
	xlr_netdev = dev = alloc_etherdev(sizeof(struct driver_data));
	if (!dev) {
		ret = -ENOMEM;
		goto out;
	}
	setup_shared_mem();	
	ip_over_pci_priv = priv = netdev_priv(dev);
	priv->dev = dev;

	setup_net_ops(dev);
	put_dummy_mac_address(dev);

	ether_setup(dev);

	ret = register_netdev(dev);

	if(xlr_replenish_buffer()){
		unregister_netdev(xlr_netdev);
		free_netdev(xlr_netdev);
		return -ENOMEM;
	}
#if defined(CONFIG_NLM_COMMON) || !defined(XLR_MSI_IS_SUPPORTED)
  schedule_delayed_work(&ip_over_pci_task_host, 0);
#else
	nlm_common_request_msi_handler(ip_over_pci_rx,(void *)NULL,&msi_index);
#endif
	status = XLR_MAGIC_NO;
	nlm_pci_write((void *)&status,1,(void *)xlr_magic_no);	
	init_timer(&xlr_link_status_timer);
	xlr_link_status_timer.function = xlr_link_status;
	xlr_link_status_timer.expires = jiffies + 10;		
	add_timer(&xlr_link_status_timer);	

	printk("xlr_ip_over_pci_host registered\n");
out:
	return ret;
}

static void free_tx_desc(void)
{
	while(xmit_consumer != xmit_producer){
                pci_unmap_single(nlm_pdev,
                        xlr_tx_free_info[xmit_consumer].phys_addr,
                        xlr_tx_free_info[xmit_consumer].len,
                        DMA_TO_DEVICE);
		dev_kfree_skb(tx_skb_ptr[xmit_consumer]);
		xmit_consumer = (xmit_consumer + 1 ) % XLR_TX_DESC;
	}
}

static void free_rx_desc_index(int i)
{
	struct xlr_rcv_desc *curr_rcv_desc, tmp_rx;
	uint64_t phys_addr;
	struct sk_buff *skb;
	curr_rcv_desc = (struct xlr_rcv_desc *)xlr_rcv_desc_base+i;
	nlm_pci_read((uint32_t *)curr_rcv_desc,RX_DESC_SIZE,
			(uint32_t *)&tmp_rx);
	phys_addr = XLR_GET_HOST_PHYS(&tmp_rx);
        pci_unmap_single(nlm_pdev,xlr_rx_info[i].phys_addr,
                                xlr_rx_info[i].len,DMA_FROM_DEVICE);
        skb = xlr_rx_info[i].skb;
	dev_kfree_skb(skb);	
}

static void free_rx_desc(void)
{
	int i;
	for(i=0;i<XLR_RX_DESC;i++){
		free_rx_desc_index(i);
	}
}

static void reset_tx_desc(void)
{
        int i=0;
	uint32_t info;
        for(i=0;i<XLR_TX_DESC;i++){
		nlm_pci_read((void *)&info,1,
			(void *)&((xlr_xmit_desc_base+i)->info));
		info = info & (0x7fffffffU);
		nlm_pci_write((void *)&info,1,
			(void *)&((xlr_xmit_desc_base+i)->info));
	}
}
static void reset_rx_desc(void)
{
        int i=0;
	uint32_t info;
        for(i=0;i<XLR_RX_DESC;i++){
		nlm_pci_read((void *)&info,1,
			(void *)&((xlr_rcv_desc_base+i)->info));
		info = info & (0x7fffffffU);
		nlm_pci_write((void *)&info,1,
			(void *)&((xlr_rcv_desc_base+i)->info));
	}
}
		

static void xlr_ip_over_pci_exit(void)
{
	uint32_t status;
	if(!xlr_netdev)
		return;	
	del_timer_sync(&xlr_link_status_timer);
	status = 0x0;
	nlm_pci_write((uint32_t *)&status,1,(uint32_t *)xlr_magic_no);	
	nlm_nlm_common_interrupt_device();

#if defined(CONFIG_NLM_COMMON) || !defined(XLR_MSI_IS_SUPPORTED)
	rmmod_is_set=1;
#else
	nlm_common_free_msi_handler(&msi_index);
#endif

	
	reset_rx_desc();
	reset_tx_desc();
	mdelay(3000);
	unregister_netdev(xlr_netdev);
	free_tx_desc();
	free_rx_desc();
	free_netdev(xlr_netdev);
}

module_init(xlr_ip_over_pci_init);
module_exit(xlr_ip_over_pci_exit);
MODULE_LICENSE("GPL");

