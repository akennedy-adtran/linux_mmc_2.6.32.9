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

#include <asm/netlogic/debug.h>
#include <asm/netlogic/pci.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/proc.h>
#include <asm/netlogic/nlm_pcix_gen_dev.h>

#define DRV_NAME	"nlm_ip_over_pci"
#define DRV_VERSION	"0.1"
#define DRV_RELDATE	"10Feb2004"

#define XLR_TX_DESC 512 
#define XLR_RX_DESC 512

#define BRIDGE_PCIXMEM_BAR 35

#define Message(a,b...) //printk("\n[%s] - "a"\n",__FUNCTION__,##b)
#define ErrorMsg(a,b...) printk("\nError in [%s] - "a"\n",__FUNCTION__,##b)

#define XLR_INTERFACE_IS_UP 1
#define XLR_INTERFACE_IS_DOWN 2
#define XLR_MAGIC_NO 0xdeadbeef

static struct net_device *xlr_netdev= NULL;
static unsigned int *xlr_ip_over_pci_base = NULL;
typedef volatile unsigned int xlr_reg_t;
static struct net_device_ops nlm_dev_net_ops;
#ifdef XLR_MAILBOX_IS_SUPPORTED
static int mailbox_index;
#endif

extern int xlr_init_dma(void);
extern int xlr_request_dma(uint64_t src, uint64_t dest, uint32_t len);
extern int xlr_async_request_dma(uint64_t src, uint64_t dest, uint32_t len,
			void (*func)(void *,uint64_t),void *data);
extern long nlm_common_get_shared_mem_base_dev(void);
static void setup_shared_mem(void);
static void xlr_pcix_init(void);

/*DEBUG CNTRS*/
static int xlr_rx_pkt;
static int xlr_tx_pkt;
static int xlr_rx_dma_enqueue_failed;
static int xlr_tx_dma_enqueue_failed;
static int xlr_rx_dma_error;
static int xlr_tx_dma_error;
/*DEBUG CNTRS END*/

#define XLR_GET_OWN(x) (x->info & 0x80000000)
#define XLR_RESET_OWN(x) (x->info = x->info & 0x7fffffff)
#define XLR_GET_HOST_PHYS(x) (x->addr)
#define XLR_SET_LEN(x,len) x->info = x->info & ~(0x3fff); \
			   x->info |= len;
#define XLR_GET_LEN(x) (x->info & 0x3fff)


struct xlr_xmit_desc{
	uint32_t addr;
	uint32_t info;/*  Bit 0 to 15 LEN
    		 	  Bit 16 to 30 Reserved 
			  Bit 31 OWN						                       */

};
struct xlr_rcv_desc{
	uint32_t addr;
	uint32_t info;/*  Bit 0 to 15 LEN
    		 	  Bit 16 to 30 RESERVED
			  Bit 31 OWN						                       */

};

static volatile struct xlr_xmit_desc *xlr_xmit_desc_base;

static volatile struct xlr_rcv_desc *xlr_rcv_desc_base;

static volatile unsigned int xmit_producer=0;
static volatile unsigned int rx_consumer=0;

static volatile uint32_t *xlr_magic_no;
static volatile uint32_t *xlr_host_status;
static volatile uint32_t *xlr_dev_status;
static struct timer_list xlr_link_status_timer;
static volatile uint32_t xlr_dev_status_local;
static struct driver_data{
	struct net_device *dev;	
	struct xlr_xmit_desc *curr_xmit;
	struct xlr_xmit_desc *xmit_base;
	struct xlr_rcv_desc *curr_rx;
	struct xlr_rcv_desc *rx_base;
}*ip_over_pci_priv;


static volatile int xmit_dma_request_submitted=0;
static volatile int xmit_dma_request_completed=0;
static volatile int rx_dma_request_submitted=0;
static volatile int rx_dma_request_completed=0;
static volatile int xlr_queue_is_stop=0;
static spinlock_t xlr_tx_ok_sync=SPIN_LOCK_UNLOCKED;
static spinlock_t xlr_rx_dma=SPIN_LOCK_UNLOCKED;
static spinlock_t xlr_tx_dma=SPIN_LOCK_UNLOCKED;
static spinlock_t xlr_xmit_sync = SPIN_LOCK_UNLOCKED;

#if !defined(XLR_MAILBOX_IS_SUPPORTED)
static void ip_over_pci_rx(struct work_struct *data);
#else
static irqreturn_t ip_over_pci_rx(void *data, struct pt_regs *regs);
#endif
struct priv_rx_data{
	struct sk_buff *skb;
	int rx_consumer;
};

struct priv_tx_data{
	struct sk_buff *skb;
	int tx_producer;
};

static void tx_dma_done(void *data, uint64_t status)
{
	struct priv_tx_data *priv= (struct priv_tx_data *)data;
	int ret,err;
	int error_flag=0;
	volatile struct xlr_xmit_desc *curr_xmit_desc;
	unsigned long mflags;

	ret = (status >> 62) & 0x3;
	err = (status >> 60) & 0x3;
	
	if(ret != 0x3) {
		ErrorMsg("%s: Bad return code %d from DMA engine\n", \
				__FUNCTION__,ret);
		error_flag=1;
	}
	if(err & 0x2) {
		ErrorMsg("%s:DMA engine reported Message format error\n", \
				__FUNCTION__);
		error_flag=1;
	}
	if(err & 0x1) {
		ErrorMsg("%s:DMA engine reported Bus error\n", __FUNCTION__);
		error_flag=1;
	}

	dev_kfree_skb_irq(priv->skb);
	if(error_flag){
		curr_xmit_desc = xlr_xmit_desc_base + priv->tx_producer;
		//TODO: Instead of own bit set some error flag in descriptor.
		//This packet will be dropped by host proto stack.
		XLR_RESET_OWN(curr_xmit_desc);
		xlr_tx_dma_error++;
		nlm_common_interrupt_host();
	}
	else{
		curr_xmit_desc = xlr_xmit_desc_base + priv->tx_producer;
		XLR_RESET_OWN(curr_xmit_desc);
		xlr_tx_pkt++;
		//Send msi to host - Rx For HOST 
		nlm_common_interrupt_host();
	}
	spin_lock_irqsave(&xlr_tx_dma,mflags);
	xmit_dma_request_completed = (xmit_dma_request_completed+1)%XLR_TX_DESC;
	spin_unlock_irqrestore(&xlr_tx_dma,mflags);
	kfree(priv);
	spin_lock_irqsave(&xlr_tx_ok_sync,mflags);
	if(xlr_queue_is_stop){
		netif_start_queue(xlr_netdev);
		xlr_queue_is_stop = 0;
	}
	spin_unlock_irqrestore(&xlr_tx_ok_sync,mflags);
}


static void rx_dma_done(void *data, uint64_t status)
{
	struct priv_rx_data *priv= (struct priv_rx_data *)data;
	int ret,err;
	int error_flag=0;
	volatile struct xlr_rcv_desc *curr_rcv_desc;
	unsigned long mflags;

	
	ret = (status >> 62) & 0x3;
	err = (status >> 60) & 0x3;
	if(ret != 0x3) {
		ErrorMsg("%s: Bad return code %d from DMA engine\n", \
				__FUNCTION__,ret);
		error_flag=1;
	}
	if(err & 0x2) {
		ErrorMsg("%s:DMA engine reported Message format error\n", \
				__FUNCTION__);
		error_flag=1;
	}
	if(err & 0x1) {
		ErrorMsg("%s:DMA engine reported Bus error\n", __FUNCTION__);\
		error_flag=1;
	}
	if(error_flag){
		dev_kfree_skb_irq(priv->skb);
		curr_rcv_desc = xlr_rcv_desc_base + priv->rx_consumer;
		XLR_RESET_OWN(curr_rcv_desc);
		xlr_rx_dma_error++;
		nlm_common_interrupt_host();
	}
	else{
		curr_rcv_desc = xlr_rcv_desc_base + priv->rx_consumer;
		XLR_RESET_OWN(curr_rcv_desc);
		priv->skb->protocol = eth_type_trans(priv->skb,priv->skb->dev);
		netif_rx(priv->skb);
		Message("\nSKB QUEUED TO UPPER LAYER - Len %d\n",priv->skb->len);
		//Send msi to host - tx_ok
		nlm_common_interrupt_host();
		xlr_rx_pkt++;
	}
	spin_lock_irqsave(&xlr_rx_dma,mflags);
	rx_dma_request_completed = (rx_dma_request_completed+1)%XLR_RX_DESC;
	spin_unlock_irqrestore(&xlr_rx_dma,mflags);
	kfree(priv);
}

#if !defined(XLR_MAILBOX_IS_SUPPORTED)
static DECLARE_DELAYED_WORK(ip_over_pci_task,ip_over_pci_rx);
static void ip_over_pci_rx(struct work_struct  *data)
#else
static irqreturn_t ip_over_pci_rx (void *data,struct pt_regs *regs)
#endif
{
	volatile struct xlr_rcv_desc *curr_rcv_desc;
	uint64_t phys_addr=0;
	struct sk_buff *skb;
	int data_len;
	int count = 0x0;
	struct priv_rx_data *priv;

	curr_rcv_desc = xlr_rcv_desc_base + rx_consumer;
	/*Check If There Is ANyting To Rcv.*/
	while(XLR_GET_OWN(curr_rcv_desc)){
		if(((rx_dma_request_submitted + 1)%XLR_RX_DESC) == 
						rx_dma_request_completed){
			//ErrorMsg("rx_dma_request_completed = %d,rx_dma_request_submitted = %d",	rx_dma_request_completed,rx_dma_request_submitted);
			break;
		}
    count++;
		priv = kmalloc(sizeof(struct priv_rx_data),GFP_ATOMIC);
		
		if(!priv){
			ErrorMsg("Priv Allocation Failed In tasklet");
			break;
		}
		
		phys_addr = XLR_GET_HOST_PHYS(curr_rcv_desc);
		phys_addr = phys_addr | 0x8000000000ULL;
		data_len = XLR_GET_LEN(curr_rcv_desc);

		skb = dev_alloc_skb(data_len+14);
		Message("\nRcvd Buf From Host\n");
		if(!skb){
			ErrorMsg("\nSKB Alloction Failed\n");
			kfree(priv);
			break;
		}
		priv->skb = skb;
		priv->rx_consumer = rx_consumer;

		Message("\nRcvd Pkt Len of %d\n",data_len);
		skb_reserve(skb, 2);
		skb_put(skb,data_len);
		skb->dev = xlr_netdev;

		if(xlr_async_request_dma(phys_addr,virt_to_phys(skb->data),
					data_len,rx_dma_done,(void *)priv)){
			//ErrorMsg("Rx DMA Queue Failed");
			xlr_rx_dma_enqueue_failed++;
			dev_kfree_skb(skb);
			kfree(priv);
			break;
		}
		rx_dma_request_submitted = (rx_dma_request_submitted+1)%
							XLR_RX_DESC;
		rx_consumer = (rx_consumer + 1)% XLR_RX_DESC;
		curr_rcv_desc = xlr_rcv_desc_base + rx_consumer;
	}
	if(*xlr_magic_no != XLR_MAGIC_NO){
		//Reset All Prod/Cons here.
		spin_lock(&xlr_xmit_sync);
                xmit_dma_request_submitted = xmit_dma_request_completed = 0;
                xmit_producer = rx_consumer = 0;
                rx_dma_request_submitted = rx_dma_request_completed = 0;
		spin_unlock(&xlr_xmit_sync);
	}	
#if defined(XLR_MAILBOX_IS_SUPPORTED)
	return IRQ_HANDLED;
#else
	schedule_delayed_work(&ip_over_pci_task, 1);
#endif
}


static void put_dummy_mac_address(struct net_device *dev)
{
	dev->dev_addr[0] = 0x0;
	dev->dev_addr[1] = 0xa;
	dev->dev_addr[2] = 0x0;
	dev->dev_addr[3] = 0xa;
	dev->dev_addr[4] = 0x0;
	dev->dev_addr[5] = 0xa;
}

static int xlr_ip_over_pci_open(struct net_device *dev)
{
	xlr_dev_status_local = 1;
	*xlr_dev_status = 1;
	return 0;
}

static int xlr_ip_over_pci_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	xlr_dev_status_local = 2;
	*xlr_dev_status = 2;
	return 0;
}

static int xlr_ip_over_pci_xmit (struct sk_buff *skb,struct net_device *dev)
{
	volatile struct xlr_xmit_desc *curr_xmit_desc;
	uint64_t phys_addr=0;
	struct priv_tx_data *priv;
	int old_val;
	unsigned long mflags;

	if(*xlr_magic_no != XLR_MAGIC_NO){
		netif_stop_queue(dev);
		return -EIO;
	}
	if(*xlr_host_status == XLR_INTERFACE_IS_DOWN){
		netif_stop_queue(dev);
		return -EIO;
	}
	if(skb->len > 1514){
		ErrorMsg("\n %d Len Not Supported\n",skb->len);
		return -ENOMEM;
	}
	if(((xmit_dma_request_submitted + 1)%XLR_TX_DESC) == 
				xmit_dma_request_completed){
		Message("xmit_dma_request_completed = %d,xmit_dma_request_submitted = %d",xmit_dma_request_completed,xmit_dma_request_submitted);
		spin_lock_irqsave(&xlr_tx_ok_sync,mflags);
		if(((xmit_dma_request_submitted + 1)%XLR_TX_DESC) !=
                                xmit_dma_request_completed){
			spin_unlock_irqrestore(&xlr_tx_ok_sync,mflags);
			goto try_again;
		}
		xlr_queue_is_stop = 1;
		netif_stop_queue(dev);	
		spin_unlock_irqrestore(&xlr_tx_ok_sync,mflags);
		return -ENOMEM;
	}
try_again:
	priv = kmalloc(sizeof(struct priv_tx_data),GFP_KERNEL);
	if(!priv){
		ErrorMsg("priv allocation failed");
		return -ENOMEM;
	}
	curr_xmit_desc = xlr_xmit_desc_base + xmit_producer;

	if(!XLR_GET_OWN(curr_xmit_desc)){
		Message("\nHost Memory is Not Available for XMIT\n");
		kfree(priv);
		return -ENOMEM;
	}
	phys_addr = XLR_GET_HOST_PHYS(curr_xmit_desc);
	
	
	phys_addr = phys_addr | 0x8000000000ULL;
	priv->tx_producer = xmit_producer;
	priv->skb = skb;
	XLR_SET_LEN(curr_xmit_desc,skb->len);

	old_val = xmit_dma_request_submitted;
	xmit_dma_request_submitted = (xmit_dma_request_submitted+1)%XLR_TX_DESC;
	if(xlr_async_request_dma(virt_to_phys(skb->data),phys_addr,skb->len,\
				tx_dma_done,priv)){
		//ErrorMsg("Cant Queue XMIT Msg.");
		kfree(priv);
		xlr_tx_dma_enqueue_failed++;
		xmit_dma_request_submitted = old_val;
		return -EIO;
	}
	Message("Xmit Pkt With Len skblen %d -> desc len %d",
				skb->len,XLR_GET_LEN(curr_xmit_desc));

	spin_lock_irqsave(&xlr_xmit_sync,mflags);
	if(*xlr_magic_no == XLR_MAGIC_NO)
		xmit_producer = (xmit_producer + 1) % XLR_TX_DESC;
	spin_unlock_irqrestore(&xlr_xmit_sync,mflags);
	return 0;
}


static void setup_shared_mem(void)
{
	xlr_ip_over_pci_base = (unsigned int *)(nlm_common_get_shared_mem_base_dev()
				 +NLM_IP_OVER_PCI_MEM_BASE);
	xlr_xmit_desc_base = (struct xlr_xmit_desc *)xlr_ip_over_pci_base;
	xlr_rcv_desc_base = (struct xlr_rcv_desc *)
				((unsigned char *)xlr_xmit_desc_base
				 + XLR_TX_DESC*sizeof(struct xlr_xmit_desc));

	xlr_magic_no = (uint32_t *)((xlr_rcv_desc_base + XLR_RX_DESC));
        xlr_host_status = xlr_magic_no + 1;
        xlr_dev_status = xlr_host_status + 1;

        Message("\nXlrXmitBase %#x to %#x\n",(uint32_t)xlr_xmit_desc_base,
			(uint32_t)xlr_rcv_desc_base);
        Message("\nXlrRxBase %#x to %#x\n",(uint32_t)xlr_rcv_desc_base,
	(uint32_t)(xlr_rcv_desc_base+XLR_RX_DESC*sizeof(struct xlr_rcv_desc)));
	Message("XlrMagicNo. %#x",xlr_magic_no);
}

static void xlr_pcix_init(void)
{
        xlr_reg_t *bmmio = 0;
        bmmio = netlogic_io_mmio(NETLOGIC_IO_BRIDGE_OFFSET);
	/*Set Defeature bit*/
	bmmio[59] |= 0x2;
	/* Use 0x8000000000ULL and above for PCI addresses in XLR memory map */
	bmmio[BRIDGE_PCIXMEM_BAR] = 0x8000ffff;
	Message("\nDefeature REgister Addr %#x And Value Is %#x\n",
			(unsigned int)&bmmio[59],bmmio[59]);
	Message("\nBRIDGE PCIXMEM BAR REG ADDR %#x and Value Is %#x\n",
			(unsigned int)&bmmio[BRIDGE_PCIXMEM_BAR],
			bmmio[BRIDGE_PCIXMEM_BAR]);
}

static struct net_device_stats xlr_net_stats;;
static struct net_device_stats* xlr_get_stats(struct net_device *dev)
{
	memset(&xlr_net_stats,0,sizeof(struct net_device_stats));
	xlr_net_stats.rx_packets = xlr_rx_pkt;
	xlr_net_stats.tx_packets = xlr_tx_pkt;
	xlr_net_stats.rx_errors = xlr_rx_dma_error;
	xlr_net_stats.tx_errors = xlr_tx_dma_error;
	return &xlr_net_stats;
}

static void xlr_link_status(unsigned long data)
{
	uint32_t host_status, dev_status;

	*xlr_dev_status = xlr_dev_status_local;

	if(*xlr_magic_no != XLR_MAGIC_NO){
		if(!netif_queue_stopped(xlr_netdev))
			netif_stop_queue(xlr_netdev);
		goto try_again;
	}

	/*Read Host n Dev Status*/
	/*1 = open, 2 = close*/
	host_status = *xlr_host_status;
	dev_status = *xlr_dev_status;

	if(dev_status == 1 && host_status == 1){
		if(netif_queue_stopped(xlr_netdev) && !xlr_queue_is_stop){
			netif_start_queue(xlr_netdev);
		}
	}else if(!netif_queue_stopped(xlr_netdev)){
		Message("\nStoppin Queue.\n");
		netif_stop_queue(xlr_netdev);
	}

try_again:
	xlr_link_status_timer.expires = jiffies + 10;
	add_timer(&xlr_link_status_timer);
}

static void setup_net_ops(struct net_device *dev)
{
	nlm_dev_net_ops.ndo_open = xlr_ip_over_pci_open;
	nlm_dev_net_ops.ndo_stop = xlr_ip_over_pci_close;
	nlm_dev_net_ops.ndo_get_stats = xlr_get_stats;
	nlm_dev_net_ops.ndo_start_xmit = xlr_ip_over_pci_xmit;
	dev->netdev_ops = &nlm_dev_net_ops;
}

static int xlr_ip_over_pci_init(void)
{
	struct net_device *dev = 0;
	struct driver_data *priv = 0;
	int ret = 0;

  if(xlr_get_pci_mode() == XLR_PCI_HOST_MODE){
    Message("\nXlr Is Configured In Host Mode - Unloading xlr_ip_over_pci_dev");
		return -EIO;
  }
	
	/*Do DMA initialization*/
	if(xlr_init_dma()){
		ErrorMsg("xlr_init_dma failed");
		ret = -EIO;
		goto out;
	}

	xlr_netdev = dev = alloc_etherdev(sizeof(struct driver_data));
	if (!dev) {
		ret = -ENOMEM;
		goto out;
	}
	/*Set Defeature bit and bridge pcix mmio bar*/	
	xlr_pcix_init();	
	/*Get the shared region base address for ip_over_pci descriptors.*/
	setup_shared_mem();

	ip_over_pci_priv = priv = netdev_priv(dev);
	priv->dev = dev;

	setup_net_ops(dev);
	
	put_dummy_mac_address(dev);

	ether_setup(dev);

	ret = register_netdev(dev);
#ifdef XLR_MAILBOX_IS_SUPPORTED
	nlm_common_request_mailbox_handler(ip_over_pci_rx,NULL,&mailbox_index);
#else
	schedule_delayed_work(&ip_over_pci_task, 0);
#endif
	init_timer(&xlr_link_status_timer);       
        xlr_link_status_timer.function = xlr_link_status;
        xlr_link_status_timer.expires = jiffies + 10;
        add_timer(&xlr_link_status_timer);

  printk("xlr_ip_over_pci_dev registered\n");
out:
	return ret;
}

/**********************************************************************
 **********************************************************************/
static void xlr_ip_over_pci_exit(void)
{
	if(!xlr_netdev)
		return;	
	unregister_netdev(xlr_netdev);
	free_netdev(xlr_netdev);
#ifdef XLR_MAILBOX_IS_SUPPORTED
	nlm_common_free_mailbox_handler(&mailbox_index);
#endif
}

module_init(xlr_ip_over_pci_init);
module_exit(xlr_ip_over_pci_exit);
