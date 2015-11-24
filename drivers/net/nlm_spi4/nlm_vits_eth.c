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
#include <linux/workqueue.h>
#include <linux/types.h>
#include <linux/errno.h>

#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "nlm_vits_eth.h"
#include "nlm_spi4_config.h"
#include "os_layer.h"

#include <asm/netlogic/msgring.h>
#include <asm/netlogic/sim.h>
#include "meigsii_reg.h"
#include "nlm_vits_wrapper.h"
#include "nlm_vits_driver.h"
#include "vitesse_common.h"
#include "vitesse_highlevel.h"
#include "vitesse_io.h"
#include "vitesse_phy_ctrl.h"

#include <asm/netlogic/xlr_mac.h>

#define DRV_NAME  "nlm_vits_spi4"
#define DRV_VERSION "0.1"
#define DRV_RELDATE "10Aug2005"

#define PHY_MONITOR	1
#define NLM_SPI4_MAX_CPUS 32
#define SPI4_FRIN_THRESHOLD spi4_frin_threashold
#define NLM_SPI4_TX_MAX_COUNTER 100
#define MAKE_CACHE_ALIGN ~(0x1f)
#define NLM_SPI4_DEBUG 0

enum vits_returns{
	VITS_PROGRAM_RX_DESC_FAIL,
	VITS_PROGRAM_RX_DESC_SUCCESS,
	VITS_COMMON_INIT_SUCCESS,
};

struct net_device  *spi4_dev[NLM_SPI4_MAX_PORTS];
typedef struct _driver_data{

	struct net_device 	*dev;
	spinlock_t          lock;

	uint	port; // this is eth port: 0-19
	uint	slot;
	uint	spi4_port;	//this is spi4 port :0-9
	uint	speed;
	uint	duplex;
	uint 	autoneg;
	uint	type;
	uint	cfg_flag;
	struct net_device_stats        stats;
  atomic_t     frin_to_be_sent[NLM_SPI4_MAX_CPUS];

}driver_data ;

static int spi4_frin_threashold;
static struct net_device_ops nlm_spi4_net_ops;

unsigned int g_spi4_card_flag;
static spinlock_t pending_tx_lock[NLM_SPI4_MAX_PORTS] __cacheline_aligned;
static spinlock_t base_change=SPIN_LOCK_UNLOCKED;

struct timer_list link_monitor_timer;
extern unsigned long g_dip4_error[];
extern int cpu_to_bktmask[];
static struct 	work_struct vits_frin_replenish_work[NLM_SPI4_MAX_CPUS];

void set_ethtool_ops(struct net_device *netdev);

void nlm_vits_rx_tx_done( uint32  cmd, uint32 slot,  	uint32	port,
		char*	addr, uint32  len,	uint32	error);


unsigned char spi4_check_daughter_cards(void);
static __inline__ struct sk_buff *spi4_get_skb_back_ptr(unsigned long addr)
{
	unsigned long *back_ptr =
		(unsigned long *)(addr - MAC_SKB_BACK_PTR_SIZE);

	/* this function should be used only for newly allocated packets.
	 * It assumes the first cacheline is for the back pointer related
	 * book keeping info
	 */
	return (struct sk_buff *)(*back_ptr);
}

static __inline__ void spi4_put_skb_back_ptr(struct sk_buff *skb)
{
	unsigned long *back_ptr = (unsigned long *)skb->data;

	/* this function should be used only for newly allocated
	 * packets. It assumes  the first cacheline is for the back
	 * pointer related book keeping info
	 */
	skb_reserve(skb, MAC_SKB_BACK_PTR_SIZE);

	*back_ptr = (unsigned long)skb;
}

void change_vits_base(int port)
{
    if(port < NLM_SPI4_PORTS_PER_CARD){
      megis_nlm_common_init(1);
    }
    else{
      megis_nlm_common_init(2);
    }
	return;
}

/*******************************************************************************
* Function name :       nlm_link_monitor
* Input         :
* Description   :       This function monitors all the phy channels and if speed
*	or duplexity changed, then it changes the mac's speed and duplexity.
* RETURNS       :       void
*******************************************************************************/

void nlm_link_monitor(unsigned long data)
{

	int i, port, cpu, thr_id, slot;
	struct net_device *dev;
	driver_data *priv ;
	int speed, duplexity;
	unsigned long flags;
	vtss_phy_status_t phy_status;

	cpu = netlogic_cpu_id() ;
	thr_id = netlogic_thr_id() ;

	spin_lock_irqsave(&base_change, flags);
	for(i =0; i< NLM_SPI4_MAX_PORTS; i++){
		slot = active_port[i].slot ;

		if((spi4_slot[slot] == INVALID_SLOT ) ||
		(active_port[i].port == INVALID_PORT)) continue;

		if(!(g_spi4_card_flag & (1<<slot)))
      			continue;

		change_vits_base(i);

		if(active_port[i].slot == SPI4_0)
			port = active_port[i].port;
		else if(active_port[i].slot == SPI4_1)
			port = (active_port[i].port) - NLM_SPI4_PORTS_PER_CARD;
		else{
			printk("ERROR: wrong slot should never happen\n");
			continue;
		}

		/*if the link is down for the particular port just continue
		  for the remaining ports */
		port++;
		if(!vtss_nlm_monitor_phy_status(port, &speed, &duplexity)){
			continue;
		}
		dev = spi4_dev[i];
		if(!dev) continue;
		priv = netdev_priv(dev);
		if(speed == UNDEFINED_SPEED){
			continue;
		}
#if NLM_SPI4_DEBUG
		printk("new speed=%d, old_speed=%d\n",
			speed, priv->speed);
#endif
		if((priv->speed != speed) || (priv->duplex != duplexity)){
#if NLM_SPI4_DEBUG
			printk("speed change to port=%d : speed=%d, dup=%d\n",
				port, speed, duplexity);
#endif
			if(vtss_nlm_change_port_status(port, speed, duplexity)){
				priv->speed = speed;
				priv->duplex = duplexity;
			}
		}

		if(vtss_phy_status_get(port, &phy_status) == VTSS_OK){
			if (phy_status.link_status) {
				netif_carrier_on(dev);
			}
			else {
				netif_carrier_off(dev);
			}
		}
		else {
#if NLM_SPI4_DEBUG
			printk("phy status read failed\n");
#endif
		}
	}// end of for loop

	spin_unlock_irqrestore(&base_change,flags);
	link_monitor_timer.expires = jiffies +  HZ;
	add_timer(&link_monitor_timer);

	return;
}



static int nlm_vits_open(struct net_device *dev)
{
	unsigned int vits_port;
  unsigned long  tx_rx_status;
	driver_data *priv = netdev_priv(dev);
  unsigned long flags;

#if NLM_SPI4_DEBUG
	printk("opening the spi4 interface: %d\n", priv->port);
#endif

  spin_lock_irqsave(&base_change, flags);
  if(priv->port < NLM_SPI4_PORTS_PER_CARD){
		vits_port =  priv->port ;
	}
	else {
		vits_port =  priv->port - NLM_SPI4_PORTS_PER_CARD;
	}
	change_vits_base(priv->port);
  tx_rx_status = vtss_io_read(M2_BLK_MACS, vits_port, M2_MODE_CFG);
	tx_rx_status |= 0x3 ; // enable TX and RX
	vtss_io_write(M2_BLK_MACS, vits_port, M2_MODE_CFG, tx_rx_status);

  spin_unlock_irqrestore(&base_change,flags);
	return 0;
}

static int nlm_vits_close(struct net_device *dev)
{
  unsigned int vits_port;
  unsigned long  tx_rx_status;
  unsigned long flags;

	driver_data *priv = netdev_priv(dev);

#if NLM_SPI4_DEBUG
	printk("closing the spi4 interface: %d\n", priv->port);
#endif
  spin_lock_irqsave(&base_change, flags);
  if(priv->port < NLM_SPI4_PORTS_PER_CARD){
		vits_port =  priv->port ;
	}
	else {
		vits_port =  priv->port - NLM_SPI4_PORTS_PER_CARD;
	}
  change_vits_base(priv->port);
  tx_rx_status = vtss_io_read(M2_BLK_MACS, vits_port, M2_MODE_CFG);
	tx_rx_status &= ~(0x3) ; // disable TX and RX
	vtss_io_write(M2_BLK_MACS, vits_port, M2_MODE_CFG, tx_rx_status);
  spin_unlock_irqrestore(&base_change,flags);
	return 0;
}

#if NLM_SPI4_DEBUG
void print_stats(vtss_port_counters_t *stat)
{

	printk("out byte		: %ld\n",stat->tx_out_bytes);
	printk("tx_pause		: %ld\n", stat->tx_pause);
	printk("tx_ok_by		: %ld\n", stat->tx_ok_bytes);
	printk("tx_unicast		: %ld\n", stat->tx_unicast);
	printk("tx_multicast		: %ld\n", stat->tx_multicast);
	printk("tx_broadcast		: %ld\n", stat->tx_broadcast);
	printk("tx_multiple_coll	: %ld\n", stat->tx_multiple_coll);
	printk("tx_late_coll		: %ld\n", stat->tx_late_coll);
	printk("tx_xcoll		: %ld\n", stat->tx_xcoll);
	printk("tx_defer		: %ld\n", stat->tx_defer);
	printk("tx_xdefer		: %ld\n", stat->tx_xdefer);
	printk("tx_carrier_sense	: %ld\n", stat->tx_carrier_sense);
	printk("tx_size_64		: %ld\n", stat->tx_size_64);
	printk("tx_size_65_to_127	: %ld\n", stat->tx_size_65_to_127);
	printk("tx_size_128_to_255	: %ld\n", stat->tx_size_128_to_255);
	printk("tx_size_256_to_511	: %ld\n", stat->tx_size_256_to_511);

	printk("tx_single_coll		: %ld\n", stat->tx_single_coll);
	printk("tx_backoff2		: %ld\n", stat->tx_backoff2);
	printk("tx_backoff3		: %ld\n", stat->tx_backoff3);
	printk("\n");
	printk("tx_underrun		: %ld\n", stat->tx_underrun);
	printk("ingress_overflow_drop	: %ld\n", stat->ingress_overflow_drop);
	printk("egress_overflow_drop	: %ld\n", stat->egress_overflow_drop);
	printk("\n");


	printk("rx_in_bytes            : %ld\n",stat->rx_in_bytes);
	printk("rx_ok_bytes            : %ld\n",stat->rx_ok_bytes);
	printk("rx_bad_bytes           : %ld\n", stat->rx_bad_bytes);
	printk("rx_unicast             : %ld\n", stat->rx_unicast);
	printk("rx_multicast           : %ld\n",stat->rx_multicast);
	printk("rx_broadcast           : %ld\n", stat->rx_broadcast);
	printk("rx_crc                 : %ld\n", stat->rx_crc);
	printk("rx_undersize           : %ld\n", stat->rx_undersize);
	printk("rx_fragments           : %ld\n", stat->rx_fragments);
	printk("rx_in_range_error      : %ld\n", stat->rx_in_range_error);
	printk("rx_out_of_range_error  : %ld\n", stat->rx_out_of_range_error);
	printk("rx_oversize            : %ld\n", stat->rx_oversize);

	printk("\n");
	printk("\n");

	return;
}

void nlm_vits_dump_dbg_regs(struct net_device *dev)
{

	int port;
	long reg_value;

	driver_data *priv = netdev_priv(dev);
	if(priv->slot == SPI4_0)
		port = priv->port;
	else if(priv->slot == SPI4_1)
		port = priv->port - NLM_SPI4_PORTS_PER_CARD;
	else
		return ;

	port++;
	reg_value = spi4_read_reg(priv->slot, R_TX_CONTROL);
	printk("GLUE REG: TX CONTROL addr : A0 = %lx\n", reg_value);
	reg_value = spi4_read_reg(priv->slot, SPI4_TX_STATUS);
	printk("TX STATUS = 0x%lx\n", reg_value);
	reg_value = spi4_read_reg(priv->slot, SPI4_RX_STATUS);
	printk("RX STATUS = 0x%lx\n", reg_value);
	reg_value = spi4_read_reg(priv->slot, 0x70);
	printk("INT REG = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(1, port, 0x0a);
	printk("VTSS reg stick_bit - addr 0x0a = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(1, port, 0x0c);
	printk("VTSS reg drop count - addr 0x0c = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(7, 0xf, 0x0a);
	printk("VTSS reg egr crc count -(7,f,a)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(7, 0xf, 0x0b);
	printk("VTSS reg crc config -(7,f,b)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x0f);
	printk("VTSS reg EGR_CONTROL -(2,1,f)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x60);
	printk("VTSS reg EGR DROP COUNT -(2,1,60)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x61);
	printk("VTSS reg EGR DROP COUNT  -(2,1,61)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x62);
	printk("VTSS reg EGR DROP COUNT  -(2,1,62)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x63);
	printk("VTSS reg EGR DROP COUNT  -(2,1,63)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x64);
	printk("VTSS reg EGR DROP COUNT  -(2,1,64)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x65);
	printk("VTSS reg EGR DROP COUNT  -(2,1,65)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x66);
	printk("VTSS reg EGR DROP COUNT  -(2,1,66)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x67);
	printk("VTSS reg EGR DROP COUNT  -(2,1,67)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x68);
	printk("VTSS reg EGR DROP COUNT  -(2,1,68)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(2, 1, 0x69);
	printk("VTSS reg EGR DROP COUNT  -(2,1,69)  = 0x%lx\n", reg_value);
	reg_value = vtss_io_read(5, 0, 0x30);
	printk("VTSS reg spi4_sticky : host spi:   -(5,0,30)  = 0x%lx\n", reg_value);

	return;
}
#endif

int nlm_vits_collect_stats(struct net_device *dev,
		vtss_port_counters_t *stat)
{
	int port;
	driver_data *priv = netdev_priv(dev);

	if(priv->slot == SPI4_0)
		port = priv->port;
	else if(priv->slot == SPI4_1)
		port = priv->port - NLM_SPI4_PORTS_PER_CARD ;
	else
		return 1;
	change_vits_base(priv->port);
	// this is for the VTSS API, which will search the port data structure
	port++;
	if(vtss_port_counters_get((const vtss_port_no_t)port,stat)== VTSS_OK){
		return 0;
	}
	return 1;
}
/*
   VTSS API for clearing stat has some problem, one can't clear for a
   particular port, it will clear all the ports, so currently stat clear
   is not done
 */
static struct net_device_stats* nlm_vits_get_stats(struct net_device *dev)
{
  unsigned long flags, i ;
	unsigned long total_dip4=0;
	vtss_port_counters_t stat;
	driver_data *priv = netdev_priv(dev);

  spin_lock_irqsave(&base_change, flags);
	if((priv->slot == INVALID_SLOT ) ||
	(priv->port == INVALID_PORT)){
	  spin_unlock_irqrestore(&base_change,flags);
		return &priv->stats;
	}
	if(nlm_vits_collect_stats(dev, &stat)){
	  spin_unlock_irqrestore(&base_change,flags);
		return &priv->stats;
	}

	/*update the vits stats to dev stats*/
	priv->stats.rx_packets = stat.rx_unicast + stat.rx_broadcast;
	priv->stats.tx_packets = stat.tx_unicast;
	priv->stats.rx_bytes =	stat.rx_ok_bytes;
	priv->stats.tx_bytes = stat.tx_ok_bytes;
	priv->stats.rx_fifo_errors = stat.ingress_overflow_drop;
	priv->stats.tx_fifo_errors = stat.egress_overflow_drop;
	priv->stats.multicast = stat.rx_multicast;
	for(i = 0; i< NLM_SPI4_MAX_THREADS; i++){
		total_dip4 += g_dip4_error[i] ;
	}
	priv->stats.rx_errors = total_dip4;
	priv->stats.rx_crc_errors = stat.rx_crc ;
	priv->stats.multicast = stat.rx_multicast;
  spin_unlock_irqrestore(&base_change,flags);
	return &priv->stats;

}

static int nlm_vits_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned int thr_id, ret = 0;
	unsigned eth_port;
  unsigned long flags;
	driver_data *priv = netdev_priv(dev);

#if NLM_SPI4_DEBUG
	printk("[%s]: port=%d, skb=%p, len=%d\n",
			__FUNCTION__, priv->port,skb, skb->len);
	for(i=0; i<skb->len; i++)
		printk("%x  ", skb->data[i]);
	printk("\n");
#endif
	thr_id = hard_smp_processor_id();
	
	if(cpu_to_bktmask[thr_id] == 0) {
		printk("Tx fail : No buckets are allocated for this cpu\n");
		return 1;
	}


	if(spi4_tx(thr_id,priv->slot, priv->spi4_port, skb->data,
						(unsigned char*) skb, skb->len)){
		eth_port = priv->port;
		spin_lock_irqsave(&pending_tx_lock[eth_port], flags);
		/* now once again attempt to send, in case, by this time
			 if any TX_DONE happend for the corresponding port*/
  	if(spi4_tx(thr_id,priv->slot, priv->spi4_port, skb->data,
            (unsigned char*) skb, skb->len)){
	    netif_stop_queue(dev);
			ret = 1;
		}
		else{
			ret = 0;
		}
		spin_unlock_irqrestore(&pending_tx_lock[eth_port], flags);
	}

	return ret;
}


int nlm_vits_program_rx_desc(unsigned int slot,
		int total_desc)
{
	int     i;
	struct  sk_buff *skb = 0;
	unsigned int ret = VITS_PROGRAM_RX_DESC_SUCCESS;

	for(i=0; i< total_desc; i++){
		skb = os_alloc_skb();
		if (!skb) {
			printk("ERROR: alloc skb failed\n");
			ret = VITS_PROGRAM_RX_DESC_FAIL;
			break;
		}
		spi4_put_skb_back_ptr(skb);
		spi4_program_rx_desc(slot, skb->data) ;
	}
	return ret;
}

/*******************************************************************************
* Function name :       nlm_vits_frin_replenish
* Input         :
* Description   :       This function replenish RX desc for all ports for
*	a cpu on which it is scheduled.
* RETURNS       :       void
*******************************************************************************/
static void nlm_vits_frin_replenish(struct work_struct *data)
{
  int done = 0;
  int cpu = hard_smp_processor_id();

  driver_data *priv;
  struct net_device *dev;
  atomic_t *frin_to_be_sent;
  int i;

  while(1){
    done = 0;
    for(i=0; i< NLM_SPI4_MAX_PORTS; i++){
      if(active_port[i].port == INVALID_PORT){
        goto skip;
      }
      dev = spi4_dev[i];
      if(dev == NULL){
        goto skip;
      }
      priv =netdev_priv(dev);

	  if(!(MSGRNG_OWN(priv->cfg_flag)))
			continue;

      frin_to_be_sent = &priv->frin_to_be_sent[cpu];
      if(atomic_read(frin_to_be_sent) < 0) {
        printk("ERROR: wrong frin_to_be_sent \n");
      }

		
      if (!atomic_read(frin_to_be_sent)) goto skip;
      			if(nlm_vits_program_rx_desc(priv->slot, 1) ==
					VITS_PROGRAM_RX_DESC_SUCCESS){
        atomic_dec(frin_to_be_sent);
      }
      continue;
      skip:
      done++;
    }
    if(done == NLM_SPI4_MAX_PORTS) break;
  }
  return;
}

/*******************************************************************************
* Function name :    	nlm_vits_rx_tx_done
* Input         :
* Description   :       This function handles the TX DONE and RX IND. It keeps
*	track the RX desc to be replenish, if it crosses the water mark then
*	schedules the replenish work item.
* RETURNS       :       void
*******************************************************************************/

void nlm_vits_rx_tx_done(	uint32  cmd, 	uint32 	slot,
				uint32 	port,	char*   addr,
				uint32  len,	uint32	error)
{
	struct 	net_device   *dev;
	struct 	sk_buff      *skb = 0;
	int 	cpu ;
	driver_data	*priv;
	unsigned int 	eth_interface ;

	cpu = hard_smp_processor_id();
	switch(cmd){
	case SPI4_TX_DONE:
	 	/*in TX_DONE case, port indicates bucket*/
    skb = (struct  sk_buff*)addr;

		if(skb==NULL){
			printk("ERROR - TX_DONE: null skb \n");
			return;
		}
    		priv = netdev_priv(skb->dev);
		eth_interface = priv->port;
    		spin_lock(&pending_tx_lock[eth_interface]);
		if(netif_queue_stopped(skb->dev)){
		 	netif_wake_queue(skb->dev);
		}
    		spin_unlock(&pending_tx_lock[eth_interface]);
		os_free(skb);
		break;
	case SPI4_RX_IND:
#if NLM_SPI4_DEBUG
		printk("[%s]RX_IND :   port=%d\n",
			__FUNCTION__,  priv->port);
#endif

    if(slot == SPI4_0){
      eth_interface = port;
    }
    else if(slot == SPI4_1){
      eth_interface = port + NLM_SPI4_PORTS_PER_CARD;
    }
    else{
      printk("[%s] ERROR!!! invalid slot\n",__FUNCTION__);
      return;
    }
    dev = spi4_dev[eth_interface];
    if(dev == NULL){
      			printk("[%s]ERROR!! eth interface=%d , port=%d , \
					not registered\n",
          __FUNCTION__, eth_interface,port);
      return;
    }

    priv = netdev_priv(dev);
		skb = spi4_get_skb_back_ptr((unsigned long)addr);
		if(!skb){
			printk("NULL skb pointer in RX handling\n");
			return;
		}
		skb->dev = dev;
		skb_reserve(skb, MAC_PREPAD+SPI4_BYTE_OFFSET);
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, skb->dev);
		//schedule work to do replenishing
    if (atomic_inc_return(&priv->frin_to_be_sent[cpu]) >
      SPI4_FRIN_THRESHOLD){
      schedule_work(&vits_frin_replenish_work[cpu]);
    }

#if NLM_SPI4_DEBUG
		{
			int loop;
			printk("[%s] skb: len=%d, data is:\n",
				__FUNCTION__, skb->len);
			for(loop=0; loop<skb->len; loop++)
				printk("%x  ", skb->data[loop]);
			printk("\n");

		}
#endif
		netif_rx(skb);
		break;
	}//end of switch
	return;
}

/*******************************************************************************
* Function name :    	nlm_vits_station_unowned_rx_tx_done
* Input         :
* Description   :       This function handles the TX DONE and RX IND. It keeps
*	track the RX desc to be replenish, if it crosses the water mark then
*	schedules the replenish work item.
* RETURNS       :       void
*******************************************************************************/

void nlm_vits_station_unowned_rx_tx_done(	uint32  cmd, 	uint32 	slot,
				uint32 	port,	char*   addr,
				uint32  len,	uint32	error)
{
	struct 	net_device   *dev;
	struct 	sk_buff      *skb = 0;
	int 	cpu ;
	driver_data	*priv;
	unsigned int 	eth_interface ;
	int fbstid;

	cpu = hard_smp_processor_id();
	switch(cmd){
	case SPI4_TX_DONE:
	 	/*in TX_DONE case, port indicates bucket*/
    skb = (struct  sk_buff*)addr;

		if(skb==NULL){
			printk("ERROR - TX_DONE: null skb \n");
			return;
		}
    		priv = netdev_priv(skb->dev);
		eth_interface = priv->port;
    		spin_lock(&pending_tx_lock[eth_interface]);
		if(netif_queue_stopped(skb->dev)){
		 	netif_wake_queue(skb->dev);
		}
    		spin_unlock(&pending_tx_lock[eth_interface]);
		os_free(skb);
		break;
	case SPI4_RX_IND:
#if NLM_SPI4_DEBUG
		printk("[%s]RX_IND :   port=%d\n",
			__FUNCTION__,  priv->port);
#endif

    if(slot == SPI4_0){
		fbstid = msgrng_xgmac_stid_rfr(0);
      eth_interface = port;
    }
    else if(slot == SPI4_1){
		fbstid = msgrng_xgmac_stid_rfr(1);
      eth_interface = port + NLM_SPI4_PORTS_PER_CARD;
    }
    else{
      printk("[%s] ERROR!!! invalid slot\n",__FUNCTION__);
      return;
    }
    dev = spi4_dev[eth_interface];
    if(dev == NULL){
      			printk("[%s]ERROR!! eth interface=%d , port=%d , \
					not registered\n",
          __FUNCTION__, eth_interface,port);
      return;
    }

    priv = netdev_priv(dev);
	/*
	 * Allocate an skbuff, initialize it, and copy the data to it.
	 */
	skb = __dev_alloc_skb(NLM_RX_BUF_SIZE, GFP_ATOMIC);
	if (!skb) {
		printk("[%s] - no skbuff\n", __FUNCTION__);
		goto err_exit;
	}

		skb->dev = dev;
		skb_reserve(skb, MAC_PREPAD+SPI4_BYTE_OFFSET);
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, skb->dev);
		memcpy(skb->data, (char *)addr + 2, len);
/*
		if(rmik_queue_pkt_mem(fbstid, virt_to_phys(addr) & 0xffffffffe0ULL) < 0)
		nlm_nlm_common_drop_message_unowned(fbstid, virt_to_phys(addr) & 0xffffffffe0ULL, 1);
*/
	
#if NLM_SPI4_DEBUG
		{
			int loop;
			printk("[%s] skb: len=%d, data is:\n",
				__FUNCTION__, skb->len);
			for(loop=0; loop<skb->len; loop++)
				printk("%x  ", skb->data[loop]);
			printk("\n");

		}
#endif
		netif_rx(skb);
		return;
		err_exit:
/*
		nlm_nlm_common_drop_message_unowned(fbstid, virt_to_phys(addr) & 0xffffffffe0ULL, 1);
*/
		if(skb)
			kfree_skb(skb);

	}//end of switch
	return;
}


/****************************************************************************
* Function name :       spi4_check_daughter_cards
* Input         :
* Description   :       This function reads the cpld register and checks
*                       whether any spi4 daughter card present on the board.
* RETURNS       :       0-if no spi4 cards present
*                       1-if only spi4-A present
*                       2-if only spi4-B present
*                       3-if both spi4-A and spi4-B present
****************************************************************************/
unsigned char spi4_check_daughter_cards(void)
{
	unsigned char value, flag = 0;
	unsigned long cpld_base;
	unsigned char *mmio ;

        cpld_base = (unsigned long)(NETLOGIC_CPLD_OFFSET);
        mmio = (unsigned char*) cpld_base;
	value = mmio[CPLD_MISC_STATUS_REG];


	value = value>>3;
       	if(!(value & SPI4_MASK_BIT1))
		flag = 1;

	value = value >>1;
	if(!(value & SPI4_MASK_BIT1))
		flag |=2;

        return flag;

}

/*******************************************************************************
* Function name :	nlm_vits_common_init
* Input         :
* Description   :       This function calls the spi4 drivers functions to
*	ititilize the spi4, configure spills and programs RX desc.
* RETURNS       :       void
*******************************************************************************/

int  nlm_vits_common_init(unsigned int slot)
{
	unsigned int spi4_ret_value;
	static int work_init = 0;
	extern struct net_device_cfg xlr_net_dev_cfg;
	struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;
	struct port_cfg *port_cfg;
	spi4_callback_func callback;

	port_cfg = &net_cfg->xgs_port[slot];

	if(spi4_frin_threashold == 0)
		spi4_frin_threashold =  MAX_NUM_DESC / NR_CPUS;

	if(MSGRNG_OWN(port_cfg->cfg_flag))
		callback = nlm_vits_rx_tx_done;
	else
		callback = nlm_vits_station_unowned_rx_tx_done;

	spi4_ret_value = spi4_init(slot, (spi4_callback_func)callback);
	if(spi4_ret_value != SPI4_INIT_SUCCESS)
		return spi4_ret_value ;

	if(MSGRNG_OWN(port_cfg->cfg_flag)) {
		spi4_ret_value = nlm_vits_program_rx_desc(slot,
							  (int)MAX_NUM_DESC * XLR_TOTAL_CHANNELS);
		if( spi4_ret_value != VITS_PROGRAM_RX_DESC_SUCCESS ){
			return spi4_ret_value ;
		}
	}

	if(PORT_INIT(port_cfg->cfg_flag)) {
		spi4_ret_value = spi4_open(slot);
		if( spi4_ret_value != SPI4_OPEN_SUCCESS ){
			return spi4_ret_value ;
		}
	}

	if(MSGRNG_OWN(port_cfg->cfg_flag)) {
		if(!work_init){
			for(slot=0; slot<NLM_SPI4_MAX_CPUS; slot++){
				INIT_WORK(&vits_frin_replenish_work[slot],
					nlm_vits_frin_replenish);
			}
			work_init = 1;
		}
	}
	return  VITS_COMMON_INIT_SUCCESS;
}


static void setup_net_ops(struct net_device_ops *spi4_ops)
{
	spi4_ops->ndo_open = nlm_vits_open;
	spi4_ops->ndo_stop = nlm_vits_close;
	spi4_ops->ndo_get_stats = nlm_vits_get_stats;
	spi4_ops->ndo_start_xmit = nlm_vits_xmit;
}
/*******************************************************************************
* Function name :      	nlm_vits_init
* Input         :
* Description   :       This function allocates the dev data structure for the
*	eth drivers and initializes it
* RETURNS       :       void
*******************************************************************************/

int nlm_vits_init(void)
{
	unsigned int 				slot, port_start, port_end;
	struct net_device  	*dev  = 0;
	driver_data 				*priv = 0;
	extern struct net_device_cfg xlr_net_dev_cfg;
	struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;
	struct port_cfg *port_cfg;
	int 	i = 0;
	int 	ret = 0, port_register_flag=0;

  	for(slot =0; slot < NLM_SPI4_MAX_SLOTS; slot++){
		if(net_cfg->xgs_type[slot] == TYPE_SPI4)
			break;
	}
	if(slot == NLM_SPI4_MAX_SLOTS) {
		printk(KERN_INFO "This board does not support spi4\n");
		return -1;
	}

  	g_spi4_card_flag = spi4_check_daughter_cards();

	if(!g_spi4_card_flag){
        printk(KERN_INFO "nlm_spi4: No SPI4 cards detected\n");
		return -1;
	}

	setup_net_ops(&nlm_spi4_net_ops);

	for(slot =0; slot < NLM_SPI4_MAX_SLOTS; slot++){
		if(spi4_slot[slot] == INVALID_SLOT) continue;

    		if(!(g_spi4_card_flag & (1<<slot))) continue;

			port_cfg = &net_cfg->xgs_port[slot];
			if(port_cfg->cfg_flag == 0)
				continue;

    		printk("initializing spi4-%d\n", slot);
		   if(nlm_vits_common_init(slot) != VITS_COMMON_INIT_SUCCESS){
      			printk("initialization failed for spi4-%d\n",slot);
			continue;
		}

		if(slot == SPI4_0){
			port_start = 0;
			port_end =  NLM_SPI4_PORTS_PER_CARD;
		}
		else if(slot == SPI4_1){
      port_start = 0 + NLM_SPI4_PORTS_PER_CARD;
      port_end = NLM_SPI4_MAX_PORTS ;
    }
		else{
			port_start = port_end = 0;
		}
		for(i = port_start; i < port_end; i++){
			if((active_port[i].slot == INVALID_SLOT ) ||
			(active_port[i].port == INVALID_PORT)) continue;

			dev = alloc_etherdev(sizeof(driver_data));
			if (!dev) {
				ret = -ENOMEM;
				goto out;
			}
			spi4_dev[i] = dev;
			priv = netdev_priv(dev);
			priv->dev= dev;
			ether_setup(dev);

			sprintf(dev->name, "%s%d", "spi",i);
			dev->netdev_ops = &nlm_spi4_net_ops;

			/* initializing priv member */
			spin_lock_init(&priv->lock);
			priv->port =(uint) active_port[i].port;
			priv->slot =(uint) active_port[i].slot;
			priv->cfg_flag = port_cfg->cfg_flag;

			if(priv->slot == SPI4_0){
				priv->spi4_port = priv->port;
			}
			else{
				priv->spi4_port = priv->port - NLM_SPI4_PORTS_PER_CARD;
			}
			priv->type = (uint)TYPE_SPI4;
			/*this is the default link configuration*/
			priv->speed = SPEED_100M;
			priv->duplex = 1;
			priv->autoneg = 1;

			dev->dev_addr[0] = 0x0;
			dev->dev_addr[1] = 0x0f;
			dev->dev_addr[2] = 0x30;
			dev->dev_addr[3] = 0x00;
			dev->dev_addr[4] = 0x01;
			dev->dev_addr[5] = i ;

			if(PORT_ATTACH(port_cfg->cfg_flag)){
			ret = register_netdev(dev);
			if (ret) {
				printk("Unable to register %s  eth interface \n",
				dev->name);
	    	free_netdev(dev);
				continue;
			}
			printk("%s eth interface is registered\n", dev->name);
			set_ethtool_ops(dev);
		}
			else{
				nlm_vits_open(dev);
			}
			port_register_flag = 1;
		}
	}

  	if(port_register_flag){
	// starting a timer to monitor the link
	init_timer(&link_monitor_timer);
	link_monitor_timer.expires = jiffies + 2 * HZ/100;
	link_monitor_timer.function = nlm_link_monitor;
	add_timer(&link_monitor_timer);
	}
out:
	if (ret < 0) {
		printk("Error, ret = %d\n", ret);
	}
	return ret;
}



void nlm_vits_exit(void)
{
	struct net_device *dev;
	int i;

	for (i = 0; i < NLM_SPI4_MAX_PORTS; i++) {
		dev = spi4_dev[i];
		if (!dev)
			continue;
		printk("unregistering dev%d\n", i);
		unregister_netdev(dev);
		free_netdev(dev);
	}
}

/*******************************************************************************
* Function name :       spi4_get_settings
* Input         :
* Description   :       This function provides info about speed, duplexity,
*	autoneg to ethtool commands.
* RETURNS       :       void
*******************************************************************************/

static int
spi4_get_settings(	struct net_device *dev,
			struct ethtool_cmd *ecmd)
{

	driver_data *priv = netdev_priv(dev);


	ecmd->supported = (SUPPORTED_10baseT_Half |
			SUPPORTED_10baseT_Full |
			SUPPORTED_100baseT_Half |
			SUPPORTED_100baseT_Full |
			SUPPORTED_1000baseT_Full|
			SUPPORTED_Autoneg
			);

	ecmd->advertising = ADVERTISED_10baseT_Full | ADVERTISED_10baseT_Half |
		ADVERTISED_100baseT_Full | ADVERTISED_100baseT_Half |
		ADVERTISED_1000baseT_Full | ADVERTISED_Autoneg;

	switch(priv->speed){
	case  SPEED_1000M:
		ecmd->speed = SPEED_1000;
		break;

	case SPEED_100M:
		ecmd->speed = SPEED_100;
		break;

	case SPEED_10M:
		ecmd->speed = SPEED_10;
		break;

	}// end of switch
	ecmd->duplex = priv->duplex;
	ecmd->phy_address = priv->port;
	ecmd->port = PORT_TP;
	ecmd->autoneg = priv->autoneg;
	return 0;
}

/*******************************************************************************
* Function name :       spi4_set_settings
* Input         :
* Description   :       This function provides ethtool to set speed, duplexity,
*       autoneg to ethtool commands.
* RETURNS       :       void
*******************************************************************************/

static int
spi4_set_settings(	struct net_device *netdev,
			struct ethtool_cmd *ecmd)
{

  unsigned long flags;
	driver_data *priv = netdev_priv(netdev);
	vtss_phy_control_t    phy_cnt;
	int speed	, driver_speed;
	int ret = 0;
	int port;
	vtss_pcs_autoneg_control_t  autoneg;

	/*
	   only speed and duplexity parameters are supported
	 */
  spin_lock_irqsave(&base_change, flags);
	port = active_port[ecmd->port].port ;
	if(port == INVALID_PORT){
	  	spin_unlock_irqrestore(&base_change,flags);
		return -EINVAL;
	}
	change_vits_base(port);
	port++;// this is to make the vtss APIs to get the correct PHY port
	if(vtss_phy_control_get(port, &phy_cnt) == VTSS_OK){
#if NLM_SPI4_DEBUG
		printk("phy status read success\n");
#endif
	}
	else{
	  spin_unlock_irqrestore(&base_change,flags);
#if NLM_SPI4_DEBUG
		printk("phy status read failed\n");
#endif
		return  1;
	}


	if(ecmd->autoneg == AUTONEG_ENABLE){
		phy_cnt.autoneg_enable = 1 ;
	}
	else{
		phy_cnt.autoneg_enable = 0 ;
	}
	if(priv->autoneg != phy_cnt.autoneg_enable){
		/*changeing the autoneg*/
		autoneg.enable = phy_cnt.autoneg_enable;
		if(vtss_pcs_autoneg_control_set(port,&autoneg) == VTSS_OK){
			priv->autoneg = phy_cnt.autoneg_enable;
		}
		else{
		  spin_unlock_irqrestore(&base_change,flags);
			return 1;
		}
	}
	phy_cnt.fdx = ecmd->duplex ;
	switch(ecmd->speed){
	case SPEED_1000:
		speed = (int) VTSS_SPEED_1G ;
		driver_speed = SPEED_1000M;
		break;
	case SPEED_100:
		speed = (int) VTSS_SPEED_100M ;
		driver_speed = SPEED_100M;
		break;
	case SPEED_10 :
		speed = (int) VTSS_SPEED_10M ;
		driver_speed = SPEED_10M;
		break;
	default:
	  spin_unlock_irqrestore(&base_change,flags);
		return -EINVAL;
	}

	phy_cnt.speed =  speed;
	if(vtss_port_set_mode( port, speed, ecmd->duplex) == VTSS_OK){
	}
	else{
		ret = 1;
	}
	if(ret){
		/*mac speed is not success so, no need to change phy speed*/
		return ret;
	}
	if(vtss_phy_control_set(port, &phy_cnt) == VTSS_OK){
	}
	else{
		ret = 1;
	}

	if(ret==0){
		/*update the speed, and duplexity*/
		priv->speed = driver_speed;
		priv->duplex = ecmd->duplex;
	}

  spin_unlock_irqrestore(&base_change,flags);
	return ret;
}


static void
spi4_get_drvinfo(	struct net_device *dev,
		struct ethtool_drvinfo *drvinfo)
{

	printk("[%s]: \n",__FUNCTION__);
	strcpy(drvinfo->driver,DRV_NAME);
	strcpy(drvinfo->version, DRV_VERSION);
	strcpy(drvinfo->fw_version, "N/A");
	return;
}

struct ethtool_ops spi4_ethtool_ops = {
	.get_settings           = spi4_get_settings,
	.set_settings           = spi4_set_settings,
	.get_drvinfo            = spi4_get_drvinfo
};

void set_ethtool_ops(struct net_device *netdev)
{
	SET_ETHTOOL_OPS(netdev, &spi4_ethtool_ops);
}




module_init(nlm_vits_init);
module_exit(nlm_vits_exit);

