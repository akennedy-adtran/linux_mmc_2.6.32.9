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
#include <linux/types.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/inet.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/skbuff.h>
#include <linux/autoconf.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/cache.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/config_net.h>
#include <asm/netlogic/xlr_mac.h>
#include <asm/netlogic/xlr_rmik.h>

#include <net/netevent.h>
#include <net/neighbour.h>
#include <linux/route.h>
#include <net/ip_fib.h>


extern struct net_device *dev_mac_type[][NETLOGIC_MAX_MACS];
extern void mac_stats_update(int pkts, struct sk_buff *skb);
extern struct net_device  *spi4_dev[];
extern void nlm_nlm_common_mac_msgring_handler(int bucket, int size, int code,
				  int stid, struct msgrng_msg *msg,
				  void *data);

static int rmik_net_events_reg = 0;
struct nlm_cluster *nlm_pkt_pool = NULL;
#define NLM_SPI4_MAX_PORTS 20
#define NLM_SPI4_MAX_SLOTS 2

void rmik_cpu_to_cpu_pkt_msgring_handler(int size, struct msgrng_msg *msg)
{
	unsigned long addr;
	__u32 length;
	int port, type, stid;
	struct sk_buff *skb = NULL; 
	int fbstid = 0x0;
	nlm_physaddr_t physaddr;
	struct net_device   *dev;
	extern struct net_device_cfg xlr_net_dev_cfg;
	struct port_cfg *port_cfg;
	struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;
	struct nlm_event_packet_fw_msg *rx_pkt = nlm_addr_to_ptr(msg);

	type =  rx_pkt->fwinfo.ptype;
	fbstid = rx_pkt->fwinfo.fbstid;
	port = rx_pkt->pktinfo.fields.fwport;
	physaddr = msg->msg1 & 0xffffffffe0ULL;
	addr = (unsigned long)bus_to_virt(physaddr);
	length = rx_pkt->pktinfo.fields.length;

	if(length == 0x0)  {
		printk("Invalid length 0 packet received \n");
		goto err_exit;
	}
		
	/*
	 * Do nothing during the boot.
	 */
	if (system_state != SYSTEM_RUNNING) {
		printk("Invalid system state pkt received \n");
		goto err_exit;
	}

#if 0
	printk("nlm_nlm_common_dom_msgring_handler ingress port=%d type=%d fbstid=%d, msg=%llx:%llx\n", 
				port, type, fbstid, msg->msg0, msg->msg1);
#endif

	switch(type) {
		case NLM_GMAC0 :
			dev = dev_mac_type[TYPE_GMAC][port];
			port_cfg = &net_cfg->gmac_port[port];
			stid = MSGRNG_STNID_GMAC0;
			break; 
		case NLM_GMAC1 :
			dev = dev_mac_type[TYPE_GMAC][port + NETLOGIC_GMAC_PORTS_PER_CTRL];
			port_cfg = &net_cfg->gmac_port[port + NETLOGIC_GMAC_PORTS_PER_CTRL];
			stid = MSGRNG_STNID_GMAC1;
			break; 
		case NLM_XGS0  :
			dev = dev_mac_type[TYPE_XGMAC][0];
			if(spi4_dev[port] != NULL)
				dev = spi4_dev[port];
			stid = MSGRNG_STNID_XGS0FR;
			port_cfg = &net_cfg->xgs_port[0];
			break;
		case NLM_XGS1  :
			port = port + NLM_SPI4_MAX_PORTS / NLM_SPI4_MAX_SLOTS;
			dev = dev_mac_type[TYPE_XGMAC][1];
			if(spi4_dev[port] != NULL)
				dev = spi4_dev[port];
			stid = MSGRNG_STNID_XGS1FR;
			port_cfg = &net_cfg->xgs_port[1];
			break;
		default:
			printk("Unknown type\n");
			goto err_exit;
	}

	if(dev == 0) {
		printk("[%s] - no dev\n", __FUNCTION__);
		goto err_exit;
	}

	if(MSGRNG_OWN(port_cfg->cfg_flag) == 0) {
		/*
		 * Allocate an skbuff, initialize it, and copy the data to it.
		 */
		skb = __dev_alloc_skb(NLM_RX_BUF_SIZE, GFP_ATOMIC);
		if (!skb) {
			printk("[%s] - no skbuff\n", __FUNCTION__);
			goto err_exit;
		}
		skb->dev = dev;

		length = length - (BYTE_OFFSET + MAC_CRC_LEN);
		skb_put(skb, length);
		memcpy(skb->data, (char *)addr + 2, length);

		if(rmik_queue_pkt_mem(fbstid, physaddr) < 0)
			nlm_nlm_common_drop_message_unowned(fbstid, physaddr, 1);

		#if 0
		{
			int i = 0;
			printk("[%s] Rx Packet: length=%d\n", dev->name, length);
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
		mac_stats_update(1, skb);
		/*
		 * Queue the packet to the upper layer.
		 */
		netif_rx(skb);
		return;
	} else {
		msg->msg0 = msg->msg1;
		size = size - 1;
		/* printk("calling mac msgring handler size=%d stid=%d\n", size, stid); */
		return nlm_nlm_common_mac_msgring_handler(0 /* bucket ignored */, size, 0 /* code ignored */, stid, msg, NULL);
	}

	err_exit:
		nlm_nlm_common_drop_message_unowned(fbstid, physaddr, 1);
		if(skb != NULL)
			kfree_skb(skb);
	return;

}

void rmik_config_pde(int type, int instance, nlm_reg_t *base)
{
	uint32_t *pde_addr;
	int rv;
	int j, off, len;
	uint64_t pde_bkt_map = 0ULL;

	if(type == TYPE_GMAC) {
		instance = instance < NETLOGIC_GMAC_PORTS_PER_CTRL ? 0 : NETLOGIC_GMAC_PORTS_PER_CTRL;
		rv = fdt_get_gmac_pde_reginfo(instance, &pde_bkt_map);
	} else if(type == TYPE_XGMAC)
		rv = fdt_get_xgmac_pde_reginfo(instance, &pde_bkt_map);
	else if(type == TYPE_SPI4)
		rv = fdt_get_spi4_pde_reginfo(instance, &pde_bkt_map);
	else
		return ;
		
	if(rv == 0 )
		return;
	pde_addr = nlm_addr_to_ptr(rv);
	while(1) {
		base =  (nlm_reg_t *)(unsigned long)(int)((pde_addr[0] >> 12) << 12);
        off = pde_addr[0] & 0xfff;
        len =   pde_addr[1];


		pde_addr = pde_addr + 2;
		if(len == 0)
			break;
		for(j = 0; j < len; j++, off++) {
			netlogic_write_reg(base, off, *pde_addr++);
		}
	}
	return;
}

uint64_t rmik_get_pde_bktmap(int type, int instance)
{
	uint64_t pde_bkt_map = 0ULL;

	return pde_bkt_map;

	if(type == TYPE_GMAC) {
		instance = instance < NETLOGIC_GMAC_PORTS_PER_CTRL ? 0 : NETLOGIC_GMAC_PORTS_PER_CTRL;
		fdt_get_gmac_pde_reginfo(instance, &pde_bkt_map);
	} else if(type == TYPE_XGMAC)
		fdt_get_xgmac_pde_reginfo(instance, &pde_bkt_map);
	else if(type == TYPE_SPI4)
		fdt_get_spi4_pde_reginfo(instance, &pde_bkt_map);
	
	return pde_bkt_map;
}


struct work_struct rmik_replenish_work[NR_CPUS];
uint64_t *rmik_replenish_data[NR_CPUS];
atomic_t rmik_replenish_cnt[NR_CPUS];
int rmik_schedule_thr = 4;
#define RMIK_MAX_DESC_IN_QUEUE 64

static void rmik_frin_replenish(struct work_struct *args /* ignored */ )
{
	int cpu;
	atomic_t *rep_cnt;
	unsigned long msgrng_flags;
	int i, fbstid, cnt;
	uint64_t *data, physaddr;

	msgrng_access_enable(msgrng_flags);

	cpu = hard_smp_processor_id();
	rep_cnt =  &rmik_replenish_cnt[cpu];
	data = rmik_replenish_data[cpu];

	/* printk("[%d]rmik_frin_replenish[%d] \n", cpu, atomic_read(rep_cnt)); */

	if ((cnt = atomic_read(rep_cnt)) < 0) {
		printk("Error replenish cnt becomes negative\n");
		msgrng_access_disable(msgrng_flags);
		return;
	}
	if (cnt == 0) {
		msgrng_access_disable(msgrng_flags);
		return;
	}


	for(i = 0; i < (cnt * 2); i = i + 2) {
		fbstid = data[i];
		physaddr = data[i + 1];
		nlm_nlm_common_drop_message_unowned(fbstid, physaddr, 0);
	}
	atomic_set(rep_cnt, 0);
	msgrng_access_disable(msgrng_flags);
}

void rmik_init_replenish_work(int numdesc)
{
	int i;
	if(numdesc > NR_CPUS)
		rmik_schedule_thr = numdesc / NR_CPUS;
	if(rmik_schedule_thr > 16)
		rmik_schedule_thr = 16;

	for(i = 0; i < NR_CPUS; i++) {
		INIT_WORK(&rmik_replenish_work[i], rmik_frin_replenish);
		rmik_replenish_data[i] = kmalloc(RMIK_MAX_DESC_IN_QUEUE * sizeof(uint64_t), GFP_KERNEL);
		if(rmik_replenish_data[i] == NULL)
			panic("Invalid mem in rmik replenish\n");
	}
	return;
}

int rmik_queue_pkt_mem(uint32_t fbstid, uint64_t physaddr)
{
	int cpu, i;
	unsigned long msgrng_flags;
	atomic_t *rep_cnt;
	uint64_t *data;


	msgrng_access_enable(msgrng_flags);
	cpu = hard_smp_processor_id();
    rep_cnt =  &rmik_replenish_cnt[cpu];

	/* printk(" [cpu%d]rmik_queue_pkt_mem[cnt%d, fbid%d] \n", cpu, atomic_read(rep_cnt), fbstid);*/
	data = rmik_replenish_data[cpu];
	if(((data == NULL) || (i = atomic_read(rep_cnt)) >= RMIK_MAX_DESC_IN_QUEUE / 2)) {
		msgrng_access_disable(msgrng_flags);
		return -1;
	}
	data[i * 2] = fbstid;
	data[i * 2 + 1] = physaddr;
	atomic_inc(rep_cnt);
	msgrng_access_disable(msgrng_flags);

	if(atomic_read(rep_cnt) >= rmik_schedule_thr) {
		if(rmik_replenish_work[cpu].func == NULL)
			return -1;
		schedule_work(&rmik_replenish_work[cpu]);
	}
	
	return 0;
}

/*
 * Event handlers 
 */

static int rmik_inetaddr_event(struct notifier_block *, unsigned long, void *);
static int rmik_net_event(struct notifier_block *, unsigned long, void *);
static struct notifier_block rmik_inetaddr_notifier = {
	.notifier_call = rmik_inetaddr_event
};

static struct notifier_block rmik_net_notifier = {
	.notifier_call = rmik_net_event
};

static int get_ptype_from_dev(struct net_device *netdev, int *ptype, int *instance, char *rname)
{
	int spi4_port_per_ctrl = NLM_SPI4_MAX_PORTS / NLM_SPI4_MAX_SLOTS;
	int nettype, port;

	for(nettype = 0; nettype < MAX_NET_TYPES; nettype++) {
		if(nettype == TYPE_GMAC || nettype == TYPE_XGMAC) {
			for(port = 0; port < NETLOGIC_MAX_MACS; port++) {
				if(netdev == dev_mac_type[nettype][port])
					goto found;
			}
		} else  {
			for(port = 0; port < NLM_SPI4_MAX_PORTS; port++) {
				if(netdev == spi4_dev[port]) 
					goto found;
			}
		}
	}
	return -1;
			

found:
	switch(nettype) {
		case TYPE_GMAC :
			if(port >= NETLOGIC_GMAC_PORTS_PER_CTRL)  {
				*ptype = NLM_GMAC1;
				*instance = port - NETLOGIC_GMAC_PORTS_PER_CTRL;
				strcpy(rname, "gmac-block@1");
			} else {
				*ptype = NLM_GMAC0;
				*instance = port;
				strcpy(rname, "gmac-block@0");
			}
			break;
		case TYPE_XGMAC: 
			*instance = 0;
			if(port == 0) {
				*ptype = NLM_XGS0;
				strcpy(rname, "xgmac@0");

			} else {
				*ptype = NLM_XGS1;
				strcpy(rname, "xgmac@1");
			}
			break;
		case TYPE_SPI4:
			if(port < spi4_port_per_ctrl) {
				*ptype = NLM_XGS0;
				*instance = port;
				strcpy(rname, "spi4@0");
			} else {
				*ptype = NLM_XGS1;
				*instance = port - spi4_port_per_ctrl;
				strcpy(rname, "spi4@1");
			}
			break;
	}
	return 0;
}

int send_interface_state_change(struct net_device *netdev, uint32_t myip, int msgtype)
{
	struct nlm_event_eth_ifc_msg *ifc;	
	struct nlm_cluster *cl;
	struct nlm_event_simple_msg smsg;
	int dom, ptype, instance, i;
	char rname[NLM_MAX_NAMELEN];
	uint64_t dom_map;
	uint64_t *ip_mac, tmac = 0;

	if(get_ptype_from_dev(netdev, &ptype, &instance, rname) != 0)
		return NOTIFY_DONE;


	if(nlm_get_event_recipients(rname, &dom_map) < 0) 
		return NOTIFY_DONE;
	
	/* No receivng domains */
	if(dom_map == 0ULL)
		return NOTIFY_DONE;
	dom_map &= ~(1ULL << nlm_this_domain->id);

	for(dom = 0; dom < NLM_MAX_DOMAINS; dom++) {
		if(!((1ULL << dom) & dom_map))
			continue;
		if((cl = nlm_clpool_getref(dom, NLM_PKT_POOL_NAME)) == NULL) {
			printk("No pool found for domain %d\n", dom);
			nlm_send_pool_event(dom, NLM_EVENT_MSG_CL_POOL_NOT_FOUND, 0ULL);
			continue;
		}

		if((ifc = nlm_cluster_alloc(cl, 0)) == NULL)  {
			printk("Cluster alloc failed %d\n", dom);
			nlm_send_pool_event(dom, NLM_EVENT_MSG_CL_POOL_EMPTY, 0ULL);
			nlm_clpool_putref(cl);
			continue;
		}

		memset(ifc, 0, sizeof(*ifc));
		ifc->ptype = ptype;
		ifc->port = instance;
		ifc->count = 1;
		ip_mac = nlm_addr_to_ptr(ifc->data);
		ip_mac[0] = myip;
		ip_mac[1] = 0ULL;
		for(i = 5; i >= 0 ; i--) {
			tmac = (uint64_t)(uint32_t)netdev->dev_addr[5 - i];
			ip_mac[1] |= (tmac << (i * 8)); 
		}

		smsg.msgtype = msgtype;
		smsg.domid = nlm_this_domain->id;
		smsg.arg = nlm_ptr_to_addr(ifc);

		if(nlm_send_event(dom, NLM_EVENT_VETH_INFO, sizeof(smsg),
						nlm_addr_to_ptr(&smsg)) < 0) {
			printk("Critical : Event failed to send for dom %d\n", dom);
			nlm_cluster_free(cl, ifc);
		}
		nlm_clpool_putref(cl);
	}
	return NOTIFY_DONE;
}

struct nlm_event_eth_ifc_msg *update_arp_entry(struct net_device *netdev, 
				uint32_t ipaddr, uint8_t *mac_addr, int msgtype)
{
	int i;
	uint64_t *ip_mac, tmac;
	struct nlm_cluster *cl;
	struct nlm_event_eth_ifc_msg *ifc;	
	struct nlm_event_simple_msg smsg;
	int dom, ptype, instance;
	char rname[NLM_MAX_NAMELEN];
	uint64_t dom_map;

	
	if(get_ptype_from_dev(netdev, &ptype, &instance, rname) != 0)
		return NOTIFY_DONE;


	if(nlm_get_event_recipients(rname, &dom_map) < 0) 
		return NOTIFY_DONE;
	
	/* No receivng domains */
	if(dom_map == 0ULL)
		return NOTIFY_DONE;
	dom_map &= ~(1ULL << nlm_this_domain->id);

	for(dom = 0; dom < NLM_MAX_DOMAINS; dom++) {
		if(!((1ULL << dom) & dom_map))
			continue;
		if((cl = nlm_clpool_getref(dom, NLM_PKT_POOL_NAME)) == NULL) {
			printk("No pool found for domain %d\n", dom);
			nlm_send_pool_event(dom, NLM_EVENT_MSG_CL_POOL_NOT_FOUND, 0ULL);
			continue;
		}

		if((ifc = nlm_cluster_alloc(cl, 0)) == NULL)  {
			printk("Cluster alloc failed %d\n", dom);
			nlm_send_pool_event(dom, NLM_EVENT_MSG_CL_POOL_EMPTY, 0ULL);
			nlm_clpool_putref(cl);
			continue;
		}

		memset(ifc, 0, sizeof(*ifc));
		ifc->count = 1;
		ifc->ptype = ptype;
		ifc->port = instance;

		ip_mac = nlm_addr_to_ptr(ifc->data);
		ip_mac[0] = ipaddr;
		ip_mac[1] = 0ULL;
		for(i = 5; i >= 0 ; i--) {
			tmac = (uint64_t)(uint32_t)mac_addr[5 - i];
			ip_mac[1] |= (tmac << (i * 8)); 
		}

		smsg.msgtype = msgtype;
		smsg.domid = nlm_this_domain->id;
		smsg.arg = nlm_ptr_to_addr(ifc);


		if(nlm_send_event(dom, NLM_EVENT_VETH_INFO, sizeof(smsg),
					nlm_addr_to_ptr(&smsg)) < 0) {
			printk("Critical : Event failed to send for dom %d\n", dom);
			nlm_cluster_free(cl, ifc);
		}
		nlm_clpool_putref(cl);
	}

	return NOTIFY_DONE;
}

/**
 * rmik_inetaddr_event
 */
static int rmik_inetaddr_event(struct notifier_block *notifier,
		unsigned long event, void *ptr)
{
	struct in_ifaddr *ifa = ptr;
	struct net_device *netdev;
	unsigned int addr;
	unsigned int mask;
	       
	if(!rmik_net_events_reg)
		return NOTIFY_DONE;
	
	netdev = ifa->ifa_dev->dev;
	addr = ntohl(ifa->ifa_address);
	mask = ntohl(ifa->ifa_mask);

	/*printk("rmik inetaddr_event: ip address " NIPQUAD_FMT
			", netmask " NIPQUAD_FMT ".\n",
			HIPQUAD(addr), HIPQUAD(mask)); */

	switch (event) {
		case NETDEV_DOWN:
			/* printk("event:DOWN \n"); */
			send_interface_state_change(netdev, addr, NLM_EVENT_MSG_IFC_DOWN);
			return NOTIFY_OK;

		case NETDEV_UP:
			/*printk("event:UP \n"); */
			send_interface_state_change(netdev, addr, NLM_EVENT_MSG_IFC_UP);
			return NOTIFY_OK;

		default:
			break;
	}
	return NOTIFY_DONE;
}


/**
 * nes_net_event
 */
static int rmik_net_event(struct notifier_block *notifier,
		unsigned long event, void *ptr)
{
	struct neighbour *neigh = ptr;
	struct net_device *netdev = neigh->dev;

	if(!rmik_net_events_reg)
		return NOTIFY_DONE;
	
	switch (event) {
		case NETEVENT_NEIGH_UPDATE:
			/* printk("Neighbour update\n");  */
			if (neigh->nud_state & NUD_VALID) {
				update_arp_entry(netdev, ntohl(*(__be32 *)neigh->primary_key), neigh->ha, NLM_EVENT_MSG_IFC_ARP_ADD);
				return NOTIFY_OK;
			} else {
				update_arp_entry(netdev, ntohl(*(__be32 *)neigh->primary_key), neigh->ha, NLM_EVENT_MSG_IFC_ARP_DEL);
				return NOTIFY_OK;
			}
			break;
		default:
			printk("NETEVENT_ %lu undefined\n", event);
			break;
	}

	return NOTIFY_DONE;
}

static void create_pkt_pool(void)
{
	int num_units = 128;
	static int pkt_pool_init_done = 0;

	if(pkt_pool_init_done == 0) {
		if(nlm_clpool_create(NLM_PKT_POOL_NAME, num_units, NLM_PKT_POOL_UNIT_SIZE, NLM_MALLOC_UNMAPPED) != 0) {
			printk("Pktpool create failed\n");
			return;
		}
		if((nlm_pkt_pool = nlm_clpool_getref(nlm_this_domain->id, NLM_PKT_POOL_NAME)) == NULL) {
			printk("No pool found for domain %d\n", nlm_this_domain->id);
		}
		if(nlm_pkt_pool)
			nlm_clpool_putref(nlm_pkt_pool);

		pkt_pool_init_done = 1;
	}
}

static void register_net_events(char *rname)
{

#if 0
	nlm_atomic_t dom_map;
	
	if((nlm_get_event_recipients(rname, &dom_map) < 0) || (dom_map == 0)) 
		return ;
#endif
	
	if(rmik_net_events_reg == 0) {
		rmik_net_events_reg = 1;
		register_inetaddr_notifier(&rmik_inetaddr_notifier);
		register_netevent_notifier(&rmik_net_notifier);
	}

	return;
}

void rmik_register_net_events(void)
{
	extern struct net_device_cfg xlr_net_dev_cfg;
	struct net_device_cfg *net_cfg = &xlr_net_dev_cfg;

	if (!rmik_en)
		return;

	create_pkt_pool();

	if (rmik_net_events_reg == 0) {
		if(net_cfg->gmac_port[0].cfg_flag != 0)
			register_net_events("gmac-block@0");

		if(net_cfg->gmac_port[NLM_GMAC_PORTS_PER_CTRL].cfg_flag != 0)
			register_net_events("gmac-block@1");

		if(net_cfg->xgs_port[0].cfg_flag != 0) {
			if(net_cfg->xgs_type[0] == TYPE_SPI4)
				register_net_events("spi4@0");
			else
				register_net_events("xgmac@0");
		}

		if(net_cfg->xgs_port[1].cfg_flag != 0) {
			if(net_cfg->xgs_type[1] == TYPE_SPI4)
				register_net_events("spi4@1");
			else
				register_net_events("xgmac@1");
		}

	}
}
