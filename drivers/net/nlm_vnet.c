/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * Copyright (C) 2001 Alessandro Rubini and Jonathan Corbet
 * Copyright (C) 2001 O'Reilly & Associates
 *
 * The source code in this file can be freely used, adapted,
 * and redistributed in source or binary form, so long as an
 * acknowledgment appears in derived source files.  The citation
 * should list that the code comes from the book "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published
 * by O'Reilly & Associates.   No warranty is attached;
 * we cannot take responsibility for errors or fitness for use.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>

#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>

#include "nlm_vnet.h"


/*
 * Transmitter lockup simulation, normally disabled.
 */
static int lockup = 0;
module_param(lockup, int, 0);

static int timeout = NLM_VNET_TIMEOUT;
module_param(timeout, int, 0);

int nlm_vnet_debug = 0;
struct net_device *vnet_dev;  /* The device */

/*
 * This structure is private to each device. It is used to pass
 * packets in and out, so there is place for a packet
 */
struct nlm_vnet_priv {
	struct net_device_stats stats;
	int status;
	struct sk_buff *skb;
	spinlock_t lock;
};

struct nlm_vnet_cluster_cache {
	struct nlm_cluster *cl;
};

static struct nlm_vnet_cluster_cache cl_cache_send[NLM_MAX_DOMAINS];
static struct nlm_vnet_cluster_cache cl_cache_rx;

/*
 * Addr of 64-bit vnet dom-map which has respective domain id bits
 * set for the domains where vnet0 interface is up.
 */
static nlm_addr_t nlm_vnet_dom_map_addr;
/*
 * Addr of 64-bit vnet promisc-dom-map which has respective domain id
 * bits set for the domains where vnet0 interface is in promiscuous mode.
 * .
 */
static nlm_addr_t nlm_vnet_promisc_dom_map_addr;


static inline void nlm_vnet_dump_ether_hdr(struct ethhdr* eth)
{
	DECLARE_MAC_BUF(dest);
	DECLARE_MAC_BUF(src);

	VNET_DBG("eth header: dest %s source %s proto %04x\n",
		 print_mac(dest, eth->h_dest),
		 print_mac(src, eth->h_source),
		 ntohs(eth->h_proto));
}


static void nlm_vnet_dump_pkt_info(struct sk_buff *skb)
{
	struct iphdr *ih;
	uint32_t *saddr, *daddr;

	nlm_vnet_dump_ether_hdr((struct ethhdr*)skb->data);

	/*
	 * Ethhdr is 14 bytes, but the kernel arranges for iphdr
	 * to be aligned (i.e., ethhdr is unaligned)
	 */
	ih = (struct iphdr *)(skb->data + sizeof (struct ethhdr));
	saddr = &ih->saddr;
	daddr = &ih->daddr;

	VNET_DBG("%s: %08x:%05i --> %08x:%05i\n", __FUNCTION__,
		ntohl(ih->saddr),ntohs(((struct tcphdr *)(ih+1))->source),
		ntohl(ih->daddr),ntohs(((struct tcphdr *)(ih+1))->dest));
}


static void nlm_vnet_rx(struct net_device *dev, uint64_t datalen, void *data)
{

	struct sk_buff *skb;
	struct nlm_vnet_priv *priv = netdev_priv(dev);
	int rval;

	/*
	 * The packet has been retrieved from the transmission
	 * medium. Build an skb around it, so upper layers can handle it
	 */
	skb = dev_alloc_skb(datalen + 2);
	if (!skb) {
		if (printk_ratelimit())
		    printk(KERN_NOTICE "nlm_vnet: rx - low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		goto out;
	}

	skb_reserve(skb, 2); /* align IP on 16B boundary */  
	memcpy(skb_put(skb, datalen), data, datalen);

	/* Write metadata, and then pass to the receive level */
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += datalen;

	rval = netif_rx(skb);
  out:
	return;
}


static void nlm_vnet_pkt_get_event(int len, void *msg)
{
	struct nlm_event_simple_msg *smsg = msg;
	struct nlm_event_vnet_msg *vnet_msg = nlm_addr_to_ptr(smsg->arg);

	VNET_DBG("%s: rx from domid %u in %u msg len %u type %u datalen %llu "
		 "data %p arg 0x%llx\n", __FUNCTION__, smsg->domid,
		 nlm_this_domain->id, len, smsg->msgtype,
		 (unsigned long long)vnet_msg->datalen,
		 nlm_addr_to_ptr(vnet_msg->data),
		 (unsigned long long)smsg->arg);

	if (!cl_cache_rx.cl) {
	    if ((cl_cache_rx.cl = nlm_clpool_getref(nlm_this_domain->id,
						 NLM_PKT_POOL_NAME)) == NULL) {
		printk("No pool found for domain %u\n", nlm_this_domain->id);
		nlm_send_pool_event(nlm_this_domain->id,
				    NLM_EVENT_MSG_CL_POOL_NOT_FOUND, 0ULL);
		return;
	    }
	}

	nlm_vnet_dump_ether_hdr((struct ethhdr*)nlm_addr_to_ptr(vnet_msg->data));
	nlm_vnet_rx(vnet_dev, vnet_msg->datalen, vnet_msg->data);
	nlm_cluster_free(cl_cache_rx.cl, vnet_msg);
}


static void nlm_vnet_pkt_unicast(nlm_dom_t domid, struct sk_buff *skb)
{
    struct nlm_event_vnet_msg *vnet_msg;
    struct nlm_event_simple_msg smsg;
    char *shortpkt = NULL;
    char *skbdata;
    int skblen;
    struct nlm_vnet_priv *priv;

    skblen = skb->len;
    skbdata = skb->data;

    if (skblen < ETH_ZLEN) {
	    shortpkt = kmalloc(ETH_ZLEN, GFP_KERNEL);
	    if (shortpkt == NULL) {
		printk (KERN_NOTICE "Ran out of memory allocating short packet\n");
		return;
	    }
	    memset(shortpkt, 0, ETH_ZLEN);
	    memcpy(shortpkt, skbdata, skblen);
	    skblen = ETH_ZLEN;
	    skbdata = shortpkt;
    }

    if (!cl_cache_send[domid].cl) {

	VNET_DBG("%s: caching cluster ref for domid %u\n", __FUNCTION__, domid);

	if ((cl_cache_send[domid].cl =
	    nlm_clpool_getref(domid, NLM_PKT_POOL_NAME)) == NULL) {
	    printk("No pool found for domain %u\n", domid);
	    nlm_send_pool_event(domid, NLM_EVENT_MSG_CL_POOL_NOT_FOUND, 0ULL);
	    return;
	}
    }

    if ((vnet_msg = nlm_cluster_alloc(cl_cache_send[domid].cl, 0)) == NULL) {
	printk("Cluster alloc failed %d\n", domid);
	nlm_send_pool_event(domid, NLM_EVENT_MSG_CL_POOL_EMPTY, 0ULL);
	return;
    }

    memset(vnet_msg, 0, sizeof(*vnet_msg));
    vnet_msg->datalen = skblen;
    memcpy(vnet_msg->data, skbdata, skblen);

    smsg.msgtype = NLM_EVENT_VNET_PKT;
    smsg.domid = nlm_this_domain->id;
    smsg.arg = nlm_ptr_to_addr(vnet_msg);

    VNET_DBG("%s: tx from domid %u to %u msglen %llu type %u datalen %llu "
	     "data %p arg 0x%llx\n", __FUNCTION__, smsg.domid, domid,
	     (unsigned long long)sizeof(smsg), smsg.msgtype,
	     (unsigned long long)vnet_msg->datalen,
	     nlm_addr_to_ptr(vnet_msg->data), (unsigned long long)smsg.arg);

    if (nlm_send_event(domid, NLM_EVENT_VNET, sizeof(smsg),
		       nlm_addr_to_ptr(&smsg)) < 0) {
	printk("Critical : Event send failed for domain %d\n", domid);
	nlm_cluster_free(cl_cache_send[domid].cl, vnet_msg);
    }

    priv = netdev_priv(vnet_dev);
    priv->stats.tx_packets++;
    priv->stats.tx_bytes += skblen;

    if (shortpkt)
	kfree(shortpkt);

}


/*
 * Broadcast vnet packet to all up and running domains which
 * have vnet0 interface up.
 */
static void nlm_vnet_pkt_broadcast(uint64_t vnet_dom_map, struct sk_buff *skb)
{
    struct nlm_domain dest_dom;
    nlm_dom_t domid;

    for (domid = NLM_DOMAIN_REMOVED + 1; domid < NLM_MAX_DOMAINS; domid++) {

	nlm_get_domain(domid, &dest_dom);

	if ((domid == nlm_this_domain->id) ||
	    (dest_dom.state != NLM_D_RUNNING)) {
	    continue;
	}

	if (!((1ULL << domid) & vnet_dom_map))
	    continue;
	

	VNET_DBG("broadcast to domid %u dest_dom.id %u state %d\n",
		 domid, dest_dom.id, dest_dom.state);

	nlm_vnet_pkt_unicast(domid, skb);

    }
}


static void nlm_vnet_pkt_send_event(struct sk_buff *skb, int broadcast)
{
    struct nlm_domain promisc_dest_dom;
    nlm_dom_t promisc_domid;
    struct nlm_domain dest_dom;
    nlm_dom_t domid;
    char *skb_data;
    volatile uint64_t vnet_dom_map = 0x0ULL;
    volatile uint64_t vnet_promisc_map = 0x0ULL;
    uint64_t mask;
    int i;

    skb_data = skb->data;
    domid = skb_data[ETH_ALEN - 1];

    vnet_dom_map = *(uint64_t*)(long)nlm_vnet_dom_map_addr;
    vnet_promisc_map = *(uint64_t*)(long)nlm_vnet_promisc_dom_map_addr;

    if (!broadcast) { /* unicast packets */

	/*
	 * First check for promiscuous vnet0 interfaces and steer the packets
	 * to the owning domains.
	 */
	for (i = NLM_DOMAIN_REMOVED + 1; i < NLM_MAX_DOMAINS; i++) {
	    
	    mask = 0x1ULL << i;
	    promisc_domid = i;

	    if (!(vnet_promisc_map & mask))
		continue;

	    nlm_get_domain(promisc_domid, &promisc_dest_dom);
	    if ((promisc_domid == nlm_this_domain->id) ||
		(promisc_dest_dom.state != NLM_D_RUNNING)) {
		continue;
	    }

	    VNET_DBG("unicast to promisc domid %u dest_dom.id %u state %d\n",
		    promisc_domid, promisc_dest_dom.id, promisc_dest_dom.state);
	    nlm_vnet_pkt_unicast(promisc_domid, skb);
	}

	/*
	 * Now send the packets to the specified domain id.
	 */
	if ((domid <= NLM_DOMAIN_REMOVED) || (domid >= NLM_MAX_DOMAINS)) {
	    return;
	}

	nlm_get_domain(domid, &dest_dom);
	if ((domid == nlm_this_domain->id) || (dest_dom.state != NLM_D_RUNNING)) {
	    return;
	}

	if (!((1ULL << domid) & vnet_dom_map))
	    return;

	nlm_vnet_pkt_unicast(domid, skb);
    } else {  /* broadcast packets */
	nlm_vnet_pkt_broadcast(vnet_dom_map, skb);
    }
}


/*
 * nlm_vnet_open
 *
 * Open the "vnet0" device.
 */
static int nlm_vnet_open(struct net_device *dev)
{
	/* Interface coming up; set the domain bit in vnet bitmap. */
	nlm_vnet_dom_map_set_clear_bit(nlm_vnet_dom_map_addr,
				       nlm_this_domain->id, 1);
	netif_start_queue(dev);
	return 0;
}


static int nlm_vnet_release(struct net_device *dev)
{
	int domid;
    
	for (domid = 0; domid < NLM_MAX_DOMAINS; domid++) {
	    if (cl_cache_send[domid].cl) {
		nlm_clpool_putref(cl_cache_send[domid].cl);
		cl_cache_send[domid].cl = NULL;
	    }
	}

	if (cl_cache_rx.cl) {
	    nlm_clpool_putref(cl_cache_rx.cl);
	    cl_cache_rx.cl = NULL;
	}

	/* Interface going down; clear domain bit in vnet bitmap. */
	nlm_vnet_dom_map_set_clear_bit(nlm_vnet_dom_map_addr,
				       nlm_this_domain->id, 0);
	/* Clear promisc bit */
	nlm_vnet_promisc_map_set_clear_bit(nlm_vnet_promisc_dom_map_addr,
					   nlm_this_domain->id, 0);
	netif_stop_queue(dev); /* can't transmit any more */
	return 0;
}


/*
 * Configuration changes (passed on by ifconfig)
 */
static int nlm_vnet_config(struct net_device *dev, struct ifmap *map)
{

	if (dev->flags & IFF_UP) /* can't act on a running interface */
		return -EBUSY;

	/* Don't allow changing the I/O address */
	if (map->base_addr != dev->base_addr) {
		printk(KERN_WARNING "nlm_vnet: Can't change I/O address\n");
		return -EOPNOTSUPP;
	}

	/* ignore other fields */
	return 0;
}



/*
 * Transmit a packet (low level interface)
 */
static void nlm_vnet_hw_tx(struct sk_buff *skb)
{
	int is_broadcast = 1;	
	struct ethhdr *eth;
	int i;
    
	VNET_DBG("%s\n", __FUNCTION__);
	/* I am paranoid. Ain't I? */
	if (skb->len < sizeof(struct ethhdr) + sizeof(struct iphdr)) {
		printk("nlm_vnet: Hmm... packet too short (%i octets)\n", skb->len);
		return;
	}

	nlm_vnet_dump_pkt_info(skb);

	/*
	 * If it is a broadcast packet, broadcast it to all running CRF
	 * domains.
	 */
	eth = (struct ethhdr*)skb->data;

	for (i = 0; i < 6; i++) {
	    if ((eth->h_dest[i] & 0xff) != 0xff) {
		is_broadcast = 0;
		break;
	    }
	}

	nlm_vnet_pkt_send_event(skb, is_broadcast);
}


/*
 * Transmit a packet (called by the kernel)
 */
static int nlm_vnet_tx(struct sk_buff *skb, struct net_device *dev)
{
	dev->trans_start = jiffies; /* save the timestamp */
	nlm_vnet_hw_tx(skb);
	dev_kfree_skb(skb); /* Free the skbuf */

	return 0; /* Our simple device can not fail */
}


/*
 * Deal with a transmit timeout.
 */
static void nlm_vnet_tx_timeout (struct net_device *dev)
{
	struct nlm_vnet_priv *priv = netdev_priv(dev);

	VNET_DBG("nlm_vnet: Transmit timeout at %ld, latency %ld\n", jiffies,
		jiffies - dev->trans_start);

	priv->stats.tx_errors++;
	netif_wake_queue(dev);
	return;
}


/*
 * Ioctl commands 
 */
static int nlm_vnet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	VNET_DBG("nlm_vnet: ioctl\n");
	return 0;
}


/*
 * Return statistics to the caller
 */
static struct net_device_stats *nlm_vnet_stats(struct net_device *dev)
{
	struct nlm_vnet_priv *priv = netdev_priv(dev);
	return &priv->stats;
}


/*
 * Change MTU
 */
static int nlm_vnet_change_mtu(struct net_device *dev, int new_mtu)
{
	unsigned long flags;
	struct nlm_vnet_priv *priv = netdev_priv(dev);
	spinlock_t *lock = &priv->lock;
    
	/* check ranges */
	if ((new_mtu < 68) || (new_mtu > 1500))
		return -EINVAL;

	/* accept the value */
	spin_lock_irqsave(lock, flags);
	dev->mtu = new_mtu;
	spin_unlock_irqrestore(lock, flags);

	return 0; /* success */
}


/*
 * Hard code mac address and associate the CRF domainid with it.
 */
static void nlm_vnet_set_mac_address(struct net_device *dev) {
	/* 
	 * Assign the hardware address to the vnet interface:
	 * Use "\0VNETx", where x is 0. The first byte is '\0'
	 * to avoid being a multicast address (the first byte
	 * of multicast addrs is odd).
	 */
	memcpy(dev->dev_addr, "\0VNET0", ETH_ALEN);

	/*
	 * Make last byte of MAC addr as domainid. This helps
	 * identify CRF domain to which ether packet belongs.
	 */
	dev->dev_addr[ETH_ALEN - 1] = nlm_this_domain->id;
}


static void nlm_vnet_multicast_list(struct net_device *dev)
{

    if (dev->flags & IFF_PROMISC) {
	VNET_DBG("%s: vnet0 interface entering promisc mode\n", __FUNCTION__);
	nlm_vnet_promisc_map_set_clear_bit(
				nlm_vnet_promisc_dom_map_addr,
				nlm_this_domain->id, 1);
    } else {
	VNET_DBG("%s: vnet0 interface exiting promisc mode\n", __FUNCTION__);
	nlm_vnet_promisc_map_set_clear_bit(
				nlm_vnet_promisc_dom_map_addr,
				nlm_this_domain->id, 0);
    }
}


/*
 * The init function (sometimes called probe).
 * It is invoked by register_netdev()
 */
static void nlm_vnet_init(struct net_device *dev)
{
	struct nlm_vnet_priv *priv;

	ether_setup(dev);

	dev->open            = nlm_vnet_open;
	dev->stop            = nlm_vnet_release;

	/*
	 * MAC addr change is not allowed as last byte is hardcoded
	 * to represent domainid within CRF
	 */
	dev->set_mac_address = NULL; 

	dev->set_config      = nlm_vnet_config;
	dev->hard_start_xmit = nlm_vnet_tx;
	dev->do_ioctl        = nlm_vnet_ioctl;
	dev->get_stats       = nlm_vnet_stats;
	dev->change_mtu      = nlm_vnet_change_mtu;  
	dev->tx_timeout      = nlm_vnet_tx_timeout;
	dev->set_multicast_list = nlm_vnet_multicast_list;
	dev->watchdog_timeo = timeout;

	/*
	 * Then, initialize the priv field. This encloses the statistics
	 * and a few private fields.
	 */
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof(struct nlm_vnet_priv));
	spin_lock_init(&priv->lock);
	nlm_vnet_dom_map_addr = nlm_vnet_dom_map_get_addr();
	nlm_vnet_promisc_dom_map_addr = nlm_vnet_promisc_map_get_addr();
}


/*
 * Finally, the module stuff
 */
static void nlm_vnet_cleanup(void)
{
	if (vnet_dev) {
	    unregister_netdev(vnet_dev);
	    free_netdev(vnet_dev);
	}

	return;
}


static int nlm_vnet_init_module(void)
{
	int result, ret = -ENOMEM;

	/* 
	 * This driver is meant for communicating between different domains in CRF.
	 * So, if CRF is not up, simply return.
	 */
	if (!rmik_en) {
	    return -ENODEV;
	}

	/* Allocate the devices */
	vnet_dev = alloc_netdev(sizeof(struct nlm_vnet_priv), "vnet%d", nlm_vnet_init);

	if (vnet_dev == NULL)
		goto out;

	ret = -ENODEV;


	if ((result = register_netdev(vnet_dev))) {
	    printk("nlm_vnet: error %i registering device \"%s\" \n",
		    result, vnet_dev->name);
	} else {
	    ret = 0;
	}

	/* Assign mac address */
	nlm_vnet_set_mac_address(vnet_dev);

	/* Register virtual net handler for receiving events from other CRF domain */
	nlm_vnet_pkt_event_handler = nlm_vnet_pkt_get_event;

   out:
	if (ret) 
		nlm_vnet_cleanup();
	return ret;
}


module_init(nlm_vnet_init_module);
module_exit(nlm_vnet_cleanup);
