/*-
 * Copyright 2005-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
 * rionet - Ethernet driver over RapidIO messaging services
 *
 * Copyright 2005 MontaVista Software, Inc.
 * Matt Porter <mporter@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/rio_ids.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>

#define DRV_NAME        "rionet"
#define DRV_VERSION     "0.2"
#define DRV_AUTHOR      "Matt Porter <mporter@kernel.crashing.org>"
#define DRV_DESC        "Ethernet over RapidIO"

#define NLM_RIO_MAX_ROUTE_ENTRIES (1<<16)
MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

#define MYDBG(a, ...) //printk
#define Message(a,b...) //printk("\nFunction [%s], Line [%d] "a"\n",__FUNCTION__,__LINE__,##b)

#define RIONET_DEFAULT_MSGLEVEL \
			(NETIF_MSG_DRV          | \
			 NETIF_MSG_LINK         | \
			 NETIF_MSG_RX_ERR       | \
			 NETIF_MSG_TX_ERR)

#define RIONET_DOORBELL_JOIN	0x1000
#define RIONET_DOORBELL_LEAVE	0x1001

#define RIONET_MAILBOX		0

#define RIONET_TX_RING_SIZE	CONFIG_RIONET_TX_SIZE
#define RIONET_RX_RING_SIZE	CONFIG_RIONET_RX_SIZE

static LIST_HEAD(rionet_peers);

struct rionet_private {
	struct rio_mport *mport;
	struct sk_buff *rx_skb[RIONET_RX_RING_SIZE];
	struct sk_buff *tx_skb[RIONET_TX_RING_SIZE];
	struct net_device_stats stats;
	int rx_slot;
	int tx_slot;
	int tx_cnt;
	int ack_slot;
	spinlock_t lock;
	spinlock_t tx_lock;
	spinlock_t tx_cnt_lock;
	u32 msg_enable;
};

struct rionet_peer {
	struct list_head node;
	struct rio_dev *rdev;
	struct resource *res;
};

static int rionet_check = 0;
static int rionet_capable = 1;

/*
 * This is a fast lookup table for for translating TX
 * Ethernet packets into a destination RIO device. It
 * could be made into a hash table to save memory depending
 * on system trade-offs.
 */
static struct rio_dev *rionet_active[NLM_RIO_MAX_ROUTE_ENTRIES];
#if 0
#define is_rionet_capable(pef, src_ops, dst_ops)		\
			((pef & RIO_PEF_INB_MBOX) &&		\
			 (pef & RIO_PEF_INB_DOORBELL) &&	\
			 (src_ops & RIO_SRC_OPS_DOORBELL) &&	\
			 (dst_ops & RIO_DST_OPS_DOORBELL))
#endif
#define is_rionet_capable(pef, src_ops, dst_ops)		\
            ((src_ops & RIO_SRC_OPS_DOORBELL) && \
              (dst_ops & RIO_DST_OPS_DOORBELL) && \
               (src_ops & RIO_SRC_OPS_DATA_MSG) && \
               (dst_ops & RIO_DST_OPS_DATA_MSG))

#define dev_rionet_capable(dev) \
	is_rionet_capable(dev->pef, dev->src_ops, dev->dst_ops)

#define RIONET_MAC_MATCH(x)    (*(u32 *)x == be32_to_cpu(0x00010001))
#define RIONET_GET_DESTID(x)   cpu_to_be16((*(u16 *)(x + 4)))

#define DATAMSG_BUF_SIZE	4096
static __inline__ struct sk_buff *nlm_rio_alloc_skb(int size)
{
        int offset = 0;
        struct sk_buff *skb = __dev_alloc_skb(size + DATAMSG_BUF_SIZE, GFP_ATOMIC);

        if (!skb) {
                return NULL;
        }

        /* align the data to the next cache line */
        offset = ((unsigned long)skb->data + DATAMSG_BUF_SIZE) &
                ~(DATAMSG_BUF_SIZE - 1);
        skb_reserve(skb, (offset - (unsigned long)skb->data));
        return skb;
}

static struct net_device_stats *rionet_stats(struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);
	return &rnet->stats;
}

static int rionet_rx_clean(struct net_device *ndev)
{
	int i;
	int error = 0;
	struct rionet_private *rnet = netdev_priv(ndev);
	void *data;
    volatile unsigned long dummy;

	i = rnet->rx_slot;

	do {
		if (!rnet->rx_skb[i])
			continue;

		if (!(data = rio_get_inb_message(rnet->mport, RIONET_MAILBOX))){
			break;
        	}

        /*dummy read to make sure data is valid!*/
        dummy = *(volatile unsigned long *)data;

		rnet->rx_skb[i]->data = data;
		skb_put(rnet->rx_skb[i], RIO_MAX_MSG_SIZE);
		rnet->rx_skb[i]->dev = ndev;
		rnet->rx_skb[i]->protocol =
		    eth_type_trans(rnet->rx_skb[i], ndev);
		error = netif_rx(rnet->rx_skb[i]);

		if (error == NET_RX_DROP) {
			rnet->stats.rx_dropped++;
		}
		else if (error == NET_RX_SUCCESS) {
			rnet->stats.rx_packets++;
                        rnet->stats.rx_bytes += RIO_MAX_MSG_SIZE;
		}
		else {
      		  	if (netif_msg_rx_err(rnet))
				printk(KERN_WARNING "%s: bad rx packet\n",
				       DRV_NAME);
			rnet->stats.rx_errors++;
		} 

	} while ((i = (i + 1) % RIONET_RX_RING_SIZE) != rnet->rx_slot);

	return i;
}

static void rionet_rx_fill(struct net_device *ndev, int end)
{
	int i;
	struct rionet_private *rnet = netdev_priv(ndev);

	i = rnet->rx_slot;
	do {
		
		rnet->rx_skb[i] = nlm_rio_alloc_skb(RIO_MAX_MSG_SIZE);

		if (!rnet->rx_skb[i])
			break;
		rnet->rx_slot++;
		if(rio_add_inb_buffer(rnet->mport, RIONET_MAILBOX,
				   rnet->rx_skb[i]->data))
        	    break;
	} while ((i = (i + 1) % RIONET_RX_RING_SIZE) != end);

	rnet->rx_slot = i;
}

static int rionet_queue_tx_msg(struct sk_buff *skb, struct net_device *ndev,
			       struct rio_dev *rdev)
{
	struct rionet_private *rnet = netdev_priv(ndev);
    int len = skb->len;
    unsigned long flags;
    int ret = 0;

    if(len & 0x7)
        len = (len + 0x7) & ~(0x7);

	if(rio_add_outb_message(rnet->mport, rdev, 0, skb->data, len))
        return -EIO;

	rnet->tx_skb[rnet->tx_slot] = skb;

	rnet->stats.tx_packets++;
	rnet->stats.tx_bytes += skb->len;

    spin_lock_irqsave(&rnet->tx_cnt_lock, flags);
    rnet->tx_cnt++;
	if (rnet->tx_cnt == RIONET_TX_RING_SIZE){
        MYDBG("Max tx_cnt reached stopping queue");
		netif_stop_queue(ndev);
    }
    spin_unlock_irqrestore(&rnet->tx_cnt_lock, flags);

	++rnet->tx_slot;
	rnet->tx_slot &= (RIONET_TX_RING_SIZE - 1);

	if (netif_msg_tx_queued(rnet))
		printk(KERN_INFO "%s: queued skb %8.8x len %8.8x\n", DRV_NAME,
		       (u32) skb, skb->len);

	return ret;
}

static int rionet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	int i;
	struct rionet_private *rnet = netdev_priv(ndev);
	struct ethhdr *eth = (struct ethhdr *)skb->data;
	u16 destid;
	unsigned long flags;
    int ret = 0;

	local_irq_save(flags);
	if ((rnet->tx_cnt + 1) > RIONET_TX_RING_SIZE) {
        MYDBG("Stopping queue");
		netif_stop_queue(ndev);
		local_irq_restore(flags);
		printk(KERN_ERR "%s: BUG! Tx Ring full when queue awake!\n",
		       ndev->name);
		return NETDEV_TX_BUSY;
	}

	if (eth->h_dest[0] & 0x01) {
		for (i = 0; i < NLM_RIO_MAX_ROUTE_ENTRIES; i++){
			if (rionet_active[i]){
				if((ret = rionet_queue_tx_msg(skb, ndev,
						    rionet_active[i]))){
                    MYDBG("Xmit failed!!!\n");
                    break;
                }
            }
        }
	} else if (RIONET_MAC_MATCH(eth->h_dest)) {
		destid = RIONET_GET_DESTID(eth->h_dest);
		if (rionet_active[destid]){
			ret = rionet_queue_tx_msg(skb, ndev, rionet_active[destid]);
            if(ret)
                MYDBG("XMIT FAILED!!\n");
        }
	}else{
        ret = -EIO;
	}

    local_irq_restore(flags);
	return ret;
}

static void rionet_dbell_event(struct rio_mport *mport, void *dev_id, u16 sid, 
                                u16 tid, u16 info)
{
	struct net_device *ndev = dev_id;
	struct rionet_private *rnet = netdev_priv(ndev);
	struct rionet_peer *peer;

	//printk("%s info 0x%x devid 0x%p sid %d tid %d \n",__func__, info, dev_id, sid, tid);
	if (netif_msg_intr(rnet))
		printk(KERN_INFO "%s: doorbell sid %4.4x tid %4.4x info %4.4x",
		       DRV_NAME, sid, tid, info);
	if (info == RIONET_DOORBELL_JOIN) {
		if (!rionet_active[sid]) {
			list_for_each_entry(peer, &rionet_peers, node) {
				if (peer->rdev->destid == sid)
					rionet_active[sid] = peer->rdev;
			}
			rio_mport_send_doorbell(mport, sid,
						RIONET_DOORBELL_JOIN);
		}
	} else if (info == RIONET_DOORBELL_LEAVE) {
		rionet_active[sid] = NULL;
	} else {
		if (netif_msg_intr(rnet))
			printk(KERN_WARNING "%s: unhandled doorbell\n",
			       DRV_NAME);
	}
}

static void rionet_inb_msg_event(struct rio_mport *mport, void *dev_id, int mbox, int slot)
{
	int n;
	struct net_device *ndev = dev_id;
	struct rionet_private *rnet = (struct rionet_private *)netdev_priv(ndev);
	unsigned long flags;

	if (netif_msg_intr(rnet))
		printk(KERN_INFO "%s: inbound message event, mbox %d slot %d\n",
		       DRV_NAME, mbox, slot);

	spin_lock_irqsave(&rnet->lock, flags);
	if ((n = rionet_rx_clean(ndev)) != rnet->rx_slot)
		rionet_rx_fill(ndev, n);
	spin_unlock_irqrestore(&rnet->lock, flags);
}

static void rionet_outb_msg_event(struct rio_mport *mport, void *dev_id, int mbox, int slot)
{
	struct net_device *ndev = dev_id;
	struct rionet_private *rnet = netdev_priv(ndev);
	unsigned long flags;

	spin_lock_irqsave(&rnet->lock, flags);
    MYDBG("Rcvd TX OK!!! - slot %d, ack_slot = %d",slot,rnet->ack_slot);
	if (netif_msg_intr(rnet))
		printk(KERN_INFO
		       "%s: outbound message event, mbox %d slot %d\n",
		       DRV_NAME, mbox, slot);

	while (rnet->tx_cnt && (rnet->ack_slot != slot)) {
		/* dma unmap single */
		dev_kfree_skb_irq(rnet->tx_skb[rnet->ack_slot]);
		rnet->tx_skb[rnet->ack_slot] = NULL;
		++rnet->ack_slot;
		rnet->ack_slot &= (RIONET_TX_RING_SIZE - 1);
        spin_lock(&rnet->tx_cnt_lock);
		rnet->tx_cnt--;
        spin_unlock(&rnet->tx_cnt_lock);
	}

    spin_lock(&rnet->tx_cnt_lock);
	if (rnet->tx_cnt < RIONET_TX_RING_SIZE){
        MYDBG("Waking queue\n");
		netif_wake_queue(ndev);
    }
    spin_unlock(&rnet->tx_cnt_lock);

	spin_unlock_irqrestore(&rnet->lock, flags);
}

static int rionet_open(struct net_device *ndev)
{
	int i, rc = 0;
	struct rionet_peer *peer, *tmp;
	struct rionet_private *rnet = netdev_priv(ndev);
	unsigned long flags;

	if (netif_msg_ifup(rnet))
		printk(KERN_INFO "%s: open\n", DRV_NAME);

	/* Initialize inbound message ring */
	for (i = 0; i < RIONET_RX_RING_SIZE; i++)
		rnet->rx_skb[i] = NULL;
	rnet->rx_slot = 0;
	
	if ((rc = rio_request_inb_dbell(rnet->mport,
					(void *)ndev,
					RIONET_DOORBELL_JOIN,
					RIONET_DOORBELL_LEAVE,
					rionet_dbell_event)) < 0)
		goto out;

	if ((rc = rio_request_inb_mbox(rnet->mport,
				       (void *)ndev,
				       RIONET_MAILBOX,
				       RIONET_RX_RING_SIZE,
				       rionet_inb_msg_event)) < 0)
		goto out;

	if ((rc = rio_request_outb_mbox(rnet->mport,
					(void *)ndev,
					RIONET_MAILBOX,
					RIONET_TX_RING_SIZE,
					rionet_outb_msg_event)) < 0)
		goto out;

	spin_lock_irqsave(&rnet->lock, flags);
	rionet_rx_fill(ndev, 0);
	spin_unlock_irqrestore(&rnet->lock, flags);

	rnet->tx_slot = 0;
	rnet->tx_cnt = 0;
	rnet->ack_slot = 0;

	netif_carrier_on(ndev);
	netif_start_queue(ndev);

	list_for_each_entry_safe(peer, tmp, &rionet_peers, node) {
		if (!(peer->res = rio_request_outb_dbell(peer->rdev,
							 RIONET_DOORBELL_JOIN,
							 RIONET_DOORBELL_LEAVE)))
		{
			printk(KERN_ERR "%s: error requesting doorbells\n",
			       DRV_NAME);
			continue;
		}

		/*
		 * send a join message to peer.
		 */
			rio_send_doorbell(peer->rdev, RIONET_DOORBELL_JOIN);
	}
	return rc;
      out:
	return rc;
}

static int rionet_close(struct net_device *ndev)
{
	struct rionet_private *rnet = (struct rionet_private *)netdev_priv(ndev);
	struct rionet_peer *peer, *tmp;
	int i;

	if (netif_msg_ifup(rnet))
		printk(KERN_INFO "%s: close\n", DRV_NAME);

    MYDBG("Stopping queue");
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	list_for_each_entry_safe(peer, tmp, &rionet_peers, node) {
		if (rionet_active[peer->rdev->destid]) {
			rio_send_doorbell(peer->rdev, RIONET_DOORBELL_LEAVE);
		}
		rio_release_outb_dbell(peer->rdev, peer->res);
	}

	rio_release_inb_dbell(rnet->mport, RIONET_DOORBELL_JOIN,
			      RIONET_DOORBELL_LEAVE);
	rio_release_inb_mbox(rnet->mport, RIONET_MAILBOX);
	rio_release_outb_mbox(rnet->mport, RIONET_MAILBOX);

	for (i = 0; i < RIONET_RX_RING_SIZE; i++)
		if (rnet->rx_skb[i])
			kfree_skb(rnet->rx_skb[i]);
	return 0;
}

static void rionet_remove(struct rio_dev *rdev)
{
	struct net_device *ndev = NULL;
	struct rionet_peer *peer, *tmp;

	unregister_netdev(ndev);
	kfree(ndev);

	list_for_each_entry_safe(peer, tmp, &rionet_peers, node) {
		list_del(&peer->node);
		kfree(peer);
	}
}

static void rionet_get_drvinfo(struct net_device *ndev,
			       struct ethtool_drvinfo *info)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->fw_version, "n/a");
	strcpy(info->bus_info, rnet->mport->name);
}

static u32 rionet_get_msglevel(struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	return rnet->msg_enable;
}

static void rionet_set_msglevel(struct net_device *ndev, u32 value)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	rnet->msg_enable = value;
}

static const struct ethtool_ops rionet_ethtool_ops = {
	.get_drvinfo = rionet_get_drvinfo,
	.get_msglevel = rionet_get_msglevel,
	.set_msglevel = rionet_set_msglevel,
	.get_link = ethtool_op_get_link,
};

static const struct net_device_ops nlm_rionet_ops = {
	.ndo_open = rionet_open,
	.ndo_start_xmit = rionet_start_xmit,
	.ndo_stop = rionet_close,
	.ndo_get_stats = rionet_stats,
};

static int rionet_setup_netdev(struct rio_mport *mport)
{
	int rc = 0;
	struct net_device *ndev = NULL;
	struct rionet_private *rnet;
	u16 device_id;

	/* Allocate our net_device structure */
	ndev = alloc_etherdev(sizeof(struct rionet_private));
	if (ndev == NULL) {
		printk(KERN_INFO "%s: could not allocate ethernet device.\n",
		       DRV_NAME);
		rc = -ENOMEM;
		goto out;
	}

	/* Set up private area */
	rnet = netdev_priv(ndev);
	rnet->mport = mport;

	/* Set the default MAC address */
	device_id = rio_local_get_device_id(mport);
	ndev->dev_addr[0] = 0x00;
	ndev->dev_addr[1] = 0x01;
	ndev->dev_addr[2] = 0x00;
	ndev->dev_addr[3] = 0x01;
	ndev->dev_addr[4] = device_id >> 8;
	ndev->dev_addr[5] = device_id & 0xff;

	/* Fill in the driver function table */
	ndev->netdev_ops = &nlm_rionet_ops;
	ndev->ethtool_ops = &rionet_ethtool_ops;
	ndev->mtu = RIO_MAX_MSG_SIZE - 14;

	spin_lock_init(&rnet->lock);
	spin_lock_init(&rnet->tx_lock);
	spin_lock_init(&rnet->tx_cnt_lock);

	rnet->msg_enable = RIONET_DEFAULT_MSGLEVEL;

	rc = register_netdev(ndev);
	if (rc != 0)
		goto out;

	printk("%s: %s %s Version %s, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
	       ndev->name,
	       DRV_NAME,
	       DRV_DESC,
	       DRV_VERSION,
	       ndev->dev_addr[0], ndev->dev_addr[1], ndev->dev_addr[2],
	       ndev->dev_addr[3], ndev->dev_addr[4], ndev->dev_addr[5]);

      out:
	return rc;
}

/*
 * XXX Make multi-net safe
 */
static int rionet_probe(struct rio_dev *rdev, const struct rio_device_id *id)
{
	int rc = -ENODEV;
	u32 lpef, lsrc_ops, ldst_ops;
	struct rionet_peer *peer;

	/* If local device is not rionet capable, give up quickly */
	if (!rionet_capable)
		goto out;

	/*
	 * First time through, make sure local device is rionet
	 * capable, setup netdev,  and set flags so this is skipped
	 * on later probes
	 */
	if (!rionet_check) {
		rio_local_read_config_32(rdev->net->hport, RIO_PEF_CAR, &lpef);
		rio_local_read_config_32(rdev->net->hport, RIO_SRC_OPS_CAR,
					 &lsrc_ops);
		rio_local_read_config_32(rdev->net->hport, RIO_DST_OPS_CAR,
					 &ldst_ops);
		if (!is_rionet_capable(lpef, lsrc_ops, ldst_ops)) {
			printk(KERN_ERR
			       "%s: local device is not network capable srcops 0x%x dstops 0x%x\n",
			       DRV_NAME, lsrc_ops, ldst_ops);
			rionet_check = 1;
			rionet_capable = 0;
			goto out;
		}

		rc = rionet_setup_netdev(rdev->net->hport);
		rionet_check = 1;
	}

	/*
	 * If the remote device has mailbox/doorbell capabilities,
	 * add it to the peer list.
	 */
	if (dev_rionet_capable(rdev)) {
		if (!(peer = kmalloc(sizeof(struct rionet_peer), GFP_KERNEL))) {
			rc = -ENOMEM;
			goto out;
		}
		peer->rdev = rdev;
        rionet_active[rdev->destid] = peer->rdev;
		list_add_tail(&peer->node, &rionet_peers);
	}

      out:
	return rc;
}

static struct rio_device_id rionet_id_table[] = {
	{RIO_DEVICE(RIO_ANY_ID, RIO_ANY_ID)}
};

static struct rio_driver rionet_driver = {
	.name = "rionet",
	.id_table = rionet_id_table,
	.probe = rionet_probe,
	.remove = rionet_remove,
};

static int __init rionet_init(void)
{
	return rio_register_driver(&rionet_driver);
}

static void __exit rionet_exit(void)
{
	rio_unregister_driver(&rionet_driver);
}

module_init(rionet_init);
module_exit(rionet_exit);
