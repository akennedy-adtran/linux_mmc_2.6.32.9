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


#ifndef CONFIG_NLM_COMMON
#ifndef MODULE
#define MODULE
#endif
#endif
#ifndef __KERNEL__
#define __KERNEL__
#endif


#include <linux/sched.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/init.h>

#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <asm/io.h>
#include <linux/completion.h>
#include <linux/crc32.h>

#ifdef CONFIG_NLM_COMMON
#include <asm/netlogic/nlm_pcix_gen_host.h>
#include <asm/netlogic/xlr_pcix_boot.h>
#else
#include "nlm_pcix_gen_host.h"
#include "xlr_pcix_boot.h"
#endif

#define Message(a,b...) //printk("\nFun [%s]\t"a"\n",__FUNCTION__,##b);
#define ErrorMsg(a,b...) printk("\nFun [%s]\t"a"\n",__FUNCTION__,##b);

#define NLM_IOCTL_DRIVER "NLM_PCIX_BOOT_DRIVER"
#define NLM_IMAGE_BUFF_LEN 1024
static struct net_device_ops nlm_boot_net_ops;

extern unsigned long nlm_common_get_shared_mem_base_host(void);
static struct net_device *ndev;

extern void nlm_common_pci_writeb(unsigned char data, void *addr);
extern void nlm_common_pci_writel(unsigned int  data,unsigned int *addr);
struct priv
{
				struct net_device *dev;
				struct net_device_stats stats;
				int port;
				u32 nlm_common_tx_producer;
				u32 nlm_common_pending_tx;
				u32 nlm_common_rx_consumer;
};

static int kimage_nlm_common_open(struct net_device *);
static int kimage_nlm_common_close(struct net_device *);

static int nlm_ioctl_nlm_common_probe(void);
static void nlm_nlm_common_remove(void);


static int nlm_nlm_common_pcix_init(void);
static void nlm_nlm_common_pcix_uninit(void);

static unsigned volatile int *nlm_nlm_common_shared_mem_base;


static int kimage_nlm_common_xmit(struct sk_buff *skb, struct net_device *dev)
{
				Message("in kimage_nlm_common_xmit !! BLANK BLANK !! \n");
				dev_kfree_skb(skb);
				return 0;
}



static int kimage_nlm_common_close(struct net_device *dev)
{
				Message("in close !!! BLANK BLANK !!! \n");
				return 0;

}

static int kimage_nlm_common_open(struct net_device *dev)
{
				Message("in open !!! BLANK BLANK !!! \n");
				return 0;
}


static int kimage_nlm_common_ioctl(struct net_device *dev,struct ifreq *ifr, int cmd)
{
				u8 *ptr;
				unsigned long result;
				static unsigned char *kimage_loc ;
				static int nlm_common_image_len;
				int buff_len;
				int argc;
				int argv_len;
				u8 *arg_buf, *dst;
				int i;

				switch (cmd){

								case SIOCDEVPRIVATE+0x03:
												kimage_loc = (unsigned char *)nlm_nlm_common_shared_mem_base +
																PCIX_BOOT_FILE_START; //kernel image location;
												nlm_common_image_len = 0;
												return 0; 

								case SIOCDEVPRIVATE+0x04:
												// send argc + len + args

												result = __copy_from_user((void *)&argc,
																				(void *)ifr->ifr_data,4);
												if(result > 0){
																ErrorMsg("invalid address frm user space");			
																return -1;
												}	

												result = __copy_from_user((void *)&argv_len,
																				(void *)(ifr->ifr_data+4),4);
												if(result > 0){
																ErrorMsg("invalid address frm user space");
																return -1;
												}							

												nlm_common_pci_writel(argc,
														(uint32_t *)((u8 *)nlm_nlm_common_shared_mem_base + 
														 			PCIX_BOOT_ARG_CNT_OFF));

												nlm_common_pci_writel(argv_len,
														(uint32_t *)((u8 *)nlm_nlm_common_shared_mem_base + 
														 			PCIX_BOOT_ARGS_LEN_OFF));

												arg_buf = kmalloc(argv_len, GFP_KERNEL);
												if(arg_buf == NULL)
													return -ENOMEM;

												__copy_from_user((void *)arg_buf, 
															(void *)(ifr->ifr_data+8), argv_len);

												dst = ((u8 *)nlm_nlm_common_shared_mem_base + 
																								PCIX_BOOT_ARGS_OFF);

												for(i=0; i < argv_len; i++) {
													nlm_common_pci_writeb(arg_buf[i], dst);
													dst++;
												}

												return 0;

								case SIOCDEVPRIVATE+0x02:

												ptr = (u8 *)kmalloc(NLM_IMAGE_BUFF_LEN, GFP_KERNEL);
												if (ptr == NULL){
																ErrorMsg(KERN_ERR "Unable to allocate memory !!!\n");
																return -ENOMEM;
												}

												result = __copy_from_user(ptr, ifr->ifr_data, 
																				NLM_IMAGE_BUFF_LEN);
												if (result > 0)
																return -EIO;
												buff_len = *((int *)ptr);
												Message("Got %d bytes Chunk \n", buff_len);

												for(i=0; i < buff_len; i++)
													nlm_common_pci_writeb(ptr[i+4], kimage_loc+i);

												kimage_loc += buff_len; 
												nlm_common_image_len += buff_len;

												kfree(ptr);
												if (buff_len < (NLM_IMAGE_BUFF_LEN-4) ||
																							 buff_len == 0){
																Message("File Download \
																				completed Total len %d\n", 
																				nlm_common_image_len);
																nlm_common_pci_writel(nlm_common_image_len, 
																		((uint32_t *)nlm_nlm_common_shared_mem_base + 1));
																nlm_common_pci_writel(0xa5a5a5a5,
																		(uint32_t *) nlm_nlm_common_shared_mem_base);

												}
												return 0;

								default:
												return -EINVAL;

				}
				return -EINVAL;
}



static void nlm_nlm_common_remove(void)
{
				nlm_nlm_common_pcix_uninit();
				return;
}

static void setup_net_ops(struct net_device *dev)
{
	nlm_boot_net_ops.ndo_open = kimage_nlm_common_open;
	nlm_boot_net_ops.ndo_stop = kimage_nlm_common_close;
	nlm_boot_net_ops.ndo_start_xmit = kimage_nlm_common_xmit;
	nlm_boot_net_ops.ndo_do_ioctl = kimage_nlm_common_ioctl;
	dev->netdev_ops = &nlm_boot_net_ops;
}

static int nlm_ioctl_nlm_common_probe(void)
{
				struct priv *priv = NULL;
				int i;
				int ret=0;
				Message("\n%s Called\n",__FUNCTION__);

				if(nlm_nlm_common_pcix_init()){
								ErrorMsg("pcix_init failed");
								return -EIO;
				}

				ndev = alloc_etherdev(sizeof(struct priv));
				if(!ndev){
								ret = -ENOMEM;
								goto out;
				}

				priv = netdev_priv(ndev);
				priv->dev = ndev;

				setup_net_ops(ndev);

				strcpy(ndev->name, "nlm_common_boot0");

				for(i=0; i<6; i++)
								ndev->dev_addr[i] = i;
				printk("\"nlm_common_boot0\" Boot Over PCI - interface registered\n");
				register_netdev(ndev);
				return ret;
out:
				ErrorMsg("Returnin Error %d",ret);
				return ret;  
}

/*  ioctl driver starts */
#define NLM_VENDOR_ID 0x0182e
#define NLM_DEVICE_ID 0x0


int __init nlm_virt_driver_init_module(void)
{
#ifdef CONFIG_NLM_COMMON
  if(nlm_get_pci_mode() == XLR_PCI_DEV_MODE){
    Message("Xlr Is configured in Dev Mode - unloading xlr_pcix_boot");
    return -ENODEV;
  }
#endif
  if(nlm_common_get_shared_mem_base_host() == 0){
    printk("\nLooks like device is not connected.\n");
    return -ENODEV; 
  }
  Message("\n%s called\n",__FUNCTION__);
	return nlm_ioctl_nlm_common_probe();
}

void __exit nlm_virt_driver_cleanup_module(void)
{

				nlm_nlm_common_remove();
				return;
}


static int nlm_nlm_common_pcix_init(void)
{
				int err;

				nlm_nlm_common_shared_mem_base = (uint32_t *)nlm_common_get_shared_mem_base_host();
				Message("\n%s Called\n",__FUNCTION__);
				if(nlm_nlm_common_shared_mem_base == NULL)
				{
								ErrorMsg("Shared MEM ioremap failed");
								err = -ENODEV;
								return err;
				}
				return 0;
}

/* This should be called only after successful return from nlm_nlm_common_pcix_init */
static void nlm_nlm_common_pcix_uninit(void)
{
				unregister_netdev(ndev);
				free_netdev(ndev);

}


module_init(nlm_virt_driver_init_module);
module_exit(nlm_virt_driver_cleanup_module);
MODULE_LICENSE("GPL");
