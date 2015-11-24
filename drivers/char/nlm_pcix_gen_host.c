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


#include <linux/sched.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/timer.h>
#include <linux/sysctl.h>
#ifdef CONFIG_NLM_COMMON
#include <asm/netlogic/sim.h>
#include <asm/netlogic/nlm_pcix_gen_host.h>
#else
#include "nlm_pcix_gen_host.h"
#endif

#define Message(a,b...) //printk("\n[%s]\t"a"\n",__FUNCTION__,##b)
#define ErrorMsg(a,b...) printk("\n[%s]\t"a"\n",__FUNCTION__,##b)
#define NLM_VENDOR_ID 0x182e
#define NLM_DEVICE_ID 0x0000

#define NLM_DRIVER "nlm_pcix_gen_drv"
#define NLM_MAX_IRQS_SUPPORTED 16

#ifdef XLR_MAILBOX_IS_SUPPORTED
static unsigned volatile int *nlm_nlm_common_mailbox_addr=NULL;
#endif
static unsigned volatile int *nlm_nlm_common_shared_mem_base_host=NULL;


struct pci_dev *nlm_pdev=NULL;

#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
static rwlock_t nlm_msi_rw_lock = RW_LOCK_UNLOCKED;
static spinlock_t nlm_msi_spin_lock = SPIN_LOCK_UNLOCKED;
static int no_of_active_handler=0;
static int msi_irq=0;
struct nlm_msi_irq{
  int valid;
  int disabled;
  msi_handler func;
  void *data;  
}msi_desc[NLM_MAX_IRQS_SUPPORTED] = {{0}};
#endif

void nlm_nlm_common_interrupt_device(void);

void nlm_common_pci_writel(unsigned int  data,unsigned int *addr)
{
#ifdef CONFIG_NLM_COMMON
  writel(cpu_to_le32(data),addr);
#else
  writel(cpu_to_be32(data),addr);
#endif
}
void nlm_common_pci_writeb(unsigned char data, void *addr)
{
  writeb(data,addr);
}

unsigned int nlm_common_pci_readl(unsigned int  *base)
{
#ifdef CONFIG_NLM_COMMON
  return le32_to_cpu(readl(base));
#else
  return be32_to_cpu(readl(base));
#endif
}
 

u8 nlm_common_pci_readb(u8 *base)
{
  return readb(base);
}
#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
static void nlm_common_unmask_msi(void)
{
#if 0
  unsigned short  word;
  Message("\nPhnxUnmaskMsi Called.\n");
  pci_read_config_word(nlm_pdev, 0x56, &word);
  word = word | 0x1;
  pci_write_config_word(nlm_pdev, 0x56, word);
  Message("--Returns..");
#endif
}

static void nlm_common_mask_msi(void)
{
#if 0
  unsigned short  word;
  Message("\nMask Msi Called.\n");
  pci_read_config_word(nlm_pdev, 0x56, &word);
  word = word & ~(0x1);
  pci_write_config_word(nlm_pdev, 0x56, word);
#endif
}

static int nlm_nlm_common_generic_msi_handler(int irq, void *data, struct pt_regs *regs)
{
  int i;
  Message("Got MSI intr");
  read_lock(&nlm_msi_rw_lock);
  for(i=0; i<NLM_MAX_IRQS_SUPPORTED; i++){
    if(msi_desc[i].valid && !msi_desc[i].disabled)
      msi_desc[i].func(msi_desc[i].data,regs);
  }
  read_unlock(&nlm_msi_rw_lock);
  return IRQ_HANDLED;
}

int nlm_common_enable_msi(int *index)
{
  Message("\nEnable Msi Called...With Index %d\n",*index);
	if(*index < 0 || *index > NLM_MAX_IRQS_SUPPORTED)
		return -EINVAL;

  msi_desc[*index].disabled = 0;

  spin_lock(&nlm_msi_spin_lock);

  no_of_active_handler++;
  nlm_common_unmask_msi();
  Message("\nNoActiveHandler %d\n",no_of_active_handler);
  spin_unlock(&nlm_msi_spin_lock);
  return 0;
  
}

int nlm_common_disable_msi(int *index)
{
  if(*index < 0 || *index > NLM_MAX_IRQS_SUPPORTED)
    	return -EINVAL;

  msi_desc[*index].disabled = 1;

  spin_lock(&nlm_msi_spin_lock);

  no_of_active_handler--;
  if(no_of_active_handler == 0)
    nlm_common_mask_msi();

  spin_unlock(&nlm_msi_spin_lock);
  return 0;
}

int nlm_common_request_msi_handler(msi_handler func,void *data, int *index)
{
  int i;
  unsigned long flags=0;

  Message("\nRequest for MSI handler\n");

  write_lock_irqsave(&nlm_msi_rw_lock, flags);
 
  for(i=0; i<NLM_MAX_IRQS_SUPPORTED; i++)
    if(!msi_desc[i].valid)
      break;
  
  if(i == NLM_MAX_IRQS_SUPPORTED) {
    write_unlock_irqrestore(&nlm_msi_rw_lock, flags);
    return -EIO;
	}
  Message("\nGot The Index %d Max %d\n",i,NLM_MAX_IRQS_SUPPORTED);
  msi_desc[i].valid = 1;
  msi_desc[i].disabled = 0;
  msi_desc[i].func = func;
  msi_desc[i].data = data;
  *index = i;

  Message("\nRequest for MSI handler DONE at index %d\n", *index);

  write_unlock_irqrestore(&nlm_msi_rw_lock, flags);
  nlm_common_enable_msi(index);
  return 0;  

}

void nlm_common_free_msi_handler(int *index)
{
  unsigned long flags=0;

  write_lock_irqsave(&nlm_msi_rw_lock, flags);
  msi_desc[*index].valid = 0;  
  write_unlock_irqrestore(&nlm_msi_rw_lock, flags);
  
  nlm_common_disable_msi(index);
  Message("\nIndex %d Freed\n",*index);
  return;
}
#endif

static struct pci_device_id nlm_id_table[] = {
  {NLM_VENDOR_ID, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
  {0,}
};

#ifdef CONFIG_SYSCTL
static int nlm_gen_mailbox=0;
static struct ctl_table_header *nlm_pcix_sysctl_header;
int nlm_ctl_handler(ctl_table *ctl, int write,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;
	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);

	if (write && *(int *)(ctl->data))
		*(int *)(ctl->data) = 1;
	nlm_nlm_common_interrupt_device();
	return ret;
}

static ctl_table nlm_tbl[] = {
	{
		.ctl_name       = 28,
		.procname       = "nlm_pcix",
		.mode           = 0555,
		.data           = &nlm_gen_mailbox,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = &nlm_ctl_handler,
	},
	{ .ctl_name = 0 }

};
static ctl_table nlm_pcix_sysctl_tbl[] = {
	{
		.ctl_name       = CTL_NET,
		.procname       = "net",
		.mode           = 0555,
		.child          = nlm_tbl,
	},
	{ .ctl_name = 0 }

};
#endif


static int nlm_nlm_common_generic_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{

  int err;
  unsigned long base;
#ifdef CONFIG_NLM_COMMON
  static int x=0;
  uint32_t tmp;

  if(!x){
    /*Setting MaxOutSplitTrans to zero*/
    pci_read_config_dword(pdev,0x40,&tmp); 
    tmp = tmp & ~(0x7U<<20);
    pci_write_config_dword(pdev,0x40,tmp);
    pci_read_config_dword(pdev,0x40,&tmp); 
    x=1;
    return -1;
  }
#endif
  nlm_pdev = pdev;
#ifndef CONFIG_NLM_COMMON
  if((err = pci_enable_device(pdev)))
  {
    ErrorMsg("Cannot enable PCI device, aborting.");
    return err;
  }
#endif
  if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM))
  {
    ErrorMsg("Cannot find proper PCI device " 
	     	    "base address BAR0, aborting.\n");
    err = -ENODEV;
    pci_disable_device(pdev);
    return err;
  }
  err = pci_request_region(pdev, 0, NLM_DRIVER);
  if (err)
  {
    ErrorMsg("Cannot obtain PCI resources, aborting.");
    err = -ENODEV;
    pci_disable_device(pdev);
    return err;
  }
  pci_set_master(pdev);
#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
  if ((err = pci_find_capability(pdev, PCI_CAP_ID_MSI)))
  {
    Message("Device is MSI capable..Enabling MSI");
    err = pci_enable_msi(pdev);
    msi_irq = pdev->irq;
    if(err == 0) {
      Message("MSI Enabled");
	}
    else{
      ErrorMsg("MSI Enable failed");
      return err;
    }
  }
  else
  {
    ErrorMsg("Device is NOT MSI capable");
    err = -ENODEV;
    pci_disable_device(pdev);
    return err;
  }
#endif
  base = pci_resource_start(pdev, 0);
  nlm_nlm_common_shared_mem_base_host = (unsigned volatile int *)
			ioremap_nocache(base,pci_resource_len(pdev, 0));
  printk("Device Memory Available @ %#x \n",
			(uint32_t)(unsigned long)nlm_nlm_common_shared_mem_base_host);
  if(nlm_nlm_common_shared_mem_base_host == NULL)
  {
    err = -ENODEV;
#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
    pci_disable_msi(pdev);
#endif
#ifndef CONFIG_NLM_COMMON
    pci_disable_device(pdev);
#endif
    return err;
  }

#ifdef XLR_MAILBOX_IS_SUPPORTED
  /* Use BAR2 as the mailbox address */
  base = pci_resource_start(pdev, 2);
  nlm_nlm_common_mailbox_addr = (unsigned int *)ioremap(base,pci_resource_len(pdev, 2));

  if(nlm_nlm_common_mailbox_addr == NULL || base == 0)
  {
    ErrorMsg("MailBox Is Not Supported");
    err = -ENODEV;
    iounmap((void *)nlm_nlm_common_shared_mem_base_host);
		nlm_nlm_common_mailbox_addr	= nlm_nlm_common_shared_mem_base_host = 0;

#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
    pci_disable_msi(pdev);
#endif
#ifndef CONFIG_NLM_COMMON
    pci_disable_device(pdev);
#endif
    return err;
  }
#endif 

#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
  if((err = request_irq(msi_irq,nlm_nlm_common_generic_msi_handler, SA_INTERRUPT,												"nlm_nlm_common_generic_msi_handler", (void *)NULL)))
	{
		ErrorMsg("Cant Register interrupt handler irq %d",msi_irq);
		iounmap((void *)nlm_nlm_common_shared_mem_base_host);
#ifdef XLR_MAILBOX_IS_SUPPORTED
		iounmap((void *)nlm_nlm_common_mailbox_addr);
#endif
		pci_disable_msi(pdev);
		pci_disable_device(pdev);
		return err ;
	}
//pci_set_mwi(pdev); 
#endif

#ifdef CONFIG_SYSCTL
  nlm_pcix_sysctl_header = register_sysctl_table(nlm_pcix_sysctl_tbl);
  if(nlm_pcix_sysctl_header == NULL) {
	  printk(KERN_WARNING "Could not register to sysctl\n");
  }
  else{
	  printk("nlm_pcix: registered with sysctl\n");
  }
#endif

  return 0;
}

void nlm_nlm_common_interrupt_device(void)
{
#ifdef XLR_MAILBOX_IS_SUPPORTED
  writel(0x1234abcd, nlm_nlm_common_mailbox_addr);
#endif
}



unsigned volatile long nlm_common_get_shared_mem_base_host(void)
{
  return (unsigned volatile long)nlm_nlm_common_shared_mem_base_host;
}

static void nlm_nlm_common_generic_remove(struct pci_dev *pdev)
{
#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
  nlm_common_unmask_msi();	
  free_irq(pdev->irq, NULL);
  pci_disable_msi(pdev);
#endif
  iounmap((void *)nlm_nlm_common_shared_mem_base_host);
#ifdef XLR_MAILBOX_IS_SUPPORTED
  iounmap((void *)nlm_nlm_common_mailbox_addr);
#endif
  pci_release_regions(pdev);
  pci_disable_device(pdev);
  pci_set_drvdata(pdev, NULL);

#ifdef CONFIG_SYSCTL
  if(nlm_pcix_sysctl_header)
  	unregister_sysctl_table(nlm_pcix_sysctl_header);
#endif
  return;     
}

static struct pci_driver nlm_pci_driver = {
  .name = NLM_DRIVER,
  .id_table = nlm_id_table,
  .probe  = nlm_nlm_common_generic_probe,
  .remove = nlm_nlm_common_generic_remove,
  #ifdef POWER_MANAGEMENT
    .suspend = nlm_suspend,
    .resume  = nlm_resume
  #endif
};
#ifdef CONFIG_NLM_COMMON
#include <asm/netlogic/iomap.h>
int nlm_get_pci_mode()
{
  nlm_reg_t *pcix_ctrl_mmio;
  uint32_t mode;

  if (is_xls()) {
	  return XLR_PCI_HOST_MODE;
  }

  pcix_ctrl_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);

  mode = pcix_ctrl_mmio[PCIX_HOST_MODE_CTRL_STATUS_REG];
  if(mode & 0x2){
	  return XLR_PCI_HOST_MODE;
  }
  return XLR_PCI_DEV_MODE;
}
#endif

int __init nlm_pcix_gen_init(void)
{
#ifdef CONFIG_NLM_COMMON
  if(nlm_get_pci_mode() == XLR_PCI_DEV_MODE)
    return -EIO;
  /* This driver is currently support on XLR hosts in case of RMI boards */
  if(is_xls())
      return -EIO;
#endif
  return pci_register_driver(&nlm_pci_driver);
}

void __exit nlm_pcix_gen_uninit(void)
{
  pci_unregister_driver(&nlm_pci_driver);
}
EXPORT_SYMBOL(nlm_common_pci_readl);
EXPORT_SYMBOL(nlm_common_pci_readb);
EXPORT_SYMBOL(nlm_common_pci_writel);
EXPORT_SYMBOL(nlm_common_pci_writeb);
EXPORT_SYMBOL(nlm_common_get_shared_mem_base_host);
#if !defined(CONFIG_NLM_COMMON) && defined(XLR_MSI_IS_SUPPORTED)
EXPORT_SYMBOL(nlm_common_request_msi_handler);
EXPORT_SYMBOL(nlm_common_free_msi_handler);
EXPORT_SYMBOL(nlm_common_enable_msi);
EXPORT_SYMBOL(nlm_common_disable_msi);
#endif
EXPORT_SYMBOL(nlm_nlm_common_interrupt_device);
EXPORT_SYMBOL(nlm_pdev);

module_init(nlm_pcix_gen_init);
module_exit(nlm_pcix_gen_uninit);
#ifdef CONFIG_NLM_COMMON
//Xlr Can be either pci dev or pci host
EXPORT_SYMBOL(nlm_get_pci_mode);
#endif
#ifndef CONFIG_NLM_COMMON
MODULE_LICENSE("GPL");
#endif
