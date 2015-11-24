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
#include <linux/interrupt.h>
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
#include <asm/netlogic/sim.h>
#include <linux/bootmem.h>
#include <asm/bootinfo.h>
#include <asm/netlogic/nlm_pcix_gen_dev.h>
//#include <linux/moduleparam.h>
#define Message(a,b...)  //printk("\nFun [%s]\t"a"\n",__FUNCTION__,##b);
#define ErrorMsg(a,b...) printk("\nFun [%s]\t"a"\n",__FUNCTION__,##b);


#ifdef XLR_MAILBOX_IS_SUPPORTED
static rwlock_t nlm_mailbox_rw_lock = RW_LOCK_UNLOCKED;
static spinlock_t nlm_mailbox_spin_lock = SPIN_LOCK_UNLOCKED;
static int no_of_active_handler;

struct nlm_mailbox_irq{
  int valid;
  int disabled;
  mailbox_handler func;
  void *data;
}mailbox_desc[NLM_MAX_IRQS_SUPPORTED];
#endif

static long nlm_common_shared_mem_base;
int nlm_pcix_early_setup_dev(void);
long nlm_common_get_shared_mem_base_dev(void)
{
	return nlm_common_shared_mem_base;
}
void nlm_common_interrupt_host(void)
{
#ifdef XLR_MSI_IS_SUPPORTED
  nlm_reg_t *pcix_ctrl_mmio = 0;
  pcix_ctrl_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);
  /* Trigger an MSI to the host */
  pcix_ctrl_mmio[PCIX_INTRPT_CONTROL_REG] = 0x2;	
#endif
}

#ifdef XLR_MAILBOX_IS_SUPPORTED
static inline void nlm_nlm_common_mask_mailbox(void)
{
  nlm_reg_t *pcix_ctrl_mmio = 0;
  pcix_ctrl_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);
  /*Mask the mailbox interrupts */
  pcix_ctrl_mmio[PCIX_INTRPT_CONTROL_REG] = 0xc0;
}


static void nlm_nlm_common_unmask_mailbox(void)
{
  nlm_reg_t *pcix_ctrl_mmio = 0;
 /* Setup the mailbox and MSI interrupts
  * Setup the shared memory regions with the host.
 */
  pcix_ctrl_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);
  //pcix_ctrl_mmio[PCIX_NETLOGIC_CONTROL_REG] = 0xf2;
  pcix_ctrl_mmio[PCIX_NETLOGIC_CONTROL_REG] = (pcix_ctrl_mmio[PCIX_NETLOGIC_CONTROL_REG] & 0xffffff00) | 0xf2;  
  pcix_ctrl_mmio[PCIX_INTRPT_CONTROL_REG] = 0x0;
}

static int nlm_nlm_common_generic_mailbox_handler(int irq, void *data, struct pt_regs *regs)
{
  int i;
  unsigned int status;
  nlm_reg_t *pcix_ctrl_mmio = 0;

  pcix_ctrl_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);

  Message("Got some Mailbox intr");
/* First ack the interrupt */
  status = pcix_ctrl_mmio[PCIX_INTRPT_STATUS_REG];
  if(status == 0) /* Not our interrupt */
    return IRQ_NONE;

  pcix_ctrl_mmio[PCIX_INTRPT_STATUS_REG] = status;

  read_lock(&nlm_mailbox_rw_lock);

  Message("Scheduling mailbox ISRs");
  for(i=0; i<NLM_MAX_IRQS_SUPPORTED; i++)
    if(mailbox_desc[i].valid && !mailbox_desc[i].disabled)
      mailbox_desc[i].func(mailbox_desc[i].data,regs);
    
  read_unlock(&nlm_mailbox_rw_lock);
  return IRQ_HANDLED;
}

int nlm_common_enable_mailbox_intr(int *index)
{
  mailbox_desc[*index].disabled = 0;
  spin_lock(&nlm_mailbox_spin_lock);

  no_of_active_handler++;
  nlm_nlm_common_unmask_mailbox();

  spin_unlock(&nlm_mailbox_spin_lock);
  return 0;
}

int nlm_common_disable_mailbox_intr(int *index)
{
  mailbox_desc[*index].disabled = 1;

  spin_lock(&nlm_mailbox_spin_lock);
  no_of_active_handler--;
  if(no_of_active_handler == 0)
    nlm_nlm_common_mask_mailbox();

  spin_unlock(&nlm_mailbox_spin_lock);
  return 0;
}

int nlm_common_request_mailbox_handler(mailbox_handler func,void *data, int *index)
{
  int i;
  u32 flags;

  Message("Request for mailbox intr reg");
  write_lock_irqsave(&nlm_mailbox_rw_lock, flags);
  for(i=0; i<NLM_MAX_IRQS_SUPPORTED; i++)
    if(!mailbox_desc[i].valid)
      break;
  if(i == NLM_MAX_IRQS_SUPPORTED)
    return -EIO;
		  
  mailbox_desc[i].valid = 1;
  mailbox_desc[i].func = func;
  mailbox_desc[i].data = data;
  *index = i;
  Message("Request for mailbox intr reg SUCCESSFUL index %d", i);

  write_unlock_irqrestore(&nlm_mailbox_rw_lock, flags);

  nlm_common_enable_mailbox_intr(index);
  return 0;
}

int nlm_common_free_mailbox_handler(int *index)
{
  u32 flags;

  write_lock_irqsave(&nlm_mailbox_rw_lock, flags);
  mailbox_desc[*index].valid = 0;
  write_unlock_irqrestore(&nlm_mailbox_rw_lock, flags);

  nlm_common_disable_mailbox_intr(index);
  return 0;
}
#endif

static int nlm_nlm_common_device_generic_init(void)
{
#ifdef XLR_MAILBOX_IS_SUPPORTED
  int err;
#endif

  nlm_pcix_early_setup_dev();
  if(xlr_get_pci_mode() == XLR_PCI_HOST_MODE){
	  nlm_common_shared_mem_base = 0;
	  return -ENODEV;
  }
#ifdef XLR_MAILBOX_IS_SUPPORTED
  Message("Registring Generic Intr Handler.");	
  if((err = request_irq(PIC_PCIX_IRQ,nlm_nlm_common_generic_mailbox_handler,
		SA_INTERRUPT,"nlm_nlm_common_generic_mailbox_handler",NULL)))
  {
	  ErrorMsg("Cannot register handler for PCIX irq Error %d", err);
	  return err;
  }
#endif
  printk(KERN_INFO "Phoenix PCIX Shared membase is %lx\n", nlm_common_shared_mem_base);
  return 0;
}

void nlm_nlm_common_device_generic_cleanup(void)
{
  free_irq(PIC_PCIX_IRQ,NULL);	
  return;
}
int xlr_get_pci_mode()
{
  nlm_reg_t *pcix_ctrl_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);
  uint32_t mode;
  if(is_xls()){
      return XLR_PCI_HOST_MODE;
  }
  mode = pcix_ctrl_mmio[PCIX_HOST_MODE_CTRL_STATUS_REG];
  if(mode & 0x2){
	  return XLR_PCI_HOST_MODE;
  }
  return XLR_PCI_DEV_MODE;
}

int nlm_pcix_early_setup_dev(void)
{
  uint32_t mapper ;
  unsigned long phy_addr;
  static int done = 0;
  nlm_reg_t *pcix_ctrl_mmio = netlogic_io_mmio(NETLOGIC_IO_PCIX_OFFSET);

  if(xlr_get_pci_mode() == XLR_PCI_HOST_MODE){
	  nlm_common_shared_mem_base = 0;
	  return -ENODEV;
  }

  if(done)
    return 0;
  done = 1;	

  pcix_ctrl_mmio[PCIX_DEVMODE_TBL_BAR0_REG] = (0x8000000 >> 8);

  mapper = pcix_ctrl_mmio[PCIX_DEVMODE_TBL_BAR0_REG];
  phy_addr = (mapper << 8);
  nlm_common_shared_mem_base = (long)(int)((mapper << 8) | 0x80000000);
  
  return 0;
}

arch_initcall(nlm_pcix_early_setup_dev);

module_init(nlm_nlm_common_device_generic_init); 
module_exit(nlm_nlm_common_device_generic_cleanup);

EXPORT_SYMBOL(nlm_common_get_shared_mem_base_dev);
EXPORT_SYMBOL(nlm_common_interrupt_host);
EXPORT_SYMBOL(xlr_get_pci_mode);
#ifdef XLR_MAILBOX_IS_SUPPORTED
EXPORT_SYMBOL(nlm_common_free_mailbox_handler);
EXPORT_SYMBOL(nlm_common_request_mailbox_handler);
EXPORT_SYMBOL(nlm_common_disable_mailbox_intr);
EXPORT_SYMBOL(nlm_common_enable_mailbox_intr);
#endif
