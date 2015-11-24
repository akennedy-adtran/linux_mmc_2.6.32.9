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


#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
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
#ifdef CONFIG_NLM_COMMON
#include <asm/netlogic/nlm_pcix_gen_host.h>
#include <asm/netlogic/devices.h>
#else
#include "nlm_pcix_gen_host.h"
#endif


#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "RMI-INDIA"
#define DRIVER_DESC "console over pci"
#define BUF_SIZE (1024*4)
#define NLM_USER_CMD_SIZE (1*1024)
#define NLM_USER_RESULT_SIZE (7*1024)

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

#define DELAY_TIME  2	/* 2 seconds per character */

#ifndef CONFIG_NLM_COMMON
#define XLR_CONSOLE_OVER_PCI_MAJOR 246
#endif
#define CONSOLE_OVER_PCI_MINORS	244	/* only have one minor */
#define UART_NR			1	/* only use one port */

#define CONSOLE_OVER_PCI	"pci_console"


#define Message(a,b...) //printk("Function [%s]"a"\n",__FUNCTION__,##b)
#define ErrorMsg(a,b...) printk("Function [%s]"a"\n",__FUNCTION__,##b)

static volatile u32 *nlm_common_user_cmd_producer;
static volatile u32 *nlm_common_user_cmd_consumer;
static volatile u32 *nlm_common_user_result_consumer;
static volatile u32 *nlm_common_user_result_producer;
static volatile u8 *nlm_common_user_cmd;
static volatile u8 *nlm_common_user_result;
static volatile u32 *nlm_common_dev_status;
static volatile u32 *nlm_common_host_status;
static int nlm_common_pcix_dev_up=0;

extern unsigned long nlm_common_get_shared_mem_base_host(void);
static struct timer_list *nlm_common_timer=NULL;
extern void nlm_common_pci_writel(unsigned int  data,unsigned int *addr);
extern void nlm_common_pci_writeb(unsigned char data, unsigned int *addr);

unsigned int nlm_nlm_common_readl(unsigned int *base)
{
  return (nlm_common_pci_readl(base));
}

static void get_user_data(struct uart_port *port)
{
  u32 next_consumer;
  u8 ch;
  struct tty_struct *tty;
  int flag=0;
  
  
  nlm_common_pcix_dev_up=1;
  Message("\nGot MSI - nlm_common_pcix_dev_up iz %d\n",nlm_common_pcix_dev_up);
  
  if (!port){
    ErrorMsg("port is nt there...");
    return;
  }

  tty = port->state->port.tty;
  if (!tty){
    ErrorMsg("tty is nt thr...");
    return;
  }

  /* Read The Data And Push It To TTY Buffer */

  next_consumer = nlm_nlm_common_readl((uint32_t *)nlm_common_user_result_consumer);
  Message("next_consumer [%d]",next_consumer);
  Message("nlm_common_user_result_producer [%d]",
		nlm_nlm_common_readl((uint32_t *)nlm_common_user_result_producer));
  tty->low_latency = 1;
  while(next_consumer != nlm_nlm_common_readl((uint32_t *)nlm_common_user_result_producer)){
    ch = nlm_common_pci_readb((u8 *)(nlm_common_user_result + next_consumer));
    tty_insert_flip_char(tty, ch, TTY_NORMAL);
    tty_flip_buffer_push(tty);  
    flag = 1;
    next_consumer = (next_consumer + 1) % (NLM_USER_RESULT_SIZE);

    Message("next_consumer [%d]",next_consumer);
    Message("nlm_common_user_result_producer [%d]",nlm_nlm_common_readl((uint32_t *)nlm_common_user_result_producer));
  }
  
  
  if(flag)
    nlm_common_pci_writel(next_consumer,(uint32_t *)nlm_common_user_result_consumer);
  
  return;
}
#ifdef CONFIG_NLM_COMMON
static void nlm_pcix_console_stop_tx(struct uart_port *port)
#else
static void nlm_pcix_console_stop_tx(struct uart_port *port, unsigned int tty_stop)
#endif
{
	Message("");
}

static void nlm_pcix_console_stop_rx(struct uart_port *port)
{
	Message("");
}

static void nlm_pcix_console_enable_ms(struct uart_port *port)
{
	Message("");
}

static int nlm_host_put_char(char ch)
{
  u32 next_producer=0;
 
  Message("-- Called"); 
  next_producer = nlm_nlm_common_readl((uint32_t *)nlm_common_user_cmd_producer);

  if(((next_producer + 1) % NLM_USER_CMD_SIZE) == nlm_nlm_common_readl((uint32_t *)nlm_common_user_cmd_consumer)){
      Message("in if cond\n");
      return 0;
  }

  nlm_common_pci_writeb(ch,(void *)(nlm_common_user_cmd + next_producer));

  next_producer = (next_producer + 1) % (NLM_USER_CMD_SIZE);

  nlm_common_pci_writel((next_producer),(uint32_t *)nlm_common_user_cmd_producer);
  
  Message("next_producer [%u] nlm_common_user_cmd_producer [%p]",
               next_producer,nlm_common_user_cmd_producer);

  Message("[%c]",ch);

  return 1;

}

static void nlm_pcix_console_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	int count;

	if(!nlm_common_pcix_dev_up)   // Xmit if Device Is Up.
		return;
  Message("\nInside TX CHARS\n");	
	if (port->x_char) {
		
		if(!nlm_host_put_char(port->x_char))
		   return;
		Message ("DATA [%c] - port->x_char", port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
#ifdef CONFIG_NLM_COMMON
		nlm_pcix_console_stop_tx(port);
#else
		nlm_pcix_console_stop_tx(port, 0);
#endif
		return;
	}

	count = port->fifosize >> 1;
	do{
		Message ("DATA [%c] - do while", xmit->buf[xmit->tail]);
		
		if(!nlm_host_put_char(xmit->buf[xmit->tail]))
		  return;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		
		if (uart_circ_empty(xmit))
			break;
		
	}while(--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS){
		Message("uart write wakeup");
		uart_write_wakeup(port);
	}

	if (uart_circ_empty(xmit))
#ifdef CONFIG_NLM_COMMON
		nlm_pcix_console_stop_tx(port);
#else
		nlm_pcix_console_stop_tx(port, 0);
#endif
	else
	   Message("xmit->tail [%d] != xmit->head = [%d]",xmit->tail,
			   xmit->head);
}

#ifdef CONFIG_NLM_COMMON
static void nlm_pcix_console_start_tx(struct uart_port *port)
#else
static void nlm_pcix_console_start_tx(struct uart_port *port, unsigned int tty_start)
#endif
{
	Message("");
}

static void nlm_pcix_console_timer (unsigned long data)
{
	struct uart_port *port;


	port = (struct uart_port *)data;
	if (!port){
		ErrorMsg("port is nt there...");
		return;
	}

	/* see if we have any data to transmit */
	nlm_pcix_console_tx_chars(port);
 
        /* see if nythin to rcv */
//	if(nlm_common_pci_readl((uint32_t *)nlm_common_dev_status) == 0xdeadbeef)
	get_user_data(port);
	/* resubmit the timer again */
	nlm_common_timer->expires = jiffies + DELAY_TIME;
	add_timer (nlm_common_timer);
}

static unsigned int nlm_pcix_console_tx_empty(struct uart_port *port)
{
	return 1;
}

static unsigned int nlm_pcix_console_get_mctrl(struct uart_port *port)
{
	return port->mctrl;
}

static void nlm_pcix_console_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
  port->mctrl = mctrl;
}

static void nlm_pcix_console_break_ctl(struct uart_port *port, int break_state)
{
}

static int nlm_pcix_console_startup(struct uart_port *port)
{
	/* this is the first time this port is opened */
	/* do any hardware initialization needed here */

	/* create our timer and submit it */

  Message("Open Called..");	
  if (!nlm_common_timer) {
	  nlm_common_timer = kmalloc (sizeof(*nlm_common_timer), GFP_KERNEL);
	  if (!nlm_common_timer)
		  return -ENOMEM;
	  Message("\nRequest Msi Handler Returned..\n");
  }
  init_timer(nlm_common_timer);
  nlm_common_timer->data = (unsigned long )port;
  nlm_common_timer->expires = jiffies + DELAY_TIME;
  nlm_common_timer->function = nlm_pcix_console_timer;


  Message("Opening Port");
  nlm_common_pci_writel(0xdeadbeef,(uint32_t *)nlm_common_host_status);

  add_timer (nlm_common_timer);
  return 0;
}

static void nlm_pcix_console_shutdown(struct uart_port *port)
{
  /* The port is being closed by the last user. */
  /* Do any hardware specific stuff here */

  /* shut down our timer */
  Message("Close Called");
  Message("\nGoin To Free MSI HANDLER Index %d\n",index);
  if(nlm_common_timer){
  	del_timer_sync(nlm_common_timer);
  	kfree(nlm_common_timer);
  }
  nlm_common_timer = NULL;
  nlm_common_pci_writel(0,(uint32_t *)nlm_common_host_status);
  Message("nlm_common_host_status 2 written");
}

static const char *nlm_pcix_console_type(struct uart_port *port)
{
  return "nlm_pcix_consoletty";
}

static void nlm_pcix_console_release_port(struct uart_port *port)
{

}

static int nlm_pcix_console_request_port(struct uart_port *port)
{
  return 0;
}

static void nlm_pcix_console_config_port(struct uart_port *port, int flags)
{
}

static int nlm_pcix_console_verify_port(struct uart_port *port, struct serial_struct *ser)
{
  return 0;
}
static void
nlm_pcix_console_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	return;
}
static struct uart_ops nlm_pcix_console_ops = {
  .tx_empty	= nlm_pcix_console_tx_empty,
  .set_mctrl	= nlm_pcix_console_set_mctrl,
  .get_mctrl	= nlm_pcix_console_get_mctrl,
  .stop_tx	= nlm_pcix_console_stop_tx,
  .start_tx	= nlm_pcix_console_start_tx,
  .stop_rx	= nlm_pcix_console_stop_rx,
  .enable_ms	= nlm_pcix_console_enable_ms,
  .break_ctl	= nlm_pcix_console_break_ctl,
  .startup	= nlm_pcix_console_startup,
  .shutdown	= nlm_pcix_console_shutdown,
  .set_termios  = nlm_pcix_console_set_termios,
  .type		= nlm_pcix_console_type,
  .release_port	= nlm_pcix_console_release_port,
  .request_port	= nlm_pcix_console_request_port,
  .config_port	= nlm_pcix_console_config_port,
  .verify_port	= nlm_pcix_console_verify_port,
};

static struct uart_port nlm_pcix_console_port = {
  .ops		= &nlm_pcix_console_ops,
  .type 	= PORT_8250,
  .fifosize	= 255,
};

static struct uart_driver nlm_pcix_console_reg = {
  .owner  = THIS_MODULE,
  .driver_name  = CONSOLE_OVER_PCI,
  .dev_name  = CONSOLE_OVER_PCI,
  .major  = XLR_CONSOLE_OVER_PCI_MAJOR,
  .minor  = CONSOLE_OVER_PCI_MINORS,
  .nr  = UART_NR,
};


static int __init nlm_pcix_console_init(void)
{
  int result;

#ifdef CONFIG_NLM_COMMON
  if(nlm_get_pci_mode() == XLR_PCI_DEV_MODE){
    return -EIO;
  }
#endif
  if(nlm_common_get_shared_mem_base_host() == 0){
    printk("\nLooks like device is not connected.\n");
    return -ENODEV; 
  }
  result = uart_register_driver(&nlm_pcix_console_reg);
  if (result){
    ErrorMsg("Cant Register Driver");
    return result;
  }

  result = uart_add_one_port(&nlm_pcix_console_reg, &nlm_pcix_console_port);
		
  if (result){
    ErrorMsg("Cant Add Port");	  
    uart_unregister_driver(&nlm_pcix_console_reg);
    return result;
  }
  Message("Returning From init_module [%d]",result);
  
  nlm_common_user_cmd = (u8 *)nlm_common_get_shared_mem_base_host() + 
                   NLM_CONSOLE_OVER_PCI_SHARED_MEM_BASE;
  nlm_common_user_result = nlm_common_user_cmd + NLM_USER_CMD_SIZE;
  nlm_common_user_cmd_producer = (u32 *)(nlm_common_user_result + NLM_USER_RESULT_SIZE);
  nlm_common_user_cmd_consumer = nlm_common_user_cmd_producer + 1;
  nlm_common_user_result_consumer= nlm_common_user_cmd_consumer + 1;
  nlm_common_user_result_producer= nlm_common_user_result_consumer + 1;

  nlm_common_dev_status  = nlm_common_user_result_producer + 1;
  nlm_common_host_status = nlm_common_dev_status + 1;

  Message("\nnlm_common_user_cmd @ %#x\n",(uint32_t)nlm_common_user_cmd);
  Message("nlm_common_user_result @ %#x\n",(uint32_t )nlm_common_user_result);
  Message("nlm_common_user_cmd_producer @ %#x\n",(uint32_t )nlm_common_user_cmd_producer);
  Message("nlm_common_user_cmd_consumer @ %#x\n",(uint32_t )nlm_common_user_cmd_consumer);
  Message("nlm_common_user_result_consumer @ %#x\n",
			(uint32_t )nlm_common_user_result_consumer);
  Message("nlm_common_user_result_producer @ %#x\n",
				(uint32_t )nlm_common_user_result_producer);
  Message("*nlm_common_user_cmd %#x\n",(uint32_t )*nlm_common_user_cmd);
  Message("nlm_common_dev_status %#x\n",(uint32_t)nlm_common_dev_status);
  Message("nlm_common_host_status %#x\n",(uint32_t)nlm_common_host_status);

  nlm_common_pci_writel(0,(uint32_t *)nlm_common_user_result_producer);
  nlm_common_pci_writel(0,(uint32_t *)nlm_common_user_result_consumer);
  nlm_common_pci_writel(0,(uint32_t *)nlm_common_user_cmd_producer);
  nlm_common_pci_writel(0,(uint32_t *)nlm_common_user_cmd_consumer);
  nlm_common_pci_writel(0,(uint32_t *)nlm_common_host_status);
  printk("xlr_console_over_pci: registerd successfully.\n");
  return 0;
}

static void __exit nlm_pcix_console_cleanup(void)
{
  uart_remove_one_port(&nlm_pcix_console_reg, &nlm_pcix_console_port);
  uart_unregister_driver(&nlm_pcix_console_reg);
}

module_init(nlm_pcix_console_init);
module_exit(nlm_pcix_console_cleanup);
