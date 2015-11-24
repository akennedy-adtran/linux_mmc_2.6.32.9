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


#include <linux/console.h>
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
#include <asm/netlogic/nlm_pcix_gen_dev.h>
#include <asm/netlogic/devices.h>

#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Netlogic"
#define DRIVER_DESC "Virtual serial console driver for Arizona3"
#define BUF_SIZE (1024*4)
#define USER_CMD_SIZE (1*1024)
#define USER_RESULT_SIZE (7*1024)


/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

#define DELAY_TIME   2	     /* 2 seconds per character */
#define CONSOLE_OVER_PCI_MINORSS	244	/* only have one minor */
#define UART_NR			1	/* only use one port */

#define CONSOLE_OVER_PCI	"pci_co"

#define Message(a,b...) //printk("Function [%s]"a"\n",__FUNCTION__,##b)
#define ErrorMsg(a,b...) printk("Function [%s]"a"\n",__FUNCTION__,##b)
static u32 *nlm_common_user_cmd_producer;
static u32 *nlm_common_user_cmd_consumer;
static u32 *nlm_common_user_result_consumer;
static u32 *nlm_common_user_result_producer;
static u8 *nlm_common_user_cmd;
static u8 *nlm_common_user_result;
static volatile u32 *nlm_common_dev_status;
static volatile u32 *nlm_common_host_status;


static wait_queue_head_t user_result_buffer;

static struct timer_list *timer;

extern int nlm_pcix_early_setup_dev(void);
extern long nlm_common_get_shared_mem_base_dev(void);
static void nlm_pcix_console_stop_tx(struct uart_port *port)
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

static void nlm_dev_put_char(char ch)
{
 
  if(*nlm_common_host_status != 0xdeadbeef)
	return;

  Message("-- Called"); 

  if((*nlm_common_user_result_producer) + 1 == *nlm_common_user_result_consumer){
    return;
  }  
  *(nlm_common_user_result + *nlm_common_user_result_producer) = ch;
  
  *nlm_common_user_result_producer = (*nlm_common_user_result_producer + 1) % (USER_RESULT_SIZE);

   Message("nlm_common_user_result_producer [%d]",*nlm_common_user_result_producer);
}

static void nlm_pcix_console_rx_chars(struct uart_port *port)
{
  struct tty_struct *tty;
  char ch;
	
  if(*nlm_common_user_cmd_producer == *nlm_common_user_cmd_consumer)
    return;

  tty = port->state->port.tty;
  if (!tty){
	  ErrorMsg("tty is nt thr...");
	  return;
  }

  do{
    ch = *(nlm_common_user_cmd + (*nlm_common_user_cmd_consumer));
    Message("[%c]",ch);
    tty_insert_flip_char(tty, ch, TTY_NORMAL); 
    tty_flip_buffer_push(tty);
    *nlm_common_user_cmd_consumer = (*nlm_common_user_cmd_consumer + 1) % (USER_CMD_SIZE);
  }while(*nlm_common_user_cmd_producer != *nlm_common_user_cmd_consumer);
  
}

static void nlm_pcix_console_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	int count;
	Message("");
	if (port->x_char) {
		Message ("DATA [%c] - port->x_char", port->x_char);
		nlm_dev_put_char(port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		nlm_pcix_console_stop_tx(port);
		return;
	}

	count = port->fifosize >> 1;
	do{
		Message ("DATA [%c] - do while", xmit->buf[xmit->tail]);
		
		nlm_dev_put_char(xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		
		if (uart_circ_empty(xmit))
			break;
		
	}while(--count > 0);


	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS){
		Message("uart write wakeup");
		uart_write_wakeup(port);
	}

}

static void nlm_pcix_console_start_tx(struct uart_port *port)
{
	Message("");
}

static void nlm_pcix_console_timer (unsigned long data)
{
	struct uart_port *port;

	Message("");
	port = (struct uart_port *)data;
	if (!port){
		ErrorMsg("port is nt there...");
		return;
	}

	/* see if port is closed ??*/
        if(*nlm_common_host_status != 0xdeadbeef){
	  /*hangup device*/  	
	  goto out;
	}
	/* see if we have any data for rx*/
	nlm_pcix_console_rx_chars(port);
out:
	/* see if we have any data to transmit */
	nlm_pcix_console_tx_chars(port);
	/* resubmit the timer again */
	timer->expires = jiffies + DELAY_TIME;
	add_timer (timer);
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
  Message("nlm_common_user_result_consumer is [%d]",*nlm_common_user_result_consumer);
  Message("nlm_common_user_result_producer is [%d]",*nlm_common_user_result_producer);

  if (!timer) {
    timer = kmalloc (sizeof (*timer), GFP_KERNEL);
    if (!timer)
      return -ENOMEM;
  }
  init_timer(timer);
  timer->data = (unsigned long )port;
  timer->expires = jiffies + DELAY_TIME;
  timer->function = nlm_pcix_console_timer;
  add_timer (timer);
  *nlm_common_dev_status = 0xdeadbeef;
  Message("-- Returns");	
  return 0;
}

static void nlm_pcix_console_shutdown(struct uart_port *port)
{
  /* The port is being closed by the last user. */
  /* Do any hardware specific stuff here */

  /* shut down our timer */
  Message("Close Called");
  del_timer (timer);
  *nlm_common_dev_status = 0x0;
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

static struct console nlm_common_pcix_console; 

static struct uart_driver nlm_pcix_console_reg = {
  .owner  = THIS_MODULE,
  .driver_name  = CONSOLE_OVER_PCI,
  .dev_name  = CONSOLE_OVER_PCI,
  .major  = XLR_CONSOLE_OVER_PCI_MAJOR,
  .minor  = CONSOLE_OVER_PCI_MINORSS,
  .nr  = UART_NR,
  .cons = &nlm_common_pcix_console
};

static int __init nlm_pcix_console_init(void)
{
  int result;

  if(xlr_get_pci_mode() == XLR_PCI_HOST_MODE){
    Message("Xlr Is configured in Host Mode - unloading console_over_pci_dev\n");
    return -EIO;
  }
  if(nlm_common_get_shared_mem_base_dev() == 0){
	ErrorMsg("Get Shared Mem Base Iz Zero");
	return -ENODEV;
  }

  Message ("Tiny serial driver");

  result = uart_register_driver(&nlm_pcix_console_reg);
  if (result){
    ErrorMsg("Cant Register Driver");
    return result;
  }

  result = uart_add_one_port(&nlm_pcix_console_reg, &nlm_pcix_console_port);
		
  if (result){
    ErrorMsg("Cant Add Port");	  
    uart_unregister_driver(&nlm_pcix_console_reg);
  }
  Message("Returning From init_module [%d]",result);

  Message("\nnlm_common_user_cmd @ %#x\n",(u32)nlm_common_user_cmd);
  Message("nlm_common_user_result @ %#x\n",(u32)nlm_common_user_result);
  Message("User_Cmd_Producer [%#x]\n",(u32)nlm_common_user_cmd_producer);
  Message("User_Cmd_Consumer [%#x]\n",(u32)nlm_common_user_cmd_consumer);
  Message("User_Result_Consumer [%#x]\n",(u32)nlm_common_user_result_consumer);
  Message("User_Result_Producer [%#x]\n",(u32)nlm_common_user_result_producer);

  init_waitqueue_head(&user_result_buffer);
  printk("\nConsole Over Pci Driver - Registered\n"); 
  return result;
}

static void __exit nlm_pcix_console_cleanup(void)
{
  uart_remove_one_port(&nlm_pcix_console_reg, &nlm_pcix_console_port);
  uart_unregister_driver(&nlm_pcix_console_reg);
}

static void
nlm_common_pcix_console_write(struct console *co, const char *str, unsigned int count)
{
  int i;
  *nlm_common_dev_status = 0xdeadbeef;
  for(i=0;i<count;i++){
    nlm_dev_put_char(*(str+i));
    if(*(str+i) == '\n')
      nlm_dev_put_char('\r');	    
  }
}

static int __init nlm_common_pcix_console_setup(struct console *co,char *options)
{
  return 0;
}
static struct console nlm_common_pcix_console= {
	.name           = CONSOLE_OVER_PCI,
	.write          = nlm_common_pcix_console_write,
	.device         = uart_console_device,
	.setup          = nlm_common_pcix_console_setup,
	.flags          = CON_PRINTBUFFER,
	.index          = -1,
	.data           = &nlm_pcix_console_reg,
};



static int __init nlm_common_pcix_console_init(void)
{

  if(xlr_get_pci_mode() == XLR_PCI_HOST_MODE)
    return -EIO;

  nlm_pcix_early_setup_dev();

  if(nlm_common_get_shared_mem_base_dev() == 0){
	ErrorMsg("Either XLR Is in Host Mode or pci_shared_mem option is not specified");
	return -ENODEV;
  }
  nlm_common_user_cmd = (u8 *)nlm_common_get_shared_mem_base_dev() + 
                   NLM_CONSOLE_OVER_PCI_SHARED_MEM_BASE;

  nlm_common_user_result = nlm_common_user_cmd + USER_CMD_SIZE;
  nlm_common_user_cmd_producer = (u32 *)(nlm_common_user_result + USER_RESULT_SIZE);
  
  nlm_common_user_cmd_consumer = nlm_common_user_cmd_producer + 1;
  nlm_common_user_result_consumer= nlm_common_user_cmd_consumer + 1;
  nlm_common_user_result_producer= nlm_common_user_result_consumer + 1;
  nlm_common_dev_status = nlm_common_user_result_producer + 1;
  nlm_common_host_status = nlm_common_dev_status + 1;
  *nlm_common_user_cmd_consumer = *nlm_common_user_result_consumer = 
	  *nlm_common_user_cmd_producer = *nlm_common_user_result_producer =
	  *nlm_common_dev_status = 0;
  Message("\nnlm_common_user_cmd @ %#x\n",(uint32_t)nlm_common_user_cmd);
  Message("nlm_common_user_result @ %#x\n",(uint32_t)nlm_common_user_result);
  Message("nlm_common_user_cmd_producer @ %#x\n",(uint32_t)nlm_common_user_cmd_producer);
  Message("nlm_common_user_cmd_consumer @ %#x\n",(uint32_t)nlm_common_user_cmd_consumer);
  Message("nlm_common_user_result_consumer @ %#x\n",
				(uint32_t)nlm_common_user_result_consumer);
  Message("nlm_common_user_result_producer @ %#x\n",
			(uint32_t)nlm_common_user_result_producer);
  Message("*nlm_common_user_cmd %#x\n",(uint32_t)*nlm_common_user_cmd);
  Message("nlm_common_dev_status %#x\n",(uint32_t)nlm_common_dev_status);
  Message("nlm_common_host_status %#x\n",(uint32_t)nlm_common_host_status);

  register_console(&nlm_common_pcix_console);
  return 0;
}

console_initcall(nlm_common_pcix_console_init);


module_init(nlm_pcix_console_init);
module_exit(nlm_pcix_console_cleanup);

