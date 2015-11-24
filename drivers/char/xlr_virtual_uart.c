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
#include <asm/netlogic/xlr_virt_uart.h>

#define RX_THRESHOLD 10
#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Netlogic"
#define DRIVER_DESC "Virtual uart serial console driver for Arizona3"

#define Message(a,b...) //printk("Function [%s]"a"\n",__FUNCTION__,##b)
#define ErrorMsg(a,b...) printk("Function [%s]"a"\n",__FUNCTION__,##b)

virt_uart_struct virt_uart_tx_rx[32];

static struct timer_list *timer[32];

extern int xlr_loader_support;

static struct uart_driver virtual_uart_console_reg = {
  	.owner  = THIS_MODULE,
	.driver_name  = VIRTUAL_UART_CONSOLE,
  	.dev_name  = "virt_uart",
  	.major  = VIRTUAL_UART_CONSOLE_MAJOR,
  	.minor  = VIRTUAL_UART_CONSOLE_MINOR,
  	.nr  = VIRTUAL_UART_NR,
};

static int virtual_uart_console_request_port(struct uart_port *port) 
{
 	return 0;
}

static void virtual_uart_console_config_port(struct uart_port *port, int flags)
{
}

static int virtual_uart_console_verify_port(struct uart_port *port, struct serial_struct *ser)
{
  	return 0;
}

static void virtual_uart_console_release_port(struct uart_port *port)
{

}

static const char *virtual_uart_console_type(struct uart_port *port)
{
  	return "nlm_virtual_uart_consoletty";
}

static void virtual_uart_console_set_termios(struct uart_port *port, 
			struct ktermios *termios, struct ktermios *old)
{
        return;
}

static unsigned int virtual_uart_console_tx_empty(struct uart_port *port)
{
        return 1;
}

static unsigned int virtual_uart_console_get_mctrl(struct uart_port *port)
{
        return port->mctrl;
}

static void virtual_uart_console_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
  	port->mctrl = mctrl;
}

static void virtual_uart_console_break_ctl(struct uart_port *port, int break_state)
{
}

static void virtual_uart_console_start_tx(struct uart_port *port)
{
        Message("");
}

static void virtual_uart_console_stop_tx(struct uart_port *port)
{
        Message("");
}

static void virtual_uart_console_stop_rx(struct uart_port *port)
{
        Message("");
}

static void virtual_uart_console_enable_ms(struct uart_port *port)
{
        Message("");
}

static int virtual_uart_console_startup(struct uart_port *port);
static void virtual_uart_console_shutdown(struct uart_port *port);

static struct uart_ops virtual_uart_console_ops = {
  	.tx_empty     = virtual_uart_console_tx_empty,
  	.set_mctrl    = virtual_uart_console_set_mctrl,
  	.get_mctrl    = virtual_uart_console_get_mctrl,
  	.stop_tx      = virtual_uart_console_stop_tx,
  	.start_tx     = virtual_uart_console_start_tx,
  	.stop_rx      = virtual_uart_console_stop_rx,
  	.enable_ms    = virtual_uart_console_enable_ms,
  	.break_ctl    = virtual_uart_console_break_ctl,
  	.startup      = virtual_uart_console_startup,
  	.shutdown     = virtual_uart_console_shutdown,
  	.set_termios  = virtual_uart_console_set_termios,
  	.type         = virtual_uart_console_type,
  	.release_port = virtual_uart_console_release_port,
  	.request_port = virtual_uart_console_request_port,
  	.config_port  = virtual_uart_console_config_port,
  	.verify_port  = virtual_uart_console_verify_port,
};


static struct uart_port virtual_uart_console_port [VIRTUAL_UART_NR] = {
  	[0 ... VIRTUAL_UART_NR-1] = {
  	.ops          = &virtual_uart_console_ops,
  	.type         = PORT_8250,
  	.fifosize     = 255,
  	}
};

static void virtual_uart_console_rx_chars(struct uart_port *port)
{
  	struct tty_struct *tty;
  	char ch;
	int rx_cnt = 0;
	if (*(virt_uart_tx_rx[port->line].rx_pro) == *(virt_uart_tx_rx[port->line].rx_con))
		return;
     
  	tty = port->state->port.tty;
  	if (!tty){
          	ErrorMsg("tty is nt thr...");
          	return;
  	}

  	tty->low_latency = 1;
  	do{     
	
		ch = *((virt_uart_tx_rx[port->line].rx_fifo) + (*(virt_uart_tx_rx[port->line].rx_con)));
		tty_insert_flip_char(tty, ch, TTY_NORMAL); 
    		tty_flip_buffer_push(tty);
		*(virt_uart_tx_rx[port->line].rx_con) = (*(virt_uart_tx_rx[port->line].rx_con) + 1) % (USER_RESULT_SIZE);

		if(rx_cnt++ > RX_THRESHOLD)
			break;
  	}while(*(virt_uart_tx_rx[port->line].rx_pro) != *(virt_uart_tx_rx[port->line].rx_con));

}

static int nlm_cmd_put_char(char ch, int thrd_id)
{

  	Message("-- Called");
	Message("\nXmitting [%d]\n",ch);
	
	if(((*(virt_uart_tx_rx[thrd_id].tx_pro) + 1) % USER_CMD_SIZE ) == (*(virt_uart_tx_rx[thrd_id].tx_con)))	
		return -1;

   	*((virt_uart_tx_rx[thrd_id].tx_fifo) + *(virt_uart_tx_rx[thrd_id].tx_pro)) = ch;

	*(virt_uart_tx_rx[thrd_id].tx_pro) = (*(virt_uart_tx_rx[thrd_id].tx_pro) + 1) % (USER_CMD_SIZE);
	return 0;
}


/* Take the data(command) from the user and write it in to the Tx buf buffer.*/
static void virtual_uart_console_tx_chars(struct uart_port *port)
{
        struct circ_buf *xmit = &port->state->xmit; 
        int count;

        if (port->x_char) {

               	if(nlm_cmd_put_char(port->x_char, port->line))
			return;
                port->icount.tx++;
                port->x_char = 0;
                return;
        }
        if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
                virtual_uart_console_stop_tx(port);
                return;
        }

        count = port->fifosize >> 1;
        do{
		if(nlm_cmd_put_char(xmit->buf[xmit->tail], port->line))
			break;
                xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
                port->icount.tx++;

                if (uart_circ_empty(xmit))
                        break;

        }while(--count > 0);


        if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS){
                uart_write_wakeup(port);
        }
}


static void virtual_uart_console_timer (unsigned long data)
{
        struct uart_port *port;

        port = (struct uart_port *)data;
        if (!port){
                ErrorMsg("port is nt there...");
                return;
        }
        if (!port->state){
                ErrorMsg("port->info is nt there... ");
                return;
        }

        /* see if we have any data for rx*/
        virtual_uart_console_rx_chars(port);
        /* see if we have any data to transmit */
        virtual_uart_console_tx_chars(port);
        /* resubmit the timer again */
        timer[port->line]->expires = jiffies + DELAY_TIME;
        add_timer (timer[port->line]);
}

static int virtual_uart_console_startup(struct uart_port *port)
{
  	/* this is the first time this port is opened */
  	/* do any hardware initialization needed here */

  	/* create our timer and submit it */

  	Message("Open Called..");
  	if (!timer[port->line]) {
          	timer[port->line] = kmalloc (sizeof(*timer[port->line]), GFP_KERNEL);
          	if (!timer[port->line])
                  	return -ENOMEM;
  	}
  	init_timer(timer[port->line]);
  	timer[port->line]->data = (unsigned long )port;
  	timer[port->line]->expires = jiffies + DELAY_TIME;
  	timer[port->line]->function = virtual_uart_console_timer;
  	Message("Opening Port");
	*(virt_uart_tx_rx[port->line].tx_pro) = 0;
	*(virt_uart_tx_rx[port->line].tx_con) = 0;
	*(virt_uart_tx_rx[port->line].rx_pro) = 0;
	*(virt_uart_tx_rx[port->line].rx_con) = 0;
  	add_timer(timer[port->line]);
	*(virt_uart_tx_rx[port->line].status) = VIRT_UART_OPENED;
  	return 0;
}


static int __init virtual_uart_init(void)
{
	int result, i, size;
  	unsigned int line;

  	result = uart_register_driver(&virtual_uart_console_reg);

  	if (result){
    		ErrorMsg("Cant Register Driver");
    		return result;
  	}
  
  	for (line=0; line < VIRTUAL_UART_NR; line++){
        	virtual_uart_console_port[line].line = line;
        	result = uart_add_one_port(&virtual_uart_console_reg, &virtual_uart_console_port[line]);

        	if (result){
                	for(; line > 0; line--) {
                        	virtual_uart_console_port[line].line = line;
                        	uart_remove_one_port(&virtual_uart_console_reg, &virtual_uart_console_port[line]);
                	}
			virtual_uart_console_port[line].line = line;
			uart_remove_one_port(&virtual_uart_console_reg, &virtual_uart_console_port[line]);
                	uart_unregister_driver(&virtual_uart_console_reg);
                	return result;
        	}
  	}

  
  	size = USER_CMD_SIZE + 4 + 4 + USER_RESULT_SIZE + 4 + 4 + 4;

  	for (i=0; i<VIRTUAL_UART_NR; i++){
        	virt_uart_tx_rx[i].tx_fifo = i*size + (unsigned char *) VIRT_UART_BUF_START;
        	virt_uart_tx_rx[i].tx_pro = (unsigned int *) (virt_uart_tx_rx[i].tx_fifo + USER_CMD_SIZE); 
        	virt_uart_tx_rx[i].tx_con = (unsigned int *) (virt_uart_tx_rx[i].tx_pro + 1);
        	virt_uart_tx_rx[i].rx_fifo = (unsigned char *) (virt_uart_tx_rx[i].tx_con + 1);
        	virt_uart_tx_rx[i].rx_pro = (unsigned int *) (virt_uart_tx_rx[i].rx_fifo + USER_RESULT_SIZE); 
        	virt_uart_tx_rx[i].rx_con = (unsigned int *) (virt_uart_tx_rx[i].rx_pro + 1);
		virt_uart_tx_rx[i].status = (unsigned int *) (virt_uart_tx_rx[i].rx_con + 1);
		*(virt_uart_tx_rx[i].tx_pro) = *(virt_uart_tx_rx[i].tx_con) = 
		*(virt_uart_tx_rx[i].rx_pro) = *(virt_uart_tx_rx[i].rx_con) = 0;
		*(virt_uart_tx_rx[i].status) = 0;
  	}

  	return 0;
}

static void virtual_uart_console_shutdown(struct uart_port *port)
{
  	/* The port is being closed by the last user. */
  	/* Do any hardware specific stuff here */

  	/* shut down our timer */
  	Message("Close Called");
	*(virt_uart_tx_rx[port->line].status) = 0;
  	del_timer_sync(timer[port->line]);
}


static void __exit virtual_uart_console_cleanup(void)
{
  	int line;

  	for (line=0; line < VIRTUAL_UART_NR; line++)  {
		virtual_uart_console_port[line].line = line;
        	uart_remove_one_port(&virtual_uart_console_reg, &virtual_uart_console_port[line]);
  	}
  	uart_unregister_driver(&virtual_uart_console_reg);
}

module_init(virtual_uart_init);
module_exit(virtual_uart_console_cleanup);



