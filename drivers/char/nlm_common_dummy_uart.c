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
#include <linux/termios.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/kdev_t.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/interrupt.h>

#define MMIO_START          (0xbf402000)
#define GETCHAR_ADDRESS     (MMIO_START)
#define PUTCHAR_ADDRESS     (MMIO_START + 4)
#define UART_STATUS_ADDRESS (MMIO_START + 8)


/**************************************************************
 *
 * Serial driver
 *
 ****************************************************************/
static struct tty_struct *serial_tty;
static int open_count = 0;

static irqreturn_t serial_int_handler(int irq, void *data, struct pt_regs *regs)
{
  unsigned char ch = 0;
  struct tty_struct *tty = serial_tty;

  //printk("[%s]: IN\n", __FUNCTION__);
  if (siminfo->uart_status & UART_RX_RDY) {
    while (siminfo->uart_status & UART_RX_RDY) {
      ch = siminfo->getchar;
      if (tty) {
	//printk("[%s]: inserting %c\n", __FUNCTION__, ch);
	tty_insert_flip_char(tty, ch, 0);
      }
    }
    if (tty)
      tty_flip_buffer_push(tty);
  }

  return IRQ_HANDLED;
}

static int serial_open(struct tty_struct *tty, struct file *filp)
{
  //printk("[%s]: IN tty %p\n", __FUNCTION__, tty);
  serial_tty = tty;
  open_count++;
  
  return 0;
}

static void serial_close(struct tty_struct *tty, struct file *filp)
{
  //printk("[%s]: IN count %d tty %p\n", __FUNCTION__, open_count, tty);
  if (open_count == 0)
    return;
  if (--open_count == 0)
    serial_tty = 0;
}

static void serial_put_char(struct tty_struct *tty, u_char ch)
{
  static int flag = 1;

  if (flag == 1) {

#ifdef CONFIG_NLMCOMMON_SMP_PREFIX
    siminfo->putchar = '0' + (__u8)(smp_processor_id() & 0xff);
    siminfo->putchar = ':';
#endif
    flag = 0;
  }

  siminfo->putchar = (char)ch;  
  
  if (ch == '\n')
    flag = 1;  
}

static int serial_write_room(struct tty_struct *tty)
{
  /* This doesn't quite apply to the simulator. However, it is
   * needed by the tty code. So, fake it
   */
  return 15;
}

static int serial_write(struct tty_struct *tty, int from_user, 
			const unsigned char *buf, int count)
{
  int i=0, total=0;
  unsigned char ch = 0;

  //printk("[%s]: count = %d\n", __FUNCTION__, count);
  if (from_user && verify_area(VERIFY_READ, buf, count))
    return -EINVAL;
 
  if (count == 0)
    return 0;

  for (i = 0, total=0; i < count; i++, total++) {
    if (from_user) 
      copy_from_user(&ch, &buf[i], 1);
    else 
      ch = buf[i];
    serial_put_char(tty, ch);
  }

  return total;
}

void serial_set_termios(struct tty_struct *tty, struct termios *old)
{
#if 0
  printk("[%s]: IN old_cflag=%x, new_cflag=%x\n", __FUNCTION__, 
	 (int)old->c_cflag, (int)tty->termios->c_cflag);
#endif
}

void serial_start(struct tty_struct *tty)
{
  printk("[%s]: IN\n", __FUNCTION__);
}

void serial_stop(struct tty_struct *tty)
{
  printk("[%s]: IN\n", __FUNCTION__);
}

void serial_hangup(struct tty_struct *tty)
{
  printk("[%s]: IN\n", __FUNCTION__);
}

static struct termios    *serial_termios[2];
static struct termios    *serial_termios_locked[2];

static struct tty_driver serial_driver = {
  magic:                       TTY_DRIVER_MAGIC,
  driver_name:                 "serial",
  name:                        "ttyS",
  major:                       TTY_MAJOR,
  minor_start:                 0,
  num:                         1,
  type:                        TTY_DRIVER_TYPE_SERIAL,
  subtype:                     SERIAL_TYPE_NORMAL,
  flags:                       TTY_DRIVER_REAL_RAW,
  refcount:                    1,
  termios:                     serial_termios,
  termios_locked:              serial_termios_locked,

  open:                        serial_open,
  close:                       serial_close,
  write:                       serial_write,
  write_room:                  serial_write_room,
  put_char:                    serial_put_char,
  set_termios:                 serial_set_termios,
  start:                       serial_start,
  stop:                        serial_stop,
  hangup:                      serial_hangup  
};

int nlm_common_duart_init(void)
{
  if (request_irq(NLM_IRQ_DUMMY_UART, serial_int_handler, SA_INTERRUPT, "uart0", 0)) {
    panic("Couldn't get uart0 interrupt line");
  }
  
  serial_driver.init_termios = tty_std_termios;
  
  if (tty_register_driver(&serial_driver)) {
    printk("Couldn't register xlr duart serial driver\n");
    return -1;
  }
  printk("Registered xlr dummy uart driver\n");
  
  return 0;
}

void nlm_common_duart_exit(void)
{
  if (tty_unregister_driver(&serial_driver)) {
    printk("Couldn't unregister xlr duart serial driver\n");
  }
  printk("unregistered xlr duart serial driver\n");
  
}

module_init(nlm_common_duart_init);
module_exit(nlm_common_duart_exit);


/**************************************************************
 *
 * Serial Console 
 *
 ****************************************************************/

static int console_read(struct console *cons, char *str, 
			unsigned int count)
{
  return 0;
}

static void console_write(struct console *cons, const char *str,
                              unsigned int count)
{
  int i=0;

  for(i=0; i<count; i++) {
    if (str[i] == 0)
      continue;
    siminfo->putchar = (char)str[i];
  }  
}

static struct tty_driver *console_device(struct console *c, int *index)
{
  *index = c->index;
  return &serial_driver;
}

static int console_setup(struct console *cons, char *str)
{
  siminfo->uart_status |= 0x80;

  return 0;
}

static struct console nlm_common_dummy_uart_console = {
  name:              "ttyS",
  read:              console_read,
  write:             console_write,
  device:            console_device,
  setup:             console_setup,
  flags:             CON_PRINTBUFFER,
  index:             -1,
};

int nlm_common_console_init(void)
{
  register_console(&nlm_common_dummy_uart_console);
  
  return 0;
}

void nlm_common_console_exit(void)
{
  /* do nothing */
}

console_initcall(nlm_common_console_init);
