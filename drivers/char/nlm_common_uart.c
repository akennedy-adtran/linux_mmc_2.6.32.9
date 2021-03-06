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
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <asm/netlogic/interrupt.h>
#include <asm/netlogic/nlm_common_uart.h>

/**************************************************************
 *
 * Serial Console 
 *
 ****************************************************************/

#define NUM_CHANNELS     2
#define NUM_CHANNEL_REGS 32
#define NUM_GLOBAL_REGS  5

#define CONFIG_CHAN_REG_SIZE (NUM_CHANNELS     *  \
                                    NUM_CHANNEL_REGS )

#define CONFIG_REG_SIZE      (NUM_CHANNELS     *  \
                              NUM_CHANNEL_REGS +  \
                              NUM_GLOBAL_REGS )

enum GDA_UART_REGS {
  THR = 0,
  RHR,
  IER,
  IIR,
  FCR,
  LCR,
  MCR,
  LSR,
  MSR,
  SCR,
  CSR,
  M_IER,
  M_ISR,
  M_CSR,
  M_TX_ADDR,
  M_RX_AD,
  M_TX_LEN,
  M_RX_LEN,
  GI_GR,
  GI_SR_0,
  GI_SR_1,
  GI_SR_2,
  GI_SR_3,
  DLL,
  DLM,
};

static volatile __u32 *mmio;

#define GDA_TX_FIFO_BUF_SIZE 128

struct fifo {
  __u8 buf[GDA_TX_FIFO_BUF_SIZE];
  int head;
  int tail;
  spinlock_t lock;
};

struct channel_info {
  struct tty_struct *tty;
  struct fifo        tx_fifo;
  unsigned int       open_count;
};

static struct channel_info gda_channels[NUM_CHANNELS];

static __inline__ void fifo_init(struct fifo *f)
{
  f->head = f->tail = 0;
  spin_lock_init(&f->lock);
}

static __inline__ int fifo_empty(struct fifo *f)
{
  int ret = 0;
  __u32 flags = 0;
  
  spin_lock_irqsave(&f->lock, flags);
  ret = (f->head == f->tail);
  spin_unlock_irqrestore(&f->lock, flags);

  return ret;
}

#define FIFO_NEXT(p) ( ((p) + 1) % GDA_TX_FIFO_BUF_SIZE)

static __inline__ int fifo_full(struct fifo *f)
{
  int ret = 0;
  __u32 flags = 0;
  
  spin_lock_irqsave(&f->lock, flags);
  ret = (FIFO_NEXT(f->tail) == f->head);
  spin_unlock_irqrestore(&f->lock, flags);

  return ret;  
}

static __inline__ int fifo_enqueue(struct fifo *f, __u8 data)
{
  int ret = 0;
  __u32 flags = 0;
  
  spin_lock_irqsave(&f->lock, flags);
  if (FIFO_NEXT(f->tail) == f->head)
    goto out;
  
  f->buf[f->tail] = data;
  f->tail = FIFO_NEXT(f->tail);
  ret = 1;

out:
  spin_unlock_irqrestore(&f->lock, flags);
  
  return ret;
}

static __inline__ int fifo_dequeue(struct fifo *f, __u8 *data)
{
  __u32 flags = 0;
  int ret = 0;
  
  spin_lock_irqsave(&f->lock, flags);
  if (f->head == f->tail)
    goto out;
  
  *data = f->buf[f->head];
  f->head = FIFO_NEXT(f->head);
  ret = 1;

 out:
  spin_unlock_irqrestore(&f->lock, flags);
  
  return ret;
}

static __inline__ int fifo_count(struct fifo *f)
{
  int ret = 0;
  __u32 flags = 0;

  spin_lock_irqsave(&f->lock, flags);  
  ret = (f->head <= f->tail ? (f->tail - f->head) : (GDA_TX_FIFO_BUF_SIZE - f->head + f->tail));
  spin_unlock_irqrestore(&f->lock, flags);    

  return ret;
}

static __inline__ int fifo_room(struct fifo *f)
{
  int ret = 0;
  __u32 flags = 0;
  __u32 count = 0;

  /* Note: fifo implementation "wastes" one space in the buffer */
  spin_lock_irqsave(&f->lock, flags);
  count = (f->head <= f->tail ? (f->tail - f->head) : (GDA_TX_FIFO_BUF_SIZE - f->head + f->tail));
  ret = (GDA_TX_FIFO_BUF_SIZE - count - 1);
  spin_unlock_irqrestore(&f->lock, flags);

  return ret;
}

static volatile __u32  gda_read_reg(int chan, int reg);
static void  gda_write_reg(int chan, int reg, __u32 data);

static void  gda_write_reg(int chan, int reg, __u32 data)
{
  if (reg == GI_GR || (reg >= GI_SR_0 && reg <= GI_SR_0+3)) {
    mmio[ CONFIG_CHAN_REG_SIZE + (reg - GI_GR) ] = data;
    return;
  }

  if (chan < 0 || chan >= NUM_CHANNELS || reg < 0 || reg >= 32)
    return;
  
  if (reg == DLL || reg == DLM) {
    gda_write_reg(chan, LCR, (gda_read_reg(chan, LCR) | (0x01<<7)) );

    if (reg == DLL)
      gda_write_reg(chan, THR, data);
    else
      gda_write_reg(chan, RHR, data);
    
    gda_write_reg(chan, LCR, (gda_read_reg(chan, LCR) & ~(0x01<<7)) );
    return;
  }
  
  mmio[ (chan << 5) + (reg) ] = data;
}

static volatile __u32 gda_read_reg(int chan, int reg)
{
  if (reg == GI_GR || (reg >= GI_SR_0 && reg <= GI_SR_0+3)) 
    return mmio[ CONFIG_CHAN_REG_SIZE + (reg - GI_GR) ];

  if (chan < 0 || chan >= NUM_CHANNELS || reg < 0 || reg >= 32)
    return ~0;
  
  if (reg == DLL || reg == DLM) {
    volatile __u32 data = 0;

    gda_write_reg(chan, LCR, (gda_read_reg(chan, LCR) | (0x01<<7)) );

    if (reg == DLL)
      data = gda_read_reg(chan, THR);
    else
      data = gda_read_reg(chan, RHR);

    gda_write_reg(chan, LCR, (gda_read_reg(chan, LCR) & ~(0x01<<7)) );

    return data;
  }
  
  return mmio[ (chan << 5) + (reg)];
}
/**************************************************************
 *
 * Serial driver
 *
 ****************************************************************/
static spinlock_t open_lock = SPIN_LOCK_UNLOCKED;

static __inline__ unsigned int get_channel(struct tty_struct *tty)
{
  /*return MINOR(tty->device) - tty->driver.minor_start;*/
  return tty->index;
}

static irqreturn_t serial_int_handler(int irq, void *data, struct pt_regs *regs)
{
  unsigned char ch             = 0;
  unsigned int chan            = 0;
  struct channel_info *channel = 0;
   __u32 lsr             = 0; 
   __u32 flags           = 0;
   __u32 int_grp         = 0;
   __u32 int_sel         = 0;
   __u32 iid             = 0;
   __u32 iir             = 0;
   int i = 0, j = 0;

   for (;;) { /* Repeat as long as the GI_GR has bits set */

     /* Determine the Channel */
     int_grp = gda_read_reg(-1, GI_GR);
     if (!int_grp) break;
     
     /* XXX: Should use count leading zeros or figure out someother smarter
      * way to calc the set bits instead of iterating thru all possible bits
      */     
     for (i=0;i<4;i++) {

       if (!(int_grp & (1<<i)))
	 continue;
   
       int_sel = gda_read_reg(-1, GI_SR_0 + i);
       if (!int_sel) break;

       for (j=0;j<8;j++) {

	 if (!(int_sel & (1<<j)))
	   continue;

	 chan     = (i<<8) + j;
	 channel  = gda_channels + chan;

	 iir = gda_read_reg(chan, IIR);
   
	 /* intr_pending? */
	 if (iir & 0x01)
	   continue;

	 iid = (iir >> 1) & 0x07;

	 if (iid == 3 || iid == 2 || iid == 1) {
	   /* Recv Line Status, Recv Data Avail, THR Empty */
     
	   for (;;) {
	     int k=0;
       
	     lsr = gda_read_reg(chan, LSR);
       
	     if (lsr == 0)
	       break;
       
	     if (lsr & 0x40) {
	       __u8 data = 0;
	 
	       for(k=0;k<8;k++) {

		 if (fifo_dequeue(&channel->tx_fifo, &data))
		   gda_write_reg(chan, THR, data);	 
		 else
		   break;

	       }
	       if (fifo_empty(&channel->tx_fifo)) {

		 gda_write_reg(chan, IER, (gda_read_reg(chan, IER) & ~0x02));
		 if (channel->tty->ldisc.write_wakeup)
		   channel->tty->ldisc.write_wakeup(channel->tty);
		 wake_up_interruptible(&(channel->tty->write_wait));

	       }
	     }
       
	     ch = 0;     
	     if (lsr & 0x81) {

	       /* Receive FIFO error */
	       if (lsr & 0x08)
		 flags |= TTY_FRAME;

	       if (lsr & 0x04)
		 flags |= TTY_PARITY;     

	       if (lsr & 0x01)
		 ch = (gda_read_reg(chan, RHR) & 0xff);

	       tty_insert_flip_char(channel->tty, ch, flags);
	     }
	     if (lsr & 0x02) 
	       tty_insert_flip_char(channel->tty, 0, TTY_OVERRUN);
	   }
	   tty_flip_buffer_push(channel->tty);
	   
	 }
	 else {
	   /* Char Time Out, Modem Status Change, Tx & Rx Buffer Status */
	   printk("[%s]: Unhandled value in IIR!\n", __FUNCTION__);
	 }
       }
     }
   } /* All channels */

   return IRQ_HANDLED;
}

static int serial_open(struct tty_struct *tty, struct file *filp)
{
  unsigned long flags          = 0;
  unsigned int chan            = get_channel(tty);
  struct channel_info *channel = gda_channels + chan;
  
  spin_lock_irqsave(&open_lock, flags);
  if (!channel->open_count) {
    tty->driver_data = channel;
    channel->tty     = tty;
  }
  channel->open_count++;
  gda_write_reg(chan, IER, 0x05);/* Enable all interrupts */  
  spin_unlock_irqrestore(&open_lock, flags);
  
  return 0;
}

static void serial_close(struct tty_struct *tty, struct file *filp)
{
  struct channel_info *channel = gda_channels + get_channel(tty);
  unsigned long flags = 0;

  if (!channel || !channel->open_count)
    return;

  spin_lock_irqsave(&open_lock, flags);
  channel->open_count--;
  if (!channel->open_count) {
    channel->tty     = 0;
    tty->driver_data = 0;
  }
  spin_unlock_irqrestore(&open_lock, flags);
}

static void serial_put_char(struct tty_struct *tty, u_char ch)
{
  struct channel_info *channel = (struct channel_info *)tty->driver_data;
  unsigned int chan            = get_channel(tty);

  fifo_enqueue(&channel->tx_fifo, ch);
  gda_write_reg(chan, IER, (gda_read_reg(chan, IER) | 0x02));
}

static int serial_write_room(struct tty_struct *tty)
{
  struct channel_info *channel = (struct channel_info *)tty->driver_data;
  
  return fifo_room(&channel->tx_fifo);
}

static int serial_write(struct tty_struct *tty, int from_user, 
			const unsigned char *buf, int count)
{
  int i=0, total=0;
  unsigned char ch             = 0;
  struct channel_info *channel = (struct channel_info *)tty->driver_data;
  unsigned int chan            = get_channel(tty);
  
  if (from_user && verify_area(VERIFY_READ, buf, count))
    return -EINVAL;
 
  if (count == 0)
    return 0;

#ifdef CONFIG_NLMCOMMON_SMP_PREFIX  
  {
    __u8 tmp_buf[8];
    __u8 *p = tmp_buf;
    sprintf(tmp_buf, "%1d:", smp_processor_id());
    tmp_buf[7] = 0;
    for(p=tmp_buf; *p; p++)
      if (!fifo_enqueue(&channel->tx_fifo, *p))
	break;
  }
#endif

  for (i = 0, total=0; i < count; i++, total++) {
    if (from_user) 
      copy_from_user(&ch, &buf[i], 1);
    else 
      ch = buf[i];
    if (!fifo_enqueue(&channel->tx_fifo, ch))
      break;
  }

  if (total)
    gda_write_reg(chan, IER, (gda_read_reg(chan, IER) | 0x02));

  return total;
}

void serial_set_termios(struct tty_struct *tty, struct termios *old)
{
  printk("[%s]: IN\n", __FUNCTION__);
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

static struct pci_device_id nlm_common_serial_id_table[] = {
  {NLM_PCI_VENDOR_ID, NLM_PCI_UART_DEV_ID, 0xaaaa, 0xaaaa, 0, 0},
  {0}
};

static int nlm_common_serial_pci_probe(struct pci_dev *dev, 
				    const struct pci_device_id *id)
{
  static unsigned long     ioaddr = 0;
  int i = 0;

  printk("[%s]: slot_name = [%s], irq=%d, base=%lx\n", 
	 __FUNCTION__, dev->slot_name, dev->irq, pci_resource_start(dev, 0));
  
  /* Initialize the device */
  if (request_irq(dev->irq, serial_int_handler, SA_INTERRUPT, "uart0", 0)) 
    panic("Couldn't get uart0 interrupt line");
  
  if (pci_request_regions(dev, "Phoenix Uart")) 
    panic("[%s]: Cannot reserve MMIO region\n", __FUNCTION__);    
  
  ioaddr = (unsigned long)ioremap(pci_resource_start(dev, 0),
				  pci_resource_len(dev, 0));
  if (!ioaddr) {
    panic("[%s]: Unable to ioremap MMIO region %lx @ %lx\n",
	  __FUNCTION__, pci_resource_len(dev, 0), pci_resource_start(dev, 0));
  }
  printk("[%s]: Registered MMIO region (%lx @ %lx): %lx\n",
	 __FUNCTION__, pci_resource_len(dev, 0), pci_resource_start(dev, 0),
	 ioaddr);
  
  mmio = (volatile __u32 *)ioaddr;
  
  for (i=0;i<NUM_CHANNELS;i++) 
    fifo_init(&(gda_channels[i].tx_fifo));

  return 0;
}

int nlm_common_uart_init(void)
{
  int count = 0;
  
  count = pci_register_driver(&nlm_common_serial_driver);
  if (!count) 
    printk("[%s]: No devices found? Should wait for a Hot Plug!\n", __FUNCTION__);
  /*printk("[%s]: count = %d\n", __FUNCTION__, count);*/
  
  serial_driver.init_termios = tty_std_termios;
  
  gda_write_reg(0, CSR, 0x0f);/* Enable the receiver and transmitter */

  /* TODO: Configure these registers later:
   * The performance of the simulator varies drastically depending on these
   * values
   */
  /*gda_write_reg(0, DLL, ((16) & 0x00ff));
    gda_write_reg(0, DLM, ((16) & 0xff00) >> 8);*/

  gda_write_reg(0, LCR, (gda_read_reg(0, LCR) | 0x03));
  gda_write_reg(0, FCR, 0); /* int trigger level is 0 */
#if 0
  {
    __u32 csr = gda_read_reg(0, CSR);
    __u32 ier = gda_read_reg(0, IER);
    
    /*printk("[%s]: configuring divisor = %08x\n", __FUNCTION__, (512 * smp_num_cpus));*/
    printk("[%s]: csr = %08x, ier = %08x, divisor = %08x\n", __FUNCTION__, csr, ier, 
	   ( (gda_read_reg(0, DLM) << 8)| gda_read_reg(0, DLL)) );
  }
#endif
  
  if (tty_register_driver(&serial_driver)) {
    printk("Couldn't register xlr uart serial driver\n");
    return -1;
  }
  printk("Registered xlr uart serial driver\n");
  
  return 0;
}

void nlm_common_uart_exit(void)
{
  if (tty_unregister_driver(&serial_driver)) 
    printk("Couldn't unregister xlr uart serial driver\n");
  
  pci_unregister_driver(&nlm_common_serial_driver);

  printk("unregistered xlr uart serial driver\n");  
}

module_init(nlm_common_uart_init);
module_exit(nlm_common_uart_exit);

/************************************************************************
 * Phoenix Serial Console 
 ************************************************************************/

static void console_write(struct console *cons, const char *str,
                              unsigned int count)
{
  nlm_reg_t *mmio = nlm_common_mmio_offset(NETLOGIC_IO_UART_0_OFFSET);
  int i=0;
  int next=0;

  for(;;) {

    for (;;) {
      nlm_reg_t lsr = netlogic_read_reg(mmio, UART_LSR);

      /* Tx Fifo empty */
      if (lsr & 0x20) break;
    }

    for (i=0; i<14 && next<count; i++, next++)
      netlogic_write_reg(mmio, UART_THR, (int)str[next]);

    /* More to process? */
    if (next < count) break;
  }  
}

static struct tty_driver *console_device(struct console *c, int *index)
{
  *index = c->index;
  return &serial_driver;
}

static int console_setup(struct console *cons, char *str)
{
  nlm_reg_t *mmio = nlm_common_mmio_offset(NETLOGIC_IO_UART_0_OFFSET);
  int lcr = netlogic_read_reg(mmio, UART_LCR);
  
  netlogic_write_reg(mmio, UART_LCR, (lcr | (1<<7)));
  netlogic_write_reg(mmio, UART_DLB_1, 0x01);
  netlogic_write_reg(mmio, UART_DLB_2, 0x00);
  netlogic_write_reg(mmio, UART_LCR, (lcr & ~(1<<7)));

  return 0;
}

static struct console nlm_common_console = {
  name:              "ttyS",
  write:             console_write,
  device:            console_device,
  setup:             console_setup,
  flags:             CON_PRINTBUFFER,
  index:             -1,
};

int nlm_common_console_init(void)
{
  register_console(&nlm_common_console);
  
  return 0;
}

void nlm_common_console_exit(void)
{
  /* do nothing */
}

console_initcall(nlm_common_console_init);

