/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*  Derived loosely from ide-pmac.c, so:
 *  
 *  Copyright (C) 1998 Paul Mackerras.
 *  Copyright (C) 1995-1998 Mark Lord
 */

/*
 * Boards with xlr processors so far have supported IDE devices via
 * the Generic Bus, PCI bus, and built-in PCMCIA interface.  In all
 * cases, byte-swapping must be avoided for these devices (whereas
 * other PCI devices, for example, will require swapping).  Any
 * xlr-targetted kernel including IDE support will include this
 * file.  Probing of a Generic Bus for an IDE device is controlled by
 * the definitions of "NETLOGIC_HAVE_IDE" and "IDE_PHYS", which are
 * provided by <asm/netlogic/nlm_common_ide.h> for ATX1 boards.
 */

#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/platform_device.h>
#include <asm/netlogic/nlm_common_ide.h>
#include <asm/netlogic/64bit.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/sim.h>

#ifdef CONFIG_MIPS
#include <asm/ide.h>
#else
#include <asm-generic/ide_iops.h>
#endif

/* #define DEBUG_PORT  */

#define GPIO_INTR_CLR_REG    0x1EF19180
#define PCMCIA_CONFIG_REG    0x1EF19140

/*
 * Our non-swapping I/O operations.  
 */
static inline void nlm_common_outb(u8 val, unsigned long port) 
{
	unsigned int flags=0;
#ifdef DEBUG_PORT
	printk(" %s port = %x %x \n", __FUNCTION__, (mips_io_port_base + port),val);
#endif

	flags = nlm_br_write_lock();
	*(volatile u8 *)(mips_io_port_base + (port)) = val;
	nlm_br_write_unlock(flags);
}

static inline void nlm_common_outw(u16 val, unsigned long port) 
{
	unsigned int flags=0;
#ifdef DEBUG_PORT
	printk("%s  port = %x  %x\n",__FUNCTION__,  (mips_io_port_base + port),val);
#endif
	flags = nlm_br_write_lock();
	*(volatile u16 *)(mips_io_port_base + (port)) = val;
	nlm_br_write_unlock(flags);
}

static inline void nlm_common_outl(u32 val, unsigned long port) 
{
	unsigned int flags=0;
#ifdef DEBUG_PORT
	printk("%s  port = %x %x\n",__FUNCTION__,  (mips_io_port_base + port),val);
#endif
	flags = nlm_br_write_lock();
	*(volatile u32 *)(mips_io_port_base + (port)) = val;
	nlm_br_write_unlock(flags);
}

static inline unsigned char nlm_common_inb(unsigned long port)
{
	unsigned int flags=0;
	unsigned char val ;

	flags = nlm_br_read_lock();
	val =  (*(volatile u8 *)(mips_io_port_base + (port)));
	nlm_br_read_unlock(flags);
#ifdef DEBUG_PORT
	printk("%s  port = %x %x \n",__FUNCTION__,  (mips_io_port_base + port),val );
#endif
	return val;
}

static inline unsigned short nlm_common_inw(unsigned long port)
{
	unsigned int flags=0;
	unsigned short val ;
	flags = nlm_br_read_lock();
	val = (*(volatile u16 *)(mips_io_port_base + (port)));
	nlm_br_read_unlock(flags);
#ifdef DEBUG_PORT
	printk("%s  port = %x %x \n",__FUNCTION__,  (mips_io_port_base + port),val );
#endif
	return val;
}

static inline unsigned int nlm_common_inl(unsigned long port)
{
	unsigned int flags=0;
	unsigned int val ;
	flags = nlm_br_read_lock();
	val =  (*(volatile u32 *)(mips_io_port_base + (port)));
	nlm_br_read_unlock(flags);
#ifdef DEBUG_PORT
	printk("%s  port = %x %x \n",__FUNCTION__,  (mips_io_port_base + port),val );
#endif
	return val;
}

static inline void nlm_common_outsb(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		nlm_common_outb(*(u8 *)addr, port);
		addr++;
	}
}

static inline void nlm_common_insb(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u8 *)addr = nlm_common_inb(port);
		addr++;
	}
}

static inline void nlm_common_outsw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		nlm_common_outw(*(u16 *)addr, port);
		addr += 2;
	}
}

static inline void nlm_common_insw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u16 *)addr = nlm_common_inw(port);
		addr += 2;
	}
}

static inline void nlm_common_outsl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		nlm_common_outl(*(u32 *)addr, port);
		addr += 4;
	}
}

static inline void nlm_common_insl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u32 *)addr = nlm_common_inl(port);
		addr += 4;
	}
}


/* Note:
   Following functions are taken from drivers/ide/ide-io-std.c
   unmodified, as required for PCMCIA.
   The original functions in drivers/ide/ide-io-std.c have been modified so 
   that PCI devices work properly
   */

/*
 *	Conventional PIO operations for ATA devices
 */

static u8 ide_inb(unsigned long port)
{
	return (u8) inb(port);
}

static void ide_outb(u8 val, unsigned long port)
{
	outb(val, port);
}

/*
 *	MMIO operations, typically used for SATA controllers
 */

static u8 ide_mm_inb(unsigned long port)
{
	return (u8) readb((void __iomem *) port);
}

static void ide_mm_outb(u8 value, unsigned long port)
{
	writeb(value, (void __iomem *) port);
}

static void nlm_common_ide_exec_command(ide_hwif_t *hwif, u8 cmd)
{
	if (hwif->host_flags & IDE_HFLAG_MMIO)
		writeb(cmd, (void __iomem *)hwif->io_ports.command_addr);
	else
		outb(cmd, hwif->io_ports.command_addr);
}

static u8 nlm_common_ide_read_status(ide_hwif_t *hwif)
{
	if (hwif->host_flags & IDE_HFLAG_MMIO)
		return readb((void __iomem *)hwif->io_ports.status_addr);
	else
		return inb(hwif->io_ports.status_addr);
}

static u8 nlm_common_ide_read_altstatus(ide_hwif_t *hwif)
{
	if (hwif->host_flags & IDE_HFLAG_MMIO)
		return readb((void __iomem *)hwif->io_ports.ctl_addr);
	else
		return inb(hwif->io_ports.ctl_addr);
}

static void nlm_common_ide_write_devctl(ide_hwif_t *hwif, u8 ctl)
{
	if (hwif->host_flags & IDE_HFLAG_MMIO)
		writeb(ctl, (void __iomem *)hwif->io_ports.ctl_addr);
	else
		outb(ctl, hwif->io_ports.ctl_addr);
}

static void nlm_common_ide_dev_select(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 select = drive->select | ATA_DEVICE_OBS;

	if (hwif->host_flags & IDE_HFLAG_MMIO)
		writeb(select, (void __iomem *)hwif->io_ports.device_addr);
	else
		outb(select, hwif->io_ports.device_addr);
}

static void nlm_common_ide_tf_load(ide_drive_t *drive, struct ide_taskfile *tf, u8 valid)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	void (*tf_outb)(u8 addr, unsigned long port);
	u8 mmio = (hwif->host_flags & IDE_HFLAG_MMIO) ? 1 : 0;

	if (mmio)
		tf_outb = ide_mm_outb;
	else
		tf_outb = ide_outb;

	if (valid & IDE_VALID_FEATURE)
		tf_outb(tf->feature, io_ports->feature_addr);
	if (valid & IDE_VALID_NSECT)
		tf_outb(tf->nsect, io_ports->nsect_addr);
	if (valid & IDE_VALID_LBAL)
		tf_outb(tf->lbal, io_ports->lbal_addr);
	if (valid & IDE_VALID_LBAM)
		tf_outb(tf->lbam, io_ports->lbam_addr);
	if (valid & IDE_VALID_LBAH)
		tf_outb(tf->lbah, io_ports->lbah_addr);
	if (valid & IDE_VALID_DEVICE)
		tf_outb(tf->device, io_ports->device_addr);
}

static void nlm_common_ide_tf_read(ide_drive_t *drive, struct ide_taskfile *tf, u8 valid)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	u8 (*tf_inb)(unsigned long port);
	u8 mmio = (hwif->host_flags & IDE_HFLAG_MMIO) ? 1 : 0;

	if (mmio)
		tf_inb  = ide_mm_inb;
	else
		tf_inb  = ide_inb;

	if (valid & IDE_VALID_ERROR)
		tf->error  = tf_inb(io_ports->feature_addr);
	if (valid & IDE_VALID_NSECT)
		tf->nsect  = tf_inb(io_ports->nsect_addr);
	if (valid & IDE_VALID_LBAL)
		tf->lbal   = tf_inb(io_ports->lbal_addr);
	if (valid & IDE_VALID_LBAM)
		tf->lbam   = tf_inb(io_ports->lbam_addr);
	if (valid & IDE_VALID_LBAH)
		tf->lbah   = tf_inb(io_ports->lbah_addr);
	if (valid & IDE_VALID_DEVICE)
		tf->device = tf_inb(io_ports->device_addr);
}

/*
 * Some localbus EIDE interfaces require a special access sequence
 * when using 32-bit I/O instructions to transfer data.  We call this
 * the "vlb_sync" sequence, which consists of three successive reads
 * of the sector count register location, with interrupts disabled
 * to ensure that the reads all happen together.
 */
static void ata_vlb_sync(unsigned long port)
{
	(void)inb(port);
	(void)inb(port);
	(void)inb(port);
}

/*
 * This is used for most PIO data transfers *from* the IDE interface
 *
 * These routines will round up any request for an odd number of bytes,
 * so if an odd len is specified, be sure that there's at least one
 * extra byte allocated for the buffer.
 */
static void nlm_common_ide_input_data(ide_drive_t *drive, struct ide_cmd *cmd, void *buf,
		    unsigned int len)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	unsigned long data_addr = io_ports->data_addr;
	unsigned int words = (len + 1) >> 1;
	u8 io_32bit = drive->io_32bit;
	u8 mmio = (hwif->host_flags & IDE_HFLAG_MMIO) ? 1 : 0;

	if (io_32bit) {
		unsigned long uninitialized_var(flags);

		if ((io_32bit & 2) && !mmio) {
			local_irq_save(flags);
			ata_vlb_sync(io_ports->nsect_addr);
		}

		words >>= 1;
		if (mmio)
			__ide_mm_insl((void __iomem *)data_addr, buf, words);
		else
			insl(data_addr, buf, words);

		if ((io_32bit & 2) && !mmio)
			local_irq_restore(flags);

		if (((len + 1) & 3) < 2)
			return;

		buf += len & ~3;
		words = 1;
	}

	if (mmio)
		__ide_mm_insw((void __iomem *)data_addr, buf, words);
	else
		insw(data_addr, buf, words);
}

/*
 * This is used for most PIO data transfers *to* the IDE interface
 */
static void nlm_common_ide_output_data(ide_drive_t *drive, struct ide_cmd *cmd, void *buf,
		     unsigned int len)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	unsigned long data_addr = io_ports->data_addr;
	unsigned int words = (len + 1) >> 1;
	u8 io_32bit = drive->io_32bit;
	u8 mmio = (hwif->host_flags & IDE_HFLAG_MMIO) ? 1 : 0;

	if (io_32bit) {
		unsigned long uninitialized_var(flags);

		if ((io_32bit & 2) && !mmio) {
			local_irq_save(flags);
			ata_vlb_sync(io_ports->nsect_addr);
		}

		words >>= 1;
		if (mmio)
			__ide_mm_outsl((void __iomem *)data_addr, buf, words);
		else
			outsl(data_addr, buf, words);

		if ((io_32bit & 2) && !mmio)
			local_irq_restore(flags);

		if (((len + 1) & 3) < 2)
			return;

		buf += len & ~3;
		words = 1;
	}

	if (mmio)
		__ide_mm_outsw((void __iomem *)data_addr, buf, words);
	else
		outsw(data_addr, buf, words);
}

const struct ide_tp_ops nlm_common_tp_ops = {
	.exec_command		= nlm_common_ide_exec_command,
	.read_status		= nlm_common_ide_read_status,
	.read_altstatus		= nlm_common_ide_read_altstatus,
	.write_devctl		= nlm_common_ide_write_devctl,

	.dev_select		= nlm_common_ide_dev_select,
	.tf_load		= nlm_common_ide_tf_load,
	.tf_read		= nlm_common_ide_tf_read,

	.input_data		= nlm_common_ide_input_data,
	.output_data		= nlm_common_ide_output_data,
};

/* Note ends:
   Above functions are taken from drivers/ide/ide-io-std.c
   */

static const struct ide_port_info nlm_common_port_info = {
	.tp_ops			= &nlm_common_tp_ops,
	.host_flags 		= IDE_HFLAG_NO_DMA,
};

#define NETLOGIC_IDE_REG(pcaddr) (IOADDR(IDE_PHYS) + ((pcaddr)) - mips_io_port_base)

/*
 * nlm_common_ide_probe :
 *      - Probe the PCMCIA interface
 *        on selected  XLR Boards.
 */

static int __devinit nlm_common_ide_probe (struct platform_device *dev)
{
#if defined(NETLOGIC_HAVE_IDE) && defined(IDE_PHYS)
	unsigned int i = 0;
	int ret = 0;
	struct ide_host *host;
	struct ide_hw hw, *hws[] = { &hw };
	ide_hwif_t *nlm_common_ide_hwif;
	extern int dev_tree_en;
	extern int fdt_get_flash_enabled(void);

	if(dev_tree_en && fdt_get_flash_enabled() == 0) {
		printk("FLASH disabled by boot arg, skipping..\n");
		return 0;
	}


	if (xlr_board_atx_iii() || xlr_board_atx_v())
	{
		printk("** Skipping PCMCIA Interface Probe.\n");
		goto out;
	}

	printk ("Initializing xlr PCMCIA IDE...\n");

	memset(&hw, 0, sizeof(hw));
	
	/* setup ports here */
	hw.io_ports.data_addr   = NETLOGIC_IDE_REG(0x1f0);
	hw.io_ports.error_addr  = NETLOGIC_IDE_REG(0x1f1);
	hw.io_ports.nsect_addr  = NETLOGIC_IDE_REG(0x1f2);
	hw.io_ports.lbal_addr   = NETLOGIC_IDE_REG(0x1f3);
	hw.io_ports.lbam_addr   = NETLOGIC_IDE_REG(0x1f4);
	hw.io_ports.lbah_addr   = NETLOGIC_IDE_REG(0x1f5);
	hw.io_ports.device_addr = NETLOGIC_IDE_REG(0x1f6);
	hw.io_ports.status_addr = NETLOGIC_IDE_REG(0x1f7);
	hw.io_ports.ctl_addr    = NETLOGIC_IDE_REG(0x3f6);
	hw.io_ports.irq_addr    = NETLOGIC_IDE_REG(0x3f7);

	hw.irq = PIC_PCMCIA_IRQ;
	hw.dev = &dev->dev;

	ret = ide_host_add(&nlm_common_port_info, hws, 1, &host);
	if (ret)
		goto out;

	platform_set_drvdata(dev, host);

	nlm_common_ide_hwif = host->ports[0];

        printk("Netlogic XLR PCMCIA configured as IDE interface = %d\n", i);
out:
	return ret;

#endif
}

static int nlm_common_ide_remove (struct platform_device *dev)
{

	return 0;
}

static struct platform_driver nlm_common_ide_driver = {
	.driver = {
		.name = "xlr-pcmcia",
		.owner = THIS_MODULE,
	},
	.probe          = nlm_common_ide_probe,
	.remove         = nlm_common_ide_remove,
};

static int __init nlm_common_ide_init(void)
{
	return platform_driver_register(&nlm_common_ide_driver);
}

static void __exit nlm_common_ide_exit(void)
{
	platform_driver_unregister(&nlm_common_ide_driver);
}

module_init(nlm_common_ide_init);
module_exit(nlm_common_ide_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Netlogic XLR-PCMCIA IDE driver");
