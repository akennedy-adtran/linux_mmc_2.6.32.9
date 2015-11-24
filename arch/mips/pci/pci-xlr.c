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


#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/ide.h>

#include <asm/io.h>

#include <asm/netlogic/interrupt.h>
#include <asm/netlogic/pci.h>
#include <asm/netlogic/io.h>         
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/nlm_srio.h>

#define  PCI_HT_LCTR_INIT   0x0020  /* Initialization Complete  */
#define  PCIE_LINK_STATE    0x4000  /* Bit 14, Datalink Status  */

#define LSU_CFG0_REGID       0
#define LSU_CERRLOG_REGID    9
#define LSU_CERROVF_REGID    10
#define LSU_CERRINT_REGID    11


#define pci_cfg_offset(bus,devfn,where) (((bus)<<16)+((devfn)<<8)+(where))
#define pci_cfg_addr(bus,devfn,where) pci_cfg_offset((bus)->number,(devfn),where)
int  pci_start_busno;
static int  pci_bus_status;
static int  pci_start_bus_fixed;
#if 0
/*
  Maximum bus number on PCI is 0xff,
  hence, start the ht_busno with
  0xff + 1. This variable will get
  reset to the actual value when
  the enumeration of HT begins.
*/
int  ht_start_busno = 0x100;
#else
int  ht_start_busno = 0;
#endif
static int  ht_start_bus_fixed;
static void *pci_config_base;
#if 0
static void *pci_io_base;
#else
void *pci_io_base;
#endif
static void *ht_io_base;

volatile void *ht_config_base;
/* Global Link Status */
int link0 = 0, link1 = 0, link2 = 0, link3 = 0;

#define CFGTYPE(x) ((x)<(1)?(0):(1))
#define MB16 0x01000000

#define SWAP32(x)				\
        (((x) & 0xff000000) >> 24) |		\
        (((x) & 0x000000ff) << 24) |		\
        (((x) & 0x0000ff00) << 8)  |		\
        (((x) & 0x00ff0000) >> 8)

static __inline__ void disable_and_clear_cache_error(void)
{
        uint64_t lsu_cfg0 = read_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID);
        lsu_cfg0 = lsu_cfg0 & ~0x2e;
        write_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID, lsu_cfg0);
        /* Clear cache error log */
        write_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID, 0);
}

static __inline__ void clear_and_enable_cache_error(void)
{
        uint64_t lsu_cfg0 = 0;

        /* first clear the cache error logging register */
        write_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID, 0);
        write_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERROVF_REGID, 0);
        write_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRINT_REGID, 0);

        lsu_cfg0 = read_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID);
        lsu_cfg0 = lsu_cfg0 | 0x2e;
        write_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID, lsu_cfg0);
}

static inline int ht_controller_init_done(void)
{
	int init_done=0;
	nlm_reg_t *ht_mmio = netlogic_io_mmio(NETLOGIC_IO_HT_OFFSET);
	nlm_reg_t reg = cpu_to_le32(netlogic_read_reg(ht_mmio, (0xA4 >> 2)));
	if ((uint16_t)reg & PCI_HT_LCTR_INIT)
		init_done = 1;
	else
		printk("Skipping XLR HT-Controller Registration...\n");
	return init_done;
}

void pcie_controller_init_done(void) {

    nlm_reg_t *pcie_mmio_le = netlogic_io_mmio(NETLOGIC_IO_PCIE_1_OFFSET);
	nlm_reg_t reg_link0     = netlogic_read_reg(pcie_mmio_le, (0x80 >> 2));
	nlm_reg_t reg_link1     = netlogic_read_reg(pcie_mmio_le, (0x84 >> 2));
	nlm_reg_t reg_link2  = 0;
	nlm_reg_t reg_link3  = 0;

	if ((uint16_t)reg_link0 & PCIE_LINK_STATE)
		link0 = 1;
    else
        link0 = 0;

	if ((uint16_t)reg_link1 & PCIE_LINK_STATE)
		link1 = 1;
    else
        link1 = 0;

    if(is_xls2xx() || is_xls_b0()){

        reg_link2 = netlogic_read_reg(pcie_mmio_le, (0x180 >> 2));

        if((uint16_t)reg_link2 & PCIE_LINK_STATE)
            link2 = 1;
        else 
            link2 = 0;

	    reg_link3 = netlogic_read_reg(pcie_mmio_le, (0x184 >> 2));

        if((uint16_t)reg_link3 & PCIE_LINK_STATE)
            link3 = 1;
        else
            link3 = 0;
    }
}

#if 1
static inline __u32 pci_cfg_read_32bit(__u32 addr)
{
    __u32 temp = 0;
    __u32 *p = (__u32 *) (pci_config_base + (addr & ~3));
    __u64 cerr_cpu_log = 0;

    disable_and_clear_cache_error();

    temp = SWAP32(*p);

    /* Read cache err log */
    cerr_cpu_log = read_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID);
    if(cerr_cpu_log) {
        /* Device doesn't exist. */
        temp = ~0x0;
    }
    clear_and_enable_cache_error();
    return temp;
}

static inline void pci_cfg_write_32bit(__u32 addr, __u32 data) {
    
        unsigned int *p = (unsigned int *)(pci_config_base + (addr & ~3));
        *p = SWAP32(data);
}
#else
/*
 * Read/write 32-bit values in config space.
 * pci config space is little endian
 */
static inline __u32 pci_cfg_read_32bit(__u32 addr)
{
	__u8 *p = (__u8 *)(pci_config_base + (addr & ~3));
	//printk("[%s]: addr = %p, data = %x\n", __FUNCTION__, p, *(__u32*)p);
	return ( (*(p+3) << 24) | (*(p+2) << 16) | (*(p+1) << 8) | *p);
}

static inline void pci_cfg_write_32bit(__u32 addr, __u32 data)
{
	__u8 *p = (__u8 *)(pci_config_base + (addr & ~3));
	int i=0;
	for(i=0;i<4;i++)
		p[i] = (data >> (i<<3)) & 0xff;
}
#endif

/*
 * Low-level HT Configuration READ and Write Routines
 */
static inline __u32 ht_cfg_read_32bit(unsigned long addr) {

    __u8 *p;
    __u32 temp = 0;
    __u64 cerr_cpu_log = 0;

    disable_and_clear_cache_error();
    p = (__u8 *)((addr & ~3));
    //printk("[%s]: addr = %p, data = %x\n", __FUNCTION__, p, *(__u32*)p);
    temp =  ( (*(p+3) << 24) | (*(p+2) << 16) | (*(p+1) << 8) | *p);

    cerr_cpu_log = read_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID);

    if(cerr_cpu_log) {
        /* Device doesn't exist. */
        temp = ~0x0;
    }
    clear_and_enable_cache_error();
    return temp;
}

static inline void ht_cfg_write_32bit(unsigned long addr, __u32 data) {

    __u8 *p;
    int i=0;
    p = (__u8 *)((addr & ~3));

    for(i=0;i<4;i++)
        p[i] = (data >> (i<<3)) & 0xff;
}
/*
 * HT Wrapper Routine: READ
 */
static int nlm_common_htbios_read(struct pci_bus *bus, unsigned int devfn,
                               int where, int size, u32 * val)
{
	__u32 data = 0;
	unsigned long long int cfgaddr;

	/* Keep track of where the PCIX
	 * bus numbering starts from..
	 */
	if (!ht_start_bus_fixed) {
		ht_start_busno     = (int)(bus->number);
		ht_start_bus_fixed = 1;
	}

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	cfgaddr = (long) ht_config_base +
		CFGTYPE(bus->number - ht_start_busno) * MB16 +
		pci_cfg_offset((int)(bus->number-ht_start_busno),devfn,where);

	if (pci_bus_status)
		data = ht_cfg_read_32bit(cfgaddr);
	else
		data = 0xFFFFFFFF;

	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	else
		*val = data;

	return PCIBIOS_SUCCESSFUL;
}
/*
 * HT Wrapper Routine: WRITE
 */
static int nlm_common_htbios_write(struct pci_bus *bus, unsigned int devfn,
                                int where, int size, u32 val)
{
	unsigned long long int cfgaddr;
	//    __u32 cfgaddr = pci_cfg_offset(bus->number , devfn, where);
	__u32 data = 0;

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (!pci_bus_status)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	cfgaddr = (long) ht_config_base +
		CFGTYPE(bus->number - ht_start_busno) * MB16 +
		pci_cfg_offset((bus->number-ht_start_busno),devfn,where);


	data = ht_cfg_read_32bit(cfgaddr);

	if (size == 1)
		data = (data & ~(0xff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	else if (size == 2)
		data = (data & ~(0xffff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	else
		data = val;

	ht_cfg_write_32bit(cfgaddr, data);

	return PCIBIOS_SUCCESSFUL;
}

static int nlm_common_pcibios_read(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 * val)
{
	__u32 data = 0;

	/* Keep track of where the PCIX
	 * bus numbering starts from..
	 */
	if (!pci_start_bus_fixed) {
		pci_start_busno     = (int)(bus->number);
		pci_start_bus_fixed = 1;
	}

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (pci_bus_status)
		data = pci_cfg_read_32bit(pci_cfg_offset((bus->number-pci_start_busno), devfn, where));
	else
		data = 0xFFFFFFFF;

	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	else
		*val = data;

	return PCIBIOS_SUCCESSFUL;
}

static int nlm_common_pcibios_write(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	__u32 cfgaddr = pci_cfg_offset((bus->number-pci_start_busno), devfn, where);
	__u32 data = 0;

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (!pci_bus_status)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	data = pci_cfg_read_32bit(cfgaddr);

	if (size == 1)
		data = (data & ~(0xff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	else if (size == 2)
		data = (data & ~(0xffff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	else
		data = val;

	pci_cfg_write_32bit(cfgaddr, data);

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops nlm_common_pci_ops = {
	.read  = nlm_common_pcibios_read,
	.write = nlm_common_pcibios_write
};

struct pci_ops nlm_common_ht_ops = {
	.read  = nlm_common_htbios_read,
	.write = nlm_common_htbios_write
};

/*
 * XLR PCIX Controller
 */
static struct resource nlm_common_mem_resource = {
	.name           = "XLR PCI MEM",
	.start          = 0xd0000000UL,          /* 256MB PCI mem @ 0xd000_0000 */
	.end            = 0xdfffffffUL,
	.flags          = IORESOURCE_MEM,
};
static struct resource netlogic_io_resource = {
	.name           = "XLR IO MEM",
	.start          = 0x10000000UL,         /* 16MB PCI IO @ 0x1000_0000 */
	.end            = 0x100fffffUL,
	.flags          = IORESOURCE_IO,
};
struct pci_controller nlm_common_controller = {
	.index          = 0,
	.pci_ops        = &nlm_common_pci_ops,
	.mem_resource   = &nlm_common_mem_resource,
	.io_resource    = &netlogic_io_resource,
	.io_offset      = 0x00000000UL,
	.mem_offset     = 0x00000000UL
};

/*
 * XLR HT Controller
 */
static struct resource nlm_common_htmem_resource = {
	.name           = "XLR HT MEM",
	.start          = 0xc0000000UL,                 /* 256MB HT mem @ 0xC0000000 */
	.end            = 0xcfffffffUL,
	.flags          = IORESOURCE_MEM,
};
static struct resource nlm_common_htio_resource = {
	.name           = "XLR HT IO",
	.start          = 0x14000000UL,                 /* 16MB HT IO @ 0x1400_0000 */
	.end            = 0x140fffffUL,
	.flags          = IORESOURCE_IO,
};
struct pci_controller nlm_common_ht_controller = {
	.index          = 1,
	.pci_ops        = &nlm_common_ht_ops,
	.mem_resource   = &nlm_common_htmem_resource,
	.io_resource    = &nlm_common_htio_resource,
	.io_offset      = 0x00000000UL,
	.mem_offset     = 0x00000000UL
};

/* I/O routines for IDE on PCI */
#define pci_ide_phys_to_virt(x) (((x) - (netlogic_io_resource.start)) + (unsigned long)pci_io_base )

inline void nlm_ide_mm_insw(unsigned long port, void *addr, u32 count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		*(u16 *)addr = (__raw_readw(v_port));
		addr += 2;
	}
}

inline void nlm_ide_mm_insl(unsigned long port, void *addr, unsigned int count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		*(u32 *)addr = (__raw_readl(v_port));
		addr += 4;
	}
}

inline void nlm_ide_mm_outsw(unsigned long port, void *addr, unsigned int count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		__raw_writew(*(u16 *)addr, v_port);
		addr += 2;
	}
}

inline void nlm_ide_mm_outsl(unsigned long port, void *addr, unsigned int count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		__raw_writel(*(u32 *)addr, v_port);
		addr += 4;
	}
}

u8 nlm_ide_mm_inb (unsigned long port)
{
	return((u8)__raw_readb(pci_ide_phys_to_virt(port)));
}

u16 nlm_ide_mm_inw (unsigned long port)
{
	return ((u16) swab16(__raw_readw(pci_ide_phys_to_virt(port))));
}
/* Not part of hwif anymore; remove static declaration */
u32 nlm_ide_mm_inl (unsigned long port)
{
	return ((u32)swab32(__raw_readl(pci_ide_phys_to_virt(port))));
}

void nlm_ide_mm_outb (u8 value, unsigned long port)
{
	__raw_writeb(value, pci_ide_phys_to_virt(port));
}

void nlm_ide_mm_outw (u16 value, unsigned long port)
{
	__raw_writew(swab16(value), pci_ide_phys_to_virt(port));
}
/* Not part of hwif anymore; remove static declaration */
void nlm_ide_mm_outl (u32 value, unsigned long port)
{
	__raw_writel(swab32(value), pci_ide_phys_to_virt(port));
}

#if 0
static void nlm_ide_mm_outbsync (ide_drive_t *drive, u8 value, unsigned long port)
{
	__raw_writeb(value, pci_ide_phys_to_virt(port));
}

void xlr_hwif_mmiops (ide_hwif_t *hwif)
{
	hwif->OUTB      = nlm_ide_mm_outb;
	hwif->OUTBSYNC  = nlm_ide_mm_outbsync;
	hwif->OUTW      = nlm_ide_mm_outw;
	hwif->OUTSW     = nlm_ide_mm_outsw;
	hwif->OUTSL     = nlm_ide_mm_outsl;
	hwif->INB       = nlm_ide_mm_inb;
	hwif->INW       = nlm_ide_mm_inw;
	hwif->INSW      = nlm_ide_mm_insw;
	hwif->INSL      = nlm_ide_mm_insl;
}

EXPORT_SYMBOL(xlr_hwif_mmiops);
#endif

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
#if defined(XLP_SIM)
	return 0;
#else
    /* Sane default values for XLR */
    int index = 0;
    int retVal= PIC_PCIX_IRQ;    

    if (is_xls() && !is_xls2xx() && !is_xls_b0()) {
        if (link0) {
            if (dev->bus->number == 1)
                return 0x22;
            else
                return 0x23;
        }
        else if (link1) {
            if (dev->bus->number == 1)
                return 0x23;
        }
    }else if(is_xls2xx() || is_xls_b0()){
        if(dev->bus->number>0){
            switch(dev->bus->self->devfn){
            case 0x0:
                return 0x22;
            case 0x8:
                return 0x23;
            case 0x10:
                if(is_xls_b0())
                    return 0x24;
                return 0x1f;
            case 0x18:
                if(is_xls_b0())
                    return 0x25;
                return 0x20;
            default:
                break;
            }
        }else 
            return 0x23;    /*Need to FIX!!!, XLS logic does same.. Or probably
                             we would never come here with bus number ZERO*/
    }else{
        /* XLR */
        index = ((struct pci_controller *)(dev->sysdata))->index;
        if(index == 0)
            /* IRQ Vector 24 for PCI/X Devices */
            retVal = PIC_PCIX_IRQ;   
        else if (index == 1)
            /* IRQ Vector 23 for HT Devices */
            retVal = PIC_HYPER_IRQ;  
    }
    return retVal;
#endif
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
        return 0;
}

extern int pci_probe_only;

/* Enabled by default */
static int __initdata phoeni_nopci = 0; 

#if defined(CONFIG_NLM_XLR)
static int __init xlr_nopci_setup(char *str)
{
    /* Disable PCI/X/E; disables HT also */
	phoeni_nopci = 1;  
	return 1;
}
__setup("xlr_nopci", xlr_nopci_setup);
#endif

#if defined(CONFIG_NLM_XLP)
static int __init xlp_nopci_setup(char *str)
{
    /* Disable PCI/X/E; disables HT also */
	phoeni_nopci = 1;  
	return 1;
}
__setup("xlp_nopci", xlp_nopci_setup);
#endif
/*
extern uint32_t dev_tree_en;
*/
extern int fdt_get_pci_ht_map(int *pci_enable, int *ht_enable);

static int __init pcibios_init(void)
{
	int pci_enable, ht_enable;
#if 0
	if(dev_tree_en)
		fdt_get_pci_ht_map(&pci_enable, &ht_enable);
	else
#endif
		pci_enable = ht_enable = !phoeni_nopci;

	if (!pci_enable && !ht_enable)  {
		printk("PCI & HT disabled by boot arguments  - skipping.\n");
		return 0;
	}
		
	if (phoeni_nopci || xlr_board_atx_iii() || xlr_board_atx_v()) 
        return 0;

    if(is_xls_b0() && is_xlsb0_srio()){
        printk("Detected XLS B0 in SRIO Mode, Skipping PCIE\n");
        return 0;
    }
    
	/* PSB assigns PCI resources */
	pci_probe_only = 1;

	/* Map the PCIX CFG space */
	if(pci_enable) {
		pci_config_base = ioremap(DEFAULT_PCI_CONFIG_BASE, (32<<20));
		if (!pci_config_base) {
			printk("Unable to map PCI config space!\n");
			return 1;
		}


		{
			unsigned long phys = netlogic_io_resource.start;
			unsigned long size = netlogic_io_resource.end - netlogic_io_resource.start + 1;

			pci_io_base = ioremap(phys, size);
			if (!pci_io_base) {
				printk("[%s]: Unable to IO-Remap phys=%lx, size=%lx\n",
			       __FUNCTION__, phys, size);
			}
			else {
				printk("[%s]: IO-Remapped phys=%lx, size=%lx to vaddr=%p\n",
			       __FUNCTION__, phys, size, pci_io_base);
			}
		}
	}
	
	if(ht_enable) {
		/* Map the HT CFG spaces... */
		ht_config_base = ioremap(DEFAULT_HT_TYPE0_CFG_BASE, (32<<20));
		if (!ht_config_base) {
			printk("Unable to map HT config space!\n");
			return 1;
		}

		{
			unsigned long phys = nlm_common_htio_resource.start;
			unsigned long size = nlm_common_htio_resource.end - nlm_common_htio_resource.start + 1;

			ht_io_base = ioremap(phys, size);
			if (!ht_io_base) {
				printk("[%s]: Unable to IO-Remap phys=%lx, size=%lx\n",
			       __FUNCTION__, phys, size);
			}
			else {
				printk("[%s]: IO-Remapped phys=%lx, size=%lx to vaddr=%p\n",
			       __FUNCTION__, phys, size, ht_io_base);
			}
		}
	}

	pci_bus_status = 1;
	pci_start_bus_fixed = 0;
	ht_start_bus_fixed = 0;

	/* IO Range for 16MB from where the MEM Range Ends */
	ioport_resource.start =  0;
	ioport_resource.end   = ~0;
	if(pci_enable) {
	    printk("Registering XLR/XLS PCIX/PCIE Controller. \n");
    	if (is_xls())
        	pcie_controller_init_done();
	    register_pci_controller(&nlm_common_controller);
	}

    /* XLS : No native HT */
    if (ht_enable && !is_xls()) {
        /* XLR : ATX1, ATX2B Boards */
        if ((xlr_board_atx_i() || xlr_board_atx_ii_b()) && ht_controller_init_done()) {
            printk("Registering XLR HT Controller. \n");
            register_pci_controller(&nlm_common_ht_controller);
        }
    }
	return 0;
}

#if 0
void __iomem *pci_iomap(struct pci_dev *dev, int bar, unsigned long max)
{
	unsigned long start = pci_resource_start(dev, bar);
	unsigned long len = pci_resource_len(dev, bar);
	unsigned long flags = pci_resource_flags(dev, bar);

	if (!len)
		return NULL;
	if (max && len > max)
		len = max;
#if 0
	if (flags & IORESOURCE_IO)
		return ioport_map(start, len);
#endif
	if (flags & IORESOURCE_MEM)
		return ioremap(start, len);
	/* What? */
	return NULL;
}


void pci_iounmap(struct pci_dev *dev, void __iomem *addr)
{
	/* Nothing to do */
}
#endif

int
pci_set_dma_mask(struct pci_dev *dev, u64 mask)
{
	if (!pci_dma_supported(dev, mask))
		return -EIO;

	if(mask > DMA_BIT_MASK(32))
		return -EIO;

	if(xlr_revision_c())
		dev->dma_mask = mask & 0x7fffffffULL;
	else
		dev->dma_mask = mask;

	dev_dbg(&dev->dev, "using %dbit DMA mask\n", fls64(mask));

	return 0;
}

int
pci_set_consistent_dma_mask(struct pci_dev *dev, u64 mask)
{
	if (!pci_dma_supported(dev, mask))
		return -EIO;

	if(mask > DMA_BIT_MASK(32))
		return -EIO;

	if(xlr_revision_c())
		dev->dev.coherent_dma_mask = mask & 0x7fffffffULL;
	else
		dev->dev.coherent_dma_mask = mask;

	dev_dbg(&dev->dev, "using %dbit consistent DMA mask\n", fls64(mask));

	return 0;
}

arch_initcall(pcibios_init);

struct pci_fixup pcibios_fixups[] = {
	{0}
};

