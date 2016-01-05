/*-
 * Copyright (c) 2003-2015 Broadcom Corporation
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


/*
 * This file contains specific functions for XLP chipsets and
 * EVP boards.
 */
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/ide.h>
#if defined CONFIG_PCIEPORTBUS
#include <linux/pcieport_if.h>
#endif
#include <asm/io.h>

#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/pci.h>
#include <asm/netlogic/io.h>
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/mach-netlogic/mmu.h>

extern int pci_probe_only;

void __iomem *xlp_pci_config_base;
EXPORT_SYMBOL(xlp_pci_config_base);

static const volatile void *pci_io_base;

int xlp_intx_enable(u8, int);
int xlp_intx_disable(u8, int);
int xlp_msi_enable(u8, int, u32);
int xlp_msix_enable(u8, int);
int xlp_msi_disable(u8, int, u32);
int xlp_msix_disable(u8, int);
u32 xlp_msi_set_mask(u8, int, int, int);

u64 xlp_syscfg_base[NLM_MAX_CPU_NODE] = { XLP_BDF_BASE(0,6,5),
	XLP_BDF_BASE(0,14,5), XLP_BDF_BASE(0,22,5), XLP_BDF_BASE(0,30,5) };

u64 xlp_pci_base[NLM_MAX_CPU_NODE][XLP_MAX_SLOTS] = {
	/* This should be accessed as xlp_pci_base[node][fn] */

	{XLP_BDF_BASE(0,1,0), XLP_BDF_BASE(0,1,1),
	XLP_BDF_BASE(0,1,2), XLP_BDF_BASE(0,1,3)},

	{XLP_BDF_BASE(0,9,0), XLP_BDF_BASE(0,9,1),
	XLP_BDF_BASE(0,9,2), XLP_BDF_BASE(0,9,3)},

	{XLP_BDF_BASE(0,17,0), XLP_BDF_BASE(0,17,1),
	XLP_BDF_BASE(0,17,2), XLP_BDF_BASE(0,17,3)},

	{XLP_BDF_BASE(0,25,0), XLP_BDF_BASE(0,25,1),
	XLP_BDF_BASE(0,25,2), XLP_BDF_BASE(0,25,3)}
};

u64 __nlh_pci_r32o(u8 nid, u8 fn, u64 offset)
{
	return (nlh_read_cfg_reg32(xlp_pci_base[nid][fn] + offset));
}

void __nlh_pci_w32o(u8 nid, u8 fn, u64 offset, u32 val)
{
	nlh_write_cfg_reg32(xlp_pci_base[nid][fn] + offset, val);
}

/*
 * Possible values are no more hard coded.
 * For mapping of these values to IRT, refer
 * arch/mips/netlogic/xlp/irq.c
 *
 * Here a table is defined to figure out the interrupt assignments to different
 * cards placed on any of the 4 PCI slots.
 *
 * We have some unique problems here.
 * 1. Board could be configured in different lane widths. That means, the cards
 * could be controlled by different functions of the controller on board
 * Eg. 2x8 config can have two cards (fn 0 and fn 2)
 *	4x4 config can also have two cards (under fn0 through fn 3)
 * 2. Cards can be placed on any available slot
 * 3. The card can have a switch built in, thus giving rise to multiple devices
 * on the slot.
 * 4. All these problems can occur on four different nodes.
 *
 * So, it is important to figure out the lanes and nodes on which cards are
 * placed.  First we read the lane config from POWER_ON_RESET_CFG on each node
 * Then each line's LTSSM state would give the card presence
 * Based on that we have to assign interrupt values; while keeping the
 * possibility of same interrupt assigned to multiple devices open.
 *
 * So, we have a map: XLP irq map is as follows (on node 0, but similar for any
 * node
 *  \fn 0	1	2	3
 *plc\
 * 0	86	0	88	89
 * 1	86	87	88	0
 * 2	86	0	88	89
 * 3	86	87	88	89
 * This map can differ among different versions of processors. Check PRM or RTL
 * because the values are a function of XLP_PCIE_LINK_IRT_OFFSET. To make them
 * somewhat independent, I have defined macros and used them here.
 *
 * This map is dynamically populated based on card presence in the slot.
 * If a card is present, and is a switch, then the secondary and subordinate
 * numbers would be different. Based on this fact, we can figure out from
 * pci_dev structure the slot where a card is placed at run time.
 */

/* There are 4 PCI lane config modes for every node in XLP8xx
 * 0:	2x8 lanes (ctrl 0, 2)
 * 1:	2x4 lanes (ctrl 0, 2), 1x8 lanes (ctrl 1)
 * 2:	1x8 lanes (ctrl 0), 2x4 lanes (ctrl 2, 3)
 * 3:	4x4 lanes (all ctrls 4 lanes)
 * PRM 31.11.7.2 (power on reset config register)
 *
 * But XLP3xx{L,H,Q} have fixed configuration (no need to read plc)
 * given in Section 22 of PRM
 */
#define XLP_PCI_LANE_CONFIG	4

struct xlp_link_struct {
	int intno;
	int sec;
	int sub;
};

struct xlp_plc_fn_struct {
	int plc;
	struct xlp_link_struct farray[XLP_MAX_SLOTS];
};

static struct xlp_plc_fn_struct
	node_irqmap[NLM_MAX_CPU_NODE][XLP_PCI_LANE_CONFIG] = {
{
	{0, {{XLP_PCIE_LINK_IRQ(0,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(0,2), 0, 0}, {0, 0, 0}}},
	{1, {{XLP_PCIE_LINK_IRQ(0,0), 0, 0}, {XLP_PCIE_LINK_IRQ(0,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(0,2), 0, 0}, {0, 0, 0}}},
	{2, {{XLP_PCIE_LINK_IRQ(0,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(0,2), 0, 0}, {XLP_PCIE_LINK_IRQ(0,3),0,0}}},
	{3, {{XLP_PCIE_LINK_IRQ(0,0), 0, 0}, {XLP_PCIE_LINK_IRQ(0,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(0,2), 0, 0}, {XLP_PCIE_LINK_IRQ(0,3),0,0}}},
}, {
	{0, {{XLP_PCIE_LINK_IRQ(1,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(1,2), 0, 0}, {0, 0, 0}}},
	{1, {{XLP_PCIE_LINK_IRQ(1,0), 0, 0}, {XLP_PCIE_LINK_IRQ(1,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(1,2), 0, 0}, {0, 0, 0}}},
	{2, {{XLP_PCIE_LINK_IRQ(1,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(1,2), 0, 0}, {XLP_PCIE_LINK_IRQ(1,3),0,0}}},
	{3, {{XLP_PCIE_LINK_IRQ(1,0), 0, 0}, {XLP_PCIE_LINK_IRQ(1,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(1,2), 0, 0}, {XLP_PCIE_LINK_IRQ(1,3),0,0}}},
}, {
	{0, {{XLP_PCIE_LINK_IRQ(2,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(2,2), 0, 0}, {0, 0, 0}}},
	{1, {{XLP_PCIE_LINK_IRQ(2,0), 0, 0}, {XLP_PCIE_LINK_IRQ(2,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(2,2), 0, 0}, {0, 0, 0}}},
	{2, {{XLP_PCIE_LINK_IRQ(2,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(2,2), 0, 0}, {XLP_PCIE_LINK_IRQ(2,3),0,0}}},
	{3, {{XLP_PCIE_LINK_IRQ(2,0), 0, 0}, {XLP_PCIE_LINK_IRQ(2,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(2,2), 0, 0}, {XLP_PCIE_LINK_IRQ(2,3),0,0}}},
}, {
	{0, {{XLP_PCIE_LINK_IRQ(3,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(3,2), 0, 0}, {0, 0, 0}}},
	{1, {{XLP_PCIE_LINK_IRQ(3,0), 0, 0}, {XLP_PCIE_LINK_IRQ(3,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(3,2), 0, 0}, {0, 0, 0}}},
	{2, {{XLP_PCIE_LINK_IRQ(3,0), 0, 0}, {0, 0, 0},
		{XLP_PCIE_LINK_IRQ(3,2), 0, 0}, {XLP_PCIE_LINK_IRQ(3,3),0,0}}},
	{3, {{XLP_PCIE_LINK_IRQ(3,0), 0, 0}, {XLP_PCIE_LINK_IRQ(3,1), 0, 0},
		{XLP_PCIE_LINK_IRQ(3,2), 0, 0}, {XLP_PCIE_LINK_IRQ(3,3),0,0}}},
}
};

void dump_node_irqmap(u8 node, u32 plc)
{
	struct xlp_plc_fn_struct *p = &node_irqmap[node][plc];
	u8 idx;

	printk(KERN_WARNING "node_irqmap[%d][%d] PLC %d\n", node, plc, p->plc);
	for (idx = 0; idx < XLP_MAX_SLOTS; idx++) {
		printk("p->farray[%d] : int %d, sec %d, sub %d\n",idx,
		p->farray[idx].intno, p->farray[idx].sec, p->farray[idx].sub);
	}
	return;
}

/* The following is the table describing current interrupt modes of
 * XLP controllers. When an external switch is present, different devices
 * can request different interrupt mode on the same controller which might lead
 * to controller changing previous interrupt mode. If this happens, interrupt
 * delivery will not work correctly. So, we need a way to prevent different
 * devices requesting different interrupt modes. This is kind of impossible
 * because we can't control all device drivers, but we can
 *	1. fail pci_enable_msi{x} if intX is set and at least one interrupt
 *	is allocated
 *	2. fail request_irq() for any interrupt outside current interrupt
 *	distribution range
 */
struct xlp_intmode_struct {
	u32 mode;
	int usage;
};
static struct xlp_intmode_struct intmode[NLM_MAX_CPU_NODE][XLP_MAX_SLOTS];

int xlp_ctrl_intmode_add(u8 node, int fn, int mode, int i)
{
	if (intmode[node][fn].mode != mode) {
		return -EBUSY;
	}
	intmode[node][fn].usage += i;
	if ((intmode[node][fn].usage < 0) || (intmode[node][fn].usage == 0)) {
		intmode[node][fn].usage = 0;
	}
	return intmode[node][fn].usage;
}


int xlp_get_ctrl_intmode(u8 node, int fn)
{
	return intmode[node][fn].mode;
}

int xlp_set_ctrl_intmode(u8 node, int fn, int mode)
{
	int ret = 0;
	if (intmode[node][fn].mode == mode) {
		/* do nothing */
	} else if (intmode[node][fn].usage == 0) {
		intmode[node][fn].mode = mode;
	} else {
		ret = -EBUSY;
	}
	return ret;
}

/* Just a helper function to fill up xlp_irq_map table's entries
 * This function checks whether a PCIe slot is populated and if yes,
 * fills up the table with subordinate and secondary bus numbers. These
 * numbers would be different only if the PCIe device has a switch inside.
 */
static int xlp_map_helper(u8 node, int row, int fn)
{
	u32 reg6, ltssm;

	ltssm = nlh_pci_r32r(node, fn,  0x25E);
	if (ltssm != 0x00446000) {
		printk(KERN_WARNING "LTSSM state is %#x. Fn %x link not up\n",
				ltssm, fn);
		return -ENODEV;
	}
	reg6 = nlh_pci_r32r(node, fn,  0x6);
	node_irqmap[node][row].farray[fn].sec = (reg6 >> 8) & 0xff;
	node_irqmap[node][row].farray[fn].sub = (reg6 >> 16) & 0xff;
	return 0;
}

#if defined CONFIG_PCIEPORTBUS
int xlp_is_dev_rc(struct pci_dev *dev)
{
	u16 reg16 = 0;
	int pos, port_type = 0;

	if (!(pos = pci_find_capability(dev, PCI_CAP_ID_EXP)))
		return -EINVAL;
	pci_read_config_word(dev, pos + PCIE_CAPABILITIES_REG, &reg16);
	port_type = (reg16 >> 4) & PORT_TYPE_MASK;
	if (port_type == PCIE_RC_PORT) {
		return 0;
	}
	return -EINVAL;
}
#endif

/*
 * Iterates over buses to find out the slot (thus pci controller fn)
 */
int xlp_ctrl_fn_from_dev(const struct pci_dev *dev, struct xlp_nodefn_struct *n)
{
	__label__ out;
	int row = 0, fn = 0, node;

#if defined CONFIG_PCIEPORTBUS
	if (xlp_is_dev_rc(dev) == 0) {
		int slot = (dev->devfn >> 3) & 0x1f;
		fn = dev->devfn & 0x3;
		node = slot / 8;
		n->node = node; n->fn = fn;
		return 0;
	}
#endif
	for_each_online_node(node){
		row = 0;
		while (row < XLP_MAX_SLOTS) {
			fn = 0;
			while (fn < XLP_MAX_SLOTS) {
				if ((dev->bus->number >= node_irqmap[node][row].farray[fn].sec)
				&&(dev->bus->number <= node_irqmap[node][row].farray[fn].sub)){
					goto out; /* No `break', note two loops */;
				}
				fn++;
			}
			row++;
		}
	}
out:
	if (fn >= 4) {
		return -ENODEV;
	}
	n->node = node;
	n->fn = fn;
	return 0;
}

/*
 * We discard the idea of a fixed address for MSI. But if that is ever required,
 * define CONFIG_XLP_MSI_ADDRESSES
 */
#ifndef CONFIG_XLP_MSI_ADDRESSES
static u64 XLP_MSI_ADDR = 0;
#endif

volatile const void *xlp_msix_addr_start(u8 node, int fn)
{
	if (XLP_MSI_ADDR == 0) {
		return 0;
	}
	return (volatile const void *)(XLP_MSI_ADDR +
		(XLP_MSIX_ADDR_SIZE * XLP_MAX_SLOTS * node) +
		(fn * XLP_MSIX_ADDR_SIZE));
}

volatile const void *xlp_msi_addr_start(u8 node, int fn)
{
	if (XLP_MSI_ADDR == 0) {
		return 0;
	}
	return (volatile const void *)(XLP_MSI_ADDR +
		(XLP_MSI_ADDR_SIZE * XLP_MAX_SLOTS * node) +
		(fn * XLP_MSI_ADDR_SIZE));
}

/* Irrespective of any device requesting MSI/MSI-X, we keep the controller
 * ready by programming the corresponding registers. This action, per se,
 * does not start MSI/MSI-X for they have to be enabled explicitly.
 */
static void xlp_msi_controller_init(u8 node, int fn)
{
	u32 mmc, msi;

	if (XLP_MSI_ADDR == 0) {
		printk(KERN_ERR "MSI/MSI-X CANNOT be programmed\n");
		return;
	}
	msi = nlh_pci_r32r(node, fn,  0x14);
	mmc = (msi >> 17) & 0x7;
	/* Initialize MSI Base register */
	nlh_pci_w32r(node, fn, 0x15,
		virt_to_phys(xlp_msi_addr_start(node, fn)) & 0xffffffff);
	nlh_pci_w32r(node, fn, 0x16,
		(virt_to_phys(xlp_msi_addr_start(node, fn)) >> 32) & 0xffffffff);
	nlh_pci_w32r(node, fn, 0x17, 0x0);
	msi |= ((mmc << 20) | (1 << 16));
	nlh_pci_w32r(node, fn, 0x14, msi);
	/* Initialize MSI-X Base and Address reg. Note >> 8 in the address.
	 * This is how 40bit address goes in 32bit registers.*/
	nlh_pci_w32r(node, fn, 0x24F,
		(virt_to_phys(xlp_msix_addr_start(node, fn)) >> 8));
	nlh_pci_w32r(node, fn, 0x250,
		(virt_to_phys(xlp_msix_addr_start(node, fn) + XLP_MSIX_ADDR_SIZE) >> 8));
}

/*
 * Controller is initialized and explicity disabled
 *
 * @fn : controller function no.
 */
void xlp_pcie_controller_setup(u8 node, int fn)
{
	u32 reg;

	xlp_msi_controller_init(node, fn);
	//xlp_msix_disable(fn);
	//xlp_msi_disable(fn, 0xf);
	/* By default, leave INTX enabled */
	xlp_intx_enable(node, fn);

	//enable timeout to void system hang, when there is no device on slot
	reg=nlh_pci_r32r(node, fn,  0x240);
	nlh_pci_w32r(node, fn, 0x240, reg | (3<<23) );
	nlh_pci_w32r(node, fn, 0x244, 25*1000*1000);	//0.1 second delay, 250MHz clock
	nlh_pci_w32r(node, fn, 0x245, 25*1000*1000);
}

/*
 * Utility function to get syscfg
 *
 * @node : node id in multi chip config
 */
u32 xlp_get_power_on_reset_cfg(int node)
{
	return nlm_hal_read_32bit_reg(xlp_syscfg_base[node], 0x41);
}
EXPORT_SYMBOL(xlp_get_power_on_reset_cfg);

static void xlp_2xx_pcie_controller_init(void)
{
	u32 plc, syscfg, mode, count = 0, node = 0;

	for_each_online_node(node) {
	syscfg = xlp_get_power_on_reset_cfg(node);
	/* We don't manipulate pci_address space.
	 * Get the link status from pcie lane config from 34.9.7.2 XLP PRM */
	mode = (syscfg >> 8) & 0xf;
	while (count < 4) {
		printk(KERN_DEBUG "Controller %d is in %s mode\n",
				count, (mode & (1 << count)) ? "RC" : "EP");
		count++;
	}
	plc = (syscfg >> 12) & 0x3;
	printk(KERN_DEBUG "node %d, PLC = %#x, mode = %#x\n", node, plc, mode);

	switch (plc) {
	/* The correlation between plc and lane config is very specific to XLP
	 * and not very clear in PRM */
	case 0:
		/* controller 0 is active on x4 mode */
		if (mode & 0x1){
			xlp_map_helper(node, plc, 0);
			xlp_pcie_controller_setup(node, 0);
		}
		break;
	case 1:
		/* controllers 0 and 2 are active in x2 mode*/
		if (mode & 0x1){
			xlp_map_helper(node, plc, 0);
			xlp_pcie_controller_setup(node, 0);
		}
		if (mode & 0x4){
			xlp_map_helper(node, plc, 2);
			xlp_pcie_controller_setup(node, 2);
		}
		break;
	case 2 ... 3:
		/* controllers 0, 1, 2 and 3 are active in x1 mode*/
		if (mode & 0x1){
			xlp_map_helper(node, plc, 0);
			xlp_pcie_controller_setup(node, 0);
		}
		if (mode & 0x2){
			xlp_map_helper(node, plc, 1);
			xlp_pcie_controller_setup(node, 1);
		}
		if (mode & 0x4){
			xlp_map_helper(node, plc, 2);
			xlp_pcie_controller_setup(node, 2);
		}
		if (mode & 0x8){
			xlp_map_helper(node, plc, 3);
			xlp_pcie_controller_setup(node, 3);
		}
		break;
	}
	dump_node_irqmap(node, plc);
	}	/* for_each_online_node */
	return;
}

static void xlp_8xx_pcie_controller_init(void)
{
	u32 plc, syscfg, mode, count = 0, node = 0;

	for_each_online_node(node) {
	syscfg = xlp_get_power_on_reset_cfg(node);
	/* We don't manipulate pci_address space.
	 * Get the link status from pcie lane config from 34.9.7.2 XLP PRM */
	mode = (syscfg >> 19) & 0xf;
	while (count < 4) {
		printk(KERN_DEBUG "Controller %d is in %s mode\n",
				count, (mode & (1 << count)) ? "RC" : "EP");
		count++;
	}
	plc = (syscfg >> 23) & 0x3;
	printk(KERN_DEBUG "node %d, PLC = %#x, mode = %#x\n", node, plc, mode);

	switch (plc) {
	/* The correlation between plc and lane config is very specific to XLP
	 * and not very clear in PRM
	 */
	case 0:
		/* controller 0 and 2 are active with 8lanes each */
		if (mode & 0x1){
			xlp_map_helper(node, plc, 0);
			xlp_pcie_controller_setup(node, 0);
		}
		if (mode & 0x4) {
			xlp_map_helper(node, plc, 2);
			xlp_pcie_controller_setup(node, 2);
		}
		break;
	case 1:
		/* controllers 0,1 and 2 are active */
		if (mode & 0x1){
			xlp_map_helper(node, plc, 0);
			xlp_pcie_controller_setup(node, 0);
		}
		if (mode & 0x2){
			xlp_map_helper(node, plc, 1);
			xlp_pcie_controller_setup(node, 1);
		}
		if (mode & 0x4){
			xlp_map_helper(node, plc, 2);
			xlp_pcie_controller_setup(node, 2);
		}
		break;
	case 2:
		/* controllers 0,2 and 3 are active */
		if (mode & 0x1){
			xlp_map_helper(node, plc, 0);
			xlp_pcie_controller_setup(node, 0);
		}
		if (mode & 0x4){
			xlp_map_helper(node, plc, 2);
			xlp_pcie_controller_setup(node, 2);
		}
		if (mode & 0x8){
			xlp_map_helper(node, plc, 3);
			xlp_pcie_controller_setup(node, 3);
		}
		break;
	case 3:
		/* All four controllers are active with 4 lanes each */
		if (mode & 0x1){
			xlp_map_helper(node, plc, 0);
			xlp_pcie_controller_setup(node, 0);
		}
		if (mode & 0x2){
			xlp_map_helper(node, plc, 1);
			xlp_pcie_controller_setup(node, 1);
		}
		if (mode & 0x4){
			xlp_map_helper(node, plc, 2);
			xlp_pcie_controller_setup(node, 2);
		}
		if (mode & 0x8){
			xlp_map_helper(node, plc, 3);
			xlp_pcie_controller_setup(node, 3);
		}
		break;
	}
	dump_node_irqmap(node, plc);
	}	/* for_each_online_node */
	return;
}

static void xlp_3xx_pcie_controller_init(u32 pid)
{
	struct xlp_plc_fn_struct *nirqmap = &node_irqmap[0][0];
	/* zero out node_irqmap */
	memset(node_irqmap, 0, sizeof(struct xlp_plc_fn_struct) * NLM_MAX_CPU_NODE * XLP_PCI_LANE_CONFIG);
	switch(pid) {
		case CPU_EXTPID_XLP_3XX_L:
			/* Single node, port 0 only in x4 lane mode*/
			nirqmap->plc = 0;
			nirqmap->farray[0].intno = XLP_PCIE_LINK_IRQ(0,0);
			xlp_map_helper(0, 0, 0);
			xlp_pcie_controller_setup(0, 0);
			break;
		case CPU_EXTPID_XLP_3XX_LP:
			/* Port 0 and port 1 are active in x2 lane mode*/
			nirqmap->plc = 0;
			nirqmap->farray[0].intno = XLP_PCIE_LINK_IRQ(0,0);
			nirqmap->farray[1].intno = XLP_PCIE_LINK_IRQ(0,1);
			xlp_map_helper(0, 0, 0);
			xlp_pcie_controller_setup(0, 0);
			xlp_map_helper(0, 0, 1);
			xlp_pcie_controller_setup(0, 1);
			break;
		case CPU_EXTPID_XLP_3XX_LP2:
			/* All ports (0, 1, 2, 3) are active in x1 lane mode */
			nirqmap->plc = 0;
			nirqmap->farray[0].intno = XLP_PCIE_LINK_IRQ(0,0);
			nirqmap->farray[1].intno = XLP_PCIE_LINK_IRQ(0,1);
			nirqmap->farray[2].intno = XLP_PCIE_LINK_IRQ(0,2);
			nirqmap->farray[3].intno = XLP_PCIE_LINK_IRQ(0,3);
			xlp_map_helper(0, 0, 0);
			xlp_pcie_controller_setup(0, 0);
			xlp_map_helper(0, 0, 1);
			xlp_pcie_controller_setup(0, 1);
			xlp_map_helper(0, 0, 2);
			xlp_pcie_controller_setup(0, 2);
			xlp_map_helper(0, 0, 3);
			xlp_pcie_controller_setup(0, 3);
			break;
		default:
			printk(KERN_WARNING "Could not find a PCI interrupt allocation scheme\n");
			break;
	}
	dump_node_irqmap(0, 0);
}

extern int nlm_hal_get_cpuinfo(struct nlm_netl_proc_info *);

/*
 * Called from system startup routine
 */
static void pcie_controller_init_done(void)
{
#ifndef CONFIG_XLP_MSI_ADDRESSES
	/* The controller will never read from /write to this area.
	 * Strictly speaking this allocation is not necessary.
	 * But if we don't allocate, we will have to keep in different
	 * processors/boards a range of address which is not used anywhere else
	 * like physical mem, pci mem ..etc. It is just easier to allocate
	 * some memory and not use it for anything else.  */
#ifdef CONFIG_32BIT
	XLP_MSI_ADDR = (u64)__get_free_pages(GFP_KERNEL, get_order(0x100000));
#else
	/* We don't need 16M, all we need is 2 pages per controller
	 * => 2 * 4ctr * 4 nodes pages */
	XLP_MSI_ADDR = (u64)__get_free_pages(GFP_KERNEL,
			get_order(0x1000000));
#endif
	if (XLP_MSI_ADDR == 0) {
		printk(KERN_ERR "Failed to get memory for MSI/MSI-X tables\n");
	}
#endif
	if (!pci_probe_only){
		printk(KERN_WARNING "PCIe bus IRQs configured incorrectly\n");
		return;
	}
	/* Configure controllers for running cpu type */
	if (is_nlm_xlp8xx()) {
		printk(KERN_DEBUG "Initializing PCIe for XLP8xx/4xx\n");
		xlp_8xx_pcie_controller_init();
	} else if (is_nlm_xlp2xx()) {
		printk(KERN_DEBUG "Initializing PCIe for XLP2xx/1xx\n");
		xlp_2xx_pcie_controller_init();
	} else if (is_nlm_xlp3xx()) {
		uint32_t epid = efuse_cfg0() >> 4 & 0xf;
		if(epid) {
			printk(KERN_DEBUG "Initializing PCIe for XLP3xxL\n");
			xlp_3xx_pcie_controller_init(epid);
		} else {
			printk(KERN_DEBUG "Initializing PCIe for XLP3xx\n");
			/* ordinary 3xx and 8xx has same controller init seq. */
			xlp_8xx_pcie_controller_init();
		}
	} else {
		panic("Can't configure PCIe controller for unknown XLP CPU type\n");
	}
}

static inline __u32 pci_cfg_read_32bit(__u32 addr)
{
	__u32 temp = 0;
    __u32 *p = (__u32 *) (xlp_pci_config_base + (addr & ~3));

	temp = *p;

	return temp;
}

static inline __u16 pci_cfg_read_16bit(__u32 addr)
{
    return *((__u16*)(xlp_pci_config_base + (addr & ~1)));
}

static inline __u8 pci_cfg_read_8bit(__u32 addr)
{
    return *((__u8 *)(xlp_pci_config_base + (addr & ~0)));
}

static inline void pci_cfg_write_32bit(__u32 addr, __u32 data)
{
        __u32 *p = (__u32 *)(xlp_pci_config_base + (addr & ~3));

	*p = data;
}

static inline void pci_cfg_write_16bit(__u32 addr, __u16 data)
{
    *((__u16*)(xlp_pci_config_base + (addr & ~1))) = data;
}

static inline void pci_cfg_write_8bit(__u32 addr, __u8 data)
{
    *((__u8 *)(xlp_pci_config_base + (addr & ~0))) = data;
}

static int pci_bus_status = 0;
//#define pci_cfg_offset(bus, devfn, where) (((bus)<<16)+((devfn)<<8)+(where))  //for PCI config space
#define pci_cfg_offset(bus, devfn, where) (((bus)<<20)+((devfn)<<12)+(where))	//for PCIE config space
#define pci_cfg_addr(bus, devfn, where) pci_cfg_offset((bus)->number,(devfn),where)

static int xlp_pcibios_read(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 * val)
{
    if( (!bus->self) || (bus->self->vendor == NETL_VENDOR_ID)) {
        __u32 data = 0;

        if ((size == 2) && (where & 1))
            return PCIBIOS_BAD_REGISTER_NUMBER;
        else if ((size == 4) && (where & 3))
            return PCIBIOS_BAD_REGISTER_NUMBER;

        if (pci_bus_status)
            data = pci_cfg_read_32bit(pci_cfg_offset((bus->number), devfn, where));
        else
            data = 0xFFFFFFFF;

        if (size == 1)
            *val = (data >> ((where & 3) << 3)) & 0xff;
        else if (size == 2)
            *val = (data >> ((where & 3) << 3)) & 0xffff;
        else
            *val = data;
    } else { /* other vendors */
        if      (size == 1)
            *val = pci_cfg_read_8bit (pci_cfg_offset((bus->number), devfn, where));
        else if (size == 2)
            *val = pci_cfg_read_16bit(pci_cfg_offset((bus->number), devfn, where));
        else  /*(size == 4) */
            *val = pci_cfg_read_32bit(pci_cfg_offset((bus->number), devfn, where));
    }
    return PCIBIOS_SUCCESSFUL;
}

static int xlp_pcibios_write(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	__u32 cfgaddr = pci_cfg_offset((bus->number), devfn, where);
	__u32 data = 0;

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (!pci_bus_status)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if( (!bus->self) || (bus->self->vendor == NETL_VENDOR_ID)) {
		data = pci_cfg_read_32bit(cfgaddr);

		if (size == 1)
			data = (data & ~(0xff << ((where & 3) << 3))) |
				    ((val & 0xff) << ((where & 3) << 3));
		else if (size == 2)
			data = (data & ~(0xffff << ((where & 3) << 3))) |
				    ((val & 0xffff) << ((where & 3) << 3));
		else
			data = val;

		pci_cfg_write_32bit(cfgaddr, data);
	} else { /* every other vendor can have byte, word or double word access */
		if      (size == 1)
			pci_cfg_write_8bit ( cfgaddr, (__u8 )val);
		else if (size == 2)
			pci_cfg_write_16bit( cfgaddr, (__u16)val);
		else /* (size == 4) */
			pci_cfg_write_32bit( cfgaddr,        val);
	}

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops xlp_pci_ops = {
	.read  = xlp_pcibios_read,
	.write = xlp_pcibios_write
};

/*
 * XLP PCIE Controller
 */
static struct resource xlp_pcie_cfg_resource = {
	.name			= "XLP PCI-E config",
	.start			= XLP_PCI_ECONFIG_BASE,
	.end			= XLP_PCI_ECONFIG_BASE + XLP_PCI_ECONFIG_SIZE - 1,
	.flags			= IORESOURCE_MEM
};

static struct resource xlp_mem_resource = {
	.name           = "XLP PCI MEM",
	.start          = 0xd0000000ULL,	// 256MB PCI mem @ 0xd000_0000
	.end            = 0xdfffffffULL,
	.flags          = IORESOURCE_MEM
};

static struct resource xlp_io_resource = {
	.name           = "XLP IO MEM",
	.start          = 0x14000000UL,		// 32MB PCI IO @ 0x1400_0000
	.end            = 0x15ffffffUL,
	.flags          = IORESOURCE_IO
};

struct pci_controller xlp_controller = {
	.index          = 0,
	.pci_ops        = &xlp_pci_ops,
	.mem_resource   = &xlp_mem_resource,
	.io_resource    = &xlp_io_resource,
	.io_offset      = 0x00000000UL,
	.mem_offset     = 0x00000000UL
};

/*
 * This function is called for all pci controller functions
 * viz. 0:1.0, 0:1.1, 0:1.2 and 0:1.3, 0:9.X, 0:17.X and 0:25.X
 * In fact, we need not assign them any interrupt because they are not
 * interrupt generating devices.
 * But for any devices connected on these controllers, consult the populated
 * table and return corresponding interrupt.
 */
int __init xlp_8xx_pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int row = 0;
	struct xlp_nodefn_struct nfn;

	if (xlp_ctrl_fn_from_dev(dev, &nfn) != 0) {
		dev_printk(KERN_ERR, &dev->dev, "Could not resolve device to a node/bus pair\n");
		return 0;
	}
	if (is_nlm_xlp8xx()) {
		row = (xlp_get_power_on_reset_cfg(nfn.node) >> 23) & 0x3;
	} else {
		row = (xlp_get_power_on_reset_cfg(nfn.node) >> 12) & 0x3;
	}
	dev_printk(KERN_DEBUG, &dev->dev, "Assigning interrupt %#x\n", node_irqmap[nfn.node][row].farray[nfn.fn].intno);
	return node_irqmap[nfn.node][row].farray[nfn.fn].intno;
}

int __init xlp_3xx_pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int row = 0;
	struct xlp_nodefn_struct nfn;

	if (xlp_ctrl_fn_from_dev(dev, &nfn) != 0) {
		dev_printk(KERN_ERR, &dev->dev, "Could not resolve device to a node/bus pair\n");
		return 0;
	}
	dev_printk(KERN_DEBUG, &dev->dev, "Assigning interrupt %#x\n", node_irqmap[nfn.node][row].farray[nfn.fn].intno);
	return node_irqmap[nfn.node][row].farray[nfn.fn].intno;

}

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	if (dev->bus->number == 0) {
		return 0;
	}
	if (is_nlm_xlp8xx() || is_nlm_xlp2xx()) {
		return xlp_8xx_pcibios_map_irq(dev, slot, pin);
	} else if (is_nlm_xlp3xx()) {
		return xlp_3xx_pcibios_map_irq(dev, slot, pin);
	}
	panic("Can't map IRQ for unknown CPU type\n");
}

/*
 * Enables INTx on a controller
 */
static int __xlp_intx_enable(u8 node, int fn)
{
	u32 pci;

	pci = nlh_pci_r32r(node, fn,  0x1);
	pci &= ~(1 << 10);	/* Enable IntX assertion */
	nlh_pci_w32r(node, fn, 0x1, pci);
	pci = nlh_pci_r32r(node, fn,  0x261);
	pci |= 0xf;	/* Enable INT A,B,C,D */
	nlh_pci_w32r(node, fn, 0x261, pci);
	return 0;
}

int xlp_intx_enable(u8 node, int fn)
{
	int mode = xlp_get_ctrl_intmode(node, fn);

	if ((mode & XLP_INTMODE_MSI) || (mode & XLP_INTMODE_MSIX)) {
		return -EBUSY;
	}
	__xlp_intx_enable(node, fn);
	xlp_incr_ctrl_intmode(node, fn, XLP_INTMODE_INTX);
	return 0;
}

/*
 * Disables INTx on a controller
 */
static int __xlp_intx_disable(u8 node, int fn)
{
	u32 pci;

	pci = nlh_pci_r32r(node, fn,  0x1);
	pci |= (1 << 10);
	nlh_pci_w32r(node, fn, 0x1, pci);
	pci = nlh_pci_r32r(node, fn,  0x261);
	pci &= ~(0xf);
	nlh_pci_w32r(node, fn, 0x261, pci);
	return 0;
}

int xlp_intx_disable(u8 node, int fn)
{
	int mode = xlp_get_ctrl_intmode(node, fn);

	if (!(mode & XLP_INTMODE_INTX)) {
		return -EBUSY;
	}
	__xlp_intx_disable(node, fn);
	xlp_decr_ctrl_intmode(node, fn, XLP_INTMODE_INTX);
	return 0;
}

/*
 * Finds the slot on which this device is placed and enables corresponding
 * MSI enable register on the _controller_ if not already enabled
 * @dev : pci device corresponding to this device
 */
static int __xlp_msi_enable(u8 node, int fn, u32 bit)
{
	u32 msi_en;

	/* First, set PCIe MSI Enable register. __KEEP_THIS_ORDER__ */
	msi_en = nlh_pci_r32r(node, fn,  0x261);
	msi_en &= ~(0xf);
	if ((msi_en & (1 << 9)) == 0) {
		msi_en |= (1 << 9);	/* controls ONLY MSI, Not MSI-X */
		nlh_pci_w32r(node, fn, 0x261, msi_en);
	}
	/* Now, set the individual bit */
	xlp_msi_set_mask(node, fn, bit, 1);
	return 0;
}

int xlp_msi_enable(u8 node, int fn, u32 bit)
{
	int tmp = xlp_get_ctrl_intmode(node, fn);

	if ((tmp & XLP_INTMODE_INTX) || (tmp & XLP_INTMODE_MSIX)) {
		return -EBUSY;
	}

	/* Enable MSI bis
	 * Multiple MSI can get enabled at different point of time (especially
	 * with a switch present. So, setting the bitmap should not depend on
	 * present value of reg 0x25b or 0x261
	 */
	__xlp_msi_enable(node, fn, bit);
	xlp_incr_ctrl_intmode(node, fn, XLP_INTMODE_MSI);
	return 0;
}

/*
 * Finds the slot on which this device is placed and enables corresponding
 * MSI-X enable register on the controller
 */
static int __xlp_msix_enable(u8 node, int fn)
{
	u32 msix_ctrl;

	msix_ctrl = nlh_pci_r32r(node, fn,  0x2C);
	if (!(msix_ctrl & 0x80000000)) {
		msix_ctrl |= 0x80000000;	/* MSI-X enable */
		nlh_pci_w32r(node, fn, 0x2C, msix_ctrl);
	}
	//nlh_pci_w32r(node, fn, 0xf, 0xFF);
	return 0;
}

int xlp_msix_enable(u8 node, int fn)
{
	int mode = xlp_get_ctrl_intmode(node, fn);

	if ((mode & XLP_INTMODE_MSI) || (mode & XLP_INTMODE_INTX)) {
		return -EBUSY;
	}
	__xlp_msix_enable(node, fn);
	xlp_incr_ctrl_intmode(node, fn, XLP_INTMODE_MSIX);
	return 0;
}

/*
 * Disables MSI on controller function
 */
static int __xlp_msi_disable(u8 node, int fn)
{
	u32 msi_en;

	/* We dont call xlp_decr_ctrl.... here because it has already been 
	 * called before xlp_msi_disable is called */

	/*set PCIe Int Enable register */
	msi_en = nlh_pci_r32r(node, fn,  0x261);
	if ((msi_en & (1 << 9)) != 0) {
		msi_en &= ~(1 << 9);
		msi_en |= 0xf;
		nlh_pci_w32r(node, fn, 0x261, msi_en);
	}
	return 0;
}

int xlp_msi_disable(u8 node, int fn, u32 bit)
{
	int tmp = xlp_get_ctrl_intmode(node, fn);
	u32 r25b;

	if (!(tmp & XLP_INTMODE_MSI)) {
		return -EBUSY;
	}
	r25b = xlp_msi_set_mask(node, fn, bit, 0);
	if (r25b == 0) {
		__xlp_msi_disable(node, fn);
	}
	xlp_decr_ctrl_intmode(node, fn, XLP_INTMODE_MSI);
	return 0;
}

/*
 * Disables MSI-X on a controller function
 */
static int __xlp_msix_disable(u8 node, int fn)
{
	u32 msix_ctrl;

	msix_ctrl = nlh_pci_r32r(node, fn,  0x2C);
	msix_ctrl &= ~(0x80000000);	/* MSI-X disable */
	nlh_pci_w32r(node, fn, 0x2C, msix_ctrl);
	//nlh_pci_w32r(node, fn, 0xf, 0xFF);	/* TODO Get from dev */
	return 0;
}

int xlp_msix_disable(u8 node, int fn)
{
	int mode = xlp_get_ctrl_intmode(node, fn);

	if (!(mode & XLP_INTMODE_MSIX)) {
		return -EBUSY;
	}
	if (xlp_decr_ctrl_intmode(node, fn, XLP_INTMODE_MSIX) == 0) {
		__xlp_msix_disable(node, fn);
	}
	return 0;
}

/*
 * checks if msi is enabled for this controller
 * @fn	: controller function number
 */
int is_msi_set(u8 node, int fn)
{
	u32 msi_en, status;

	msi_en = nlh_pci_r32r(node, fn,  0x261);
	status = (msi_en >> 9) & 1 ;
	return status;
}


u32 calc_msi_vector_offset(u8 node, int fn)
{
	u32 msi_en, msi_stat;

	msi_en = nlh_pci_r32r(node, fn,  0x25B);
	msi_stat = nlh_pci_r32r(node, fn,  0x25A);
	nlh_pci_w32r(node, fn, 0x25A, msi_stat);
	msi_stat &= msi_en;
	return msi_stat;
}

/*
 * Clears MSI-X status bits for a controller
 * @fn : controller number
 *
 * status register is Read, Write 1 to clear.
 * Figure out the mask (the bits corresponding to fn), read register, clear
 * them and return the bits corresponding to fn
 */
u32 xlp_msix_status_clear(u8 node, int fn)
{
	u32 msix_stat;
	u32 mask = ((XLP_MSIX_PER_SLOT - 1) << (fn * XLP_MSIX_PER_SLOT));

	msix_stat = nlh_pci_r32r(node, fn,  0x25D);
	msix_stat &= mask;
	nlh_pci_w32r(node, fn, 0x25D, msix_stat);
	return (msix_stat >> (fn * XLP_MSIX_PER_SLOT));
}

/*
 * Masks the bit corresponding to an MSI and return the resulting bitmask
 */
u32 xlp_msi_set_mask(u8 node, int fn, int bit, int val)
{
	u32 bits;

	bits = nlh_pci_r32r(node, fn,  0x25B);
	if (val == 0) {	/* Clear bit `bit` */
		bits &= ~( 1 << bit);
	} else {	/* Set bit `bit` */
		bits |= ( 1 << bit);
	}
	nlh_pci_w32r(node, fn, 0x25B, bits);
	return bits;
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
        return 0;
}

/* Enabled by default */
static int __initdata xlp_nopci = 0;

static int __init xlp_nopci_setup(char *str)
{
	/* Disable PCI/X/E; disables HT also */
	xlp_nopci = 1;

	return 1;
}
__setup("xlp_nopci", xlp_nopci_setup);

static int __init pcibios_init(void)
{
	unsigned long phys = 0;
	unsigned long size = 0;
	int ret;

	if (xlp_nopci) return 0;

	/* Bootloader assigns PCI resources */
	pci_probe_only = 1;

	/* Map the PCI-e CFG space */
	ret = request_resource(&iomem_resource, &xlp_pcie_cfg_resource);
	xlp_pci_config_base = ioremap_nocache(XLP_PCI_ECONFIG_BASE, XLP_PCI_ECONFIG_SIZE);
	if (!xlp_pci_config_base) {
		printk(KERN_ERR "%s: Unable to map PCI-E config space!\n", __func__);
		return 1;
	}
	printk(KERN_DEBUG "%s: PCI-E config phys=0x%08x, size=0x%08x, vaddr=0x%p\n",
			__func__,
			(unsigned)XLP_PCI_ECONFIG_BASE, (unsigned)XLP_PCI_ECONFIG_SIZE,
			xlp_pci_config_base);

	phys = xlp_io_resource.start;
	size = xlp_io_resource.end - xlp_io_resource.start + 1;

	pci_io_base = ioremap(phys, size);
	if (!pci_io_base) {
		printk(KERN_WARNING "%s: Unable to IO-Remap phys=0x%08x, size=0x%08x\n",
		       __FUNCTION__, (unsigned)phys, (unsigned)size);
		/* Eventually this is going to panic() */
	}
	else {
		printk(KERN_DEBUG "%s: PCI I/O area phys=0x%08x, size=0x%08x, vaddr=0x%p\n",
		       __FUNCTION__, (unsigned)phys, (unsigned)size, pci_io_base);
	}
	set_io_port_base((unsigned long) pci_io_base);
	xlp_controller.io_map_base = (unsigned long) pci_io_base;
	xlp_controller.io_map_base -= xlp_controller.io_offset;

	/* IO Range for 16MB from where the MEM Range Ends */
	ioport_resource.start =  0;
	ioport_resource.end   = ~0;

	printk(KERN_DEBUG "Registering XLP PCIE Controller\n");
	/* Setting up controller specific data */
	pcie_controller_init_done();
	register_pci_controller(&xlp_controller);

	pci_bus_status = 1;
	return 0;
}

arch_initcall(pcibios_init);

struct pci_fixup pcibios_fixups[] = {
	{0}
};


/*
 * some ide specific io routines on PCI
 */
#define pci_ide_phys_to_virt(x) (((x) - (xlp_io_resource.start)) + (unsigned long)pci_io_base )

inline void nlm_ide_mm_insw(unsigned long port, void *addr, u32 count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		*(u16 *)addr = (readw((const volatile void *)v_port));
		addr += 2;
	}
}

EXPORT_SYMBOL(nlm_ide_mm_insw);

inline void nlm_ide_mm_insl(unsigned long port, void *addr, unsigned int count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		*(u32 *)addr = readl((const volatile void *) v_port);
		addr += 4;
	}
}
EXPORT_SYMBOL(nlm_ide_mm_insl);

inline void nlm_ide_mm_outsw(unsigned long port, void *addr, unsigned int count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		writew(*(u16 *)addr, (volatile void *)v_port);
		addr += 2;
	}
}
EXPORT_SYMBOL(nlm_ide_mm_outsw);

inline void nlm_ide_mm_outsl(unsigned long port, void *addr, unsigned int count)
{
	unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		writel(*(u32 *)addr, (volatile void *)v_port);
		addr += 4;
	}
}
EXPORT_SYMBOL(nlm_ide_mm_outsl);

u8 nlm_ide_mm_inb (unsigned long port)
{
	return((u8)readb((const volatile void *)pci_ide_phys_to_virt(port)));
}

EXPORT_SYMBOL(nlm_ide_mm_inb);
u16 nlm_ide_mm_inw (unsigned long port)
{
	return ((u16) (readw((const volatile void *)pci_ide_phys_to_virt(port))));
}

EXPORT_SYMBOL(nlm_ide_mm_inw);
/* Not part of hwif anymore; remove static declaration */
u32 nlm_ide_mm_inl (unsigned long port)
{
	return ((u32)readl((const volatile void *)pci_ide_phys_to_virt(port)));
}

EXPORT_SYMBOL(nlm_ide_mm_inl);
void nlm_ide_mm_outb (u8 value, unsigned long port)
{
	writeb(value, (volatile void *)pci_ide_phys_to_virt(port));
}

EXPORT_SYMBOL(nlm_ide_mm_outb);
void nlm_ide_mm_outw (u16 value, unsigned long port)
{
	writew(value, (volatile void *)pci_ide_phys_to_virt((u64)port));
}

EXPORT_SYMBOL(nlm_ide_mm_outw);
/* Not part of hwif anymore; remove static declaration */
void nlm_ide_mm_outl (u32 value, unsigned long port)
{
	writel((value), (volatile void *)pci_ide_phys_to_virt(port));
}
EXPORT_SYMBOL(nlm_ide_mm_outl);
