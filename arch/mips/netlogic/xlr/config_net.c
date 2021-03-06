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


/*
 * Setup code for Netlogic's XLR-based boards
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/config_net.h>
#include <asm/netlogic/xlr_mac.h>
#include <asm/netlogic/gpio.h>

struct net_device_cfg xlr_net_dev_cfg;
extern unsigned long netlogic_io_base;
/*
extern uint32_t dev_tree_en;
extern void nlm_dev_config_net(void);
*/
static uint32_t gmac_offsets[] = { NETLOGIC_IO_GMAC_0_OFFSET, NETLOGIC_IO_GMAC_1_OFFSET, 
			NETLOGIC_IO_GMAC_2_OFFSET, NETLOGIC_IO_GMAC_3_OFFSET,
			NETLOGIC_IO_GMAC_4_OFFSET, NETLOGIC_IO_GMAC_5_OFFSET,
			NETLOGIC_IO_GMAC_6_OFFSET, NETLOGIC_IO_GMAC_7_OFFSET };

#if !defined(XLP_SIM)
static uint32_t gmac_irqs[] = { PIC_GMAC_0_IRQ, PIC_GMAC_1_IRQ, 
			PIC_GMAC_2_IRQ, PIC_GMAC_3_IRQ,
			PIC_GMAC_4_IRQ, PIC_GMAC_5_IRQ,
			PIC_GMAC_6_IRQ, PIC_GMAC_7_IRQ };

static uint32_t xgmac_offsets[] = { NETLOGIC_IO_XGMAC_0_OFFSET, NETLOGIC_IO_XGMAC_1_OFFSET };
static uint32_t spi4_offsets[] = { NETLOGIC_IO_SPI4_0_OFFSET, NETLOGIC_IO_SPI4_1_OFFSET };
static uint32_t xgs_irqs[] = { PIC_XGS_0_IRQ, PIC_XGS_1_IRQ };
#endif /* XLP_SIM */

#define MAX_NUM_DESC		512
#define NLM_BASE(x) (netlogic_io_base + x)

/*
   This functions returns:
   True (1): if block is in XAUI mode
   False(0): if block is in GMAC mode
*/

int xlsb0_in_xaui(int block)
{
    unsigned int gpio_xaui = 0;
    nlm_reg_t *gpio_mmio =
                    (unsigned int *)(netlogic_io_base + NETLOGIC_IO_GPIO_OFFSET);
    if (xlr_board_atx_xi() || xlr_board_atx_xii()) {
        gpio_xaui = ((netlogic_read_reg(gpio_mmio,21) >> 24) & 0x3);
        switch(gpio_xaui){
            case 0x1:
                return block==0?1:0;
            case 0x2:
                 return block==1?1:0;
            case 0x3:
                 return 1;
            default:
                return 0;
        }
    }
    return 0;
}

static int sgmii_daughter_card_present(int block)
{
	unsigned long cpld_base = (unsigned long)(NETLOGIC_CPLD_OFFSET);
	unsigned char *mmio = (unsigned char*)cpld_base;
	unsigned char value = mmio[0x0d];
	value = value & 0x03;

	switch (block)
	{
		case 0:
			if ((value == 0x0) || (value == 0x1))
				return 1;
			break;
		case 1:
			if ((value == 0x0) || (value == 0x2))
				return 1;
			break;
		default:
			return 0;
	}
	return 0;
}

/* This arrray is indexed with the processor id 8bits */
char nlm_chip_gmac_count[256];
void init_gmac_ports(void)
{
	int processor_id;

    processor_id = ((read_c0_prid() & 0xff00) >> 8);

    /* Currently handle for XLS B0 parts... */
    switch(processor_id) {
        case CHIP_PROCESSOR_ID_XLS_616_B0:
        case CHIP_PROCESSOR_ID_XLS_608_B0:
        case CHIP_PROCESSOR_ID_XLS_416_B0:
        case CHIP_PROCESSOR_ID_XLS_412_B0:
        case CHIP_PROCESSOR_ID_XLS_408_B0:
        case CHIP_PROCESSOR_ID_XLS_404_B0:
            nlm_chip_gmac_count[processor_id] = 8;
            break;

        default:
            break;
    }
}
int xlr_is_mac_active(int instance, int type, int *mode)
{
    uint32_t *gpio_base = (uint32_t *)(DEFAULT_NETLOGIC_IO_BASE +
                                    NETLOGIC_IO_GPIO_OFFSET);    
	int processor_id;
    int xaui_board = 0;


    if(xlr_board_atx_xi() || xlr_board_atx_xii())
        xaui_board = 1;

	*mode = PHY_MODE_RGMII;

	/* On XLS xgmac is not available */
	if (is_xls()) {
		if(type == TYPE_XGMAC || type == TYPE_SPI4)
                return 0;

		processor_id = ((read_c0_prid() & 0xff00) >> 8);

		*mode = PHY_MODE_SGMII;

		if(instance == 0) {
			/* Lite board does not have rgmii ifc */
			if(xlr_board_atx_viii())
				*mode = PHY_MODE_SGMII;
			/* atx-xi/xii boards: SGMII mode for gmac 0 only if 
			   daughter card for gmac block-0 is present.
			   */
			else if((xlr_board_atx_xi() || xlr_board_atx_xii()) &&
					(!sgmii_daughter_card_present(0)))
				*mode = PHY_MODE_RGMII;
			else
				*mode = PHY_MODE_RGMII | PHY_MODE_SELECTABLE;
		}

		if(is_xls_b0() && xaui_board){
			if(instance < 4){
                /* If port is not in XAUI mode, this board does not have SGMII.
                   Only port 0 is in RGMII mode
                   */
				if(xlsb0_in_xaui(0)) {
                    *mode = PHY_MODE_XAUI; /* else mode is set above */
                    printk("Port %d is in XAUI mode\n", instance);
                    if(instance == 0)
                        return 1;
                    else
                        return 0;
                } else
				{
					/* return TRUE if (instance == 0) OR
					   if daughter card for block0 is 
					   present 
					 */
					if ((instance == 0) || 
						sgmii_daughter_card_present(0))
						return 1;
					else
						return 0;
				}
			}
			else if(instance < 8){
                /* If port is not in XAUI mode, this board does not have SGMII*/
				if(xlsb0_in_xaui(1)) {
                    *mode = PHY_MODE_XAUI;
                    printk("Port %d is in XAUI mode\n", instance);
                    if(instance == 4)
                        return 1;
                    else
                        return 0;
                } else 
				{
					/* return TRUE only if daughter card 
					   for block 1 is present */
					if (sgmii_daughter_card_present(1))
						return 1;
					else
						return 0;
				}
			}else
				return 0;
		}

		/* all XLS parts have gmac0, gmac1 */
		if (instance < 2)
				return 1;

		if (processor_id <= CHIP_PROCESSOR_ID_XLS_104) {
			/* all XLS parts with processor_id <=104, have gmac3 */
			if (instance < 3) 	
				return 1;
		}
		if (processor_id <= CHIP_PROCESSOR_ID_XLS_204) {
			/* all XLS parts with processor_id <=204, have gmac4 */
			if (instance < 4) 	
				return 1;
		}

		if ((processor_id >= CHIP_PROCESSOR_ID_XLS_608) &&
		    (processor_id < CHIP_PROCESSOR_ID_XLS_208)) {
			if (instance < 6) 
				return 1;

			if(((gpio_base[NETLOGIC_GPIO_FUSE_BANK_REG] & (1<<28)) == 0)  &&
    	            ((gpio_base[NETLOGIC_GPIO_FUSE_BANK_REG] & (1<<29)) ==  0)){
				/*Below bits are set when ports are disabled.
				28 - GMAC7
				29 - GMAC6
				30 - GMAC5
				31 - GMAC4
				*/
				/*We found an XLS-408 with 8 gmacs*/
				if (instance < 8) 
					return 1;
			}
		}
		if (processor_id == CHIP_PROCESSOR_ID_XLS_608) {
			if (instance == 6 || instance == 7) 
				return 1;
		}

        if(nlm_chip_gmac_count[processor_id] &&
                (instance < nlm_chip_gmac_count[processor_id]))
            return 1;

		/* should never come here */
		return 0;
	}
	
	if (type == TYPE_GMAC) {
		/* On XLR gmac4 to gmac7 are unavailable */
		if(instance >= 4)
			return 0;

		/* On ATX-II, gmac 0 and gmac 1 are not available */
		if (xlr_board_atx_ii() && !xlr_board_atx_ii_b()) {
			if(instance < 2)
				return 0;
		}

		/* On ATX-IV-B and ATX-V, gmac 3 is not available */
		if ((xlr_board_atx_v() || xlr_board_atx_iv_b())) {
			if(instance > 2)
				return 0;
		}

		return 1;

	} else if(type == TYPE_XGMAC) {
		/* On ATX-II and ATX IIB 2 xgmac is  available */
		if (xlr_board_atx_ii() || xlr_board_atx_ii_b())
			return 1;
		return 0;

	}  else if(type == TYPE_SPI4) {
		if(xlr_board_atx_i())
			return 1;
		return 0;
	}
	
	return 0;
}

int xlr_get_phy_info(int instance, int mode, unsigned long *mii_addr, 
					unsigned long *pcs_addr, unsigned long *serdes_addr)
{
	uint32_t phy_addr;

	*pcs_addr = 0x0;
	*serdes_addr = 0x0;
	*mii_addr = NLM_BASE(gmac_offsets[0]);

	if(is_xls()) {
		if(instance < NETLOGIC_GMAC_PORTS_PER_CTRL) {
			*pcs_addr = NLM_BASE(gmac_offsets[0]);
		} else  {
			*pcs_addr = NLM_BASE(gmac_offsets[NETLOGIC_GMAC_PORTS_PER_CTRL]);
		}
		*serdes_addr = NLM_BASE(gmac_offsets[0]);

		if(mode & PHY_MODE_RGMII) {
			phy_addr = 0 + instance;
			/*only atx-vi has rgmii-0 linked to sgmii-4 offset*/
			if(xlr_board_atx_vi())
				*mii_addr = NLM_BASE(gmac_offsets[NETLOGIC_GMAC_PORTS_PER_CTRL]);		
		} else
			phy_addr = 0x10 + instance;

		/* boards 11 / 12 may have SGMII ports due to daughter
		   cards.  In this case, the phy for block-0 is same as 
		   gmac[0], but phy for block-1 is taken from gmac[4].
		   Hence, update phy values if gmac instance >= 4.
		   */
		if ((xlr_board_atx_xi() || xlr_board_atx_xii()) &&
				(instance >= NETLOGIC_GMAC_PORTS_PER_CTRL))
		{
			*mii_addr = NLM_BASE(gmac_offsets[NETLOGIC_GMAC_PORTS_PER_CTRL]);
			phy_addr -= NETLOGIC_GMAC_PORTS_PER_CTRL;
		}
	} else {
		if (xlr_board_atx_ii() && !xlr_board_atx_ii_b()) {
			if(instance < 2)
					phy_addr =  0;
			phy_addr = instance - 2;
		}
		phy_addr = 0 + instance;
	}
	return phy_addr;
}

void config_net_init(void)
{
#if !defined(XLP_SIM)
	struct net_device_cfg *net_dev = &xlr_net_dev_cfg;
	int i, mode, gmac_pblock = 0;
	int num_desc = MAX_NUM_DESC;

    init_gmac_ports();
	for(i = 0; i < NETLOGIC_MAX_GMACS; i++)  {
		/* general config for gmac */
		net_dev->gmac_port[i].instance = i;
		net_dev->gmac_port[i].irqno = gmac_irqs[i];
		net_dev->gmac_port[i].config_pde = 1;
		/* chip specific config for gmac */
		if(xlr_is_mac_active(i, TYPE_GMAC, &mode) == 1) {
			net_dev->gmac_port[i].mmio_addr = NLM_BASE(gmac_offsets[i]);
			net_dev->gmac_port[i].cfg_flag 	= PORT_OWN_LINUX;
			
			if(xlr_board_atx_vii() && i==4)
				/*atx-vii board workaround for mdio-1*/
				net_dev->gmac_port[i].cfg_flag 	= PORT_OWN_LINUX & ~(NLM_INT_ATTACH);

			if(i >= gmac_pblock) {
				net_dev->gmac_port[i].num_desc = num_desc;
				if(is_xls()) {
					if(i < NETLOGIC_GMAC_PORTS_PER_CTRL) {
						net_dev->gmac_port[i].bucket = &xls_bucket_sizes.bucket[MSGRNG_STNID_GMAC0];
						net_dev->gmac_port[i].credit = &xls_cc_table_gmac0;
					} else {
						net_dev->gmac_port[i].bucket = &xls_bucket_sizes.bucket[MSGRNG_STNID_GMAC1];
						net_dev->gmac_port[i].credit = &xls_cc_table_gmac1;
					}
				} else {
					net_dev->gmac_port[i].bucket = &bucket_sizes.bucket[MSGRNG_STNID_GMAC];
					net_dev->gmac_port[i].credit = &cc_table_gmac;
				}
				gmac_pblock += NETLOGIC_GMAC_PORTS_PER_CTRL;
			}
 
			net_dev->gmac_port[i].phy_mode = mode;

			net_dev->gmac_port[i].phy_addr = xlr_get_phy_info(i, 
				net_dev->gmac_port[i].phy_mode, 
				&net_dev->gmac_port[i].mii_addr, 
				&net_dev->gmac_port[i].pcs_addr, 
				&net_dev->gmac_port[i].serdes_addr);
			
		}
	}

	/* general config for xgmac */
	for(i = 0; i < NETLOGIC_MAX_XGMACS; i++)  {
		net_dev->xgs_port[i].instance = i;
		net_dev->xgs_port[i].irqno = xgs_irqs[i];
		net_dev->xgs_port[i].config_pde = 1;

		if(xlr_is_mac_active(i, TYPE_XGMAC, &mode) == 1) {
			net_dev->xgs_port[i].mmio_addr = NLM_BASE(xgmac_offsets[i]);
			net_dev->xgs_port[i].cfg_flag 	= PORT_OWN_LINUX;
			net_dev->xgs_port[i].num_desc = num_desc;
			net_dev->xgs_type[i] = TYPE_XGMAC;

		} else 	if(xlr_is_mac_active(i, TYPE_SPI4, &mode) == 1) {
			net_dev->xgs_port[i].mmio_addr = NLM_BASE(spi4_offsets[i]);
			net_dev->xgs_port[i].cfg_flag 	= PORT_OWN_LINUX;
			net_dev->xgs_port[i].num_desc = num_desc;
			net_dev->xgs_type[i] = TYPE_SPI4;
		}
		/* as descriptors are discontinues we need to pass the 
           full list */
		net_dev->xgs_port[i].bucket = &bucket_sizes.bucket[0];
		if(i == 0)
			net_dev->xgs_port[i].credit = &cc_table_xgs_0;
		else
			net_dev->xgs_port[i].credit = &cc_table_xgs_1;
	}

	/* Modify the basic configurations with the options 
			supported in Linux */
	/* usermac support */
	if(xlr_hybrid_user_mac()) {
		for(i = 0; i < NETLOGIC_MAX_GMACS; i++)  {
			if(net_dev->gmac_port[i].mmio_addr == 0)
				continue;
			net_dev->gmac_port[i].cfg_flag     = NLM_PORT_INIT;
		}
	}
	if(xlr_hybrid_user_mac() || xlr_hybrid_user_mac_xgmac()) {
		for(i = 0; i < NETLOGIC_MAX_XGMACS; i++)  {
			if(net_dev->xgs_port[i].mmio_addr == 0)
				continue;
			net_dev->xgs_port[i].cfg_flag = NLM_PORT_INIT;
		}
	}

	if(xlr_hybrid_rmios_ipsec()) {
		/* port should be enabled by rmios apps
          after configuring the descriptors */
		for(i = 0; i < NETLOGIC_MAX_GMACS; i++)  {
			if(net_dev->gmac_port[i].mmio_addr == 0)
				continue;
		//	net_dev->gmac_port[i].cfg_flag     = NLM_PORT_INIT | NLM_PORT_ATTACH;
			net_dev->gmac_port[i].cfg_flag     = NLM_PORT_ATTACH;
		}
		for(i = 0; i < NETLOGIC_MAX_XGMACS; i++)  {
			net_dev->xgs_port[i].cfg_flag     = 0;
		}
	}

	if(xlr_hybrid_rmios_tcpip_stack()) {
		/* port should be enabled by rmios apps
          after configuring the descriptors */
		for(i = 0; i < NETLOGIC_MAX_GMACS; i++)  {
			if(net_dev->gmac_port[i].mmio_addr == 0)
				continue;
			net_dev->gmac_port[i].cfg_flag     = NLM_PORT_ATTACH;
		}
		for(i = 0; i < NETLOGIC_MAX_XGMACS; i++)  {
			net_dev->xgs_port[i].cfg_flag     = 0;
		}
	}

	/* dev_tree_en */
/*
	if(dev_tree_en) {
		nlm_dev_config_net();
	}
*/

	return;
#endif /* XLP_SIM */
}

static int __init xlr_mac_desc_setup(char *str)
{
	int desc = simple_strtoul(str, 0, 10);
	struct net_device_cfg *net_dev = &xlr_net_dev_cfg;
	int i;

	printk("[%s]: str = \"%s\", desc=%d\n", __FUNCTION__, str, desc);
	if(desc == 0)
		return 1;

	for(i = 0; i < NETLOGIC_MAX_GMACS; i++) {
		if(net_dev->gmac_port[i].num_desc != 0)
			net_dev->gmac_port[i].num_desc = desc;
	}

	for(i = 0; i < NETLOGIC_MAX_XGMACS; i++)  {
		if(net_dev->xgs_port[i].num_desc != 0)
			net_dev->xgs_port[i].num_desc = desc;
	}

	return 1;
}

__setup("xlr_mac_desc=", xlr_mac_desc_setup);

static int __init xls_gmac0_sgmii_setup(char *str)
{
	struct net_device_cfg *net_dev = &xlr_net_dev_cfg;
	
	if (is_xls()) {
		if(net_dev->gmac_port[0].phy_mode & PHY_MODE_SELECTABLE) {
			net_dev->gmac_port[0].phy_mode = 
				(net_dev->gmac_port[0].phy_mode & PHY_MODE_SELECTABLE) | PHY_MODE_SGMII;
			net_dev->gmac_port[0].phy_addr = xlr_get_phy_info(0, 
				net_dev->gmac_port[0].phy_mode, 
				&net_dev->gmac_port[0].mii_addr, 
				&net_dev->gmac_port[0].pcs_addr, 
				&net_dev->gmac_port[0].serdes_addr);

			printk("[%s]: *********************************************\n", __FUNCTION__);
			printk("[%s]: Enabling SGMII mode for gmac0\n", __FUNCTION__);
			printk("[%s]: *********************************************\n", __FUNCTION__);
		}
	}

	return 1;
}
__setup("xls_gmac0_sgmii=", xls_gmac0_sgmii_setup);


