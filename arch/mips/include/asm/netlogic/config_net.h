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


#ifndef _ASM_NLM_NET_H
#define _ASM_NLM_NET_H

#include <asm/netlogic/msgring.h>

#define NETLOGIC_MAX_GMACS 8
#define NETLOGIC_MAX_XGMACS 2
#define NETLOGIC_MAX_XAUIS 2

#define NETLOGIC_GMAC_PORTS_PER_CTRL 4

#define NETLOGIC_MAX_MACS (NETLOGIC_MAX_GMACS + NETLOGIC_MAX_XGMACS)

enum config_flags { NLM_PORT_INIT = 1, 
					NLM_PORT_ATTACH = 2, 
					NLM_INT_ATTACH = 4, 
					NLM_MSGRNG_OWN = 8, 
					NLM_PORT_EN = 0x10 };

#define PORT_OWN_LINUX  ( NLM_PORT_INIT | NLM_PORT_ATTACH | NLM_INT_ATTACH | NLM_MSGRNG_OWN | NLM_PORT_EN )

/* 	PORT_INIT  : GMAC/XGMAC IP initialization will be done. 
	Port will be disabled after the initialization. 
	Glue logic(spill, packet descriptors will not be initialized 

	PORT_ATTACH : Eth interface will be attached to Linux 

	INT_ATTACH : GMAC/XGMAC MDIO interrupt will be attached to Linux

	MSGRNG_OWN : Glue logic(spill, packet descriptors will be initialized by linux

	PORT_EN : Option to enable the port
*/


struct port_cfg {
	/* port number */
	int instance;

	/* See enum config_flags */
	uint32_t cfg_flag;

	/* Interrupt Request number */
	int irqno; 

	/* number of descriptors configured */
	int num_desc; 

	/* pointer to the bucket config */
	bucket_t *bucket;

	/* pointer to the credit config */
	struct stn_cc *credit;

	/* driver should configure the pde */
	int config_pde;

	unsigned long mmio_addr; /* config address */
	uint32_t phy_addr; /* phy id */
	int phy_mode; /* sgmii or rgmii */
	unsigned long mii_addr; /* mdio addr */
	unsigned long pcs_addr; /* only for sgmii ports */
	unsigned long serdes_addr; /* only for sgmii ports */
};

struct net_device_cfg {
	struct port_cfg gmac_port[NETLOGIC_MAX_GMACS];
	int xgs_type[NETLOGIC_MAX_XGMACS];
	struct port_cfg xgs_port[NETLOGIC_MAX_XGMACS];
};


enum net_types { TYPE_GMAC = 0, TYPE_XGMAC, TYPE_SPI4, MAX_NET_TYPES };
enum phy_modes { PHY_MODE_SGMII	= 1, PHY_MODE_RGMII = 2, 
    PHY_MODE_SELECTABLE = 4, PHY_MODE_XAUI=8};

extern int xlr_get_phy_info(int instance, int mode, unsigned long *mii_addr, 
					unsigned long *pcs_addr, unsigned long *serdes_addr);

#define PORT_INIT(x) (x & NLM_PORT_INIT)
#define PORT_ATTACH(x) (x & NLM_PORT_ATTACH)
#define PORT_INT_ATTACH(x) (x & NLM_INT_ATTACH)
#define MSGRNG_OWN(x) (x & NLM_MSGRNG_OWN)
#define PORT_EN(x) (x & NLM_PORT_EN)

#endif


