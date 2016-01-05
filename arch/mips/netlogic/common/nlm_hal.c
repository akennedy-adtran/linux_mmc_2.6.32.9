/*-
 * Copyright (c) 2003-2015 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * http://www.gnu.org/licenses/gpl-2.0.txt
 * or the Broadcom license below:

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
 * #BRCM_4# */

/**
* @defgroup hal_overview Hardware Abstraction Layer APIs
* @brief HAL is the lowest API layer provided in the XLP SDK.
*
* HAL APIs provide register level access, configuration, and initialization to HyperExec. HAL is primarily used to implement the HyperExec library, as well as to implement OS level features such as kernel drivers or BSPs. <br>
*
*/

/**
* @defgroup hal Generic Hardware Abstraction Layer APIs
* @brief This section describes the generic and miscellaneous HAL APIs. <br>
*
* The generic HAL APIs provide the rudimentary functions for XLP register access.
*
* <b>Source:</b> libraries/hal/nlm_hal.c <br>
* <b>Header:</b> hyperexec/srcs/drivers/hal/nlm_hal_fmn.h
* @ingroup hal_overview
*/

/**
* @defgroup hal_nae NAE Hardware Abstraction Layer APIs
* @brief This section describes the NAE(Network Acceleration Engine) HAL APIs.<br>
*
* The NAE HAL APIs provide accses to all aspects of networking with the NAE, including NAE programming, POE programming, MAC programming, and external PHY access.
*
* <b>Source:</b> libraries/hal/nlm_hal.c <br>
* <b>Header:</b> hyperexec/srcs/drivers/hal/nlm_hal.h
* @ingroup hal_overview
*/

/**
* @defgroup hal_sae SAE Hardware Abstraction Layer APIs
* @brief This section describes the SAE(Security Acceleration Engine) HAL APIs <br>
*
* The SAE HAL APIs configure the SAE for application usage.
*
* <b>Source:</b> libraries/hal/nlm_hal.c <br>
* <b>Header:</b> hyperexec/srcs/drivers/hal/nlm_hal_sae.h
* @ingroup hal_overview
*/

/**
* @defgroup hal_fmn FMN Hardware Abstraction Layer APIs
* @brief This section describes the FMN(Fast Messaging Network) HAL APIs <br>
*
* The FMN HAL APIs configure and initialize the FMN (output queue carving and credits), as well as provide the fundamental messaging APIs.
*
* <b>Source:</b> libraries/hal/nlm_hal.c <br>
* <b>Header:</b> hyperexec/srcs/drivers/hal/nlm_hal_fmn.h
* @ingroup hal_overview
*/
#include "nlm_hal.h"
#include "nlm_hal_fmn.h"
#include "nlm_hal_nae.h"
#include "nlm_hal_crypto.h"
#include "nlm_hal_xlp_dev.h"
#include "nlm_hal_sys.h"
#include "libfdt.h"
#include "fdt_helper.h"

//#define INCLUDE_PHY_DEBUG		// SGMII phy detect and init debug
//#define INCLUDE_NAE_DEBUG		// General NAE debug

#ifdef INCLUDE_NAE_DEBUG
#define NAE_DEBUG	nlm_print
#else
#define NAE_DEBUG(...)
#endif

#ifdef INCLUDE_PHY_DEBUG
#define PHY_DEBUG	nlm_print
#else
#define PHY_DEBUG(...)
#endif

/* These addresses are computed by the nlm_hal_init() */
unsigned long xlp_io_base;
unsigned long xlp_fmn_base[NLM_MAX_NODES];
unsigned long xlp_nae_base[NLM_MAX_NODES];
unsigned long xlp_sae_base[NLM_MAX_NODES];
unsigned long xlp_rsa_base;
unsigned long xlp_mac_base[NLM_MAX_NODES];
unsigned long xlp_poe_base_pcie[NLM_MAX_NODES];
unsigned long xlp_poe_base_pcim[NLM_MAX_NODES];
unsigned long xlp_sys_base[NLM_MAX_NODES];
unsigned long xlp_regex_base_pcie;
unsigned long xlp_regex_base_pcim;

struct nlm_node_config nlm_node_cfg;

static int reg_num_phys;

extern void nae_ext_mdio_wait(void);

static int  mvl_get_phy_status(struct nlm_hal_ext_phy *phy,
		struct nlm_hal_mii_info* mii_info, int node);
static void mvl_restart_an(struct nlm_hal_ext_phy *phy, int node);
static int  mvl_init_phy(struct nlm_hal_ext_phy *phy, int node);

static int  bcm_get_phy_status(struct nlm_hal_ext_phy *phy,
		struct nlm_hal_mii_info* mii_info, int node);
static void bcm_restart_an(struct nlm_hal_ext_phy *phy, int node);
static int  bcm_init_phy(struct nlm_hal_ext_phy *phy, int node);
static int  xmc_init_phy(struct nlm_hal_ext_phy *phy, int node);

#define MAX_PHYS 18
/*PHYs */
static struct nlm_hal_ext_phy known_ext_phys[] = {
		{"mvs88e1114", 0x0c97, 0, 0, 0, 0, mvl_get_phy_status, mvl_restart_an, mvl_init_phy},
		{"bcm5461s",   0x60c1, 0, 0, 0, 0, bcm_get_phy_status, bcm_restart_an, bcm_init_phy},
		{"bcm5482s",   0xbcb2, 0, 0, 0, 0, bcm_get_phy_status, bcm_restart_an, xmc_init_phy},
		{"bcm5416",    0x5e74, 0, 0, 0, 0, bcm_get_phy_status, bcm_restart_an, bcm_init_phy},
		{"", 0, 0, 0, 0, 0, NULL, NULL, NULL}
};
static struct nlm_hal_ext_phy regs_ext_phys[MAX_PHYS];

static __inline__ unsigned int power_on_reset_cfg(void)
{
	return nlh_read_cfg_reg32(0x18035104);
}

#define PCI_MEM_BAR_0 0x4
#define PCIE_CONTROL_0 0x240

/**
* @brief nlm_hal_xlp_pcie_rc_init function is used to initialize the XLP PCIE controllers configured in RC mode.
*
* @return
*  - Returns no value.
*
* @ingroup hal
*
*/
__inline__ void nlm_hal_xlp_pcie_rc_init(void)
{
	int num_pcie = 4; /* Number of PCIe controllers */
	unsigned long base = 0x18000000;
	int dev = 1;
	int pcie = 0;

	unsigned int pciemode = (power_on_reset_cfg() >> 19) & 0xf;

	for (pcie = 0; pcie < num_pcie; pcie++) {
		unsigned long addr;
		unsigned int val;

		if (!(pciemode & (1 << pcie)))
			continue;

		addr = base + (dev << 15) + (pcie << 12);

		val = nlm_hal_read_32bit_reg(addr, PCIE_CONTROL_0);
		val |= (1 << 21); /* BAR mask enable */
		nlm_hal_write_32bit_reg(addr, PCIE_CONTROL_0, val);

		nlm_hal_write_32bit_reg(addr, PCI_MEM_BAR_0, 0x0);
	}
}

/* Basic Reg access
 */

/**
* @brief nlm_hal_read_16bit_reg function is used to read 16-bit registers (e.g. CPLD)
*
* @param [in] base Physical address where the register space starts
* @param [in] index Register Index
*
* @return
*  - 16bit register value
*
* @sa nlm_hal_write_16bit_reg, nlm_hal_read_32bit_reg, nlm_hal_write_32bit_reg, nlm_hal_read_64bit_reg, nlm_hal_write_64bit_reg??
* @ingroup hal
*
*/
__inline__ uint16_t nlm_hal_read_16bit_reg(uint64_t base, uint32_t index){
    return nlh_read_cfg_reg16(base + (index << 1));
}
/**
* @brief nlm_hal_write_16bit_reg function is used to write 16-bit registers (e.g. CPLD)
*
* @param [in] base Physical address where the register space starts
* @param [in] index Register Index
* @param [in] val Register value
*
* @return
*  - none
*
* @sa nlm_hal_read_16bit_reg, nlm_hal_read_32bit_reg, nlm_hal_write_32bit_reg, nlm_hal_read_64bit_reg, nlm_hal_write_64bit_reg??
* @ingroup hal
*
*/
__inline__ void nlm_hal_write_16bit_reg(uint64_t base, uint32_t index, uint16_t val){
    nlh_write_cfg_reg16(base +  (index << 1) , val);
}

/**
* @brief nlm_hal_read_32bit_reg function is used to read 32bit registers
*
* @param [in] base Physical address where the register space starts
* @param [in] index Register Index
*
* @return
*  - 32bit register value
*
* @sa nlm_hal_write_32bit_reg, nlm_hal_read_64bit_reg, nlm_hal_write_64bit_reg
* @ingroup hal
*
*/
__inline__ uint32_t nlm_hal_read_32bit_reg(uint64_t base, int index)
{
	return nlh_read_cfg_reg32(base + (index << 2));
}

/**
* @brief nlm_hal_write_32bit_reg function is used to write 32bit registers
*
* @param [in] base Physical address where the register space starts
* @param [in] index Register Index
* @param [in] val Register value
*
* @return
*  - none
*
* @sa nlm_hal_read_32bit_reg, nlm_hal_read_64bit_reg, nlm_hal_write_64bit_reg
* @ingroup hal
*
*/
__inline__ void nlm_hal_write_32bit_reg(uint64_t base, int index, uint32_t val)
{
	nlh_write_cfg_reg32(base +  (index << 2) , val);
}

/**
* @brief nlm_hal_read_64bit_reg function is used to read 64bit registers
*
* @param [in] base Physical address where the register space starts
* @param [in] index Register Index
*
* @return
*  - 64bit register value
*
* @sa nlm_hal_write_32bit_reg, nlm_hal_read_32bit_reg, nlm_hal_write_64bit_reg
* @ingroup hal
*
*/
__inline__ uint64_t nlm_hal_read_64bit_reg(uint64_t base, int index)
{
	return nlh_read_cfg_reg64(base + (index << 3));
}
/**
* @brief nlm_hal_read_64bit_reg function is used to write 64bit registers
*
* @param [in] base Physical address where the register space starts
* @param [in] index Register Index
* @param [in] val Register value
*
* @return
*  - none
*
* @sa nlm_hal_write_32bit_reg, nlm_hal_read_32bit_reg, nlm_hal_read_64bit_reg
* @ingroup hal
*
*/
__inline__ void nlm_hal_write_64bit_reg(uint64_t base, int index, uint64_t val)
{
	nlh_write_cfg_reg64(base +  (index << 3) , val);
}

/*
 *    Generic Devices
 */
/**
* @brief nlm_hal_get_dev_base function is used to get device base address
*
* @param [in] node Node ID
* @param [in] bus Bus ID
* @param [in] dev Device ID
* @param [in] func Function ID
*
* @return
*  - Physical address of the base address for a given (node, bus, device, function) combination
*
* @ingroup hal
*
*/
__inline__ uint64_t nlm_hal_get_dev_base(int node, int bus, int dev, int func)
{
	uint64_t base = xlp_io_base & 0x1fffffffULL;

	return (uint64_t)  (base +
			    (bus << 20) +
			    (dev << 15) +
			    (node*8 << 15) +
			    (func << 12));
}

/*
 *     FMN
 */
/**
* @brief nlm_hal_send_msg4 function is a non-blocking API used to send a three entry message to a mailbox. Does not retry the send message. Performs a sync before sending.
*
* @param [in] dst Destination Message Queue number
* @param [in] code 8b SW code to send with the message
* @param [in] data0 64b data value for the first message
* @param [in] data1 64b data value for the second message
* @param [in] data2 64b data value for the third message
* @param [in] data3 64b data value for the fourth message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
*
* @ingroup hal_fmn
*
*/
__inline__ uint32_t nlm_hal_send_msg4(uint32_t dst, uint32_t code, uint64_t data0, uint64_t data1, uint64_t data2, uint64_t data3)
{
	return nlh_send_msg4(dst, code, data0, data1, data2, data3);
}

/**
* @brief nlm_hal_send_msg3 function is a non-blocking API used to send a three entry message to a mailbox. Does not retry the send message. Performs a sync before sending.
*
* @param [in] dst Destination Message Queue number
* @param [in] code 8b SW code to send with the message
* @param [in] data0 64b data value for the first message
* @param [in] data1 64b data value for the second message
* @param [in] data2 64b data value for the third message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
*
* @ingroup hal_fmn
*
*/
__inline__ uint32_t nlm_hal_send_msg3(uint32_t dst, uint32_t code, uint64_t data0, uint64_t data1, uint64_t data2)
{
	return nlh_send_msg3(dst, code, data0, data1, data2);
}

/**
* @brief nlm_hal_send_msg2 function is a non-blocking API used to send a two entry message to a mailbox. Will retry the message send 16 times. Performs a sync before sending.
*
* @param [in] dst Destination Message Queue number
* @param [in] code 8b SW code to send with the message
* @param [in] data0 64b data value for the first message
* @param [in] data1 64b data value for the second message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
*
* @ingroup hal_fmn
*
*/
__inline__ uint32_t nlm_hal_send_msg2(uint32_t dst, uint32_t code, uint64_t data0, uint64_t data1)
{
	return nlh_send_msg2(dst, code, data0, data1);
}
/**
* @brief nlm_hal_send_msg1 function is a non-blocking API used to send a single entry message to a mailbox. Will retry the message send 16 times. Performs a sync before sending.
*
* @param [in] dst Destination Message Queue number
* @param [in] code 8b SW code to send with the message
* @param [in] data0 64b data value for the single message
*
* @return
*  - 0 on success, TxMsgStatus register on failure
*
* @ingroup hal_fmn
*
*/
__inline__ uint32_t nlm_hal_send_msg1(uint32_t dst, uint32_t code, uint64_t data0)
{
	return nlh_send_msg1(dst, code, data0);
}
/**
* @brief nlm_hal_recv_msg2 function is used to receive a two entry message from a VC of the CPU. Size should be used to determine how many of msg0-msg1 have valid data and if there were more messages available.
*
* @param [in] vc VC mailbox of the CPU (1 to 4)
* @param [out] src_id Source Message Queue Number
* @param [out] size # of messages that were in this received message (1 to 4)
* @param [out] code 8b SW code of the received message
* @param [out] msg0 64b data value for the first received message
* @param [out] msg1 64b data value for the second received message
*
* @return
*  - "0" on receive success, "-1" on failure
*
* @ingroup hal_fmn
*
*/
__inline__ uint32_t nlm_hal_recv_msg2(uint32_t dst, uint32_t *src, uint32_t *size, uint32_t *code, uint64_t *data0, uint64_t *data1)
{
	return nlh_recv_msg2(dst, src, size, code, data0, data1);
}
/**
* @brief nlm_hal_recv_msg1 function is used to receive a single entry message from a VC of the CPU. Size should be used to determine how other 64b messages were available with data.
*
* @param [in] dst VC mailbox of the CPU (1 to 4)
* @param [out] src Source Message Queue Number
* @param [out] size # of messages returned (1 to 4)
* @param [out] code 8b SW code of the received message
* @param [out] data0 64b data value for the received message
*
* @return
*  - "0" on receive success, "-1" on failure
*
* @ingroup hal_fmn
*
*/
__inline__ uint32_t nlm_hal_recv_msg1(uint32_t dst, uint32_t *src, uint32_t *size, uint32_t *code, uint64_t *data0)
{
	return nlh_recv_msg1(dst, src, size, code, data0);
}

/* Removed nlm_hal_is_xlp_a0() - assume no more A0s in field */

__inline__ int nlm_hal_is_xlp_le(void)
{
	unsigned int pwronrst = power_on_reset_cfg();
	int little_endian = ((pwronrst & (1 << 5)) == 0);
	return little_endian;
}

/*
 * @brief nlm_hal_get_fdt_freq function is used to read the frequency specified in the fdt file.
 *
 * @param [in]  pointer to the fdt file
 * @param [in]  block for which we need the frequency.
 *
 * @return
 * actual frequency on success & "-1" if the frency is not specified in fdt file.
 *
 * @ingroup hal
 *
 **/
int nlm_hal_get_fdt_freq(void *fdt, int type)
{
	uint32_t freq;
	int ret = 250;  /* Set the default frequency to 250 */
	char path_str[50];

	sprintf(path_str,"/frequency-config");

	switch(type)
	{
	case NLM_NAE:
		if(copy_fdt_prop(fdt, path_str, "nae", PROP_CELL, &freq, sizeof(uint32_t)) < 0)
			nlm_print("Unable to find the frequency in the FDT file for type:%d, \
					using the default value\n", type);
		else
			ret = freq;
#ifdef FREQ_DEBUG
		nlm_print("NAE frequency is %d\n", ret);
#endif
		break;
	case NLM_RSA:
		if(copy_fdt_prop(fdt, path_str, "rsa", PROP_CELL, &freq, sizeof(uint32_t)) < 0)
			nlm_print("Unable to find the frequency in the FDT file for type:%d, \
					using the default value\n", type);
		else
			ret = freq;
#ifdef FREQ_DEBUG
		nlm_print("RSA frequency is %d\n", ret);
#endif
		break;
	case NLM_SAE:
		if(copy_fdt_prop(fdt, path_str, "sae", PROP_CELL, &freq, sizeof(uint32_t)) < 0)
			nlm_print("Unable to find the frequency in the FDT file for type:%d, \
					using the default value\n", type);
		else
			ret = freq;
#ifdef FREQ_DEBUG
		nlm_print("SAE frequency is %d\n", ret);
#endif
		break;
	case NLM_DTRE:
		if(copy_fdt_prop(fdt, path_str, "dtre", PROP_CELL, &freq, sizeof(uint32_t)) < 0)
			nlm_print("Unable to find the frequency in the FDT file for type:%d, \
					using the default value\n", type);
		else
			ret = freq;
#ifdef FREQ_DEBUG
		nlm_print("DTRE frequency is %d\n", ret);
#endif
		break;
	case NLM_CDE:
		if(copy_fdt_prop(fdt, path_str, "cde", PROP_CELL, &freq, sizeof(uint32_t)) < 0)
			nlm_print("Unable to find the frequency in the FDT file for type:%d, \
					using the default value\n", type);
		else
			ret = freq;
#ifdef FREQ_DEBUG
		nlm_print("CDE frequency is %d\n", ret);
#endif
		break;
	default:
		{
			nlm_print("Frequency not specified in the FDT file for type:%d", type);
			ret = -1;
		}
	}
	return ret;
}

/* Main initialization */
/**
* @brief nlm_hal_init function is used to Initialize HAL
*
* @return
*  - Returns no value.
*
* @note
*    This function must be the first to be called before any other HAL APIs
*
* @ingroup hal
*
*/

void nlm_hal_init(void)
{
	unsigned long long mask = ~0xf;
	int node = 0;
#if !defined(NLM_HAL_LINUX_USER)
	unsigned int flags = 0;
	enable_KX(flags);
#endif

#ifdef NLM_HAL_LINUX_KERNEL
	printk("Initializing XLP Hardware Abstraction Library\n");
#endif
	nlm_node_cfg.valid = 1;
	nlm_node_cfg.num_nodes = 1;

 	for(node = 0; node < NLM_MAX_NODES; node++) {
		/*nlm_node_cfg.nae_cfg[node] = NULL; */
		nlm_node_cfg.nae_cfg[node] = NULL;
		nlm_node_cfg.fmn_cfg[node] = NULL;

		xlp_io_base = KSEG1 + 0x18000000;

		/* PCI enumeration of supported devices*/
		xlp_fmn_base[node] = mask & nlm_hal_read_32bit_reg((0x18000000 + XLP_CFG_BASE(node, XLP_FMN)), PCI_MEM_BAR_0);

		xlp_mac_base[node] = mask & nlm_hal_read_32bit_reg((0x18000000 + XLP_CFG_BASE(node, XLP_NAE)), PCI_MEM_BAR_0); /*0x18018000 */
		/*printf("Node:%d NAE_MAC_Base:%lX\n", node, xlp_mac_base[node]); */
		xlp_nae_base[node] = xlp_mac_base[node] + 0xe000;	// BLOCK_7 << 13, used to access NAE global configuration registers

		xlp_poe_base_pcim[node] = mask & nlm_hal_read_32bit_reg((0x18000000 + XLP_CFG_BASE(node, XLP_POE)), PCI_MEM_BAR_0);	/*0x18019000 */
		xlp_poe_base_pcie[node] = (xlp_io_base | XLP_CFG_BASE(node, XLP_POE)) & 0x1fffffff; /* For now . Will be fixed soon.*/

		xlp_sys_base[node] = (xlp_io_base | XLP_CFG_BASE(node, XLP_SYS)) & 0x1fffffff; /*For now . Will be fixed soon.*/
		xlp_sae_base[node] = (xlp_io_base | XLP_CFG_BASE(node, XLP_SAE)) & 0x1fffffff; /* For now . Will be fixed soon.*/
	}

        xlp_rsa_base = (xlp_io_base | XLP_CFG_BASE(0, XLP_RSA)) & 0x1fffffff; /* For now . Will be fixed soon.*/

	if ( is_nlm_xlp3xx() || is_nlm_xlp2xx() ) {
		xlp_regex_base_pcie = (xlp_io_base | (XLP_CFG_BASE(0, XLP_3XX_REGEX))) & 0x1fffffff;
		xlp_regex_base_pcim = mask & nlm_hal_read_32bit_reg((0x18000000 + XLP_CFG_BASE(0, XLP_3XX_REGEX)), PCI_MEM_BAR_0);
//		nlm_print("xlp3xx/2xx Regex (netl7) vendor_device id:%#x\n"
//				"xlp_regex_base_pcim:%#lx\n"
//				"xlp_regex_base_pcie:%#lx\n",
//				nlm_hal_read_32bit_reg(xlp_regex_base_pcie, 0),
//				xlp_regex_base_pcim, xlp_regex_base_pcie);
	}

#ifdef CONFIG_NLM_XLP_EVP_CPLD
	nlm_hal_cpld_init(0);
#endif

#if !defined(NLM_HAL_LINUX_USER) && (_MIPS_SZLONG == 64)
	disable_KX(flags);
#endif
}


/*
 * Naming convention: NLM_HAL_XXX for external API
 *                    NLH_XXX for internal naming of NL HAL
 */
#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/module.h>
EXPORT_SYMBOL(xlp_io_base);
EXPORT_SYMBOL(xlp_fmn_base);
EXPORT_SYMBOL(xlp_nae_base);
EXPORT_SYMBOL(xlp_mac_base);
EXPORT_SYMBOL(xlp_sys_base);
EXPORT_SYMBOL(xlp_sae_base);
EXPORT_SYMBOL(xlp_rsa_base);
EXPORT_SYMBOL(xlp_poe_base_pcie);
EXPORT_SYMBOL(xlp_poe_base_pcim);
EXPORT_SYMBOL(nlm_hal_init);
EXPORT_SYMBOL(nlm_hal_read_32bit_reg);
EXPORT_SYMBOL(nlm_hal_write_32bit_reg);
EXPORT_SYMBOL(nlm_hal_send_msg1);
EXPORT_SYMBOL(nlm_hal_recv_msg1);
EXPORT_SYMBOL(nlm_hal_send_msg2);
EXPORT_SYMBOL(nlm_hal_recv_msg2);
EXPORT_SYMBOL(nlm_hal_send_msg3);
EXPORT_SYMBOL(nlm_hal_send_msg4);
#else
#include "nlm_hal_pic.h"
/*
   This is to map 160 irt entry to 64 interrupt vector
   Each row has three elements
   irq		shared     number of sharing
   irq:  assigned irq number used in linux
   shared:  0: this irq not shared,  1: this irq is shared
   number of sharing:  if shared = 1,  this variable indicate number of irt line to shared the same irq
   if shared = 0,  this should be 0.
*/
#define SHARED_IRQ	1
#define NOT_SHARED	0

int irt_irq_table[160][4]= {
        {9,     1,      2,      0},     /*PICIRT_WD_0_INDEX         0	*/
        {9,     1,      2,      0},     /*PICIRT_WD_1_INDEX         1	*/
        {19,    1,      2,      0},     /*PICIRT_WD_NMI_0_INDEX     2	*/
        {19,    1,      2,      0},     /*PICIRT_WD_NMI_1_INDEX     3	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_0_INDEX      4	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_1_INDEX      5	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_2_INDEX      6	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_3_INDEX      7	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_4_INDEX      8	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_5_INDEX      9	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_6_INDEX      10	*/
        {10,    1,      8,      0},     /*PICIRT_TIMER_7_INDEX      11	*/
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(0),    12	*/
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(1),    13  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(2),    14  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(3),    15  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(4),    16  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(5),    17  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(6),    18  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(7),    19  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(8),    20  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(9),    21  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(10),   22  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(11),   23  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(12),   24  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(13),   25  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(14),   26  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(15),   27  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(16),   28  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(17),   29  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(18),   30  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(19),   31  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(20),   32  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(21),   33  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(22),   34  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(23),   35  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(24),   36  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(25),   37  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(26),   38  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(27),   39  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(28),   40  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(29),   41  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(30),   42  */
        {59,    1,      32,     0},     /*PICIRT_MSG_Q_INDEX(31),   43  */
        {49,    0,      0,      0},     /*PICIRT_MSG_0_INDEX,       44	*/
        {48,    0,      0,      0},     /*PICIRT_MSG_1_INDEX,       45	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(0) 46  */
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(1) 47  */
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(2) 48	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(3) 49	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(4) 50	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(5) 51	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(6) 52	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(7) 53	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(8) 54	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(9) 55	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(10)56 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(11)57 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(12)58	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(13)59 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(14)60 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(15)61 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(16)62 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(17)63	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(18)64 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(19)65 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(20)66 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(21)67 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(22)68 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(23)69 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(24)70 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(25)71 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(26)72 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(27)73 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(28)74 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(29)75 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(30)76 	*/
        {46,    1,      32,     0},     /*PICIRT_PCIE_MSIX_INDEX(31)77 	*/
        {44,    0,      0,      0},     /*PICIRT_PCIE_LINK_INDEX(0) 78	*/
        {43,    0,      0,      0},     /*PICIRT_PCIE_LINK_INDEX(1) 79	*/
        {42,    0,      0,      0},     /*PICIRT_PCIE_LINK_INDEX(2) 80	*/
        {41,    0,      0,      0},     /*PICIRT_PCIE_LINK_INDEX(3) 81	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(0)        82	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(1)        83	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(2)        84	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(3)        85	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(4)        86	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(5)        87	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(6)        88	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(7)        89	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(8)        90	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(9)        91	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(10)       92	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(11)       93	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(12)       94	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(13)       95	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(14)       96	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(15)       97	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(16)       98	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(17)       99	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(18)       100	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(19)       101	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(20)       102 */
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(21)       103	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(22)       104	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(23)       105	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(24)       106	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(25)       107	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(26)       108	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(27)       109	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(28)       110	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(29)       111	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(30)       112	*/
        {58,    1,      32,     0},     /*PICIRT_NA_INDEX(31)       113	*/
        {60,    0,      0,      0},     /*PICIRT_POE_INDEX          114	*/
        {24,    1,      6,      0},     /*PICIRT_USB_INDEX(0)       115	*/
        {25,    1,      6,      0},     /*PICIRT_USB_INDEX(1)       116	*/
        {25,    1,      6,      0},     /*PICIRT_USB_INDEX(2)       117	*/
        {24,    1,      6,      0},     /*PICIRT_USB_INDEX(3)       118	*/
        {25,    1,      6,      0},     /*PICIRT_USB_INDEX(4)       119	*/
        {25,    1,      6,      0},     /*PICIRT_USB_INDEX(5)       120	*/
        {61,    0,      0,      0},     /*PICIRT_GDX_INDEX          121 */
        {63,    0,      0,      0},     /*PICIRT_SEC_INDEX          122 */
        {62,    0,      0,      0},     /*PICIRT_RSA_INDEX          123 */
        {39,    1,      4,      0},     /*PICIRT_COMP_INDEX(0)      124 */
        {39,    1,      4,      0},     /*PICIRT_COMP_INDEX(1)      125 */
        {39,    1,      4,      0},     /*PICIRT_COMP_INDEX(2)      126 */
        {39,    1,      4,      0},     /*PICIRT_COMP_INDEX(3)      127 */
        {0,     0,      0,      0},     /*                          128 */
        {37,    1,      3,      0},     /*PICIRT_ICC_0_INDEX        129  ICC - Inter Chip Coherency*/
        {37,    1,      3,      0},     /*PICIRT_ICC_1_INDEX        130 */
        {37,    1,      3,      0},     /*PICIRT_ICC_2_INDEX        131 */
        {36,    0,      0,      0},     /*PICIRT_CAM_INDEX          132 */
        {17,    0,      0,      0},     /*PICIRT_UART_0_INDEX       133 */
        {18,    0,      0,      0},     /*PICIRT_UART_1_INDEX       134 */
        {11,    1,      2,      0},     /*PICIRT_I2C_0_INDEX        135	*/
        {11,    1,      2,      0},     /*PICIRT_I2C_1_INDEX        136	*/
        {12,    1,      2,      0},     /*PICIRT_SYS_0              137	*/
        {12,    1,      2,      0},     /*PICIRT_SYS_1              138	*/
        {55,    0,      0,      0},     /*PICIRT_JTAG_INDEX         139	*/
        {50,    0,      0,      0},     /*PICIRT_PIC                140	*/
        {0,     0,      0,      0},     /*Reserved                  141	*/
        {0,     0,      0,      0},     /*Reserved                  142	*/
        {0,     0,      0,      0},     /*Reserved                  143 */
        {0,     0,      0,      0},     /*Reserved        	    144	*/
        {0,     0,      0,      0},     /*Reserved        	    145	*/
        {13,    0,      0,      0},     /*PICIRT_GPIO_INDEX(0)      146	*/
        {14,    0,      0,      0},     /*PICIRT_GPIO_INDEX(1)      147	*/
        {15,    0,      0,      0},     /*PICIRT_GPIO_INDEX(2)      148	*/
        {16,    0,      0,      0},     /*PICIRT_GPIO_INDEX(3)      149	*/
        {20,    0,      0,      0},     /*PICIRT_NOR                150	*/
        {21,    0,      0,      0},     /*PICIRT_NAND               151	*/
        {22,    0,      0,      0},     /*PICIRT_SPI                152	*/
        {23,    0,      0,      0},     /*PICIRT_MMC                153	*/
        {54,    0,      0,      0},     /*PICIRT_NBU		    154	*/
        {53,    0,      0,      0},     /*PICIRT_TCU                155	*/
        {52,    0,      0,      0},     /*PICIRT_GCU                156	*/
        {36,    1,      2,      0},     /*DDR3 DMC                  157 */
        {36,    1,      2,      0},     /*DDR3 DMC		    158 */
        {57,    0,      0,      0},     /*Trace Buffer	TCB	    159 */
};


/*
  short find_irt_from_irq( int irq)

  find irt number from irq
  irq: input irq number,
  return:  irt number,  if it is -1, indicate can't find irt line for irq number.
*/
int find_irt_from_irq( int irq)
{
        unsigned long long irt_pending0, irt_pending1, irt_pending2;
        int base_irt, num_shared = 0, i,j;
        uint64_t val;
        uint64_t shared_mask;

        if(irq <0 || irq >63)
        {
                return -1;
        }
        if(irq < 8)
                return irq;

        /*from base irt_irq_table, find base irt number, for shared irq, need figure out which is the*/
        for ( i = 0; i < PIC_NUM_IRTS; i++)
        {
                if(irq == irt_irq_table[i][0])
                {
                        /* unshared irq*/
                        if(irt_irq_table[i][1] == NOT_SHARED)
                        {
                                /* this is irt number we needed;*/
                                return  i;
                        }
                        else if(irt_irq_table[i][1] == SHARED_IRQ)
                        {
                                /*if shared bit is 1,  this irq is shared by a number of irt line*/
                                num_shared = irt_irq_table[i][2];
                                break;
                        }
                }

        }
        base_irt = i;
        if(num_shared == NOT_SHARED || base_irt == 160)
                return -1;
        /* for shared irq, need figure out which irt line produce this irq*/
        /* we can determine it by look at the interrupt pending register*/

        /*first scan col 4 of enabled field to see whether any IRT is enabled,*/
        /*it could be just first time to register*/
        for(j = 0; j < num_shared; j++)
        {
		shared_mask = (1ULL << num_shared) - 1;
                if( irt_irq_table[base_irt][0] != irt_irq_table[base_irt + j][0])
                        continue;

                if(irt_irq_table[base_irt + j][3] == 1)
                {
                        val = nlm_hal_read_pic_reg(nlm_hal_pic_offset(), PIC_IRT(base_irt + j));
                        if(!(val & (1 << 31)))
                        {
                                /*this irt entry not enable yet*/
                                return base_irt + j;

                        }
                        else
                        {
                                /*check pending register*/
                                if(base_irt+j < 64)
                                {
                                        irt_pending0 = nlm_hal_read_pic_reg(nlm_hal_pic_offset(), PIC_INT_PENDING0);
                                        shared_mask  = shared_mask << base_irt;
                                        irt_pending0 = irt_pending0 & shared_mask;
                                        if(irt_pending0 & (1ULL << (base_irt + j)))
                                                return (j + base_irt);
                                }
                                else if(base_irt + j >= 64 && base_irt + j < 128)
                                {
                                        irt_pending1 = nlm_hal_read_pic_reg(nlm_hal_pic_offset(), PIC_INT_PENDING1);
                                        shared_mask  = shared_mask << (base_irt - 64);
                                        irt_pending1 = irt_pending1 & shared_mask;
                                        if(irt_pending1 & (1ULL << (base_irt + j - 64)))
                                                return (j + base_irt);

                                }
                                else if(base_irt+j > 128)
                                {
                                        irt_pending2 = nlm_hal_read_pic_reg(nlm_hal_pic_offset(), PIC_INT_PENDING2);
                                        shared_mask  = shared_mask << (base_irt - 128);
                                        irt_pending2 = irt_pending2 & shared_mask;
                                        if(irt_pending2 & (1ULL << (base_irt + j - 128)))
                                                return (j + base_irt);
                                }
                        }
                }
        }

        /*if we get here, means penging register is not set for all */
        for(j = 0; j < num_shared ; j++)
        {

                if(irt_irq_table[base_irt+j][3] == 1)
                        return (base_irt+j);
        }
        return -1;
}


int nlm_hal_request_shared_irq(int irt)
{
        uint64_t  val;

        if(irt < 0 || irt > PIC_NUM_IRTS)
                return -1;
        irt_irq_table[irt][3] = 1;
        val = nlm_hal_read_pic_reg(nlm_hal_pic_offset(), PIC_IRT(irt));
        /* clear DB and DTE field */
        val &= ~(0x3f << 20);
        val |= (irt_irq_table[irt][0] << 20 | 1 << 31 | 1 << 28);
        nlm_hal_write_pic_reg(nlm_hal_pic_offset(), PIC_IRT(irt),val);


        return irt_irq_table[irt][0];
}
void nlm_hal_unrequest_shared_irq(int irt)
{
        if(irt < 0 || irt > PIC_NUM_IRTS)
                return;
        if(irt_irq_table[irt][3] == 1)
                irt_irq_table[irt][3] = 0;
        return;
}


unsigned long tlb_size_to_page_size(unsigned long size)
{
	if (size <= (4*1024)) return 4*1024;
	if (size <= (16*1024)) return 16*1024;
	if (size <= (64*1024)) return 64*1024;
	if (size <= (256*1024)) return 256*1024;
	if (size <= (1024*1024)) return 1024*1024;
	if (size <= (4*1024*1024)) return 4*1024*1024;
	if (size <= (16*1024*1024)) return 16*1024*1024;
	if (size <= (64*1024*1024)) return 64*1024*1024;

	return 256*1024*1024;
}

unsigned long tlb_size_to_mask(unsigned long size)
{
	if (size <= (4*1024)) return 0x0 << 13;
	if (size <= (16*1024)) return 0x03 << 13;
	if (size <= (64*1024)) return 0x0f << 13;
	if (size <= (256*1024)) return 0x3f << 13;
	if (size <= (1024*1024)) return 0xff << 13;
	if (size <= (4*1024*1024)) return 0x3ff << 13;
	if (size <= (16*1024*1024)) return 0xfff << 13;
	if (size <= (64*1024*1024)) return 0x3fff << 13;

	return 0xffff << 13;
}
#endif

/* Convenience functions */
static inline uint16_t phy_read(int node, unsigned int bus,
		unsigned int phyaddr, unsigned int reg)
{
	return nlm_hal_mdio_read(node, NLM_HAL_EXT_MDIO, bus,
			BLOCK_7, LANE_CFG, phyaddr, reg);
}

static inline void phy_write(int node, unsigned int bus,
		unsigned int phyaddr, unsigned int reg, uint16_t data)
{
	nlm_hal_mdio_write(node, NLM_HAL_EXT_MDIO, bus,
			BLOCK_7, LANE_CFG, phyaddr, reg, data);
}

/* PHY Registers */
/* Broadcom */
#define BCM_SHADOW_REG_18		0x18
#define BCM_SHADOW_REG_1C		0x1C
#define BCM_AUX_STATUS_REG		0x19
/* Shadow registers accessed through register 0x1C */
#define BCM_SPARE_CTRL1_REG		0x02
#define BCM_CLK_ALIGN_CTRL_REG	0x03
#define BCM_LED_SEL1_REG		0x0D
#define BCM_LED_SEL2_REG		0x0E
#define BCM_LED_GPIO_CTRL_REG	0x0F
#define BCM_PRI_SERDES_CTRL_REG	0x16
#define BCM_1000BASEX_CTRL2_REG	0x17
#define BCM_MODE_CTRL_REG		0x1F

/* Marvell */
#define MVL_PAGE_SEL_REG		22
#define MVL_CTRL1_REG			16
#define MVL_CTRL2_REG			26

static inline uint16_t read_bcm_shadow_reg(int node, unsigned int bus,
		unsigned int phyaddr, unsigned int reg)
{
	uint16_t val = reg << 10;

	/* Set shadow register address for read, only return bits 9:0 */
	phy_write(node, bus, phyaddr, BCM_SHADOW_REG_1C, val);
	return phy_read(node, bus, phyaddr, BCM_SHADOW_REG_1C) & 0x3FF;
}

static inline void write_bcm_shadow_reg(int node, unsigned int bus,
		unsigned int phyaddr, unsigned int reg, uint16_t data)
{
	uint16_t val = (1 << 15) | (reg << 10) | (data & 0x3FF);

	phy_write(node, bus, phyaddr, BCM_SHADOW_REG_1C, val);
}

/**
* @brief nlm_hal_init_ext_phy function initializes the external PHY of an interface.
*
* @param [in] node Node number
* @param [in] inf Interface number
*
* @return
* 	- 0 if the phy was successfully initialized
*   - -1 if the phy was not successfully initialized
*
* @ingroup hal_nae
*
*/
int nlm_hal_init_ext_phy(int node, int inf)
{
	int int_inf;
	uint16_t val;
	uint8_t phy_mode;
	struct nlm_hal_ext_phy *this_phy=NULL;

	this_phy = get_phy_info(inf);
	if(!this_phy) return -1;
	int_inf = this_phy->inf;
	phy_mode = this_phy->phy_mode;

	/* Soft-reset XLP PCS, enable AN if Phy operating mode is SGMII-Copper. Note per
	 * XLP datasheet, MII_Advertise register power-on reset value is 0x0000 - must
	 * set selector field to CSMA.
	 */
	nlm_hal_mdio_write(NODE_0, NLM_HAL_INT_MDIO, 0, BLOCK_7, LANE_CFG, inf, MII_ADVERTISE, ADVERTISE_CSMA);
	if(phy_mode) val = BMCR_RESET | BMCR_SPEED1000 | BMCR_FULLDPLX;		// 1000BASE-X modes
	else         val = BMCR_RESET | BMCR_ANENABLE;						// Copper mode
	nlm_hal_mdio_write(node, NLM_HAL_INT_MDIO, 0, BLOCK_7, LANE_CFG, int_inf, MII_BMCR, val);
	PHY_DEBUG("%s: Port %d SGMII PCS wrote 0x%04X to MII_BMCR\n", __func__, int_inf, val);
	return this_phy->ext_phy_init(this_phy, node);
}

/**
* @brief xmc_init_phy function initializes an external BROADCOM 5482 PHY on the BCM965502 board.
*
* Note - XMC board support broken due to quick changes for DSL BSP - must add #define to revert
* this to support XMC
*
* @param [in] phy nlm_hal_ext_phy struct pointing to the BROADCOM PHY
* @param [in] node Node number
*
* @return
* 	- 0 if the phy was successfully initialized
*   - -1 if the phy was not successfully initialized
*
* @ingroup hal_nae
*
*/
static int xmc_init_phy(struct nlm_hal_ext_phy *phy, int node)
{
	unsigned int bus = phy->ext_mdio_bus;
	unsigned int phyaddr = phy->phy_addr;
	int int_inf = phy->inf;
	uint16_t phymode, status;

	PHY_DEBUG("%s: bus %d phyaddr %d interface %d\n", __func__, bus, phyaddr, int_inf);

#ifdef NLM_HAL_XMC_SUPPORT
	/* XMC board has INTFSEL[1:0] = 01 which selects fiber and power down modes,
	 * so setup for SGMII-Copper mode.
	 */
	status = read_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG);
	/* Bits 3 and 9 are reserved, write as 1 and 0 */
	status &= 0x1FC;	// Clear bit 0 for copper registers
	status |= 0x00C;	// SGMII mode (bits 2:1 = 10)
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG, status);

	/* Power up copper side */
	status = phy_read(node, bus, phyaddr, MII_BMCR);
	status &= ~BMCR_PDOWN;
	PHY_DEBUG("writing phyaddr %d BMCR = 0x%04X\n", phyaddr, status);
	phy_write(node, bus, phyaddr, MII_BMCR, status);
	nlm_mdelay(100);
#else
	/* Read Mode Control shadow register 0x1F - bits 2:1 are phy mode.
	 * Verify we are in one of two supported modes (1 = SGMII-RGMII, 2 = SGMII-Copper).
	 */
	status = read_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG);
	phymode = (status & 0x006) >> 1;
	if((phymode == 0) || (phymode == 3)) {
		nlm_print("SGMII Interface %d: Unsupported BCM5482S operating mode %d (phy %d)\n",
				int_inf, phymode, phyaddr);
		return -1;
	}
#endif

	/* Turn off Signal Detect Enable on 1000BASE-X side */
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_PRI_SERDES_CTRL_REG, 0x000);
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_1000BASEX_CTRL2_REG, 0x000);

	if(phymode == 2) {		// SGMII - Copper mode
		/* Setup PHY LED control */
		/* Set bit 1: Enable Link LED */
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_SPARE_CTRL1_REG, 0x001);
		/* LED3 (bits 7:4) = 0xE (off), LED1 (bits 3:0) = 0x5 (SLAVE - link) */
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_LED_SEL1_REG, 0x0E5);
		/* LED2 (bits 7:4) = 0x3 (ACTIVITY), LED4 (bits 3:0) = 0xE (off) */
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_LED_SEL2_REG, 0x03E);

		/* Select copper registers (clear bit 0) */
		status = read_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG);
		status &= 0x1FE;	// Bit 9 reserved, write as 0
		status |= 0x008;	// Bit 3 reserved, write as 1
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG, status);

		/* Setup AN advertising */
		status = ADVERTISE_ALL | ADVERTISE_CSMA;
		phy_write(node, bus, phyaddr, MII_ADVERTISE, status);
		status = ADVERTISE_1000FULL | ADVERTISE_1000HALF;
		phy_write(node, bus, phyaddr, MII_CTRL1000, status);

		/* Enable and restart AN in MII control register */
		status = BMCR_ANENABLE | BMCR_ANRESTART;
		phy_write(node, bus, phyaddr, MII_BMCR, status);
		PHY_DEBUG("%s: Phy %d wrote 0x%04x to copper MII control register\n",
				__func__, phyaddr, status);

	} else {	// SGMII - RGMII mode
		PHY_DEBUG("%s: Phy %d configuring for RGMII-1000BASE-X mode\n", __func__, phyaddr);
		/* Disable LEDs */
		/* LED Selector 1 reg 0x0D: LED3 (bits 7:4), LED1 (bits 3:0) = 0xE (off) */
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_LED_SEL1_REG, 0xEE);
		/* LED Selector 2 reg 0x0E: LED2 (bits 7:4), LED4 (bits 3:0) = 0xE (off) */
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_LED_SEL2_REG, 0xEE);
		/* LED GPIO Control reg 0x0F: Disable LED2 & 1 (set bits 3 & 0) */
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_LED_GPIO_CTRL_REG, 0x09);

		/* Switch to 1000BASE-X registers: Mode Control reg 0x1F set bit 0 */
		status = read_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG);
		status &= 0x1FF;	// Bit 9 reserved, write as 0
		status |= 0x009;	// Bit 3 reserved, write as 1
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG, status);

		/* Disable auto-negotiation, set 1000Mbps, full duplex */
		phy_write(node, bus, phyaddr, MII_BMCR, BMCR_FULLDPLX | BMCR_SPEED1000);

#if 1
		/* Disable RGMII TX Delay (DSL Customer Ref Code enables RGMII-ID feature on BCM65500
		 * RGMII interface). Clock Alignment Control reg 0x03 -> Clear bit 9, bits 8:0 are
		 * reserved, write as 0. Alternative is to disable this feature in the DSL ref code:
		 * BCM65500 register 0x3500 - set bit 1.
		 */
		write_bcm_shadow_reg(node, bus, phyaddr, BCM_CLK_ALIGN_CTRL_REG, 0x000);
		PHY_DEBUG("%s: Phy %d disabling RGMII TX Delay\n", __func__, phyaddr);
#endif
#if 0
		/* Disable RGMII RX Delay: Register 0x18 shadow 7 -> Clear bit 8 */
		phy_write(node, bus, phyaddr, BCM_SHADOW_REG_18, 0x7007);
		status = phy_read(node, bus, phyaddr, BCM_SHADOW_REG_18);
		status |= (1 << 15);
		status &= ~((1 << 10) | (1 << 8));	// Bit 10 reserved - write as 0
		phy_write(node, bus, phyaddr, BCM_SHADOW_REG_18, status);
		PHY_DEBUG("%s: Phy %d disabling RGMII RX Delay\n", __func__, phyaddr);
#endif

	}

	return 0;
}

/**
* @brief bcm_init_phy function initializes an external BROADCOM PHY.
*
* @param [in] phy nlm_hal_ext_phy struct pointing to the BROADCOM PHY
* @param [in] node Node number
*
* @return
* 	- 0 if the phy was successfully initialized
*   - -1 if the phy was not successfully initialized
*
* @ingroup hal_nae
*
*/
static int bcm_init_phy(struct nlm_hal_ext_phy *phy, int node)
{
	unsigned int bus = phy->ext_mdio_bus;
	unsigned int phyaddr = phy->phy_addr;
	uint16_t status;

	PHY_DEBUG("%s: bus %d phyaddr %d interface %d\n", __func__, bus, phyaddr, phy->inf);

	/* Turn off Signal Detect Enable on 1000BASE-X side */
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_PRI_SERDES_CTRL_REG, 0x000);
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_1000BASEX_CTRL2_REG, 0x000);

	/* Setup PHY LED control */
	/* Set bit 1: Enable Link LED */
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_SPARE_CTRL1_REG, 0x001);
	/* LED3 (bits 7:4) = 0xE (off), LED1 (bits 3:0) = 0x5 (SLAVE - link) */
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_LED_SEL1_REG, 0x0E5);
	/* LED2 (bits 7:4) = 0x3 (ACTIVITY), LED4 (bits 3:0) = 0xE (off) */
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_LED_SEL2_REG, 0x03E);

	/* Select copper registers (clear bit 0) */
	status = read_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG);
	status &= 0x1FE;	// Bit 9 reserved, write as 0
	status |= 0x008;	// Bit 3 reserved, write as 1
	write_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG, status);

	/* Setup AN advertising */
	status = ADVERTISE_ALL | ADVERTISE_CSMA;
	phy_write(node, bus, phyaddr, MII_ADVERTISE, status);
	status = ADVERTISE_1000FULL | ADVERTISE_1000HALF;
	phy_write(node, bus, phyaddr, MII_CTRL1000, status);

	/* Enable and restart AN in MII control register */
	status = BMCR_ANENABLE | BMCR_ANRESTART;
	phy_write(node, bus, phyaddr, MII_BMCR, status);
	PHY_DEBUG("%s: Phy %d wrote 0x%04x to copper MII control register\n",
			__func__, phyaddr, status);

	return 0;
}

/**
* @brief mvl_init_phy function initializes an external MARVELL PHY.
*
* @param [in] phy nlm_hal_ext_phy struct pointing to the MARVELL PHY
* @param [in] node Node number
*
* @return
* 	- 0 if the phy was successfully initialized
*   - -1 if the phy was not successfully initialized
*
* @ingroup hal_nae
*
*/
static int mvl_init_phy(struct nlm_hal_ext_phy *phy, int node)
{
	unsigned int bus = phy->ext_mdio_bus;
	unsigned int phyaddr = phy->phy_addr;

	PHY_DEBUG("%s: bus %d phyaddr %d interface %d\n", __func__, bus, phyaddr, phy->inf);

	/* Device initialization required per Marvell datasheet pg. 25. Note MVL phy
	 * permanently configured for preamble suppression thus an idle MDIO clock
	 * cycle is required between back-to-back operations. Do not remove delays.
	 */
	/* Select page 2 */
	phy_write(node, bus, phyaddr, MVL_PAGE_SEL_REG, 0x0002);
	nae_ext_mdio_wait();
	/* Set mode = Copper (bits 9:7 = 0x5), disable SGMII power down (bit 3) */
	phy_write(node, bus, phyaddr, MVL_CTRL1_REG, 0x0288);
	nae_ext_mdio_wait();
	/* Enable AN Bypass (allow link if link partner doesn't support AN - bit 15) */
	phy_write(node, bus, phyaddr, MVL_CTRL2_REG, 0x8000);
	nae_ext_mdio_wait();
	/* Select page 0, auto page select (reg 22 bit 15) */
	phy_write(node, bus, phyaddr, MVL_PAGE_SEL_REG, 0x8000);
	nae_ext_mdio_wait();
	/* Enable Copper Energy Detect mode 2 (bits 9:8) */
	phy_write(node, bus, phyaddr, MVL_CTRL1_REG, 0x6360);
	nae_ext_mdio_wait();
	/* Setup AN advertising */
	phy_write(node, bus, phyaddr, MII_ADVERTISE, ADVERTISE_ALL | ADVERTISE_CSMA);
	nae_ext_mdio_wait();
	phy_write(node, bus, phyaddr, MII_CTRL1000, ADVERTISE_1000FULL | ADVERTISE_1000HALF);
	nae_ext_mdio_wait();
	/* Soft reset and enable AN */
	phy_write(node, bus, phyaddr, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);

    return 0;
}

/**
* @brief nlm_hal_ext_phy_an function restarts auto-negotiation on an interface.
*
* @param [in] node Node number
* @param [in] inf Interface on which to restart auto-negotiation
*
* @return
* 	- 0 if auto-negotiation completed successfully
*   - -1 if auto-negotiation failed at the PHY
*   - -2 if auto-negotiation failed at the XLP
*
* @ingroup hal_nae
*
*/
int nlm_hal_ext_phy_an(int node, int inf)
{
#ifdef WAIT_FOR_AN_COMPLETE
	int count = 0;
	uint16_t status;
	unsigned int int_inf, bus;
#endif
	unsigned int phyaddr;
	struct nlm_hal_ext_phy *this_phy=NULL;

	this_phy = get_phy_info(inf);
	if(!this_phy) return -1;
	if(this_phy->phy_mode) return -1;	// 1000BASE-X mode - no AN

	phyaddr = this_phy->phy_addr;
	this_phy->start_phy_an(this_phy, node);

#ifdef WAIT_FOR_AN_COMPLETE
	int_inf = this_phy->inf;
	bus = this_phy->ext_mdio_bus;

	/* Allow up to five seconds for phy AN to complete */
	do {
		nlm_mdelay(100);
		status = phy_read(node, bus, phyaddr, MII_BMSR);
		if(status & BMSR_ANEGCOMPLETE) {
			PHY_DEBUG("Phy AN OK, count = %d\n", count);
			break;
		}
		count++;
	} while(count < 50);
	if(count == 50) {
		nlm_print("Port %d Phy AN failed\n", int_inf);
		return -1;
	}

	/* Restart XLP AN */
	status = nlm_hal_mdio_read(node, NLM_HAL_INT_MDIO, 0, BLOCK_7, LANE_CFG, int_inf, MII_BMCR);
	status |= BMCR_ANRESTART;
	nlm_hal_mdio_write(node, NLM_HAL_INT_MDIO, 0, BLOCK_7, LANE_CFG, int_inf, MII_BMCR, status);

	/* Wait for XLP<->SGMII-PHY AN to be OK - this should be almost instantaneous */
	PHY_DEBUG("Restarting XLP AN for port %d\n", int_inf);
	count = 0;
	do {
		nlm_mdelay(1);
		status = nlm_hal_mdio_read(node, NLM_HAL_INT_MDIO, 0, BLOCK_7, LANE_CFG, int_inf, MII_BMSR);
		if(status & BMSR_ANEGCOMPLETE) {
			PHY_DEBUG("XLP AN OK, count = %d\n", count);
			return 0;
		}
		count++;
	} while(count < 50);

	PHY_DEBUG("Port %d XLP AN failed\n", int_inf);

	return -2;
#else
	return 0;
#endif
}

/**
* @brief bcm_start_an function restarts auto-negotiation on external BROADCOM PHY
*
* @param [in] phy nlm_hal_ext_phy struct pointing to the BROADCOM PHY
* @param [in] node Node number
*
* @return
* 	- none
*
* @ingroup hal_nae
*
*/
static void bcm_restart_an(struct nlm_hal_ext_phy *phy, int node)
{
	uint16_t val;
	unsigned int phyaddr = phy->phy_addr;
	unsigned int bus = phy->ext_mdio_bus;

	val = phy_read(node, bus, phyaddr, MII_BMCR);
	val |= BMCR_ANENABLE | BMCR_ANRESTART;
	phy_write(node, bus, phyaddr, MII_BMCR, val);
	PHY_DEBUG("%s: Phy %d wrote 0x%04x to control register\n", __func__,
			phyaddr, val);
}

/**
* @brief mvl_restart_an function restarts auto-negotiation on an external MARVELL PHY.
*
* @param [in] phy nlm_hal_ext_phy struct pointing to the MARVELL PHY
* @param [in] node Node number
*
* @return
* 	- none
*
* @ingroup hal_nae
*
*/
static void mvl_restart_an(struct nlm_hal_ext_phy *phy, int node)
{
	uint16_t val;
	unsigned int phyaddr = phy->phy_addr;
	unsigned int bus = phy->ext_mdio_bus;

	val = phy_read(node, bus, phyaddr, MII_BMCR);
	val |= BMCR_ANENABLE | BMCR_ANRESTART;
	nae_ext_mdio_wait();
	phy_write(node, bus, phyaddr, MII_BMCR, val);
	PHY_DEBUG("%s: Phy %d wrote 0x%04x to control register\n", __func__,
			phyaddr, val);
}

/**
* @brief nlm_hal_status_ext_phy function fetches link status info from an external phy
*
* @param [in] node Node number
* @param [in] inf Interface number to query
* @param [in] mii_info Pointer to struct nlm_hal_mii_info to populate
* @param [out] link_stat Link status (up or down)
* @param [out] media_detect Cable/media detected status
* @param [out] speed Link speed
* @param [out] duplex Full(1) or half(0) duplex
*
* @return
* 	- 1 if link up, 0 if link down
*
* @ingroup hal_nae
*
*/

int nlm_hal_status_ext_phy(int node, int inf, struct nlm_hal_mii_info* mii_info)
{
	struct nlm_hal_ext_phy *this_phy=NULL;
	this_phy = get_phy_info(inf);
	if(!this_phy) return 0;

	return this_phy->phy_get_status(this_phy, mii_info, node);
}

/**
* @brief bcm_get_phy_status function returns the status of an interface from the external BROADCOM PHY.
*
* @param [in] phy Pointer to struct nlm_hal_ext_phy for the BROADCOM PHY
* @param [in] mii_info Pointer to nlm_hal_mii_info struct to hold Link info
* @param [in] node Node number
* @param [out] link_stat Link status (up or down)
* @param [out] media_detect Cable/media detected status
* @param [out] speed Link speed
* @param [out] duplex Full(1) or half(0) duplex
*
* @return
* 	- 1 if link up (or media detected and AN in process), 0 if link down
*
* @ingroup hal_nae
*
*/
static int bcm_get_phy_status(struct nlm_hal_ext_phy *phy,
		struct nlm_hal_mii_info* mii_info, int node)
{
	uint16_t status, aux_status;
	unsigned int phyaddr = phy->phy_addr;
	unsigned int bus = phy->ext_mdio_bus;

	mii_info->phyaddr=phyaddr;

	/* Logic:
	 * 1) Check MII link state, populate mii_info. If up, get speed/duplex info and return 1
	 * 2) Check for media (energy detect), populate mii_info. If no media, return 0
	 * 3) Check if AN state. If AN not completed, return 1
	 * 4) Return 0
	 */

	phy_read(node, bus, phyaddr, MII_BMSR);
	status = phy_read(node, bus, phyaddr, MII_BMSR);
	PHY_DEBUG("%s: Port %d phy %d MII status = 0x%x\n", __func__,
			phy->inf, phyaddr, status);

	/* If link is up, get speed and duplex from aux_status, populate mii_info */
	if(status & BMSR_LSTATUS) {
		mii_info->link_stat = 1;
		mii_info->media_detect = 1;
		aux_status = phy_read(node, bus, phyaddr, BCM_AUX_STATUS_REG);
		PHY_DEBUG("  Link is up, MII Status = 0x%04x, aux status = 0x%04x\n",
				status, aux_status);
		switch ((aux_status >> 8) & 0x7) {
		case 0x7:
			mii_info->speed = SPEED_1000M;
			mii_info->duplex = 1;
		break;
		case 0x6:
			mii_info->speed = SPEED_1000M;
			mii_info->duplex = 0;
		break;
		case 0x5:
			mii_info->speed = SPEED_100M;
			mii_info->duplex = 1;
		break;
		case 0x3:
			mii_info->speed = SPEED_100M;
			mii_info->duplex = 0;
		break;
		case 0x2:
			mii_info->speed = SPEED_10M;
			mii_info->duplex = 1;
		break;
		case 0x1:
			mii_info->speed = SPEED_10M;
			mii_info->duplex = 0;
		break;
		default:
			mii_info->speed = SPEED_UNKNOWN;
			mii_info->duplex = 0;
			nlm_print("Port %d unknown operating speed, aux status = 0x%04x\n",
					phy->inf, aux_status);
		}
		PHY_DEBUG("  Link is up, MII Status = 0x%04x, aux status = 0x%04x\n",
				status, aux_status);
		return 1;
	}

	PHY_DEBUG("  Link is down, MII status = 0x%x\n", status);
	mii_info->link_stat = 0;
	mii_info->speed = SPEED_UNKNOWN;
	mii_info->duplex = 0;

	/* Energy detected on copper interface? Phy mode control shadow reg 0x1F bit 5 */
	aux_status = read_bcm_shadow_reg(node, bus, phyaddr, BCM_MODE_CTRL_REG);
	PHY_DEBUG("  Phy mode control = 0x%x\n", aux_status);
	if (!(aux_status & 0x0020)) {
		PHY_DEBUG("  No media detected\n");
		mii_info->media_detect = 0;
		return 0;
	}
	mii_info->media_detect = 1;

	/* AN in process? Check status bit 5 - 1 means AN is complete. If AN in process and
	 * media detected, assume AN will succeed and return 1.
	 */
	if(!(status & BMSR_ANEGCOMPLETE)) {
		PHY_DEBUG("  AN in progress\n");
		return 1;
	}

	return 0;
}

/**
* @brief mvl_get_phy_status function returns the status of an interface from the external MARVELL PHY.
*
* @param [in] phy nlm_hal_ext_phy struct pointing to the MARVELL PHY
* @param [in] mii_info Pointer to nlm_hal_mii_info struct to hold Link info
* @param [in] node Node number
* @param [out] link_stat Link status (up or down)
* @param [out] media_detect Cable/media detected status
* @param [out] speed Link speed
* @param [out] duplex Full(1) or half(0) duplex
*
* @return
* 	- 1 if link up (or media detected and AN in process), 0 if link down
*
* @ingroup hal_nae
*
*/
#define MVL_EXT_STATUS		17
static int mvl_get_phy_status(struct nlm_hal_ext_phy *phy,
		struct nlm_hal_mii_info* mii_info, int node)
{
	uint16_t status, extstatus;
	unsigned int phyaddr = phy->phy_addr;
	unsigned int bus = phy->ext_mdio_bus;

	mii_info->phyaddr=phyaddr;

	/* Logic:
	 * 1) Check MII link state, populate mii_info. If up, get speed/duplex info and return 1
	 * 2) Check for media (energy detect), populate mii_info. If no media, return 0
	 * 3) Check if AN state. If AN not completed, return 1
	 * 4) Return 0
	 */

	/* Note the below two reads will be issued back-to-back without an idle
	 * in-between. Should be OK though since both are the same operation. If
	 * one read fails, we just get stale (latched) status bits instead of the
	 * right now status.
	 */
	phy_read(node, bus, phyaddr, MII_BMSR);
	status = phy_read(node, bus, phyaddr, MII_BMSR);
	PHY_DEBUG("%s: Port %d phy %d MII Status = 0x%04x\n", __func__,
			phy->inf, phyaddr, status);

	/* If link is up, get speed and duplex from ext_status, populate mii_info */
	if (status & BMSR_LSTATUS) {
		mii_info->link_stat = 1;
		mii_info->media_detect = 1;
		extstatus = phy_read(node, bus, phyaddr, MVL_EXT_STATUS);
		mii_info->speed =  (extstatus >> 14) & 0x3;
		mii_info->duplex = (extstatus >> 13) & 0x1;
		PHY_DEBUG("  Link is up, MII Status = 0x%04x, ext status = 0x%04x\n",
				status, extstatus);
		return 1;
	}

	PHY_DEBUG("  Link is down, MII Status = 0x%04x\n", status);
	mii_info->link_stat = 0;
	mii_info->speed = SPEED_UNKNOWN;
	mii_info->duplex = 0;

	/* Media attached? Check extstatus bit 4 - 1 means cable detached */
	extstatus = phy_read(node, bus, phyaddr, MVL_EXT_STATUS);
	PHY_DEBUG("  Extended status = 0x%x\n", extstatus);
	if(extstatus & 0x0010) {
		PHY_DEBUG("  No media detected\n");
		mii_info->media_detect = 0;
		return 0;
	}
	mii_info->media_detect = 1;

	/* AN in process? Check status bit 5 - 1 means AN is complete. If AN in process and
	 * media detected, assume AN will succeed and return 1.
	 */
	if(!(status & BMSR_ANEGCOMPLETE)) {
		PHY_DEBUG("  AN in progress\n");
		return 1;
	}

	return 0;
}

/**
* @brief get_phy_info function returns PHY information from the external PHY of an interface.
*
* @param [in] inf Interface number
*
* @return
* 	- Pointer to external phy information structure
* 	- NULL if no registered external phy exists for inf
*
* @ingroup hal_nae
*
*/
struct nlm_hal_ext_phy *get_phy_info(int inf)
{
	struct nlm_hal_ext_phy *phy_info = NULL;
	/*search through scanned and registered phys*/
	int reg_idx=0;
	for(; reg_idx<MAX_PHYS; reg_idx++){
		if(regs_ext_phys[reg_idx].inf==inf){
			phy_info = regs_ext_phys + reg_idx;
			return phy_info;
		}
	}
	NAE_DEBUG("Interface could not be initialised for inf=0x%x\n", inf);
	return NULL;
}

/**
* @brief register_phy function registers external PHY information for an interface.
*
* @param [in] node Node number
* @param [in] inf Interface number
*
* @return
* 	- pointer to struct nlm_hal_ext_phy for this phy for success
*   - NULL for failure
*
* @ingroup hal_nae
*
*/
struct nlm_hal_ext_phy *register_phy(int node, uint8_t inf)
{
	nlm_nae_config_ptr nae_cfg = nlm_node_cfg.nae_cfg[node];
	int i;
	uint8_t phy_addr = 0xFF;
	uint8_t phy_mode = 0;
	for(i = 0; i < nae_cfg->num_ports; i++)
		if(nae_cfg->ports[i].hw_port_id == inf) {
			phy_addr = nae_cfg->ports[i].ext_phy_addr;
			phy_mode = nae_cfg->ports[i].ext_phy_mode;
			break;
		}

	for(i = 0; i < reg_num_phys; i++){
		if(phy_addr == regs_ext_phys[i].phy_addr) {
			regs_ext_phys[i].inf = inf;
			regs_ext_phys[i].phy_mode = phy_mode;
			PHY_DEBUG("[%s] Interface %d phy address is %d\n",
					__func__, inf, phy_addr);
			return &(regs_ext_phys[i]);
		}
	}

	nlm_print("[%s] Could not find interface %d\n", __func__, inf);
	return NULL;
}

/**
* @brief sgmii_scan_phys function scans all possibel PHYs on the external MDIO busses and logs active ports.
*
* @param [in] node Node number
*
* @return
* 	- none
*
* @ingroup hal_nae
*
*/
void sgmii_scan_phys(int node)
{
	int phyid, inf, bus, maxbus;
	int j;
	int reg_idx=0;

	/*init regs_ext_phys data*/
	for (j=0; j<MAX_PHYS; j++){
		regs_ext_phys[j].phy_get_status = NULL;
		regs_ext_phys[j].start_phy_an = NULL;
		regs_ext_phys[j].ext_phy_init = NULL;
		regs_ext_phys[j].phy_addr = 0xff;
		regs_ext_phys[j].inf = -1;
	}
	/* scan all PHYs available on ext MDIOs */
	/* check with phys IDs against registered phys */
	/* Two MDIO controllers on Eagle, only one on Storm/Firefly */
	maxbus = is_nlm_xlp8xx() ? 2 : 1;
	for(bus=0; bus < maxbus; bus++) {
		PHY_DEBUG("Scanning MDIO external BUS %d...\n", bus);
		for(inf=0; inf<31; inf++) {
			phyid = phy_read(node, bus, inf, MII_PHYSID2);
			if (phyid != 0xffff) {
				PHY_DEBUG("  MDIO Device ID %d returned PhyID 0x%04x\n", inf, phyid);
				for(j=0; j < sizeof(known_ext_phys)/ sizeof(struct nlm_hal_ext_phy); j++)
					if(phyid == known_ext_phys[j].phy_idfer) {
						regs_ext_phys[reg_idx].ext_mdio_bus = bus;
						regs_ext_phys[reg_idx].phy_get_status = known_ext_phys[j].phy_get_status;
						regs_ext_phys[reg_idx].start_phy_an = known_ext_phys[j].start_phy_an;
						regs_ext_phys[reg_idx].ext_phy_init = known_ext_phys[j].ext_phy_init;
						regs_ext_phys[reg_idx].phy_addr = inf;
						regs_ext_phys[reg_idx].inf = -1;
						regs_ext_phys[reg_idx].phy_idfer = known_ext_phys[j].phy_idfer;
						PHY_DEBUG("    Phy registered as %s\n", known_ext_phys[j].name);
						reg_idx++;
						break;
					}
			}
		}
	}

	reg_num_phys =  reg_idx;
	PHY_DEBUG("Total SGMII PHYs found = %d\n", reg_idx);
}

/* CDE SUPPORT
 */
/**
* @brief nlm_hal_set_cde_freq function sets the frequency of the CDE block.
*
* @param [in] node Node number
* @param [in] freq Frequency to set in MHz
*
* @return
* 	- none
*
* @ingroup hal
*
*/
void nlm_hal_set_cde_freq(int node, int freq)
{
	const uint32_t mhz = 1000000;
	soc_device_id_t device = is_nlm_xlp2xx() ? XLP2XX_CLKDEVICE_CMP : DFS_DEVICE_CMP;

#ifdef FREQ_DEBUG
	uint32_t set_freq = nlm_hal_set_soc_freq(node, device, freq * mhz);
	nlm_print("-- CDE Frequency set to %d MHz\n", (set_freq + 500000) / mhz);
#else
	nlm_hal_set_soc_freq(node, device, freq * mhz);
#endif
}

/* DTRE SUPPORT
 */
/**
* @brief nlm_hal_set_dtre_freq function sets the frequency of the DTR block.
*
* @param [in] node Node number
* @param [in] freq Frequency to set in MHz
*
* @return
* 	- none
*
* @ingroup hal
*
*/
void nlm_hal_set_dtre_freq(int node, int freq)
{
	const uint32_t mhz = 1000000;
	soc_device_id_t device = is_nlm_xlp2xx() ? XLP2XX_CLKDEVICE_GDX : DFS_DEVICE_DTRE;

#ifdef FREQ_DEBUG
	uint32_t set_freq = nlm_hal_set_soc_freq(node, device, freq * mhz);
	nlm_print("-- DTRE Frequency set to %d MHz\n", (set_freq + 500000) / mhz);
#else
	nlm_hal_set_soc_freq(node, device, freq * mhz);
#endif
}

/**
* @brief nlm_hal_dtr_init function is used to enable DTR block on XLP.
*
* @return : none
*
* @ingroup hal
*
*/
void nlm_hal_dtr_init(void *fdt)
{
    uint64_t base = nlm_hal_get_dev_base (XLP_DTR_NODE, XLP_DTR_BUS, XLP_DTR_DEVICE, XLP_DTR_FUNC);
    int frequency = 0;
    int node = 0;
    /* Enable Master Control register */
    nlm_hal_write_32bit_reg (base, XLP_DTR_MASTER_CONTROL_REG, 0x1);
    /* Channel control registers */
    nlm_hal_write_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_0, 0x3fe);
    nlm_hal_write_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_1, 0x3fe);
    nlm_hal_write_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_2, 0x3fe);
    nlm_hal_write_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_3, 0x3fe);

    frequency = nlm_hal_get_fdt_freq(fdt, NLM_DTRE);
    nlm_hal_set_dtre_freq(node, frequency);

#ifdef DUMP
    nlm_print ("Base Register 0x%llx\n", (unsigned long long)base);
    nlm_print ("Master control 0x%x\n", nlm_hal_read_32bit_reg (base, XLP_DTR_MASTER_CONTROL_REG));
    nlm_print ("Channel control0 0x%x\n", nlm_hal_read_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_0));
    nlm_print ("Channel control1 0x%x\n", nlm_hal_read_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_1));
    nlm_print ("Channel control2 0x%x\n", nlm_hal_read_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_2));
    nlm_print ("Channel control3 0x%x\n", nlm_hal_read_32bit_reg (base, XLP_DTR_CHANNEL_CONTROL_REG_3));
#endif
}

/* SAE SUPPORT
 */
/**
* @brief nlm_hal_set_sae_freq function sets the frequency of the SAE block.
*
* @param [in] node Node number
* @param [in] freq Frequency to set in MHz
*
* @return
* 	- none
*
* @ingroup hal_sae
*
*/
void nlm_hal_set_sae_freq(int node, int freq)
{
	const uint32_t mhz = 1000000;
	soc_device_id_t device = is_nlm_xlp2xx() ? XLP2XX_CLKDEVICE_SAE : DFS_DEVICE_SAE;

#ifdef FREQ_DEBUG
	uint32_t set_freq = nlm_hal_set_soc_freq(node, device, freq * mhz);
	nlm_print("-- SAE Frequency set to %d MHz\n", (set_freq + 500000) / mhz);
#else
	nlm_hal_set_soc_freq(node, device, freq * mhz);
#endif
}

/* Comment this out if using SDK 2.3 libraries/crypto and linux-userspace */
#if 1
enum chip_specific_features {
	INIT_DONE = 0x1,
	ZUC = 0x2,
	DES3_KEY_SWAP = 0x4
};
#endif

int nlm_hal_get_sae_chip_feature(void )
{
	int chip_features;
	if( is_nlm_xlp2xx())
		chip_features = (INIT_DONE | ZUC | DES3_KEY_SWAP);
	else
		chip_features = INIT_DONE;
	return chip_features;
}

/* RSA/ECC SUPPORT
 */
/**
* @brief nlm_hal_set_rsa_freq function sets the frequency of the RSA/ECC block.
*
* @param [in] node Node number
* @param [in] freq Frequency to set in MHz
*
* @return
* 	- none
*
* @ingroup hal_sae
*
*/
void nlm_hal_set_rsa_freq(int node, int freq)
{
	const uint32_t mhz = 1000000;
	soc_device_id_t device = is_nlm_xlp2xx() ? XLP2XX_CLKDEVICE_RSA : DFS_DEVICE_RSA;

#ifdef FREQ_DEBUG
	uint32_t set_freq = nlm_hal_set_soc_freq(node, device, freq * mhz);
	nlm_print("--RSA Frequency set to %d MHz\n", (set_freq + 500000) / mhz);
#else
	nlm_hal_set_soc_freq(node, device, freq * mhz);
#endif
}

void nlm_hal_sata_firmware_init(void)
{
	uint32_t readdata;
	int i = 0;

	nlm_print("Started AHCI Firmware Initialization.\n");

	nlm_mdelay(1000);

	readdata = rd_sata_glue_reg(XLP_HAL_SATA_CTL);

//	nlm_print ("Reseting PHYs.\n");
	clear_sata_glue_reg(XLP_HAL_SATA_CTL, SATA_RST_N);
	clear_sata_glue_reg(XLP_HAL_SATA_CTL, PHY3_RESET_N);
	clear_sata_glue_reg(XLP_HAL_SATA_CTL, PHY2_RESET_N);
	clear_sata_glue_reg(XLP_HAL_SATA_CTL, PHY1_RESET_N);
	clear_sata_glue_reg(XLP_HAL_SATA_CTL, PHY0_RESET_N);
	readdata = rd_sata_glue_reg(XLP_HAL_SATA_CTL);
	nlm_mdelay(10);

	set_sata_glue_reg(XLP_HAL_SATA_CTL, SATA_RST_N);
	set_sata_glue_reg(XLP_HAL_SATA_CTL, PHY3_RESET_N);
	set_sata_glue_reg(XLP_HAL_SATA_CTL, PHY2_RESET_N);
	set_sata_glue_reg(XLP_HAL_SATA_CTL, PHY1_RESET_N);
	set_sata_glue_reg(XLP_HAL_SATA_CTL, PHY0_RESET_N);

	readdata = rd_sata_glue_reg(XLP_HAL_SATA_CTL);
	wr_sata_glue_reg(XLP_HAL_SATA_CTL, readdata);
	readdata = rd_sata_glue_reg(XLP_HAL_SATA_CTL);

//	nlm_print ("Waiting for PHYs to come up.\n");

	readdata = rd_sata_glue_reg(XLP_HAL_SATA_STATUS);
	while ( ((readdata & 0x00F0) != 0x00F0) && (i < 30))
	{
		readdata = rd_sata_glue_reg(XLP_HAL_SATA_STATUS);
		nlm_mdelay(10);
		i++;
	}

#if 0
	if (readdata  & P0_PHY_READY) nlm_print(" PHY0 is up.\n");
	else nlm_print(" PHY0 is down.\n");
	if (readdata  & P1_PHY_READY) nlm_print(" PHY1 is up.\n");
	else nlm_print(" PHY1 is down.\n");
	if (readdata  & P2_PHY_READY) nlm_print(" PHY2 is up.\n");
	else nlm_print(" PHY2 is down.\n");
	if (readdata  & P3_PHY_READY) nlm_print(" PHY3 is up.\n");
	else nlm_print(" PHY3 is down.\n");
#endif

	nlm_print("AHCI Firmware Init Done.\n");
}

void nlm_hal_sata_intr_setup(void)
{
	uint32_t val;

	/* clear pending interrupts and then enable them */
	val = rd_sata_glue_reg(XLP_HAL_SATA_INT);
	nlm_mdelay(10);
	wr_sata_glue_reg(XLP_HAL_SATA_INT, val);
	nlm_mdelay(10);

	val = rd_sata_glue_reg(XLP_HAL_SATA_INT_MASK);
	nlm_mdelay(10);
#if 1
	wr_sata_glue_reg(XLP_HAL_SATA_INT_MASK, 0x1);
#else
	wr_sata_glue_reg(XLP_HAL_SATA_INT_MASK, val & 0x1BFF3);
#endif
}

void nlm_hal_sata_intr_ack(void)
{
	uint32_t val = 0;

	val = rd_sata_glue_reg(XLP_HAL_SATA_INT);
	wr_sata_glue_reg(XLP_HAL_SATA_INT, val & 0x1BFF3);
}

void nlm_hal_sata_init(void)
{
	nlm_hal_sata_firmware_init();
}

uint32_t get_dom_owner_mask(void *fdt, int dom_id, char *module)
{
	char dom_node_str[32];
	unsigned int *pval;
	int nodeoffset;
	int plen;
	uint32_t flag;

	sprintf(dom_node_str, "/doms/dom@%d/owner-config", dom_id);
	nodeoffset = fdt_path_offset(fdt, dom_node_str);

	if (nodeoffset >= 0)
	{
		pval = ((unsigned int *)fdt_getprop(fdt, nodeoffset, module, &plen));
		if (pval != NULL) {
			flag = fdt32_to_cpu(*(unsigned int *)pval);
			NAE_DEBUG("owner flag for %s is %#x.\n", module, flag);
		}
		else {
			flag = 0;
			NAE_DEBUG("ERROR: pval is NULL.\n");
		}
	}
	else
	{
		flag = 0;
		NAE_DEBUG("ERROR: unable to find nodeoffset.\n");
	}

	return flag;
}

void nlm_hal_set_rsa_cge(int node, int enable)
{
#define NLM_RSA_CFG_REG 0x40
	uint32_t d32 = nlm_hal_read_rsa_reg(NLM_RSA_CFG_REG);
	if(enable)
		d32 |= 1<<9;
	else
		d32 &= ~(1<<9);
	nlm_hal_write_rsa_reg(NLM_RSA_CFG_REG, d32);
}

#define NLM_SAE_ENGINE_SELECT_REG_0 0x41
void nlm_hal_set_sae_engine_sel(int node)
{
	int i, n;
	if(is_nlm_xlp2xx()) {
		return;
	}
	if(is_nlm_xlp3xx())
		n = 1;
	else
		n = 8;

	for (i = 0; i < n; i++) {
		nlm_hal_write_sae_reg(node, NLM_SAE_ENGINE_SELECT_REG_0 + i, 0x00FFFFFF);
	}
}

#define NLM_RSA_ENGINE_SELECT_REG_0 0x41
void nlm_hal_set_rsa_engine_sel(void)
{
	int i, n;
	unsigned int val;
	if(is_nlm_xlp2xx()) {
		return;
	}
	if(is_nlm_xlp3xx()) {
		n = 1;
		val = 0xffff;
	} else {
		n = 3;
		val = 0x7ffffff;
	}
	for (i = 0; i < n; i++) {
		nlm_hal_write_rsa_reg(NLM_RSA_ENGINE_SELECT_REG_0 + i, val);
	}
}

void nlm_hal_get_crypto_vc_nums(int *vcbase, int *vclimit)
{

	if(is_nlm_xlp3xx()) {
		*vcbase = XLP_3XX_CRYPTO_VC_BASE;
		*vclimit = XLP_3XX_CRYPTO_VC_LIMIT;
	} else if(is_nlm_xlp2xx()) {
		*vcbase = XLP_2XX_CRYPTO_VC_BASE;
		*vclimit = XLP_2XX_CRYPTO_VC_LIMIT;
	} else {
		*vcbase = XLP_CRYPTO_VC_BASE;
		*vclimit = XLP_CRYPTO_VC_LIMIT;
	}
}

void nlm_hal_get_rsa_vc_nums(int *vcbase, int *vclimit)
{
	if(is_nlm_xlp3xx()) {
		*vcbase  = XLP_3XX_RSA_ECC_VC_BASE;
		*vclimit = XLP_3XX_RSA_ECC_VC_LIMIT;
	} else if(is_nlm_xlp2xx()) {
		*vcbase  = XLP_2XX_RSA_ECC_VC_BASE;
		*vclimit = XLP_2XX_RSA_ECC_VC_LIMIT;
	} else {
		*vcbase = XLP_RSA_ECC_VC_BASE;
		*vclimit = XLP_RSA_ECC_VC_LIMIT;

	}
}

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/module.h>
EXPORT_SYMBOL(nlm_hal_is_xlp_le);
EXPORT_SYMBOL(sgmii_scan_phys);
EXPORT_SYMBOL(nlm_hal_get_dev_base);
EXPORT_SYMBOL(nlm_hal_set_sae_freq);
EXPORT_SYMBOL(nlm_hal_get_sae_chip_feature);
EXPORT_SYMBOL(nlm_hal_set_rsa_freq);
EXPORT_SYMBOL(nlm_hal_set_dtre_freq);
EXPORT_SYMBOL(nlm_hal_set_cde_freq);
EXPORT_SYMBOL(nlm_node_cfg);
EXPORT_SYMBOL(nlm_hal_init_ext_phy);
EXPORT_SYMBOL(nlm_hal_ext_phy_an);
EXPORT_SYMBOL(nlm_hal_status_ext_phy);
EXPORT_SYMBOL(register_phy);
EXPORT_SYMBOL(get_dom_owner_mask);
EXPORT_SYMBOL(nlm_hal_sata_init);
EXPORT_SYMBOL(nlm_hal_sata_intr_setup);
EXPORT_SYMBOL(nlm_hal_sata_intr_ack);
EXPORT_SYMBOL(nlm_hal_get_fdt_freq);
EXPORT_SYMBOL(get_phy_info);
EXPORT_SYMBOL(copy_fdt_prop);
EXPORT_SYMBOL(nlm_hal_set_rsa_cge);
EXPORT_SYMBOL(nlm_hal_set_sae_engine_sel);
EXPORT_SYMBOL(nlm_hal_set_rsa_engine_sel);
EXPORT_SYMBOL(nlm_hal_get_crypto_vc_nums);
EXPORT_SYMBOL(nlm_hal_get_rsa_vc_nums);
#endif
