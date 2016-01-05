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

/* Uncomment this define this for basic debugging.  Uncomment both for more
 * extensive debug output (warning - likely to overflow kernel log buffer).
 */
//#define FMN_DEBUG
//#define FMN_DEBUG_MORE

/* Uncomment for configuration more like stock SDK (very wasteful of on-chip SRAM */
//#define OLD_SCHOOL

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/netlogic/hal/nlm_hal_fmn.h>
#else
#define KERN_DEBUG
#define KERN_WARNING
#define KERN_ERR
#define __init
#define __initdata
#include "nlm_hal_fmn.h"
#endif

#include "libfdt.h"
#include "fdt_helper.h"
#include "nlm_hal_sys.h"

#define OUTQ_EN		0x8000000000000000ULL
#define SPILL_EN	0x4000000000000000ULL

#ifdef FMN_DEBUG
#define FMN_DEBUG_PRINTK		nlm_print
#else
#define FMN_DEBUG_PRINTK(...)
#endif

#ifdef FMN_DEBUG_MORE
#define FMN_DEBUG_VERBOSE		nlm_print
#else
#define FMN_DEBUG_VERBOSE(...)
#endif

/* Removed a bunch of defines not used in the code */

extern struct nlm_node_config nlm_node_cfg;

#if defined(NLM_HAL_LINUX_KERNEL)
extern cpumask_t fdt_cpumask;
#endif

static unsigned int *tmp_credit_arr, *tmp_spill_arr, *tmp_pages_arr;
static unsigned int max_qs, max_nodes;

/* Table of sending stations mapping an ennumerated handle (station name) to a pretty
 * string, base station ID, and credit configuration table to all system-wide recipients.
 * Default programming is for XLP832. */
static struct fmn_sender def_src_cfg[XLP_MSG_HANDLE_MAX] __initdata = {
	[XLP_MSG_HANDLE_CPU0]    = { "core0",   XLP_STNID_CPU0,       1 },
	[XLP_MSG_HANDLE_CPU1]    = { "core1",   XLP_STNID_CPU1,       1 },
	[XLP_MSG_HANDLE_CPU2]    = { "core2",   XLP_STNID_CPU2,       1 },
	[XLP_MSG_HANDLE_CPU3]    = { "core3",   XLP_STNID_CPU3,       1 },
	[XLP_MSG_HANDLE_CPU4]    = { "core4",   XLP_STNID_CPU4,       1 },
	[XLP_MSG_HANDLE_CPU5]    = { "core5",   XLP_STNID_CPU5,       1 },
	[XLP_MSG_HANDLE_CPU6]    = { "core6",   XLP_STNID_CPU6,       1 },
	[XLP_MSG_HANDLE_CPU7]    = { "core7",   XLP_STNID_CPU7,       1 },
	[XLP_MSG_HANDLE_PCIE0]   = { "pcie0",   XLP_PCIE0_VC_BASE,    1 },
	[XLP_MSG_HANDLE_PCIE1]   = { "pcie1",   XLP_PCIE1_VC_BASE,    1 },
	[XLP_MSG_HANDLE_PCIE2]   = { "pcie2",   XLP_PCIE2_VC_BASE,    1 },
	[XLP_MSG_HANDLE_PCIE3]   = { "pcie3",   XLP_PCIE3_VC_BASE,    1 },
	[XLP_MSG_HANDLE_GDX]     = { "gdx",     XLP_GDX_VC_BASE,      1 },
	[XLP_MSG_HANDLE_REGX]    = { "regx",    XLP_INVALID_STATION,  0 },
	[XLP_MSG_HANDLE_RSA_ECC] = { "rsa",     XLP_RSA_ECC_VC_BASE,  1 },
	[XLP_MSG_HANDLE_CRYPTO]  = { "crypto",  XLP_CRYPTO_VC_BASE,   1 },
	[XLP_MSG_HANDLE_SRIO]    = { "srio",    XLP_INVALID_STATION,  0 },
	[XLP_MSG_HANDLE_CMP]     = { "cmp",     XLP_CMP_VC_BASE,      1 },
	[XLP_MSG_HANDLE_POE]     = { "poe",     XLP_POE_VC_BASE,      1 },
	[XLP_MSG_HANDLE_NAE_0]   = { "nae",     XLP_NET_TX_VC_BASE,   1 }
};

/* Table of receiving station groups mapping an ennumerated block (station type) to
 * a pretty string and a range of destination IDs. */
static struct fmn_receiver def_dest_cfg[XLP_MSG_BLK_MAX] __initdata = {
	[XLP_MSG_BLK_CPU]      = { "cpu",     XLP_CPU0_VC_BASE,     XLP_CPU7_VC_LIMIT,     1 },
	[XLP_MSG_BLK_POPQ]     = { "popq",    XLP_POPQ_VC_BASE,     XLP_POPQ_VC_LIMIT,     1 },
	[XLP_MSG_BLK_PCIE0]    = { "pcie0",   XLP_PCIE0_VC_BASE,    XLP_PCIE0_VC_LIMIT,    1 },
	[XLP_MSG_BLK_PCIE1]    = { "pcie1",   XLP_PCIE1_VC_BASE,    XLP_PCIE1_VC_LIMIT,    1 },
	[XLP_MSG_BLK_PCIE2]    = { "pcie2",   XLP_PCIE2_VC_BASE,    XLP_PCIE2_VC_LIMIT,    1 },
	[XLP_MSG_BLK_PCIE3]    = { "pcie3",   XLP_PCIE3_VC_BASE,    XLP_PCIE3_VC_LIMIT,    1 },
	[XLP_MSG_BLK_GDX]      = { "gdx",     XLP_GDX_VC_BASE,      XLP_GDX_VC_LIMIT,      1 },
	[XLP_MSG_BLK_REGX]     = { "regx",    XLP_INVALID_STATION,  XLP_INVALID_STATION,   0 },
	[XLP_MSG_BLK_RSA_ECC]  = { "rsa",     XLP_RSA_ECC_VC_BASE,  XLP_RSA_ECC_VC_LIMIT,  1 },
	[XLP_MSG_BLK_CRYPTO]   = { "crypto",  XLP_CRYPTO_VC_BASE,   XLP_CRYPTO_VC_LIMIT,   1 },
	[XLP_MSG_BLK_SRIO]     = { "srio",    XLP_INVALID_STATION,  XLP_INVALID_STATION,   0 },
	[XLP_MSG_BLK_CMP]      = { "cmp",     XLP_CMP_VC_BASE,      XLP_CMP_VC_LIMIT,      1 },
	[XLP_MSG_BLK_POE]      = { "poe",     XLP_POE_VC_BASE,      XLP_POE_VC_LIMIT,      1 },
	[XLP_MSG_BLK_NAE]      = { "nae",     XLP_NET_TX_VC_BASE,   XLP_NET_TX_VC_LIMIT,   1 },
	[XLP_MSG_BLK_FREEIN]   = { "freein",  XLP_NET_RX_VC_BASE,   XLP_NET_RX_VC_LIMIT,   1 }
};

static void __init update_fmn_config(int msg_blk, int msg_handle, int b_stid, int e_stid, int valid)
{
	if(msg_handle < XLP_MSG_HANDLE_INVALID) {
		def_src_cfg[msg_handle].base_vc = b_stid;
		def_src_cfg[msg_handle].valid = valid;
	}
	def_dest_cfg[msg_blk].b_stid = b_stid;
	def_dest_cfg[msg_blk].e_stid = e_stid;
	def_dest_cfg[msg_blk].valid = valid;
}

#ifndef NLM_HAL_XLP2
/* Determine present SOC accelerator blocks.  Adjust default sender and receiver tables
 * accordingly.  This function should be called before copying either array to fmn_node_cfg.
 */
static void __init update_eagle(void)
{
	unsigned int defeature;

	/* If CMP clock is enabled, check for defeatured pipes; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_CMP)) {
		defeature = bitcount((efuse_cfg1() >> 9) & 0xF);
		if(defeature == 4)
			update_fmn_config(XLP_MSG_BLK_CMP, XLP_MSG_HANDLE_CMP,
					XLP_INVALID_STATION, XLP_INVALID_STATION, 0);
		else if(defeature)
			update_fmn_config(XLP_MSG_BLK_CMP, XLP_MSG_HANDLE_CMP,
					XLP_CMP_VC_BASE, XLP_CMP_VC_LIMIT - defeature, 1);
	} else
		update_fmn_config(XLP_MSG_BLK_CMP, XLP_MSG_HANDLE_CMP,
				XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* If SAE clock is enabled, check for defeatured pipes; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_SAE)) {
		defeature = bitcount((efuse_cfg1() >> 14) & 0xFFF);
		if(defeature == 12)
			update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
					XLP_INVALID_STATION, XLP_INVALID_STATION, 0);
		else if(defeature)
			update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
					XLP_CRYPTO_VC_BASE, XLP_CRYPTO_VC_LIMIT - defeature, 1);
	} else
		update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
				XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* If RSA clock is enabled, check for defeatured pipes; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_RSA)) {
		defeature = bitcount(efuse_cfg2() & 0x1FF);
		if(defeature == 9)
			update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
					XLP_INVALID_STATION, XLP_INVALID_STATION, 0);
		else if(defeature)
			update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
					XLP_RSA_ECC_VC_BASE, XLP_RSA_ECC_VC_LIMIT - defeature, 1);
	} else
		update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
				XLP_INVALID_STATION, XLP_INVALID_STATION, 0);
}

static void __init update_storm(void)
{
	unsigned int defeature;

	/* Update number of PopQs for Storm */
	update_fmn_config(XLP_MSG_BLK_POPQ, XLP_MSG_HANDLE_INVALID,
			XLP_POPQ_VC_BASE, XLP_3XX_POPQ_VC_LIMIT, 1);

	/* Enable RegEx if SOC clock is enabled and defeature bit is not set */
	if(nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_REGEX_FAST) &&
	   nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_REGEX_SLOW) &&
	   (((efuse_cfg2() >> 11) & 0x1) == 0))
		update_fmn_config(XLP_MSG_BLK_REGX, XLP_MSG_HANDLE_REGX,
				XLP_3XX_REGEX_VC_BASE, XLP_3XX_REGEX_VC_LIMIT, 1);

	/* If RSA clock is enabled, check for defeatured pipes; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_RSA)) {
		defeature = bitcount(efuse_cfg2() & 0xF);
		if(defeature == 4)
			update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
					XLP_INVALID_STATION, XLP_INVALID_STATION, 0);
		else
			update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
					XLP_3XX_RSA_ECC_VC_BASE, XLP_3XX_RSA_ECC_VC_LIMIT - defeature, 1);
	} else
		update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
				XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* If SAE clock is enabled, check for defeatured pipes; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, DFS_DEVICE_SAE)) {
		defeature = bitcount((efuse_cfg1() >> 14) & 0xF);
		if(defeature == 4)
			update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
					XLP_3XX_CRYPTO_VC_BASE, XLP_3XX_CRYPTO_VC_LIMIT - defeature, 1);
		else
			update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
					XLP_INVALID_STATION, XLP_INVALID_STATION, 0);
	} else
		update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
				XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* Storm doesn't have compression engine - disable it */
	update_fmn_config(XLP_MSG_BLK_CMP, XLP_MSG_HANDLE_CMP,
			XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* Update POE and NAE station addresses for Storm */
	update_fmn_config(XLP_MSG_BLK_POE, XLP_MSG_HANDLE_POE,
			XLP_3XX_POE_VC_BASE, XLP_3XX_POE_VC_LIMIT, 1);
	update_fmn_config(XLP_MSG_BLK_NAE, XLP_MSG_HANDLE_NAE_0,
			XLP_3XX_NET_TX_VC_BASE, XLP_3XX_NET_TX_VC_LIMIT, 1);
	update_fmn_config(XLP_MSG_BLK_FREEIN, XLP_MSG_HANDLE_INVALID,
			XLP_3XX_NET_RX_VC_BASE, XLP_3XX_NET_RX_VC_LIMIT, 1);

	/* If SRIO is not defeatured, enable it */
	if(((efuse_cfg2() >> 10) & 0x1) == 0)
		update_fmn_config(XLP_MSG_BLK_SRIO, XLP_MSG_HANDLE_SRIO,
				XLP_3XX_SRIO_VC_BASE, XLP_3XX_B0_SRIO_VC_LIMIT, 1);
}
#endif

#ifndef NLM_HAL_XLP1
static void __init update_firefly(void)
{
	update_fmn_config(XLP_MSG_BLK_POPQ, XLP_MSG_HANDLE_INVALID,
			XLP_POPQ_VC_BASE, XLP_2XX_POPQ_VC_LIMIT, 1);

	/* Enable RegEx if SOC clock is enabled and defeature bit is not set */
	if(nlm_hal_get_soc_freq(NODE_0, XLP2XX_CLKDEVICE_RGXF) &&
	   nlm_hal_get_soc_freq(NODE_0, XLP2XX_CLKDEVICE_RGXS) &&
	   nlm_xlp2xx_has_regx())
		update_fmn_config(XLP_MSG_BLK_REGX, XLP_MSG_HANDLE_REGX,
				XLP_2XX_REGEX_VC_BASE, XLP_2XX_REGEX_VC_LIMIT, 1);

	/* If RSA clock is enabled, check not defeatured; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, XLP2XX_CLKDEVICE_RSA) && nlm_xlp2xx_has_rsa())
		update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
				XLP_2XX_RSA_ECC_VC_BASE, XLP_2XX_RSA_ECC_VC_LIMIT, 1);
	else
		update_fmn_config(XLP_MSG_BLK_RSA_ECC, XLP_MSG_HANDLE_RSA_ECC,
			XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* If SAE clock is enabled, check not defeatured; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, XLP2XX_CLKDEVICE_SAE) && nlm_xlp2xx_has_crypto())
		update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
				XLP_2XX_CRYPTO_VC_BASE, XLP_2XX_CRYPTO_VC_LIMIT, 1);
	else
		update_fmn_config(XLP_MSG_BLK_CRYPTO, XLP_MSG_HANDLE_CRYPTO,
				XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* If CMP clock is enabled, check not defeatured; Otherwise disable */
	if(nlm_hal_get_soc_freq(NODE_0, XLP2XX_CLKDEVICE_CMP) && nlm_xlp2xx_has_cmp())
		update_fmn_config(XLP_MSG_BLK_CMP, XLP_MSG_HANDLE_CMP,
				XLP_2XX_CDE_VC_BASE, XLP_2XX_CDE_VC_LIMIT, 1);
	else
		update_fmn_config(XLP_MSG_BLK_CMP, XLP_MSG_HANDLE_CMP,
				XLP_INVALID_STATION, XLP_INVALID_STATION, 0);

	/* Update POE and NAE station addresses for Firefly */
	update_fmn_config(XLP_MSG_BLK_POE, XLP_MSG_HANDLE_POE,
			XLP_2XX_POE_VC_BASE, XLP_2XX_POE_VC_LIMIT, 1);
	update_fmn_config(XLP_MSG_BLK_NAE, XLP_MSG_HANDLE_NAE_0,
			XLP_2XX_NET_TX_VC_BASE, XLP_2XX_NET_TX_VC_LIMIT, 1);
	update_fmn_config(XLP_MSG_BLK_FREEIN, XLP_MSG_HANDLE_INVALID,
			XLP_2XX_NET_RX_VC_BASE, XLP_2XX_NET_RX_VC_LIMIT, 1);
}
#endif

static int __init update_defaults(void)
{
	int i;
	unsigned int ncores = num_xlp_cores();
	unsigned int nvcs = MAX_VC_PERTHREAD * NLM_MAX_CPUS_PER_CORE * ncores;

#ifndef NLM_HAL_XLP2
	if(is_nlm_xlp8xx()) update_eagle();
	if(is_nlm_xlp3xx()) update_storm();
#endif
#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx()) update_firefly();
#endif

	FMN_DEBUG_PRINTK("Configuring FMN for %d cores, %d CPUs per core\n",
			ncores, num_xlp_threads());

	nvcs = MAX_VC_PERTHREAD * NLM_MAX_CPUS_PER_CORE * ncores;
	def_dest_cfg[XLP_MSG_BLK_CPU].e_stid = nvcs - 1;
	for(i = 0; i < NLM_MAX_CORES_PER_NODE; i++) {
		if(i >= ncores) {
			def_src_cfg[i].base_vc = XLP_INVALID_STATION;
			def_src_cfg[i].valid = 0;
		}
	}

	return 0;
}

#ifdef NLM_HAL_LINUX_KERNEL
static void __init fmn_validate_credit(int d_node)
{
	unsigned int credits, qsize, required_qsize, onchip;
	unsigned int s_stn, d_stn, s_node;
	struct fmn_receiver *receiver;
	struct fmn_sender *sender;

	nlm_print("Validating FMN credit allocation\n");

	for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_MAX; d_stn++) {
		receiver = &(nlm_node_cfg.fmn_cfg[d_node]->receivers[d_stn]);
		if(!receiver->valid)
			continue;
		qsize = receiver->q_size;
		onchip = XLP_FMN_ONCHIP_PAGE_SIZE * receiver->pages;
		credits = 0;
		/* Total credits from all source stations to this station */
		for(s_node = 0; s_node < max_nodes; s_node++) {
			for(s_stn = XLP_MSG_HANDLE_CPU0; s_stn < XLP_MSG_HANDLE_INVALID; s_stn++) {
				sender = &(nlm_node_cfg.fmn_cfg[s_node]->senders[s_stn]);
				credits += sender->credits[d_node][d_stn];
			}
		}

		/* Multiplier of 12 below accounts for least efficient message storage - single
		 * entry messages.  Modified the original code to not warn if the allocated spill
		 * memory is exactly big enough.
		 */
		if(onchip > credits)
			required_qsize = 0;
		else {
			required_qsize = (credits - onchip) * 12;
#if 0	// Round up required size to 4K
			if(required_qsize % XLP_FMN_Q_PAGE_SIZE)
				required_qsize += XLP_FMN_Q_PAGE_SIZE - (required_qsize % XLP_FMN_Q_PAGE_SIZE);
#endif
		}
		if(qsize < required_qsize) {
			nlm_print(KERN_WARNING "  Allocated credits to destination %6s = %4u,"
					" required spill size = %5u configured size = %5u (WARNING)\n",
					receiver->q_name, credits, required_qsize, qsize);
		 } else {
			nlm_print(KERN_DEBUG   "  Allocated credits to destination %6s = %4d,"
					" required spill size = %5u configured size = %5u\n", 
					receiver->q_name, credits, required_qsize, qsize);
		}
	}
}
#endif /* NLM_HAL_LINUX_KERNEL */

/*********************************************************************
 * nlm_hal_enable_vc_intr
 *
 * In xlp, there are 4 VC per cpu. Each vc can be configured to generate
 * an interrupt when message receive event happens.  Threshold interrupts
 * apply ONLY to the configured on-chip storage, not to VC spill.
 *
 * Threshold types: LVL_INT_DISABLE, LVL_INT_LOW_WM, LVL_INT_HIGHWM
 * Low watermark:   LWM_EMPTY, LWM_1_4_FULL, LWM_1_2_FULL, LWM_3_4_FULL, LWM_NON_FULL
 * High watermark: 	HWM_NON_EMPTY, HWM_1_4_FULL, HWM_1_2_FULL, HWM_3_4_FULL, HWM_FULL
 * Timer types:     TMR_INT_DISABLE, TMR_INT_CONSUMER, TMR_INT_PRODUCER
 * Timer values:    TMR_2E8_CYCLES - TMR_2E22_CYCLES (even numbers only)
 * Timer tick is in CPU cycles and will be between the programmed value and 2x the value
 * At 1200 MHz, reasonable timer values are from 2^14 - 2^18.  Lower values
 * increase interrupt overhead without much or any improvement in latency, higher
 * values reduce throughput AND take latency north of 1ms.
 ********************************************************************/
void nlm_hal_enable_vc_intr(int node, int vc,
		int level_type, int thresh_val, int tmr_type, int tmr_val)
{
	uint64_t val = nlm_hal_read_outq_config(node, vc);
	val &= ~((0x7ULL<<56) | (0x3ULL<<54) | (0x7ULL<<51) | (0x3ULL<<49));
	val |= ((uint64_t)(thresh_val) << 56) | ((uint64_t)(level_type) << 54);
	val |= ((uint64_t)(tmr_val) <<51 ) | ((uint64_t)(tmr_type) << 49);
	nlm_hal_write_outq_config(node, vc, val);
	FMN_DEBUG_PRINTK("VC %3d interrupt: level type = %d, threshold = %d, "
			"timer type = %d, value = %d\n",
			vc, level_type, thresh_val, tmr_type, tmr_val);
}

/*********************************************************************
 * nlm_hal_disable_vc_intr
*********************************************************************/
void nlm_hal_disable_vc_intr(int node, int vc)
{
	uint64_t val = nlm_hal_read_outq_config(node, vc);
	val &= ~((0x3ULL<<54) | (0x3ULL<<49));
	nlm_hal_write_outq_config(node, vc, val);
	FMN_DEBUG_PRINTK("VC %3d interrupt: Disabled\n", vc);
}

/*********************************************************************
 * nlm_hal_set_fmn_interrupt
 *
 * setup cp2 msgconfig register for fmn interrup vector as 6
 *
 ********************************************************************/
void nlm_hal_set_fmn_interrupt(int irq)
{
	uint32_t val;
	/* Need write interrupt vector to cp2 msgconfig register */

	val =  _read_32bit_cp2_register(XLP_MSG_CONFIG_REG);
	val &= ~(0x3f << 16);
	val |= (irq << 16) | (0xf << 22);	// Enable all four VCs to generate interrupts (redundant?)
	_write_32bit_cp2_register(XLP_MSG_CONFIG_REG, val);
}

/* Populate an array tmp_credit_array (re-used) with the credits to every possible destination
 * for one particular sender (b_stid).  Assume other code has handled invalid / disabled /
 * non-existent stations and allocating credits to self - so no tests for that here. */
static void __init fmn_update_credit(int s_node, int s_stn, int d_node)
{
	unsigned int d_stn, did, credits;
	struct fmn_receiver *receiver;
	struct fmn_sender *sender = &(nlm_node_cfg.fmn_cfg[s_node]->senders[s_stn]);
	unsigned int cpus_per_core = num_xlp_threads();

	/* Zero out the temporary array since it gets re-used */
	for(did = 0; did <= max_qs; did++)
		tmp_credit_arr[did] = 0;

	/* Now loop through all destinations block by block and set credits to each output queue
	 * in each block. */
	for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_MAX; d_stn++) {
		receiver = &(nlm_node_cfg.fmn_cfg[s_node]->receivers[d_stn]);
		if(!receiver->valid && !receiver->pages)
			continue;
		credits = sender->credits[d_node][d_stn];
		for(did = receiver->b_stid; did <= receiver->e_stid; did++) {
#ifndef OLD_SCHOOL
			if(did < XLP_POPQ_VC_BASE) {
				/* Handle CPU exceptions */
				/* Single thread / control plane cases - no credits to VCs on CPUs 1-3 of a core */
				if((cpus_per_core == 1) && ((did % 16) > 3))
					continue;
				/* CPU online_mask is not all CPUs (e.g. not all 'f's) */
#ifdef NLM_HAL_LINUX_KERNEL
				if(!cpumask_test_cpu(did >> 2, &fdt_cpumask))
#else
				/* Firmware - assume only CPU 0 is online */
				if(did > 3)
#endif
					continue;
			}
#endif
			tmp_credit_arr[did] = credits;
		}
	}
}

static void __init nlm_hal_write_credit(int node, uint64_t src, uint64_t dst, uint64_t credits)
{
	uint64_t regaddr = nlh_qid_to_virt_addr(node, XLP_CREDIT_CONFIG_REG, 0); 
	uint64_t value = (((src) & 0xfff) | (((dst) & 0xfff) << 12) | (((credits) & 0xffff) << 24));

	nlh_write_cfg_reg64(regaddr, value);
	FMN_DEBUG_VERBOSE("  src %3d dest %3d credits %d\n", (int)src, (int)dst, (int)credits);
}

static void __init nlm_hal_write_fmn_credit(int s_node) 
{
	unsigned int did, s_stn, d_node;
	unsigned int credits;
	struct fmn_sender *sender;

	FMN_DEBUG_VERBOSE("[%s]\n", __FUNCTION__);

	/* Loop through valid senders.  Build a table for credits from this sender to all
	 * valid recipients then write the credits to this sender's credit table. */
	for(d_node = 0; d_node < max_nodes; d_node++) {
		for(s_stn = XLP_MSG_HANDLE_CPU0; s_stn < XLP_MSG_HANDLE_INVALID; s_stn++) {
			sender = &(nlm_node_cfg.fmn_cfg[s_node]->senders[s_stn]);
			if(!sender->valid)
				continue;
			fmn_update_credit(s_node, s_stn, d_node);
			for (did = 0; did <= max_qs; did++) {
				if((credits = tmp_credit_arr[did])) {
					if(s_stn <= XLP_MSG_HANDLE_CPU7)
						credits--;	// Must write (configured_credits - 1) to CPU credit tables
					nlm_hal_write_credit(s_node, sender->base_vc, ((d_node << 10) | did), credits);
				}
			}
		}
	}
}

static void __init fmn_update_qsize(int d_node)
{
	unsigned int did, d_stn;
	struct fmn_receiver *receiver;
	unsigned int cpus_per_core = num_xlp_threads();

	/* Zero out the temporary arrays */
	for(did = 0; did <= max_qs; did++) {
		tmp_spill_arr[did] = 0;
		tmp_pages_arr[did] = 0;
	}

	for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_MAX; d_stn++) {
		receiver = &(nlm_node_cfg.fmn_cfg[d_node]->receivers[d_stn]);
		if(!receiver->valid)
			continue;
		for(did = receiver->b_stid; did <= receiver->e_stid; did++) {
#ifndef OLD_SCHOOL
			if(did < XLP_POPQ_VC_BASE) {
				/* Handle CPU exceptions */
				/* Single thread / control plane cases - no credits to VCs on CPUs 1-3 of a core */
				if((cpus_per_core == 1) && ((did % 16) > 3))
					continue;
				/* CPU online_mask is not all CPUs (e.g. not all 'f's) */
#ifdef NLM_HAL_LINUX_KERNEL
				if(!cpumask_test_cpu(did >> 2, &fdt_cpumask))
#else
				/* Firmware - assume only CPU 0 is online */
				if(did > 3)
#endif
					continue;
			}
#endif
			tmp_spill_arr[did] = receiver->q_size;
			tmp_pages_arr[did] = receiver->pages;
		}
	}
}

/*********************************************************************
 * nlm_hal_setup_outq
 *
 * In xlp, there is 1024 receive message queues for fmn network. The
 * queue, allocated to cpu and high speed IO device, identified by
 * their vc number. When A send B a FMN message, receive VC is dest
 * number A need addressing. This function is to config each queue with
 * initial defaule value
 *
 * Total spill size for each Q is 16KB
 * This allows for 1024 q entries with 16B of entry size
 * This assumes credits across all sending agents to this queue is < 1024
 *
 * In xlp3xx/2xx there are 512 queues.  xlp4xx on-chip storage is 32K
 * messages.  xlp3xx is 16K and xlp2xx is 8K.
 ********************************************************************/
static int __init nlm_hal_setup_outq(int d_node)
{
	uint32_t d_stn, qid;
	uint64_t val;
	uint64_t q_spill_start_page, q_spill_pages;
	uint32_t q_ram_base = 0;
	uint32_t q_ram_start_page;
	int cnt = 0, cnt_limit;
	struct fmn_receiver *receiver;
	struct fmn_cfg *fmn_config = nlm_node_cfg.fmn_cfg[d_node];
	uint64_t spill_base = fmn_config->fmn_spill_base;		 /*fmn_spill_mem_addr; */
	uint32_t spill_size = fmn_config->fmn_spill_size;

	FMN_DEBUG_VERBOSE("[%s]\n", __FUNCTION__);

	/* Define limit of on-chip message storage SRAM */
	if(is_nlm_xlp2xx() )
		cnt_limit = 256;
	else if(is_nlm_xlp3xx() )
		cnt_limit = 512;
	else
		cnt_limit = 1024;

	fmn_config->spill_base_cur = spill_base;
	fmn_config->q_ram_base_cur = q_ram_base;

	fmn_update_qsize(d_node);

	for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_MAX; d_stn++) {
		receiver = &(fmn_config->receivers[d_stn]);
		if(!receiver->valid)
			continue;
		for(qid = receiver->b_stid; qid <= receiver->e_stid; qid++) {
			if(!tmp_pages_arr[qid]) {
				nlm_hal_write_outq_config(d_node, qid, 0);
				FMN_DEBUG_PRINTK("Output Queue %4d (%6s): Disabled\n", qid, receiver->q_name);
				continue;
			}

			/* Enable output queue, do not enable interrupts (will happen elsewhere if needed) */
			val = OUTQ_EN;

			/***************************************************************
			 * Configuration of on-chip RAM area
			 **************************************************************
			 */

			/* Test if on-chip area for this queue would cross 1K boundary, round up if so */
			if(((cnt % XLP_FMN_ONCHIP_PAGE_SIZE) + tmp_pages_arr[qid]) > XLP_FMN_ONCHIP_PAGE_SIZE)
				cnt += (XLP_FMN_ONCHIP_PAGE_SIZE - (cnt % XLP_FMN_ONCHIP_PAGE_SIZE));

			q_ram_base = cnt * XLP_FMN_ONCHIP_PAGE_SIZE;	/* 32 entries per on-chip RAM page */
			cnt += tmp_pages_arr[qid];

			if(cnt > cnt_limit) {
				nlm_print("FMN Error: On-chip SRAM exhausted on node %d!\n", d_node);
				return -1;
			}

			val |= ( ((q_ram_base >> 10) & 0x1f) << 10);	/* [14:10] of q_ram_base */

			q_ram_start_page = (q_ram_base >> 5) & 0x1f;	/* [9:5] of q_ram_base */
			val |= (q_ram_start_page << 0);					
			val |= ( (q_ram_start_page + tmp_pages_arr[qid] - 1) << 5) ;

			FMN_DEBUG_PRINTK("Output Queue %4d (%6s): SRAM = 0x%04X-0x%04X", qid, receiver->q_name,
					(int)((((val >> 10) & 0xf) << 10) | ((val & 0x1f) << 5)),
					(int)(((((val >> 10) & 0xf) << 10) | (((val >> 5) & 0x1f) << 5)) + 0x1f));

			/* Configure spill if spill base is valid and qsize is > 0 */
			/* pages in 4K units */
			q_spill_pages = tmp_spill_arr[qid] / XLP_FMN_Q_PAGE_SIZE;
			if (fmn_config->fmn_spill_base && q_spill_pages) {
				val |= SPILL_EN;

				/***************************************************************
				 * Configuration of spill area
				 **************************************************************
				 */

				/* if spill_start + qsize crosses 2^18 boundary, configuration will be wrong as 
				 only 17-12 bits only considered for spill last */
				if(((spill_base % XLP_FMN_MAX_Q_SIZE) + tmp_spill_arr[qid]) > XLP_FMN_MAX_Q_SIZE)
					spill_base += XLP_FMN_MAX_Q_SIZE - (spill_base % XLP_FMN_MAX_Q_SIZE);
		
				val |= ( ((spill_base >> 18) & 0x3fffff) << 27); /* [39:18] of q_spill_base */

				q_spill_start_page = (spill_base >> 12) & 0x3f; /* [17:12] of q_spill_base */
				val |= (q_spill_start_page << 15);
				val |= ( (q_spill_start_page + q_spill_pages - 1) << 21);

				spill_base += tmp_spill_arr[qid];
				FMN_DEBUG_PRINTK(" spill = 0x%010llX-0x%010llX",
						(unsigned long long)((((val >> 27) & 0x3fffff) << 18)
										   | (((val >> 15) & 0x3f) << 12)),
						(unsigned long long)(((((val >> 27) & 0x3fffff) << 18)
										    | (((val >> 21) & 0x3f) << 12)) + 0xfff));
			}
			FMN_DEBUG_PRINTK("\n");

			/* Write to the configuration register */
			nlm_hal_write_outq_config(d_node, qid, val);
		}
	}

	fmn_config->spill_base_cur = spill_base;
	fmn_config->q_ram_base_cur = q_ram_base;
 
	if ((spill_base - fmn_config->fmn_spill_base) > spill_size) {
		nlm_print(KERN_ERR "FMN Error: Total DRAM spill exceeds the limit\n");
		return -1;
	}
#if defined(NLM_HAL_LINUX_KERNEL) || defined(FMN_DEBUG)
	nlm_print(KERN_DEBUG "Node %d on-chip SRAM utilization: %d of maximum %d\n",
			d_node, cnt * XLP_FMN_ONCHIP_PAGE_SIZE, cnt_limit * XLP_FMN_ONCHIP_PAGE_SIZE);
	if (spill_size)
		nlm_print(KERN_DEBUG "DRAM spill utilization 0x%x of maximum 0x%x\n",
				(unsigned int)(spill_base - fmn_config->fmn_spill_base), (unsigned int)spill_size);
#endif
	return 0;
}

static void __init parse_defaults(int d_node, struct fmn_cfg *fmn_config, void *fdt) {
	int nodeoffset = 0, plen;
	char fmn_cfg_str[64];
  	uint32_t *pval;
  	unsigned int qsize, credits;

	fmn_config->fmn_default_credits = XLP_FMN_DEFAULT_CREDITS/max_nodes;
	fmn_config->fmn_default_qsize = XLP_FMN_DEFAULT_QUEUE_SIZE;
	fmn_config->fmn_default_onchip_pages = XLP_FMN_DEFAULT_ONCHIP_PAGES;

	sprintf(fmn_cfg_str,"/soc/fmn@node-%d", d_node);
	nodeoffset = fdt_path_offset(fdt, fmn_cfg_str);
	if(nodeoffset < 0) {
#ifdef NLM_HAL_LINUX_KERNEL
		nlm_print(KERN_WARNING "No 'fmn@node-%d' param in dtb \n", d_node);
		nlm_print(KERN_WARNING "  Using defaults - on-chip messages %d qsize %d credits %d\n",
				fmn_config->fmn_default_onchip_pages * XLP_FMN_ONCHIP_PAGE_SIZE,
				fmn_config->fmn_default_qsize,
				fmn_config->fmn_default_credits);
#endif
		return;
	}

#ifdef NLM_HAL_LINUX_KERNEL		// No spill in u-boot
	pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "fmn-spill-mem-base", &plen);
	if(pval != NULL) {
		fmn_config->fmn_spill_base = fdt64_to_cpu(*(unsigned long long *)pval);

		pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "fmn-spill-mem-size", &plen);
		if(pval != NULL)
			fmn_config->fmn_spill_size = fdt64_to_cpu(*(unsigned long long *)pval) ;

		if(fmn_config->fmn_spill_size) {
			if (!fmn_config->fmn_spill_base) {
				fmn_config->fmn_spill_base = nlm_spill_alloc(d_node, (fmn_config->fmn_spill_size));
				if (!fmn_config->fmn_spill_base) {
					nlm_print("Node %d FMN spill_mem alloc failed \n", d_node);
					fmn_config->fmn_spill_size = 0;
				}
			}
		} else {
			fmn_config->fmn_spill_base = 0ULL;
		}
	}
#endif /* NLM_HAL_LINUX_KERNEL */

	pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "default-queue-size", &plen);
	if (pval != NULL) {
		qsize = fdt32_to_cpu(*(unsigned int *)pval);
		fmn_config->fmn_default_qsize = qsize;
	}

	pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "default-credits", &plen);
	if (pval != NULL) {
		credits = fdt32_to_cpu(*(unsigned int *)pval);
		fmn_config->fmn_default_credits = credits / max_nodes;
	}

#ifdef FMN_DEBUG
	nlm_print("Node %d default on-chip messages %d qsize %d credits %d\n", d_node,
			fmn_config->fmn_default_onchip_pages * XLP_FMN_ONCHIP_PAGE_SIZE,
			fmn_config->fmn_default_qsize,
			fmn_config->fmn_default_credits);
#endif
}

static void __init parse_queue_config(int d_node, struct fmn_cfg *fmn_config, void *fdt)
{
	int nodeoffset, d_stn;
	char fmn_cfg_str[32];
	int plen;
	uint32_t *pval;
	struct fmn_receiver *receiver;
	unsigned int qsize;
#ifndef OLD_SCHOOL
	unsigned int onchip, num_ctxt;
#endif

	for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_FREEIN; d_stn++) {
		receiver = &(fmn_config->receivers[d_stn]);
		if(!receiver->valid)
			continue;

		/* Get on-chip and spill queue size for this station type, if defined in dts.
		 * Otherwise, use defaults.
		 */
		sprintf(fmn_cfg_str, "/soc/fmn@node-%d/q-config/%s", d_node, receiver->q_name);
		nodeoffset = fdt_path_offset(fdt, fmn_cfg_str);
		if(nodeoffset < 0) {
			receiver->q_size = fmn_config->fmn_default_qsize;
			receiver->pages = fmn_config->fmn_default_onchip_pages;
			continue;
		}

#ifndef OLD_SCHOOL
		/* New parameter - check for override of on-chip FMN storage.  Zero means disable
		 * receive mailbox completely.
		 */
		pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "onchip", &plen);
		if(pval) {
			onchip = fdt32_to_cpu(*(unsigned int *)pval);
			if(!onchip) {
				nlm_print(KERN_DEBUG "Disabling*** output queues for node %d %6s\n",
						d_node, receiver->q_name);
				continue;
			}
			if(onchip > XLP_FMN_MAX_ONCHIP_SIZE) {
				nlm_print("Limiting specified on-chip size %d for %s "
				"(exceeds %d limit)\n", (int)qsize, receiver[d_stn].q_name,
				(int)XLP_FMN_MAX_ONCHIP_SIZE);
				onchip = XLP_FMN_MAX_ONCHIP_SIZE;
			}
			if( (onchip % XLP_FMN_ONCHIP_PAGE_SIZE) != 0) {
				nlm_print("Rounding specified value %d on-chip for node %d %6s to %d\n",
					onchip, d_node, receiver->q_name,
					onchip + (XLP_FMN_ONCHIP_PAGE_SIZE - (onchip % XLP_FMN_ONCHIP_PAGE_SIZE)));
					onchip += (XLP_FMN_ONCHIP_PAGE_SIZE - (onchip % XLP_FMN_ONCHIP_PAGE_SIZE));
			}
			nlm_print(KERN_DEBUG "Overriding default on-chip for node %d %6s"
					" -> %d messages\n", d_node, receiver->q_name, onchip);
			receiver->pages = onchip / XLP_FMN_ONCHIP_PAGE_SIZE;
		} else
#endif
			receiver->pages = fmn_config->fmn_default_onchip_pages;

		pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "spill", &plen);
		if(pval) {
			qsize = fdt32_to_cpu(*(unsigned int *)pval);
			if(qsize > XLP_FMN_MAX_Q_SIZE) {
				nlm_print(KERN_WARNING "Limiting specified queue size %d for %s "
				"(exceeds %d limit)\n", (int)qsize, receiver[d_stn].q_name,
				(int)XLP_FMN_MAX_Q_SIZE);
				qsize = XLP_FMN_MAX_Q_SIZE;
			}
			if((qsize % XLP_FMN_Q_PAGE_SIZE) != 0) {
				nlm_print("Rounding specified value %d queue size for node %d %6s to %d\n",
					(int)qsize, d_node, receiver->q_name,
					(int)(qsize + (XLP_FMN_Q_PAGE_SIZE - (qsize % XLP_FMN_Q_PAGE_SIZE))));
					qsize += (XLP_FMN_Q_PAGE_SIZE - (qsize % XLP_FMN_Q_PAGE_SIZE));
			}
			receiver->q_size = qsize;
			nlm_print(KERN_DEBUG "Overriding default spillsz for node %d %6s -> %d bytes\n",
					d_node, receiver->q_name, qsize);
		} else
			receiver->q_size = fmn_config->fmn_default_qsize;

#ifndef OLD_SCHOOL
		/* New parameter - check for override of nae TX contexts used */
		if(d_stn == XLP_MSG_BLK_NAE) {
			pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "contexts", &plen);
			if(pval) {
				num_ctxt = fdt32_to_cpu(*(unsigned int *)pval);
				if((receiver->b_stid + num_ctxt) < receiver->e_stid) {
					nlm_print(KERN_DEBUG "Disabling*** output queues for node %d    nae tx contexts %d - %d\n",
							d_node, receiver->b_stid + num_ctxt, receiver->e_stid);
					receiver->e_stid = receiver->b_stid + num_ctxt - 1;
				}
			}
		}
#endif
	}

	/* Handle NAE Free-in LIFOs as an exception */
	receiver = &(fmn_config->receivers[XLP_MSG_BLK_FREEIN]);
#ifndef OLD_SCHOOL
	receiver->q_size = 0;
#else
	receiver->q_size = fmn_config->fmn_default_qsize;
#endif
	receiver->pages = fmn_config->fmn_default_onchip_pages;
}

/* Calculate credits for a given sender-receiver pair, taking into account a number of
 * exceptions, highlighted in the code comments below.  Returns the configured default
 * number of credits if none of the exceptions are met.
 */
static unsigned int __init calc_credits(struct fmn_cfg *fmn_config, int d_stn, int s_node, int s_stn)
{
#ifndef OLD_SCHOOL
	int s_d_stn;
	struct fmn_receiver *s_receiver;
	struct fmn_sender *sender = &(nlm_node_cfg.fmn_cfg[s_node]->senders[s_stn]);

	/* No credits from senders whose on-chip size is zero (disabled) */
	for (s_d_stn = XLP_MSG_BLK_CPU; s_d_stn < XLP_MSG_BLK_MAX; s_d_stn++) {
		s_receiver = &(nlm_node_cfg.fmn_cfg[s_node]->receivers[s_d_stn]);
		if(!s_receiver->valid)
			continue;
		if((s_receiver->b_stid == sender->base_vc) && !s_receiver->pages)
			return 0;
	}

	/* No credits from self, except for CPU (needed for PopQ) */
	if((sender->base_vc == fmn_config->receivers[d_stn].b_stid)
			&& (d_stn > XLP_MSG_BLK_CPU))
		return 0;

	/* NAE only needs credits to send to CPUs (freeback messages).
	 * all RX packets come from POE
	 */
	if((s_stn == XLP_MSG_HANDLE_NAE_0) && (d_stn != XLP_MSG_BLK_CPU))
		return 0;

	/* Only CPU needs credits to free-in queues, and only need minimal number.  Using 4 per core */
	if(d_stn == XLP_MSG_BLK_FREEIN) {
		if(s_stn <= XLP_MSG_HANDLE_CPU7)
			return 4;
		else
			return 0;
	}
#endif
	return fmn_config->fmn_default_credits;
}

static void __init parse_credit_config(void *fdt)
{
	int nodeoffset, s_node, s_stn, d_node, d_stn;
	char fmn_cfg_str[64];
	int plen;
	uint32_t *pval;
	struct fmn_sender *sender;
	struct fmn_receiver *receiver;
	struct fmn_cfg *fmn_config;
	unsigned int credits;

	for(d_node = 0; d_node < max_nodes; d_node++) {
		fmn_config = nlm_node_cfg.fmn_cfg[d_node];
		for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_MAX; d_stn++) {
			receiver = &(fmn_config->receivers[d_stn]);
			if(!receiver->valid || !receiver->pages)
				continue;

			/* Get credits to this station type, if defined in dts.
			 * Otherwise, use defaults.
			 */
			for(s_node = 0; s_node < max_nodes; s_node++) {
				sprintf(fmn_cfg_str, "/soc/fmn@node-%d/q-config/%s/credits-from-node-%d",
						d_node, receiver->q_name, s_node);
				nodeoffset = fdt_path_offset(fdt, fmn_cfg_str);
				for(s_stn = XLP_MSG_HANDLE_CPU0; s_stn < XLP_MSG_HANDLE_INVALID; s_stn++) {
					sender = &(nlm_node_cfg.fmn_cfg[s_node]->senders[s_stn]);
					if(!sender->valid)
						continue;
					if(nodeoffset) {
						pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, sender->q_name, &plen);
						if(pval) {
							credits = fdt32_to_cpu(*(unsigned int *)pval);
							sender->credits[d_node][d_stn] = credits;
							nlm_print(KERN_DEBUG "Overriding default credits for node %d %6s"
									" -> %d credits to node %d %s\n", s_node, sender->q_name,
									credits, d_node, receiver->q_name);
							continue;
						}
					}
					/* Assign default credits */
					sender->credits[d_node][d_stn] = calc_credits(fmn_config, d_stn, s_node, s_stn);
				}
			}
		}
	}
}

/* Configuration of FMN based on FDT parameters.  Config entries are all based on destinations
 * - i.e. outer loop through destination nodes then inner loop through destination stations
 * (in the parse_queue_config function above).  Must calculate all receive queues first, then do
 * credits.  Since credits are source-based, each destination has a table of credits to that
 * destination from other sources.  If the table is not present or if the source is not listed in
 * the table, that source is assigned the default credits to the destination.
 */
static int __init parse_fdt_fmn_config(void *fdt)
{
	int nodeoffset, d_node;
	char fmn_cfg_str[16];
	int plen;
	uint32_t *pval;
	struct fmn_cfg *fmn_config;

	max_nodes = 1;
	sprintf(fmn_cfg_str,"/soc/nodes");
	nodeoffset = fdt_path_offset(fdt, fmn_cfg_str);
	if(nodeoffset >= 0) {
		pval = (uint32_t *)fdt_getprop(fdt, nodeoffset, "num-nodes", &plen);
		if(pval != NULL) {
			max_nodes = fdt32_to_cpu(*(unsigned int *)pval);
		}
	}

	nlm_node_cfg.num_nodes = max_nodes;

	for(d_node = 0; d_node < max_nodes; d_node++) {
		fmn_config = nlm_malloc(sizeof(struct fmn_cfg));
		if (fmn_config == NULL) {
			nlm_print("nlm_malloc failed for node %d\n", d_node);
			return -1;
		}
		nlm_node_cfg.fmn_cfg[d_node] = fmn_config;
		memset(fmn_config, 0, sizeof(struct fmn_cfg));
		memcpy(fmn_config->senders, def_src_cfg, sizeof(def_src_cfg));
		memcpy(fmn_config->receivers, def_dest_cfg, sizeof(def_dest_cfg));

		parse_defaults(d_node, fmn_config, fdt);
		parse_queue_config(d_node, fmn_config, fdt);
	}

	parse_credit_config(fdt);
	return 0;
}

#ifdef FMN_DEBUG_MORE
static void __init dump_fmn_config(int s_node)
{
	int d_stn, s_stn, d_node;
	struct fmn_sender *sender;
	struct fmn_receiver *receiver;

	nlm_print("*** FMN Configuration Summary for node %d ***\n", s_node);

	nlm_print("Sending station table:\n");
	for(s_stn = XLP_MSG_HANDLE_CPU0; s_stn < XLP_MSG_HANDLE_INVALID; s_stn++) {
		sender = &(nlm_node_cfg.fmn_cfg[s_node]->senders[s_stn]);
		if (!sender->valid)
			continue;
		nlm_print("  Station %6s: Base station ID %4d\n", sender->q_name, sender->base_vc);
		for(d_node = 0; d_node < max_nodes; d_node++) {
			for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_MAX; d_stn++) {
				receiver = &(nlm_node_cfg.fmn_cfg[s_node]->receivers[d_stn]);
				if(!receiver->valid)
					continue;
				nlm_print("    Credits to node %d station group %6s = %d\n", d_node,
						receiver->q_name, sender->credits[d_node][d_stn]);
			}
		}
	}

	nlm_print("Receiving station table:\n");
	for(d_stn = XLP_MSG_BLK_CPU; d_stn < XLP_MSG_BLK_MAX; d_stn++) {
		receiver = &(nlm_node_cfg.fmn_cfg[s_node]->receivers[d_stn]);
		if (!receiver->valid)
			continue;
		nlm_print("  Station %6s: Station IDs %4d - %4d, on-chip %3d messages, spill %d bytes\n",
				receiver->q_name, receiver->b_stid, receiver->e_stid,
				receiver->pages * XLP_FMN_ONCHIP_PAGE_SIZE, receiver->q_size);
	}
}
#endif

int get_dom_fmn_node_ownership(void *fdt, int dom_id)
{
	uint32_t owner_mask;

	owner_mask = get_dom_owner_mask(fdt, dom_id, "fmn");

	return owner_mask;
}

/**
* @brief nlm_hal_fmn_init function Initializes FMN (outpu Queues and Credit registers)
*
* @param [in]  fdt		:Pointer to copy of FDT in kernel memory
* @param [in]  node		:Node to initialize
*
* @return
*  - Returns no value.
*
* @note
*    1. This function must be the first to be called before any FMN HAL APIs.
* @n
*    2. This function is typically called twice (once from U-boot and once from OS). This is due to
*       Netlogic's SDK convention that only 1 cpu is running while in U-boot and potentially more than 
*       one cpu running while in OS and also the requirement that Credit configuration can only
*       happen after the cpu is out of reset.
* @n
*    3. The credits between any source and any destination is chosen to be same for simplification.
* 
* @ingroup hal_fmn
*
*/
void __init nlm_hal_fmn_init(void *fdt, int node)
{
	nlm_print("Configuring XLP Fast Message Network\n");

	if (is_nlm_xlp3xx() || is_nlm_xlp2xx()) {
		max_qs = XLP_3XX_NET_VC_LIMIT;
	}
	else {
		max_qs = XLP_NET_VC_LIMIT;
	}

	/* Malloc memory for working arrays */
	tmp_credit_arr = nlm_malloc(sizeof(unsigned int) * (max_qs + 1));
	tmp_spill_arr  = nlm_malloc(sizeof(unsigned int) * (max_qs + 1));
	tmp_pages_arr  = nlm_malloc(sizeof(unsigned int) * (max_qs + 1));

	if((tmp_credit_arr == NULL) || (tmp_spill_arr == NULL) || (tmp_pages_arr == NULL)) {
		nlm_print(KERN_ERR "nlm_malloc failed - unable to configure FMN\n");
		return;
	}

	/* Set-up for actual silicon */
	if (update_defaults() < 0)
		return;

	/* Parse FDT settings and overrides */
	if (parse_fdt_fmn_config(fdt) < 0)
		return;

#ifdef FMN_DEBUG_MORE
	dump_fmn_config(node);
#endif

	/* verify out_q config */
	if(nlm_hal_setup_outq(node) < 0)
		return;

#ifdef NLM_HAL_LINUX_KERNEL
	/* Verify credit config */
	fmn_validate_credit(node);
#endif

	nlm_hal_write_fmn_credit(node);

	nlm_free(tmp_credit_arr);
	nlm_free(tmp_spill_arr);
	nlm_free(tmp_pages_arr);
}

#ifdef NLM_HAL_LINUX_KERNEL 
#include <linux/types.h>
#include <linux/module.h>
EXPORT_SYMBOL(nlm_hal_disable_vc_intr);
EXPORT_SYMBOL(nlm_hal_enable_vc_intr);
#endif
