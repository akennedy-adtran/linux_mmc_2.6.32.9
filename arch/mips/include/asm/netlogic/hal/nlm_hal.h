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

#ifndef __NLM_HAL_H__
#define __NLM_HAL_H__

/* Added to enable dependency checking for other SDK pieces */
#define NLM_HAL_SDK_VERSION_230
#define NLM_HAL_SDK_VERSION_231

#include "nlm_hal_macros.h"
#include "nlm_hal_xlp_dev.h"
#include "nlm_nae.h"
#include "nlm_hal_sys.h"
#include "linux/mii.h"

struct nlm_netl_proc_info {
	uint16_t proc_id;
	uint16_t chipid;		/*example: xlp832=>0x8084, xlp316=>0x3044, xlp208=>0x2024 etc */
	uint16_t revision;
	char cpu_info_str[26];
};

struct nlm_hal_mii_info{
	int16_t speed;
	uint16_t duplex;
	uint16_t link_stat;
	uint16_t media_detect;
	uint16_t phyaddr;
};

/**
* @brief Used by function ::get_phy_info and various internal PHY manufacturer-specific functions.
* @ingroup hal
*/
struct nlm_hal_ext_phy {
	char name[16];
	uint32_t phy_idfer;
	uint16_t ext_mdio_bus;
	uint8_t phy_mode;
	uint8_t phy_addr;
	int inf;
	int (*phy_get_status)(struct nlm_hal_ext_phy* ext_phy,
			struct nlm_hal_mii_info *mii_info, int node);
	void (*start_phy_an)(struct nlm_hal_ext_phy* ext_phy, int node);
	int (*ext_phy_init)(struct nlm_hal_ext_phy* ext_phy, int node);
	/*(void) (*dump_regs)(void); */
};

extern void nlm_hal_init(void);

extern int nlm_hal_is_xlp_le(void);
extern void nlm_hal_xlp_pcie_rc_init(void);

extern void nlm_hal_cpld_init(int node);
/* #define SKIP_INTERFACE_TYPE_FROMCPLD 1 */
extern int nlm_get_interface_type(int node, int slot);
extern int is_ilk_card_onslot(int);
extern int is_xlp_evp1(void);
extern int is_xlp_evp2(void);
extern int nlm_xlp_boardver(void);
extern int nlm_xlp_cpldver(void);
extern void sgmii_scan_phys(int node);

#ifndef NLM_HAL_LINUX_KERNEL
extern void enable_cpus(unsigned int node, unsigned long thread_bitmask, unsigned long park_func);
#endif /* #ifndef NLM_HAL_LINUX_KERNEL */

extern uint16_t nlm_hal_read_16bit_reg(uint64_t base, uint32_t index);
extern void nlm_hal_write_16bit_reg(uint64_t base, uint32_t index, uint16_t val);
extern uint32_t nlm_hal_read_32bit_reg(uint64_t base, int index);
extern void nlm_hal_write_32bit_reg(uint64_t base, int index, uint32_t val);
extern uint64_t nlm_hal_read_64bit_reg(uint64_t base, int index);
extern void nlm_hal_write_64bit_reg(uint64_t base, int index, uint64_t val);
extern void nlm_hal_cpld_write_16(int cs, uint16_t val, uint16_t reg);
extern uint16_t nlm_hal_cpld_read_16(int cs, uint16_t reg);

extern void nlm_hal_set_rsa_cge(int node, int enable);
extern void nlm_hal_set_sae_engine_sel(int node);
extern void nlm_hal_set_rsa_engine_sel(void);
extern void nlm_hal_set_sae_freq(int node, int freq);
extern int  nlm_hal_get_sae_chip_feature(void);
extern void nlm_hal_set_rsa_freq(int node, int freq);
extern void nlm_hal_set_dtre_freq(int node, int freq);
extern void nlm_hal_set_cde_freq(int node, int freq);
extern void nlm_hal_get_crypto_vc_nums(int *vcbase, int *vclimit);
extern void nlm_hal_get_rsa_vc_nums(int *vcbase, int *vclimit);

#define nlh_read_dev_reg(dev, index) nlm_hal_read_32bit_reg(nlm_hal_get_dev_base(dev), index)
#define nlh_write_dev_reg(dev, index, val) nlm_hal_write_32bit_reg(nlm_hal_get_dev_base(dev), index, val)

extern uint64_t nlm_hal_get_dev_base(int node, int bus, int dev, int func);

/* CPU family identification functions in nlm_hal_cpu_info.c */
extern uint32_t get_proc_id(void);
extern int nlm_hal_get_cpuinfo(struct nlm_netl_proc_info *);
extern int is_nlm_xlp4xx(void);
extern int is_nlm_xlp2xx_cp(void);
extern int is_nlm_xlp1xx(void);
extern int nlm_xlp2xx_has_cmp(void);
extern int nlm_xlp2xx_has_crypto(void);
extern int nlm_xlp2xx_has_rsa(void);
extern int nlm_xlp2xx_has_regx(void);

/* Functions to read defeature programming fuses */
static inline uint32_t efuse_cfg0(void)
{
	return nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG0)));
}

static inline uint32_t efuse_cfg1(void)
{
	return nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG1)));
}

static inline uint32_t efuse_cfg2(void)
{
	return nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG2)));
}

static inline uint32_t efuse_cfg6(void)
{
	return nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG6)));
}

static inline int bitcount(unsigned int n)
{
	register unsigned int tmp;

	tmp = n - ((n >> 1) & 033333333333)
	      - ((n >> 2) & 011111111111);
	return ((tmp + (tmp >> 3)) & 030707070707) % 63;
}

static inline unsigned int num_xlp_cores(void)
{
	int ret;
	uint32_t mask;

	switch(get_proc_id()) {
	case CHIP_PROCESSOR_ID_XLP_8_4_XX:
		ret = 8;
		mask = 0xFF;
		break;
	case CHIP_PROCESSOR_ID_XLP_3XX:
		ret = 4;
		mask = 0x0F;
		break;
	case CHIP_PROCESSOR_ID_XLP_2XX:
		ret = 2;
		mask = 0x03;
		break;
	default:
		return 0;
	}
	return(ret - bitcount(efuse_cfg0() & mask));
}

static inline unsigned int num_xlp_threads(void)
{
	int ret;

	/* Number of enabled CPUs = 4 minus the number disabled */
	switch((efuse_cfg0() >> 28) & 0x3) {
		case 3: ret = 1; break;
		case 2:						// Not currently used
		case 1: ret = 2; break;		// Not currently used
		case 0: ret = 4;
	}
	return ret;
}

extern uint32_t get_dom_owner_mask(void *fdt, int dom_id, char *module);

struct nlm_sae_init_param {
	int node;
	int freq;
};

struct nlm_rsa_init_param {
	int node;
	int freq;
};

extern int nlm_hal_get_fdt_freq(void *fdt, int type);

#endif /* __NLM_HAL_H__ */

