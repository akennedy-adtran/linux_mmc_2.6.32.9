/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#ifndef _NLM_HAL_H_
#define _NLM_HAL_H_

/* Added to enable dependency checking for other SDK pieces */
#define NLM_HAL_SDK_VERSION_230
#define NLM_HAL_SDK_VERSION_231

#include "nlm_hal_macros.h"
#include "nlm_hal_xlp_dev.h"
#include "nlm_nae.h"

#ifndef __ASSEMBLY__
#include "nlm_hal_sys.h"

struct nlm_netl_proc_info{
	unsigned int proc_id;
	unsigned int chipid;		/*example: xlp832=>0x8084, xlp316=>0x3044, xlp208=>0x2024 etc */
	unsigned int revision;
	unsigned int efuse_config[8];
	char cpu_info_str[32];
};

struct nlm_hal_mii_info{
       uint32_t speed;
       uint32_t duplex;
       int link_stat;
       int phyaddr;
};

/**
* @brief Used by function ::get_phy_info and various internal PHY manufacturer-specific functions.
* @ingroup hal
*/
struct nlm_hal_ext_phy{
	char name[16];
	uint32_t phy_idfer;
	uint32_t ext_mdio_bus;
	int inf;
	int phy_addr;
	int basex;
	int (*phy_get_status)(struct nlm_hal_ext_phy* ext_phy, struct nlm_hal_mii_info *mii_info, int node);
	void (*start_phy_an)(struct nlm_hal_ext_phy* ext_phy, int node);
	void (*ext_phy_init)(struct nlm_hal_ext_phy* ext_phy, int node);
	/*(void) (*dump_regs)(void); */
};
extern void nlm_hal_init(void);

extern unsigned long long nlm_hal_cpu_freq(void);
extern int naecfg_hack;

extern int nlm_hal_is_xlp_a0(void);
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

#ifdef NLM_HAL_XLOADER
#define nlm_hal_read_16bit_reg(base, index)	(uint16_t)(nlh_read_cfg_reg16(base + (index << 1)))
#define nlm_hal_write_16bit_reg(base, index, val)	(nlh_write_cfg_reg16(base +  (index << 1) , val))
#define nlm_hal_read_32bit_reg(base, index)	(uint32_t)(nlh_read_cfg_reg32(base + (index << 2)))
#define nlm_hal_write_32bit_reg(base, index, val)	(nlh_write_cfg_reg32(base +  (index << 2) , val))
#define nlm_hal_read_64bit_reg(base, index) (uint64_t)(nlh_read_cfg_reg64(base + (index << 3)))
#define nlm_hal_write_64bit_reg(base, index, val) 	(nlh_write_cfg_reg64(base +  (index << 3) , val))

#else

extern uint16_t nlm_hal_read_16bit_reg(uint64_t base, uint32_t index);
extern void nlm_hal_write_16bit_reg(uint64_t base, uint32_t index, uint16_t val);
extern uint32_t nlm_hal_read_32bit_reg(uint64_t base, int index);
extern void nlm_hal_write_32bit_reg(uint64_t base, int index, uint32_t val);
extern uint64_t nlm_hal_read_64bit_reg(uint64_t base, int index);
extern void nlm_hal_write_64bit_reg(uint64_t base, int index, uint64_t val);

#endif /*NLM_HAL_XLOADER*/

extern void nlm_hal_cpld_write_16(int cs, uint16_t val, uint16_t reg);
extern uint16_t nlm_hal_cpld_read_16(int cs, uint16_t reg);

extern uint32_t efuse_cfg0(void);
extern uint32_t efuse_cfg1(void);
extern uint32_t efuse_cfg2(void);
extern uint32_t efuse_cfg6(void);
extern int nlm_xlp2xx_has_cmp(void);
extern int nlm_xlp2xx_has_crypto(void);
extern int nlm_xlp2xx_has_rsa(void);
extern int nlm_xlp2xx_has_regx(void);
extern uint32_t get_proc_id(void);

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

extern int nlm_hal_get_cpuinfo(struct nlm_netl_proc_info *);

extern int is_nlm_xlp208(void);
extern int is_nlm_xlp108(void);
extern int is_nlm_xlp204(void);
extern int is_nlm_xlp104(void);
extern int is_nlm_xlp202(void);
extern int is_nlm_xlp201(void);
extern int is_nlm_xlp101(void);

extern uint32_t get_dom_owner_mask(void *fdt, int dom_id, char *module);

struct nlm_sae_init_param {
	int node;
	int freq;
};

struct nlm_rsa_init_param {
	int node;
	int freq;
};

/* Removed define for XLP_PIT_TICK_RATE - needs to be a function since XLP2xx PIC timer
   is different from XLP PIC timer which is either 133 MHz or 66 MHz, and doesn't even
   work if reference clock is 125 MHz.
 */

extern int nlm_hal_get_fdt_freq(void *fdt, int type);
/*
TODO :
  1. support Debug flags
  2. XLP support ?
 */

static inline uint32_t get_xlp3xx_epid(void)
{
        uint32_t cfg0, epid;

        cfg0=efuse_cfg0();
        epid = (uint8_t)(( cfg0>>4 )  & 0xf);
        return epid;
}

/* Backport from 2.3.1 - support XLP416 fusing */
static inline int bitcount(unsigned int n)
{
	register unsigned int tmp;

	tmp = n - ((n >> 1) & 033333333333)
	      - ((n >> 2) & 011111111111);
	return ((tmp + (tmp >> 3)) & 030707070707) % 63;
}

#endif /* __ASSEMBLY__ */

#endif /* #ifndef _NLM_HAL_H_ */

