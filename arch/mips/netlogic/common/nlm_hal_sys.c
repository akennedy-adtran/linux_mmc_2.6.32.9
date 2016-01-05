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

#if !defined(__KERNEL__) && !defined(NLM_HAL_XLP2) && !defined(NLM_HAL_XLP1)
#include <stddef.h>
#endif

#ifdef NLM_HAL_LINUX_KERNEL
#include <asm/div64.h>
#endif

#include "nlm_hal.h"
#include "nlm_hal_sys.h"
#include "nlm_hal_xlp_dev.h"

static void nlm_hal_soc_clock_enable(int node, soc_device_id_t device);
static void nlm_hal_soc_clock_disable(int node, soc_device_id_t device);
static int nlm_hal_get_soc_clock_state(int node, soc_device_id_t device);

#ifndef NLM_HAL_XLP1
static uint32_t xlp2xx_get_ref_clk(uint32_t* ref_clk_num, uint16_t* ref_clk_den);
static uint32_t nlm_hal_xlp2xx_set_clkdev_frq(soc_device_id_t dev_type, uint32_t freq);
static uint32_t nlm_hal_xlp2xx_get_clkdev_frq(soc_device_id_t dev_type);
#endif

/* For x-loader and u-boot, only include the subset of clocking APIs
   for the right part, to save space */
#ifndef NLM_HAL_XLP2
/**
 * Calculate the DFS divider value for the specified reference clock
 * and target output frequency.
 * @param [in] reference the reference clock frequency, in Hz.
 * @param [in] target the target output frequency, in Hz.
 * @returns the divider that produces the target frequency
 * (if the target frequency is within FREQ_RESOLUTION of the actual output).
 * Otherwise, it will round up to the nearest divider (rounding down to the
 * the lower output frequency).
 */
static uint16_t fuzzy_divider(uint32_t reference, uint32_t target)
{
	uint64_t divider = reference;
	uint64_t freq = reference;
	uint64_t delta;
	uint16_t result;
	NLM_HAL_DO_DIV(divider, target);
	NLM_HAL_DO_DIV(freq, divider);
	delta = freq - target;
	result = (uint16_t)divider;
	return (delta <= FREQ_RESOLUTION)? result : (result + 1);
}

/**
 * @returns the numerator for the reference clock frequency.
 */
static inline uint64_t ref_clk_num(void)
{
	return 400000000;
}

/**
 * @returns the denominator for the reference clock frequency.
 */
static inline uint16_t ref_clk_den(void)
{
	return 3;
}
#endif

/**
 * @returns the current reference clock frequency, in Hz.
 */
uint32_t nlm_hal_get_ref_clk_freq(void)
{
	uint32_t rv = 0;
#ifndef NLM_HAL_XLP1
	uint32_t clk_num;
	uint16_t clk_den;
	if(is_nlm_xlp2xx())
		return xlp2xx_get_ref_clk(&clk_num, &clk_den);
#endif
#ifndef NLM_HAL_XLP2
	rv = (ref_clk_num() / ref_clk_den());
#endif
	return rv;
}

#ifndef NLM_HAL_XLP2
/**
 * Mapping of DFS indices to actual DFS divider values.
 */
static uint16_t DFS[] = {1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 15};

/**
 * @returns the maximum DFS divider value.
 */
static inline uint16_t max_dfs_val(void)
{
	return DFS[COUNT_OF(DFS) - 1];
}
/**
 * @returns the minimum DFS divider value.
 */
static inline uint16_t min_dfs_val(void)
{
	return DFS[0];
}

/**
 * Get the output frequency for the PLL, based on the
 * R-divider, F-divider, and PLL DFS divider.
 * @param [in] divr R-divider for the PLL.
 * @param [in] divf F-divider for the PLL.
 * @param [in] pll_dfs divider (CPU / SOC PLLs only).
 * @return PLL frequency, in Hz.
 */
static uint32_t pll_freq(uint16_t divr, uint16_t divf, uint16_t pll_dfs)
{
	uint64_t num = ref_clk_num() * (divf + 1) * 4 / 2;
	uint32_t den = ref_clk_den() * (divr + 1) * (pll_dfs + 1);
	NLM_HALT_IF_XLPII();
	NLM_HAL_DO_DIV(num, den);
	return num;
}

/**
 * Get the DFS index for the DFS value (to be used with the stepping functions).
 * The DFS value is rounded up (producing the lower output frequency) if
 * the exact DFS value does not exist.
  @param [in] dfs DFS divider value.
 * @return the DFS index greater than or equal to the specified DFS divider value.
 */
static uint16_t closest_dfs_index(uint16_t dfs)
{
	int i;
	NLM_HALT_IF_XLPII();
	if (dfs > max_dfs_val())
		return COUNT_OF(DFS)-1;

	for (i = (COUNT_OF(DFS) - 2); i >= 0; i--) {
		if ((DFS[i+1] >= dfs) && (dfs > DFS[i]))
			return i+1;
	}

	return 0;
}

/**
 * Determine whether the Core PLL DFS is bypassed.
 * @returns 0 if the Core PLL is not bypassed.
 * @returns 1 if the Core PLL is bypassed, or if the Core PLL does not exist.
 */
static int is_core_pll_dfs_bypassed(int node)
{
	NLM_HALT_IF_XLPII();
	return (nlm_hal_read_sys_reg(node, PLL_DFS_BYP_CTRL) & 0x1) ? 1 : 0;
}

/**
 * Determine whether the SoC PLL DFS is bypassed.
 * @returns 0 if the SoC PLL is not bypassed.
 * @returns 1 if the SoC PLL is bypassed, or if the SoC PLL does not exist.
 */
static int is_soc_pll_dfs_bypassed(int node)
{
	NLM_HALT_IF_XLPII();
	return (nlm_hal_read_sys_reg(node, PLL_DFS_BYP_CTRL) & 0x2) ? 1 : 0;
}

/**
 * @return the Core PLL DFS divider value.
 * @return 0 if the Core PLL DFS is bypassed.
 */
static uint16_t core_pll_dfs_val(int node)
{
	NLM_HALT_IF_XLPII();
	if (is_core_pll_dfs_bypassed(node))
		return 0;
	/* TODO = Eagle PRM shows this is one bit - is this correct?  Storm PRM shows 4 bits */
	return nlm_hal_read_sys_reg(node, PLL_DFS_DIV_VALUE) & 0xf;
}

/**
 * @return the SoC PLL DFS divider value.
 * @return 0 if the SoC PLL DFS is bypassed.
 */
static uint16_t soc_pll_dfs_val(int node)
{
	NLM_HALT_IF_XLPII();
	if (is_soc_pll_dfs_bypassed(node))
		return 0;
	/* TODO = Eagle PRM shows this is one bit - is this correct?  Storm PRM shows 4 bits */
	return (nlm_hal_read_sys_reg(node, PLL_DFS_DIV_VALUE) >> 4) & 0xf;
}

/**
 * Get the Core PLL frequency post PLL DFS.
 */
static uint32_t core_pll_freq(int node)
{
	uint32_t freq;
	uint32_t reg = nlm_hal_read_sys_reg(node, POWER_ON_RESET_CFG);
	uint16_t divr = (reg >> 8)  & 0x3;
	uint16_t divf = (reg >> 10) & 0x7f;
	uint16_t dfs  = core_pll_dfs_val(node);
	NLM_HALT_IF_XLPII();
	freq = pll_freq(divr, divf, dfs);
	return freq;
}

/**
 * Get the SoC PLL frequency post PLL DFS.
 */
static uint32_t soc_pll_freq(int node)
{
	uint32_t reg = nlm_hal_read_sys_reg(node, PLL_CTRL);
	uint16_t divf = (reg >> 3) & 0x7F;
	uint16_t divr = (reg >> 1) & 0x3;
	uint16_t dfs  = soc_pll_dfs_val(node);
	NLM_HALT_IF_XLPII();
	return pll_freq(divr, divf, dfs);
}

/**
 * Get the DDR PLL frequency.
 */
static uint32_t ddr_pll_freq(int node)
{
	uint32_t reg = nlm_hal_read_sys_reg(node, PLL_CTRL);
	uint16_t divf = (reg >> 19) & 0x7F;
	uint16_t divr = (reg >> 17) & 0x3;
	NLM_HALT_IF_XLPII();
	return pll_freq(divr, divf, 0);
}

uint32_t nlm_hal_get_pll_freq(int node, xlp_pll_type_t pll)
{
	if (pll == CORE_PLL) return core_pll_freq(node);
	if (pll == SOC_PLL)  return soc_pll_freq(node);
	if (pll == DDR_PLL)  return ddr_pll_freq(node);
	return 0;
}

/**
 * Get the DFS divider value for the specified SoC device.
 * @param [in] device the SoC device.
 */
static uint16_t soc_dfs_val(int node, soc_device_id_t device)
{
	int device_index = device;
	NLM_HALT_IF_XLPII();
	if (device_index >= 8)
	{
		device_index -= 8;
		return (nlm_hal_read_sys_reg(node, SYS_DFS_DIV_VALUE1) >> (device_index * 4)) & 0xF;
	}
	return (nlm_hal_read_sys_reg(node, SYS_DFS_DIV_VALUE0) >> (device_index * 4)) & 0xF;
}

/**
 * Get the DFS divider value for the specified Core.
 * @param [in] core CPU core index.
 */
static uint16_t core_dfs_val(int node, uint8_t core)
{
	NLM_HALT_IF_XLPII();
	return (nlm_hal_read_sys_reg(node, CORE_DFS_DIV_VALUE) >> (core * 4)) & 0xF; 
}

/**
 * Determine whether the SoC device's DFS is bypassed.
 * @param [in] device the SoC device.
 * @returns 1 if the SoC device's DFS is bypassed.
 * @returns 0 if the SoC device's DFS is not bypassed.
 */

static int is_soc_dfs_bypassed(int node, soc_device_id_t device)
{
	int device_index = device;
	NLM_HALT_IF_XLPII();
	return ((nlm_hal_read_sys_reg(node, SYS_DFS_BYP_CTRL) >> device_index) & 1) ? 1 : 0;
}

uint16_t nlm_hal_get_soc_dfs_val(int node, soc_device_id_t device)
{
	if (is_soc_dfs_bypassed(node, device))
		return 0;
	else
		return soc_dfs_val(node, device);
}

/**
 * Enable/disable the DFS bypass for the specified SoC device.
 * @param [in] device the SoC device.
 * @param [in] bypass 1: bypass the DFS. 0: do not bypass DFS.
 */
static void set_soc_dfs_bypass(int node, soc_device_id_t device, int bypass)
{
	int device_index = device;
	uint32_t val;
	NLM_HALT_IF_XLPII();
	val = nlm_hal_read_sys_reg(node, SYS_DFS_BYP_CTRL) & ~(1 << device_index);
	nlm_hal_write_sys_reg(node, SYS_DFS_BYP_CTRL, val | ((bypass? 0x1 : 0x0) << device_index));
}

/**
 * Determine whether the CPU core's DFS is bypassed.
 * @param [in] core CPU core index.
 * @returns 1 if the CPU core's DFS is bypassed.
 * @returns 0 if the CPU core's DFS is not bypassed.
 */
static int is_core_dfs_bypassed(int node, int core)
{
	NLM_HALT_IF_XLPII();
	return (nlm_hal_read_sys_reg(node, CORE_DFS_BYP_CTRL) >> core) & 1;
}

/**
 * Enable/disable the DFS bypass for the specified CPU core.
 * @param [in] core CPU core index.
 * @param [in] bypass 1: bypass the DFS. 0: do not bypass DFS.
 */
static void set_core_dfs_bypass(int node, int core, int bypass)
{
	uint32_t val;
	NLM_HALT_IF_XLPII();
	val = nlm_hal_read_sys_reg(node, CORE_DFS_BYP_CTRL) & ~(1 << core);
	nlm_hal_write_sys_reg(node, CORE_DFS_BYP_CTRL, val | ((bypass? 0x1 : 0x0) << core));
}
#endif /* NLM_HAL_XLP2 */

/**
 * Get the operating frequency for the specified SOC device.
 * @param [in] device the SoC device.
 * @returns The SoC device operating frequency, in Hz.
 */
uint32_t nlm_hal_get_soc_freq(int node, soc_device_id_t device)
{
	uint32_t freq = 0;

	if(nlm_hal_get_soc_clock_state(node, device) == XLP_DISABLE)
		return freq;

#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx())
		return nlm_hal_xlp2xx_get_clkdev_frq(device);
#endif
#ifndef NLM_HAL_XLP2
	switch (device) {
		case DFS_DEVICE_NAND:
		case DFS_DEVICE_NOR:
		case DFS_DEVICE_MMC:
			/* NOR, NAND and MMC devices are derived from the reference clock. */
			freq = nlm_hal_get_ref_clk_freq();
			break;
		case DFS_DEVICE_DMC:
			freq = ddr_pll_freq(node);
			break;
		case DFS_DEVICE_CORE:
			/* SOC Core is derived from the CPU core PLL */
			freq = core_pll_freq(node);
			break;
		default:
			freq = soc_pll_freq(node);
			break;
	}

	if (!is_soc_dfs_bypassed(node, device)) {
		uint16_t den = soc_dfs_val(node, device) + 1;
		NLM_HAL_DO_DIV(freq, den);
	}
#endif
	return freq;
}

#ifndef NLM_HAL_XLP2
/**
 * Step the DFS of the specified SoC device to the target DFS index.
 * @param [in] device the SoC device.
 * @param [in] dfs_index DFS index (**not** the DFS value).
 */
static void step_soc_dfs(int node, soc_device_id_t device, uint16_t dfs_index)
{
	int device_index;
	int cur;
	int delta, i;

	NLM_HALT_IF_XLPII();
	device_index = device;
	cur = closest_dfs_index(soc_dfs_val(node, device));
	delta = cur - dfs_index;

	if (delta >= 0) {
		/* positive delta, decrement dfs */
		for (i=0; i < delta; i++)
			nlm_hal_write_sys_reg(node, SYS_DFS_DIV_DEC_CTRL, 1 << device_index);
	} else {
		/* negative delta, increment dfs */
		for (i=0; i > delta; i--)
			nlm_hal_write_sys_reg(node, SYS_DFS_DIV_INC_CTRL, 1 << device_index);
	}
}
#endif

/**
 * Set the operating frequency for the specified SoC device.
 * This is achieved only by stepping the SoC device DFS.
 * @param [in] device the SoC device.
 * @param [in] freq target SoC device frequency, in Hz.
 * @returns the new SoC device operating frequency, in Hz.
 */
uint32_t nlm_hal_set_soc_freq(int node, soc_device_id_t device, uint32_t freq)
{
	uint32_t rv = 0;
	uint32_t reference __attribute__((unused));
	uint16_t  target __attribute__((unused));

	/* If freq is zero, disable the device */
	if (freq == 0) {
		nlm_hal_soc_clock_disable(node, device);
		return rv;
	}
#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx())
		return nlm_hal_xlp2xx_set_clkdev_frq(device, freq);
#endif
#ifndef NLM_HAL_XLP2
	if(nlm_hal_get_soc_clock_state(node, device) == XLP_DISABLE)
		nlm_hal_soc_clock_enable(node, device);

	switch (device) {
		case DFS_DEVICE_NAND:
		case DFS_DEVICE_NOR:
		case DFS_DEVICE_MMC:
			/* NOR, NAND and MMC devices are derived from the reference clock. */
			reference = nlm_hal_get_ref_clk_freq();
			break;
		case DFS_DEVICE_DMC:
			reference = ddr_pll_freq(node);
			break;
		case DFS_DEVICE_CORE:
			reference = core_pll_freq(node);
			break;
		default:
			reference = soc_pll_freq(node);
			break;
	}

	target = closest_dfs_index(fuzzy_divider(reference, freq) - 1);
	if (freq >= (reference - FREQ_RESOLUTION)) {
		/* bypass DFS if freq is reference freq */
		set_soc_dfs_bypass(node, device, 1);
	} else {
		/* otherwise, step dfs and clear bypass */
		step_soc_dfs(node, device, target);
		set_soc_dfs_bypass(node, device, 0);
	}
	rv = nlm_hal_get_soc_freq(node, device);
#endif
	return rv;
}

/**
 * Get the operating frequency for the specified CPU core.
 * @param [in] core CPU core index.
 * @returns The CPU core operating frequency, in Hz.
 */
uint32_t nlm_hal_get_core_freq(int node, int core)
{
	uint32_t reference __attribute__((unused)) = 0;

#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx())
		return nlm_hal_xlp2xx_get_pllfreq_dyn(core);
#endif
#ifndef NLM_HAL_XLP2
	reference = core_pll_freq(node);
	if (!is_core_dfs_bypassed(node, core)) {
		uint16_t den = core_dfs_val(node, core) + 1;;
		NLM_HAL_DO_DIV(reference, den);
	}
#endif
	return reference;
}

#ifndef NLM_HAL_XLP2
/**
 * Step the DFS of the specified CPU core to the target DFS index.
 * @param [in] core CPU core index.
 * @param [in] dfs_index DFS index (**not** the DFS value).
 */
static void step_core_dfs(int node, int core, uint16_t dfs_index)
{
	int cur;
	int delta;
	int i;

	NLM_HALT_IF_XLPII();
	cur = closest_dfs_index(core_dfs_val(node, core));
	delta = cur - dfs_index;

	if (delta >= 0) {
		/* positive delta, decrement dfs */
		for (i=0; i < delta; i++)
			nlm_hal_write_sys_reg(node, CORE_DFS_DIV_DEC_CTRL, 1 << core);
	} else {
		/* negative delta, increment dfs */
		for (i=0; i > delta; i--)
			nlm_hal_write_sys_reg(node, CORE_DFS_DIV_INC_CTRL, 1 << core);
	}
}
#endif

/**
 * Set the operating frequency for the specified CPU core.
 * This is achieved only by stepping the Core DFS.
 * @param [in] core CPU core index.
 * @param [in] freq target CPU core frequency, in Hz.
 * @returns the new CPU core operating frequency, in Hz.
 */
uint32_t nlm_hal_set_core_freq(int node, int core, uint32_t freq)
{
	uint32_t rv = 0;
	uint32_t reference __attribute__((unused));
	uint16_t  target __attribute__((unused));
#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx())
		return nlm_hal_xlp2xx_set_pllfreq_dyn(core, freq);
#endif
#ifndef NLM_HAL_XLP2
	reference = core_pll_freq(node);
	target = closest_dfs_index(fuzzy_divider(reference, freq) - 1);

	if (freq >= (reference - FREQ_RESOLUTION)) {
		/* bypass DFS if freq is reference freq */
		set_core_dfs_bypass(node, core, 1);
	} else {
		/* otherwise, step dfs and clear bypass */
		step_core_dfs(node, core, target);
		set_core_dfs_bypass(node, core, 0);
	}
	rv = nlm_hal_get_core_freq(node, core);
#endif
	return rv;
}

/**
 * Get the operating frequency for the current core.
 * @returns The core operating frequency (in Hz).
 */
uint32_t nlm_hal_cpu_freq(void)  
{
	int core = (nlm_cpu_id() >> 2) & 0x7;
	return nlm_hal_get_core_freq(nlm_node_id(), core);
}

static uint32_t nlm_hal_get_biu_mask_by_soc_device_id(soc_device_id_t device)
{
	uint32_t biu_mask = 0;
#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx()) {
		switch(device) {
		case XLP2XX_CLKDEVICE_NAE:
			biu_mask  = (1 << XLP2XX_IO_NET_BIU_NUMBER);
//			biu_mask |= (1 << XLP2XX_IO_MSG_BIU_NUMBER);
			biu_mask |= (1 << XLP2XX_IO_POE_BIU_NUMBER);
			break;
		case XLP2XX_CLKDEVICE_SAE:
			biu_mask = (1 << XLP2XX_IO_SEC_BIU_NUMBER);
			break;
		case XLP2XX_CLKDEVICE_RSA:
			biu_mask = (1 << XLP2XX_IO_RSA_BIU_NUMBER);
			break;
		case XLP2XX_CLKDEVICE_CMP:
			biu_mask = (1 << XLP2XX_IO_CMP_BIU_NUMBER);
			break;
		case XLP2XX_CLKDEVICE_NAND:
		case XLP2XX_CLKDEVICE_MMC:
		case XLP2XX_CLKDEVICE_NOR:
			biu_mask = (1 << XLP2XX_IO_GBU_BIU_NUMBER);
			break;
		case XLP2XX_CLKDEVICE_RGXF:
		case XLP2XX_CLKDEVICE_RGXS:
			biu_mask = (1 << XLP2XX_IO_REGX_BIU_NUMBER);
			break;
		case XLP2XX_CLKDEVICE_USB:
			biu_mask = (1 << XLP2XX_IO_USB_BIU_NUMBER);
			break;
		default: { }
		}
	}
#endif
#ifndef NLM_HAL_XLP2
	if(is_nlm_xlp3xx()) {
		switch(device) {
		case DFS_DEVICE_NAE_2X:
			biu_mask  = (1 << XLP3XX_IO_NET_BIU_NUMBER);
//			biu_mask |= (1 << XLP3XX_IO_MSG_BIU_NUMBER);
			biu_mask |= (1 << XLP3XX_IO_POE_BIU_NUMBER);
			break;
		case DFS_DEVICE_SAE:
			biu_mask = (1 << XLP3XX_IO_SEC_BIU_NUMBER);
			break;
		case DFS_DEVICE_RSA:
			biu_mask = (1 << XLP3XX_IO_RSA_BIU_NUMBER);
			break;
		case DFS_DEVICE_NAND:
		case DFS_DEVICE_MMC:
		case DFS_DEVICE_NOR:
			biu_mask = (1 << XLP3XX_IO_GBU_BIU_NUMBER);
			break;
		case DFS_DEVICE_REGEX_FAST:
		case DFS_DEVICE_REGEX_SLOW:
			biu_mask = (1 << XLP3XX_IO_REGX_BIU_NUMBER);
			break;
		case DFS_DEVICE_SATA:
			biu_mask = (1 << XLP3XX_IO_SATA_BIU_NUMBER);
			break;
		default: { }
		}
	} else if(is_nlm_xlp8xx()) {
		switch(device) {
		case DFS_DEVICE_NAE_2X:
			biu_mask  = (1 << XLP8XX_IO_NET_BIU_NUMBER);
//			biu_mask |= (1 << XLP8XX_IO_MSG_BIU_NUMBER);
			biu_mask |= (1 << XLP8XX_IO_POE_BIU_NUMBER);
			break;
		case DFS_DEVICE_SAE:
			biu_mask = (1 << XLP8XX_IO_SEC_BIU_NUMBER);
			break;
		case DFS_DEVICE_RSA:
			biu_mask = (1 << XLP8XX_IO_RSA_BIU_NUMBER);
			break;
		case DFS_DEVICE_CMP:
			biu_mask = (1 << XLP8XX_IO_CMP_BIU_NUMBER);
			break;
		case DFS_DEVICE_NAND:
		case DFS_DEVICE_MMC:
		case DFS_DEVICE_NOR:
			biu_mask = (1 << XLP8XX_IO_GBU_BIU_NUMBER);
			break;
		default: { }
		}
	}
#endif
	return biu_mask;
}

static int nlm_hal_get_soc_clock_state(int node, soc_device_id_t device)
{
	uint32_t biu_mask = nlm_hal_get_biu_mask_by_soc_device_id(device);
	int reg = is_nlm_xlp2xx() ? XLP2XX_SYSDISABLE : SYS_DISABLE;

	if(!biu_mask) return XLP_ENABLE;

	return (nlm_hal_read_sys_reg(node, reg) & biu_mask) ? XLP_DISABLE : XLP_ENABLE;
}

/* Warning - if disabling the clock to an SOC device, must also disable it in
 * the Bus Interface Unit (BIU) - otherwise reads to the device (e.g. PCI/PCI-e
 * config space header) will time-out and hang the CPU. Use the SYSDISABLE register
 * for this. NAND, NOR and MMC all live in GBU - must enable/disable all together.
 * Same with RegEx Fast / Slow clocks.
 */
static void nlm_hal_soc_clock_enable(int node, soc_device_id_t device)
{
	uint32_t d32;
	uint32_t biu_mask = nlm_hal_get_biu_mask_by_soc_device_id(device);

	if(!biu_mask) return;
	
#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx()) {
		d32 = nlm_hal_read_sys_reg(node, XLP2XX_SYS_CLK_DEV_DIS);
		if ((device == XLP2XX_CLKDEVICE_NAND) || (device == XLP2XX_CLKDEVICE_MMC) || (device == XLP2XX_CLKDEVICE_NOR)) {
			d32 &= ~(1 << (XLP2XX_CLKDEVICE_NAND - XLP2XX_CLKDEVICE_NAE));
			d32 &= ~(1 << (XLP2XX_CLKDEVICE_MMC - XLP2XX_CLKDEVICE_NAE));
			d32 &= ~(1 << (XLP2XX_CLKDEVICE_NOR - XLP2XX_CLKDEVICE_NAE));
		} else if ((device == XLP2XX_CLKDEVICE_RGXF) || (device == XLP2XX_CLKDEVICE_RGXS)) {
			d32 &= ~(1 << (XLP2XX_CLKDEVICE_RGXF - XLP2XX_CLKDEVICE_NAE));
			d32 &= ~(1 << (XLP2XX_CLKDEVICE_RGXS - XLP2XX_CLKDEVICE_NAE));
		} else
			d32 &= ~(1 << (device - XLP2XX_CLKDEVICE_NAE));
		nlm_hal_write_sys_reg(node, XLP2XX_SYS_CLK_DEV_DIS, d32);	// Enable clock
		
		d32 = nlm_hal_read_sys_reg(node, XLP2XX_SYSDISABLE);
		d32 &= ~biu_mask;
		nlm_hal_write_sys_reg(node, XLP2XX_SYSDISABLE, d32);	// Enable device on the bus
		return;
	}
#endif

#ifndef NLM_HAL_XLP2
	d32 = nlm_hal_read_sys_reg(node, SYS_DFS_DIS_CTRL);
	if ((device == DFS_DEVICE_NAND) || (device == DFS_DEVICE_MMC) || (device == DFS_DEVICE_NOR)) {
		d32 &= ~(1 << DFS_DEVICE_NAND);
		d32 &= ~(1 << DFS_DEVICE_MMC);
		d32 &= ~(1 << DFS_DEVICE_NOR);
	} else if ((device == DFS_DEVICE_REGEX_FAST) || (device == DFS_DEVICE_REGEX_SLOW)) {
		d32 &= ~(1 << DFS_DEVICE_REGEX_FAST);
		d32 &= ~(1 << DFS_DEVICE_REGEX_SLOW);
	} else
		d32 &= ~(1 << device);
	nlm_hal_write_sys_reg(node, SYS_DFS_DIS_CTRL, d32);		// Enable clock

	d32 = nlm_hal_read_sys_reg(node, SYS_DISABLE);
	d32 &= ~biu_mask;
	nlm_hal_write_sys_reg(node, SYS_DISABLE, d32);		// Enable device on the bus
#endif
}

static void nlm_hal_soc_clock_disable(int node, soc_device_id_t device)
{
	uint32_t d32;
	uint32_t biu_mask = nlm_hal_get_biu_mask_by_soc_device_id(device);

	if(!biu_mask) return;

#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx()) {
		d32 = nlm_hal_read_sys_reg(node, XLP2XX_SYS_CLK_DEV_DIS);
		if ((device == XLP2XX_CLKDEVICE_NAND) || (device == XLP2XX_CLKDEVICE_MMC) || (device == XLP2XX_CLKDEVICE_NOR)) {
			d32 |= (1 << (XLP2XX_CLKDEVICE_NAND - XLP2XX_CLKDEVICE_NAE));
			d32 |= (1 << (XLP2XX_CLKDEVICE_MMC - XLP2XX_CLKDEVICE_NAE));
			d32 |= (1 << (XLP2XX_CLKDEVICE_NOR - XLP2XX_CLKDEVICE_NAE));
		} else if ((device == XLP2XX_CLKDEVICE_RGXF) || (device == XLP2XX_CLKDEVICE_RGXS)) {
			d32 |= (1 << (XLP2XX_CLKDEVICE_RGXF - XLP2XX_CLKDEVICE_NAE));
			d32 |= (1 << (XLP2XX_CLKDEVICE_RGXS - XLP2XX_CLKDEVICE_NAE));
		} else
			d32 |= (1 << (device - XLP2XX_CLKDEVICE_NAE));
		nlm_hal_write_sys_reg(node, XLP2XX_SYS_CLK_DEV_DIS, d32);	// Disable clock
		
		d32 = nlm_hal_read_sys_reg(node, XLP2XX_SYSDISABLE);
		d32 |= biu_mask;
		nlm_hal_write_sys_reg(node, XLP2XX_SYSDISABLE, d32);	// Disable device on the bus
		return;
	}
#endif

#ifndef NLM_HAL_XLP2
	d32 = nlm_hal_read_sys_reg(node, SYS_DFS_DIS_CTRL);
	if ((device == DFS_DEVICE_NAND) || (device == DFS_DEVICE_MMC) || (device == DFS_DEVICE_NOR)) {
		d32 |= (1 << DFS_DEVICE_NAND);
		d32 |= (1 << DFS_DEVICE_MMC);
		d32 |= (1 << DFS_DEVICE_NOR);
	} else if ((device == DFS_DEVICE_REGEX_FAST) || (device == DFS_DEVICE_REGEX_SLOW)) {
		d32 |= (1 << DFS_DEVICE_REGEX_FAST);
		d32 |= (1 << DFS_DEVICE_REGEX_SLOW);
	} else
		d32 |= (1 << device);
	nlm_hal_write_sys_reg(node, SYS_DFS_DIS_CTRL, d32);		// Disable clock

	d32 = nlm_hal_read_sys_reg(node, SYS_DISABLE);
	d32 |= biu_mask;
	nlm_hal_write_sys_reg(node, SYS_DISABLE, d32);		// Disable device on the bus
#endif
}

#ifndef NLM_HAL_XLP1
/*
 * XLP 2XX Clock Management
 */
/*  Reference Clock Select 00:66; 01:100; 10:125; 11:133 */
#define SYS_PWRON_RCS(x) (((x)>>18) & 0x3)

static uint16_t xlp2xx_get_rcs(void)
{
	uint32_t reg = nlm_hal_read_sys_reg(NODE_0, XLP2XX_POWER_ON_RESET_CFG);
	return SYS_PWRON_RCS(reg);
}

static void nlm_hal_xlp2xx_dev_pll_cfg(soc_device_id_t dev_type,
		xlp2xx_clkdev_sel_t dev_pll_sel, xlp2xx_clkdev_div_t div)
{
	int dev_idx = dev_type - XLP2XX_CLKDEVICE_NAE;
	uint32_t rsel, rdiv, rchg;
	NLM_HALT_IF(dev_idx<0);
	rsel = nlm_hal_read_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_SEL_REG) & (~(3<<(dev_idx*2)));
	rdiv = nlm_hal_read_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_DIV_REG) & (~(3<<(dev_idx*2)));
	nlm_hal_write_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_SEL, rsel | ((dev_pll_sel&3) << (dev_idx*2)));
	nlm_hal_write_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_DIV, rdiv | ((div&3)<<(dev_idx*2)));
	rchg = nlm_hal_read_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_CHG);
	nlm_hal_write_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_CHG, rchg|(1<<dev_idx));
	while((nlm_hal_read_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_CHG) & (1<<dev_idx)));
	return;
}

/*
 * SYS_CLK_DEV_SEL
 *  00  400MHz
 *  01  Dev0 PLL
 *  10  Dev1 PLL
 *  11  Dev2 PLL
 */
static xlp2xx_clkdev_sel_t xlp2xx_get_clk_dev_sel(soc_device_id_t dev_type)
{
	int dev_idx = dev_type - XLP2XX_CLKDEVICE_NAE;
	NLM_HALT_IF(dev_idx<0);
	return (nlm_hal_read_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_SEL_REG) >> (dev_idx*2) ) & 0x3 ;
}	

/*
 * SYS_CLK_DEV_DIV
 * 00  1
 * 01  2
 * 10  4
 * 11  8
 */
static uint16_t xlp2xx_get_clkdev_div(soc_device_id_t dev_type)
{
	int dev_idx = dev_type - XLP2XX_CLKDEVICE_NAE;
	NLM_HALT_IF(dev_idx<0);
	return 1 << (( nlm_hal_read_sys_reg(NODE_0, XLP2XX_SYS_CLK_DEV_DIV_REG) >> (dev_idx*2) ) & 0x3);
}

static uint16_t xlp2xx_pll_Rate2Div(uint32_t reg)
{
	uint16_t div;
	reg = (reg>>24) & 7;
	switch( reg )
	{
		case 0:  div = 1;  break;
		case 1:  div = 2;  break;
		case 3:  div = 4;  break;
		case 7:  div = 8;  break;
		case 6:  div = 16; break;
		default: div = 1;  break;
	}

	return div;
}

uint32_t nlm_hal_xlp2xx_get_pllfreq_dyn(xlp2xx_pll_type_t pll_type)
{
	uint16_t pll_mult; /* [5:0] */
	uint32_t pll_rate;
	uint64_t ref_clk_num;
	uint32_t reg_ctrl0, reg_ctrl;

	switch(pll_type)
	{
		case CORE0_PLL:
		case CORE1_PLL:
			reg_ctrl0 = XLP2XX_CORE0_PLL_CTRL0+pll_type*4;
			reg_ctrl = XLP2XX_CORE0_PLL_CTRL1+pll_type*4;
			break;
		case SYS_PLL:
		case DMC_PLL:
		case DEV0_PLL:
		case DEV1_PLL:
		case DEV2_PLL:
			reg_ctrl0 = XLP2XX_SYS_PLL_CTRL0+(pll_type-SYS_PLL)*4;
			reg_ctrl = XLP2XX_SYS_PLL_CTRL1+(pll_type-SYS_PLL)*4;
			break;
		default:
			nlm_print("Unknown PLL type:%d\n", pll_type);
			return 0;
	}

	pll_mult = nlm_hal_read_sys_reg(NODE_0, reg_ctrl) & (0x3f);
	pll_rate = nlm_hal_read_sys_reg(NODE_0, reg_ctrl0);
	ref_clk_num = (XLP2_PLL_STEP_NUM * pll_mult);
	NLM_HAL_DO_DIV(ref_clk_num, XLP2_PLL_STEP_DEN);
	ref_clk_num += XLP2_PLL_BASE;
	NLM_HAL_DO_DIV(ref_clk_num, xlp2xx_pll_Rate2Div(pll_rate));

	return ref_clk_num;
}

uint32_t nlm_hal_xlp2xx_set_pllfreq_dyn(xlp2xx_pll_type_t pll_type, uint32_t freq)
{
	/*Target PLL output frequency; 400 MHz + (33.333 MHz x [5:0]).*/
	uint16_t pll_mult; /* [5:0] */
	uint32_t reg_ctrl, reg_chg, chg_mask;

	if(freq < XLP2_PLL_BASE){
		nlm_print("Freq for PLL cant be less than %dMHz\n", XLP2_PLL_BASE/1000/1000);
		return 0;
	}

	freq = XLP2_PLL_STEP_DEN * (freq - XLP2_PLL_BASE);
	NLM_HAL_DO_DIV(freq, XLP2_PLL_STEP_NUM);
	pll_mult = freq;
	switch(pll_type)
	{
		case CORE0_PLL:
		case CORE1_PLL:
			reg_ctrl = XLP2XX_CORE0_PLL_CTRL1+pll_type*4;
			reg_chg  = XLP2XX_CPU_PLL_CHG_CTRL;
			chg_mask = 1<<(pll_type-CORE0_PLL);
		break;
		case SYS_PLL:
		case DMC_PLL:
		case DEV0_PLL:
		case DEV1_PLL:
		case DEV2_PLL:
			reg_ctrl = XLP2XX_SYS_PLL_CTRL1+(pll_type-SYS_PLL)*4;
			reg_chg  = XLP2XX_SYS_PLL_CHG_CTRL;
			chg_mask = 1<<(pll_type-SYS_PLL);
		break;
		default:
			nlm_print("Unknown PLL type:%d\n", pll_type);
			return 0;
	}

	nlm_hal_write_sys_reg(NODE_0, reg_ctrl, pll_mult);
	nlm_hal_write_sys_reg(NODE_0, reg_chg,  chg_mask);
	while(nlm_hal_read_sys_reg(NODE_0, reg_chg) & chg_mask);

	freq = nlm_hal_xlp2xx_get_pllfreq_dyn(pll_type);
	return freq;
}

uint32_t xlp2xx_get_ref_clk(uint32_t* ref_clk_num, uint16_t* ref_clk_den)
{
	uint32_t frq_num;
	uint16_t frq_den;
	uint16_t rcs = xlp2xx_get_rcs();
	switch(rcs) {
		case 0x0:
			frq_num = 200000000;
			frq_den = 3;
		break;	
		case 0x1:
			frq_num = 100000000;
			frq_den = 1;
		break;	
		case 0x2:
			frq_num = 125000000;
			frq_den = 1;
		break;	
		case 0x3:
		default:
			frq_num = 400000000;
			frq_den = 3;
		break;	
	}
	*ref_clk_num = frq_num;
	*ref_clk_den = frq_den;
	NLM_HAL_DO_DIV(frq_num, frq_den);
	return frq_num;	
}

static uint32_t nlm_hal_xlp2xx_get_clkdev_frq(soc_device_id_t dev_type)
{
	uint32_t frq = 0, ref_clk_num;
	uint16_t ref_clk_den;
	uint16_t div;
	xlp2xx_clkdev_sel_t pll_sel;

	/* Test if disabled */
	if(nlm_hal_get_soc_clock_state(NODE_0, dev_type) == XLP_DISABLE)
		return 0;

	div = xlp2xx_get_clkdev_div(dev_type);
	pll_sel = xlp2xx_get_clk_dev_sel(dev_type);

	switch(pll_sel) {
		case SEL_REF_CLK:
			frq = xlp2xx_get_ref_clk(&ref_clk_num, &ref_clk_den);
		break;
		case SEL_DEV0PLL:
			frq = nlm_hal_xlp2xx_get_pllfreq_dyn(DEV0_PLL);
		break;
		case SEL_DEV1PLL:
			frq = nlm_hal_xlp2xx_get_pllfreq_dyn(DEV1_PLL);
		break;
		case SEL_DEV2PLL:
			frq = nlm_hal_xlp2xx_get_pllfreq_dyn(DEV2_PLL);
		break;
		default:
		break;
	}
	NLM_HAL_DO_DIV(frq, div);
	return frq;
}

static xlp2xx_clkdev_div_t best_divider(uint32_t reference, uint32_t target, uint32_t *error)
{
	xlp2xx_clkdev_div_t i;
	uint32_t temp;
	for (i = DIV_BYPASS; i < DIV_DIV8; i++) {
		temp = reference >> i;
		if(temp < (target + FREQ_RESOLUTION/2)) {
			if(temp > (target - FREQ_RESOLUTION/2))
				*error = 0;
			else
				*error = target - temp;
			return i;
		}
	}
	*error = ~(0);
	return i;
}

static uint32_t nlm_hal_xlp2xx_set_clkdev_frq(soc_device_id_t dev_type, uint32_t new_freq)
{
	uint32_t freq, error, min_error;
	uint16_t best_div, div, i;
	xlp2xx_clkdev_sel_t best_pll_sel, pll_sel;

	/* Test if the clock is disabled, enable it if so */
	if(nlm_hal_get_soc_clock_state(NODE_0, dev_type) == XLP_DISABLE)
		nlm_hal_soc_clock_enable(NODE_0, dev_type);

	/* Find the best source and divider */
	best_pll_sel = SEL_REF_CLK;
	freq = nlm_hal_get_ref_clk_freq();
	best_div = best_divider(freq, new_freq, &min_error);

	for(i = 0; i < 3; i++) {
		pll_sel = SEL_DEV0PLL + i;
		freq = nlm_hal_xlp2xx_get_pllfreq_dyn(DEV0_PLL + i);
		div = best_divider(freq, new_freq, &error);
		if(error < min_error) {
			best_pll_sel = pll_sel;
			best_div = div;
			min_error = error;
		}
	}

	nlm_hal_xlp2xx_dev_pll_cfg(dev_type, best_pll_sel, best_div);
	return nlm_hal_xlp2xx_get_clkdev_frq(dev_type);
}
#endif /* !NLM_HAL_XLP1 */

/**
 * @returns true if reference clock is 133MHz
 */
int nlm_hal_is_ref_clk_133MHz(void)
{
	int rv = 0;
#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx())
		return 3 == xlp2xx_get_rcs();
#endif
#ifndef NLM_HAL_XLP2
	rv = 1;
#endif
	return rv;
}

#ifndef NLM_HAL_XLP1
const char* nlm_hal_xlp2xx_get_dev_name(soc_device_id_t dev) {
	static char* name[] = {
		"NAE  ",
		"SAE  ",
		"RSA  ",
		"DTRE ",
		"CMP  ",
		"NAND ",
		"MMC  ",
		"NOR  ",
		"REGXF",
		"REGXS",
		"USB  ",
		"PIC  "
	};
	if(! ((dev >= XLP2XX_CLKDEVICE_NAE) && (dev <= XLP2XX_CLKDEVICE_PIC)))
		dev = XLP2XX_CLKDEVICE_NULL;
	return name[dev - XLP2XX_CLKDEVICE_NAE];
}

const char* nlm_hal_xlp2xx_get_pll_name(xlp2xx_pll_type_t pll) {
	static char* name[] = {
		"CORE0",
		"CORE1",
		"SOC  ",
		"DRAM ",
		"DEV0 ",
		"DEV1 ",
		"DEV2 "
	};
	static char *unknown = "UNKNOWN";
	if(! ((pll >= CORE0_PLL) && (pll <= DEV2_PLL)))
		return unknown;
	return name[pll];
}
#endif

#ifndef NLM_HAL_XLP2
const char* nlm_hal_get_dev_name(soc_device_id_t dev) {
	static char* name[] = {
		"NAE2X",
		"SAE  ",
		"RSA  ",
		"DTRE ",
		"CMP  ",
		"KBP  ",
		"DRAM ",
		"NAND ",
		"MMC  ",
		"NOR  ",
		"SOC2X",
		"REGXF",
		"REGXS",
		"SATA ",
		"NULL "
	};
	if(! ((dev>=DFS_DEVICE_NAE_2X)&&(dev<=DFS_DEVICE_SATA)))
	{
		dev = DFS_DEVICE_NULL;
	}
	return name[dev-DFS_DEVICE_NAE_2X];
}
const char* nlm_hal_get_pll_name(xlp_pll_type_t pll) {
	static char* name[] = {
		"CORE_PLL",
		"SOC_PLL ",
		"DRAM_PLL"
};
	static char *unknown = "UNKNOWN";
	if(! ((pll >= CORE_PLL) && (pll <= DDR_PLL)))
		return unknown;
	return name[pll];
}

#endif

/* Return reference clock for XLP8xx/4xx/3xx, return programmed PIC clock for XLP2xx */
uint32_t nlm_hal_get_xlp_pit_tick_rate(void)
{
	uint32_t rv = 0;
#ifndef NLM_HAL_XLP1
	if(is_nlm_xlp2xx() )
		return nlm_hal_xlp2xx_get_clkdev_frq(XLP2XX_CLKDEVICE_PIC);
#endif
#ifndef NLM_HAL_XLP2
	rv = nlm_hal_get_ref_clk_freq();
#endif
	return rv;
}

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/module.h>
EXPORT_SYMBOL(nlm_hal_xlp2xx_set_clkdev_frq);
EXPORT_SYMBOL(nlm_hal_xlp2xx_get_dev_name);
EXPORT_SYMBOL(nlm_hal_get_soc_freq);
EXPORT_SYMBOL(nlm_hal_set_soc_freq);
EXPORT_SYMBOL(nlm_hal_get_core_freq);
EXPORT_SYMBOL(nlm_hal_set_core_freq);
EXPORT_SYMBOL(nlm_hal_cpu_freq);
EXPORT_SYMBOL(nlm_hal_is_ref_clk_133MHz);
EXPORT_SYMBOL(nlm_hal_get_ref_clk_freq);
EXPORT_SYMBOL(nlm_hal_get_xlp_pit_tick_rate);
#endif
