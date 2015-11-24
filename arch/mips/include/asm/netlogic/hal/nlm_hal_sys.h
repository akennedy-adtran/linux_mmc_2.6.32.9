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

#ifndef _NLH_SYS_H
#define _NLH_SYS_H
#if !defined(__KERNEL__) && !defined(NLM_HAL_UBOOT)
#include <stdint.h>
#endif

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

/* Base frequency for XLP2 PLLs, numerator and denominator to yield 33.333 MHz */
#define XLP2_PLL_BASE		400000000
#define XLP2_PLL_STEP_NUM	100000000
#define XLP2_PLL_STEP_DEN	3

/* 1 MHz resolution for frequency setting */
#define FREQ_RESOLUTION 1000000ULL

/* System Clock Reg: Device:6, Func:5
 * 0x56 Clock Disable Control
 * 0x57 Clock Reset Control
 * 0x58 Clock Bypass Control
 * 0x59 Clock Divider Increment Control
 * 0x5A clock Divider Decrement Control
 */
typedef enum soc_dfs_device {
	DFS_DEVICE_NAE_2X		= 0,
	DFS_DEVICE_SAE			= 1,
	DFS_DEVICE_RSA			= 2,
	DFS_DEVICE_DTRE			= 3,
	DFS_DEVICE_CMP			= 4, /*xlp8xx only*/
	DFS_DEVICE_KBP			= 5, /*xlp8xx only*/
	DFS_DEVICE_DMC			= 6,
	DFS_DEVICE_NAND			= 7,
	DFS_DEVICE_MMC			= 8,
	DFS_DEVICE_NOR			= 9,
	DFS_DEVICE_CORE			= 10, /*0xA*/
	DFS_DEVICE_REGEX_FAST	= 11, /*xlp3xx only 0xB*/
	DFS_DEVICE_REGEX_SLOW	= 12, /*xlp3xx only 0xC*/
	DFS_DEVICE_SATA			= 13, /*xlp3xx only 0xD*/
	DFS_DEVICE_NULL			= 14,

	XLP2XX_CLKDEVICE_NAE	= 0x10,
	XLP2XX_CLKDEVICE_SAE	= 0x11,
	XLP2XX_CLKDEVICE_RSA	= 0x12,
	XLP2XX_CLKDEVICE_GDX	= 0x13,
	XLP2XX_CLKDEVICE_CMP	= 0x14,
	XLP2XX_CLKDEVICE_NAND	= 0x15,
	XLP2XX_CLKDEVICE_MMC	= 0x16,
	XLP2XX_CLKDEVICE_GBU	= 0x17,
	XLP2XX_CLKDEVICE_RGXF	= 0x18,
	XLP2XX_CLKDEVICE_RGXS	= 0x19,
	XLP2XX_CLKDEVICE_USB	= 0x1a,
	XLP2XX_CLKDEVICE_PIC	= 0x1b,
	XLP2XX_CLKDEVICE_NULL	= 0x1c,

	INVALID_DFS_DEVICE = 0xFF
} soc_device_id_t;

/* So u-boot can configure SOC accelerator frequencies */
#ifdef CONFIG_NETL2XX
#define MAX_SKUS			3		// Number of speed grades
#define MAX_SOC_DEVS		13		// Number of devices in the speed grade SOC speed table
#endif
#ifdef CONFIG_NETL3XX
#define MAX_SKUS			2
#define MAX_SOC_DEVS		14
#endif
#ifdef CONFIG_NETL8XX
#define MAX_SKUS			2
#define MAX_SOC_DEVS		11
#endif

#ifdef NLM_HAL_LINUX_KERNEL 
#define NLM_HAL_DO_DIV(n, base)   if(base) { do_div((n), (base)); }
#else
#define NLM_HAL_DO_DIV(n, base)   if(base) { ((n) /= (base)); }
#endif 

/*XLP2XX soc dev*/
typedef enum xlp2xx_pll_type {
	CORE0_PLL = 0,
	CORE1_PLL = 1,
	SYS_PLL   = 2,
	DMC_PLL   = 3,
	DEV0_PLL  = 4,
	DEV1_PLL  = 5,
	DEV2_PLL  = 6
} xlp2xx_pll_type_t;

typedef enum xlp_pll_type {
	CORE_PLL = 0,
	SOC_PLL  = 1,
	DDR_PLL  = 2
} xlp_pll_type_t;

typedef enum xlp2xx_clkdev_sel{
	SEL_REF_CLK	=0x0,
	SEL_DEV0PLL	=0x1,
	SEL_DEV1PLL	=0x2,
	SEL_DEV2PLL	=0x3
} xlp2xx_clkdev_sel_t;

typedef enum xlp2xx_clkdev_div{
	DIV_BYPASS	=0x0,
	DIV_DIV2	=0x1,
	DIV_DIV4	=0x2,
	DIV_DIV8	=0x3
} xlp2xx_clkdev_div_t;

extern uint8_t nlm_hal_get_soc_clock_state(int node, soc_device_id_t device);
extern void nlm_hal_soc_clock_enable(int node, soc_device_id_t device);
extern void nlm_hal_soc_clock_disable(int node, soc_device_id_t device);
extern void nlm_hal_soc_clock_reset(int node, soc_device_id_t device);

/* XLP1xx APIs */
extern const char* nlm_hal_get_dev_name(soc_device_id_t dev);
extern const char* nlm_hal_get_pll_name(xlp_pll_type_t pll);
extern uint32_t nlm_hal_get_soc_dfs_val(int node, soc_device_id_t device);
extern uint64_t nlm_hal_get_pll_freq(int node, xlp_pll_type_t pll);
extern uint64_t nlm_hal_get_soc_freq(int node, soc_device_id_t device);
extern uint64_t nlm_hal_set_soc_freq(int node, soc_device_id_t device, uint64_t freq);
extern uint64_t nlm_hal_get_core_freq(int node, uint8_t core);
extern uint64_t nlm_hal_set_core_freq(int node, uint8_t core, uint64_t freq);
extern unsigned long long nlm_hal_cpu_freq(void);
extern int nlm_hal_is_ref_clk_133MHz(void);
extern uint64_t nlm_hal_get_ref_clk_freq(void);
extern unsigned int nlm_hal_get_xlp_pit_tick_rate(void);

/* XLP2xx APIs*/
extern const char* nlm_hal_xlp2xx_get_dev_name(soc_device_id_t dev);
extern const char* nlm_hal_xlp2xx_get_pll_name(xlp2xx_pll_type_t pll);
extern uint64_t xlp2xx_get_ref_clk(uint64_t* ref_clk_num, uint32_t* ref_clk_den);
extern void nlm_hal_xlp2xx_dev_pll_cfg(soc_device_id_t dev_type,
		xlp2xx_clkdev_sel_t dev_pll_sel, xlp2xx_clkdev_div_t div);
extern uint64_t nlm_hal_xlp2xx_set_pllfreq_dyn(xlp2xx_pll_type_t pll_type, uint64_t freq);
extern uint64_t nlm_hal_xlp2xx_get_pllfreq_dyn(xlp2xx_pll_type_t pll_type);
extern uint64_t nlm_hal_xlp2xx_set_clkdev_frq(soc_device_id_t dev_type, uint64_t freq);
extern uint64_t nlm_hal_xlp2xx_get_clkdev_frq(soc_device_id_t dev_type);

#define NLM_HALT_IF(cond) while(cond) { \
				nlm_print("ERROR: %s\n", __FUNCTION__); \
				nlm_mdelay(10000); \
			}

#define NLM_HALT_IF_XLPII() NLM_HALT_IF(is_nlm_xlp2xx())
#define XLP_DISABLE 1
#define XLP_ENABLE  0

#endif /* _NLH_SYS_H */
