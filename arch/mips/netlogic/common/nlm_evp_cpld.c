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


#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/netlogic/hal/nlm_evp_cpld.h>
#include <asm/netlogic/hal/nlm_hal_sys.h>
#else
#include <byteorder.h>
#include "nlm_evp_cpld.h"
#include "nlm_hal_sys.h"
#endif


static nlm_xlp_nor_t xlp_nor_dev[8] = {
{ 0x16000000, SIZE_16MB, 0, 0x2e00 },
{ 0xffffffff, 0, 0, 0 },
{ 0x17000000, SIZE_1MB , 1, 0x2C00 },
{ 0x17200000, SIZE_1MB , 0, 0x2C00 },
{ 0x17300000, SIZE_1MB , 0, 0x2f84 },
{ 0xffffffff, 0, 0, 0 },
{ 0xffffffff, 0, 0, 0 },
{ 0xffffffff, 0, 0, 0 },
};

static inline uint32_t nlm_hal_nor_read(uint32_t reg)
{
        return nlm_hal_read_32bit_reg(NLM_NOR_CFG_BASE, reg);
}

static inline void nlm_hal_nor_write(uint32_t reg, uint32_t val)
{
        nlm_hal_write_32bit_reg(NLM_NOR_CFG_BASE, reg, val);
}

uint16_t nlm_hal_cpld_read_16(int cs, uint16_t reg)
{
#ifndef CONFIG_N511
        uint16_t val;
	if ((cs == 3) || (cs ==4))
	        nlm_hal_nor_write(XLP_NOR_DEVPARAM + cs , 0x2f84);
	
	val = nlm_hal_read_16bit_reg(xlp_nor_dev[cs].base, reg);
        return (xlp_nor_dev[cs].swap ? le16_to_cpu(val): be16_to_cpu(val));
#else
	return 0;
#endif
}

void nlm_hal_cpld_write_16(int cs, uint16_t val, uint16_t reg)
{
#ifndef CONFIG_N511
        uint16_t data = xlp_nor_dev[cs].swap ? cpu_to_le16(val): cpu_to_be16(val);
	if ((cs == 3) || (cs ==4))
	        nlm_hal_nor_write(XLP_NOR_DEVPARAM + cs , 0x2d84);

        nlm_hal_write_16bit_reg(xlp_nor_dev[cs].base, reg, data);
#else
#endif
}

int nlm_xlp_boardver(void)
{
	uint16_t data = nlm_hal_cpld_read_16(2, 5);
	return ((EVP_VER(data) >> 3) + 1);
}

int nlm_xlp_cpldver(void)
{
	return nlm_hal_cpld_read_16(2, 0);
}

int is_xlp_evp1(void)
{
#ifndef CONFIG_N511
        uint16_t data = nlm_hal_cpld_read_16(2, 5);

        if (EVP_VER(data) == 0)
                return 1;
        else
#endif
                return 0;
}

int is_xlp_evp2(void)
{
#ifndef CONFIG_N511
        uint16_t data = nlm_hal_cpld_read_16(2, 5);

        if (EVP_VER(data))
                return 1;
        else
#endif
                return 0;
}

int is_ilk_card_onslot(int slot)
{
	uint16_t data = nlm_hal_cpld_read_16(2, 6);

	slot >>= 1;

	if (DC_TYPE(data, slot) == DC_ILK) 
		return 1;
	else
		return 0;
}

#if defined(NLM_HAL_LINUX_USER) || defined(NLM_HAL_LINUX_KERNEL)
/* cop0 hwren register should be set */
static inline int my_cpu_id(void)
{
	unsigned int cpu = 0;

	__asm__ volatile (".set push\n"
			".set noreorder\n"
			".set arch=xlp\n"
			"rdhwr %0, $0\n"
			".set pop\n"
			: "=r" (cpu)
			:);

	return cpu;
}
#endif

int nlm_get_interface_type(int node, int slot)
{
	uint16_t data;
       
	/* there is no cpld in the existing multi node board for node 1-3 */
#if defined(NLM_HAL_LINUX_USER) || defined(NLM_HAL_LINUX_KERNEL)
	if(my_cpu_id() >= 32)
		return DC_NOT_PRSNT;
#endif
	
	data = nlm_hal_cpld_read_16(2, 6);
//	nlm_print("Slot present status 0x%x\n", (data & 0xFF));
	if (slot == 4)
		return DC_SGMII;

	if (nlm_xlp_cpldver() == 0)
		return DC_NOT_PRSNT;

#ifdef SKIP_INTERFACE_TYPE_FROMCPLD
	return DC_NOT_PRSNT;
#else
	if (slot == 2)
		slot >>= 1;
	else if (slot == 1)
		slot <<= 1;
	return DC_TYPE(data, slot);
#endif
}

int xlp_cpld_init(uint32_t cs)
{
	unsigned long base = xlp_nor_dev[cs].base;
	unsigned long limit = base + xlp_nor_dev[cs].size - 1;

	if (cs > NLM_XLP_MAX_CS)
		return -1;

	nlm_hal_nor_write(XLP_NOR_CS_BASE + cs , (base >> 8));
	nlm_hal_nor_write(XLP_NOR_CS_LIMIT + cs , (limit >> 8));

	nlm_hal_nor_write(XLP_NOR_DEVPARAM + cs , xlp_nor_dev[cs].devparam); 

	nlm_hal_nor_write(XLP_NOR_DEV_TIME0 + (cs * 2), 0x4F646EC2 );
	nlm_hal_nor_write(XLP_NOR_DEV_TIME1 + (cs * 2), 0x8CF3);

	return 0;
}

void set_gbu_frequency(int node, int frequency)
{
	const uint32_t mhz = 1000000;
	uint32_t set_freq;
	if(is_nlm_xlp2xx())
		set_freq = nlm_hal_set_soc_freq(node, XLP2XX_CLKDEVICE_NOR, frequency * mhz);
	else
		set_freq = nlm_hal_set_soc_freq(node, DFS_DEVICE_NOR, frequency * mhz);
	NLM_HAL_DO_DIV(set_freq, mhz);
	nlm_print("GBU Frequency set to %u MHz\n", set_freq);
}

void nlm_hal_cpld_init(int node)
{
	int i;

#if defined(NLM_CORTINA_SUPPORT)
	set_gbu_frequency(node, NLM_GBU_FREQ_ILK);
#else
	set_gbu_frequency(node, NLM_GBU_FREQ_DEFAULT);
#endif

	for(i=2; i<5; i++)
		xlp_cpld_init(i);
}

#ifdef NLM_HAL_LINUX_KERNEL
EXPORT_SYMBOL(nlm_hal_cpld_init);
EXPORT_SYMBOL(nlm_get_interface_type);
EXPORT_SYMBOL(is_xlp_evp1);
EXPORT_SYMBOL(is_xlp_evp2);
EXPORT_SYMBOL(nlm_xlp_boardver);
EXPORT_SYMBOL(is_ilk_card_onslot);
EXPORT_SYMBOL(nlm_xlp_cpldver);
EXPORT_SYMBOL(nlm_hal_cpld_read_16);
EXPORT_SYMBOL(nlm_hal_cpld_write_16);
#endif

