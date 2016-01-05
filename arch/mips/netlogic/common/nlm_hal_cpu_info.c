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

#include "nlm_hal.h"
#include "nlm_hal_macros.h"
#include "nlm_hal_xlp_dev.h"

extern void *memset(void *s, int c, size_t n);

/* Test if ICI is disabled - if so call it XLP4xx */
int is_nlm_xlp4xx(void)
{
	return (efuse_cfg1() & 0x7) == 0x7;
}

int nlm_xlp2xx_has_cmp(void)
{
    uint32_t cfg1 = efuse_cfg1();
    return (cfg1 & (1 << XLP2XX_CMP_BLOCK_INDEX)) ? 0 : 1;
}

int nlm_xlp2xx_has_crypto(void)
{
    uint32_t cfg1 = efuse_cfg1();
    return (cfg1 & (1 << XLP2XX_CRYPTO_BLOCK_INDEX)) ? 0 : 1;
}

int nlm_xlp2xx_has_rsa(void)
{
    uint32_t cfg2 = efuse_cfg2();
    return (cfg2 & (1 << XLP2XX_RSA_BLOCK_INDEX)) ? 0 : 1;
}

int nlm_xlp2xx_has_regx(void)
{
    uint32_t cfg2 = efuse_cfg2();
    return (cfg2 & (1 << XLP2XX_REGX_BLOCK_INDEX)) ? 0 : 1;
}

int is_nlm_xlp2xx_cp(void)
{
	return num_xlp_threads() == 1;
}

int is_nlm_xlp1xx(void)
{
	if(num_xlp_threads() == 1) {			// XLP202/201/101
		if(num_xlp_cores() > 1)
			return 0;							// XLP202
		else {
			if(nlm_xlp2xx_has_regx())
				return 0;						// XLP201
			else
				return 1;						// XLP101
		}
	} else {								// XLP208/204/108/104
		if (nlm_xlp2xx_has_cmp() && nlm_xlp2xx_has_crypto() && nlm_xlp2xx_has_rsa() && nlm_xlp2xx_has_regx())
			return 0;							// No accelerators disabled - XLP208/204
		else
			return 1;							// At least one accelerator disabled - XLP108/104
	}
}

/* Returns the 16 bit code for the chip type as follows:
 *   Family type (8, 4, 3, 2, 1) << 12
 *   # of cores << 4
 *   # of CPUs per core
 * XLP832 = 0x8084, XLP101 = 0x1011
 * Also populates nthreads and ncores
 */
static uint16_t get_chipid(unsigned int *ncores, unsigned int *nthreads)
{
	uint16_t chipid;

	/* eagle 4xx 8xx BX, unfused chip treat as 8xx */
	*ncores = num_xlp_cores();
	*nthreads = num_xlp_threads();
	switch(get_proc_id()) {
	case CHIP_PROCESSOR_ID_XLP_8_4_XX:
		chipid = is_nlm_xlp4xx() ? 0x4000 : 0x8000;
		break;
	case CHIP_PROCESSOR_ID_XLP_3XX:
		chipid = 0x3000;
		break;
	case CHIP_PROCESSOR_ID_XLP_2XX:
		chipid = is_nlm_xlp1xx() ? 0x1000 : 0x2000;
		break;
	default:
		return 0;
	}

	return chipid + ((*ncores << 4) + *nthreads);
}

/* Returns 0 if all is well, -1 for unknown / unsupported CPU */
int nlm_hal_get_cpuinfo(struct nlm_netl_proc_info* cpu_info)
{
	unsigned int type, ncores, nthreads, lite = 0;
	uint32_t hw_rev = nlm_read_prid() & 0xff;
	char major = 'A';
	uint32_t cfg0 = efuse_cfg0();

	memset(cpu_info, 0, sizeof(struct nlm_netl_proc_info));
	cpu_info->proc_id = get_proc_id();
	cpu_info->chipid = get_chipid(&ncores, &nthreads);
	cpu_info->revision = hw_rev;
	type = cpu_info->chipid >> 12;

	switch(cpu_info->proc_id) {
	case CHIP_PROCESSOR_ID_XLP_2XX:
		if(hw_rev >= XLP2XX_REVISION_B0) {
			major = 'B';
			hw_rev -= XLP2XX_REVISION_B0;
		}
		break;
	case CHIP_PROCESSOR_ID_XLP_3XX:
		if(hw_rev >= XLP3XX_REVISION_B0) {
			major = 'B';
			hw_rev -= XLP3XX_REVISION_B0;
		}
		if((cfg0 >> 4) & 0xF)
			lite = 1;	// Not differentiating lite, lite plus, lite plus 2 variants
		break;
	case CHIP_PROCESSOR_ID_XLP_8_4_XX:
		if(hw_rev >= XLP_REVISION_B0) {
			major = 'B';
			hw_rev -= XLP_REVISION_B0;
		}
	default:
		nlm_print("[%s] Unknown processor (proc_id = 0x%08X)\n",
				__func__, nlm_read_prid());
		return -1;
	}

	sprintf(cpu_info->cpu_info_str, "XLP%u%02u%sRev %c%1u",
			type, (ncores * nthreads), lite ? "L " : " ", major, hw_rev);

	return 0;
}

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/module.h>
EXPORT_SYMBOL(nlm_hal_get_cpuinfo);
EXPORT_SYMBOL(is_nlm_xlp4xx);
EXPORT_SYMBOL(is_nlm_xlp2xx_cp);
EXPORT_SYMBOL(is_nlm_xlp1xx);
#endif
