
/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
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

#ifndef __NLM_CPLD_H__
#define __NLM_CPLD_H__

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/netlogic/hal/nlm_hal.h>
#else
#include "nlm_hal.h"
#endif

#define CORTINA_ILK1_BASE	0x17100000
#define CORTINA_ILK2_BASE	0x17200000
#define NLM_GBU_FREQ_DEFAULT	133
#define NLM_GBU_FREQ_ILK	16
	
#define NLM_XLP_MAX_CS  7

typedef struct {
        uint32_t        base;
        uint32_t        size;
        uint32_t        swap;
        uint32_t        devparam;
}nlm_xlp_nor_t;

#define SWAB16(x)       ((uint16_t)((((uint16_t)x & (uint16_t)0x00FFU) << 8) |  \
                        (((uint16_t)x & (uint16_t)0xFF00U) >> 8)))

#define SIZE_16MB       (0x1000000)
#define SIZE_1MB        (0x100000)

#define DC_ILK          0
#define DC_HIGIG        0
#define DC_SGMII        1
#define DC_XAUI         2
#define DC_NOT_PRSNT    3
#define DC_RXAUI        6 /*not support by CPLD : Defined for software using only */

#define DC_TYPE(val,slot)       ((val >> (slot * 2)) & 0x3)
#define EVP_VER(val)            (val & 0x8)

#endif
