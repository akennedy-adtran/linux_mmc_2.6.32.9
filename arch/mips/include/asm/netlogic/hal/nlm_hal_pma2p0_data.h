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
#ifndef _NLM_HAL_VESMI_DATA_H_
#define _NLM_HAL_PMA2P0_DATA_H_

#define PMA2P0_CMD_RETRIES	100		// Moved from hal_nae_interface hard-coded at 100

/*156 Mhz related functions*/
extern void nlm_hal_config_pma2p0_mem_16G_4page(void);
extern void nlm_hal_config_pma2p0_mem_16G(void);
extern void nlm_hal_config_pma2p0_mem_12G_4page(void);
extern void nlm_hal_config_pma2p0_mem_12G(void);
extern void nlm_hal_config_pma2p0_mem_xaui(void);
extern void nlm_hal_config_pma2p0_mem_xaui_4page(void);

/*125 Mhz related functions*/
void nlm_hal_config_pma2p0_mem_16G_4page_125(void);
void nlm_hal_config_pma2p0_mem_16G_125(void);
void nlm_hal_config_pma2p0_mem_12G_4page_125(void);
void nlm_hal_config_pma2p0_mem_12G_125(void);
void nlm_hal_config_pma2p0_mem_xaui_4page_125(void);
void nlm_hal_config_pma2p0_mem_xaui_125(void);	
#endif


