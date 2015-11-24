/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
 * All Rights Reserved
 *
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
 * #BRCM_2# */


#ifndef _FLASH_PCMCIA_H_
#define _FLASH_PCMCIA_H_

#define R_FLASH_PCMCIA_BASE_ADDR                    0x0
#define   O_BASE_ADDR__base_addr                    0
#define   W_BASE_ADDR__base_addr                    16
#define R_FLASH_PCMCIA_BASE_ADDR_MASK               0x1
#define   O_BASE_ADDR_MASK__base_amask              0
#define   W_BASE_ADDR_MASK__base_amask              16
#define R_FLASH_PCMCIA_DEV_PARM                     0x2
#define   O_DEV_PARM__burst_mode_en                 0
#define   O_DEV_PARM__burst_mode                    1
#define   W_DEV_PARM__burst_mode                    3
#define   O_DEV_PARM__wait_en                       4
#define   O_DEV_PARM__wait_pol                      5
#define   O_DEV_PARM__mx_addr                       6
#define   O_DEV_PARM__dwidth                        7
#define   W_DEV_PARM__dwidth                        2
#define   O_DEV_PARM__pcmcia_en                     9
#define   O_DEV_PARM__genif_en                      10
#define   O_DEV_PARM__genparity_en                  11
#define   O_DEV_PARM__genparity_type                12
#define   O_DEV_PARM__adv_type                      13
#define R_FLASH_TIMING_PARM_0                       0x3
#define   O_FLASH_TIMING_PARM_0__ale_width          0
#define   W_FLASH_TIMING_PARM_0__ale_width          3
#define   O_FLASH_TIMING_PARM_0__ale_to_cs          3
#define   W_FLASH_TIMING_PARM_0__ale_to_cs          3
#define   O_FLASH_TIMING_PARM_0__cs_width           6
#define   W_FLASH_TIMING_PARM_0__cs_width           5
#define   O_FLASH_TIMING_PARM_0__wait_to_data       11
#define   W_FLASH_TIMING_PARM_0__wait_to_data       5
#define   O_FLASH_TIMING_PARM_0__cs_to_oe           16
#define   W_FLASH_TIMING_PARM_0__cs_to_oe           3
#define   O_FLASH_TIMING_PARM_0__cs_to_we           19
#define   W_FLASH_TIMING_PARM_0__cs_to_we           3
#define   O_FLASH_TIMING_PARM_0__oe_to_cs           22
#define   W_FLASH_TIMING_PARM_0__oe_to_cs           2
#define   O_FLASH_TIMING_PARM_0__we_to_cs           24
#define   W_FLASH_TIMING_PARM_0__we_to_cs           4
#define   O_FLASH_TIMING_PARM_0__cs_to_cs           28
#define   W_FLASH_TIMING_PARM_0__cs_to_cs           4
#define R_FLASH_TIMING_PARM_1                       0x4
#define   O_FLASH_TIMING_PARM_1__oe_width           0
#define   W_FLASH_TIMING_PARM_1__oe_width           6
#define   O_FLASH_TIMING_PARM_1__we_width           6
#define   W_FLASH_TIMING_PARM_1__we_width           6
#define   O_FLASH_TIMING_PARM_1__wait_timeout       12
#define   W_FLASH_TIMING_PARM_1__wait_timeout       15
#define R_PCMCIA_CONFIG                             0x5
#define   O_PCMCIA_CONFIG__pcmcia_en                0
#define   O_PCMCIA_CONFIG__reg_access               1
#define   O_PCMCIA_CONFIG__reset                    2
#define   O_PCMCIA_CONFIG__cdmask                   4
#define   O_PCMCIA_CONFIG__wpmask                   5
#define   O_PCMCIA_CONFIG__rdymask                  6
#define   O_PCMCIA_CONFIG__rybymask                 7
#define R_PCMCIA_STATUS                             0x6
#define   O_PCMCIA_STATUS__bvd1_sts                 0
#define   O_PCMCIA_STATUS__bvd2_sts                 1
#define   O_PCMCIA_STATUS__cd1_sts                  2
#define   O_PCMCIA_STATUS__cd2_sts                  3
#define   O_PCMCIA_STATUS__vs1_sts                  4
#define   O_PCMCIA_STATUS__vs2_sts                  5
#define   O_PCMCIA_STATUS__wp_sts                   6
#define   O_PCMCIA_STATUS__rdy_sts                  7
#define   O_PCMCIA_STATUS__ryby_sts                 8
#define   O_PCMCIA_STATUS__cd_intr                  9
#define   O_PCMCIA_STATUS__wp_intr                  10
#define   O_PCMCIA_STATUS__rdy_intr                 11
#define   O_PCMCIA_STATUS__illegal_addr_intrpt      12
#define   O_PCMCIA_STATUS__mutl_cs_intrpt           13
#define   O_PCMCIA_STATUS__wait_timeout_intrpt      14
#define   O_PCMCIA_STATUS__ryby_intrpt              16
#define   O_PCMCIA_STATUS__werr_intrpt              17
#define   O_PCMCIA_STATUS__illegal_pcmcia_intrpt    18
#define R_GENERIC_REGION_STATUS                     0x7
#define   O_GENERIC_REGION_STATUS__cs_err_intrpt    0
#define   W_GENERIC_REGION_STATUS__cs_err_intrpt    8
#define R_GENERIC_ERROR_ADDR                        0x8
#define   O_GENERIC_ERROR_ADDR__err_addr            0
#define   W_GENERIC_ERROR_ADDR__err_addr            32


#endif /* _FLASH_PCMCIA_H_ */

