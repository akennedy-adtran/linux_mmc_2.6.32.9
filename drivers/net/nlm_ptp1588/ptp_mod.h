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

#ifndef PTP_MOD_H
#define PTP_MOD_H


#define     PTP_SOURCE_DIV          0x24a
#define     PTP_SOURCE              0x24b
#define     PTP_FRAC_DIV            0x70
#define     PTP_FRAC_MUL            0x71
#define     PTP_INCR                0x72
#define     PTP_OFFSET0             0x73
#define     PTP_OFFSET1             0x74
#define     PTP_TX_LATCH_LSB        0x75
#define     PTP_TX_LATCH_MSB        0x76
#define     PTP_CTRL                0x77
#define     O_PTP_CTRL_RTC_LATCH    12
#define     PTP_TIMER_LATCH_VAL     0x7e
#define     PTP_TIMER_LATCH_VAL1    0x7f

#define     O_PTP_Rx1588TS          11

#define     PTP_BIU_HALF_CLOCK      (1<<23)
#define     PTP_CLK_SRC_GMAC        0x0
#define     PTP_CLK_SRC_CORE        0x1
#define     PTP_DEV_MAJOR_NUM       250


#define   NUM_BITS_1        0x1
#define   NUM_BITS_2        0x3
#define   NUM_BITS_3        0x7
#define   NUM_BITS_32       0xffffffff  

#define   PTP_DRV_NAME      "ptp1588"     
#endif
