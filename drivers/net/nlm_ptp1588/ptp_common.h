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

#ifndef PTP_COMMON_H
#define PTP_COMMON_H
                
enum {
    PTP_INIT     = 1, 
    PTP_TX_TIMESTAMP,        
    PTP_SET_TIME,
    PTP_GET_INF_IDX,
    PTP_SET_INF_IDX,
    PTP_MAX_IOCTL
 };
            
typedef struct _ptp_ts_t_ {
    u32    ts_msb;
    u32    ts_lsb;
    } ptp_ts_t;

typedef struct _ptp_clock_val_t_ {
       u32 inf_type;
       u32 inf_idx;
       u32 offset0;
       u32 offset1;
       u32 frac_div;
       u32 frac_mul;
       u32 src_div;
       u32 src_clk;
    }ptp_clk_t;

typedef struct _intf_type_t_{
     unsigned char name[8][10];
     unsigned long numif;
     unsigned int inf_idx[8];
 } intf_type_t;

#endif
