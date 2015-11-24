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


#ifndef __ASM_NLM_PERF_CTR_H
#define __ASM_NLM_PERF_CTR_H

#include <asm/mipsregs.h>

#define CP0_PERF_CTR  $25

/* Subset of perf ctr events */

#define PERF_CTR_INSTR_FETCHED           0
#define PERF_CTR_ICACHE_MISSES           1
#define PERF_CTR_SLEEP_CYCLES           12
#define PERF_CTR_INSTR_RETIRED          17
#define PERF_CTR_BRJMP_INSTR            20
#define PERF_CTR_BRJMP_FLUSH            21
#define PERF_CTR_REPLAYFLUSH            27
#define PERF_CTR_REPLAYFLUSH_LDUSE      28
#define PERF_CTR_L1_HIT                 38
#define PERF_CTR_L1_REF                 39
#define PERF_CTR_SNOOP_UPGRADE_FAIL     47
#define PERF_CTR_SNOOP_TRANSFERS        48
#define PERF_CTR_SNOOP_HITS             49
#define PERF_CTR_SNOOP_OPS              50
#define PERF_CTR_CYCLES                 63

/* 2 sets of counters are supported across all threads of a core */
#define PERF_CTR_EVENT0        0
#define PERF_CTR_EVENT0_VALUE  1
#define PERF_CTR_EVENT1        2
#define PERF_CTR_EVENT1_VALUE  3

#define PERF_CTR_DEFAULT 0x0f /* disable int, enable counting in all modes */

#define perf_ctr_start(ctr, event, global, thr) __write_32bit_c0_register($25, ctr, ((PERF_CTR_DEFAULT)|((global)<<13)|((thr)<<11)|((event)<<5)) ) 

#define perf_ctr_stop(ctr) __write_32bit_c0_register($25, ctr, 0)

#define perf_ctr_reset(ctr) __write_32bit_c0_register($25, ctr, 0)

#define perf_ctr_read(ctr) __read_32bit_c0_register($25, ctr)

#endif /* __ASM_NLM_PERF_CTR_H */
