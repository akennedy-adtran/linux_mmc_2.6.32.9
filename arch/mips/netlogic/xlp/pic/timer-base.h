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

#if !defined XLP_PIC_TIMER_BASE_H
#define XLP_PIC_TIMER_BASE_H

/* These names follow an important convention.
 * [8] should denote the systimer no. */
#define XLP_SYSTIMER0	"sys_pit_0"
#define XLP_SYSTIMER1	"sys_pit_1"
#define XLP_SYSTIMER2	"sys_pit_2"
#define XLP_SYSTIMER3	"sys_pit_3"
#define XLP_SYSTIMER4	"sys_pit_4"
#define XLP_SYSTIMER5	"sys_pit_5"

#define XLP_CSDTIMER	"sys_pit_1"
#define XLP_CEDTIMER	"sys_pit_0"

#define XLP_SYSTIMER_OFFSET	8	/* Index of the above string denoting the timer number; we bank on the fact that there are 8 timers only */
#define xlp_str2timer(str)\
({\
	(u32) (str[XLP_SYSTIMER_OFFSET] - '0');\
})
#define xlp_str2node(str)\
({\
	(u32) (str[XLP_SYSTIMER_OFFSET + 1] - '0');\
})

extern spinlock_t pit_lock;
int xlp_timer_enable(u8 nid, u8 timer);
void xlp_timer_disable(u8 nid, u8 timer);
u64 xlp_timer_regread(u8 nid, u32 reg);	/* reads any register of timers after locking*/
int xlp_timer_regwrite(u8 nid, u8 timer, u64 val);/* writes any reg. of timers after locking */
void __xlp_timer_ctrl(u8 nid, u8 timer, u8 en);	/* enable and disable without locking */

#define xlp_timer_mvwrite(nid, timer, val)\
({\
xlp_timer_regwrite(nid, (u32)XLP_PIC_SYSTIMER_MAXVAL(timer), val);\
 })

#define xlp_timer_ctread(nid, timer)\
({\
xlp_timer_regread(nid, (u32)XLP_PIC_SYSTIMER_COUNT(timer));\
 })
#endif
