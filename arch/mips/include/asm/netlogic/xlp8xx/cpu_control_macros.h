/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#ifndef __CPUCONTROL_MACROS_H__
#define __CPUCONTROL_MACROS_H__
#include <asm/netlogic/xlp8xx/cpu.h>
#include <asm/netlogic/xlp8xx/xlp_sys.h>

#define CP0_EBASE			$15
#ifdef CONFIG_64BIT
#define NMI_BASE			0xffffffffbfc00000UL
#else
#define NMI_BASE			0xbfc00000UL	
#endif
#define NMI_BASE_ASM		0xbfc00000

#define LSU_DEFEATURE		0x304
#define LSU_DEBUG_ADDR		0x305
#define LSU_DEBUG_DATA0		0x306
#define MMU_SETUP			0x400
#define THREAD_FAIRNESS		0x601
#define SCHED_DEFEATURE		0x700
#define L2_FEATURE_CTRL0	0x800

#ifndef __ASSEMBLY__
#define	 XLP_THREADS_PER_CORE	4
#define  XLP_CORES_PER_NODE		7
void enable_cpus(unsigned int, unsigned int);
u32 get_core_dfs(int);
u32 change_cpu_freq(int, int);
#endif	// __ASSEMBLY__
#endif /* __CPUCONTROL_MACROS_H__ */
