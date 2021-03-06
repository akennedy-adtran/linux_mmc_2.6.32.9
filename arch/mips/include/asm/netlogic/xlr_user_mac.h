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


#ifndef _ASM_NLM_XLR_USER_MAC_H
#define _ASM_NLM_XLR_USER_MAC_H

#define NLM_USER_MAC_MMAP_VIRT_START 0x60000000
#define NLM_USER_MAC_SIZE            0x800000

#ifndef __ASSEMBLY__
#include <asm/netlogic/sim.h>

extern void nlm_common_user_mac_update_time(void);

struct xlr_user_mac_config {
	int l4_extract;
	int fast_syscall;
};

extern struct xlr_user_mac_config xlr_user_mac;

static __inline__ int xlr_user_mac_l4_extract(void)
{
	return xlr_hybrid_user_mac() ? xlr_user_mac.l4_extract  : 0;
}

static __inline__ int xlr_user_mac_fast_syscall(void)
{
	return xlr_hybrid_user_mac() && (xlr_user_mac.fast_syscall == 1);
}

#endif

#endif
