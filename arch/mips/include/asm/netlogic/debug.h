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


#ifndef _ASM_NLM_DEBUG_H
#define _ASM_NLM_DEBUG_H

/*Enable below macro to enable net stats. */
//#define CONFIG_NLM_STATS
extern void prom_printf(char *fmt, ...);
#include <linux/threads.h>
#include <asm/atomic.h>

enum {
  //cacheline 0
  MSGRNG_INT,
  MSGRNG_PIC_INT,
  MSGRNG_MSG,
  MSGRNG_EXIT_STATUS,
  MSGRNG_MSG_CYCLES,
  //cacheline 1
  NETIF_TX = 8,
  NETIF_RX,
  NETIF_TX_COMPLETE,
  NETIF_TX_COMPLETE_TX,
  NETIF_RX_CYCLES,
  NETIF_TX_COMPLETE_CYCLES,
  NETIF_TX_CYCLES,
  NETIF_TIMER_START_Q,
  //NETIF_REG_FRIN,
  //NETIF_INT_REG,
  //cacheline 2
  REPLENISH_ENTER = 16,
  REPLENISH_ENTER_COUNT,
  REPLENISH_CPU,
  REPLENISH_FRIN,
  REPLENISH_CYCLES,
  NETIF_STACK_TX,
  NETIF_START_Q,
  NETIF_STOP_Q,
  //cacheline 3
  USER_MAC_START = 24,
  USER_MAC_INT   = 24,
  USER_MAC_TX_COMPLETE,
  USER_MAC_RX,
  USER_MAC_POLL,
  USER_MAC_TX,
  USER_MAC_TX_FAIL,
  USER_MAC_TX_COUNT,
  USER_MAC_FRIN,
  //cacheline 4
  USER_MAC_TX_FAIL_GMAC_CREDITS = 32,
  USER_MAC_DO_PAGE_FAULT,
  USER_MAC_UPDATE_TLB,
  USER_MAC_UPDATE_TLB_PFN0,
  USER_MAC_UPDATE_TLB_PFN1,
  
  NLM_MAX_COUNTERS = 40
};
extern atomic_t nlm_common_counters[NR_CPUS][NLM_MAX_COUNTERS];
extern __u32 msgrng_msg_cycles;

#ifdef CONFIG_NLM_STATS 
#define xlr_inc_counter(x) atomic_inc(&nlm_common_counters[0][(x)])
#define nlm_common_dec_counter(x) atomic_dec(&nlm_common_counters[0][(x)])
#define xlr_set_counter(x, value) atomic_set(&nlm_common_counters[0][(x)], (value))
#define nlm_common_get_counter(x) atomic_read(&nlm_common_counters[0][(x)])
#else
#define xlr_inc_counter(x) //atomic_inc(&nlm_common_counters[0][(x)])
#define nlm_common_dec_counter(x) //atomic_dec(&nlm_common_counters[0][(x)])
#define xlr_set_counter(x, value) //atomic_set(&nlm_common_counters[0][(x)], (value))
#define nlm_common_get_counter(x) //atomic_read(&nlm_common_counters[0][(x)])
#endif
#if 0
#define dbg_msg(fmt, args...) printk("[%s@%d|%s]: cpu_%d: " fmt, \
                               __FILE__, __LINE__, __FUNCTION__,  smp_processor_id(), ##args)

#define dbg_panic(fmt, args...) panic("[%s@%d|:%s]: cpu_%d: " fmt, \
                               __FILE__, __LINE__, __FUNCTION__, smp_processor_id(), ##args)

#define prom_dbg_msg(fmt, args...) prom_printf("[%s@%d|%s]: cpu_%d: " fmt, \
                               __FILE__, __LINE__, __FUNCTION__,  smp_processor_id(), ##args)
#else
#define dbg_msg(fmt, args...)

#define dbg_panic(fmt, args...) panic(fmt, ##args)

#define prom_dbg_msg(fmt, args...) printk(fmt, ##args)
#endif

#endif
