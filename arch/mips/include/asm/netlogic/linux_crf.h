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

#ifndef _RMICRF_LINUX_H
#define _RMICRF_LINUX_H
#include <asm/netlogic/msgring.h>

extern void fdt_parse_args(void);
extern int fdt_get_core_tlb_size(int core);
extern int fdt_add_console_string(char *cmdline);
extern int fdt_get_pic_timer_map(unsigned int *timer_list, int max);
extern int fdt_get_gmac_pde_reginfo(int id, uint64_t *);
extern int fdt_get_xgmac_pde_reginfo(int id, uint64_t *);
extern int fdt_get_spi4_pde_reginfo(int id, uint64_t *);
extern uint64_t fdt_get_heap_size(void);
extern int fdt_get_uart_status(int uartno);
extern uint64_t fdt_get_vuart_fifo_addr(int tx, int instance);
extern int fdt_get_msgring_int_status(int core, uint32_t *en);
extern int fdt_get_core_bucket_conf(int core, char buckets[], int bklen, char credits[][8], int crlen);
extern int fdt_get_bucketmask(uint64_t *);
extern int fdt_get_sae_bucket_conf(char buckets[], int bklen, char credits[][8], int crlen);
extern int fdt_get_cde_bucket_conf(char buckets[], int bklen, char credits[][8], int crlen);
extern int fdt_get_sae_enabled(void);
extern int fdt_get_cde_enabled(void);

extern void *nlm_get_usermac_addr(int size);
extern uint32_t dev_tree_en;
extern uint32_t rmik_en;
extern uint32_t rmik_cpu_msgring_int_mask[];

extern void rmik_wakeup_cpus(void *fn, void *args, uint32_t cpu_mask);
extern void config_net_init(void);
extern void rmik_init(char *g_argv[], int *argc, char *g_envp[]);
extern void rmik_cpu_to_cpu_msgring_handler(int bucket, int size, int code,
				    int stid, struct msgrng_msg *msg,
				    void *data /* ignored */ );
extern void rmik_config_pde(int type, int instance, nlm_reg_t *mmio);
extern void rmik_eventq_ipi_handler(void);
extern void rmik_cpu_to_cpu_pkt_msgring_handler(int size, struct msgrng_msg *msg);
extern void rmik_vmips_init(void);
extern void rmik_nlm_common_msgring_cpu_init(void);
extern int rmik_own_bucket_list_get(int *start, int *end, int *mask);
extern int rmik_derive_msgring_int_mask(void);
extern void rmik_register_net_events(void);
extern int rmik_get_free_running_timer(void);
extern void (*nlm_vnet_pkt_event_handler)(int len, void *msg);
extern uint64_t rmik_get_pde_bktmap(int type, int instance);
#endif
