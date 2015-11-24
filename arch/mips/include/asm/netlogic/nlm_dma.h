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

#ifndef _NLM_DMA_H
#define _NLM_DMA_H

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/signal.h>
#include <linux/poll.h>
#include <linux/percpu.h>

#ifdef CONFIG_NLM_XLP
#include <asm/netlogic/iomap.h>
#include <asm/netlogic/xlp_irq.h>
#include <hal/nlm_hal.h>
#include <hal/nlm_hal_macros.h>
#include <hal/nlm_hal_fmn.h>
extern u32 xlp_get_power_on_reset_cfg(int);
#endif
u64 setup_pcie_shared_memspace(u64 *);
void raise_host_interrupt(int);
int xlp_async_request_dma(uint64_t src, uint64_t dest, uint32_t len,
	void (*func)(void *,uint64_t),void *data, enum dma_data_direction dir);
extern struct proc_dir_entry *dma_procfs_dir;
int xlp_init_dma(void);

#define NLM_XLP_PCIE_SHARED_MEMSIZE		(32 * 1024 * 1024)
#define DEFAULT_NLMXLP_IO_BASE DEFAULT_NETLOGIC_IO_BASE

#define DPRINTK(level,fmt,args...)\
do{\
	printk(level "%s()@%s:%d " fmt,__func__, __FILE__, __LINE__, ##args);\
}while(0)

#define XLP_PCIE0_TX_BUCKET	(256)	// PRM 10.8, FMN message addressing.
#define XLP_PCIE0_RX_BUCKET	(257)	// PRM 10.8, FMN message addressing.
#define XLP_PCIE1_TX_BUCKET	(258)	// PRM 10.8, FMN message addressing.
#define XLP_PCIE1_RX_BUCKET	(259)	// PRM 10.8, FMN message addressing.
#define XLP_PCIE2_TX_BUCKET	(260)	// PRM 10.8, FMN message addressing.
#define XLP_PCIE2_RX_BUCKET	(261)	// PRM 10.8, FMN message addressing.
#define XLP_PCIE3_TX_BUCKET	(262)	// PRM 10.8, FMN message addressing.
#define XLP_PCIE3_RX_BUCKET	(263)	// PRM 10.8, FMN message addressing.

#define XLP_DMA_MSGSIZE		(3)	// PRM 22.4.1
#define XLP_P2D_MAX_MSGSIZE	(4)	// PRM 10.2, for more, use P2P
#endif // _XLP_DMA_H_
