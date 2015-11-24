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


#ifndef __NLM_PCIX_DEVICE_H__
#define __NLM_PCIX_DEVICE_H__

#include <asm/types.h>
#include <linux/cache.h>

/*Define this macro if device supports MAILBOX interrupt.*/
//#define XLR_MAILBOX_IS_SUPPORTED 1

/*Define this macro if host is MSI capable.*/
//#define XLR_MSI_IS_SUPPORTED 1

#ifdef CONFIG_NLM_COMMON
#ifdef NETLOGIC_LITTLE_ENDIAN
#define PCIX_REG_BASE 64
#else
#define PCIX_REG_BASE (512 + 64)
#endif
#define PCIX_HOST_MODE_CTRL_STATUS_REG ( PCIX_REG_BASE + 35)
#endif

#define XLR_PCI_HOST_MODE 0x1
#define XLR_PCI_DEV_MODE 0x2

#define nlm_common_host_to_pci(addr) ((uint64_t)(addr) | 0x8000000000UL)

#define CACHELINE_ALIGNED_ADDR(addr) \
			(((unsigned long)(addr)) & ~(SMP_CACHE_BYTES-1))

#ifndef CONFIG_NLM_COMMON
#ifdef XLR_MSI_IS_SUPPORTED
typedef int (*msi_handler)(void *, struct pt_regs *);
#endif
#endif

// HOST SIDE
#ifndef CONFIG_NLM_COMMON
#ifdef XLR_MSI_IS_SUPPORTED
int nlm_common_request_msi_handler(msi_handler,void *,int *);
void nlm_common_free_msi_handler(int *);
int nlm_common_enable_msi(int *);
int nlm_common_disable_msi(int *);
void nlm_common_interrupt_host(void);
#endif
#endif

#ifdef CONFIG_NLM_COMMON
int nlm_get_pci_mode(void);
#endif

unsigned long nlm_common_get_shared_mem_base(void);
unsigned int nlm_common_pci_readl(unsigned int *);
u8 nlm_common_pci_readb(u8 *);
void nlm_nlm_common_interrupt_device(void);




/*****************************************************************************************************************/
/**********************************SHARED    MEMORY***************************************************************/
/*****************************************************************************************************************/
// DURING BOOT ONLY

#define NLM_BOOT_SHARED_MEM_BASE 0x1000
#define NLM_BOOT_SHARED_MEM_SIZE (32 * 1024 * 1024)


// AFTER BOOTIN WHOLE SHARED MEMORY IS CLAIMED BY THE GENERIC PCI DRIVER 
#define NLM_GENERIC_SHARED_MEM_BASE (20*1024*1024)
#define NLM_GENERIC_SHARED_MEM_SIZE (10* 1024 * 1024)

// All The Shared Address must be unique for each driver. Confliction of Address Space can cause unpredictable result.


// SHARED SPACE BETWEEN MAC DRIVERS
#define NLM_MAC_SHARED_MEM_BASE NLM_GENERIC_SHARED_MEM_BASE
#define NLM_MAC_SHARED_MEM_SIZE (1 * 1024 * 1024)


// SHARED SPACE BETWEEN CONSOLE DRIVERS
#define NLM_CONSOLE_OVER_PCI_SHARED_MEM_BASE \
	         (NLM_MAC_SHARED_MEM_BASE + NLM_MAC_SHARED_MEM_SIZE)
#define NLM_CONSOLE_OVER_PCI_SHARED_MEM_SIZE (9 * 1024)

// SHARED space for DMA
#define NLM_DMA_MEM_BASE \
			(NLM_CONSOLE_OVER_PCI_SHARED_MEM_BASE + \
	 		NLM_CONSOLE_OVER_PCI_SHARED_MEM_SIZE)
#define NLM_DMA_MEM_SIZE 1024

// SHARED SPACE BETWEEN IP OVER PCI DRIVER...
#define NLM_IP_OVER_PCI_MEM_BASE \
			(NLM_DMA_MEM_BASE + NLM_DMA_MEM_SIZE)
#define NLM_IP_OVER_PCI_MEM_SIZE (8*512+8*512+1024) 

// SHARED SPACE BETWEEN SECURITY DRIVER... xxxx
//
//

#endif
