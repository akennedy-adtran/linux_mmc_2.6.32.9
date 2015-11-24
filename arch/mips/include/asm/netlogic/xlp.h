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


#ifndef _ASM_XLP_H
#define _ASM_XLP_H

#include <linux/types.h>
#include <asm/cpu.h>
#include <asm/mipsregs.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/hal/nlm_hal_xlp_dev.h>

#define MAX_CPU_REV_LEN			32
#define NLM_MAX_CPU_NODE		4
#define NLM_MAX_DRAM_REGION		8
#define NLM_MAX_CPU_PER_NODE	32
#define NLM_MAX_THREADS_PER_CPU	4
#define NLM_MAX_VC_PER_THREAD	4

#define PCI_NETL_VENDOR         0x184E
#define XLP_DEVID_DRAM          0x1001
#define XLP_DEVID_PIC           0x1003
#define XLP_DEVID_INTLA         0x1005
#define XLP_DEVID_USBDEV        0x1006
#define XLP_DEVID_EHCI          0x1007  
#define XLP_DEVID_OHCI          0x1008
#define XLP_DEVID_NAE           0x1009
#define XLP_DEVID_POE           0x100A
#define XLP_DEVID_CMS           0x100B
#define XLP_DEVID_DTRE_RAID     0x100C
#define XLP_DEVID_SAE           0x100D
#define XLP_DEVID_RSAECC        0x100E
#define XLP_DEVID_CPM           0x100F
#define XLP_DEVID_UART          0x1010
#define XLP_DEVID_I2C           0x1011
#define XLP_DEVID_GPIO          0x1012
#define XLP_DEVID_SYS           0x1013
#define XLP_DEVID_JTAG          0x1014
#define XLP_DEVID_NOR           0x1015
#define XLP_DEVID_NAND          0x1016
#define XLP_DEVID_SPI           0x1017
#define XLP_DEVID_MMC           0x1018
#define XLP_DEVID_REGEX			0x1019
#define XLP2XX_DEVID_I2C        0x101C
#define XLP2XX_DEVID_XHCI       0x101D

struct smp_boot_info_percpu {
  volatile unsigned long ready;
  volatile unsigned long sp;
  volatile unsigned long gp;
  volatile unsigned long fn;
};

struct smp_boot_info {
  struct smp_boot_info_percpu boot_info[NR_CPUS];
  cpumask_t online_map;
};

extern struct smp_boot_info smp_boot;
extern void prom_boot_cpus_secondary(void *);
extern cpumask_t phys_cpu_present_map;
extern int is_nlm_xlp2xx_compat;

extern char cpu_model_info[MAX_CPU_REV_LEN];
extern char* get_cpu_info(void);
#endif /*_ASM_XLP_H */ 
