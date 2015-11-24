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


#define NLM_DEB_DEV_NAME  "phnxdeb"

#define NLM_RMIOS_DEBUGGER_WRITE                   0xaa10
#define NLM_RMIOS_DEBUGGER_READ                    0xaa11
#define NLM_RMIOS_DEBUGGER_TX_MEM_WRITE            0xaa12
#define NLM_RMIOS_DEBUGGER_TX_MEM_READ             0xaa13
#define NLM_RMIOS_DEBUGGER_RX_MEM_WRITE            0xaa14
#define NLM_RMIOS_DEBUGGER_RX_MEM_READ             0xaa15
#define NLM_RMIOS_DEBUGGER_MEM_READ                0xaa16
#define NLM_RMIOS_DEBUGGER_MEM_WRITE               0xaa17
#define NLM_RMIOS_DEBUGGER_PIC_IPI                 0xaa18
#define LINUX_RMIOS_SHARED_BASE                     0x00040000UL
#define LINUX_RMIOS_VCPU                            32
#define LINUX_RMIOS_TX_BUF_SIZE                     1600
#define LINUX_RMIOS_RX_BUF_SIZE                     256
#define LINUX_RMIOS_RX_BUF_BASE            LINUX_RMIOS_SHARED_BASE
#define LINUX_RMIOS_TX_BUF_BASE            (LINUX_RMIOS_RX_BUF_BASE + (LINUX_RMIOS_RX_BUF_SIZE * LINUX_RMIOS_VCPU))
#define LINUX_RMIOS_CPU_ONLINE_MAP_LOCK    (LINUX_RMIOS_TX_BUF_BASE + (LINUX_RMIOS_TX_BUF_SIZE * LINUX_RMIOS_VCPU))
#define LINUX_RMIOS_CPU_ONLINE_MAP_LOCK_INIT_DONE  (LINUX_RMIOS_CPU_ONLINE_MAP_LOCK + CACHE_LINE_SIZE)
#define LINUX_RMIOS_CPU_ONLINE_MAP         (LINUX_RMIOS_CPU_ONLINE_MAP_LOCK_INIT_DONE + CACHE_LINE_SIZE)

#define LOCK_INIT_DONE                              0x900ddeed

