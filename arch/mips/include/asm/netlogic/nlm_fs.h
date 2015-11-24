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

#ifndef _ASM_NLM_XLR_UACCESS_H
#define _ASM_NLM_XLR_UACCESS_H

#define NLM_FS_UNUSED             0
#define NLM_FS_MSGSND             1
#define NLM_FS_MSGRCV             2
#define NLM_FS_C0_COUNT           3
#define NLM_FS_MEM_READ           4
#define NLM_FS_MEM_WRITE          5
#define NLM_FS_MSGINT             6
#define NLM_FS_READ_COP           7
#define NLM_FS_PERFCTR_START      8
#define NLM_FS_PERFCTR_STOP       9
#define NLM_FS_READ_CPUMASKS      10
#define NLM_FS_READ_PROCID        11
#define NLM_FS_PROMINFO           12
#define NLM_FS_READ_TIMER         13
#define NLM_FS_HARD_CPUID         14
#define NLM_FS_ENDIANESS          15
#define NLM_FS_REVERSE_ENDIANESS  16
#define NLM_FS_USPACE_64BIT_INS   17
#define NLM_FS_CPU_MAX_FREQ       18
#define NLM_FS_MEM_READ64         19
#define NLM_FS_MEM_WRITE64        20
#define NLM_FS_MEM_READ32         21
#define NLM_FS_MEM_WRITE32        22
#define NLM_FS_MSGSND3            23
#define NLM_FS_MSGRCV1            24
#define NLM_FS_MEM_READ16        25
#define NLM_FS_MEM_WRITE16       26

#ifndef __ASSEMBLY__

extern void nlm_fs_mem_read16(void);
extern void nlm_fs_mem_write16(void);
extern void nlm_fs_mem_read32(void);
extern void nlm_fs_mem_write32(void);
extern void nlm_fs_mem_read64(void);
extern void nlm_fs_mem_write64(void);
extern void nlm_fs_msgsnd(void);
extern void nlm_fs_msgrcv(void);
extern void nlm_fs_c0_count(void);
extern void nlm_fs_processorId(void);
extern void nlm_fs_iomem_read(void);
extern void nlm_fs_iomem_write(void);
extern void nlm_fs_msg_read(void);
extern void nlm_fs_perf_ctr_start(void);
extern void nlm_fs_perf_ctr_stop(void);
extern void nlm_fs_get_cpumasks(void);
extern void nlm_fs_read_timer(void);
extern void nlm_fs_read_timer(void);
extern void nlm_fs_hard_cpuid(void);
extern void nlm_fs_is_big_endian(void);
extern void nlm_fs_is_endian_reversed(void);
extern void nlm_fs_uspace_64bit_ins_enabled(void);
extern void nlm_fs_cpu_max_freq(void);

#if defined(CONFIG_NLM_XLR)
extern void nlm_fs_prominfo(void);
#else
extern void nlm_fs_msgsnd3(void);
extern void nlm_fs_msgrcv1(void);
#endif /* CONFIG_NLM_XLR */

#endif /* __ASSEMBLY__ */

#endif /* _ASM_NLM_XLR_UACCESS_H */
