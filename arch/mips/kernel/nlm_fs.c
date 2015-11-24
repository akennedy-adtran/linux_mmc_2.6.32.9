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

#include <linux/compiler.h>
#include <linux/module.h>

#include <asm/netlogic/nlm_fs.h>

static void nlm_fs_unused(void)
{
	/* dummy function */
}

unsigned long nlm_fs_table[] = {
	[NLM_FS_UNUSED]           = (unsigned long)nlm_fs_unused,
	[NLM_FS_MSGSND]           = (unsigned long)nlm_fs_msgsnd,
	[NLM_FS_MSGRCV]           = (unsigned long)nlm_fs_msgrcv,
	[NLM_FS_C0_COUNT]         = (unsigned long)nlm_fs_c0_count,
	[NLM_FS_MEM_READ]         = (unsigned long)nlm_fs_iomem_read,
	[NLM_FS_MEM_WRITE]        = (unsigned long)nlm_fs_iomem_write,
	[NLM_FS_MSGINT]           = (unsigned long)nlm_fs_unused,
	[NLM_FS_READ_COP]         = (unsigned long)nlm_fs_msg_read,
	[NLM_FS_PERFCTR_START]    = (unsigned long)nlm_fs_perf_ctr_start,
	[NLM_FS_PERFCTR_STOP]     = (unsigned long)nlm_fs_perf_ctr_stop,
	[NLM_FS_CPU_MAX_FREQ]     = (unsigned long)nlm_fs_cpu_max_freq,
	[NLM_FS_READ_PROCID]      = (unsigned long)nlm_fs_processorId,
	[NLM_FS_READ_TIMER]       = (unsigned long)nlm_fs_read_timer,
	[NLM_FS_HARD_CPUID]       = (unsigned long)nlm_fs_hard_cpuid,
	[NLM_FS_ENDIANESS]        = (unsigned long)nlm_fs_is_big_endian,
	[NLM_FS_REVERSE_ENDIANESS]= (unsigned long)nlm_fs_is_endian_reversed,
	[NLM_FS_USPACE_64BIT_INS] = (unsigned long)nlm_fs_uspace_64bit_ins_enabled,
	[NLM_FS_MEM_READ64]       = (unsigned long)nlm_fs_mem_read64,
	[NLM_FS_MEM_WRITE64]      = (unsigned long)nlm_fs_mem_write64,
	[NLM_FS_MEM_READ32]       = (unsigned long)nlm_fs_mem_read32,
	[NLM_FS_MEM_WRITE32]      = (unsigned long)nlm_fs_mem_write32,

#if defined(CONFIG_NLM_XLR)
	[NLM_FS_READ_CPUMASKS]    = (unsigned long)nlm_fs_get_cpumasks,
	[NLM_FS_PROMINFO]         = (unsigned long)nlm_fs_prominfo,
	[NLM_FS_MSGSND3]          = (unsigned long)nlm_fs_unused,
	[NLM_FS_MSGRCV1]          = (unsigned long)nlm_fs_unused,
#else
	[NLM_FS_READ_CPUMASKS]    = (unsigned long)nlm_fs_unused,
	[NLM_FS_PROMINFO]         = (unsigned long)nlm_fs_unused,
	[NLM_FS_MSGSND3]          = (unsigned long)nlm_fs_msgsnd3,
	[NLM_FS_MSGRCV1]          = (unsigned long)nlm_fs_msgrcv1,
#endif /* CONFIG_NLM_XLR */
	[NLM_FS_MEM_READ16]       = (unsigned long)nlm_fs_mem_read16,
        [NLM_FS_MEM_WRITE16]      = (unsigned long)nlm_fs_mem_write16,
	0
};

