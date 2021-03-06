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



#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/asm-offsets.h>

#include <asm/netlogic/perf_ctr.h>
#include <asm/netlogic/interrupt.h>
#include <asm/netlogic/nlm_fs.h>

#ifdef CONFIG_32BIT
	#define T0 t0
	#define T1 t1
	#define T2 t2
	#define T3 t3
	#define T4 t4
	#define T5 t5
	#define T6 t6
	#define T7 t7
#else
	#define T0 ta0
	#define T1 ta1
	#define T2 ta2
	#define T3 ta3
	#define T4 t0
	#define T5 t1
	#define T6 t2
	#define T7 t3
#endif
	.text
	.set	push
	.set	noreorder
	.set	mips64
	.align	5

.macro	fs_eret
	/* skip the syscall instruction */
	MFC0    k0, CP0_EPC
	PTR_ADDIU   k0, 4
	MTC0    k0, CP0_EPC
	eret
.endm

#if defined(CONFIG_NLM_XLR)

NESTED(nlm_fs_msgsnd, PT_SIZE, sp)

	/* msgsnd arg0 is in (t1, t2) */
	dsll32  k0, T1, 0
	dsll32	k1, T2, 0
	dsrl32	T2, k1, 0
	or      T1, k0, T2
	dmtc2	T1, $0, 0

	/* msgsnd arg1 is in (t3, t4) */
	dsll32  k0, T3, 0
	dsll32	k1, T4, 0
	dsrl32	T4, k1, 0
	or      T3, k0, T4
	dmtc2   T3, $0, 1

	/* msgsnd dst is in t0 */
1:	c2	0x80001
	mfc2	T1, $2
	andi	T1, T1, 0x06
	bnez	T1, 1b
	nop

	fs_eret
END(nlm_fs_msgsnd)

NESTED(nlm_fs_msgrcv, PT_SIZE, sp)
	/* t0 has the bucket arg */
	c2	0x80002
1:	mfc2	T3, $2
	andi	k0, T3, 0x08
	bnez	k0, 1b
	nop
	andi    T0, T3, 0x30
	bnez	T0, 2f
	nop
	dmfc2	T2, $1, 0
	dmfc2   T5, $1, 1
	nop
	dsra32  T1, T2, 0
	dsra32  T4, T5, 0
2:

	/* move    t0, k0 */

	fs_eret
END(nlm_fs_msgrcv)

NESTED(nlm_fs_get_cpumasks, PT_SIZE, sp)

	.word 0x40088007
	move k0, T0
	srl  T0, T0, 4
	andi T0, T0, 0x3f
	sll  T0, T0, 2
	andi k0, k0, 0x0f
	or T0, T0, k0

	PTR_LA k0, fast_syscall_cpumask_phy		;
	lw T1, (k0)

	fs_eret
END(nlm_fs_get_cpumasks)

NESTED(nlm_fs_prominfo, PT_SIZE, sp)

	PTR_LA	k0, prom_info_copy
	move	k1, T0
	sll	k1, k1, 3
	addu	k0, k0, k1
	lw	T1, (k0)
	lw	T2, 4(k0)

	fs_eret
END(nlm_fs_prominfo)

#else /* CONFIG_NLM_XLR */

#include <asm/netlogic/hal/nlm_hal_xlp_dev.h>

	.set	arch=xlp

NESTED(nlm_fs_msgsnd3, PT_SIZE, sp)
	/* msgsnd arg0 is in (t1, t2) */
	dsll32  k0, T1, 0
	dsll32	k1, T2, 0
	dsrl32	T2, k1, 0
	or      T1, k0, T2
	dmtc2	T1, XLP_TX_BUF_REG, 0

	/* msgsnd arg1 is in (t3, t4) */
	dsll32  k0, T3, 0
	dsll32	k1, T4, 0
	dsrl32	T4, k1, 0
	or      T3, k0, T4
	dmtc2   T3, XLP_TX_BUF_REG, 1

	/* msgsnd arg1 is in (t5, t6) */
	dsll32  k0, T5, 0
	dsll32	k1, T6, 0
	dsrl32	T6, k1, 0
	or      T5, k0, T6
	dmtc2   T5, XLP_TX_BUF_REG, 2

	sync

	/* msgsnd dst is in t0, status returned in t1 */
1:	msgsnds	T1, T0
	beqz	T1, 1b /* comment out the branch              */
	nop            /* for non-blocking msgsnd fastsyscall */

	fs_eret
END(nlm_fs_msgsnd3)

NESTED(nlm_fs_msgsnd, PT_SIZE, sp)
	/* msgsnd arg0 is in (t1, t2) */
	dsll32  k0, T1, 0
	dsll32	k1, T2, 0
	dsrl32	T2, k1, 0
	or      T1, k0, T2
	dmtc2	T1, XLP_TX_BUF_REG, 0

	/* msgsnd arg1 is in (t3, t4) */
	dsll32  k0, T3, 0
	dsll32	k1, T4, 0
	dsrl32	T4, k1, 0
	or      T3, k0, T4
	dmtc2   T3, XLP_TX_BUF_REG, 1

	sync

	/* msgsnd dst is in t0, status returned in t1 */
1:	msgsnds	T1, T0
	beqz	T1, 1b /* comment out the branch              */
	nop            /* for non-blocking msgsnd fastsyscall */

	fs_eret
END(nlm_fs_msgsnd)

NESTED(nlm_fs_msgrcv, PT_SIZE, sp)
	/* msgld vc is in t0, status returned in k0 */
	msglds	k0, T0
	beqz	k0, 1f
	move	T0, k0

	/* msgld status  t0       *
	 * arg0          (t1, t2) *
	 * msg_rxstatus  t3       *
	 * arg1          (t4, t5) */
	mfc2	T3, XLP_MSG_RXSTATUS_REG
	dmfc2	T2, XLP_RX_BUF_REG, 0
	dmfc2   T5, XLP_RX_BUF_REG, 1
	dsra32  T1, T2, 0
	dsra32  T4, T5, 0
1:
	fs_eret
END(nlm_fs_msgrcv)

NESTED(nlm_fs_msgrcv1, PT_SIZE, sp)
	/* msgld vc is in t0, status returned in k0 */
	msglds	k0, T0
	beqz	k0, 1f
	move	T0, k0

	/* msgld status  t0       *
	 * arg0          (t1, t2) *
	 * msg_rxstatus  t3       *
	 * arg1          (t4, t5) */
	mfc2	T3, XLP_MSG_RXSTATUS_REG
	dmfc2	T2, XLP_RX_BUF_REG, 0
	dsra32  T1, T2, 0
1:
	fs_eret
END(nlm_fs_msgrcv1)

#endif /* CONFIG_NLM_XLP */

NESTED(nlm_fs_mem_read64, PT_SIZE, sp)
	/* address is in (t0, t1) */
	dsll32  k0, T0, 0
	dsll32	k1, T1, 0
	dsrl32	T1, k1, 0
	or      T0, k0, T1

	/* data is in (t2, t3) */
	ld	T3, (T0)
	dsra32	T2, T3, 0

	fs_eret
END(nlm_fs_mem_read64)

NESTED(nlm_fs_mem_write64, PT_SIZE, sp)
	/* address is in (t0, t1) */
	dsll32  k0, T0, 0
	dsll32	k1, T1, 0
	dsrl32	T1, k1, 0
	or      T0, k0, T1

	/* data is in (t2, t3) */
	dsll32  k0, T2, 0
	dsll32	k1, T3, 0
	dsrl32	T3, k1, 0
	or      T1, k0, T3

	sd	T1, (T0)

	fs_eret
END(nlm_fs_mem_write64)

NESTED(nlm_fs_mem_read32, PT_SIZE, sp)
	/* address is in (t0, t1) */
	dsll32  k0, T0, 0
	dsll32	k1, T1, 0
	dsrl32	T1, k1, 0
	or      T0, k0, T1

	/* data is in t2 */
	lw	T2, (T0)

	fs_eret
END(nlm_fs_mem_read32)

NESTED(nlm_fs_mem_write32, PT_SIZE, sp)
	/* address is in (t0, t1) */
	dsll32  k0, T0, 0
	dsll32	k1, T1, 0
	dsrl32	T1, k1, 0
	or      T0, k0, T1

	/* data is in t2 */
	sw	T2, (T0)

	fs_eret
END(nlm_fs_mem_write32)

NESTED(nlm_fs_mem_read16, PT_SIZE, sp)
       /* address is in (t0-msb, t1-lsb) */
       dsll32  k0, T0, 0
       dsll32  k1, T1, 0
       dsrl32  T1, k1, 0
       or      T0, k0, T1

       /* data is in t2 */
       lh      T2, (T0)
       fs_eret
END(nlm_fs_mem_read16)

NESTED(nlm_fs_mem_write16, PT_SIZE, sp)
        /* address is in (t0-msb, t1-lsb) */
        dsll32  k0, T0, 0
        dsll32  k1, T1, 0
        dsrl32  T1, k1, 0
        or      T0, k0, T1

        /* data is in t2 */
        sh      T2, (T0)
        fs_eret
END(nlm_fs_mem_write16)

NESTED(nlm_fs_c0_count, PT_SIZE, sp)

	mfc0    T0, $9, 0

	fs_eret
END(nlm_fs_c0_count)

NESTED(nlm_fs_iomem_read, PT_SIZE, sp)

	/* t0 has the address */
	lw      T1, (T0)

	fs_eret
END(nlm_fs_iomem_read)

NESTED(nlm_fs_iomem_write, PT_SIZE, sp)

	/* t0 has the address, t1 has the data */
	sw      T1, (T0)

	fs_eret
END(nlm_fs_iomem_write)


NESTED(nlm_fs_msg_write, PT_SIZE, sp)

	/* disable the message ring interrupts and enable 64 bits operations */
	mfc0    k0, CP0_STATUS
	li      k1, 1
	dsll    k1, k1, 30
	or      k0, k0, k1
	li      k1, 1
	dsll    k1, k1, 23
	or      k0, k0, k1
	mtc0    k0, CP0_STATUS
	bnez	T1, 1f
	nop
	mtc2    T1, $3, 0
1:

	fs_eret
END(nlm_fs_msg_write)


NESTED(nlm_fs_msg_read, PT_SIZE, sp)

	/* read C0 and C2 registers */
	mfc0    T0, CP0_STATUS
	mfc2    T1, $2, 0
	mfc2    T2, $2, 1
	mfc2    T3, $3, 0
	mfc2    T4, $3, 1

	fs_eret
END(nlm_fs_msg_read)

NESTED(nlm_fs_perf_ctr_start, PT_SIZE, sp)

	/* reset value counters */
	dmtc0    $0, CP0_PERF_CTR, PERF_CTR_EVENT0_VALUE
	dmtc0    $0, CP0_PERF_CTR, PERF_CTR_EVENT1_VALUE

	/* arm the counters */
	mtc0    T0, CP0_PERF_CTR, PERF_CTR_EVENT0
	mtc0    T1, CP0_PERF_CTR, PERF_CTR_EVENT1

	fs_eret
END(nlm_fs_perf_ctr_start)

	/* {evt@0:hi=T1, evt@0:lo=T0}
	 * {evt@1:hi=T3, evt@1:lo=T2}
	 * nlm_fs_perf_ctr_stop(uint32_t val) */
NESTED(nlm_fs_perf_ctr_stop, PT_SIZE, sp)

	/* stop the counters */
	mtc0    T0, CP0_PERF_CTR, PERF_CTR_EVENT0
	mtc0    T0, CP0_PERF_CTR, PERF_CTR_EVENT1

	/* load return values */
	dmfc0    k1, CP0_PERF_CTR, PERF_CTR_EVENT0_VALUE
	dsrl32 	 T1, k1, 0
	dsll32   k1, k1, 0
	dsrl32   T0, k1, 0

	dmfc0    k1, CP0_PERF_CTR, PERF_CTR_EVENT1_VALUE
	dsrl32 	 T3, k1, 0
	dsll32   k1, k1, 0
	dsrl32   T2, k1, 0

	fs_eret
END(nlm_fs_perf_ctr_stop)

NESTED(nlm_fs_processorId, PT_SIZE, sp)

	mfc0    T0, $15, 0

	fs_eret
END(nlm_fs_processorId)

NESTED(nlm_fs_read_timer, PT_SIZE, sp)

#if defined(CONFIG_NLM_XLP)
#include <asm/netlogic/xlp_irq.h>
#define PIC_TIMER_6_COUNTER	0x28
        MFC0    k0, CP0_PRID, 1
        andi    k0, k0, 0x3ff
        srl     k0, k0, 5  /* grab node id */
        sll     k0, k0, 18 /* Use local PIC */
        dli     k1, ASM_XLP_IO_PIC_OFFSET + (PIC_TIMER_6_COUNTER << 3)
        dadd    k0, k1, k0
	ld	k1, 0(k0)
	dsrl32	$9, k1, 0
	dsll32  $8, k1, 0
	dsrl32  $8, T0, 0
#else
	dli	k0, 0xffffffffbef00000 + NETLOGIC_IO_PIC_OFFSET + (PIC_TIMER_6_COUNTER_0 << 2)
	lw	$8, 0(k0)
	dli	k0, 0xffffffffbef00000 + NETLOGIC_IO_PIC_OFFSET + (PIC_TIMER_6_COUNTER_1 << 2)
	lw	$9, 0(k0)
#endif
	fs_eret

END(nlm_fs_read_timer)

NESTED(nlm_fs_hard_cpuid, PT_SIZE, sp)

	mfc0	T0, $15, 1
	andi	T0, T0, 0x3ff

	fs_eret

END(nlm_fs_hard_cpuid)

NESTED(nlm_fs_is_big_endian, PT_SIZE, sp)
	mfc0	T0, $16, 0
	andi	T0, T0, 0x8000

	fs_eret
END(nlm_fs_is_big_endian)

NESTED(nlm_fs_is_endian_reversed, PT_SIZE, sp)
	mfc0	T0, $12, 0
	li	k1, 0x2000000
	and	T0, T0, k1

	fs_eret
END(nlm_fs_is_endian_reversed)

NESTED(nlm_fs_uspace_64bit_ins_enabled, PT_SIZE, sp)
	mfc0	T0, $12, 0
	li	k1, 0x800000
	and	T0, T0, k1

	fs_eret
END(nlm_fs_uspace_64bit_ins_enabled)

	/* {hi=T1, lo=T0} nlm_fs_cpu_max_freq(void) */
NESTED(nlm_fs_cpu_max_freq, PT_SIZE, sp)
	PTR_LA k0, mips_hpt_frequency
	lw     k1, 0(k0)
	dsrl32 T1, k1, 0
	dsll32 k1, k1, 0
	dsrl32 T0, k1, 0

	fs_eret
END(nlm_fs_cpu_max_freq)

	.set pop
