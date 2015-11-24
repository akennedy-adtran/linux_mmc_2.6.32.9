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


#include <linux/init.h>

#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/cacheops.h>
#include <asm/irqflags.h>
#include <asm/regdef.h>
#include <asm/fpregdef.h>
#include <asm/mipsregs.h>
#include <asm/stackframe.h>
#include <asm/war.h>
#include <asm/page.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/interrupt.h>


NESTED(nlm_except_vec_nmi, 0, sp)
	.set push
	.set noat
	.set mips64
	.set noreorder
	MTC0	k0, OS_KGDB_SCRATCH_REG6
	nop
	nop
	PTR_LA	k0, nlm_nmi_handler
	jr       k0
	nop
	.set pop
END(nlm_except_vec_nmi)


	/* This nmi handler is currently only for taking oprofile samples
	   on non-zero cpus
	   */
NESTED(nlm_nmi_handler, PT_SIZE,  sp)
	.set	push
	.set	noat
	.set noreorder
	.set 	mips64

	/* Save K0 and K1 first */
	/* K0 is already saved in nlm_except_vec_nmi */
	MTC0	k1, OS_KGDB_SCRATCH_REG7
	
	/* Clear the  NMI and BEV bits */
	MFC0	k0, CP0_STATUS
	li 	k1, 0xffb7ffff
	and	k0, k0, k1
	MTC0	k0, CP0_STATUS

	SAVE_ALL
	CLI
	TRACE_IRQS_OFF

	li	a0, IRQ_IPI_SMP_KGDB
	move	a1, sp
	/* jal	do_nlm_common_IRQ */
	/* nop */
	jal	nlm_kgdb_call_nmi_hook
	nop

	RESTORE_ALL

	/*
	MFC0 	k0, $15, 1
	andi	k0, 0x1f
	sll	k0, 2
	la	k1, nlm_cpus_in_nmi
 	PTR_ADDU	k1, k0
	sw	zero, 0(k1)
	*/

	MFC0	k0, OS_KGDB_SCRATCH_REG6
	MFC0	k1, OS_KGDB_SCRATCH_REG7
	
	.set mips3
	eret

	.set pop
END(nlm_nmi_handler)
