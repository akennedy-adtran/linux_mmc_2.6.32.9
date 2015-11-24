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

#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/netlogic/interrupt.h>
#include <asm/asm-offsets.h>

/*
 * void nlm_cop2_save()
 * sp is assumed to point to a struct pt_regs
 *
 * NOTE: This is called in SAVE_SOME in stackframe.h. It can only
 *       safely modify k0 and k1.
 */
	.align	7
	.set push
	.set noreorder
	LEAF(nlm_cop2_save)
	/*msgring tx buf reg*/
	dmfc2 k0, $0, 0
	LONG_S  k0, NLM_COP2_TX_BUF(sp)
	dmfc2 k0, $0, 1
	LONG_S  k0, (NLM_COP2_TX_BUF+8)(sp)
	dmfc2 k0, $0, 2
	LONG_S k0, (NLM_COP2_TX_BUF+16)(sp)
	dmfc2 k0, $0, 3
	LONG_S k0, (NLM_COP2_TX_BUF+24)(sp)
	
	/*msgring rx buf reg*/
	dmfc2 k0, $1, 0
	LONG_S  k0, NLM_COP2_RX_BUF(sp)
	dmfc2 k0, $1, 1
	LONG_S  k0, (NLM_COP2_RX_BUF+8)(sp)
	dmfc2 k0, $1, 2
	LONG_S  k0, (NLM_COP2_RX_BUF+16)(sp)
	dmfc2 k0, $1, 3
	LONG_S  k0, (NLM_COP2_RX_BUF+24)(sp)

	mfc2 k0, $2, 0
	sw  k0, NLM_COP2_TX_MSG_STATUS(sp)
	
	mfc2 k0, $3, 0
	jr	ra
	sw k0, NLM_COP2_RX_MSG_STATUS(sp)

	END(nlm_cop2_save)
	.set pop

/*
 * void nlm_cop2_restore()
 * sp is assumed to point to a struct pt_regs
 *
 * NOTE: This is called in RESTORE_SOME in stackframe.h.
 */
	.align	7
	.set push
	.set noreorder
	LEAF(nlm_cop2_restore)
	/*msgring tx buf reg*/
	LONG_L  v1, NLM_COP2_TX_BUF(sp)
	dmtc2 v1, $0, 0
	LONG_L  v1, (NLM_COP2_TX_BUF+8)(sp)
	dmtc2 v1, $0, 1
	LONG_L  v1, (NLM_COP2_TX_BUF+16)(sp)
	dmtc2 v1, $0, 2
	LONG_L  v1, (NLM_COP2_TX_BUF+24)(sp)
	dmtc2 v1, $0, 3

	/*msgring rx buf reg*/
	LONG_L  v1, NLM_COP2_RX_BUF(sp)
	dmtc2 v1, $1, 0
	LONG_L  v1, (NLM_COP2_RX_BUF+8)(sp)
	dmtc2 v1, $1, 1
	LONG_L  v1, (NLM_COP2_RX_BUF+16)(sp)
	dmtc2 v1, $1, 2
	LONG_L  v1, (NLM_COP2_RX_BUF+24)(sp)
	dmtc2 v1, $1, 3
	lw  v1, NLM_COP2_TX_MSG_STATUS(sp)
	mtc2 v1, $2, 0

	/*leave bits 28-31 up to date*/
	li k0, 0xf0000000
	lw v1, NLM_COP2_RX_MSG_STATUS(sp)
	or v1, k0, v1
	xor v1, k0, v1
	mfc2 k0, $3, 0
	srl k0, k0, 28
	sll k0, k0, 28
	or v1, k0, v1
	jr	ra
	mtc2 v1, $3, 0
	END(nlm_cop2_restore)
	.set pop
