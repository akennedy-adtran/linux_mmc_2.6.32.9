/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995 - 1999 Ralf Baechle
 * Copyright (C) 1999 Silicon Graphics, Inc.
 *
 * Cache error handler
 */
#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/stackframe.h>
#include <asm/asm-offsets.h>
#include <asm/netlogic/mips-exts.h>

/*
 * Game over.  Go to the button.  Press gently.  Swear where allowed by
 * legislation.
 */
LEAF( cache_save_regs )
	.set push
	.set mips64
	.set noat
	.set noreorder

	LONG_S  $1 , PT_R1 (sp)
	LONG_S  $2 , PT_R2 (sp)
	LONG_S  $3 , PT_R3 (sp)
	LONG_S  $4 , PT_R4 (sp)
	LONG_S  $5 , PT_R5 (sp)
	LONG_S  $6 , PT_R6 (sp)
	LONG_S  $7 , PT_R7 (sp)
	LONG_S  $8 , PT_R8 (sp)
	LONG_S  $9 , PT_R9 (sp)
	LONG_S  $10, PT_R10(sp)
	LONG_S  $11, PT_R11(sp)
	LONG_S  $12, PT_R12(sp)
	LONG_S  $13, PT_R13(sp)
	LONG_S  $14, PT_R14(sp)
	LONG_S  $15, PT_R15(sp)
	LONG_S  $16, PT_R16(sp)
	LONG_S  $17, PT_R17(sp)
	LONG_S  $18, PT_R18(sp)
	LONG_S  $19, PT_R19(sp)
	LONG_S  $20, PT_R20(sp)
	LONG_S  $21, PT_R21(sp)
	LONG_S  $22, PT_R22(sp)
	LONG_S  $23, PT_R23(sp)
	LONG_S  $24, PT_R24(sp)
	LONG_S  $25, PT_R25(sp)
	LONG_S  $26, PT_R26(sp)
	LONG_S  $27, PT_R27(sp)
	LONG_S  $28, PT_R28(sp)
	LONG_S  $29, PT_R29(sp)
	LONG_S  $30, PT_R30(sp)

	j   ra
	nop
	.set pop
END(cache_save_regs)

LEAF(cache_restore_regs)
	.set push
	.set mips64
	.set noat
	.set noreorder

	LONG_L  $1 , PT_R1 (sp)
	LONG_L  $2 , PT_R2 (sp)
	LONG_L  $3 , PT_R3 (sp)
	LONG_L  $4 , PT_R4 (sp)
	LONG_L  $5 , PT_R5 (sp)
	LONG_L  $6 , PT_R6 (sp)
	LONG_L  $7 , PT_R7 (sp)
	LONG_L  $8 , PT_R8 (sp)
	LONG_L  $9 , PT_R9 (sp)
	LONG_L  $10, PT_R10(sp)
	LONG_L  $11, PT_R11(sp)
	LONG_L  $12, PT_R12(sp)
	LONG_L  $13, PT_R13(sp)
	LONG_L  $14, PT_R14(sp)
	LONG_L  $15, PT_R15(sp)
	LONG_L  $16, PT_R16(sp)
	LONG_L  $17, PT_R17(sp)
	LONG_L  $18, PT_R18(sp)
	LONG_L  $19, PT_R19(sp)
	LONG_L  $20, PT_R20(sp)
	LONG_L  $21, PT_R21(sp)
	LONG_L  $22, PT_R22(sp)
	LONG_L  $23, PT_R23(sp)
	LONG_L  $24, PT_R24(sp)
	LONG_L  $25, PT_R25(sp)
	LONG_L  $26, PT_R26(sp)
	LONG_L  $27, PT_R27(sp)
	LONG_L  $28, PT_R28(sp)
	LONG_L  $29, PT_R29(sp)
	LONG_L  $30, PT_R30(sp)

	j ra
	nop
	.set pop
END(cache_restore_regs)

LEAF(except_vec2_generic)
	.set push
	.set noreorder
	.set noat
	.set mips64r2

	ehb

	MTC0      k0, CP0_DIAGNOSTIC, 3
	MTC0      k1, CP0_DIAGNOSTIC, 4

	/*If some other cpu is already in the handler just wait... */
	PTR_LA    k0, nlm_cerr_lock
1:	lw        k1, 0(k0)
	bnez      k1, 1b
	nop
	li        k1, 1
	sw        k1, 0(k0)

	/*save: sp, ra */
	PTR_LA    k0, nlm_cerr_stack
	PTR_ADDU  k0, k0, 0x2000
	LONG_S    sp, -1*8(k0)
	LONG_S    ra, -2*8(k0)

	PTR_LA    k1, except_vec2_generic_body
	jal       k1
	nop

	/*recover: sp,ra*/
	PTR_LA    k0, nlm_cerr_stack
	PTR_ADDU  k0, k0, 0x2000
	LONG_L    sp, -1*8(k0)
	LONG_L    ra, -2*8(k0)

	PTR_LA    k0, nlm_cerr_lock
	li        k1, 0
	sw        k1, 0(k0)

	MFC0      k0, CP0_DIAGNOSTIC, 3
	MFC0      k1, CP0_DIAGNOSTIC, 4
	eret
	nop
	.set pop
END(except_vec2_generic)

LEAF(except_vec2_generic_body)
	.set push
	.set noreorder
	.set noat
	.set mips64r2
	/*
	 * This is a very bad place to be.  Our cache error
	 * detection has triggered.  If we have write-back data
	 * in the cache, we may not be able to recover.  As a
	 * first-order desperate measure, turn off KSEG0 cacheing.
	 */

	mfc0    k0, CP0_CONFIG
	move    sp, k0
	li      k1,~CONF_CM_CMASK
	and     k0,k0,k1
	ori     k0,k0,CONF_CM_UNCACHED
	mtc0    k0,CP0_CONFIG
	ehb

#ifndef CONFIG_NLM_COMMON

	/* cache_parity_error will not return */
	j     cache_parity_error
	nop
#else
	PTR_LA    k0, nlm_cerr_stack
	PTR_ADDU  k0, k0, 0x2000
	MFC0      k1, CP0_ERROREPC
	LONG_S    k1, -3*8(k0)
	LONG_S    ra, -4*8(k0)
	LONG_S    sp, -5*8(k0) /*save CP0_CONFIG*/
	PTR_ADDU  sp, k0, -40*8

	jal   cache_save_regs
	nop

	jal   nlm_cache_error
	nop

	jal   cache_restore_regs
	nop

	PTR_LA    k0, nlm_cerr_stack
	PTR_ADDU  k0, k0, 0x2000
	LONG_L    ra, -4*8(k0)
	LONG_L    k1, -5*8(k0) /*save CP0_CONFIG*/
	mtc0      k1, CP0_CONFIG
	LONG_L    k1, -3*8(k0)
	MTC0      k1, CP0_ERROREPC
	ehb

	j     ra
	nop
#endif

	.set pop
END(except_vec2_generic_body)
