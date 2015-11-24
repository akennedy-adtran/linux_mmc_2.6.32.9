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


#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>

#include <asm/mach-netlogic/kernel-entry-init.h>

NESTED(ptr_smp_boot, 16, sp)

	move	sp, a1
	move	gp, a2
	jal	a0
	nop
	
END(ptr_smp_boot)
	
/* Don't jump to linux function from Bootloader stack. Change it 
 * here. Kernel might allocate bootloader memory before all the CPUs are 
 * brought up (eg: Inode cache region) and we better don't overwrite this 
 * memory
 */
NESTED(prom_pre_boot_secondary_cpus, 16, sp)
	SET_MIPS64
	mfc0 t0, $15, 1 #read ebase
	andi t0, 0x1f #t0 has the processor_id()
	PTR_LA	t1, xlr_stack_pages_temp
	li   t2, _THREAD_SIZE
	srl  t2, 2
	mul  t3, t2, t0
	nop
	nop
	nop
	nop
	nop
	nop
	PTR_ADDU  gp, t1, t3
	PTR_ADDU       sp, gp, t2
	PTR_ADDI       sp, sp, -32
	PTR_LA t0, prom_boot_cpus_secondary
	jr t0
	nop
END(prom_pre_boot_secondary_cpus)

NESTED(nlm_handle_boot_smp_nmi, 0, sp)
	.set push
	.set noat
	.set mips64
	.set noreorder

	/* Clear the  NMI and BEV bits */
	MFC0	k0, CP0_STATUS
	li 	k1, 0xffb7ffff
	and	k0, k0, k1
	MTC0	k0, CP0_STATUS


	PTR_LA  k1, secondary_entry_point
	PTR_L	k0, 0(k1)
	jr	k0
	nop

	.set pop
END(nlm_handle_boot_smp_nmi)



NESTED(nlm_boot_smp_nmi, 0, sp)
	.set push
	.set noat
	.set mips64
	.set noreorder
	MTC0	k0, OS_KGDB_SCRATCH_REG6
	MTC0	k1, OS_KGDB_SCRATCH_REG7
	nop
	nop

	MAPPED_KERNEL_SETUP_TLB

	PTR_LA	k0, nlm_handle_boot_smp_nmi
	jr       k0
	nop
	.set pop
END(nlm_boot_smp_nmi)
