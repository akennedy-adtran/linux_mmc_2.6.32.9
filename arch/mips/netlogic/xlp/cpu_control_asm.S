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

#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/stackframe.h>
#include <asm/asmmacro.h>
#include <asm/cacheops.h>

#include <asm/mach-netlogic/kernel-entry-init.h>
#include <asm/netlogic/xlp8xx/cpu_control_macros.h>

.macro	prog_c0_status set clr
	.set push
	.set noreorder
	mfc0		t0, CP0_STATUS
	or			t0, ST0_CU0|\set|0x1f|\clr
	xor			t0, 0x1f|\clr
	mtc0		t0, CP0_STATUS
	EHB
	.set pop
.endm

/* setup TLBs for non-0 cpus for KSEG2 access */
.macro 	SETUP_PERTHREAD_TLB
#ifdef CONFIG_64BIT
	dli			t3, CKSSEG
	dmtc0		t3, CP0_ENTRYHI
#else
	li			t3, CKSSEG
	mtc0		t3, CP0_ENTRYHI
#endif
	li			t1, 0x1f
	MTC0		t1, CP0_ENTRYLO0    # physaddr, DVG, cach exlwr
	li			t2, 0x1
	MTC0		t2, CP0_ENTRYLO1    # physaddr, G
	li			t1, 0x1fffe000      # MAPPED_KERN_TLBMASK, TLBPGMASK_256M
	mtc0		t1, CP0_PAGEMASK
	mtc0		zero, CP0_INDEX
	tlbwi
	li			t0, 1
	mtc0		t0, CP0_WIRED
	EHB
.endm

.macro 	__start_secondary
	.set push
#ifdef CONFIG_MAPPED_KERNEL
	SETUP_PERTHREAD_TLB
#endif
#ifdef CONFIG_64BIT
	prog_c0_status ST0_KX ST0_BEV
#else
	prog_c0_status 0 ST0_BEV
#endif
	mfc0		t0, CP0_EBASE, 1
	andi		t0, 0x3
	bnez		t0, 20f

	/* Do this after tlb is set up */
	__config_lsu

20:
	mfc0		t0, CP0_EBASE, 1
	andi		t0, 0x7f

	PTR_LA		t1, xlp_stack_pages_temp
	li			t2, _THREAD_SIZE
	srl			t2, 2
	mul			t3, t2, t0
	PTR_ADDU	gp, t1, t3
	PTR_ADDU	sp, gp, t2
	PTR_ADDI	sp, sp, -64
	PTR_LA		t0, prom_boot_cpus_secondary
	jr			t0
	nop
	.set pop
.endm

.macro __config_lsu
	.set push
	.set noreorder

/* XLP8xx/4xx/3xx Unaligned access erratum - two possible work-arounds:
 * 1) Enable safe retry mode in SCHED_DEFEATURE (see below)
 * 2) Disable hardware acceleration for unaligned accesses
 * Enable only one of the below LUI opcodes.  For XLP2xx/XLP1xx leave
 * unaligned access enabled.  L2HPE = bit 23.  L1HPE = bit 20.  Both default 0.
 */
	li			t0, LSU_DEFEATURE
	mfcr		t1, t0
//	lui			t2, 0x4080	/* Enable Unaligned Access, enable L2HPE */
	lui			t2, 0x0080	/* Enable L2HPE */
	or			t1, t1, t2
	mtcr		t1, t0

#if 0
	li			t0, THREAD_FAIRNESS
	li			t1, 0x8		/* Experimental: Set thread fairness to 8 if DRAM utilization is high */
	mtcr		t1, t0
#endif

/* CPU Scheduler Issue Queue 3 erratum - two possible work-arounds:
 * 1) Disable Branch Resolution ALU from accepting ALU operations
 * 2) Equally weight ALU and branch operations in issue queue 3
 * Also options to enable or disable safe retry mode for unaligned access erratum
 * Enable only one of the below four LUI opcodes
 */
	li      	t0, SCHED_DEFEATURE		// Enable only ONE of the following four lines
//	lui			t1, 0x0100		// Disable BRU accepting ALU ops, don't enable safe retry
//	lui			t1, 0xF900		// Disable BRU accepting ALU ops, enable safe retry
	lui			t1, 0x0020		// Equally weight ALU and branch ops in issue Q3, don't enable safe rerty
//	lui			t1, 0xF820		// Equally weight ALU and branch ops in issue Q3, enable safe rerty
	mtcr    	t1, t0

#if 0
	/* Set up serial access mode for uncached accesses - applies to multi-node only */
	li			t0, L2_FEATURE_CTRL0
	mfcr		t1, t0
	ori			t1, t1, 0x200
	mtcr		t1, t0
#endif
	nop

/* removed S1RCM errata work-around for XLP8xx A0/A1 and L1 2-way disable for XLP2xx A0 */

10:
	.set pop
.endm

/*
 * L1D cache has to be flushed before enabling threads in XLP.
 * On XLP8xx/XLP3xx, we do a low level flush using processor control
 * registers. On XLPII CPUs, usual cache instructions work.
 */
.macro	flush_l1_dcache
	.set push
	.set noreorder

	mfc0		t0, CP0_EBASE, 0
	andi		t0, t0, 0xff00
	slt			t1, t0, 0x1200		// Test for PRID = 0x12xx
	beqz		t1, 15f
	nop

	/* XLP8xx/3xx low level cache flush */
	li			t0, LSU_DEBUG_DATA0
	li			t1, LSU_DEBUG_ADDR
	li			t2, 0
	li			t3, 0x400
1:
	sll			v0, t2, 5	
	mtcr		zero, t0
	ori			v1, v0, 0x3
	mtcr		v1, t1
2:
	mfcr		v1, t1
	andi		v1, 0x1
	bnez		v1, 2b
	nop
	mtcr		zero, t0
	ori			v1, v0, 0x7
	mtcr		v1, t1
3:
	mfcr		v1, t1
	andi		v1, 0x1
	bnez		v1, 3b
	nop
	addi		t2, 1
	bne			t3, t2, 1b
	nop

    li			t2, 0
4:
	sll			v0, t2, 5	
	mtcr		zero, t0
	ori			v1, v0, 0x4003
	mtcr		v1, t1
5:
	mfcr		v1, t1
	andi		v1, 0x1
	bnez		v1, 5b
	nop
	mtcr    	zero, t0
	ori			v1, v0, 0x4007
	mtcr    	v1, t1
6:
	mfcr    	v1, t1
	andi    	v1, 0x1
	bnez    	v1, 6b
	nop
	addi		t2, 1
	bne			t3, t2, 4b
	nop
	b			17f
	nop

	/* XLPII CPUs, Invalidate all 64k of L1 D-cache */
15:
	li			t0, 0x80000000
	li			t1, 0x80010000
16:	cache		Index_Writeback_Inv_D, 0(t0)
	addiu		t0, t0, 32
	bne			t0, t1, 16b
	nop

17:
	.set pop
.endm

	/* T0 of Non-0 Cores jump
	 * here, from enable_cores
	 * This code sits in KSEG0
	 * (@0xbfc00000)
	 */
EXPORT(reset_entry)
	mfc0		t0, CP0_EBASE, 1
	mfc0		t1, CP0_EBASE, 1
	andi		t1, 0x60
	srl			t1,  5
	li			t2, 0x40000  #load 0x40000
	mul			t3, t2, t1  #t3 = node * 0x40000
	srl			t0, t0 , 2
	and			t0, t0 , 0x7 # t0 contains the core number
	li			t1, 0x1
	sll			t0, t1, t0
	nor			t0, t0, zero
#ifdef CONFIG_64BIT
	dla			t2, CPU_MMIO_OFFSET(0, SYS)
#else
	la			t2, CPU_MMIO_OFFSET(0, SYS)
#endif
	add			t2, t2, t3  #get node based SYS offset
	lw			t1, (SYS_CPUNONCOHERENTMODE_REG << 2)(t2)
	and			t1, t1, t0
	sw			t1, (SYS_CPUNONCOHERENTMODE_REG<< 2)(t2)

	/* read back to ensure complete */
	lw			t1, (SYS_CPUNONCOHERENTMODE_REG << 2)(t2)
	sync

#ifdef CONFIG_64BIT
	dla			t1, boot_siblings_start
	dla			t2, __boot_siblings
#else
	la			t1, boot_siblings_start
	la			t2, __boot_siblings
#endif
	subu		t2, t2, t1		/* t2 now has the jump offset */

	/* Jump to KSEG0 addr of __boot_siblings
	 * We cant use 'dla' here.
	 */
	dli			k0,	NMI_BASE_ASM
	ori			k0,	k0, 2048
	addu		k0, k0, t2
	jr			k0
	nop
EXPORT(reset_entry_end)

	/* boot_siblings is copied into
	 * NMI_BASE in KSEG0 space. This
	 * sets up TLBs for other threads
	 * so that non-0 threads can run
	 * out of KSEG2
	 */

EXPORT(boot_siblings_start)			/* "Master" (n0c0t0) cpu starts from here */
	.set noreorder
	LONG_S		sp, linuxsp($0)
	SAVE_ALL
	sync

EXPORT(__boot_siblings)				/* T0 of every core in every node starts from here */

	flush_l1_dcache

	mfc0		t3, CP0_EBASE, 1
	srl			t3, t3 , 2
	and			t3, t3 , 0x7  			/* t3 contains the core number */
	mul			t3, t3 , 4                        
#ifdef CONFIG_64BIT
	dla			t0, threads_to_enable
#else
	la			t0, threads_to_enable
#endif
#ifdef CONFIG_MAPPED_KERNEL
	subu		t0, t0, 0x40000000		/* t0 has the kseg0 address of threads_to_enable */
#endif
	lw			t1, 0(t0)			/* t1 now has the entire threads_to_enable */
	srl			t1, t3                     
	and			t1, t1, 0xf 			/* t1 has the threads to enable for this core. */
	beq			t1, 0x2, 2f
	nop
	addi		t1, -1
2:
	li			t0, ((CPU_BLOCKID_MAP << 8) | BLKID_MAP_THREADMODE)
	mfcr		t2, t0
	or			t2, t2, t1
	mtcr		t2, t0

	/* threads (incl. T0) of this core
	 * start fetching from this point
	 */
	mfc0		t0, CP0_EBASE, 1		/* EBASE, Select 1 	*/
	andi		t0, 0x7f		        /* Linear CPU ID	*/
	beqz		t0, 2f
	nop
1:
    __start_secondary				/* "Slave" cpu (of every core on every node) go here */
2:
	LONG_L		sp, linuxsp($0)		/* "Master" (n0c0t0) cpu restores from here */
	PTR_SUBU	sp, PT_SIZE
	RESTORE_ALL

	jr			ra
	nop
EXPORT(boot_siblings_end)

NESTED(ptr_smp_boot, 16, sp)

	move		sp, a1
	move		gp, a2
	jal			a0
	nop

END(ptr_smp_boot)

.macro	setup_c0_status set clr
	.set push
	.set noreorder
	mfc0		t0, CP0_STATUS
	or			t0, ST0_CU0|\set|0x1f|\clr
	xor			t0, 0x1f|\clr
	mtc0		t0, CP0_STATUS
	EHB
	.set pop
.endm

/* Don't jump to linux function from Bootloader stack. Change it
 * here. Kernel might allocate bootloader memory before all the CPUs are
 * brought up (eg: Inode cache region) and we better don't overwrite this
 * memory
 */
NESTED(prom_pre_boot_secondary_cpus, 16, sp)
	SET_MIPS64
	MAPPED_KERNEL_SETUP_TLB

	/* Don't trust the bootstrapper to set cp0_status to what you want */
#ifdef CONFIG_64BIT
	setup_c0_status ST0_KX ST0_BEV
#else
	setup_c0_status 0 ST0_BEV
#endif
	mfc0		t0, CP0_EBASE, 1 #read ebase
	andi		t0, 0x7f #t0 has the processor_id()
	PTR_LA		t1, xlp_stack_pages_temp
	li			t2, _THREAD_SIZE
	srl			t2, 2
	mul			t3, t2, t0
	PTR_ADDU	gp, t1, t3
	PTR_ADDU	sp, gp, t2
	PTR_ADDI	sp, sp, -32
	PTR_LA		t0, prom_boot_cpus_secondary
	jr			t0
	nop
END(prom_pre_boot_secondary_cpus)
