/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: mips_tests.c,v 1.1.2.6 2007-10-31 17:34:41 kmurthy Exp $
 * Performance-monitoring counters driver.
 * Optional PPC32-specific init-time tests.
 *
 * Copyright (C) 2004  Mikael Pettersson
 */
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/perfctr.h>
#include <asm/processor.h>
#include <asm/time.h>
#include <asm/mipsregs.h>
#include <asm/perfctr.h>
#include "mips_tests.h"

#define NITER	256
#define X2(S)	S"; "S
#define X8(S)	X2(X2(X2(S)))

static void __init do_read_tsc(unsigned int unused)
{
	unsigned int i, dummy;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mfc0 %0, $9, 0") : "=r" (dummy));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_read_pmc1(unsigned int unused)
{
	unsigned int i, dummy;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mfc0 %0, $25, 1") : "=r" (dummy));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_read_pmc2(unsigned int unused)
{
	unsigned int i, dummy;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mfc0 %0, $25, 3") : "=r" (dummy));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_read_ctrl1(unsigned int unused)
{
	unsigned int i, dummy;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mfc0 %0, $25, 0") : "=r" (dummy));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_read_ctrl2(unsigned int unused)
{
	unsigned int i, dummy;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mfc0 %0, $25, 2") : "=r" (dummy));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_write_pmc1(unsigned int arg)
{
	unsigned int i;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mtc0 %z0, $25, 1") : : "Jr" ((unsigned int)arg));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_write_pmc2(unsigned int arg)
{
	unsigned int i;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mtc0 %z0, $25, 3") : : "Jr" ((unsigned int)arg));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_write_ctrl1(unsigned int arg)
{
	unsigned int i;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mtc0 %z0, $25, 0") : : "Jr" ((unsigned int)arg));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_write_ctrl2(unsigned int arg)
{
	unsigned int i;
	__asm__ __volatile__ (".set mips32\n\t");
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__(X8("mtc0 %z0, $25, 2") : : "Jr" ((unsigned int)arg));
	__asm__ __volatile__ (".set mips0\n\t");
}

static void __init do_empty_loop(unsigned int unused)
{
	unsigned i;
	for(i = 0; i < NITER/8; ++i)
		__asm__ __volatile__("" : : );
}

static unsigned __init run(void (*doit)(unsigned int), unsigned int arg)
{
	unsigned int start, stop;
	start = __read_32bit_c0_register ($25, 1);
	(*doit)(arg);	/* should take < 2^32 cycles to complete */
	stop = __read_32bit_c0_register ($25, 1);
	return (stop - start);
}

static void __init init_tests_message(void)
{
	printk(KERN_INFO "Please email the following PERFCTR INIT lines "
	       "to mikpe@csd.uu.se\n"
	       KERN_INFO "To remove this message, rebuild the driver "
	       "with CONFIG_PERFCTR_INIT_TESTS=n\n");
	printk(KERN_INFO "PERFCTR INIT: CPU clock %u kHz\n", perfctr_info.cpu_khz);
}

static void __init clear(void)
{
	__write_32bit_c0_register($25, 0, 0);
	__write_32bit_c0_register($25, 1, 0);
	__write_32bit_c0_register($25, 2, 0);
	__write_32bit_c0_register($25, 3, 0);
}

static void __init measure_overheads(void)
{
	int i;
	unsigned int loop, ticks[9];
	const char *name[9];
	unsigned int ctrl1, ctrl2;

	/* PMC1 = "processor cycles", PMC2 = "completed instructions",
	   not disabled in any mode, no interrupts */

	clear();

	//setup control register 1 and 2

	ctrl1 = ctrl2 = 0;

	ctrl1 |= MIPS_XLR_DOM_KERNEL | MIPS_XLR_DOM_USR;
	MIPS_XLR_OVF_PMI_DABLE(ctrl1);
	MIPS_XLR_SET_CNT_ALL_THREADS(ctrl1);
	MIPS_XLR_SET_EVNT(ctrl1, 63); // CYCLE_CNT

	ctrl2 |= MIPS_XLR_DOM_KERNEL | MIPS_XLR_DOM_USR;
	MIPS_XLR_OVF_PMI_DABLE(ctrl2);
	MIPS_XLR_SET_CNT_ALL_THREADS(ctrl2);
	MIPS_XLR_SET_EVNT(ctrl2, 0); // INSTRS FETCHED

	__write_32bit_c0_register($25, 0, ctrl1);
	__write_32bit_c0_register($25, 2, ctrl2);

 	name[0] = "read (tsc)";
	ticks[0] = run(do_read_tsc, 0);

	name[1] = "read (pmc0)";
	ticks[1] = run(do_read_pmc1, 0);
	name[2] = "read (pmc1)";
	ticks[2] = run(do_read_pmc2, 0);

	name[3] = "read (ctrl0)";
	ticks[3] = run(do_read_ctrl1, 0);
	name[4] = "read (ctrl1)";
	ticks[4] = run(do_read_ctrl2, 0);

	name[5] = "write (pmc1)";
	ticks[5] = run(do_write_pmc1, 0);
	name[6] = "write (pmc2)";
	ticks[6] = run(do_write_pmc2, 0);

	name[7] = "write (ctrl1)";
	ticks[7] = run(do_write_ctrl1, ctrl1);
	name[8] = "write (ctrl2)";
	ticks[8] = run(do_write_ctrl2, ctrl2);

	loop = run(do_empty_loop, 0);

	clear();

	init_tests_message();
	printk(KERN_INFO "PERFCTR INIT: NITER == %u\n", NITER);
	printk(KERN_INFO "PERFCTR INIT: loop overhead is %u cycles\n", loop);
	for(i = 0; i < ARRAY_SIZE(ticks); ++i) {
		unsigned int x;
		if (!ticks[i])
			continue;
		x = ((ticks[i] - loop) * 10) / NITER;
		printk(KERN_INFO "PERFCTR INIT: %s cost is %u.%u cycles (%u total)\n",
		       name[i], x/10, x%10, ticks[i]);
	}
}

void __init perfctr_mips_init_tests()
{
	preempt_disable();
	measure_overheads();
	preempt_enable();
}
