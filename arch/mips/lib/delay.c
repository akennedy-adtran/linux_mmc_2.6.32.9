/*-
 * Copyright 2007-2012 Broadcom Corporation
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
 * Copyright (C) 1994 by Waldorf Electronics
 * Copyright (C) 1995 - 2000, 01, 03 by Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2007  Maciej W. Rozycki
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/smp.h>

#include <asm/compiler.h>
#include <asm/war.h>

#if defined CONFIG_NLM_XLP /* For XLP with PIC timer */
#include <asm/netlogic/xlp_irq.h>
#include <linux/clockchips.h>
extern int read_current_timer(unsigned long *cyc);

static inline void __pic_delay(u64 ns, u32 m)
{
	u64 delta;
	unsigned long cc, ic;
	unsigned int xlp_pit_tick_rate = nlm_hal_get_xlp_pit_tick_rate();

	delta = (((div_sc(xlp_pit_tick_rate, NSEC_PER_SEC, 20) * ns * m) >> 20) + 1);
	read_current_timer(&ic);
	while(1){
		read_current_timer(&cc);
		if (((u64)(cc - ic)) > delta) {
			return;
		}
	}
}

void __ndelay(unsigned int ns)
{
	return __pic_delay((u64)ns, 1);
}

void __udelay(unsigned int us)
{
	return __pic_delay((u64)us, 1000);
}

void __delay(unsigned int s)
{
	return __pic_delay((u64)s, 1000000);
}

#elif defined CONFIG_NLM_COMMON		/* for XLR and xlp without pic timer */

extern u64 xlr_hpt_read(void);
inline void __delay(unsigned int loops)
{
	uint32_t initial_count,curr_count;
	uint32_t delta;

	delta = loops;

	initial_count = xlr_hpt_read();
        while (1) {
		curr_count = xlr_hpt_read();
		if ((uint32_t)(curr_count - initial_count) > delta)
			return;
	}
}

inline void __udelay(unsigned long us)
{
	uint32_t initial_count,curr_count;
	uint32_t delta;

	delta = us * 66; /* clock runs at 66.6MHz speed */
			    /* cant do floating point ops here */

	initial_count = xlr_hpt_read();
        while (1) {
		curr_count = xlr_hpt_read();
		if ((uint32_t)(curr_count - initial_count) > delta)
			return;
	}
}

inline void __ndelay(unsigned long ns)
{
	uint32_t initial_count,curr_count;
	uint32_t delta;

	/* clock runs at 66.6MHz speed
	the minimum delay we have have is 1/66.67 = ~15 nsec */
	delta = (ns >> 4) +1;

	initial_count = xlr_hpt_read();
        while (1) {
		curr_count = xlr_hpt_read();
		if ((uint32_t)(curr_count - initial_count) > delta)
			return;
	}
}

#else

inline void __delay(unsigned int loops)
{
	__asm__ __volatile__ (
	"	.set	noreorder				\n"
	"	.align	3					\n"
	"1:	bnez	%0, 1b					\n"
	"	subu	%0, 1					\n"
	"	.set	reorder					\n"
	: "=r" (loops)
	: "0" (loops));
}

/*
 * Division by multiplication: you don't have to worry about
 * loss of precision.
 *
 * Use only for very small delays ( < 1 msec).  Should probably use a
 * lookup table, really, as the multiplications take much too long with
 * short delays.  This is a "reasonable" implementation, though (and the
 * first constant multiplications gets optimized away if the delay is
 * a constant)
 */

void __udelay(unsigned long us)
{
	unsigned int lpj = current_cpu_data.udelay_val;

	__delay((us * 0x000010c7ull * HZ * lpj) >> 32);
}

void __ndelay(unsigned long ns)
{
	unsigned int lpj = current_cpu_data.udelay_val;

	__delay((ns * 0x00000005ull * HZ * lpj) >> 32);
}
#endif
EXPORT_SYMBOL(__delay);
EXPORT_SYMBOL(__udelay);
EXPORT_SYMBOL(__ndelay);
