/*-
 * Copyright 2006-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: mips_tests.h,v 1.1.2.3 2006-09-28 01:24:17 nphilips Exp $
 * Performance-monitoring counters driver.
 * Optional PPC32-specific init-time tests.
 *
 * Copyright (C) 2004  Mikael Pettersson
 */

#ifdef CONFIG_PERFCTR_INIT_TESTS
extern void perfctr_mips_init_tests(void);
#else
static inline void perfctr_mips_init_tests(void) { };
#endif
