/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: virtual.h,v 1.1.2.2 2006-09-28 01:24:17 nphilips Exp $
 * Virtual per-process performance counters.
 *
 * Copyright (C) 1999-2004  Mikael Pettersson
 */

#ifdef CONFIG_PERFCTR_VIRTUAL
extern int vperfctr_init(void);
extern void vperfctr_exit(void);
#else
static inline int vperfctr_init(void) { return 0; }
static inline void vperfctr_exit(void) { }
#endif
