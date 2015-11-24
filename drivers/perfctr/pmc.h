/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#ifndef __PERFCTR_PMC_H
#define __PERFCTR_PMC_H

#define PMC_KERNEL_MODE		0x2
#define PMC_SUP_MODE		0x4
#define PMC_USER_MODE		0x8

#define PMC_SET_EVNTCNT_MODE(x, mode)		x |= mode
#define PMC_UNSET_EVNTCNT_MODE(x, mode)		x &= ~mode

#define PMC_ENABLE_ITRPT(x)					x |= 0x10
#define PMC_DISABLE_ITRPT(x)				x &= ~0x10

#define PMC_EVNTSEL_MASK			0x3f
#define PMC_EVNTSEL_SHIFT			5
#define PMC_SET_EVNT(x, event)		x |= ((event & PMC_EVNTSEL_MASK) << PMC_EVNTSEL_SHIFT)
#define PMC_GET_EVNT(x)				((x >> PMC_EVNTSEL_SHIFT) & PMC_EVNTSEL_MASK)

#define PMC_THREADID_MASK				0x03
#define PMC_THREADID_SHIFT				11
#define PMC_SET_THREADID(x, tid)		x |= ((tid & PMC_EVNTSEL_MASK) << PMC_EVNTSEL_SHIFT)

#define PMC_SET_CNT_ALL_THREADS(x)		x |= 0x2000
#define PMC_UNSET_CNT_ALL_THREADS(x)	x &= ~0x2000

#endif
