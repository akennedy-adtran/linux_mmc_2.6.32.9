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

#ifndef _ASM_NETLOGIC_CPUMASK_H
#define _ASM_NETLOGIC_CPUMASK_H

#include <linux/cpumask.h>
#include <linux/sched.h>

#define CPUMASK_BUF 256

/* Removed deprecated attribute below.  Why are we using this in our own code that only we use? */
static __inline__ unsigned int cpumask_to_uint32(cpumask_t *);
static __inline__ void uint32_to_cpumask(cpumask_t *, unsigned int);

static __inline__ unsigned int cpumask_to_uint32(cpumask_t *cpumask)
{
	unsigned int mask = 0;
	int i;

	for (i = 0; i < 32; i++) {
		if (cpumask_test_cpu(i, cpumask)) {
			mask |= (1 << i);
		}
	}
	return mask;
}

static __inline__ void uint32_to_cpumask(cpumask_t *cpumask, unsigned int mask)
{
	int i;

	for (i = 0; i < 32; i++) {
		if (mask & (1 << i)) {
			cpumask_set_cpu(i, cpumask);
		}
		else {
			cpumask_clear_cpu(i, cpumask);
		}
	}
}

static __inline__ void sched_bindto_save_affinity(int cpu, struct cpumask *mask)
{
	struct cpumask node0cpu0mask;

	/* Save the current affinity mask */
	sched_getaffinity(0, mask);

	/* Force the process to run on the specified cpu */
	cpumask_clear(&node0cpu0mask);
	cpumask_set_cpu(cpu, &node0cpu0mask);

	sched_setaffinity(0, &node0cpu0mask);
}

static __inline__ void sched_bindto_restore_affinity(struct cpumask *mask)
{
	sched_setaffinity(0, mask);
}


#endif /* _ASM_NETLOGIC_CPUMASK_H */
