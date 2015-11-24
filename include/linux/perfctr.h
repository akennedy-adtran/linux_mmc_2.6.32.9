/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: perfctr.h,v 1.1.2.4 2006-09-28 01:24:22 nphilips Exp $
 * Performance-Monitoring Counters driver
 *
 * Copyright (C) 1999-2004  Mikael Pettersson
 */
#ifndef _LINUX_PERFCTR_H
#define _LINUX_PERFCTR_H

#ifdef CONFIG_PERFCTR	/* don't break archs without <asm/perfctr.h> */

#include <asm/perfctr.h>

// Ensure that the following constant doesn't conflict constants specified
// for other entries in "/proc/sys"

#define CTL_PERFCTR	171

// Extend the following enum type if more entries have to be created under
// "/proc/sys/perfctr"

enum {
	PERFCTR_CNTMODE = 1
};

struct perfctr_info {
	unsigned int abi_version;
	char driver_version[32];
	unsigned int cpu_type;
	unsigned int cpu_features;
	unsigned int cpu_khz;
	unsigned int tsc_to_cpu_mult;
	unsigned int _reserved2;
	unsigned int _reserved3;
	unsigned int _reserved4;
};

struct perfctr_cpu_mask {
	unsigned int nrwords;
	unsigned int mask[1];	/* actually 'nrwords' */
};

/* abi_version values: Lower 16 bits contain the CPU data version, upper
   16 bits contain the API version. Each half has a major version in its
   upper 8 bits, and a minor version in its lower 8 bits. */
#define PERFCTR_API_VERSION	0x0600	/* 6.0 */
#define PERFCTR_ABI_VERSION	((PERFCTR_API_VERSION<<16)|PERFCTR_CPU_VERSION)

/* cpu_features flag bits */
#define PERFCTR_FEATURE_RDPMC	0x01
#define PERFCTR_FEATURE_RDTSC	0x02
#define PERFCTR_FEATURE_PCINT	0x04

/* user's view of mmap:ed virtual perfctr */
struct vperfctr_state {
	struct perfctr_cpu_state cpu_state;
};

/* virtual perfctr control object */
struct vperfctr_control {
	int si_signo;
	unsigned int preserve;
	unsigned int _reserved1;
	unsigned int _reserved2;
	unsigned int _reserved3;
	unsigned int _reserved4;
	struct perfctr_cpu_control cpu_control;
};

/* commands for sys_vperfctr_read() */
#define VPERFCTR_READ_SUM	0x01
#define VPERFCTR_READ_CONTROL	0x02
#define VPERFCTR_READ_CHILDREN	0x03

#else
struct perfctr_info;
struct perfctr_cpu_mask;
struct perfctr_sum_ctrs;
struct vperfctr_control;
#endif	/* CONFIG_PERFCTR */

#ifdef __KERNEL__

/*
 * The perfctr system calls.
 */
asmlinkage long sys_vperfctr_open(int tid, int creat);
asmlinkage long sys_vperfctr_control(int fd,
				     const struct vperfctr_control __user *argp,
				     unsigned int argbytes);
asmlinkage long sys_vperfctr_unlink(int fd);
asmlinkage long sys_vperfctr_iresume(int fd);
asmlinkage long sys_vperfctr_read(int fd, unsigned int cmd, void __user *argp, unsigned int argbytes);

extern struct perfctr_info perfctr_info;

#ifdef CONFIG_PERFCTR_VIRTUAL

/*
 * Virtual per-process performance-monitoring counters.
 */
struct vperfctr;	/* opaque */

/* process management operations */
extern void __vperfctr_copy(struct task_struct*, struct pt_regs*);
extern void __vperfctr_release(struct task_struct*);
extern void __vperfctr_exit(struct vperfctr*);
extern void __vperfctr_suspend(struct vperfctr*);
extern void __vperfctr_resume(struct vperfctr*);
extern void __vperfctr_sample(struct vperfctr*);
extern void __vperfctr_set_cpus_allowed(struct task_struct*, struct vperfctr*, cpumask_t);

static inline void perfctr_copy_task(struct task_struct *tsk, struct pt_regs *regs)
{
	if (tsk->thread.perfctr) {
		__vperfctr_copy(tsk, regs);
	}
}

static inline void perfctr_release_task(struct task_struct *tsk)
{
	if (tsk->thread.perfctr) {
		__vperfctr_release(tsk);
	}
}

static inline void perfctr_exit_thread(struct thread_struct *thread)
{
	struct vperfctr *perfctr;
	perfctr = thread->perfctr;
	if (perfctr) {
		__vperfctr_exit(perfctr);
	}
}

static inline void perfctr_suspend_thread(struct thread_struct *prev)
{
	struct vperfctr *perfctr;
	perfctr = prev->perfctr;
	if (perfctr) {
		__vperfctr_suspend(perfctr);
	}
}

static inline void perfctr_resume_thread(struct thread_struct *next)
{
	struct vperfctr *perfctr;
	perfctr = next->perfctr;
	if (perfctr) {
		__vperfctr_resume(perfctr);
	}
}

static inline void perfctr_sample_thread(struct thread_struct *thread)
{
	struct vperfctr *perfctr;
	perfctr = thread->perfctr;
	if (perfctr) {
		__vperfctr_sample(perfctr);
	}
}

static inline void perfctr_set_cpus_allowed(struct task_struct *p, cpumask_t new_mask)
{
#ifdef CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK
	struct vperfctr *perfctr;

	task_lock(p);
	perfctr = p->thread.perfctr;
	if (perfctr) {
		__vperfctr_set_cpus_allowed(p, perfctr, new_mask);
	}
	task_unlock(p);
#endif
}

#else	/* !CONFIG_PERFCTR_VIRTUAL */

static inline void perfctr_copy_task(struct task_struct *p, struct pt_regs *r) { }
static inline void perfctr_release_task(struct task_struct *p) { }
static inline void perfctr_exit_thread(struct thread_struct *t) { }
static inline void perfctr_suspend_thread(struct thread_struct *t) { }
static inline void perfctr_resume_thread(struct thread_struct *t) { }
static inline void perfctr_sample_thread(struct thread_struct *t) { }
static inline void perfctr_set_cpus_allowed(struct task_struct *p, cpumask_t m) { }

#endif	/* CONFIG_PERFCTR_VIRTUAL */

#endif	/* __KERNEL__ */

#endif	/* _LINUX_PERFCTR_H */
