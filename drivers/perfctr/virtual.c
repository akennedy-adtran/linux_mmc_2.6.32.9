/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: virtual.c,v 1.1.2.8 2007-11-15 13:42:01 kmurthy Exp $
 * Virtual per-process performance counters.
 *
 * Copyright (C) 1999-2004  Mikael Pettersson
 */
#include <linux/init.h>
#include <linux/compiler.h>	/* for unlikely() in 2.4.18 and older */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/ptrace.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/perfctr.h>
#include <linux/nsproxy.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "cpumask.h"
#include "virtual.h"
#include "compat.h"

// #define __DEBUG__ 1

/****************************************************************
 * Data types and macros.					*
 ****************************************************************/

struct vperfctr {
	/* User-visible fields: (must be first for mmap()) */
	struct perfctr_cpu_state cpu_state;
	
	/* Kernel-private fields: */
	int si_signo;
	atomic_t count;
	spinlock_t owner_lock;
	struct task_struct *owner;

	/* sampling_timer and bad_cpus_allowed are frequently
	   accessed, so they get to share a cache line 
	*/
	unsigned int sampling_timer ____cacheline_aligned;

	#ifdef CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK
	atomic_t bad_cpus_allowed;
	#endif

	#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT
	unsigned int iresume_cstatus;
	#endif

	/* children_lock protects inheritance_id and children,
	   when parent is not the one doing release_task() 
	*/
	spinlock_t children_lock;
	unsigned long long inheritance_id;
	struct perfctr_sum_ctrs children;

	/* schedule_work() data for when an operation cannot be
	   done in the current context due to locking rules 
	*/
	struct work_struct work;
	struct task_struct *parent_tsk;
};

#define IS_RUNNING(perfctr)	perfctr_cstatus_enabled((perfctr)->cpu_state.cstatus)

#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT

static void vperfctr_ihandler(unsigned long pc);
static inline void vperfctr_handle_overflow(struct task_struct*, struct vperfctr*);

static inline void vperfctr_set_ihandler(void)
{
	perfctr_cpu_set_ihandler(vperfctr_ihandler);
}

static inline void vperfctr_clear_iresume_cstatus(struct vperfctr *perfctr)
{
	perfctr->iresume_cstatus = 0;
}

#else
static inline void vperfctr_set_ihandler(void) { }
static inline void vperfctr_clear_iresume_cstatus(struct vperfctr *perfctr) { }
#endif

#ifdef CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK

static inline void vperfctr_init_bad_cpus_allowed(struct vperfctr *perfctr)
{
	atomic_set(&perfctr->bad_cpus_allowed, 0);
}

#else	/* !CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK */
static inline void vperfctr_init_bad_cpus_allowed(struct vperfctr *perfctr) { }
#endif	/* !CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK */

/****************************************************************
 *																*
 * Resource management.											*
 *																*
 ****************************************************************/

/* XXX: perhaps relax this to number of _live_ perfctrs */
static DECLARE_MUTEX(nrctrs_mutex);
static int nrctrs;
static const char this_service[] = __FILE__;

static int inc_nrctrs(void)
{
	const char *other;

	other = NULL;
	down(&nrctrs_mutex);
	if (++nrctrs == 1) {
		other = perfctr_cpu_reserve(this_service);
		if (other)
			nrctrs = 0;
	}
	up(&nrctrs_mutex);
	if (other) {
		printk(KERN_ERR __FILE__ ": cannot operate, perfctr hardware taken by '%s'\n", other);
		return -EBUSY;
	}
	vperfctr_set_ihandler();
	return 0;
}

static void dec_nrctrs(void)
{
	down(&nrctrs_mutex);
	if (--nrctrs == 0)
		perfctr_cpu_release(this_service);
	up(&nrctrs_mutex);
}

/* Allocate a `struct vperfctr'. Claim and reserve
   an entire page so that it can be mmap():ed. */
static struct vperfctr *vperfctr_alloc(void)
{
	unsigned long page;

	if (inc_nrctrs() != 0)
		return ERR_PTR(-EBUSY);
	page = get_zeroed_page(GFP_KERNEL);
	if (!page) {
		dec_nrctrs();
		return ERR_PTR(-ENOMEM);
	}
	SetPageReserved(virt_to_page((void *)(page)));
	return (struct vperfctr*) page;
}

static void vperfctr_free(struct vperfctr *perfctr)
{
	ClearPageReserved(virt_to_page(perfctr));
	free_page((unsigned long)perfctr);
	dec_nrctrs();
}

static struct vperfctr *get_empty_vperfctr(void)
{
	struct vperfctr *perfctr = vperfctr_alloc();
	if (!IS_ERR(perfctr)) {
		atomic_set(&perfctr->count, 1);
		vperfctr_init_bad_cpus_allowed(perfctr);
		spin_lock_init(&perfctr->owner_lock);
		spin_lock_init(&perfctr->children_lock);
	}
	return perfctr;
}

static void put_vperfctr(struct vperfctr *perfctr)
{
	// returns true if the value after decrement is zero
	if (atomic_dec_and_test(&perfctr->count))
		vperfctr_free(perfctr);
}

static void scheduled_vperfctr_free(struct work_struct *work)
{
	struct vperfctr *perfctr =
		container_of(work, struct vperfctr, work);
	// printk ("vperfctr being released from %s\n", __FUNCTION__);
	vperfctr_free((struct vperfctr*)perfctr);
}

static void schedule_put_vperfctr(struct vperfctr *perfctr)
{
	if (!atomic_dec_and_test(&perfctr->count))
		return;
	INIT_WORK(&perfctr->work, scheduled_vperfctr_free);
	schedule_work(&perfctr->work);
}

static unsigned long long new_inheritance_id(void)
{
	static spinlock_t lock = SPIN_LOCK_UNLOCKED;
	static unsigned long long counter;
	unsigned long long id;

	spin_lock(&lock);
	id = ++counter;
	spin_unlock(&lock);
	return id;
}

/****************************************************************
 *								*
 * Basic counter operations.					*
 * These must all be called by the owner process only.		*
 * These must all be called with preemption disabled.		*
 *								*
 ****************************************************************/

/* PRE: IS_RUNNING(perfctr)
 * Suspend the counters.
 */
static inline void vperfctr_suspend(struct vperfctr *perfctr)
{
	perfctr_cpu_suspend(&perfctr->cpu_state);
}

static inline void vperfctr_reset_sampling_timer(struct vperfctr *perfctr)
{
	/* XXX: base the value on perfctr_info.cpu_khz instead! */
	perfctr->sampling_timer = HZ/2;
}

/* PRE: perfctr == current->thread.perfctr && IS_RUNNING(perfctr)
 * Restart the counters.
 */
static inline void vperfctr_resume(struct vperfctr *perfctr)
{
	perfctr_cpu_resume(&perfctr->cpu_state);
	vperfctr_reset_sampling_timer(perfctr);
}

static inline void vperfctr_resume_with_overflow_check(struct vperfctr *perfctr)
{
#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT
	if (perfctr_cpu_has_pending_interrupt(&perfctr->cpu_state)) {
		vperfctr_handle_overflow(current, perfctr);
		return;
	}
#endif
	vperfctr_resume(perfctr);
}

/* Sample the counters but do not suspend them. */
static inline void vperfctr_sample(struct vperfctr *perfctr)
{
	if (IS_RUNNING(perfctr)) {
		// logical place to see if the counters are ours else return
	
		perfctr_cpu_sample(&perfctr->cpu_state);
		vperfctr_reset_sampling_timer(perfctr);
	}
}

#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT

// PREEMPT note: called in IRQ context with preemption disabled.
static void vperfctr_ihandler(unsigned long pc)
{
	struct task_struct *tsk = current;
	struct vperfctr *perfctr;
	unsigned int pmc, cstatus, now = 0;
	int i;

	perfctr = tsk->thread.perfctr;
	if (!perfctr) {
		return;
	}
	if (!perfctr_cstatus_has_ictrs(perfctr->cpu_state.cstatus)) {
		return;
	}

	// if someone has really overflown then continue else return
	// just read, don't freeze them
	
	cstatus = perfctr->cpu_state.cstatus;
	for (i = perfctr_cstatus_nractrs(cstatus); (i < perfctr_cstatus_nrctrs(cstatus)) && ((int)now >= 0); ++i) {
		pmc = perfctr->cpu_state.pmc[i].map;
		now = read_pmc(pmc);
	}
	if ((int)now >= 0) {
		return;
	}

	// Fine, we are suspending the counters and reading them. vperfctr_suspend() 
	// in turn invokes _suspend() on i-mode ctrs (where they are frozen and read) 
	// and a-mode counters (where they are just read)

	vperfctr_suspend(perfctr);

	// Ok, Signal to the userland is sent in the following routine. But before that
	// the following routine calls vperfctr_resume() if the TSC counting is on.
	// what happens in that resume is just the TSC value is read and stored in the
	// 'start' state of the TSC

	vperfctr_handle_overflow(tsk, perfctr);
}

static inline void vperfctr_handle_overflow(struct task_struct *tsk,
				     struct vperfctr *perfctr)
{
	unsigned int pmc_mask;
	siginfo_t si;
	sigset_t old_blocked;

	pmc_mask = perfctr_cpu_identify_overflow(&perfctr->cpu_state);
	if (!pmc_mask) {
		printk(KERN_ERR "%s: BUG! pid %d has unidentifiable overflow source\n", __FUNCTION__, tsk->pid);
		return;
	}
	/* suspend a-mode and i-mode PMCs, leaving only TSC on */
	/* XXX: some people also want to suspend the TSC */

	// we are storing 'cpu_state.cstatus' in 'iresume_cstatus' because
	// in vperfctr_resume, we only want to read the status of those
	// In the following TSC is resumed and continues to collect the
	// stats

	// if 'perfctr->iresume_cstatus' is not updated below, vperfctr_iresume() fails
	// as it thinks it was spuriously called inspite of absence of i-mode counters.
	// vperfctr_iresume() -> ... -> do_vperfctr_iresume() is a different thread of
	// execution from vperfctr_resume() -> ... -> vperfctr_iresume() -> __write_control() ->
	// ...
	perfctr->iresume_cstatus = perfctr->cpu_state.cstatus;

	if (perfctr_cstatus_has_tsc(perfctr->iresume_cstatus)) {
		perfctr->cpu_state.cstatus = perfctr_mk_cstatus(1, 0, 0);
		vperfctr_resume(perfctr);
	} 
	else {
		perfctr->cpu_state.cstatus = 0;
	}

	// the following siginfo_t structure helps the kernel in invoking
	// the correct signal handler. Is that right ?
	// what's the deal with si_errno? what does it say ? si_code ?

	si.si_signo = perfctr->si_signo;
	si.si_errno = 0;
	si.si_code = SI_PMC_OVF;
	si.si_pmc_ovf_mask = pmc_mask;

	/* deliver signal without waking up the receiver */
	spin_lock_irq(&tsk->sighand->siglock);
	old_blocked = tsk->blocked;
	sigaddset(&tsk->blocked, si.si_signo);
	spin_unlock_irq(&tsk->sighand->siglock);

	if (!send_sig_info(si.si_signo, &si, tsk)) {
		send_sig(si.si_signo, tsk, 1);
	}

	spin_lock_irq(&tsk->sighand->siglock);
	tsk->blocked = old_blocked;
	recalc_sigpending();
	spin_unlock_irq(&tsk->sighand->siglock);
}

#else
static void vperfctr_ihandler(unsigned long pc) { };
#endif


/****************************************************************
 *								*
 * Process management operations.				*
 * These must all, with the exception of vperfctr_unlink()	*
 * and __vperfctr_set_cpus_allowed(), be called by the owner	*
 * process only.						*
 *								*
 ****************************************************************/

/* do_fork() -> copy_process() -> copy_thread() -> __vperfctr_copy().
 * Inherit the parent's perfctr settings to the child.
 * PREEMPT note: do_fork() etc do not run with preemption disabled.
*/
void __vperfctr_copy(struct task_struct *child_tsk, struct pt_regs *regs)
{
	struct vperfctr *parent_perfctr;
	struct vperfctr *child_perfctr;

	/* Do not inherit perfctr settings to kernel-generated
	   threads, like those created by kmod. */
	child_perfctr = NULL;
	if (!user_mode(regs)) {
		goto out;
	}

	/* Allocation may sleep. Do it before the critical region. */
	child_perfctr = get_empty_vperfctr();
	if (IS_ERR(child_perfctr)) {
		child_perfctr = NULL;
		goto out;
	}

	/* Although we're executing in the parent, if it is scheduled
	   then a remote monitor may attach and change the perfctr
	   pointer or the object it points to. This may already have
	   occurred when we get here, so the old copy of the pointer
	   in the child cannot be trusted. */
	preempt_disable();
	parent_perfctr = current->thread.perfctr;
	if (parent_perfctr) {
		child_perfctr->cpu_state.control = parent_perfctr->cpu_state.control;
		child_perfctr->si_signo = parent_perfctr->si_signo;
		child_perfctr->inheritance_id = parent_perfctr->inheritance_id;
	}
	preempt_enable();
	if (!parent_perfctr) {
		put_vperfctr(child_perfctr);
		child_perfctr = NULL;
		goto out;
	}
	(void)perfctr_cpu_update_control(&child_perfctr->cpu_state, 0);
	child_perfctr->owner = child_tsk;
 out:
	child_tsk->thread.perfctr = child_perfctr;
}

/* Called from exit_thread() or do_vperfctr_unlink().
 * If the counters are running, stop them and sample their final values.
 * Mark the vperfctr object as dead.
 * Optionally detach the vperfctr object from its owner task.
 * PREEMPT note: exit_thread() does not run with preemption disabled.
 */

// exit_thread() --> ... ; do_vperfctr_unlink() --> ...
static void vperfctr_unlink(struct task_struct *owner, struct vperfctr *perfctr, int do_unlink)
{
	/* this synchronises with sys_vperfctr() */
	spin_lock(&perfctr->owner_lock);
	perfctr->owner = NULL;
	spin_unlock(&perfctr->owner_lock);

	/* perfctr suspend+detach must be atomic wrt process suspend */
	/* this also synchronises with perfctr_set_cpus_allowed() */
	task_lock(owner);
	if (IS_RUNNING(perfctr) && owner == current)
		vperfctr_suspend(perfctr);
	if (do_unlink)
		owner->thread.perfctr = NULL;
	task_unlock(owner);

	perfctr->cpu_state.cstatus = 0;
	vperfctr_clear_iresume_cstatus(perfctr);
	if (do_unlink)
		put_vperfctr(perfctr);
}

// called from the _exit_thread() in arch/mips/kernel/process.c
// this is called by tasks onselves. Since we specified '0' as the 3rd
// argument for vperfctr_unlink(), only the stats are collected but the
// the task structure itself is not freed

void __vperfctr_exit(struct vperfctr *perfctr)
{
	vperfctr_unlink(current, perfctr, 0);
}

// release_task() is called during the deallocation of resources of
// a zombie thread/process and not when a process/thread exits

/* release_task() -> perfctr_release_task() -> __vperfctr_release().
 * A task is being released. If it inherited its perfctr settings
 * from its parent, then merge its final counts back into the parent.
 * Then unlink the child's perfctr.
 * PRE: caller has write_lock_irq(&tasklist_lock).
 * PREEMPT note: preemption is disabled due to tasklist_lock.
 */

static void do_vperfctr_release(struct vperfctr *child_perfctr, struct task_struct *parent_tsk)
{
	struct vperfctr *parent_perfctr;
	unsigned int cstatus, nrctrs, i;

	parent_perfctr = parent_tsk->thread.perfctr;
	if (parent_perfctr && child_perfctr) {
		
		// since more than one child can try to add to parent's
		// counters, we need a lock

		spin_lock(&parent_perfctr->children_lock);
		if (parent_perfctr->inheritance_id == child_perfctr->inheritance_id) {
			cstatus = parent_perfctr->cpu_state.cstatus;
			if (perfctr_cstatus_has_tsc(cstatus))
				parent_perfctr->children.tsc +=
					child_perfctr->cpu_state.tsc_sum +
					child_perfctr->children.tsc;
			nrctrs = perfctr_cstatus_nrctrs(cstatus);
			for(i = 0; i < nrctrs; ++i)
				parent_perfctr->children.pmc[i] +=
					child_perfctr->cpu_state.pmc[i].sum +
					child_perfctr->children.pmc[i];
		}
		spin_unlock(&parent_perfctr->children_lock);
	}

	// now that we reaped the data from child's task structure
	// the child's task structure can be freed. Only the child's
	// vperfctr structure seems to be released. Is the 'task_struct'
	// released in __vperfctr_release() itself? Doesn't seem so.
	schedule_put_vperfctr(child_perfctr);
}

static void scheduled_release(struct work_struct *data)
{
	struct vperfctr *child_perfctr = 
		container_of(data, struct vperfctr, work);
	struct task_struct *parent_tsk = child_perfctr->parent_tsk;

	// why are we getting a lock on the parent task structure ?
	// of course, we incremented the reference count to parent's task_struct
	task_lock(parent_tsk);
	do_vperfctr_release(child_perfctr, parent_tsk);
	task_unlock(parent_tsk);
	
	// good, the incremented reference count of the parent task is now
	// decremented, now that we are done adding up our counts to that
	// of the parent
	put_task_struct(parent_tsk);
}

void __vperfctr_release(struct task_struct *child_tsk)
{
	struct task_struct *parent_tsk = child_tsk->parent;
	struct vperfctr *child_perfctr = child_tsk->thread.perfctr;

	// this is invoked either in the waitpid() or if there the parent is not
	// interesting in its children. In the latter case, "parent_tsk != current"

	// one releases oneself, when the parent is not interested in one's data
	// but even then we would like to add our counters to those of the parent's
	
	// another step towards freeing the task_struct(ure).
	child_tsk->thread.perfctr = NULL;

	// if the parent is releasing the children's task structure, then it (the
	// parent) can go ahead and add the children's vperfctr's values to the
	// 'children' field in the parent's 'vperfctr' structure.
	// So, am 'I' the parent of the task_structure I am attempting to release?
	// When current == parent_tsk, the child's counts can be merged
	// into the parent's immediately. This is the common case.

	// printk ("%s, %d\n", __FUNCTION__, __LINE__);
	if (child_perfctr == NULL) {
		// printk("%s, %d, child_perfctr == NULL\n", __FUNCTION__, __LINE__);
	}

	if (parent_tsk == current)
		do_vperfctr_release(child_perfctr, parent_tsk);
	else {

		/* When current != parent_tsk, the parent must be task_lock()ed
		 * before its perfctr state can be accessed. task_lock() is illegal
		 * here due to the write_lock_irq(&tasklist_lock) in release_task(),
		 * so the operation is done via schedule_work(). Also, increment
		 * the reference count of parent's task_struct so that it will not be
		 * freed for good
	     */

		get_task_struct(parent_tsk);	// increments the reference count

		INIT_WORK(&child_perfctr->work, scheduled_release);
		child_perfctr->parent_tsk = parent_tsk;
		schedule_work(&child_perfctr->work);
	}
}

/* schedule() --> switch_to() --> .. --> __vperfctr_suspend().
 * If the counters are running, suspend them.
 * PREEMPT note: switch_to() runs with preemption disabled.
 */
void __vperfctr_suspend(struct vperfctr *perfctr)
{
	if (IS_RUNNING(perfctr))
		vperfctr_suspend(perfctr);
}

/* schedule() --> switch_to() --> .. --> __vperfctr_resume().
 * PRE: perfctr == current->thread.perfctr
 * If the counters are runnable, resume them.
 * PREEMPT note: switch_to() runs with preemption disabled.
 */
void __vperfctr_resume(struct vperfctr *perfctr)
{
	if (IS_RUNNING(perfctr)) {
		// logical place to add the functionality

	// what exactly are we doing here ?
#ifdef CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK
		if (unlikely(atomic_read(&perfctr->bad_cpus_allowed)) &&
		    perfctr_cstatus_nrctrs(perfctr->cpu_state.cstatus)) {
			perfctr->cpu_state.cstatus = 0;
			vperfctr_clear_iresume_cstatus(perfctr);
			BUG_ON(current->state != TASK_RUNNING);
			send_sig(SIGILL, current, 1);
			return;
		}
#endif
		vperfctr_resume_with_overflow_check(perfctr);
	}
}

/* Called from update_one_process() [triggered by timer interrupt].
 * PRE: perfctr == current->thread.perfctr.
 * Sample the counters but do not suspend them.
 * Needed to avoid precision loss due to multiple counter
 * wraparounds between resume/suspend for CPU-bound processes.
 * PREEMPT note: called in IRQ context with preemption disabled.
 */
void __vperfctr_sample(struct vperfctr *perfctr)
{
	if (--perfctr->sampling_timer == 0)
		vperfctr_sample(perfctr);
}

#ifdef CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK
/* Called from set_cpus_allowed().
 * PRE: current holds task_lock(owner)
 * PRE: owner->thread.perfctr == perfctr
 */
void __vperfctr_set_cpus_allowed(struct task_struct *owner,
				 struct vperfctr *perfctr,
				 cpumask_t new_mask)
{
	if (cpus_intersects(new_mask, perfctr_cpus_forbidden_mask)) {
		atomic_set(&perfctr->bad_cpus_allowed, 1);
		if (printk_ratelimit())
			printk(KERN_WARNING "perfctr: process %d (comm %s) issued unsafe"
				" set_cpus_allowed() on process %d (comm %s)\n",
			    	current->pid, current->comm, owner->pid, owner->comm);
	} else
		atomic_set(&perfctr->bad_cpus_allowed, 0);
}
#endif

/****************************************************************
 *								*
 * Virtual perfctr system calls implementation.			*
 * These can be called by the owner process (tsk == current),	*
 * a monitor process which has the owner under ptrace ATTACH	*
 * control (tsk && tsk != current), or anyone with a handle to	*
 * an unlinked perfctr (!tsk).					*
 *								*
 ****************************************************************/

static int do_vperfctr_control(struct vperfctr *perfctr,
			       const struct vperfctr_control __user *argp,
			       unsigned int argbytes,
			       struct task_struct *tsk)
{
	struct vperfctr_control *control;
	int err;
	unsigned int next_cstatus;
	unsigned int nrctrs, i;

	if (!tsk) {
		return -ESRCH;	/* attempt to update unlinked perfctr */
	}

	/* The control object can be large (over 300 bytes on i386),
	   so kmalloc() it instead of storing it on the stack.
	   We must use task-private storage to prevent racing with a
	   monitor process attaching to us before the non-preemptible
	   perfctr update step. Therefore we cannot store the copy
	   in the perfctr object itself. */
	control = kmalloc(sizeof(*control), GFP_USER);
	if (!control) {
		return -ENOMEM;
	}

	err = -EINVAL;
	if (argbytes > sizeof *control) {
		goto out_kfree;
	}

	err = -EFAULT;
	if (copy_from_user(control, argp, argbytes)) {
		goto out_kfree;
	}

	if (argbytes < sizeof *control)
		memset((char*)control + argbytes, 0, sizeof *control - argbytes);

	// figure out what is happening in the following 'if' loop

	if (control->cpu_control.nractrs || control->cpu_control.nrictrs) {
		cpumask_t old_mask, new_mask;

		old_mask = tsk->cpus_allowed;
		cpus_andnot(new_mask, old_mask, perfctr_cpus_forbidden_mask);

		err = -EINVAL;
		if (cpus_empty(new_mask)) {
			goto out_kfree;
		}
		if (!cpus_equal(new_mask, old_mask))
			set_cpus_allowed(tsk, new_mask);
	}

	/* PREEMPT note: preemption is disabled over the entire
	   region since we're updating an active perfctr. */
	preempt_disable();

	// the task whose control register I am changing might actually be
	// in suspended state. That can happen when the other is executing
	// under the control of another task as in the case of debugging
	// or ptrace. However, if the write_control is done for the current
	// executing process, first suspend them and then do the update
	// why are we resetting 'perfctr->cpu_state.cstatus' ?

	if (IS_RUNNING(perfctr)) {
		if (tsk == current)
			vperfctr_suspend(perfctr);
	
		// not sure why we are zeroing out the following explicitly
		perfctr->cpu_state.cstatus = 0;
		vperfctr_clear_iresume_cstatus(perfctr);
	}

	// coying the user-specified control values to 'state'
	perfctr->cpu_state.control = control->cpu_control;

	/* remote access note: perfctr_cpu_update_control() is ok */
	err = perfctr_cpu_update_control(&perfctr->cpu_state, 0);
	if (err < 0) {
		goto out;
	}
	next_cstatus = perfctr->cpu_state.cstatus;
	if (!perfctr_cstatus_enabled(next_cstatus))
		goto out;

	/* XXX: validate si_signo? */
	perfctr->si_signo = control->si_signo;

	if (!perfctr_cstatus_has_tsc(next_cstatus))
		perfctr->cpu_state.tsc_sum = 0;

	nrctrs = perfctr_cstatus_nrctrs(next_cstatus);
	for(i = 0; i < nrctrs; ++i)
		if (!(control->preserve & (1<<i)))
			perfctr->cpu_state.pmc[i].sum = 0;

	// I am not sure why we are removing the inheritance just because
	// we updated the control information. True, because the children might
	// be performing something else. So, the control will have to be set
	// before spawning any children

	spin_lock(&perfctr->children_lock);
	perfctr->inheritance_id = new_inheritance_id();
	memset(&perfctr->children, 0, sizeof perfctr->children);
	spin_unlock(&perfctr->children_lock);

	if (tsk == current) {
		vperfctr_resume(perfctr);
	}

 out:
	preempt_enable();
 out_kfree:
	kfree(control);
	return err;
}

static int do_vperfctr_iresume(struct vperfctr *perfctr, const struct task_struct *tsk)
{
#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT
	unsigned int iresume_cstatus;

	if (!tsk)
		return -ESRCH;	/* attempt to update unlinked perfctr */

	iresume_cstatus = perfctr->iresume_cstatus;
	if (!perfctr_cstatus_has_ictrs(iresume_cstatus)) {
		return -EPERM;
	}

	/* PREEMPT note: preemption is disabled over the entire
	   region because we're updating an active perfctr. */
	preempt_disable();

	// this is for resuming a task whose signal was handled prior to this call
	// are the i-mode counters frozen before the overflow-signal is delivered
	// yes, they are. in the suspend call invoked in the handler

	// why exactly are we suspending the following? Makes sense ... if the
	// counters are already running, then one should not just resume the task
	// which will overwrite the PMC registers with old values. Nice. Under
	// what condition do counters continue to count after the signal is delivered
	// remember TSC was not suspend in the handler and continues to count

	if (IS_RUNNING(perfctr) && tsk == current)
		vperfctr_suspend(perfctr);

	// setting the cstatus of 'cpu_state' back to what it was prior to its
	// zeroing out in the interrupt handler
	perfctr->cpu_state.cstatus = iresume_cstatus;
	perfctr->iresume_cstatus = 0;

	/* remote access note: perfctr_cpu_ireload() is ok */
	// the following forces the reload of control registers that 
	// unfreezes the i-mode registers
	perfctr_cpu_ireload(&perfctr->cpu_state);

	if (tsk == current)
		vperfctr_resume(perfctr);

	preempt_enable();

	return 0;
#else
	return -ENOSYS;
#endif
}

static int do_vperfctr_unlink(struct vperfctr *perfctr, struct task_struct *tsk)
{
	if (tsk)
		vperfctr_unlink(tsk, perfctr, 1);
	return 0;
}

// sys_vperfctr_read() -> this()
static int do_vperfctr_read(struct vperfctr *perfctr,
			    unsigned int cmd,
			    void __user *argp,
			    unsigned int argbytes,
			    struct task_struct *tsk)
{
	union {
		struct perfctr_sum_ctrs sum;
		struct vperfctr_control control;
		struct perfctr_sum_ctrs children;
	} *tmp;
	unsigned int tmpbytes;
	int ret;

	/* The state snapshot can be large, so kmalloc() it instead of storing it on the stack.
	   We must use task-private storage to prevent racing with a monitor process attaching to 
	   us during the preemptible copy_to_user() step. Therefore we cannot store the snapshot
	   in the perfctr object itself. 
    */
	tmp = kmalloc(sizeof(*tmp), GFP_USER);
	if (!tmp)
		return -ENOMEM;

	/* PREEMPT note: While we're reading our own control, another process may ptrace ATTACH 
       to us and update our control. Disable preemption to ensure we get a consistent copy.
	   Not needed for other cases since the perfctr is either unlinked or its owner is ptrace 
       ATTACH suspended by us. 
	*/
	if (tsk == current)
		preempt_disable();

	switch (cmd) {
    	case VPERFCTR_READ_SUM: {
    		int j;
    
    		vperfctr_sample(perfctr);
    		tmp->sum.tsc = perfctr->cpu_state.tsc_sum;
    		for(j = 0; j < ARRAY_SIZE(tmp->sum.pmc); ++j)
    			tmp->sum.pmc[j] = perfctr->cpu_state.pmc[j].sum;
    		tmpbytes = sizeof(tmp->sum);
    	}
    	break;
    	case VPERFCTR_READ_CONTROL:
    		tmp->control.si_signo = perfctr->si_signo;
    		tmp->control.cpu_control = perfctr->cpu_state.control;
    		tmp->control.preserve = 0;
    		tmpbytes = sizeof(tmp->control);
    	break;
    	case VPERFCTR_READ_CHILDREN:
    		if (tsk)
    			spin_lock(&perfctr->children_lock);
    		tmp->children = perfctr->children;
    		if (tsk)
    			spin_unlock(&perfctr->children_lock);
    		tmpbytes = sizeof(tmp->children);
    	break;
    	default:
    		tmpbytes = 0;
	}

	if (tsk == current)
		preempt_enable();

	ret = -EINVAL;
	if (tmpbytes > argbytes)
		tmpbytes = argbytes;
	if (tmpbytes > 0) {
		ret = tmpbytes;
		if (copy_to_user(argp, tmp, tmpbytes))
			ret = -EFAULT;
	}
	kfree(tmp);
	return ret;
}

/****************************************************************
 *																*
 * Virtual perfctr file operations.								*
 *																*
 ****************************************************************/

static int vperfctr_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct vperfctr *perfctr;

	/* Only allow read-only mapping of first page. */
	// (pgprot_val(vma->vm_page_prot) & _PAGE_RW) ||

	if ( ((vma->vm_end - vma->vm_start) != PERFCTR_PAGE_SIZE) || (vma->vm_pgoff != 0) ||
	     (pgprot_val(vma->vm_page_prot) & _PAGE_WRITE) || (vma->vm_flags & (VM_WRITE | VM_MAYWRITE))
       ) {
		return -EPERM;
	}
	perfctr = filp->private_data;
	if (!perfctr) {
		return -EPERM;
	}

    return remap_pfn_range (vma, vma->vm_start, 
		    (virt_to_phys(perfctr) >> PAGE_SHIFT),
			PERFCTR_PAGE_SIZE, vma->vm_page_prot);
}

static int vperfctr_release(struct inode *inode, struct file *filp)
{
	struct vperfctr *perfctr = filp->private_data;
	filp->private_data = NULL;

	// printk("%s, %d\n", __FUNCTION__, __LINE__);
	if (perfctr) {
		// printk("%s, %d\n", __FUNCTION__, __LINE__);
		put_vperfctr(perfctr);
	}
	return 0;
}

static struct file_operations vperfctr_file_ops = {
	.mmap = vperfctr_mmap,
	.release = vperfctr_release,
};

/****************************************************************
 *																*
 * File system for virtual perfctrs. Based on pipefs.			*
 *																*
 ****************************************************************/

#define VPERFCTRFS_MAGIC (('V'<<24)|('P'<<16)|('M'<<8)|('C'))

/* The code to set up a `struct file_system_type' for a pseudo fs
   is unfortunately not the same in 2.4 and 2.6. */
#include <linux/mount.h> /* needed for 2.6, included by fs.h in 2.4 */

static struct vfsmount *vperfctr_mnt;
static int 
vperfctrfs_get_sb(struct file_system_type *fs_type,
		  int flags, const char *dev_name, void *data,  
		  struct vfsmount *mount)
{
	return get_sb_pseudo(fs_type, "vperfctr:", NULL, VPERFCTRFS_MAGIC,
						mount);
}

static struct file_system_type vperfctrfs_type = {
	.name		= "vperfctrfs",
	.get_sb		= vperfctrfs_get_sb,
	.kill_sb	= kill_anon_super,
};

/* XXX: check if s/vperfctr_mnt/vperfctrfs_type.kern_mnt/ would work */
#define vperfctr_fs_init_done()	(vperfctr_mnt != NULL)

static int __init vperfctrfs_init(void)
{
	int err = register_filesystem(&vperfctrfs_type);
	if (!err) {
		vperfctr_mnt = kern_mount(&vperfctrfs_type);
		if (!IS_ERR(vperfctr_mnt))
			return 0;
		err = PTR_ERR(vperfctr_mnt);
		unregister_filesystem(&vperfctrfs_type);
		vperfctr_mnt = NULL;
	}
	return err;
}

static void __exit vperfctrfs_exit(void)
{
	unregister_filesystem(&vperfctrfs_type);
	mntput(vperfctr_mnt);
}

static struct inode *vperfctr_get_inode(void)
{
	struct inode *inode;

	inode = new_inode(vperfctr_mnt->mnt_sb);
	if (!inode)
		return NULL;
	inode->i_fop = &vperfctr_file_ops;
	inode->i_state = I_DIRTY;
	inode->i_mode = S_IFCHR | S_IRUSR | S_IWUSR;
	inode->i_uid = current->cred->fsuid;
	inode->i_gid = current->cred->fsgid;
	inode->i_atime = inode->i_mtime = inode->i_ctime = CURRENT_TIME;
	return inode;
}

static int vperfctrfs_delete_dentry(struct dentry *dentry)
{
	return 1;
}

static struct dentry_operations vperfctrfs_dentry_operations = {
	.d_delete	= vperfctrfs_delete_dentry,
};

static struct dentry *vperfctr_d_alloc_root(struct inode *inode)
{
	struct qstr this;
	char name[32];
	struct dentry *dentry;

	sprintf(name, "[%lu]", inode->i_ino);
	this.name = name;
	this.len = strlen(name);
	this.hash = inode->i_ino; /* will go */
	dentry = d_alloc(vperfctr_mnt->mnt_sb->s_root, &this);
	if (dentry) {
		dentry->d_op = &vperfctrfs_dentry_operations;
		d_add(dentry, inode);
	}
	return dentry;
}

static struct file *vperfctr_get_filp(void)
{
	struct file *filp;
	struct inode *inode;
	struct dentry *dentry;

	filp = get_empty_filp();
	if (!filp)
		goto out;
	inode = vperfctr_get_inode();
	if (!inode)
		goto out_filp;
	dentry = vperfctr_d_alloc_root(inode);
	if (!dentry)
		goto out_inode;

	filp->f_vfsmnt = mntget(vperfctr_mnt);
	filp->f_dentry = dentry;
	filp->f_mapping = dentry->d_inode->i_mapping;

	filp->f_pos = 0;
	filp->f_flags = 0;
	filp->f_op = &vperfctr_file_ops; /* fops_get() if MODULE */
	filp->f_mode = FMODE_READ;
	filp->f_version = 0;

	return filp;

 out_inode:
	iput(inode);
 out_filp:
	put_filp(filp);	/* doesn't run ->release() like fput() does */
 out:
	return NULL;
}

/****************************************************************
 *								*
 * Virtual perfctr actual system calls.				*
 *								*
 ****************************************************************/

/* tid is the actual task/thread id (née pid, stored as ->pid),
   pid/tgid is that 2.6 thread group id crap (stored as ->tgid) */
asmlinkage long sys_vperfctr_open(int tid, int creat)
{
	struct file *filp;
	struct task_struct *tsk;
	struct vperfctr *perfctr;
	int err;
	int fd;

	if (!vperfctr_fs_init_done())
		return -ENODEV;
	filp = vperfctr_get_filp();
	if (!filp)
		return -ENOMEM;
	err = fd = get_unused_fd();
	if (err < 0)
		goto err_filp;
	perfctr = NULL;
	if (creat) {
		perfctr = get_empty_vperfctr(); /* may sleep */
		if (IS_ERR(perfctr)) {
			err = PTR_ERR(perfctr);
			goto err_fd;
		}
	}
	tsk = current;

	if (tid != 0 && tid != tsk->pid) { /* remote? */
		// tasklist_lock is to access the linked list of task_struct structures exclusively
		read_lock(&tasklist_lock);
		//tsk = find_task_by_pid(tid);
		tsk = find_task_by_pid_ns(tid, current->nsproxy->pid_ns);
		if (tsk)
			get_task_struct(tsk);
		read_unlock(&tasklist_lock);
		err = -ESRCH;
		if (!tsk)
			goto err_perfctr;
		err = ptrace_check_attach(tsk, 0);
		if (err < 0)
			goto err_tsk;
	}
	if (creat) {
		/* check+install must be atomic to prevent remote-control races */
		task_lock(tsk);
		if (!tsk->thread.perfctr) {
			perfctr->owner = tsk;
			tsk->thread.perfctr = perfctr;
			err = 0;
		} else
			err = -EEXIST;
		task_unlock(tsk);
		if (err)
			goto err_tsk;
	} else {
		perfctr = tsk->thread.perfctr;
		/* XXX: Old API needed to allow NULL perfctr here.
		   Do we want to keep or change that rule? */
	}
	filp->private_data = perfctr;
	if (perfctr)
		atomic_inc(&perfctr->count);
	if (tsk != current)
		put_task_struct(tsk);
	#if 0
	if (perfctr) {
    	printk ("sys_vperfctr_open(): fd = %d, perfctr is NOT null\n", fd);
	}
	else {
    	printk ("sys_vperfctr_open(): fd = %d, perfctr is null\n", fd);
	}
	#endif
	fd_install(fd, filp);
	return fd;
 err_tsk:
	if (tsk != current)
		put_task_struct(tsk);
 err_perfctr:
	if (perfctr)	/* can only occur if creat != 0 */
		put_vperfctr(perfctr);
 err_fd:
	put_unused_fd(fd);
 err_filp:
	fput(filp);
	return err;
}

static struct vperfctr *fd_get_vperfctr(int fd)
{
	struct vperfctr *perfctr;
	struct file *filp;
	int err;

	err = -EBADF;
	filp = fget(fd);
	if (!filp)
		goto out;
	err = -EINVAL;
	if (filp->f_op != &vperfctr_file_ops)
		goto out_filp;
	perfctr = filp->private_data;
	if (!perfctr)
		goto out_filp;
	atomic_inc(&perfctr->count);
	fput(filp);
	return perfctr;
 out_filp:
	fput(filp);
 out:
	return ERR_PTR(err);
}

static struct task_struct *vperfctr_get_tsk(struct vperfctr *perfctr)
{
	struct task_struct *tsk;

	tsk = current;
	if (perfctr != current->thread.perfctr) {
		/* this synchronises with vperfctr_unlink() and itself */
		spin_lock(&perfctr->owner_lock);
		tsk = perfctr->owner;
		if (tsk)
			get_task_struct(tsk);
		spin_unlock(&perfctr->owner_lock);
		if (tsk) {
			int ret = ptrace_check_attach(tsk, 0);
			if (ret < 0) {
				put_task_struct(tsk);
				return ERR_PTR(ret);
			}
		}
	}
	return tsk;
}

static void vperfctr_put_tsk(struct task_struct *tsk)
{
	if (tsk && tsk != current)
		put_task_struct(tsk);
}

asmlinkage long sys_vperfctr_control(int fd,
				     const struct vperfctr_control __user *argp,
				     unsigned int argbytes)
{
	struct vperfctr *perfctr;
	struct task_struct *tsk;
	int ret;

	perfctr = fd_get_vperfctr(fd);
	if (IS_ERR(perfctr)) {
		printk ("sys_vperfctr_control(): fd: %d, perfctr == NULL\n", fd);
		return PTR_ERR(perfctr);
	}
	tsk = vperfctr_get_tsk(perfctr);
	if (IS_ERR(tsk)) {
		ret = PTR_ERR(tsk);
		goto out;
	}

	// About the arguments:
	// perfctr: kernel-level 'struct vperfctr'
	// argp: pointer to a user level 'struct vperfctr_control'
    // argp: sizeof(struct vperfctr_control)
	// tsk, the task_struct associated with 'perfctr'

	ret = do_vperfctr_control(perfctr, argp, argbytes, tsk);
	vperfctr_put_tsk(tsk);
 out:
	put_vperfctr(perfctr);
	return ret;
}

asmlinkage long sys_vperfctr_unlink(int fd)
{
	struct vperfctr *perfctr;
	struct task_struct *tsk;
	int ret;

	perfctr = fd_get_vperfctr(fd);
	if (IS_ERR(perfctr))
		return PTR_ERR(perfctr);
	tsk = vperfctr_get_tsk(perfctr);
	if (IS_ERR(tsk)) {
		ret = PTR_ERR(tsk);
		goto out;
	}

	ret = do_vperfctr_unlink(perfctr, tsk);
	vperfctr_put_tsk(tsk);
 out:
	put_vperfctr(perfctr);
	return ret;
}

asmlinkage long sys_vperfctr_iresume(int fd)
{
	struct vperfctr *perfctr;
	struct task_struct *tsk;
	int ret;


	perfctr = fd_get_vperfctr(fd);
	if (IS_ERR(perfctr)) {
		return PTR_ERR(perfctr);
	}

	tsk = vperfctr_get_tsk(perfctr);
	if (IS_ERR(tsk)) {
		ret = PTR_ERR(tsk);
		goto out;
	}

	ret = do_vperfctr_iresume(perfctr, tsk);

	vperfctr_put_tsk(tsk);
 out:
	put_vperfctr(perfctr);
	return ret;
}

asmlinkage long sys_vperfctr_read(int fd, unsigned int cmd, void __user *argp, unsigned int argbytes)
{
	struct vperfctr *perfctr;
	struct task_struct *tsk;
	int ret;

	perfctr = fd_get_vperfctr(fd);
	if (IS_ERR(perfctr)) {
		return PTR_ERR(perfctr);
	}
	tsk = vperfctr_get_tsk(perfctr);
	if (IS_ERR(tsk)) {
		ret = PTR_ERR(tsk);
		goto out;
	}

	ret = do_vperfctr_read(perfctr, cmd, argp, argbytes, tsk);

	vperfctr_put_tsk(tsk);
 out:
	put_vperfctr(perfctr);
	return ret;
}

/****************************************************************
 *								*
 * module_init/exit						*
 *								*
 ****************************************************************/

int __init vperfctr_init(void)
{
	return vperfctrfs_init();
}

void __exit vperfctr_exit(void)
{
	vperfctrfs_exit();
}
