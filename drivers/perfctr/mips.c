/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: mips.c,v 1.1.2.9 2007-10-31 17:34:41 kmurthy Exp $
 * MIPS64 performance-monitoring counters driver.
 *
 * Copyright (C) 2004
 */
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/perfctr.h>
#include <asm/time.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/perfctr.h>

#include <asm/mipsregs.h>

#include "mips_tests.h"

// Support for lazy evntsel and perfctr control regiters updates.
// good, I don't see any point in maintaing the values of the
// counters.

struct per_cpu_cache {    /* roughly a subset of perfctr_cpu_state */
    union {
        unsigned int id;    /* cache owner id */
    } k1;

    /* Physically indexed cache of control registers */
    unsigned int ctrl_regs[2];
};
static DEFINE_PER_CPU(struct per_cpu_cache, per_cpu_cache);
#define __get_cpu_cache(cpu) (&per_cpu(per_cpu_cache, cpu))
#define get_cpu_cache()    (&__get_cpu_var(per_cpu_cache))

/* Structure for counter snapshots, as 32-bit values. */
struct perfctr_low_ctrs {
    unsigned int tsc;
    unsigned int pmc[2];
};

static int pm_type;

static struct {
	spinlock_t lock ____cacheline_aligned;
	int current_thread;
} pmc_resource[8];

// Bits users shouldn't set in control registers
// #define MIPS_XLR_PERFCTRL_RESERVED        0

// returns a new id each time
static unsigned int new_id(void)
{
    static spinlock_t lock = SPIN_LOCK_UNLOCKED;
    static unsigned int counter;
    int id;

    spin_lock(&lock);
    id = ++counter;
    spin_unlock(&lock);
    return id;
}

#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT

static void perfctr_default_ihandler(unsigned long pc)
{
    // do nothing
    return;
}

perfctr_ihandler_t perfctr_ihandler = perfctr_default_ihandler;

void perfctr_cpu_set_ihandler(perfctr_ihandler_t ihandler)
{
    perfctr_ihandler = ihandler ? ihandler : perfctr_default_ihandler;
}
#else
#define perfctr_cstatus_has_ictrs(cstatus)    0
#endif

#if defined(CONFIG_SMP) && defined(CONFIG_PERFCTR_INTERRUPT_SUPPORT)

// to set the cpu on which the current thread is suspended
// aids in telling if the control and PMC registers are warm

static inline void
set_isuspend_cpu(struct perfctr_cpu_state *state, int cpu)
{
    state->k1.isuspend_cpu = cpu;
}

static inline int
is_isuspend_cpu(const struct perfctr_cpu_state *state, int cpu)
{
    return state->k1.isuspend_cpu == cpu;
}

static inline void clear_isuspend_cpu(struct perfctr_cpu_state *state)
{
    state->k1.isuspend_cpu = NR_CPUS;
}

#else
static inline void set_isuspend_cpu(struct perfctr_cpu_state *state, int cpu) { }
static inline int is_isuspend_cpu(const struct perfctr_cpu_state *state, int cpu) { return 1; }
static inline void clear_isuspend_cpu(struct perfctr_cpu_state *state) { }
#endif

/****************************************************************
 *                                                                *
 * Driver procedures.                                            *
 *                                                                *
 ****************************************************************/

/*
 * The MIPS familiy, currently only support for RMI XLR
 *
 * Common features
 * ---------------
 * - Per counter event selection data in subfields of control registers.
 * - Overflow interrupt support is present in all processors,
 * - The counter register available on a per-thread basis is used to
 *   to sample the TSC value  
 */

inline unsigned int read_pmc(unsigned int pmc)
{
    switch (pmc) {
        default: 
        case 0:
            return __read_32bit_c0_register($25, 1);
        case 1:
            return __read_32bit_c0_register($25, 3);
    }
}

inline void write_pmc (unsigned int pmc, unsigned int value)
{
    switch (pmc) {
        default: 
        case 0:
            __write_32bit_c0_register($25, 1, value);
            break;
        case 1:
            __write_32bit_c0_register($25, 3, value);
            break;
    }
}

inline void write_pmctrl(unsigned int pmc, unsigned int value)
{
    switch (pmc) {
        default: 
        case 0:
            __write_32bit_c0_register($25, 0, value);
            break;
        case 1:
            __write_32bit_c0_register($25, 2, value);
            break;
    }
}

// when asked to read, we will have to read only if the thread
// id in the control register corresponds to our's. What
// do we do if the performance registers are configured to
// record numbers for all the 4 threads. It would be better
// if the user dictates whethere or not to pick the values.
// provide a mechansim for the user to tell the same to the driver

static void mips_read_counters(struct perfctr_cpu_state *state,
                  struct perfctr_low_ctrs *ctrs)
{
    unsigned int cstatus, nrctrs, i;

    cstatus = state->cstatus;
    if (perfctr_cstatus_has_tsc(cstatus)) {
        ctrs->tsc = read_c0_count();
	}
    nrctrs = perfctr_cstatus_nractrs(cstatus);
    for(i = 0; i < nrctrs; ++i) {
        unsigned int pmc = state->pmc[i].map;
        ctrs->pmc[i] = read_pmc(pmc);
    }
}

// The highest index of the event that one can specify

/* static unsigned int pmc_max_event(unsigned int pmc)
{
    switch (pmc) {
    default:
    case 0:
        return 63;
    case 1:
        return 63;
    }
} */

static unsigned int get_nr_pmcs(void)
{
    switch (pm_type) {
        case MIPS_XLR:
            return 2;
        default: /* MIPS_GENERIC, but silences gcc warning */
            return 0;
    }
}

static int mips_check_control(struct perfctr_cpu_state *state)
{
    unsigned int i, nractrs, nrctrs, pmc_mask, pmi_mask, pmc;
    unsigned int nr_pmcs;

    // ensuring that the total no of performance registers that
    // are monitored are not more than available number
    nr_pmcs = get_nr_pmcs();
    nractrs = state->control.nractrs;
    nrctrs = nractrs + state->control.nrictrs;    // we could have got this from cstatus. Isn't that so?
    if ( (nrctrs < nractrs) || (nrctrs > nr_pmcs) )
        return -EINVAL;

    // ctrl_reg are the control registers, while .map contains
    // the actual register number to use while reading and writing
    pmc_mask = 0;
    pmi_mask = 0;
    // memset(ctrl_regs, 0, sizeof(ctrl_regs));

    for(i = 0; i < nrctrs; ++i) {
        pmc = state->control.pmc[i].map;    // for ppc, 0 <= pmc, the map value <= 5

        // ok, here is where the user-specified values are being copied to the
        // variables in the 'state' variable

        state->pmc[i].map = pmc;

        if (pmc >= nr_pmcs || (pmc_mask & (1<<pmc))) {
            return -EINVAL;
        }

        pmc_mask |= (1<<pmc);
        if (i >= nractrs) {
            pmi_mask |= (1<<pmc);
        }

        // IMPORTANT: check that we haven't set the interrupt-enable bit for a-mode registers

        // do some more sanity check that the user specifed valid control data
        // if ( (ctrl_reg[pmc] & MIPS_XLR_EVNTSEL_RESERVED) != 0 )
        //    return -EINVAL;

        // what more sanity checks do we need ?
    }

    state->k1.id = new_id();

    return 0;
}

#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT
/* PRE: perfctr_cstatus_has_ictrs(state->cstatus) != 0 */
/* PRE: counters frozen */

// Suspend the collection of statistics in the performance registers
// Both mips_isuspend() and mips_iresume() are for i-mode regs only

static void mips_isuspend(struct perfctr_cpu_state *state)
{
    struct per_cpu_cache *cache;
    unsigned int cstatus, nrctrs, i;
    int cpu;

    // it is on the cpu no 'cpu' that we suspended gathering statistics
    cpu = smp_processor_id();

    // what are we going to with the stored 'cpu' no? telling somone
    // look this state was last suspended on cpu 'cpu'
    set_isuspend_cpu(state, cpu); /* early to limit cpu's live range */

    // what are we caching?
    cache = __get_cpu_cache(cpu);
    cstatus = state->cstatus;
    nrctrs = perfctr_cstatus_nrctrs(cstatus);
    for(i = perfctr_cstatus_nractrs(cstatus); i < nrctrs; ++i) {
        unsigned int pmc, now;

        pmc = state->pmc[i].map;

        // instead of setting the freeze bits, just zero out the whole reg
        cache->ctrl_regs[pmc] = 0;
        write_pmctrl(pmc, cache->ctrl_regs[pmc]);

        now = read_pmc(pmc);
        state->pmc[i].sum += now - state->pmc[i].start;
        state->pmc[i].start = now;
    }
    /* cache->k1.id is still == state->k1.id */

    // sampled the i-mode registers
}

static void mips_iresume(const struct perfctr_cpu_state *state)
{
    struct per_cpu_cache *cache;
    unsigned int cstatus, nrctrs, i;
    int cpu;

    cpu = smp_processor_id();
    cache = __get_cpu_cache(cpu);

    if (cache->k1.id == state->k1.id) {
        // we need to do this and force reload of control registers
        // to unfreeze control registers
        cache->k1.id = 0; 

        // if no one else was scheduled after we were suspended,
        // the regiseters are still warm, actually hot and don't
        // have to reload them. Is that right ?

        // we are being rescheduled on the same processor
        if (is_isuspend_cpu(state, cpu))
            return; /* skip reload of PMCs */
    }

    // The CPU state wasn't ours.
    // The counters must be frozen before being reinitialised,
    // to prevent unexpected increments and missed overflows.

    // At this point, only the i-mode registers are frozen. Is there
    // any reason to freeze a-mode counters ?!

    // All unused counters must be reset to a non-overflow state.
    // accumulation mode registers are reset to zero, while the i-mode
    // registers are being written from state->pmc[i].start. The field
    // state->pmc[].start for i-mode registers was set to the values
    // specified in the .ireset field in the function ...

    cstatus = state->cstatus;
    nrctrs = perfctr_cstatus_nrctrs(cstatus);
    for(i = perfctr_cstatus_nractrs(cstatus); i < nrctrs; ++i) {
        unsigned int map = state->pmc[i].map;

        cache->ctrl_regs[map] = 0;
        write_pmctrl(map, 0); // zero value
        write_pmc(map, state->pmc[i].start);
    }
    // cache->k1.id remains != state->k1.id
}
#endif

// this is invoked by the _resume() routine. If the id in the cache
// equals to our id, it implies no one else touched the control
// registers of this id and hence need not be written all over again
// however this is not a good idea when control registers have to
// be written to for some reason, such as when the control registers
// have to unfrozen and so on. This is done by settind the id in the
// cache to 0. The cache id is updated with the id of the verpfctr
// id when a new state is schduled to collect statistics on this cpu

int perfctr_cntmode = 0;

static void mips_write_control(const struct perfctr_cpu_state *state)
{
    struct per_cpu_cache *cache;
    unsigned int nrctrs, i;

    // cache stores the information pertaining to one id. Under
    // what conditions does that cache state remain intact? Can some
    // processes tell that their statistics be not recorded. In such
    // a case when a thread is rescheuldes on the same processpor
    // without the intervening thread recording the statistics, then
    // the cache will be hot

    cache = get_cpu_cache();
    if (cache->k1.id == state->k1.id) {
        return;
    }
    nrctrs = perfctr_cstatus_nrctrs(state->cstatus);

	preempt_disable();
    for (i = 0; i < nrctrs; ++i) {
        unsigned int ctrl_reg = state->control.pmc[i].ctrl_reg;
        unsigned int pmc = state->pmc[i].map;    // assuming that the 'state' values have been
                                                 // updated from control values specified by users
        if (ctrl_reg != cache->ctrl_regs[pmc]) {
			if (!perfctr_cntmode) {
				MIPS_XLR_UNSET_CNT_ALL_THREADS(ctrl_reg);
				MIPS_XLR_SET_THREADID(ctrl_reg, netlogic_thr_id());
			}
			else {
				MIPS_XLR_SET_CNT_ALL_THREADS(ctrl_reg);
			}
            cache->ctrl_regs[pmc] = ctrl_reg;
            write_pmctrl(pmc, ctrl_reg);
        }
    }
    cache->k1.id = state->k1.id;
	preempt_enable();
}

static void mips_clear_counters(void)
{
    switch (pm_type) {
        case MIPS_XLR:
            __write_32bit_c0_register($25, 0, 0);
            __write_32bit_c0_register($25, 1, 0);
            __write_32bit_c0_register($25, 2, 0);
            __write_32bit_c0_register($25, 3, 0);
        case MIPS_GENERIC:
            ;
    }
}

// Driver methods, internal and exported.

static inline void perfctr_cpu_write_control(const struct perfctr_cpu_state *state)
{
    return mips_write_control(state);
}

static inline void perfctr_cpu_read_counters(struct perfctr_cpu_state *state,
                      struct perfctr_low_ctrs *ctrs)
{
    return mips_read_counters(state, ctrs);
}

#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT
static inline void perfctr_cpu_isuspend(struct perfctr_cpu_state *state)
{
    return mips_isuspend(state);
}

static inline void perfctr_cpu_iresume(const struct perfctr_cpu_state *state)
{
    return mips_iresume(state);
}

/* Call perfctr_cpu_ireload() just before perfctr_cpu_resume() to
   bypass internal caching and force a reload if the I-mode PMCs. */
void perfctr_cpu_ireload(struct perfctr_cpu_state *state)
{
#ifdef CONFIG_SMP
    clear_isuspend_cpu(state);
#else
    get_cpu_cache()->k1.id = 0;
#endif
}

/* PRE: the counters have been suspended and sampled by perfctr_cpu_suspend() */
// the following overflow check is being done only for the i-mode registers
// how is the overflow of a-mode registers handled ?
inline unsigned int perfctr_cpu_identify_overflow(struct perfctr_cpu_state *state)
{
    unsigned int cstatus, nrctrs, i, pmc_mask;

    cstatus = state->cstatus;
    i = perfctr_cstatus_nractrs(cstatus);    // a-mode count
    nrctrs = perfctr_cstatus_nrctrs(cstatus);

    for(pmc_mask = 0; i < nrctrs; ++i) {

        // Ok, reset the overflown i-mode counters

        if ((int)state->pmc[i].start < 0) { /* MIPS-specific */
            /* XXX: "+=" to correct for overshots */
            state->pmc[i].start = state->control.pmc[i].ireset;
            pmc_mask |= (1 << i);
        }
    }
    return pmc_mask;
}

static inline int check_ireset(const struct perfctr_cpu_state *state)
{
    unsigned int nrctrs, i;

    i = state->control.nractrs;
    nrctrs = i + state->control.nrictrs;
    for(; i < nrctrs; ++i) {
        if (state->control.pmc[i].ireset < 0)    /* MIPS-specific */ {
            return -EINVAL;
        }
    }
    return 0;
}

// the start values have to be reset as we might have changed then in
// _isuspend()
static inline void setup_imode_start_values(struct perfctr_cpu_state *state)
{
    unsigned int cstatus, nrctrs, i;

    cstatus = state->cstatus;
    nrctrs = perfctr_cstatus_nrctrs(cstatus);
    for(i = perfctr_cstatus_nractrs(cstatus); i < nrctrs; ++i)
        state->pmc[i].start = state->control.pmc[i].ireset;
}

#else    /* CONFIG_PERFCTR_INTERRUPT_SUPPORT */
static inline void perfctr_cpu_isuspend(struct perfctr_cpu_state *state) { }
static inline void perfctr_cpu_iresume(const struct perfctr_cpu_state *state) { }
static inline int check_ireset(const struct perfctr_cpu_state *state) { return 0; }
static inline void setup_imode_start_values(struct perfctr_cpu_state *state) { }
#endif    /* CONFIG_PERFCTR_INTERRUPT_SUPPORT */

static int check_control(struct perfctr_cpu_state *state)
{
    return mips_check_control(state);
}

int perfctr_cpu_update_control(struct perfctr_cpu_state *state, int is_global)
{
    int err;

    // since we updated the control, we invalidate the cpu id in the state
    // so that we can force reload of control registers
    clear_isuspend_cpu(state);
    state->cstatus = 0;

    /* disallow i-mode counters if we cannot catch the interrupts */
    if (!(perfctr_info.cpu_features & PERFCTR_FEATURE_PCINT)
        && state->control.nrictrs) {
        
        return -EPERM;
    }

    err = check_ireset(state);
    if (err < 0) {
        return err;
    }
    err = check_control(state); /* may initialise state->cstatus */
    if (err < 0) {
        return err;
    }

    // Ok, while the map values and the start values for i-mode counters
    // are updated in the above function check_control() and the following
    // function setup_imode_start_values(), the 'cstatus' values is set here

    // how do u ensure that all i-mode registers are specified beyond a-mode
    // registers
    state->cstatus |= perfctr_mk_cstatus(state->control.tsc_on,
                         state->control.nractrs,
                         state->control.nrictrs);
    setup_imode_start_values(state);
    return 0;
}

inline void perfctr_cpu_suspend(struct perfctr_cpu_state *state)
{
    unsigned int i, cstatus, nractrs;
    struct perfctr_low_ctrs now;
	int   cpu_id;

	cpu_id = hard_smp_processor_id() / 4;
	spin_lock (&pmc_resource[cpu_id].lock);
	if ( pmc_resource[cpu_id].current_thread != netlogic_thr_id() ) {
		// printk (KERN_INFO "PMCounters do not belong to this process[%d]\n", current->pid);
		spin_unlock (&pmc_resource[cpu_id].lock);
		return;
	}
	pmc_resource[cpu_id].current_thread = -1;
	spin_unlock (&pmc_resource[cpu_id].lock);

    // To prevent polluting the numbers, can we freeze the counters
    // here, as early as possible ?

    if (perfctr_cstatus_has_ictrs(state->cstatus)) {
        perfctr_cpu_isuspend(state);
	}
    perfctr_cpu_read_counters(state, &now);
    cstatus = state->cstatus;
    if (perfctr_cstatus_has_tsc(cstatus)) {
        state->tsc_sum += now.tsc - state->tsc_start;
    }
    nractrs = perfctr_cstatus_nractrs(cstatus);
    for(i = 0; i < nractrs; ++i) {
        state->pmc[i].sum += now.pmc[i] - state->pmc[i].start;
    }
}

inline void perfctr_cpu_resume(struct perfctr_cpu_state *state)
{
	int   cpu_id;

	cpu_id = hard_smp_processor_id() / 4;
	spin_lock (&pmc_resource[cpu_id].lock);
	if ( pmc_resource[cpu_id].current_thread != -1 ) {
		// printk (KERN_INFO "PMCounters unavailable for process %d\n", current->pid);
		spin_unlock (&pmc_resource[cpu_id].lock);
		return;
	}
	pmc_resource[cpu_id].current_thread = netlogic_thr_id();
	spin_unlock (&pmc_resource[cpu_id].lock);

    if (perfctr_cstatus_has_ictrs(state->cstatus)) {
        perfctr_cpu_iresume(state);
	}

    // the counters are triggered, having been frozen in _iresume()
    // that preceded this point. So, the model is to trigger the
    // registere to collect the numbers and record the start state
    // that completes the 'resume' process.

    perfctr_cpu_write_control(state);
    {
        struct perfctr_low_ctrs now;
        unsigned int i, cstatus, nrctrs;
        perfctr_cpu_read_counters(state, &now);
        cstatus = state->cstatus;

        // the start state of the registers has to be recorded only
        // in resume() and that is what is being done.

        if (perfctr_cstatus_has_tsc(cstatus)) {
            state->tsc_start = now.tsc;
		}
        nrctrs = perfctr_cstatus_nractrs(cstatus);
        for (i = 0; i < nrctrs; ++i) {
            state->pmc[i].start = now.pmc[i];
		}
    }
    /* XXX: if (SMP && start.tsc == now.tsc) ++now.tsc; */
}

// Sampling only a-mode registers
void perfctr_cpu_sample(struct perfctr_cpu_state *state)
{
    unsigned int i, cstatus, nractrs;
    struct perfctr_low_ctrs now;
	int   cpu_id;

	cpu_id = hard_smp_processor_id() / 4;
	spin_lock (&pmc_resource[cpu_id].lock);
	if ( pmc_resource[cpu_id].current_thread != netlogic_thr_id() ) {
		// printk (KERN_INFO "PMCounters do not belong to this process[%d]\n", current->pid);
		spin_unlock (&pmc_resource[cpu_id].lock);
		return;
	}
	spin_unlock (&pmc_resource[cpu_id].lock);

    perfctr_cpu_read_counters(state, &now);        // reads only a-mode registers
    cstatus = state->cstatus;
    if (perfctr_cstatus_has_tsc(cstatus)) {
        state->tsc_sum += now.tsc - state->tsc_start;
        // one needs to update the start status as we continue to gather
        // statistics without interruption
        state->tsc_start = now.tsc;
    }
    nractrs = perfctr_cstatus_nractrs(cstatus);
    for(i = 0; i < nractrs; ++i) {
		
        state->pmc[i].sum += now.pmc[i] - state->pmc[i].start;
        state->pmc[i].start = now.pmc[i];
    }
}

static void perfctr_cpu_clear_counters(void)
{
    struct per_cpu_cache *cache;

    cache = get_cpu_cache();
    memset(cache, 0, sizeof *cache);
    cache->k1.id = -1;

    mips_clear_counters();
}

/****************************************************************
 *                                                            *
 * Processor detection and initialisation procedures.        *
 *                                                            *
 ****************************************************************/

cpumask_t perfctr_cpus_forbidden_mask;

static inline void clear_perfctr_cpus_forbidden_mask(void)
{
#if !defined(perfctr_cpus_forbidden_mask)
    cpus_clear(perfctr_cpus_forbidden_mask);
#endif
}

static inline void set_perfctr_cpus_forbidden_mask(cpumask_t mask)
{
#if !defined(perfctr_cpus_forbidden_mask)
    perfctr_cpus_forbidden_mask = mask;
#endif
}

static void __init mips_setup_cpu_mask(void *forbidden)
{
    unsigned int logical_processor_id = hard_smp_processor_id();
    if ((logical_processor_id % 4) != 0) {
        // We rely on cpu_set() being atomic!
        cpu_set(logical_processor_id, *(cpumask_t*)forbidden);
    }
}

static int __init mips_smp_init(void)
{
    cpumask_t forbidden;
    unsigned int cpu;

    cpus_clear(forbidden);
#ifdef CONFIG_SMP
    smp_call_function(mips_setup_cpu_mask, &forbidden, 1);
#endif
    mips_setup_cpu_mask(&forbidden);
    if (cpus_empty(forbidden))
        return 0;
    perfctr_cpus_forbidden_mask = forbidden;
    for(cpu = 0; cpu < NR_CPUS; ++cpu)
        if (cpu_isset(cpu, forbidden))
            printk(" %u", cpu);
        printk("\n");
    return 0;
}

static void perfctr_cpu_clear_one(void *ignore)
{
    // cache too is also getting invalidated in the following routine
    perfctr_cpu_clear_counters();
}

static int init_done;

int __init perfctr_cpu_init(void)
{
    int i, err = 0;

    preempt_disable();
    
    mips_smp_init();

    // The  first two fields of the structure 'perfctr_info' are defined/assigned
    // in the declaration itself
    perfctr_info.cpu_type = pm_type = MIPS_XLR;
    perfctr_info.cpu_features |= PERFCTR_FEATURE_PCINT;
    perfctr_info.cpu_khz = mips_hpt_frequency;
    perfctr_info.tsc_to_cpu_mult = 1;

    /* for 2.7 versions only. This variable is not visible to users. */
    perfctr_cpu_name = "MIPS_XLR";

    perfctr_mips_init_tests();

	// Init Spinlocks and the current thead of PMC registers
	for (i = 0; i < 8; ++i) {
		spin_lock_init(&pmc_resource[i].lock);
		pmc_resource[i].current_thread = -1;
	}

	init_done = 1;
    preempt_enable();
    return err;
}

void __exit perfctr_cpu_exit(void)
{
}

/****************************************************************
 *                                                                *
 * Hardware reservation.                                        *
 *                                                                *
 ****************************************************************/

static DECLARE_MUTEX(mutex);
static const char *current_service = 0;

const char *perfctr_cpu_reserve(const char *service)
{
    const char *ret;
	int i;

    if (!init_done)
        return "unsupported hardware";
    down(&mutex);
    ret = current_service;
    if (ret)
        goto out_up;
    current_service = service;
    on_each_cpu(perfctr_cpu_clear_one, NULL, 1);

	// Ideally at this point of time, all the current thread values of
	// pmc_resource must be -1
	for (i = 0; i < 8; ++i) {
		if (pmc_resource[i].current_thread != -1) {
			printk (KERN_INFO "pmc_resource[%d].current_thread != -1\n", i);
		}
		pmc_resource[i].current_thread = -1;
	}

    perfctr_cpu_set_ihandler(NULL);
    ret = NULL;
out_up:
    up(&mutex);
    return ret;
}

void perfctr_cpu_release(const char *service)
{
	int i;

    down(&mutex);
    if (service != current_service) {
        printk(KERN_ERR "%s: attempt by %s to release while reserved by %s\n", __FUNCTION__, service, current_service);
        goto out_up;
    } else {
        /* power down the counters */
        on_each_cpu(perfctr_cpu_clear_one, NULL, 1);
        perfctr_cpu_set_ihandler(NULL);
        current_service = 0;

		// Ideally at this point of time, all the current thread values of
		// pmc_resource must be -1
		for (i = 0; i < 8; ++i) {
			if (pmc_resource[i].current_thread != -1) {
				printk (KERN_INFO "pmc_resource[%d].current_thread != -1\n", i);
			}
			pmc_resource[i].current_thread = -1;
		}
    }
out_up:
    up(&mutex);
}
