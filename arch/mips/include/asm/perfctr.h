/*-
 * Copyright 2005-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: perfctr.h,v 1.1.2.5 2006-09-28 01:24:19 nphilips Exp $
 * MIPS_XLR Performance-Monitoring Counters driver
 *
 * Copyright (C) 2005  Prasad Boddupalli
 */
#ifndef _ASM_MIPS_PERFCTR_H
#define _ASM_MIPS_PERFCTR_H

#define PERFCTR_PAGE_SIZE 4096

/* perfctr_info.cpu_type values */
#define MIPS_GENERIC	0
#define MIPS_XLR        1

// Architecture-specific container for counter values.
// Used in the kernel/user API, but not by low-level drivers.
struct perfctr_sum_ctrs {
	unsigned long long tsc;
	unsigned long long pmc[2];	// just two perf registers
};

// architecture-specific container for control data.
// used in both kernel/user API and by low-level drivers
struct perfctr_cpu_control {
	unsigned int tsc_on;		// initialize on perfctr_cpu_init()?
	unsigned int nractrs;		/* # of a-mode counters (max(0,2)?) */
	unsigned int nrictrs;		/* # of i-mode counters */

    // We already have the control registers in the eventsel field
	// of the struct {} pmc[]. We might not need the following
	// registers 

	/* struct {
		unsigned int perfctrl0;
		unsigned int perfctrl1;
	} mips; */

	unsigned int _reserved1;
	unsigned int _reserved2;
	unsigned int _reserved3;
	unsigned int _reserved4;

	struct {
		unsigned int map;	/* physical counter to use */
		unsigned int ctrl_reg;
		int ireset;		/* [0,0x7fffffff], for i-mode counters */
	} pmc[2];
};

struct perfctr_cpu_state {
	unsigned int cstatus;

	// k1 is opaque in the user ABI
	struct {
		unsigned int id;
		int isuspend_cpu;
	} k1;

	// tsc fields must be inlined. Placing them in a sub-struct might 
	// cause unwanted internal padding
	unsigned int tsc_start;
	unsigned long long tsc_sum;

	struct {
		unsigned int map;
		unsigned int start;
		unsigned long long sum;
	} pmc[2];

#ifdef __KERNEL__
	struct perfctr_cpu_control control;
#endif
};

/* cstatus is a re-encoding of control.tsc_on/nractrs/nrictrs
   which should have less overhead in most cases */
/* XXX: mips driver internally also uses cstatus&(1<<30) */

// construct a cstatus value. 
static inline
unsigned int perfctr_mk_cstatus(unsigned int tsc_on, unsigned int nractrs,
				unsigned int nrictrs)
{
	return ((tsc_on<<31) | (nrictrs<<16) | ((nractrs+nrictrs)<<8) | nractrs);
}

// check if any part (tsc_on, nractrs, nrictrs) of the cstatus is non-zero
static inline unsigned int perfctr_cstatus_enabled(unsigned int cstatus)
{
	return cstatus;
}

// check if the tsc_on part of the cstatus is non-zero
static inline int perfctr_cstatus_has_tsc(unsigned int cstatus)
{
	return ((int)cstatus < 0);	/* test and jump on sign */
}

// retrieve nractrs field
static inline unsigned int perfctr_cstatus_nractrs(unsigned int cstatus)
{
	return (cstatus & 0x7F);		/* and with imm8 */
}

// retrieve nractrs+nrictrs from the cstatus
static inline unsigned int perfctr_cstatus_nrctrs(unsigned int cstatus)
{
	return ((cstatus >> 8) & 0x7F);
}

// check if the nrictrs part of cstatus is non-zero
static inline unsigned int perfctr_cstatus_has_ictrs(unsigned int cstatus)
{
	return (cstatus & (0x7F << 16));
}

/*
 * 'struct siginfo' support for perfctr overflow signals.
 * In unbuffered mode, si_code is set to SI_PMC_OVF and a bitmask
 * describing which perfctrs overflowed is put in si_pmc_ovf_mask.
 * A bitmask is used since more than one perfctr can have overflowed
 * by the time the interrupt handler runs.
 *
 * glibc's <signal.h> doesn't seem to define __SI_FAULT or __SI_CODE(),
 * and including <asm/siginfo.h> as well may cause redefinition errors,
 * so the user and kernel values are different #defines here.
 */
#ifdef __KERNEL__
#define SI_PMC_OVF	(__SI_FAULT|'P')
#else
#define SI_PMC_OVF	('P')
#endif
#define si_pmc_ovf_mask	_sifields._pad[0] /* XXX: use an unsigned field later */

/* version number for user-visible CPU-specific data */
#define PERFCTR_CPU_VERSION	0	/* XXX: not yet cast in stone */

#ifdef __KERNEL__

#ifdef CONFIG_PERFCTR

/* Driver init/exit. */
extern int perfctr_cpu_init(void);
extern void perfctr_cpu_exit(void);

/* CPU type name. */
extern char *perfctr_cpu_name;

/* Hardware reservation. */
extern const char *perfctr_cpu_reserve(const char *service);
extern void perfctr_cpu_release(const char *service);

/* PRE: state has no running interrupt-mode counters.
   Check that the new control data is valid.
   Update the driver's private control data.
   Returns a negative error code if the control data is invalid.
*/
extern int perfctr_cpu_update_control(struct perfctr_cpu_state *state, int is_global);


/* Read a-mode counters. Subtract from start and accumulate into sums.
   Must be called with preemption disabled. */
extern void perfctr_cpu_suspend(struct perfctr_cpu_state *state);

/* Write control registers. Read a-mode counters into start.
   Must be called with preemption disabled. */
extern void perfctr_cpu_resume(struct perfctr_cpu_state *state);

/* Perform an efficient combined suspend/resume operation.
   Must be called with preemption disabled. */
extern void perfctr_cpu_sample(struct perfctr_cpu_state *state);

/* The type of a perfctr overflow interrupt handler.
   It will be called in IRQ context, with preemption disabled. */
typedef void (*perfctr_ihandler_t)(unsigned long pc);

unsigned int read_pmc (unsigned int);
void write_pmc (unsigned int, unsigned int);
void write_pmctrl (unsigned int, unsigned int);

/* Operations related to overflow interrupt handling. */

#ifdef CONFIG_PERFCTR_INTERRUPT_SUPPORT
extern void perfctr_cpu_set_ihandler(perfctr_ihandler_t);
extern void perfctr_cpu_ireload(struct perfctr_cpu_state*);
extern unsigned int perfctr_cpu_identify_overflow(struct perfctr_cpu_state*);
#else
static inline void perfctr_cpu_set_ihandler(perfctr_ihandler_t x) { }
#endif

static inline int perfctr_cpu_has_pending_interrupt(const struct perfctr_cpu_state *state)
{
	return 0;
}

extern perfctr_ihandler_t perfctr_ihandler;

#endif	/* CONFIG_PERFCTR */

#endif	/* __KERNEL__ */

#define MIPS_XLR_DOM_KERNEL		0x2
#define MIPS_XLR_DOM_SUP		0x4
#define MIPS_XLR_DOM_USR		0x8

#define MIPS_XLR_SET_EVNTCNT_MODE(x, mode)		x |= mode
#define MIPS_XLR_UNSET_EVNTCNT_MODE(x, mode)	x &= ~mode

#define MIPS_XLR_OVF_BIT                0x10
#define MIPS_XLR_OVF_PMI_EABLE(x)		x |= MIPS_XLR_OVF_BIT
#define MIPS_XLR_OVF_PMI_DABLE(x)		x &= ~MIPS_XLR_OVF_BIT
#define MIPS_XLR_IS_OVF_PMI(x)			(x & MIPS_XLR_OVF_BIT)

#define MIPS_XLR_EVNTSEL_MASK			0x3f
#define MIPS_XLR_EVNTSEL_SHIFT			5
#define MIPS_XLR_SET_EVNT(x, event)		\
		x |= ((event & MIPS_XLR_EVNTSEL_MASK) << MIPS_XLR_EVNTSEL_SHIFT)
#define MIPS_XLR_GET_EVNT(x)	\
		((x >> MIPS_XLR_EVNTSEL_SHIFT) & MIPS_XLR_EVNTSEL_MASK)

#define MIPS_XLR_THREADID_MASK				0x03
#define MIPS_XLR_THREADID_SHIFT				11
#define MIPS_XLR_SET_THREADID(x, tid)		\
		x |= ((tid & MIPS_XLR_THREADID_MASK) << MIPS_XLR_THREADID_SHIFT)

#define MIPS_XLR_SET_CNT_ALL_THREADS(x)		x |= 0x2000
#define MIPS_XLR_UNSET_CNT_ALL_THREADS(x)	x &= ~0x2000

#endif	/* _ASM_MIPS_PERFCTR_H */
