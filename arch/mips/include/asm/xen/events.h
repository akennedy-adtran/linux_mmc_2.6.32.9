#ifndef _ASM_MIPS_XEN_EVENTS_H
#define _ASM_MIPS_XEN_EVENTS_H

#include <asm/ptrace.h>
#include <asm/irqflags.h>

enum ipi_vector {
	XEN_RESCHEDULE_VECTOR,
	XEN_CALL_FUNCTION_VECTOR,
	XEN_CALL_FUNCTION_SINGLE_VECTOR,
	XEN_SPIN_UNLOCK_VECTOR,

	XEN_NR_IPIS,
};

static inline int xen_irqs_disabled(struct pt_regs *regs)
{
#ifdef NETL_USE_X86_XEN_PORT
	return raw_irqs_disabled_flags(regs->flags);
#else
	return raw_irqs_disabled_flags(regs->cp0_status);
#endif
}

#endif /* _ASM_MIPS_XEN_EVENTS_H */
