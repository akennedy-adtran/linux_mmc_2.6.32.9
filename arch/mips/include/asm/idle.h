/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#ifndef _ASM_MIPS_IDLE_H
#define _ASM_MIPS_IDLE_H

#define IDLE_START 1
#define IDLE_END 2

struct notifier_block;
extern void idle_notifier_register(struct notifier_block *n);
extern void idle_notifier_unregister(struct notifier_block *n);

static inline void enter_idle(void) { }
static inline void exit_idle(void) { }

#endif /* _ASM_MIPS_IDLE_H */
