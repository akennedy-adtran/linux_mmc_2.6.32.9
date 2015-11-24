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

#ifndef __NLM_RW_LOCK_H_
#define __NLM_RW_LOCK_H_

#define NLM_MAX_CPUS 32
typedef struct {
	volatile unsigned int lock;
	unsigned int read_cpus[NLM_MAX_CPUS]; /* cpus that hold rd lock */
	int write_cpu; /* CPU that is currently holding wr lock */
} nlm_rwlock_t;

#define nlm_sync() __asm__ __volatile__("sync": : :"memory")

__asm__ (
		".macro\tnlm_local_irq_save result\n\t"
		".set\tpush\n\t"
		".set\treorder\n\t"
		".set\tnoat\n\t"
		"mfc0\t\\result, $12\n\t"
		"ori\t$1, \\result, 1\n\t"
		"xori\t$1, 1\n\t"
		".set\tnoreorder\n\t"
		"mtc0\t$1, $12\n\t"
		"sll\t$0, $0, 1\t\t\t# nop\n\t"
		"sll\t$0, $0, 1\t\t\t# nop\n\t"
		"sll\t$0, $0, 1\t\t\t# nop\n\t"
		".set\tpop\n\t"
		".endm");

__asm__(".macro\tnlm_local_irq_restore flags\n\t"
		".set\tnoreorder\n\t"
		".set\tnoat\n\t"
		"mfc0\t$1, $12\n\t"
		"andi\t\\flags, 1\n\t"
		"ori\t$1, 1\n\t"
		"xori\t$1, 1\n\t"
		"or\t\\flags, $1\n\t"
		"mtc0\t\\flags, $12\n\t"
		"sll\t$0, $0, 1\t\t\t# nop\n\t"
		"sll\t$0, $0, 1\t\t\t# nop\n\t"
		"sll\t$0, $0, 1\t\t\t# nop\n\t"
		".set\tat\n\t"
		".set\treorder\n\t"
		".endm");


#define nlm_local_irq_save(x)		\
	__asm__ __volatile__(           \
		"nlm_local_irq_save\t%0"                                    \
		: "=r" (x)		\
		: /* no inputs */	\
		: "memory")


#define nlm_local_irq_restore(flags)		\
	do {                           		\
		unsigned long __tmp1;  		\
		__asm__ __volatile__(  		\
		"nlm_local_irq_restore\t%0"        	\
		: "=r" (__tmp1)                 \
		: "0" (flags)                   \
		: "memory");                    \
	} while(0)


#define nlm_processor_id() 				\
	({ int __res;                                   \
	 __asm__ __volatile__(                          \
		 ".set\tmips32\n\t"                     \
		 "mfc0\t%0, $15, 1\n\t"           	\
		 "andi\t%0, 0x1f\n\t"			\
		 ".set\tmips0\n\t"                      \
		 : "=r" (__res));                       \
	 __res;                                         \
	 })


static inline unsigned int nlm_read_lock_irq_save(nlm_rwlock_t *rw)
{
	unsigned int temp;
	unsigned int cpu;
	unsigned int flags;

	nlm_local_irq_save(flags);
	cpu = nlm_processor_id();

		__asm__ __volatile__(
		"	.set	noreorder	\n"
		"1:	ll	%1, %2		\n"
		"	bltz	%1, 2f		\n"
		"	 addu	%1, 1		\n"
		"	sc	%1, %0		\n"
		"	beqz	%1, 1b		\n"
		"	 nop			\n"
		"	.subsection 2		\n"
		"2:	ll	%1, %2		\n"
		"	bltz	%1, 2b		\n"
		"	 addu	%1, 1		\n"
		"	b	1b		\n"
		"	 nop			\n"
		"	.previous		\n"
		"	.set	reorder		\n"
		: "=m" (rw->lock), "=&r" (temp)
		: "m" (rw->lock)
		: "memory");

		rw->read_cpus[cpu] = 1;
		nlm_sync();

		return flags;
}

static inline void nlm_read_unlock_irq_restore(nlm_rwlock_t *rw, 
			unsigned int flags)
{
	unsigned int temp;
	unsigned int cpu;

	cpu = nlm_processor_id();

	nlm_sync();
	__asm__ __volatile__(
		"       .set    noreorder       			\n"
		"1:     ll      %1, %2                                  \n"
		"       sub     %1, 1                                   \n"
		"       sc      %1, %0                                  \n"
		"       beqz    %1, 2f                                  \n"
		"        nop                                            \n"
		"       .subsection 2                                   \n"
		"2:     b       1b                                      \n"
		"        nop                                            \n"
		"       .previous                                       \n"
		"       .set    reorder                                 \n"
		: "=m" (rw->lock), "=&r" (temp)
		: "m" (rw->lock)
		: "memory");
	rw->read_cpus[cpu] = 0;
	nlm_local_irq_restore(flags);

}

static inline unsigned int nlm_write_lock_irq_save(nlm_rwlock_t *rw)
{
	unsigned int temp;
	unsigned int cpu;
	unsigned int flags;

	nlm_local_irq_save(flags);
	cpu = nlm_processor_id();

	__asm__ __volatile__(
		"       .set    noreorder       			\n"
		"1:     ll      %1, %2                                  \n"
		"       bnez    %1, 2f                                  \n"
		"        lui    %1, 0x8000                              \n"
		"       sc      %1, %0                                  \n"
		"       beqz    %1, 2f                                  \n"
		"        nop                                            \n"
		"       .subsection 2                                   \n"
		"2:     ll      %1, %2                                  \n"
		"       bnez    %1, 2b                                  \n"
		"        lui    %1, 0x8000                              \n"
		"       b       1b                                      \n"
		"        nop                                            \n"
		"       .previous                                       \n"
		"       .set    reorder                                 \n"
		: "=m" (rw->lock), "=&r" (temp)
		: "m" (rw->lock)
		: "memory");

	rw->write_cpu = cpu;
	nlm_sync();
	
	return flags;

}


static inline void nlm_write_unlock_irq_restore(nlm_rwlock_t *rw, 
						unsigned int flags)
{
	nlm_sync();

	__asm__ __volatile__(
		"       sw      $0, %0                                  \n"
		: "=m" (rw->lock)
		: "m" (rw->lock)
		: "memory");
	rw->write_cpu = -1;
	nlm_local_irq_restore(flags);
}


#endif
