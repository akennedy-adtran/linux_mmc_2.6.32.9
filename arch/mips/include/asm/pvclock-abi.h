/*
 * same structure to x86's
 * Hopefully asm-x86/pvclock-abi.h would be mo
 * For now, define same duplicated definitions
 */

#ifndef _ASM_MIPS__PVCLOCK_ABI_H
#define _ASM_MIPS__PVCLOCK_ABI_H

#ifndef __ASSEMBLY__

/*
 * These structs MUST NOT be changed.
 * They are the ABI between hypervisor and gue
 * Both Xen and KVM are using this.
 *
 * pvclock_vcpu_time_info holds the system tim
 * of the last update. So the guest can use th
 * more precise system time.  There is one per
 *
 * pvclock_wall_clock references the point in
 * time was zero (usually boot time), thus the
 * current wall clock by adding the system tim
 *
 * Protocol for the "version" fields is: hyper
 * it uneven) before it starts updating the fi
 * (making it even) when it is done.  Thus the
 * time values it got are consistent by checki
 * and after reading them.
 */

struct pvclock_vcpu_time_info {
	u32   version;
	u32   pad0;
	u64   tsc_timestamp;
	u64   system_time;
	u32   tsc_to_system_mul;
	s8    tsc_shift;
	u8    pad[3];
} __attribute__((__packed__)); /* 32 bytes */

struct pvclock_wall_clock {
	u32   version;
	u32   sec;
	u32   nsec;
} __attribute__((__packed__));

#endif /* __ASSEMBLY__ */

#endif /* _ASM_MIPS__PVCLOCK_ABI_H */
