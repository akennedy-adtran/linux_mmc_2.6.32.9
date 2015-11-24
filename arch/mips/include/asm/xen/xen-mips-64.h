/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/******************************************************************************
 * xen-mips.h
 * 
 * Guest OS interface to x86 64-bit Xen.
 * 
 */

#ifndef __XEN_PUBLIC_ARCH_MIPS_XEN_MIPS_64_H__
#define __XEN_PUBLIC_ARCH_MIPS_XEN_MIPS_64_H__

/*
 * Hypercall interface:
 *  Input:  %rdi, %rsi, %rdx, %r10, %r8 (arguments 1-5)
 *  Output: %rax
 * Access is via hypercall page (set up by guest loader or via a Xen MSR):
 *  call hypercall_page + hypercall-number * 32
 * Clobbered: argument registers (e.g., 2-arg hypercall clobbers %rdi,%rsi)
 */

#if __XEN_INTERFACE_VERSION__ < 0x00030203
/*
 * Legacy hypercall interface:
 * As above, except the entry sequence to the hypervisor is:
 *  mov $hypercall-number*32,%eax ; syscall
 * Clobbered: %rcx, %r11, argument registers (as above)
 */
#define TRAP_INSTR "syscall"
#endif

/*
 * 64-bit segment selectors
 * These flat segments are in the Xen-private section of every GDT. Since these
 * are also present in the initial GDT, many OSes will be able to avoid
 * installing their own GDT.
 */

/*
 * others related to the following are
 * defined in asm/addrspace.h
 */

#define __HYPERVISOR_VIRT_START 0xFFFFFFFFC0000000
#define __HYPERVISOR_VIRT_END   0xFFFFFFFFD0000000
#define __MACH2PHYS_VIRT_START  0xFFFF800000000000
#define __MACH2PHYS_VIRT_END    0xFFFF804000000000

#ifndef HYPERVISOR_VIRT_START
#define HYPERVISOR_VIRT_START mk_unsigned_long(__HYPERVISOR_VIRT_START)
#define HYPERVISOR_VIRT_END   mk_unsigned_long(__HYPERVISOR_VIRT_END)
#endif

#define MACH2PHYS_VIRT_START  mk_unsigned_long(__MACH2PHYS_VIRT_START)
#define MACH2PHYS_VIRT_END    mk_unsigned_long(__MACH2PHYS_VIRT_END)
#define MACH2PHYS_NR_ENTRIES  ((MACH2PHYS_VIRT_END-MACH2PHYS_VIRT_START)>>3)
#ifndef machine_to_phys_mapping
#define machine_to_phys_mapping ((unsigned long *)HYPERVISOR_VIRT_START)
#endif

#ifndef __ASSEMBLY__

struct iret_context {
    /* Top of stack (%rsp at point of hypercall). */
//    uint64_t rax, r11, rcx, flags, rip, cs, rflags, rsp, ss;
    /* Bottom of iret stack frame. */
};

#if defined(__GNUC__) && !defined(__STRICT_ANSI__)
/* Anonymous union includes both 32- and 64-bit names (e.g., eax/rax). */
#define __DECL_REG(name) union { \
    uint64_t r ## name, e ## name; \
    uint32_t _e ## name; \
}
#else
/* Non-gcc sources must always use the proper 64-bit name (e.g., rax). */
#define __DECL_REG(name) uint64_t r ## name
#endif

#undef __DECL_REG

struct arch_vcpu_info {
    unsigned long cr2;
    unsigned long pad; /* sizeof(vcpu_info_t) == 64 */
};
typedef struct arch_vcpu_info arch_vcpu_info_t;

#endif /* !__ASSEMBLY__ */

#define XENCOMM_INLINE_MASK 0xf800000000000000UL
#define XENCOMM_INLINE_FLAG 0x8000000000000000UL

#endif /* __XEN_PUBLIC_ARCH_MIPS_XEN_MIPS_64_H__ */
