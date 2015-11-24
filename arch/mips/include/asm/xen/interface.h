/******************************************************************************
 * arch-mips/xen.h
 * 
 * Guest OS interface to mips Xen.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Copyright (c) 2004-2006, K A Fraser
 */

#ifndef __XEN_PUBLIC_ARCH_MIPS_XEN_H__
#define __XEN_PUBLIC_ARCH_MIPS_XEN_H__

#define __XEN_LATEST_INTERFACE_VERSION__ 0x00030209

#define __XEN_INTERFACE_VERSION__ __XEN_LATEST_INTERFACE_VERSION__

#include <linux/threads.h>
#include <asm/sgidefs.h>

#if __XEN_INTERFACE_VERSION__ >= 0x00030201
#define __DEFINE_GUEST_HANDLE(name, type) \
    typedef struct { type *p; } __guest_handle_ ## name
#else
#define __DEFINE_GUEST_HANDLE(name, type) \
    typedef type * __guest_handle_ ## name
#endif

#define DEFINE_GUEST_HANDLE_STRUCT(name) \
        __DEFINE_GUEST_HANDLE(name, struct name)
#define DEFINE_GUEST_HANDLE(name) __DEFINE_GUEST_HANDLE(name, name)
#define GUEST_HANDLE(name)        __guest_handle_ ## name

#define set_xen_guest_handle(hnd, val)  do { (hnd).p = val; } while (0)
#ifdef __XEN_TOOLS__
#define get_xen_guest_handle(val, hnd)  do { val = (hnd).p; } while (0)
#endif

#if _MIPS_SIM == _MIPS_SIM_ABI32
#include <asm/xen/xen-mips-32.h>
#else
#include <asm/xen/xen-mips-64.h>
#endif

#ifndef __ASSEMBLY__
typedef unsigned long xen_pfn_t;
#define PRI_xen_pfn "lx"
#endif

/* Maximum number of virtual CPUs in multi-processor guests. */
#define MAX_VIRT_CPUS 32

#ifndef __ASSEMBLY__

typedef unsigned long xen_ulong_t;

/*
 * Send an array of these to HYPERVISOR_set_trap_table().
 * The privilege level specifies which modes may enter a trap via a software
 * interrupt. On x86/64, since rings 1 and 2 are unavailable, we allocate
 * privilege levels as follows:
 *  Level == 0: Noone may enter
 *  Level == 1: Kernel may enter
 *  Level == 2: Kernel may enter
 *  Level == 3: Everyone may enter
 */
#define TI_GET_DPL(_ti)      ((_ti)->flags & 3)
#define TI_GET_IF(_ti)       ((_ti)->flags & 4)
#define TI_SET_DPL(_ti,_dpl) ((_ti)->flags |= (_dpl))
#define TI_SET_IF(_ti,_if)   ((_ti)->flags |= ((!!(_if))<<2))

struct trap_info {
    uint8_t       vector;  /* exception vector                              */
    uint8_t       flags;   /* 0-3: privilege level; 4: clear event enable?  */
    uint16_t      cs;      /* code selector                                 */
    unsigned long address; /* code offset                                   */
};
typedef struct trap_info trap_info_t;
DEFINE_GUEST_HANDLE(trap_info_t);

typedef uint64_t tsc_timestamp_t; /* RDTSC timestamp */

/*
 * The following is all CPU context. Note that the 
 * fpu_ctxt block is filled  in by FXSAVE if the 
 * CPU has feature FXSR; otherwise FSAVE is used.
 * 
 */

#define NUM_FPU_REGS 32

typedef unsigned long long __fpureg_t;

struct __mips_fpu_struct {
        __fpureg_t      fpr[NUM_FPU_REGS];
        unsigned int    fcr31;
};

struct vcpu_guest_context {
	/* Saved main processor registers. */
	unsigned long reg4;
	unsigned long reg5;
	unsigned long reg6;
	unsigned long reg7;
	unsigned long reg16;
	unsigned long reg17;
	unsigned long reg18;
	unsigned long reg19;
	unsigned long reg20;
	unsigned long reg21;
	unsigned long reg22;
	unsigned long reg23;
	unsigned long reg29;
	unsigned long reg30;
	unsigned long reg31;

	/* Saved cp0 stuff. */
	unsigned long cp0_status;

	/* Saved fpu/fpu emulator stuff. */
	struct __mips_fpu_struct fpu;

	/* Other stuff associated with the thread. */
	unsigned long cp0_badvaddr; /* Last user fault */
	unsigned long cp0_baduaddr; /* Last kernel fault accessing USEG */
	unsigned long error_code;
	unsigned long trap_no;
#define MF_FIXADE  1           /* Fix address errors in software */
#define MF_LOGADE  2           /* Log address errors to syslog */
#define MF_32BIT_REGS  4       /* also implies 16/32 fprs */
#define MF_32BIT_ADDR  8       /* 32-bit address space (o32/n32) */
#define MF_FPUBOUND    0x10    /* thread bound to FPU-full CPU set */
	unsigned long mflags;
};
typedef struct vcpu_guest_context vcpu_guest_context_t;
DEFINE_GUEST_HANDLE(vcpu_guest_context_t);

struct arch_shared_info {
    unsigned long max_pfn;                  /* max pfn that appears in table */
	unsigned long start_info_pfn;
    /* Frame containing list of mfns containing list of mfns containing p2m. */
    xen_pfn_t     pfn_to_mfn_frame_list_list;
    unsigned long nmi_reason;
	unsigned long *xensp[NR_CPUS];
	unsigned int dom_page_shift;
	unsigned int pgtable_levels;
    uint64_t pad[32];
};

typedef struct arch_shared_info arch_shared_info_t;

typedef unsigned long xen_callback_t;

#endif /* !__ASSEMBLY__ */

/* Size of the shared_info area (this is not related to page size).  */
#define XSI_SHIFT			14
#define XSI_SIZE			(1 << XSI_SHIFT)

#define XEN_PAGE_SHIFT 16
#define XEN_PAGE_SIZE   (1 << XEN_PAGE_SHIFT)

/*
 * Here we define more architecture-specific CALLBACK types
 * Till we move to complete cpu virtualization, we need to
 * register __get_free_pages() as a callback function for
 * memory allocator/allocation. The following definitions
 * have to be in sync with definitions in public/xen.h.
 * Currently 7 events are defined and netl-specific events
 * are set to begin @ 32 to accommodate additions to xen.h
 */

#define CALLBACKTYPE_dom0_getfreepages    32

#endif /* __XEN_PUBLIC_ARCH_MIPS_XEN_H__ */
