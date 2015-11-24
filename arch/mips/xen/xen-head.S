/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* Xen-specific pieces of head.S, intended to be included in the right
	place in head.S */

#include <linux/elfnote.h>
#include <linux/init.h>

#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/asm-offsets.h>
#include <asm/addrspace.h>
#include <asm/page.h>
#include <asm/pgtable-bits.h>

#include <xen/interface/elfnote.h>
#include <asm/xen/interface.h>

#define __ASM_FORM(x)	x
# define __ASM_SEL(a,b) __ASM_FORM(a)

#define _ASM_PTR	__ASM_SEL(.long, .quad)

.pushsection .text
	.align PAGE_SHIFT_asm
ENTRY(hypercall_page)
	.skip PAGE_SIZE_asm
.popsection

	ELFNOTE(Xen, XEN_ELFNOTE_GUEST_OS,       .asciz "linux")
	ELFNOTE(Xen, XEN_ELFNOTE_GUEST_VERSION,  .asciz "2.6")
	ELFNOTE(Xen, XEN_ELFNOTE_XEN_VERSION,    .asciz "xen-3.0")
/*        ELFNOTE(Xen, XEN_ELFNOTE_VIRT_BASE,      _ASM_PTR PAGE_OFFSET) */
	ELFNOTE(Xen, XEN_ELFNOTE_HYPERCALL_PAGE, _ASM_PTR hypercall_page)
	ELFNOTE(Xen, XEN_ELFNOTE_LOADER,         .asciz "generic")
	ELFNOTE(Xen, XEN_ELFNOTE_SUSPEND_CANCEL, .long 1)
/*	ELFNOTE(Xen, XEN_ELFNOTE_HV_START_LOW,   _ASM_PTR __HYPERVISOR_VIRT_START) */
/*	ELFNOTE(Xen, XEN_ELFNOTE_PADDR_OFFSET,   _ASM_PTR 0) */
