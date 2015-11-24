/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
#ifndef __XEN_PUBLIC_DOMCTL_H__
#define __XEN_PUBLIC_DOMCTL_H__

#include <xen/interface/xen.h>
#include <xen/interface/grant_table.h>

#define XEN_DOMCTL_INTERFACE_VERSION 0x00000006
#define uint64_aligned_t uint64_t

struct xen_domctl_vcpucontext {
    uint32_t vcpu;                  /* IN */
    GUEST_HANDLE(vcpu_guest_context_t) ctxt; /* IN/OUT */
};
typedef struct xen_domctl_vcpucontext xen_domctl_vcpucontext_t;
DEFINE_GUEST_HANDLE(xen_domctl_vcpucontext_t);

/* XEN_DOMCTL_max_mem */
struct xen_domctl_max_mem {
   /* IN variables. */
   uint64_aligned_t max_memkb;
};
typedef struct xen_domctl_max_mem xen_domctl_max_mem_t;
DEFINE_GUEST_HANDLE(xen_domctl_max_mem_t);

struct xen_domctl {
    uint32_t cmd;
#define XEN_DOMCTL_destroydomain        1
#define XEN_DOMCTL_bootvcpu             2
#define XEN_DOMCTL_max_mem              11
    uint32_t interface_version; /* XEN_DOMCTL_INTERFACE_VERSION */
    domid_t domain;
    union {
        struct xen_domctl_max_mem     max_mem;
        struct xen_domctl_vcpucontext vcpucontext;
        uint8_t pad[128];
    } u;
};
typedef struct xen_domctl xen_domctl_t;
DEFINE_GUEST_HANDLE(xen_domctl_t);

#endif
