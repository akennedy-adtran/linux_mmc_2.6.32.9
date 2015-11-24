/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#ifndef __NLM_SPACES_H__
#define __NLM_SPACES_H__

#ifndef __ASSEMBLY__
#ifdef CONFIG_NLM_VMIPS
extern unsigned long nlm_vmips_phys_offset;
#define PHYS_OFFSET nlm_vmips_phys_offset

extern unsigned long long nlm_vmips_highmem_start;
#define        HIGHMEM_START   (nlm_vmips_highmem_start)

#define PAGE_OFFSET (CAC_BASE)

#endif
#endif

#include <asm/mach-generic/spaces.h>


#endif
