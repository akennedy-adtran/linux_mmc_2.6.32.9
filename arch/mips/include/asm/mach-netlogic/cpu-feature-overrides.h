/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#ifndef __ASM_MACH_NLM_CPU_FEATURE_OVERRIDES_H
#define __ASM_MACH_NLM_CPU_FEATURE_OVERRIDES_H

#ifdef CONFIG_NLM_RIXI
#define kernel_uses_smartmips_rixi (cpu_data[0].cputype == CPU_XLP)
#else 
#define kernel_uses_smartmips_rixi  0
#endif

#ifdef CONFIG_NLM_XLP
#define cpu_has_dc_aliases     (PAGE_SIZE < (cpu_data[0].dcache.sets * cpu_data[0].dcache.linesz))
#endif

#endif  /* __ASM_MACH_NLM_CPU_FEATURE_OVERRIDES_H */
