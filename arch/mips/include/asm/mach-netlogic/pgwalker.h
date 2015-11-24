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

#ifndef _ASM_MACH_NLM_PGWALKER_H
#define _ASM_MACH_NLM_PGWALKER_H

#include <linux/percpu.h>

#define PGW_REGS_BLOCK 4

enum {
	PGW_MMU_SETUP = 0x0,
	PGW_MMU_INFO = 0x10,
	PGW_PGD_BASES,
	PGW_PGD_SHIFT,
	PGW_PGD_MASK,
	PGW_PUD_SHIFT,
	PGW_PUD_MASK,
	PGW_PMD_SHIFT,
	PGW_PMD_MASK,
	PGW_PTE_SHIFT,
	PGW_PTE_MASK
};

#define PGD 0x8
#define PUD 0x4
#define PMD 0x2
#define PTE 0x1

#define pgw_register_write_w(reg, value) write_32bit_nlm_ctrl_reg(PGW_REGS_BLOCK, reg, value)
#define pgw_register_write_d(reg, value) write_64bit_nlm_ctrl_reg(PGW_REGS_BLOCK, reg, value)
#define pgw_register_read_w(reg) read_32bit_nlm_ctrl_reg(PGW_REGS_BLOCK, reg)
#define pgw_register_read_d(reg) read_64bit_nlm_ctrl_reg(PGW_REGS_BLOCK, reg)

#define pgw_print_w(reg) printk(KERN_INFO #reg " = 0x%x\n", pgw_register_read_w(reg))

extern void dump_pgwalker_config(void);

/* pagewalker control register field offset and width */
enum {
  //offset, width pair
  PWFIELD_BD_O  = 32, PWFIELD_BD_W  = 6,
  PWFIELD_GD_O  = 24, PWFIELD_GD_W  = 6,
  PWFIELD_UD_O  = 18, PWFIELD_UD_W  = 6,
  PWFIELD_MD_O  = 12, PWFIELD_MD_W  = 6,
  PWFIELD_PT_O  =  6, PWFIELD_PT_W  = 6,
  PWFIELD_PTE_O =  0, PWFIELD_PTE_W = 6,

  PWSIZE_BD_O  = 32, PWSIZE_BD_W  = 6,
  PWSIZE_PS_O  = 30, PWSIZE_PS_W  = 1,
  PWSIZE_GD_O  = 24, PWSIZE_GD_W  = 6,
  PWSIZE_UD_O  = 18, PWSIZE_UD_W  = 6,
  PWSIZE_MD_O  = 12, PWSIZE_MD_W  = 6,
  PWSIZE_PT_O  =  6, PWSIZE_PT_W  = 6,
  PWSIZE_PTE_O =  0, PWSIZE_PTE_W = 6,

  PWCTL_PW_EN_O = 31, PWCTL_PW_EN_W = 1,

  CFG4_FTLBPAGESIZE_O = 8, CFG4_FTLBPAGESIZE_W = 5
};

#endif
