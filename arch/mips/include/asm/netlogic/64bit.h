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


#ifndef _ASM_NLM_64BIT_H
#define _ASM_NLM_64BIT_H

#include <linux/types.h>
#include <asm/system.h>

/* Implement 64bit read and write operations */

static inline void out64(u64 val, unsigned long addr)
{
  u32 low, high, tmp;
  unsigned long flags=0;

  high = val >> 32;
  low = val & 0xffffffff;
  local_irq_save(flags);
  __asm__ __volatile__ (
			".set push\t\t\t# out64n"
			".set noreorder\n"
			".set noat\n"
			".set mips4\n"
			"   dsll32 %0, %2, 0   \n"
			"   dsll32 $1, %1, 0   \n"
			"   dsrl32 %0, %0, 0   \n"
			"   or     $1, $1, %0  \n"
			"   sd $1, (%3)\n"
			".set pop\n"
			: "=&r" (tmp)
			: "r" (high), "r" (low), "r" (addr));
  local_irq_restore(flags);
}

static inline u64 in64(unsigned long addr)
{
  unsigned long flags;
  u32 low, high;

  local_irq_save(flags);
  __asm__ __volatile__ (
			".set push\t\t\t# in64\n"
			".set noreorder\n"
			".set noat     \n"
			".set mips4    \n"
			"  ld     %1, (%2)\n"
			"  dsra32 %0, %1, 0\n"
			"  sll    %1, %1, 0\n"
			".set pop\n"
			: "=r" (high), "=r" (low)
			: "r" (addr));
  local_irq_restore(flags);

  return (((u64)high) << 32) | low;
}

#endif 
