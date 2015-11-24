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


#ifdef CONFIG_64BIT
#define LLX_FMT "lx"
#define LLD_FMT "ld"
#define LLU_FMT "lu"
#else
#define LLX_FMT "llx"
#define LLD_FMT "lld"
#define LLU_FMT "llu"
#endif

#define find_32bit_1st_one_bit(source)    \
({ unsigned int __res;                    \
    __asm__ __volatile__(                 \
	".set\tpush\n\t"                  \
	".set\tnoreorder\n\t"             \
        ".set\tmips32\n\t"                \
	"move\t$8,%1\n\t"                 \
	"clz\t%0,$8\n\t"                  \
	".set\tpop"                       \
	: "=r"(__res): "r"(source): "$8");   \
    __res;})

#define find_32bit_1st_zero_bit(source)   \
({ unsigned int __res;                    \
    __asm__ __volatile__(                 \
	".set\tpush\n\t"                  \
	".set\tnoreorder\n\t"             \
        ".set\tmips32\n\t"                \
	"move\t$8,%1\n\t"                 \
	"clo\t%0,$8\n\t"                  \
	".set\tpop"                       \
	: "=r"(__res): "r"(source): "$8" );      \
    __res;})

#define find_64bit_1st_one_bit(source)    \
({ unsigned int __res;                    \
    __asm__ __volatile__(                 \
	".set\tpush\n\t"                  \
	".set\tnoreorder\n\t"             \
        ".set\tmips32\n\t"                \
	"move\t$8,%1\n\t"                 \
	"dclz\t%0,$8\n\t"                 \
	".set\tpop"                       \
	: "=r"(__res): "r"(source): "$8" );      \
    __res;})

#define find_64bit_1st_zero_bit(source)    \
({ unsigned int __res;                    \
    __asm__ __volatile__(                 \
	".set\tpush\n\t"                  \
	".set\tnoreorder\n\t"             \
        ".set\tmips32\n\t"                \
	"move\t$8,%1\n\t"                 \
	"dclo\t%0,$8\n\t"                 \
	".set\tpop"                       \
	: "=r"(__res): "r"(source): "$8" );      \
    __res;})

