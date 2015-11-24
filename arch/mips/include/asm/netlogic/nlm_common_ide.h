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


#ifndef __ASM_NETLOGIC_H
#define __ASM_NETLOGIC_H


#define  CONFIG_XLR 1

#ifdef CONFIG_XLR
#define NETLOGIC_BOARD_NAME "XLR -ATX1"
#define NETLOGIC_HAVE_PCMCIA 0
#define NETLOGIC_HAVE_IDE    1
#endif


#ifdef NETLOGIC_HAVE_IDE
#define IDE_CS          6
#define IDE_PHYS        0x1D000000
#define K_GPIO_GB_IDE   4
#define K_GPIO_PC_READY 11 
#define K_INT_GPIO_0    32 
#define K_INT_GB_IDE    (K_INT_GPIO_0 + K_GPIO_GB_IDE)
#endif

#ifdef NETLOGIC_HAVE_PCMCIA
#define PCMCIA_CS       4
#define PCMCIA_PHYS     0x11000000
#define K_INT_PC_READY  (K_INT_GPIO_0 + K_GPIO_PC_READY)
#endif


#define IOADDR(a) (UNCAC_BASE + (a))

#endif /* __ASM_NETLOGIC_H */
