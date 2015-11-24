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


#ifndef _ASM_NLM_ATX_CPLD_H
#define _ASM_NLM_ATX_CPLD_H

/*
	 * bit_0 : xgs0 phy reset
	 * bit_1 : xgs1 phy reset
	 * bit_2 : HT reset
	 * bit_3 : RTC reset
	 * bit_4 : gmac phy soft reset
	 * bit_5 : gmac phy hard reset
	 * bit_6 : board reset
	 * bit_7 : reserved
*/
#define ATX_CPLD_RESET_1   2

/*
 *  bit_0_2 : reserved
 *  bit_3 : turn off xpak_0 tx
 *  bit_4 : turn off xpak_1 tx
 *  bit_5 : HT stop (active low)
 *  bit_6 : flash program enable
 *  bit_7 : compact flash io mode
 */
#define ATX_CPLD_MISC_CTRL 8

/*
 * bit_0 : reset tcam 
 * bit_1 : reset xpak_0 module
 * bit_2 : reset xpak_1 module
 * bit_3_7 : reserved
 */
#define ATX_CPLD_RESET_2   9

#endif /* _ASM_NLM_ATX_CPLD_H */