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

#ifndef __XLS_NAND_H__
#define __XLS_NAND_H__

static void nand_write_cmd(unsigned long offset, unsigned char cmd)
{
	*(volatile uint32_t *)offset = (uint32_t)cmd;
}

static void nand_write_addr(unsigned long offset, unsigned char addr)
{
	*(volatile uint32_t *)offset = (uint32_t)addr;
}

static void nand_read_multi(unsigned long offset, void *buf, unsigned short len)
{
	int i;
	volatile unsigned char *tbuf = (volatile unsigned char *)buf;

	for (i = 0; i < len; i++) {
		*tbuf = *(volatile unsigned char *)offset;
		tbuf++;
	}
}

static void nand_read_byte(unsigned long offset, void *buf)
{
	*(volatile unsigned char *) buf = *(volatile unsigned char *)offset;
}

static void nand_write_multi(unsigned long offset, void *buf, unsigned short len)
{
	int i;
	volatile unsigned char *tbuf = (volatile char *)buf;

	for (i = 0; i < len; i++) {
		*(volatile unsigned char *)offset = *(volatile unsigned char *)tbuf;
		tbuf++;
	}
}

#endif /* __XLS_NAND_H__ */
