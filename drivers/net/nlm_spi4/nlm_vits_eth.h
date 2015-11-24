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


#ifndef _ASM_NLM_VITS_ETH_H
#define _ASM_NLM_VITS_ETH_H

#include "nlm_spi4.h"

#define INVALID_SLOT 	100
#define INVALID_PORT	100

#define NLM_SPI4_MAX_PORTS 20 
#define NLM_SPI4_MAX_SLOTS 2

#define PORT_0 0
#define PORT_1 1
#define PORT_2 2
#define PORT_3 3 
#define PORT_4 4
#define PORT_5 5
#define PORT_6 6
#define PORT_7 7
#define PORT_8 8
#define PORT_9 9
#define PORT_10 10
#define PORT_11 11
#define PORT_12 12
#define PORT_13 13
#define PORT_14 14
#define PORT_15 15 
#define PORT_16 16
#define PORT_17 17
#define PORT_18 18
#define PORT_19 19

struct spi4_port{
	int	slot; // value should be either SPI4_0 or SPI4_1
	int port;
	unsigned long	io_base;

};

int spi4_slot[] = {SPI4_0,SPI4_1};
static struct spi4_port active_port[] = {
	{
		.slot=SPI4_0,
		.port=PORT_0
	},
	{
		.slot=SPI4_0,
		.port=PORT_1
	},
	{
		.slot=SPI4_0,
		.port=PORT_2
	},
	{
		.slot=SPI4_0,
		.port=PORT_3
	},
	{
		.slot=SPI4_0,
		.port=PORT_4
	},
	{
		.slot=SPI4_0,
		.port=PORT_5
	},
	{
		.slot=SPI4_0,
		.port=PORT_6
	},
	{
		.slot=SPI4_0,
		.port=PORT_7
	},
	{
		.slot=SPI4_0,
		.port=PORT_8
	},
	{
		.slot=SPI4_0,
		.port=PORT_9
	},
	{
		.slot=SPI4_1,
		.port=PORT_10
	},
	{
		.slot=SPI4_1,
		.port=PORT_11
	},
	{
		.slot=SPI4_1,
		.port=PORT_12
	},
	{
		.slot=SPI4_1,
		.port=PORT_13
	},
	{
		.slot=SPI4_1,
		.port=PORT_14
	},
	{
		.slot=SPI4_1,
		.port=PORT_15
	},
	{
		.slot=SPI4_1,
		.port=PORT_16
	},
	{
		.slot=SPI4_1,
		.port=PORT_17
	},
	{
		.slot=SPI4_1,
		.port=PORT_18
	},
	{
		.slot=SPI4_1,
		.port=PORT_19
	},

};
#endif
