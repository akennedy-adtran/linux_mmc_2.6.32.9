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


#include <asm/netlogic/iomap.h>
#include "nlm_vits_wrapper.h"
#include <asm/netlogic/debug.h>


// global reference   
volatile unsigned int *megis = (unsigned int*)(long)VITESS_BASE_ADDR_1;


// function to setup CS_2 area correctly
void megis_nlm_common_init(int vitess_device) 
{

	if (vitess_device == 1)
		megis = (unsigned int*)(long)VITESS_BASE_ADDR_1;

	if (vitess_device == 2)
		megis = (unsigned int*)(long)VITESS_BASE_ADDR_2;
	return;
}  





// function to convert from littleE to bigE
unsigned int swizzle(unsigned int _in) 
{
	unsigned int i;
	i = (((_in & 0xff00ff00) >> 8) | ((_in & 0x00ff00ff) << 8));
	return i;
}




// lower level register write routines
void megis_write(unsigned int _block, unsigned int _sub, 
		unsigned int _addr, unsigned int value) 
{
	//  printf ("MEIGS: Write block %x sub %x Addr %x Value %x\n", _block, _sub, 
	//  	_addr, value);
	int lvalue = swizzle(value);
	megis[(  (_block << 12) + (_sub << 8) + (_addr))] = lvalue; 
	return;
}




// lower level register read routine
unsigned int megis_read(unsigned int _block, unsigned int _sub, 
			unsigned int _addr) 
{
	unsigned int i;
	i =  megis[((_block << 12) + (_sub << 8) + (_addr))];
	i = swizzle(i);
	return i;
}



// lower level posted register read routine
unsigned int megis_pread(unsigned int _block, unsigned int _sub, 
unsigned int _addr) 
{
	unsigned int i;
	i =  megis[((_block << 12) + (_sub << 8) + (_addr))];
	i =  megis[((_block << 12) + (_sub << 8) + (0xfe))];
	i = swizzle(i);
	return i;
}

