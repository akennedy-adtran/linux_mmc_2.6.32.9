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

/*
 *  i2c-algo-palm.c i2c driver algorithms for the BK3220 I2C Host 
 *  adapter on the RMI Phoenix System.
 *  Derived from the PCA-ISA I2C-Algo/Bus files.
 */

#ifndef I2C_PALM_H
#define I2C_PALM_H 			1

#define I2C_PALM_CFG			0x00 
#define I2C_PALM_CLKDIV			0x01 
#define I2C_PALM_DEVADDR		0x02 
#define I2C_PALM_ADDR			0x03
#define I2C_PALM_DATAOUT		0x04 
#define I2C_PALM_DATAIN			0x05 
#define I2C_PALM_STATUS			0x06
#define I2C_PALM_STARTXFR		0x07
#define I2C_PALM_BYTECNT		0x08
#define I2C_PALM_HDSTATIM		0x09

/* TEST Values!! Change as required */
#define I2C_PALM_CFG_DEF		0x000000F8	/* 8-Bit Addr + POR Values */
#define I2C_PALM_CLKDIV_DEF		0x14A //0x00000052	
#define I2C_PALM_HDSTATIM_DEF		0x107 //0x00000000

#define I2C_PALM_STARTXFR_RD		0x00000001
#define I2C_PALM_STARTXFR_WR		0x00000000

#endif /* I2C_PALM_H */
