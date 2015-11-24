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

#ifndef _XLP_NAND_H
#define _XLP_NAND_H

#define NAND_CMD		0x40
#define NAND_CTRL		0x41
#define NAND_STATUS		0x42
#define NAND_INTMASK		0x43
#define NAND_INT_STATUS		0x44
#define	NAND_ECC_CTRL		0x45
#define NAND_ECC_OFFSET		0x46
#define NAND_ADDR0_L		0x47
#define NAND_ADDR0_H		0x49
#define NAND_ADDR1_L		0x48
#define NAND_ADDR1_H		0x4A
#define NAND_SPARE_SIZE		0x4C
#define NAND_DMA_ADDR		0x59
#define NAND_DMA_CNT		0x5A
#define NAND_DMA_CTRL		0x5B
#define NAND_MEMCTRL		0x60
#define NAND_DATA_SIZE		0x61
#define NAND_READ_STATUS	0x62
#define NAND_TIME_SEQ0		0x63
#define NAND_TIMINGS_ASYN	0x64
#define NAND_TIMINGS_SYN	0x65
#define NAND_FIFO_DATA		0x66
#define NAND_TIME_MODE		0x67
#define NAND_DMA_ADDR_H		0x68
#define NAND_FIFO_INIT		0x6C
#define NAND_GENERIC_SEQ	0x6D
#define NAND_FIFO_STATE		0x6E
#define NAND_TIME_SEQ1		0x6F

#define NAND_SYSCTRL		0x80
#define NAND_RYBYSEL		0x81

/*                                     CMD 3         CMD 2          CMD 1         SEQ */
#define NAND_RESET_CMD              ( (0x0  << 24) | (0x0  << 16) | (0xFF << 8) | 0x0)
#define NAND_READ_PARAMETER_CMD     ( (0x0  << 24) | (0x0  << 16) | (0xEC << 8) | 0x22)
#define NAND_READ_ID_CMD            ( (0x0  << 24) | (0x0  << 16) | (0x90 << 8) | 0x21)
#define NAND_READ_PAGE_CMD          ( (0x0  << 24) | (0x30 << 16) | (0x00 << 8) | 0x2a)
#define NAND_ERASE_BLOCK_CMD        ( (0x0  << 24) | (0xD0 << 16) | (0x60 << 8) | 0xe)
#define NAND_PAGE_PROGRAM_CMD       ( (0x0  << 24) | (0x10 << 16) | (0x80 << 8) | 0xc)
#define NAND_READ_STATUS_CMD        ( (0x0  << 24) | (0x00 << 16) | (0x70 << 8) | 0x24)

#define NAND_CMD_DMA_FLAG   (1<<6)
#define NAND_CMD_ADDR1_FLAG (1<<7)

#define NAND_CTRL_X16_FLAG (1<<12)
#define NAND_CTRL_CUSTOM_XFER_FLAG (1<<11)
#define NAND_CTRL_PAGE_SIZE(size) (size<<8)
#define NAND_CTRL_BLOCK_SIZE(size) (size<<6)
#define NAND_CTRL_ADDR_CYCLE(cyc) (cyc<<0)
#define NAND_CTRL_ECC_EN(en) (en<<5)
#define NAND_CTRL_SPARE_EN(en) (en<<3)

/*Sync mode WE High->RE Low*/
#define NAND_TIME_SEQ0_TWHR(x) (x<<24)
/*ASync mode RE High->WE Low*/
#define NAND_TIME_SEQ0_TRHW(x) (x<<16)
/*Async ALE->Data start*/
#define NAND_TIME_SEQ0_TADL(x) (x<<8)
/*Chance column setup*/
#define NAND_TIME_SEQ0_TCCS(x) (x<<0)
/*TRR time peroid*/
#define NAND_TIME_SEQ1_TRR(x) (x<<9)
/*Busy time peroid for async->sync*/
#define NAND_TIME_SEQ1_TWB(x) (x<<0)
/*RE/WE high hold time*/
#define NAND_TIME_ASYN_TRWH(x) (x<<4)
/*RE/WE pulse width*/
#define NAND_TIME_ASYN_TRWP(x) (x<<0)

#endif
