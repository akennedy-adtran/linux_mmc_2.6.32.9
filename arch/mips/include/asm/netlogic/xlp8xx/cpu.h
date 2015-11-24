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

 
/* XLP CPU specific */

#ifndef __XLP_CPU_H_
#define __XLP_CPU_H_

/* so that these need not 
 * be included explicitly 

#include "xlp_bridge.h"
#include "xlp_i2c.h"
#include "xlp_spi.h"
#include "xlp_sys.h"
#include "xlp_ddr.h"
#include "xlp_gbu.h"
#include "xlp_nand.h"
#include "xlp_ici.h"
 */

#define CHIP_PID_XLP					0x00
#define MAX_NODES						0x04

/* CPU Internal Blocks specific to XLP .
 * These are accessed using the mfcr/mtcr
 * instructions. Blocks [0-5] are same for
 * XLR and XLP
 */
#define CPU_BLOCKID_MAP					0x0a
/* Offsets of interest from the 'MAP' Block */
#define BLKID_MAP_THREADMODE			0x00 
#define BLKID_MAP_EXT_EBASE_ENABLE		0x04 
#define BLKID_MAP_CCDI_CONFIG			0x08
#define BLKID_MAP_THRD0_CCDI_STATUS		0x0c	
#define BLKID_MAP_THRD1_CCDI_STATUS		0x10
#define BLKID_MAP_THRD2_CCDI_STATUS		0x14	
#define BLKID_MAP_THRD3_CCDI_STATUS		0x18
#define BLKID_MAP_THRD0_DEBUG_MODE		0x1c
#define BLKID_MAP_THRD1_DEBUG_MODE		0x20
#define BLKID_MAP_THRD2_DEBUG_MODE		0x24
#define BLKID_MAP_THRD3_DEBUG_MODE		0x28
#define BLKID_MAP_MISC_STATE			0x60
#define BLKID_MAP_DEBUG_READ_CTL		0x64
#define BLKID_MAP_DEBUG_READ_REG0		0x68
#define BLKID_MAP_DEBUG_READ_REG1		0x6c

#define CPU_BLOCKID_SCH				7
#define CPU_BLOCKID_SCU				8
#define CPU_BLOCKID_FPU				9

/* ----------------------------------
 *   XLP RESET Physical Address Map
 * ----------------------------------
 * PCI ECFG : 0x18000000 - 0x1bffffff 
 * PCI CFG  : 0x1c000000 - 0x1cffffff 
 * FLASH    : 0x1fc00000 - 0x1fffffff 
 * ----------------------------------
 */

/* The DEFAULT_XLP_IO_BASE value is what is
 * programmed in the NBU's (NorthBridge Unit) 
 * ECFG_BAR register. The NBU itself is 
 * accessible as [BDF:0,0,0].
 */
#ifdef __ASSEMBLY__
#define DEFAULT_XLP_IO_BASE		0xffffffffb8000000ULL
#define DEFAULT_XLP_IO_BASE_VIRT	0xb8000000	/* IO_BASE for Assembly macros */
#else
#define DEFAULT_XLP_IO_BASE		0xffffffffb8000000ULL
#define DEFAULT_XLP_IO_BASE_VIRT	DEFAULT_XLP_IO_BASE
#define DEFAULT_XLP_IO_BASE_PHYS	0x18000000
#endif

#ifdef NLM_HAL_LINUX_KERNEL		/* Hal requires phy add :-) */
#define DEFAULT_XLP_IO_BASE_PHYS        0x18000000
#define DEFAULT_CPU_IO_BASE		DEFAULT_XLP_IO_BASE_PHYS
#else
#define DEFAULT_CPU_IO_BASE		DEFAULT_XLP_IO_BASE_VIRT
#endif

#define CPU_IO_SIZE			(64<<20)/* Size of the ECFG Space */
#define HDR_OFFSET			0x100 /* Skip 256 bytes of cfg. hdrs */

#define NETL_VENDOR_ID			0x184e
#define ICI_DEVICE_ID			0x1002

/* The On-Chip functional blocks for XLP */

/* ------------------------------------------------------------------------*/
/* Accesses Based on Enhanced Configuration Mechanism					   */
/* ------------------------------------------------------------------------*/
/* Interface			|	Bus    |	Dev	|   Func   */
/* ------------------------------------------------------------------------*/
#define BRIDGE			(0x00<<20) | (0x00<<15) | (0x00<<12)
#define ICI0			(0x00<<20) | (0x00<<15) | (0x01<<12)
#define ICI1			(0x00<<20) | (0x00<<15) | (0x02<<12)
#define ICI2			(0x00<<20) | (0x00<<15) | (0x03<<12)
#define	PIC			(0x00<<20) | (0x00<<15) | (0x04<<12)
#define UART0			(0x00<<20) | (0x06<<15) | (0x00<<12)
#define UART1			(0x00<<20) | (0x06<<15) | (0x01<<12)
#define I2C0			(0x00<<20) | (0x06<<15) | (0x02<<12)
#define I2C1			(0x00<<20) | (0x06<<15) | (0x03<<12)
#define	GPIO			(0x00<<20) | (0x06<<15) | (0x04<<12)
#define SYS			(0x00<<20) | (0x06<<15) | (0x05<<12)
#define	JTAG			(0x00<<20) | (0x06<<15) | (0x06<<12)
#define	NOR			(0x00<<20) | (0x07<<15) | (0x00<<12)
#define	NAND			(0x00<<20) | (0x07<<15) | (0x01<<12)
#define	SPI			(0x00<<20) | (0x07<<15) | (0x02<<12)
#define	MMC			(0x00<<20) | (0x07<<15) | (0x03<<12)
/* same as NOR ? */
#define GBU			(0x00<<20) | (0x07<<15) | (0x00<<12)
/* ------------------------------------------------------------------------*/

#define CPU_MMIO_OFFSET(y,x)	(DEFAULT_CPU_IO_BASE + (x) + \
				HDR_OFFSET + (y<<18))

#define DMC_MMIO_OFFSET(y,x)	(DEFAULT_CPU_IO_BASE + (0x200*x) \
				HDR_OFFSET + (y<<18) + (BRIDGE) + 0x300)

#ifndef __ASSEMBLY__
#define cpu_io_mmio(node,offset)	((__u32 *)((u64)DEFAULT_CPU_IO_BASE + \
					(node<<18) + (offset) + HDR_OFFSET))

#define dmc_io_mmio(node,offset)\
	((__u32 *)(DEFAULT_CPU_IO_BASE + \
	(node<<18) + (0x200*offset) + (BRIDGE) + 0x300 + HDR_OFFSET))

#define ici_io_mmio(node,link)\
	((__u32 *)(DEFAULT_CPU_IO_BASE + (ICI0) + \
	(node<<18) + HDR_OFFSET + (link << 12) ) )
#endif	/* __ASSEMBLY__ */
#endif	/* __XLP_CPU_H_ */
