/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*

   vitesse_io.c  -- Vitesse hardware access layer.

   This file provides the hardware access to target chip registers.
   Modify it to fit your configuration.

   Copyright (c) 2003 Vitesse Semiconductor Corporation. All Rights Reserved.
   Unpublished rights reserved under the copyright laws of the United States of 
   America, other countries and international treaties. The software is provided
   without fee. Permission to use, copy, store, modify, disclose, transmit or 
   distribute the software is granted, provided that this copyright notice must 
   appear in any copy, modification, disclosure, transmission or distribution of 
   the software. Vitesse Semiconductor Corporation retains all ownership, 
   copyright, trade secret and proprietary rights in the software. THIS SOFTWARE
   HAS BEEN PROVIDED "AS IS," WITHOUT EXPRESS OR IMPLIED WARRANTY INCLUDING, 
   WITHOUT LIMITATION, IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
   PARTICULAR USE AND NON-INFRINGEMENT.

   $Id: vitesse_io.c,v 1.1.2.4 2007-05-17 01:15:36 rpmbuilder Exp $

 */

#include "nlm_vits_wrapper.h"
#include <asm/netlogic/debug.h>

#define __VTSS_LIBRARY__

#define VTSS_TRACE_LAYER 1

#include "vitesse_io.h"
#include "meigsii_reg.h"


#define VTSS_CHIP_ADDR_MMAP_SIZE (0x8000*sizeof(ulong))

#ifdef VITGENIO
#include <linux/vitgenio.h>
#include <fcntl.h> /* open() */
#include <sys/ioctl.h> /* ioctl() */
#include <sys/mman.h> /* mmap() */
#endif




#ifdef CUSTOMERIO
#endif

#define VTSS_BLK_SYSTEM             M2_BLK_SYSTEM
#define VTSS_SUBBLK_CTRL            M2_SUBBLK_CTRL
#define VTSS_REG_SW_RESET           M2_SW_RESET
#define VTSS_REG_CPU_TRANSFER_SEL   M2_SI_TRANSFER_SEL
#define VTSS_REG_LOCAL_DATA         M2_LOCAL_DATA
#define VTSS_REG_LOCAL_STATUS       M2_LOCAL_STATUS

#define VTSS_T_RESET 2000000L

/* ================================================================= *
 *  I/O Layer Helper functions
 * ================================================================= */



/* ================================================================= *
 *  I/O Layer initialisation and
 *  Chip hardware access configuration (pin polarity etc.)
 * ================================================================= */
#if 0 
/* Reset and configure the chip. */
static void vtss_io_reset_chip( void )
{
	/* Reset the chip */
	vtss_io_write(VTSS_BLK_SYSTEM,VTSS_SUBBLK_CTRL,VTSS_REG_SW_RESET,0x80000001);
	VTSS_NSLEEP(VTSS_T_RESET);

	/* Set the chip hardware access configuration (pin polarity etc.). */
	{
		const ulong DONE_PINPOLARITY_BITS = 0x99999999;

		vtss_io_writemasked(VTSS_BLK_SYSTEM,VTSS_SUBBLK_CTRL,
					VTSS_REG_CPU_TRANSFER_SEL,
			(0/*done_pinpolarity_activehigh*/)?0:
			DONE_PINPOLARITY_BITS,DONE_PINPOLARITY_BITS);
	}
}
#endif
/* Note: This must be called before any access to the target chip.
 *       vtss_reset_io_layer() calls this directly.
 */
void vtss_io_reset( void )
{

#ifdef VITGENIO
	/* Setup the hardware access (open device driver and setup MMU). */
	if (vtss_io_state->chip_addr == NULL) {
		vtss_io_state->fd_driver = open( "/dev/vitgenio", 0 );
		//VTSS_ASSERT(vtss_io_state->fd_driver != -1);

#ifdef NO_MMAP
		printf ("VTSS:io_reset no_mmap\n");

		vtss_io_state->chip_addr = (void*)4; /* Any non-NULL value to indicate that the driver is ready. */
#else
		vtss_io_state->chip_addr = mmap( 0, VTSS_CHIP_ADDR_MMAP_SIZE, 
						PROT_READ | PROT_WRITE, 
						MAP_SHARED, 
						vtss_io_state->fd_driver, 
						0 );
		//VTSS_ASSERT( vtss_io_state->chip_addr != MAP_FAILED );
#endif
	}

	/* Call the I/O Layer driver callback, if present. */
	if (vtss_io_state->io_driver_callback) vtss_io_state->io_driver_callback();
#endif


#ifdef CUSTOMERIO
	/* Setup the hardware access (open device driver and setup MMU). */
	/* Call the I/O Layer driver callback, if present. */
	vtss_io_state->chip_addr = (void*)4;
	if (vtss_io_state->io_driver_callback) vtss_io_state->io_driver_callback();
#endif
	/* Reset and configure the chip. */
	/* vtss_io_reset_chip(); */
}




/* ================================================================= *
 *  Chip register access methods
 * ================================================================= */

static ulong vtss_io_pi_read(const uint block, const uint subblock, const uint reg)
{
	//VTSS_ASSERT( (block<=0x7)&&(subblock<=0xF)&&(reg<=0xFF) );

	/* Use 32 bit access, letting the CPU split it up to two 16 bit bus accesses */
	VTSS_NSLEEP( 240 ); /* Refer to the data sheet for timing diagrams. */
	if ((block==VTSS_BLK_SYSTEM)
		&&
	(subblock==VTSS_SUBBLK_CTRL)
		&&
	((reg==VTSS_REG_LOCAL_DATA)||(reg==VTSS_REG_LOCAL_STATUS))) {
		return (ulong)megis_read(block, subblock, reg);
	} else {
		/* Perform a dummy read to activate read request. */
		megis_read(block, subblock, reg) ;
		/* Wait for data ready. */
		VTSS_NSLEEP( 1000 ); /* Refer to the data sheet for timing diagrams. */
		return (ulong)megis_read(VTSS_BLK_SYSTEM, 
					VTSS_SUBBLK_CTRL, 
					VTSS_REG_LOCAL_DATA);
	}
}

static void vtss_io_pi_write(const uint block, const uint subblock, const uint reg, const ulong value)
{
	//VTSS_ASSERT( (block<=0x7)&&(subblock<=0xF)&&(reg<=0xFF) );

	/* Use 32 bit access, letting the CPU 
	split it up to two 16 bit bus accesses */
	VTSS_NSLEEP( 120 ); /* Refer to the data sheet for timing diagrams. */
	megis_write(block, subblock, reg, value);
}


ulong vtss_io_read(uint block, uint subblock, const uint reg)
{
	ulong value;

	value = vtss_io_pi_read(block, subblock, reg);

	VTSS_N(("R 0x%01X 0x%01X 0x%02X 0x%08lX",block,subblock,reg,value));

	return value;
}

void vtss_io_write(uint block, uint subblock, const uint reg, const ulong value)
{
	VTSS_N(("W 0x%01X 0x%01X 0x%02X 0x%08lX",block,subblock,reg,value));

	vtss_io_pi_write(block, subblock, reg, value);
}

void vtss_io_writemasked(uint block, uint subblock, 
			const uint reg, const ulong value, 
			const ulong mask)
{
	VTSS_N(("M 0x%01X 0x%01X 0x%02X 0x%08lX 0x%08lX",
		block,subblock,reg,value,mask));

	vtss_io_write(block,subblock,reg, 
			(vtss_io_read(block,subblock,reg) & ~mask) | 
			(value & mask) );
}


/* ================================================================= *
 *  I/O Layer state information
 * ================================================================= */

static vtss_io_state_t default_io_state =
{

	/* The following members must be present: */
	/* This optional callback function will be called after 
		the vtss_io_init function has completed. */
	NULL,       /*void                (*io_driver_callback) (void);*/

	/* The following members are implementation specific: */
	0,          /*int                 fd_driver;*/
	NULL        /*ulong *             chip_addr;*/

};

/* Pointer to current state. */
vtss_io_state_t * vtss_io_state = &default_io_state;

