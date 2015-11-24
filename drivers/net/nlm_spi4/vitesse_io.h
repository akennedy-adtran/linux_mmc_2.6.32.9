/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*

 vitesse_io.h  -- Vitesse hardware access layer.
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
 
 $Id: vitesse_io.h,v 1.1.2.3 2006-09-28 01:24:16 nphilips Exp $

*/

#ifndef _VITESSE_IO_H
#define _VITESSE_IO_H 1

//#include <stdio.h>
#include "vitesse_common.h"

/* ================================================================= *
 *  I/O Layer initialisation and
 *  Chip hardware access configuration (pin polarity etc.)
 * ================================================================= */

/* Note: This must be called before any access to the target chip.
 *       vtss_reset_io_layer() calls this directly.
 */
void vtss_io_reset( void );

/* ================================================================= *
 *  Chip register access
 * ================================================================= */

ulong vtss_io_read(uint block, uint subblock, const uint reg);
void vtss_io_write(uint block, uint subblock, const uint reg, const ulong value);
void vtss_io_writemasked(uint block, uint subblock, const uint reg, const ulong value, const ulong mask);


/* ================================================================= *
 *  I/O Layer state information
 * ================================================================= */

typedef struct _vtss_io_state_t {

/* The following members must be present: */
    /* This optional callback function will be called after the chip hardware access driver has been opened,
       but before the chip has been reset and configured for hardware access (pin polarity etc.).
       It can be used for passing extra parameters to the chip hardware access driver. */
    void                (*io_driver_callback) (void);

/* The following members are implementation specific: */
    int                 fd_driver; /* File descriptor to VitGenIO Linux driver */
    ulong *             chip_addr; /* mmap'ed address of chip */

} vtss_io_state_t;

/* Pointer to I/O layer current state information. */
extern vtss_io_state_t * vtss_io_state;

#endif /* _VITESSE_IO_H */
