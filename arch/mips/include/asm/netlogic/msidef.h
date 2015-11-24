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


#ifndef ASM_NLM_MSIDEF_H
#define ASM_NLM_MSIDEF_H

/*
 * Constants for Intel APIC based MSI messages.
 * Adapted for the RMI XLR using identical defines
 */

/*
 * Shifts for MSI data
 */

#define MSI_DATA_VECTOR_SHIFT		0
#define  MSI_DATA_VECTOR_MASK		0x000000ff
#define	 MSI_DATA_VECTOR(v)		(((v) << MSI_DATA_VECTOR_SHIFT) & MSI_DATA_VECTOR_MASK)

#define MSI_DATA_DELIVERY_MODE_SHIFT	8
#define  MSI_DATA_DELIVERY_FIXED	(0 << MSI_DATA_DELIVERY_MODE_SHIFT)
#define  MSI_DATA_DELIVERY_LOWPRI	(1 << MSI_DATA_DELIVERY_MODE_SHIFT)

#define MSI_DATA_LEVEL_SHIFT		14
#define	 MSI_DATA_LEVEL_DEASSERT	(0 << MSI_DATA_LEVEL_SHIFT)
#define	 MSI_DATA_LEVEL_ASSERT		(1 << MSI_DATA_LEVEL_SHIFT)

#define MSI_DATA_TRIGGER_SHIFT		15
#define  MSI_DATA_TRIGGER_EDGE		(0 << MSI_DATA_TRIGGER_SHIFT)
#define  MSI_DATA_TRIGGER_LEVEL		(1 << MSI_DATA_TRIGGER_SHIFT)

/*
 * Shift/mask fields for msi address
 */

#define MSI_ADDR_BASE_HI		0
#define MSI_ADDR_BASE_LO		0xfee00000

#define MSI_ADDR_DEST_MODE_SHIFT	2
#define  MSI_ADDR_DEST_MODE_PHYSICAL	(0 << MSI_ADDR_DEST_MODE_SHIFT)
#define	 MSI_ADDR_DEST_MODE_LOGICAL	(1 << MSI_ADDR_DEST_MODE_SHIFT)

#define MSI_ADDR_REDIRECTION_SHIFT	3
#define  MSI_ADDR_REDIRECTION_CPU	(0 << MSI_ADDR_REDIRECTION_SHIFT) /* dedicated cpu */
#define  MSI_ADDR_REDIRECTION_LOWPRI	(1 << MSI_ADDR_REDIRECTION_SHIFT) /* lowest priority */

#define MSI_ADDR_DEST_ID_SHIFT		12
#define	 MSI_ADDR_DEST_ID_MASK		0x00ffff0
#define  MSI_ADDR_DEST_ID(dest)		(((dest) << MSI_ADDR_DEST_ID_SHIFT) & MSI_ADDR_DEST_ID_MASK)

#endif /* ASM_NLM_MSIDEF_H */
