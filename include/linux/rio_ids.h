/*-
 * Copyright 2005-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
 * RapidIO devices
 *
 * Copyright 2005 MontaVista Software, Inc.
 * Matt Porter <mporter@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef LINUX_RIO_IDS_H
#define LINUX_RIO_IDS_H

#define RIO_ANY_ID			0xffff

#define RIO_VID_FREESCALE		0x0002
#define RIO_DID_MPC8560			0x0003

#define RIO_VID_TUNDRA			0x000d
#define RIO_DID_TSI500			0x0500
#ifdef CONFIG_NLM_COMMON
#define RIO_DID_TSI578			0x0578
#endif
#endif				/* LINUX_RIO_IDS_H */
