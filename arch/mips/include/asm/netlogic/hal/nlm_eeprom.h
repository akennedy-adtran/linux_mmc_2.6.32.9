/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * http://www.gnu.org/licenses/gpl-2.0.txt  
 * or the Broadcom license below:

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
 * #BRCM_4# */

#ifndef _NLM_EEPROM_H
#define _NLH_EEPROM_H

/* MACID i2c memory definitions
 */
#define MAGIC_OFF   		0x00	// Room for two MAC addresses
#define MAGIC_LEN   		0x02
#define MAGIC_BYTE0 		0xAA
#define MAGIC_BYTE1 		0x55
#define MAC_OFF     		(MAGIC_OFF + MAGIC_LEN)
#define MAC_LEN     		0x06
#define MAGIC2_OFF			(MAC_OFF + MAC_LEN)
#define MAC2_OFF			(MAGIC2_OFF + MAGIC_LEN)

#define NAME_OFF     		0x10
#define NAME_LEN     		0x20	// Increase to 32 for compat with BCM naming
#define REV_OFF     		(NAME_OFF + NAME_LEN)
#define REV_LEN     		0x08	// Increase to 8
#define SN_OFF      		(REV_OFF + REV_LEN)
#define SN_LEN      		0x10	// Increase to 16
#define UPD_OFF     		(SN_OFF + SN_LEN)
#define UPD_LEN     		0x08	// Increase to 8

#ifdef CONFIG_NLM_XLP_EEPROM
#define NLM_EEPROM_LEN		(2 * (MAGIC_LEN + MAC_LEN))
#define NLM_EEPROM_MAGIC	((MAGIC_BYTE0 << 8) | MAGIC_BYTE1)
extern int nlm_eeprom_get_mac_addr(unsigned char *mac, int interface);
extern void nlm_eeprom_set_mac_addr(unsigned char *mac, int interface);
extern void nlm_eeprom_dump(unsigned char *data, int offset, int len);

#else
#define nlm_eeprom_get_mac_addr(...)	0
#define nlm_eeprom_set_mac_addr(...)
#define nlm_eeprom_dump(...)

#endif /* CONFIG_NLM_XLP_EEPROM */

#endif /* _NLM_EEPROM_H */
